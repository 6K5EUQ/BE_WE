#pragma once
// ── AIS RF 지문 (Specific Emitter ID) — 특징벡터 규약 + 스코어러 seam ──────────
// 목적: CFO 등 손수 특징으로 (1) MMSI 스푸핑/이중송신기 검증 (2) 지문→MMSI 역추정.
// 확장성: 특징벡터는 버전드·append-only, 스코어러는 인터페이스라 추후 DL 백엔드로 교체 가능
//         (StatFpScorer → TorchFpScorer 등, 호출부 무변경). ais_fp.md/계획 참조.
//   - AIS 디코드 워커/모듈에서만 사용. 순수 헤더(무외부 의존).
#include "ais_meta.hpp"
#include <cstdint>
#include <cmath>
#include <deque>
#include <unordered_map>

namespace ais_fp {

// ── 정규 특징벡터 (append-only; 새 특징은 끝에만, FP_VER++·FP_DIM↑) ──────────
constexpr uint16_t FP_VER = 1;
constexpr int      FP_DIM = 4;   // [cfo_hz, fdev_std_hz, clk_ppm, rssi_db]
// named 스칼라 → 정규 순서 벡터 (DB/학습/모델 입출력의 단일 진실원천).
inline void fp_vector(const AisRecord& m, float out[FP_DIM]){
    out[0]=m.cfo_hz; out[1]=m.fdev_std_hz; out[2]=m.clk_ppm; out[3]=m.rssi_db;
}

// ── spoof_flag 값 ──
enum { SPOOF_NONE=0, SPOOF_OK=1, SPOOF_ALERT=2 };

// ── 스코어러 인터페이스 (DL 교체 seam) ─────────────────────────────────────
// observe: 버스트 관측 반영(온라인 갱신). classify: 그 MMSI 스푸핑 판정(0/1/2).
// identify: 지문 최근접 MMSI + 신뢰(0..1). 구현은 receiver-local 상태 보유.
struct FpScorer {
    virtual void     observe(uint32_t mmsi, const AisRecord& m, int64_t now_ms) = 0;
    virtual uint8_t  classify(uint32_t mmsi, const AisRecord& m, float& cfo_z) = 0;
    virtual uint32_t identify(const AisRecord& m, float& conf) = 0;
    virtual void     clear() = 0;
    virtual ~FpScorer() = default;
};

// ── 기본 구현: 통계 기반 (Welford + 2-클러스터 이봉탐지 + 최근접) ───────────
class StatFpScorer : public FpScorer {
public:
    struct MmsiRf {
        long   n=0; double mean=0, m2=0;      // CFO Welford (variance=m2/(n-1))
        double cA=0; long nA=0;               // 클러스터 A
        double cB=0; long nB=0;               // 클러스터 B (이봉시 지연생성)
        bool   hasB=false;
        std::deque<uint8_t> labels;           // 최근 클러스터 라벨 (interleave 판정)
        double fdev_mean=0, ppm_mean=0;       // 보조특징 running mean
        float  max_sog=0.f;                   // 관측 최대 SOG (Doppler 게이팅)
        int64_t last_ms=0;
    };

    void observe(uint32_t mmsi, const AisRecord& m, int64_t now_ms) override {
        if(!m.has_rf || !mmsi) return;
        MmsiRf& s = db_[mmsi];
        stale_reset(s, now_ms);
        s.last_ms = now_ms;
        if(m.sog>=0.f && m.sog>s.max_sog) s.max_sog=m.sog;
        // 보조특징 EMA
        double a = s.n<1 ? 1.0 : 0.1;
        s.fdev_mean += a*(m.fdev_std_hz - s.fdev_mean);
        s.ppm_mean  += a*(m.clk_ppm     - s.ppm_mean);
        // CFO Welford
        double x=m.cfo_hz; s.n++; double d=x-s.mean; s.mean+=d/s.n; s.m2+=d*(x-s.mean);
        // 2-클러스터 갱신
        double var = s.n>1 ? s.m2/(s.n-1) : 0.0;
        double split = split_hz(s, var);
        if(s.nA==0){ s.cA=x; s.nA=1; push_label(s,0); return; }
        if(!s.hasB && std::fabs(x-s.cA)>split){ s.cB=x; s.nB=1; s.hasB=true; push_label(s,1); return; }
        // 가까운 클러스터에 할당
        uint8_t lab = (s.hasB && std::fabs(x-s.cB)<std::fabs(x-s.cA)) ? 1 : 0;
        if(lab==0){ s.cA += (x-s.cA)/(++s.nA); } else { s.cB += (x-s.cB)/(++s.nB); }
        push_label(s, lab);
    }

    uint8_t classify(uint32_t mmsi, const AisRecord& m, float& cfo_z) override {
        cfo_z = 0.f;
        auto it=db_.find(mmsi); if(it==db_.end()) return SPOOF_NONE;
        MmsiRf& s=it->second;
        if(s.n>1 && s.m2>0) cfo_z = (float)((m.cfo_hz - s.mean)/std::sqrt(s.m2/(s.n-1)));
        if(s.n < ESTABLISH_N) return SPOOF_NONE;
        double var = s.m2/(s.n-1);
        double split = split_hz(s, var);
        // 두 클러스터 다 채워짐 + 분리 + 시간 interleave → 동시 2송신기
        if(s.hasB && s.nA>=3 && s.nB>=3 && std::fabs(s.cA-s.cB)>split){
            bool a=false,b=false; for(uint8_t l : s.labels){ if(l==0)a=true; else b=true; }
            if(a&&b) return SPOOF_ALERT;   // interleave = 동시
            // 전부 한쪽 = 단순 드리프트 → 재기준
            if(b && !a){ s.cA=s.cB; s.nA=s.nB; s.hasB=false; s.nB=0; }
            return SPOOF_OK;
        }
        return SPOOF_OK;
    }

    // 지문 최근접 MMSI (확립된 것만 후보). 신뢰 = 마진 기반.
    uint32_t identify(const AisRecord& m, float& conf) override {
        conf=0.f; if(!m.has_rf) return 0;
        uint32_t best=0, second=0; double bd=1e30, sd=1e30;
        for(auto& kv : db_){
            MmsiRf& s=kv.second;
            if(s.n<ESTABLISH_N) continue;
            // 분산 = 레퍼런스 분산과 단일버스트 CFO 스프레드(데이터불균형 바이어스 ~150Hz) 중 큰 값.
            // identify 는 단일 incoming 버스트를 다중버스트 레퍼런스와 비교 → Mahalanobis 상 incoming 잡음 반영해야 과신뢰 방지.
            double var = s.n>1 ? s.m2/(s.n-1) : 1.0;
            if(var<CFO_FLOOR*CFO_FLOOR) var=CFO_FLOOR*CFO_FLOOR;
            if(var<BURST_CFO_STD*BURST_CFO_STD) var=BURST_CFO_STD*BURST_CFO_STD;
            double dc=(m.cfo_hz-s.mean); double dz=dc*dc/var;                   // CFO 정규화 거리²
            double dp=(m.clk_ppm-s.ppm_mean)/PPM_SC; dz += dp*dp;              // 보조 (Doppler무관)
            double df=(m.fdev_std_hz-s.fdev_mean)/FDEV_SC; dz += df*df;
            if(dz<bd){ sd=bd; second=best; bd=dz; best=kv.first; }
            else if(dz<sd){ sd=dz; second=kv.first; }
        }
        if(!best) return 0;
        // 신뢰 = 최근접이 충분히 가깝고(2위와 마진 큼) 일 때만
        if(bd > MATCH_MAXD) return 0;
        double margin = (sd<1e29) ? (sd-bd) : bd*4.0;
        conf = (float)(margin/(margin+bd+1e-9));   // 0..1, 마진 클수록 ↑
        if(conf < MATCH_MINCONF){ conf=0.f; return 0; }
        (void)second;
        return best;
    }

    void clear() override { db_.clear(); }

    // 영속화용 접근 (ais_module 이 DB 파일 read/write)
    std::unordered_map<uint32_t,MmsiRf>& table(){ return db_; }

private:
    static constexpr long   ESTABLISH_N = 8;
    static constexpr double CFO_FLOOR   = 30.0;   // Hz, 레퍼런스 분산 하한 (버스트간 emitter 지터)
    static constexpr double BURST_CFO_STD = 150.0; // Hz, 단일버스트 CFO 스프레드 (데이터불균형 바이어스; de-bias 후 40~60 로 하향)
    static constexpr double PPM_SC      = 5.0;    // ppm 정규화 스케일
    static constexpr double FDEV_SC     = 200.0;  // Hz 정규화 스케일
    static constexpr double MATCH_MAXD  = 9.0;    // 최근접 정규화거리² 상한 (~3σ)
    static constexpr double MATCH_MINCONF = 0.6;  // 마진 신뢰 하한 (미달=—)
    static constexpr int64_t STALE_MS   = 6LL*3600*1000;
    std::unordered_map<uint32_t,MmsiRf> db_;

    static void push_label(MmsiRf& s, uint8_t l){ s.labels.push_back(l); if(s.labels.size()>8) s.labels.pop_front(); }
    // SPLIT: 잡음바닥(60Hz) + Doppler 함의(0.6Hz/kt·maxSOG) 초과 요구
    static double split_hz(const MmsiRf& s, double var){
        double base = std::max(60.0, 4.0*std::sqrt(var));
        double dopp = 0.6*s.max_sog;
        return std::max(base, dopp*1.5);
    }
    static void stale_reset(MmsiRf& s, int64_t now){
        if(s.last_ms && now-s.last_ms>STALE_MS){ s = MmsiRf{}; }
    }
};

} // namespace ais_fp
