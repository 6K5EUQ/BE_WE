#pragma once
// ── ACARS 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ─────────────────────
#include "acars_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct FFTViewer;

namespace acars_mod {

// ── 파이프 opcode (PktModulePipe.kind) ──
enum : uint8_t {
    K_TOGGLE     = 1,   // JOIN→HOST: { ch, on }
    K_STATE      = 2,   // HOST→JOIN: { mask } 채널별 디코드 ON 비트
    K_MSG        = 3,   // HOST→JOIN: WireMsg 라이브 디코드 1건
    K_HIST_REQ   = 4,   // JOIN→HOST: 오늘 히스토리 파일 요청
    K_HIST_META  = 5,   // HOST→JOIN: { total_bytes } 전송 시작
    K_HIST_CHUNK = 6,   // HOST→JOIN: 파일 바이트 청크
    K_HIST_DONE  = 7,   // HOST→JOIN: 전송 끝
};

struct __attribute__((packed)) WireToggle { uint8_t ch, on; };
struct __attribute__((packed)) WireState  { uint32_t mask; };
struct __attribute__((packed)) WireMsg {
    int64_t t_ms; float freq;
    uint8_t ch, crc_ok, downlink;
    char mode, block, ack;
    char reg[10], flight[8], label[3];
    char text[256];
};
struct __attribute__((packed)) WireHistMeta { uint32_t total_bytes; };

// ── 디코드 로그 (표시/히스토리) ──
extern std::mutex            mtx;
extern std::vector<AcarsMsg> msglog;
extern bool                  scroll;
extern char                  filter[64];
constexpr int LOG_MAX = 5000;

extern std::atomic<uint32_t> on_mask;   // HOST: 실제 워커 상태 / JOIN: 미러

// ── HOST 측 워커 (acars_decode.cpp) ──
void worker(FFTViewer& v, int ch_idx);
// 워커 시작/정지 + on_mask 갱신 + K_STATE 브로드캐스트 (acars_module.cpp)
void host_set(FFTViewer& v, int ch, bool on);
void host_emit(FFTViewer& v, AcarsMsg m);    // 워커 → 스탬프+로그+저장+브로드캐스트

// ── 공통 헬퍼 (acars_module.cpp) ──
void append_log(const AcarsMsg& m);
void msg_to_wire(const AcarsMsg& m, WireMsg& w);
void wire_to_msg(const WireMsg& w, AcarsMsg& m);
// 일 단위 저장소: ~/BE_WE/modules/acars/acars_YYYYMMDD.jsonl (KST)
std::string store_dir();
std::string store_path_today();
void        store_append(const AcarsMsg& m);
bool        store_read_today(std::string& out);          // 파일 통째로 (전송용)
void        store_parse_jsonl(const char* data, size_t n, std::vector<AcarsMsg>& out);

// ── GUI (acars_view.cpp) ──
#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void channel_ui(FFTViewer& v, int ch_idx);
// JOIN 히스토리 수신 상태 (view 가 요청 트리거, module 이 수신 처리)
extern std::atomic<bool> hist_waiting;
extern std::string       hist_buf;
void request_history(FFTViewer& v);   // 원격이면 K_HIST_REQ, LOCAL 이면 파일 직접 로드
#endif

} // namespace acars_mod
