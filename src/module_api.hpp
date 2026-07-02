#pragma once
// ── BE_WE 모듈 시스템 (선택 설치형 demod/프로토콜 해독 모듈) ────────────────
//
// src/modules/<id>/ 폴더가 존재하면 static-init 으로 자기등록. 폴더가 없으면
// 레지스트리가 비고 모든 hook 은 no-op — 코어는 모듈의 존재를 모른다.
// 코어는 모듈 헤더를 include 하지 않는다 (이 파일이 유일한 경계).
//
// 데이터/제어 흐름 (Central 컨트롤플레인):
//   런처(DEMOD 패널)에서 전 스테이션 채널 타깃 목록을 Central 에서 받아
//   원하는 채널에 디코드를 on/off (누구나, 전 유저 동기화).
//   HOST 가 복조 → BEWE_MK_DATA 로 Central 전송 → Central 이 일 단위 저장 +
//   Recv 구독 JOIN 에게 팬아웃. JOIN 이 Recv 켜면 오늘 히스토리 수신 후 라이브.
#include "net_protocol.hpp"
#include <cstdint>
#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <vector>

struct FFTViewer;

struct BeweModule {
    const char* id;        // wire id, 7자 이내 ("acars")
    const char* label;     // DEMOD 패널 표시 이름 ("ACARS")
    uint8_t target_modes;  // 적합 채널 demod 모드 비트 (1<<DM_AM 등). 0 = 채널타깃형 아님
    bool planned;          // true = 예정(placeholder) 모듈: 런처 목록에 비활성 표시, 기능 없음
    // ── GUI hooks (CLI 빌드에선 nullptr) ──
    void (*init)(FFTViewer& v);                            // 시작 시 1회 (DB 로드 등)
    void (*draw_content)(FFTViewer& v, bool just_opened);  // 데이터 뷰 탭 내용
    // ── HOST hooks ──
    bool (*host_start)(FFTViewer& v, int ch_idx);          // 워커 시작 (성공 여부)
    void (*host_stop)(FFTViewer& v, int ch_idx);           // 워커 정지
    void (*on_ch_stop)(FFTViewer& v, int ch_idx);          // 채널 demod 종료/모드변경 → 정리
    // ── 데이터 수신 (JOIN/뷰어): 라이브 + 히스토리 레코드 공용 ──
    // station = 복조한 기지 station_id ("DGS-2_DGS-2" / "LOCAL")
    void (*on_data)(FFTViewer& v, const char* station, const uint8_t* d, size_t n);
    // ── 온디맨드 녹음(WAV) 페치 ──
    void (*on_rec_req)(FFTViewer& v, int ch, uint64_t rec_id);  // HOST: 요청 수신 → WAV 청크 회신
    void (*on_rec_data)(FFTViewer& v, uint64_t rec_id, uint32_t total, uint32_t off, const uint8_t* b, uint32_t n); // JOIN: 청크 수신
    // ── 단일 대상 온디맨드 전체 이력 (JOIN): d = 레코드 스트림(u32 len + MpData+payload)*.
    // 그 대상(예: MMSI) 기존 log 레코드를 이 전체 세트로 교체 (구독 요약과 중복 제거). nullptr=미지원.
    void (*on_vessel_hist)(FFTViewer& v, const uint8_t* d, size_t n);
    // ── 과거(전일) 데이터 조회 (JOIN, Hist 버튼) — 셋 다 구현해야 Hist 지원 ──
    // v 안 받음 (log/mtx 만 접근) → 재접속 리셋(set_my_station, v 없음) 에서도 복원 호출 가능.
    void (*log_stash)();     // 라이브 log → 임시버퍼 (Hist 진입; log 비움)
    void (*log_restore)();   // 임시버퍼 → log 복원 (Hist 종료; 과거 데이터 폐기)
    void (*on_hist_file)(const char* station, const char* data, size_t n); // 과거 JSONL 1개 파싱→log append
};

void bewe_register_module(const BeweModule& m);
const std::vector<BeweModule>& bewe_modules();

// ── 송신 백엔드 (코어가 연결; 인자 = PktModulePipe+payload 완성 버퍼) ──
void bewe_mod_set_send_up(std::function<bool(const void*, uint32_t)> f);    // JOIN→Central / (LAN 불사용)
void bewe_mod_set_broadcast(std::function<bool(const void*, uint32_t)> f); // HOST→relay(Central)

// ── 수신 라우팅 (코어 내부) ──
void bewe_mod_route(FFTViewer& v, bool host_side, const uint8_t* payload, size_t len);

// ── HOST 측 framework ──
void bewe_mod_host_announce(FFTViewer& v);                       // 전 모듈 STATE 브로드캐스트 (conn_open 등)
void bewe_mod_host_mask_clear(FFTViewer& v, const char* id, int ch); // 워커 자연 종료 → mask 정리+브로드캐스트
uint64_t bewe_mod_host_mask(const char* id);                     // HOST 자기 mask (ch 0~63)
void bewe_mod_reconcile(FFTViewer& v);                           // want↔host_mask 재조정 (HOST 주기 호출)
void bewe_mod_want_clear_ch(int ch);                             // 채널 진짜 삭제 시 그 ch want 해제
void bewe_mod_rec_request(const char* id, const char* station, int ch, uint64_t rec_id);            // JOIN→HOST: 녹음 WAV 요청
void bewe_mod_rec_send(const char* id, uint64_t rec_id, uint32_t total, uint32_t off, const void* b, uint32_t n); // HOST→JOIN: WAV 청크 회신
// HOST 워커 → 디코드 1건 방출: Central 전송(+로컬 뷰 반영). payload = 모듈 정의 레코드
void bewe_mod_emit(FFTViewer& v, const char* id, const void* payload, size_t n);

// ── 채널별 디코드 레이트 통계 (코어 보관, 모듈이 on_data 에서 1건마다 bump) ──
// key = (id, station_raw, ch). DEMOD 통합 테이블이 행마다 "N/min · 마지막수신" 표시용.
// 와이어 변경 없음 — 레코드가 보이는 곳(HOST 로컬 / Recv 구독 JOIN)에서만 집계.
void bewe_mod_stat_bump(const char* id, const char* station, int ch, int64_t t_ms);
void bewe_mod_ch_stat(const char* id, const char* station, int ch, int64_t now_ms,
                      int& cnt60, int64_t& last_ms);
// HOST: 채널 ch 에서 도는 디코더의 누적건수 + 동작경과(초) — ChSyncEntry/LOCAL 타깃 채움용
void bewe_mod_host_ch_decstat(int ch, uint32_t& count, uint32_t& runtime_s);
// HOST: 채널 ch Holding 진입(true)/이탈(false) — runtime 누적 freeze/resume (디코더 워커가 호출)
void bewe_mod_host_ch_hold(int ch, bool holding);

// ── JOIN/뷰어 측 framework (런처·뷰 UI 가 사용) ──
bool bewe_mod_recv(const char* id);                              // Recv 구독 상태
void bewe_mod_set_recv(FFTViewer& v, const char* id, bool on);   // 구독 토글 (on → 히스토리+라이브)
bool bewe_mod_hist_loading(const char* id);
// JOIN→Central: 단일 대상(key=AIS MMSI)의 오늘 전체 이력 온디맨드 요청. 응답 레코드는
// on_data 로 흘러 log 에 병합 (구독 요약이 보낸 최초/최근만 → 전체로 채워짐). 뷰가 배 활성화 시 호출.
void bewe_mod_req_vessel(const char* id, uint32_t key);
// ── 과거 데이터 조회 (Hist 버튼; HOST 가 00시 push 한 전일 아카이브를 Central 서 다운로드) ──
struct BeweHistDate { char date[9]; uint32_t bytes; uint8_t stations; };  // date="YYYYMMDD"
bool bewe_mod_hist_supported(const char* id);                    // 모듈이 Hist 훅 구현?
void bewe_mod_hist_list_req(const char* id);                     // 날짜 목록 요청 (Hist 팝업 열 때)
std::vector<BeweHistDate> bewe_mod_hist_dates(const char* id);   // 수신된 날짜 목록
bool bewe_mod_hist_list_loading(const char* id);
// Hist 진입/날짜 전환: 라이브 log 버퍼 대피(첫 진입) + Recv OFF + 그 날짜 데이터 다운로드→오버레이
void bewe_mod_hist_fetch(FFTViewer& v, const char* id, const char* date8);
// Hist 종료 (Recv ON 클릭): 과거 데이터 폐기 + 버퍼 복원 + 재구독 (clear 없음)
void bewe_mod_hist_exit(FFTViewer& v, const char* id);
bool bewe_mod_hist_mode(const char* id, char* date8_out /*[9], nullable*/);  // 현재 과거조회 모드?
bool bewe_mod_hist_fetching(const char* id);                     // 과거 데이터 다운로드 중?
void bewe_mod_req_ch_list(const char* id);                       // 타깃 목록 요청 (런처 폴링)
std::vector<MpChEntry> bewe_mod_targets(FFTViewer& v, const char* id); // 타깃 목록 (LOCAL 이면 로컬 채널)
void bewe_mod_set_target(FFTViewer& v, const char* id, const char* station, int ch, bool on); // 디코드 on/off
// 채널필터 geometry/mode 변경 (어느 기지든): center/bw → lo/hi, mode=0/1/2. 원격은 Central
// 경유 해당 HOST 적용 + CHANNEL_SYNC 로 전 유저 동기화. LOCAL/HOST 는 즉시 적용.
void bewe_mod_edit_ch(FFTViewer& v, const char* station, int ch, int mode, float lo, float hi);
// 채널 생성/삭제 (어느 기지든). 원격은 Central 경유 해당 HOST 적용 + CHANNEL_SYNC 동기화.
// add: HOST 가 빈 슬롯 선택; lo/hi/mode 는 새 채널 초기값. del: ch 로 지정.
void bewe_mod_add_ch(FFTViewer& v, const char* station, int mode, float lo, float hi);
void bewe_mod_del_ch(FFTViewer& v, const char* station, int ch);
// HOST: 채널 geometry/SR 변경 시 그 채널서 도는 디코더를 새 band 로 재시작 (mask 유지).
// 디코더는 채널 audio 모드 무관(IQ 직접 탭). 채널 없으면 no-op.
void bewe_mod_ch_retune(FFTViewer& v, int ch);
// 기지 하드웨어 튜닝 변경 (어느 기지든): cf_mhz / sr_msps. 0 = 그 필드 유지.
// 원격은 Central 경유 해당 HOST 적용, LOCAL/HOST 는 즉시. CF/SR 은 STATE 폴링으로 전 유저 반영.
void bewe_mod_tune(FFTViewer& v, const char* station, float cf_mhz, float sr_msps);
const char* bewe_mod_my_station();                               // 자기 station_id ("" = LOCAL)
void bewe_mod_set_my_station(const char* station_id);            // 코어가 접속 시 설정
