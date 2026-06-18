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
uint32_t bewe_mod_host_mask(const char* id);                     // HOST 자기 mask
// HOST 워커 → 디코드 1건 방출: Central 전송(+로컬 뷰 반영). payload = 모듈 정의 레코드
void bewe_mod_emit(FFTViewer& v, const char* id, const void* payload, size_t n);

// ── 채널별 디코드 레이트 통계 (코어 보관, 모듈이 on_data 에서 1건마다 bump) ──
// key = (id, station_raw, ch). DEMOD 통합 테이블이 행마다 "N/min · 마지막수신" 표시용.
// 와이어 변경 없음 — 레코드가 보이는 곳(HOST 로컬 / Recv 구독 JOIN)에서만 집계.
void bewe_mod_stat_bump(const char* id, const char* station, int ch, int64_t t_ms);
void bewe_mod_ch_stat(const char* id, const char* station, int ch, int64_t now_ms,
                      int& cnt60, int64_t& last_ms);

// ── JOIN/뷰어 측 framework (런처·뷰 UI 가 사용) ──
bool bewe_mod_recv(const char* id);                              // Recv 구독 상태
void bewe_mod_set_recv(FFTViewer& v, const char* id, bool on);   // 구독 토글 (on → 히스토리+라이브)
bool bewe_mod_hist_loading(const char* id);
void bewe_mod_req_ch_list(const char* id);                       // 타깃 목록 요청 (런처 폴링)
std::vector<MpChEntry> bewe_mod_targets(FFTViewer& v, const char* id); // 타깃 목록 (LOCAL 이면 로컬 채널)
void bewe_mod_set_target(FFTViewer& v, const char* id, const char* station, int ch, bool on); // 디코드 on/off
// 채널필터 geometry/mode 변경 (어느 기지든): center/bw → lo/hi, mode=0/1/2. 원격은 Central
// 경유 해당 HOST 적용 + CHANNEL_SYNC 로 전 유저 동기화. LOCAL/HOST 는 즉시 적용.
void bewe_mod_edit_ch(FFTViewer& v, const char* station, int ch, int mode, float lo, float hi);
const char* bewe_mod_my_station();                               // 자기 station_id ("" = LOCAL)
void bewe_mod_set_my_station(const char* station_id);            // 코어가 접속 시 설정
