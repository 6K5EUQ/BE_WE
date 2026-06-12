#pragma once
// ── BE_WE 모듈 시스템 (선택 설치형 demod/프로토콜 해독 모듈) ────────────────
//
// src/modules/<id>/ 폴더가 존재하면 그 모듈이 static-init 으로 자기등록한다.
// 폴더가 없으면 레지스트리가 비고 모든 hook 은 no-op — 코어는 모듈의 존재를
// 알지 못하며 UI 에 어떤 흔적도 남지 않는다. 코어는 모듈 헤더를 include 하지
// 않는다 (이 파일이 유일한 경계).
//
// 데이터 이동은 전부 공용 파이프 (PacketType::MODULE_PIPE) 하나로:
//   JOIN → HOST  : bewe_mod_send_to_host()   (명령, 히스토리 요청 등)
//   HOST → 전JOIN: bewe_mod_broadcast()      (상태, 라이브 데이터, 파일 청크)
// Central 은 opaque 포워딩하므로 모듈 추가에 Central 재빌드 불필요.
#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>

struct FFTViewer;

struct BeweModule {
    const char* id;       // wire id, 7자 이내 ("acars")
    const char* label;    // DEMOD 패널 표시 이름 ("ACARS")
    // ── GUI hooks (CLI 빌드에선 nullptr) ──
    void (*init)(FFTViewer& v);                              // 시작 시 1회 (DB 로드 등)
    void (*draw_content)(FFTViewer& v, bool just_opened);    // DEMOD 패널 탭 내용
    void (*channel_ui)(FFTViewer& v, int ch_idx);            // 채널 행 추가 버튼
    // ── 파이프 수신 ──
    void (*on_pipe_join)(FFTViewer& v, uint8_t kind, const uint8_t* d, size_t n); // JOIN측: HOST→JOIN 수신
    void (*on_pipe_host)(FFTViewer& v, uint8_t kind, const uint8_t* d, size_t n); // HOST측: JOIN→HOST 수신
    // ── HOST hooks ──
    void (*on_join_open)(FFTViewer& v);                      // 새 JOIN 접속 → 상태 push
    void (*on_ch_stop)(FFTViewer& v, int ch_idx);            // 채널 demod 종료/모드변경 → 모듈 정리
};

// 모듈 → 코어 등록 (모듈 .cpp 의 static-init 에서 호출)
void bewe_register_module(const BeweModule& m);
const std::vector<BeweModule>& bewe_modules();

// ── 파이프 송신 (모듈이 호출; 코어가 백엔드 연결) ──
// 백엔드 미연결(LOCAL 등) 시 false 반환 — 모듈은 로컬 경로로 처리.
bool bewe_mod_send_to_host(const char* mod_id, uint8_t kind, const void* d, size_t n);
bool bewe_mod_broadcast(const char* mod_id, uint8_t kind, const void* d, size_t n);

// ── 코어 내부용 ──
// 수신 MODULE_PIPE payload(PktModulePipe+data) → 해당 모듈 hook 라우팅
void bewe_mod_route(FFTViewer& v, bool host_side, const uint8_t* payload, size_t len);
// 송신 백엔드 등록 (인자 = 완성된 MODULE_PIPE payload 버퍼)
void bewe_mod_set_send_to_host(std::function<bool(const void*, uint32_t)> f);
void bewe_mod_set_broadcast(std::function<bool(const void*, uint32_t)> f);
