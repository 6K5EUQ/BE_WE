#pragma once
// SIGINT Mission System — 운영자 단위 미션 lifecycle.
// 미션 ID: 'A03' (월 letter A-L + 일 DD). 1년 안에서 고유.
// 라이프사이클: 수동 Start/End + UTC 0시 자동 rollover.
// 파일 조직: recordings/missions/<year>/<code>/{iq,audio,hist}/

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "long_waterfall.hpp"   // LongWaterfall::mission_letter

class FFTViewer;

namespace Mission {

enum class State : uint8_t {
    IDLE    = 0,   // 활성 미션 없음 — 신규 녹음 차단
    ACTIVE  = 1,   // 진행 중
    CLOSING = 2,   // End/UTC0 트리거 — 진행 중 녹음 finalize
};

// "A03" 같은 코드 생성. year 인자는 호환성용 (현재 1년 안에서 고유라 사용 안 함).
inline std::string make_code(int /*year*/, int mon0_11, int mday){
    char b[8];
    snprintf(b, sizeof(b), "%c%02d",
             LongWaterfall::mission_letter(mon0_11), mday);
    return std::string(b);
}

// "A03" → (mon0=0, mday=3). 형식 오류 시 false.
inline bool parse_code(const std::string& code, int& mon0_11, int& mday){
    if(code.size() != 3) return false;
    char L = code[0];
    if(L < 'A' || L > 'L') return false;
    mon0_11 = L - 'A';
    int d = atoi(code.c_str() + 1);
    if(d < 1 || d > 31) return false;
    mday = d;
    return true;
}

// UTC0 자동 rollover 워커 (cli_host / ui 모드에서 시작).
void start_utc0_worker(FFTViewer* v);
void stop_utc0_worker();

} // namespace Mission

// ── GUI viewer (mission_view.cpp; headless 빌드 제외) ──────────────────────
class NetClient;
#include "net_protocol.hpp"   // PktMissionFileList / PktMissionFileDlData / MissionFileEntry
#include <vector>
namespace MissionView {
    void draw_modal(FFTViewer& v, NetClient* cli);  // v.mission_modal_open일 때 매 프레임 호출
    void show_toast(const char* msg);               // IDLE 차단 등 일시 알림
    void draw_toast();                              // 매 프레임 호출 — 만료 안 됐으면 그림
    // NetClient 콜백 — ui.cpp 에서 cli->on_mission_file_list/dl_data 에 등록.
    void on_mission_file_list_recv(const PktMissionFileList& page,
                                   const std::vector<MissionFileEntry>& rows);
    void on_mission_file_dl_data_recv(const PktMissionFileDlData& d,
                                      const uint8_t* chunk, uint32_t chunk_len);
}
