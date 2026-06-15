#pragma once
// ── HOST 상태 영속화 (재시작 시 직전 상태 그대로 복원) ─────────────────────
// 기지(headless cli_host)가 SIGINT 등으로 꺼졌다 켜져도 사용자 입장에서
// "잠깐 깜빡인" 것처럼 보이도록 center freq / sample rate / gain / 채널 필터를
// $HOME/BE_WE/host_state_<station>.json 에 저장하고 부팅 시 복원한다.
//   - 저장: 변경 시마다 (메인 루프에서 fingerprint 비교 → 바뀌면 write)
//   - 복원: cf/sr/gain 은 initialize 단계, 채널은 net_srv 기동 후
// Central 은 stateless relay 라 상태 보관 불가 — 각 HOST 로컬에만 저장.

#include <string>
#include <cstdint>
#include "config.hpp"   // MAX_CHANNELS

class FFTViewer;

namespace HostState {

// $HOME/BE_WE/host_state_<station>.json
std::string file_path(const std::string& station);

struct ChanSnap {
    float    s = 0, e = 0;            // 필터 경계 (절대 MHz)
    int      mode = 0;               // Channel::DemodMode (0=NONE,1=AM,2=FM)
    char     owner[32] = {};
    uint32_t audio_mask = 0xFFFFFFFFu;
    int      pan = 0;
    float    sq = -50.0f;
};

struct Snapshot {
    bool     ok = false;             // 파일이 있고 파싱됨
    float    cf_mhz = 0;
    float    sr_msps = 0;            // 0 = 미지정 (HW 기본 사용)
    bool     has_gain = false;
    float    gain_db = 0;
    int      n_chans = 0;
    ChanSnap chans[MAX_CHANNELS];
};

// 현재 v 의 persistent 상태를 station 별 파일에 기록 (tmp+rename 으로 원자적).
void save(const FFTViewer& v, const std::string& station);

// 디스크에서 읽기 (파일 없으면 ok=false).
Snapshot load(const std::string& station);

// persistent 필드 fingerprint — 변경 감지용. dem_paused 같은 런타임 상태는 제외.
uint64_t fingerprint(const FFTViewer& v);

// 스냅샷의 채널들을 v.channels[] 에 복원 + 복조 시작 + 범위 판정.
// (CF/SR/gain 은 initialize 단계에서 이미 적용됨 — 여기선 채널만)
void apply_channels(FFTViewer& v, const Snapshot& st);

} // namespace HostState
