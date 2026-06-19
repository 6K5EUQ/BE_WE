#pragma once
#include "config.hpp"
#include "bewe_paths.hpp"
#include "net_server.hpp"
#include "net_client.hpp"
#include "hw_config.hpp"
#include "channel.hpp"
#include "audio_playback.hpp"
#include "mission.hpp"

#ifndef BEWE_HEADLESS
  #include <GL/glew.h>
  #include <GLFW/glfw3.h>
  #include <imgui.h>
#else
  typedef unsigned int GLuint;
#endif
#include <libbladeRF.h>
#include <rtl-sdr.h>
#include <fftw3.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <climits>
#include <functional>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <deque>
#include <condition_variable>
#include <memory>
#include <sys/types.h>

// ── Global log helper (ui.cpp에서 정의, 모든 .cpp에서 사용 가능) ─────────
extern std::string g_sdr_force; // "" = 자동, "bladerf"|"rtlsdr"|"pluto"
extern std::vector<std::string> scan_available_sdrs();
extern void bewe_log(const char* fmt, ...);
// LOG 오버레이용 글로벌 로그 (col: 0=HOST 1=SERVER 2=JOIN)
extern void bewe_log_push(int col, const char* fmt, ...);

// ── 녹음 .info 자동 생성 (이미 존재하면 덮어쓰지 않음) ─────────────────────
// source_type: "IQ Recording" / "Audio Recording" / "Region IQ" / "Scheduled IQ"
// modulation: "AM"/"FM"/"MAGIC"/"" 등
// freq/bw/duration이 0이면 해당 줄 빈 값으로 둠
void write_default_info_file(const std::string& wav_path,
                             const char* source_type,
                             double freq_mhz,
                             double bw_khz,
                             double duration_sec,
                             const char* modulation,
                             const char* operator_name,
                             const char* station_name,
                             time_t start_wall_time,
                             int utc_offset_hours = INT_MIN,
                             uint32_t sample_rate = 0);  // SigMF meta용 (IQ .sigmf-data); .wav는 무시

// ── FFTViewer ─────────────────────────────────────────────────────────────
class FFTViewer {
public:
    // ── RAII: 모든 스레드 멤버가 joinable인 채로 파괴되지 않도록 방어 ────────
    // 정상 경로(ui.cpp / cli_host.cpp cleanup)에서는 스레드가 이미 join된 상태
    // 예외/이탈/정리 누락 시에도 std::terminate 방지
    ~FFTViewer();

    // ── FFT / waterfall data ──────────────────────────────────────────────
    FFTHeader            header;
    std::vector<float>   fft_data;
    GLuint               waterfall_texture=0;
    std::vector<uint32_t> wf_row_buf;

    int   fft_size=DEFAULT_FFT_SIZE*FFT_PAD_FACTOR, time_average=TIME_AVERAGE;
    int   fft_input_size=DEFAULT_FFT_SIZE;  // 실제 입력 샘플 수 (윈도우 길이)
    float* win_buf=nullptr;                 // pre-computed Nuttall window (fft_input_size)
    float* mag_sq_buf=nullptr;              // VOLK magnitude squared buffer (fft_size)
    bool  fft_size_change_req=false; int pending_fft_size=DEFAULT_FFT_SIZE;
    bool  sr_change_req=false; float pending_sr_msps=61.44f; // 샘플레이트 변경 요청
    bool  texture_needs_recreate=false;
    int   current_fft_idx=0, last_wf_update_idx=-1;
    float freq_zoom=1, freq_pan=0;
    float display_power_min=-80, display_power_max=0;
    bool  join_manual_scale=false; // JOIN 수동 스케일: true면 HOST pmin/pmax 덮어쓰기 금지
    float spectrum_height_ratio=0.2f;
    float right_panel_ratio=0.0f;
    std::atomic<bool> render_visible{true}; // false=좌측 패널 완전 숨김 → FFT/WF 연산 중단

    // ── 파워 스펙트럼 Max Decay (스펙트럼 빈 영역 좌클릭 On/Off 토글) ─────
    // 0=Off, 1=Max Decay (피크 즉시 반영 + 5 dB/s로 천천히 감쇠)
    // per-bin 저장 > pan/zoom 변경에도 유지, cf/fft_size 변경 시만 리셋
    int  max_hold_mode = 0;
    std::vector<float> max_hold_spectrum;   // per-bin peak (현재 표시값)
    int  last_maxhold_sp_idx = -1;          // 중복 업데이트 방지 (FFT row 변경 감지)
    uint64_t last_maxhold_cf = 0;
    int      last_maxhold_fft_size = 0;

    // ── 노치필터 (파워스펙트럼 Ctrl+우클릭 드래그로 생성, 세션 한정) ───────
    // fft_data는 pristine 유지 - 렌더/워터폴 매핑 시점에만 영역을 주변 노이즈
    // 플로어(±32 bin 평균)로 치환하고 빨간색으로 표시
    struct NotchFilter {
        float freq_lo_mhz;
        float freq_hi_mhz;
        // 프레임 간 안정화용 EMA 상태 (렌더 경로에서 매 프레임 블렌딩 업데이트)
        float lo_lvl    = -80.0f;  // 좌측 인접 median EMA (녹색 선용)
        float hi_lvl    = -80.0f;  // 우측 인접 median EMA
        float lo_spread = 2.0f;    // 좌측 인접 변동 폭 EMA
        float hi_spread = 2.0f;    // 우측 인접 변동 폭 EMA
        float mh_lo_lvl    = -200.0f; // Max Decay용 동일
        float mh_hi_lvl    = -200.0f;
        float mh_lo_spread = 2.0f;
        float mh_hi_spread = 2.0f;
        // 경계 anchor EMA (edge 인접 픽셀의 median을 EMA 블렌딩) > 1프레임 transient 방지
        float edge_lvl_L    = -80.0f;
        float edge_lvl_R    = -80.0f;
        float mh_edge_lvl_L = -200.0f;
        float mh_edge_lvl_R = -200.0f;
        bool  inited    = false;
        bool  mh_inited = false;
        bool  edge_inited    = false;
        bool  mh_edge_inited = false;
    };
    std::vector<NotchFilter> notches;
    std::mutex               notches_mtx;
    static constexpr int     NOTCH_NF_NEIGHBOR_BINS = 32;
    struct NotchDrag {
        bool  selecting = false;
        float drag_x0 = 0, drag_x1 = 0;
    } notch_drag;

    // ── Frequency Band Plan (Central 공유 — 라벨 띠) ─────────────────────
    struct BandSegment {
        float   freq_lo_mhz = 0;
        float   freq_hi_mhz = 0;
        char    label[24]   = {};
        uint8_t category    = 10;       // 0=Broadcast 1=Aero 2=Marine 3=Amateur
                                        // 4=Cell 5=ISM 6=WiFi-BT 7=Mil
                                        // 8=Public-Safety 9=Government 10=Other
        char    description[128] = {};
    };
    std::vector<BandSegment> band_segments;
    std::mutex               band_mtx;
    bool                     band_show = false; // 기본 off (세션 한정 토글)

    // ── System monitor (bottom bar) ───────────────────────────────────────
    float sysmon_cpu=0, sysmon_ghz=0, sysmon_ram=0, sysmon_io=0;
    std::atomic<int>     sysmon_cpu_temp_c{0};   // CPU 온도 (정수 °C, heartbeat 전송용)
    std::atomic<uint8_t> sysmon_bat{255};         // 배터리 % (255=없음/데스크탑)
    // 자기 머신의 recordings 디스크 여유 (JOIN 의 "Local" 표시용 / HOST 자기 disk_stat 송신 시 참조 가능)
    std::atomic<uint64_t> local_disk_free{0};
    std::atomic<uint64_t> local_disk_total{0};

    // ── Timemachine ───────────────────────────────────────────────────────
    // 워터폴/스펙트럼: 항상 2500행 메모리 유지 (위 MAX_FFTS_MEMORY)
    // IQ 롤링: T키로 활성화
    std::atomic<bool> tm_iq_on{false};     // T키: IQ SSD 롤링 활성
    // ── Signal Library / Emitter DB ───────────────────────────────────
    bool                            sig_lib_panel_open = false;   // M키 토글
    std::mutex                      sig_lib_mtx;                  // emitters/sightings 보호
    std::vector<PktEmitterEntry>    sig_lib_emitters;             // Central 캐시
    std::vector<PktSightingEntry>   sig_lib_sightings;            // 선택된 emitter의 sightings (또는 전체)
    bool                            sig_lib_dirty = false;        // overlay 진입 시 list_req 트리거
    std::string                     sig_lib_selected_uid;         // 좌측 테이블 선택 emitter
    std::string                     sig_lib_sightings_filter_uid; // sightings 캐시의 현재 필터
    char                            sig_lib_search[128] = {};
    bool                            sig_lib_show_pending = true;
    bool                            sig_lib_show_auto = true;
    bool                            sig_lib_show_confirmed = true;
    // ── SIGINT Mission System ─────────────────────────────────────────
    // 'A03' (월+일) 코드, UTC 0시 자동 rollover, 신규 녹음은 미션 dir로 라우팅
    struct MissionEntry {
        int      year       = 0;
        char     code[8]    = {};       // "A03"
        time_t   start_utc  = 0;
        time_t   end_utc    = 0;        // 0 = open
        char     started_by[32] = {};
        uint8_t  op_index   = 0;        // 0=HOST, 1..N=JOIN
        uint8_t  rollover   = 0;        // 1 = UTC0 자동 시작
        // Mission metadata captured at start (station ctx, written to mission.json)
        char     station_name[64] = {};
        char     host_name[32]    = {};
        float    lat = 0.f, lon = 0.f;
        char     sdr_kind[24]     = {};   // "BladeRF" / "RTL-SDR" / "Pluto"
        char     antenna[64]      = {};
    };
    mutable std::mutex mission_mtx;
    Mission::State mission_state = Mission::State::IDLE;
    int            mission_year = 0;
    char           mission_code[8]        = {};
    char           mission_started_by[32] = {};
    uint8_t        mission_op_index       = 0;
    time_t         mission_start_utc      = 0;
    time_t         mission_end_utc        = 0;
    // 활성 미션 메타데이터 (start 시점 캡처)
    char           mission_station_name[64] = {};
    char           mission_host_name[32]    = {};
    float          mission_lat = 0.f, mission_lon = 0.f;
    char           mission_sdr_kind[24]     = {};
    char           mission_antenna[64]      = {};
    std::vector<MissionEntry> mission_history;
    bool           mission_modal_open       = false;
    bool           mission_start_modal_open = false;
    bool           mission_end_confirm_open = false;

    // mission_view 의 LOCAL 우클릭 → main 페이지 file_ctx 메뉴 (Signal Analysis/
    // Info/Report/Save DB/Delete) 를 동일하게 트리거. mission_view 가 set 하고,
    // run_streaming_viewer() 매 프레임 루프 시작 시 소비.
    struct PendingFileCtx {
        std::atomic<bool> pending{false};
        std::string filepath;
        std::string filename;
        float       x = 0, y = 0;
    } pending_file_ctx;

    // Mission lifecycle (mission.cpp에 정의, thread-safe)
    bool mission_start(const char* started_by, uint8_t op_index, bool rollover);
    bool mission_end();
    void mission_rollover_utc0();
    void mission_load_history();
    void mission_save_meta_to_disk();
    void mission_broadcast_sync();
    // v3.20.0 migration: 기존 recordings/missions/<YYYY>/<code>/ 구조를
    // station-keyed recordings/missions/<station>/<YYYY>/<code>/ 로 한 번만 이동.
    void mission_migrate_old_layout();
    // 특정 미션의 디렉토리(파일 전체) + history 엔트리 영구 삭제.
    // 활성 미션이면 먼저 mission_end() 후 삭제.
    bool mission_delete(int year, const char* code);
    // 현재 활성 미션 디렉토리. IDLE이면 빈 문자열 (호출자가 차단).
    std::string active_iq_dir() const;
    std::string active_audio_dir() const;
    std::string active_hist_dir() const;
    // 현재 활성 미션 station_name (MissionPush key 채우기용). IDLE이면 빈.
    std::string mission_active_station_name() const;
    // 현재 활성 미션 year/code (MissionPush + file LIST_REQ 용).
    int  mission_active_year() const;
    std::string mission_active_code() const;

    // ── Long Waterfall (24h+ FFT magnitude image) ─────────────────────
    bool              lwf_modal_open = false;     // IMG 버튼 토글 → viewer 모달
    std::atomic<int>  lwf_rotate_seq{0};          // SR/CF/fft_size/IQ on-off 변경 시 ++ → worker가 새 파일
    std::atomic<bool> tm_active{false};    // 스페이스바: 타임머신 뷰 모드
    std::atomic<bool> capture_pause{false};// 캡처 스레드 pause (타임머신과 무관)
    std::atomic<bool> net_bcast_pause{false}; // /chassis 2 reset: 방송 일시 중단
    std::atomic<bool> sdr_stream_error{false};  // SDR 스트림 오류 (뽑힘/초기화 실패)
    std::atomic<bool> dem_restart_needed{false}; // SR 변경 후 demod 재시작 필요
    std::atomic<bool> wf_area_visible{true};    // 워터폴 영역 실제 표시 여부 (수평바 포함)
    bool tm_iq_was_stopped=false;
    int  tm_freeze_idx=0;                  // 스페이스바 누른 시점의 fft 인덱스
    float tm_offset=0.0f;                  // 현재 보는 과거 오프셋 (초)
    float tm_max_sec=0.0f;                     // 현재 사용 가능한 최대 과거 초

    // 행별 IQ 데이터 존재 플래그 (MAX_FFTS_MEMORY 크기)
    bool iq_row_avail[MAX_FFTS_MEMORY]={};
    // 각 FFT 행 커밋 시점의 tm_iq_write_sample 기록
    // row_write_pos[fi % MAX_FFTS_MEMORY] = 그 행의 IQ 데이터가 끝나는 파일 위치
    int64_t row_write_pos[MAX_FFTS_MEMORY]={};
    // 각 FFT 행 커밋 시점의 wall_time (밀리초)
    int64_t row_wall_ms[MAX_FFTS_MEMORY]={};

    // ── 워터폴 좌측 이벤트 태그 ───────────────────────────────────────────
    struct WfEvent {
        int   fft_idx;   // 이벤트 발생 시점의 fft 인덱스
        time_t wall_time;// 실제 시각
        int   type;      // 0=시간태그(5초단위), 1=TM_IQ Start, 2=TM_IQ Stop
        char  label[32];
    };
    std::vector<WfEvent> wf_events;
    mutable std::mutex   wf_events_mtx;
    int                  last_tagged_sec=-1; // 마지막으로 5초태그 붙인 초

    // fft_idx → wall_time 변환 (wf_events 보간, 없으면 rps 기반 추정)
    // 반환값: Unix timestamp (time_t), 0이면 변환 불가
    time_t fft_idx_to_wall_time(int fft_idx) const;
    // fft_idx → wall_time_ms 변환 (row_wall_ms 직접 참조, 밀리초 정밀도)
    // 반환값: Unix timestamp * 1000 (int64_t), 0이면 변환 불가
    int64_t fft_idx_to_wall_time_ms(int fft_idx) const;

    // IQ 롤링 파일 관리
    // TM_IQ_DIR: BEWEPaths::time_temp_dir() 로 런타임 결정
    static constexpr size_t      TM_IQ_SECS = 60;     // 롤링 길이 (초)
    int      tm_iq_fd=-1;   // unbuffered POSIX fd
    // IQ 배치 버퍼: 65536샘플 모아서 한 번에 pwrite (syscall 최소화)
    static constexpr int TM_IQ_BATCH = 65536;
    std::vector<int16_t> tm_iq_batch_buf;
    int tm_iq_batch_cnt=0;
    int64_t  tm_iq_write_sample=0;         // 현재 파일 내 쓰기 샘플 위치
    int64_t  tm_iq_total_samples=0;        // 파일 전체 샘플 수 (미리 할당)
    // 초 단위 타임스탬프 배열 [0..TM_IQ_SECS-1]: 각 초 청크의 시작 시각
    time_t   tm_iq_chunk_time[TM_IQ_SECS]={};
    int      tm_iq_chunk_write=0;          // 현재 쓰고 있는 청크 인덱스
    int64_t  tm_iq_chunk_sample_start=0;   // 현재 청크 샘플 시작
    bool     tm_iq_file_ready=false;

    // 영역 녹음 진행 상태
    enum RecState { REC_IDLE, REC_BUSY, REC_SUCCESS } rec_state=REC_IDLE;
    float    rec_anim_timer=0.0f;   // 점 애니메이션용
    float    rec_success_timer=0.0f;// 성공 메시지 표시 시간
    std::atomic<bool> rec_busy_flag{false};

    // 디스플레이용 타임머신 오프셋 FFT 인덱스
    int tm_display_fft_idx=0;

    void tm_iq_open();
    void tm_iq_close();
    void tm_iq_write(const int16_t* samples, int n_pairs);
    void tm_iq_flush_batch();
    void tm_mark_rows(int fft_idx);
    void tm_update_display();
    bool tm_rec_start();
    void tm_add_time_tag(int fft_idx);
    void tm_add_event_tag(int type); // 1=Start, 2=Stop

    // ── 영역 IQ 녹음 (Ctrl+우클릭 드래그) ───────────────────────────────
    struct RegionSel {
        bool   selecting=false;
        bool   active=false;
        float  drag_x0=0, drag_y0=0, drag_x1=0, drag_y1=0;
        float  freq_lo=0, freq_hi=0;
        int    fft_top=0, fft_bot=0;
        int64_t time_start_ms=0, time_end_ms=0;
        int64_t samp_start=0, samp_end=0; // HOST IQ 좌표 직접 지정 시 사용
        int    lclick_count=0;
        float  lclick_timer=0;

        // 이동/리사이즈 상태
        enum EditMode { EDIT_NONE, EDIT_MOVE,
                        EDIT_RESIZE_L, EDIT_RESIZE_R,
                        EDIT_RESIZE_T, EDIT_RESIZE_B } edit_mode=EDIT_NONE;
        float  edit_mx0=0, edit_my0=0;   // 드래그 시작 마우스 위치
        float  edit_flo0=0, edit_fhi0=0; // 드래그 시작 주파수
        int    edit_ftop0=0, edit_fbot0=0; // 드래그 시작 fft 인덱스
    } region;

    void region_save();
    std::string do_region_save_work();

    // ── SA (Signal Analyzer) 패널 ─────────────────────────────────────────
    bool              sa_panel_open  = false;
    int               sa_fft_size    = 1024;
    int               sa_window_type = 0;   // 0=Blackman-Harris 1=Hann 2=Nuttall
    GLuint            sa_texture     = 0;
    int               sa_tex_w       = 0;
    int               sa_tex_h       = 0;
    std::atomic<bool> sa_computing   {false};
    std::thread       sa_thread;
    std::string       sa_temp_path;
    bool              sa_mode        = false;
    float             sa_anim_timer  = 0.0f;  // 로딩 점 애니메이션
    bool              sa_drag_active = false;
    float right_panel_x   = 0.0f;

    // SA 픽셀 버퍼 (스레드 → 메인 스레드 전달)
    std::vector<uint32_t> sa_pixel_buf;
    std::mutex            sa_pixel_mtx;
    std::atomic<bool>     sa_pixel_ready{false};

    void sa_start(const std::string& wav_path);  // 비동기 FFT 계산 시작
    void sa_cleanup();                            // 임시파일 삭제 + 텍스처 해제
    void sa_upload_texture();                     // 메인스레드에서 GL 업로드

    // SA 메타데이터 (WAV BEWE 청크에서 읽음)
    uint64_t sa_center_freq_hz = 0;   // 중심주파수 (Hz)
    int64_t  sa_start_time     = 0;   // 시작 시각 (Unix timestamp)
    uint32_t sa_sample_rate    = 0;   // 출력 샘플레이트 (데시메이션 후)
    int64_t  sa_total_rows     = 0;   // 총 FFT 행 수 (시간축 길이)
    int      sa_actual_fft_n   = 0;   // 실제 FFT 크기 (주파수축 길이)

    // SA 뷰 상태 (줌/팬)
    float    sa_view_x0 = 0.0f;   // 텍스처 UV: 주파수축 시작 [0,1]
    float    sa_view_x1 = 1.0f;   // 텍스처 UV: 주파수축 끝
    float    sa_view_y0 = 0.0f;   // 텍스처 UV: 시간축 시작
    float    sa_view_y1 = 1.0f;   // 텍스처 UV: 시간축 끝

    // SA 범위 선택 (Ctrl+우클릭 드래그)
    bool     sa_sel_active = false;
    float    sa_sel_x0 = 0.f, sa_sel_x1 = 0.f; // 텍스처 UV 좌표
    float    sa_sel_y0 = 0.f, sa_sel_y1 = 0.f;
    bool     sa_sel_dragging = false;
    float    sa_sel_drag_ox = 0.f, sa_sel_drag_oy = 0.f; // 드래그 시작 UV

    // SA 복조 재생
    int      sa_demod_mode = 0;   // 0=없음 1=AM 2=FM
    std::atomic<bool> sa_playing{false};
    std::thread       sa_play_thread;
    void sa_play_demod();         // 선택 영역 복조 재생 (별도 스레드)

    // ── Scheduled IQ Recording ──────────────────────────────────────────
    struct SchedEntry {
        time_t  start_time   = 0;
        float   duration_sec = 0;
        float   freq_mhz     = 0;
        float   bw_khz       = 0;
        enum Status : int { WAITING=0, ARMED=1, RECORDING=2, DONE=3, FAILED=4 } status = WAITING;
        int     temp_ch_idx  = -1;
        std::chrono::steady_clock::time_point rec_started;
        char    operator_name[32] = {};  // 예약자 (HOST면 login_get_id(), JOIN이면 op_name)
        uint8_t op_index          = 0;   // 0=HOST, 1..N=JOIN op_index
        char    target[32]        = {};  // free-form 식별 라벨 (UI에서 운용자 입력)
        // Mission context — HOST stamps these at add time so mission_view can
        // filter the list. Empty year/code = added outside any mission.
        int     mission_year      = 0;
        char    mission_code[8]   = {};
    };
    static constexpr float SCHED_PRE_ARM_SEC = 5.0f;
    std::vector<SchedEntry> sched_entries;
    std::mutex              sched_mtx;
    int   sched_active_idx  = -1;
    float sched_saved_cf    = 0;
    bool  sched_panel_open  = false;
    void sched_tick();
    void sched_arm_entry(int idx);
    void sched_begin_rec(int idx);
    void sched_stop_entry(int idx);
    // Overlap 검사 — [start, start+dur)이 기존 WAITING/RECORDING entry와 겹치는지 (sched_mtx 잡은 채로 호출)
    bool sched_has_overlap(time_t start, float dur) const;
    // 전체 sched 리스트를 JOIN 클라이언트에 브로드캐스트 (SCHED_SYNC)
    void broadcast_sched_list();         // 내부에서 sched_mtx 잡음
    void broadcast_sched_list_locked();  // 호출자가 이미 sched_mtx를 잡은 상태여야 함
    // 예약 녹음 완료 시 자동 DB 업로드 콜백 (cli_host/ui.cpp에서 설정)
    // args: (file_path, operator_name, info_text)
    std::function<void(const std::string& path, const std::string& op, const std::string& info)> sched_db_upload_fn;
    // 동시에 여러 sched가 끝날 때 업로드를 직렬화 (Central 측 동일-룸 단일 db_fp 슬롯 보호)
    std::mutex sched_db_upload_mtx;

    // sched_begin_rec → start_iq_rec 핸드오프: 다음 IQ 녹음이 SCHED 형식 파일명을 쓰도록.
    // sched_begin_rec에서 채워두고 start_iq_rec가 한 번 사용 후 active=false로 클리어.
    struct PendingSchedMeta {
        bool   active = false;
        time_t start_utc = 0;
        time_t end_utc   = 0;
    } pending_sched_meta;

    // ── LOG 오버레이 (L키 토글) ──────────────────────────────────────────
    bool log_panel_open = false;
    struct LogEntry { char msg[512]; };
    static constexpr int LOG_MAX = 500;
    std::vector<LogEntry> log_buf[3];  // 0=HOST 1=SERVER 2=JOIN
    std::mutex log_mtx;
    bool log_scroll[3] = {true,true,true};
    void log_push(int col, const char* fmt, ...);

    // ── DEMOD 모듈 패널 (src/modules/ 설치형 모듈 컨테이너) ──────────────
    bool                 demod_panel_open = false;

    // ── EID (Emitter ID / RF Fingerprint) 패널 ─────────────────────────────
    bool              eid_panel_open = false;
    std::atomic<bool> eid_computing  {false};
    std::thread       eid_thread;
    float             eid_anim_timer = 0.0f;

    // 뷰 모드: 0=Signal(envelope), 1=I/Q, 2=Phase, 3=Frequency
    int eid_view_mode = 0;

    // envelope 데이터
    std::vector<float> eid_envelope;       // sqrt(I²+Q²), 전체 샘플
    std::mutex         eid_data_mtx;
    std::atomic<bool>  eid_data_ready{false};
    int64_t            eid_total_samples = 0;
    uint32_t           eid_sample_rate   = 0;

    // I/Q 분리 데이터
    std::vector<float> eid_ch_i;       // I(t) normalized
    std::vector<float> eid_ch_q;       // Q(t) normalized

    // 순시 위상/주파수
    std::vector<float> eid_phase;      // atan2(Q,I)
    std::vector<float> eid_inst_freq;  // d(phase)/dt

    float eid_phase_detrend_hz = 0.0f;  // sweep line 주파수 오프셋 (Hz)

    // 노이즈 레벨 (Signal 모드용)
    float eid_noise_level = 0.f;

    // center freq (표시용)
    uint64_t eid_center_freq_hz = 0;
    // 원본 WAV bewe 청크 start_time (Save File 시 새 WAV에 보존)
    int64_t  eid_start_time_meta = 0;

    // 뷰 상태 (double: 대용량 샘플 인덱스 정밀도)
    double  eid_view_t0 = 0.0;   // 보이는 시작 (샘플 인덱스)
    double  eid_view_t1 = 0.0;   // 보이는 끝
    float   eid_amp_min = 0.0f;  // 자동 스케일 최소
    float   eid_amp_max = 1.0f;  // 자동 스케일 최대

    // 좌클릭 드래그 영역 선택 (줌)
    bool    eid_sel_active = false;
    float   eid_sel_x0 = 0.f, eid_sel_x1 = 0.f;

    // 뷰 히스토리 (우클릭 뒤로가기)
    std::vector<std::pair<double,double>> eid_view_stack;

    // 태그 영역
    struct EidTag {
        double s0, s1;       // 샘플 인덱스 시작/끝
        ImU32  color;
        char   label[32];
        bool   selected = false;    // 활성화 상태 (클릭 토글)
        float  auto_pri_us = 0;     // 자동 검출 PRI (us)
        float  auto_prf_hz = 0;     // 자동 검출 PRF (Hz)
        int    auto_pulse_count = 0; // 검출된 펄스 수
    };
    void eid_auto_analyze_tag(EidTag& tag);

    // 성상도 재생 상태
    double eid_const_pos = 0;       // 현재 재생 위치 (샘플 인덱스)
    bool   eid_const_playing = false; // 자동 재생 중
    int    eid_const_win = 4096;    // 윈도우 크기 (샘플)
    float  eid_const_zoom = 0.0f;   // 0 = 자동 스케일, >0 = 수동 줌 배율


    // M-th power spectrum 분석 상태
    int    eid_power_order = 1;     // M 값 (1, 2, 4, 8)
    int    eid_power_fft_n = 4096;  // FFT 크기

    // Y축 수동 스케일 (per mode: 0=Amp, 1=I/Q, 2=Phase, 3=Freq)
    float  eid_y_min[4] = {0.f,   -1.f,              -(float)M_PI, -1.f};
    float  eid_y_max[4] = {1.f,    1.f,               (float)M_PI,  1.f};

    // 비트 판단선 (baud mode에서 우클릭 Make Baseline으로 설정)
    bool   eid_baseline_active = false;
    float  eid_baseline_val = 0.f;   // Y축 데이터 값 기준
    int    eid_baseline_imode = 0;   // 설정된 모드 (0=envelope,1=IQ,2=phase,3=freq)

    // 비트 구분 모드 (B키 토글)
    bool   eid_baud_mode = false;
    double eid_baud_s0 = -1;     // 시작 샘플 인덱스 (-1=미설정)
    double eid_baud_s1 = -1;     // 끝 샘플 인덱스 (-1=미설정)
    int    eid_baud_click = 0;   // 0=대기, 1=시작설정됨
    int    eid_baud_drag = -1;   // 드래그 중 선 (-1=없음, 0=시작, 1=끝)
    bool   eid_baud_drag_band = false;    // 밴드 전체 드래그 중
    double eid_baud_band_drag_offset = 0.0; // 드래그 시작 시 s0 기준 마우스 오프셋
    bool    eid_tag_dragging = false;
    float   eid_tag_drag_x0 = 0.f, eid_tag_drag_x1 = 0.f;
    // 임시 선택 영역 (우클릭 드래그 후 확정 전)
    bool    eid_pending_active = false;
    double  eid_pending_s0 = 0.0, eid_pending_s1 = 0.0;
    std::vector<EidTag> eid_tags;

    // Bits 탭 상태
    int    eid_bits_per_row = 128;   // 한 줄당 비트 수 (8~512)
    int    eid_bits_offset = 0;      // 수동 비트 오프셋
    int    eid_bits_view = 0;        // 0=BIN(이진수), 1=HEX(16진수), 2=BITMAP(픽셀)
    int    eid_bits_scroll = 0;      // 스크롤 위치 (줄 단위)
    float  eid_bits_zoom = 1.0f;    // 줌 배율 (1.0 = 기본, Ctrl+휠로 조절)
    float  eid_bits_hscroll = 0.0f; // 수평 스크롤 (픽셀 단위)

    // 스펙트로그램 통합 뷰 히스토리
    struct SaViewEntry { float x0,x1,y0,y1; bool had_bpf; };
    std::vector<SaViewEntry> sa_view_history;

    // BPF 상태 (원본 IQ 백업 + 필터 상태)
    std::vector<float> eid_orig_ch_i;   // 필터 전 원본 I
    std::vector<float> eid_orig_ch_q;   // 필터 전 원본 Q
    bool eid_bpf_active = false;

    // ── Undo/Redo 시스템 ──────────────────────────────────────────────
    struct EidUndoEntry {
        std::vector<float> envelope, ch_i, ch_q, phase, inst_freq;
        std::vector<float> orig_ch_i, orig_ch_q;
        std::vector<EidTag> tags;
        std::vector<std::pair<double,double>> view_stack;
        std::vector<SaViewEntry> sa_history;
        int64_t total_samples;
        double view_t0, view_t1;
        float sa_vx0, sa_vx1, sa_vy0, sa_vy1;
        bool bpf_active;
        bool baud_mode;
        double baud_s0, baud_s1;
        int baud_click;
        bool baseline_active;
        float baseline_val;
        int baseline_imode;
        bool pending_active;
        double pending_s0, pending_s1;
        float y_min[4], y_max[4];
        float amp_min, amp_max;
        float noise_level;
    };
    std::deque<EidUndoEntry> eid_undo_stack;
    std::deque<EidUndoEntry> eid_redo_stack;
    static constexpr int EID_UNDO_MAX = 10;
    EidUndoEntry eid_snapshot() const;
    void eid_restore(const EidUndoEntry& e);
    void eid_push_undo();
    void eid_do_undo();
    void eid_do_redo();

    void eid_start(const std::string& wav_path);
    void eid_cleanup();
    void eid_remove_samples(double s0, double s1);
    void eid_select_samples(double s0, double s1);
    void eid_apply_bpf(float uv_lo, float uv_hi);  // UV [0,1] 주파수 좌표
    void eid_undo_bpf();
    void eid_recompute_derived();
    void sa_recompute_from_iq(bool reset_view = false);
    // Save File: 현재 eid_ch_i/q 상태(필터·샘플 수정 반영)를 원본 폴더에 새 WAV로 저장.
    // 파일명: IQ_Filtered_... / Audio_Filtered_... 형식, 중복 시 _2, _3 접미.
    // 원본 .info가 있으면 같은 규칙으로 복사. 반환: 생성 경로(실패 시 "").
    std::string eid_save_filtered();
    // 기본 저장 경로 계산만 수행 (파일명 결정, 중복 _N 처리)
    std::string eid_default_filtered_path();
    // 지정 경로에 WAV만 저장 (헤더/bewe 청크/stereo int16). .info는 호출자 책임.
    std::string eid_save_filtered_to(const std::string& out_path);

    // .info 파일의 Recorder 필드용 장비 표시명 (HOST/Local=hw.type, JOIN=remote_hw)
    const char* recorder_name() const {
        if(remote_mode && net_cli){
            uint8_t rh = net_cli->remote_hw.load();
            if(rh == 0) return "BladeRF 2.0 micro xA9 (12bit ADC)";
            if(rh == 1) return "RTL-SDR v4 (8bit ADC)";
            if(rh == 2) return "ADALM Pluto SDR (12bit ADC)";
            return "";
        }
        return hw_recorder_name(hw.type);
    }
    // 시간 기준: 항상 KST (UTC+9). 시스템 TZ나 station 위치와 무관.
    int utc_offset_hours() const { return 9; }

    // tm_rec 내부 상태
    bool    tm_rec_active=false;
    int64_t tm_rec_read_pos=0; // R키 실행 시 파일 추출
    std::vector<float> current_spectrum;
    int   cached_sp_idx=-1; float cached_pan=-999, cached_zoom=-999;
    int   cached_px=-1;     float cached_pmin=-999, cached_pmax=-999;
    // autoscale: 고정 크기 순환 버퍼 (push_back per FFT 제거)
    // 최대 fft_size*100 샘플 고정 → nth_element 시 재할당 없음
    std::vector<float> autoscale_accum;
    size_t             autoscale_wp=0;   // 순환 버퍼 write pointer
    bool               autoscale_buf_full=false;
    std::chrono::steady_clock::time_point autoscale_last;
    std::chrono::steady_clock::time_point autoscale_check_last{};  // 10s 천장초과 감시 타이머
    bool  autoscale_init=false, autoscale_active=true;
    // 비-캡처 스레드(set_frequency/init)가 autoscale 재트리거를 요청 → 캡처 스레드가 처리.
    // autoscale_accum/active/init 를 캡처 스레드 밖에서 직접 건드리면 레이스 → 이 플래그로 위임.
    std::atomic<bool> autoscale_req{false};
    std::atomic<bool> spectrum_pause{false};

    // ── Network ──────────────────────────────────────────────────────────
    NetServer*  net_srv   = nullptr;  // HOST 모드
    NetClient*  net_cli   = nullptr;  // CONNECT 모드
    bool        remote_mode = false;  // true = CONNECT 모드 (하드웨어 없음)
    char        host_name[32] = {};   // 접속한 유저 ID (표시용)
    char        host_antenna[32] = {}; // HOST 안테나 자유텍스트 (Central 통해 동기화)
    // 런타임 SDR 교체: UI 클릭 시 아래 플래그/이름 세팅 → 메인 루프가 교체 수행
    std::atomic<bool> pending_sdr_switch{false};
    std::mutex        pending_sdr_mtx;
    std::string       pending_sdr_name; // "bladerf" | "pluto" | "rtlsdr"
    uint8_t     my_op_index   = 0;

    // ── Globe / Station Discovery ─────────────────────────────────────────
    struct DiscoveredStation {
        std::string name;
        std::string station_id;    // relay 모드: 룸 ID; LAN 모드: ""
        float       lat        = 0.f;
        float       lon        = 0.f;
        uint16_t    tcp_port   = 0;
        std::string ip;
        uint8_t     user_count = 0;
        uint8_t     host_tier  = 1;
        double      last_seen  = 0.0; // glfwGetTime()
    };
    std::vector<DiscoveredStation> discovered_stations;
    std::mutex                     discovered_stations_mtx;

    // Station identity (set during HOST mode placement on globe)
    std::string station_name;
    float       station_lat          = 0.f;
    float       station_lon          = 0.f;
    bool        station_location_set = false;

    // 로컬 오디오 출력 선택 (각 PC 독립): 0=L, 1=L+R, 2=R, 3=M(mute)
    // JOIN에서 M이면 cmd_toggle_recv(ch, false) 전송
    std::array<int,MAX_CHANNELS> local_ch_out = []{ std::array<int,MAX_CHANNELS> a; a.fill(1); return a; }(); // 기본: L+R
    bool ch_created_by_me[MAX_CHANNELS] = {}; // JOIN: 내가 생성한 채널 여부
    bool ch_pending_create[MAX_CHANNELS] = {}; // JOIN: CMD_CREATE_CH 송신 후 HOST 확인 전 (stale sync 무시용)
    // JOIN: 서버에서 수신한 전체 audio_mask (리스너 표시용)
    uint32_t srv_audio_mask[MAX_CHANNELS] = {};
    // JOIN: 오디오 녹음 시작 전 뮤트 상태 저장 (녹음 후 복원용)
    bool join_rec_was_muted[MAX_CHANNELS] = {};

    // 파일 전송 진행상태 (HOST: 전송 중, JOIN: 수신 중)
    struct FileXfer {
        std::string filename;
        uint64_t    total_bytes = 0;
        uint64_t    done_bytes  = 0;
        bool        finished    = false;
        bool        is_sa       = false;   // SA로 열 수 있는 파일
        std::string local_path;
        enum Dir : uint8_t { DIR_UNKNOWN=0, DIR_DOWNLOAD=1, DIR_UPLOAD=2 } dir = DIR_UNKNOWN;
        // 속도 측정 (EWMA bytes/sec) — render에서 갱신
        int64_t     last_done_bytes = 0;
        int64_t     last_steady_us  = 0;   // steady_clock microseconds
        double      bps_ewma        = 0.0;
    };
    std::vector<FileXfer> file_xfers;
    std::mutex            file_xfer_mtx;
    std::atomic<uint8_t>  next_transfer_id{1};

    // 파일 리스트 한 줄 정보 포맷 — "HH:MM:SS ###.#M" / "HH:MM:SS ###.#G".
    // M/G 모두 5글자 폭으로 통일되어 컬럼 정렬됨.
    // Archive / HIST / DB Archive 공통 사용.
    static inline std::string format_file_info(double sec, uint64_t bytes){
        char tbuf[16] = "";
        if(sec > 0){
            uint64_t s = (uint64_t)sec;
            snprintf(tbuf, sizeof(tbuf), "%02llu:%02llu:%02llu",
                (unsigned long long)(s/3600),
                (unsigned long long)((s/60)%60),
                (unsigned long long)(s%60));
        }
        char sbuf[16];
        // 통일 형식: "%.1f MB" (IQ/DEMOD 동일).
        if(bytes >= (1ULL<<30))      snprintf(sbuf, sizeof(sbuf), "%.1f GB", bytes/(1024.0*1024.0*1024.0));
        else if(bytes >= (1ULL<<20)) snprintf(sbuf, sizeof(sbuf), "%.1f MB", bytes/(1024.0*1024.0));
        else if(bytes >= (1ULL<<10)) snprintf(sbuf, sizeof(sbuf), "%.1f KB", bytes/1024.0);
        else                          snprintf(sbuf, sizeof(sbuf), "%llu B", (unsigned long long)bytes);
        char buf[40];
        if(tbuf[0]) snprintf(buf, sizeof(buf), "%s %s", tbuf, sbuf);
        else        snprintf(buf, sizeof(buf), "%s", sbuf);
        return buf;
    }

    // 단일 전송 항목 렌더링 (Archive / HIST 공통 사용).
    // x의 EWMA 속도 필드(last_*, bps_ewma)는 호출마다 갱신됨.
    static inline void render_file_xfer_row(FileXfer& x){
    #ifndef BEWE_HEADLESS
        const char* dir_label =
            (x.dir == FileXfer::DIR_UPLOAD)   ? "Upload" :
            (x.dir == FileXfer::DIR_DOWNLOAD) ? "Download" :
                                                "Transfer";
        ImVec4 col = (x.dir == FileXfer::DIR_UPLOAD)
            ? ImVec4(1.0f, 0.85f, 0.4f, 1.f)
            : ImVec4(0.5f, 0.95f, 1.0f, 1.f);
        ImGui::PushStyleColor(ImGuiCol_Text, col);
        ImGui::Text("%s", dir_label);
        ImGui::PopStyleColor();
        float frac = (x.total_bytes > 0)
            ? (float)((double)x.done_bytes / (double)x.total_bytes) : 0.f;
        if(frac < 0.f) frac = 0.f;
        if(frac > 1.f) frac = 1.f;
        auto fmt_sz = [](uint64_t b, char* o, size_t sz){
            if(b < 1024)                    snprintf(o,sz,"%llu B",(unsigned long long)b);
            else if(b < 1024*1024)          snprintf(o,sz,"%.1f KB",(double)b/1024);
            else if(b < 1024ULL*1024*1024)  snprintf(o,sz,"%.1f MB",(double)b/(1024*1024));
            else                            snprintf(o,sz,"%.2f GB",(double)b/(1024ULL*1024*1024));
        };
        char dn[32], tn[32];
        fmt_sz(x.done_bytes, dn, sizeof(dn));
        fmt_sz(x.total_bytes, tn, sizeof(tn));
        int64_t now_us = (int64_t)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if(x.last_steady_us == 0){
            x.last_steady_us = now_us; x.last_done_bytes = (int64_t)x.done_bytes;
        } else if(now_us - x.last_steady_us >= 200000){
            int64_t db = (int64_t)x.done_bytes - x.last_done_bytes;
            double  dt = (now_us - x.last_steady_us) / 1e6;
            if(dt > 0){
                double inst_bps = (db > 0) ? (double)db / dt : 0.0;
                x.bps_ewma = (x.bps_ewma == 0.0) ? inst_bps
                                                 : x.bps_ewma * 0.7 + inst_bps * 0.3;
            }
            x.last_steady_us  = now_us;
            x.last_done_bytes = (int64_t)x.done_bytes;
        }
        char sn[24] = "";
        if(x.bps_ewma > 0.0){
            if(x.bps_ewma < 1024)                  snprintf(sn,sizeof(sn)," (%.0f B/s)",  x.bps_ewma);
            else if(x.bps_ewma < 1024*1024)        snprintf(sn,sizeof(sn)," (%.1f KB/s)", x.bps_ewma/1024);
            else if(x.bps_ewma < 1024.0*1024*1024) snprintf(sn,sizeof(sn)," (%.2f MB/s)", x.bps_ewma/(1024*1024));
            else                                   snprintf(sn,sizeof(sn)," (%.2f GB/s)", x.bps_ewma/(1024.0*1024*1024));
        }
        char buf[128];
        snprintf(buf,sizeof(buf),"%s / %s%s  %.0f%%", dn, tn, sn, frac*100.f);
        ImGui::ProgressBar(frac, ImVec2(-1, 0), buf);
        ImGui::TextDisabled("  %s", x.filename.c_str());
    #else
        (void)x;
    #endif
    }

    // ── 브로드캐스트 전용 스레드 (캡처 스레드와 분리) ──────────────────
    std::atomic<int>        net_bcast_seq{0};   // 캡처 스레드가 올림
    std::mutex              net_bcast_mtx;
    std::condition_variable net_bcast_cv;
    std::atomic<bool>       net_bcast_stop{false};
    std::thread             net_bcast_thr;

    void net_bcast_worker();  // 선언

    // ── Hardware (공통) ───────────────────────────────────────────────────
    HWConfig hw;                          // 런타임 HW 파라미터
    struct bladerf*  dev_blade = nullptr; // BladeRF 디바이스
    rtlsdr_dev_t*    dev_rtl   = nullptr; // RTL-SDR 디바이스
    // ADALM-Pluto (libiio) — void* 로 선언해 header include 오염 방지
    void*            pluto_ctx       = nullptr; // iio_context*
    void*            pluto_phy_dev   = nullptr; // iio_device* (ad9361-phy)
    void*            pluto_rx_dev    = nullptr; // iio_device* (cf-ad9361-lpc)
    void*            pluto_rx_i_ch   = nullptr; // iio_channel* voltage0
    void*            pluto_rx_q_ch   = nullptr; // iio_channel* voltage1
    void*            pluto_rx_buf    = nullptr; // iio_buffer*
    fftwf_plan      fft_plan=nullptr;
    fftwf_complex  *fft_in=nullptr, *fft_out=nullptr;
    bool  is_running=true;
    std::atomic<bool> rx_stopped{false};  // /rx stop: SDR 의도적 중단 (자동 재연결 방지)
    int   total_ffts=0;
    std::string window_title;
    std::mutex  data_mtx;
    float pending_cf=0; bool freq_req=false, freq_prog=false;
    bool  sc8_mode=false; // SC8_Q7 모드 (122.88 MSPS)
    std::atomic<uint64_t> live_cf_hz{0};  // 스레드 안전 현재 중심주파수 (Hz)

    // ── IQ Ring ───────────────────────────────────────────────────────────
    std::vector<int16_t> ring;
    std::atomic<size_t>  ring_wp{0};

    // ── Channels ──────────────────────────────────────────────────────────
    Channel channels[MAX_CHANNELS];
    int     selected_ch=-1;

    struct NewDrag{ bool active=false; float anch=0,s=0,e=0; } new_drag;

    // ── Record 탭 표시용 항목 ─────────────────────────────────────────────
    struct RecEntry {
        std::string path;       // 전체 경로
        std::string filename;   // 표시용 파일명
        bool        finished = false;
        bool        is_audio = false; // false=IQ, true=복조오디오
        bool        is_region = false; // true=선택영역 IQ
        // 요청 상태 (JOIN 측 region IQ 요청 및 HOST 측 표시용)
        enum ReqState { REQ_NONE=0, REQ_PENDING, REQ_CONFIRMED, REQ_DENIED, REQ_TRANSFERRING } req_state = REQ_NONE;
        float       req_deny_timer = 5.f; // DENY 후 자동 제거 카운트다운
        uint64_t    xfer_done = 0, xfer_total = 0; // 전송 진행
        uint8_t     req_op_idx = 0;  // HOST: 요청한 op_idx
        char        req_op_name[32] = {}; // HOST: 요청한 op 이름
        int32_t     req_fft_top = 0, req_fft_bot = 0;
        float       req_freq_lo = 0, req_freq_hi = 0;
        int32_t     req_time_start = 0, req_time_end = 0; // Unix timestamps from JOIN
        uint32_t    req_sr = 0;       // JOIN: region IQ SR (IQ_CHUNK START 동봉, .sigmf-meta용)
        std::string local_path_to_delete; // HOST: 전송 후 삭제할 파일 경로
        std::chrono::steady_clock::time_point t_start; // 시작 시각
        std::chrono::steady_clock::time_point t_last_tick; // 이전 프레임 시각 (Holding 중 정지용)
        float       total_elapsed = 0.f; // Holding 정지 반영된 경과 시간(초)
        int         ch_idx = -1;  // 오디오 녹음 채널 인덱스
    };
    std::vector<RecEntry> rec_entries;
    std::mutex            rec_entries_mtx;

    // 광대역 모듈(WiFi 등)이 full-rate IQ ring 공급을 요청 — rec/dem 없이 ring 채움.
    // (복조 없는 채널필터에 모듈 활성 시 host_start 가 raise, host_stop/on_ch_stop 가 lower)
    std::atomic<bool>     mod_wants_ring{false};

    // ── IQ Recording ──────────────────────────────────────────────────────
    std::atomic<bool>     rec_on{false}, rec_stop{false};
    std::thread           rec_thr;
    std::atomic<size_t>   rec_rp{0};
    float                 rec_cf_mhz=0;
    uint32_t              rec_sr=0;
    int                   rec_ch=-1;
    std::string           rec_filename;
    std::atomic<uint64_t> rec_frames{0};
    std::chrono::steady_clock::time_point rec_t0;

    // ── Audio mix ─────────────────────────────────────────────────────────
    std::atomic<bool> mix_stop{false};
    std::thread       mix_thr;

    // ── EID Audio 탭 재생 ─────────────────────────────────────────────────
    // EID 패널 Audio 탭에서 좌클릭 cursor → 스페이스로 재생/일시정지
    std::unique_ptr<AudioPlayback> audio_player;
    int64_t eid_audio_cursor_sample = 0;  // 좌클릭으로 설정된 cursor (샘플 인덱스)
    void  audio_play_start(const std::string& path, double offset_sec);
    void  audio_play_stop();
    void  audio_play_pause();
    void  audio_play_resume();
    bool  audio_play_active() const;
    bool  audio_play_paused() const;
    float audio_play_pos_sec() const;
    float audio_play_total_sec() const;
    const std::string& audio_play_path() const;
    // ── IQ 파일 Audio 탭: AM/FM 복조 후 재생 ──────────────────────────────
    // IQ(stereo) 녹음을 Audio 탭에서 AM/FM 복조해 임시 mono WAV 로 듣기.
    bool        eid_is_iq = false;       // 현재 로드된 EID 파일이 IQ(stereo)인가
    int         eid_audio_demod = 1;     // 0=AM 1=FM (Audio 탭 상단 버튼)
    float       eid_bpf_center_uv = 0.5f;// 활성 BPF 대역 중심 UV (0.5=DC) — 복조 전 재중심용
    uint64_t    eid_edit_gen = 0;        // eid_ch_i/q 수정(BPF/remove/undo) 세대 — 복조 캐시 무효화용
    std::string eid_iq_tmp_path;         // 복조 결과 임시 wav
    std::string eid_iq_tmp_src;          // 그 임시 wav 가 어느 소스/모드로 만들어졌는지
    int         eid_iq_tmp_mode = -1;
    uint64_t    eid_iq_tmp_gen = (uint64_t)-1;  // 그 임시 wav 가 만들어진 시점의 edit_gen
    std::string eid_iq_demod_tempwav(int am_fm);  // eid_ch_i/q → AM/FM mono wav, path 반환
    void        eid_audio_play(double off_sec);    // IQ면 복조 wav, 아니면 원본 재생

    // ── hw_detect / bladerf_io / rtlsdr_io ───────────────────────────────
    bool initialize(float cf_mhz, float sr_msps = 0.f);  // sr_msps=0 → HW별 기본 (BladeRF 61.44, Pluto 3.2)
    bool initialize_bladerf(float cf_mhz, float sr_msps);
    bool initialize_rtlsdr(float cf_mhz);
    bool initialize_pluto(float cf_mhz, float sr_msps);
    float pluto_get_temp_c() const;  // AD9361 내부 온도 °C (실패 시 음수)
    void capture_and_process();
    void capture_and_process_rtl();
    void capture_and_process_pluto();
    void set_frequency(float cf_mhz);
    void set_gain(float db);
    float gain_db = 0.0f;

    // ── 채널 스컬치 (UI 스레드, FFT 기반) ──────────────────────────────────
    void update_channel_squelch();

    // ── demod.cpp ─────────────────────────────────────────────────────────
    void dem_worker(int ch_idx);
    void start_dem(int ch_idx, Channel::DemodMode mode);
    void stop_dem(int ch_idx, bool stop_decoders=true);  // stop_decoders=false: 재튜닝(디코더 보존)

    void stop_all_dem();
    void update_dem_by_freq(float new_cf_mhz); // 주파수 변경 시 복조 pause/resume

    // ── iq_record.cpp ─────────────────────────────────────────────────────
    void rec_worker();
    void start_rec();
    void stop_rec();
    void start_audio_rec(int ch_idx);
    void stop_audio_rec(int ch_idx);
    void start_iq_rec(int ch_idx);
    void stop_iq_rec(int ch_idx);
    void iq_only_worker(int ch_idx);  // demod 우회 IQ-only 녹음 worker

    void start_join_audio_rec(int ch_idx); // JOIN 모드 로컬 오디오 녹음
    void stop_join_audio_rec(int ch_idx);

    // ── audio.cpp ─────────────────────────────────────────────────────────
    void mix_worker();

    // ── fft_viewer.cpp (waterfall + display helpers) ──────────────────────
    void create_waterfall_texture();
    void update_wf_row(int fi);
    void get_disp(float& ds, float& de) const;
    float x_to_abs(float x, float gx, float gw) const;
    float abs_to_x(float abs_mhz, float gx, float gw) const;
    int   channel_at_x(float mx, float gx, float gw) const;
    int   freq_sorted_display_num(int arr_idx) const;

#ifndef BEWE_HEADLESS
    // ── ui.cpp (rendering — GUI only) ────────────────────────────────────
    void handle_new_channel_drag(float gx, float gw);
    void handle_channel_interactions(float gx, float gw, float gy, float gh);
    void draw_all_channels(ImDrawList* dl, float gx, float gw, float gy, float gh, bool show_label);
    void draw_freq_axis(ImDrawList* dl, float gx, float gw, float gy, float gh, bool ticks_only=false);
    void handle_zoom_scroll(float gx, float gw, float mouse_x);
    void draw_spectrum_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
#endif

    // ── 주파수 축 드래그 (center frequency 이동) ────────────────────────
    bool   freq_drag_active = false;
    float  freq_drag_start_x = 0;     // 드래그 시작 시 마우스 x
    float  freq_drag_start_cf = 0;    // 드래그 시작 시 center_frequency (MHz)
#ifndef BEWE_HEADLESS
    void draw_waterfall_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
#endif
};

// ── Entry point ───────────────────────────────────────────────────────────
#ifdef BEWE_HEADLESS
void run_cli_host();
#else
void run_streaming_viewer();
#endif

// ── BladeRF USB 소프트 리셋 (sudo 불필요, udev rule 권한 사용) ─────────────
// USBDEVFS_RESET ioctl: 물리적으로 뽑았다 꽂는 것과 동일한 효과
bool bladerf_usb_reset();

// ── RTL-SDR USB 소프트 리셋 (VID:PID 0bda:2838) ───────────────────────────
// rtlsdr_read_sync 가 wedge된 커널 sync-URB 큐에서 영구 hang 할 때, 단순 reopen
// 으로는 안 풀리고 포트 re-enumeration 만 복구됨. USBDEVFS_RESET ioctl 사용.
bool rtl_usb_reset();

// ── DEMOD 모듈 패널 렌더 (demod_panel.cpp) ────────────────────────────────
void demod_draw_panel(FFTViewer& v, bool just_opened);