#pragma once
#include "config.hpp"
#include "bewe_paths.hpp"
#include "net_server.hpp"
#include "net_client.hpp"
#include "hw_config.hpp"
#include "channel.hpp"

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
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <condition_variable>

// ── Global log helper (ui.cpp에서 정의, 모든 .cpp에서 사용 가능) ─────────
extern void bewe_log(const char* fmt, ...);
// LOG 오버레이용 글로벌 로그 (col: 0=HOST 1=SERVER 2=JOIN)
extern void bewe_log_push(int col, const char* fmt, ...);

// ── FFTViewer ─────────────────────────────────────────────────────────────
class FFTViewer {
public:
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

    // ── System monitor (bottom bar) ───────────────────────────────────────
    float sysmon_cpu=0, sysmon_ghz=0, sysmon_ram=0, sysmon_io=0;

    // ── Timemachine ───────────────────────────────────────────────────────
    // 워터폴/스펙트럼: 항상 2500행 메모리 유지 (위 MAX_FFTS_MEMORY)
    // IQ 롤링: T키로 활성화
    std::atomic<bool> tm_iq_on{false};     // T키: IQ SSD 롤링 활성
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

    // REC N/A 표시 타이머
    float    rec_na_timer=0.0f;  // (deprecated, kept for compat)
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
        time_t time_start=0, time_end=0;
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
    void do_region_save_work();

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
        enum Status : int { WAITING=0, RECORDING=1, DONE=2, FAILED=3 } status = WAITING;
        int     temp_ch_idx  = -1;
        std::chrono::steady_clock::time_point rec_started;
    };
    std::vector<SchedEntry> sched_entries;
    std::mutex              sched_mtx;
    int   sched_active_idx  = -1;
    float sched_saved_cf    = 0;
    bool  sched_panel_open  = false;
    void sched_tick();
    void sched_start_entry(int idx);
    void sched_stop_entry(int idx);

    // ── LOG 오버레이 (L키 토글) ──────────────────────────────────────────
    bool log_panel_open = false;
    struct LogEntry { char msg[512]; };
    static constexpr int LOG_MAX = 500;
    std::vector<LogEntry> log_buf[3];  // 0=HOST 1=SERVER 2=JOIN
    std::mutex log_mtx;
    bool log_scroll[3] = {true,true,true};
    void log_push(int col, const char* fmt, ...);

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
    int    eid_const_win = 1024;    // 윈도우 크기 (샘플)
    float  eid_const_zoom = 0.0f;   // 0 = 자동 스케일, >0 = 수동 줌 배율

    // M-th power spectrum 분석 상태
    int    eid_power_order = 1;     // M 값 (1, 2, 4, 8)
    int    eid_power_fft_n = 4096;  // FFT 크기

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

    // 스펙트로그램 통합 뷰 히스토리 (우클릭 1단계 undo)
    struct SaViewEntry { float x0,x1,y0,y1; bool had_bpf; };
    std::vector<SaViewEntry> sa_view_history;

    // BPF 상태 (원본 IQ 백업 + 필터 상태)
    std::vector<float> eid_orig_ch_i;   // 필터 전 원본 I
    std::vector<float> eid_orig_ch_q;   // 필터 전 원본 Q
    bool eid_bpf_active = false;

    void eid_start(const std::string& wav_path);
    void eid_cleanup();
    void eid_remove_samples(double s0, double s1);
    void eid_select_samples(double s0, double s1);
    void eid_apply_bpf(float uv_lo, float uv_hi);  // UV [0,1] 주파수 좌표
    void eid_undo_bpf();
    void eid_recompute_derived();
    void sa_recompute_from_iq(bool reset_view = false);

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
    bool  autoscale_init=false, autoscale_active=true;
    std::atomic<bool> spectrum_pause{false};

    // ── Network ──────────────────────────────────────────────────────────
    NetServer*  net_srv   = nullptr;  // HOST 모드
    NetClient*  net_cli   = nullptr;  // CONNECT 모드
    bool        remote_mode = false;  // true = CONNECT 모드 (하드웨어 없음)
    char        host_name[32] = {};   // 접속한 유저 ID (표시용)
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
    int  local_ch_out[MAX_CHANNELS] = {1,1,1,1,1,1,1,1,1,1}; // 기본: L+R
    bool ch_created_by_me[MAX_CHANNELS] = {}; // JOIN: 내가 생성한 채널 여부
    bool digi_panel_on[MAX_CHANNELS] = {};   // 채널별 디지털 버튼 패널 표시 여부 (D키 토글)
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
    };
    std::vector<FileXfer> file_xfers;
    std::mutex            file_xfer_mtx;
    std::atomic<uint8_t>  next_transfer_id{1};

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
    fftwf_plan      fft_plan=nullptr;
    fftwf_complex  *fft_in=nullptr, *fft_out=nullptr;
    bool  is_running=true;
    std::atomic<bool> rx_stopped{false};  // /rx stop: SDR 의도적 중단 (자동 재연결 방지)
    int   total_ffts=0;
    std::string window_title;
    std::mutex  data_mtx;
    float pending_cf=0; bool freq_req=false, freq_prog=false;
    bool  sc8_mode=false; // SC8_Q7 모드 (122.88 MSPS)

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
        float       req_deny_timer = 30.f; // DENY 후 자동 제거 카운트다운
        uint64_t    xfer_done = 0, xfer_total = 0; // 전송 진행
        uint8_t     req_op_idx = 0;  // HOST: 요청한 op_idx
        char        req_op_name[32] = {}; // HOST: 요청한 op 이름
        int32_t     req_fft_top = 0, req_fft_bot = 0;
        float       req_freq_lo = 0, req_freq_hi = 0;
        int32_t     req_time_start = 0, req_time_end = 0; // Unix timestamps from JOIN
        std::string local_path_to_delete; // HOST: 전송 후 삭제할 파일 경로
        std::chrono::steady_clock::time_point t_start; // 시작 시각
        int         ch_idx = -1;  // 오디오 녹음 채널 인덱스
    };
    std::vector<RecEntry> rec_entries;
    std::mutex            rec_entries_mtx;

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

    // ── hw_detect / bladerf_io / rtlsdr_io ───────────────────────────────
    bool initialize(float cf_mhz, float sr_msps = 61.44f); // HW 자동 감지 후 초기화
    bool initialize_bladerf(float cf_mhz, float sr_msps);
    bool initialize_rtlsdr(float cf_mhz);
    void capture_and_process();
    void capture_and_process_rtl();
    void set_frequency(float cf_mhz);
    void set_gain(float db);
    float gain_db = 0.0f;

    // ── 채널 스컬치 (UI 스레드, FFT 기반) ──────────────────────────────────
    void update_channel_squelch();

    // ── demod.cpp ─────────────────────────────────────────────────────────
    void dem_worker(int ch_idx);
    void start_dem(int ch_idx, Channel::DemodMode mode);
    void stop_dem(int ch_idx);

    // ── ais.cpp ───────────────────────────────────────────────────────────
    void ais_worker(int ch_idx);
    void start_digi(int ch_idx, Channel::DigitalMode mode);
    void stop_digi(int ch_idx);
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