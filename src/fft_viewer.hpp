#pragma once
#include "config.hpp"
#include "bewe_paths.hpp"
#include "net_server.hpp"
#include "net_client.hpp"
#include "hw_config.hpp"
#include "channel.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <libbladeRF.h>
#include <rtl-sdr.h>
#include <mbelib.h>
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

// ── FFTViewer ─────────────────────────────────────────────────────────────
class FFTViewer {
public:
    // ── FFT / waterfall data ──────────────────────────────────────────────
    FFTHeader            header;
    std::vector<int8_t>  fft_data;
    GLuint               waterfall_texture=0;
    std::vector<uint32_t> wf_row_buf;

    int   fft_size=DEFAULT_FFT_SIZE, time_average=TIME_AVERAGE;
    bool  fft_size_change_req=false; int pending_fft_size=DEFAULT_FFT_SIZE;
    bool  texture_needs_recreate=false;
    int   current_fft_idx=0, last_wf_update_idx=-1;
    float freq_zoom=1, freq_pan=0;
    float display_power_min=-80, display_power_max=0;
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
    std::mutex           wf_events_mtx;
    int                  last_tagged_sec=-1; // 마지막으로 5초태그 붙인 초

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
    int               sa_fft_size    = 8192;
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

    // tm_rec 내부 상태
    bool    tm_rec_active=false;
    int64_t tm_rec_read_pos=0; // R키 실행 시 파일 추출
    std::vector<float> current_spectrum;
    int   cached_sp_idx=-1; float cached_pan=-999, cached_zoom=-999;
    int   cached_px=-1;     float cached_pmin=-999, cached_pmax=-999;
    std::vector<float> autoscale_accum;
    std::chrono::steady_clock::time_point autoscale_last;
    bool  autoscale_init=false, autoscale_active=true;
    std::atomic<bool> spectrum_pause{false};

    // ── Network ──────────────────────────────────────────────────────────
    NetServer*  net_srv   = nullptr;  // HOST 모드
    NetClient*  net_cli   = nullptr;  // CONNECT 모드
    bool        remote_mode = false;  // true = CONNECT 모드 (하드웨어 없음)
    char        host_name[32] = {};   // 접속한 유저 ID (표시용)
    uint8_t     my_op_index   = 0;

    // 로컬 오디오 출력 선택 (각 PC 독립): 0=L, 1=L+R, 2=R, 3=M(mute)
    // JOIN에서 M이면 cmd_toggle_recv(ch, false) 전송
    int  local_ch_out[5] = {1,1,1,1,1}; // 기본: L+R

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
    int   total_ffts=0;
    std::string window_title;
    std::mutex  data_mtx;
    float pending_cf=0; bool freq_req=false, freq_prog=false;

    // ── IQ Ring ───────────────────────────────────────────────────────────
    std::vector<int16_t> ring;
    std::atomic<size_t>  ring_wp{0};

    // ── Channels ──────────────────────────────────────────────────────────
    Channel channels[MAX_CHANNELS];
    int     selected_ch=-1;
    bool    topbar_sel_this_frame=false;

    struct NewDrag{ bool active=false; float anch=0,s=0,e=0; } new_drag;

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
    bool initialize(float cf_mhz);          // HW 자동 감지 후 초기화
    bool initialize_bladerf(float cf_mhz, float sr_msps);
    bool initialize_rtlsdr(float cf_mhz);
    void capture_and_process();
    void capture_and_process_rtl();
    void set_frequency(float cf_mhz);
    void set_gain(float db);
    float gain_db = 0.0f;

    // ── demod.cpp ─────────────────────────────────────────────────────────
    void dem_worker(int ch_idx);
    void dmr_worker(int ch_idx);
    void start_dem(int ch_idx, Channel::DemodMode mode);
    void stop_dem(int ch_idx);
    void stop_all_dem();
    void update_dem_by_freq(float new_cf_mhz); // 주파수 변경 시 복조 pause/resume

    // ── iq_record.cpp ─────────────────────────────────────────────────────
    void rec_worker();
    void start_rec();
    void stop_rec();

    // ── audio.cpp ─────────────────────────────────────────────────────────
    void mix_worker();

    // ── fft_viewer.cpp (waterfall + display helpers) ──────────────────────
    void create_waterfall_texture();
    void update_wf_row(int fi);
    void get_disp(float& ds, float& de) const;
    float x_to_abs(float x, float gx, float gw) const;
    float abs_to_x(float abs_mhz, float gx, float gw) const;
    int   channel_at_x(float mx, float gx, float gw) const;

    // ── ui.cpp ────────────────────────────────────────────────────────────
    void handle_new_channel_drag(float gx, float gw);
    void handle_channel_interactions(float gx, float gw, float gy, float gh);
    void draw_all_channels(ImDrawList* dl, float gx, float gw, float gy, float gh, bool show_label);
    void draw_freq_axis(ImDrawList* dl, float gx, float gw, float gy, float gh, bool ticks_only=false);
    void handle_zoom_scroll(float gx, float gw, float mouse_x);
    void draw_spectrum_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
    void draw_waterfall_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
};

// ── Entry point ───────────────────────────────────────────────────────────
void run_streaming_viewer();