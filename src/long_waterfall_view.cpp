// Long Waterfall viewer modal.
// Layout: viewer (left, large) + resizable splitter + file list (right, status-style).
// Y=freq (top=high, bottom=low, linear FFT-shifted). X=time (left=old, right=new).
// Wheel = X(time) cursor-anchored zoom; Ctrl+wheel = Y(freq) cursor-anchored zoom.
// Arrow ←/→ = pan one full screen (EID parity).
// File click = select. Right-click = context menu (Info / Delete).
//
// Source files:
//   - HOST 자기 파일: ~/BE_WE/recordings/long_waterfall/*.bewewf  (worker가 직접 기록)
//   - JOIN 다운로드: 같은 디렉토리에 host에서 받은 파일을 같은 이름으로 저장

#include "long_waterfall.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"
#include "net_client.hpp"

#include <imgui.h>
#include <GL/glew.h>

#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

// ── Jet colormap ─────────────────────────────────────────────────────────
ImU32 jet_color(uint8_t v){
    float t = v / 255.0f;
    float r = std::max(0.0f, std::min(1.0f, 1.5f - std::fabs(4.0f*t - 3.0f)));
    float g = std::max(0.0f, std::min(1.0f, 1.5f - std::fabs(4.0f*t - 2.0f)));
    float b = std::max(0.0f, std::min(1.0f, 1.5f - std::fabs(4.0f*t - 1.0f)));
    return IM_COL32((int)(r*255),(int)(g*255),(int)(b*255),255);
}

// ── Open file state ──────────────────────────────────────────────────────
struct OpenFile {
    std::string path;
    LongWaterfall::FileHeader hdr{};
    uint64_t total_size = 0;
    uint32_t num_rows = 0;
    FILE*    fp = nullptr;
};
OpenFile g_open;

// ── Texture ──────────────────────────────────────────────────────────────
GLuint   g_tex = 0;
int      g_tex_w = 1024;
int      g_tex_h = 512;
std::vector<uint32_t> g_pixel_buf;
bool     g_tex_dirty = true;

// View region: t0/t1 in row indices, f0/f1 in linear freq idx (0=lowest)
double   g_t0=0, g_t1=0;
double   g_f0=0, g_f1=0;

// Right-panel split ratio (file list width fraction, 0..1)
float    g_right_ratio = 0.22f;
float    g_right_saved = 0.22f;
bool     g_files_panel_open = true;   // S키 토글

// File-list cache (HOST tab — local hist_host_dir, JOIN tab — local hist_join_dir)
struct HistFileEntry { std::string path; std::string base; uint64_t start_utc; uint64_t size; };
std::vector<HistFileEntry> g_host_files;
std::vector<HistFileEntry> g_join_files;
bool      g_host_list_dirty  = true;
bool      g_join_list_dirty  = true;
time_t    g_join_dir_mtime   = 0;
int       g_active_tab       = 0;       // 0=HOST, 1=JOIN

// File-list selection + context state
std::string g_sel_path;             // currently selected file path (may equal g_open.path)
std::string g_ctx_path;             // file targeted by current right-click context menu
bool        g_info_modal_open = false;
std::string g_info_path;            // file shown in Info modal

// JOIN download progress
struct DlState { std::string filename; uint64_t total=0, recv=0; FILE* fp=nullptr; };
DlState  g_dl;
std::mutex g_dl_mtx;

// LWF list cached from host (JOIN side)
PktLwfList g_remote_list{};
bool       g_remote_list_valid = false;
std::mutex g_remote_list_mtx;

uint64_t   g_last_known_rows = 0;

// ── Helpers ──────────────────────────────────────────────────────────────
void close_open(){
    if(g_open.fp){ fclose(g_open.fp); g_open.fp = nullptr; }
    g_open = OpenFile{};
    g_last_known_rows = 0;
    g_tex_dirty = true;
}

bool open_file(const std::string& path){
    close_open();
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    LongWaterfall::FileHeader h{};
    if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic,"BWWF",4)!=0){
        fclose(fp); return false;
    }
    fseek(fp, 0, SEEK_END);
    uint64_t sz = (uint64_t)ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if(h.fft_size == 0 || sz < sizeof(h)){ fclose(fp); return false; }
    g_open.path = path;
    g_open.hdr = h;
    g_open.total_size = sz;
    g_open.num_rows = (uint32_t)((sz - sizeof(h)) / h.fft_size);
    g_open.fp = fp;
    g_t0 = 0; g_t1 = std::max<uint32_t>(1, g_open.num_rows);
    g_f0 = 0; g_f1 = h.fft_size;
    g_tex_dirty = true;
    g_last_known_rows = g_open.num_rows;
    return true;
}

bool read_header_only(const std::string& path, LongWaterfall::FileHeader& h, uint64_t& size){
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic,"BWWF",4)!=0){
        fclose(fp); return false;
    }
    fseek(fp, 0, SEEK_END);
    size = (uint64_t)ftell(fp);
    fclose(fp);
    return true;
}

void refresh_size_live(){
    if(!g_open.fp) return;
    struct stat st{};
    if(stat(g_open.path.c_str(), &st) != 0) return;
    uint64_t sz = (uint64_t)st.st_size;
    if(sz == g_open.total_size) return;
    g_open.total_size = sz;
    uint32_t old_rows = g_open.num_rows;
    g_open.num_rows = (uint32_t)((sz - sizeof(LongWaterfall::FileHeader)) / g_open.hdr.fft_size);
    if(g_open.num_rows != old_rows){
        if(g_t1 >= old_rows - 0.5){
            double w = g_t1 - g_t0;
            g_t1 = g_open.num_rows;
            g_t0 = g_t1 - w;
            if(g_t0 < 0) g_t0 = 0;
        }
        g_tex_dirty = true;
        g_last_known_rows = g_open.num_rows;
    }
}

void rebuild_texture(){
    if(!g_open.fp) return;
    if(g_pixel_buf.size() != (size_t)g_tex_w * g_tex_h)
        g_pixel_buf.assign((size_t)g_tex_w * g_tex_h, 0);

    int W = g_tex_w, H = g_tex_h;
    double t_span = std::max(1.0, g_t1 - g_t0);
    double f_span = std::max(1.0, g_f1 - g_f0);
    uint32_t fft_sz = g_open.hdr.fft_size;
    if(fft_sz == 0) return;
    int fft_half = (int)fft_sz / 2;

    std::vector<uint8_t> rowbuf(fft_sz);

    int rows_total = (int)t_span;
    int cols_total = W;
    int rows_per_col_max = 64;
    int rows_step = std::max(1, rows_total / (cols_total * rows_per_col_max));

    for(int x=0; x<W; x++){
        double t_a = g_t0 + (x      / (double)W) * t_span;
        double t_b = g_t0 + ((x+1)  / (double)W) * t_span;
        int ra = (int)std::floor(t_a);
        int rb = (int)std::floor(t_b);
        if(rb <= ra) rb = ra + 1;
        if(ra < 0) ra = 0;
        if(rb > (int)g_open.num_rows) rb = g_open.num_rows;
        if(ra >= rb){
            for(int y=0; y<H; y++) g_pixel_buf[(size_t)y*W + x] = IM_COL32(20,20,25,255);
            continue;
        }
        std::vector<uint8_t> col_max(fft_sz, 0);
        int n_sampled = 0;
        for(int r=ra; r<rb; r += rows_step){
            uint64_t off = sizeof(LongWaterfall::FileHeader) + (uint64_t)r * fft_sz;
            fseek(g_open.fp, (long)off, SEEK_SET);
            if(fread(rowbuf.data(), 1, fft_sz, g_open.fp) != fft_sz) break;
            for(uint32_t i=0; i<fft_sz; i++)
                if(rowbuf[i] > col_max[i]) col_max[i] = rowbuf[i];
            if(++n_sampled >= rows_per_col_max) break;
        }
        // Y axis: max-hold across all bins mapped to each pixel row (avoids
        // missing strong signals when many bins fall into one pixel).
        for(int y=0; y<H; y++){
            double f_top = g_f1 - (y       / (double)H) * f_span;
            double f_bot = g_f1 - ((y+1.0) / (double)H) * f_span;
            int idx_lo = (int)std::floor(std::min(f_top, f_bot));
            int idx_hi = (int)std::ceil(std::max(f_top, f_bot));
            if(idx_lo < 0) idx_lo = 0;
            if(idx_hi > (int)fft_sz) idx_hi = fft_sz;
            if(idx_hi <= idx_lo) idx_hi = idx_lo + 1;
            uint8_t mx = 0;
            for(int idx = idx_lo; idx < idx_hi; idx++){
                int bin = (idx < fft_half) ? (idx + fft_half) : (idx - fft_half);
                if(col_max[bin] > mx) mx = col_max[bin];
            }
            g_pixel_buf[(size_t)y*W + x] = jet_color(mx);
        }
    }

    if(!g_tex){
        glGenTextures(1, &g_tex);
        glBindTexture(GL_TEXTURE_2D, g_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, g_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, g_pixel_buf.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    g_tex_dirty = false;
}

// UTC offset (hours) for the file. v2+ has reliable utc_offset_hours
// (host's system TZ at file open). v1 didn't store it → fallback to viewer's TZ.
int header_utc_offset(const LongWaterfall::FileHeader& h){
    if(h.version >= 0x0002){
        return (int)h.utc_offset_hours;
    }
    // Legacy v1: best effort = viewer's local TZ.
    time_t now = time(nullptr);
    struct tm lt; localtime_r(&now, &lt);
    return (int)(lt.tm_gmtoff / 3600);
}

// Format epoch+offset → "YYYY-MM-DD HH:MM:SS UTC±N"
std::string fmt_local_time(uint64_t utc, int off_h){
    time_t shifted = (time_t)utc + off_h * 3600;
    struct tm tm_utc; gmtime_r(&shifted, &tm_utc);
    char buf[64];
    if(off_h == 0){
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &tm_utc);
    } else {
        char base[40];
        strftime(base, sizeof(base), "%Y-%m-%d %H:%M:%S", &tm_utc);
        snprintf(buf, sizeof(buf), "%s UTC%+d", base, off_h);
    }
    return buf;
}

std::string fmt_duration_hms(uint64_t total_sec){
    uint64_t h = total_sec / 3600;
    uint64_t m = (total_sec % 3600) / 60;
    uint64_t s = total_sec % 60;
    char buf[24];
    snprintf(buf, sizeof(buf), "%02llu:%02llu:%02llu",
             (unsigned long long)h, (unsigned long long)m, (unsigned long long)s);
    return buf;
}

// Recognize a HIST file by name (.bewehist new, .bewewf legacy, or "wfimg_/HIST_" prefix).
static bool is_hist_filename(const std::string& fn){
    if(fn.size() >= 9 && fn.compare(fn.size()-9, 9, ".bewehist")==0) return true;
    if(fn.size() >= 7 && fn.compare(fn.size()-7, 7, ".bewewf")==0)  return true;
    if(fn.rfind("wfimg_", 0) == 0) return true;
    if(fn.rfind("HIST_",  0) == 0) return true;
    return false;
}

void register_dl_callbacks_once(NetClient* cli){
    static NetClient* s_bound = nullptr;
    if(s_bound == cli) return;
    if(!cli) return;
    s_bound = cli;
    cli->on_lwf_list = [](const PktLwfList& list){
        std::lock_guard<std::mutex> lk(g_remote_list_mtx);
        g_remote_list = list;
        g_remote_list_valid = true;
    };
    // 다운로드는 기존 FILE_META/FILE_DATA 메커니즘 재사용. 저장 dir만 결정.
    // 다른 file transfer (region/share)와 충돌하지 않도록 HIST 파일명만 리다이렉트.
    auto prev_get_dir = cli->on_get_save_dir;
    cli->on_get_save_dir = [prev_get_dir](const std::string& fn) -> std::string {
        if(is_hist_filename(fn)){
            std::string dir = BEWEPaths::hist_join_dir();
            mkdir(BEWEPaths::recordings_dir().c_str(), 0755);
            mkdir(BEWEPaths::hist_dir().c_str(), 0755);
            mkdir(dir.c_str(), 0755);
            return dir;
        }
        return prev_get_dir ? prev_get_dir(fn) : std::string();
    };
    auto prev_meta = cli->on_file_meta;
    cli->on_file_meta = [prev_meta](const std::string& name, uint64_t total){
        if(is_hist_filename(name)){
            std::lock_guard<std::mutex> lk(g_dl_mtx);
            g_dl.filename = name;
            g_dl.total = total;
            g_dl.recv = 0;
        }
        if(prev_meta) prev_meta(name, total);
    };
    auto prev_prog = cli->on_file_progress;
    cli->on_file_progress = [prev_prog](const std::string& name, uint64_t done, uint64_t total){
        if(is_hist_filename(name)){
            std::lock_guard<std::mutex> lk(g_dl_mtx);
            if(g_dl.filename == name){ g_dl.recv = done; g_dl.total = total; }
        }
        if(prev_prog) prev_prog(name, done, total);
    };
}

// ── Info modal renderer ──────────────────────────────────────────────────
void draw_info_modal(){
    if(!g_info_modal_open) return;
    LongWaterfall::FileHeader h{};
    uint64_t sz = 0;
    bool ok = read_header_only(g_info_path, h, sz);

    ImGui::SetNextWindowSize(ImVec2(520.f, 0.f));
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 260.f,
                                   ImGui::GetIO().DisplaySize.y*0.30f),
                            ImGuiCond_Appearing);
    ImGui::SetNextWindowBgAlpha(0.97f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f,0.10f,0.16f,1.f));
    ImGui::Begin("Long-Waterfall Info##lwf_info", &g_info_modal_open,
        ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoCollapse);

    if(!ok){
        ImGui::TextDisabled("Cannot read header.");
    } else {
        std::string base = g_info_path;
        size_t s = base.find_last_of('/');
        if(s != std::string::npos) base = base.substr(s+1);

        int off_h = header_utc_offset(h);
        uint32_t row_rate = (uint32_t)std::max(1.0f, h.row_rate_hz);
        uint64_t dur_sec = (uint64_t)((sz - sizeof(h)) / std::max<uint32_t>(1,h.fft_size)) / row_rate;
        uint64_t stop_utc = h.start_utc_unix + dur_sec;

        const float L = 110.f;
        auto row = [&](const char* k, const std::string& v){
            ImGui::Text("%s", k); ImGui::SameLine(L);
            ImGui::TextWrapped("%s", v.c_str());
        };
        row("File:",       base);
        row("Path:",       g_info_path);
        ImGui::Separator();
        char buf[64];
        snprintf(buf, sizeof(buf), "%.4f MHz", h.center_freq_hz / 1e6);
        row("CF:",         buf);
        snprintf(buf, sizeof(buf), "%.3f MSPS", h.sample_rate_hz / 1e6);
        row("SR:",         buf);
        unsigned fft_disp = h.fft_input_size > 0 ? h.fft_input_size : h.fft_size;
        snprintf(buf, sizeof(buf), "%u", fft_disp);
        row("FFT:",        buf);
        snprintf(buf, sizeof(buf), "%.1f / %.1f dB", h.db_min, h.db_max);
        row("dB range:",   buf);
        if(h.station_lon != 0.0f){
            snprintf(buf, sizeof(buf), "%.4f°", h.station_lon);
            row("Host lon:", buf);
        }
        ImGui::Separator();
        row("Start:",      fmt_local_time(h.start_utc_unix, off_h));
        row("Stop:",       fmt_local_time(stop_utc, off_h));
        row("Duration:",   fmt_duration_hms(dur_sec));
        ImGui::Separator();
        snprintf(buf, sizeof(buf), "%.2f MB", sz / 1048576.0);
        row("Size:",       buf);
    }
    ImGui::Spacing();
    if(ImGui::Button("Close", ImVec2(80, 0))
       || ImGui::IsKeyPressed(ImGuiKey_Escape, false)){
        g_info_modal_open = false;
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

} // anon

namespace LongWaterfallView {

void draw_modal(FFTViewer& v, NetClient* cli){
    if(!v.lwf_modal_open) return;

    register_dl_callbacks_once(cli);

    ImGuiIO& io = ImGui::GetIO();
    constexpr float kBottomBarH = 32.0f;
    float modal_h = io.DisplaySize.y - kBottomBarH;
    if(modal_h < 100.f) modal_h = 100.f;

    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, modal_h));
    ImGui::SetNextWindowBgAlpha(0.97f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f,0.06f,0.10f,1.f));
    // NoTitleBar — 상단 파란 제목바 제거 (사용자 요청).
    ImGui::Begin("##lwf_modal", &v.lwf_modal_open,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
        ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar);

    // S키: HIST file panel 토글 (입력 중이거나 모달 안 focus면 무시).
    bool modal_focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if(modal_focused && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_S, false)){
        g_files_panel_open = !g_files_panel_open;
    }
    // ESC로 모달 닫기 (titlebar X 없음 보완).
    if(modal_focused && ImGui::IsKeyPressed(ImGuiKey_Escape, false)){
        v.lwf_modal_open = false;
    }

    ImVec2 win_sz = ImGui::GetContentRegionAvail();
    constexpr float kSplitW = 6.f;
    constexpr float kRightMin = 0.10f;
    constexpr float kRightMax = 0.55f;
    if(g_right_ratio < kRightMin) g_right_ratio = kRightMin;
    if(g_right_ratio > kRightMax) g_right_ratio = kRightMax;
    float right_w = g_files_panel_open ? std::max(160.f, win_sz.x * g_right_ratio) : 0.f;
    float split_w = g_files_panel_open ? kSplitW : 0.f;
    float view_w  = std::max(200.f, win_sz.x - right_w - split_w);

    // ── Left: viewer ─────────────────────────────────────────────────────
    // 정보 영역만 작은 패딩, 이미지는 child 가장자리까지 꽉 차게.
    ImGui::BeginChild("##lwf_view", ImVec2(view_w, win_sz.y), false);

    // Live refresh size if open file is current LIVE.
    std::string live_path = LongWaterfall::current_file_path();
    bool g_open_loaded = !g_open.path.empty();

    if(!g_open_loaded){
        ImGui::TextDisabled("Select a file from the right panel.");
    } else {
        if(g_open.path == live_path) refresh_size_live();
        const auto& h = g_open.hdr;
        int off_h = header_utc_offset(h);
        uint32_t row_rate = (uint32_t)std::max(1.0f, h.row_rate_hz);
        uint64_t dur_sec = (uint64_t)g_open.num_rows / row_rate;
        uint64_t stop_utc = h.start_utc_unix + dur_sec;

        unsigned fft_disp = h.fft_input_size > 0 ? h.fft_input_size : h.fft_size;
        ImGui::Dummy(ImVec2(0, 4));
        ImGui::Indent(10.0f);
        ImGui::Text("CF : %.3f MHz   SR : %.2f MSPS   FFT : %u   Duration : %s   Size : %.1f MB",
            h.center_freq_hz / 1e6,
            h.sample_rate_hz / 1e6,
            fft_disp,
            fmt_duration_hms(dur_sec).c_str(),
            g_open.total_size / 1048576.0);
        ImGui::Text("Start : %s", fmt_local_time(h.start_utc_unix, off_h).c_str());
        ImGui::Text("Stop  : %s", fmt_local_time(stop_utc, off_h).c_str());
        ImGui::Unindent(10.0f);
        ImGui::Dummy(ImVec2(0, 2));
        ImGui::Separator();

        // Image는 viewer child 끝까지 꽉 차게 (FramePadding/ItemSpacing 0).
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,  ImVec2(0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
        ImVec2 img_sz = ImGui::GetContentRegionAvail();
        if(img_sz.x >= 64 && img_sz.y >= 64){
            int target_w = std::min(2048, std::max(256, (int)img_sz.x));
            int target_h = std::min(1024, std::max(128, (int)img_sz.y));
            if(target_w != g_tex_w || target_h != g_tex_h){
                g_tex_w = target_w; g_tex_h = target_h;
                g_tex_dirty = true;
            }
            if(g_tex_dirty) rebuild_texture();

            ImVec2 img_pos = ImGui::GetCursorScreenPos();
            if(g_tex){
                ImGui::Image((ImTextureID)(intptr_t)g_tex, img_sz);
            }
            bool hov = ImGui::IsItemHovered();
            if(hov && io.MouseWheel != 0.f){
                double zf = (io.MouseWheel > 0) ? 0.8 : 1.25;
                ImVec2 mp = io.MousePos;
                if(io.KeyCtrl){
                    double frac_top = (mp.y - img_pos.y) / (double)img_sz.y;
                    double mv = g_f1 - frac_top * (g_f1 - g_f0);
                    double new_span = (g_f1 - g_f0) * zf;
                    g_f1 = mv + frac_top * new_span;
                    g_f0 = g_f1 - new_span;
                    if(g_f0 < 0){ g_f1 -= g_f0; g_f0 = 0; }
                    if(g_f1 > h.fft_size){ g_f0 -= (g_f1 - h.fft_size); g_f1 = h.fft_size; }
                    if(g_f0 < 0) g_f0 = 0;
                    if(g_f1 - g_f0 < 4) g_f1 = g_f0 + 4;
                } else {
                    double frac_x = (mp.x - img_pos.x) / (double)img_sz.x;
                    double mu = g_t0 + frac_x * (g_t1 - g_t0);
                    double new_span = (g_t1 - g_t0) * zf;
                    g_t0 = mu - frac_x * new_span;
                    g_t1 = g_t0 + new_span;
                    if(g_t0 < 0){ g_t1 -= g_t0; g_t0 = 0; }
                    if(g_t1 > g_open.num_rows){ g_t0 -= (g_t1 - g_open.num_rows); g_t1 = g_open.num_rows; }
                    if(g_t0 < 0) g_t0 = 0;
                    if(g_t1 - g_t0 < 4) g_t1 = g_t0 + 4;
                }
                g_tex_dirty = true;
            }
            bool focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
            if(focused && ImGui::IsKeyPressed(ImGuiKey_Home, false)){
                g_t0 = 0; g_t1 = std::max<uint32_t>(1, g_open.num_rows);
                g_f0 = 0; g_f1 = h.fft_size;
                g_tex_dirty = true;
            }
            if(focused && !io.WantTextInput){
                double span = g_t1 - g_t0;
                double total = (double)g_open.num_rows;
                if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow, false)){
                    double t0 = g_t0 - span;
                    if(t0 < 0) t0 = 0;
                    g_t0 = t0; g_t1 = t0 + span;
                    g_tex_dirty = true;
                }
                if(ImGui::IsKeyPressed(ImGuiKey_RightArrow, false)){
                    double t1 = g_t1 + span;
                    if(t1 > total) t1 = total;
                    g_t0 = t1 - span; g_t1 = t1;
                    g_tex_dirty = true;
                }
            }
            if(hov){
                ImVec2 mp = io.MousePos;
                double t = g_t0 + (mp.x - img_pos.x) / img_sz.x * (g_t1 - g_t0);
                double freq_idx = g_f1 - (mp.y - img_pos.y) / img_sz.y * (g_f1 - g_f0);
                double t_sec = t / (double)h.row_rate_hz;
                uint64_t hover_utc = h.start_utc_unix + (uint64_t)t_sec;
                std::string tstr = fmt_local_time(hover_utc, off_h);
                double cf_mhz = h.center_freq_hz / 1e6;
                double sr_mhz = h.sample_rate_hz / 1e6;
                double fmhz = cf_mhz + (freq_idx / (double)h.fft_size - 0.5) * sr_mhz;
                ImGui::SetTooltip("%s\n%.4f MHz", tstr.c_str(), fmhz);
            }
        }
        ImGui::PopStyleVar(2);   // ItemSpacing + FramePadding
    }
    ImGui::EndChild();

    // ── Splitter + Right file list — file panel이 열려있을 때만 ─────────
    if(g_files_panel_open){
        ImGui::SameLine(0,0);
        ImVec2 sp_pos = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##lwf_split", ImVec2(kSplitW, win_sz.y));
        bool sp_hov = ImGui::IsItemHovered();
        bool sp_act = ImGui::IsItemActive();
        if(sp_hov || sp_act) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if(sp_act){
            float new_right = (io.DisplaySize.x - io.MousePos.x) / io.DisplaySize.x;
            g_right_ratio = std::max(kRightMin, std::min(kRightMax, new_right));
            g_right_saved = g_right_ratio;
        }
        ImGui::GetWindowDrawList()->AddRectFilled(
            sp_pos, ImVec2(sp_pos.x + kSplitW, sp_pos.y + win_sz.y),
            sp_act ? IM_COL32(120,140,200,255) : IM_COL32(50,55,70,255));

        ImGui::SameLine(0,0);
        ImGui::BeginChild("##lwf_files", ImVec2(right_w, win_sz.y), true);

        // ── Tab strip: HOST | JOIN (ARCHIVE-style top tabs) ─────────────
        bool is_join_mode = (cli != nullptr);
        {
            ImDrawList* dlf = ImGui::GetWindowDrawList();
            ImVec2 strip_pos = ImGui::GetCursorScreenPos();
            float  strip_h   = ImGui::GetFontSize() + 8.f;
            // Strip bg (회색)
            dlf->AddRectFilled(strip_pos,
                ImVec2(strip_pos.x + ImGui::GetContentRegionAvail().x, strip_pos.y + strip_h),
                IM_COL32(30,30,38,255));
            const char* labels[2] = {"HOST", "JOIN"};
            float bx = strip_pos.x + 8.f;
            float by = strip_pos.y + 4.f;
            for(int i=0;i<2;i++){
                ImVec2 ts = ImGui::CalcTextSize(labels[i]);
                bool hov = io.MousePos.x >= bx && io.MousePos.x <= bx + ts.x + 4 &&
                           io.MousePos.y >= strip_pos.y && io.MousePos.y <= strip_pos.y + strip_h;
                bool active = (g_active_tab == i);
                ImU32 col = active ? IM_COL32(120,180,255,255)
                          : (hov ? IM_COL32(180,180,200,255) : IM_COL32(140,140,160,255));
                dlf->AddText(ImVec2(bx, by), col, labels[i]);
                if(hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                    g_active_tab = i;
                bx += ts.x + 16.f;
            }
            ImGui::Dummy(ImVec2(0, strip_h));
            // Active tab header (파란 ARCHIVE-style)
            ImVec2 hdr_pos = ImGui::GetCursorScreenPos();
            float  hdr_h   = ImGui::GetFontSize() + 6.f;
            dlf->AddRectFilled(hdr_pos,
                ImVec2(hdr_pos.x + ImGui::GetContentRegionAvail().x, hdr_pos.y + hdr_h),
                IM_COL32(40,70,140,255));
            dlf->AddText(ImVec2(hdr_pos.x + 8.f, hdr_pos.y + 3.f),
                IM_COL32(220,230,255,255),
                labels[g_active_tab]);
            ImGui::Dummy(ImVec2(0, hdr_h + 2));
        }

        // ── HOST tab content (local files when HOST/Local, remote when JOIN) ──
        if(g_active_tab == 0){
                if(ImGui::Button("Reload", ImVec2(-1, 0))){
                    if(is_join_mode){
                        cli->cmd_lwf_list_req();          // ask remote host
                    } else {
                        g_host_list_dirty = true;         // rescan local dir next frame
                    }
                }
                ImGui::Separator();

                // Build list rows
                struct Row { std::string path_or_name; uint64_t start; uint64_t size; bool is_live; bool is_remote; };
                std::vector<Row> rows;
                if(is_join_mode){
                    std::lock_guard<std::mutex> lk(g_remote_list_mtx);
                    if(g_remote_list_valid){
                        for(uint16_t i=0;i<g_remote_list.count;i++){
                            const auto& e = g_remote_list.entries[i];
                            Row r; r.path_or_name = e.filename;
                            r.start = e.start_utc; r.size = e.size_bytes;
                            r.is_live = false; r.is_remote = true;
                            rows.push_back(std::move(r));
                        }
                    }
                } else {
                    // Local — scan hist_host_dir
                    if(g_host_list_dirty){
                        g_host_files.clear();
                        DIR* d = opendir(BEWEPaths::hist_host_dir().c_str());
                        if(d){
                            struct dirent* de;
                            while((de = readdir(d)) != nullptr){
                                const char* n = de->d_name;
                                if(!n || n[0]=='.') continue;
                                if(!is_hist_filename(n)) continue;
                                std::string full = BEWEPaths::hist_host_dir() + "/" + n;
                                LongWaterfall::FileHeader hh{}; uint64_t fsz=0;
                                if(!read_header_only(full, hh, fsz)) continue;
                                HistFileEntry e; e.path=full; e.base=n;
                                e.start_utc=hh.start_utc_unix; e.size=fsz;
                                g_host_files.push_back(std::move(e));
                            }
                            closedir(d);
                        }
                        std::sort(g_host_files.begin(), g_host_files.end(),
                            [](const HistFileEntry& a, const HistFileEntry& b){ return a.start_utc > b.start_utc; });
                        g_host_list_dirty = false;
                    }
                    if(!live_path.empty()){
                        Row r; r.path_or_name = live_path; r.is_live = true; r.is_remote = false;
                        r.start = 0; r.size = 0;
                        rows.push_back(std::move(r));
                    }
                    for(auto& e : g_host_files){
                        if(e.path == live_path) continue;
                        Row r; r.path_or_name = e.path; r.is_live = false; r.is_remote = false;
                        r.start = e.start_utc; r.size = e.size;
                        rows.push_back(std::move(r));
                    }
                }

                ImGui::BeginChild("##lwf_host_rows", ImVec2(0, 0), false);
                for(auto& r : rows){
                    std::string base = r.path_or_name;
                    if(!r.is_remote){
                        size_t s = base.find_last_of('/');
                        if(s != std::string::npos) base = base.substr(s+1);
                    }
                    bool sel = (g_sel_path == r.path_or_name);
                    ImGui::PushID(r.path_or_name.c_str());
                    if(r.is_live){
                        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(80,220,80,255));
                        std::string label = "[LIVE] " + base;
                        if(ImGui::Selectable(label.c_str(), sel)){
                            g_sel_path = r.path_or_name;
                            if(g_open.path != r.path_or_name) open_file(r.path_or_name);
                        }
                        ImGui::PopStyleColor();
                    } else {
                        if(ImGui::Selectable(base.c_str(), sel)){
                            g_sel_path = r.path_or_name;
                            // Local: open in viewer immediately. Remote: just select.
                            if(!r.is_remote && g_open.path != r.path_or_name)
                                open_file(r.path_or_name);
                        }
                    }
                    if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                        g_ctx_path = r.path_or_name;
                        g_sel_path = r.path_or_name;
                        ImGui::OpenPopup("##lwf_host_ctx");
                    }
                    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 8));
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,  ImVec2(8, 6));
                    if(g_ctx_path == r.path_or_name && ImGui::BeginPopup("##lwf_host_ctx")){
                        // Download — JOIN mode (remote) only
                        if(is_join_mode && r.is_remote){
                            if(ImGui::MenuItem("Download")){
                                cli->cmd_lwf_dl_req(r.path_or_name.c_str());
                            }
                        } else {
                            ImGui::BeginDisabled();
                            ImGui::MenuItem("Download");
                            ImGui::EndDisabled();
                        }
                        // Delete — Local/Host mode only (delete local file)
                        if(!is_join_mode && !r.is_live && !r.is_remote){
                            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255,80,80,255));
                            if(ImGui::MenuItem("Delete")){
                                if(g_open.path == r.path_or_name) close_open();
                                if(g_sel_path == r.path_or_name) g_sel_path.clear();
                                unlink(r.path_or_name.c_str());
                                g_host_list_dirty = true;
                            }
                            ImGui::PopStyleColor();
                        } else {
                            ImGui::BeginDisabled();
                            ImGui::MenuItem("Delete");
                            ImGui::EndDisabled();
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopStyleVar(2);
                    ImGui::PopID();
                }
                ImGui::EndChild();
        }

        // ── JOIN tab: local hist_join_dir() (auto-refresh on mtime change) ──
        if(g_active_tab == 1){
                // Poll dir mtime — refresh on change.
                struct stat dst{};
                time_t cur_mtime = 0;
                if(stat(BEWEPaths::hist_join_dir().c_str(), &dst) == 0) cur_mtime = dst.st_mtime;
                if(cur_mtime != g_join_dir_mtime || g_join_list_dirty){
                    g_join_dir_mtime = cur_mtime;
                    g_join_list_dirty = false;
                    g_join_files.clear();
                    DIR* d = opendir(BEWEPaths::hist_join_dir().c_str());
                    if(d){
                        struct dirent* de;
                        while((de = readdir(d)) != nullptr){
                            const char* n = de->d_name;
                            if(!n || n[0]=='.') continue;
                            if(!is_hist_filename(n)) continue;
                            std::string full = BEWEPaths::hist_join_dir() + "/" + n;
                            LongWaterfall::FileHeader hh{}; uint64_t fsz=0;
                            if(!read_header_only(full, hh, fsz)) continue;
                            HistFileEntry e; e.path=full; e.base=n;
                            e.start_utc=hh.start_utc_unix; e.size=fsz;
                            g_join_files.push_back(std::move(e));
                        }
                        closedir(d);
                    }
                    std::sort(g_join_files.begin(), g_join_files.end(),
                        [](const HistFileEntry& a, const HistFileEntry& b){ return a.start_utc > b.start_utc; });
                }

                // DL progress (active host→join transfer via FILE_META/DATA path)
                {
                    std::lock_guard<std::mutex> lk(g_dl_mtx);
                    if(g_dl.total > 0 && g_dl.recv < g_dl.total){
                        ImGui::Text("DL %s", g_dl.filename.c_str());
                        float frac = g_dl.total ? (float)g_dl.recv / (float)g_dl.total : 0.f;
                        ImGui::ProgressBar(frac, ImVec2(-1, 0));
                        ImGui::Separator();
                    }
                }

                ImGui::BeginChild("##lwf_join_rows", ImVec2(0, 0), false);
                for(auto& e : g_join_files){
                    bool sel = (g_sel_path == e.path);
                    ImGui::PushID(e.path.c_str());
                    if(ImGui::Selectable(e.base.c_str(), sel)){
                        g_sel_path = e.path;
                        if(g_open.path != e.path) open_file(e.path);
                    }
                    if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                        g_ctx_path = e.path;
                        g_sel_path = e.path;
                        ImGui::OpenPopup("##lwf_join_ctx");
                    }
                    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 8));
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,  ImVec2(8, 6));
                    if(g_ctx_path == e.path && ImGui::BeginPopup("##lwf_join_ctx")){
                        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255,80,80,255));
                        if(ImGui::MenuItem("Delete")){
                            if(g_open.path == e.path) close_open();
                            if(g_sel_path == e.path) g_sel_path.clear();
                            unlink(e.path.c_str());
                            g_join_list_dirty = true;
                        }
                        ImGui::PopStyleColor();
                        ImGui::EndPopup();
                    }
                    ImGui::PopStyleVar(2);
                    ImGui::PopID();
                }
                ImGui::EndChild();

                // Del key on selected (only when JOIN tab focused)
                if(ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows) &&
                   ImGui::IsKeyPressed(ImGuiKey_Delete, false) &&
                   !g_sel_path.empty()){
                    // delete only if path is in hist_join_dir
                    std::string prefix = BEWEPaths::hist_join_dir() + "/";
                    if(g_sel_path.compare(0, prefix.size(), prefix) == 0){
                        if(g_open.path == g_sel_path) close_open();
                        unlink(g_sel_path.c_str());
                        g_sel_path.clear();
                        g_join_list_dirty = true;
                    }
                }
        } // JOIN tab
        ImGui::EndChild();    // ##lwf_files

    } // if(g_files_panel_open)

    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();   // WindowRounding

    // Info modal (rendered as separate window, on top of viewer)
    draw_info_modal();
}

void close_modal(){
    close_open();
    if(g_tex){ glDeleteTextures(1, &g_tex); g_tex = 0; }
}

} // namespace LongWaterfallView
