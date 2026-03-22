#include "fft_viewer.hpp"
#include "login.hpp"
#include "net_server.hpp"
#include "net_client.hpp"
#include "bewe_paths.hpp"
#include "globe.hpp"
#include "udp_discovery.hpp"
#include "central_client.hpp"
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <algorithm>
#include <chrono>
#include <map>
#include <unordered_map>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <mutex>
#include <cstdarg>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// Returns the first non-loopback IPv4 address, or "127.0.0.1"
static std::string get_local_ip(){
    struct ifaddrs* ifa_list = nullptr;
    if(getifaddrs(&ifa_list) != 0) return "127.0.0.1";
    std::string result = "127.0.0.1";
    for(auto* ifa = ifa_list; ifa; ifa = ifa->ifa_next){
        if(!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
        auto* sin = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        uint32_t addr = ntohl(sin->sin_addr.s_addr);
        if((addr >> 24) == 127) continue; // skip 127.x.x.x
        char buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf));
        result = buf;
        break;
    }
    freeifaddrs(ifa_list);
    return result;
}

void bewe_log(const char* fmt, ...){
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    fputs(buf, stdout);
}

// ── 파일 크기 포맷 ────────────────────────────────────────────────────────
static std::string fmt_filesize(const std::string& dir, const std::string& fname){
    std::string path = dir.empty() ? fname : (dir + "/" + fname);
    struct stat st{};
    if(stat(path.c_str(), &st) != 0) return "";
    double sz = (double)st.st_size;
    char buf[32];
    if(sz >= 1e9)       snprintf(buf,sizeof(buf),"[%.1fG]",sz/1e9);
    else if(sz >= 1e6)  snprintf(buf,sizeof(buf),"[%.1fM]",sz/1e6);
    else if(sz >= 1e3)  snprintf(buf,sizeof(buf),"[%.1fK]",sz/1e3);
    else                snprintf(buf,sizeof(buf),"[%dB]",(int)sz);
    return buf;
}

// ── 채널 스컬치: FFT 스펙트럼에서 직접 채널 파워 계산 ─────────────────────
// filter_active인 모든 채널에 대해 동작 (복조 없어도 회색 상태에서 작동)
// dB값이 FFT 스펙트럼과 동일한 스케일 (-100~0)
void FFTViewer::update_channel_squelch(){
    if(total_ffts < 1 || fft_size < 1) return;
    std::lock_guard<std::mutex> lk(data_mtx);
    float cf_mhz = (float)(header.center_frequency / 1e6);
    float nyq_mhz = header.sample_rate / 2e6f;
    if(nyq_mhz < 0.001f) return;
    int hf = fft_size / 2;

    // 최신 FFT 행 읽기 (float dB 값 직접)
    int fi = (total_ffts > 0 ? total_ffts - 1 : 0) % MAX_FFTS_MEMORY;
    const float* rowp = fft_data.data() + fi * fft_size;

    auto freq_to_bin = [&](float rel_mhz) -> int {
        int bin = (rel_mhz >= 0)
            ? (int)((rel_mhz / nyq_mhz) * hf)
            : fft_size + (int)((rel_mhz / nyq_mhz) * hf);
        return std::max(0, std::min(fft_size - 1, bin));
    };

    for(int c = 0; c < MAX_CHANNELS; c++){
        Channel& ch = channels[c];
        if(!ch.filter_active) continue;

        // 채널 주파수 범위 → FFT 빈
        float s_mhz = std::min(ch.s, ch.e) - cf_mhz;
        float e_mhz = std::max(ch.s, ch.e) - cf_mhz;
        int bin_s = freq_to_bin(s_mhz);
        int bin_e = freq_to_bin(e_mhz);

        // 채널 대역 내 피크 파워 (float dB 직접)
        float peak_db = -120.0f;
        if(bin_s <= bin_e){
            for(int b = bin_s; b <= bin_e; b++){
                if(rowp[b] > peak_db) peak_db = rowp[b];
            }
        } else {
            // DC 경계를 넘는 경우
            for(int b = bin_s; b < fft_size; b++){
                if(rowp[b] > peak_db) peak_db = rowp[b];
            }
            for(int b = 0; b <= bin_e; b++){
                if(rowp[b] > peak_db) peak_db = rowp[b];
            }
        }

        // IIR 스무딩 (UI 프레임 기반, ~60fps)
        float prev = ch.sq_sig.load(std::memory_order_relaxed);
        float sig = 0.3f * peak_db + 0.7f * prev;
        ch.sq_sig.store(sig, std::memory_order_relaxed);

        // 캘리브레이션: 처음 60프레임(~1초) 수집 후 20th percentile + 10dB
        if(!ch.sq_calibrated.load(std::memory_order_relaxed)){
            if(ch.sq_calib_cnt < 60){
                ch.sq_calib_buf[ch.sq_calib_cnt++] = peak_db;
            }
            if(ch.sq_calib_cnt >= 60){
                // 20th percentile
                float tmp[60];
                memcpy(tmp, ch.sq_calib_buf, sizeof(tmp));
                std::nth_element(tmp, tmp + 12, tmp + 60);  // 12/60 = 20%
                float noise_floor = tmp[12];
                ch.sq_threshold.store(noise_floor + 10.0f, std::memory_order_relaxed);
                ch.sq_calibrated.store(true, std::memory_order_relaxed);
                ch.sq_calib_cnt = 0;
            }
        }

        // 게이트 로직 (히스테리시스 + 홀드)
        float thr = ch.sq_threshold.load(std::memory_order_relaxed);
        bool gate = ch.sq_gate.load(std::memory_order_relaxed);
        const float HYS = 3.0f;
        const int HOLD_FRAMES = 18;  // ~0.3초 @ 60fps

        if(ch.sq_calibrated.load(std::memory_order_relaxed)){
            if(!gate && sig >= thr){
                gate = true;
                ch.sq_gate_hold = HOLD_FRAMES;
            }
            if(gate){
                if(sig >= thr - HYS)
                    ch.sq_gate_hold = HOLD_FRAMES;
                else if(--ch.sq_gate_hold <= 0)
                    gate = false;
            }
        }
        ch.sq_gate.store(gate, std::memory_order_relaxed);
    }
}

// ── Channel overlays ──────────────────────────────────────────────────────
void FFTViewer::handle_new_channel_drag(float gx, float gw){
    ImVec2 m=ImGui::GetIO().MousePos;
    bool in_graph=(m.x>=gx&&m.x<=gx+gw);
    // Ctrl 누른 상태에서는 채널 필터 생성 차단 (영역 녹음 모드)
    if(ImGui::GetIO().KeyCtrl) return;

    if(in_graph&&ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
        float af=x_to_abs(m.x,gx,gw);
        new_drag.active=true; new_drag.anch=af; new_drag.s=af; new_drag.e=af;
    }
    if(new_drag.active){
        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){
            float f=x_to_abs(m.x,gx,gw);
            float s=std::min(new_drag.anch,f), e=std::max(new_drag.anch,f);
            float cf=(s+e)/2.0f;
            float bw_khz=roundf((e-s)*1000.0f);
            bw_khz=std::max(1.0f,bw_khz);
            new_drag.s=cf-bw_khz/2000.0f; new_drag.e=cf+bw_khz/2000.0f;
        }
        if(ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
            new_drag.active=false;
            float bw=fabsf(new_drag.e-new_drag.s);
            if(bw>0.001f){
                int slot=-1;
                for(int i=0;i<MAX_CHANNELS;i++) if(!channels[i].filter_active){slot=i;break;}
                if(slot>=0){
                    if(net_cli){
                        // JOIN: 서버에 CMD_CREATE_CH 전송 (서버가 처리 후 sync)
                        net_cli->cmd_create_ch(slot, new_drag.s, new_drag.e);
                        ch_created_by_me[slot] = true; // 내가 만든 채널 → 초기 Mute 제외
                    } else {
                        channels[slot].s=new_drag.s; channels[slot].e=new_drag.e;
                        channels[slot].filter_active=true; channels[slot].mode=Channel::DM_NONE;
                        channels[slot].pan=0; channels[slot].selected=false;
                        channels[slot].audio_mask.store(0xFFFFFFFFu);
                        channels[slot].sq_calibrated.store(false);
                        channels[slot].ar_wp.store(0); channels[slot].ar_rp.store(0);
                        strncpy(channels[slot].owner, host_name[0]?host_name:"Host", 31);
                        srv_audio_mask[slot] = channels[slot].audio_mask.load();
                        if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
                    }
                    if(selected_ch>=0) channels[selected_ch].selected=false;
                    selected_ch=slot; channels[slot].selected=true;
                }
            }
        }
    }
}

void FFTViewer::handle_channel_interactions(float gx, float gw, float gy, float gh){
    ImVec2 m=ImGui::GetIO().MousePos;
    if(m.x<gx-8||m.x>gx+gw+8) return;
    bool in_graph=(m.y>=gy&&m.y<=gy+gh);
    const float EDGE_GRAB=6.0f; // px

    // ── Active resize drag ────────────────────────────────────────────────
    bool any_resize=false;
    for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].resize_drag){any_resize=true;break;}

    if(any_resize){
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
            float cur_abs=x_to_abs(m.x,gx,gw);
            for(int i=0;i<MAX_CHANNELS;i++){
                if(!channels[i].resize_drag) continue;
                float ss=std::min(channels[i].s,channels[i].e);
                float se=std::max(channels[i].s,channels[i].e);
                // snap to 1kHz
                float snapped=roundf(cur_abs*1000.0f)/1000.0f;
                const float MIN_BW=0.001f; // 1kHz min
                if(channels[i].resize_side==-1){
                    // left edge
                    channels[i].s=std::min(snapped, se-MIN_BW);
                    channels[i].e=se;
                } else {
                    // right edge
                    channels[i].s=ss;
                    channels[i].e=std::max(snapped, ss+MIN_BW);
                }
            }
        } else {
            // drag ended → restart demod if active
            for(int i=0;i<MAX_CHANNELS;i++){
                if(!channels[i].resize_drag) continue;
                channels[i].resize_drag=false;
                if(channels[i].dem_run.load()){
                    Channel::DemodMode md=channels[i].mode;
                    stop_dem(i); start_dem(i,md);
                }
                // HOST: sync to JOIN clients
                if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
                // JOIN: send new range to HOST
                if(net_cli && remote_mode)
                    net_cli->cmd_update_ch_range(i, channels[i].s, channels[i].e);
            }
        }
        return;
    }

    // ── Active move drag ──────────────────────────────────────────────────
    bool any_move=false;
    for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].move_drag){any_move=true;break;}

    if(any_move){
        if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
            float cur_abs=x_to_abs(m.x,gx,gw);
            for(int i=0;i<MAX_CHANNELS;i++){
                if(!channels[i].move_drag) continue;
                float delta_abs=cur_abs-channels[i].move_anchor;
                float snapped=roundf(delta_abs*1000.0f)/1000.0f;
                float half_bw=(channels[i].move_e0-channels[i].move_s0)/2.0f;
                float new_cf=(channels[i].move_s0+channels[i].move_e0)/2.0f+snapped;
                channels[i].s=new_cf-half_bw; channels[i].e=new_cf+half_bw;
            }
        } else {
            for(int i=0;i<MAX_CHANNELS;i++){
                if(!channels[i].move_drag) continue;
                bool moved=(channels[i].s!=channels[i].move_s0||channels[i].e!=channels[i].move_e0);
                channels[i].move_drag=false;
                if(moved){
                    if(channels[i].dem_run.load()){
                        Channel::DemodMode md=channels[i].mode;
                        stop_dem(i); start_dem(i,md);
                    }
                    // HOST: sync to JOIN clients
                    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
                    // JOIN: send new range to HOST
                    if(net_cli && remote_mode)
                        net_cli->cmd_update_ch_range(i, channels[i].s, channels[i].e);
                }
            }
        }
        return;
    }

    // ── Hover: edge detection for cursor change ───────────────────────────
    if(in_graph){
        bool near_edge=false;
        for(int i=0;i<MAX_CHANNELS;i++){
            if(!channels[i].filter_active) continue;
            float x0=abs_to_x(std::min(channels[i].s,channels[i].e),gx,gw);
            float x1=abs_to_x(std::max(channels[i].s,channels[i].e),gx,gw);
            if(fabsf(m.x-x0)<EDGE_GRAB||fabsf(m.x-x1)<EDGE_GRAB){
                near_edge=true; break;
            }
        }
        if(near_edge) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    // ── Double-click: delete channel ──────────────────────────────────────
    if(in_graph&&ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
        int ci=channel_at_x(m.x,gx,gw);
        if(ci>=0){
            if(net_cli){
                // JOIN: 서버에 삭제 요청 → 서버가 broadcast 처리
                net_cli->cmd_delete_ch(ci);
            }
            // 로컬 즉시 반영 (서버 sync가 확인해줌)
            stop_dem(ci); stop_digi(ci); digi_panel_on[ci]=false;
            channels[ci].filter_active=false;
            channels[ci].selected=false;
            channels[ci].mode=Channel::DM_NONE;
            local_ch_out[ci] = 1;
            ch_created_by_me[ci] = false;
            if(selected_ch==ci) selected_ch=-1;
            if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
        }
        return;
    }

    // ── Single click: resize edge or move ────────────────────────────────
    if(in_graph&&ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
        // Check edge first
        int edge_ch=-1; int edge_side=0;
        for(int i=0;i<MAX_CHANNELS;i++){
            if(!channels[i].filter_active) continue;
            float x0=abs_to_x(std::min(channels[i].s,channels[i].e),gx,gw);
            float x1=abs_to_x(std::max(channels[i].s,channels[i].e),gx,gw);
            if(fabsf(m.x-x0)<EDGE_GRAB){ edge_ch=i; edge_side=-1; break; }
            if(fabsf(m.x-x1)<EDGE_GRAB){ edge_ch=i; edge_side= 1; break; }
        }
        if(edge_ch>=0){
            // Select + start resize
            if(selected_ch>=0) channels[selected_ch].selected=false;
            selected_ch=edge_ch; channels[edge_ch].selected=true;
            channels[edge_ch].resize_drag=true;
            channels[edge_ch].resize_side=edge_side;
        } else {
            // Normal move
            int ci=channel_at_x(m.x,gx,gw);
            if(selected_ch>=0) channels[selected_ch].selected=false;
            if(ci>=0){
                selected_ch=ci; channels[ci].selected=true;
                channels[ci].move_drag=true;
                channels[ci].move_anchor=x_to_abs(m.x,gx,gw);
                channels[ci].move_s0=std::min(channels[ci].s,channels[ci].e);
                channels[ci].move_e0=std::max(channels[ci].s,channels[ci].e);
            } else {
                selected_ch=-1;
            }
        }
    }
}

// 배열 인덱스 → 주파수 순 표시 번호 (1-based). 비활성 채널은 0 반환.
int FFTViewer::freq_sorted_display_num(int arr_idx) const {
    if(arr_idx<0||arr_idx>=MAX_CHANNELS||!channels[arr_idx].filter_active) return 0;
    float my_cf=(channels[arr_idx].s+channels[arr_idx].e)*0.5f;
    int rank=1;
    for(int i=0;i<MAX_CHANNELS;i++){
        if(!channels[i].filter_active||i==arr_idx) continue;
        float cf_i=(channels[i].s+channels[i].e)*0.5f;
        if(cf_i<my_cf||(cf_i==my_cf&&i<arr_idx)) rank++;
    }
    return rank;
}

void FFTViewer::draw_all_channels(ImDrawList* dl, float gx, float gw, float gy, float gh, bool show_label){
    float cf=header.center_frequency/1e6f;
    float ds,de; get_disp(ds,de); float dw=de-ds;

    if(new_drag.active){
        float x0=gx+(new_drag.s-cf-ds)/dw*gw, x1=gx+(new_drag.e-cf-ds)/dw*gw;
        float c0=std::max(gx,x0), c1=std::min(gx+gw,x1);
        if(c1>c0){
            dl->AddRectFilled(ImVec2(c0,gy),ImVec2(c1,gy+gh),IM_COL32(255,255,255,20));
            dl->AddLine(ImVec2(x0,gy),ImVec2(x0,gy+gh),IM_COL32(200,200,200,160),1.5f);
            dl->AddLine(ImVec2(x1,gy),ImVec2(x1,gy+gh),IM_COL32(200,200,200,160),1.5f);
        }
    }

    for(int i=0;i<MAX_CHANNELS;i++){
        Channel& ch=channels[i];
        if(!ch.filter_active) continue;
        float ss=std::min(ch.s,ch.e), se=std::max(ch.s,ch.e);
        float x0=gx+(ss-cf-ds)/dw*gw, x1=gx+(se-cf-ds)/dw*gw;
        float c0=std::max(gx,x0), c1=std::min(gx+gw,x1);
        if(c1<=c0) continue;

        // ── Mode-based colors ─────────────────────────────────────────────
        // fill alpha: normal=25, selected=60
        ImU32 fill, bord;
        bool is_rec=(rec_on.load()&&i==rec_ch);
        bool dem = remote_mode
            ? (ch.mode != Channel::DM_NONE)
            : ch.dem_run.load();

        if(is_rec){
            // 녹음 중: 빨간색
            bord=IM_COL32(255, 60, 60,220);
            fill=IM_COL32(255, 60, 60, ch.selected?70:30);
        } else if(ch.digi_run.load()){
            // 디지털 복조 활성: 보라색
            bord=IM_COL32(160, 60,255,220);
            fill=IM_COL32(160, 60,255, ch.selected?70:25);
        } else if(!dem || ch.mode==Channel::DM_NONE){
            // 복조 없음: 회색 투명
            bord=IM_COL32(160,160,160,160);
            fill=IM_COL32(160,160,160, ch.selected?40:15);
        } else if(ch.mode==Channel::DM_AM){
            // AM: 하늘색
            bord=IM_COL32( 80,200,255,220);
            fill=IM_COL32( 80,200,255, ch.selected?70:25);
        } else if(ch.mode==Channel::DM_FM){
            // FM: 노란색
            bord=IM_COL32(255,220, 50,220);
            fill=IM_COL32(255,220, 50, ch.selected?70:25);
        } else if(ch.mode==Channel::DM_MAGIC){
            // 매직: 보라색
            bord=IM_COL32(180, 80,255,220);
            fill=IM_COL32(180, 80,255, ch.selected?70:25);
        } else {
            bord=CH_BORD[i];
            fill=ch.selected?CH_SFIL[i]:CH_FILL[i];
        }

        dl->AddRectFilled(ImVec2(c0,gy),ImVec2(c1,gy+gh),fill);
        auto dash=[&](float x){
            if(x<gx-1||x>gx+gw+1) return;
            for(float y=gy;y<gy+gh;y+=10){
                float ye=std::min(y+5.0f,gy+gh);
                dl->AddLine(ImVec2(x,y),ImVec2(x,ye),bord,1.5f);
            }
        };
        dash(x0); dash(x1);
        if(ch.selected){
            dl->AddLine(ImVec2(std::max(gx,x0),gy),ImVec2(std::max(gx,x0),gy+gh),bord,2.0f);
            dl->AddLine(ImVec2(std::min(gx+gw,x1),gy),ImVec2(std::min(gx+gw,x1),gy+gh),bord,2.0f);
        }
        if(!show_label) continue;
        char lb[16]; snprintf(lb,sizeof(lb),"[%d]",freq_sorted_display_num(i));
        ImVec2 ts=ImGui::CalcTextSize(lb);
        float cx=std::max(gx,std::min(gx+gw-ts.x,(c0+c1)/2-ts.x/2));
        float ly=gy+4;
        dl->AddRectFilled(ImVec2(cx-2,ly),ImVec2(cx+ts.x+2,ly+ts.y+2),IM_COL32(0,0,0,190));
        dl->AddText(ImVec2(cx,ly+1),ch.sq_gate.load()?bord:IM_COL32(160,160,160,200),lb);
    }
}

void FFTViewer::draw_freq_axis(ImDrawList* dl, float gx, float gw, float gy, float gh, bool ticks_only){
    float cf=header.center_frequency/1e6f;
    float ds,de; get_disp(ds,de); float dr=de-ds;

    // 항상 1MHz 단위 고정
    const float step_mhz = 1.0f;
    float abs_start = cf+ds;
    float first = ceilf(abs_start/step_mhz)*step_mhz;
    for(float af=first; af<=cf+de+1e-5f; af+=step_mhz){
        float x=gx+(af-cf-ds)/dr*gw;
        if(x<gx||x>gx+gw) continue;
        if(!ticks_only) dl->AddLine(ImVec2(x,gy),ImVec2(x,gy+gh),IM_COL32(60,60,60,100),1);
        dl->AddLine(ImVec2(x,gy+gh-5),ImVec2(x,gy+gh),IM_COL32(100,100,100,200),1);
        if(!ticks_only){
            dl->AddLine(ImVec2(x,gy+gh),ImVec2(x,gy+gh+5),IM_COL32(100,100,100,200),1);
            char lb[16]; snprintf(lb,sizeof(lb),"%.0f",af);
            ImVec2 ts=ImGui::CalcTextSize(lb);
            dl->AddText(ImVec2(x-ts.x/2,gy+gh+8),IM_COL32(0,255,0,255),lb);
        }
    }
}

void FFTViewer::handle_zoom_scroll(float gx, float gw, float mouse_x){
    float wheel=ImGui::GetIO().MouseWheel;
    if(wheel==0) return;

    // Ctrl+휠: TM 모드일 때만 오프셋 조작 (자동 진입/해제 없음)
    if(ImGui::GetIO().KeyCtrl){
        if(!tm_active.load()) return;
        float delta=(wheel>0)?-1.0f:1.0f;
        tm_offset=std::max(0.0f, tm_offset+delta);
        tm_update_display();
        return;
    }

    // 일반 휠: 주파수 줌
    float nyq=header.sample_rate/2.0f/1e6f, eff=nyq*0.875f, rng=2*eff;
    float mx=(mouse_x-gx)/gw; mx=std::max(0.0f,std::min(1.0f,mx));
    float fmx=-eff+freq_pan*rng+mx*(rng/freq_zoom);
    freq_zoom*=(1+wheel*0.15f); freq_zoom=std::max(1.0f,std::min(200.0f,freq_zoom));
    float nw=rng/freq_zoom, ns=fmx-mx*nw;
    freq_pan=(ns+eff)/rng; freq_pan=std::max(0.0f,std::min(1-1/freq_zoom,freq_pan));
}

void FFTViewer::draw_spectrum_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h){
    if(total_w < AXIS_LABEL_WIDTH + 2.0f || total_h < 4.0f) return;
    float gx=full_x+AXIS_LABEL_WIDTH, gy=full_y;
    float gw=total_w-AXIS_LABEL_WIDTH, gh=total_h-BOTTOM_LABEL_HEIGHT;
    dl->AddRectFilled(ImVec2(full_x,full_y),ImVec2(full_x+total_w,full_y+total_h),IM_COL32(10,10,10,255));
    // /rx stop 시 검은 화면만 표시
    if(rx_stopped.load()) { draw_freq_axis(dl,gx,gw,gy,gh,false); return; }

    float ds,de; get_disp(ds,de);
    float sr_mhz=header.sample_rate/1e6f; int np=(int)gw;
    // 타임머신 모드: tm_display_fft_idx 기준, 아니면 current_fft_idx
    int sp_idx=tm_active.load() ? tm_display_fft_idx : current_fft_idx;
    bool cv=(cached_sp_idx==sp_idx&&cached_pan==freq_pan&&cached_zoom==freq_zoom&&
             cached_px==np&&cached_pmin==display_power_min&&cached_pmax==display_power_max);
    if(!cv){
        // assign() 대신 resize() — fill 불필요 (아래 루프가 전부 덮어씀)
        if((int)current_spectrum.size() != np) current_spectrum.resize(np, -80.0f);
        std::lock_guard<std::mutex> lk(data_mtx);
        float nyq=sr_mhz/2.0f; int hf=header.fft_size/2;
        int mi=sp_idx%MAX_FFTS_MEMORY;
        const float* rowp=fft_data.data()+mi*fft_size;
        // Peak detection: 각 픽셀에 매핑되는 빈 범위의 최대값 사용
        auto freq_to_bin=[&](float fd)->int{
            int b=(fd>=0)?(int)((fd/nyq)*hf+0.5f):fft_size+(int)((fd/nyq)*hf-0.5f);
            return std::max(0,std::min(fft_size-1,b));
        };
        for(int px=0;px<np;px++){
            float fd0=ds+(float)px/np*(de-ds);
            float fd1=ds+(float)(px+1)/np*(de-ds);
            int b0=freq_to_bin(fd0), b1=freq_to_bin(fd1);
            if(b0>b1) std::swap(b0,b1);
            float mx=-200.0f;
            for(int b=b0;b<=b1;b++) if(rowp[b]>mx) mx=rowp[b];
            current_spectrum[px]=mx;
        }
        cached_sp_idx=sp_idx; cached_pan=freq_pan; cached_zoom=freq_zoom;
        cached_px=np; cached_pmin=display_power_min; cached_pmax=display_power_max;
    }
    // AddPolyline 1회 호출 — cache 히트 시 ImVec2 재계산 스킵
    {
        float pr_inv=1.0f/std::max(1.0f,display_power_max-display_power_min);
        static thread_local std::vector<ImVec2> sp_pts;
        // cache miss 이거나 gx/gy/gh가 바뀐 경우에만 재계산
        static thread_local float sp_cached_gx=0,sp_cached_gy=0,sp_cached_gh=0;
        bool pts_dirty = !cv || (int)sp_pts.size()!=np
                      || sp_cached_gx!=gx || sp_cached_gy!=gy || sp_cached_gh!=gh;
        if(pts_dirty){
            sp_pts.resize(np);
            for(int px=0;px<np;px++){
                float t=(current_spectrum[px]-display_power_min)*pr_inv;
                t=t<0.0f?0.0f:t>1.0f?1.0f:t;
                sp_pts[px]=ImVec2(gx+(float)px, gy+(1.0f-t)*gh);
            }
            sp_cached_gx=gx; sp_cached_gy=gy; sp_cached_gh=gh;
        }
        dl->AddPolyline(sp_pts.data(), np, IM_COL32(0,255,0,255), ImDrawFlags_None, 1.5f);
    }
    // 파워 축 그리드 라인 + 레이블 (실제 dB 값 표시, 10단계 균등 분할)
    // i=0(max), i=10(min): 레이블 숨김
    // i=1: max 편집용 (흰색), i=9: min 편집용 (흰색), i=2~8: 일반 회색
    // 편집 상태: 0=없음, 1=max 편집(i=1 위치), 2=min 편집(i=9 위치)
    static int  pax_edit      = 0;
    static char pax_buf[16]   = {};
    static bool pax_had_focus = false;

    for(int i=0;i<=10;i++){
        float y  = gy + (float)i/10.0f * gh;
        float db = display_power_max - (display_power_max - display_power_min) * (float)i / 10.0f;
        // 그리드 선: i=1~9
        if(i > 0 && i < 10)
            dl->AddLine(ImVec2(gx,y),ImVec2(gx+gw,y),IM_COL32(60,60,60,100),1);
        // 눈금 짧은 선: 모든 i
        dl->AddLine(ImVec2(gx-5,y),ImVec2(gx,y),IM_COL32(100,100,100,200),1);

        // i=0, i=10: 레이블/버튼 없음 (숨김)
        if(i == 0 || i == 10) continue;

        // i=1: max 편집 칸, i=9: min 편집 칸
        bool is_edit_row = (i == 1 || i == 9);
        bool editing_this = (pax_edit == 1 && i == 1) || (pax_edit == 2 && i == 9);

        if(editing_this){
            // 해당 눈금 위치에 InputText 인라인
            float input_w = AXIS_LABEL_WIDTH - 4;
            float text_h  = ImGui::GetTextLineHeight();
            float iy      = y - text_h * 0.5f;
            if(iy < full_y) iy = full_y;
            ImGui::SetCursorScreenPos(ImVec2(full_x + 2, iy));
            ImGui::SetNextItemWidth(input_w);
            if(!pax_had_focus) ImGui::SetKeyboardFocusHere();
            ImGuiInputTextFlags fl = ImGuiInputTextFlags_CharsDecimal
                                   | ImGuiInputTextFlags_EnterReturnsTrue;
            bool confirmed = ImGui::InputText("##pax_inp", pax_buf, sizeof(pax_buf), fl);
            bool is_active = ImGui::IsItemActive();
            if(!pax_had_focus && is_active) pax_had_focus = true;
            if(confirmed || (pax_had_focus && !is_active)){
                float val = (float)atof(pax_buf);
                if(pax_edit == 1){ // max
                    display_power_max = val;
                    if(display_power_max - display_power_min < 5.f)
                        display_power_min = display_power_max - 5.f;
                } else { // min
                    display_power_min = val;
                    if(display_power_max - display_power_min < 5.f)
                        display_power_max = display_power_min + 5.f;
                }
                cached_sp_idx = -1;
                join_manual_scale = true; // JOIN 모드: 수동 입력 → HOST 값 덮어쓰기 차단
                pax_edit = 0; pax_had_focus = false;
            }
            if(ImGui::IsKeyPressed(ImGuiKey_Escape)){ pax_edit = 0; pax_had_focus = false; }
        } else {
            char lb[16]; snprintf(lb, 16, "%.0f", db);
            ImVec2 ts = ImGui::CalcTextSize(lb);
            float lx  = gx - 10 - ts.x;
            float ly  = y - ts.y * 0.5f;
            // i=1/9: 흰색 (클릭 가능), i=2~8: 회색
            uint32_t col = is_edit_row ? IM_COL32(255,255,255,255) : IM_COL32(200,200,200,255);
            dl->AddText(ImVec2(lx, ly), col, lb);

            // i=1/9 클릭 감지
            if(is_edit_row && pax_edit == 0){
                ImGui::SetCursorScreenPos(ImVec2(full_x, ly - 2));
                const char* btn_id = (i == 1) ? "##pax_top_btn" : "##pax_bot_btn";
                ImGui::InvisibleButton(btn_id, ImVec2(AXIS_LABEL_WIDTH, ts.y + 6));
                if(ImGui::IsItemHovered()) ImGui::SetMouseCursor(ImGuiMouseCursor_TextInput);
                if(ImGui::IsItemClicked()){
                    // 편집값은 실제 max 또는 min
                    float edit_val = (i == 1) ? display_power_max : display_power_min;
                    snprintf(pax_buf, sizeof(pax_buf), "%.0f", edit_val);
                    pax_edit = (i == 1) ? 1 : 2;
                    pax_had_focus = false;
                }
            }
        }
    }

    draw_freq_axis(dl,gx,gw,gy,gh,false);
    draw_all_channels(dl,gx,gw,gy,gh,true);

    ImGui::SetCursorScreenPos(ImVec2(gx,gy));
    ImGui::InvisibleButton("sp_graph",ImVec2(gw,gh));
    bool hov=ImGui::IsItemHovered();
    if(!tm_active.load()) handle_new_channel_drag(gx,gw);
    if(!region.active) handle_channel_interactions(gx,gw,gy,gh);
    if(hov){
        ImVec2 mm=ImGui::GetIO().MousePos;
        float af=x_to_abs(mm.x,gx,gw);
        char info[48]; snprintf(info,48,"%.3f MHz",af);
        ImVec2 ts=ImGui::CalcTextSize(info);
        float tx=gx+gw-ts.x-4, ty=gy+2;
        dl->AddRectFilled(ImVec2(tx-2,ty),ImVec2(tx+ts.x+2,ty+ts.y+4),IM_COL32(20,20,20,220));
        dl->AddRect(ImVec2(tx-2,ty),ImVec2(tx+ts.x+2,ty+ts.y+4),IM_COL32(100,100,100,255));
        dl->AddText(ImVec2(tx,ty+2),IM_COL32(0,255,0,255),info);
        handle_zoom_scroll(gx,gw,mm.x);
    }
    // 파워 축 드래그 (편집 중이 아닐 때만)
    // 드래그 방향: 위로 → 값 증가, 아래로 → 값 감소
    // 상반부 드래그 → max 조절, 하반부 → min 조절
    // 1픽셀 이동 = (range/gh) dB 선형 변화 (시작 시점 값 기준)
    if(pax_edit == 0){
        ImGui::SetCursorScreenPos(ImVec2(full_x,gy));
        ImGui::InvisibleButton("pax",ImVec2(AXIS_LABEL_WIDTH,gh));
        static float drag_start_y=0, drag_start_val=0;
        static bool  drag_is_max=false;
        if(ImGui::IsItemActive()){
            if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                ImVec2 m2=ImGui::GetMousePos();
                float mid_y=gy+gh*0.5f;
                drag_is_max = (m2.y <= mid_y); // 상반부=max, 하반부=min
                drag_start_y   = m2.y;
                drag_start_val = drag_is_max ? display_power_max : display_power_min;
            }
            if(ImGui::IsMouseDragging(ImGuiMouseButton_Left,0)){
                float dy   = ImGui::GetMousePos().y - drag_start_y;
                float range= display_power_max - display_power_min;
                // 1픽셀 = range/gh dB, 위로 드래그(-dy)=값 증가
                float delta= -dy * (range / std::max(1.0f, gh));
                if(drag_is_max){
                    display_power_max = drag_start_val + delta;
                    if(display_power_max - display_power_min < 5.f)
                        display_power_max = display_power_min + 5.f;
                } else {
                    display_power_min = drag_start_val + delta;
                    if(display_power_max - display_power_min < 5.f)
                        display_power_min = display_power_max - 5.f;
                }
                join_manual_scale = true; // JOIN 모드: 수동 조절 → HOST 값 덮어쓰기 차단
                cached_sp_idx=-1;
            }
        }
    }

    // ── 주파수 축 드래그 → center frequency 이동 (SDR++ 스타일) ──────────
    {
        float fax_y = gy + gh;  // 주파수 레이블 영역 상단
        float fax_h = BOTTOM_LABEL_HEIGHT;
        ImGui::SetCursorScreenPos(ImVec2(gx, fax_y));
        ImGui::InvisibleButton("freq_axis_drag", ImVec2(gw, fax_h));
        bool fax_hov = ImGui::IsItemHovered();
        if(fax_hov) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

        if(fax_hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
            freq_drag_active = true;
            freq_drag_start_x = ImGui::GetIO().MousePos.x;
            freq_drag_start_cf = (float)(header.center_frequency / 1e6);
        }
        if(freq_drag_active){
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                float dx = ImGui::GetIO().MousePos.x - freq_drag_start_x;
                // 화면 픽셀 → MHz 변환: 현재 표시 범위 기준
                float ds2, de2; get_disp(ds2, de2);
                float mhz_per_px = (de2 - ds2) / gw;
                float new_cf = freq_drag_start_cf - dx * mhz_per_px;
                if(new_cf < 0.1f) new_cf = 0.1f;
                // 캡처 스레드에 주파수 변경 요청 (블로킹 방지)
                pending_cf = new_cf;
                freq_req = true;
            } else {
                freq_drag_active = false;
            }
        }
    }
}

void FFTViewer::draw_waterfall_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h){
    if(total_w < AXIS_LABEL_WIDTH + 2.0f || total_h < 4.0f) return;
    float gx=full_x+AXIS_LABEL_WIDTH, gy=full_y;
    float gw=total_w-AXIS_LABEL_WIDTH, gh=total_h;
    dl->AddRectFilled(ImVec2(full_x,full_y),ImVec2(full_x+total_w,full_y+total_h),IM_COL32(10,10,10,255));
    // /rx stop 시 검은 화면만 표시
    if(rx_stopped.load()) { draw_freq_axis(dl,gx,gw,gy,gh,true); return; }
    if(!waterfall_texture) create_waterfall_texture();
    // 타임머신 모드 아닐 때만 텍스처 업데이트
    // 워터폴 텍스처 업데이트: TM 모드 중에도 계속 갱신 (복귀 시 검은화면 방지)
    if(total_ffts>0&&last_wf_update_idx!=current_fft_idx){
        update_wf_row(current_fft_idx); last_wf_update_idx=current_fft_idx;
    }
    if(waterfall_texture){
        float ds,de; get_disp(ds,de);
        float nyq=header.sample_rate/2.0f/1e6f;
        int dr2=std::min(total_ffts,MAX_FFTS_MEMORY);
        int tex_w=std::min(fft_size, 16384);
        if(tex_w<1) tex_w=1;
        float half_texel=0.5f/tex_w;
        float us=(ds+nyq)/(2*nyq)+half_texel, ue=(de+nyq)/(2*nyq)+half_texel;
        float dh=(dr2>=(int)gh)?gh:(float)dr2;
        // 타임머신: tm_display_fft_idx 기준, 일반: current_fft_idx 기준
        int disp_idx=tm_active.load() ? tm_display_fft_idx : current_fft_idx;
        float vn=(float)(disp_idx%MAX_FFTS_MEMORY)/MAX_FFTS_MEMORY;
        float vt=vn+1.0f/MAX_FFTS_MEMORY;
        float vb=vt-dh/MAX_FFTS_MEMORY;
        ImTextureID tid=(ImTextureID)(intptr_t)waterfall_texture;
        dl->AddImage(tid,ImVec2(gx,gy),ImVec2(gx+gw,gy+dh),ImVec2(us,vt),ImVec2(ue,vb),IM_COL32(255,255,255,255));
        // IQ 가용 오버레이 제거됨 — 좌측 태그로 대체
    }
    draw_freq_axis(dl,gx,gw,gy,gh,true);
    draw_all_channels(dl,gx,gw,gy,gh,false);

    // ── 좌측 시간/이벤트 태그 렌더링 ─────────────────────────────────────
    {
        float rps=(float)header.sample_rate/(float)fft_input_size/(float)time_average;
        if(rps<=0) rps=37.5f;
        int disp_idx=tm_active.load() ? tm_display_fft_idx : current_fft_idx;
        float label_x=full_x; // AXIS_LABEL_WIDTH 영역 내
        float label_w=gx-full_x;

        std::lock_guard<std::mutex> lk(wf_events_mtx);
        for(auto& ev : wf_events){
            // 현재 뷰 기준 몇 행 위인지
            int row=disp_idx - ev.fft_idx;
            if(row<0||row>=(int)gh) continue; // 화면 밖

            float ey=gy+(float)row;

            if(ev.type==0){
                // 5초 단위 시간 태그: 텍스트만
                ImVec2 tsz=ImGui::CalcTextSize(ev.label);
                float tx=label_x+(label_w-tsz.x)/2.0f;
                dl->AddText(ImVec2(tx,ey-ImGui::GetFontSize()/2),
                            IM_COL32(180,180,180,200), ev.label);
            } else {
                // TM_IQ Start(1) / Stop(2): 가로 선 + 텍스트
                ImU32 col=(ev.type==1)?IM_COL32(80,200,255,200):IM_COL32(255,100,100,200);
                // 가로 선: 좌측 태그 영역부터 워터폴 전체 폭까지
                dl->AddLine(ImVec2(label_x,ey),ImVec2(gx+gw,ey),col,1.0f);
                ImVec2 tsz=ImGui::CalcTextSize(ev.label);
                float tx=label_x+1;
                // 텍스트가 레이블 영역 넘치면 잘라서 표시
                dl->AddText(ImVec2(tx,ey-ImGui::GetFontSize()),col,ev.label);
            }
        }
    }
    ImGui::SetCursorScreenPos(ImVec2(gx,gy));
    ImGui::InvisibleButton("wf_graph",ImVec2(gw,gh));
    bool hov=ImGui::IsItemHovered();
    if(!tm_active.load()) handle_new_channel_drag(gx,gw);
    if(!region.active) handle_channel_interactions(gx,gw,gy,gh);

    // ── Ctrl+우클릭 드래그: 영역 IQ 녹음 선택 ────────────────────────────
    {
        ImGuiIO& mio=ImGui::GetIO();
        ImVec2 mp=mio.MousePos;
        bool ctrl=mio.KeyCtrl;
        bool in_wf=(mp.x>=gx&&mp.x<=gx+gw&&mp.y>=gy&&mp.y<=gy+gh);

        // ── 신규 선택: Ctrl+우클릭 드래그 ──────────────────────────────
        if(ctrl&&ImGui::IsMouseClicked(ImGuiMouseButton_Right)&&in_wf&&(tm_iq_file_ready||remote_mode)){
            region.selecting=true; region.active=false;
            region.edit_mode=RegionSel::EDIT_NONE;
            region.drag_x0=mp.x; region.drag_y0=mp.y;
            region.drag_x1=mp.x; region.drag_y1=mp.y;
        }
        if(region.selecting&&ImGui::IsMouseDown(ImGuiMouseButton_Right)){
            region.drag_x1=mp.x; region.drag_y1=mp.y;
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
        }
        if(region.selecting&&ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
            region.selecting=false;
            float rx0=std::min(region.drag_x0,region.drag_x1);
            float rx1=std::max(region.drag_x0,region.drag_x1);
            float ry0=std::min(region.drag_y0,region.drag_y1);
            float ry1=std::max(region.drag_y0,region.drag_y1);
            if(rx1-rx0>4&&ry1-ry0>4){
                region.freq_lo=x_to_abs(rx0,gx,gw);
                region.freq_hi=x_to_abs(rx1,gx,gw);
                int disp_idx=tm_active.load()?tm_display_fft_idx:current_fft_idx;
                region.fft_top=disp_idx-(int)(ry0-gy);
                region.fft_bot=disp_idx-(int)(ry1-gy);
                // wall_time 기반 타임스탬프 (JOIN 포함 정확한 시간)
                {
                    time_t wt_top = fft_idx_to_wall_time(region.fft_top);
                    time_t wt_bot = fft_idx_to_wall_time(region.fft_bot);
                    if(wt_top > 0 && wt_bot > 0){
                        region.time_end   = wt_top;
                        region.time_start = wt_bot;
                    } else {
                        // fallback: rps 기반
                        float rps=(float)header.sample_rate/(float)fft_input_size/(float)time_average;
                        if(rps<=0) rps=37.5f;
                        time_t now=time(nullptr);
                        region.time_end  =now-(time_t)((current_fft_idx-region.fft_top)/rps);
                        region.time_start=now-(time_t)((current_fft_idx-region.fft_bot)/rps);
                    }
                }
                region.active=true;
                region.lclick_count=0; region.lclick_timer=0;
            }
        }

        // ── 활성 영역 이동/리사이즈 ─────────────────────────────────────
        if(region.active&&!region.selecting){
            int disp_idx=tm_active.load()?tm_display_fft_idx:current_fft_idx;
            float ry0=gy+(float)(disp_idx-region.fft_top);
            float ry1=gy+(float)(disp_idx-region.fft_bot);
            float rx0=abs_to_x(region.freq_lo,gx,gw);
            float rx1=abs_to_x(region.freq_hi,gx,gw);
            // 화면 클램프 (렌더용)
            float dry0=std::max(gy,std::min(gy+gh,ry0));
            float dry1=std::max(gy,std::min(gy+gh,ry1));
            float drx0=std::max(gx,std::min(gx+gw,rx0));
            float drx1=std::max(gx,std::min(gx+gw,rx1));

            const float E=6.0f; // 엣지 감지 픽셀
            // 픽셀 → 주파수 변환 배율 (MHz/px)
            float mhz_per_px=(region.freq_hi-region.freq_lo)/(rx1-rx0+1e-5f);
            // 주파수 최소 폭: 0.001MHz(1kHz)
            const float MIN_BW=0.001f;

            // 마우스가 어느 엣지에 있는지 판단 (편집 시작 전)
            bool on_edge_l=fabsf(mp.x-rx0)<E&&mp.y>=ry0&&mp.y<=ry1;
            bool on_edge_r=fabsf(mp.x-rx1)<E&&mp.y>=ry0&&mp.y<=ry1;
            bool on_edge_t=fabsf(mp.y-ry0)<E&&mp.x>=rx0&&mp.x<=rx1;
            bool on_edge_b=fabsf(mp.y-ry1)<E&&mp.x>=rx0&&mp.x<=rx1;
            bool inside_box=(mp.x>rx0+E&&mp.x<rx1-E&&mp.y>ry0+E&&mp.y<ry1-E);

            // 커서 모양
            if(region.edit_mode==RegionSel::EDIT_NONE&&in_wf&&!ctrl){
                if(on_edge_l||on_edge_r)         ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                else if(on_edge_t||on_edge_b)     ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
                else if(inside_box)               ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            }

            // Ctrl+좌클릭: SA 드래그 시작
            if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)&&in_wf&&ctrl&&
               region.edit_mode==RegionSel::EDIT_NONE&&inside_box&&
               sa_panel_open){
                sa_drag_active = true;
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }

            // 편집 시작: 엣지는 클릭 즉시, 내부 이동은 드래그 시작 시점에
            if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)&&in_wf&&!ctrl&&
               region.edit_mode==RegionSel::EDIT_NONE){
                if(on_edge_l||on_edge_r||on_edge_t||on_edge_b){
                    region.edit_mx0=mp.x; region.edit_my0=mp.y;
                    region.edit_flo0=region.freq_lo; region.edit_fhi0=region.freq_hi;
                    region.edit_ftop0=region.fft_top; region.edit_fbot0=region.fft_bot;
                    if(on_edge_l)      region.edit_mode=RegionSel::EDIT_RESIZE_L;
                    else if(on_edge_r) region.edit_mode=RegionSel::EDIT_RESIZE_R;
                    else if(on_edge_t) region.edit_mode=RegionSel::EDIT_RESIZE_T;
                    else               region.edit_mode=RegionSel::EDIT_RESIZE_B;
                } else if(inside_box){
                    // 내부 클릭: 일단 이동 준비만 (드래그 시작 전까지 lclick 카운트 유지)
                    region.edit_mx0=mp.x; region.edit_my0=mp.y;
                    region.edit_flo0=region.freq_lo; region.edit_fhi0=region.freq_hi;
                    region.edit_ftop0=region.fft_top; region.edit_fbot0=region.fft_bot;
                    region.edit_mode=RegionSel::EDIT_MOVE; // 드래그 없으면 released에서 취소
                }
            }

            // SA 드래그 커서 추적 (Ctrl+좌클릭, edit_mode 없이 독립)
            if(sa_drag_active && ImGui::IsMouseDown(ImGuiMouseButton_Left))
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            // 편집 중 (마우스 드래그)
            if(region.edit_mode!=RegionSel::EDIT_NONE&&ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                float dx=mp.x-region.edit_mx0;
                float dy=mp.y-region.edit_my0;
                float df=dx*mhz_per_px;
                int   dr=(int)dy;

                switch(region.edit_mode){
                case RegionSel::EDIT_MOVE:
                    region.freq_lo=region.edit_flo0+df;
                    region.freq_hi=region.edit_fhi0+df;
                    region.fft_top=region.edit_ftop0-dr;
                    region.fft_bot=region.edit_fbot0-dr;
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                    break;
                case RegionSel::EDIT_RESIZE_L:
                    region.freq_lo=std::min(region.edit_flo0+df, region.freq_hi-MIN_BW);
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                    break;
                case RegionSel::EDIT_RESIZE_R:
                    region.freq_hi=std::max(region.edit_fhi0+df, region.freq_lo+MIN_BW);
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                    break;
                case RegionSel::EDIT_RESIZE_T:
                    region.fft_top=region.edit_ftop0-dr;
                    if(region.fft_top<=region.fft_bot) region.fft_top=region.fft_bot+1;
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
                    break;
                case RegionSel::EDIT_RESIZE_B:
                    region.fft_bot=region.edit_fbot0-dr;
                    if(region.fft_bot>=region.fft_top) region.fft_bot=region.fft_top-1;
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
                    break;
                default: break;
                }
            }

            // SA 드롭: Ctrl+좌클릭 release
            if(sa_drag_active && ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                sa_drag_active = false;
                // 우측 패널 위에서 놓으면 SA 계산
                if(mp.x >= right_panel_x && right_panel_x > 0 && sa_panel_open){
                    sa_cleanup();
                    sa_mode = true;
                    region_save();
                }
            }

            // 편집 종료: time_start/end 재계산
            if(region.edit_mode!=RegionSel::EDIT_NONE&&ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                region.edit_mode=RegionSel::EDIT_NONE;
                float rps2=(float)header.sample_rate/(float)fft_input_size/(float)time_average;
                if(rps2<=0) rps2=37.5f;
                time_t now2=time(nullptr);
                region.time_end  =now2-(time_t)((current_fft_idx-region.fft_top)/rps2);
                region.time_start=now2-(time_t)((current_fft_idx-region.fft_bot)/rps2);
            }
            // 내부 클릭 카운트 (드래그 없이 release된 경우만)
            if(ImGui::IsMouseReleased(ImGuiMouseButton_Left)&&in_wf&&!ctrl&&inside_box){
                float dmx=mp.x-region.edit_mx0, dmy=mp.y-region.edit_my0;
                if(fabsf(dmx)<4&&fabsf(dmy)<4){
                    region.lclick_count++; region.lclick_timer=0.4f;
                }
            }

            // ── 렌더링 ────────────────────────────────────────────────────
            dl->AddRectFilled(ImVec2(drx0,dry0),ImVec2(drx1,dry1),IM_COL32(255,40,40,50));
            dl->AddRect(ImVec2(drx0,dry0),ImVec2(drx1,dry1),IM_COL32(255,60,60,220),0,0,1.5f);
            // 엣지 하이라이트
            if(in_wf&&!ctrl&&region.edit_mode==RegionSel::EDIT_NONE){
                if(on_edge_l) dl->AddLine(ImVec2(rx0,dry0),ImVec2(rx0,dry1),IM_COL32(255,150,150,255),2.5f);
                if(on_edge_r) dl->AddLine(ImVec2(rx1,dry0),ImVec2(rx1,dry1),IM_COL32(255,150,150,255),2.5f);
                if(on_edge_t) dl->AddLine(ImVec2(drx0,ry0),ImVec2(drx1,ry0),IM_COL32(255,150,150,255),2.5f);
                if(on_edge_b) dl->AddLine(ImVec2(drx0,ry1),ImVec2(drx1,ry1),IM_COL32(255,150,150,255),2.5f);
            }
            char hint[64];
            float cf=(region.freq_lo+region.freq_hi)*0.5f;
            float bw=(region.freq_hi-region.freq_lo)*1000.0f;
            snprintf(hint,sizeof(hint),"%.3f MHz  BW %.0f kHz  [R]Save",cf,bw);
            dl->AddText(ImVec2(drx0+2,dry0+2),IM_COL32(255,180,180,255),hint);

            // 영역 해제: 영역 안 클릭 2회 또는 더블클릭
            // lclick 카운트 처리 (편집 중 아닐때)
            {
                region.lclick_timer-=ImGui::GetIO().DeltaTime;
                if(region.lclick_count>=2){ region.active=false; region.lclick_count=0; }
                if(region.lclick_timer<=0)  region.lclick_count=0;
                if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)&&in_wf&&inside_box)
                    region.active=false;
            }
        }

        // ── 선택 중 미리보기 렌더링 ──────────────────────────────────────
        if(region.selecting){
            float rx0=std::min(region.drag_x0,region.drag_x1);
            float rx1=std::max(region.drag_x0,region.drag_x1);
            float ry0=std::min(region.drag_y0,region.drag_y1);
            float ry1=std::max(region.drag_y0,region.drag_y1);
            dl->AddRectFilled(ImVec2(rx0,ry0),ImVec2(rx1,ry1),IM_COL32(255,40,40,50));
            dl->AddRect(ImVec2(rx0,ry0),ImVec2(rx1,ry1),IM_COL32(255,60,60,220),0,0,1.5f);
        }
    }

    if(hov){
        ImVec2 mm=ImGui::GetIO().MousePos;
        float af=x_to_abs(mm.x,gx,gw);
        char info[64]; snprintf(info,64,"%.3f MHz",af);
        ImVec2 ts=ImGui::CalcTextSize(info);
        float tx=gx+gw-ts.x, ty=gy;
        dl->AddRectFilled(ImVec2(tx,ty),ImVec2(tx+ts.x,ty+ts.y+5),IM_COL32(20,20,20,220));
        dl->AddRect(ImVec2(tx,ty),ImVec2(tx+ts.x,ty+ts.y+5),IM_COL32(100,100,100,255));
        dl->AddText(ImVec2(tx,ty+2),IM_COL32(0,255,0,255),info);
        handle_zoom_scroll(gx,gw,mm.x);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void run_streaming_viewer(){
    float cf=450.0f;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED,GLFW_FALSE);
    GLFWmonitor* primary=glfwGetPrimaryMonitor();
    const GLFWvidmode* vmode=glfwGetVideoMode(primary);
    glfwWindowHint(GLFW_RED_BITS,  vmode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS,vmode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, vmode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE,vmode->refreshRate);
    GLFWwindow* win=glfwCreateWindow(vmode->width,vmode->height,"BEWE",primary,nullptr);
    glfwMakeContextCurrent(win); glfwSwapInterval(0);
    glewExperimental=GL_TRUE; glewInit();
    ImGui::CreateContext(); ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win,true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ── Early chat state (login + globe loops 공유) ───────────────────────
    struct EarlyChatMsg { char from[32]; char msg[256]; bool is_error=false; };
    std::vector<EarlyChatMsg> early_chat_log;
    bool  early_chat_open   = false;
    bool  early_chat_scroll = false;
    char  early_chat_input[256] = {};

    // early loop 명령 플래그
    bool early_do_shutdown    = false;
    bool early_do_logout      = false;
    bool early_chat_focus_req  = false; // Enter → 입력칸 포커스 요청
    bool early_chat_cursor_end = false; // 다음 프레임에 커서를 끝으로 이동 (/ 입력 후 선택 방지)

    auto draw_early_chat = [&](int fw, int fh){
        // RShift 토글
        ImGuiIO& eio = ImGui::GetIO();
        if(ImGui::IsKeyPressed(ImGuiKey_RightShift, false) && !eio.WantTextInput)
            early_chat_open = !early_chat_open;

        // 채팅창 열린 상태에서 Enter → 입력칸 포커스
        if(early_chat_open &&
           (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) &&
           !eio.WantTextInput)
            early_chat_focus_req = true;

        // '/' 키 → 채팅창 열고 '/' 미리 입력 (빠른 명령 입력)
        if(ImGui::IsKeyPressed(ImGuiKey_Slash, false) && !eio.WantTextInput){
            if(!early_chat_open){ early_chat_open = true; }
            early_chat_input[0] = '/'; early_chat_input[1] = '\0';
            early_chat_focus_req  = true;
            early_chat_cursor_end = true; // 커서를 '/' 뒤에 위치
        }

        if(!early_chat_open) return;

        const float CW=360.f, CH=320.f;
        ImGui::SetNextWindowPos(ImVec2((float)fw-CW-10.f, (float)fh-CH-10.f));
        ImGui::SetNextWindowSize(ImVec2(CW, CH));
        ImGui::SetNextWindowBgAlpha(0.92f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f,0.07f,0.12f,1.f));
        ImGui::PushStyleColor(ImGuiCol_FrameBg,  ImVec4(0.10f,0.12f,0.20f,1.f));
        ImGui::Begin("##early_chat", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

        ImGui::TextColored(ImVec4(0.4f,0.7f,1.f,1.f), "Chat");
        ImGui::Separator();

        float msg_h = CH - 60.f;
        ImGui::BeginChild("##early_chat_msgs", ImVec2(0, msg_h), false);
        for(auto& m : early_chat_log){
            ImVec4 col = m.is_error ? ImVec4(1.f,0.3f,0.3f,1.f) : ImVec4(0.3f,1.f,0.5f,1.f);
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::TextWrapped("[%s] %s", m.from, m.msg);
            ImGui::PopStyleColor();
        }
        if(early_chat_scroll){ ImGui::SetScrollHereY(1.f); early_chat_scroll=false; }
        ImGui::EndChild();

        ImGui::Separator();
        ImGui::SetNextItemWidth(CW - 16.f);
        if(early_chat_focus_req){
            ImGui::SetKeyboardFocusHere(0);
            early_chat_focus_req = false;
        }
        bool send = false;
        if(ImGui::InputText("##early_chat_in", early_chat_input, sizeof(early_chat_input),
                            ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackAlways,
                            [](ImGuiInputTextCallbackData* d) -> int {
                                bool* flag = (bool*)d->UserData;
                                if(*flag){ d->CursorPos = d->BufTextLen; d->SelectionStart = d->SelectionEnd = d->CursorPos; *flag = false; }
                                return 0;
                            }, &early_chat_cursor_end))
            send = true;

        if(send && early_chat_input[0]){
            std::string s = early_chat_input;
            auto push = [&](const char* from, const char* msg, bool err=false){
                EarlyChatMsg m{}; m.is_error=err;
                strncpy(m.from, from, 31); strncpy(m.msg, msg, 255);
                early_chat_log.push_back(m);
                early_chat_scroll = true;
            };
            if(s[0] == '/'){
                if(s == "/shutdown"){
                    early_do_shutdown = true;
                } else if(s == "/logout"){
                    early_do_logout = true;
                } else if(s == "/main" || s == "/chassis 1 reset" || s == "/chassis 2 reset"){
                    push("System", "Not available here.", true);
                } else {
                    char errmsg[280];
                    snprintf(errmsg, sizeof(errmsg), "Unknown command: %s", s.c_str());
                    push("System", errmsg, true);
                }
            } else {
                push(login_get_id()[0] ? login_get_id() : "me", s.c_str());
            }
            early_chat_input[0] = '\0';
        }

        ImGui::End();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar();
    };

    // ── Login screen loop ─────────────────────────────────────────────────
    {
        glfwSwapInterval(1); // VSync ON — 로그인 화면은 정적, CPU 낭비 방지
        bool logged_in = false;
        while(!logged_in && !glfwWindowShouldClose(win)){
            glfwPollEvents();
            int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
            glViewport(0,0,fw,fh);
            glClearColor(0.047f,0.071f,0.137f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
            logged_in = draw_login_screen(fw,fh);
            draw_early_chat(fw, fh);
            if(early_do_shutdown){ glfwSetWindowShouldClose(win, GLFW_TRUE); }
            if(early_do_logout){ early_do_logout = false; /* login창에서 logout = 이미 로그아웃 상태, 무시 */ }
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(win);
        }
        if(glfwWindowShouldClose(win)){
            // 로그인 전 창 닫힘: HW 미초기화 상태이므로 ImGui/GLFW만 정리
            ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
            return;
        }
    }

    // ── 모드 선택 화면 ───────────────────────────────────────────────────
    // ── 모드선택 outer 루프 (do_main_menu 시 재진입) ─────────────────────
    bool do_main_menu = false;
    bool do_logout = false;
    bool do_chassis_reset = false;
    int  chassis_reset_mode = 0; // 0=LOCAL, 1=HOST
    bool usb_reset_pending = false; // chassis 1 reset 시 USB reset 수행 플래그
    std::atomic<bool> pending_chassis1_reset{false}; // 네트워크 스레드 → 메인 루프 전달
    std::atomic<bool> pending_chassis2_reset{false}; // 네트워크 스레드 → 메인 루프 전달
    std::atomic<bool> pending_rx_stop{false};        // JOIN → HOST: /rx stop
    std::atomic<bool> pending_rx_start{false};       // JOIN → HOST: /rx start
    // HOST 모드 로컬 채팅 로그 (do-while 외부에서 선언해야 콜백 람다에서 접근 가능)
    struct LocalChatMsg { char from[32]; char msg[256]; bool is_error=false; };
    std::vector<LocalChatMsg> host_chat_log;
    std::mutex host_chat_mtx;
    // 함수 레벨 파일 목록 (스코프 공유 필요)
    // Record 탭: 세션 중 실시간 녹음 (record/iq, record/audio)
    static std::vector<std::string> rec_iq_files;
    static std::vector<std::string> rec_audio_files;
    // Private 탭: 이전 세션 녹음 (private/iq, private/audio)
    static std::vector<std::string> priv_iq_files;
    static std::vector<std::string> priv_audio_files;
    // Public 탭 (HOST 로컬): public/iq, public/audio
    static std::vector<std::string> pub_iq_files;
    static std::vector<std::string> pub_audio_files;
    // Share 탭: 다운로드된 파일 (share/iq, share/audio)
    static std::vector<std::string> share_iq_files;
    static std::vector<std::string> share_audio_files;
    // 레거시: priv_files → priv_iq+priv_audio 합산 (일부 기존 코드 참조용)
    static std::vector<std::string> priv_files; // 스캔 후 priv_iq+priv_audio 합산
    static std::map<std::string,std::string> priv_extra_paths; // filename → full path
    static std::vector<std::string> shared_files; // 스캔 후 pub_iq+pub_audio 합산
    static std::vector<std::string> downloaded_files; // 스캔 후 share_iq+share_audio 합산
    // JOIN: HOST Public 파일 목록 (filename, size_bytes, uploader)
    struct JoinShareEntry { std::string filename; uint64_t size_bytes=0; std::string uploader; };
    static std::vector<JoinShareEntry> join_share_files;
    static std::mutex join_share_mtx;
    // HOST: Public 파일 다운로드 리스너 추적 (filename → 다운로드한 op_name 목록)
    static std::map<std::string,std::vector<std::string>> pub_listeners;
    // Public 파일 소유자 추적 (filename → uploader_name): 업로드한 사람만 삭제 가능
    static std::map<std::string,std::string> pub_owners;
    static std::atomic<bool> ch_sync_dirty_flag{false};
    do {
    do_main_menu = false;
    FFTViewer v;
    std::thread cap;
    v.create_waterfall_texture();
    // 0=LOCAL, 1=HOST, 2=CONNECT
    // /reset 재진입을 위해 루프 간 상태 보존
    static int   s_host_port    = 7701;
    static char  s_connect_host[128] = "192.168.1.";
    static int   s_connect_port = 7701;
    static char  s_connect_id[32]  = {};
    static char  s_connect_pw[64]  = {};
    static uint8_t s_connect_tier  = 1;
    // HOST reset용 station 정보 보존
    static std::string s_station_name;
    static float       s_station_lat = 0.f, s_station_lon = 0.f;
    static bool        s_station_set = false;
    // Central Server 설정 (로그인 화면에서 입력한 값으로 초기화)
    static char s_central_host[128] = {};
    if(s_central_host[0] == '\0')
        strncpy(s_central_host, login_get_server(), 127);
    static constexpr int s_central_port = CENTRAL_PORT;
    // Central Server 경유 JOIN 시 station_id 보존 (재연결용)
    static std::string s_central_join_station_id;
    int  mode_sel     = do_chassis_reset ? chassis_reset_mode : 0;
    int& host_port    = s_host_port;
    char (&connect_host)[128] = s_connect_host;
    int& connect_port = s_connect_port;
    char (&connect_id)[32]   = s_connect_id;
    char (&connect_pw)[64]   = s_connect_pw;
    uint8_t& connect_tier    = s_connect_tier;
    // HOST reset: 저장된 station 정보 복원
    if(do_chassis_reset && (chassis_reset_mode == 1) && s_station_set){
        v.station_name         = s_station_name;
        v.station_lat          = s_station_lat;
        v.station_lon          = s_station_lon;
        v.station_location_set = true;
    }
    std::string mode_err_msg;
    float mode_err_timer = 0.0f;
    bool  mode_done = do_chassis_reset; // chassis reset: skip mode selection
    do_chassis_reset = false;
    NetServer* srv = nullptr;
    NetClient* cli = nullptr;

    // ── Globe-based station discovery ─────────────────────────────────────
    GlobeRenderer globe;
    bool globe_ok = globe.init();

    // Discovery listener: populate v.discovered_stations from LAN broadcasts
    DiscoveryListener disc_listener;
    disc_listener.on_station_found = [&](const DiscoveryAnnounce& ann){
        std::lock_guard<std::mutex> lk(v.discovered_stations_mtx);
        double now = glfwGetTime();
        // Upsert by ip+port key
        for(auto& s : v.discovered_stations){
            if(s.ip == ann.host_ip && s.tcp_port == ann.tcp_port){
                s.name             = ann.station_name;
                s.lat              = ann.lat;
                s.lon              = ann.lon;
                s.user_count       = ann.user_count;
                s.host_tier        = ann.host_tier ? ann.host_tier : 1;
                s.last_seen        = now;
                return;
            }
        }
        FFTViewer::DiscoveredStation ns;
        ns.name             = ann.station_name;
        ns.lat              = ann.lat;
        ns.lon              = ann.lon;
        ns.tcp_port         = ann.tcp_port;
        ns.ip               = ann.host_ip;
        ns.user_count       = ann.user_count;
        ns.host_tier        = ann.host_tier ? ann.host_tier : 1;
        ns.last_seen        = now;
        v.discovered_stations.push_back(ns);
    };
    disc_listener.start();


    // Relay 클라이언트: Relay 주소가 설정돼 있으면 인터넷 스테이션 폴링
    CentralClient central_cli;
    if(s_central_host[0] != '\0'){
        central_cli.start_polling(s_central_host, s_central_port,
            [&](const std::vector<CentralClient::Station>& stations){
                std::lock_guard<std::mutex> lk(v.discovered_stations_mtx);
                double now = glfwGetTime();
                for(auto& rs : stations){
                    // station_id 기반 upsert (Central Server 모드: ip/port 없음)
                    bool found = false;
                    for(auto& s : v.discovered_stations){
                        if(!s.station_id.empty() && s.station_id == rs.station_id){
                            s.name       = rs.name;
                            s.lat        = rs.lat;
                            s.lon        = rs.lon;
                            s.user_count = rs.user_count;
                            s.host_tier  = rs.host_tier ? rs.host_tier : 1;
                            s.last_seen  = now + 12.0;
                            found = true; break;
                        }
                    }
                    if(!found){
                        FFTViewer::DiscoveredStation ns;
                        ns.station_id = rs.station_id;
                        ns.name       = rs.name;
                        ns.lat        = rs.lat;
                        ns.lon        = rs.lon;
                        ns.tcp_port   = 0;  // Central Server 모드: 포트 직접 연결 없음
                        ns.ip         = ""; // Central Server 모드: IP 직접 연결 없음
                        ns.user_count = rs.user_count;
                        ns.host_tier  = rs.host_tier ? rs.host_tier : 1;
                        ns.last_seen  = now + 12.0;
                        v.discovered_stations.push_back(ns);
                    }
                }
            });
    }

    // Pop-up state machine
    enum GlobePop { POP_NONE, POP_HOST, POP_JOIN } pop_state = POP_NONE;
    FFTViewer::DiscoveredStation pending_join;
    float pending_lat=0.f, pending_lon=0.f;
    char  new_station_name[64] = {};
    bool  was_dragging = false;

    // 클릭 좌표 표시 상태
    bool  show_coord     = false;
    float coord_lat      = 0.f, coord_lon = 0.f;
    float coord_sx       = 0.f, coord_sy  = 0.f;
    float coord_timer    = 0.f; // 표시 유지 시간

    glfwSwapInterval(1); // VSync ON — globe loop는 무거운 연산 없음
    while(!mode_done && !glfwWindowShouldClose(win)){
        glfwPollEvents();
        int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
        glViewport(0,0,fw,fh);
        glClearColor(0.03f,0.05f,0.10f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        // ── Render globe ──────────────────────────────────────────────────
        if(globe_ok){
            glEnable(GL_DEPTH_TEST);
            globe.set_viewport(fw,fh);
            globe.render();
            glDisable(GL_DEPTH_TEST);
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        ImGuiIO& io = ImGui::GetIO();

        // Purge stale stations (>6 s without announcement)
        {
            std::lock_guard<std::mutex> lk(v.discovered_stations_mtx);
            double now2 = glfwGetTime();
            v.discovered_stations.erase(
                std::remove_if(v.discovered_stations.begin(),
                               v.discovered_stations.end(),
                               [now2](const FFTViewer::DiscoveredStation& s){
                                   return now2 - s.last_seen > 6.0;
                               }),
                v.discovered_stations.end());
        }

        // ── Globe mouse interaction (only when no popup and globe init ok) ─
        if(globe_ok && pop_state == POP_NONE && !io.WantCaptureMouse){
            if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                globe.on_drag_begin(io.MousePos.x, io.MousePos.y);
                was_dragging = false;
            }
            if(ImGui::IsMouseDragging(ImGuiMouseButton_Left, 4.f)){
                globe.on_drag(io.MousePos.x, io.MousePos.y);
                was_dragging = true;
            }
            if(io.MouseWheel != 0.f)
                globe.on_scroll(io.MouseWheel);

            // Click (not drag) → pick lat/lon
            if(ImGui::IsMouseReleased(ImGuiMouseButton_Left) && !was_dragging){
                float plat, plon;
                if(globe.pick(io.MousePos.x, io.MousePos.y, plat, plon)){
                    // Check if a station marker was clicked (20px radius)
                    bool hit_station = false;
                    {
                        std::lock_guard<std::mutex> lk(v.discovered_stations_mtx);
                        for(auto& st : v.discovered_stations){
                            float sx, sy;
                            if(globe.project(st.lat, st.lon, sx, sy)){
                                float dx = sx - io.MousePos.x;
                                float dy = sy - io.MousePos.y;
                                if(dx*dx + dy*dy < 400.f){
                                    pending_join = st;
                                    hit_station = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(hit_station){
                        if(login_get_tier() < 3) pop_state = POP_JOIN;
                    } else {
                        if(login_get_tier() < 3){
                            pending_lat = plat;
                            pending_lon = plon;
                            memset(new_station_name, 0, sizeof(new_station_name));
                            // 빈 지역 클릭 시 좌표 표시 (tier 1,2는 HOST 팝업으로 대체)
                            show_coord  = false;
                            pop_state = POP_HOST;
                        } else {
                            // tier 3: 좌표 표시만
                            show_coord  = true;
                            coord_lat   = plat;
                            coord_lon   = plon;
                            coord_sx    = io.MousePos.x;
                            coord_sy    = io.MousePos.y;
                            coord_timer = 3.f;
                        }
                    }
                }
            }
        }

        // ── Station markers overlay ───────────────────────────────────────
        if(globe_ok){
            ImDrawList* fdl = ImGui::GetForegroundDrawList();
            std::lock_guard<std::mutex> lk(v.discovered_stations_mtx);
            for(auto& st : v.discovered_stations){
                float sx, sy;
                if(!globe.project(st.lat, st.lon, sx, sy)) continue;
                // Outer glow
                fdl->AddCircle(ImVec2(sx,sy), 14.f, IM_COL32(80,200,255,60), 32, 3.f);
                // Inner fill
                fdl->AddCircleFilled(ImVec2(sx,sy), 8.f, IM_COL32(60,160,255,220));
                // Core
                fdl->AddCircleFilled(ImVec2(sx,sy), 4.f, IM_COL32(200,240,255,255));
                // Hover tooltip
                float dx = sx - io.MousePos.x, dy = sy - io.MousePos.y;
                if(dx*dx + dy*dy < 196.f){
                    fdl->AddText(ImVec2(sx+12,sy-8),
                                 IM_COL32(220,240,255,255), st.name.c_str());
                    char ubuf[32];
                    snprintf(ubuf, sizeof(ubuf), "Tier %d", (int)st.host_tier);
                    fdl->AddText(ImVec2(sx+12,sy+4),
                                 IM_COL32(160,200,220,200), ubuf);
                }
            }
        }

        // ── Click coordinate display ──────────────────────────────────────
        if(show_coord){
            coord_timer -= io.DeltaTime;
            if(coord_timer <= 0.f){ show_coord = false; }
            else {
                // fade out last 0.5s
                float alpha = (coord_timer < 0.5f) ? coord_timer / 0.5f : 1.f;
                ImU32 col_text = IM_COL32(220, 255, 180, (int)(220*alpha));
                ImU32 col_bg   = IM_COL32(10,  30,  10,  (int)(180*alpha));
                char cbuf[48];
                snprintf(cbuf, sizeof(cbuf), "%.4f°%s  %.4f°%s",
                         fabsf(coord_lat), coord_lat >= 0.f ? "N" : "S",
                         fabsf(coord_lon), coord_lon >= 0.f ? "W" : "E");
                ImVec2 tsz = ImGui::CalcTextSize(cbuf);
                float cx = coord_sx + 14.f;
                float cy = coord_sy - 22.f;
                // keep inside screen
                if(cx + tsz.x + 6 > fw) cx = coord_sx - tsz.x - 14.f;
                if(cy < 4.f) cy = coord_sy + 10.f;
                ImDrawList* fdl = ImGui::GetForegroundDrawList();
                fdl->AddRectFilled(ImVec2(cx-4,cy-3), ImVec2(cx+tsz.x+4,cy+tsz.y+3),
                                   col_bg, 4.f);
                fdl->AddText(ImVec2(cx, cy), col_text, cbuf);
            }
        }

        // ── Title ─────────────────────────────────────────────────────────
        {
            ImDrawList* fdl = ImGui::GetForegroundDrawList();
            fdl->AddText(ImVec2(20,20), IM_COL32(100,180,255,200),
                         "BEWE Station Discovery");
        }

        // ── LOCAL button (top-right corner) ──────────────────────────────
        {
            ImGui::SetNextWindowPos(ImVec2((float)fw-170.f, 14.f));
            ImGui::SetNextWindowSize(ImVec2(154.f, 38.f));
            ImGui::SetNextWindowBgAlpha(0.75f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
            ImGui::Begin("##local_btn", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                ImGuiWindowFlags_NoNav);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f,0.35f,0.15f,1.f));
            if(ImGui::Button("LOCAL", ImVec2(138,24))){
                mode_sel=0; mode_done=true;
            }
            ImGui::PopStyleColor();
            ImGui::End();
            ImGui::PopStyleVar();
        }

        // ── HOST placement popup ──────────────────────────────────────────
        if(pop_state == POP_HOST){
            const float PW=330.f, PH=110.f;
            ImGui::SetNextWindowPos(ImVec2(((float)fw-PW)*0.5f,((float)fh-PH)*0.5f));
            ImGui::SetNextWindowSize(ImVec2(PW,PH));
            ImGui::SetNextWindowBgAlpha(0.92f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,10.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f,0.08f,0.15f,1.f));
            ImGui::Begin("##pop_host", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
            const char* lbl = "Station Name:";
            float lbl_w = ImGui::CalcTextSize(lbl).x;
            float input_w = 160.f;
            float total_w = lbl_w + ImGui::GetStyle().ItemSpacing.x + input_w;
            ImGui::SetCursorPosX((PW - total_w) * 0.5f);
            ImGui::Text("%s", lbl);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(input_w);
            ImGui::InputText("##sname", new_station_name, sizeof(new_station_name));
            ImGui::Spacing();
            bool can_host = new_station_name[0] != '\0';
            float btn_host_w = 90.f, btn_cancel_w = 80.f;
            float btns_w = btn_host_w + ImGui::GetStyle().ItemSpacing.x + btn_cancel_w;
            ImGui::SetCursorPosX((PW - btns_w) * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f,0.40f,0.14f,1.f));
            if(!can_host) ImGui::BeginDisabled();
            if(ImGui::Button("Host##sh", ImVec2(btn_host_w,26))){
                v.station_name = new_station_name;
                v.station_lat  = pending_lat;
                v.station_lon  = pending_lon;
                v.station_location_set = true;
                mode_sel=1; pop_state=POP_NONE; mode_done=true;
            }
            if(!can_host) ImGui::EndDisabled();
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if(ImGui::Button("Cancel##hc", ImVec2(btn_cancel_w,26))) pop_state=POP_NONE;
            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }

        // ── JOIN confirm popup ────────────────────────────────────────────
        if(pop_state == POP_JOIN){
            // tier 접속 제한: Tier1=모두 가능, Tier2=Tier2 서버만 가능
            bool tier_ok = (login_get_tier() == 1) ||
                           (login_get_tier() == 2 && pending_join.host_tier == 2);

            const float PW=340.f, PH=150.f;
            ImGui::SetNextWindowPos(ImVec2(((float)fw-PW)*0.5f,((float)fh-PH)*0.5f));
            ImGui::SetNextWindowSize(ImVec2(PW,PH));
            ImGui::SetNextWindowBgAlpha(0.92f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,10.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f,0.08f,0.15f,1.f));
            ImGui::Begin("##pop_join", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

            // Station : NAME  — 가운데 정렬
            char name_buf[80];
            snprintf(name_buf, sizeof(name_buf), "Station : %s", pending_join.name.c_str());
            float nw = ImGui::CalcTextSize(name_buf).x;
            ImGui::SetCursorPosX((PW - nw) * 0.5f);
            ImGui::TextColored(ImVec4(0.5f,0.9f,1.f,1.f), "%s", name_buf);

            // 좌표 — 약간 작은 폰트, 표준 N/S E/W 표시
            char coord_buf[64];
            snprintf(coord_buf, sizeof(coord_buf), "(%.4f%s %.4f%s)",
                     fabsf(pending_join.lat), pending_join.lat >= 0.f ? "N" : "S",
                     fabsf(pending_join.lon), pending_join.lon >= 0.f ? "W" : "E");
            ImGui::SetWindowFontScale(0.85f);
            float cw2 = ImGui::CalcTextSize(coord_buf).x;
            ImGui::SetCursorPosX((PW - cw2) * 0.5f);
            ImGui::TextDisabled("%s", coord_buf);
            ImGui::SetWindowFontScale(1.0f);

            char op_buf[48];
            snprintf(op_buf, sizeof(op_buf), "%d Operator%s connected",
                     (int)pending_join.user_count,
                     pending_join.user_count == 1 ? "" : "s");
            float ow = ImGui::CalcTextSize(op_buf).x;
            ImGui::SetCursorPosX((PW - ow) * 0.5f);
            ImGui::TextDisabled("%s", op_buf);

            char tier_buf[32];
            snprintf(tier_buf, sizeof(tier_buf), "Tier %d", (int)pending_join.host_tier);
            float tbw = ImGui::CalcTextSize(tier_buf).x;
            ImGui::SetCursorPosX((PW - tbw) * 0.5f);
            if(!tier_ok)
                ImGui::TextColored(ImVec4(1.f,0.4f,0.4f,1.f), "%s", tier_buf);
            else
                ImGui::TextColored(ImVec4(0.3f,0.9f,0.4f,1.f), "%s", tier_buf);

            ImGui::Spacing();

            // 버튼 가운데 정렬
            const float btn_join_w=90.f, btn_cancel_w=80.f, spacing=8.f;
            float btns_total = btn_join_w + spacing + btn_cancel_w;
            ImGui::SetCursorPosX((PW - btns_total) * 0.5f);

            if(!tier_ok) ImGui::BeginDisabled();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f,0.30f,0.60f,1.f));
            if(ImGui::Button("Join##jb", ImVec2(btn_join_w,28))){
                { std::lock_guard<std::mutex> lk(join_share_mtx); join_share_files.clear(); }
                cli = new NetClient();
                bool join_ok = false;

                if(!pending_join.station_id.empty() && s_central_host[0] != '\0'){
                    // ── 외부 Central Server 경유 JOIN ─────────────────────────────
                    int rfd = central_cli.join_room(s_central_host, s_central_port,
                                                  pending_join.station_id);
                    if(rfd >= 0){
                        join_ok = cli->connect_fd(rfd, login_get_id(), login_get_pw(),
                                                  (uint8_t)login_get_tier());
                        if(!join_ok) close(rfd);
                    }
                } else {
                    // ── 직접 TCP JOIN (LAN) ───────────────────────────────
                    join_ok = cli->connect(pending_join.ip.c_str(),
                                           (int)pending_join.tcp_port,
                                           login_get_id(), login_get_pw(),
                                           (uint8_t)login_get_tier());
                }

                if(join_ok){
                    strncpy(connect_host, pending_join.ip.c_str(), sizeof(connect_host)-1);
                    connect_port = (int)pending_join.tcp_port;
                    strncpy(connect_id, login_get_id(), 31); connect_id[31]='\0';
                    strncpy(connect_pw, login_get_pw(), 63); connect_pw[63]='\0';
                    connect_tier = (uint8_t)login_get_tier();
                    // 외부 Central Server 경유 JOIN이면 station_id 보존 (재연결용)
                    if(!pending_join.station_id.empty() && s_central_host[0] != '\0')
                        s_central_join_station_id = pending_join.station_id;
                    else
                        s_central_join_station_id.clear();
                    mode_sel=2; pop_state=POP_NONE; mode_done=true;
                } else {
                    delete cli; cli=nullptr;
                    mode_err_msg="Connection failed";
                    mode_err_timer=3.f;
                    pop_state=POP_NONE;
                }
            }
            ImGui::PopStyleColor();
            if(!tier_ok) ImGui::EndDisabled();

            ImGui::SameLine(0.f, spacing);
            if(ImGui::Button("Cancel##jc", ImVec2(btn_cancel_w,28))) pop_state=POP_NONE;

            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }

        draw_early_chat(fw, fh);
        if(early_do_shutdown){ glfwSetWindowShouldClose(win, GLFW_TRUE); }
        if(early_do_logout){
            early_do_logout = false;
            do_logout = true;
            mode_done = true;
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    disc_listener.stop();
    central_cli.stop_polling();
    globe.destroy();

    if(glfwWindowShouldClose(win)){
        if(cli){ cli->disconnect(); delete cli; }
        ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
        return;
    }

    if(do_logout){
        if(cli){ cli->disconnect(); delete cli; cli=nullptr; }
    }
    if(!do_logout){

    // ── 모드에 따라 초기화 ────────────────────────────────────────────────
    // /reset(JOIN): cli가 없으면 저장된 connect 정보로 자동 재접속
    if(mode_sel==2 && !cli && connect_host[0] && connect_port > 0){
        cli = new NetClient();
        if(!cli->connect(connect_host, connect_port,
                         connect_id, connect_pw, connect_tier)){
            delete cli; cli = nullptr;
            mode_sel = 0; // 접속 실패 시 LOCAL로 fallback
        }
    }

    if(mode_sel==2 && cli){
        // CONNECT 모드: 하드웨어 없이 원격 수신
        v.remote_mode = true;
        v.net_cli     = cli;
        v.my_op_index = cli->my_op_index;
        strncpy(v.host_name, cli->my_name, 31);
        // 워터폴 타임스탬프 초기화
        { std::lock_guard<std::mutex> wlk(v.wf_events_mtx); v.wf_events.clear(); }
        v.last_tagged_sec = -1;

        // 채널 sync 콜백 등록
        cli->on_channel_sync = [&](const PktChannelSync& sync){
            for(int i=0;i<MAX_CHANNELS;i++){
                bool was_active = v.channels[i].filter_active;
                auto  was_mode  = v.channels[i].mode;
                bool now_active = (sync.ch[i].active != 0);
                auto now_mode   = (Channel::DemodMode)sync.ch[i].mode;

                v.channels[i].filter_active = now_active;
                // 드래그 중인 채널은 s/e를 덮어쓰지 않음 (덜덜 떨림 방지)
                bool dragging = v.channels[i].move_drag || v.channels[i].resize_drag;
                if(!dragging){
                    v.channels[i].s   = sync.ch[i].s;
                    v.channels[i].e   = sync.ch[i].e;
                }
                v.channels[i].mode= now_mode;
                v.channels[i].pan = sync.ch[i].pan;

                // audio_mask: 내 op_index 비트가 켜져 있으면 오디오 수신
                uint32_t srv_mask = sync.ch[i].audio_mask;
                uint32_t my_bit   = 1u << cli->my_op_index;
                uint32_t local_mask = (srv_mask & my_bit) ? 0x1u : 0x0u;
                v.channels[i].audio_mask.store(local_mask);
                v.channels[i].sq_threshold.store(sync.ch[i].sq_threshold,
                                                  std::memory_order_relaxed);
                v.channels[i].sq_sig.store(sync.ch[i].sq_sig,
                                           std::memory_order_relaxed);
                v.channels[i].sq_gate.store(sync.ch[i].sq_gate != 0,
                                            std::memory_order_relaxed);
                strncpy(v.channels[i].owner, sync.ch[i].owner_name, 31);
                v.srv_audio_mask[i] = sync.ch[i].audio_mask; // 전체 서버 마스크 보존

                // JOIN 모드: 복조는 HOST에서 수행, 로컬 dem 불필요
                // 오디오는 AUDIO_FRAME 수신으로 net_cli->audio 링 통해 출력
                // (dem_run은 실행하지 않음)

                // ── 채널 활성화 전이 처리 ──────────────────────────────
                if(!was_active && now_active){
                    if(!v.ch_created_by_me[i]){
                        // 내가 만들지 않은 채널 → 초기 M(Mute) 적용
                        v.local_ch_out[i] = 3;
                        cli->cmd_toggle_recv(i, false); // 서버에 오디오 수신 차단 요청
                    }
                    // 내가 만든 채널은 local_ch_out 기본값(1=L+R) 유지
                }
                if(was_active && !now_active){
                    // 채널 삭제 시 상태 초기화
                    v.ch_created_by_me[i] = false;
                    v.local_ch_out[i] = 1;
                }
            }
        };

        // WF 이벤트 수신 콜백 (IQ Start/Stop 표시)
        cli->on_wf_event = [&](const PktWfEvent& ev){
            FFTViewer::WfEvent wev{};
            wev.fft_idx  = v.current_fft_idx - (int)ev.fft_idx_offset;
            wev.wall_time= (time_t)ev.wall_time;
            wev.type     = (int)ev.type;
            strncpy(wev.label, ev.label, 31);
            std::lock_guard<std::mutex> lk(v.wf_events_mtx);
            v.wf_events.push_back(wev);
        };

        // 파일 수신 메타/진행/완료 콜백
        cli->on_file_meta = [&](const std::string& name, uint64_t total){
            {
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                bool found=false;
                for(auto& x : v.file_xfers){
                    if(!x.finished && x.total_bytes==0){
                        x.filename=name; x.total_bytes=total; found=true; break;
                    }
                }
                if(!found){
                    FFTViewer::FileXfer xf{}; xf.filename=name; xf.total_bytes=total;
                    v.file_xfers.push_back(xf);
                }
            }
            // JOIN: REQ_CONFIRMED region IQ → REQ_TRANSFERRING
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_CONFIRMED){
                        e.req_state=FFTViewer::RecEntry::REQ_TRANSFERRING;
                        e.filename=name; e.xfer_total=total; e.xfer_done=0;
                        break;
                    }
            }
        };
        cli->on_file_progress = [&](const std::string& name, uint64_t done, uint64_t total){
            {
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                for(auto& x : v.file_xfers)
                    if(x.filename==name && !x.finished){ x.done_bytes=done; x.total_bytes=total; break; }
            }
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING && e.filename==name){
                        e.xfer_done=done; break;
                    }
            }
        };
        cli->on_file_done = [&](const std::string& path, const std::string& name){
            {
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                bool found=false;
                for(auto& x : v.file_xfers){
                    if(x.filename==name && !x.finished){
                        x.finished=true; x.local_path=path; x.is_sa=true; found=true; break;
                    }
                }
                if(!found){
                    FFTViewer::FileXfer xf{}; xf.filename=name; xf.finished=true;
                    xf.local_path=path; xf.is_sa=true;
                    v.file_xfers.push_back(xf);
                }
            }
            // JOIN: REQ_TRANSFERRING → done (영역 IQ 요청 결과)
            bool is_region_iq = false;
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING && e.filename==name){
                        e.req_state=FFTViewer::RecEntry::REQ_NONE;
                        e.finished=true; e.path=path;
                        is_region_iq = true; break;
                    }
            }
            // 영역 IQ 요청 결과 → record/iq에 저장됨, rec_iq_files에 추가
            if(is_region_iq){
                bool f2=false; for(auto& s:rec_iq_files) if(s==name){f2=true;break;}
                if(!f2) rec_iq_files.push_back(name);
            } else {
                // Public 다운로드 파일 → share 폴더 (share/iq 또는 share/audio)
                std::string share_iq  = BEWEPaths::share_iq_dir();
                std::string share_aud = BEWEPaths::share_audio_dir();
                if(path.find(share_iq) == 0 || path.find(share_aud) == 0){
                    bool found=false;
                    for(auto& df:downloaded_files) if(df==name){found=true;break;}
                    if(!found) downloaded_files.push_back(name);
                    // share_iq_files / share_audio_files 에도 추가
                    if(path.find(share_iq) == 0){
                        bool f2=false; for(auto& s:share_iq_files) if(s==name){f2=true;break;}
                        if(!f2) share_iq_files.push_back(name);
                    } else {
                        bool f2=false; for(auto& s:share_audio_files) if(s==name){f2=true;break;}
                        if(!f2) share_audio_files.push_back(name);
                    }
                }
            }
        };

        // JOIN: 호스트가 실패한 경우 → REQ_CONFIRMED 항목을 REQ_DENIED로
        cli->on_region_response = [&](bool allowed){
            if(!allowed){
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_CONFIRMED){
                        e.req_state=FFTViewer::RecEntry::REQ_DENIED;
                        e.req_deny_timer=30.f; break;
                    }
            }
        };

        // JOIN: IQ 전송 진행상황 수신 (phase: 0=REC, 1=Transferring, 2=Done)
        cli->on_iq_progress = [&](const PktIqProgress& p){
            std::string fn(p.filename, strnlen(p.filename, 128));
            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
            for(auto& e : v.rec_entries){
                if(!e.is_region) continue;
                if(e.filename != fn && p.phase == 0) {
                    // phase=0(REC): 파일명으로 새로 매칭
                    if(e.req_state == FFTViewer::RecEntry::REQ_CONFIRMED && e.filename.empty())
                        e.filename = fn;
                }
                if(e.filename == fn || (p.phase == 0 && e.req_state == FFTViewer::RecEntry::REQ_CONFIRMED)){
                    if(p.phase == 0){
                        // [REC] Recording...
                        e.filename = fn;
                        e.req_state = FFTViewer::RecEntry::REQ_CONFIRMED;
                    } else if(p.phase == 1){
                        // [Transferring]
                        e.req_state = FFTViewer::RecEntry::REQ_TRANSFERRING;
                        e.xfer_done = p.done; e.xfer_total = p.total;
                    } else if(p.phase == 2){
                        // [Done]
                        e.xfer_done = p.total; e.xfer_total = p.total;
                        e.finished = true;
                    }
                    break;
                }
            }
        };

        // JOIN: 중앙서버 파이프 준비 신호 → 7702 포트에 연결해서 파일 수신
        cli->on_iq_pipe_ready = [&](uint32_t req_id, const char* filename, uint64_t filesize){
            std::string fn(filename);
            std::string save_dir = BEWEPaths::record_iq_dir();
            std::string save_path = save_dir + "/" + fn;
            std::string central_host_cap = s_central_host;
            // rec_entries에 TRANSFERRING 항목 추가
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                bool found = false;
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.filename == fn){ found=true; break; }
                if(!found){
                    FFTViewer::RecEntry e{};
                    e.filename = fn; e.is_region = true;
                    e.req_state = FFTViewer::RecEntry::REQ_TRANSFERRING;
                    e.xfer_total = filesize; e.xfer_done = 0;
                    e.t_start = std::chrono::steady_clock::now();
                    v.rec_entries.push_back(e);
                }
            }
            std::thread([req_id, fn, save_path, filesize, central_host_cap, &v, &central_cli](){
                int pipe_fd = central_cli.pipe_connect_join(central_host_cap, req_id);
                if(pipe_fd < 0){
                    printf("[UI] pipe_connect_join failed req_id=%u\n", req_id);
                    return;
                }
                FILE* fp = fopen(save_path.c_str(), "wb");
                if(!fp){
                    close(pipe_fd);
                    printf("[UI] cannot open save path: %s\n", save_path.c_str());
                    return;
                }
                std::vector<uint8_t> buf(256*1024);
                uint64_t received = 0;
                while(true){
                    ssize_t n = recv(pipe_fd, buf.data(), buf.size(), 0);
                    if(n <= 0) break;
                    fwrite(buf.data(), 1, (size_t)n, fp);
                    received += (uint64_t)n;
                    // xfer_done 업데이트
                    std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                    for(auto& e : v.rec_entries)
                        if(e.is_region && e.filename == fn){ e.xfer_done = received; break; }
                }
                fclose(fp);
                close(pipe_fd);
                printf("[UI] IQ pipe recv done: %s (%.1f MB)\n", fn.c_str(), received/1048576.0);
                // Done 상태로 전환
                {
                    std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                    for(auto& e : v.rec_entries)
                        if(e.is_region && e.filename == fn){
                            e.xfer_done = received; e.xfer_total = received;
                            e.finished = true; e.path = save_path;
                            break;
                        }
                }
                // rec_iq_files에 추가
                bool f2=false;
                for(auto& s : rec_iq_files) if(s==fn){f2=true;break;}
                if(!f2) rec_iq_files.push_back(fn);
                // 3초 후 항목 제거
                std::this_thread::sleep_for(std::chrono::seconds(3));
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                v.rec_entries.erase(
                    std::remove_if(v.rec_entries.begin(), v.rec_entries.end(),
                        [&fn](const FFTViewer::RecEntry& e){ return e.filename==fn && e.finished; }),
                    v.rec_entries.end());
            }).detach();
        };

        // JOIN: HOST Public 파일 목록 수신 (filename, size_bytes, uploader)
        cli->on_share_list = [&](const std::vector<std::tuple<std::string,uint64_t,std::string>>& files){
            std::lock_guard<std::mutex> lk(join_share_mtx);
            // 목록 갱신
            for(auto& f : files){
                const std::string& fn  = std::get<0>(f);
                uint64_t           fsz = std::get<1>(f);
                const std::string& upl = std::get<2>(f);
                bool found=false;
                for(auto& e : join_share_files)
                    if(e.filename==fn){ e.size_bytes=fsz; if(!upl.empty()) e.uploader=upl; found=true; break; }
                if(!found){ JoinShareEntry e; e.filename=fn; e.size_bytes=fsz; e.uploader=upl; join_share_files.push_back(e); }
            }
            // HOST 목록에 없는 항목은 제거
            join_share_files.erase(std::remove_if(join_share_files.begin(),join_share_files.end(),
                [&](const JoinShareEntry& e){
                    for(auto& f:files) if(std::get<0>(f)==e.filename) return false;
                    return true;
                }), join_share_files.end());
        };

        // JOIN: 수신 파일 저장 경로 결정
        // - region IQ 요청 결과 (REQ_TRANSFERRING 상태) → record/iq
        // - Public 다운로드 IQ_ / sa_                  → share/iq
        // - Public 다운로드 Audio_                      → share/audio
        cli->on_get_save_dir = [&v](const std::string& filename) -> std::string {
            // region IQ 요청 여부 확인
            // on_get_save_dir은 on_file_meta보다 먼저 호출되므로 e.filename이 아직 비어있음.
            // REQ_CONFIRMED/TRANSFERRING 상태의 region 항목이 있으면 이 파일이 region IQ 결과임.
            bool is_region = false;
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && (e.req_state==FFTViewer::RecEntry::REQ_CONFIRMED
                                    || e.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING)){
                        // 파일명을 여기서 미리 기록 (on_file_meta에서도 덮어씀)
                        if(e.filename.empty()) e.filename = filename;
                        is_region = true; break;
                    }
            }
            if(is_region){
                std::string dir = BEWEPaths::record_iq_dir();
                struct stat sd{}; if(stat(dir.c_str(),&sd)!=0) mkdir(dir.c_str(),0755);
                return dir;
            }
            // Public 다운로드: IQ/Audio 구분
            bool is_iq = (filename.size()>3 && filename.substr(0,3)=="IQ_")
                      || (filename.size()>3 && filename.substr(0,3)=="sa_");
            std::string dir = is_iq ? BEWEPaths::share_iq_dir() : BEWEPaths::share_audio_dir();
            struct stat sd{}; if(stat(dir.c_str(),&sd)!=0) mkdir(dir.c_str(),0755);
            return dir;
        };

        // 오퍼레이터 목록 팝업 — 건너뜀 (바로 메인으로 진입)
        bool op_popup_open = false;
        while(op_popup_open && !glfwWindowShouldClose(win)){
            glfwPollEvents();
            int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
            glViewport(0,0,fw,fh);
            glClearColor(0.03f,0.05f,0.10f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();

            const float OW=320.f, OH=240.f;
            ImGui::SetNextWindowPos(ImVec2((fw-OW)*0.5f,(fh-OH)*0.5f));
            ImGui::SetNextWindowSize(ImVec2(OW,OH));
            ImGui::SetNextWindowBgAlpha(0.95f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,8.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg,ImVec4(0.06f,0.08f,0.15f,1.f));
            ImGui::Begin("##op_list_popup",nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

            ImGui::SetWindowFontScale(1.1f);
            ImGui::TextColored(ImVec4(0.4f,0.7f,1.f,1.f),"Connected Operators");
            ImGui::SetWindowFontScale(1.0f);
            ImGui::Separator(); ImGui::Spacing();

            {
                std::lock_guard<std::mutex> lk(cli->op_mtx);
                if(cli->op_list.count==0){
                    ImGui::TextDisabled("(Waiting...)");
                } else {
                    for(int i=0;i<cli->op_list.count;i++){
                        auto& op=cli->op_list.ops[i];
                        char buf[80];
                        snprintf(buf,sizeof(buf),"%d. %s  [Tier%d]",
                                 op.index, op.name, op.tier);
                        bool is_me=(op.index==cli->my_op_index);
                        if(is_me)
                            ImGui::TextColored(ImVec4(0.3f,1.f,0.5f,1.f),"▶ %s",buf);
                        else
                            ImGui::Text("%s",buf);
                    }
                }
            }
            ImGui::Spacing();
            float bw2=90.f;
            ImGui::SetCursorPosX((OW-bw2)*0.5f);
            if(ImGui::Button("ENTER",ImVec2(bw2,26)) ||
               cli->op_list_updated.load()){
                op_popup_open=false;
            }
            ImGui::End();
            ImGui::PopStyleColor(); ImGui::PopStyleVar();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(win);
        }

        // ── remote mode: 버퍼/텍스처 초기화 (HW 없음) ──────────────────
        // 첫 FFT 수신 전 기본 크기로 초기화 (수신 후 재조정됨)
        v.fft_size = DEFAULT_FFT_SIZE;
        v.header.fft_size  = DEFAULT_FFT_SIZE;
        v.header.power_min = -100.f;
        v.header.power_max = 0.f;
        v.display_power_min = -80.f;
        v.display_power_max = 0.f;
        v.fft_data.assign((size_t)MAX_FFTS_MEMORY * DEFAULT_FFT_SIZE, 0);
        v.current_spectrum.assign(DEFAULT_FFT_SIZE, -80.f);
        v.autoscale_active = false;
        v.create_waterfall_texture();

        // remote mode: mix worker만 시작 (오디오 출력용)
        v.mix_stop.store(false);
        v.mix_thr=std::thread(&FFTViewer::mix_worker,&v);

    } else {
        // LOCAL or HOST: 하드웨어 초기화
        // host_name = 로그인 ID
        strncpy(v.host_name, login_get_id(), 31);
        if(!v.initialize(cf)){
            // SDR 없음: 오류 상태로 표시하고 대기 (프로그램은 계속 실행)
            printf("SDR init failed — running without hardware (SDR LED red)\n");
            v.sdr_stream_error.store(true);
            // 버퍼/텍스처 기본 초기화 (SA 재생 등은 가능)
            v.fft_size = DEFAULT_FFT_SIZE;
            v.header.fft_size  = DEFAULT_FFT_SIZE;
            v.header.power_min = -100.f;
            v.header.power_max = 0.f;
            v.display_power_min = -80.f;
            v.display_power_max = 0.f;
            v.fft_data.assign((size_t)MAX_FFTS_MEMORY * DEFAULT_FFT_SIZE, 0);
            v.current_spectrum.assign(DEFAULT_FFT_SIZE, -80.f);
            v.autoscale_active = false;
            v.create_waterfall_texture();
        } else {
            if(v.hw.type == HWType::BLADERF)
                cap = std::thread(&FFTViewer::capture_and_process, &v);
            else
                cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
        }
        v.mix_stop.store(false);
        v.mix_thr=std::thread(&FFTViewer::mix_worker,&v);

        if(mode_sel==1){
            // HOST: 서버 시작 (public 목록 + 소유자/리스너 초기화)
            shared_files.clear(); pub_iq_files.clear(); pub_audio_files.clear();
            pub_listeners.clear(); pub_owners.clear();
            ch_sync_dirty_flag.store(false);
            srv = new NetServer();
            // 인증: 로그인 시스템과 동일하게 처리 (여기서는 간단히 항상 허용)
            srv->cb.on_auth = [&,srv](const char* id, const char* pw,
                                   uint8_t tier, uint8_t& idx) -> bool {
                // TODO: 실제 인증 로직 연결
                static uint8_t next=1;
                idx = next++;
                if(next>MAX_OPERATORS) next=1;
                // 인증 성공 시 현재 share 목록 전송 (join 초기 동기화)
                // 백그라운드 스레드에서 약간 지연 후 전송 (AUTH_ACK 후 처리)
                uint8_t new_idx = idx;
                // pub_owners를 캡처하여 업로더 정보 포함 전송
                std::thread([srv, new_idx](){
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
                    auto scan_pub = [&](const std::string& dir){
                        DIR* ds = opendir(dir.c_str());
                        if(!ds) return;
                        struct dirent* ent;
                        while((ent=readdir(ds))!=nullptr){
                            const char* n = ent->d_name;
                            size_t nl = strlen(n);
                            if(nl>4 && strcmp(n+nl-4,".wav")==0){
                                std::string fp = dir+"/"+n;
                                struct stat st{}; uint64_t fsz=0;
                                if(stat(fp.c_str(),&st)==0) fsz=(uint64_t)st.st_size;
                                std::string upl;
                                auto it=pub_owners.find(n);
                                if(it!=pub_owners.end()) upl=it->second;
                                slist.push_back({n, fsz, upl});
                            }
                        }
                        closedir(ds);
                    };
                    scan_pub(BEWEPaths::public_iq_dir());
                    scan_pub(BEWEPaths::public_audio_dir());
                    if(!slist.empty()) srv->send_share_list((int)new_idx, slist);
                }).detach();
                return true;
            };
            // 서버 콜백 → FFTViewer 직접 제어
            srv->cb.on_set_freq   = [&](float cf){ v.set_frequency(cf); };
            srv->cb.on_set_gain   = [&](float db){ v.gain_db=db; v.set_gain(db); };
            srv->cb.on_create_ch  = [&](int idx, float s, float e, const char* creator){
                if(idx<0||idx>=MAX_CHANNELS) return;
                v.channels[idx].s=s; v.channels[idx].e=e;
                v.channels[idx].filter_active=true;
                v.channels[idx].mode=Channel::DM_NONE;
                v.channels[idx].pan=0;
                v.channels[idx].sq_calibrated.store(false);
                v.channels[idx].ar_wp.store(0); v.channels[idx].ar_rp.store(0);
                strncpy(v.channels[idx].owner, creator?creator:"", 31);
                // JOIN이 만든 채널: 모든 오퍼레이터 비트 ON, HOST bit(0) OFF → HOST 초기 Mute
                v.channels[idx].audio_mask.store(0xFFFFFFFFu & ~0x1u);
                v.local_ch_out[idx] = 3; // HOST도 비생성자이므로 초기 M(Mute)
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_delete_ch  = [&](int idx){
                if(idx<0||idx>=MAX_CHANNELS) return;
                v.stop_dem(idx); v.stop_digi(idx); v.digi_panel_on[idx]=false;
                v.channels[idx].filter_active=false;
                v.channels[idx].mode=Channel::DM_NONE;
                v.local_ch_out[idx] = 1; // 슬롯 재사용 대비 기본값 복원
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_set_ch_mode= [&](int idx, int mode){
                if(idx<0||idx>=MAX_CHANNELS) return;
                v.stop_dem(idx);
                auto dm=(Channel::DemodMode)mode;
                v.channels[idx].mode=dm;
                if(dm!=Channel::DM_NONE && v.channels[idx].filter_active)
                    v.start_dem(idx,dm);
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_set_ch_audio=[&](int idx, uint32_t mask){
                if(idx<0||idx>=MAX_CHANNELS) return;
                v.channels[idx].audio_mask.store(mask);
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_set_ch_pan =[&](int idx, int pan){
                if(idx<0||idx>=MAX_CHANNELS) return;
                v.channels[idx].pan=pan;
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_set_sq_thresh = [&](int idx2, float thr){
                if(idx2<0||idx2>=MAX_CHANNELS) return;
                v.channels[idx2].sq_threshold.store(thr, std::memory_order_relaxed);
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_set_autoscale = [&](){
                v.autoscale_active=true; v.autoscale_init=false;
                v.autoscale_accum.clear();
            };
            srv->cb.on_toggle_tm_iq = [&](){
                bool cur=v.tm_iq_on.load();
                if(cur){
                    v.tm_iq_on.store(false); v.tm_add_event_tag(2); v.tm_iq_was_stopped=true;
                    srv->broadcast_wf_event(0,(int64_t)time(nullptr),2,"IQ Stop");
                } else {
                    if(v.tm_iq_was_stopped){ v.tm_iq_close(); v.tm_iq_was_stopped=false; }
                    v.tm_iq_open();
                    if(v.tm_iq_file_ready){
                        v.tm_iq_on.store(true); v.tm_add_event_tag(1);
                        srv->broadcast_wf_event(0,(int64_t)time(nullptr),1,"IQ Start");
                    }
                }
            };
            srv->cb.on_set_capture_pause = [&](bool pause){
                v.capture_pause.store(pause);
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS); // status 동기화
            };
            srv->cb.on_set_spectrum_pause = [&](bool pause){
                v.spectrum_pause.store(pause);
            };
            srv->cb.on_request_region = [&](uint8_t op_idx, const char* op_name,
                                             int32_t fft_top, int32_t fft_bot,
                                             float freq_lo, float freq_hi,
                                             int32_t time_start, int32_t time_end){
                std::string fname;
                {
                    std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                    FFTViewer::RecEntry e{};
                    time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
                    char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
                    float cf_mhz = (freq_lo+freq_hi)/2.0f;
                    char fn[128]; snprintf(fn,sizeof(fn),"IQ_%.3fMHz_%s.wav",cf_mhz,dts);
                    e.filename = fn;
                    e.is_region = true;
                    e.req_state = FFTViewer::RecEntry::REQ_CONFIRMED;
                    e.req_op_idx = op_idx;
                    strncpy(e.req_op_name, op_name?op_name:"?", 31);
                    e.req_fft_top=fft_top; e.req_fft_bot=fft_bot;
                    e.req_freq_lo=freq_lo; e.req_freq_hi=freq_hi;
                    e.req_time_start=time_start; e.req_time_end=time_end;
                    e.t_start=std::chrono::steady_clock::now();
                    v.rec_entries.push_back(e);
                    fname = fn;
                }
                float rps=(float)v.header.sample_rate/(float)v.fft_input_size/(float)v.time_average;
                if(rps<=0.f) rps=37.5f;
                time_t now_h=time(nullptr);
                int cur_fi=v.current_fft_idx;
                int32_t ft=(int32_t)(cur_fi-(int32_t)((now_h-(time_t)time_end)*rps));
                int32_t fb=(int32_t)(cur_fi-(int32_t)((now_h-(time_t)time_start)*rps));
                float fl=freq_lo, fh=freq_hi;
                int32_t ts=time_start, te=time_end;
                uint8_t oidx=op_idx;
                // station_id: central_cli에 등록된 룸 ID
                std::string sid = v.station_name + "_" + std::string(login_get_id());
                std::string central_host_cap = s_central_host;
                std::thread([&v,srv,ft,fb,fl,fh,ts,te,oidx,fname,sid,central_host_cap,&central_cli](){
                    // [REC] 상태 표시
                    {
                        std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                        for(auto& e:v.rec_entries)
                            if(e.filename==fname){ e.req_state=FFTViewer::RecEntry::REQ_CONFIRMED; break; }
                    }
                    for(int w=0;w<200&&v.rec_busy_flag.load();w++)
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    v.region.fft_top=ft; v.region.fft_bot=fb;
                    v.region.freq_lo=fl; v.region.freq_hi=fh;
                    v.region.time_start=(time_t)ts;
                    v.region.time_end=(time_t)te;
                    v.region.active=true;
                    v.rec_busy_flag.store(true);
                    v.rec_state = FFTViewer::REC_BUSY;
                    v.rec_anim_timer = 0.0f;
                    v.region.active = false;
                    // IQ_PROGRESS phase=0 (REC 중) 브로드캐스트
                    if(srv){
                        PktIqProgress prog{};
                        static std::atomic<uint32_t> req_id_counter{1};
                        prog.req_id = req_id_counter.fetch_add(1);
                        strncpy(prog.filename, fname.c_str(), 127);
                        prog.done=0; prog.total=0; prog.phase=0;
                        srv->broadcast_iq_progress(prog);
                    }
                    v.do_region_save_work();
                    v.rec_state = FFTViewer::REC_SUCCESS;
                    v.rec_success_timer = 3.0f;
                    v.rec_busy_flag.store(false);
                    std::string path;
                    {
                        std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                        for(auto it=v.rec_entries.rbegin();it!=v.rec_entries.rend();++it)
                            if(it->is_region&&it->req_state==FFTViewer::RecEntry::REQ_NONE&&it->finished){
                                path=it->path;
                                v.rec_entries.erase(std::next(it).base());
                                break;
                            }
                    }
                    if(path.empty()){
                        if(srv) srv->send_region_response((int)oidx, false);
                        {
                            std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                            for(auto it2=v.rec_entries.begin();it2!=v.rec_entries.end();++it2)
                                if(it2->filename==fname){ v.rec_entries.erase(it2); break; }
                        }
                        return;
                    }
                    uint64_t fsz=0;
                    {FILE* f=fopen(path.c_str(),"rb");if(f){fseek(f,0,SEEK_END);fsz=(uint64_t)ftell(f);fclose(f);}}
                    {
                        std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                        for(auto& e:v.rec_entries)
                            if(e.filename==fname){
                                e.req_state=FFTViewer::RecEntry::REQ_TRANSFERRING;
                                e.xfer_total=fsz; e.xfer_done=0;
                                e.local_path_to_delete=path;
                                break;
                            }
                    }
                    static std::atomic<uint32_t> g_req_id{1000};
                    uint32_t req_id = g_req_id.fetch_add(1);
                    const char* fn_only = strrchr(path.c_str(),'/');
                    fn_only = fn_only ? fn_only+1 : path.c_str();

                    // 중앙서버 파이프에 연결
                    int pipe_fd = -1;
                    if(central_cli.is_central_connected()){
                        pipe_fd = central_cli.pipe_connect_host(
                            central_host_cap, sid, req_id, 0xFFFF,
                            fn_only, fsz);
                    }

                    if(pipe_fd >= 0 && srv){
                        srv->send_file_via_pipe(pipe_fd, path.c_str(), req_id,
                            [&v,fname,fsz](uint64_t done, uint64_t total){
                                std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                for(auto& e:v.rec_entries)
                                    if(e.filename==fname){
                                        e.xfer_done=done; e.xfer_total=total>0?total:fsz;
                                        if(done >= (total>0?total:fsz) && total>0)
                                            e.finished=true;
                                        break;
                                    }
                            });
                    } else if(srv){
                        // 파이프 실패 시 기존 MUX 경유로 폴백
                        uint8_t tid=v.next_transfer_id.fetch_add(1);
                        srv->send_file_to((int)oidx,path.c_str(),tid,
                            [&v,fname](uint64_t done, uint64_t total){
                                std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                for(auto& e:v.rec_entries)
                                    if(e.filename==fname){ e.xfer_done=done; break; }
                            });
                    }
                    // Done 후 3초 표시 후 HOST rec_entries 제거 + 파일 삭제
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    {
                        std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                        for(auto it2=v.rec_entries.begin();it2!=v.rec_entries.end();++it2)
                            if(it2->filename==fname){ v.rec_entries.erase(it2); break; }
                    }
                    remove(path.c_str());
                    printf("[UI] IQ pipe transfer done, deleted %s\n", path.c_str());
                }).detach();
            };
            srv->cb.on_toggle_recv = [&](int ch_idx, uint8_t op_idx, bool enable){
                // 릴레이가 TOGGLE_RECV를 처리하므로 여기는 호출되지 않지만
                // 만약 호출되면 로컬 mask만 갱신 (broadcast 안 함)
                if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
                uint32_t bit = 1u << op_idx;
                uint32_t old_mask = v.channels[ch_idx].audio_mask.load();
                uint32_t new_mask;
                do {
                    new_mask = enable ? (old_mask | bit) : (old_mask & ~bit);
                } while(!v.channels[ch_idx].audio_mask.compare_exchange_weak(old_mask, new_mask));
            };
            srv->cb.on_update_ch_range = [&](int idx, float s, float e){
                if(idx<0||idx>=MAX_CHANNELS) return;
                v.channels[idx].s = s;
                v.channels[idx].e = e;
                // 복조 중이면 재시작
                if(v.channels[idx].dem_run.load()){
                    Channel::DemodMode md = v.channels[idx].mode;
                    v.stop_dem(idx); v.start_dem(idx, md);
                }
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_start_rec  = [&](int){ v.start_rec(); };
            srv->cb.on_stop_rec   = [&](){ v.stop_rec(); };
            srv->cb.on_chat       = [&](const char*,const char*){};
            // HOST: JOIN이 public 파일 다운로드 요청
            srv->cb.on_share_download_req = [&](uint8_t op_idx, const char* filename){
                // IQ/Audio 구분하여 올바른 폴더에서 찾기
                std::string fn(filename);
                bool is_iq = (fn.size()>3 && fn.substr(0,3)=="IQ_") || (fn.size()>3 && fn.substr(0,3)=="sa_");
                std::string path = (is_iq ? BEWEPaths::public_iq_dir() : BEWEPaths::public_audio_dir()) + "/" + fn;
                struct stat st{};
                if(stat(path.c_str(),&st)!=0){ fprintf(stderr,"share_download: file not found: %s\n",path.c_str()); return; }
                // 다운로드 요청자 이름 기록
                {
                    auto ops = srv->get_operators();
                    for(auto& op : ops){
                        if(op.index == op_idx){
                            std::string fn(filename);
                            auto& listeners = pub_listeners[fn];
                            bool already = false;
                            for(auto& s : listeners) if(s==std::string(op.name)){already=true;break;}
                            if(!already) listeners.push_back(std::string(op.name));
                            break;
                        }
                    }
                }
                uint8_t tid = v.next_transfer_id.fetch_add(1);
                // 백그라운드 스레드에서 전송: client_loop 스레드 블로킹 방지
                // (inline 호출 시 send_audio가 send_mtx를 기다리며 demod 스레드 블로킹 → HOST 오디오 끊김)
                std::string path_copy = path;
                int op_int = (int)op_idx;
                std::thread([srv, path_copy, op_int, tid](){
                    srv->send_file_to(op_int, path_copy.c_str(), tid);
                }).detach();
            };
            // HOST: JOIN이 파일을 업로드 완료 → public/iq 또는 public/audio로 이동 + 목록 갱신
            srv->cb.on_share_upload_done = [&](uint8_t /*op_idx*/, const char* op_name, const char* tmp_path){
                const char* fn = strrchr(tmp_path, '/'); fn = fn ? fn+1 : tmp_path;
                // "bewe_up_" 접두사 제거
                if(strncmp(fn,"bewe_up_",8)==0) fn+=8;
                // IQ/Audio 구분
                bool is_iq = (strlen(fn)>3 && strncmp(fn,"IQ_",3)==0)
                          || (strlen(fn)>3 && strncmp(fn,"sa_",3)==0);
                std::string pub_dir = is_iq ? BEWEPaths::public_iq_dir() : BEWEPaths::public_audio_dir();
                struct stat sd{}; if(stat(pub_dir.c_str(),&sd)!=0) mkdir(pub_dir.c_str(),0755);
                std::string dst = pub_dir + "/" + fn;
                // 임시 파일 → public 폴더로 복사
                FILE* fin = fopen(tmp_path,"rb"); FILE* fout = fopen(dst.c_str(),"wb");
                if(fin&&fout){ char buf[65536]; size_t n; while((n=fread(buf,1,sizeof(buf),fin))>0) fwrite(buf,1,n,fout); }
                if(fin) fclose(fin); if(fout) fclose(fout);
                remove(tmp_path); // 임시 파일 삭제
                std::string fname(fn);
                // pub_owners: 업로드한 사람 기록
                pub_owners[fname] = std::string(op_name);
                // pub_iq_files / pub_audio_files / shared_files 갱신
                if(is_iq){
                    bool dup=false; for(auto& sf:pub_iq_files) if(sf==fname){dup=true;break;}
                    if(!dup) pub_iq_files.push_back(fname);
                } else {
                    bool dup=false; for(auto& sf:pub_audio_files) if(sf==fname){dup=true;break;}
                    if(!dup) pub_audio_files.push_back(fname);
                }
                {
                    bool dup=false; for(auto& sf:shared_files) if(sf==fname){dup=true;break;}
                    if(!dup) shared_files.push_back(fname);
                }
                // 모든 JOIN에게 갱신된 public 목록 브로드캐스트
                {
                    std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
                    for(auto& sf : shared_files){
                        bool siq = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
                        std::string sfp = (siq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
                        struct stat sst{}; uint64_t fsz=0;
                        if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
                        std::string upl;
                        auto it=pub_owners.find(sf);
                        if(it!=pub_owners.end()) upl=it->second;
                        slist.push_back({sf,fsz,upl});
                    }
                    srv->send_share_list(-1, slist);
                }
                printf("Public upload done: %s (from %s)\n", fn, op_name);
            };
            // JOIN이 /chassis 1 reset 명령 전송 → HOST 측에서 재시작
            // JOIN → HOST: FFT size 변경 (HOST 적용 후 FFT_FRAME으로 자동 동기화)
            srv->cb.on_set_fft_size = [&](uint32_t size){
                static const int valid[]={512,1024,2048,4096,8192,16384};
                for(int vs : valid){
                    if((uint32_t)vs==size){
                        v.pending_fft_size=size; v.fft_size_change_req=true;
                        break;
                    }
                }
            };
            // JOIN → HOST: SR 변경
            srv->cb.on_set_sr = [&](float msps){
                v.pending_sr_msps=msps; v.sr_change_req=true;
                v.autoscale_active=true; v.autoscale_init=false; v.autoscale_accum.clear();
            };
            srv->cb.on_chassis_reset = [&](){
                // 네트워크 스레드에서 호출 → 플래그만 세팅, 실제 처리는 메인 루프
                pending_chassis1_reset.store(true);
            };
            // JOIN이 /chassis 2 reset 전송 → 네트워크 브로드캐스트 일시 중단 후 재개
            srv->cb.on_net_reset = [&](){
                // 네트워크 스레드에서 호출 → 플래그만 세팅, 실제 처리는 메인 루프
                pending_chassis2_reset.store(true);
            };
            srv->cb.on_rx_stop = [&](){ pending_rx_stop.store(true); };
            srv->cb.on_rx_start = [&](){ pending_rx_start.store(true); };
            // JOIN이 public 파일 삭제 요청 → 소유자 확인 후 삭제 + 브로드캐스트
            srv->cb.on_pub_delete_req = [&](const char* op_name, const char* filename){
                std::string fname(filename);
                // 소유자만 삭제 허용
                auto oit = pub_owners.find(fname);
                if(oit == pub_owners.end() || oit->second != std::string(op_name)) return;
                // 실제 파일 삭제
                bool is_iq = (fname.size()>3&&fname.substr(0,3)=="IQ_")||(fname.size()>3&&fname.substr(0,3)=="sa_");
                std::string fp = (is_iq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+fname;
                remove(fp.c_str());
                // 목록에서 제거
                auto rm_from = [&](std::vector<std::string>& vec){
                    vec.erase(std::remove(vec.begin(),vec.end(),fname),vec.end());
                };
                rm_from(pub_iq_files); rm_from(pub_audio_files); rm_from(shared_files);
                pub_owners.erase(fname);
                pub_listeners.erase(fname);
                // 갱신된 목록 브로드캐스트
                std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
                for(auto& sf : shared_files){
                    bool siq2 = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
                    std::string sfp = (siq2?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
                    struct stat sst{}; uint64_t fsz=0;
                    if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
                    std::string upl; auto it=pub_owners.find(sf); if(it!=pub_owners.end()) upl=it->second;
                    slist.push_back({sf,fsz,upl});
                }
                srv->send_share_list(-1, slist);
            };

            // port 0 → OS가 빈 포트 자동 할당
            if(!srv->start(0)){
                printf("Server start failed\n");
                delete srv; srv=nullptr;
            } else {
                host_port = srv->listen_port(); // 실제 할당된 포트 기록
                v.net_srv = srv;
                srv->set_host_info(login_get_id(), (uint8_t)login_get_tier());
                // HOST station 정보를 static에 저장 (/reset 재진입 시 복원)
                if(v.station_location_set){
                    s_station_name = v.station_name;
                    s_station_lat  = v.station_lat;
                    s_station_lon  = v.station_lon;
                    s_station_set  = true;
                }
                // LAN 브로드캐스트 시작
                if(v.station_location_set){
                    std::string lip = get_local_ip();
                    srv->start_discovery_broadcast(
                        v.station_name.c_str(),
                        v.station_lat, v.station_lon,
                        (uint16_t)host_port,
                        lip.c_str(),
                        (uint8_t)login_get_tier());
                }
                // Central MUX 어댑터 시작 (Central Server)
                if(s_central_host[0] != '\0' && v.station_location_set){
                    auto central_connect = [&v, &central_cli,
                                          rh = std::string(s_central_host),
                                          rp = s_central_port,
                                          _log_mtx = &host_chat_mtx,
                                          _log = &host_chat_log](){
                        std::string sid = v.station_name + "_" + std::string(login_get_id());
                        int rfd = central_cli.open_room(
                            rh, rp, sid, v.station_name,
                            v.station_lat, v.station_lon,
                            (uint8_t)login_get_tier());
                        if(rfd >= 0){
                            // 릴레이가 재작성한 CHANNEL_SYNC → HOST의 audio_mask 갱신
                            central_cli.set_on_central_ch_sync([&v](const uint8_t* pkt, size_t len){
                                if(len < 9 + 60*10) return;  // BEWE_HDR + 10 entries
                                const uint8_t* payload = pkt + 9;
                                for(int i=0; i<MAX_CHANNELS && i<10; i++){
                                    uint32_t mask;
                                    memcpy(&mask, payload + i*60 + 12, sizeof(mask));
                                    v.channels[i].audio_mask.store(mask);
                                }
                            });
                            if(v.net_srv){
                                v.net_srv->cb.on_relay_broadcast = [&central_cli](const uint8_t* pkt, size_t len){
                                    central_cli.enqueue_relay_broadcast(pkt, len);
                                };
                            }
                            central_cli.set_on_central_chat([_log_mtx, _log](const char* from, const char* msg){
                                std::lock_guard<std::mutex> lk(*_log_mtx);
                                if((int)_log->size() >= 200) _log->erase(_log->begin());
                                LocalChatMsg m{}; strncpy(m.from,from,31); strncpy(m.msg,msg,255);
                                _log->push_back(m);
                            });
                            // 재귀적 자동 재연결 함수 (shared_ptr로 캡처)
                            auto reconnect_fn = std::make_shared<std::function<void()>>();
                            *reconnect_fn = [&v, &central_cli, rh, rp, reconnect_fn](){
                                // Central Server 끊김 → 자동 재연결 (3초 후, 최대 5회)
                                std::thread([&v, &central_cli, rh, rp, reconnect_fn](){
                                    for(int attempt=0; attempt<5; attempt++){
                                        std::this_thread::sleep_for(std::chrono::seconds(3));
                                        if(!v.net_srv) return;  // HOST 모드 종료됨
                                        printf("[UI] Central auto-reconnect attempt %d/5\n", attempt+1);
                                        std::string sid = v.station_name + "_" + std::string(login_get_id());
                                        int rfd2 = central_cli.open_room(
                                            rh, rp, sid, v.station_name,
                                            v.station_lat, v.station_lon,
                                            (uint8_t)login_get_tier());
                                        if(rfd2 >= 0){
                                            central_cli.start_mux_adapter(rfd2,
                                                [&v](int fd2){ if(v.net_srv) v.net_srv->inject_fd(fd2); },
                                                [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                                                *reconnect_fn);
                                            printf("[UI] Central auto-reconnected\n");
                                            return;
                                        }
                                    }
                                    printf("[UI] Central auto-reconnect failed after 5 attempts\n");
                                }).detach();
                            };
                            central_cli.start_mux_adapter(rfd,
                                [&v](int local_fd){ if(v.net_srv) v.net_srv->inject_fd(local_fd); },
                                [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                                *reconnect_fn);
                            printf("[UI] Central MUX adapter started\n");
                        } else {
                            printf("[UI] Central open_room failed, Central unavailable\n");
                        }
                    };
                    central_connect();
                }
                // 브로드캐스트 전용 스레드 시작 (캡처 스레드 분리)
                v.net_bcast_stop.store(false);
                v.net_bcast_thr = std::thread(&FFTViewer::net_bcast_worker, &v);
            }
        }
    }

    // ── System monitor state ──────────────────────────────────────────────
    auto cpu_last_time  =std::chrono::steady_clock::now();
    auto status_last    =std::chrono::steady_clock::now();
    auto sq_sync_last   =std::chrono::steady_clock::now();
    auto heartbeat_last =std::chrono::steady_clock::now();
    long long cpu_last_idle=0, cpu_last_total=0, io_last_ms=0;

    auto read_cpu=[&](long long& idle, long long& total){
        FILE* f=fopen("/proc/stat","r"); if(!f){idle=total=0;return;}
        long long u,n,s,i,iow,irq,sirq;
        fscanf(f,"cpu %lld %lld %lld %lld %lld %lld %lld",&u,&n,&s,&i,&iow,&irq,&sirq);
        fclose(f); idle=i+iow; total=u+n+s+i+iow+irq+sirq;
    };
    auto read_ram=[&]()->float{
        FILE* f=fopen("/proc/meminfo","r"); if(!f) return 0.0f;
        long long total=0,avail=0; char key[64]; long long val;
        for(int i=0;i<10;i++){
            if(fscanf(f,"%63s %lld kB",key,&val)!=2) break;
            if(!strcmp(key,"MemTotal:"))      total=val;
            else if(!strcmp(key,"MemAvailable:")) avail=val;
        }
        fclose(f);
        return (total>0)?(float)(total-avail)/total*100.0f:0.0f;
    };
    auto read_ghz=[&]()->float{
        double sum=0; int cnt=0;
        for(int c=0;c<256;c++){
            char path[128];
            snprintf(path,sizeof(path),"/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq",c);
            FILE* f=fopen(path,"r"); if(!f) break;
            long long khz=0; fscanf(f,"%lld",&khz); fclose(f);
            sum+=khz; cnt++;
        }
        return cnt>0?(float)(sum/cnt/1e6):0.0f;
    };
    auto read_io_ms=[&]()->long long{
        FILE* f=fopen("/proc/diskstats","r"); if(!f) return 0;
        long long sum=0; char dev[32]; unsigned int maj,min_;
        long long f1,f2,f3,f4,f5,f6,f7,f8,f9,io_ticks;
        while(fscanf(f,"%u %u %31s %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %*[^\n]",
                     &maj,&min_,dev,&f1,&f2,&f3,&f4,&f5,&f6,&f7,&f8,&f9,&io_ticks)==13){
            if(dev[0]=='s'&&dev[2]>='a'&&dev[2]<='z'&&dev[3]=='\0') sum+=io_ticks;
            else if(dev[0]=='n'&&dev[1]=='v'&&strstr(dev,"p")==nullptr) sum+=io_ticks;
            else if(dev[0]=='v'&&dev[1]=='d'&&dev[3]=='\0') sum+=io_ticks;
        }
        fclose(f); return sum;
    };
    read_cpu(cpu_last_idle,cpu_last_total);
    io_last_ms=read_io_ms();

    // ── 채팅/오퍼레이터 UI 상태 ──────────────────────────────────────────
    bool  chat_open        = false;
    bool  chat_focus_input = false; // 채팅창 입력 포커스 상태 (외부 키 핸들러에서 접근)
    bool  chat_cursor_end  = false; // 다음 프레임에 커서를 끝으로 이동 (/ 입력 후 선택 방지)
    char  chat_input[256] = {};

    bool  ops_open     = false;
    bool  stat_open    = false;
    bool  board_open   = false;
    int   last_fft_seq = -1;  // CONNECT 모드 FFT 시퀀스 추적
    bool  chat_scroll_bottom = false;
    // 파일 우클릭 컨텍스트 메뉴
    struct FileCtxMenu {
        bool open=false; float x=0,y=0;
        std::string filepath, filename;
        bool is_public=false; // Public 탭 파일 (소유자만 삭제)
        bool selected=false;  // 좌클릭 선택 상태
    } file_ctx;

    // chassis 1 reset 후 HOST 재시작: stable 메시지 (JOIN 재접속 전이므로 로컬만)
    if(mode_sel == 1 && chassis_reset_mode == 1 && v.net_srv){
        LocalChatMsg lm{}; lm.is_error = false;
        strncpy(lm.from, "BEWE", 31);
        strncpy(lm.msg,  "Chassis 1 stable ...", 255);
        host_chat_log.push_back(lm);
    }

    // HOST 모드: 수신 채팅을 로컬 로그에도 저장
    if(v.net_srv){
        v.net_srv->cb.on_chat = [&](const char* from, const char* msg){
            std::lock_guard<std::mutex> lk(host_chat_mtx);
            if((int)host_chat_log.size() >= 200) host_chat_log.erase(host_chat_log.begin());
            LocalChatMsg m{}; strncpy(m.from,from,31); strncpy(m.msg,msg,255);
            host_chat_log.push_back(m);
            chat_scroll_bottom = true;
        };
    }

    // ── TM IQ 기본 활성화 (HOST/LOCAL 모드) ───────────────────────────────
    if(!v.remote_mode){
        v.tm_iq_open();
        if(v.tm_iq_file_ready) v.tm_iq_on.store(true);
    }

    // 모든 모드: VSync OFF, 60fps 자체 캡 (포커스 여부 무관)
    // 워터폴 연속성 보장을 위해 백그라운드도 동일 프레임레이트 유지
    glfwSwapInterval(0);
    // ── Main loop ─────────────────────────────────────────────────────────
    using clk = std::chrono::steady_clock;
    static constexpr float FRAME_TARGET = 1.0f / 60.0f; // 60fps
    clk::time_point frame_last = clk::now();
    while(!glfwWindowShouldClose(win) && !do_logout && !do_main_menu){
        // 60fps 캡: 포커스/백그라운드 구분 없이 동일
        {
            auto now = clk::now();
            float elapsed = std::chrono::duration<float>(now - frame_last).count();
            if(elapsed < FRAME_TARGET)
                std::this_thread::sleep_for(std::chrono::microseconds(
                    (int)((FRAME_TARGET - elapsed) * 1e6f)));
            frame_last = clk::now();
        }
        glfwPollEvents();

        // Window title: system monitor (1s interval)
        {
            auto now=std::chrono::steady_clock::now();
            float dt=std::chrono::duration<float>(now-cpu_last_time).count();
            if(dt>=1.0f){
                long long idle,total; read_cpu(idle,total);
                long long d_idle=idle-cpu_last_idle, d_total=total-cpu_last_total;
                float cpu_pct=(d_total>0)?(1.0f-(float)d_idle/d_total)*100.0f:0.0f;
                cpu_last_idle=idle; cpu_last_total=total; cpu_last_time=now;
                long long io_now=read_io_ms();
                float io_pct=std::min(100.0f,(float)(io_now-io_last_ms)/10.0f);
                io_last_ms=io_now;
                v.sysmon_cpu=cpu_pct;
                v.sysmon_ghz=read_ghz();
                v.sysmon_ram=read_ram();
                v.sysmon_io =io_pct;
                glfwSetWindowTitle(win,"BEWE");
            }
        }

        // ── HOST: 100ms마다 sq_sig/gate 포함 채널 sync ─────────────────────
        if(v.net_srv && v.net_srv->client_count()>0){
            auto now2=std::chrono::steady_clock::now();
            float el2=std::chrono::duration<float>(now2-sq_sync_last).count();
            if(el2>=0.1f){
                sq_sync_last=now2;
                v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            }
        }

        // ── HOST: 1초마다 STATUS 브로드캐스트 ───────────────────────────────
        if(v.net_srv && v.net_srv->client_count()>0){
            auto now=std::chrono::steady_clock::now();
            float el=std::chrono::duration<float>(now-status_last).count();
            if(el>=1.0f){
                status_last=now;
                v.net_srv->broadcast_status(
                    (float)(v.header.center_frequency/1e6),
                    v.gain_db, v.header.sample_rate,
                    (v.hw.type==HWType::RTLSDR)?1:0);
            }
        }
        // ── HOST: 3초마다 HEARTBEAT 브로드캐스트; SDR 뽑힘 감지 시 즉시 ──────
        if(v.net_srv){
            static bool prev_sdr_err = false;
            bool cur_sdr_err = v.sdr_stream_error.load();
            auto now2=std::chrono::steady_clock::now();
            float elh=std::chrono::duration<float>(now2-heartbeat_last).count();
            bool sdr_err_changed = (cur_sdr_err != prev_sdr_err);
            if(elh>=3.0f || sdr_err_changed){
                if(sdr_err_changed) prev_sdr_err = cur_sdr_err;
                heartbeat_last=now2;
                uint8_t sdr_t_hb = 0;
                if(v.dev_blade){
                    float _t = 0.f;
                    if(bladerf_get_rfic_temperature(v.dev_blade, &_t) == 0)
                        sdr_t_hb = (uint8_t)std::min(255.f, std::max(0.f, _t));
                }
                // host_state: 0=OK, 2=SPECTRUM_PAUSED (JOIN에게 노란 LINK 표시)
                uint8_t hst = (v.spectrum_pause.load() || !v.render_visible.load()) ? 2 : 0;
                // sdr_state: 0=OK, 1=stream error/rx stopped (SDR 뽑힘/초기화 실패/의도적 정지)
                uint8_t sdr_st = (cur_sdr_err || v.rx_stopped.load()) ? 1 : 0;
                // iq_on: HOST IQ 롤링 상태
                uint8_t iq_st = v.tm_iq_on.load() ? 1 : 0;
                v.net_srv->broadcast_heartbeat(hst, sdr_t_hb, sdr_st, iq_st);
            }
        }

        // ── JOIN→HOST chassis 명령: 네트워크 스레드 플래그 → 메인 루프 처리 ────
        // HOST 직접 입력과 완전히 동일한 경로로 실행 (race condition 방지)
        if(v.net_srv && pending_chassis1_reset.load()){
            pending_chassis1_reset.store(false);
            { std::lock_guard<std::mutex> lk(host_chat_mtx);
              LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
              strncpy(lm.msg,"Chassis 1 reset ...",255);
              if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
              host_chat_log.push_back(lm); }
            v.net_srv->broadcast_chat("BEWE", "Chassis 1 reset ...");
            v.net_srv->broadcast_heartbeat(1);
            v.is_running = false;
            v.sdr_stream_error.store(true);
            v.tm_iq_on.store(false);
            v.spectrum_pause.store(true);
            usb_reset_pending = true;
        }
        if(v.net_srv && pending_chassis2_reset.load()){
            pending_chassis2_reset.store(false);
            // 1) 채팅 로그
            { std::lock_guard<std::mutex> lk(host_chat_mtx);
              LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
              strncpy(lm.msg,"Chassis 2 reset ...",255);
              if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
              host_chat_log.push_back(lm); }
            // 2) JOIN 클라이언트에게 리셋 알림
            v.net_srv->broadcast_chat("BEWE", "Chassis 2 reset ...");
            v.net_srv->broadcast_heartbeat(2);
            // 3) 브로드캐스트 중단 + 송신 큐 flush
            v.net_bcast_pause.store(true, std::memory_order_relaxed);
            v.net_srv->pause_broadcast();
            v.net_srv->flush_clients();
            // 4) Central Server에 NET_RESET 전송 (지구본 마커 사라짐)
            if(central_cli.is_central_connected())
                central_cli.send_net_reset(0);  // 0 = reset start
            // 5) 1초 후 재개
            NetServer* srv_ptr = v.net_srv;
            std::atomic<bool>* bcast_pause_ptr = &v.net_bcast_pause;
            std::mutex* log_mtx_ptr2 = &host_chat_mtx;
            std::vector<LocalChatMsg>* log_ptr2 = &host_chat_log;
            CentralClient* central_ptr = &central_cli;
            FFTViewer* vp = &v;
            std::string central_host_str = s_central_host;
            int central_port_val = s_central_port;
            std::thread([srv_ptr, bcast_pause_ptr, log_mtx_ptr2, log_ptr2,
                         central_ptr, vp, central_host_str, central_port_val](){
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // 재개 전 한번 더 큐 flush (1초간 쌓인 잔여)
                srv_ptr->flush_clients();
                srv_ptr->resume_broadcast();
                bcast_pause_ptr->store(false, std::memory_order_relaxed);
                srv_ptr->broadcast_heartbeat(0);
                srv_ptr->broadcast_chat("BEWE", "Chassis 2 stable ...");
                // relay가 끊겨있으면 재연결
                if(!central_ptr->is_central_connected() && !central_host_str.empty()){
                    central_ptr->stop_mux_adapter();
                    std::string sid = vp->station_name + "_" + std::string(login_get_id());
                    int rfd = central_ptr->open_room(
                        central_host_str, central_port_val,
                        sid, vp->station_name,
                        vp->station_lat, vp->station_lon,
                        (uint8_t)login_get_tier());
                    if(rfd >= 0){
                        central_ptr->start_mux_adapter(rfd,
                            [vp](int local_fd){ if(vp->net_srv) vp->net_srv->inject_fd(local_fd); },
                            [vp](){ return vp->net_srv ? (uint8_t)vp->net_srv->client_count() : (uint8_t)0; });
                        printf("[UI] Central reconnected after chassis 2 reset\n");
                    } else {
                        printf("[UI] Central reconnect failed after chassis 2 reset\n");
                    }
                } else if(central_ptr->is_central_connected()){
                    central_ptr->send_net_reset(1);  // 1 = open
                }
                std::lock_guard<std::mutex> lk(*log_mtx_ptr2);
                LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
                strncpy(lm.msg,"Chassis 2 stable ...",255);
                if((int)log_ptr2->size()>=200) log_ptr2->erase(log_ptr2->begin());
                log_ptr2->push_back(lm);
            }).detach();
        }

        // ── JOIN→HOST /rx stop/start: 네트워크 스레드 플래그 → 메인 루프 처리 ──
        if(v.net_srv && pending_rx_stop.load()){
            pending_rx_stop.store(false);
            if(!v.rx_stopped.load() && (v.is_running || cap.joinable())){
                { std::lock_guard<std::mutex> lk(host_chat_mtx);
                  LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
                  strncpy(lm.msg,"RX stop (remote)",255);
                  if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                  host_chat_log.push_back(lm); }
                v.net_srv->broadcast_chat("BEWE", "RX stop");
                // 녹음/demod/TM 중지
                if(v.rec_on.load()) v.stop_rec();
                if(v.tm_iq_on.load()){ v.tm_iq_on.store(false); v.tm_iq_close(); }
                v.stop_all_dem();
                // 캡처 스레드 종료
                v.is_running = false;
                if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
                v.mix_stop.store(true);
                if(v.mix_thr.joinable()) v.mix_thr.join();
                if(cap.joinable()) cap.join();
                // FFTW 정리
                if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                // 디바이스 close
                if(v.dev_blade){
                    bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
                    bladerf_close(v.dev_blade); v.dev_blade=nullptr;
                }
                if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                v.rx_stopped.store(true);
                v.sdr_stream_error.store(false);
                v.spectrum_pause.store(false);
                { std::lock_guard<std::mutex> lk(host_chat_mtx);
                  LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
                  strncpy(lm.msg,"RX stopped.",255);
                  if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                  host_chat_log.push_back(lm); }
            }
        }
        if(v.net_srv && pending_rx_start.load()){
            pending_rx_start.store(false);
            if(v.rx_stopped.load()){
                { std::lock_guard<std::mutex> lk(host_chat_mtx);
                  LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
                  strncpy(lm.msg,"RX start (remote)",255);
                  if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                  host_chat_log.push_back(lm); }
                v.rx_stopped.store(false);
                float cur_cf = (float)(v.header.center_frequency / 1e6);
                if(cur_cf < 0.1f) cur_cf = 100.f;
                v.is_running = true;
                if(v.initialize(cur_cf)){
                    v.set_gain(v.gain_db);
                    if(v.hw.type == HWType::BLADERF)
                        cap = std::thread(&FFTViewer::capture_and_process, &v);
                    else
                        cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                    v.mix_stop.store(false);
                    v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);
                    v.net_srv->broadcast_chat("BEWE", "RX start");
                    { std::lock_guard<std::mutex> lk(host_chat_mtx);
                      LocalChatMsg lm{}; strncpy(lm.from,"BEWE",31);
                      strncpy(lm.msg,"RX started.",255);
                      if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                      host_chat_log.push_back(lm); }
                } else {
                    v.is_running = false;
                    v.rx_stopped.store(true);
                    { std::lock_guard<std::mutex> lk(host_chat_mtx);
                      LocalChatMsg lm{}; lm.is_error=true; strncpy(lm.from,"System",31);
                      strncpy(lm.msg,"RX start failed — SDR not found.",255);
                      if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                      host_chat_log.push_back(lm); }
                }
            }
        }

        // ── chassis 1 reset 후 스펙트럼 pause 자동 해제 (1초 딜레이) ──────────
        static float chassis_unpause_timer = -1.f;
        if(chassis_unpause_timer > 0.f){
            chassis_unpause_timer -= ImGui::GetIO().DeltaTime;
            if(chassis_unpause_timer <= 0.f){
                chassis_unpause_timer = -1.f;
                v.spectrum_pause.store(false);
                printf("[UI] chassis 1 reset: spectrum_pause released\n");
                if(v.net_srv) v.net_srv->broadcast_heartbeat(0, 0, 0);
            }
        }

        // ── LOCAL/HOST: SDR 뽑힘 감지 → 백그라운드 join + 주기적 재탐지 ──────
        // /rx stop으로 의도적 중단 시 자동 재연결 하지 않음
        if(!v.remote_mode && v.sdr_stream_error.load() && !v.rx_stopped.load()){
            // cap 스레드 종료를 백그라운드에서 대기 (메인 렌더 스레드 블로킹 방지)
            static bool     bg_join_started = false;
            static std::atomic<bool> cap_joined{false};
            static bool     usb_reset_done = false;
            static std::atomic<bool> usb_reset_in_progress{false};
            // BladeRF IO 오류 시 자동으로 USB reset 트리거 (chassis reset 명령 없이도)
            if(!bg_join_started && v.hw.type == HWType::BLADERF)
                usb_reset_pending = true;
            if(!bg_join_started && cap.joinable()){
                bg_join_started = true;
                cap_joined.store(false);
                usb_reset_done = false;
                std::thread([&cap, &cap_joined](){
                    if(cap.joinable()) cap.join();
                    // bladerf_close 후 libusb 내부 event thread가 transfer callback을
                    // 완전히 정리할 때까지 대기 (너무 빨리 재open하면 "out of order" crash)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                    cap_joined.store(true);
                }).detach();
            } else if(!cap.joinable()){
                cap_joined.store(true);
            }

            // cap 종료 확인 후 USB reset (chassis reset 요청 시 한 번만)
            if(cap_joined.load() && usb_reset_pending && !usb_reset_done){
                usb_reset_done = true;
                usb_reset_pending = false;
                usb_reset_in_progress.store(true);
                // capture_and_process 종료 시 dev_blade가 nullptr로 세팅됨
                // (혹시 남아있으면 닫기)
                if(v.dev_blade){
                    bladerf_close(v.dev_blade);
                    v.dev_blade = nullptr;
                }
                std::thread([&usb_reset_in_progress = usb_reset_in_progress](){
                    printf("[UI] chassis 1 reset: USB reset BladeRF...\n");
                    bladerf_usb_reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                    printf("[UI] USB re-enumeration wait done\n");
                    usb_reset_in_progress.store(false);
                }).detach();
            }

            static float sdr_retry_timer = 0.f;
            sdr_retry_timer -= ImGui::GetIO().DeltaTime;
            // USB reset 진행 중에는 재시도 타이머 리셋 (완료 후 즉시 시도)
            if(usb_reset_in_progress.load()) sdr_retry_timer = 1.f;
            if(sdr_retry_timer <= 0.f && cap_joined.load() && !usb_reset_in_progress.load()){
                sdr_retry_timer = 2.f;
                // 이전 FFTW 리소스 정리
                if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                // SDR 재탐지: 현재 설정(주파수) 기반으로 재초기화
                float cur_cf = (float)(v.header.center_frequency / 1e6);
                if(cur_cf < 0.1f) cur_cf = 100.f;
                v.is_running = true; // 새 캡처 스레드를 위해 복구
                if(v.initialize(cur_cf)){
                    printf("SDR reconnected — resuming at %.2f MHz\n", cur_cf);
                    v.sdr_stream_error.store(false);
                    bg_join_started = false;  // 다음 뽑힘을 위해 리셋
                    cap_joined.store(false);
                    usb_reset_in_progress.store(false);
                    // 이전 게인 복원
                    v.set_gain(v.gain_db);
                    // 캡처 스레드 재시작
                    if(v.hw.type == HWType::BLADERF)
                        cap = std::thread(&FFTViewer::capture_and_process, &v);
                    else
                        cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                    // chassis reset으로 pause 걸린 경우: 1초 후 자동 해제
                    if(v.spectrum_pause.load())
                        chassis_unpause_timer = 1.f;
                    // HOST면 즉시 heartbeat로 JOIN에게 SDR 복구 알림 (pause 상태 포함)
                    if(v.net_srv){
                        uint8_t hst = v.spectrum_pause.load() ? 2 : 0;
                        v.net_srv->broadcast_heartbeat(hst, 0, 0);
                    }
                } else {
                    v.is_running = false; // initialize 실패 시 다시 false
                }
            }
        }

        // ── CONNECT 모드: 연결 끊김 감지 → 자동 재연결 (백그라운드) ────────
        if(v.remote_mode && v.net_cli && !v.net_cli->is_connected()){
            static float reconn_timer = 0.f;
            static std::atomic<bool> reconn_busy{false};
            reconn_timer -= ImGui::GetIO().DeltaTime;
            if(reconn_timer <= 0.f && !reconn_busy.load()){
                reconn_timer = 3.f;
                reconn_busy.store(true);
                // 재연결을 백그라운드 스레드에서 수행 → UI 블로킹 방지
                auto* cli_ptr = v.net_cli;
                auto* v_ptr   = &v;
                auto* central_ptr = &central_cli;
                std::thread([cli_ptr, v_ptr, central_ptr,
                             central_host  = s_central_host,
                             central_port  = s_central_port,
                             station_id  = s_central_join_station_id,
                             c_host = std::string(connect_host),
                             c_port = connect_port,
                             c_id   = std::string(connect_id),
                             c_pw   = std::string(connect_pw),
                             c_tier = connect_tier](){
                    bool ok = false;
                    if(!station_id.empty()){
                        int rfd = central_ptr->join_room(central_host, central_port, station_id);
                        if(rfd >= 0){
                            ok = cli_ptr->connect_fd(rfd, c_id.c_str(), c_pw.c_str(), c_tier);
                            if(!ok) close(rfd);
                        }
                    } else if(!c_host.empty()){
                        ok = cli_ptr->connect(c_host.c_str(), c_port,
                                              c_id.c_str(), c_pw.c_str(), c_tier);
                    }
                    if(ok){
                        for(int ci=0;ci<MAX_CHANNELS;ci++)
                            if(v_ptr->local_ch_out[ci]==3) cli_ptr->cmd_toggle_recv(ci,false);
                        v_ptr->autoscale_active=true; v_ptr->autoscale_init=false;
                        v_ptr->autoscale_accum.clear();
                        v_ptr->join_manual_scale=false;
                        { std::lock_guard<std::mutex> wlk(v_ptr->wf_events_mtx);
                          v_ptr->wf_events.clear(); }
                        v_ptr->last_tagged_sec = -1;
                    }
                    reconn_busy.store(false);
                }).detach();
            }
        }

        // ── CONNECT 모드: STATUS → gain/hw 동기화 + 주파수 변화 감지 ────────
        if(v.remote_mode && v.net_cli && v.net_cli->is_connected()){
            v.gain_db = v.net_cli->remote_gain_db.load();
            uint8_t hwt = v.net_cli->remote_hw.load();
            if(hwt == 1){ v.hw.gain_min=0.f;   v.hw.gain_max=49.6f; }
            else         { v.hw.gain_min=-12.f; v.hw.gain_max=60.f;  }
            // 주파수 변화 감지 → 오토스케일 트리거
            static float last_cf_mhz = 0.f;
            float cur_cf = v.net_cli->remote_cf_mhz.load();
            if(cur_cf > 0.f && fabsf(cur_cf - last_cf_mhz) > 0.001f){
                v.autoscale_active  = true;
                v.autoscale_init    = false;
                v.autoscale_accum.clear();
                v.join_manual_scale = false;
            }
            last_cf_mhz = cur_cf;
        }

        // ── CONNECT 모드: 수신 FFT → waterfall 업데이트 (1초 버퍼) ─────────
        // 수신된 프레임은 1초 지연 후 표시 → 끊김 없는 스크롤 보장
        if(v.remote_mode && v.net_cli && !v.spectrum_pause.load()){
            NetClient::FftFrame frm;
            // 한 프레임 루프 반복 (버퍼에 쌓인 모든 준비 프레임 소화)
            while(v.net_cli->pop_fft_frame(frm)){
                last_fft_seq++;
                int fsz = (int)frm.fft_sz;
                if(fsz <= 0 || (int)frm.data.size() != fsz) continue;

                // FFT 크기 변경 시 재초기화
                if(fsz != v.fft_size){
                    v.fft_size = fsz;
                    v.fft_input_size = fsz / FFT_PAD_FACTOR;
                    v.header.fft_size = (uint32_t)fsz;
                    v.fft_data.assign((size_t)MAX_FFTS_MEMORY * fsz, 0);
                    v.current_spectrum.assign(fsz, -80.f);
                    v.texture_needs_recreate = true;
                }
                v.header.center_frequency = frm.cf_hz;
                v.header.sample_rate      = frm.sr;
                v.header.power_min        = frm.pmin;
                v.header.power_max        = frm.pmax;
                if(!v.autoscale_active && !v.join_manual_scale){
                    v.display_power_min = frm.pmin;
                    v.display_power_max = frm.pmax;
                }
                {
                    std::lock_guard<std::mutex> dlk(v.data_mtx);
                    int fi = v.total_ffts % MAX_FFTS_MEMORY;
                    float* dst = v.fft_data.data() + (size_t)fi * fsz;
                    memcpy(dst, frm.data.data(), fsz * sizeof(float));
                    v.total_ffts++;
                    v.current_fft_idx = v.total_ffts - 1;
                    // wall_time 기반 워터폴 시간태그
                    if(frm.wall_time > 0){
                        time_t wt = (time_t)frm.wall_time;
                        struct tm* tt = localtime(&wt);
                        int cur5 = tt->tm_hour*720+tt->tm_min*12+tt->tm_sec/5;
                        if(cur5 != v.last_tagged_sec){
                            v.last_tagged_sec = cur5;
                            FFTViewer::WfEvent wev{};
                            wev.fft_idx  = v.current_fft_idx;
                            wev.wall_time= wt;
                            wev.type     = 0;
                            strftime(wev.label,sizeof(wev.label),"%H:%M:%S",tt);
                            std::lock_guard<std::mutex> wlk(v.wf_events_mtx);
                            v.wf_events.push_back(wev);
                            int cutoff=v.current_fft_idx-MAX_FFTS_MEMORY;
                            v.wf_events.erase(std::remove_if(v.wf_events.begin(),v.wf_events.end(),
                                [&](const FFTViewer::WfEvent& e){return e.fft_idx<cutoff;}),
                                v.wf_events.end());
                        }
                    }
                    // 오토스케일 누적
                    if(v.autoscale_active){
                        if(!v.autoscale_init){
                            v.autoscale_accum.reserve(fsz*200);
                            v.autoscale_last = std::chrono::steady_clock::now();
                            v.autoscale_init = true;
                        }
                        for(int _i=1;_i<fsz;_i++){
                            v.autoscale_accum.push_back(dst[_i]);
                        }
                        float _el=std::chrono::duration<float>(
                            std::chrono::steady_clock::now()-v.autoscale_last).count();
                        if(_el>=1.0f && !v.autoscale_accum.empty()){
                            size_t _idx=(size_t)(v.autoscale_accum.size()*0.15f);
                            std::nth_element(v.autoscale_accum.begin(),
                                v.autoscale_accum.begin()+_idx,
                                v.autoscale_accum.end());
                            v.display_power_min = v.autoscale_accum[_idx] - 10.f;
                            v.display_power_max = v.display_power_min + 60.f;
                            v.autoscale_accum.clear();
                            v.autoscale_active = false;
                            v.cached_sp_idx = -1;
                        }
                    }
                    v.header.num_ffts = std::min(v.total_ffts, MAX_FFTS_MEMORY);
                }
                v.update_wf_row(v.current_fft_idx);
            }
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        // TM 모드: freeze_idx는 스페이스바 진입 시점에만 세팅 (매 프레임 갱신 금지)
        // Ctrl+휠로 tm_offset 변경 시 tm_update_display()가 display_idx를 재계산함
        if(v.texture_needs_recreate){ v.texture_needs_recreate=false; v.create_waterfall_texture(); }

        ImGuiIO& io=ImGui::GetIO();
        bool editing=ImGui::IsAnyItemActive();
        int sci=v.selected_ch;

        // ── Keyboard shortcuts ────────────────────────────────────────────
        if(!editing){
            if(ImGui::IsKeyPressed(ImGuiKey_R,false)){
                if(v.remote_mode && v.net_cli){
                    if(v.region.active){
                        // R키 누르는 시점에 time_start/end 재계산 (정확도 최대화)
                        {
                            // wall_time 기반 재계산 (JOIN 포함 정확)
                            time_t wt_top = v.fft_idx_to_wall_time(v.region.fft_top);
                            time_t wt_bot = v.fft_idx_to_wall_time(v.region.fft_bot);
                            if(wt_top > 0 && wt_bot > 0){
                                v.region.time_end   = wt_top;
                                v.region.time_start = wt_bot;
                            } else {
                                float rps_r=(float)v.header.sample_rate/(float)v.fft_input_size/(float)v.time_average;
                                if(rps_r<=0) rps_r=37.5f;
                                time_t now_r=time(nullptr);
                                v.region.time_end  =now_r-(time_t)((v.current_fft_idx-v.region.fft_top)/rps_r);
                                v.region.time_start=now_r-(time_t)((v.current_fft_idx-v.region.fft_bot)/rps_r);
                            }
                        }
                        // JOIN: 영역 IQ 녹음 요청 → HOST가 즉시 처리+전송
                        v.net_cli->cmd_request_region(
                            v.region.fft_top, v.region.fft_bot,
                            v.region.freq_lo, v.region.freq_hi,
                            (int32_t)v.region.time_start, (int32_t)v.region.time_end);
                        v.region.active = false;
                        {
                            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                            FFTViewer::RecEntry e{};
                            time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
                            char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
                            float cf_mhz = (v.region.freq_lo + v.region.freq_hi) / 2.0f;
                            char fn[128]; snprintf(fn,sizeof(fn),"IQ_%.3fMHz_%s.wav",cf_mhz,dts);
                            e.filename = fn;
                            e.is_region = true;
                            e.req_state = FFTViewer::RecEntry::REQ_CONFIRMED;
                            e.t_start = std::chrono::steady_clock::now();
                            v.rec_entries.push_back(e);
                        }
                    } else {
                        // JOIN: 채널 선택 시 → 로컬 오디오 녹음 (채널필터에서 IQ 녹음 없음)
                        int sci2 = v.selected_ch;
                        if(sci2>=0 && v.channels[sci2].mode != Channel::DM_NONE){
                            if(v.channels[sci2].audio_rec_on.load())
                                v.stop_join_audio_rec(sci2);
                            else
                                v.start_join_audio_rec(sci2);
                        }
                    }
                } else if(v.region.active){
                    v.region_save();
                } else {
                    // LOCAL/HOST: 채널 선택 시 Audio REC; 채널 없을 때만 IQ REC
                    int sci2 = v.selected_ch;
                    bool ch_demod = (sci2>=0 && v.channels[sci2].dem_run.load());
                    if(ch_demod){
                        if(v.channels[sci2].audio_rec_on.load())
                            v.stop_audio_rec(sci2);
                        else
                            v.start_audio_rec(sci2);
                    } else {
                        // 채널 선택 안 됐거나 복조 안 중일 때만 IQ REC
                        bool any_ch = false;
                        for(int i=0;i<MAX_CHANNELS;i++) if(v.channels[i].filter_active){any_ch=true;break;}
                        if(!any_ch){
                            if(v.rec_on.load()) v.stop_rec();
                            else if(v.tm_active.load()) v.tm_rec_start();
                            else v.start_rec();
                        }
                    }
                }
            }

            if(ImGui::IsKeyPressed(ImGuiKey_P,false)){
                bool np = !v.spectrum_pause.load();
                // JOIN이든 HOST든 로컬 FFT 표시만 토글 (채널 복조 스트리밍과 무관)
                v.spectrum_pause.store(np);
                if(v.net_srv){
                    v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    // 즉시 heartbeat: JOIN에게 pause 상태 즉시 반영
                    v.net_srv->broadcast_heartbeat(np ? 2 : 0);
                    heartbeat_last = std::chrono::steady_clock::now();
                }
            }
            // T키: IQ 롤링 버퍼 활성/비활성 (JOIN이면 HOST에 CMD 전송)
            if(ImGui::IsKeyPressed(ImGuiKey_T,false)){
                if(v.remote_mode && v.net_cli){
                    v.net_cli->cmd_toggle_tm_iq();
                } else {
                    bool cur=v.tm_iq_on.load();
                    if(cur){
                        v.tm_iq_on.store(false);
                        v.tm_add_event_tag(2);
                        v.tm_iq_was_stopped=true;
                        if(v.net_srv) v.net_srv->broadcast_wf_event(0,(int64_t)time(nullptr),2,"IQ Stop");
                    } else {
                        if(v.tm_iq_was_stopped){ v.tm_iq_close(); v.tm_iq_was_stopped=false; }
                        v.tm_iq_open();
                        if(v.tm_iq_file_ready){
                            v.tm_iq_on.store(true);
                            v.tm_add_event_tag(1);
                            if(v.net_srv) v.net_srv->broadcast_wf_event(0,(int64_t)time(nullptr),1,"IQ Start");
                        }
                    }
                }
            }
            // 스페이스바: TM 토글 (진입/해제)
            if(ImGui::IsKeyPressed(ImGuiKey_Space,false)){
                if(v.tm_active.load()){
                    // 해제: 라이브 복귀
                    v.tm_offset=0.0f;
                    v.tm_active.store(false);
                } else {
                    // 진입: 현재 시점을 freeze 기준으로
                    v.tm_freeze_idx=v.current_fft_idx;
                    v.tm_display_fft_idx=v.current_fft_idx;
                    v.tm_offset=0.0f;
                    v.tm_active.store(true);
                }
            }
            if(sci>=0&&v.channels[sci].filter_active){
                auto set_mode=[&](Channel::DemodMode m){
                    if(v.remote_mode && v.net_cli){
                        // CONNECT 모드: 서버에 CMD 전송
                        int cur=(int)v.channels[sci].mode;
                        int nm=(v.channels[sci].mode==m)?0:(int)m;
                        v.net_cli->cmd_set_ch_mode(sci, nm);
                    } else {
                        Channel& ch=v.channels[sci];
                        if(ch.dem_run.load()&&ch.mode==m){ v.stop_dem(sci); }
                        else { v.stop_dem(sci); v.start_dem(sci,m); }
                        // HOST 모드: 채널 sync 브로드캐스트
                        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    }
                };
                if(ImGui::IsKeyPressed(ImGuiKey_A,false)) set_mode(Channel::DM_AM);
                if(ImGui::IsKeyPressed(ImGuiKey_F,false)) set_mode(Channel::DM_FM);
                if(ImGui::IsKeyPressed(ImGuiKey_D,false)){
                    // D키: 선택된 채널의 디지털 버튼 패널 show/hide 토글
                    v.digi_panel_on[sci] = !v.digi_panel_on[sci];
                    // 패널 닫으면 실행 중인 디지털 복조도 정지
                    if(!v.digi_panel_on[sci] && v.channels[sci].digi_run.load())
                        v.stop_digi(sci);
                }
                if(ImGui::IsKeyPressed(ImGuiKey_M,false)){
                    if(v.remote_mode && v.net_cli){
                        int nm=(v.channels[sci].mode==Channel::DM_MAGIC)?0:(int)Channel::DM_MAGIC;
                        v.net_cli->cmd_set_ch_mode(sci, nm);
                    } else {
                        Channel& ch=v.channels[sci];
                        if(ch.dem_run.load()&&ch.mode==Channel::DM_MAGIC){ v.stop_dem(sci); }
                        else {
                            v.stop_dem(sci);
                            ch.magic_det.store(0,std::memory_order_relaxed);
                            v.start_dem(sci,Channel::DM_MAGIC);
                        }
                        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    }
                }
                // ── 방향키: 로컬 오디오 출력 전환 (L/R/L+R/M) ──────────
                auto arrow_set_out = [&](int ci, int lco){
                    int prev=v.local_ch_out[ci]; v.local_ch_out[ci]=lco;
                    if(v.net_cli && v.remote_mode){
                        bool now_mute=(lco==3), was_mute=(prev==3);
                        if(now_mute&&!was_mute) v.net_cli->cmd_toggle_recv(ci,false);
                        else if(!now_mute&&was_mute) v.net_cli->cmd_toggle_recv(ci,true);
                    }
                    if(v.net_srv){
                        uint32_t mask=v.channels[ci].audio_mask.load();
                        if(lco==3) mask&=~0x1u; else mask|=0x1u;
                        v.channels[ci].audio_mask.store(mask);
                        v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    }
                };
                if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow,false))  arrow_set_out(sci, 0); // L
                if(ImGui::IsKeyPressed(ImGuiKey_RightArrow,false)) arrow_set_out(sci, 2); // R
                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow,false))    arrow_set_out(sci, 1); // L+R
                if(ImGui::IsKeyPressed(ImGuiKey_DownArrow,false))  arrow_set_out(sci, 3); // M(뮤트)
            }
            if(ImGui::IsKeyPressed(ImGuiKey_O,false) && !editing){
                ops_open = !ops_open;
            }
            if(ImGui::IsKeyPressed(ImGuiKey_Escape,false)){
                if(sci>=0){ v.channels[sci].selected=false; v.selected_ch=-1; }
            }
            if(ImGui::IsKeyPressed(ImGuiKey_Delete,false)){
                if(sci>=0&&v.channels[sci].filter_active){
                    if(v.remote_mode && v.net_cli){
                        v.net_cli->cmd_delete_ch(sci);
                    } else {
                        v.stop_dem(sci); v.stop_digi(sci); v.digi_panel_on[sci]=false;
                        v.channels[sci].filter_active=false;
                        v.channels[sci].selected=false;
                        v.channels[sci].mode=Channel::DM_NONE;
                        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    }
                    v.selected_ch=-1;
                } else if(file_ctx.selected && !file_ctx.filepath.empty()){
                    // 선택된 파일 DEL 키로 삭제
                    bool can_delete = true;
                    if(file_ctx.is_public){
                        auto it = pub_owners.find(file_ctx.filename);
                        if(it != pub_owners.end()){
                            const char* my_id = login_get_id();
                            can_delete = (it->second == std::string(my_id));
                        }
                    }
                    if(can_delete){
                        remove(file_ctx.filepath.c_str());
                        auto rm_from = [&](std::vector<std::string>& v2){
                            v2.erase(std::remove(v2.begin(),v2.end(),file_ctx.filename),v2.end());
                        };
                        rm_from(rec_iq_files); rm_from(rec_audio_files);
                        rm_from(priv_iq_files); rm_from(priv_audio_files);
                        rm_from(pub_iq_files); rm_from(pub_audio_files);
                        rm_from(share_iq_files); rm_from(share_audio_files);
                        rm_from(priv_files); rm_from(shared_files); rm_from(downloaded_files);
                        priv_extra_paths.erase(file_ctx.filename);
                        pub_owners.erase(file_ctx.filename);
                        pub_listeners.erase(file_ctx.filename);
                        {
                            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                            v.rec_entries.erase(std::remove_if(v.rec_entries.begin(),v.rec_entries.end(),
                                [&](const FFTViewer::RecEntry& e){return e.path==file_ctx.filepath;}),
                                v.rec_entries.end());
                        }
                        if(v.net_srv && file_ctx.is_public){
                            std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
                            for(auto& sf : shared_files){
                                bool siq = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
                                std::string sfp = (siq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
                                struct stat sst{}; uint64_t fsz=0;
                                if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
                                std::string upl;
                                auto oit=pub_owners.find(sf); if(oit!=pub_owners.end()) upl=oit->second;
                                slist.push_back({sf,fsz,upl});
                            }
                            v.net_srv->send_share_list(-1, slist);
                        }
                        file_ctx.selected=false; file_ctx.filepath=""; file_ctx.filename="";
                    }
                }
            }
        }

        // ── 우측 패널 토글 (S키) ──────────────────────────────────────────────
        // saved_ratio: 사용자가 마지막으로 설정한 패널 너비 (0=초기상태 미설정)
        static float right_panel_saved_ratio = 0.0f;
        if(ImGui::IsKeyPressed(ImGuiKey_S, false) && !ImGui::GetIO().WantTextInput){
            if(v.right_panel_ratio > 0.01f){
                // 열려있음 → 저장 후 닫기
                right_panel_saved_ratio = v.right_panel_ratio;
                v.right_panel_ratio = 0.0f;
            } else {
                // 닫혀있음 → 마지막 저장값으로 열기 (미설정시 기본값 0.3)
                v.right_panel_ratio = (right_panel_saved_ratio > 0.01f) ? right_panel_saved_ratio : 0.3f;
            }
        }

        // ── 채팅창 토글 / 빠른 명령 입력 (항상 우선 처리, editing 무관) ─────
        if(ImGui::IsKeyPressed(ImGuiKey_RightShift, false) && !ImGui::GetIO().WantTextInput){
            chat_open = !chat_open;
        }
        if(ImGui::IsKeyPressed(ImGuiKey_Slash, false) && !ImGui::GetIO().WantTextInput){
            if(!chat_open){ chat_open = true; }
            chat_input[0] = '/'; chat_input[1] = '\0';
            chat_focus_input = true;
            chat_cursor_end  = true;
        }

        // ── Main window ───────────────────────────────────────────────────
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(0,0));
        ImGui::SetNextWindowPos(ImVec2(0,0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("##main",nullptr,
                     ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|
                     ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoTitleBar|
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::PopStyleVar();

        ImDrawList* dl=ImGui::GetWindowDrawList();
        float disp_w=io.DisplaySize.x, disp_h=io.DisplaySize.y;

        dl->AddRectFilled(ImVec2(0,0),ImVec2(disp_w,TOPBAR_H),IM_COL32(30,30,30,255));
        ImGui::SetCursorPos(ImVec2(6,6));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,ImVec2(8,4));

        // ── Frequency input ───────────────────────────────────────────────
        static float new_freq=450.0f; static bool fdeact=false, focus_freq=false;
        // Tab → 주파수 입력창 포커스 (스펙트럼 뷰에서만, 어떤 입력칸도 활성화 안 된 상태)
        if(ImGui::IsKeyPressed(ImGuiKey_Tab, false) && !editing){
            focus_freq = true;
        }
        // 채팅창 열린 상태에서 Enter → 채팅 입력칸 포커스
        if(chat_open &&
           (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) &&
           !ImGui::GetIO().WantTextInput){
            chat_focus_input = true;
        }
        if(fdeact){ fdeact=false; ImGui::SetWindowFocus(nullptr); }
        if(focus_freq){ ImGui::SetKeyboardFocusHere(); focus_freq=false; }
        {
            // 원격 주파수 동기화 (편집 중이 아닐 때)
            if(!ImGui::IsItemActive()){
                if(v.net_cli)
                    new_freq = v.net_cli->remote_cf_mhz.load();
                else if(!v.remote_mode && v.header.center_frequency > 0)
                    new_freq = (float)(v.header.center_frequency / 1e6);
            }
            char fbuf[32]; snprintf(fbuf,sizeof(fbuf),"%.3f MHz",new_freq);
            float tw=ImGui::CalcTextSize(fbuf).x;
            float box_w=96.0f;
            float px=std::max(2.0f,(box_w-tw)*0.5f-1.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(px,ImGui::GetStyle().FramePadding.y));
            ImGui::SetNextItemWidth(box_w);
            ImGui::InputFloat("##freq",&new_freq,0,0,"%.3f MHz");
            ImGui::PopStyleVar();
        }
        if(ImGui::IsItemDeactivatedAfterEdit()){
            if(v.remote_mode && v.net_cli){
                v.net_cli->cmd_set_freq(new_freq);
                v.net_cli->cmd_set_autoscale();
            } else {
                v.pending_cf=new_freq; v.freq_req=true;
                v.autoscale_active=true; v.autoscale_init=false;
                v.autoscale_accum.clear();
            }
            fdeact=true;
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Center Frequency  [Enter] to edit");
        ImGui::SameLine();

        // ── FFT size combo ────────────────────────────────────────────────
        static const int fft_sizes[]={512,1024,2048,4096,8192,16384};
        static const char* fft_lbls[]={"512","1024","2048","4096","8192","16384"};
        static int fft_si=4;
        {
            float tw2=ImGui::CalcTextSize(fft_lbls[fft_si]).x;
            float box_w2=72.0f;
            float px2=std::max(2.0f,(box_w2-tw2)*0.5f-12.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(px2,ImGui::GetStyle().FramePadding.y));
            ImGui::SetNextItemWidth(box_w2);
            // JOIN: HOST fft_input_size로 콤보 동기화
            if(v.remote_mode){
                for(int i=0;i<6;i++) if(fft_sizes[i]==v.fft_input_size){ fft_si=i; break; }
            }
            if(ImGui::BeginCombo("##fftsize",fft_lbls[fft_si],ImGuiComboFlags_HeightSmall)){
                for(int i=0;i<6;i++){
                    bool sel2=(fft_si==i);
                    if(ImGui::Selectable(fft_lbls[i],sel2)){
                        if(i != fft_si){
                            fft_si=i;
                            if(v.remote_mode && v.net_cli){
                                v.net_cli->cmd_set_fft_size((uint32_t)fft_sizes[i]);
                            } else {
                                v.pending_fft_size=fft_sizes[i]; v.fft_size_change_req=true;
                            }
                        }
                    }
                    if(sel2) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::PopStyleVar();
        }
        ImGui::SameLine();

        // ── Sample Rate combo (LOCAL/HOST/JOIN 표시) ──────────────────────
        if((v.dev_blade || v.dev_rtl) || v.remote_mode){
            // BladeRF: 2.5/5/10/20/30.72/61.44 MSPS
            // RTL-SDR: 0.25/0.96/1.44/2.56 MSPS
            static const float blade_srs[]  = {2.5f,5.0f,10.0f,20.0f,30.72f,61.44f,122.88f};
            static const char* blade_lbls[] = {"2.5M","5M","10M","20M","30.72M","61.44M","122.88M"};
            static const float rtl_srs[]    = {0.25f,0.96f,1.44f,2.56f};
            static const char* rtl_lbls[]   = {"0.25M","0.96M","1.44M","2.56M"};
            // JOIN: HOST hw_type으로 SR 리스트 결정
            bool use_blade = v.remote_mode ? (v.net_cli && v.net_cli->remote_hw.load()==0)
                                           : (v.dev_blade != nullptr);
            const float* sr_list  = use_blade ? blade_srs  : rtl_srs;
            const char** sr_lbls  = use_blade ? blade_lbls : rtl_lbls;
            int          sr_count = use_blade ? 7 : 4;

            // 현재 SR에 맞는 인덱스 선택
            static int sr_si = -1;
            float cur_sr_msps = v.remote_mode
                ? (v.net_cli ? v.net_cli->remote_sr.load() / 1e6f : 0.f)
                : v.hw.sample_rate_mhz;
            {
                float best_diff = 1e9f;
                for(int i=0;i<sr_count;i++){
                    float d = fabsf(sr_list[i] - cur_sr_msps);
                    if(d < best_diff){ best_diff=d; sr_si=i; }
                }
            }

            const char* cur_lbl = (sr_si>=0 && sr_si<sr_count) ? sr_lbls[sr_si] : "?";
            float tw_sr  = ImGui::CalcTextSize(cur_lbl).x;
            float box_sr = 76.0f;
            float px_sr  = std::max(2.0f,(box_sr-tw_sr)*0.5f-12.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px_sr, ImGui::GetStyle().FramePadding.y));
            ImGui::SetNextItemWidth(box_sr);
            if(ImGui::BeginCombo("##srcombo", cur_lbl, ImGuiComboFlags_HeightSmall)){
                for(int i=0;i<sr_count;i++){
                    bool sel = (sr_si==i);
                    if(ImGui::Selectable(sr_lbls[i], sel)){
                        if(i != sr_si){
                            sr_si = i;
                            if(v.remote_mode && v.net_cli){
                                v.net_cli->cmd_set_sr(sr_list[i]);
                            } else {
                            v.pending_sr_msps = sr_list[i];
                            v.sr_change_req   = true;
                            // autoscale 리셋
                            v.autoscale_active=true; v.autoscale_init=false;
                            v.autoscale_accum.clear();
                            // SR 변경 시 digi 워커 재시작 (새 SR 파라미터로)
                            for(int ci=0;ci<MAX_CHANNELS;ci++){
                                if(v.channels[ci].digi_run.load()){
                                    Channel::DigitalMode dm=v.channels[ci].digital_mode;
                                    v.stop_digi(ci);
                                    // SR 실제 적용은 다음 프레임이므로 지연 재시작은 사용자가 버튼 재클릭
                                }
                            }
                            } // end local/host branch
                        }
                    }
                    if(sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::PopStyleVar();
            ImGui::SameLine();
        }

        // ── Gain Control 슬라이더 ─────────────────────────────────────────
        {
            const float GW=140.0f, GH=14.0f;
            ImVec2 gsp=ImGui::GetCursorScreenPos();
            gsp.y=ImGui::GetWindowPos().y+(TOPBAR_H-GH)/2.0f;
            ImDrawList* gdl=ImGui::GetWindowDrawList();
            const float GMIN=v.hw.gain_min, GMAX=v.hw.gain_max;
            const float GRNG=std::max(0.1f,GMAX-GMIN);
            float gdb=v.gain_db;
            float gt=(gdb-GMIN)/GRNG; gt=gt<0?0:gt>1?1:gt;
            // 트랙
            gdl->AddRectFilled(ImVec2(gsp.x,gsp.y),ImVec2(gsp.x+GW,gsp.y+GH),IM_COL32(40,40,40,255),3);
            // 채워진 부분
            gdl->AddRectFilled(ImVec2(gsp.x,gsp.y),ImVec2(gsp.x+gt*GW,gsp.y+GH),IM_COL32(50,140,255,180),3);
            // 레이블 중앙정렬
            char glbl[20]; snprintf(glbl,sizeof(glbl),"Gain:%.1fdB",gdb);
            ImVec2 gsz=ImGui::CalcTextSize(glbl);
            gdl->AddText(ImVec2(gsp.x+GW/2-gsz.x/2,gsp.y+(GH-gsz.y)/2),IM_COL32(230,230,230,255),glbl);
            ImGui::SetCursorScreenPos(gsp);
            ImGui::InvisibleButton("##gain",ImVec2(GW,GH));
            if(ImGui::IsItemHovered()){
                float wheel=ImGui::GetIO().MouseWheel;
                if(wheel!=0.0f){
                    float step=(v.hw.type==HWType::RTLSDR)?0.5f:1.0f;
                    float ng=v.gain_db+(wheel>0?step:-step);
                    ng=ng<GMIN?GMIN:ng>GMAX?GMAX:ng;
                    if(v.remote_mode && v.net_cli){ v.gain_db=ng; v.net_cli->cmd_set_gain(ng); }
                    else { v.gain_db=ng; v.set_gain(ng); }
                }
                ImGui::SetTooltip("Gain Control  Scroll or drag");
            }
            if(ImGui::IsItemActive()){
                float mx=ImGui::GetIO().MousePos.x;
                float ng=GMIN+((mx-gsp.x)/GW)*GRNG;
                ng=ng<GMIN?GMIN:ng>GMAX?GMAX:ng;
                if(v.remote_mode && v.net_cli){ v.gain_db=ng; v.net_cli->cmd_set_gain(ng); }
                else { v.gain_db=ng; v.set_gain(ng); }
            }
            ImGui::SetCursorScreenPos(ImVec2(gsp.x+GW+6,ImGui::GetCursorScreenPos().y));
        }
        ImGui::SameLine();

        // ── Squelch slider (선택 채널만) ──────────────────────────────────
        if(sci>=0&&v.channels[sci].filter_active){
            Channel& sch=v.channels[sci];
            float sig  =sch.sq_sig .load(std::memory_order_relaxed);
            bool  gopen=sch.sq_gate.load(std::memory_order_relaxed);
            const float SLIDER_W=160.0f, SLIDER_H=14.0f;
            ImVec2 sp=ImGui::GetCursorScreenPos();
            sp.y=ImGui::GetWindowPos().y+(TOPBAR_H-SLIDER_H)/2.0f;
            ImDrawList* bdl=ImGui::GetWindowDrawList();
            const float DB_MIN=-100.0f, DB_MAX=0.0f;
            const float DB_RNG=std::max(1.0f,DB_MAX-DB_MIN);
            float thr_db=sch.sq_threshold.load(std::memory_order_relaxed);
            auto to_x=[&](float db)->float{
                float t=(db-DB_MIN)/DB_RNG; t=t<0?0:t>1?1:t;
                return sp.x+t*SLIDER_W;
            };
            bdl->AddRectFilled(ImVec2(sp.x,sp.y),ImVec2(sp.x+SLIDER_W,sp.y+SLIDER_H),IM_COL32(40,40,40,255),3);
            float sw=to_x(sig)-sp.x;
            if(sw>0){
                ImU32 sc=gopen?IM_COL32(60,220,60,200):IM_COL32(40,110,40,150);
                bdl->AddRectFilled(ImVec2(sp.x,sp.y),ImVec2(sp.x+sw,sp.y+SLIDER_H),sc,3);
            }
            float tx=to_x(thr_db);
            bdl->AddLine(ImVec2(tx,sp.y),ImVec2(tx,sp.y+SLIDER_H),IM_COL32(255,220,0,230),2.5f);
            char lbl[20]; snprintf(lbl,sizeof(lbl),"SQL:%.0fdB",thr_db);
            ImVec2 lsz=ImGui::CalcTextSize(lbl);
            bdl->AddText(ImVec2(sp.x+SLIDER_W/2-lsz.x/2,sp.y+(SLIDER_H-lsz.y)/2),IM_COL32(230,230,230,255),lbl);
            ImGui::SetCursorScreenPos(sp);
            ImGui::InvisibleButton("##sql",ImVec2(SLIDER_W,SLIDER_H));
            if(ImGui::IsItemHovered()){
                float wheel=ImGui::GetIO().MouseWheel;
                if(wheel!=0.0f){
                    float nthr=thr_db+(wheel>0?3.0f:-3.0f);
                    nthr=nthr<DB_MIN?DB_MIN:nthr>DB_MAX?DB_MAX:nthr;
                    if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_sq_thresh(sci,nthr);
                    else {
                        sch.sq_threshold.store(nthr,std::memory_order_relaxed);
                        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    }
                }
            }
            if(ImGui::IsItemActive()){
                float mx=ImGui::GetIO().MousePos.x;
                float nthr=DB_MIN+((mx-sp.x)/SLIDER_W)*DB_RNG;
                nthr=nthr<DB_MIN?DB_MIN:nthr>DB_MAX?DB_MAX:nthr;
                if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_sq_thresh(sci,nthr);
                else {
                    sch.sq_threshold.store(nthr,std::memory_order_relaxed);
                    if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                }
            }
            ImGui::SetCursorScreenPos(ImVec2(sp.x+SLIDER_W+6,ImGui::GetCursorScreenPos().y));
        }

        // ── Right side: channel status + REC + PAUSED ────────────────────
        {
            float rx=disp_w-8.0f;
            float ty2=(TOPBAR_H-ImGui::GetFontSize())/2;

            if(v.rec_on.load()){
                float el=std::chrono::duration<float>(std::chrono::steady_clock::now()-v.rec_t0).count();
                uint64_t fr=v.rec_frames.load(); float mb=(float)(fr*4)/1048576.0f;
                int mm2=(int)(el/60), ss2=(int)(el)%60;
                char rbuf[80]; snprintf(rbuf,sizeof(rbuf),"REC %d:%02d %.1fMB  ",mm2,ss2,mb);
                ImVec2 rs=ImGui::CalcTextSize(rbuf); rx-=rs.x;
                dl->AddText(ImVec2(rx,ty2),IM_COL32(255,80,80,255),rbuf);
            }
            if(v.spectrum_pause.load()){
                const char* ps="PAUSED  ";
                ImVec2 psz=ImGui::CalcTextSize(ps); rx-=psz.x;
                dl->AddText(ImVec2(rx,ty2),IM_COL32(255,180,0,255),ps);
            }

        }
        ImGui::PopStyleVar(); // ItemSpacing

        // ── Spectrum + Waterfall ──────────────────────────────────────────
        // ── 레이아웃 계산 ─────────────────────────────────────────────────
        float content_y=TOPBAR_H, content_h=disp_h-content_y-TOPBAR_H;
        const float div_h=14.0f, vdiv_w=8.0f;

        // ── 우측 패널 계산 ────────────────────────────────────────────────
        // right_panel_ratio: 0=완전 닫힘(우측벽에 붙음), 1=전체폭
        // right_w: 우측 패널 실제 픽셀 폭
        float right_w = disp_w * v.right_panel_ratio;
        right_w = std::max(0.0f, std::min(disp_w - vdiv_w, right_w));
        bool right_visible = right_w > 2.0f;

        // 세로 구분선은 항상 표시: 우측 패널 왼쪽 경계
        float vdiv_x = disp_w - vdiv_w - right_w;
        vdiv_x = std::max(0.0f, vdiv_x);

        // 좌측 영역 폭
        float left_w = vdiv_x;
        bool left_visible = left_w > 2.0f;
        {
            bool was_visible = v.render_visible.load();
            v.render_visible.store(left_visible);
            // 워터폴창 숨김/표시 전환 시 HOST는 즉시 heartbeat로 JOIN에게 알림
            if(v.net_srv && was_visible != left_visible){
                bool paused = v.spectrum_pause.load() || !left_visible;
                v.net_srv->broadcast_heartbeat(paused ? 2 : 0);
                heartbeat_last = std::chrono::steady_clock::now();
            }
            // JOIN: 수직바 왼쪽 끝이면 FFT 패킷을 큐에 넣지 않음 (트래픽 절약)
            if(v.remote_mode && v.net_cli)
                v.net_cli->fft_recv_enabled.store(left_visible);
        }

        // ── 가로 분할: 끝까지 허용 (0~1 풀레인지) ───────────────────────
        float sp_h = (content_h - div_h) * v.spectrum_height_ratio;
        float wf_h = content_h - div_h - sp_h;
        sp_h = std::max(0.0f, sp_h);
        wf_h = std::max(0.0f, wf_h);
        bool wf_visible = left_visible && wf_h > 1.0f;
        v.wf_area_visible.store(wf_visible);

        // ── 파워스펙트럼 ──────────────────────────────────────────────────
        if(left_visible && sp_h > 1.0f)
            v.draw_spectrum_area(dl, 0, content_y, left_w, sp_h);

        // ── 채널 스컬치 업데이트 (FFT 기반, 필터만 있으면 동작) ──────────
        v.update_channel_squelch();

        // ── 가로 구분선 (항상 드래그 가능하도록 클램프) ──────────────────
        // div_y를 content_y+1 ~ content_y+content_h-div_h-1 사이로 클램프
        float div_y = content_y + sp_h;
        div_y = std::max(content_y + 1.0f, std::min(content_y + content_h - div_h - 1.0f, div_y));
        if(left_visible){
            dl->AddRectFilled(ImVec2(0,div_y),ImVec2(left_w,div_y+div_h),IM_COL32(50,50,50,255));
            dl->AddLine(ImVec2(0,div_y+div_h/2),ImVec2(left_w,div_y+div_h/2),IM_COL32(80,80,80,255),1);
        }
        // 가로 구분선 드래그: InvisibleButton 대신 수동 마우스 감지
        // (ratio=0 시 TopBar 경계 겹침 버그 방지)
        {
            ImVec2 mpos = ImGui::GetIO().MousePos;
            bool hdiv_hov = (mpos.x >= 0 && mpos.x <= left_w &&
                             mpos.y >= div_y && mpos.y <= div_y + div_h);
            static bool hdiv_dragging = false;
            if(hdiv_hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                hdiv_dragging = true;
            if(!ImGui::IsMouseDown(ImGuiMouseButton_Left))
                hdiv_dragging = false;
            if(hdiv_dragging){
                v.spectrum_height_ratio += io.MouseDelta.y / content_h;
                v.spectrum_height_ratio = std::max(0.0f, std::min(1.0f, v.spectrum_height_ratio));
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
            } else if(hdiv_hov){
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
            }
            // 더미 버튼 (ImGui 레이아웃 커서 유지용)
            ImGui::SetCursorScreenPos(ImVec2(0, div_y));
            ImGui::InvisibleButton("hdiv", ImVec2(std::max(left_w,1.0f), div_h));
        }

        // ── 워터폴 ───────────────────────────────────────────────────────
        if(wf_visible)
            v.draw_waterfall_area(dl, 0, div_y+div_h, left_w, wf_h);
        else if(left_visible && wf_h > 0)
            dl->AddRectFilled(ImVec2(0,div_y+div_h),ImVec2(left_w,div_y+div_h+wf_h),IM_COL32(10,10,10,255));

        // SA 드래그 중 드롭 존 하이라이트
        if(v.sa_drag_active && v.sa_panel_open && right_visible){
            float rpx2=vdiv_x+vdiv_w;
            float rp_cy=content_y+TOPBAR_H*0.5f;
            dl->AddRectFilled(ImVec2(rpx2,rp_cy),ImVec2(disp_w,content_y+content_h),
                              IM_COL32(80,180,255,30));
            dl->AddRect(ImVec2(rpx2,rp_cy),ImVec2(disp_w,content_y+content_h),
                        IM_COL32(80,180,255,200),0,0,2.0f);
        }

        // ── 세로 구분선 (항상 표시) ───────────────────────────────────────
        dl->AddRectFilled(ImVec2(vdiv_x,content_y),ImVec2(vdiv_x+vdiv_w,content_y+content_h),IM_COL32(50,50,50,255));
        dl->AddLine(ImVec2(vdiv_x+vdiv_w/2,content_y),ImVec2(vdiv_x+vdiv_w/2,content_y+content_h),IM_COL32(80,80,80,255),1);
        // 세로 구분선 드래그: 수동 마우스 감지 (빠른 드래그에서 InvisibleButton 놓침 방지)
        {
            ImVec2 mpos2 = io.MousePos;
            bool vdiv_hov = (mpos2.x >= vdiv_x && mpos2.x <= vdiv_x + vdiv_w &&
                             mpos2.y >= content_y && mpos2.y <= content_y + content_h);
            static bool vdiv_dragging = false;
            if(vdiv_hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                vdiv_dragging = true;
            if(!ImGui::IsMouseDown(ImGuiMouseButton_Left))
                vdiv_dragging = false;
            if(vdiv_dragging){
                v.right_panel_ratio -= io.MouseDelta.x / disp_w;
                v.right_panel_ratio = std::max(0.0f, std::min(1.0f, v.right_panel_ratio));
                // 드래그 중 실시간으로 열린 상태의 너비를 기억 (닫힘 상태 제외)
                if(v.right_panel_ratio > 0.01f) right_panel_saved_ratio = v.right_panel_ratio;
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            } else if(vdiv_hov){
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            }
            // 더미 버튼 (ImGui 레이아웃 커서 유지용)
            ImGui::SetCursorScreenPos(ImVec2(vdiv_x, content_y));
            ImGui::InvisibleButton("vdiv", ImVec2(vdiv_w, content_h));
        }

        // ── 우측 패널 ─────────────────────────────────────────────────────
        static bool prev_right_visible_outer = false;
        if(!right_visible) prev_right_visible_outer = false;
        if(right_visible){
            float rpx  = vdiv_x + vdiv_w;
            float rp_w = disp_w - rpx;
            const float SUBBAR_H = TOPBAR_H * 0.5f;
            float rp_content_y = content_y + SUBBAR_H;
            float rp_content_h = content_h - SUBBAR_H;

            // right_panel_x 갱신
            v.right_panel_x = rpx;

            // 패널이 처음 열릴 때 STAT 자동 활성화
            if(!prev_right_visible_outer){
                if(!v.sa_panel_open && !board_open) stat_open = true;
            }
            prev_right_visible_outer = true;

            // ── SA 픽셀 준비되면 GL 업로드 ──────────────────────────────
            if(v.sa_pixel_ready.load()){ v.sa_upload_texture(); v.sa_anim_timer=0.0f; }

            // ── 서브바 배경 ──────────────────────────────────────────────
            dl->AddRectFilled(ImVec2(rpx,content_y),ImVec2(disp_w,rp_content_y),IM_COL32(35,35,40,255));
            dl->AddLine(ImVec2(rpx,rp_content_y-1),ImVec2(disp_w,rp_content_y-1),IM_COL32(60,60,70,255),1);

            float btn_y = content_y + (SUBBAR_H - ImGui::GetFontSize())/2;
            auto subbar_btn = [&](float bx, const char* lbl, bool active, ImU32 col_on) -> bool {
                ImVec2 tsz = ImGui::CalcTextSize(lbl);
                bool hov = io.MousePos.x>=bx && io.MousePos.x<=bx+tsz.x+2 &&
                           io.MousePos.y>=content_y && io.MousePos.y<rp_content_y;
                ImU32 col = active ? col_on
                          : (hov ? IM_COL32(160,160,180,255) : IM_COL32(110,110,130,255));
                dl->AddText(ImVec2(bx, btn_y), col, lbl);
                return hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
            };

            // ── STAT 버튼 ─────────────────────────────────────────────────
            float btn_x = rpx + 6;
            if(subbar_btn(btn_x, "STAT", stat_open, IM_COL32(80,255,160,255))){
                stat_open = !stat_open;
                if(stat_open){ v.sa_panel_open=false; board_open=false; }
            }

            // ── SA 버튼 ──────────────────────────────────────────────────
            float sa_btn_x = btn_x + 44;
            if(subbar_btn(sa_btn_x, "SA", v.sa_panel_open, IM_COL32(80,180,255,255))){
                v.sa_panel_open = !v.sa_panel_open;
                if(v.sa_panel_open){ stat_open=false; board_open=false; }
            }

            // ── BOARD 버튼 ───────────────────────────────────────────────
            float board_btn_x = sa_btn_x + 32;
            if(subbar_btn(board_btn_x, "BOARD", board_open, IM_COL32(255,200,80,255))){
                board_open = !board_open;
                if(board_open){ stat_open=false; v.sa_panel_open=false; }
            }

            // ── FFT size 선택 (SA 활성화 시에만) ─────────────────────────
            if(v.sa_panel_open){
                static const int fft_sizes[] = {32,64,128,256,512,1024,2048,4096,8192};
                static const char* fft_labels[] = {"32","64","128","256","512","1024","2048","4096","8192"};
                const int n_sizes = 9;
                char cur_label[16]; snprintf(cur_label,16,"%d",v.sa_fft_size);
                float combo_w = 62;
                float combo_x = disp_w - combo_w - 6;
                float combo_y = content_y + (SUBBAR_H - ImGui::GetFontSize() - 4)/2;
                ImGui::SetCursorScreenPos(ImVec2(combo_x, combo_y));
                ImGui::SetNextItemWidth(combo_w);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4,2));
                if(ImGui::BeginCombo("##sa_fft", cur_label)){
                    for(int i=0;i<n_sizes;i++){
                        bool sel = (v.sa_fft_size == fft_sizes[i]);
                        if(ImGui::Selectable(fft_labels[i], sel)){
                            if(v.sa_fft_size != fft_sizes[i]){
                                v.sa_fft_size = fft_sizes[i];
                                if(!v.sa_temp_path.empty() && !v.sa_computing.load()){
                                    v.sa_start(v.sa_temp_path);
                                }
                            }
                        }
                        if(sel) ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                ImGui::PopStyleVar();
                float lbl_x = combo_x - ImGui::CalcTextSize("FFT").x - 4;
                dl->AddText(ImVec2(lbl_x, combo_y+1), IM_COL32(130,130,150,255), "FFT");
            }

            // ── 패널 콘텐츠 영역 ─────────────────────────────────────────
            dl->AddRectFilled(ImVec2(rpx,rp_content_y),ImVec2(disp_w,content_y+content_h),IM_COL32(12,12,15,255));

            // ── STAT 패널 ─────────────────────────────────────────────────
            if(stat_open){
                float px=rpx, py=rp_content_y, pw=disp_w-rpx, ph=rp_content_h;
                ImGui::SetNextWindowPos(ImVec2(px,py));
                ImGui::SetNextWindowSize(ImVec2(pw,ph));
                ImGui::SetNextWindowBgAlpha(0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8,8));
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
                ImGui::Begin("##stat_panel", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                    ImGuiWindowFlags_NoDecoration);

                // ── Archive 파일 목록 ──────────────────────────────────
                static bool arch_priv_open = true;
                static bool arch_share_open = true;
                static bool arch_pub_open = true;
                static float arch_scan_timer = 0.0f;
                // 파일 크기 문자열 캐시 (scan 시점에 한 번만 stat → 매 프레임 stat 제거)
                static std::unordered_map<std::string,std::string> fsz_cache;
                arch_scan_timer += io.DeltaTime;
                if(arch_scan_timer >= 1.0f){
                    arch_scan_timer = 0.0f;

                    // 폴더별 .wav 스캔 헬퍼 (mtime 내림차순 정렬)
                    // stat()을 sort 비교자 밖에서 1회만 수집 → sort 중 syscall 제거
                    // 파일 크기도 여기서 캐싱 → 렌더 중 stat() 불필요
                    auto scan_dir = [](const std::string& dir, std::vector<std::string>& out,
                                       std::unordered_map<std::string,std::string>& szc){
                        out.clear();
                        DIR* d = opendir(dir.c_str());
                        if(!d) return;
                        struct dirent* ent;
                        struct { time_t mt; off_t sz; std::string name; } e;
                        std::vector<decltype(e)> tmp;
                        while((ent=readdir(d))!=nullptr){
                            const char* n = ent->d_name;
                            size_t nl = strlen(n);
                            if(nl>4 && strcmp(n+nl-4,".wav")==0){
                                struct stat st{};
                                std::string fp = dir + "/" + n;
                                stat(fp.c_str(), &st);
                                tmp.push_back({st.st_mtime, st.st_size, n});
                            }
                        }
                        closedir(d);
                        std::sort(tmp.begin(),tmp.end(),[](const auto& a,const auto& b){
                            return a.mt > b.mt;
                        });
                        out.reserve(tmp.size());
                        for(auto& p : tmp){
                            // 파일 크기 문자열 캐시 갱신
                            char buf[32];
                            double sz=(double)p.sz;
                            if(sz>=1e9)      snprintf(buf,sizeof(buf),"[%.1fG]",sz/1e9);
                            else if(sz>=1e6) snprintf(buf,sizeof(buf),"[%.1fM]",sz/1e6);
                            else if(sz>=1e3) snprintf(buf,sizeof(buf),"[%.1fK]",sz/1e3);
                            else             snprintf(buf,sizeof(buf),"[%dB]",(int)sz);
                            szc[p.name] = buf;
                            out.push_back(std::move(p.name));
                        }
                    };

                    // Record 스캔 (record/iq, record/audio)
                    scan_dir(BEWEPaths::record_iq_dir(),    rec_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::record_audio_dir(), rec_audio_files, fsz_cache);

                    // Private 스캔 (private/iq, private/audio)
                    scan_dir(BEWEPaths::private_iq_dir(),    priv_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::private_audio_dir(), priv_audio_files, fsz_cache);
                    priv_files.clear();
                    priv_files.insert(priv_files.end(), priv_iq_files.begin(),    priv_iq_files.end());
                    priv_files.insert(priv_files.end(), priv_audio_files.begin(), priv_audio_files.end());

                    // Public 스캔 (public/iq, public/audio) - HOST only
                    scan_dir(BEWEPaths::public_iq_dir(),    pub_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::public_audio_dir(), pub_audio_files, fsz_cache);
                    shared_files.clear();
                    shared_files.insert(shared_files.end(), pub_iq_files.begin(),    pub_iq_files.end());
                    shared_files.insert(shared_files.end(), pub_audio_files.begin(), pub_audio_files.end());

                    // Share 스캔 (share/iq, share/audio) - 다운로드된 파일
                    scan_dir(BEWEPaths::share_iq_dir(),    share_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::share_audio_dir(), share_audio_files, fsz_cache);
                    downloaded_files.clear();
                    downloaded_files.insert(downloaded_files.end(), share_iq_files.begin(),    share_iq_files.end());
                    downloaded_files.insert(downloaded_files.end(), share_audio_files.begin(), share_audio_files.end());
                }

                // ── 탭 바 ─────────────────────────────────────────────────
                ImGui::PushStyleColor(ImGuiCol_Tab,            ImVec4(0.12f,0.12f,0.16f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabHovered,     ImVec4(0.20f,0.30f,0.45f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabActive,      ImVec4(0.15f,0.40f,0.65f,1.f));
                if(ImGui::BeginTabBar("##stat_tabs")){

                // ══════════════════════════════════════════════════════════
                // ── LINK 탭 ───────────────────────────────────────────────
                // ══════════════════════════════════════════════════════════
                if(ImGui::BeginTabItem("LINK")){
                    ImGui::BeginChild("##link_scroll", ImVec2(0,0), false,
                        ImGuiWindowFlags_HorizontalScrollbar);

                    // ── Hardware ─────────────────────────────────────────
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::CollapsingHeader("Hardware")){
                        ImGui::Indent(8.f);
                        const char* hw_role = v.net_cli ? "[JOIN]" : (v.net_srv ? "[HOST]" : "[LOCAL]");
                        // 하드웨어 이름: JOIN이면 remote_hw로 판단, 아니면 로컬 디바이스
                        const char* hw_name = "Unknown";
                        if(v.net_cli){
                            uint8_t rh = v.net_cli->remote_hw.load();
                            hw_name = (rh == 0) ? "BladeRF 2.0 micro xA9" : "RTL-SDR";
                        } else {
                            if(v.dev_blade)    hw_name = "BladeRF 2.0 micro xA9";
                            else if(v.dev_rtl) hw_name = "RTL-SDR";
                        }
                        ImGui::TextColored(ImVec4(0.4f,0.85f,1.f,1.f), "%s", hw_role);
                        ImGui::SameLine(0,6);
                        ImGui::TextUnformatted(hw_name);

                        // 주파수 / SR 표시
                        if(v.net_cli){
                            ImGui::TextDisabled("  %.4f MHz  /  %.3f MSPS",
                                (double)(v.net_cli->remote_cf_mhz.load()),
                                (double)(v.net_cli->remote_sr.load()/1e6));
                        } else if(v.dev_blade||v.dev_rtl){
                            ImGui::TextDisabled("  %.4f MHz  /  %.3f MSPS",
                                (double)(v.header.center_frequency/1e6),
                                (double)(v.header.sample_rate/1e6));
                        }
                        ImGui::Unindent(8.f);
                    }
                    ImGui::Spacing();

                    // ── Operators ────────────────────────────────────────
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::CollapsingHeader("Operators")){
                        ImGui::Indent(8.f);
                        if(!v.net_srv && !v.net_cli){
                            // LOCAL 단독
                            const char* nm = v.host_name[0] ? v.host_name : "(no login)";
                            ImGui::TextColored(ImVec4(0.55f,0.9f,0.55f,1.f),
                                "[LOCAL] %s  [Tier%d]", nm, login_get_tier());
                        } else {
                            // HOST/JOIN 통합 표시 (index=0: HOST, index>=1: JOIN)
                            auto draw_op_entry = [&](const OpEntry& op){
                                bool is_host = (op.index == 0);
                                const char* badge = is_host ? "[HOST]" : "[JOIN]";
                                ImVec4 col = is_host ? ImVec4(0.4f,0.85f,1.f,1.f)
                                                     : ImVec4(0.7f,0.92f,0.7f,1.f);
                                ImGui::TextColored(col, "%s %s  [Tier%d]",
                                    badge, op.name, op.tier);
                            };
                            if(v.net_srv){
                                // HOST 모드: 자신(index=0) 먼저
                                OpEntry host_e{}; host_e.index=0;
                                host_e.tier=(uint8_t)login_get_tier();
                                strncpy(host_e.name, v.host_name[0]?v.host_name:"Host", 31);
                                draw_op_entry(host_e);
                                auto joins = v.net_srv->get_operators();
                                for(auto& op : joins) draw_op_entry(op);
                                if(joins.empty())
                                    ImGui::TextDisabled("  (no clients connected)");
                            } else if(v.net_cli){
                                std::lock_guard<std::mutex> lk(v.net_cli->op_mtx);
                                if(v.net_cli->op_list.count==0)
                                    ImGui::TextDisabled("  (none)");
                                else
                                    for(int oi=0;oi<(int)v.net_cli->op_list.count;oi++)
                                        draw_op_entry(v.net_cli->op_list.ops[oi]);
                            }
                        }
                        ImGui::Unindent(8.f);
                    }
                    ImGui::Spacing();

                    // ── Active Channels ──────────────────────────────────
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::CollapsingHeader("Active Channels")){
                        ImGui::Indent(8.f);
                        auto set_local_out = [&](int ci, int lco){
                            int prev = v.local_ch_out[ci];
                            v.local_ch_out[ci] = lco;
                            if(v.net_cli){
                                bool now_mute=(lco==3), was_mute=(prev==3);
                                if(now_mute&&!was_mute) v.net_cli->cmd_toggle_recv(ci,false);
                                else if(!now_mute&&was_mute) v.net_cli->cmd_toggle_recv(ci,true);
                            }
                            if(v.net_srv){
                                uint32_t mask=v.channels[ci].audio_mask.load();
                                if(lco==3) mask&=~0x1u; else mask|=0x1u;
                                v.channels[ci].audio_mask.store(mask);
                                v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                            }
                        };
                        bool any_ch = false;
                        // 주파수 오름차순 정렬
                        int ch_order[MAX_CHANNELS];
                        int ch_count=0;
                        for(int i=0;i<MAX_CHANNELS;i++)
                            if(v.channels[i].filter_active) ch_order[ch_count++]=i;
                        std::sort(ch_order, ch_order+ch_count, [&](int a, int b){
                            float ca=(v.channels[a].s+v.channels[a].e)*0.5f;
                            float cb=(v.channels[b].s+v.channels[b].e)*0.5f;
                            return ca<cb;
                        });
                        for(int ci_idx=0;ci_idx<ch_count;ci_idx++){
                            int ci=ch_order[ci_idx];
                            Channel& ch=v.channels[ci];
                            any_ch = true;

                            float cf_mhz=(ch.s+ch.e)/2.0f;
                            float bw_khz=(ch.e-ch.s)*1000.0f;
                            const char* mnames[]={"--","AM","FM","MAG"};
                            int mi=(int)ch.mode; if(mi<0||mi>3) mi=0;

                            // ── 채널 색상 (draw_all_channels와 동일) ──────
                            bool is_arec = ch.audio_rec_on.load();
                            bool is_irec = (v.rec_on.load()&&ci==v.rec_ch);
                            bool dem = v.remote_mode
                                ? (ch.mode!=Channel::DM_NONE)
                                : ch.dem_run.load();
                            bool gate = ch.sq_gate.load();

                            ImU32 mode_col;
                            if(is_irec||is_arec)
                                mode_col=IM_COL32(255,60,60,255);
                            else if(!dem||ch.mode==Channel::DM_NONE)
                                mode_col=IM_COL32(160,160,160,255);
                            else if(ch.mode==Channel::DM_AM)
                                mode_col=IM_COL32(80,200,255,255);
                            else if(ch.mode==Channel::DM_FM)
                                mode_col=IM_COL32(255,220,50,255);
                            else
                                mode_col=IM_COL32(180,80,255,255);

                            // 스컬치 게이트: 열리면 밝게, 닫히면 50% 어둡게
                            ImVec4 tc_v;
                            if(gate){
                                tc_v=ImVec4(
                                    ((mode_col>>IM_COL32_R_SHIFT)&0xFF)/255.f,
                                    ((mode_col>>IM_COL32_G_SHIFT)&0xFF)/255.f,
                                    ((mode_col>>IM_COL32_B_SHIFT)&0xFF)/255.f, 1.f);
                            } else {
                                tc_v=ImVec4(
                                    ((mode_col>>IM_COL32_R_SHIFT)&0xFF)/255.f*0.55f,
                                    ((mode_col>>IM_COL32_G_SHIFT)&0xFF)/255.f*0.55f,
                                    ((mode_col>>IM_COL32_B_SHIFT)&0xFF)/255.f*0.55f, 0.85f);
                            }

                            // 선택 강조 배경
                            if(ch.selected){
                                ImVec2 cp=ImGui::GetCursorScreenPos();
                                float rw=ImGui::GetContentRegionAvail().x;
                                ImGui::GetWindowDrawList()->AddRectFilled(
                                    ImVec2(cp.x-4,cp.y-1),
                                    ImVec2(cp.x+rw,cp.y+ImGui::GetTextLineHeight()+2),
                                    IM_COL32(
                                        (uint8_t)(((mode_col>>IM_COL32_R_SHIFT)&0xFF)*0.25f),
                                        (uint8_t)(((mode_col>>IM_COL32_G_SHIFT)&0xFF)*0.25f),
                                        (uint8_t)(((mode_col>>IM_COL32_B_SHIFT)&0xFF)*0.25f),180));
                                // 왼쪽 세로줄 (두꺼운 강조)
                                ImGui::GetWindowDrawList()->AddLine(
                                    ImVec2(cp.x-4,cp.y-1),
                                    ImVec2(cp.x-4,cp.y+ImGui::GetTextLineHeight()+2),
                                    mode_col, gate?3.0f:1.5f);
                            }

                            // 텍스트
                            char label[64];
                            snprintf(label,sizeof(label),"[%d] %s  %.3f MHz  %.0fkHz",
                                v.freq_sorted_display_num(ci),mnames[mi],cf_mhz,bw_khz);
                            ImGui::PushID(ci*1000+700);
                            ImGui::PushStyleColor(ImGuiCol_Text,tc_v);
                            // gate open이면 볼드 효과 (1px offset)
                            if(gate && dem){
                                ImVec2 cp2=ImGui::GetCursorScreenPos();
                                ImGui::GetWindowDrawList()->AddText(
                                    ImVec2(cp2.x+1,cp2.y),
                                    IM_COL32((uint8_t)(tc_v.x*255*0.4f),
                                             (uint8_t)(tc_v.y*255*0.4f),
                                             (uint8_t)(tc_v.z*255*0.4f),100), label);
                            }
                            ImGui::TextUnformatted(label);
                            ImGui::PopStyleColor();

                            // 클릭 상호작용
                            if(ImGui::IsItemHovered()){
                                // Owner/Listener 툴팁 (비뮤트 멤버만, HOST/JOIN 동기화)
                                {
                                    char tip[512]; int tp=0;
                                    const char* ow = ch.owner[0] ? ch.owner : "?";
                                    tp += snprintf(tip+tp,sizeof(tip)-tp,"Owner: %s\nListener:",ow);
                                    // JOIN은 srv_audio_mask (서버 전체 마스크) 사용
                                    uint32_t mask = v.net_cli
                                        ? v.srv_audio_mask[ci]
                                        : ch.audio_mask.load();
                                    bool any_listener = false;
                                    if(v.net_srv){
                                        // HOST: bit0=자신, bit i=JOIN op_idx i
                                        if(mask & 0x1u){
                                            const char* hname=v.host_name[0]?v.host_name:"Host";
                                            tp+=snprintf(tip+tp,sizeof(tip)-tp," %s",hname);
                                            any_listener=true;
                                        }
                                        auto ops2=v.net_srv->get_operators();
                                        for(auto& op:ops2){
                                            if(mask & (1u<<op.index)){
                                                tp+=snprintf(tip+tp,sizeof(tip)-tp,"%s%s",any_listener?", ":"",op.name);
                                                any_listener=true;
                                            }
                                        }
                                    } else if(v.net_cli){
                                        // JOIN: op_list에 HOST(index=0)와 JOINs 모두 포함
                                        std::lock_guard<std::mutex> lk2(v.net_cli->op_mtx);
                                        for(int oi=0;oi<(int)v.net_cli->op_list.count;oi++){
                                            auto& op=v.net_cli->op_list.ops[oi];
                                            if(mask & (1u<<op.index)){
                                                tp+=snprintf(tip+tp,sizeof(tip)-tp,"%s%s",any_listener?", ":"",op.name);
                                                any_listener=true;
                                            }
                                        }
                                    }
                                    if(!any_listener) tp+=snprintf(tip+tp,sizeof(tip)-tp," (none)");
                                    ImGui::SetTooltip("%s",tip);
                                }
                                auto delete_ch = [&](){
                                    if(v.net_cli) v.net_cli->cmd_delete_ch(ci);
                                    v.stop_dem(ci); v.stop_digi(ci);
                                    v.channels[ci].filter_active=false;
                                    v.channels[ci].selected=false;
                                    v.local_ch_out[ci]=1;
                                    v.ch_created_by_me[ci]=false;
                                    v.digi_panel_on[ci]=false;
                                    if(v.selected_ch==ci) v.selected_ch=-1;
                                    if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                                };
                                if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                                    delete_ch();
                                } else if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                    if(v.selected_ch>=0) v.channels[v.selected_ch].selected=false;
                                    v.selected_ch=ci; v.channels[ci].selected=true;
                                }
                            }
                            // Del 키 (선택된 채널)
                            if(ch.selected && ImGui::IsKeyPressed(ImGuiKey_Delete,false)){
                                if(v.net_cli) v.net_cli->cmd_delete_ch(ci);
                                v.stop_dem(ci); v.stop_digi(ci);
                                v.channels[ci].filter_active=false;
                                v.channels[ci].selected=false;
                                v.local_ch_out[ci]=1;
                                v.ch_created_by_me[ci]=false;
                                v.digi_panel_on[ci]=false;
                                if(v.selected_ch==ci) v.selected_ch=-1;
                                if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                                ImGui::PopID();
                                continue;
                            }

                            // L/L+R/R/M 버튼
                            ImGui::SameLine(0,8);
                            int lco=v.local_ch_out[ci];
                            const char* lbl_out[4]={"L","L+R","R","M"};
                            for(int bi=0;bi<4;bi++){
                                bool active=(lco==bi);
                                if(active) ImGui::PushStyleColor(ImGuiCol_Button,
                                    bi==3?ImVec4(0.6f,0.1f,0.1f,1.f):ImVec4(0.1f,0.55f,0.1f,1.f));
                                if(ImGui::SmallButton(lbl_out[bi])) set_local_out(ci,bi);
                                if(active) ImGui::PopStyleColor();
                                if(bi<3) ImGui::SameLine(0,2);
                            }


                            // ── 디지털 모드 버튼 (D키로 패널 열었을 때만 표시) ──
                            if(v.digi_panel_on[ci]){
                                ImGui::SameLine(0,10);
                                ImGui::TextDisabled("|");
                                ImGui::SameLine(0,4);
                                struct DigiBtn { const char* lbl; Channel::DigitalMode mode; };
                                static const DigiBtn dbtn[]={
                                    {"ADS-B", Channel::DIGI_ADSB},
                                    {"AIS",   Channel::DIGI_AIS},
                                    {"DMR",   Channel::DIGI_DMR},
                                };
                                for(int di=0;di<3;di++){
                                    Channel::DigitalMode dm = dbtn[di].mode;
                                    bool dactive = (ch.digital_mode==dm && ch.digi_run.load());
                                    if(dactive)
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f,0.2f,0.9f,1.f));
                                    else
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f,0.15f,0.2f,1.f));
                                    if(ImGui::SmallButton(dbtn[di].lbl)){
                                        if(dactive){
                                            v.stop_digi(ci);
                                        } else {
                                            v.stop_digi(ci);
                                            if(dm == Channel::DIGI_AIS || dm == Channel::DIGI_DMR)
                                                v.start_digi(ci, dm);
                                            // ADS-B: 미구현
                                        }
                                    }
                                    ImGui::PopStyleColor();
                                    if(di<2) ImGui::SameLine(0,2);
                                }
                            }
                            ImGui::PopID();
                        }
                        if(!any_ch) ImGui::TextDisabled("  (none)");
                        ImGui::Unindent(8.f);
                    }
                    ImGui::Spacing();

                    // ── Record ───────────────────────────────────────────
                    {
                        std::lock_guard<std::mutex> lk(v.rec_entries_mtx);

                        // region_save 진행 중 감지
                        bool region_saving = v.rec_busy_flag.load() && !v.sa_mode;

                        bool any = !v.rec_entries.empty() || region_saving;
                        for(int ci=0;ci<MAX_CHANNELS&&!any;ci++)
                            if(v.channels[ci].audio_rec_on.load()) any=true;

                        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                        if(ImGui::CollapsingHeader("Record")){
                            ImGui::Indent(8.f);
                            if(!any){
                                ImGui::TextDisabled("  (none)");
                            } else {
                                // ── IQ 목록 ──────────────────────────────
                                        // Deny 타이머 감소 + 만료 항목 제거
                                for(auto it=v.rec_entries.begin();it!=v.rec_entries.end();){
                                    if(it->req_state==FFTViewer::RecEntry::REQ_DENIED){
                                        it->req_deny_timer -= io.DeltaTime;
                                        if(it->req_deny_timer <= 0.f){ it=v.rec_entries.erase(it); continue; }
                                    }
                                    ++it;
                                }

                                bool has_iq=false;
                                for(auto& re : v.rec_entries) if(!re.is_audio) has_iq=true;
                                if(region_saving) has_iq=true;

                                bool has_audio=false;
                                for(auto& re : v.rec_entries) if(re.is_audio) has_audio=true;

                                if(has_iq){
                                    ImGui::TextDisabled("  IQ");
                                    ImGui::Indent(6.f);
                                    // region save 진행중 (HOST 본인 녹음만 — JOIN 요청은 REQ_CONFIRMED에서 표시)
                                    if(region_saving){
                                        bool has_req_confirmed = false;
                                        for(auto& re2 : v.rec_entries)
                                            if(re2.is_region && re2.req_state==FFTViewer::RecEntry::REQ_CONFIRMED)
                                                { has_req_confirmed=true; break; }
                                        if(!has_req_confirmed){
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImGui::PushStyleColor(ImGuiCol_Text,
                                                blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                            ImGui::Text("[REC]  IQ Recording ...");
                                            ImGui::PopStyleColor();
                                        }
                                    }
                                    using RS = FFTViewer::RecEntry::ReqState;
                                    for(int ri=(int)v.rec_entries.size()-1;ri>=0;ri--){
                                        auto& re=v.rec_entries[ri];
                                        if(re.is_audio) continue;
                                        ImGui::PushID(ri+30000);
                                        if(re.req_state == RS::REQ_NONE){
                                            // 일반 IQ 녹음 항목
                                            if(re.finished){
                                                auto it_rz=fsz_cache.find(re.filename);
                                                const std::string szstr=(it_rz!=fsz_cache.end())?it_rz->second:fmt_filesize("",re.path);
                                                std::string lbl = std::string("[Done]  ")+re.filename;
                                                if(!szstr.empty()) lbl += "  "+szstr;
                                                bool is_sel_r = file_ctx.selected && file_ctx.filepath==re.path;
                                                ImGui::Selectable(lbl.c_str(), is_sel_r);
                                                if(ImGui::IsItemHovered()){
                                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                                        file_ctx.selected=true; file_ctx.open=false;
                                                        file_ctx.filepath=re.path; file_ctx.filename=re.filename;
                                                        file_ctx.is_public=false;
                                                    }
                                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                        file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                                        file_ctx.selected=true;
                                                    }
                                                }
                                            } else {
                                                float t2=(float)ImGui::GetTime();
                                                bool blink=(fmodf(t2,0.8f)<0.4f);
                                                ImGui::PushStyleColor(ImGuiCol_Text,
                                                    blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                                // 녹음 중 파일 크기: 0.5초마다만 갱신
                                                static std::unordered_map<std::string,std::pair<float,std::string>> rec_sz_cache;
                                                auto& rc=rec_sz_cache[re.filename];
                                                if(t2-rc.first >= 0.5f){ rc.first=t2; rc.second=fmt_filesize("",re.path); }
                                                if(!rc.second.empty())
                                                    ImGui::Text("[REC]  IQ Recording ...  %s", rc.second.c_str());
                                                else
                                                    ImGui::Text("[REC]  IQ Recording ...");
                                                ImGui::PopStyleColor();
                                            }
                                        } else {
                                            // 영역 IQ 요청 항목 — 상태별 표시
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImU32 col=IM_COL32(200,200,200,255);
                                            if(re.req_state==RS::REQ_CONFIRMED){
                                                // 녹음 중
                                                col=blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255);
                                                ImGui::PushStyleColor(ImGuiCol_Text,col);
                                                ImGui::Text("[REC]  IQ Recording ...");
                                                ImGui::PopStyleColor();
                                            } else if(re.req_state==RS::REQ_TRANSFERRING){
                                                // 전송 중
                                                col=IM_COL32(80,180,255,255);
                                                ImGui::PushStyleColor(ImGuiCol_Text,col);
                                                if(re.xfer_total>0)
                                                    ImGui::Text("[Transferring]  %s  (%.1f / %.1f MB)",
                                                        re.filename.c_str(),
                                                        re.xfer_done/1048576.0, re.xfer_total/1048576.0);
                                                else
                                                    ImGui::Text("[Transferring]  %s", re.filename.c_str());
                                                ImGui::PopStyleColor();
                                            } else if((re.req_state==RS::REQ_NONE || re.req_state==RS::REQ_TRANSFERRING) && re.finished){
                                                // 전송 완료
                                                col=IM_COL32(120,200,120,255);
                                                ImGui::PushStyleColor(ImGuiCol_Text,col);
                                                if(re.xfer_total > 0)
                                                    ImGui::Text("[Done]  %s  (%.1f MB)", re.filename.c_str(), re.xfer_total/1048576.0);
                                                else {
                                                    auto it_rz2=fsz_cache.find(re.filename);
                                                    const std::string szstr2=(it_rz2!=fsz_cache.end())?it_rz2->second:fmt_filesize("",re.path);
                                                    if(!szstr2.empty())
                                                        ImGui::Text("[Done]  %s  %s", re.filename.c_str(), szstr2.c_str());
                                                    else
                                                        ImGui::Text("[Done]  %s", re.filename.c_str());
                                                }
                                                ImGui::PopStyleColor();
                                            } else if(re.req_state==RS::REQ_DENIED){
                                                col=IM_COL32(200,80,80,255);
                                                ImGui::PushStyleColor(ImGuiCol_Text,col);
                                                ImGui::Text("[Denied]  %s  %.0fs", re.filename.c_str(), re.req_deny_timer);
                                                ImGui::PopStyleColor();
                                            } else {
                                                ImGui::PushStyleColor(ImGuiCol_Text,col);
                                                ImGui::Text("%s", re.filename.c_str());
                                                ImGui::PopStyleColor();
                                            }
                                        }
                                        ImGui::PopID();
                                    }
                                    ImGui::Unindent(6.f);
                                }

                                if(has_iq && has_audio){
                                    ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0.3f,0.3f,0.3f,0.6f));
                                    ImGui::Separator();
                                    ImGui::PopStyleColor();
                                }

                                if(has_audio){
                                    ImGui::TextDisabled("  Audio");
                                    ImGui::Indent(6.f);
                                    for(int ri=(int)v.rec_entries.size()-1;ri>=0;ri--){
                                        auto& re=v.rec_entries[ri];
                                        if(!re.is_audio) continue;
                                        ImGui::PushID(ri+32000);
                                        if(re.finished){
                                            auto it_az=fsz_cache.find(re.filename);
                                            const std::string szstr=(it_az!=fsz_cache.end())?it_az->second:fmt_filesize("",re.path);
                                            std::string lbl = std::string("[Done]  ")+re.filename;
                                            if(!szstr.empty()) lbl += "  "+szstr;
                                            bool is_sel_a = file_ctx.selected && file_ctx.filepath==re.path;
                                            ImGui::Selectable(lbl.c_str(), is_sel_a);
                                            if(ImGui::IsItemHovered()){
                                                if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                                    file_ctx.selected=true; file_ctx.open=false;
                                                    file_ctx.filepath=re.path; file_ctx.filename=re.filename;
                                                    file_ctx.is_public=false;
                                                }
                                                if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                    file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                                    file_ctx.selected=true;
                                                }
                                            }
                                        } else {
                                            float elapsed=std::chrono::duration<float>(
                                                std::chrono::steady_clock::now()-re.t_start).count();
                                            int secs=(int)elapsed;
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImGui::PushStyleColor(ImGuiCol_Text,
                                                blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                            static std::unordered_map<std::string,std::pair<float,std::string>> aud_sz_cache;
                                            auto& ac=aud_sz_cache[re.filename];
                                            if(t2-ac.first >= 0.5f){ ac.first=t2; ac.second=fmt_filesize("",re.path); }
                                            if(!ac.second.empty())
                                                ImGui::Text("[REC]  %s  [%ds]  %s", re.filename.c_str(), secs, ac.second.c_str());
                                            else
                                                ImGui::Text("[REC]  %s  [%ds]", re.filename.c_str(), secs);
                                            ImGui::PopStyleColor();
                                        }
                                        ImGui::PopID();
                                    }
                                    ImGui::Unindent(6.f);
                                }
                            }
                            ImGui::Unindent(8.f);
                        }
                        ImGui::Spacing();

                    }
                    // ── Archive ──────────────────────────────────────────
                    // 공통 파일 리스트 렌더 헬퍼 (IQ/Audio 분리)
                    auto draw_file_list = [&](
                        const std::string& child_id,
                        const std::vector<std::string>& iq_files,
                        const std::vector<std::string>& audio_files,
                        const std::string& iq_dir,
                        const std::string& audio_dir,
                        int id_base,
                        std::function<void(const std::string& fp, const std::string& fn)> on_hover = nullptr,
                        bool is_public_section = false)
                    {
                        int total_rows = (int)iq_files.size() + (int)audio_files.size()
                            + (!iq_files.empty()?1:0) + (!audio_files.empty()?1:0);
                        float ph = std::min(total_rows*18.f+4.f, 160.f);
                        if(ph < 36.f) ph = 36.f;
                        ImGui::BeginChild(child_id.c_str(), ImVec2(0, ph), true);
                        if(!iq_files.empty()){
                            ImGui::TextDisabled("  IQ");
                            for(int fi=0;fi<(int)iq_files.size();fi++){
                                ImGui::PushID(id_base+fi);
                                std::string fp = iq_dir+"/"+iq_files[fi];
                                auto it_sz=fsz_cache.find(iq_files[fi]);
                                const std::string& szstr=(it_sz!=fsz_cache.end())?it_sz->second:fmt_filesize("",fp);
                                std::string display = "  "+iq_files[fi];
                                if(!szstr.empty()) display += "  "+szstr;
                                bool is_sel = file_ctx.selected && file_ctx.filepath==fp;
                                ImGui::Selectable(display.c_str(), is_sel);
                                if(ImGui::IsItemHovered()){
                                    if(on_hover) on_hover(fp, iq_files[fi]);
                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                        file_ctx.selected=true; file_ctx.open=false;
                                        file_ctx.filepath=fp; file_ctx.filename=iq_files[fi];
                                        file_ctx.is_public=is_public_section;
                                    }
                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                        file_ctx={true,io.MousePos.x,io.MousePos.y,fp,iq_files[fi]};
                                        file_ctx.is_public=is_public_section;
                                        file_ctx.selected=true;
                                    }
                                }
                                ImGui::PopID();
                            }
                        }
                        if(!audio_files.empty()){
                            ImGui::TextDisabled("  Audio");
                            for(int fi=0;fi<(int)audio_files.size();fi++){
                                ImGui::PushID(id_base+500+fi);
                                std::string fp = audio_dir+"/"+audio_files[fi];
                                auto it_sz2=fsz_cache.find(audio_files[fi]);
                                const std::string& szstr=(it_sz2!=fsz_cache.end())?it_sz2->second:fmt_filesize("",fp);
                                std::string display = "  "+audio_files[fi];
                                if(!szstr.empty()) display += "  "+szstr;
                                bool is_sel2 = file_ctx.selected && file_ctx.filepath==fp;
                                ImGui::Selectable(display.c_str(), is_sel2);
                                if(ImGui::IsItemHovered()){
                                    if(on_hover) on_hover(fp, audio_files[fi]);
                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                        file_ctx.selected=true; file_ctx.open=false;
                                        file_ctx.filepath=fp; file_ctx.filename=audio_files[fi];
                                        file_ctx.is_public=is_public_section;
                                    }
                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                        file_ctx={true,io.MousePos.x,io.MousePos.y,fp,audio_files[fi]};
                                        file_ctx.is_public=is_public_section;
                                        file_ctx.selected=true;
                                    }
                                }
                                ImGui::PopID();
                            }
                        }
                        if(iq_files.empty() && audio_files.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::EndChild();
                    };

                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::CollapsingHeader("Archive")){
                        ImGui::Indent(8.f);

                        // ── Private ──────────────────────────────────────
                        ImGui::SetNextItemOpen(arch_priv_open, ImGuiCond_Always);
                        {
                        int priv_cnt = (int)priv_iq_files.size() + (int)priv_audio_files.size();
                        if(ImGui::TreeNodeEx("##priv_node", ImGuiTreeNodeFlags_None, "Private  (%d)", priv_cnt)){
                            arch_priv_open = true;
                            draw_file_list("##priv_list",
                                priv_iq_files, priv_audio_files,
                                BEWEPaths::private_iq_dir(), BEWEPaths::private_audio_dir(),
                                20000);
                            ImGui::TreePop();
                        } else { arch_priv_open = false; }
                        } // priv block

                        ImGui::Spacing();

                        // ── Public ────────────────────────────────────────
                        ImGui::SetNextItemOpen(arch_pub_open, ImGuiCond_Always);
                        {
                        int pub_cnt2;
                        if(v.net_cli){
                            std::lock_guard<std::mutex> jlk(join_share_mtx);
                            pub_cnt2 = (int)join_share_files.size();
                        } else {
                            pub_cnt2 = (int)shared_files.size();
                        }
                        if(ImGui::TreeNodeEx("##pub_node", ImGuiTreeNodeFlags_None,
                                "Public  (%d)", pub_cnt2)){
                            arch_pub_open = true;
                            if(v.net_cli){
                                // ── JOIN: HOST Public 파일 목록 표시 (IQ/Audio 구분) ─
                                std::lock_guard<std::mutex> jlk(join_share_mtx);
                                std::vector<int> jiq_idx, jaudio_idx;
                                for(int si=0;si<(int)join_share_files.size();si++){
                                    const auto& jse = join_share_files[si];
                                    if(jse.filename.size()>3 && jse.filename.substr(0,3)=="IQ_")
                                        jiq_idx.push_back(si);
                                    else
                                        jaudio_idx.push_back(si);
                                }
                                int j_total_rows = (int)join_share_files.size()
                                    + (!jiq_idx.empty()?1:0) + (!jaudio_idx.empty()?1:0);
                                float sh2 = std::min(j_total_rows*18.f+8.f, 160.f);
                                if(sh2 < 36.f) sh2 = 36.f;
                                ImGui::BeginChild("##pub_list", ImVec2(0, sh2), true);
                                auto draw_join_pub_file = [&](int si){
                                    auto& jse = join_share_files[si];
                                    ImGui::PushID(si+22000);
                                    bool already_dl = false;
                                    for(auto& df:downloaded_files) if(df==jse.filename){already_dl=true;break;}
                                    const char* pfx = already_dl ? "[Done]  " : "";
                                    ImU32 col = already_dl ? IM_COL32(80,220,80,255) : IM_COL32(80,180,255,255);
                                    std::string sdisplay = std::string(pfx)+jse.filename;
                                    if(jse.size_bytes > 0){
                                        char szb[32];
                                        if(jse.size_bytes >= 1024*1024)
                                            snprintf(szb,sizeof(szb)," [%.1fM]",(double)jse.size_bytes/1048576.0);
                                        else
                                            snprintf(szb,sizeof(szb)," [%.1fK]",(double)jse.size_bytes/1024.0);
                                        sdisplay += szb;
                                    }
                                    ImGui::PushStyleColor(ImGuiCol_Text, col);
                                    ImGui::Selectable(sdisplay.c_str(), false);
                                    ImGui::PopStyleColor();
                                    if(ImGui::IsItemHovered()){
                                        // 업로더 툴팁
                                        if(!jse.uploader.empty()){
                                            ImGui::BeginTooltip();
                                            ImGui::TextDisabled("by %s", jse.uploader.c_str());
                                            ImGui::EndTooltip();
                                        }
                                        if(ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                                            ImGui::OpenPopup("##pub_dl_ctx");
                                    }
                                    if(ImGui::BeginPopup("##pub_dl_ctx")){
                                        ImGui::TextDisabled("%s", jse.filename.c_str());
                                        if(!jse.uploader.empty()){
                                            ImGui::TextDisabled("by %s", jse.uploader.c_str());
                                        }
                                        ImGui::Separator();
                                        if(!already_dl && ImGui::MenuItem("Download")){
                                            v.net_cli->cmd_request_share_download(jse.filename.c_str());
                                        }
                                        // 소유자만 Delete 가능
                                        std::string my_name = login_get_id();
                                        if(jse.uploader == my_name){
                                            ImGui::Separator();
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f,0.35f,0.35f,1.f));
                                            if(ImGui::MenuItem("Delete")){
                                                v.net_cli->cmd_delete_pub_file(jse.filename.c_str());
                                                // 로컬에서도 즉시 제거 (서버 응답 브로드캐스트로 확인)
                                                join_share_files.erase(join_share_files.begin()+si);
                                                ImGui::PopStyleColor();
                                                ImGui::EndPopup();
                                                ImGui::PopID();
                                                return;
                                            }
                                            ImGui::PopStyleColor();
                                        }
                                        ImGui::EndPopup();
                                    }
                                    ImGui::PopID();
                                };
                                if(!jiq_idx.empty()){
                                    ImGui::TextDisabled("  IQ");
                                    for(int si : jiq_idx) draw_join_pub_file(si);
                                }
                                if(!jaudio_idx.empty()){
                                    ImGui::TextDisabled("  Audio");
                                    for(int si : jaudio_idx) draw_join_pub_file(si);
                                }
                                if(join_share_files.empty()) ImGui::TextDisabled("  (empty)");
                                ImGui::EndChild();
                            } else {
                                // ── HOST/LOCAL: public 폴더 파일 표시 (소유자 표시) ─
                                // draw_file_list 대신 직접 렌더: 소유자 인라인 표시
                                auto draw_pub_files = [&](const std::vector<std::string>& files,
                                                          const std::string& dir, int id_base2){
                                    for(int fi=0;fi<(int)files.size();fi++){
                                        ImGui::PushID(id_base2+fi);
                                        const std::string& fn = files[fi];
                                        std::string fp = dir+"/"+fn;
                                        auto it_psz=fsz_cache.find(fn);
                                        const std::string& szstr=(it_psz!=fsz_cache.end())?it_psz->second:fmt_filesize("",fp);
                                        std::string display = "  " + fn;
                                        if(!szstr.empty()) display += "  " + szstr;
                                        bool is_sel_p = file_ctx.selected && file_ctx.filepath==fp;
                                        ImGui::Selectable(display.c_str(), is_sel_p);
                                        if(ImGui::IsItemHovered()){
                                            if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                                file_ctx.selected=true; file_ctx.open=false;
                                                file_ctx.filepath=fp; file_ctx.filename=fn;
                                                file_ctx.is_public=true;
                                            }
                                            // 소유자 + 다운로드한 사람 툴팁
                                            auto oit = pub_owners.find(fn);
                                            auto it2 = pub_listeners.find(fn);
                                            bool has_owner = oit != pub_owners.end() && !oit->second.empty();
                                            bool has_dl    = it2 != pub_listeners.end() && !it2->second.empty();
                                            if(has_owner || has_dl){
                                                ImGui::BeginTooltip();
                                                if(has_owner)
                                                    ImGui::TextDisabled("by %s", oit->second.c_str());
                                                if(has_dl){
                                                    if(has_owner) ImGui::Separator();
                                                    ImGui::TextDisabled("Downloaded by:");
                                                    for(auto& nm : it2->second)
                                                        ImGui::Text("  %s", nm.c_str());
                                                }
                                                ImGui::EndTooltip();
                                            }
                                            if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                file_ctx={true,io.MousePos.x,io.MousePos.y,fp,fn};
                                                file_ctx.is_public=true;
                                                file_ctx.selected=true;
                                            }
                                        }
                                        ImGui::PopID();
                                    }
                                };
                                int pub_total = (int)pub_iq_files.size() + (int)pub_audio_files.size();
                                int pub_total_rows = pub_total
                                    + (!pub_iq_files.empty()?1:0) + (!pub_audio_files.empty()?1:0);
                                float pub_ph = std::min(pub_total_rows*18.f+4.f, 160.f);
                                if(pub_ph < 36.f) pub_ph = 36.f;
                                ImGui::BeginChild("##pub_list", ImVec2(0,pub_ph), true);
                                if(!pub_iq_files.empty()){
                                    ImGui::TextDisabled("  IQ");
                                    draw_pub_files(pub_iq_files, BEWEPaths::public_iq_dir(), 21000);
                                }
                                if(!pub_audio_files.empty()){
                                    ImGui::TextDisabled("  Audio");
                                    draw_pub_files(pub_audio_files, BEWEPaths::public_audio_dir(), 21500);
                                }
                                if(pub_iq_files.empty() && pub_audio_files.empty())
                                    ImGui::TextDisabled("  (empty)");
                                ImGui::EndChild();
                            }
                            ImGui::TreePop();
                        } else { arch_pub_open = false; }
                        } // pub block

                        ImGui::Spacing();

                        // ── Share (다운로드된 파일) ───────────────────────
                        ImGui::SetNextItemOpen(arch_share_open, ImGuiCond_Always);
                        {
                        int dl_cnt = (int)downloaded_files.size();
                        if(ImGui::TreeNodeEx("##share_dl_node", ImGuiTreeNodeFlags_None,
                                "Share  (%d)", dl_cnt)){
                            arch_share_open = true;
                            draw_file_list("##share_dl_list",
                                share_iq_files, share_audio_files,
                                BEWEPaths::share_iq_dir(), BEWEPaths::share_audio_dir(),
                                23000);
                            ImGui::TreePop();
                        } else { arch_share_open = false; }
                        } // share_dl block

                        ImGui::Unindent(8.f);
                    }
                    ImGui::Spacing();

                    ImGui::EndChild(); // ##link_scroll
                    ImGui::EndTabItem();
                } // LINK tab

                ImGui::EndTabBar();
                } // BeginTabBar
                ImGui::PopStyleColor(3); // Tab colors

                ImGui::End();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar();
            }

                        if(v.sa_panel_open){
                if(v.sa_mode || v.sa_computing.load()){
                    // 로딩 애니메이션: "Loading ." → "Loading .." → "Loading ..."
                    v.sa_anim_timer += io.DeltaTime;
                    int dots = ((int)(v.sa_anim_timer / 0.5f) % 3) + 1;
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Loading %.*s", dots, "...");
                    ImVec2 msz = ImGui::CalcTextSize(msg);
                    dl->AddText(ImVec2(rpx+(rp_w-msz.x)/2, rp_content_y+(rp_content_h-msz.y)/2),
                                IM_COL32(180,180,100,255), msg);
                } else if(v.sa_texture){
                    // ── SA 텍스처 표시 (줌 뷰 적용) ─────────────────────
                    float sa_x0 = rpx, sa_y0 = rp_content_y;
                    float sa_x1 = disp_w, sa_y1 = content_y + content_h;
                    float sa_w  = sa_x1 - sa_x0;
                    float sa_h  = sa_y1 - sa_y0;

                    ImTextureID tid = (ImTextureID)(intptr_t)v.sa_texture;
                    dl->AddImage(tid, ImVec2(sa_x0, sa_y0), ImVec2(sa_x1, sa_y1),
                                 ImVec2(v.sa_view_x0, v.sa_view_y0),
                                 ImVec2(v.sa_view_x1, v.sa_view_y1),
                                 IM_COL32(255,255,255,255));

                    // ── 스크롤 휠: 줌 ─────────────────────────────────────
                    ImVec2 mp = io.MousePos;
                    bool mouse_in_sa = (mp.x >= sa_x0 && mp.x < sa_x1 &&
                                        mp.y >= sa_y0 && mp.y < sa_y1);
                    if(mouse_in_sa && io.MouseWheel != 0.f){
                        float zoom_factor = (io.MouseWheel > 0) ? 0.8f : 1.25f;
                        // 마우스 위치 기준 UV
                        float mu = v.sa_view_x0 + (mp.x - sa_x0) / sa_w * (v.sa_view_x1 - v.sa_view_x0);
                        float mv = v.sa_view_y0 + (mp.y - sa_y0) / sa_h * (v.sa_view_y1 - v.sa_view_y0);
                        // 시간축(Y)만 줌 (주파수축은 고정 BW이므로)
                        float new_half = (v.sa_view_y1 - v.sa_view_y0) * 0.5f * zoom_factor;
                        v.sa_view_y0 = mv - new_half;
                        v.sa_view_y1 = mv + new_half;
                        // 클램프
                        if(v.sa_view_y0 < 0.f){ v.sa_view_y1 -= v.sa_view_y0; v.sa_view_y0 = 0.f; }
                        if(v.sa_view_y1 > 1.f){ v.sa_view_y0 -= (v.sa_view_y1 - 1.f); v.sa_view_y1 = 1.f; }
                        v.sa_view_y0 = std::max(0.f, v.sa_view_y0);
                        v.sa_view_y1 = std::min(1.f, v.sa_view_y1);
                        if(v.sa_view_y1 - v.sa_view_y0 < 0.01f){
                            float mid = (v.sa_view_y0 + v.sa_view_y1) * 0.5f;
                            v.sa_view_y0 = mid - 0.005f; v.sa_view_y1 = mid + 0.005f;
                        }
                    }

                    // ── Ctrl+우클릭 드래그: 범위 선택 ─────────────────────
                    if(mouse_in_sa && io.KeyCtrl &&
                       ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                        float u = (mp.x - sa_x0) / sa_w;
                        float vv = (mp.y - sa_y0) / sa_h;
                        // UV → 텍스처 공간
                        v.sa_sel_drag_ox = v.sa_view_x0 + u * (v.sa_view_x1 - v.sa_view_x0);
                        v.sa_sel_drag_oy = v.sa_view_y0 + vv * (v.sa_view_y1 - v.sa_view_y0);
                        v.sa_sel_x0 = v.sa_sel_drag_ox; v.sa_sel_x1 = v.sa_sel_drag_ox;
                        v.sa_sel_y0 = v.sa_sel_drag_oy; v.sa_sel_y1 = v.sa_sel_drag_oy;
                        v.sa_sel_dragging = true;
                        v.sa_sel_active   = false;
                    }
                    if(v.sa_sel_dragging){
                        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){
                            float u  = (mp.x - sa_x0) / sa_w;
                            float vv = (mp.y - sa_y0) / sa_h;
                            float cu = v.sa_view_x0 + u  * (v.sa_view_x1 - v.sa_view_x0);
                            float cv = v.sa_view_y0 + vv * (v.sa_view_y1 - v.sa_view_y0);
                            v.sa_sel_x0 = std::min(v.sa_sel_drag_ox, cu);
                            v.sa_sel_x1 = std::max(v.sa_sel_drag_ox, cu);
                            v.sa_sel_y0 = std::min(v.sa_sel_drag_oy, cv);
                            v.sa_sel_y1 = std::max(v.sa_sel_drag_oy, cv);
                        } else {
                            v.sa_sel_dragging = false;
                            if(v.sa_sel_x1 - v.sa_sel_x0 > 0.005f ||
                               v.sa_sel_y1 - v.sa_sel_y0 > 0.005f)
                                v.sa_sel_active = true;
                        }
                    }

                    // 선택 범위 사각형 표시
                    if((v.sa_sel_active || v.sa_sel_dragging) &&
                       v.sa_sel_x0 < v.sa_sel_x1 && v.sa_sel_y0 < v.sa_sel_y1){
                        // 텍스처 UV → 화면 좌표
                        float vr_w = v.sa_view_x1 - v.sa_view_x0;
                        float vr_h = v.sa_view_y1 - v.sa_view_y0;
                        float rx0 = sa_x0 + (v.sa_sel_x0 - v.sa_view_x0) / vr_w * sa_w;
                        float rx1 = sa_x0 + (v.sa_sel_x1 - v.sa_view_x0) / vr_w * sa_w;
                        float ry0 = sa_y0 + (v.sa_sel_y0 - v.sa_view_y0) / vr_h * sa_h;
                        float ry1 = sa_y0 + (v.sa_sel_y1 - v.sa_view_y0) / vr_h * sa_h;
                        dl->AddRectFilled(ImVec2(rx0,ry0), ImVec2(rx1,ry1), IM_COL32(255,200,0,40));
                        dl->AddRect(ImVec2(rx0,ry0), ImVec2(rx1,ry1), IM_COL32(255,200,0,200), 0.f, 0, 1.5f);
                        // 복조 모드 표시
                        if(v.sa_demod_mode != 0){
                            const char* dm = (v.sa_demod_mode == 1) ? "AM" : "FM";
                            ImVec2 dmsz = ImGui::CalcTextSize(dm);
                            dl->AddText(ImVec2(rx0 + 2, ry0 + 2), IM_COL32(255,220,60,255), dm);
                            (void)dmsz;
                        }
                    }

                    // ── A/F 키: 복조 모드 선택 (SA 패널 마우스 위 + sel_active) ─
                    if(mouse_in_sa && !ImGui::GetIO().WantTextInput){
                        if(ImGui::IsKeyPressed(ImGuiKey_A, false)) v.sa_demod_mode = 1; // AM
                        if(ImGui::IsKeyPressed(ImGuiKey_F, false)) v.sa_demod_mode = 2; // FM
                        // 스페이스바: 복조 재생 시작
                        if(ImGui::IsKeyPressed(ImGuiKey_Space, false) &&
                           v.sa_sel_active && v.sa_demod_mode != 0 && !v.sa_playing.load()){
                            if(v.sa_play_thread.joinable()) v.sa_play_thread.join();
                            v.sa_play_thread = std::thread([&v](){ v.sa_play_demod(); });
                        }
                    }

                    // ── Freq/Time 오버레이 (마우스 커서 위치 정보) ────────
                    if(mouse_in_sa && v.sa_center_freq_hz > 0){
                        float u  = (mp.x - sa_x0) / sa_w;
                        float vv = (mp.y - sa_y0) / sa_h;
                        // 텍스처 내 UV
                        float tu = v.sa_view_x0 + u  * (v.sa_view_x1 - v.sa_view_x0);
                        float tv = v.sa_view_y0 + vv * (v.sa_view_y1 - v.sa_view_y0);

                        // 주파수 계산: 텍스처 X → bin → 실제 주파수
                        // bin 0 = cf - sr/2, bin N-1 = cf + sr/2 (FFT shift 후)
                        if(v.sa_actual_fft_n > 0 && v.sa_sample_rate > 0){
                            double cf_hz = (double)v.sa_center_freq_hz;
                            double bw_hz = (double)v.sa_sample_rate;
                            double freq_hz = cf_hz - bw_hz * 0.5 + tu * bw_hz;
                            double freq_mhz = freq_hz / 1e6;

                            // 시간 계산: 텍스처 Y → 행 → 시간
                            // total_rows 행이 n_samples/sa_sample_rate 초에 해당
                            double row_sec = 0.0;
                            if(v.sa_sample_rate > 0 && v.sa_actual_fft_n > 0)
                                row_sec = (double)v.sa_actual_fft_n / (double)v.sa_sample_rate;
                            double t_offset = tv * v.sa_total_rows * row_sec;
                            time_t t_abs = (time_t)(v.sa_start_time + (int64_t)t_offset);
                            struct tm* tmv = localtime(&t_abs);
                            char time_buf[16]="--:--:--";
                            if(tmv) strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tmv);

                            // 오버레이 텍스트 (우측 상단)
                            char freq_buf[32]; snprintf(freq_buf, sizeof(freq_buf), "Freq : %.3fMHz", freq_mhz);
                            char time_txt[32]; snprintf(time_txt, sizeof(time_txt), "Time : %s", time_buf);
                            float fw = std::max(ImGui::CalcTextSize(freq_buf).x,
                                                ImGui::CalcTextSize(time_txt).x);
                            float fh = ImGui::GetFontSize() * 2.5f;
                            float ox = sa_x1 - fw - 10.f, oy = sa_y0 + 6.f;
                            dl->AddRectFilled(ImVec2(ox-4,oy-2), ImVec2(ox+fw+4,oy+fh),
                                             IM_COL32(0,0,0,160), 4.f);
                            dl->AddText(ImVec2(ox, oy),
                                        IM_COL32(200,230,255,255), freq_buf);
                            dl->AddText(ImVec2(ox, oy + ImGui::GetFontSize() + 2.f),
                                        IM_COL32(200,230,255,255), time_txt);
                        }
                    }
                } else {
                    // 안내
                    const char* msg = "Drag region here";
                    ImVec2 msz = ImGui::CalcTextSize(msg);
                    dl->AddText(ImVec2(rpx+(rp_w-msz.x)/2, rp_content_y+(rp_content_h-msz.y)/2),
                                IM_COL32(100,100,120,255), msg);
                }
            }

            // ── BOARD 패널 ────────────────────────────────────────────────
            if(board_open){
                float px=rpx, py=rp_content_y, pw=disp_w-rpx, ph=rp_content_h;
                ImGui::SetNextWindowPos(ImVec2(px,py));
                ImGui::SetNextWindowSize(ImVec2(pw,ph));
                ImGui::SetNextWindowBgAlpha(0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8,6));
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
                ImGui::Begin("##board_panel", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                    ImGuiWindowFlags_NoDecoration);

                ImGui::BeginChild("##board_scroll", ImVec2(0,0), false,
                    ImGuiWindowFlags_HorizontalScrollbar);

                // ── 1. Operators ─────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Operators")){
                    ImGui::Indent(6.f);
                    // HOST 자신
                    {
                        const char* my_id = v.net_srv ? login_get_id()
                                          : (v.net_cli ? v.net_cli->my_name : login_get_id());
                        const char* badge = v.net_cli ? "[JOIN]" : (v.net_srv ? "[HOST]" : "[LOCAL]");
                        ImVec4 bcol = v.net_cli ? ImVec4(0.7f,0.92f,0.7f,1.f)
                                                : ImVec4(0.4f,0.85f,1.f,1.f);
                        ImGui::TextColored(bcol, "%s %s", badge, my_id);
                    }
                    // HOST 모드: JOIN 목록
                    if(v.net_srv){
                        auto ops = v.net_srv->get_operators();
                        for(auto& op : ops){
                            ImGui::TextColored(ImVec4(0.7f,0.92f,0.7f,1.f),
                                "[JOIN] %s  [T%d]", op.name, op.tier);
                        }
                        if(ops.empty()) ImGui::TextDisabled("  (no operators)");
                    } else if(v.net_cli){
                        std::lock_guard<std::mutex> lk(v.net_cli->op_mtx);
                        for(int i=0;i<(int)v.net_cli->op_list.count;i++){
                            auto& op = v.net_cli->op_list.ops[i];
                            bool is_host = (op.index==0);
                            ImVec4 oc = is_host ? ImVec4(0.4f,0.85f,1.f,1.f)
                                                : ImVec4(0.7f,0.92f,0.7f,1.f);
                            ImGui::TextColored(oc, "%s %s  [T%d]",
                                is_host?"[HOST]":"[JOIN]", op.name, op.tier);
                        }
                    }
                    ImGui::Unindent(6.f);
                }
                ImGui::Spacing();

                // ── 2. Channels ───────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Channels")){
                    ImGui::Indent(6.f);
                    bool any_ch = false;
                    for(int ci=0;ci<MAX_CHANNELS;ci++){
                        const Channel& ch = v.channels[ci];
                        if(!ch.filter_active) continue;
                        any_ch = true;
                        float cf = (ch.s + ch.e) * 0.5f;
                        float bw = fabsf(ch.e - ch.s) * 1000.f; // kHz
                        // 모드 문자열
                        const char* mode_str = "NONE";
                        switch(ch.mode){
                            case Channel::DM_AM:    mode_str="AM";    break;
                            case Channel::DM_FM:    mode_str="FM";    break;
                            case Channel::DM_MAGIC: mode_str="MAGIC"; break;
                            default: break;
                        }
                        // 복조 실행 중 여부
                        bool dem_active = ch.dem_run.load();
                        ImU32 col = dem_active ? IM_COL32(80,220,80,255) : IM_COL32(160,160,160,255);
                        ImGui::PushID(ci+40000);
                        ImGui::PushStyleColor(ImGuiCol_Text, col);
                        char ch_line[80];
                        snprintf(ch_line,sizeof(ch_line),"CH%d  %.4f MHz  %.0f kHz  [%s]",
                                 v.freq_sorted_display_num(ci), (double)cf, (double)bw, mode_str);
                        ImGui::TextUnformatted(ch_line);
                        ImGui::PopStyleColor();
                        // 생성자
                        if(ch.owner[0]){
                            ImGui::SameLine(0,6);
                            ImGui::TextDisabled("by %s", ch.owner);
                        }
                        // 복조 중이면 수신 중인 오퍼레이터 표시
                        if(dem_active){
                            uint32_t mask = v.net_srv
                                ? ch.audio_mask.load()
                                : v.srv_audio_mask[ci];
                            if(mask){
                                char recv_str[64] = "  Recv:";
                                if(mask & 0x1) strncat(recv_str," HOST",sizeof(recv_str)-strlen(recv_str)-1);
                                if(v.net_srv){
                                    auto ops2 = v.net_srv->get_operators();
                                    for(auto& op2 : ops2){
                                        if(mask & (1u<<op2.index))
                                            strncat(recv_str, (" "+std::string(op2.name)).c_str(),
                                                    sizeof(recv_str)-strlen(recv_str)-1);
                                    }
                                }
                                ImGui::TextDisabled("%s", recv_str);
                            }
                        }
                        ImGui::PopID();
                    }
                    if(!any_ch) ImGui::TextDisabled("  (no active channels)");
                    ImGui::Unindent(6.f);
                }
                ImGui::Spacing();

                // ── 3. Signal Activity ────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Signal Activity")){
                    ImGui::Indent(6.f);
                    bool any_sig = false;
                    for(int ci=0;ci<MAX_CHANNELS;ci++){
                        Channel& ch = v.channels[ci];
                        if(!ch.filter_active) continue;
                        if(!ch.dem_run.load()) continue;
                        any_sig = true;
                        // 오디오 링버퍼에 데이터가 있으면 신호 있음
                        bool has_signal = (ch.audio_avail() > 0);
                        ImU32 sig_col = has_signal ? IM_COL32(80,255,100,255) : IM_COL32(100,100,100,200);
                        const char* sig_str = has_signal ? "SIG" : "---";
                        ImGui::PushID(ci+41000);
                        ImGui::PushStyleColor(ImGuiCol_Text, sig_col);
                        ImGui::Text("CH%d  [%s]  %.4f MHz", v.freq_sorted_display_num(ci), sig_str, (double)(ch.s+ch.e)*0.5f);
                        ImGui::PopStyleColor();
                        ImGui::PopID();
                    }
                    if(!any_sig) ImGui::TextDisabled("  (no demod running)");
                    ImGui::Unindent(6.f);
                }
                ImGui::Spacing();

                // ── 4. Recordings ─────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Recordings")){
                    ImGui::Indent(6.f);
                    // 진행 중
                    bool any_rec = false;
                    {
                        std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                        for(auto& re : v.rec_entries){
                            if(re.finished) continue;
                            any_rec = true;
                            float elapsed = std::chrono::duration<float>(
                                std::chrono::steady_clock::now()-re.t_start).count();
                            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255,100,100,255));
                            ImGui::Text("[REC]  %s  %.0fs", re.filename.c_str(), (double)elapsed);
                            ImGui::PopStyleColor();
                        }
                    }
                    // Record 폴더 파일 수
                    int rec_iq_cnt   = (int)rec_iq_files.size();
                    int rec_aud_cnt  = (int)rec_audio_files.size();
                    int priv_iq_cnt  = (int)priv_iq_files.size();
                    int priv_aud_cnt = (int)priv_audio_files.size();
                    int pub_cnt2     = (int)shared_files.size();
                    int shr_cnt      = (int)downloaded_files.size();
                    ImGui::Spacing();
                    ImGui::TextDisabled("  Record   IQ:%d  Audio:%d", rec_iq_cnt,  rec_aud_cnt);
                    ImGui::TextDisabled("  Private  IQ:%d  Audio:%d", priv_iq_cnt, priv_aud_cnt);
                    ImGui::TextDisabled("  Public   %d",  pub_cnt2);
                    ImGui::TextDisabled("  Share    %d",  shr_cnt);
                    if(!any_rec && rec_iq_cnt+rec_aud_cnt==0)
                        ImGui::TextDisabled("  (no active recordings)");
                    ImGui::Unindent(6.f);
                }

                ImGui::EndChild();
                ImGui::End();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar();
            }
        }

        // ── Bottom bar ────────────────────────────────────────────────────
        float bot_y=disp_h-TOPBAR_H;
        dl->AddRectFilled(ImVec2(0,bot_y),ImVec2(disp_w,disp_h),IM_COL32(30,30,30,255));
        dl->AddLine(ImVec2(0,bot_y),ImVec2(disp_w,bot_y),IM_COL32(60,60,60,255),1);
        {
            float ty_b=bot_y+(TOPBAR_H-ImGui::GetFontSize())/2;

            // ── 중앙: CPU온도  HH:MM:SS  SDR온도 ─────────────────────────
            {
                // CPU 온도 (2초마다 백그라운드 갱신)
                // sysfs hwmon 방식: sensors 불필요, AppImage 호환
                static char  cpu_temp_str[16]  = "";
                static float cpu_temp_timer    = 1.f;
                static std::atomic<bool> cpu_fetching{false};
                static std::mutex        cpu_temp_mtx;
                cpu_temp_timer += io.DeltaTime;
                if(cpu_temp_timer >= 1.0f && !cpu_fetching.load()){
                    cpu_temp_timer = 0.f;
                    cpu_fetching.store(true);
                    std::thread([](){
                        char tmp[16] = "";
                        // 1) hwmon에서 coretemp 드라이버의 temp1_input (Package id 0) 탐색
                        bool found = false;
                        for(int i = 0; i < 32 && !found; i++){
                            char name_path[64];
                            snprintf(name_path, sizeof(name_path),
                                     "/sys/class/hwmon/hwmon%d/name", i);
                            FILE* fn = fopen(name_path, "r");
                            if(!fn) continue;
                            char name[32] = {};
                            fgets(name, sizeof(name), fn);
                            fclose(fn);
                            // coretemp: temp1 = Package id 0
                            if(strncmp(name, "coretemp", 8) == 0){
                                char tp[80];
                                snprintf(tp, sizeof(tp),
                                         "/sys/class/hwmon/hwmon%d/temp1_input", i);
                                FILE* ft = fopen(tp, "r");
                                if(ft){
                                    int milli = 0;
                                    if(fscanf(ft, "%d", &milli) == 1)
                                        snprintf(tmp, sizeof(tmp), "%.0f\xC2\xB0""C",
                                                 milli / 1000.f);
                                    fclose(ft);
                                    found = true;
                                }
                            }
                        }
                        // 2) fallback: /sys/class/thermal/thermal_zone* 에서 x86_pkg_temp
                        if(!found){
                            for(int i = 0; i < 16 && !found; i++){
                                char tp[80], tt[80];
                                snprintf(tt, sizeof(tt),
                                         "/sys/class/thermal/thermal_zone%d/type", i);
                                FILE* ft = fopen(tt, "r");
                                if(!ft) continue;
                                char zone_type[32] = {};
                                fgets(zone_type, sizeof(zone_type), ft);
                                fclose(ft);
                                if(strncmp(zone_type, "x86_pkg_temp", 12) == 0 ||
                                   strncmp(zone_type, "TCPU", 4) == 0){
                                    snprintf(tp, sizeof(tp),
                                             "/sys/class/thermal/thermal_zone%d/temp", i);
                                    FILE* fv = fopen(tp, "r");
                                    if(fv){
                                        int milli = 0;
                                        if(fscanf(fv, "%d", &milli) == 1)
                                            snprintf(tmp, sizeof(tmp), "%.0f\xC2\xB0""C",
                                                     milli / 1000.f);
                                        fclose(fv);
                                        found = true;
                                    }
                                }
                            }
                        }
                        // 3) fallback: acpitz (노트북 등)
                        if(!found){
                            FILE* fv = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
                            if(fv){
                                int milli = 0;
                                if(fscanf(fv, "%d", &milli) == 1 && milli > 0)
                                    snprintf(tmp, sizeof(tmp), "%.0f\xC2\xB0""C",
                                             milli / 1000.f);
                                fclose(fv);
                            }
                        }
                        { std::lock_guard<std::mutex> lk(cpu_temp_mtx);
                          strncpy(cpu_temp_str, tmp, sizeof(cpu_temp_str)-1); }
                        cpu_fetching.store(false);
                    }).detach();
                }

                // SDR 온도
                // - LOCAL/HOST + BladeRF: libbladeRF 직접 쿼리 (3초마다)
                // - LOCAL/HOST + RTL-SDR: 온도 API 없음 → "00°C"
                // - JOIN: HOST HEARTBEAT에서 수신한 remote_sdr_temp_c 사용
                static char  sdr_temp_str[16]  = "";
                static float sdr_temp_timer    = 1.f;
                static std::mutex sdr_temp_mtx;
                sdr_temp_timer += io.DeltaTime;
                if(v.remote_mode && v.net_cli){
                    // JOIN: HOST에서 받은 SDR 온도
                    uint8_t rt = v.net_cli->remote_sdr_temp_c.load();
                    std::lock_guard<std::mutex> lk(sdr_temp_mtx);
                    snprintf(sdr_temp_str, sizeof(sdr_temp_str),
                             "%02d\xC2\xB0""C", (int)rt);
                } else if(sdr_temp_timer >= 3.0f){
                    sdr_temp_timer = 0.f;
                    if(v.dev_blade){
                        float sdr_t = 0.f;
                        if(bladerf_get_rfic_temperature(v.dev_blade, &sdr_t) == 0){
                            std::lock_guard<std::mutex> lk(sdr_temp_mtx);
                            snprintf(sdr_temp_str, sizeof(sdr_temp_str),
                                     "%.0f\xC2\xB0""C", sdr_t);
                        }
                    } else if(v.dev_rtl){
                        // RTL-SDR: 온도 API 미지원
                        std::lock_guard<std::mutex> lk(sdr_temp_mtx);
                        snprintf(sdr_temp_str, sizeof(sdr_temp_str), "00\xC2\xB0""C");
                    }
                }

                // 시계
                time_t tnow = time(nullptr);
                struct tm tlocal{}; localtime_r(&tnow, &tlocal);
                char clock_str[16];
                strftime(clock_str, sizeof(clock_str), "%H:%M:%S", &tlocal);

                // 조합 문자열: "CPU: ##°C  HH:MM:SS  SDR: ##°C"
                char center_str[80] = {};
                {
                    std::lock_guard<std::mutex> lk(cpu_temp_mtx);
                    std::lock_guard<std::mutex> lk2(sdr_temp_mtx);
                    if(cpu_temp_str[0] && sdr_temp_str[0])
                        snprintf(center_str, sizeof(center_str),
                                 "CPU: %s  %s  SDR: %s", cpu_temp_str, clock_str, sdr_temp_str);
                    else if(cpu_temp_str[0])
                        snprintf(center_str, sizeof(center_str),
                                 "CPU: %s  %s", cpu_temp_str, clock_str);
                    else if(sdr_temp_str[0])
                        snprintf(center_str, sizeof(center_str),
                                 "%s  SDR: %s", clock_str, sdr_temp_str);
                    else
                        snprintf(center_str, sizeof(center_str), "%s", clock_str);
                }
                ImVec2 csz = ImGui::CalcTextSize(center_str);
                dl->AddText(ImVec2((disp_w - csz.x) / 2.f, ty_b),
                            IM_COL32(200,200,200,255), center_str);
            }

            // ── 좌측: 타임머신 오프셋 (활성 시만 표시) ───────────────────
            if(v.tm_active.load()){
                char tm_txt[48];
                if(v.tm_offset<=0.0f) snprintf(tm_txt,sizeof(tm_txt),"LIVE");
                else snprintf(tm_txt,sizeof(tm_txt),"-%.1f sec",v.tm_offset);
                dl->AddText(ImVec2(8,ty_b),IM_COL32(255,200,50,255),tm_txt);
            }

            // ── 우측: 상태 인디케이터 (오른쪽→왼쪽) ─────────────────────
            // 초록=활성, 빨간=비활성
            bool capturing       = !v.capture_pause.load();
            bool spectrum_paused = v.spectrum_pause.load();
            bool fft_panel_on    = v.render_visible.load(); // 수직바: false이면 FFT 연산 스킵
            bool tm_on           = v.tm_active.load();
            // IQ: JOIN이면 HOST의 iq 상태를 표시, LOCAL/HOST는 자체 상태
            bool iq_on = (v.remote_mode && v.net_cli)
                         ? (v.net_cli->remote_iq_on.load() != 0)
                         : v.tm_iq_on.load();

            // ── FFT / WF LED ──────────────────────────────────────────────
            // LOCAL/HOST: fft_panel_on && capturing && !spectrum_paused
            // JOIN: fft_seq가 최근 3초 이내에 변한 경우 → 실제 데이터 수신 중
            int fft_led, wf_led; // 0=빨간, 1=초록
            if(v.remote_mode && v.net_cli){
                static int   join_last_fft_seq = -1;
                static float join_fft_stall    = 0.f;
                int cur_seq = v.net_cli->fft_seq.load();
                if(cur_seq != join_last_fft_seq){ join_last_fft_seq=cur_seq; join_fft_stall=0.f; }
                else join_fft_stall += io.DeltaTime;
                bool connected  = v.net_cli->is_connected();
                int  hs         = v.net_cli->host_state.load();
                bool host_paused = (hs == 2);  // HOST spectrum pause / 수직바 끝
                bool fft_recv   = connected && (join_fft_stall < 3.f);
                // host_state 2 = HOST 의도적 정지 → 노란, 연결 끊김/stall → 빨간, 수신 중 → 초록
                // JOIN 로컬 수직바(fft_panel_on=false) → FFT/WF 빨간
                // JOIN 로컬 수평바(wf_area_visible=false) → WF만 빨간
                bool local_fft_hidden = !fft_panel_on;   // 수직바 왼쪽 끝
                bool local_wf_hidden  = !v.wf_area_visible.load(); // 수평바 아래 끝
                if(!connected || local_fft_hidden)
                                      { fft_led = 0; wf_led = 0; } // 빨간: 연결 없음 or 수직바
                else if(host_paused)  { fft_led = 2; wf_led = local_wf_hidden ? 0 : 2; } // 노란(HOST멈춤)
                else if(fft_recv)     { fft_led = 1; wf_led = local_wf_hidden ? 0 : 1; } // 초록: 수신 중
                else                  { fft_led = 0; wf_led = 0; } // 빨간: stall
            } else {
                bool fft_active = fft_panel_on && capturing && !spectrum_paused;
                fft_led = fft_active ? 1 : 0;
                // WF: FFT 활성 + 워터폴 영역이 실제 표시 중 (wf_area_visible: 수평바 포함)
                wf_led  = (fft_active && v.wf_area_visible.load()) ? 1 : 0;
            }

            // SDR: 스트리밍 실제 끊김 여부
            static int   sdr_last_fft_idx = -1;
            static float sdr_stall_timer  = 0.f;
            if(!v.remote_mode){
                if(v.current_fft_idx != sdr_last_fft_idx){
                    sdr_last_fft_idx = v.current_fft_idx;
                    sdr_stall_timer  = 0.f;
                } else if(!spectrum_paused && capturing && fft_panel_on){
                    sdr_stall_timer += io.DeltaTime;
                }
                if(spectrum_paused || !capturing || !fft_panel_on) sdr_stall_timer = 0.f;
            } else {
                sdr_last_fft_idx = -1;
                sdr_stall_timer  = 0.f;
            }
            bool sdr_on;
            if(v.remote_mode && v.net_cli){
                double lht = v.net_cli->last_heartbeat_time.load();
                bool hb_received = (lht > 0.0);  // 한 번이라도 HB 수신
                bool hb_ok = hb_received && (glfwGetTime() - lht) < 5.0;
                bool sdr_ok = v.net_cli->remote_sdr_state.load() == 0;
                sdr_on = v.net_cli->is_connected() && hb_ok && sdr_ok;
            } else {
                bool stream_err = v.sdr_stream_error.load();
                sdr_on = !v.remote_mode && !stream_err && capturing && (sdr_stall_timer < 2.0f);
            }

            // LINK: HOST=Central Server 연결 상태, JOIN=HOST 연결 상태, LOCAL=꺼짐
            int link_state = 0; // 0=빨간, 1=초록, 2=노란
            if(v.net_srv){
                // HOST: chassis 2 reset 중이면 노란, Central Server 연결 확인
                if(v.net_bcast_pause.load(std::memory_order_relaxed))
                    link_state = 2;
                else if(central_cli.is_central_connected())
                    link_state = 1;
                else
                    link_state = 0; // Central Server 미연결
            } else if(v.net_cli){
                bool connected = v.net_cli->is_connected();
                int  hs        = v.net_cli->host_state.load();
                double lht2    = v.net_cli->last_heartbeat_time.load();
                // steady_clock 기반으로 비교 (glfwGetTime과 시계 불일치 방지)
                double now_sc  = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                bool hb_ok     = (lht2 > 0.0) ? (now_sc - lht2) < 5.0 : connected;
                if(!connected || !hb_ok) link_state = 0;
                else if(hs==1) link_state = 2; // 노란: HOST chassis 리셋 중
                else           link_state = 1; // 초록: 연결됨 + heartbeat 수신 중
            }

            // ── AUD LED: 오디오 출력 상태 ────────────────────────────────
            // LOCAL/HOST: 복조채널 없음=빨간, 채널있고 스컬치 미통과=노란, 스컬치 통과+출력=초록
            // JOIN: 채널 없음=빨간, 채널있으나 수신 안됨=노란, 오디오 수신 중=초록
            int aud_led = 0; // 0=빨간, 1=초록, 2=노란
            if(v.remote_mode && v.net_cli){
                // JOIN: 채널 존재 여부 + 실제 오디오 수신
                bool any_ch = false;
                bool audio_active = false;
                for(int ci=0;ci<MAX_CHANNELS;ci++){
                    if(v.channels[ci].mode == Channel::DM_NONE) continue;
                    any_ch = true;
                    // 뮤트(local_ch_out==3)가 아니고 NetAudioRing에 데이터 있으면 활성
                    if(v.local_ch_out[ci] != 3){
                        auto& ar = v.net_cli->audio[ci];
                        size_t wp = ar.wp.load(std::memory_order_acquire);
                        size_t rp = ar.rp.load(std::memory_order_relaxed);
                        if(wp != rp) audio_active = true;
                    }
                }
                if(!any_ch)           aud_led = 0; // 빨간: 채널 없음
                else if(audio_active) aud_led = 1; // 초록: 오디오 수신 중
                else                  aud_led = 2; // 노란: 채널 있으나 수신 없음(스컬치 등)
            } else {
                // LOCAL/HOST: dem_run 채널 존재 + sq_gate + 뮤트 아님
                bool any_dem = false;
                bool sq_pass = false;
                bool muted_all = true;
                for(int ci=0;ci<MAX_CHANNELS;ci++){
                    if(!v.channels[ci].dem_run.load()) continue;
                    any_dem = true;
                    if(v.local_ch_out[ci] != 3) muted_all = false;
                    if(v.channels[ci].sq_gate.load() && v.local_ch_out[ci] != 3)
                        sq_pass = true;
                }
                if(!any_dem)        aud_led = 0; // 빨간: 복조채널 없음
                else if(sq_pass)    aud_led = 1; // 초록: 스컬치 통과 + 출력 중
                else                aud_led = 2; // 노란: 채널 있으나 스컬치 미통과 or 전체 뮤트
            }

            // ── SDR 오류 시 FFT/WF/AUD/IQ 모두 빨간 (LINK/TM/SDR은 독립) ──
            if(!v.remote_mode && v.sdr_stream_error.load()){
                fft_led = 0; wf_led = 0; aud_led = 0;
                iq_on = false;
            }
            // ── /rx stop: SDR/FFT/WF/AUD/IQ 모두 빨간 ──
            if(!v.remote_mode && v.rx_stopped.load()){
                sdr_on = false; fft_led = 0; wf_led = 0; aud_led = 0;
                iq_on = false;
            }

            // ── JOIN: HOST 연결 끊김 시 AUD/IQ 빨간 ──
            if(v.remote_mode && link_state == 0){
                aud_led = 0;
                iq_on = false;
            }

            // 인디케이터 그리기 헬퍼: state 0=빨간, 1=초록, 2=노란
            auto draw_ind=[&](float rx, const char* txt, int state) -> float {
                ImVec2 sz=ImGui::CalcTextSize(txt);
                float x=rx-sz.x;
                ImU32 col = (state==1) ? IM_COL32(80,220,80,255)
                          : (state==2) ? IM_COL32(255,200,0,255)
                          :              IM_COL32(220,60,60,255);
                if(state>0) dl->AddText(ImVec2(x+1,ty_b),col,txt); // bold shadow
                dl->AddText(ImVec2(x,ty_b),col,txt);
                return x-14.0f;
            };
            // 클릭 가능 인디케이터 헬퍼 (state 0=빨간, 1=초록, 2=노란)
            auto click_ind=[&](float& rx2, const char* txt, int state) -> bool {
                ImVec2 sz=ImGui::CalcTextSize(txt);
                float x=rx2-sz.x;
                ImU32 col = (state==1) ? IM_COL32(80,220,80,255)
                          : (state==2) ? IM_COL32(255,200,0,255)
                          :              IM_COL32(220,60,60,255);
                if(state>0) dl->AddText(ImVec2(x+1,ty_b),col,txt);
                dl->AddText(ImVec2(x,ty_b),col,txt);
                bool clicked=ImGui::IsMouseClicked(ImGuiMouseButton_Left)&&
                    io.MousePos.x>=x&&io.MousePos.x<=x+sz.x&&
                    io.MousePos.y>=ty_b&&io.MousePos.y<=ty_b+sz.y;
                rx2=x-14.0f;
                return clicked;
            };

            // 오른쪽→왼쪽: TM IQ AUD WF FFT LINK SDR
            float rx=disp_w-8.0f;

            // TM
            rx=draw_ind(rx,"TM", tm_on ? 1 : 0);

            // IQ (클릭 가능)
            if(click_ind(rx,"IQ", iq_on ? 1 : 0)){
                if(v.remote_mode && v.net_cli) v.net_cli->cmd_toggle_tm_iq();
                else {
                    bool cur=v.tm_iq_on.load();
                    if(cur){ v.tm_iq_on.store(false); v.tm_add_event_tag(2); v.tm_iq_was_stopped=true;
                        if(v.net_srv) v.net_srv->broadcast_wf_event(0,(int64_t)time(nullptr),2,"IQ Stop"); }
                    else { if(v.tm_iq_was_stopped){v.tm_iq_close();v.tm_iq_was_stopped=false;}
                        v.tm_iq_open(); if(v.tm_iq_file_ready){
                            v.tm_iq_on.store(true); v.tm_add_event_tag(1);
                            if(v.net_srv) v.net_srv->broadcast_wf_event(0,(int64_t)time(nullptr),1,"IQ Start"); }}
                }
            }

            // AUD (3색)
            rx=draw_ind(rx,"AUD", aud_led);

            // WF (클릭 가능, 3색→초록/빨간)
            if(click_ind(rx,"WF", wf_led)){
                v.spectrum_pause.store(!v.spectrum_pause.load());
            }

            // FFT (클릭 가능, 3색→초록/빨간)
            if(click_ind(rx,"FFT", fft_led)){
                v.spectrum_pause.store(!v.spectrum_pause.load());
            }

            // LINK (3색: draw_ind 사용)
            rx=draw_ind(rx,"LINK", link_state);
            // LINK 클릭: capture_pause 토글
            {
                ImVec2 lsz=ImGui::CalcTextSize("LINK");
                float lx=rx+14.0f; // draw_ind가 이미 이동시킴, 실제 그려진 위치 복원
                bool lclicked=ImGui::IsMouseClicked(ImGuiMouseButton_Left)&&
                    io.MousePos.x>=lx&&io.MousePos.x<=lx+lsz.x&&
                    io.MousePos.y>=ty_b&&io.MousePos.y<=ty_b+lsz.y;
                if(lclicked){
                    bool np=!v.capture_pause.load();
                    if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_capture_pause(np);
                    else { v.capture_pause.store(np); }
                }
            }

            // SDR
            rx=draw_ind(rx,"SDR", sdr_on ? 1 : 0);

        }
        ImGui::End();

                // Chat panel (C key toggle) HOST/JOIN unified
                // Chat panel (C key toggle) HOST/JOIN unified
        // ── 파일 우클릭 컨텍스트 메뉴 ────────────────────────────────────
        if(file_ctx.open){
            ImGui::SetNextWindowPos(ImVec2(file_ctx.x, file_ctx.y));
            ImGui::SetNextWindowSize(ImVec2(220.f, 0.f)); // 높이 자동
            ImGui::SetNextWindowBgAlpha(0.95f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.f,6.f));
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f,0.12f,0.18f,1.f));
            ImGui::Begin("##file_ctx", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoDecoration);

            // Spectrogram Analysis → SA 탭으로 연결
            if(ImGui::Selectable("  Spectrogram Analysis")){
                v.sa_cleanup();
                v.sa_mode = false;
                v.sa_temp_path = file_ctx.filepath;
                v.sa_panel_open = true;
                stat_open = false; board_open = false;
                if(v.right_panel_ratio < 0.05f) v.right_panel_ratio = 0.3f;
                prev_right_visible_outer = false;
                v.sa_start(file_ctx.filepath);
                file_ctx.open = false;
            }
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f,0.5f,0.5f,1.f));
            ImGui::Selectable("  Time-Domain Analysis");
            ImGui::Selectable("  Freq-Domain Analysis");
            ImGui::PopStyleColor();

            ImGui::Separator();

            // Public → HOST: public/iq 또는 public/audio 복사 / JOIN: 서버에 업로드
            if(ImGui::Selectable("  Public")){
                if(v.net_cli){
                    // JOIN: 백그라운드로 서버에 업로드
                    std::string fp_cap = file_ctx.filepath;
                    uint8_t tid = v.next_transfer_id.fetch_add(1);
                    NetClient* cli_cap = v.net_cli;
                    std::thread([cli_cap, fp_cap, tid](){
                        cli_cap->cmd_share_upload(fp_cap.c_str(), tid);
                    }).detach();
                } else {
                    // HOST: public/iq 또는 public/audio 폴더로 복사
                    bool is_iq = (file_ctx.filename.size()>3 && file_ctx.filename.substr(0,3)=="IQ_")
                              || (file_ctx.filename.size()>3 && file_ctx.filename.substr(0,3)=="sa_");
                    std::string pub_dir = is_iq ? BEWEPaths::public_iq_dir() : BEWEPaths::public_audio_dir();
                    struct stat sd{}; if(stat(pub_dir.c_str(),&sd)!=0) mkdir(pub_dir.c_str(),0755);
                    std::string dst = pub_dir + "/" + file_ctx.filename;
                    FILE* fin=fopen(file_ctx.filepath.c_str(),"rb");
                    FILE* fout=fopen(dst.c_str(),"wb");
                    if(fin&&fout){
                        char buf[65536]; size_t n;
                        while((n=fread(buf,1,sizeof(buf),fin))>0) fwrite(buf,1,n,fout);
                    }
                    if(fin) fclose(fin);
                    if(fout) fclose(fout);
                    // pub_owners: HOST 자신이 올린 파일 소유자 등록
                    pub_owners[file_ctx.filename] = std::string(login_get_id());
                    // pub_iq_files / pub_audio_files / shared_files에 추가
                    if(is_iq){
                        bool dup=false; for(auto& sf:pub_iq_files) if(sf==file_ctx.filename){dup=true;break;}
                        if(!dup) pub_iq_files.push_back(file_ctx.filename);
                    } else {
                        bool dup=false; for(auto& sf:pub_audio_files) if(sf==file_ctx.filename){dup=true;break;}
                        if(!dup) pub_audio_files.push_back(file_ctx.filename);
                    }
                    {
                        bool dup=false;
                        for(auto& sf:shared_files) if(sf==file_ctx.filename){dup=true;break;}
                        if(!dup) shared_files.push_back(file_ctx.filename);
                    }
                    // JOIN들에게 public 목록 브로드캐스트
                    if(v.net_srv){
                        std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
                        for(auto& sf : shared_files){
                            bool siq = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
                            std::string sfp = (siq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
                            struct stat sst{}; uint64_t fsz=0;
                            if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
                            std::string upl;
                            auto it=pub_owners.find(sf); if(it!=pub_owners.end()) upl=it->second;
                            slist.push_back({sf,fsz,upl});
                        }
                        v.net_srv->send_share_list(-1, slist);
                    }
                }
                file_ctx.open = false;
            }

            // Download → share 폴더로 복사 (Public 파일이고 HOST/LOCAL인 경우)
            if(file_ctx.is_public && !v.net_cli){
                if(ImGui::Selectable("  Download")){
                    bool is_iq2 = (file_ctx.filename.size()>3 && file_ctx.filename.substr(0,3)=="IQ_")
                               || (file_ctx.filename.size()>3 && file_ctx.filename.substr(0,3)=="sa_");
                    std::string share_dir2 = is_iq2 ? BEWEPaths::share_iq_dir() : BEWEPaths::share_audio_dir();
                    struct stat sd2{}; if(stat(share_dir2.c_str(),&sd2)!=0) mkdir(share_dir2.c_str(),0755);
                    std::string dst2 = share_dir2 + "/" + file_ctx.filename;
                    FILE* fin2=fopen(file_ctx.filepath.c_str(),"rb");
                    FILE* fout2=fopen(dst2.c_str(),"wb");
                    if(fin2&&fout2){
                        char buf2[65536]; size_t n2;
                        while((n2=fread(buf2,1,sizeof(buf2),fin2))>0) fwrite(buf2,1,n2,fout2);
                    }
                    if(fin2) fclose(fin2);
                    if(fout2) fclose(fout2);
                    // share 목록에 추가
                    if(is_iq2){
                        bool dup=false; for(auto& s:share_iq_files) if(s==file_ctx.filename){dup=true;break;}
                        if(!dup) share_iq_files.push_back(file_ctx.filename);
                    } else {
                        bool dup=false; for(auto& s:share_audio_files) if(s==file_ctx.filename){dup=true;break;}
                        if(!dup) share_audio_files.push_back(file_ctx.filename);
                    }
                    bool dup=false; for(auto& s:downloaded_files) if(s==file_ctx.filename){dup=true;break;}
                    if(!dup) downloaded_files.push_back(file_ctx.filename);
                    file_ctx.open = false;
                }
            }

            ImGui::Separator();

            // Delete → 파일 삭제 (Public은 소유자만 가능)
            {
                bool can_delete = true;
                if(file_ctx.is_public){
                    // 소유자 확인
                    auto it = pub_owners.find(file_ctx.filename);
                    if(it != pub_owners.end()){
                        const char* my_id = login_get_id();
                        can_delete = (it->second == std::string(my_id));
                    }
                }
                ImGui::PushStyleColor(ImGuiCol_Text,
                    can_delete ? ImVec4(1.f,0.35f,0.35f,1.f) : ImVec4(0.4f,0.4f,0.4f,1.f));
                if(ImGui::Selectable("  Delete") && can_delete){
                    remove(file_ctx.filepath.c_str());
                    // 모든 파일 목록에서 제거
                    auto rm_from = [&](std::vector<std::string>& v2){
                        v2.erase(std::remove(v2.begin(),v2.end(),file_ctx.filename),v2.end());
                    };
                    rm_from(rec_iq_files); rm_from(rec_audio_files);
                    rm_from(priv_iq_files); rm_from(priv_audio_files);
                    rm_from(pub_iq_files); rm_from(pub_audio_files);
                    rm_from(share_iq_files); rm_from(share_audio_files);
                    rm_from(priv_files); rm_from(shared_files); rm_from(downloaded_files);
                    priv_extra_paths.erase(file_ctx.filename);
                    pub_owners.erase(file_ctx.filename);
                    pub_listeners.erase(file_ctx.filename);
                    // rec_entries 목록에서도 제거
                    {
                        std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                        v.rec_entries.erase(std::remove_if(v.rec_entries.begin(),v.rec_entries.end(),
                            [&](const FFTViewer::RecEntry& e){return e.path==file_ctx.filepath;}),
                            v.rec_entries.end());
                    }
                    // HOST: public 파일이 삭제됐으면 JOIN들에게 갱신된 목록 브로드캐스트
                    if(v.net_srv && file_ctx.is_public){
                        std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
                        for(auto& sf : shared_files){
                            bool siq = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
                            std::string sfp = (siq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
                            struct stat sst{}; uint64_t fsz=0;
                            if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
                            std::string upl;
                            auto oit=pub_owners.find(sf); if(oit!=pub_owners.end()) upl=oit->second;
                            slist.push_back({sf,fsz,upl});
                        }
                        v.net_srv->send_share_list(-1, slist);
                    }
                    file_ctx.open = false;
                }
                ImGui::PopStyleColor();
            }

            // 창 밖 클릭 또는 ESC → 닫기
            if(!ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
               ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                file_ctx.open = false;
            }
            if(ImGui::IsKeyPressed(ImGuiKey_Escape, false)){
                file_ctx.open = false;
            }

            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar(2);
        }

        if(chat_open){
            const float CW=360.f, CH=320.f;
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x-CW-10,
                                           io.DisplaySize.y-CH-TOPBAR_H-10));
            ImGui::SetNextWindowSize(ImVec2(CW,CH));
            ImGui::SetNextWindowBgAlpha(0.92f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,8.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg,ImVec4(0.05f,0.07f,0.12f,1.f));
            ImGui::PushStyleColor(ImGuiCol_FrameBg,ImVec4(0.10f,0.12f,0.20f,1.f));
            ImGui::Begin("##chat",nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

            ImGui::TextColored(ImVec4(0.4f,0.7f,1.f,1.f),"Chat");
            ImGui::Separator();

            float msg_h=CH-60.f;
            ImGui::BeginChild("##chat_msgs",ImVec2(0,msg_h),false);

            auto print_chat=[&](const char* from, const char* msg){
                const char* myname = v.net_cli ? v.net_cli->my_name : login_get_id();
                bool is_me=(strcmp(from,myname)==0);
                ImVec4 col = is_me ? ImVec4(0.3f,1.f,0.5f,1.f) : ImVec4(0.85f,0.85f,0.85f,1.f);
                ImGui::PushStyleColor(ImGuiCol_Text, col);
                ImGui::TextWrapped("[%s] %s", from, msg);
                ImGui::PopStyleColor();
            };

            if(v.net_cli){
                std::unique_lock<std::mutex> lk(v.net_cli->chat_mtx, std::try_to_lock);
                if(lk.owns_lock()){
                    for(auto& m : v.net_cli->chat_log) print_chat(m.from, m.msg);
                    if(v.net_cli->chat_updated.exchange(false)) chat_scroll_bottom=true;
                }
            } else {
                std::lock_guard<std::mutex> lk(host_chat_mtx);
                for(auto& m : host_chat_log){
                    if(m.is_error){
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f,0.3f,0.3f,1.f));
                        ImGui::TextWrapped("[%s] %s", m.from, m.msg);
                        ImGui::PopStyleColor();
                    } else {
                        print_chat(m.from, m.msg);
                    }
                }
            }
            if(chat_scroll_bottom){ ImGui::SetScrollHereY(1.f); chat_scroll_bottom=false; }
            ImGui::EndChild();

            ImGui::Separator();
            ImGui::SetNextItemWidth(CW-16.f);
            // 엔터 또는 외부 포커스 요청 시 InputText 포커스
            if(chat_focus_input){
                ImGui::SetKeyboardFocusHere(0);
                chat_focus_input = false;
            }
            bool send_chat_msg=false;
            if(ImGui::InputText("##chat_in",chat_input,sizeof(chat_input),
                               ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackAlways,
                               [](ImGuiInputTextCallbackData* d) -> int {
                                   bool* flag = (bool*)d->UserData;
                                   if(*flag){ d->CursorPos = d->BufTextLen; d->SelectionStart = d->SelectionEnd = d->CursorPos; *flag = false; }
                                   return 0;
                               }, &chat_cursor_end))
                send_chat_msg=true;

            if(send_chat_msg && chat_input[0]){
                std::string chat_str = chat_input;
                // ── 로컬 메시지 추가 헬퍼 ────────────────────────────────
                auto push_local = [&](const char* from, const char* msg, bool is_err=false){
                    std::lock_guard<std::mutex> lk(host_chat_mtx);
                    LocalChatMsg lm{}; lm.is_error=is_err;
                    strncpy(lm.from, from, 31);
                    strncpy(lm.msg,  msg,  255);
                    host_chat_log.push_back(lm);
                    chat_scroll_bottom = true;
                };
                if(chat_str[0] == '/'){
                    if(chat_str == "/shutdown"){
                        // 프로그램 완전 종료
                        glfwSetWindowShouldClose(win, GLFW_TRUE);

                    } else if(chat_str == "/logout"){
                        // SDR 종료 → 로그인 화면으로 (세션 삭제, 프로세스 재시작)
                        do_logout = true;
                        // glfwSetWindowShouldClose 없이 do_logout만으로 inner while 탈출
                        // outer do-while은 !do_main_menu이므로 탈출 → if(do_logout) execv

                    } else if(chat_str == "/main"){
                        // SDR 종료 → main(지구본)으로 (로그인 세션 유지)
                        do_main_menu = true;
                        // inner while의 !do_main_menu 조건으로 탈출
                        // outer do-while이 do_main_menu=true로 재진입 → 지구본부터

                    } else if(chat_str == "/chassis 1 reset"){
                        if(v.net_srv){
                            // HOST: 전체 채팅 알림 → CHASSIS_RESETTING 브로드캐스트 → SDR 백그라운드 리셋
                            push_local("BEWE", "Chassis 1 reset ...", false);
                            v.net_srv->broadcast_chat("BEWE", "Chassis 1 reset ...");
                            v.net_srv->broadcast_heartbeat(1);
                            // SDR 연결 중일 때만 SDR 복구 경로 진입
                            if(v.is_running || cap.joinable()){
                                v.is_running = false;
                                v.sdr_stream_error.store(true);
                                v.tm_iq_on.store(false);
                                v.spectrum_pause.store(true);
                                usb_reset_pending = true;
                            } else {
                                push_local("BEWE", "No SDR connected — skip HW reset", false);
                            }
                        } else if(v.net_cli){
                            // JOIN: HOST에 명령 전달 + 로컬에 메시지 표시
                            push_local("BEWE", "Chassis 1 reset ...", false);
                            v.net_cli->cmd_chassis_reset();
                        } else {
                            // LOCAL: SDR 백그라운드 리셋 (UI 유지)
                            push_local("BEWE", "Chassis 1 reset ...", false);
                            if(v.is_running || cap.joinable()){
                                v.is_running = false;
                                v.sdr_stream_error.store(true);
                                v.tm_iq_on.store(false);
                                v.spectrum_pause.store(true);
                                usb_reset_pending = true;
                            } else {
                                push_local("BEWE", "No SDR connected — skip HW reset", false);
                            }
                        }

                    } else if(chat_str == "/chassis 2 reset"){
                        if(v.net_srv){
                            // HOST: JOIN 전체에 reset 알림 → 방송 중단 → 1초 대기 → 재개
                            push_local("BEWE", "Chassis 2 reset ...", false);
                            v.net_srv->broadcast_chat("BEWE", "Chassis 2 reset ...");
                            v.net_srv->broadcast_heartbeat(2); // JOIN에게 노란불
                            v.net_bcast_pause.store(true, std::memory_order_relaxed);
                            v.net_srv->pause_broadcast();
                            v.net_srv->flush_clients();
                            // Central Server에 NET_RESET 전송
                            if(central_cli.is_central_connected())
                                central_cli.send_net_reset(0);  // 0 = reset start
                            NetServer* srv_ptr = v.net_srv;
                            std::atomic<bool>* bcast_pause_ptr = &v.net_bcast_pause;
                            std::mutex* log_mtx_ptr = &host_chat_mtx;
                            std::vector<LocalChatMsg>* log_ptr = &host_chat_log;
                            CentralClient* central_ptr = &central_cli;
                            FFTViewer* vp = &v;
                            std::string rh = s_central_host;
                            int rp = s_central_port;
                            std::thread([srv_ptr, bcast_pause_ptr, log_mtx_ptr, log_ptr,
                                         central_ptr, vp, rh, rp](){
                                std::this_thread::sleep_for(std::chrono::seconds(1));
                                srv_ptr->flush_clients();
                                srv_ptr->resume_broadcast();
                                bcast_pause_ptr->store(false, std::memory_order_relaxed);
                                srv_ptr->broadcast_heartbeat(0);
                                srv_ptr->broadcast_chat("BEWE", "Chassis 2 stable ...");
                                // Central Server 끊겨있으면 재연결
                                if(!central_ptr->is_central_connected() && !rh.empty()){
                                    central_ptr->stop_mux_adapter();
                                    std::string sid = vp->station_name + "_" + std::string(login_get_id());
                                    int rfd = central_ptr->open_room(
                                        rh, rp, sid, vp->station_name,
                                        vp->station_lat, vp->station_lon,
                                        (uint8_t)login_get_tier());
                                    if(rfd >= 0){
                                        central_ptr->start_mux_adapter(rfd,
                                            [vp](int local_fd){ if(vp->net_srv) vp->net_srv->inject_fd(local_fd); },
                                            [vp](){ return vp->net_srv ? (uint8_t)vp->net_srv->client_count() : (uint8_t)0; });
                                        printf("[UI] Central reconnected after chassis 2 reset (chat)\n");
                                    }
                                } else if(central_ptr->is_central_connected()){
                                    central_ptr->send_net_reset(1);
                                }
                                std::lock_guard<std::mutex> lk(*log_mtx_ptr);
                                LocalChatMsg lm{}; lm.is_error = false;
                                strncpy(lm.from, "BEWE", 31);
                                strncpy(lm.msg,  "Chassis 2 stable ...", 255);
                                if((int)log_ptr->size() >= 200) log_ptr->erase(log_ptr->begin());
                                log_ptr->push_back(lm);
                            }).detach();
                        } else if(v.net_cli){
                            // JOIN: HOST에게 명령 전달 + 로컬에 메시지 표시
                            push_local("BEWE", "Chassis 2 reset ...", false);
                            v.net_cli->cmd_net_reset();
                        } else {
                            push_local("System", "/chassis 2 reset: not in HOST/JOIN mode.", true);
                        }

                    } else if(chat_str == "/rx stop"){
                        if(v.net_cli){
                            // JOIN: HOST에 명령 전달
                            push_local("BEWE", "RX stop ...", false);
                            v.net_cli->cmd_rx_stop();
                        } else if(v.rx_stopped.load()){
                            push_local("System", "RX already stopped.", true);
                        } else if(!v.is_running && !cap.joinable()){
                            push_local("System", "No SDR running.", true);
                        } else {
                            // HOST / LOCAL: 직접 실행
                            push_local("BEWE", "RX stop", false);
                            if(v.net_srv) v.net_srv->broadcast_chat("BEWE", "RX stop");
                            // 녹음/demod/TM 중지
                            if(v.rec_on.load()) v.stop_rec();
                            if(v.tm_iq_on.load()){ v.tm_iq_on.store(false); v.tm_iq_close(); }
                            v.stop_all_dem();
                            // 캡처 스레드 종료
                            v.is_running = false;
                            if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
                            v.mix_stop.store(true);
                            if(v.mix_thr.joinable()) v.mix_thr.join();
                            if(cap.joinable()) cap.join();
                            // FFTW 정리
                            if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                            if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                            if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                            // 디바이스 close
                            if(v.dev_blade){
                                bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
                                bladerf_close(v.dev_blade); v.dev_blade=nullptr;
                            }
                            if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                            v.rx_stopped.store(true);
                            v.sdr_stream_error.store(false);
                            v.spectrum_pause.store(false);
                            push_local("BEWE", "RX stopped.", false);
                        }

                    } else if(chat_str == "/rx start"){
                        if(v.net_cli){
                            // JOIN: HOST에 명령 전달
                            push_local("BEWE", "RX start ...", false);
                            v.net_cli->cmd_rx_start();
                        } else if(!v.rx_stopped.load()){
                            push_local("System", "RX already running.", true);
                        } else {
                            // HOST / LOCAL: 직접 실행
                            push_local("BEWE", "RX start — initializing SDR ...", false);
                            v.rx_stopped.store(false);
                            float cur_cf = (float)(v.header.center_frequency / 1e6);
                            if(cur_cf < 0.1f) cur_cf = 100.f;
                            v.is_running = true;
                            if(v.initialize(cur_cf)){
                                v.set_gain(v.gain_db);
                                if(v.hw.type == HWType::BLADERF)
                                    cap = std::thread(&FFTViewer::capture_and_process, &v);
                                else
                                    cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                                v.mix_stop.store(false);
                                v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);
                                push_local("BEWE", "RX started. SDR online.", false);
                                if(v.net_srv) v.net_srv->broadcast_chat("BEWE", "RX start — SDR online");
                            } else {
                                v.is_running = false;
                                v.rx_stopped.store(true);
                                push_local("System", "RX start failed — SDR not found.", true);
                            }
                        }

                    } else {
                        char errmsg[280];
                        snprintf(errmsg, sizeof(errmsg), "Unknown command: %s", chat_str.c_str());
                        push_local("System", errmsg, true);
                    }
                } else {
                    // ── 일반 메시지 ──────────────────────────────────────
                    if(v.net_cli){
                        v.net_cli->send_chat(chat_input);
                    } else {
                        // LOCAL / HOST: 로컬 로그에 표시
                        push_local(login_get_id(), chat_input);
                        if(v.net_srv)
                            v.net_srv->broadcast_chat(login_get_id(), chat_input);
                    }
                }
                chat_input[0]='\0';
                ImGui::SetWindowFocus(nullptr);
            }

            ImGui::End();
            ImGui::PopStyleColor(2); ImGui::PopStyleVar();
        }

        // ── 오퍼레이터 목록 패널 (O키 토글) ─────────────────────────────
        if(ops_open){
            const float OW=280.f;
            // HOST/JOIN 통합 목록 구성 (index=0: HOST, index>=1: JOIN)
            std::vector<OpEntry> ops_display;
            if(v.net_srv){
                // HOST 모드: 내 항목(index=0) 직접 구성 + JOIN 목록
                OpEntry host_e{}; host_e.index=0; host_e.tier=(uint8_t)login_get_tier();
                strncpy(host_e.name, v.host_name[0]?v.host_name:"Host", 31);
                ops_display.push_back(host_e);
                auto joins = v.net_srv->get_operators();
                ops_display.insert(ops_display.end(), joins.begin(), joins.end());
            } else if(v.net_cli){
                // JOIN 모드: op_list (index=0은 HOST, 나머지는 JOIN)
                std::unique_lock<std::mutex> lk(v.net_cli->op_mtx, std::try_to_lock);
                if(lk.owns_lock()){
                    for(int i=0;i<v.net_cli->op_list.count;i++)
                        ops_display.push_back(v.net_cli->op_list.ops[i]);
                }
            }
            float OH=60.f+(float)ops_display.size()*22.f;
            OH=std::max(OH,100.f);

            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x-OW-10, TOPBAR_H+10));
            ImGui::SetNextWindowSize(ImVec2(OW,OH));
            ImGui::SetNextWindowBgAlpha(0.90f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,8.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg,ImVec4(0.05f,0.07f,0.12f,1.f));
            ImGui::Begin("##ops",nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

            ImGui::TextColored(ImVec4(0.4f,0.7f,1.f,1.f),"Operators");
            ImGui::SameLine(OW-30); if(ImGui::SmallButton("X##oc")) ops_open=false;
            ImGui::Separator();

            for(auto& op : ops_display){
                bool is_host = (op.index == 0);
                const char* badge = is_host ? "[HOST]" : "[JOIN]";
                ImVec4 col = is_host ? ImVec4(0.4f,0.85f,1.f,1.f)
                                     : ImVec4(0.7f,0.92f,0.7f,1.f);
                ImGui::TextColored(col,"%s %s  [Tier%d]",
                    badge, op.name, op.tier);
            }
            ImGui::End();
            ImGui::PopStyleColor(); ImGui::PopStyleVar();
        }

        ImGui::Render();
        int dw2,dh2; glfwGetFramebufferSize(win,&dw2,&dh2);
        glViewport(0,0,dw2,dh2);
        glClearColor(0.1f,0.1f,0.1f,1); glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    } // end if(!do_logout) — skip SDR init+main loop when logout from globe

    v.is_running = false;
    // RTL-SDR: async read 즉시 취소 → cap thread 블로킹 해제
    if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
    v.stop_all_dem();
    if(v.rec_on.load()) v.stop_rec();
    if(v.tm_iq_file_ready){
        v.tm_iq_on.store(false);
        v.tm_iq_close();
    }
    v.mix_stop.store(true); if(v.mix_thr.joinable()) v.mix_thr.join();
    v.net_bcast_stop.store(true);
    v.net_bcast_cv.notify_all();
    if(v.net_bcast_thr.joinable()) v.net_bcast_thr.join();
    central_cli.stop_mux_adapter();
    central_cli.stop_polling();
    if(v.net_srv){ v.net_srv->stop_discovery_broadcast(); v.net_srv->stop(); delete v.net_srv; v.net_srv=nullptr; }
    if(v.net_cli){ v.net_cli->disconnect(); delete v.net_cli; v.net_cli=nullptr; }
    if(!v.remote_mode && cap.joinable()) cap.join();
    if(v.dev_blade){
        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
    }
    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
    if(v.waterfall_texture) glDeleteTextures(1,&v.waterfall_texture);
    v.sa_cleanup();

    // ── record/ → private/ 이동 (세션 종료 시) ─────────────────────────
    {
        auto move_dir = [](const std::string& src_dir, const std::string& dst_dir){
            DIR* d = opendir(src_dir.c_str());
            if(!d) return;
            struct dirent* ent;
            while((ent=readdir(d))!=nullptr){
                const char* n = ent->d_name;
                size_t nl = strlen(n);
                if(nl>4 && strcmp(n+nl-4,".wav")==0){
                    std::string src = src_dir+"/"+n;
                    std::string dst = dst_dir+"/"+n;
                    rename(src.c_str(), dst.c_str());
                }
            }
            closedir(d);
        };
        move_dir(BEWEPaths::record_iq_dir(),    BEWEPaths::private_iq_dir());
        move_dir(BEWEPaths::record_audio_dir(), BEWEPaths::private_audio_dir());
    }

    } while(do_main_menu && !glfwWindowShouldClose(win)); // ── 모드선택 outer 루프 끝

    if(do_logout){
        // 로그인 화면으로 돌아가기 (프로세스 재시작, 세션 삭제)
        printf("Logout: restarting to login screen...\n");
        ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
        char* argv0[] = {(char*)"/proc/self/exe", nullptr};
        execv("/proc/self/exe", argv0);
        _exit(0);
    }

    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(); glfwTerminate();
    printf("Closed\n");
}