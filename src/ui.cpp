#include "fft_viewer.hpp"
#include "login.hpp"
#include "net_server.hpp"
#include "net_client.hpp"
#include "bewe_paths.hpp"
#include "globe.hpp"
#include "udp_discovery.hpp"
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <algorithm>
#include <chrono>
#include <map>
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
            stop_dem(ci);
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
        } else if(ch.mode==Channel::DM_DMR){
            // DMR: 보라색 (MAGIC과 동일 계열)
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
        char lb[16]; snprintf(lb,sizeof(lb),"[%d]",i+1);
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

    float ds,de; get_disp(ds,de);
    float sr_mhz=header.sample_rate/1e6f; int np=(int)gw;
    // 타임머신 모드: tm_display_fft_idx 기준, 아니면 current_fft_idx
    int sp_idx=tm_active.load() ? tm_display_fft_idx : current_fft_idx;
    bool cv=(cached_sp_idx==sp_idx&&cached_pan==freq_pan&&cached_zoom==freq_zoom&&
             cached_px==np&&cached_pmin==display_power_min&&cached_pmax==display_power_max);
    if(!cv){
        current_spectrum.assign(np,-80.0f);
        float nyq=sr_mhz/2.0f; int hf=header.fft_size/2;
        int mi=sp_idx%MAX_FFTS_MEMORY;
        for(int px=0;px<np;px++){
            float fd=ds+(float)px/np*(de-ds);
            int bin=(fd>=0)?(int)((fd/nyq)*hf):fft_size+(int)((fd/nyq)*hf);
            if(bin>=0&&bin<fft_size)
                current_spectrum[px]=(fft_data[mi*fft_size+bin]/127.0f)*(header.power_max-header.power_min)+header.power_min;
        }
        cached_sp_idx=sp_idx; cached_pan=freq_pan; cached_zoom=freq_zoom;
        cached_px=np; cached_pmin=display_power_min; cached_pmax=display_power_max;
    }
    float pr=display_power_max-display_power_min;
    for(int px=0;px<np-1;px++){
        if(px>=(int)current_spectrum.size()||px+1>=(int)current_spectrum.size()) break;
        float p1=std::max(0.0f,std::min(1.0f,(current_spectrum[px]-display_power_min)/pr));
        float p2=std::max(0.0f,std::min(1.0f,(current_spectrum[px+1]-display_power_min)/pr));
        dl->AddLine(ImVec2(gx+px,gy+(1-p1)*gh),ImVec2(gx+px+1,gy+(1-p2)*gh),IM_COL32(0,255,0,255),1.5f);
    }
    for(int i=1;i<=9;i++){
        float y=gy+(float)i/10*gh;
        dl->AddLine(ImVec2(gx,y),ImVec2(gx+gw,y),IM_COL32(60,60,60,100),1);
        dl->AddLine(ImVec2(gx-5,y),ImVec2(gx,y),IM_COL32(100,100,100,200),1);
        char lb[16]; snprintf(lb,16,"%.0f",-8.0f*i);
        ImVec2 ts=ImGui::CalcTextSize(lb);
        dl->AddText(ImVec2(gx-10-ts.x,y-7),IM_COL32(200,200,200,255),lb);
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
    // Power axis drag
    ImGui::SetCursorScreenPos(ImVec2(full_x,gy));
    ImGui::InvisibleButton("pax",ImVec2(AXIS_LABEL_WIDTH,gh));
    static float dsy=0,dsmin=0,dsmax=0; static bool dl_lo=false;
    if(ImGui::IsItemActive()){
        if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
            ImVec2 m2=ImGui::GetMousePos();
            float mid=(display_power_min+display_power_max)/2;
            float midy=gy+gh*(1-(mid-display_power_min)/(display_power_max-display_power_min));
            dsy=m2.y; dsmin=display_power_min; dsmax=display_power_max; dl_lo=(m2.y>midy);
        }
        if(ImGui::IsMouseDragging(ImGuiMouseButton_Left)){
            ImVec2 m2=ImGui::GetMousePos(); float dy=m2.y-dsy;
            float midp=(dsmin+dsmax)/2, midyy=gy+gh*(1-(midp-dsmin)/(dsmax-dsmin));
            if(dl_lo){ float n=dy/(gy+gh-midyy); n=std::max(-1.0f,std::min(1.0f,n)); display_power_min=midp-n*50; }
            else      { float n=-dy/midyy;        n=std::max(-1.0f,std::min(1.0f,n)); display_power_max=midp+n*50; }
            if(display_power_max-display_power_min<5){
                float md=(display_power_min+display_power_max)/2;
                display_power_min=md-2.5f; display_power_max=md+2.5f;
            }
            cached_sp_idx=-1;
        }
    }
}

void FFTViewer::draw_waterfall_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h){
    if(total_w < AXIS_LABEL_WIDTH + 2.0f || total_h < 4.0f) return;
    float gx=full_x+AXIS_LABEL_WIDTH, gy=full_y;
    float gw=total_w-AXIS_LABEL_WIDTH, gh=total_h;
    dl->AddRectFilled(ImVec2(full_x,full_y),ImVec2(full_x+total_w,full_y+total_h),IM_COL32(10,10,10,255));
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
        float us=(ds+nyq)/(2*nyq), ue=(de+nyq)/(2*nyq);
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
        float rps=(float)header.sample_rate/(float)fft_size/(float)time_average;
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
                        float rps=(float)header.sample_rate/(float)fft_size/(float)time_average;
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
                float rps2=(float)header.sample_rate/(float)fft_size/(float)time_average;
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

    // ── Login screen loop ─────────────────────────────────────────────────
    {
        bool logged_in = false;
        while(!logged_in && !glfwWindowShouldClose(win)){
            glfwPollEvents();
            int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
            glViewport(0,0,fw,fh);
            glClearColor(0.047f,0.071f,0.137f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
            logged_in = draw_login_screen(fw,fh);
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
    bool do_restart = false;
    bool do_chassis_reset = false;
    int  chassis_reset_mode = 0; // 0=LOCAL, 1=HOST
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
    do {
    do_main_menu = false;
    FFTViewer v;
    std::thread cap;
    v.create_waterfall_texture();
    // 0=LOCAL, 1=HOST, 2=CONNECT
    int  mode_sel     = do_chassis_reset ? chassis_reset_mode : 0;
    int  host_port    = 7700;
    char connect_host[128] = "192.168.1.";
    int  connect_port = 7700;
    char connect_id[32]  = {};
    char connect_pw[64]  = {};
    uint8_t connect_tier = 1;
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
                s.name       = ann.station_name;
                s.lat        = ann.lat;
                s.lon        = ann.lon;
                s.user_count = ann.user_count;
                s.last_seen  = now;
                return;
            }
        }
        FFTViewer::DiscoveredStation ns;
        ns.name       = ann.station_name;
        ns.lat        = ann.lat;
        ns.lon        = ann.lon;
        ns.tcp_port   = ann.tcp_port;
        ns.ip         = ann.host_ip;
        ns.user_count = ann.user_count;
        ns.last_seen  = now;
        v.discovered_stations.push_back(ns);
    };
    disc_listener.start();

    // Pop-up state machine
    enum GlobePop { POP_NONE, POP_HOST, POP_JOIN } pop_state = POP_NONE;
    FFTViewer::DiscoveredStation pending_join;
    float pending_lat=0.f, pending_lon=0.f;
    char  new_station_name[64] = {};
    bool  was_dragging = false;

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
                            pop_state = POP_HOST;
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
                    snprintf(ubuf, sizeof(ubuf), "%d ops", (int)st.user_count);
                    fdl->AddText(ImVec2(sx+12,sy+4),
                                 IM_COL32(160,200,220,200), ubuf);
                }
            }
        }

        // ── Title ─────────────────────────────────────────────────────────
        {
            ImDrawList* fdl = ImGui::GetForegroundDrawList();
            fdl->AddText(ImVec2(20,20), IM_COL32(100,180,255,200),
                         "BEWE Station Discovery");
            fdl->AddText(ImVec2(20,36), IM_COL32(120,140,160,150),
                         "Click globe to place station  |  Click marker to join");
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

        // ── Manual connect bar (bottom center) — Tier 1/2 only ──────────
        if(login_get_tier() < 3)
        {
            const float MW = 480.f, MH = 60.f;
            ImGui::SetNextWindowPos(ImVec2(((float)fw-MW)*0.5f, (float)fh-MH-8.f));
            ImGui::SetNextWindowSize(ImVec2(MW, MH));
            ImGui::SetNextWindowBgAlpha(0.78f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
            ImGui::Begin("##manual_bar", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                ImGuiWindowFlags_NoNav);
            ImGui::Text("Manual:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(170);
            ImGui::InputText("##mip", connect_host, sizeof(connect_host));
            ImGui::SameLine();
            ImGui::SetNextItemWidth(72);
            ImGui::InputInt("##mport", &connect_port, 0, 0);
            if(connect_port < 1) connect_port = 1;
            if(connect_port > 65535) connect_port = 65535;
            ImGui::SameLine();
            if(ImGui::Button("Connect##man", ImVec2(72,22))){
                { std::lock_guard<std::mutex> lk(join_share_mtx); join_share_files.clear(); }
                cli = new NetClient();
                if(cli->connect(connect_host, connect_port,
                                login_get_id(), login_get_pw(),
                                (uint8_t)login_get_tier())){
                    strncpy(connect_id, login_get_id(), 31); connect_id[31]='\0';
                    strncpy(connect_pw, login_get_pw(), 63); connect_pw[63]='\0';
                    connect_tier = (uint8_t)login_get_tier();
                    mode_sel=2; mode_done=true;
                } else {
                    delete cli; cli=nullptr;
                    mode_err_msg="Connection failed. Check IP/Port";
                    mode_err_timer=3.f;
                }
            }
            if(mode_err_timer > 0.f){
                mode_err_timer -= io.DeltaTime;
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.f,0.3f,0.3f,1.f),
                                   "%s", mode_err_msg.c_str());
            }
            ImGui::End();
            ImGui::PopStyleVar();
        }

        // ── HOST placement popup ──────────────────────────────────────────
        if(pop_state == POP_HOST){
            const float PW=330.f, PH=150.f;
            ImGui::SetNextWindowPos(ImVec2(((float)fw-PW)*0.5f,((float)fh-PH)*0.5f));
            ImGui::SetNextWindowSize(ImVec2(PW,PH));
            ImGui::SetNextWindowBgAlpha(0.92f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,10.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f,0.08f,0.15f,1.f));
            ImGui::Begin("##pop_host", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
            char loc[64];
            snprintf(loc, sizeof(loc), "Location: %.3fN, %.3fE",
                     pending_lat, pending_lon);
            ImGui::TextColored(ImVec4(0.5f,0.9f,0.5f,1.f), "%s", loc);
            ImGui::Spacing();
            ImGui::Text("Station Name:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160);
            ImGui::InputText("##sname", new_station_name, sizeof(new_station_name));
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f,0.40f,0.14f,1.f));
            bool can_host = new_station_name[0] != '\0';
            if(!can_host) ImGui::BeginDisabled();
            if(ImGui::Button("Set & Host##sh", ImVec2(110,26))){
                v.station_name = new_station_name;
                v.station_lat  = pending_lat;
                v.station_lon  = pending_lon;
                v.station_location_set = true;
                mode_sel=1; pop_state=POP_NONE; mode_done=true;
            }
            if(!can_host) ImGui::EndDisabled();
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if(ImGui::Button("Cancel##hc", ImVec2(80,26))) pop_state=POP_NONE;
            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }

        // ── JOIN confirm popup ────────────────────────────────────────────
        if(pop_state == POP_JOIN){
            const float PW=330.f, PH=130.f;
            ImGui::SetNextWindowPos(ImVec2(((float)fw-PW)*0.5f,((float)fh-PH)*0.5f));
            ImGui::SetNextWindowSize(ImVec2(PW,PH));
            ImGui::SetNextWindowBgAlpha(0.92f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,10.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f,0.08f,0.15f,1.f));
            ImGui::Begin("##pop_join", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
            ImGui::TextColored(ImVec4(0.4f,0.8f,1.f,1.f),
                               "Join: %s", pending_join.name.c_str());
            char ub[48];
            snprintf(ub, sizeof(ub), "%d operator%s connected",
                     (int)pending_join.user_count,
                     pending_join.user_count == 1 ? "" : "s");
            ImGui::TextDisabled("%s", ub);
            ImGui::TextDisabled("IP: %s  Port: %d",
                                pending_join.ip.c_str(), (int)pending_join.tcp_port);
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f,0.30f,0.60f,1.f));
            if(ImGui::Button("Join##jb", ImVec2(100,26))){
                { std::lock_guard<std::mutex> lk(join_share_mtx); join_share_files.clear(); }
                cli = new NetClient();
                if(cli->connect(pending_join.ip.c_str(), (int)pending_join.tcp_port,
                                login_get_id(), login_get_pw(),
                                (uint8_t)login_get_tier())){
                    strncpy(connect_id, login_get_id(), 31); connect_id[31]='\0';
                    strncpy(connect_pw, login_get_pw(), 63); connect_pw[63]='\0';
                    connect_tier = (uint8_t)login_get_tier();
                    mode_sel=2; pop_state=POP_NONE; mode_done=true;
                } else {
                    delete cli; cli=nullptr;
                    mode_err_msg="Connection failed";
                    mode_err_timer=3.f;
                    pop_state=POP_NONE;
                }
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if(ImGui::Button("Cancel##jc", ImVec2(80,26))) pop_state=POP_NONE;
            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    disc_listener.stop();
    globe.destroy();

    if(glfwWindowShouldClose(win)){
        if(cli){ cli->disconnect(); delete cli; }
        ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
        return;
    }

    // ── 모드에 따라 초기화 ────────────────────────────────────────────────
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
                v.channels[i].s   = sync.ch[i].s;
                v.channels[i].e   = sync.ch[i].e;
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
            // JOIN: REQ_PENDING region IQ → REQ_TRANSFERRING
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_PENDING){
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

        // JOIN: 호스트가 Deny한 경우 → REQ_PENDING 항목을 REQ_DENIED로
        cli->on_region_response = [&](bool allowed){
            if(!allowed){
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_PENDING){
                        e.req_state=FFTViewer::RecEntry::REQ_DENIED;
                        e.req_deny_timer=30.f; break;
                    }
            }
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
            bool is_region = false;
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING
                       && e.filename==filename){ is_region=true; break; }
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

        // 오퍼레이터 목록 팝업 표시
        bool op_popup_open = true;
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
            printf("SDR init failed\n");
            if(cli){ cli->disconnect(); delete cli; }
            ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
            return;
        }
        if(v.hw.type == HWType::BLADERF)
            cap = std::thread(&FFTViewer::capture_and_process, &v);
        else
            cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
        v.mix_stop.store(false);
        v.mix_thr=std::thread(&FFTViewer::mix_worker,&v);

        if(mode_sel==1){
            // HOST: 서버 시작 (public 목록 + 소유자/리스너 초기화)
            shared_files.clear(); pub_iq_files.clear(); pub_audio_files.clear();
            pub_listeners.clear(); pub_owners.clear();
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
                v.stop_dem(idx);
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
                // 즉시 녹음 대신 승인 대기 항목으로 추가
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                FFTViewer::RecEntry e{};
                time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
                char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
                float cf_mhz = (freq_lo+freq_hi)/2.0f;
                char fn[128]; snprintf(fn,sizeof(fn),"IQ_%.3fMHz_%s.wav",cf_mhz,dts);
                e.filename = fn;
                e.is_region = true;
                e.req_state = FFTViewer::RecEntry::REQ_PENDING;
                e.req_op_idx = op_idx;
                strncpy(e.req_op_name, op_name?op_name:"?", 31);
                e.req_fft_top=fft_top; e.req_fft_bot=fft_bot;
                e.req_freq_lo=freq_lo; e.req_freq_hi=freq_hi;
                e.req_time_start=time_start; e.req_time_end=time_end;
                e.t_start=std::chrono::steady_clock::now();
                v.rec_entries.push_back(e);
            };
            srv->cb.on_toggle_recv = [&](int ch_idx, uint8_t op_idx, bool enable){
                if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
                uint32_t bit = 1u << op_idx;
                uint32_t mask = v.channels[ch_idx].audio_mask.load();
                if(enable) mask |= bit; else mask &= ~bit;
                v.channels[ch_idx].audio_mask.store(mask);
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
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
                srv->send_file_to((int)op_idx, path.c_str(), tid);
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

            if(!srv->start(host_port)){
                printf("Server start failed\n");
                delete srv; srv=nullptr;
            } else {
                v.net_srv = srv;
                srv->set_host_info(login_get_id(), (uint8_t)login_get_tier());
                // Start UDP discovery broadcast if station location was set via globe
                if(v.station_location_set){
                    std::string lip = get_local_ip();
                    srv->start_discovery_broadcast(
                        v.station_name.c_str(),
                        v.station_lat, v.station_lon,
                        (uint16_t)host_port,
                        lip.c_str());
                }
                // 브로드캐스트 전용 스레드 시작 (캡처 스레드 분리)
                v.net_bcast_stop.store(false);
                v.net_bcast_thr = std::thread(&FFTViewer::net_bcast_worker, &v);
            }
        }
    }

    // ── System monitor state ──────────────────────────────────────────────
    auto cpu_last_time=std::chrono::steady_clock::now();
    auto status_last  =std::chrono::steady_clock::now();
    auto sq_sync_last =std::chrono::steady_clock::now();
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
    bool  chat_open    = false;
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
    } file_ctx;

    // HOST 모드 로컬 채팅 로그 (JOIN은 net_cli->chat_log 사용)
    struct LocalChatMsg { char from[32]; char msg[256]; bool is_error=false; };
    std::vector<LocalChatMsg> host_chat_log;
    std::mutex host_chat_mtx;

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

    // ── Main loop ─────────────────────────────────────────────────────────
    while(!glfwWindowShouldClose(win) && !do_logout && !do_main_menu){
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

        // ── CONNECT 모드: 연결 끊김 감지 → 자동 재연결 ─────────────────────
        if(v.remote_mode && v.net_cli && !v.net_cli->is_connected()){
            static float reconn_timer = 0.f;
            reconn_timer -= ImGui::GetIO().DeltaTime;
            if(reconn_timer <= 0.f){
                reconn_timer = 2.f;
                bool ok = v.net_cli->connect(connect_host, connect_port,
                                              connect_id, connect_pw, connect_tier);
                if(ok){
                    // 재연결 성공: 뮤트 채널 상태 복원
                    for(int ci=0;ci<MAX_CHANNELS;ci++)
                        if(v.local_ch_out[ci]==3) v.net_cli->cmd_toggle_recv(ci,false);
                    // 재연결 후 오토스케일 트리거
                    v.autoscale_active=true; v.autoscale_init=false; v.autoscale_accum.clear();
                    // 워터폴 타임스탬프 초기화
                    { std::lock_guard<std::mutex> wlk(v.wf_events_mtx); v.wf_events.clear(); }
                    v.last_tagged_sec = -1;
                }
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
                v.autoscale_active = true;
                v.autoscale_init   = false;
                v.autoscale_accum.clear();
            }
            last_cf_mhz = cur_cf;
        }

        // ── CONNECT 모드: 수신 FFT → waterfall 업데이트 ─────────────────
        if(v.remote_mode && v.net_cli){
            int cur_seq = v.net_cli->fft_seq.load(std::memory_order_acquire);
            if(cur_seq != last_fft_seq){
                last_fft_seq = cur_seq;
                std::lock_guard<std::mutex> lk(v.net_cli->fft_mtx);
                int fsz = (int)v.net_cli->fft_sz;
                if(fsz > 0 && (int)v.net_cli->fft_data.size() == fsz){
                    // FFT 크기 변경 시 재초기화
                    if(fsz != v.fft_size){
                        v.fft_size = fsz;
                        v.header.fft_size = (uint32_t)fsz;
                        v.fft_data.assign((size_t)MAX_FFTS_MEMORY * fsz, 0);
                        v.current_spectrum.assign(fsz, -80.f);
                        v.texture_needs_recreate = true;
                    }
                    v.header.center_frequency = v.net_cli->cf_hz;
                    v.header.sample_rate      = v.net_cli->sr;
                    v.header.power_min        = v.net_cli->pmin;
                    v.header.power_max        = v.net_cli->pmax;
                    // HOST autoscale 결과만 수용 (active 중엔 덮어쓰지 않음)
                    if(!v.autoscale_active){
                        v.display_power_min = v.net_cli->pmin;
                        v.display_power_max = v.net_cli->pmax;
                    }
                    {
                        std::lock_guard<std::mutex> dlk(v.data_mtx);
                        // circular buffer의 올바른 위치에 한 행만 복사
                        int fi = v.total_ffts % MAX_FFTS_MEMORY;
                        int8_t* dst = v.fft_data.data() + (size_t)fi * fsz;
                        memcpy(dst, v.net_cli->fft_data.data(), fsz);
                        v.total_ffts++;
                        v.current_fft_idx = v.total_ffts - 1;  // 언바운드 카운터 (TM offset 계산용)
                        // JOIN: wall_time 기반 워터폴 시간태그
                        if(v.net_cli->fft_wall_time > 0){
                            time_t wt = (time_t)v.net_cli->fft_wall_time;
                            struct tm* tt = localtime(&wt);
                            int cur5 = tt->tm_hour*720+tt->tm_min*12+tt->tm_sec/5;
                            if(cur5 != v.last_tagged_sec){
                                v.last_tagged_sec = cur5;
                                FFTViewer::WfEvent wev{};
                                wev.fft_idx  = v.current_fft_idx;  // 언바운드 인덱스
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
                        // JOIN 오토스케일 누적
                        if(v.autoscale_active){
                            if(!v.autoscale_init){
                                v.autoscale_accum.reserve(fsz*200);
                                v.autoscale_last = std::chrono::steady_clock::now();
                                v.autoscale_init = true;
                            }
                            for(int _i=1;_i<fsz;_i++){
                                float val = v.net_cli->pmin +
                                    (dst[_i]/127.0f)*(v.net_cli->pmax - v.net_cli->pmin);
                                v.autoscale_accum.push_back(val);
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
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        v.topbar_sel_this_frame=false;
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
                                float rps_r=(float)v.header.sample_rate/(float)v.fft_size/(float)v.time_average;
                                if(rps_r<=0) rps_r=37.5f;
                                time_t now_r=time(nullptr);
                                v.region.time_end  =now_r-(time_t)((v.current_fft_idx-v.region.fft_top)/rps_r);
                                v.region.time_start=now_r-(time_t)((v.current_fft_idx-v.region.fft_bot)/rps_r);
                            }
                        }
                        // JOIN: 영역 IQ 녹음 요청 → 호스트 승인 필요
                        v.net_cli->cmd_request_region(
                            v.region.fft_top, v.region.fft_bot,
                            v.region.freq_lo, v.region.freq_hi,
                            (int32_t)v.region.time_start, (int32_t)v.region.time_end);
                        v.region.active = false;  // 요청 후 선택영역 UI 해제
                        // 요청 상태 추가 (Record 패널에 표시)
                        {
                            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                            FFTViewer::RecEntry e{};
                            time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
                            char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
                            float cf_mhz = (v.region.freq_lo + v.region.freq_hi) / 2.0f;
                            char fn[128]; snprintf(fn,sizeof(fn),"IQ_%.3fMHz_%s.wav",cf_mhz,dts);
                            e.filename = fn;
                            e.is_region = true;
                            e.req_state = FFTViewer::RecEntry::REQ_PENDING;
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
                if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
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
                if(ImGui::IsKeyPressed(ImGuiKey_D,false)) set_mode(Channel::DM_DMR);
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
                if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow,false)){
                    if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_ch_pan(sci,-1);
                    else { v.channels[sci].pan=-1; if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS); }
                }
                if(ImGui::IsKeyPressed(ImGuiKey_RightArrow,false)){
                    if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_ch_pan(sci, 1);
                    else { v.channels[sci].pan= 1; if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS); }
                }
                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow,false)){
                    if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_ch_pan(sci, 0);
                    else { v.channels[sci].pan= 0; if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS); }
                }
            }
            if(ImGui::IsKeyPressed(ImGuiKey_C,false) && !editing){
                chat_open = !chat_open;
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
                        v.stop_dem(sci);
                        v.channels[sci].filter_active=false;
                        v.channels[sci].selected=false;
                        v.channels[sci].mode=Channel::DM_NONE;
                        if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                    }
                    v.selected_ch=-1;
                }
            }
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
        static bool chat_focus_input=false; // 채팅창 입력 포커스 상태
        // 채팅창 열려있으면 Enter → 채팅 포커스, 아니면 주파수 포커스
        if(ImGui::IsKeyPressed(ImGuiKey_Enter,false)||ImGui::IsKeyPressed(ImGuiKey_KeypadEnter,false)){
            if(chat_open){
                chat_focus_input = true;
            } else if(!editing){
                focus_freq = true;
            }
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
            if(ImGui::BeginCombo("##fftsize",fft_lbls[fft_si],ImGuiComboFlags_HeightSmall)){
                for(int i=0;i<6;i++){
                    bool sel2=(fft_si==i);
                    if(ImGui::Selectable(fft_lbls[i],sel2)){
                        fft_si=i; v.pending_fft_size=fft_sizes[i]; v.fft_size_change_req=true;
                    }
                    if(sel2) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::PopStyleVar();
        }
        ImGui::SameLine();

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
            const float DB_MIN=v.display_power_min, DB_MAX=v.display_power_max;
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

            // Per-channel labels (right to left)
            for(int i=MAX_CHANNELS-1;i>=0;i--){
                Channel& ch=v.channels[i]; if(!ch.filter_active) continue;
                float ss_mhz=std::min(ch.s,ch.e), se_mhz=std::max(ch.s,ch.e);
                float cf_mhz=(ss_mhz+se_mhz)/2.0f;
                float bw_khz=(se_mhz-ss_mhz)*1000.0f;
                const char* mname[5]={"","AM","FM","M","DMR"};
                const char* magic_names[]={"","AM","FM","DSB","SSB","CW"};
                const char* pname[3]={" L"," L+R"," R"};
                // JOIN은 dem_run 없음 → mode로 활성 판단
                bool dem_active = v.remote_mode
                    ? (ch.mode != Channel::DM_NONE)
                    : ch.dem_run.load();
                int pi=ch.pan+1; if(pi<0)pi=0; if(pi>2)pi=2;
                char cb[96];
                const char* mname2[5]={"","AM","FM","MAG","DMR"};
                if(dem_active && ch.mode==Channel::DM_MAGIC){
                    int mdet=ch.magic_det.load(std::memory_order_relaxed);
                    const char* ms=(mdet>0&&mdet<=5)?magic_names[mdet]:"";
                    if(ms[0])
                        snprintf(cb,sizeof(cb),"[%d] %s %.3f MHz @ %.0f kHz  ",
                                 i+1,ms,cf_mhz,bw_khz);
                    else
                        snprintf(cb,sizeof(cb),"[%d] %.3f MHz @ %.0f kHz  ",
                                 i+1,cf_mhz,bw_khz);
                } else if(dem_active && ch.mode!=Channel::DM_NONE){
                    snprintf(cb,sizeof(cb),"[%d] %s %.3f MHz @ %.0f kHz  ",
                             i+1,mname2[(int)ch.mode],cf_mhz,bw_khz);
                } else {
                    snprintf(cb,sizeof(cb),"[%d] %.3f MHz @ %.0f kHz  ",
                             i+1,cf_mhz,bw_khz);
                }
                ImVec2 cs2=ImGui::CalcTextSize(cb); rx-=cs2.x;
                bool is_selected=(v.selected_ch==i);
                bool gate_open = v.remote_mode
                    ? (ch.audio_mask.load() & 0x1u)!=0  // JOIN: 내 오디오 수신 여부
                    : ch.sq_gate.load();
                bool tb_dem = v.remote_mode
                    ? (ch.mode != Channel::DM_NONE)
                    : ch.dem_run.load();
                bool tb_rec=(v.rec_on.load()&&v.rec_ch==i);

                // 모드별 기본 색상 (필터 오버레이와 동일 계열)
                ImU32 mode_col;
                if(tb_rec)
                    mode_col=IM_COL32(255, 60, 60,255);
                else if(!tb_dem || ch.mode==Channel::DM_NONE)
                    mode_col=IM_COL32(160,160,160,255);
                else if(ch.mode==Channel::DM_AM)
                    mode_col=IM_COL32( 80,200,255,255);
                else if(ch.mode==Channel::DM_FM)
                    mode_col=IM_COL32(255,220, 50,255);
                else if(ch.mode==Channel::DM_MAGIC)
                    mode_col=IM_COL32(180, 80,255,255);
                else if(ch.mode==Channel::DM_DMR)
                    mode_col=IM_COL32(180, 80,255,255);
                else
                    mode_col=IM_COL32(160,160,160,255);

                ImU32 tc;
                if(is_selected)
                    // 선택: 신호 있으면 모드색, 없으면 흰색
                    tc=gate_open ? mode_col : IM_COL32(255,255,255,255);
                else
                    // 비선택: 신호 있으면 모드색, 없으면 어두운 모드색
                    tc=gate_open ? mode_col : IM_COL32(
                        (uint8_t)(((mode_col>>IM_COL32_R_SHIFT)&0xFF)*0.5f),
                        (uint8_t)(((mode_col>>IM_COL32_G_SHIFT)&0xFF)*0.5f),
                        (uint8_t)(((mode_col>>IM_COL32_B_SHIFT)&0xFF)*0.5f),
                        200);

                ImVec2 mpos=ImGui::GetIO().MousePos;
                bool hovered=(mpos.x>=rx&&mpos.x<rx+cs2.x&&mpos.y>=0&&mpos.y<TOPBAR_H);
                if(hovered){
                    tc=IM_COL32(255,255,255,255);
                    if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                        v.stop_dem(i);
                        v.channels[i].filter_active=false;
                        v.channels[i].selected=false;
                        v.channels[i].mode=Channel::DM_NONE;
                        if(v.selected_ch==i) v.selected_ch=-1;
                    } else if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        if(v.selected_ch>=0) v.channels[v.selected_ch].selected=false;
                        v.selected_ch=i; v.channels[i].selected=true;
                        v.topbar_sel_this_frame=true;
                    }
                }
                // 선택 채널: bold (1px offset 이중 렌더링)
                if(is_selected){
                    dl->AddText(ImVec2(rx+1,ty2),IM_COL32(0,0,0,100),cb);
                    dl->AddText(ImVec2(rx+1,ty2),tc,cb);
                }
                dl->AddText(ImVec2(rx,ty2),tc,cb);


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
        v.render_visible.store(left_visible);

        // ── 가로 분할: 끝까지 허용 (0~1 풀레인지) ───────────────────────
        float sp_h = (content_h - div_h) * v.spectrum_height_ratio;
        float wf_h = content_h - div_h - sp_h;
        sp_h = std::max(0.0f, sp_h);
        wf_h = std::max(0.0f, wf_h);
        bool wf_visible = left_visible && wf_h > 1.0f;

        // ── 파워스펙트럼 ──────────────────────────────────────────────────
        if(left_visible && sp_h > 1.0f)
            v.draw_spectrum_area(dl, 0, content_y, left_w, sp_h);

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
        ImGui::SetCursorScreenPos(ImVec2(vdiv_x, content_y));
        ImGui::InvisibleButton("vdiv", ImVec2(vdiv_w, content_h));
        if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)){
            // 오른쪽으로 드래그 → 우측 패널 줄어듦, 왼쪽으로 → 우측 패널 커짐
            v.right_panel_ratio -= io.MouseDelta.x / disp_w;
            v.right_panel_ratio = std::max(0.0f, std::min(1.0f, v.right_panel_ratio));
        }
        if(ImGui::IsItemHovered()) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

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
                static const int fft_sizes[] = {256,512,1024,2048,4096,8192};
                static const char* fft_labels[] = {"256","512","1024","2048","4096","8192"};
                const int n_sizes = 6;
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
                arch_scan_timer += io.DeltaTime;
                if(arch_scan_timer >= 1.0f){
                    arch_scan_timer = 0.0f;

                    // 폴더별 .wav 스캔 헬퍼 (mtime 내림차순 정렬)
                    auto scan_dir = [](const std::string& dir, std::vector<std::string>& out){
                        out.clear();
                        DIR* d = opendir(dir.c_str());
                        if(!d) return;
                        struct dirent* ent;
                        while((ent=readdir(d))!=nullptr){
                            const char* n = ent->d_name;
                            size_t nl = strlen(n);
                            if(nl>4 && strcmp(n+nl-4,".wav")==0) out.push_back(n);
                        }
                        closedir(d);
                        std::sort(out.begin(),out.end(),[&dir](const std::string& a,const std::string& b){
                            struct stat sa{},sb{};
                            stat((dir+"/"+a).c_str(),&sa); stat((dir+"/"+b).c_str(),&sb);
                            return sa.st_mtime > sb.st_mtime;
                        });
                    };

                    // Record 스캔 (record/iq, record/audio)
                    scan_dir(BEWEPaths::record_iq_dir(),    rec_iq_files);
                    scan_dir(BEWEPaths::record_audio_dir(), rec_audio_files);

                    // Private 스캔 (private/iq, private/audio)
                    scan_dir(BEWEPaths::private_iq_dir(),    priv_iq_files);
                    scan_dir(BEWEPaths::private_audio_dir(), priv_audio_files);
                    priv_files.clear();
                    priv_files.insert(priv_files.end(), priv_iq_files.begin(),    priv_iq_files.end());
                    priv_files.insert(priv_files.end(), priv_audio_files.begin(), priv_audio_files.end());

                    // Public 스캔 (public/iq, public/audio) - HOST only
                    scan_dir(BEWEPaths::public_iq_dir(),    pub_iq_files);
                    scan_dir(BEWEPaths::public_audio_dir(), pub_audio_files);
                    shared_files.clear();
                    shared_files.insert(shared_files.end(), pub_iq_files.begin(),    pub_iq_files.end());
                    shared_files.insert(shared_files.end(), pub_audio_files.begin(), pub_audio_files.end());

                    // Share 스캔 (share/iq, share/audio) - 다운로드된 파일
                    scan_dir(BEWEPaths::share_iq_dir(),    share_iq_files);
                    scan_dir(BEWEPaths::share_audio_dir(), share_audio_files);
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
                        // HOST 모드: IP를 하드웨어 이름과 같은 줄에 표시
                        if(v.net_srv){
                            static char s_host_ip[INET_ADDRSTRLEN] = "";
                            static float s_ip_timer = 0.f;
                            s_ip_timer -= io.DeltaTime;
                            if(s_ip_timer <= 0.f){
                                s_ip_timer = 5.f;
                                s_host_ip[0] = '\0';
                                struct ifaddrs* ifa_list;
                                if(getifaddrs(&ifa_list) == 0){
                                    for(auto* ifa=ifa_list; ifa; ifa=ifa->ifa_next){
                                        if(!ifa->ifa_addr||ifa->ifa_addr->sa_family!=AF_INET) continue;
                                        if(strcmp(ifa->ifa_name,"lo")==0) continue;
                                        auto* sin=(struct sockaddr_in*)ifa->ifa_addr;
                                        inet_ntop(AF_INET,&sin->sin_addr,s_host_ip,sizeof(s_host_ip));
                                        break;
                                    }
                                    freeifaddrs(ifa_list);
                                }
                            }
                            if(s_host_ip[0])
                                ImGui::Text("%s (%s)", hw_name, s_host_ip);
                            else
                                ImGui::TextUnformatted(hw_name);
                        } else {
                            ImGui::TextUnformatted(hw_name);
                        }
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
                        for(int ci=0;ci<MAX_CHANNELS;ci++){
                            Channel& ch=v.channels[ci];
                            if(!ch.filter_active) continue;
                            any_ch = true;

                            float cf_mhz=(ch.s+ch.e)/2.0f;
                            float bw_khz=(ch.e-ch.s)*1000.0f;
                            const char* mnames[]={"--","AM","FM","MAG","DMR"};
                            int mi=(int)ch.mode; if(mi<0||mi>4) mi=0;

                            // ── 채널 색상 (draw_all_channels와 동일) ──────
                            bool is_arec = ch.audio_rec_on.load();
                            bool is_irec = (v.rec_on.load()&&ci==v.rec_ch);
                            bool dem = v.remote_mode
                                ? (ch.mode!=Channel::DM_NONE)
                                : ch.dem_run.load();
                            bool gate = v.remote_mode
                                ? (ch.audio_mask.load()&0x1u)!=0
                                : ch.sq_gate.load();

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
                                ci+1,mnames[mi],cf_mhz,bw_khz);
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
                                    v.stop_dem(ci);
                                    v.channels[ci].filter_active=false;
                                    v.channels[ci].selected=false;
                                    v.local_ch_out[ci]=1;
                                    v.ch_created_by_me[ci]=false;
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
                                v.stop_dem(ci);
                                v.channels[ci].filter_active=false;
                                v.channels[ci].selected=false;
                                v.local_ch_out[ci]=1;
                                v.ch_created_by_me[ci]=false;
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
                                    bi==3?ImVec4(0.6f,0.1f,0.1f,1.f):ImVec4(0.15f,0.45f,0.75f,1.f));
                                if(ImGui::SmallButton(lbl_out[bi])) set_local_out(ci,bi);
                                if(active) ImGui::PopStyleColor();
                                if(bi<3) ImGui::SameLine(0,2);
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
                                    // region save 진행중
                                    if(region_saving){
                                        float t2=(float)ImGui::GetTime();
                                        bool blink=(fmodf(t2,0.8f)<0.4f);
                                        ImGui::PushStyleColor(ImGuiCol_Text,
                                            blink?IM_COL32(255,160,60,255):IM_COL32(200,120,40,255));
                                        ImGui::Text("[REC]  Region saving...");
                                        ImGui::PopStyleColor();
                                    }
                                    using RS = FFTViewer::RecEntry::ReqState;
                                    for(int ri=(int)v.rec_entries.size()-1;ri>=0;ri--){
                                        auto& re=v.rec_entries[ri];
                                        if(re.is_audio) continue;
                                        ImGui::PushID(ri+30000);
                                        if(re.req_state == RS::REQ_NONE){
                                            // 일반 IQ 녹음 항목
                                            if(re.finished){
                                                std::string szstr = fmt_filesize("",re.path);
                                                std::string lbl = std::string("[Done]  ")+re.filename;
                                                if(!szstr.empty()) lbl += "  "+szstr;
                                                ImGui::Selectable(lbl.c_str(), false);
                                                if(ImGui::IsItemHovered()&&ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                                                    file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                            } else {
                                                float elapsed=std::chrono::duration<float>(
                                                    std::chrono::steady_clock::now()-re.t_start).count();
                                                float t2=(float)ImGui::GetTime();
                                                bool blink=(fmodf(t2,0.8f)<0.4f);
                                                ImGui::PushStyleColor(ImGuiCol_Text,
                                                    blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                                std::string szstr2 = fmt_filesize("", re.path);
                                                if(!szstr2.empty())
                                                    ImGui::Text("[REC]  %s  [%ds]  %s", re.filename.c_str(), (int)elapsed, szstr2.c_str());
                                                else
                                                    ImGui::Text("[REC]  %s  [%ds]", re.filename.c_str(), (int)elapsed);
                                                ImGui::PopStyleColor();
                                            }
                                        } else {
                                            // 영역 IQ 요청 항목 — 상태별 표시
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImU32 col=IM_COL32(200,200,200,255);
                                            char state_lbl[48]="";
                                            switch(re.req_state){
                                                case RS::REQ_PENDING:
                                                    snprintf(state_lbl,sizeof(state_lbl),"[Request]");
                                                    col=blink?IM_COL32(255,210,50,255):IM_COL32(200,160,40,255);
                                                    break;
                                                case RS::REQ_CONFIRMED:
                                                    snprintf(state_lbl,sizeof(state_lbl),"[Confirm]");
                                                    col=IM_COL32(80,220,80,255);
                                                    break;
                                                case RS::REQ_DENIED:
                                                    snprintf(state_lbl,sizeof(state_lbl),"[Deny] %.0fs",re.req_deny_timer);
                                                    col=IM_COL32(200,80,80,255);
                                                    break;
                                                case RS::REQ_TRANSFERRING:
                                                    snprintf(state_lbl,sizeof(state_lbl),"[Transferring]");
                                                    col=IM_COL32(80,180,255,255);
                                                    break;
                                                default:
                                                    snprintf(state_lbl,sizeof(state_lbl),"[Done]");
                                                    col=IM_COL32(120,200,120,255);
                                                    break;
                                            }
                                            ImGui::PushStyleColor(ImGuiCol_Text,col);
                                            if(re.req_state==RS::REQ_TRANSFERRING && re.xfer_total>0)
                                                ImGui::Text("%s  %s  [%.1fM / %.1fM]",
                                                    state_lbl, re.filename.c_str(),
                                                    re.xfer_done/1048576.0, re.xfer_total/1048576.0);
                                            else
                                                ImGui::Text("%s  %s", state_lbl, re.filename.c_str());
                                            ImGui::PopStyleColor();

                                            // HOST 전용: 우클릭 메뉴 (PENDING 상태)
                                            if(v.net_srv && re.req_state==RS::REQ_PENDING
                                               && ImGui::IsItemHovered()
                                               && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                ImGui::OpenPopup("##req_ctx");
                                            }
                                            char req_pop_id[32]; snprintf(req_pop_id,32,"##req_ctx");
                                            if(ImGui::BeginPopup(req_pop_id)){
                                                ImGui::TextDisabled("Request: %s", re.req_op_name);
                                                ImGui::Separator();
                                                if(ImGui::MenuItem("Allow")){
                                                    // 요청 정보 복사 후 백그라운드 스레드 실행
                                                    re.req_state = RS::REQ_CONFIRMED;
                                                    float fl=re.req_freq_lo, fh=re.req_freq_hi;
                                                    int32_t ts=re.req_time_start, te=re.req_time_end;
                                                    uint8_t oidx=re.req_op_idx;
                                                    std::string fname=re.filename;
                                                    // JOIN 요청 시각 → HOST fft 인덱스 변환
                                                    float rps=(float)v.header.sample_rate/(float)v.fft_size/(float)v.time_average;
                                                    if(rps<=0.f) rps=37.5f;
                                                    time_t now_h=time(nullptr);
                                                    int cur_fi=v.current_fft_idx;
                                                    int32_t ft=(int32_t)(cur_fi-(int32_t)((now_h-(time_t)te)*rps));
                                                    int32_t fb=(int32_t)(cur_fi-(int32_t)((now_h-(time_t)ts)*rps));
                                                    std::thread([&v,srv,ft,fb,fl,fh,ts,te,oidx,fname](){
                                                        // rec_busy_flag가 해제될 때까지 대기 (이전 저장 완료 대기)
                                                        for(int w=0;w<200&&v.rec_busy_flag.load();w++)
                                                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                                                        // region 파라미터 설정
                                                        v.region.fft_top=ft; v.region.fft_bot=fb;
                                                        v.region.freq_lo=fl; v.region.freq_hi=fh;
                                                        v.region.time_start=(time_t)ts;
                                                        v.region.time_end=(time_t)te;
                                                        v.region.active=true;
                                                        // region_save() 대신 do_region_save_work() 직접 호출
                                                        // (region_save는 내부에서 detach 스레드를 띄우므로
                                                        //  완료를 기다릴 수 없음)
                                                        v.rec_busy_flag.store(true);
                                                        v.rec_state = FFTViewer::REC_BUSY;
                                                        v.rec_anim_timer = 0.0f;
                                                        v.region.active = false;
                                                        v.do_region_save_work();
                                                        v.rec_state = FFTViewer::REC_SUCCESS;
                                                        v.rec_success_timer = 3.0f;
                                                        v.rec_busy_flag.store(false);
                                                        // 저장 완료 후 경로 수집
                                                        std::string path;
                                                        {
                                                            std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                                            for(auto it=v.rec_entries.rbegin();it!=v.rec_entries.rend();++it)
                                                                if(it->is_region&&it->req_state==FFTViewer::RecEntry::REQ_NONE&&it->finished){
                                                                    path=it->path;
                                                                    // 이 항목 제거 (request 항목이 대신 표시)
                                                                    v.rec_entries.erase(std::next(it).base());
                                                                    break;
                                                                }
                                                        }
                                                        if(path.empty()){
                                                            // region_save 실패 → JOIN에 Deny 전송, 항목 제거
                                                            srv->send_region_response((int)oidx, false);
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
                                                        uint8_t tid=v.next_transfer_id.fetch_add(1);
                                                        srv->send_file_to((int)oidx,path.c_str(),tid,
                                                            [&v,fname](uint64_t done, uint64_t total){
                                                                std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                                                for(auto& e:v.rec_entries)
                                                                    if(e.filename==fname){ e.xfer_done=done; break; }
                                                            });
                                                        // 전송 완료 → 항목 제거 + 로컬 파일 삭제
                                                        {
                                                            std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                                            for(auto it2=v.rec_entries.begin();it2!=v.rec_entries.end();++it2)
                                                                if(it2->filename==fname){
                                                                    v.rec_entries.erase(it2);
                                                                    break;
                                                                }
                                                        }
                                                        remove(path.c_str());
                                                    }).detach();
                                                }
                                                if(ImGui::MenuItem("Deny")){
                                                    re.req_state = RS::REQ_DENIED;
                                                    re.req_deny_timer = 30.f;
                                                    srv->send_region_response((int)re.req_op_idx, false);
                                                }
                                                ImGui::EndPopup();
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
                                            std::string szstr = fmt_filesize("",re.path);
                                            std::string lbl = std::string("[Done]  ")+re.filename;
                                            if(!szstr.empty()) lbl += "  "+szstr;
                                            ImGui::Selectable(lbl.c_str(), false);
                                            if(ImGui::IsItemHovered()&&ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                                                file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                        } else {
                                            float elapsed=std::chrono::duration<float>(
                                                std::chrono::steady_clock::now()-re.t_start).count();
                                            int secs=(int)elapsed;
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImGui::PushStyleColor(ImGuiCol_Text,
                                                blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                            std::string szstr3 = fmt_filesize("", re.path);
                                            if(!szstr3.empty())
                                                ImGui::Text("[REC]  %s  [%ds]  %s", re.filename.c_str(), secs, szstr3.c_str());
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
                                std::string szstr = fmt_filesize("", fp);
                                std::string display = "  "+iq_files[fi];
                                if(!szstr.empty()) display += "  "+szstr;
                                ImGui::Selectable(display.c_str(), false);
                                if(ImGui::IsItemHovered()){
                                    if(on_hover) on_hover(fp, iq_files[fi]);
                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                        file_ctx={true,io.MousePos.x,io.MousePos.y,fp,iq_files[fi]};
                                        file_ctx.is_public=is_public_section;
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
                                std::string szstr = fmt_filesize("", fp);
                                std::string display = "  "+audio_files[fi];
                                if(!szstr.empty()) display += "  "+szstr;
                                ImGui::Selectable(display.c_str(), false);
                                if(ImGui::IsItemHovered()){
                                    if(on_hover) on_hover(fp, audio_files[fi]);
                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                        file_ctx={true,io.MousePos.x,io.MousePos.y,fp,audio_files[fi]};
                                        file_ctx.is_public=is_public_section;
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
                        int priv_cnt = (int)priv_iq_files.size() + (int)priv_audio_files.size();
                        // region IQ (extra_paths)도 포함
                        int priv_extra_cnt = 0;
                        for(auto& kv : priv_extra_paths){
                            bool already=false;
                            for(auto& f : priv_iq_files) if(f==kv.first){already=true;break;}
                            for(auto& f : priv_audio_files) if(f==kv.first){already=true;break;}
                            if(!already) priv_extra_cnt++;
                        }
                        if(ImGui::TreeNodeEx("##priv_node", ImGuiTreeNodeFlags_None, "Private  (%d)", priv_cnt+priv_extra_cnt)){
                            arch_priv_open = true;
                            // extra_paths (region IQ from JOIN)
                            if(!priv_extra_paths.empty()){
                                ImGui::TextDisabled("  IQ");
                                int ep_i=0;
                                for(auto& kv : priv_extra_paths){
                                    bool already=false;
                                    for(auto& f : priv_iq_files) if(f==kv.first){already=true;break;}
                                    if(already) continue;
                                    ImGui::PushID(18000+ep_i++);
                                    std::string szstr = fmt_filesize("", kv.second);
                                    std::string display = "  "+kv.first;
                                    if(!szstr.empty()) display += "  "+szstr;
                                    ImGui::Selectable(display.c_str(), false);
                                    if(ImGui::IsItemHovered()&&ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                                        file_ctx={true,io.MousePos.x,io.MousePos.y,kv.second,kv.first};
                                    ImGui::PopID();
                                }
                            }
                            draw_file_list("##priv_list",
                                priv_iq_files, priv_audio_files,
                                BEWEPaths::private_iq_dir(), BEWEPaths::private_audio_dir(),
                                20000);
                            ImGui::TreePop();
                        } else { arch_priv_open = false; }

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
                                            ImGui::TextDisabled("Uploaded by:");
                                            ImGui::Text("  %s", jse.uploader.c_str());
                                            ImGui::EndTooltip();
                                        }
                                        if(ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                                            ImGui::OpenPopup("##pub_dl_ctx");
                                    }
                                    if(ImGui::BeginPopup("##pub_dl_ctx")){
                                        ImGui::TextDisabled("%s", jse.filename.c_str());
                                        ImGui::Separator();
                                        if(!already_dl && ImGui::MenuItem("Download")){
                                            v.net_cli->cmd_request_share_download(jse.filename.c_str());
                                        }
                                        // JOIN이 업로더라면 Delete 가능
                                        std::string my_name = login_get_id();
                                        if(jse.uploader == my_name){
                                            ImGui::Separator();
                                            if(ImGui::MenuItem("Delete")){
                                                // TODO: JOIN→HOST 삭제 요청 프로토콜 추가 시 구현
                                            }
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
                                // ── HOST/LOCAL: public 폴더 파일 표시 ─────
                                draw_file_list("##pub_list",
                                    pub_iq_files, pub_audio_files,
                                    BEWEPaths::public_iq_dir(), BEWEPaths::public_audio_dir(),
                                    21000,
                                    [&](const std::string& /*fp*/, const std::string& fn){
                                        // 마우스 오버: 다운로드한 사람 툴팁
                                        auto it = pub_listeners.find(fn);
                                        if(it != pub_listeners.end() && !it->second.empty()){
                                            ImGui::BeginTooltip();
                                            ImGui::TextDisabled("Downloaded by:");
                                            for(auto& nm : it->second)
                                                ImGui::Text("  %s", nm.c_str());
                                            ImGui::EndTooltip();
                                        }
                                    },
                                    true /* is_public_section */);
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
                    // SA 워터폴 텍스처 표시 (패널 꽉 채움)
                    ImTextureID tid = (ImTextureID)(intptr_t)v.sa_texture;
                    dl->AddImage(tid, ImVec2(rpx,rp_content_y), ImVec2(disp_w,content_y+content_h),
                                 ImVec2(0,0), ImVec2(1,1), IM_COL32(255,255,255,255));
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
                            case Channel::DM_DMR:   mode_str="DMR";   break;
                            default: break;
                        }
                        // 복조 실행 중 여부
                        bool dem_active = ch.dem_run.load();
                        ImU32 col = dem_active ? IM_COL32(80,220,80,255) : IM_COL32(160,160,160,255);
                        ImGui::PushID(ci+40000);
                        ImGui::PushStyleColor(ImGuiCol_Text, col);
                        char ch_line[80];
                        snprintf(ch_line,sizeof(ch_line),"CH%d  %.4f MHz  %.0f kHz  [%s]",
                                 ci, (double)cf, (double)bw, mode_str);
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
                        ImGui::Text("CH%d  [%s]  %.4f MHz", ci, sig_str, (double)(ch.s+ch.e)*0.5f);
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

            // ── 중앙: 실시간 시계 (HH:MM:SS) ─────────────────────────────
            {
                time_t tnow = time(nullptr);
                struct tm tlocal{}; localtime_r(&tnow, &tlocal);
                char clock_str[16];
                strftime(clock_str, sizeof(clock_str), "%H:%M:%S", &tlocal);
                ImVec2 csz = ImGui::CalcTextSize(clock_str);
                dl->AddText(ImVec2((disp_w-csz.x)/2, ty_b), IM_COL32(200,200,200,255), clock_str);
            }

            // ── 좌측: 타임머신 오프셋 (활성 시만 표시) ───────────────────
            if(v.tm_active.load()){
                char tm_txt[48];
                if(v.tm_offset<=0.0f) snprintf(tm_txt,sizeof(tm_txt),"LIVE");
                else snprintf(tm_txt,sizeof(tm_txt),"-%.1f sec",v.tm_offset);
                dl->AddText(ImVec2(8,ty_b),IM_COL32(255,200,50,255),tm_txt);
            }

            // ── 우측: 상태 인디케이터 (오른쪽→왼쪽) ─────────────────────
            // 활성=흰색두껍게, 비활성=회색연하게
            bool streaming_on = !v.capture_pause.load();
            bool fft_on       = streaming_on && !v.spectrum_pause.load() && v.render_visible.load();
            bool tm_on        = v.tm_active.load();
            bool iq_on        = v.tm_iq_on.load();

            auto draw_ind=[&](float rx, const char* txt, bool on) -> float {
                ImVec2 sz=ImGui::CalcTextSize(txt);
                float x=rx-sz.x;
                ImU32 col=on ? IM_COL32(80,220,80,255) : IM_COL32(100,100,100,110);
                if(on) dl->AddText(ImVec2(x+1,ty_b),col,txt); // bold
                dl->AddText(ImVec2(x,ty_b),col,txt);
                return x-14.0f;
            };

            // 오른쪽→왼쪽으로 그리면 화면에서는 왼쪽→오른쪽: LINK FFT IQ TM REC
            float rx=disp_w-8.0f;

            // REC: 맨 오른쪽 고정
            {
                ImVec2 sz=ImGui::CalcTextSize("REC");
                float rec_x=disp_w-8.0f-sz.x;

                ImU32 col; bool bold=false;
                if(v.rec_state==FFTViewer::REC_BUSY){
                    col=IM_COL32(255,200,0,255); bold=true;
                } else if(v.rec_state==FFTViewer::REC_SUCCESS){
                    v.rec_success_timer-=io.DeltaTime;
                    if(v.rec_success_timer<=0.0f) v.rec_state=FFTViewer::REC_IDLE;
                    col=IM_COL32(80,220,80,255); bold=true;
                } else if(v.region.active){
                    col=IM_COL32(255,255,255,255); bold=true;
                } else {
                    col=IM_COL32(100,100,100,110); bold=false;
                }
                if(bold) dl->AddText(ImVec2(rec_x+1,ty_b),col,"REC");
                dl->AddText(ImVec2(rec_x,ty_b),col,"REC");
                rx=rec_x-14.0f;
            }

            // TM IQ FFT LINK: 클릭 가능한 인디케이터
            // TM, REC: 각 PC 독립 / LINK FFT WF IQ: HOST↔JOIN 동기화
            auto click_ind=[&](float& rx2, const char* txt, bool on) -> bool {
                ImVec2 sz=ImGui::CalcTextSize(txt);
                float x=rx2-sz.x;
                ImU32 col=on ? IM_COL32(80,220,80,255) : IM_COL32(100,100,100,110);
                if(on) dl->AddText(ImVec2(x+1,ty_b),col,txt);
                dl->AddText(ImVec2(x,ty_b),col,txt);
                bool clicked=ImGui::IsMouseClicked(ImGuiMouseButton_Left)&&
                    io.MousePos.x>=x&&io.MousePos.x<=x+sz.x&&
                    io.MousePos.y>=ty_b&&io.MousePos.y<=ty_b+sz.y;
                rx2=x-14.0f;
                return clicked;
            };
            rx=draw_ind(rx,"TM", tm_on); // TM: 독립

            if(click_ind(rx,"IQ", iq_on)){
                // IQ: HOST/JOIN 동기화
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
            if(click_ind(rx,"WF", wf_visible && fft_on)){
                // WF = 로컬 FFT 표시 pause 토글 (JOIN도 로컬만)
                v.spectrum_pause.store(!v.spectrum_pause.load());
            }
            if(click_ind(rx,"FFT", fft_on)){
                v.spectrum_pause.store(!v.spectrum_pause.load());
            }
            if(click_ind(rx,"LINK", streaming_on)){
                bool np=!v.capture_pause.load();
                if(v.remote_mode && v.net_cli) v.net_cli->cmd_set_capture_pause(np);
                else { v.capture_pause.store(np); }
            }

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

            ImGui::TextColored(ImVec4(0.4f,0.7f,1.f,1.f),"Chat  [C]");
            ImGui::Separator();

            float msg_h=CH-60.f;
            ImGui::BeginChild("##chat_msgs",ImVec2(0,msg_h),false);

            auto print_chat=[&](const char* from, const char* msg){
                const char* myname = v.net_cli ? v.net_cli->my_name : login_get_id();
                bool is_me=(strcmp(from,myname)==0);
                if(is_me)
                    ImGui::TextColored(ImVec4(0.3f,1.f,0.5f,1.f),"[%s] %s",from,msg);
                else
                    ImGui::TextColored(ImVec4(0.85f,0.85f,0.85f,1.f),"[%s] %s",from,msg);
            };

            if(v.net_cli){
                std::lock_guard<std::mutex> lk(v.net_cli->chat_mtx);
                for(auto& m : v.net_cli->chat_log) print_chat(m.from, m.msg);
                if(v.net_cli->chat_updated.exchange(false)) chat_scroll_bottom=true;
            } else {
                std::lock_guard<std::mutex> lk(host_chat_mtx);
                for(auto& m : host_chat_log){
                    if(m.is_error) ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(1.f,0.3f,0.3f,1.f));
                    print_chat(m.from, m.msg);
                    if(m.is_error) ImGui::PopStyleColor();
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
                               ImGuiInputTextFlags_EnterReturnsTrue))
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
                    // ── 명령어 처리 (3개만 허용) ─────────────────────────
                    if(chat_str == "/shutdown"){
                        glfwSetWindowShouldClose(win, GLFW_TRUE);
                    } else if(chat_str == "/logout"){
                        do_logout = true;
                        glfwSetWindowShouldClose(win, GLFW_TRUE);
                    } else if(chat_str == "/restart"){
                        // 로그아웃 후 기존 계정으로 자동 재로그인
                        do_restart = true;
                        glfwSetWindowShouldClose(win, GLFW_TRUE);
                    } else {
                        // 알 수 없는 명령어
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
                std::lock_guard<std::mutex> lk(v.net_cli->op_mtx);
                for(int i=0;i<v.net_cli->op_list.count;i++)
                    ops_display.push_back(v.net_cli->op_list.ops[i]);
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

    if(do_logout || do_restart){
        // 로그인 화면으로 돌아가기 (프로세스 재시작)
        // /restart: 환경변수로 기존 계정 전달 → 재시작 후 자동 로그인
        printf("%s: restarting...\n", do_restart ? "Restart" : "Logout");
        if(do_restart){
            setenv("BEWE_AUTO_ID",   login_get_id(),             1);
            setenv("BEWE_AUTO_PW",   login_get_pw(),             1);
            setenv("BEWE_AUTO_TIER", std::to_string(login_get_tier()).c_str(), 1);
        }
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