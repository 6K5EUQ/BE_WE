#include "fft_viewer.hpp"
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>

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
                    channels[slot].s=new_drag.s; channels[slot].e=new_drag.e;
                    channels[slot].filter_active=true; channels[slot].mode=Channel::DM_NONE;
                    channels[slot].pan=0; channels[slot].selected=false;
                    channels[slot].sq_calibrated.store(false);
                    channels[slot].ar_wp.store(0); channels[slot].ar_rp.store(0);
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
                if(moved&&channels[i].dem_run.load()){
                    Channel::DemodMode md=channels[i].mode;
                    stop_dem(i); start_dem(i,md);
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
            stop_dem(ci);
            channels[ci].filter_active=false;
            channels[ci].selected=false;
            channels[ci].mode=Channel::DM_NONE;
            if(selected_ch==ci) selected_ch=-1;
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
        bool dem=ch.dem_run.load();

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
        } else if(ch.mode==Channel::DM_DETECT){
            // 디텍션: 보라색
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
    if(freq_zoom<=1.0f){
        // 광대역: 1MHz 또는 5MHz 단위
        float step=5, first=ceilf((ds+cf)/step)*step;
        for(float af=first;af<=de+cf+1e-4f;af+=step){
            float x=gx+(af-cf-ds)/dr*gw; if(x<gx||x>gx+gw) continue;
            if(!ticks_only) dl->AddLine(ImVec2(x,gy),ImVec2(x,gy+gh),IM_COL32(60,60,60,100),1);
            dl->AddLine(ImVec2(x,gy+gh-5),ImVec2(x,gy+gh),IM_COL32(100,100,100,200),1);
            if(!ticks_only){
                dl->AddLine(ImVec2(x,gy+gh),ImVec2(x,gy+gh+5),IM_COL32(100,100,100,200),1);
                char lb[32]; snprintf(lb,32,"%.0f",af);
                ImVec2 ts=ImGui::CalcTextSize(lb);
                dl->AddText(ImVec2(x-ts.x/2,gy+gh+8),IM_COL32(0,255,0,255),lb);
            }
        }
    } else {
        // 줌 상태: 100kHz 그리드 스냅
        // 화면 폭에 표시할 적정 눈금 수 기준으로 step 선택
        // dr(MHz) 기준: 틱 간격을 0.1MHz(100kHz) 단위 배수로 선택
        float step_mhz=0.1f; // 100kHz
        // 화면에 너무 많으면 배수로 늘림
        float max_ticks=gw/60.0f; // 60px당 1눈금
        while((dr/step_mhz)>max_ticks) step_mhz*=2.0f;
        // 100kHz 배수 중 가장 작은 것으로 스냅
        // step_mhz를 0.1 단위로 반올림
        step_mhz=roundf(step_mhz*10.0f)/10.0f;
        if(step_mhz<0.1f) step_mhz=0.1f;

        float abs_start=cf+ds;
        float first=ceilf(abs_start/step_mhz)*step_mhz;
        for(float af=first; af<=cf+de+1e-5f; af+=step_mhz){
            float x=gx+(af-cf-ds)/dr*gw;
            if(x<gx||x>gx+gw) continue;
            if(!ticks_only) dl->AddLine(ImVec2(x,gy),ImVec2(x,gy+gh),IM_COL32(60,60,60,100),1);
            dl->AddLine(ImVec2(x,gy+gh-5),ImVec2(x,gy+gh),IM_COL32(100,100,100,200),1);
            if(!ticks_only){
                dl->AddLine(ImVec2(x,gy+gh),ImVec2(x,gy+gh+5),IM_COL32(100,100,100,200),1);
                // 100kHz 단위로 표시: 예) 414.100, 414.200
                int khz=std::lround(af*1000.0f); // 반올림해서 정수 kHz
                int mhz_int=khz/1000;
                int khz_frac=khz%1000;
                char lb[32];
                if(khz_frac==0) snprintf(lb,sizeof(lb),"%d",mhz_int);
                else            snprintf(lb,sizeof(lb),"%d.%03d",mhz_int,khz_frac);
                ImVec2 ts=ImGui::CalcTextSize(lb);
                dl->AddText(ImVec2(x-ts.x/2,gy+gh+8),IM_COL32(0,255,0,255),lb);
            }
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
    handle_channel_interactions(gx,gw,gy,gh);
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
    handle_channel_interactions(gx,gw,gy,gh);

    // ── Ctrl+우클릭 드래그: 영역 IQ 녹음 선택 ────────────────────────────
    {
        ImGuiIO& mio=ImGui::GetIO();
        ImVec2 mp=mio.MousePos;
        bool ctrl=mio.KeyCtrl;
        bool in_wf=(mp.x>=gx&&mp.x<=gx+gw&&mp.y>=gy&&mp.y<=gy+gh);

        // ── 신규 선택: Ctrl+우클릭 드래그 ──────────────────────────────
        if(ctrl&&ImGui::IsMouseClicked(ImGuiMouseButton_Right)&&in_wf&&tm_iq_file_ready){
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
                float rps=(float)header.sample_rate/(float)fft_size/(float)time_average;
                if(rps<=0) rps=37.5f;
                time_t now=time(nullptr);
                region.time_end  =now-(time_t)((current_fft_idx-region.fft_top)/rps);
                region.time_start=now-(time_t)((current_fft_idx-region.fft_bot)/rps);
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

            // 편집 종료
            if(region.edit_mode!=RegionSel::EDIT_NONE&&ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                region.edit_mode=RegionSel::EDIT_NONE;
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
    float cf=450.0f, sr=61.44f;
    FFTViewer v;
    if(!v.initialize_bladerf(cf,sr)){ printf("BladeRF init failed\n"); return; }
    v.load_alert_mp3();

    std::thread cap(&FFTViewer::capture_and_process,&v);
    v.mix_stop.store(false);
    v.mix_thr=std::thread(&FFTViewer::mix_worker,&v);

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* win=glfwCreateWindow(1400,900,"BEWE",nullptr,nullptr);
    glfwMakeContextCurrent(win); glfwSwapInterval(0);
    glewExperimental=GL_TRUE; glewInit();
    ImGui::CreateContext(); ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win,true);
    ImGui_ImplOpenGL3_Init("#version 330");
    v.create_waterfall_texture();

    // ── System monitor state ──────────────────────────────────────────────
    auto cpu_last_time=std::chrono::steady_clock::now();
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

    // ── Main loop ─────────────────────────────────────────────────────────
    while(!glfwWindowShouldClose(win)){
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

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        v.topbar_sel_this_frame=false;
        // TM 모드: freeze_idx만 최신으로 추적 (버퍼 범위 파악용)
        // tm_display_fft_idx는 Ctrl+휠 시에만 갱신 → 화면 freeze 유지
        if(v.tm_active.load()){
            v.tm_freeze_idx=v.current_fft_idx;
        }
        if(v.texture_needs_recreate){ v.texture_needs_recreate=false; v.create_waterfall_texture(); }

        ImGuiIO& io=ImGui::GetIO();
        bool editing=ImGui::IsAnyItemActive();
        int sci=v.selected_ch;

        // ── Keyboard shortcuts ────────────────────────────────────────────
        if(!editing){
            if(ImGui::IsKeyPressed(ImGuiKey_R,false)){
                if(v.region.active){
                    // 영역 IQ 녹음 저장
                    v.region_save();
                } else if(v.rec_on.load()){
                    v.stop_rec();
                } else {
                    if(v.tm_active.load()) v.tm_rec_start();
                    else v.start_rec();
                }
            }

            if(ImGui::IsKeyPressed(ImGuiKey_P,false)){
                v.spectrum_pause.store(!v.spectrum_pause.load());
            }
            // T키: IQ 롤링 버퍼 활성/비활성
            if(ImGui::IsKeyPressed(ImGuiKey_T,false)){
                bool cur=v.tm_iq_on.load();
                if(cur){
                    // Stop
                    v.tm_iq_on.store(false);
                    // fd 기반 unbuffered: flush 불필요
                    v.tm_add_event_tag(2); // TM_IQ Stop
                    v.tm_iq_was_stopped=true;
                } else {
                    if(v.tm_iq_was_stopped){
                        // Stop 후 재시작: 기존 파일 삭제 후 새로 시작
                        v.tm_iq_close();
                        v.tm_iq_was_stopped=false;
                    }
                    v.tm_iq_open();
                    if(v.tm_iq_file_ready){
                        v.tm_iq_on.store(true);
                        v.tm_add_event_tag(1); // TM_IQ Start
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
                    Channel& ch=v.channels[sci];
                    if(ch.dem_run.load()&&ch.mode==m){ v.stop_dem(sci); }
                    else { v.stop_dem(sci); v.start_dem(sci,m); }
                };
                if(ImGui::IsKeyPressed(ImGuiKey_A,false)) set_mode(Channel::DM_AM);
                if(ImGui::IsKeyPressed(ImGuiKey_F,false)) set_mode(Channel::DM_FM);
                if(ImGui::IsKeyPressed(ImGuiKey_D,false)) set_mode(Channel::DM_DETECT);
                if(ImGui::IsKeyPressed(ImGuiKey_M,false)){
                    Channel& ch=v.channels[sci];
                    if(ch.dem_run.load()&&ch.mode==Channel::DM_MAGIC){ v.stop_dem(sci); }
                    else {
                        v.stop_dem(sci);
                        ch.magic_det.store(0,std::memory_order_relaxed);
                        v.start_dem(sci,Channel::DM_MAGIC);
                    }
                }
                if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow,false))  v.channels[sci].pan=-1;
                if(ImGui::IsKeyPressed(ImGuiKey_RightArrow,false)) v.channels[sci].pan= 1;
                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow,false))    v.channels[sci].pan= 0;
            }
            if(ImGui::IsKeyPressed(ImGuiKey_Escape,false)){
                if(sci>=0){ v.channels[sci].selected=false; v.selected_ch=-1; }
            }
            if(ImGui::IsKeyPressed(ImGuiKey_Delete,false)){
                if(sci>=0&&v.channels[sci].filter_active){
                    v.stop_dem(sci);
                    v.channels[sci].filter_active=false;
                    v.channels[sci].selected=false;
                    v.channels[sci].mode=Channel::DM_NONE;
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
        if(!editing&&(ImGui::IsKeyPressed(ImGuiKey_Enter,false)||ImGui::IsKeyPressed(ImGuiKey_KeypadEnter,false)))
            focus_freq=true;
        if(fdeact){ fdeact=false; ImGui::SetWindowFocus(nullptr); }
        if(focus_freq){ ImGui::SetKeyboardFocusHere(); focus_freq=false; }
        {
            char fbuf[32]; snprintf(fbuf,sizeof(fbuf),"%.3f MHz",new_freq);
            float tw=ImGui::CalcTextSize(fbuf).x;
            float box_w=96.0f;
            float px=std::max(2.0f,(box_w-tw)*0.5f-1.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(px,ImGui::GetStyle().FramePadding.y));
            ImGui::SetNextItemWidth(box_w);
            ImGui::InputFloat("##freq",&new_freq,0,0,"%.3f MHz");
            ImGui::PopStyleVar();
        }
        if(ImGui::IsItemDeactivatedAfterEdit()){ v.pending_cf=new_freq; v.freq_req=true; fdeact=true; }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Center Frequency  [Enter] to edit");
        ImGui::SameLine();

        // ── FFT size combo ────────────────────────────────────────────────
        static const int fft_sizes[]={512,1024,2048,4096,8192,16384};
        static const char* fft_lbls[]={"512","1024","2048","4096","8192","16384"};
        static int fft_si=4;
        ImGui::Text("FFT:"); ImGui::SameLine();
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

        // ── Squelch slider (선택 채널만) ──────────────────────────────────
        if(sci>=0&&v.channels[sci].filter_active){
            Channel& sch=v.channels[sci];
            float sig  =sch.sq_sig .load(std::memory_order_relaxed);
            bool  gopen=sch.sq_gate.load(std::memory_order_relaxed);
            ImGui::Text("SQL:"); ImGui::SameLine();
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
            char lbl[16]; snprintf(lbl,sizeof(lbl),"%.0fdB",thr_db);
            ImVec2 lsz=ImGui::CalcTextSize(lbl);
            bdl->AddText(ImVec2(sp.x+SLIDER_W/2-lsz.x/2,sp.y+(SLIDER_H-lsz.y)/2),IM_COL32(230,230,230,255),lbl);
            ImGui::SetCursorScreenPos(sp);
            ImGui::InvisibleButton("##sql",ImVec2(SLIDER_W,SLIDER_H));
            if(ImGui::IsItemHovered()){
                float wheel=ImGui::GetIO().MouseWheel;
                if(wheel!=0.0f){
                    float nthr=thr_db+(wheel>0?3.0f:-3.0f);
                    nthr=nthr<DB_MIN?DB_MIN:nthr>DB_MAX?DB_MAX:nthr;
                    sch.sq_threshold.store(nthr,std::memory_order_relaxed);
                }
            }
            if(ImGui::IsItemActive()){
                float mx=ImGui::GetIO().MousePos.x;
                float nthr=DB_MIN+((mx-sp.x)/SLIDER_W)*DB_RNG;
                nthr=nthr<DB_MIN?DB_MIN:nthr>DB_MAX?DB_MAX:nthr;
                sch.sq_threshold.store(nthr,std::memory_order_relaxed);
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
                const char* mname[5]={"","AM","FM","DET","M"};
                const char* magic_names[]={"","AM","FM","DSB","SSB","CW"};
                const char* pname[3]={" L"," L+R"," R"};
                bool dem_active=ch.dem_run.load();
                int pi=ch.pan+1; if(pi<0)pi=0; if(pi>2)pi=2;
                char cb[96];
                if(dem_active && ch.mode==Channel::DM_MAGIC){
                    int mdet=ch.magic_det.load(std::memory_order_relaxed);
                    if(mdet==0)
                        snprintf(cb,sizeof(cb),"[%d] M %.3f MHz @ %.0f kHz  ",
                                 i+1,cf_mhz,bw_khz);
                    else
                        snprintf(cb,sizeof(cb),"[%d] M:%s %.3f MHz @ %.0f kHz  ",
                                 i+1,magic_names[mdet],cf_mhz,bw_khz);
                } else if(dem_active){
                    snprintf(cb,sizeof(cb),"[%d]%s%s %.3f MHz @ %.0f kHz  ",
                             i+1,mname[(int)ch.mode],pname[pi],cf_mhz,bw_khz);
                } else {
                    snprintf(cb,sizeof(cb),"[%d] %.3f MHz @ %.0f kHz  ",
                             i+1,cf_mhz,bw_khz);
                }
                ImVec2 cs2=ImGui::CalcTextSize(cb); rx-=cs2.x;
                bool is_selected=(v.selected_ch==i);
                bool gate_open=ch.sq_gate.load();
                bool tb_dem=ch.dem_run.load();
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
                else if(ch.mode==Channel::DM_MAGIC||ch.mode==Channel::DM_DETECT)
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
        if(ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)){
            // 오른쪽으로 드래그 → 우측 패널 줄어듦, 왼쪽으로 → 우측 패널 커짐
            v.right_panel_ratio -= io.MouseDelta.x / disp_w;
            v.right_panel_ratio = std::max(0.0f, std::min(1.0f, v.right_panel_ratio));
        }
        if(ImGui::IsItemHovered()) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

        // ── 우측 패널 ─────────────────────────────────────────────────────
        if(right_visible){
            float rpx  = vdiv_x + vdiv_w;
            float rp_w = disp_w - rpx;
            const float SUBBAR_H = TOPBAR_H * 0.5f;
            float rp_content_y = content_y + SUBBAR_H;
            float rp_content_h = content_h - SUBBAR_H;

            // right_panel_x 갱신
            v.right_panel_x = rpx;

            // ── SA 픽셀 준비되면 GL 업로드 ──────────────────────────────
            if(v.sa_pixel_ready.load()){ v.sa_upload_texture(); v.sa_anim_timer=0.0f; }

            // ── 서브바 배경 ──────────────────────────────────────────────
            dl->AddRectFilled(ImVec2(rpx,content_y),ImVec2(disp_w,rp_content_y),IM_COL32(35,35,40,255));
            dl->AddLine(ImVec2(rpx,rp_content_y-1),ImVec2(disp_w,rp_content_y-1),IM_COL32(60,60,70,255),1);

            // ── SA 버튼 ──────────────────────────────────────────────────
            float btn_x = rpx + 6;
            float btn_y = content_y + (SUBBAR_H - ImGui::GetFontSize())/2;
            bool sa_hov = io.MousePos.x>=btn_x && io.MousePos.x<=btn_x+24 &&
                          io.MousePos.y>=content_y && io.MousePos.y<rp_content_y;
            ImU32 sa_btn_col = v.sa_panel_open
                ? IM_COL32(80,180,255,255)
                : (sa_hov ? IM_COL32(160,160,180,255) : IM_COL32(110,110,130,255));
            dl->AddText(ImVec2(btn_x, btn_y), sa_btn_col, "SA");
            if(sa_hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                v.sa_panel_open = !v.sa_panel_open;

            // ── FFT size 선택 (우측정렬) ─────────────────────────────────
            {
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
                                // 이미 로드된 파일 있으면 새 FFT size로 재계산
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
                // 레이블
                float lbl_x = combo_x - ImGui::CalcTextSize("FFT").x - 4;
                dl->AddText(ImVec2(lbl_x, combo_y+1), IM_COL32(130,130,150,255), "FFT");
            }

            // ── 패널 콘텐츠 영역 ─────────────────────────────────────────
            dl->AddRectFilled(ImVec2(rpx,rp_content_y),ImVec2(disp_w,content_y+content_h),IM_COL32(12,12,15,255));

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
        }

        // ── Bottom bar ────────────────────────────────────────────────────
        float bot_y=disp_h-TOPBAR_H;
        dl->AddRectFilled(ImVec2(0,bot_y),ImVec2(disp_w,disp_h),IM_COL32(30,30,30,255));
        dl->AddLine(ImVec2(0,bot_y),ImVec2(disp_w,bot_y),IM_COL32(60,60,60,255),1);
        {
            float ty_b=bot_y+(TOPBAR_H-ImGui::GetFontSize())/2;

            // ── 중앙: 시스템 모니터 ───────────────────────────────────────
            char stats[128];
            snprintf(stats,sizeof(stats),
                     "CPU %.0f%%  @  %.2f GHz     RAM %.0f%%     IO %.0f%%",
                     (double)v.sysmon_cpu,(double)v.sysmon_ghz,
                     (double)v.sysmon_ram,(double)v.sysmon_io);
            ImVec2 ssz=ImGui::CalcTextSize(stats);
            dl->AddText(ImVec2((disp_w-ssz.x)/2,ty_b),IM_COL32(160,160,160,255),stats);

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

            // TM IQ FFT LINK (오른쪽→왼쪽)
            rx=draw_ind(rx,"TM",  tm_on);
            rx=draw_ind(rx,"IQ",  iq_on);
            rx=draw_ind(rx,"WF",  wf_visible && fft_on);
            rx=draw_ind(rx,"FFT", fft_on);
            rx=draw_ind(rx,"LINK",streaming_on);

        }
        ImGui::End();

        ImGui::Render();
        int dw2,dh2; glfwGetFramebufferSize(win,&dw2,&dh2);
        glViewport(0,0,dw2,dh2);
        glClearColor(0.1f,0.1f,0.1f,1); glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    v.is_running=false;
    v.stop_all_dem();
    if(v.rec_on.load()) v.stop_rec();
    // 롤링 IQ 안전 종료: 배치 플러시 + WAV 헤더 갱신
    if(v.tm_iq_file_ready){
        v.tm_iq_on.store(false);
        v.tm_iq_close();
    }
    v.mix_stop.store(true); if(v.mix_thr.joinable()) v.mix_thr.join();
    cap.join();
    if(v.waterfall_texture) glDeleteTextures(1,&v.waterfall_texture);
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(); glfwTerminate();
    v.sa_cleanup();
    printf("Closed\n");
}