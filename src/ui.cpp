#include "fft_viewer.hpp"
#include "ais_decode.hpp"
#include "login.hpp"
#include "net_server.hpp"
#include "net_client.hpp"
#include "bewe_paths.hpp"
#include "globe.hpp"
#include "central_client.hpp"
#include "lora_demod.hpp"
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
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

void bewe_log(const char* fmt, ...){
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    bewe_log_push(0, "%s", buf);
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
    // JOIN 모드: 스컬치 계산은 HOST 전담. CH_SYNC로 받은 sq_threshold/sq_sig/sq_gate만 표시.
    if(remote_mode) return;
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

        // 채널 주파수 범위 > FFT 빈
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
                // 20th percentile + 10dB
                float tmp[60];
                memcpy(tmp, ch.sq_calib_buf, sizeof(tmp));
                std::nth_element(tmp, tmp + 12, tmp + 60);
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

        // 스컬치 누적 시간 추적 (프레임 기반 — SDR 멈추면 시간도 정지)
        // Holding(dem_paused) 상태에서는 증가 정지 — JOIN도 HOST 값이 멈춘 상태로 받음
        if(ch.filter_active && !ch.dem_paused.load()){
            // JOIN: CH_SYNC에서 HOST 값을 직접 사용 (로컬 증가 안 함)
            if(!remote_mode){
                float fps = (float)header.sample_rate / (float)std::max(1,fft_input_size) / (float)std::max(1,time_average);
                if(fps > 0){
                    float dt = 1.0f / fps;
                    if(!sdr_stream_error.load()){
                        ch.sq_total_time += dt;
                        if(gate) ch.sq_active_time += dt;
                    }
                }
            }
        } else if(!ch.filter_active){
            ch.sq_active_time = 0;
            ch.sq_total_time = 0;
        }
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
        bewe_log_push(2,"[CH_DRAG] start af=%.3f net_cli=%p\n", af, (void*)net_cli);
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
            bewe_log_push(2,"[CH_DRAG] release bw=%.4f s=%.3f e=%.3f\n", bw, new_drag.s, new_drag.e);
            if(bw>0.001f){
                int slot=-1;
                for(int i=0;i<MAX_CHANNELS;i++) if(!channels[i].filter_active){slot=i;break;}
                bewe_log_push(2,"[CH_DRAG] slot=%d net_cli=%p net_srv=%p\n", slot, (void*)net_cli, (void*)net_srv);
                if(slot>=0){
                    if(net_cli){
                        // JOIN: 서버에 CMD_CREATE_CH 전송 (서버가 처리 후 sync)
                        net_cli->cmd_create_ch(slot, new_drag.s, new_drag.e);
                        ch_created_by_me[slot] = true; // 내가 만든 채널 > 초기 Mute 제외
                        ch_pending_create[slot] = true; // HOST 확인 전까지 stale sync 무시
                        // 중앙 릴레이의 recv_audio[] 잔존 상태(이전 세션/다른 JOIN에 의한 mute)를
                        // 명시적으로 ON으로 초기화. 없으면 UI L+R 녹색인데 소리 안 나는 버그 발생.
                        net_cli->cmd_toggle_recv(slot, true);
                        // 로컬에도 즉시 값 설정 (CH_SYNC 도착 전까지 UI 일관성 유지)
                        channels[slot].reset_slot();
                        channels[slot].s = new_drag.s;
                        channels[slot].e = new_drag.e;
                        channels[slot].filter_active = true;
                    } else {
                        channels[slot].reset_slot();
                        channels[slot].s=new_drag.s; channels[slot].e=new_drag.e;
                        channels[slot].filter_active=true;
                        channels[slot].audio_mask.store(0xFFFFFFFFu);
                        strncpy(channels[slot].owner, host_name[0]?host_name:"Host", 31);
                        srv_audio_mask[slot] = channels[slot].audio_mask.load();
                        update_dem_by_freq(header.center_frequency/1e6f); // 범위 밖이면 Holding 즉시 진입
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
            // drag ended > restart demod if active
            bool any_resized = false;
            for(int i=0;i<MAX_CHANNELS;i++){
                if(!channels[i].resize_drag) continue;
                channels[i].resize_drag=false;
                any_resized = true;
                if(channels[i].dem_run.load()){
                    Channel::DemodMode md=channels[i].mode;
                    stop_dem(i); start_dem(i,md);
                }
                // JOIN: send new range to HOST
                if(net_cli && remote_mode)
                    net_cli->cmd_update_ch_range(i, channels[i].s, channels[i].e);
            }
            if(any_resized){
                // 리사이즈 결과 범위 밖/안 전환 재평가 (LOCAL/HOST)
                if(!remote_mode) update_dem_by_freq(header.center_frequency/1e6f);
                if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
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
            bool any_moved = false;
            for(int i=0;i<MAX_CHANNELS;i++){
                if(!channels[i].move_drag) continue;
                bool moved=(channels[i].s!=channels[i].move_s0||channels[i].e!=channels[i].move_e0);
                channels[i].move_drag=false;
                if(moved){
                    any_moved = true;
                    if(channels[i].dem_run.load()){
                        Channel::DemodMode md=channels[i].mode;
                        stop_dem(i); start_dem(i,md);
                    }
                    // JOIN: send new range to HOST
                    if(net_cli && remote_mode)
                        net_cli->cmd_update_ch_range(i, channels[i].s, channels[i].e);
                }
            }
            if(any_moved){
                // 이동 결과 범위 밖/안 전환 재평가 (LOCAL/HOST)
                if(!remote_mode) update_dem_by_freq(header.center_frequency/1e6f);
                if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
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
                // JOIN: 서버에 삭제 요청 > 서버가 broadcast 처리
                net_cli->cmd_delete_ch(ci);
            }
            // 녹음 중이면 중지 (R키 + audio + I키 IQ)
            if(rec_on.load() && rec_ch==ci) stop_rec();
            if(channels[ci].audio_rec_on.load()){
                if(net_cli) stop_join_audio_rec(ci);
                else stop_audio_rec(ci);
            }
            if(channels[ci].iq_rec_on.load()) stop_iq_rec(ci);
            // 로컬 즉시 반영 (서버 sync가 확인해줌)
            stop_dem(ci); stop_digi(ci); digi_panel_on[ci]=false;
            channels[ci].reset_slot();
            if(net_cli) net_cli->audio[ci].clear();
            local_ch_out[ci] = 1;
            ch_created_by_me[ci] = false; ch_pending_create[ci] = false;
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

// 배열 인덱스 > 주파수 순 표시 번호 (1-based). 비활성 채널은 0 반환.
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
        // assign() 대신 resize() - fill 불필요 (아래 루프가 전부 덮어씀)
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
    // AddPolyline 1회 호출 - cache 히트 시 ImVec2 재계산 스킵
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

        // i=0: 상단 max 값은 별도 최상단 라벨로 표시되므로 숨김
        if(i == 0) continue;
        // i=10: 최소값 텍스트만 그리고 편집 버튼은 생략
        if(i == 10){
            char lb[16]; snprintf(lb, sizeof(lb), "%.0f", db);
            ImVec2 ts = ImGui::CalcTextSize(lb);
            float lx = gx - 10 - ts.x;
            float ly = y - ts.y * 0.5f;
            dl->AddText(ImVec2(lx, ly), IM_COL32(200,200,200,255), lb);
            continue;
        }

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
                join_manual_scale = true; // JOIN 모드: 수동 입력 > HOST 값 덮어쓰기 차단
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
    if(!eid_panel_open && !log_panel_open) handle_new_channel_drag(gx,gw);
    if(!region.active && !eid_panel_open && !log_panel_open) handle_channel_interactions(gx,gw,gy,gh);
    if(hov && !eid_panel_open && !log_panel_open){
        ImVec2 mm=ImGui::GetIO().MousePos;
        float af=x_to_abs(mm.x,gx,gw);
        char info[48]; snprintf(info,48,"%.3f MHz",af);
        ImVec2 ts=ImGui::CalcTextSize(info);
        float tx=gx+gw-ts.x-4, ty=gy+2;
        dl->AddRectFilled(ImVec2(tx-2,ty),ImVec2(tx+ts.x+2,ty+ts.y+4),IM_COL32(20,20,20,220));
        dl->AddRect(ImVec2(tx-2,ty),ImVec2(tx+ts.x+2,ty+ts.y+4),IM_COL32(100,100,100,255));
        dl->AddText(ImVec2(tx,ty+2),IM_COL32(0,255,0,255),info);
        if(!eid_panel_open) handle_zoom_scroll(gx,gw,mm.x);
    }
    // 파워 축 드래그 (편집 중이 아닐 때만)
    // 드래그 방향: 위로 > 값 증가, 아래로 > 값 감소
    // 상반부 드래그 > max 조절, 하반부 > min 조절
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
                join_manual_scale = true; // JOIN 모드: 수동 조절 > HOST 값 덮어쓰기 차단
                cached_sp_idx=-1;
            }
        }
    }

    // ── 주파수 축 드래그 > center frequency 이동 (SDR++ 스타일) ──────────
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
                // 화면 픽셀 > MHz 변환: 현재 표시 범위 기준
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
        // 매 프레임 tm_update_display() 호출 → 60초 한계 follow 동작 반영
        if(tm_active.load()) tm_update_display();
        int disp_idx=tm_active.load() ? tm_display_fft_idx : current_fft_idx;
        float vn=(float)(disp_idx%MAX_FFTS_MEMORY)/MAX_FFTS_MEMORY;
        float vt=vn+1.0f/MAX_FFTS_MEMORY;
        float vb=vt-dh/MAX_FFTS_MEMORY;
        ImTextureID tid=(ImTextureID)(intptr_t)waterfall_texture;
        dl->AddImage(tid,ImVec2(gx,gy),ImVec2(gx+gw,gy+dh),ImVec2(us,vt),ImVec2(ue,vb),IM_COL32(255,255,255,255));
        // IQ 가용 오버레이 제거됨 - 좌측 태그로 대체
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
                if(tx<label_x) tx=label_x;
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
    if(!eid_panel_open && !log_panel_open) handle_new_channel_drag(gx,gw);
    if(!region.active && !eid_panel_open && !log_panel_open) handle_channel_interactions(gx,gw,gy,gh);

    // ── Ctrl+우클릭 드래그: 영역 IQ 녹음 선택 ────────────────────────────
    {
        ImGuiIO& mio=ImGui::GetIO();
        ImVec2 mp=mio.MousePos;
        bool ctrl=mio.KeyCtrl;
        bool in_wf=(mp.x>=gx&&mp.x<=gx+gw&&mp.y>=gy&&mp.y<=gy+gh);

        // ── 신규 선택: Ctrl+우클릭 드래그 ──────────────────────────────
        if(!eid_panel_open&&ctrl&&ImGui::IsMouseClicked(ImGuiMouseButton_Right)&&in_wf&&(tm_iq_file_ready||remote_mode)){
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
                // wall_time_ms 기반 타임스탬프 (JOIN 포함 정확한 시간, ms 정밀도)
                {
                    int64_t wt_top_ms = fft_idx_to_wall_time_ms(region.fft_top);
                    int64_t wt_bot_ms = fft_idx_to_wall_time_ms(region.fft_bot);
                    if(wt_top_ms > 0 && wt_bot_ms > 0){
                        region.time_end_ms   = wt_top_ms;
                        region.time_start_ms = wt_bot_ms;
                    } else {
                        // fallback: rps 기반 (ms 단위)
                        float rps=(float)header.sample_rate/(float)fft_input_size/(float)time_average;
                        if(rps<=0) rps=37.5f;
                        int64_t now_ms=(int64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                        region.time_end_ms  =now_ms-(int64_t)((current_fft_idx-region.fft_top)*1000.0f/rps);
                        region.time_start_ms=now_ms-(int64_t)((current_fft_idx-region.fft_bot)*1000.0f/rps);
                    }
                    int64_t sp_top = row_write_pos[region.fft_top % MAX_FFTS_MEMORY];
                    int64_t sp_bot = row_write_pos[region.fft_bot % MAX_FFTS_MEMORY];
                    if(sp_top > 0 && sp_bot > 0){
                        region.samp_end   = sp_top;
                        region.samp_start = sp_bot;
                    } else {
                        region.samp_start = region.samp_end = 0;
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
            // 픽셀 > 주파수 변환 배율 (MHz/px)
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
               false){ // SA panel removed
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
                if(false){ // SA panel removed
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
                int64_t now2_ms=(int64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                region.time_end_ms  =now2_ms-(int64_t)((current_fft_idx-region.fft_top)*1000.0f/rps2);
                region.time_start_ms=now2_ms-(int64_t)((current_fft_idx-region.fft_bot)*1000.0f/rps2);
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
        if(!eid_panel_open) handle_zoom_scroll(gx,gw,mm.x);
    }
}

// g_db_list / g_db_list_mtx: fft_viewer.cpp에서 정의 (CLI에서도 접근 가능)
extern std::vector<DbFileEntry> g_db_list;
extern std::mutex g_db_list_mtx;

// ─────────────────────────────────────────────────────────────────────────────
void run_streaming_viewer(){
    float cf=450.0f;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES,4);
    glfwWindowHint(GLFW_DECORATED,GLFW_TRUE);
    GLFWmonitor* primary=glfwGetPrimaryMonitor();
    const GLFWvidmode* vmode=glfwGetVideoMode(primary);
    glfwWindowHint(GLFW_RED_BITS,  vmode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS,vmode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, vmode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE,vmode->refreshRate);
    GLFWwindow* win=glfwCreateWindow(1400,900,"BEWE",nullptr,nullptr);
    glfwMakeContextCurrent(win); glfwSwapInterval(0);
    glewExperimental=GL_TRUE; glewInit();
    glEnable(GL_MULTISAMPLE);
    ImGui::CreateContext(); ImGui::StyleColorsDark();
    // Bits탭 전용 26px 폰트 (2x 크기, 선명한 래스터링)
    static ImFont* g_bits_font = nullptr;
    {
        ImGuiIO& fio = ImGui::GetIO();
        fio.Fonts->AddFontDefault(); // 기본 13px
        ImFontConfig cfg;
        cfg.SizePixels = 26.0f;
        g_bits_font = fio.Fonts->AddFontDefaultVector(&cfg);
    }
    ImGui_ImplGlfw_InitForOpenGL(win,true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ── Fullscreen toggle (F11) ────────────────────────────────────────────
    static bool is_fullscreen = false;
    static int saved_win_x=0, saved_win_y=0, saved_win_w=1400, saved_win_h=900;
    auto toggle_fullscreen = [&](){
        if(!ImGui::IsKeyPressed(ImGuiKey_F11,false)) return;
        if(is_fullscreen){
            glfwSetWindowMonitor(win,nullptr,saved_win_x,saved_win_y,saved_win_w,saved_win_h,0);
            glfwSetWindowAttrib(win,GLFW_DECORATED,GLFW_TRUE);
            is_fullscreen=false;
        } else {
            glfwGetWindowPos(win,&saved_win_x,&saved_win_y);
            glfwGetWindowSize(win,&saved_win_w,&saved_win_h);
            GLFWmonitor* mon=glfwGetPrimaryMonitor();
            const GLFWvidmode* vm=glfwGetVideoMode(mon);
            glfwSetWindowMonitor(win,mon,0,0,vm->width,vm->height,vm->refreshRate);
            is_fullscreen=true;
        }
    };

    // ── Early chat state (login + globe loops 공유) ───────────────────────
    struct EarlyChatMsg { char from[32]; char msg[256]; bool is_error=false; };
    std::vector<EarlyChatMsg> early_chat_log;
    bool  early_chat_open   = false;
    bool  early_chat_scroll = false;
    char  early_chat_input[256] = {};

    // early loop 명령 플래그
    bool early_do_shutdown    = false;
    bool early_do_logout      = false;
    bool early_chat_focus_req  = false; // Enter > 입력칸 포커스 요청
    bool early_chat_cursor_end = false; // 다음 프레임에 커서를 끝으로 이동 (/ 입력 후 선택 방지)

    auto draw_early_chat = [&](int fw, int fh){
        // RShift 토글
        ImGuiIO& eio = ImGui::GetIO();
        if(ImGui::IsKeyPressed(ImGuiKey_RightShift, false) && !eio.WantTextInput)
            early_chat_open = !early_chat_open;

        // 채팅창 열린 상태에서 Enter > 입력칸 포커스
        if(early_chat_open &&
           (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) &&
           !eio.WantTextInput)
            early_chat_focus_req = true;

        // '/' 키 > 채팅창 열고 '/' 미리 입력 (빠른 명령 입력)
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
        glfwSwapInterval(1); // VSync ON - 로그인 화면은 정적, CPU 낭비 방지
        bool logged_in = false;
        while(!logged_in && !glfwWindowShouldClose(win)){
            glfwPollEvents();
            int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
            glViewport(0,0,fw,fh);
            glClearColor(0.047f,0.071f,0.137f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
            toggle_fullscreen();
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
    static std::atomic<bool> reconn_busy{false};
    bool do_main_menu = false;
    bool do_logout = false;
    bool do_chassis_reset = false;
    int  chassis_reset_mode = 0; // 0=LOCAL, 1=HOST
    bool usb_reset_pending = false; // chassis 1 reset 시 USB reset 수행 플래그
    std::atomic<bool> pending_chassis1_reset{false}; // 네트워크 스레드 > 메인 루프 전달
    std::atomic<bool> pending_chassis2_reset{false}; // 네트워크 스레드 > 메인 루프 전달
    std::atomic<bool> pending_rx_stop{false};        // JOIN > HOST: /rx stop
    std::atomic<bool> pending_rx_start{false};       // JOIN > HOST: /rx start
    // HOST 모드 로컬 채팅 로그 (do-while 외부에서 선언해야 콜백 람다에서 접근 가능)
    struct LocalChatMsg { char from[32]; char msg[256]; bool is_error=false; };
    std::vector<LocalChatMsg> host_chat_log;
    std::mutex host_chat_mtx;
    // 함수 레벨 파일 목록 (스코프 공유 필요)
    // Record 탭: 세션 중 실시간 녹음 (record/iq, record/audio)
    static std::vector<std::string> rec_iq_files;
    static std::vector<std::string> rec_audio_files;
    static std::atomic<bool> g_arch_rescan{false}; // 파일 목록 강제 재스캔 플래그 (thread-safe)
    // Private 탭: 이전 세션 녹음 (private/iq, private/audio)
    static std::vector<std::string> priv_iq_files;
    static std::vector<std::string> priv_audio_files;
    // Public 탭 (HOST 로컬): public/iq, public/audio
    static std::vector<std::string> pub_iq_files;
    static std::vector<std::string> pub_audio_files;
    // Share 탭: 다운로드된 파일 (share/iq, share/audio)
    static std::vector<std::string> share_iq_files;
    static std::vector<std::string> share_audio_files;
    // 레거시: priv_files > priv_iq+priv_audio 합산 (일부 기존 코드 참조용)
    static std::vector<std::string> priv_files; // 스캔 후 priv_iq+priv_audio 합산
    static std::map<std::string,std::string> priv_extra_paths; // filename > full path
    static std::vector<std::string> shared_files; // 스캔 후 pub_iq+pub_audio 합산
    static std::vector<std::string> downloaded_files; // 스캔 후 share_iq+share_audio 합산
    // JOIN: HOST Public 파일 목록 (filename, size_bytes, uploader)
    struct JoinShareEntry { std::string filename; uint64_t size_bytes=0; std::string uploader; };
    static std::vector<JoinShareEntry> join_share_files;
    static std::mutex join_share_mtx;
    // HOST: Public 파일 다운로드 리스너 추적 (filename > 다운로드한 op_name 목록)
    static std::map<std::string,std::vector<std::string>> pub_listeners;
    // Public 파일 소유자 추적 (filename > uploader_name): 업로드한 사람만 삭제 가능
    static std::map<std::string,std::string> pub_owners;
    static std::atomic<bool> ch_sync_dirty_flag{false};
    do {
    do_main_menu = false;
    FFTViewer v;
    extern FFTViewer* g_log_viewer;
    g_log_viewer = &v;
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
    // 중앙서버가 보낸 OP_LIST (HOST 모드에서 오퍼레이터 창 표시용)
    static PktOperatorList s_relay_op_list{};
    static std::mutex s_relay_op_mtx;
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
                            s.last_seen  = now + 2.0;
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

    glfwSwapInterval(1); // VSync ON - globe loop는 무거운 연산 없음
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
        toggle_fullscreen();
        ImGuiIO& io = ImGui::GetIO();

        // Purge stale stations: last_seen이 현재 시각보다 과거면 즉시 제거
        // (last_seen = now + 2 로 설정되므로 poll에서 2초간 갱신 없으면 사라짐)
        {
            std::lock_guard<std::mutex> lk(v.discovered_stations_mtx);
            double now2 = glfwGetTime();
            v.discovered_stations.erase(
                std::remove_if(v.discovered_stations.begin(),
                               v.discovered_stations.end(),
                               [now2](const FFTViewer::DiscoveredStation& s){
                                   return now2 > s.last_seen;
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

            // Click (not drag) > pick lat/lon
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

            // Station : NAME  - 가운데 정렬
            char name_buf[80];
            snprintf(name_buf, sizeof(name_buf), "Station : %s", pending_join.name.c_str());
            float nw = ImGui::CalcTextSize(name_buf).x;
            ImGui::SetCursorPosX((PW - nw) * 0.5f);
            ImGui::TextColored(ImVec4(0.5f,0.9f,1.f,1.f), "%s", name_buf);

            // 좌표 - 약간 작은 폰트, 표준 N/S E/W 표시
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
                    // ── Central Server 경유 JOIN ──────────────────────────────
                    int rfd = central_cli.join_room(s_central_host, s_central_port,
                                                  pending_join.station_id);
                    if(rfd >= 0){
                        cli->stat_room_id = pending_join.station_id;
                        join_ok = cli->connect_fd(rfd, login_get_id(), login_get_pw(),
                                                  (uint8_t)login_get_tier());
                        if(!join_ok) close(rfd);
                    }
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
    // /reset(JOIN): cli가 ��으면 저장된 connect 정보로 Central Relay 경유 자동 재접속
    if(mode_sel==2 && !cli && !s_central_join_station_id.empty() && s_central_host[0] != '\0'){
        int rfd = central_cli.join_room(s_central_host, s_central_port,
                                       s_central_join_station_id);
        if(rfd >= 0){
            cli = new NetClient();
            if(!cli->connect_fd(rfd, connect_id, connect_pw, connect_tier)){
                close(rfd);
                delete cli; cli = nullptr;
                mode_sel = 0; // ���속 실패 시 LOCAL로 fallback
            }
        } else {
            mode_sel = 0;
        }
    }

    if(mode_sel==2 && cli){
        // CONNECT 모드: 하드웨어 없이 원격 수신
        v.remote_mode = true;
        v.net_cli     = cli;
        v.my_op_index = cli->my_op_index;
        strncpy(v.host_name, cli->my_name, 31);
        v.station_name = pending_join.name;
        // 워터폴 타임스탬프 초기화
        { std::lock_guard<std::mutex> wlk(v.wf_events_mtx); v.wf_events.clear(); }
        v.last_tagged_sec = -1;

        // 디지털 복조 로그 콜백
        cli->on_digi_log = [&](uint8_t tab, uint8_t ch_idx, const char* msg){
            v.digi_log_push(tab, "%s", msg);
        };

        // 채널 sync 콜백 등록
        cli->on_channel_sync = [&](const PktChannelSync& sync){
            for(int i=0;i<MAX_CHANNELS;i++){
                bool was_active = v.channels[i].filter_active;
                auto  was_mode  = v.channels[i].mode;
                bool now_active = (sync.ch[i].active != 0);
                auto now_mode   = (Channel::DemodMode)sync.ch[i].mode;

                // CMD_CREATE_CH 송신 후 HOST 미반영 상태의 stale sync 무시
                // (HOST가 현재 inactive라고 응답 중 — 곧 active로 바뀔 예정)
                if(v.ch_pending_create[i] && !now_active) continue;
                if(now_active) v.ch_pending_create[i] = false;

                v.channels[i].filter_active = now_active;
                // 드래그 중인 채널은 s/e를 덮어쓰지 않음 (덜덜 떨림 방지)
                bool dragging = v.channels[i].move_drag || v.channels[i].resize_drag;
                if(!dragging){
                    // DEBUG: 비정상 BW 수신 감지
                    float in_bw = fabsf(sync.ch[i].e - sync.ch[i].s);
                    if(in_bw > 1.0f && sync.ch[i].active){
                        bewe_log_push(0, "[JOIN-DBG] rx ch%d HUGE bw=%.4f s=%.4f e=%.4f\n",
                                      i, in_bw, sync.ch[i].s, sync.ch[i].e);
                    }
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
                v.channels[i].dem_paused.store(sync.ch[i].dem_paused != 0,
                                               std::memory_order_relaxed);
                strncpy(v.channels[i].owner, sync.ch[i].owner_name, 31);
                v.srv_audio_mask[i] = sync.ch[i].audio_mask; // 전체 서버 마스크 보존
                // HOST 녹음 시간 동기화
                v.channels[i].synced_iq_rec_secs   = sync.ch[i].iq_rec_secs;
                v.channels[i].synced_audio_rec_secs= sync.ch[i].audio_rec_secs;
                v.channels[i].sq_active_time       = (float)sync.ch[i].sq_active_secs;
                v.channels[i].sq_total_time        = (float)sync.ch[i].sq_total_secs;
                // JOIN 로컬 녹음 중이면 HOST 값으로 덮어쓰지 않음
                if(!v.channels[i].iq_rec_on.load())
                    v.channels[i].iq_rec_on.store(sync.ch[i].iq_rec_on != 0);
                if(!v.channels[i].audio_rec_on.load())
                    v.channels[i].audio_rec_on.store(sync.ch[i].audio_rec_on != 0);

                // JOIN 모드: 복조는 HOST에서 수행, 로컬 dem 불필요
                // 오디오는 AUDIO_FRAME 수신으로 net_cli->audio 링 통해 출력
                // (dem_run은 실행하지 않음)

                // 디지털 복조 상태 동기화
                v.channels[i].digital_mode    = (Channel::DigitalMode)sync.ch[i].digital_mode;
                v.channels[i].digi_run.store(sync.ch[i].digi_run != 0);
                v.channels[i].digi_demod_type = sync.ch[i].digi_demod_type;
                v.channels[i].digi_baud_rate  = sync.ch[i].digi_baud_rate;
                v.channels[i].auto_id.state.store(sync.ch[i].auto_id_state);
                v.channels[i].auto_id.mod_type.store(sync.ch[i].auto_id_mod);
                v.channels[i].auto_id.baud_rate.store(sync.ch[i].auto_id_baud);
                v.channels[i].auto_id.confidence.store(sync.ch[i].auto_id_conf);
                v.channels[i].auto_id.snr_est.store(sync.ch[i].auto_id_snr);
                strncpy(v.channels[i].auto_id.protocol_name, sync.ch[i].auto_id_proto, 31);

                // ── 채널 활성화 전이 처리 ──────────────────────────────
                if(!was_active && now_active){
                    // 채널 생성: HOST의 sq_threshold를 이미 CH_SYNC로 받으므로
                    // 캘리브레이션 건너뛰기 (즉시 calibrated)
                    v.channels[i].sq_calibrated.store(true);
                    v.channels[i].sq_calib_cnt = 0;
                    v.channels[i].sq_gate_hold = 0;
                    v.channels[i].magic_det.store(0);
                    cli->audio[i].clear();
                    if(!v.ch_created_by_me[i]){
                        // 내가 만들지 않은 채널 > 초기 M(Mute) 적용
                        v.local_ch_out[i] = 3;
                        cli->cmd_toggle_recv(i, false); // 서버에 오디오 수신 차단 요청
                    }
                    // 내가 만든 채널은 local_ch_out 기본값(1=L+R) 유지
                }
                if(was_active && !now_active){
                    v.channels[i].reset_slot();
                    cli->audio[i].clear();
                    v.ch_created_by_me[i] = false;
                    v.local_ch_out[i] = 1;
                }
            }
        };

        // 예약 녹음 리스트 수신 → 로컬 sched_entries 재구성
        cli->on_sched_sync = [&](const PktSchedSync& sync){
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            v.sched_entries.clear();
            for(int i=0; i<sync.count && i<MAX_SCHED_ENTRIES; i++){
                const auto& se = sync.entries[i];
                if(!se.valid) continue;
                FFTViewer::SchedEntry e;
                e.start_time   = (time_t)se.start_time;
                e.duration_sec = se.duration_sec;
                e.freq_mhz     = se.freq_mhz;
                e.bw_khz       = se.bw_khz;
                e.status       = (FFTViewer::SchedEntry::Status)se.status;
                e.op_index     = se.op_index;
                strncpy(e.operator_name, se.operator_name, sizeof(e.operator_name)-1);
                v.sched_entries.push_back(e);
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
            // JOIN: REQ_CONFIRMED region IQ > REQ_TRANSFERRING
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
                    if(e.is_region && !e.finished && e.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING && e.filename==name){
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
            // JOIN: REQ_TRANSFERRING > done (영역 IQ 요청 결과)
            bool is_region_iq = false;
            {
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && !e.finished && e.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING && e.filename==name){
                        e.req_state=FFTViewer::RecEntry::REQ_NONE;
                        e.finished=true; e.path=path;
                        is_region_iq = true; break;
                    }
            }
            // 영역 IQ 요청 결과 > record/iq에 저장됨, rec_iq_files에 추가
            if(is_region_iq){
                bool f2=false; for(auto& s:rec_iq_files) if(s==name){f2=true;break;}
                if(!f2) rec_iq_files.push_back(name);
            } else {
                // Public 다운로드 파일 > share 폴더 (share/iq 또는 share/audio)
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

        // JOIN: 호스트가 실패한 경우 > REQ_CONFIRMED 항목을 REQ_DENIED로
        cli->on_region_response = [&](bool allowed){
            if(!allowed){
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                for(auto& e : v.rec_entries)
                    if(e.is_region && e.req_state==FFTViewer::RecEntry::REQ_CONFIRMED){
                        e.req_state=FFTViewer::RecEntry::REQ_DENIED;
                        e.req_deny_timer=5.f; break;
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
                if(e.finished) continue; // 이미 완료된 엔트리는 건너뛰기 (같은 파일명 재요청 대비)
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
                        e.req_state = FFTViewer::RecEntry::REQ_NONE;
                    }
                    break;
                }
            }
        };

        // JOIN: IQ_CHUNK 스트림 수신 (중앙 MUX 경유, WAN 호환)
        // recv 스레드 블로킹 방지: DATA 청크를 메모리 큐에 적재, 별도 write 스레드가 파일 기록
        struct IqWriteCtx {
            std::string     save_path;
            std::string     fn;
            uint64_t        filesize = 0;
            std::deque<std::vector<uint8_t>> write_queue;
            std::mutex      mtx;
            std::condition_variable cv;
            std::thread     thr;
            std::atomic<bool> done_flag{false};
            uint64_t        xfer_done = 0;
        };
        static std::unordered_map<uint32_t, std::shared_ptr<IqWriteCtx>> s_iq_ctx;
        cli->on_iq_chunk = [&](uint32_t req_id, uint32_t seq,
                               const char* filename, uint64_t filesize,
                               const uint8_t* data, uint32_t data_len){
            std::string fn(filename && filename[0] ? filename : "");
            std::string save_dir = BEWEPaths::record_iq_dir();
            std::string save_path = save_dir + "/" + fn;

            if(seq == 0){
                // START: write 스레드 생성, rec_entries 업데이트
                bewe_log_push(2,"[JOIN] IQ_CHUNK START: req_id=%u file='%s' size=%.1fMB\n",
                       req_id, fn.c_str(), filesize/1048576.0);
                auto ctx = std::make_shared<IqWriteCtx>();
                ctx->save_path = save_path;
                ctx->fn = fn;
                ctx->filesize = filesize;
                s_iq_ctx[req_id] = ctx;
                // write 스레드 시작
                ctx->thr = std::thread([ctx, req_id, fn, save_path, filesize, &v, &rec_iq_files](){
                    FILE* fp = fopen(save_path.c_str(), "wb");
                    if(!fp){
                        bewe_log_push(0,"[JOIN] IQ write thread: fopen failed '%s' errno=%d\n",
                               save_path.c_str(), errno);
                        return;
                    }
                    uint64_t written = 0;
                    while(true){
                        std::vector<uint8_t> chunk;
                        {
                            std::unique_lock<std::mutex> lk(ctx->mtx);
                            ctx->cv.wait(lk, [&ctx]{ return !ctx->write_queue.empty() || ctx->done_flag.load(); });
                            if(ctx->write_queue.empty() && ctx->done_flag.load()) break;
                            if(!ctx->write_queue.empty()){
                                chunk = std::move(ctx->write_queue.front());
                                ctx->write_queue.pop_front();
                            }
                        }
                        if(!chunk.empty()){
                            fwrite(chunk.data(), 1, chunk.size(), fp);
                            written += chunk.size();
                            ctx->xfer_done = written;
                            // 진행률 업데이트
                            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                            for(auto& e : v.rec_entries)
                                if(e.is_region && !e.finished && e.filename == fn){ e.xfer_done = written; break; }
                        }
                    }
                    // 남은 청크 모두 처리
                    {
                        std::lock_guard<std::mutex> lk(ctx->mtx);
                        while(!ctx->write_queue.empty()){
                            auto& c2 = ctx->write_queue.front();
                            fwrite(c2.data(), 1, c2.size(), fp);
                            written += c2.size();
                            ctx->write_queue.pop_front();
                        }
                    }
                    fflush(fp);
                    fclose(fp);
                    bewe_log_push(0,"[JOIN] IQ write done: %s (%.1fMB written)\n", fn.c_str(), written/1048576.0);
                    // 빈 파일이면 디스크에서 삭제하고 [Done] 표시 안 함
                    if(written == 0){
                        remove(save_path.c_str());
                        bewe_log_push(0,"[JOIN] IQ write: EMPTY file removed '%s'\n", save_path.c_str());
                        std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                        v.rec_entries.erase(
                            std::remove_if(v.rec_entries.begin(), v.rec_entries.end(),
                                [&fn](const FFTViewer::RecEntry& e){
                                    return e.is_region && e.filename == fn;
                                }),
                            v.rec_entries.end());
                        return;
                    }
                    // rec_entries: finished 마크 (삭제 X → Record 탭에 [Done] 표시)
                    {
                        std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                        for(auto& e : v.rec_entries){
                            if(e.is_region && !e.finished && e.filename == fn){
                                e.finished = true;
                                e.req_state = FFTViewer::RecEntry::REQ_NONE;
                                if(e.xfer_total == 0) e.xfer_total = written;
                                e.xfer_done = written;
                                e.path = save_path;
                                break;
                            }
                        }
                    }
                    // Archive 강제 재스캔 → 다음 프레임에 디스크 기준으로 rec_iq_files 재구성
                    // (수동 push_back 대신 scan_dir에 일임하여 경쟁 조건 회피)
                    g_arch_rescan.store(true);
                });
                ctx->thr.detach();
                // rec_entries 업데이트
                std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                bool found = false;
                for(auto& e : v.rec_entries){
                    if(!e.is_region) continue;
                    if(e.req_state == FFTViewer::RecEntry::REQ_CONFIRMED ||
                       e.req_state == FFTViewer::RecEntry::REQ_TRANSFERRING){
                        e.filename = fn;
                        e.req_state = FFTViewer::RecEntry::REQ_TRANSFERRING;
                        e.xfer_total = filesize; e.xfer_done = 0;
                        e.path = save_path;
                        found = true; break;
                    }
                }
                if(!found){
                    FFTViewer::RecEntry e{};
                    e.filename = fn; e.is_region = true;
                    e.req_state = FFTViewer::RecEntry::REQ_TRANSFERRING;
                    e.xfer_total = filesize; e.xfer_done = 0;
                    e.t_start = std::chrono::steady_clock::now();
                    e.path = save_path;
                    v.rec_entries.push_back(e);
                }
            } else if(seq == 0xFFFFFFFFu){
                // END: write 스레드에 종료 신호
                bewe_log_push(2,"[JOIN] IQ_CHUNK END: req_id=%u file='%s'\n", req_id, fn.c_str());
                auto it = s_iq_ctx.find(req_id);
                if(it != s_iq_ctx.end()){
                    {
                        std::lock_guard<std::mutex> lk(it->second->mtx);
                        it->second->done_flag.store(true);
                    }
                    it->second->cv.notify_all();
                    s_iq_ctx.erase(it);
                }
            } else {
                // DATA: write 큐에 push (recv 스레드 비블로킹)
                auto it = s_iq_ctx.find(req_id);
                if(it != s_iq_ctx.end() && data && data_len > 0){
                    std::vector<uint8_t> chunk(data, data + data_len);
                    {
                        std::lock_guard<std::mutex> lk(it->second->mtx);
                        it->second->write_queue.push_back(std::move(chunk));
                    }
                    it->second->cv.notify_one();
                } else if(it == s_iq_ctx.end()){
                    bewe_log_push(2,"[JOIN] IQ_CHUNK DATA seq=%u: no ctx for req_id=%u\n", seq, req_id);
                }
            }
        };

        // JOIN: HOST Public 파일 목록 수신 (filename, size_bytes, uploader)
        // JOIN: Report 목록 수신
        cli->on_report_list = [](const std::vector<ReportFileEntry>& entries){
            extern std::vector<ReportFileEntry> g_report_list;
            extern std::mutex g_report_list_mtx;
            std::lock_guard<std::mutex> lk(g_report_list_mtx);
            g_report_list = entries;
        };

        // DB 목록 수신 (Central server)
        cli->on_db_list = [](const std::vector<DbFileEntry>& entries){
            std::lock_guard<std::mutex> lk(g_db_list_mtx);
            g_db_list = entries;
        };

        // DB 다운로드 .info 수신 (Central → JOIN) — .wav 보다 먼저 도착
        cli->on_db_download_info = [](const PktDbDownloadInfo* di){
            if(!di) return;
            char fn[129]={}; strncpy(fn, di->filename, 128);
            bool is_iq = (strncmp(fn,"IQ_",3)==0||strncmp(fn,"sa_",3)==0);
            std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
            mkdir(dir.c_str(), 0755);
            std::string ipath = dir + "/" + fn + ".info";
            FILE* fi = fopen(ipath.c_str(), "w");
            if(fi){
                size_t n = strnlen(di->info_data, sizeof(di->info_data));
                if(n > 0) fwrite(di->info_data, 1, n, fi);
                fclose(fi);
                bewe_log_push(2,"[DB] Download .info saved: %s\n", ipath.c_str());
            }
        };

        // DB 다운로드 데이터 수신 → private/ 폴더에 저장
        static FILE* db_dl_fp = nullptr;
        static std::string db_dl_path;
        cli->on_db_download_data = [&](const PktDbDownloadData* d, const uint8_t* data, uint32_t data_len){
            static uint64_t db_dl_recv = 0;
            if(d->is_first){
                bool is_iq = (strncmp(d->filename,"IQ_",3)==0||strncmp(d->filename,"sa_",3)==0);
                std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                mkdir(dir.c_str(), 0755);
                db_dl_path = dir + "/" + d->filename;
                if(db_dl_fp) fclose(db_dl_fp);
                db_dl_fp = fopen(db_dl_path.c_str(), "wb");
                db_dl_recv = 0;
                bewe_log_push(2,"[DB] Download start: %s (%.1fMB)\n", d->filename, d->total_bytes/1048576.0);
                // 진행률 표시용 file_xfers 등록
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                FFTViewer::FileXfer xf{};
                xf.filename = d->filename;
                xf.total_bytes = d->total_bytes;
                xf.done_bytes = 0;
                v.file_xfers.push_back(xf);
            }
            if(db_dl_fp && data_len > 0){
                fwrite(data, 1, data_len, db_dl_fp);
                db_dl_recv += data_len;
                // 진행률 갱신
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                for(auto& x : v.file_xfers)
                    if(x.filename == d->filename && !x.finished){
                        x.done_bytes = db_dl_recv;
                        if(x.total_bytes == 0) x.total_bytes = d->total_bytes;
                        break;
                    }
            }
            if(d->is_last && db_dl_fp){
                fclose(db_dl_fp);
                db_dl_fp = nullptr;
                bewe_log_push(2,"[DB] Download done: %s\n", db_dl_path.c_str());
                // 완료 표시
                {
                    std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                    for(auto& x : v.file_xfers)
                        if(x.filename == d->filename && !x.finished){
                            x.finished = true;
                            x.local_path = db_dl_path;
                            x.done_bytes = x.total_bytes;
                            break;
                        }
                }
                // record 목록에 추가
                bool is_iq = (strncmp(d->filename,"IQ_",3)==0||strncmp(d->filename,"sa_",3)==0);
                std::string fn2(d->filename);
                if(is_iq){
                    bool dup=false; for(auto& s:rec_iq_files) if(s==fn2){dup=true;break;}
                    if(!dup) rec_iq_files.push_back(fn2);
                } else {
                    bool dup=false; for(auto& s:rec_audio_files) if(s==fn2){dup=true;break;}
                    if(!dup) rec_audio_files.push_back(fn2);
                }
                db_dl_path.clear();
            }
        };

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
        // - region IQ 요청 결과 (REQ_TRANSFERRING 상태) > record/iq
        // - Public 다운로드 IQ_ / sa_                  > share/iq
        // - Public 다운로드 Audio_                      > share/audio
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

        // 오퍼레이터 목록 팝업 - 건너뜀 (바로 메인으로 진입)
        bool op_popup_open = false;
        while(op_popup_open && !glfwWindowShouldClose(win)){
            glfwPollEvents();
            int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
            glViewport(0,0,fw,fh);
            glClearColor(0.03f,0.05f,0.10f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
            toggle_fullscreen();

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
            bewe_log_push(2,"SDR init failed - running without hardware (SDR LED red)\n");
            v.sdr_stream_error.store(true);
            // 초기에 SDR이 없으면 파일 분석 모드로 간주 → 주기적 재탐지 비활성화 (CPU/로그 스팸 방지)
            v.rx_stopped.store(true);
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
            else if(v.hw.type == HWType::PLUTO)
                cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
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
            // 서버 콜백 > FFTViewer 직접 제어
            srv->cb.on_set_freq   = [&](const char* who, float cf){
                bewe_log_push(0, "[CMD:%s] Freq > %.3f MHz\n", who, cf);
                v.set_frequency(cf);
            };
            srv->cb.on_set_gain   = [&](const char* who, float db){
                bewe_log_push(0, "[CMD:%s] Gain > %.1f dB\n", who, db);
                v.gain_db=db; v.set_gain(db);
            };
            srv->cb.on_create_ch  = [&](int idx, float s, float e, const char* creator){
                if(idx<0||idx>=MAX_CHANNELS) return;
                bewe_log_push(0, "[CH%d] Created by '%s' (%.4f-%.4f MHz)\n", idx, creator?creator:"?", s, e);
                v.stop_dem(idx); v.stop_digi(idx);
                v.channels[idx].reset_slot();
                v.channels[idx].s=s; v.channels[idx].e=e;
                v.channels[idx].filter_active=true;
                strncpy(v.channels[idx].owner, creator?creator:"", 31);
                v.channels[idx].audio_mask.store(0xFFFFFFFFu & ~0x1u);
                v.local_ch_out[idx] = 3;
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_delete_ch  = [&](const char* who, int idx){
                if(idx<0||idx>=MAX_CHANNELS) return;
                bewe_log_push(0, "[CMD:%s] CH%d deleted\n", who, idx);
                if(v.channels[idx].audio_rec_on.load())
                    v.stop_audio_rec(idx);
                v.stop_dem(idx); v.stop_digi(idx); v.digi_panel_on[idx]=false;
                v.channels[idx].reset_slot();
                v.local_ch_out[idx] = 1;
                srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            };
            srv->cb.on_set_ch_mode= [&](const char* who, int idx, int mode){
                if(idx<0||idx>=MAX_CHANNELS) return;
                static const char* mn[]={"NONE","AM","FM","MAGIC"};
                bewe_log_push(0, "[CMD:%s] CH%d mode > %s\n", who, idx, mn[mode<4?mode:0]);
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
                bewe_log_push(0, "[CMD] Autoscale requested\n");
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
                                             int64_t time_start_ms, int64_t time_end_ms,
                                             int64_t samp_start, int64_t samp_end){
                int32_t time_start = (int32_t)(time_start_ms / 1000);
                int32_t time_end   = (int32_t)(time_end_ms / 1000);
                bewe_log_push(0, "[IQ] Region request from '%s' (%.3f-%.3f MHz)\n",
                              op_name?op_name:"?", freq_lo, freq_hi);
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
                static std::atomic<uint32_t> g_req_id{1000};
                uint32_t req_id_val = g_req_id.fetch_add(1);
                std::thread([&v,srv,ft,fb,fl,fh,ts,te,samp_start,samp_end,oidx,fname,sid,central_host_cap,&central_cli,req_id_val](){
                    uint32_t req_id = req_id_val;
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
                    v.region.time_start_ms=(int64_t)ts*1000LL;
                    v.region.time_end_ms=(int64_t)te*1000LL;
                    v.region.samp_start=samp_start;
                    v.region.samp_end=samp_end;
                    v.region.active=true;
                    v.rec_busy_flag.store(true);
                    v.rec_state = FFTViewer::REC_BUSY;
                    v.rec_anim_timer = 0.0f;
                    v.region.active = false;
                    // IQ_PROGRESS phase=0 (REC 중) 브로드캐스트 - 파이프와 동일한 req_id 사용
                    if(srv){
                        PktIqProgress prog{};
                        prog.req_id = req_id;
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
                            if(!it->is_audio&&it->req_state==FFTViewer::RecEntry::REQ_NONE&&it->finished){
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
                    // IQ 파일을 청크로 나눠 central MUX broadcast (WAN 지원, 포트포워딩 불필요)
                    if(srv && srv->cb.on_relay_broadcast){
                        const char* fn_only2 = strrchr(path.c_str(), '/');
                        fn_only2 = fn_only2 ? fn_only2+1 : path.c_str();
                        bewe_log_push(2,"[HOST] IQ_CHUNK transfer start: req_id=%u file='%s' size=%.1fMB\n",
                               req_id, fn_only2, fsz/1048576.0);
                        // START 패킷 (no_drop=true: IQ는 드롭 불가)
                        {
                            PktIqChunkHdr ch{};
                            ch.req_id = req_id; ch.seq = 0;
                            strncpy(ch.filename, fn_only2, 127);
                            ch.filesize = fsz; ch.data_len = 0;
                            auto bewe = make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                            srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), /*no_drop=*/true);
                        }
                        // 청크 전송 스레드
                        std::thread([&v, fname, path, fsz, srv, req_id,
                                     fn2 = std::string(fn_only2)](){
                            FILE* fp = fopen(path.c_str(), "rb");
                            if(!fp){ bewe_log_push(2,"[HOST] IQ_CHUNK: cannot open %s\n", path.c_str()); return; }
                            const size_t CHUNK = 64 * 1024;
                            std::vector<uint8_t> buf(sizeof(PktIqChunkHdr) + CHUNK);
                            uint64_t sent = 0; uint32_t seq = 1;
                            while(true){
                                size_t n = fread(buf.data() + sizeof(PktIqChunkHdr), 1, CHUNK, fp);
                                if(n == 0) break;
                                auto* ch = reinterpret_cast<PktIqChunkHdr*>(buf.data());
                                ch->req_id = req_id; ch->seq = seq++;
                                strncpy(ch->filename, fn2.c_str(), 127);
                                ch->filesize = fsz; ch->data_len = (uint32_t)n;
                                auto bewe = make_packet(PacketType::IQ_CHUNK, buf.data(), (uint32_t)(sizeof(PktIqChunkHdr)+n));
                                if(srv->cb.on_relay_broadcast)
                                    srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), /*no_drop=*/true);
                                sent += n;
                                // HOST rec_entries 진행 갱신
                                {
                                    std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                    for(auto& e : v.rec_entries)
                                        if(e.filename == fname){ e.xfer_done = sent; e.xfer_total = fsz; break; }
                                }
                            }
                            fclose(fp);
                            // END 패킷
                            {
                                PktIqChunkHdr ch{};
                                ch.req_id = req_id; ch.seq = 0xFFFFFFFF;
                                strncpy(ch.filename, fn2.c_str(), 127);
                                ch.filesize = fsz; ch.data_len = 0;
                                auto bewe = make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                                if(srv->cb.on_relay_broadcast)
                                    srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), /*no_drop=*/true);
                            }
                            bewe_log_push(2,"[HOST] IQ_CHUNK transfer done: req_id=%u %.1fMB\n", req_id, sent/1048576.0);
                            // IQ_PROGRESS Done 브로드캐스트
                            {
                                PktIqProgress prog{};
                                prog.req_id = req_id;
                                strncpy(prog.filename, fname.c_str(), 127);
                                prog.done = sent; prog.total = fsz; prog.phase = 2;
                                srv->broadcast_iq_progress(prog);
                            }
                            // 전송 완료 후 HOST rec_entries에서 TRANSFERRING 항목만 제거
                            // (파일은 삭제하지 않고 HOST record/iq에 보존)
                            {
                                std::lock_guard<std::mutex> lk2(v.rec_entries_mtx);
                                v.rec_entries.erase(
                                    std::remove_if(v.rec_entries.begin(), v.rec_entries.end(),
                                        [&fname](const FFTViewer::RecEntry& e){
                                            return e.filename == fname &&
                                                   e.req_state == FFTViewer::RecEntry::REQ_TRANSFERRING;
                                        }),
                                    v.rec_entries.end());
                            }
                        }).detach();
                    } else {
                        bewe_log_push(2,"[HOST] IQ_CHUNK: on_relay_broadcast is NULL - cannot send\n");
                    }
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
                // (inline 호출 시 send_audio가 send_mtx를 기다리며 demod 스레드 블로킹 > HOST 오디오 끊김)
                std::string path_copy = path;
                int op_int = (int)op_idx;
                std::thread([srv, path_copy, op_int, tid](){
                    srv->send_file_to(op_int, path_copy.c_str(), tid);
                }).detach();
            };
            // HOST: JOIN이 파일을 업로드 완료 > public/iq 또는 public/audio로 이동 + 목록 갱신
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
                // 임시 파일 > public 폴더로 복사
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
                bewe_log_push(2,"Public upload done: %s (from %s)\n", fn, op_name);
            };
            // JOIN이 /chassis 1 reset 명령 전송 > HOST 측에서 재시작
            // JOIN > HOST: FFT size 변경 (HOST 적용 후 FFT_FRAME으로 자동 동기화)
            srv->cb.on_set_fft_size = [&](const char* who, uint32_t size){
                bewe_log_push(0, "[CMD:%s] FFT size > %u\n", who, size);
                static const int valid[]={512,1024,2048,4096,8192,16384};
                for(int vs : valid){
                    if((uint32_t)vs==size){
                        v.pending_fft_size=size; v.fft_size_change_req=true;
                        break;
                    }
                }
            };
            srv->cb.on_set_sr = [&](const char* who, float msps){
                bewe_log_push(0, "[CMD:%s] SR > %.2f MSPS\n", who, msps);
                v.pending_sr_msps=msps; v.sr_change_req=true;
            };
            srv->cb.on_set_antenna = [&](const char* who, const char* antenna){
                bewe_log_push(0,"[CMD:%s] Antenna > '%s'\n", who, antenna?antenna:"");
                strncpy(v.host_antenna, antenna?antenna:"", sizeof(v.host_antenna)-1);
                v.host_antenna[sizeof(v.host_antenna)-1] = '\0';
            };
            // ── 예약 녹음 (JOIN → HOST) ──────────────────────────────────
            srv->cb.on_add_sched = [&](uint8_t op_idx, const char* op_name,
                                        int64_t start_time, float duration_sec,
                                        float freq_mhz, float bw_khz){
                if(duration_sec <= 0 || freq_mhz <= 0 || bw_khz <= 0) return;
                time_t now = time(nullptr);
                if((time_t)start_time + (time_t)duration_sec < now) return;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    if(v.sched_has_overlap((time_t)start_time, duration_sec)){
                        bewe_log_push(0,"[CMD:%s] SCHED denied: overlap\n", op_name);
                        return;
                    }
                    if((int)v.sched_entries.size() >= MAX_SCHED_ENTRIES) return;
                    FFTViewer::SchedEntry e;
                    e.start_time   = (time_t)start_time;
                    e.duration_sec = duration_sec;
                    e.freq_mhz     = freq_mhz;
                    e.bw_khz       = bw_khz;
                    e.status       = FFTViewer::SchedEntry::WAITING;
                    e.op_index     = op_idx;
                    strncpy(e.operator_name, op_name?op_name:"", sizeof(e.operator_name)-1);
                    v.sched_entries.push_back(e);
                    bewe_log_push(0,"[CMD:%s] SCHED added: %.3fMHz %.0fkHz dur=%.0fs\n",
                                  op_name, freq_mhz, bw_khz, duration_sec);
                }
                v.broadcast_sched_list();
            };
            srv->cb.on_remove_sched = [&](uint8_t op_idx, const char* op_name,
                                           int64_t start_time, float freq_mhz){
                bool removed = false;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    for(auto it = v.sched_entries.begin(); it != v.sched_entries.end(); ++it){
                        if((time_t)start_time != it->start_time) continue;
                        if(fabsf(freq_mhz - it->freq_mhz) > 0.0001f) continue;
                        if(it->op_index != op_idx && op_idx != 0) return;
                        if(it->status == FFTViewer::SchedEntry::RECORDING) return;
                        v.sched_entries.erase(it);
                        removed = true;
                        break;
                    }
                }
                if(removed) v.broadcast_sched_list();
            };
            srv->cb.on_chassis_reset = [&](const char* who){
                bewe_log_push(0, "[CMD:%s] /chassis 1 reset\n", who);
                pending_chassis1_reset.store(true);
            };
            srv->cb.on_net_reset = [&](const char* who){
                bewe_log_push(0, "[CMD:%s] /chassis 2 reset\n", who);
                pending_chassis2_reset.store(true);
            };
            srv->cb.on_rx_stop = [&](const char* who){
                bewe_log_push(0, "[CMD:%s] /rx stop\n", who);
                pending_rx_stop.store(true);
            };
            srv->cb.on_rx_start = [&](const char* who){
                bewe_log_push(0, "[CMD:%s] /rx start\n", who);
                pending_rx_start.store(true);
            };
            // JOIN이 public 파일 삭제 요청 > 소유자 확인 후 삭제 + 브로드캐스트
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

            // port 0 > OS가 빈 포트 자동 할당
            if(!srv->start(0)){
                bewe_log_push(2,"Server start failed\n");
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
                            // 릴레이가 재작성한 CHANNEL_SYNC > HOST의 audio_mask 갱신
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
                                v.net_srv->cb.on_relay_broadcast = [&central_cli](const uint8_t* pkt, size_t len, bool no_drop){
                                    central_cli.enqueue_relay_broadcast(pkt, len, no_drop);
                                };
                            }
                            central_cli.set_on_central_chat([_log_mtx, _log](const char* from, const char* msg){
                                std::lock_guard<std::mutex> lk(*_log_mtx);
                                if((int)_log->size() >= 200) _log->erase(_log->begin());
                                LocalChatMsg m{}; strncpy(m.from,from,31); strncpy(m.msg,msg,255);
                                _log->push_back(m);
                            });
                            central_cli.set_on_central_op_list([](const uint8_t* pkt, size_t len){
                                // BEWE 헤더(9바이트) 이후 OP_LIST payload 파싱
                                if(len < 9 + 1) return;
                                const uint8_t* payload = pkt + 9;
                                size_t plen = len - 9;
                                std::lock_guard<std::mutex> lk(s_relay_op_mtx);
                                s_relay_op_list = {};
                                uint8_t count = payload[0];
                                if(count > MAX_OPERATORS) count = MAX_OPERATORS;
                                s_relay_op_list.count = count;
                                for(int i = 0; i < count; i++){
                                    size_t off = 1 + i * BEWE_OP_ENTRY_SIZE;
                                    if(off + BEWE_OP_ENTRY_SIZE > plen) break;
                                    s_relay_op_list.ops[i].index = payload[off];
                                    s_relay_op_list.ops[i].tier  = payload[off+1];
                                    strncpy(s_relay_op_list.ops[i].name, (const char*)(payload+off+2), 31);
                                }
                            });
                            // Central DB 목록 수신
                            central_cli.set_on_central_db_list([](const uint8_t* pkt, size_t len){
                                extern std::vector<DbFileEntry> g_db_list;
                                extern std::mutex g_db_list_mtx;
                                if(len < 9 + sizeof(PktDbList)) return;
                                const uint8_t* payload = pkt + 9;
                                auto* hdr2 = reinterpret_cast<const PktDbList*>(payload);
                                uint16_t cnt2 = hdr2->count;
                                size_t expected = sizeof(PktDbList) + cnt2 * sizeof(DbFileEntry);
                                if(len - 9 < expected) return;
                                const DbFileEntry* ent = reinterpret_cast<const DbFileEntry*>(payload + sizeof(PktDbList));
                                { std::lock_guard<std::mutex> lk(g_db_list_mtx);
                                  g_db_list.assign(ent, ent + cnt2); }
                                bewe_log_push(0,"[Central] DB_LIST: %u files\n", cnt2);
                            });
                            // Central DB 다운로드 .info 수신
                            central_cli.set_on_central_db_dl_info([](const uint8_t* pkt, size_t len){
                                if(len < 9 + sizeof(PktDbDownloadInfo)) return;
                                const auto* di = reinterpret_cast<const PktDbDownloadInfo*>(pkt + 9);
                                char fn[129]={}; strncpy(fn, di->filename, 128);
                                bool is_iq = (strncmp(fn,"IQ_",3)==0||strncmp(fn,"sa_",3)==0);
                                std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                                mkdir(dir.c_str(), 0755);
                                std::string ipath = dir + "/" + fn + ".info";
                                FILE* fi = fopen(ipath.c_str(), "w");
                                if(fi){
                                    size_t n = strnlen(di->info_data, sizeof(di->info_data));
                                    if(n > 0) fwrite(di->info_data, 1, n, fi);
                                    fclose(fi);
                                    bewe_log_push(0,"[DB] Download .info saved: %s\n", ipath.c_str());
                                }
                            });
                            // Central DB 다운로드 데이터 수신
                            static FILE* host_db_dl_fp = nullptr;
                            static std::string host_db_dl_path;
                            central_cli.set_on_central_db_dl_data([&v](const uint8_t* pkt, size_t len){
                                if(len < 9 + sizeof(PktDbDownloadData)) return;
                                const auto* d = reinterpret_cast<const PktDbDownloadData*>(pkt + 9);
                                const uint8_t* data = pkt + 9 + sizeof(PktDbDownloadData);
                                uint32_t data_len = d->chunk_bytes;
                                if(d->is_first){
                                    bool is_iq = (strncmp(d->filename,"IQ_",3)==0||strncmp(d->filename,"sa_",3)==0);
                                    std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                                    mkdir(dir.c_str(), 0755);
                                    host_db_dl_path = dir + "/" + d->filename;
                                    if(host_db_dl_fp) fclose(host_db_dl_fp);
                                    host_db_dl_fp = fopen(host_db_dl_path.c_str(), "wb");
                                    bewe_log_push(0,"[DB] Download start: %s (%.1fMB)\n", d->filename, d->total_bytes/1048576.0);
                                }
                                if(host_db_dl_fp && data_len > 0)
                                    fwrite(data, 1, data_len, host_db_dl_fp);
                                if(d->is_last && host_db_dl_fp){
                                    fclose(host_db_dl_fp);
                                    host_db_dl_fp = nullptr;
                                    bewe_log_push(0,"[DB] Download done: %s\n", host_db_dl_path.c_str());
                                    host_db_dl_path.clear();
                                }
                            });
                            // 재귀적 자동 재연결 함수 (shared_ptr로 캡처)
                            auto reconnect_fn = std::make_shared<std::function<void()>>();
                            *reconnect_fn = [&v, &central_cli, rh, rp, reconnect_fn](){
                                // Central Server 끊김 > 자동 재연결 (3초 후, 최대 5회)
                                std::thread([&v, &central_cli, rh, rp, reconnect_fn](){
                                    for(int attempt=0; attempt<5; attempt++){
                                        std::this_thread::sleep_for(std::chrono::seconds(3));
                                        if(!v.net_srv) return;  // HOST 모드 종료됨
                                        bewe_log_push(2,"[UI] Central auto-reconnect attempt %d/5\n", attempt+1);
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
                                            bewe_log_push(2,"[UI] Central auto-reconnected\n");
                                            return;
                                        }
                                    }
                                    bewe_log_push(2,"[UI] Central auto-reconnect failed after 5 attempts\n");
                                }).detach();
                            };
                            central_cli.start_mux_adapter(rfd,
                                [&v](int local_fd){ if(v.net_srv) v.net_srv->inject_fd(local_fd); },
                                [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                                *reconnect_fn);
                            bewe_log_push(2,"[UI] Central MUX adapter started\n");
                        } else {
                            bewe_log_push(2,"[UI] Central open_room failed, Central unavailable\n");
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
    bool  archive_open = false;
    int   last_fft_seq = -1;  // CONNECT 모드 FFT 시퀀스 추적
    bool  chat_scroll_bottom = false;
    // 파일 우클릭 컨텍스트 메뉴
    struct FileCtxMenu {
        bool open=false; float x=0,y=0;
        std::string filepath, filename;
        bool is_public=false; // Public 탭 파일 (소유자만 삭제)
        bool selected=false;  // 좌클릭 선택 상태
    } file_ctx;

    // DB 전용 우클릭 상태
    static struct { bool open=false; float x=0,y=0; std::string filename, operator_name; bool is_iq=false; } db_ctx;

    // ── Rename 모달 ──────────────────────────────────────────────────────
    struct RenameModal {
        bool open = false;
        std::string dir;       // 디렉토리 경로
        std::string old_name;  // 원래 파일명
        char new_name[256]={};
        bool focus_set = false;
    } rename_modal;

    bool g_arch_cache_dirty = false;  // arch_info_cache / arch_info_tip_cache 무효화 플래그

    // ── .info 메타데이터 모달 ────────────────────────────────────────────
    // 인덱스: 0=File Name, 1=Day, 2=Up Time, 3=Down Time, 4=Duration,
    //        5=Frequency, 6=Target, 7=Location, 8=Modulation, 9=Bandwidth,
    //        10=Signal Strength, 11=Protocol, 12=Recorder, 13=Notes,
    //        14=Tags, 15=Priority, 16=Operator
    struct InfoModal {
        bool open = false;
        bool save_and_write = false; // true: Save 누르면 WAV/IQ 파일도 생성 (EID Save File 플로우)
        std::function<std::string(const std::string&)> save_file_fn;
        std::string filepath, info_path;
        std::string src_filepath;
        bool exists = false;
        int  utc_off_override = INT_MIN; // HOST 좌표 기반 UTC; INT_MIN이면 시스템 TZ
        char ext_buf[16] = {};      // 확장자 (.wav 등, 표시만, 편집 불가)
        char fields[17][256] = {};
        enum { N_FIELDS = 17 };
        const char* names[17] = {
            "File Name","Day","Up Time","Down Time","Duration",
            "Frequency","Target","Location","Modulation","Bandwidth",
            "Signal Strength","Protocol","Recorder","Notes",
            "Tags","Priority","Operator"
        };
        void load(){
            FILE* f = fopen(info_path.c_str(), "r");
            if(!f) return;
            char line[512];
            while(fgets(line, sizeof(line), f)){
                char key[64]={}, val[256]={};
                if(sscanf(line, "%63[^:]: %255[^\n]", key, val) >= 1){
                    // 레거시 호환 매핑
                    const char* mapped = key;
                    if(strcmp(key, "Time") == 0)       mapped = "Up Time";
                    else if(strcmp(key, "Freq") == 0)  mapped = "Frequency";
                    else if(strcmp(key, "Source Type")==0) mapped = "Recorder";
                    // "Content" 는 제거됨 (무시)
                    if(strcmp(key, "Content") == 0)    continue;
                    for(int i=0;i<N_FIELDS;i++){
                        if(strcmp(mapped, names[i])==0){
                            strncpy(fields[i], val, 255);
                            break;
                        }
                    }
                }
            }
            fclose(f);
        }
        void save(){
            FILE* f = fopen(info_path.c_str(), "w");
            if(!f) return;
            for(int i=0;i<N_FIELDS;i++)
                fprintf(f, "%s: %s\n", names[i], fields[i]);
            fclose(f);
        }
        // WAV 파일 길이 초 단위 추정 (헤더 + 파일 크기)
        static double wav_duration_sec(const std::string& path){
            FILE* f = fopen(path.c_str(), "rb");
            if(!f) return 0.0;
            double dur = 0.0;
            uint8_t hdr[44];
            if(fread(hdr, 1, 44, f) == 44){
                uint32_t sr  = *(uint32_t*)(hdr+24);
                uint16_t ch  = *(uint16_t*)(hdr+22);
                uint16_t bps = *(uint16_t*)(hdr+34);
                struct stat st{};
                if(::stat(path.c_str(), &st) == 0 && sr>0 && ch>0 && bps>0)
                    dur = (double)(st.st_size - 44) / (sr * ch * (bps/8));
            }
            fclose(f);
            return dur;
        }
        // 로컬 UTC 오프셋 (시간 단위)
        static int utc_off_hours(){
            time_t now = time(nullptr);
            struct tm lt; localtime_r(&now, &lt);
            return (int)(lt.tm_gmtoff / 3600);
        }
        // HH:MM:SS (UTC+N) 포맷
        static void fmt_time_utc(char* out, size_t sz, const struct tm& lt, int off){
            char base[16]; strftime(base, sizeof(base), "%H:%M:%S", &lt);
            if(off >= 0) snprintf(out, sz, "%s (UTC+%d)", base, off);
            else         snprintf(out, sz, "%s (UTC%d)",  base, off);
        }
        void autofill(const std::string& filename){
            memset(fields, 0, sizeof(fields));
            // File Name (stem, 확장자 제외)
            {
                size_t dot = filename.find_last_of('.');
                std::string stem = (dot==std::string::npos) ? filename : filename.substr(0, dot);
                strncpy(fields[0], stem.c_str(), 255);
            }
            // Frequency: parse _XXX.XXXMHz_
            float mhz=0;
            const char* p = strstr(filename.c_str(), "MHz");
            if(p){
                const char* q = p-1;
                while(q > filename.c_str() && ((*q>='0'&&*q<='9')||*q=='.')) q--;
                if(q < p-1) mhz = (float)atof(q+1);
            }
            if(mhz > 0) snprintf(fields[5], 256, "%.3f MHz", mhz);
            // Day / Up Time: parse _MonDD_YYYY_HHMMSS
            int utc_off = (utc_off_override == INT_MIN) ? utc_off_hours() : utc_off_override;
            bool dt_found = false;
            const char* under = filename.c_str();
            for(int i=0; i<3 && under; i++) under = strchr(under+1, '_');
            if(under && strlen(under) > 16){
                char mon[4]={};
                int day=0,yr=0,hh=0,mm=0,ss=0;
                if(sscanf(under-3, "%3s%2d_%4d_%2d%2d%2d", mon,&day,&yr,&hh,&mm,&ss) >= 5){
                    snprintf(fields[1], 256, "%s %02d, %04d", mon, day, yr);
                    if(utc_off >= 0) snprintf(fields[2], 256, "%02d:%02d:%02d (UTC+%d)", hh, mm, ss, utc_off);
                    else             snprintf(fields[2], 256, "%02d:%02d:%02d (UTC%d)",  hh, mm, ss, utc_off);
                    dt_found = true;
                }
            }
            if(!dt_found){
                time_t now = time(nullptr);
                struct tm lt; localtime_r(&now, &lt);
                strftime(fields[1], 256, "%b %d, %Y", &lt);
                fmt_time_utc(fields[2], 256, lt, utc_off);
            }
            // Duration: WAV 파일 직접 측정
            if(!filepath.empty()){
                double d = wav_duration_sec(filepath);
                if(d > 0) snprintf(fields[4], 256, "%.1f s", d);
            }
            // Operator
            strncpy(fields[16], login_get_id(), 255);
        }
        // 확장자 초기화 (파일명 fields[0]과 분리)
        void init_ext(){
            const char* fn = strrchr(filepath.c_str(), '/');
            fn = fn ? fn+1 : filepath.c_str();
            const char* dot = strrchr(fn, '.');
            if(dot){ strncpy(ext_buf, dot, sizeof(ext_buf)-1); ext_buf[sizeof(ext_buf)-1] = '\0'; }
            else   { ext_buf[0] = '\0'; }
            // fields[0] (File Name)가 비어있으면 stem으로 채움
            if(fields[0][0] == '\0'){
                size_t stem_len = dot ? (size_t)(dot - fn) : strlen(fn);
                if(stem_len >= sizeof(fields[0])) stem_len = sizeof(fields[0])-1;
                memcpy(fields[0], fn, stem_len); fields[0][stem_len] = '\0';
            }
        }
    } info_modal;

    // HH:MM:SS (UTC+N) 또는 HHMMSS 입력 → HH:MM:SS (UTC+N) 형식으로 자동 변환
    auto fmt_time_field = [](char* buf){
        if(!buf || !buf[0]) return;
        // 이미 콜론 있으면 그대로
        if(strchr(buf, ':')) return;
        // UTC 태그 추출 (있으면 나중에 다시 붙임)
        char utc_tag[32] = {};
        char* paren = strchr(buf, '(');
        if(paren){
            strncpy(utc_tag, paren, sizeof(utc_tag)-1);
            utc_tag[sizeof(utc_tag)-1] = '\0';
            // paren 이전 trim
            while(paren > buf && (paren[-1]==' '||paren[-1]=='\t')) paren--;
            *paren = '\0';
        }
        // 숫자만 추출
        char digits[8]={}; int dn=0;
        for(char* p=buf; *p && dn<7; p++) if(*p>='0' && *p<='9') digits[dn++] = *p;
        if(dn == 6){
            char out[64];
            if(utc_tag[0])
                snprintf(out, sizeof(out), "%c%c:%c%c:%c%c %s",
                         digits[0],digits[1],digits[2],digits[3],digits[4],digits[5], utc_tag);
            else
                snprintf(out, sizeof(out), "%c%c:%c%c:%c%c",
                         digits[0],digits[1],digits[2],digits[3],digits[4],digits[5]);
            strncpy(buf, out, 255); buf[255]='\0';
        }
    };

    // chassis 1 reset 후 HOST 재시작: stable 메시지 (JOIN 재접속 전이므로 로컬만)
    if(mode_sel == 1 && chassis_reset_mode == 1 && v.net_srv){
        LocalChatMsg lm{}; lm.is_error = false;
        strncpy(lm.from, "SYSTEM", 31);
        strncpy(lm.msg,  "Chassis 1 stable ...", 255);
        host_chat_log.push_back(lm);
    }

    // HOST 모드: 수신 채팅을 로컬 로그에도 저장
    if(v.net_srv){
        v.net_srv->cb.on_chat = [&](const char* from, const char* msg){
            bewe_log_push(0, "[CHAT] %s: %s\n", from, msg);
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
                uint8_t hwt = (v.hw.type==HWType::RTLSDR) ? 1 :
                              (v.hw.type==HWType::PLUTO)  ? 2 : 0;
                v.net_srv->broadcast_status(
                    (float)(v.header.center_frequency/1e6),
                    v.gain_db, v.header.sample_rate, hwt);
            }
        }
        // ── HOST: 3초마다 HEARTBEAT 브로드캐스트; SDR 뽑힘 감지 시 즉시 ──────
        if(v.net_srv){
            static bool prev_sdr_err = false;
            bool cur_sdr_err = v.sdr_stream_error.load();
            auto now2=std::chrono::steady_clock::now();
            float elh=std::chrono::duration<float>(now2-heartbeat_last).count();
            bool sdr_err_changed = (cur_sdr_err != prev_sdr_err);
            if(elh>=1.0f || sdr_err_changed){
                if(sdr_err_changed) prev_sdr_err = cur_sdr_err;
                heartbeat_last=now2;
                uint8_t sdr_t_hb = 0;
                if(v.dev_blade){
                    float _t = 0.f;
                    if(bladerf_get_rfic_temperature(v.dev_blade, &_t) == 0)
                        sdr_t_hb = (uint8_t)std::min(255.f, std::max(0.f, _t));
                } else if(v.pluto_ctx){
                    float _t = v.pluto_get_temp_c();
                    if(_t > 0.f) sdr_t_hb = (uint8_t)std::min(255.f, _t);
                }
                // host_state: 0=OK, 2=SPECTRUM_PAUSED (JOIN에게 노란 LINK 표시)
                uint8_t hst = (v.spectrum_pause.load() || !v.render_visible.load()) ? 2 : 0;
                // sdr_state: 0=OK, 1=stream error/rx stopped (SDR 뽑힘/초기화 실패/의도적 정지)
                uint8_t sdr_st = (cur_sdr_err || v.rx_stopped.load()) ? 1 : 0;
                // iq_on: HOST IQ 롤링 상태
                uint8_t iq_st = v.tm_iq_on.load() ? 1 : 0;
                uint8_t h_cpu = (uint8_t)std::min(100.f, std::max(0.f, v.sysmon_cpu));
                uint8_t h_ram = (uint8_t)std::min(100.f, std::max(0.f, v.sysmon_ram));
                uint8_t h_ct  = (uint8_t)std::min(255, std::max(0, v.sysmon_cpu_temp_c.load()));
                v.net_srv->broadcast_heartbeat(hst, sdr_t_hb, sdr_st, iq_st, h_cpu, h_ram, h_ct, v.host_antenna);
            }
        }

        // ── Scheduled recording tick ──────────────────────────────────────
        if(!v.remote_mode) v.sched_tick();

        // ── SDR 런타임 교체 (HOST/LOCAL) ──────────────────────────────────
        if(!v.remote_mode && v.pending_sdr_switch.load()){
            v.pending_sdr_switch.store(false);
            std::string new_sdr;
            { std::lock_guard<std::mutex> lk(v.pending_sdr_mtx); new_sdr = v.pending_sdr_name; }
            bewe_log_push(0, "[SDR] switching to %s ...\n", new_sdr.c_str());
            float cur_cf = (float)(v.header.center_frequency / 1e6);
            // 모든 디지털/오디오 워커 중지
            for(int ci=0; ci<MAX_CHANNELS; ci++){ v.stop_digi(ci); v.stop_dem(ci); }
            v.is_running = false;
            v.sdr_stream_error.store(true);
            if(cap.joinable()) cap.join();
            // 디바이스 핸들 정리 (worker가 스스로 close함)
            v.dev_blade = nullptr; v.dev_rtl = nullptr;
            v.pluto_ctx=nullptr; v.pluto_phy_dev=nullptr; v.pluto_rx_dev=nullptr;
            v.pluto_rx_i_ch=nullptr; v.pluto_rx_q_ch=nullptr; v.pluto_rx_buf=nullptr;
            // 강제 선택자 설정 후 재초기화
            g_sdr_force = new_sdr;
            v.is_running = true;
            if(v.initialize(cur_cf, 0.f)){
                v.set_gain(v.gain_db);
                v.sdr_stream_error.store(false);
                if(v.hw.type == HWType::BLADERF)
                    cap = std::thread(&FFTViewer::capture_and_process, &v);
                else if(v.hw.type == HWType::PLUTO)
                    cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
                else
                    cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                bewe_log_push(0, "[SDR] switched to %s\n", new_sdr.c_str());
            } else {
                bewe_log_push(2, "[SDR] switch to %s FAILED\n", new_sdr.c_str());
            }
        }

        // ── JOIN>HOST chassis 명령: 네트워크 스레드 플래그 > 메인 루프 처리 ────
        // HOST 직접 입력과 완전히 동일한 경로로 실행 (race condition 방지)
        if(v.net_srv && pending_chassis1_reset.load()){
            pending_chassis1_reset.store(false);
            { std::lock_guard<std::mutex> lk(host_chat_mtx);
              LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
              strncpy(lm.msg,"Chassis 1 reset ...",255);
              if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
              host_chat_log.push_back(lm); }
            v.net_srv->broadcast_chat("SYSTEM", "Chassis 1 reset ...");
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
              LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
              strncpy(lm.msg,"Chassis 2 reset ...",255);
              if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
              host_chat_log.push_back(lm); }
            // 2) JOIN 클라이언트에게 리셋 알림
            v.net_srv->broadcast_chat("SYSTEM", "Chassis 2 reset ...");
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
                srv_ptr->broadcast_chat("SYSTEM", "Chassis 2 stable ...");
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
                        bewe_log_push(2,"[UI] Central reconnected after chassis 2 reset\n");
                    } else {
                        bewe_log_push(2,"[UI] Central reconnect failed after chassis 2 reset\n");
                    }
                } else if(central_ptr->is_central_connected()){
                    central_ptr->send_net_reset(1);  // 1 = open
                }
                std::lock_guard<std::mutex> lk(*log_mtx_ptr2);
                LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
                strncpy(lm.msg,"Chassis 2 stable ...",255);
                if((int)log_ptr2->size()>=200) log_ptr2->erase(log_ptr2->begin());
                log_ptr2->push_back(lm);
            }).detach();
        }

        // ── JOIN>HOST /rx stop/start: 네트워크 스레드 플래그 > 메인 루프 처리 ──
        if(v.net_srv && pending_rx_stop.load()){
            pending_rx_stop.store(false);
            if(!v.rx_stopped.load() && (v.is_running || cap.joinable())){
                { std::lock_guard<std::mutex> lk(host_chat_mtx);
                  LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
                  strncpy(lm.msg,"RX stop (remote)",255);
                  if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                  host_chat_log.push_back(lm); }
                v.net_srv->broadcast_chat("SYSTEM", "RX stop");
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
                  LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
                  strncpy(lm.msg,"RX stopped.",255);
                  if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                  host_chat_log.push_back(lm); }
            }
        }
        if(v.net_srv && pending_rx_start.load()){
            pending_rx_start.store(false);
            if(v.rx_stopped.load()){
                { std::lock_guard<std::mutex> lk(host_chat_mtx);
                  LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
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
                    v.net_srv->broadcast_chat("SYSTEM", "RX start");
                    { std::lock_guard<std::mutex> lk(host_chat_mtx);
                      LocalChatMsg lm{}; strncpy(lm.from,"SYSTEM",31);
                      strncpy(lm.msg,"RX started.",255);
                      if((int)host_chat_log.size()>=200) host_chat_log.erase(host_chat_log.begin());
                      host_chat_log.push_back(lm); }
                } else {
                    v.is_running = false;
                    v.rx_stopped.store(true);
                    { std::lock_guard<std::mutex> lk(host_chat_mtx);
                      LocalChatMsg lm{}; lm.is_error=true; strncpy(lm.from,"System",31);
                      strncpy(lm.msg,"RX start failed - SDR not found.",255);
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
                bewe_log_push(2,"[UI] chassis 1 reset: spectrum_pause released\n");
                if(v.net_srv) v.net_srv->broadcast_heartbeat(0, 0, 0);
            }
        }

        // ── SR 변경 후 demod 재시작 ──────────────────────────────────────────
        if(v.dem_restart_needed.load()){
            v.dem_restart_needed.store(false);
            for(int di=0;di<MAX_CHANNELS;di++){
                if(v.channels[di].dem_run.load()){
                    auto dm=v.channels[di].mode;
                    v.stop_dem(di);
                    v.start_dem(di,dm);
                }
            }
        }

        // ── LOCAL/HOST: SDR 뽑힘 감지 > 백그라운드 join + 주기적 재탐지 ──────
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

            // cap 종료 확인 후 USB reset (BladeRF만 해당, chassis reset 요청 시 한 번만)
            if(cap_joined.load() && usb_reset_pending && !usb_reset_done){
                usb_reset_done = true;
                usb_reset_pending = false;
                if(v.hw.type == HWType::BLADERF){
                    usb_reset_in_progress.store(true);
                    // capture_and_process 종료 시 dev_blade가 nullptr로 세팅됨
                    // (혹시 남아있으면 닫기)
                    if(v.dev_blade){
                        bladerf_close(v.dev_blade);
                        v.dev_blade = nullptr;
                    }
                    std::thread([&usb_reset_in_progress = usb_reset_in_progress](){
                        bewe_log_push(2,"[UI] chassis 1 reset: USB reset BladeRF...\n");
                        bladerf_usb_reset();
                        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        bewe_log_push(2,"[UI] USB re-enumeration wait done\n");
                        usb_reset_in_progress.store(false);
                    }).detach();
                }
                // Pluto/RTL-SDR: USB reset 불필요, 재시도 타이머로 즉시 진행
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
                float cur_sr = v.header.sample_rate / 1e6f;
                if(cur_sr < 0.1f) cur_sr = 61.44f;
                v.is_running = true; // 새 캡처 스레드를 위해 복구
                if(v.initialize(cur_cf, cur_sr)){
                    bewe_log_push(2,"SDR reconnected - resuming at %.2f MHz\n", cur_cf);
                    v.sdr_stream_error.store(false);
                    bg_join_started = false;  // 다음 뽑힘을 위해 리셋
                    cap_joined.store(false);
                    usb_reset_in_progress.store(false);
                    // 이전 게인 복원
                    v.set_gain(v.gain_db);
                    // 캡처 스레드 재시작
                    if(v.hw.type == HWType::BLADERF)
                        cap = std::thread(&FFTViewer::capture_and_process, &v);
                    else if(v.hw.type == HWType::PLUTO)
                        cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
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

        // ── CONNECT 모드: 연결 끊김 감지 > 자동 재연결 (백그라운드) ────────
        if(v.remote_mode && v.net_cli && !v.net_cli->is_connected()){
            static float reconn_timer = 0.f;
            // reconn_busy는 함수 상단에서 선언됨
            reconn_timer -= ImGui::GetIO().DeltaTime;
            if(reconn_timer <= 0.f && !reconn_busy.load()){
                reconn_timer = 3.f;
                reconn_busy.store(true);
                // 재연결을 백그라운드 스레드에서 수행 > UI 블로킹 방지
                auto* cli_ptr = v.net_cli;
                auto* v_ptr   = &v;
                auto* central_ptr = &central_cli;
                std::thread([cli_ptr, v_ptr, central_ptr,
                             central_host  = s_central_host,
                             central_port  = s_central_port,
                             station_id  = s_central_join_station_id,
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
                    }
                    if(ok){
                        for(int ci=0;ci<MAX_CHANNELS;ci++)
                            if(v_ptr->local_ch_out[ci]==3) cli_ptr->cmd_toggle_recv(ci,false);
                        v_ptr->join_manual_scale=false;
                        { std::lock_guard<std::mutex> wlk(v_ptr->wf_events_mtx);
                          v_ptr->wf_events.clear(); }
                        v_ptr->last_tagged_sec = -1;
                    }
                    reconn_busy.store(false);
                }).detach();
            }
        }

        // ── CONNECT 모드: STATUS > gain/hw 동기화 + 주파수 변화 감지 ────────
        if(v.remote_mode && v.net_cli && v.net_cli->is_connected()){
            v.gain_db = v.net_cli->remote_gain_db.load();
            uint8_t hwt = v.net_cli->remote_hw.load();
            if(hwt == 1){ v.hw.gain_min=0.f;   v.hw.gain_max=49.6f; }
            else         { v.hw.gain_min=-12.f; v.hw.gain_max=60.f;  }
            // 주파수 변화 감지 > 오토스케일 트리거
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

        // ── CONNECT 모드: 수신 FFT > waterfall 업데이트 (1초 버퍼) ─────────
        // 수신된 프레임은 1초 지연 후 표시 > 끊김 없는 스크롤 보장
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
                    // JOIN: row_wall_ms 설정 (ms 정밀도, HOST wall_time 기준)
                    {
                        int _slot = v.current_fft_idx % MAX_FFTS_MEMORY;
                        v.row_wall_ms[_slot] =
                            (frm.wall_time > 0) ? (int64_t)frm.wall_time * 1000LL
                            : (int64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count());
                        // HOST IQ 좌표를 JOIN row_write_pos에 저장 → region_save가 정확 매핑
                        if(frm.iq_write_sample > 0){
                            v.row_write_pos[_slot]  = frm.iq_write_sample;
                            v.tm_iq_total_samples   = frm.iq_total_samples;
                        }
                    }
                    // 시간 태그는 HOST가 on_wf_event로 보내는 것만 사용 (중복 방지)
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
                            size_t _n = v.autoscale_accum.size();
                            size_t _lo = (size_t)(_n * 0.15f);
                            std::nth_element(v.autoscale_accum.begin(),
                                v.autoscale_accum.begin()+_lo,
                                v.autoscale_accum.end());
                            float _noise = v.autoscale_accum[_lo];
                            // 피크: 실제 max (99% 분위수는 신호 bin이 너무 적어 노이즈권에 머무름)
                            float _peak = *std::max_element(v.autoscale_accum.begin(),
                                v.autoscale_accum.end());
                            v.display_power_min = _noise - 5.f;
                            v.display_power_max = _peak + 20.f;
                            if(v.display_power_max - v.display_power_min < 20.f)
                                v.display_power_max = v.display_power_min + 20.f;
                            v.join_manual_scale = true; // 수신 frm.pmin 덮어쓰기 차단
                            v.autoscale_accum.clear();
                            v.autoscale_active = false;
                            v.cached_sp_idx = -1;
                            bewe_log_push(0,"[autoscale-JOIN] noise=%.1f peak=%.1f → pmin=%.1f pmax=%.1f\n",
                                _noise, _peak, v.display_power_min, v.display_power_max);
                        }
                    }
                    v.header.num_ffts = std::min(v.total_ffts, MAX_FFTS_MEMORY);
                }
                v.update_wf_row(v.current_fft_idx);
            }
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        toggle_fullscreen();
        // TM 모드: freeze_idx는 스페이스바 진입 시점에만 세팅 (매 프레임 갱신 금지)
        // Ctrl+휠로 tm_offset 변경 시 tm_update_display()가 display_idx를 재계산함
        if(v.texture_needs_recreate){ v.texture_needs_recreate=false; v.create_waterfall_texture(); }

        ImGuiIO& io=ImGui::GetIO();
        bool editing=ImGui::IsAnyItemActive();
        int sci=v.selected_ch;

        // ── Keyboard shortcuts ────────────────────────────────────────────
        if(!editing && !v.eid_panel_open){
            if(ImGui::IsKeyPressed(ImGuiKey_R,false)){
                if(v.remote_mode && v.net_cli){
                    if(v.region.active){
                        // R키 시점: time_start/time_end는 선택 시점에 이미
                        // fft_idx_to_wall_time_ms()으로 정확히 계산됨 (절대 wall_time_ms, ms 정밀도)
                        // HOST/JOIN 워터폴 동일 → 같은 시간 = 같은 데이터
                        {
                            int64_t wt_top_ms = v.fft_idx_to_wall_time_ms(v.region.fft_top);
                            int64_t wt_bot_ms = v.fft_idx_to_wall_time_ms(v.region.fft_bot);
                            if(wt_top_ms > 0 && wt_bot_ms > 0){
                                v.region.time_end_ms   = wt_top_ms;
                                v.region.time_start_ms = wt_bot_ms;
                            }
                            // HOST IQ 좌표: row_write_pos 기반 (파이프라인 지연 0)
                            int64_t sp_top = v.row_write_pos[v.region.fft_top % MAX_FFTS_MEMORY];
                            int64_t sp_bot = v.row_write_pos[v.region.fft_bot % MAX_FFTS_MEMORY];
                            if(sp_top > 0 && sp_bot > 0){
                                v.region.samp_end   = sp_top;
                                v.region.samp_start = sp_bot;
                            } else {
                                v.region.samp_start = v.region.samp_end = 0;
                            }
                        }
                        v.net_cli->cmd_request_region(
                            v.region.fft_top, v.region.fft_bot,
                            v.region.freq_lo, v.region.freq_hi,
                            v.region.time_start_ms, v.region.time_end_ms,
                            v.region.samp_start, v.region.samp_end);
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
                        // JOIN: 채널 선택 시 > 로컬 오디오 녹음 (채널필터에서 IQ 녹음 없음)
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

            // I key: per-channel IQ recording toggle
            if(ImGui::IsKeyPressed(ImGuiKey_I,false) && !io.WantTextInput){
                int ci = v.selected_ch;
                if(ci >= 0 && ci < MAX_CHANNELS && v.channels[ci].filter_active){
                    if(v.remote_mode && v.net_cli){
                        // JOIN: HOST에 IQ 녹음 시작/중지 요청 (100kHz 이하만)
                        float bw_khz = fabsf(v.channels[ci].e - v.channels[ci].s) * 1000.f;
                        if(bw_khz > 100.f){
                            v.rec_na_timer = 3.0f; // "N/A" 표시
                        } else {
                            if(v.channels[ci].iq_rec_on.load())
                                v.net_cli->cmd_stop_iq_rec(ci);
                            else
                                v.net_cli->cmd_start_iq_rec(ci);
                        }
                    } else if(v.channels[ci].dem_run.load()){
                        if(v.channels[ci].iq_rec_on.load())
                            v.stop_iq_rec(ci);
                        else
                            v.start_iq_rec(ci);
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
                        // Pluto에서 SR > 3.2 MSPS (10/20/40/61.44)는 파워 스펙트럼 관측 전용 → 롤링 IQ 차단
                        bool pluto_hi = (v.hw.type == HWType::PLUTO
                                      && v.header.sample_rate > 3200000u);
                        if(pluto_hi){
                            bewe_log_push(0,"[TM IQ] blocked: Pluto SR %.2f MSPS (power spectrum only)\n",
                                          v.header.sample_rate/1e6f);
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
            }
            // 스페이스바: TM 토글 (진입/해제)
            // EID Audio 탭(mode 8) 활성 시에는 audio play/pause 전용 — TM 토글 비활성
            if(ImGui::IsKeyPressed(ImGuiKey_Space,false)
               && !(v.eid_panel_open && v.eid_view_mode == 8)){
                if(v.tm_active.load()){
                    v.tm_offset=0.0f;
                    v.tm_active.store(false);
                } else {
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
                    if(!v.digi_panel_on[sci] && v.channels[sci].digi_run.load()){
                        if(v.remote_mode && v.net_cli)
                            v.net_cli->cmd_stop_digi(sci);
                        else
                            v.stop_digi(sci);
                    }
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
            if(ImGui::IsKeyPressed(ImGuiKey_Delete,false) && !v.eid_panel_open){
                if(sci>=0&&v.channels[sci].filter_active){
                    // IQ 녹음 중이면 중지
                    if(v.channels[sci].iq_rec_on.load())
                        v.stop_iq_rec(sci);
                    // 오디오 녹음 중이면 중지 + 파일 삭제
                    if(v.channels[sci].audio_rec_on.load()){
                        if(v.remote_mode && v.net_cli)
                            v.stop_join_audio_rec(sci);
                        else
                            v.stop_audio_rec(sci);
                        std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
                        for(auto it=v.rec_entries.begin();it!=v.rec_entries.end();++it){
                            if(it->is_audio && it->ch_idx==sci){
                                remove(it->path.c_str());
                                v.rec_entries.erase(it);
                                break;
                            }
                        }
                    }
                    // 전체 IQ 녹음 중이면 중지
                    if(v.rec_on.load() && v.rec_ch==sci) v.stop_rec();
                    if(v.remote_mode && v.net_cli){
                        v.net_cli->cmd_delete_ch(sci);
                    } else {
                        v.stop_dem(sci); v.stop_digi(sci); v.digi_panel_on[sci]=false;
                        v.channels[sci].reset_slot();
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
                        remove((file_ctx.filepath + ".info").c_str()); // 동반 info 삭제
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
        // ── 오버레이 포커스 스택 (LIFO) ───────────────────────────────────
        // 1=EID 2=LOG 3=DIGI 4=SIDE. 어느 경로(토글키/메뉴/닫기버튼)로 열리거나
        // 닫히든 프레임 시작 시 상태 diff를 보고 자동 push/pop.
        static std::vector<int> g_overlay_stack;
        auto push_ov = [](int n){
            g_overlay_stack.erase(std::remove(g_overlay_stack.begin(),
                                              g_overlay_stack.end(), n),
                                  g_overlay_stack.end());
            g_overlay_stack.push_back(n);
        };
        auto pop_ov = [](int n){
            g_overlay_stack.erase(std::remove(g_overlay_stack.begin(),
                                              g_overlay_stack.end(), n),
                                  g_overlay_stack.end());
        };
        auto top_ov = []()->int{
            return g_overlay_stack.empty() ? 0 : g_overlay_stack.back();
        };
        {
            static bool prev_eid=false, prev_log=false, prev_digi=false, prev_side=false;
            bool side_now = v.right_panel_ratio > 0.01f;
            if(v.eid_panel_open != prev_eid){ v.eid_panel_open ? push_ov(1) : pop_ov(1); prev_eid=v.eid_panel_open; }
            if(v.log_panel_open != prev_log){ v.log_panel_open ? push_ov(2) : pop_ov(2); prev_log=v.log_panel_open; }
            if(v.digi_decode_panel_open != prev_digi){ v.digi_decode_panel_open ? push_ov(3) : pop_ov(3); prev_digi=v.digi_decode_panel_open; }
            if(side_now != prev_side){ side_now ? push_ov(4) : pop_ov(4); prev_side=side_now; }
        }

        if(!v.eid_panel_open && ImGui::IsKeyPressed(ImGuiKey_S, false) && !ImGui::GetIO().WantTextInput){
            if(v.right_panel_ratio > 0.01f){
                // 열려있음 > 저장 후 닫기
                right_panel_saved_ratio = v.right_panel_ratio;
                v.right_panel_ratio = 0.0f;
            } else {
                // 닫혀있음 > 마지막 저장값으로 열기 (미설정시 기본값 0.3)
                v.right_panel_ratio = (right_panel_saved_ratio > 0.01f) ? right_panel_saved_ratio : 0.3f;
            }
        }

        // ── 숫자키 1/2/3/4: STATUS/ARCHIVE/BOARD/SCHED 탭 전환 ─────────────
        // 우측 패널이 "가장 상단 오버레이"일 때만 숫자키 전환 적용 (EID/LOG/DIGI이 위면 무시)
        if(top_ov() == 4 && v.right_panel_ratio > 0.01f && !ImGui::GetIO().WantTextInput){
            auto sel = [&](int n){
                stat_open = (n==1); archive_open = (n==2);
                board_open = (n==3); v.sched_panel_open = (n==4);
            };
            if(ImGui::IsKeyPressed(ImGuiKey_1, false)) sel(1);
            else if(ImGui::IsKeyPressed(ImGuiKey_2, false)) sel(2);
            else if(ImGui::IsKeyPressed(ImGuiKey_3, false)) sel(3);
            else if(ImGui::IsKeyPressed(ImGuiKey_4, false)) sel(4);
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
        // Tab > 주파수 입력창 포커스 (텍스트 입력 중이 아닐 때)
        // GLFW 레벨에서 직접 감지 (ImGui가 Tab을 소비해도 동작)
        {
            static bool tab_was_down = false;
            bool tab_down = glfwGetKey(win, GLFW_KEY_TAB) == GLFW_PRESS;
            if(tab_down && !tab_was_down && !io.WantTextInput){
                focus_freq = true;
            }
            tab_was_down = tab_down;
        }
        // 채팅창 열린 상태에서 Enter > 채팅 입력칸 포커스
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
        static int fft_si=3; // 기본 4096
        {
            float tw2=ImGui::CalcTextSize(fft_lbls[fft_si]).x;
            float box_w2=72.0f;
            float px2=std::max(2.0f,(box_w2-tw2)*0.5f-12.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(px2,ImGui::GetStyle().FramePadding.y));
            ImGui::SetNextItemWidth(box_w2);
            // 실제 fft_input_size와 콤보 인덱스 동기화 (HOST/LOCAL/JOIN 전부)
            for(int i=0;i<6;i++) if(fft_sizes[i]==v.fft_input_size){ fft_si=i; break; }
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
        if((v.dev_blade || v.dev_rtl || v.pluto_ctx) || v.remote_mode){
            // BladeRF: 2.5/5/10/20/30.72/61.44 MSPS
            // RTL-SDR: 0.25/0.96/1.44/2.56/3.2 MSPS
            // Pluto  : 0.52/1/2/2.56/3.2/10/20/40/61.44 MSPS (3.2 초과는 USB2 드롭 전제, 파워 스펙트럼 관측 전용)
            static const float blade_srs[]  = {2.5f,5.0f,10.0f,20.0f,30.72f,61.44f,122.88f};
            static const char* blade_lbls[] = {"2.5M","5M","10M","20M","30.72M","61.44M","122.88M"};
            static const float rtl_srs[]    = {0.25f,0.96f,1.44f,2.56f,3.2f};
            static const char* rtl_lbls[]   = {"0.25M","0.96M","1.44M","2.56M","3.2M"};
            static const float pluto_srs[]  = {0.52f,1.0f,2.0f,2.56f,3.2f,10.0f,20.0f,40.0f,61.44f};
            static const char* pluto_lbls[] = {"0.52M","1M","2M","2.56M","3.2M","10M","20M","40M","61.44M"};
            // 0=blade, 1=rtl, 2=pluto
            int hw_mode;
            if(v.remote_mode){
                uint8_t rh = v.net_cli ? v.net_cli->remote_hw.load() : 1;
                hw_mode = (rh == 0) ? 0 : (rh == 2) ? 2 : 1;
            } else if(v.dev_blade) hw_mode = 0;
            else if(v.pluto_ctx)   hw_mode = 2;
            else                   hw_mode = 1;
            const float* sr_list;  const char** sr_lbls;  int sr_count;
            switch(hw_mode){
                case 0: sr_list = blade_srs; sr_lbls = blade_lbls; sr_count = 7; break;
                case 2: sr_list = pluto_srs; sr_lbls = pluto_lbls; sr_count = 9; break;
                default: sr_list = rtl_srs;  sr_lbls = rtl_lbls;   sr_count = 5; break;
            }

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

        // ── Center: mode / station label (LOCAL / HOST : name / JOIN : name) ──
        {
            char mbuf[96];
            if(v.remote_mode){
                snprintf(mbuf, sizeof(mbuf), "JOIN : %s", v.station_name.c_str());
            } else if(!v.station_name.empty()){
                snprintf(mbuf, sizeof(mbuf), "HOST : %s", v.station_name.c_str());
            } else {
                snprintf(mbuf, sizeof(mbuf), "LOCAL");
            }
            ImVec2 msz = ImGui::CalcTextSize(mbuf);
            float mx = (disp_w - msz.x) * 0.5f;
            float my = (TOPBAR_H - ImGui::GetFontSize()) / 2.0f;
            dl->AddText(ImVec2(mx, my), IM_COL32(140,200,255,255), mbuf);
        }

        // ── Right side: channel status + REC + PAUSED ────────────────────
        {
            float rx=disp_w-8.0f;
            float ty2=(TOPBAR_H-ImGui::GetFontSize())/2;

            if(v.rec_on.load()){
                float el=v.rec_sr>0 ? (float)v.rec_frames.load()/(float)v.rec_sr : 0.f;
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
            // SCHED indicator
            {
                std::lock_guard<std::mutex> lk(v.sched_mtx);
                if(v.sched_active_idx >= 0){
                    const char* st="SCHED REC  ";
                    ImVec2 sz=ImGui::CalcTextSize(st); rx-=sz.x;
                    dl->AddText(ImVec2(rx,ty2),IM_COL32(255,100,100,255),st);
                } else {
                    int waiting=0;
                    for(auto& se:v.sched_entries) if(se.status==FFTViewer::SchedEntry::WAITING) waiting++;
                    if(waiting>0){
                        char buf[32]; snprintf(buf,sizeof(buf),"SCHED(%d)  ",waiting);
                        ImVec2 sz=ImGui::CalcTextSize(buf); rx-=sz.x;
                        dl->AddText(ImVec2(rx,ty2),IM_COL32(200,200,100,255),buf);
                    }
                }
            }

        }
        ImGui::PopStyleVar(); // ItemSpacing

        // ── Spectrum + Waterfall ──────────────────────────────────────────
        // ── 레이아웃 계산 ─────────────────────────────────────────────────
        float content_y=TOPBAR_H, content_h=disp_h-content_y-TOPBAR_H;
        const float div_h=14.0f, vdiv_w=8.0f;

        // ── Signal Analysis 토글 (E키) ─── 독립 오버레이 ────
        if(ImGui::IsKeyPressed(ImGuiKey_E, false) && !io.WantTextInput){
            v.eid_panel_open = !v.eid_panel_open;
            if(v.eid_panel_open){
                if(!v.sa_temp_path.empty() &&
                   !v.eid_computing.load() && !v.eid_data_ready.load())
                    v.eid_start(v.sa_temp_path);
            }
        }
        // ── LOG 토글 (L키) ─── 독립 오버레이 ────
        if(ImGui::IsKeyPressed(ImGuiKey_L, false) && !io.WantTextInput){
            v.log_panel_open = !v.log_panel_open;
        }
        // ── DIGITAL DECODE 토글 (Q키) ─── 독립 오버레이 ────
        if(ImGui::IsKeyPressed(ImGuiKey_Q, false) && !io.WantTextInput){
            v.digi_decode_panel_open = !v.digi_decode_panel_open;
        }

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
        if(false){ // SA panel removed
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

            // 패널이 열릴 때: 마지막 활성 탭 복원 (없으면 STATUS)
            if(!prev_right_visible_outer){
                if(!stat_open && !board_open && !v.sched_panel_open && !archive_open)
                    stat_open = true;
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

            // 공통: 시스템 상태 + 안테나 렌더링 (BOARD, STATUS 양쪽에서 호출)
            auto draw_system_status = [&](FFTViewer& vv, const char* tag){
                // Receiver : SDR 이름 [온도°C]  — HOST/LOCAL은 클릭하면 변경 가능
                const char* sdr_name = "Unknown";
                uint8_t sdr_t = 0;
                if(vv.net_cli){
                    uint8_t rh = vv.net_cli->remote_hw.load();
                    sdr_name = (rh == 0) ? "BladeRF 2.0 micro xA9" :
                               (rh == 2) ? "ADALM-Pluto" : "RTL-SDR v4";
                    sdr_t = vv.net_cli->remote_sdr_temp_c.load();
                } else {
                    if(vv.dev_blade){ sdr_name = "BladeRF 2.0 micro xA9";
                        float _t = 0.f;
                        if(bladerf_get_rfic_temperature(vv.dev_blade, &_t) == 0)
                            sdr_t = (uint8_t)std::min(255.f, std::max(0.f, _t));
                    } else if(vv.hw.type == HWType::PLUTO) sdr_name = "ADALM-Pluto";
                    else if(vv.dev_rtl) sdr_name = "RTL-SDR v4";
                }
                ImGui::PushID(tag);
                bool is_host_local = !vv.net_cli; // HOST 또는 LOCAL
                char rx_lbl[128];
                snprintf(rx_lbl, sizeof(rx_lbl), "Receiver : %s [%d\xC2\xB0""C]", sdr_name, (int)sdr_t);
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f,0.85f,1.f,1.f));
                if(is_host_local){
                    static std::vector<std::string> s_rx_avail_cache;
                    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0,0,0,0));
                    ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0,0,0,0));
                    if(ImGui::Selectable(rx_lbl, false, 0, ImVec2(320,0))){
                        s_rx_avail_cache = scan_available_sdrs(); // 팝업 열릴 때 한 번만
                        ImGui::OpenPopup("##rx_pop");
                    }
                    ImGui::PopStyleColor(2);
                    if(ImGui::BeginPopup("##rx_pop")){
                        if(s_rx_avail_cache.empty()){
                            ImGui::TextDisabled("(no SDR detected)");
                        } else {
                            for(const auto& nm : s_rx_avail_cache){
                                const char* pretty = (nm=="bladerf") ? "BladeRF 2.0 micro xA9"
                                                   : (nm=="pluto")   ? "ADALM-Pluto"
                                                   :                   "RTL-SDR v4";
                                bool cur =
                                    (nm=="bladerf" && vv.hw.type == HWType::BLADERF) ||
                                    (nm=="pluto"   && vv.hw.type == HWType::PLUTO)   ||
                                    (nm=="rtlsdr"  && vv.hw.type == HWType::RTLSDR);
                                if(ImGui::Selectable(pretty, cur) && !cur){
                                    std::lock_guard<std::mutex> lk(vv.pending_sdr_mtx);
                                    vv.pending_sdr_name = nm;
                                    vv.pending_sdr_switch.store(true);
                                }
                            }
                        }
                        ImGui::EndPopup();
                    }
                } else {
                    ImGui::TextUnformatted(rx_lbl);
                }
                ImGui::PopStyleColor();
                ImGui::PopID();

                // Antenna : 편집 가능 (HOST 직접, JOIN은 SET_ANTENNA cmd)
                {
                    ImGui::TextUnformatted("Antenna :"); ImGui::SameLine();
                    char cur[32] = {};
                    if(vv.net_cli){
                        std::lock_guard<std::mutex> lk(vv.net_cli->remote_antenna_mtx);
                        memcpy(cur, vv.net_cli->remote_antenna, 32);
                    } else {
                        memcpy(cur, vv.host_antenna, 32);
                    }
                    // 편집 상태: 탭/패널당 고유한 키 필요 (BOARD/STATUS 충돌 방지)
                    static std::string s_edit_tag;    // 현재 편집 중인 tag
                    static char        s_edit_buf[32];
                    bool editing_here = (s_edit_tag == tag);
                    ImGui::PushID(tag);
                    if(!editing_here){
                        // 클릭 가능 라벨
                        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0,0,0,0));
                        ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0,0,0,0));
                        bool ant_clicked = ImGui::Selectable(cur[0] ? cur : " ",
                                             false, 0, ImVec2(320,0));
                        ImGui::PopStyleColor(2);
                        if(ant_clicked){
                            s_edit_tag = tag;
                            memcpy(s_edit_buf, cur, 32);
                            s_edit_buf[31] = '\0';
                        }
                    } else {
                        ImGui::SetNextItemWidth(320);
                        ImGui::SetKeyboardFocusHere();
                        bool enter = ImGui::InputText("##antenna_edit", s_edit_buf, sizeof(s_edit_buf),
                            ImGuiInputTextFlags_EnterReturnsTrue|ImGuiInputTextFlags_AutoSelectAll);
                        bool lost  = !ImGui::IsItemActive() && !ImGui::IsItemHovered();
                        if(enter || (lost && !ImGui::IsItemActive())){
                            s_edit_buf[sizeof(s_edit_buf)-1] = '\0';
                            if(vv.net_cli){
                                vv.net_cli->cmd_set_antenna(s_edit_buf);
                            } else {
                                strncpy(vv.host_antenna, s_edit_buf, sizeof(vv.host_antenna)-1);
                                vv.host_antenna[sizeof(vv.host_antenna)-1] = '\0';
                            }
                            s_edit_tag.clear();
                        }
                        if(ImGui::IsKeyPressed(ImGuiKey_Escape)) s_edit_tag.clear();
                    }
                    ImGui::PopID();
                }

                // Freq / Sample Rate
                {
                    double cf_mhz = vv.net_cli ? (double)vv.net_cli->remote_cf_mhz.load()
                                              : vv.header.center_frequency/1e6;
                    double sr_mhz = vv.net_cli ? vv.net_cli->remote_sr.load()/1e6
                                              : vv.header.sample_rate/1e6;
                    ImGui::Text("Freq : %.4fMHz  /  Sample Rate : %.3fMSPS", cf_mhz, sr_mhz);
                }
                // HOST CPU/RAM
                if(vv.net_cli){
                    int h_cpu = vv.net_cli->remote_host_cpu.load();
                    int h_ram = vv.net_cli->remote_host_ram.load();
                    int h_ct  = vv.net_cli->remote_host_cpu_temp.load();
                    ImGui::Text("HOST | CPU : %d%% [%d\xC2\xB0""C]  /  RAM : %d%%", h_cpu, h_ct, h_ram);
                } else {
                    int ct = vv.sysmon_cpu_temp_c.load();
                    ImGui::Text("HOST | CPU : %d%% [%d\xC2\xB0""C]  /  RAM : %d%%",
                        (int)vv.sysmon_cpu, ct, (int)vv.sysmon_ram);
                }
                if(vv.net_cli){
                    int jct = vv.sysmon_cpu_temp_c.load();
                    ImGui::Text("JOIN | CPU : %d%% [%d\xC2\xB0""C]  /  RAM : %d%%",
                        (int)vv.sysmon_cpu, jct, (int)vv.sysmon_ram);
                }
            };

            // ── STATUS 버튼 ───────────────────────────────────────────────
            float btn_x = rpx + 6;
            if(subbar_btn(btn_x, "STATUS", stat_open, IM_COL32(80,255,160,255))){
                stat_open = !stat_open;
                if(stat_open){ archive_open=false; v.sched_panel_open=false; board_open=false; }
            }

            // ── ARCHIVE 버튼 ─────────────────────────────────────────────
            float arch_btn_x = btn_x + 56;
            if(subbar_btn(arch_btn_x, "ARCHIVE", archive_open, IM_COL32(180,140,255,255))){
                archive_open = !archive_open;
                if(archive_open){ stat_open=false; v.sched_panel_open=false; board_open=false; }
            }

            // ── BOARD 버튼 ───────────────────────────────────────────────
            float board_btn_x = arch_btn_x + 64;
            if(subbar_btn(board_btn_x, "BOARD", board_open, IM_COL32(255,200,80,255))){
                board_open = !board_open;
                if(board_open){ stat_open=false; archive_open=false; v.sched_panel_open=false; }
            }

            // ── SCHED 버튼 ───────────────────────────────────────────────
            float sched_btn_x = board_btn_x + 56;
            if(subbar_btn(sched_btn_x, "SCHED", v.sched_panel_open, IM_COL32(255,100,100,255))){
                v.sched_panel_open = !v.sched_panel_open;
                if(v.sched_panel_open){ stat_open=false; archive_open=false; board_open=false; }
            }

            // ── 패널 콘텐츠 영역 ─────────────────────────────────────────
            dl->AddRectFilled(ImVec2(rpx,rp_content_y),ImVec2(disp_w,content_y+content_h),IM_COL32(12,12,15,255));

            // ── 파일 스캔 (STAT/ARCHIVE 공통) ─────────────────────────────
            static bool arch_priv_open = true;
            static bool arch_share_open = true;
            static bool arch_pub_open = true;
            static float arch_scan_timer = 99.0f;
            static std::unordered_map<std::string,std::string> fsz_cache;
            {
                arch_scan_timer += io.DeltaTime;
                if(g_arch_rescan.load()){ arch_scan_timer = 99.f; g_arch_rescan.store(false); fsz_cache.clear(); }
                if(arch_scan_timer >= 1.0f){
                    arch_scan_timer = 0.0f;

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

                    scan_dir(BEWEPaths::record_iq_dir(),    rec_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::record_audio_dir(), rec_audio_files, fsz_cache);

                    scan_dir(BEWEPaths::private_iq_dir(),    priv_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::private_audio_dir(), priv_audio_files, fsz_cache);
                    priv_files.clear();
                    priv_files.insert(priv_files.end(), priv_iq_files.begin(),    priv_iq_files.end());
                    priv_files.insert(priv_files.end(), priv_audio_files.begin(), priv_audio_files.end());

                    scan_dir(BEWEPaths::public_iq_dir(),    pub_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::public_audio_dir(), pub_audio_files, fsz_cache);
                    shared_files.clear();
                    shared_files.insert(shared_files.end(), pub_iq_files.begin(),    pub_iq_files.end());
                    shared_files.insert(shared_files.end(), pub_audio_files.begin(), pub_audio_files.end());

                    scan_dir(BEWEPaths::share_iq_dir(),    share_iq_files,    fsz_cache);
                    scan_dir(BEWEPaths::share_audio_dir(), share_audio_files, fsz_cache);
                    downloaded_files.clear();
                    downloaded_files.insert(downloaded_files.end(), share_iq_files.begin(),    share_iq_files.end());
                    downloaded_files.insert(downloaded_files.end(), share_audio_files.begin(), share_audio_files.end());

                    // Database/Report 로컬 스캔 (네트워크 소스 없을 때)
                    extern std::vector<ReportFileEntry> g_report_list;
                    extern std::mutex g_report_list_mtx;
                    bool has_net_db = (v.net_cli != nullptr) ||
                                      (v.net_srv && v.net_srv->cb.on_relay_broadcast);
                    if(!has_net_db){
                        std::string db_base = BEWEPaths::database_dir();
                        std::vector<DbFileEntry> db_entries;
                        DIR* top = opendir(db_base.c_str());
                        if(top){
                            struct dirent* de;
                            while((de = readdir(top))){
                                if(de->d_name[0]=='.') continue;
                                // flat 구조: .wav 정규 파일만 수집 (_reports/ 등 디렉토리는 자동 제외)
                                std::string fn(de->d_name);
                                if(fn.size()<5 || fn.substr(fn.size()-4)!=".wav") continue;
                                std::string fp = db_base + "/" + fn;
                                struct stat st{};
                                if(stat(fp.c_str(),&st)!=0 || !S_ISREG(st.st_mode)) continue;
                                DbFileEntry e{};
                                strncpy(e.filename, fn.c_str(), 127);
                                e.size_bytes = (uint64_t)st.st_size;
                                // .info 의 Operator: 필드를 e.operator_name 에 채움
                                FILE* fi = fopen((fp+".info").c_str(), "r");
                                if(fi){
                                    char line[256];
                                    while(fgets(line,sizeof(line),fi)){
                                        char k[64]={},val[128]={};
                                        if(sscanf(line,"%63[^:]: %127[^\n]",k,val)>=2){
                                            if(strcmp(k,"Operator")==0){ strncpy(e.operator_name,val,31); break; }
                                        }
                                    }
                                    fclose(fi);
                                }
                                db_entries.push_back(e);
                            }
                            closedir(top);
                        }
                        { std::lock_guard<std::mutex> lk(g_db_list_mtx);
                          g_db_list = std::move(db_entries); }

                        std::string rpt_dir = db_base + "/_reports";
                        std::vector<ReportFileEntry> rpt_entries;
                        DIR* rd = opendir(rpt_dir.c_str());
                        if(rd){
                            struct dirent* re2;
                            while((re2 = readdir(rd))){
                                if(re2->d_name[0]=='.') continue;
                                std::string n(re2->d_name);
                                if(n.size()<6 || n.substr(n.size()-5)!=".info") continue;
                                ReportFileEntry re{};
                                strncpy(re.filename, n.substr(0,n.size()-5).c_str(), 127);
                                FILE* fi = fopen((rpt_dir+"/"+n).c_str(), "r");
                                if(fi){
                                    char line[256]; int pos=0;
                                    while(fgets(line,sizeof(line),fi)){
                                        char k[64]={},val[128]={};
                                        if(sscanf(line,"%63[^:]: %127[^\n]",k,val)>=1){
                                            if(strcmp(k,"Operator")==0) strncpy(re.reporter,val,31);
                                        }
                                        int l=(int)strlen(line);
                                        if(pos+l<255){ memcpy(re.info_summary+pos,line,l); pos+=l; }
                                    }
                                    fclose(fi);
                                }
                                rpt_entries.push_back(re);
                            }
                            closedir(rd);
                        }
                        { std::lock_guard<std::mutex> lk(g_report_list_mtx);
                          g_report_list = std::move(rpt_entries); }
                    }
                }
            }

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

                // ── 탭 바 ─────────────────────────────────────────────────
                ImGui::PushStyleColor(ImGuiCol_Tab,            ImVec4(0.12f,0.12f,0.16f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabHovered,     ImVec4(0.20f,0.30f,0.45f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabActive,      ImVec4(0.15f,0.40f,0.65f,1.f));
                if(ImGui::BeginTabBar("##stat_tabs")){

                // ══════════════════════════════════════════════════════════
                // ── STATUS 탭 ─────────────────────────────────────────────
                // ══════════════════════════════════════════════════════════
                if(ImGui::BeginTabItem("STATUS")){
                    ImGui::BeginChild("##link_scroll", ImVec2(0,0), false,
                        ImGuiWindowFlags_HorizontalScrollbar);

                    // ── System Status (Operators 위) ──────────────────────
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::CollapsingHeader("System Status##status")){
                        ImGui::Indent(8.f);
                        draw_system_status(v, "status");
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

                    // ── Active Channels / Holding Channels 공통 렌더링 ───
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
                    // 한 채널 행 렌더링 — return true면 이 채널은 delete/skip 처리되어 다음으로
                    auto render_channel_row = [&](int ci, bool is_holding) -> void {
                        Channel& ch = v.channels[ci];

                            float cf_mhz=(ch.s+ch.e)/2.0f;
                            float bw_khz=(ch.e-ch.s)*1000.0f;
                            const char* mnames[]={"--","AM","FM","MAG"};
                            // Holding: stop_dem이 ch.mode를 NONE으로 지우므로 dem_paused_mode를 표시
                            int mi_raw = is_holding ? (int)ch.dem_paused_mode : (int)ch.mode;
                            int mi = mi_raw; if(mi<0||mi>3) mi=0;

                            // ── 채널 색상 (draw_all_channels와 동일) ──────
                            bool is_arec = !is_holding && ch.audio_rec_on.load();
                            bool is_irec = !is_holding && (v.rec_on.load()&&ci==v.rec_ch);
                            bool dem = !is_holding && (v.remote_mode
                                ? (ch.mode!=Channel::DM_NONE)
                                : ch.dem_run.load());
                            bool gate = !is_holding && ch.sq_gate.load();

                            ImU32 mode_col;
                            if(is_holding)
                                mode_col=IM_COL32(120,120,140,255); // Holding: 어두운 회색
                            else if(is_irec||is_arec)
                                mode_col=IM_COL32(255,60,60,255);
                            else if(!dem||ch.mode==Channel::DM_NONE)
                                mode_col=IM_COL32(160,160,160,255);
                            else if(ch.mode==Channel::DM_AM)
                                mode_col=IM_COL32(80,200,255,255);
                            else if(ch.mode==Channel::DM_FM)
                                mode_col=IM_COL32(255,220,50,255);
                            else
                                mode_col=IM_COL32(180,80,255,255);

                            // 게이트 닫힌 시점 추적
                            float now_t=(float)ImGui::GetTime();
                            if(ch.sq_gate_prev && !gate)
                                ch.sq_last_close_t=now_t;
                            ch.sq_gate_prev=gate;
                            bool recently_active=gate || (now_t-ch.sq_last_close_t<3.0f);

                            // 스컬치 게이트: 열리면 밝게, 최근 활동이면 75%, 비활성이면 55%
                            float bright = gate ? 1.0f : (recently_active ? 0.8f : 0.55f);
                            float alpha = gate ? 1.0f : (recently_active ? 0.95f : 0.85f);
                            ImVec4 tc_v=ImVec4(
                                ((mode_col>>IM_COL32_R_SHIFT)&0xFF)/255.f*bright,
                                ((mode_col>>IM_COL32_G_SHIFT)&0xFF)/255.f*bright,
                                ((mode_col>>IM_COL32_B_SHIFT)&0xFF)/255.f*bright, alpha);

                            // 최근 활동 배경 하이라이트
                            if(recently_active && dem && !ch.selected){
                                ImVec2 cp=ImGui::GetCursorScreenPos();
                                float rw=ImGui::GetContentRegionAvail().x;
                                ImGui::GetWindowDrawList()->AddRectFilled(
                                    ImVec2(cp.x-4,cp.y-1),
                                    ImVec2(cp.x+rw,cp.y+ImGui::GetTextLineHeight()+2),
                                    IM_COL32(
                                        (uint8_t)(((mode_col>>IM_COL32_R_SHIFT)&0xFF)*0.15f),
                                        (uint8_t)(((mode_col>>IM_COL32_G_SHIFT)&0xFF)*0.15f),
                                        (uint8_t)(((mode_col>>IM_COL32_B_SHIFT)&0xFF)*0.15f),120));
                                ImGui::GetWindowDrawList()->AddLine(
                                    ImVec2(cp.x-4,cp.y-1),
                                    ImVec2(cp.x-4,cp.y+ImGui::GetTextLineHeight()+2),
                                    mode_col, 2.0f);
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
                                    mode_col, recently_active?3.0f:1.5f);
                            }

                            // 텍스트 (고정폭 정렬)
                            char label[96];
                            int dn=v.freq_sorted_display_num(ci);
                            int act_s=(int)ch.sq_active_time, tot_s=(int)ch.sq_total_time;
                            snprintf(label,sizeof(label),"[%2d] %-3s %10.3f MHz %6.0fkHz  [%4d/%5ds]",
                                dn,mnames[mi],cf_mhz,bw_khz,act_s,tot_s);
                            ImGui::PushID(ci*1000+700);
                            ImGui::PushStyleColor(ImGuiCol_Text,tc_v);
                            // gate open 또는 최근 활동이면 볼드 효과 (1px offset)
                            if(recently_active && dem){
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
                                    if(v.rec_on.load() && v.rec_ch==ci) v.stop_rec();
                                    if(v.channels[ci].audio_rec_on.load()){
                                        if(v.remote_mode && v.net_cli) v.stop_join_audio_rec(ci);
                                        else v.stop_audio_rec(ci);
                                    }
                                    if(v.channels[ci].iq_rec_on.load()) v.stop_iq_rec(ci);
                                    if(v.net_cli) v.net_cli->cmd_delete_ch(ci);
                                    v.stop_dem(ci); v.stop_digi(ci);
                                    v.channels[ci].reset_slot();
                                    if(v.net_cli) v.net_cli->audio[ci].clear();
                                    v.local_ch_out[ci]=1;
                                    v.ch_created_by_me[ci] = false; v.ch_pending_create[ci] = false;
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
                                if(v.rec_on.load() && v.rec_ch==ci) v.stop_rec();
                                if(v.channels[ci].audio_rec_on.load()){
                                    if(v.remote_mode && v.net_cli) v.stop_join_audio_rec(ci);
                                    else v.stop_audio_rec(ci);
                                }
                                if(v.channels[ci].iq_rec_on.load()) v.stop_iq_rec(ci);
                                if(v.net_cli) v.net_cli->cmd_delete_ch(ci);
                                v.stop_dem(ci); v.stop_digi(ci);
                                v.channels[ci].reset_slot();
                                if(v.net_cli) v.net_cli->audio[ci].clear();
                                v.local_ch_out[ci]=1;
                                v.ch_created_by_me[ci] = false; v.ch_pending_create[ci] = false;
                                v.digi_panel_on[ci]=false;
                                if(v.selected_ch==ci) v.selected_ch=-1;
                                if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels,MAX_CHANNELS);
                                ImGui::PopID();
                                return;
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
                                    {"DEMOD", Channel::DIGI_DEMOD},
                                    {"AUTO",  Channel::DIGI_AUTO_ID},
                                };
                                for(int di=0;di<4;di++){
                                    Channel::DigitalMode dm = dbtn[di].mode;
                                    bool dactive = (ch.digital_mode==dm && ch.digi_run.load());
                                    if(dactive)
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f,0.2f,0.9f,1.f));
                                    else
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f,0.15f,0.2f,1.f));
                                    if(ImGui::SmallButton(dbtn[di].lbl)){
                                        if(v.remote_mode && v.net_cli){
                                            if(dactive){
                                                v.net_cli->cmd_stop_digi(ci);
                                            } else {
                                                if(dm == Channel::DIGI_AIS || dm == Channel::DIGI_DEMOD || dm == Channel::DIGI_AUTO_ID)
                                                    v.net_cli->cmd_start_digi(ci, (int)dm, ch.digi_demod_type, ch.digi_baud_rate);
                                            }
                                        } else {
                                            if(dactive){
                                                v.stop_digi(ci);
                                            } else {
                                                v.stop_digi(ci);
                                                if(dm == Channel::DIGI_AIS || dm == Channel::DIGI_DEMOD || dm == Channel::DIGI_AUTO_ID)
                                                    v.start_digi(ci, dm);
                                            }
                                        }
                                    }
                                    ImGui::PopStyleColor();
                                    if(di<3) ImGui::SameLine(0,2);
                                }
                                // DEMOD config (when DEMOD active or selected)
                                if(ch.digital_mode==Channel::DIGI_DEMOD || (!ch.digi_run.load() && false)){
                                    ImGui::SameLine(0,6);
                                    static const char* mod_names[]={"ASK","FSK","BPSK"};
                                    ImGui::SetNextItemWidth(50);
                                    if(ImGui::BeginCombo("##dmod",mod_names[ch.digi_demod_type%3])){
                                        for(int m=0;m<3;m++){
                                            if(ImGui::Selectable(mod_names[m],ch.digi_demod_type==m))
                                                ch.digi_demod_type=m;
                                        }
                                        ImGui::EndCombo();
                                    }
                                    ImGui::SameLine(0,2);
                                    ImGui::SetNextItemWidth(60);
                                    ImGui::InputFloat("##baud",&ch.digi_baud_rate,0,0,"%.0f");
                                    ImGui::SameLine(0,2);
                                    ImGui::TextDisabled("bd");
                                }
                                // AUTO mode status display
                                if(ch.digital_mode==Channel::DIGI_AUTO_ID && ch.digi_run.load()){
                                    ImGui::SameLine(0,8);
                                    int st = ch.auto_id.state.load();
                                    if(st <= (int)AutoIdState::ANALYZING){
                                        ImGui::TextColored(ImVec4(1,1,0,1),"Analyzing...");
                                    } else {
                                        ModType m = (ModType)ch.auto_id.mod_type.load();
                                        float baud2 = ch.auto_id.baud_rate.load();
                                        float conf = ch.auto_id.confidence.load();
                                        ImGui::TextColored(ImVec4(0,1,0.5f,1),"%s %s %.0fbd (%.0f%%)",
                                            ch.auto_id.protocol_name,
                                            mod_type_name(m), baud2, conf*100);
                                    }
                                }
                            }
                            ImGui::PopID();
                    }; // end render_channel_row

                    // 주파수 오름차순 정렬 헬퍼
                    auto collect_sorted = [&](bool want_holding, int* out, int& out_n){
                        out_n = 0;
                        for(int i=0;i<MAX_CHANNELS;i++){
                            if(!v.channels[i].filter_active) continue;
                            bool h = v.channels[i].dem_paused.load();
                            if(h != want_holding) continue;
                            out[out_n++] = i;
                        }
                        std::sort(out, out+out_n, [&](int a, int b){
                            float ca=(v.channels[a].s+v.channels[a].e)*0.5f;
                            float cb=(v.channels[b].s+v.channels[b].e)*0.5f;
                            return ca<cb;
                        });
                    };

                    // ── Active Channels ──────────────────────────────────
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::CollapsingHeader("Active Channels")){
                        ImGui::Indent(8.f);
                        int ch_order[MAX_CHANNELS]; int ch_count=0;
                        collect_sorted(false, ch_order, ch_count);
                        for(int ci_idx=0; ci_idx<ch_count; ci_idx++)
                            render_channel_row(ch_order[ci_idx], false);
                        if(ch_count==0) ImGui::TextDisabled("  (none)");
                        ImGui::Unindent(8.f);
                    }
                    ImGui::Spacing();

                    // ── Holding Channels (범위 밖으로 나가 자동 pause됨) ──
                    {
                        int hold_order[MAX_CHANNELS]; int hold_count=0;
                        collect_sorted(true, hold_order, hold_count);
                        // 기본으로 열림. 사용자가 이후 토글 가능
                        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
                        if(ImGui::CollapsingHeader("Holding Channels")){
                            ImGui::Indent(8.f);
                            for(int hi_idx=0; hi_idx<hold_count; hi_idx++)
                                render_channel_row(hold_order[hi_idx], true);
                            if(hold_count==0) ImGui::TextDisabled("  (none)");
                            ImGui::Unindent(8.f);
                        }
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
                                    // region save 진행중 (HOST 본인 녹음만 - JOIN 요청은 REQ_CONFIRMED에서 표시)
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
                                    // 채널이 삭제된 IQ REC 항목 자동 정리
                                    for(int ri=(int)v.rec_entries.size()-1;ri>=0;ri--){
                                        auto& re=v.rec_entries[ri];
                                        if(re.is_audio || re.finished || re.is_region) continue;
                                        if(re.ch_idx>=0 && !v.channels[re.ch_idx].filter_active){
                                            v.stop_iq_rec(re.ch_idx);
                                        }
                                    }
                                    using RS = FFTViewer::RecEntry::ReqState;
                                    for(int ri=(int)v.rec_entries.size()-1;ri>=0;ri--){
                                        auto& re=v.rec_entries[ri];
                                        if(re.is_audio) continue;
                                        ImGui::PushID(ri+30000);
                                        if(re.req_state == RS::REQ_NONE && !re.is_region){
                                            // 일반 IQ 녹음 항목 (로컬 녹음)
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
                                                bool iq_hld = (re.ch_idx>=0 && v.channels[re.ch_idx].dem_paused.load());
                                                ImU32 iq_col = iq_hld
                                                    ? IM_COL32(150,150,150,255)
                                                    : (blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                                ImGui::PushStyleColor(ImGuiCol_Text, iq_col);
                                                // 녹음 중 파일 크기: 0.5초마다만 갱신
                                                static std::unordered_map<std::string,std::pair<float,std::string>> rec_sz_cache;
                                                auto& rc=rec_sz_cache[re.filename];
                                                if(t2-rc.first >= 0.5f){ rc.first=t2; rc.second=fmt_filesize("",re.path); }
                                                int iq_rec_secs=0;
                                                if(re.ch_idx>=0){
                                                    if(v.channels[re.ch_idx].iq_rec_on.load() && v.channels[re.ch_idx].iq_rec_sr>0)
                                                        iq_rec_secs=(int)(v.channels[re.ch_idx].iq_rec_frames/v.channels[re.ch_idx].iq_rec_sr);
                                                }
                                                // Holding 중에는 전체 경과 시간 정지
                                                if(!re.finished){
                                                    auto nowt = std::chrono::steady_clock::now();
                                                    if(re.t_last_tick.time_since_epoch().count()==0) re.t_last_tick = re.t_start;
                                                    float dt = std::chrono::duration<float>(nowt - re.t_last_tick).count();
                                                    re.t_last_tick = nowt;
                                                    if(!iq_hld) re.total_elapsed += dt;
                                                }
                                                int iq_total_secs = (int)re.total_elapsed;
                                                int iq_secs=iq_total_secs;
                                                int iq_dn=re.ch_idx>=0?v.freq_sorted_display_num(re.ch_idx):0;
                                                char iq_lbl[512];
                                                const char* iq_tag = iq_hld ? "[HLD]" : "[REC]";
                                                if(!rc.second.empty())
                                                    snprintf(iq_lbl,sizeof(iq_lbl),"%s [%2d] %s  [%d/%ds]  %s",
                                                             iq_tag, iq_dn, re.filename.c_str(), iq_rec_secs, iq_secs, rc.second.c_str());
                                                else
                                                    snprintf(iq_lbl,sizeof(iq_lbl),"%s [%2d] %s  [%d/%ds]",
                                                             iq_tag, iq_dn, re.filename.c_str(), iq_rec_secs, iq_secs);
                                                ImGui::Selectable(iq_lbl, re.ch_idx>=0 && v.selected_ch==re.ch_idx);
                                                if(ImGui::IsItemHovered()){
                                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                                        if(re.ch_idx>=0){
                                                            if(v.selected_ch>=0) v.channels[v.selected_ch].selected=false;
                                                            v.selected_ch=re.ch_idx;
                                                            v.channels[re.ch_idx].selected=true;
                                                        }
                                                    }
                                                    // 우클릭: 녹음 중 파일 컨텍스트 메뉴 (실시간 분석용)
                                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                        file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                                        file_ctx.selected=true;
                                                    }
                                                    // 더블클릭: IQ 녹음 중지 + 채널 삭제
                                                    if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                                                        if(re.ch_idx>=0){
                                                            int ci=re.ch_idx;
                                                            v.stop_iq_rec(ci);
                                                            if(v.channels[ci].audio_rec_on.load()){
                                                                if(v.remote_mode && v.net_cli) v.stop_join_audio_rec(ci);
                                                                else v.stop_audio_rec(ci);
                                                            }
                                                            if(v.net_cli) v.net_cli->cmd_delete_ch(ci);
                                                            v.stop_dem(ci); v.stop_digi(ci);
                                                            v.channels[ci].reset_slot();
                                                            if(v.selected_ch==ci) v.selected_ch=-1;
                                                        }
                                                    }
                                                }
                                                ImGui::PopStyleColor();
                                            }
                                        } else {
                                            // 영역 IQ 요청 항목 - 상태별 표시
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImU32 col=IM_COL32(200,200,200,255);
                                            if(re.req_state==RS::REQ_CONFIRMED){
                                                // 녹음 중
                                                col=blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255);
                                                ImGui::PushStyleColor(ImGuiCol_Text,col);
                                                ImGui::Text("[REC]  IQ Recording ...");
                                                ImGui::PopStyleColor();
                                            } else if(re.req_state==RS::REQ_TRANSFERRING && !re.finished){
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
                                                    ImGui::Selectable(("##rdone"+std::to_string(ri)).c_str(), false, 0, ImVec2(0,0));
                                                else
                                                    ImGui::Selectable(("##rdone"+std::to_string(ri)).c_str(), false, 0, ImVec2(0,0));
                                                ImGui::SameLine(0,0);
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
                                                if(ImGui::IsItemHovered()){
                                                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                        file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                                        file_ctx.selected=true;
                                                    }
                                                    if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !re.path.empty()){
                                                        v.sa_temp_path = re.path;
                                                        v.eid_panel_open = true;
                                                    }
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
                                    // Pass 1: [REC] 항목 (녹음 중) - 주파수순 정렬
                                    std::vector<int> aud_rec_order;
                                    for(int ri=0;ri<(int)v.rec_entries.size();ri++){
                                        auto& re=v.rec_entries[ri];
                                        if(re.is_audio && !re.finished) aud_rec_order.push_back(ri);
                                    }
                                    std::sort(aud_rec_order.begin(),aud_rec_order.end(),[&](int a,int b){
                                        int ca=v.rec_entries[a].ch_idx, cb=v.rec_entries[b].ch_idx;
                                        if(ca<0||cb<0) return a<b;
                                        float fa=(v.channels[ca].s+v.channels[ca].e)*0.5f;
                                        float fb=(v.channels[cb].s+v.channels[cb].e)*0.5f;
                                        return fa<fb;
                                    });
                                    for(int oi=0;oi<(int)aud_rec_order.size();oi++){
                                        int ri=aud_rec_order[oi];
                                        auto& re=v.rec_entries[ri];
                                        ImGui::PushID(ri+32000);
                                        {
                                            int rec_secs=0; // 스컬치 넘긴 실 녹음 시간
                                            bool is_hld = (re.ch_idx>=0 && v.channels[re.ch_idx].dem_paused.load());
                                            if(re.ch_idx>=0){
                                                if(v.channels[re.ch_idx].audio_rec_on.load() && v.channels[re.ch_idx].audio_rec_sr>0)
                                                    rec_secs=(int)(v.channels[re.ch_idx].audio_rec_frames/v.channels[re.ch_idx].audio_rec_sr);
                                            }
                                            // Holding 중에는 전체 경과 시간 정지, 아니면 실 delta 누적
                                            if(!re.finished){
                                                auto nowt = std::chrono::steady_clock::now();
                                                if(re.t_last_tick.time_since_epoch().count()==0) re.t_last_tick = re.t_start;
                                                float dt = std::chrono::duration<float>(nowt - re.t_last_tick).count();
                                                re.t_last_tick = nowt;
                                                if(!is_hld) re.total_elapsed += dt;
                                            }
                                            int total_secs = (int)re.total_elapsed;
                                            int secs=total_secs;
                                            float t2=(float)ImGui::GetTime();
                                            bool blink=(fmodf(t2,0.8f)<0.4f);
                                            ImU32 col_active = is_hld
                                                ? IM_COL32(150,150,150,255)
                                                : (blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                                            ImGui::PushStyleColor(ImGuiCol_Text, col_active);
                                            static std::unordered_map<std::string,std::pair<float,std::string>> aud_sz_cache;
                                            auto& ac=aud_sz_cache[re.filename];
                                            if(t2-ac.first >= 0.5f){ ac.first=t2; ac.second=fmt_filesize("",re.path); }
                                            char rec_lbl[512];
                                            int dn=re.ch_idx>=0?v.freq_sorted_display_num(re.ch_idx):0;
                                            const char* tag = is_hld ? "[HLD]" : "[REC]";
                                            if(!ac.second.empty())
                                                snprintf(rec_lbl,sizeof(rec_lbl),"%s [%2d] %s  [%d/%ds]  %s", tag, dn, re.filename.c_str(), rec_secs, secs, ac.second.c_str());
                                            else
                                                snprintf(rec_lbl,sizeof(rec_lbl),"%s [%2d] %s  [%d/%ds]", tag, dn, re.filename.c_str(), rec_secs, secs);
                                            ImGui::Selectable(rec_lbl, re.ch_idx>=0 && v.selected_ch==re.ch_idx);
                                            if(ImGui::IsItemHovered()){
                                                if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                                    if(re.ch_idx>=0){
                                                        if(v.selected_ch>=0) v.channels[v.selected_ch].selected=false;
                                                        v.selected_ch=re.ch_idx;
                                                        v.channels[re.ch_idx].selected=true;
                                                    }
                                                }
                                                if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                                    file_ctx={true,io.MousePos.x,io.MousePos.y,re.path,re.filename};
                                                    file_ctx.selected=true;
                                                }
                                            }
                                            ImGui::PopStyleColor();
                                        }
                                        ImGui::PopID();
                                    }
                                    // Pass 2: [Done] 항목
                                    for(int ri=(int)v.rec_entries.size()-1;ri>=0;ri--){
                                        auto& re=v.rec_entries[ri];
                                        if(!re.is_audio || !re.finished) continue;
                                        ImGui::PushID(ri+32000);
                                        {
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

                    // Archive는 별도 ARCHIVE 탭으로 이동됨

                    if(false){ // ── (Archive removed from STATUS) ──
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

                        if(false){ // SA panel removed - spectrogram is now in EID overlay (E key)
                if(v.sa_mode || v.sa_computing.load()){
                    (void)0;
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
                    bool mouse_in_sa = !v.eid_panel_open &&
                                       (mp.x >= sa_x0 && mp.x < sa_x1 &&
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
                        // UV > 텍스처 공간
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
                        // 텍스처 UV > 화면 좌표
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

                        // 주파수 계산: 텍스처 X > bin > 실제 주파수
                        // bin 0 = cf - sr/2, bin N-1 = cf + sr/2 (FFT shift 후)
                        if(v.sa_actual_fft_n > 0 && v.sa_sample_rate > 0){
                            double cf_hz = (double)v.sa_center_freq_hz;
                            double bw_hz = (double)v.sa_sample_rate;
                            double freq_hz = cf_hz - bw_hz * 0.5 + tu * bw_hz;
                            double freq_mhz = freq_hz / 1e6;

                            // 시간 계산: 텍스처 Y > 행 > 시간
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

            // ── SCHED 패널 ────────────────────────────────────────────────
            if(v.sched_panel_open){
                float px=rpx, py=rp_content_y, pw=disp_w-rpx, ph=rp_content_h;
                ImGui::SetNextWindowPos(ImVec2(px,py));
                ImGui::SetNextWindowSize(ImVec2(pw,ph));
                ImGui::SetNextWindowBgAlpha(0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8,8));
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
                ImGui::Begin("##sched_panel", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                    ImGuiWindowFlags_NoDecoration);

                ImGui::PushStyleColor(ImGuiCol_Tab,       ImVec4(0.12f,0.12f,0.16f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabHovered,ImVec4(0.20f,0.30f,0.45f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabActive, ImVec4(0.15f,0.40f,0.65f,1.f));
                if(ImGui::BeginTabBar("##sched_tabs")){
                    if(ImGui::BeginTabItem("SCHED")) ImGui::EndTabItem();
                    ImGui::EndTabBar();
                }
                ImGui::PopStyleColor(3);

                ImGui::TextColored(ImVec4(1.f,0.4f,0.4f,1.f), "Scheduled IQ Recording");
                ImGui::Separator();

                // ── 현재 시각 (상단 표시) ─────────────────────────────────
                {
                    time_t now_t = time(nullptr);
                    struct tm tmn; localtime_r(&now_t, &tmn);
                    char nbuf[32]; strftime(nbuf, sizeof(nbuf), "%H:%M:%S", &tmn);
                    ImGui::TextDisabled("Now  %s", nbuf);
                }

                // ── Input form ────────────────────────────────────────────
                static int sh=0,sm=0,ss=0;
                static float sdur=60, sfreq=100.0f, sbw=25.0f;

                // 라벨을 일정 폭으로 정렬해 가독성 향상
                const float LBL_W = 56.f;

                // ── When 행 ──────────────────────────────────────────────
                ImGui::AlignTextToFramePadding();
                ImGui::Text("When"); ImGui::SameLine(LBL_W);
                ImGui::SetNextItemWidth(56); ImGui::InputInt("##sh",&sh,0,0); ImGui::SameLine(0,3);
                ImGui::Text(":"); ImGui::SameLine(0,3);
                ImGui::SetNextItemWidth(56); ImGui::InputInt("##sm",&sm,0,0); ImGui::SameLine(0,3);
                ImGui::Text(":"); ImGui::SameLine(0,3);
                ImGui::SetNextItemWidth(56); ImGui::InputInt("##ss",&ss,0,0); ImGui::SameLine(0,8);
                auto set_time_offset = [&](int delta_sec){
                    time_t now2 = time(nullptr) + delta_sec;
                    struct tm t3; localtime_r(&now2, &t3);
                    sh = t3.tm_hour; sm = t3.tm_min; ss = t3.tm_sec;
                };
                if(ImGui::SmallButton("Now"))    set_time_offset(0);    ImGui::SameLine(0,3);
                if(ImGui::SmallButton("+10s"))   set_time_offset(10);   ImGui::SameLine(0,3);
                if(ImGui::SmallButton("+1m"))    set_time_offset(60);   ImGui::SameLine(0,3);
                if(ImGui::SmallButton("+5m"))    set_time_offset(300);  ImGui::SameLine(0,3);
                if(ImGui::SmallButton("+30m"))   set_time_offset(1800);

                // ── Duration 행 ──────────────────────────────────────────
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Dur"); ImGui::SameLine(LBL_W);
                ImGui::SetNextItemWidth(140); ImGui::InputFloat("##dur",&sdur,1,10,"%.0f s");
                ImGui::SameLine(0,8);
                if(ImGui::SmallButton("10s"))  sdur = 10;   ImGui::SameLine(0,3);
                if(ImGui::SmallButton("30s"))  sdur = 30;   ImGui::SameLine(0,3);
                if(ImGui::SmallButton("1m"))   sdur = 60;   ImGui::SameLine(0,3);
                if(ImGui::SmallButton("5m"))   sdur = 300;  ImGui::SameLine(0,3);
                if(ImGui::SmallButton("10m"))  sdur = 600;
                if(sdur < 1.f) sdur = 1.f;

                // ── Freq 행 ──────────────────────────────────────────────
                ImGui::AlignTextToFramePadding();
                ImGui::Text("Freq"); ImGui::SameLine(LBL_W);
                ImGui::SetNextItemWidth(140); ImGui::InputFloat("##freq",&sfreq,0.1f,1.0f,"%.4f MHz");
                ImGui::SameLine(0,8);
                if(ImGui::SmallButton("Use current")){
                    double cur_cf = v.remote_mode
                        ? (v.net_cli ? (double)v.net_cli->remote_cf_mhz.load() : 0.0)
                        : (double)(v.header.center_frequency / 1e6);
                    if(cur_cf > 0.0) sfreq = (float)cur_cf;
                }

                // ── BW 행 ────────────────────────────────────────────────
                ImGui::AlignTextToFramePadding();
                ImGui::Text("BW"); ImGui::SameLine(LBL_W);
                ImGui::SetNextItemWidth(140); ImGui::InputFloat("##bw",&sbw,1,10,"%.1f kHz");
                ImGui::SameLine(0,8);
                if(ImGui::SmallButton("12.5"))  sbw = 12.5f;  ImGui::SameLine(0,3);
                if(ImGui::SmallButton("25"))    sbw = 25;     ImGui::SameLine(0,3);
                if(ImGui::SmallButton("100"))   sbw = 100;    ImGui::SameLine(0,3);
                if(ImGui::SmallButton("1MHz"))  sbw = 1000;
                if(sbw < 1.f) sbw = 1.f;

                // 입력 시각을 절대시간으로 환산해 미리 표시 + overlap 프리뷰
                time_t preview_st = 0;
                {
                    time_t now3 = time(nullptr);
                    struct tm t4; localtime_r(&now3,&t4);
                    t4.tm_hour=sh; t4.tm_min=sm; t4.tm_sec=ss;
                    preview_st = mktime(&t4);
                    if(preview_st <= now3) preview_st += 86400;
                    int delta = (int)(preview_st - now3);
                    int dmin = delta/60, dsec = delta%60;
                    ImGui::TextDisabled("Starts in %d:%02d  (duration %.0fs)", dmin, dsec, sdur);
                }
                bool preview_overlap = false;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    preview_overlap = v.sched_has_overlap(preview_st, sdur);
                }
                if(preview_overlap)
                    ImGui::TextColored(ImVec4(1,0.4f,0.4f,1), "! overlaps existing entry");

                // LOCAL/HOST: SDR 필수 (BladeRF/RTL-SDR/Pluto), JOIN: net_cli 연결 필수
                bool can_add = v.remote_mode
                    ? (v.net_cli && v.net_cli->is_connected())
                    : (v.dev_blade || v.dev_rtl
                       || (v.hw.type == HWType::PLUTO && v.pluto_ctx != nullptr));
                bool block_add = !can_add || preview_overlap;
                if(block_add) ImGui::BeginDisabled();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.55f,0.2f,1.f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f,0.7f,0.3f,1.f));
                if(ImGui::Button("  ADD  ")){
                    time_t now=time(nullptr);
                    struct tm tm2; localtime_r(&now,&tm2);
                    tm2.tm_hour=sh; tm2.tm_min=sm; tm2.tm_sec=ss;
                    time_t st=mktime(&tm2);
                    if(st <= now) st += 86400;
                    if(v.remote_mode && v.net_cli){
                        v.net_cli->cmd_add_sched((int64_t)st, sdur, sfreq, sbw);
                        bewe_log_push(0,"[SCHED] Request sent: %02d:%02d:%02d dur=%.0fs freq=%.3fMHz bw=%.0fkHz\n",
                                      sh,sm,ss,sdur,sfreq,sbw);
                    } else {
                        std::lock_guard<std::mutex> lk(v.sched_mtx);
                        if(v.sched_has_overlap(st, sdur)){
                            bewe_log_push(0,"[SCHED] Denied: overlap with existing entry\n");
                        } else {
                            FFTViewer::SchedEntry e;
                            e.start_time=st; e.duration_sec=sdur;
                            e.freq_mhz=sfreq; e.bw_khz=sbw;
                            e.op_index = 0;
                            strncpy(e.operator_name, login_get_id(), sizeof(e.operator_name)-1);
                            v.sched_entries.push_back(e);
                            bewe_log_push(0,"[SCHED] Added: %02d:%02d:%02d dur=%.0fs freq=%.3fMHz bw=%.0fkHz\n",
                                          sh,sm,ss,sdur,sfreq,sbw);
                        }
                    }
                }
                ImGui::PopStyleColor(2);
                if(block_add) ImGui::EndDisabled();
                if(v.remote_mode && !can_add){
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1,0.5f,0.3f,1)," HOST not connected");
                }

                ImGui::Separator();

                // ── Schedule list ─────────────────────────────────────────
                ImGui::BeginChild("##sched_list",ImVec2(0,0),false);
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    time_t now_t = time(nullptr);
                    // 시작시각 오름차순 정렬용 인덱스
                    std::vector<int> order(v.sched_entries.size());
                    for(int i=0;i<(int)order.size();i++) order[i]=i;
                    std::sort(order.begin(), order.end(), [&](int a, int b){
                        return v.sched_entries[a].start_time < v.sched_entries[b].start_time;
                    });

                    const ImU32 col_border_rec = IM_COL32(220,60,60,255);
                    const ImU32 col_bg_rec     = IM_COL32(120,20,20,120);

                    for(int oi=0; oi<(int)order.size(); oi++){
                        int i = order[oi];
                        auto& e=v.sched_entries[i];
                        ImGui::PushID(i);
                        // 상태별 색상/아이콘
                        static const char* st_names[]={"WAIT","REC","DONE","FAIL"};
                        static const char* st_icons[]={"[ ]","[R]","[\xE2\x9C\x93]","[X]"}; // [ ] [R] [✓] [X]
                        static const ImVec4 st_cols[]={
                            {0.7f,0.7f,0.8f,1},{1,0.3f,0.3f,1},{0.3f,0.9f,0.3f,1},{0.9f,0.2f,0.2f,1}};

                        // RECORDING 엔트리는 깜빡이는 배경 + 테두리
                        if(e.status == FFTViewer::SchedEntry::RECORDING){
                            float t2=(float)ImGui::GetTime();
                            float a = 0.5f + 0.5f*sinf(t2*4.f);
                            ImVec2 cp = ImGui::GetCursorScreenPos();
                            float rw = ImGui::GetContentRegionAvail().x;
                            float rh = ImGui::GetTextLineHeight() + 6.f;
                            ImGui::GetWindowDrawList()->AddRectFilled(
                                ImVec2(cp.x-4, cp.y-2), ImVec2(cp.x+rw, cp.y+rh),
                                IM_COL32(120,20,20,(int)(120*a)), 3.f);
                            ImGui::GetWindowDrawList()->AddRect(
                                ImVec2(cp.x-4, cp.y-2), ImVec2(cp.x+rw, cp.y+rh),
                                col_border_rec, 3.f, 0, 1.5f);
                        }

                        ImGui::TextColored(st_cols[e.status],"%s", st_icons[e.status]);
                        ImGui::SameLine();
                        struct tm t2; localtime_r(&e.start_time,&t2);
                        char tb[16]; strftime(tb,sizeof(tb),"%H:%M:%S",&t2);
                        const char* opn = e.operator_name[0] ? e.operator_name : "?";

                        // 상태에 따라 추가 정보
                        char tail[64] = "";
                        if(e.status == FFTViewer::SchedEntry::WAITING){
                            int d = (int)(e.start_time - now_t);
                            if(d > 0){
                                int m=d/60, s=d%60;
                                if(m >= 60){ int h=m/60; m%=60; snprintf(tail, sizeof(tail), "  in %dh%02dm", h,m); }
                                else snprintf(tail, sizeof(tail), "  in %d:%02d", m,s);
                            }
                        } else if(e.status == FFTViewer::SchedEntry::RECORDING){
                            float el = std::chrono::duration<float>(
                                std::chrono::steady_clock::now() - e.rec_started).count();
                            int cur = (int)el, tot = (int)e.duration_sec;
                            snprintf(tail, sizeof(tail), "  REC %d/%ds", cur, tot);
                        }

                        ImGui::Text("%s  %.3fMHz  BW=%.0fkHz  %.0fs  by %s%s",
                                    tb,e.freq_mhz,e.bw_khz,e.duration_sec, opn, tail);
                        ImGui::SameLine();

                        // Remove 권한: HOST(local)이면 항상, JOIN이면 본인 entry만, RECORDING은 불가
                        bool can_remove = (e.status != FFTViewer::SchedEntry::RECORDING);
                        if(can_remove && v.remote_mode){
                            const char* me = login_get_id();
                            can_remove = (strncmp(opn, me, 31) == 0);
                        }
                        if(!can_remove) ImGui::BeginDisabled();
                        if(ImGui::SmallButton("X")){
                            if(v.remote_mode && v.net_cli){
                                v.net_cli->cmd_remove_sched((int64_t)e.start_time, e.freq_mhz);
                            } else {
                                v.sched_entries.erase(v.sched_entries.begin()+i);
                                ImGui::PopID();
                                if(!can_remove) ImGui::EndDisabled();
                                break;
                            }
                        }
                        if(!can_remove) ImGui::EndDisabled();
                        ImGui::PopID();
                    }
                    if(v.sched_entries.empty())
                        ImGui::TextDisabled("  (no scheduled entries)");
                }
                ImGui::EndChild();

                ImGui::End();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar();
            }

            // ── ARCHIVE 패널 ─────────────────────────────────────────────
            if(archive_open){
                float px=rpx, py=rp_content_y, pw=disp_w-rpx, ph=rp_content_h;
                ImGui::SetNextWindowPos(ImVec2(px,py));
                ImGui::SetNextWindowSize(ImVec2(pw,ph));
                ImGui::SetNextWindowBgAlpha(0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8,8));
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.04f,0.04f,0.06f,1.f));
                ImGui::Begin("##archive_panel", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
                ImGui::PushStyleColor(ImGuiCol_Tab,       ImVec4(0.12f,0.12f,0.16f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabHovered,ImVec4(0.20f,0.30f,0.45f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabActive, ImVec4(0.15f,0.40f,0.65f,1.f));
                if(ImGui::BeginTabBar("##archive_tabs")){
                    if(ImGui::BeginTabItem("ARCHIVE")) ImGui::EndTabItem();
                    ImGui::EndTabBar();
                }
                ImGui::PopStyleColor(3);
                // ── 진행 중인 파일 전송 표시 (Save DB / DB Download) ──────
                {
                    std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                    bool any_active = false;
                    for(auto& x : v.file_xfers) if(!x.finished){ any_active = true; break; }
                    if(any_active){
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f,0.95f,1.0f,1.f));
                        ImGui::Text("Transferring");
                        ImGui::PopStyleColor();
                        for(auto& x : v.file_xfers){
                            if(x.finished) continue;
                            float frac = (x.total_bytes > 0)
                                ? (float)((double)x.done_bytes / (double)x.total_bytes) : 0.f;
                            if(frac < 0.f) frac = 0.f;
                            if(frac > 1.f) frac = 1.f;
                            char buf[128];
                            auto fmt_sz = [](uint64_t b, char* o, size_t sz){
                                if(b < 1024)               snprintf(o,sz,"%llu B",(unsigned long long)b);
                                else if(b < 1024*1024)     snprintf(o,sz,"%.1f KB",(double)b/1024);
                                else if(b < 1024ULL*1024*1024) snprintf(o,sz,"%.1f MB",(double)b/(1024*1024));
                                else                       snprintf(o,sz,"%.2f GB",(double)b/(1024ULL*1024*1024));
                            };
                            char dn[32], tn[32];
                            fmt_sz(x.done_bytes, dn, sizeof(dn));
                            fmt_sz(x.total_bytes, tn, sizeof(tn));
                            snprintf(buf,sizeof(buf),"%s / %s (%.0f%%)", dn, tn, frac*100.f);
                            ImGui::ProgressBar(frac, ImVec2(-1, 0), buf);
                            ImGui::TextDisabled("  %s", x.filename.c_str());
                        }
                        // 완료된 항목은 약 5초 후 정리
                        static double last_purge = 0;
                        double now_t = ImGui::GetTime();
                        if(now_t - last_purge > 1.0){
                            last_purge = now_t;
                            v.file_xfers.erase(std::remove_if(v.file_xfers.begin(),
                                v.file_xfers.end(),
                                [](const FFTViewer::FileXfer& f){ return f.finished; }),
                                v.file_xfers.end());
                        }
                        ImGui::Separator();
                    }
                }

                ImGui::BeginChild("##archive_scroll", ImVec2(0,0), false,
                    ImGuiWindowFlags_HorizontalScrollbar);

                // ── Private ──────────────────────────────────────────────
                // 파일 목록 렌더링 헬퍼: 파일명 + 크기 + WAV 시간
                static std::unordered_map<std::string,std::string> arch_info_cache;
                static std::unordered_map<std::string,std::string> arch_info_tip_cache;
                if(g_arch_cache_dirty){ arch_info_cache.clear(); arch_info_tip_cache.clear(); g_arch_cache_dirty=false; }
                auto draw_arch_file = [&](const std::string& dir, const std::string& fn){
                    std::string fp = dir + "/" + fn;
                    // 캐시된 정보 (크기+시간)
                    auto& cached = arch_info_cache[fp];
                    if(cached.empty()){
                        struct stat st{}; char info[64]="";
                        if(stat(fp.c_str(),&st)==0){
                            double mb = st.st_size / 1048576.0;
                            // WAV 시간 추정: (filesize - 44) / (sr * 2ch * 2bytes)
                            // sr은 WAV 헤더에서 읽어야 하지만, 빠른 추정용
                            double sec = 0;
                            FILE* wf = fopen(fp.c_str(), "rb");
                            if(wf){
                                uint8_t hdr[44]; if(fread(hdr,1,44,wf)==44){
                                    uint32_t wsr = *(uint32_t*)(hdr+24);
                                    uint16_t wch = *(uint16_t*)(hdr+22);
                                    uint16_t wbps= *(uint16_t*)(hdr+34);
                                    if(wsr>0 && wch>0 && wbps>0)
                                        sec = (double)(st.st_size-44) / (wsr * wch * (wbps/8));
                                }
                                fclose(wf);
                            }
                            if(sec > 0) snprintf(info,sizeof(info),"%4.0fs %6.1fM",sec,mb);
                            else        snprintf(info,sizeof(info),"     %6.1fM",mb);
                        }
                        cached = info;
                    }
                    float pw = ImGui::GetContentRegionAvail().x;
                    float fn_w = pw * 0.66f;
                    bool sel = file_ctx.selected && file_ctx.filepath==fp;
                    ImGui::Selectable(fn.c_str(), sel, ImGuiSelectableFlags_SpanAllColumns, ImVec2(pw, 0));
                    bool item_hov = ImGui::IsItemHovered();
                    if(!cached.empty()){
                        ImGui::SameLine(fn_w + 8.f);
                        ImGui::TextDisabled("%s", cached.c_str());
                    }
                    if(item_hov){
                        // .info 파일이 있으면 툴팁 표시 (핵심 필드만)
                        std::string ipath = fp + ".info";
                        auto& tip = arch_info_tip_cache[fp];
                        if(tip.empty()){
                            FILE* fi = fopen(ipath.c_str(), "r");
                            if(fi){
                                // 핵심 필드만: Day, Up Time, Down Time, Frequency, Modulation
                                // 레거시 "Time"/"Freq" 매핑 포함
                                static const char* key_fields[] = {
                                    "Day", "Up Time", "Down Time", "Time",
                                    "Frequency", "Freq", "Modulation"
                                };
                                constexpr int KEY_N = sizeof(key_fields)/sizeof(key_fields[0]);
                                char line[256]; std::string acc;
                                while(fgets(line,sizeof(line),fi)){
                                    char k[64]={},val[256]={};
                                    if(sscanf(line,"%63[^:]: %255[^\n]",k,val)!=2 || !val[0])
                                        continue;
                                    for(int ki=0; ki<KEY_N; ki++){
                                        if(strcmp(k, key_fields[ki]) == 0){
                                            const char* disp_k = k;
                                            if(strcmp(k, "Time")==0) disp_k = "Up Time";
                                            else if(strcmp(k, "Freq")==0) disp_k = "Frequency";
                                            acc += disp_k; acc += ": "; acc += val; acc += "\n";
                                            break;
                                        }
                                    }
                                }
                                fclose(fi);
                                tip = acc.empty() ? " " : acc;
                            } else {
                                tip = " ";
                            }
                        }
                        if(tip.size() > 1) ImGui::SetTooltip("%s", tip.c_str());

                        if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                            file_ctx.selected=true; file_ctx.filepath=fp; file_ctx.filename=fn; file_ctx.is_public=false;
                        }
                        if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                            file_ctx={true,io.MousePos.x,io.MousePos.y,fp,fn}; file_ctx.selected=true;
                        }
                    }
                };

                // ── Record (Private와 동일 형식) ─────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Record##arch")){
                    ImGui::Indent(8.f);
                    // 진행중 항목 (rec_entries에서 !finished)
                    {
                        std::lock_guard<std::mutex> rlk(v.rec_entries_mtx);
                        for(auto& re : v.rec_entries){
                            if(re.finished) continue;
                            ImGui::PushID(&re);
                            float t2a=(float)ImGui::GetTime();
                            bool blink=(fmodf(t2a,0.8f)<0.4f);
                            ImGui::PushStyleColor(ImGuiCol_Text,
                                blink?IM_COL32(255,80,80,255):IM_COL32(200,60,60,255));
                            float el=std::chrono::duration<float>(std::chrono::steady_clock::now()-re.t_start).count();
                            int secs=(int)el;
                            if(re.is_region && re.req_state==FFTViewer::RecEntry::REQ_TRANSFERRING)
                                ImGui::Text("[Transferring]  %s  (%.1f/%.1fM)",
                                    re.filename.c_str(), re.xfer_done/1048576.0, re.xfer_total/1048576.0);
                            else
                                ImGui::Text("[REC]  %s  [%ds]", re.filename.c_str(), secs);
                            ImGui::PopStyleColor();
                            ImGui::PopID();
                        }
                    }
                    // 완료된 파일 (IQ/Audio) — Private와 동일 형식
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::TreeNode("IQ##rec")){
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.3f,0.4f));
                        ImGui::Separator(); ImGui::PopStyleColor();
                        for(auto& fn : rec_iq_files) draw_arch_file(BEWEPaths::record_iq_dir(), fn);
                        if(rec_iq_files.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::TreePop();
                    }
                    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.4f,0.3f));
                    ImGui::Separator(); ImGui::PopStyleColor();
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::TreeNode("Audio##rec")){
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.3f,0.4f));
                        ImGui::Separator(); ImGui::PopStyleColor();
                        for(auto& fn : rec_audio_files) draw_arch_file(BEWEPaths::record_audio_dir(), fn);
                        if(rec_audio_files.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::TreePop();
                    }
                    ImGui::Unindent(8.f);
                }
                ImGui::Spacing();

                // ── Private ──────────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Private##arch")){
                    ImGui::Indent(8.f);
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::TreeNode("IQ##priv")){
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.3f,0.4f));
                        ImGui::Separator(); ImGui::PopStyleColor();
                        for(auto& fn : priv_iq_files) draw_arch_file(BEWEPaths::private_iq_dir(), fn);
                        if(priv_iq_files.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::TreePop();
                    }
                    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.4f,0.3f));
                    ImGui::Separator(); ImGui::PopStyleColor();
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::TreeNode("Audio##priv")){
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.3f,0.4f));
                        ImGui::Separator(); ImGui::PopStyleColor();
                        for(auto& fn : priv_audio_files) draw_arch_file(BEWEPaths::private_audio_dir(), fn);
                        if(priv_audio_files.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::TreePop();
                    }
                    ImGui::Unindent(8.f);
                }
                ImGui::Spacing();

                // ── Report ───────────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                ImGui::SetNextItemAllowOverlap();
                bool rpt_open = ImGui::CollapsingHeader("Report");
                ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 60.f);
                if(ImGui::SmallButton("Reload##rpt")){
                    if(v.net_cli) v.net_cli->cmd_request_report_list();
                    else if(v.net_srv && v.net_srv->cb.on_relay_broadcast){
                        auto pkt = make_packet(PacketType::REPORT_LIST_REQ, nullptr, 0);
                        v.net_srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
                    }
                }
                if(rpt_open){
                    ImGui::Indent(8.f);
                    // Central에서 수신한 Report 목록 표시
                    extern std::vector<ReportFileEntry> g_report_list;
                    extern std::mutex g_report_list_mtx;
                    std::vector<ReportFileEntry> rpt_snap;
                    { std::lock_guard<std::mutex> lk(g_report_list_mtx); rpt_snap = g_report_list; }
                    if(rpt_snap.empty()){
                        ImGui::TextDisabled("  (no reports)");
                    } else {
                        // Report 전용 우클릭 상태
                        static struct { bool open=false; float x=0,y=0; std::string filename; } rpt_ctx;
                        ImGui::Indent(12.f);
                        for(auto& re : rpt_snap){
                            float pw_r = ImGui::GetContentRegionAvail().x;
                            float fn_wr = pw_r * 0.66f;
                            ImGui::Selectable(re.filename, false, 0, ImVec2(fn_wr, 0));
                            bool rh = ImGui::IsItemHovered();
                            if(re.reporter[0]){
                                ImGui::SameLine(fn_wr + 8.f);
                                ImGui::TextDisabled("by %s", re.reporter);
                            }
                            if(rh){
                                if(re.info_summary[0]) ImGui::SetTooltip("%s", re.info_summary);
                                if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                    rpt_ctx = {true, io.MousePos.x, io.MousePos.y, std::string(re.filename)};
                                }
                            }
                        }
                        ImGui::Unindent(12.f);
                        // Report 우클릭 팝업
                        if(rpt_ctx.open){
                            ImGui::OpenPopup("##rpt_ctx_popup");
                            rpt_ctx.open = false;
                        }
                        ImGui::SetNextWindowSize(ImVec2(120.f, 0.f));
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
                        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.f,6.f));
                        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f,0.12f,0.18f,1.f));
                        if(ImGui::BeginPopup("##rpt_ctx_popup")){
                            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255,80,80,255));
                            if(ImGui::Selectable("Delete")){
                                if(v.net_cli)
                                    v.net_cli->cmd_report_delete(rpt_ctx.filename.c_str());
                                else if(v.net_srv && v.net_srv->cb.on_relay_broadcast){
                                    PktReportDelete rd{}; strncpy(rd.filename,rpt_ctx.filename.c_str(),127);
                                    auto pkt=make_packet(PacketType::REPORT_DELETE,&rd,sizeof(rd));
                                    v.net_srv->cb.on_relay_broadcast(pkt.data(),pkt.size(),true);
                                }
                            }
                            ImGui::PopStyleColor();
                            ImGui::EndPopup();
                        }
                        ImGui::PopStyleColor();
                        ImGui::PopStyleVar(2);
                    }
                    ImGui::Unindent(8.f);
                }
                ImGui::Spacing();

                // ── Database (Central Server) ─────────────────────────────

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                ImGui::SetNextItemAllowOverlap();
                bool db_open = ImGui::CollapsingHeader("Database");
                ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 60.f);
                if(ImGui::SmallButton("Reload##db")){
                    if(v.net_cli) v.net_cli->cmd_request_db_list();
                    else if(v.net_srv && v.net_srv->cb.on_relay_broadcast){
                        auto pkt = make_packet(PacketType::DB_LIST_REQ, nullptr, 0);
                        v.net_srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
                    }
                }
                if(db_open){
                    ImGui::Indent(8.f);
                    // IQ / Audio 분류
                    std::vector<DbFileEntry> db_iq, db_audio;
                    {
                        std::lock_guard<std::mutex> lk(g_db_list_mtx);
                        for(auto& e : g_db_list){
                            bool iq = (strncmp(e.filename,"IQ_",3)==0 || strncmp(e.filename,"sa_",3)==0);
                            if(iq) db_iq.push_back(e); else db_audio.push_back(e);
                        }
                    }
                    auto draw_db_entry = [&](const DbFileEntry& e){
                        float pw2 = ImGui::GetContentRegionAvail().x;
                        float fn_w2 = pw2 * 0.66f;
                        double mb = e.size_bytes / 1048576.0;
                        // 파일명에서 시간 길이 추정: _HHMMSS-HHMMSS.wav
                        int dur_sec = 0;
                        const char* dot = strrchr(e.filename, '.');
                        if(dot && (dot - e.filename) >= 14){
                            const char* p = dot - 13; // "HHMMSS-HHMMSS"
                            if(p[6] == '-'){
                                int h1=(p[0]-'0')*10+(p[1]-'0'), m1=(p[2]-'0')*10+(p[3]-'0'), s1=(p[4]-'0')*10+(p[5]-'0');
                                int h2=(p[7]-'0')*10+(p[8]-'0'), m2=(p[9]-'0')*10+(p[10]-'0'), s2=(p[11]-'0')*10+(p[12]-'0');
                                dur_sec = (h2*3600+m2*60+s2) - (h1*3600+m1*60+s1);
                                if(dur_sec < 0) dur_sec += 86400;
                            }
                        }
                        char info2[32];
                        if(dur_sec > 0) snprintf(info2,sizeof(info2),"%4ds %6.1fM",dur_sec,mb);
                        else            snprintf(info2,sizeof(info2),"     %6.1fM",mb);
                        ImGui::Selectable(e.filename, false, ImGuiSelectableFlags_SpanAllColumns, ImVec2(pw2, 0));
                        if(ImGui::IsItemHovered()){
                            // info_data 파싱하여 Key: Value 줄 단위로 툴팁 표시 (Archive와 동일)
                            std::string tip;
                            const char* p = e.info_data;
                            while(p && *p){
                                char k[64]={},val[256]={};
                                if(sscanf(p,"%63[^:]: %255[^\n]",k,val)==2 && val[0]){
                                    tip += k; tip += ": "; tip += val; tip += "\n";
                                }
                                const char* nl = strchr(p,'\n');
                                if(!nl) break;
                                p = nl + 1;
                            }
                            if(!tip.empty()) ImGui::SetTooltip("%s", tip.c_str());
                            else ImGui::SetTooltip("by %s", e.operator_name);
                            if(ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                db_ctx = {true, io.MousePos.x, io.MousePos.y,
                                          std::string(e.filename), std::string(e.operator_name),
                                          (strncmp(e.filename,"IQ_",3)==0||strncmp(e.filename,"sa_",3)==0)};
                                bewe_log_push(0,"[UI] DB right-click: '%s'\n", e.filename);
                            }
                        }
                        ImGui::SameLine(fn_w2 + 8.f);
                        ImGui::TextDisabled("%s", info2);
                    };
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::TreeNode("IQ##db")){
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.3f,0.4f));
                        ImGui::Separator(); ImGui::PopStyleColor();
                        for(auto& e : db_iq) draw_db_entry(e);
                        if(db_iq.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::TreePop();
                    }
                    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.4f,0.3f));
                    ImGui::Separator(); ImGui::PopStyleColor();
                    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    if(ImGui::TreeNode("Audio##db")){
                        ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.3f,0.3f,0.3f,0.4f));
                        ImGui::Separator(); ImGui::PopStyleColor();
                        for(auto& e : db_audio) draw_db_entry(e);
                        if(db_audio.empty()) ImGui::TextDisabled("  (empty)");
                        ImGui::TreePop();
                    }
                    ImGui::Unindent(8.f);
                }

                ImGui::EndChild();
                ImGui::End();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar();
            }

            // ── DB 우클릭 팝업: Download / Delete ─────────────────────
            if(db_ctx.open){
                ImGui::OpenPopup("##db_ctx_popup");
                db_ctx.open = false;
            }
            ImGui::SetNextWindowSize(ImVec2(120.f, 0.f));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.f,6.f));
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f,0.12f,0.18f,1.f));
            if(ImGui::BeginPopup("##db_ctx_popup")){
                if(ImGui::Selectable("Download")){
                    if(v.net_cli){
                        bewe_log_push(0,"[UI] DB_DOWNLOAD_REQ: '%s' by '%s'\n", db_ctx.filename.c_str(), db_ctx.operator_name.c_str());
                        v.net_cli->cmd_db_download(db_ctx.filename.c_str(), db_ctx.operator_name.c_str());
                    } else if(v.net_srv){
                        PktDbDownloadReq req{};
                        strncpy(req.filename, db_ctx.filename.c_str(), 127);
                        strncpy(req.operator_name, db_ctx.operator_name.c_str(), 31);
                        auto pkt = make_packet(PacketType::DB_DOWNLOAD_REQ, &req, sizeof(req));
                        if(v.net_srv->cb.on_relay_broadcast)
                            v.net_srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
                    }
                }
                ImGui::Separator();
                ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255,80,80,255));
                if(ImGui::Selectable("Delete")){
                    if(v.net_cli){
                        bewe_log_push(0,"[UI] DB_DELETE_REQ: '%s' by '%s'\n", db_ctx.filename.c_str(), db_ctx.operator_name.c_str());
                        v.net_cli->cmd_db_delete(db_ctx.filename.c_str(), db_ctx.operator_name.c_str());
                    } else if(v.net_srv){
                        if(v.net_srv->cb.on_relay_broadcast){
                            PktDbDeleteReq req{};
                            strncpy(req.filename, db_ctx.filename.c_str(), 127);
                            strncpy(req.operator_name, db_ctx.operator_name.c_str(), 31);
                            auto pkt = make_packet(PacketType::DB_DELETE_REQ, &req, sizeof(req));
                            v.net_srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
                        } else {
                            std::string fpath = BEWEPaths::database_dir() + "/" + db_ctx.filename;
                            remove(fpath.c_str());
                            remove((fpath + ".info").c_str());
                        }
                    }
                }
                ImGui::PopStyleColor();
                ImGui::EndPopup();
            }
            ImGui::PopStyleColor();
            ImGui::PopStyleVar(2);

            // (Signal Analysis는 별도 독립 오버레이로 이동 - 아래 참고)
            if(false){ // disabled – old EID block
                if(v.eid_computing.load()){
                    v.eid_anim_timer += io.DeltaTime;
                    int dots = ((int)(v.eid_anim_timer / 0.5f) % 3) + 1;
                    char msg[32]; snprintf(msg, sizeof(msg), "Loading %.*s", dots, "...");
                    ImVec2 msz = ImGui::CalcTextSize(msg);
                    dl->AddText(ImVec2(rpx+(rp_w-msz.x)/2, rp_content_y+(rp_content_h-msz.y)/2),
                                IM_COL32(255,100,180,255), msg);
                } else if(v.eid_data_ready.load()){
                    // ── 플롯 영역 ──────────────────────────────────────────
                    const float LM = 60.f, RM = 10.f, TM = 28.f, BM = 30.f;
                    float ea_x0 = rpx + LM, ea_y0 = rp_content_y + TM;
                    float ea_x1 = disp_w - RM, ea_y1 = content_y + content_h - BM;
                    float ea_w = ea_x1 - ea_x0, ea_h = ea_y1 - ea_y0;
                    if(ea_w < 10.f || ea_h < 10.f) goto eid_skip;

                    // ── 파일 정보 헤더 바 ──────────────────────────────────
                    {
                        uint32_t sr = v.eid_sample_rate > 0 ? v.eid_sample_rate : 1;
                        double dur = (double)v.eid_total_samples / sr;
                        const char* mode_names[] = {"Signal","I/Q","Phase","Frequency"};
                        char hdr[256];
                        if(v.eid_center_freq_hz > 0){
                            double cf_mhz = v.eid_center_freq_hz / 1e6;
                            if(sr >= 1000000)
                                snprintf(hdr, sizeof(hdr), "Samples: %lld | SR: %.2f MSPS | Duration: %.4fs | CF: %.3f MHz | [%s]",
                                    (long long)v.eid_total_samples, sr/1e6, dur, cf_mhz, mode_names[v.eid_view_mode]);
                            else
                                snprintf(hdr, sizeof(hdr), "Samples: %lld | SR: %u SPS | Duration: %.4fs | CF: %.3f MHz | [%s]",
                                    (long long)v.eid_total_samples, sr, dur, cf_mhz, mode_names[v.eid_view_mode]);
                        } else {
                            if(sr >= 1000000)
                                snprintf(hdr, sizeof(hdr), "Samples: %lld | SR: %.2f MSPS | Duration: %.4fs | [%s]",
                                    (long long)v.eid_total_samples, sr/1e6, dur, mode_names[v.eid_view_mode]);
                            else
                                snprintf(hdr, sizeof(hdr), "Samples: %lld | SR: %u SPS | Duration: %.4fs | [%s]",
                                    (long long)v.eid_total_samples, sr, dur, mode_names[v.eid_view_mode]);
                        }
                        dl->AddText(ImVec2(ea_x0, rp_content_y + 4), IM_COL32(160,160,180,220), hdr);
                    }

                    // 배경
                    dl->AddRectFilled(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), IM_COL32(8,8,12,255));
                    dl->AddRect(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), IM_COL32(60,60,80,255));

                    { // scope for eid data access
                    double vt0 = v.eid_view_t0, vt1 = v.eid_view_t1;
                    int eid_mode = v.eid_view_mode;

                    // Y축 범위: 수동 스케일 사용 (Ctrl+휠로 조절)
                    int eid_y_idx = (eid_mode==0)?0:(eid_mode==1)?1:(eid_mode==2)?2:3;
                    float a_min = v.eid_y_min[eid_y_idx];
                    float a_max = v.eid_y_max[eid_y_idx];

                    float  a_rng = a_max - a_min;
                    if(a_rng < 1e-6f) a_rng = 1e-6f;
                    double vis_samp = vt1 - vt0;
                    if(vis_samp < 1.0) vis_samp = 1.0;
                    int pixels = (int)ea_w;
                    if(pixels < 1) pixels = 1;
                    double spp = vis_samp / pixels; // samples per pixel

                    // ── Y축 그리드 + 라벨 ──────────────────────────────────
                    {
                        int n_divs = std::max(2, std::min(10, (int)(ea_h / 40.f)));
                        for(int i = 0; i <= n_divs; i++){
                            float frac = (float)i / n_divs;
                            float yy = ea_y0 + frac * ea_h;
                            float amp_val = a_max - frac * a_rng; // top=max
                            char lbl[32];
                            if(eid_mode == 2) // Phase: show π fractions
                                snprintf(lbl, sizeof(lbl), "%.2f", amp_val);
                            else if(eid_mode == 3) // Freq: Hz
                                snprintf(lbl, sizeof(lbl), "%.0f", amp_val);
                            else
                                snprintf(lbl, sizeof(lbl), "%.3f", amp_val);
                            ImVec2 tsz = ImGui::CalcTextSize(lbl);
                            dl->AddText(ImVec2(ea_x0 - tsz.x - 4, yy - tsz.y * 0.5f),
                                        IM_COL32(130,130,160,255), lbl);
                        }
                    }

                    // ── X축 그리드 + 라벨 (시간) ────────────────────────────
                    {
                        uint32_t sr = v.eid_sample_rate;
                        if(sr == 0) sr = 1;
                        double t0_sec = vt0 / sr, t1_sec = vt1 / sr;
                        double dt_sec = t1_sec - t0_sec;
                        // 적응적 단위: us / ms / s
                        const char* unit = "s"; double unit_div = 1.0;
                        if(dt_sec < 0.001){ unit = "us"; unit_div = 1e-6; }
                        else if(dt_sec < 1.0){ unit = "ms"; unit_div = 1e-3; }
                        // step: 깔끔한 눈금 간격
                        double range_u = dt_sec / unit_div;
                        double raw_step = range_u / std::max(2.0, (double)(pixels / 120));
                        double mag = pow(10.0, floor(log10(raw_step)));
                        double norm = raw_step / mag;
                        double nice_step;
                        if(norm <= 1.0) nice_step = 1.0 * mag;
                        else if(norm <= 2.0) nice_step = 2.0 * mag;
                        else if(norm <= 5.0) nice_step = 5.0 * mag;
                        else nice_step = 10.0 * mag;
                        double start_u = ceil((t0_sec / unit_div) / nice_step) * nice_step;
                        for(double tu = start_u; tu * unit_div <= t1_sec + nice_step * 0.5; tu += nice_step){
                            double t_sec = tu * unit_div;
                            double samp = t_sec * sr;
                            float xx = ea_x0 + (float)((samp - vt0) / vis_samp) * ea_w;
                            if(xx < ea_x0 || xx > ea_x1) continue;
                            char lbl[32]; snprintf(lbl, sizeof(lbl), "%.4g%s", tu, unit);
                            ImVec2 tsz = ImGui::CalcTextSize(lbl);
                            dl->AddText(ImVec2(xx - tsz.x * 0.5f, ea_y1 + 4),
                                        IM_COL32(130,130,160,255), lbl);
                        }
                    }

                    // ── 파형 렌더링 (모드별) ───────────────────────────────
                    dl->PushClipRect(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), true);
                    {
                        // 렌더할 데이터 채널 결정
                        struct EidChannel { const std::vector<float>* data; ImU32 color; };
                        EidChannel channels[2]; int n_ch = 0;
                        if(eid_mode == 0){
                            channels[0] = {&v.eid_envelope, IM_COL32(80,255,140,255)}; n_ch = 1;
                        } else if(eid_mode == 1){
                            channels[0] = {&v.eid_ch_i, IM_COL32(80,255,140,255)};  // I=녹색
                            channels[1] = {&v.eid_ch_q, IM_COL32(80,140,255,255)};  // Q=파란색
                            n_ch = 2;
                        } else if(eid_mode == 2){
                            channels[0] = {&v.eid_phase, IM_COL32(255,200,80,255)}; n_ch = 1;
                        } else {
                            channels[0] = {&v.eid_inst_freq, IM_COL32(255,120,200,255)}; n_ch = 1;
                        }

                        for(int ci = 0; ci < n_ch; ci++){
                            const auto& dat = *channels[ci].data;
                            ImU32 col = channels[ci].color;
                            int64_t total = (int64_t)dat.size();
                            if(total < 1) continue;

                            if(spp <= 1.0){
                                std::vector<ImVec2> pts;
                                int64_t s0 = std::max((int64_t)0, (int64_t)vt0);
                                int64_t s1 = std::min(total, (int64_t)ceil(vt1) + 1);
                                pts.reserve(s1 - s0);
                                for(int64_t s = s0; s < s1; s++){
                                    float xx = ea_x0 + (float)((s - vt0) / vis_samp) * ea_w;
                                    float yy = ea_y0 + (1.0f - (dat[s] - a_min) / a_rng) * ea_h;
                                    pts.push_back(ImVec2(xx, yy));
                                }
                                if(pts.size() >= 2)
                                    dl->AddPolyline(pts.data(), (int)pts.size(), col, ImDrawFlags_None, 1.5f);
                                if(spp < 0.3 && pts.size() < 2000){
                                    ImU32 dot_col = (col & 0xFF000000) | ((col & 0x00FEFEFE) >> 1) | 0x00808080;
                                    for(auto& p : pts)
                                        dl->AddCircleFilled(p, 2.5f, dot_col);
                                }
                            } else {
                                ImU32 band_col = (col & 0x00FFFFFF) | 0xC8000000;
                                float prev_lo = FLT_MAX, prev_hi = -FLT_MAX;
                                for(int px = 0; px < pixels; px++){
                                    int64_t s0 = (int64_t)(vt0 + px * spp);
                                    int64_t s1 = (int64_t)(vt0 + (px + 1) * spp);
                                    s0 = std::max((int64_t)0, std::min(s0, total - 1));
                                    s1 = std::max(s0 + 1, std::min(s1, total));
                                    float lo = dat[s0], hi = dat[s0];
                                    for(int64_t s = s0 + 1; s < s1; s++){
                                        float v2 = dat[s];
                                        if(v2 < lo) lo = v2;
                                        if(v2 > hi) hi = v2;
                                    }
                                    // 이전 픽셀과 gap이 생기면 브릿지 (원본 lo/hi 기준)
                                    float draw_lo = lo, draw_hi = hi;
                                    if(prev_lo != FLT_MAX){
                                        if(prev_hi < lo) draw_lo = prev_hi;
                                        else if(prev_lo > hi) draw_hi = prev_lo;
                                    }
                                    prev_lo = lo; prev_hi = hi; // 원본 값 저장
                                    float xx = ea_x0 + px;
                                    float yy_lo = ea_y0 + (1.0f - (draw_hi - a_min) / a_rng) * ea_h;
                                    float yy_hi = ea_y0 + (1.0f - (draw_lo - a_min) / a_rng) * ea_h;
                                    if(yy_lo > yy_hi) std::swap(yy_lo, yy_hi);
                                    if(yy_hi - yy_lo < 1.0f) yy_hi = yy_lo + 1.0f;
                                    dl->AddLine(ImVec2(xx, yy_lo), ImVec2(xx, yy_hi), band_col);
                                }
                            }
                        }

                        // ── 노이즈 레벨 라인 (Signal 모드만) ──────────────
                        if(eid_mode == 0 && v.eid_noise_level > a_min && v.eid_noise_level < a_max){
                            float ny = ea_y0 + (1.0f - (v.eid_noise_level - a_min) / a_rng) * ea_h;
                            for(float nx = ea_x0; nx < ea_x1; nx += 8.f)
                                dl->AddLine(ImVec2(nx, ny), ImVec2(std::min(nx + 4.f, ea_x1), ny),
                                            IM_COL32(255,60,60,150));
                            // 라벨
                            char nl[24]; snprintf(nl, sizeof(nl), "Noise %.4f", v.eid_noise_level);
                            dl->AddText(ImVec2(ea_x1 - ImGui::CalcTextSize(nl).x - 4, ny - 14),
                                        IM_COL32(255,60,60,180), nl);
                        }

                        // ── 태그 영역 렌더링 ─────────────────────────────
                        for(auto& tag : v.eid_tags){
                            float tx0 = ea_x0 + (float)((tag.s0 - vt0) / vis_samp) * ea_w;
                            float tx1 = ea_x0 + (float)((tag.s1 - vt0) / vis_samp) * ea_w;
                            tx0 = std::max(tx0, ea_x0); tx1 = std::min(tx1, ea_x1);
                            if(tx1 <= tx0) continue;
                            ImU32 fill = (tag.color & 0x00FFFFFF) | 0x28000000;
                            dl->AddRectFilled(ImVec2(tx0, ea_y0), ImVec2(tx1, ea_y1), fill);
                            dl->AddLine(ImVec2(tx0,ea_y0), ImVec2(tx0,ea_y1), tag.color, 1.5f);
                            dl->AddLine(ImVec2(tx1,ea_y0), ImVec2(tx1,ea_y1), tag.color, 1.5f);
                            dl->AddText(ImVec2(tx0+3, ea_y0+2), tag.color, tag.label);
                        }
                    }
                    dl->PopClipRect();

                    // ── 마우스 인터랙션: 줌/팬 ─────────────────────────────
                    ImVec2 mp = io.MousePos;
                    bool mouse_in_eid = (mp.x >= ea_x0 && mp.x < ea_x1 &&
                                         mp.y >= ea_y0 && mp.y < ea_y1);

                    // 스크롤 휠 줌 (시간축) / Ctrl+휠 = Y축 줌
                    if(mouse_in_eid && io.MouseWheel != 0.f){
                        int yi = (eid_mode==0)?0:(eid_mode==1)?1:(eid_mode==2)?2:3;
                        if(io.KeyCtrl){
                            // Y축 줌: 마우스 Y값 기준
                            float a_rng_ = v.eid_y_max[yi] - v.eid_y_min[yi];
                            if(a_rng_ < 1e-9f) a_rng_ = 1e-9f;
                            float mouse_val = v.eid_y_max[yi] - ((mp.y - ea_y0) / ea_h) * a_rng_;
                            float frac_y = (mouse_val - v.eid_y_min[yi]) / a_rng_;
                            float zf = (io.MouseWheel > 0) ? 0.8f : 1.25f;
                            float new_rng = a_rng_ * zf;
                            if(new_rng < 1e-9f) new_rng = 1e-9f;
                            v.eid_y_min[yi] = mouse_val - frac_y * new_rng;
                            v.eid_y_max[yi] = mouse_val + (1.f - frac_y) * new_rng;
                        } else {
                            // X축 줌
                            v.eid_view_stack.push_back({v.eid_view_t0, v.eid_view_t1});
                            double zoom_f = (io.MouseWheel > 0) ? 0.8 : 1.25;
                            double mouse_t = vt0 + ((mp.x - ea_x0) / ea_w) * vis_samp;
                            double new_half = vis_samp * 0.5 * zoom_f;
                            v.eid_view_t0 = mouse_t - new_half;
                            v.eid_view_t1 = mouse_t + new_half;
                            if(v.eid_view_t0 < 0){ v.eid_view_t1 -= v.eid_view_t0; v.eid_view_t0 = 0; }
                            if(v.eid_view_t1 > (double)v.eid_total_samples){
                                v.eid_view_t0 -= (v.eid_view_t1 - (double)v.eid_total_samples);
                                v.eid_view_t1 = (double)v.eid_total_samples;
                            }
                            v.eid_view_t0 = std::max(0.0, v.eid_view_t0);
                            v.eid_view_t1 = std::min((double)v.eid_total_samples, v.eid_view_t1);
                            if(v.eid_view_t1 - v.eid_view_t0 < 32.0){
                                double mid = (v.eid_view_t0 + v.eid_view_t1) * 0.5;
                                v.eid_view_t0 = mid - 16.0; v.eid_view_t1 = mid + 16.0;
                            }
                        }
                    }

                    // 좌클릭 드래그 > 영역 선택 줌
                    if(mouse_in_eid && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        v.eid_sel_active = true;
                        v.eid_sel_x0 = mp.x;
                        v.eid_sel_x1 = mp.x;
                    }
                    if(v.eid_sel_active){
                        if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                            v.eid_sel_x1 = mp.x;
                            // 선택 영역 표시 (회색 반투명)
                            float sx0 = std::min(v.eid_sel_x0, v.eid_sel_x1);
                            float sx1 = std::max(v.eid_sel_x0, v.eid_sel_x1);
                            sx0 = std::max(sx0, ea_x0); sx1 = std::min(sx1, ea_x1);
                            if(sx1 - sx0 > 1.f){
                                dl->AddRectFilled(ImVec2(sx0, ea_y0), ImVec2(sx1, ea_y1),
                                                  IM_COL32(200,200,220,50));
                                dl->AddRect(ImVec2(sx0, ea_y0), ImVec2(sx1, ea_y1),
                                            IM_COL32(200,200,220,160), 0.f, 0, 1.f);
                                // ── 선택 영역 정보 표시 ──────────────────
                                double sel_s0 = vt0 + ((sx0 - ea_x0) / ea_w) * vis_samp;
                                double sel_s1 = vt0 + ((sx1 - ea_x0) / ea_w) * vis_samp;
                                int64_t is0 = std::max((int64_t)0, (int64_t)sel_s0);
                                int64_t is1 = std::min((int64_t)v.eid_total_samples, (int64_t)sel_s1);
                                int64_t n_sel = is1 - is0;
                                uint32_t sr = v.eid_sample_rate > 0 ? v.eid_sample_rate : 1;
                                double sel_ms = (double)n_sel / sr * 1000.0;
                                char sel_info[96];
                                snprintf(sel_info, sizeof(sel_info), "Sel: %lld - %lld | %lld samp | %.3fms",
                                    (long long)is0, (long long)is1, (long long)n_sel, sel_ms);
                                ImVec2 si_sz = ImGui::CalcTextSize(sel_info);
                                float si_x = sx0 + (sx1 - sx0 - si_sz.x) * 0.5f;
                                si_x = std::max(si_x, sx0 + 2.f);
                                dl->AddRectFilled(ImVec2(si_x-2,ea_y0+2), ImVec2(si_x+si_sz.x+2,ea_y0+si_sz.y+4),
                                                  IM_COL32(0,0,0,180), 3.f);
                                dl->AddText(ImVec2(si_x, ea_y0+3), IM_COL32(200,220,255,255), sel_info);
                            }
                        } else {
                            // 릴리즈: 선택 범위로 줌
                            v.eid_sel_active = false;
                            float sx0 = std::min(v.eid_sel_x0, v.eid_sel_x1);
                            float sx1 = std::max(v.eid_sel_x0, v.eid_sel_x1);
                            if(sx1 - sx0 > 5.f){ // 최소 5px 드래그
                                // 현재 뷰를 스택에 push
                                v.eid_view_stack.push_back({v.eid_view_t0, v.eid_view_t1});
                                // 화면 좌표 > 샘플 인덱스
                                double t0_new = vt0 + ((sx0 - ea_x0) / ea_w) * vis_samp;
                                double t1_new = vt0 + ((sx1 - ea_x0) / ea_w) * vis_samp;
                                v.eid_view_t0 = std::max(0.0, t0_new);
                                v.eid_view_t1 = std::min((double)v.eid_total_samples, t1_new);
                                if(v.eid_view_t1 - v.eid_view_t0 < 32.0){
                                    double mid = (v.eid_view_t0 + v.eid_view_t1) * 0.5;
                                    v.eid_view_t0 = mid - 16.0; v.eid_view_t1 = mid + 16.0;
                                }
                            }
                        }
                    }

                    // 우클릭: 드래그 > 태그 생성, 단순 클릭 > 뒤로가기
                    if(mouse_in_eid && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                        v.eid_tag_dragging = true;
                        v.eid_tag_drag_x0 = mp.x;
                        v.eid_tag_drag_x1 = mp.x;
                    }
                    if(v.eid_tag_dragging && ImGui::IsMouseDown(ImGuiMouseButton_Right)){
                        v.eid_tag_drag_x1 = mp.x;
                        float sx0 = std::max(ea_x0, std::min(v.eid_tag_drag_x0, v.eid_tag_drag_x1));
                        float sx1 = std::min(ea_x1, std::max(v.eid_tag_drag_x0, v.eid_tag_drag_x1));
                        if(sx1 - sx0 > 1.f){
                            dl->AddRectFilled(ImVec2(sx0, ea_y0), ImVec2(sx1, ea_y1),
                                              IM_COL32(255,180,60,40));
                            dl->AddRect(ImVec2(sx0, ea_y0), ImVec2(sx1, ea_y1),
                                        IM_COL32(255,180,60,180), 0.f, 0, 1.f);
                            // 선택 정보
                            double sel_s0 = vt0 + ((sx0 - ea_x0) / ea_w) * vis_samp;
                            double sel_s1 = vt0 + ((sx1 - ea_x0) / ea_w) * vis_samp;
                            int64_t n_sel = (int64_t)sel_s1 - (int64_t)sel_s0;
                            uint32_t sr = v.eid_sample_rate > 0 ? v.eid_sample_rate : 1;
                            double sel_ms = (double)n_sel / sr * 1000.0;
                            char ti[64]; snprintf(ti, sizeof(ti), "Tag: %lld samp | %.3fms", (long long)n_sel, sel_ms);
                            ImVec2 tsz2 = ImGui::CalcTextSize(ti);
                            float tx = sx0 + (sx1 - sx0 - tsz2.x) * 0.5f;
                            tx = std::max(tx, sx0 + 2.f);
                            dl->AddRectFilled(ImVec2(tx-2,ea_y0+2), ImVec2(tx+tsz2.x+2,ea_y0+tsz2.y+4),
                                              IM_COL32(0,0,0,180), 3.f);
                            dl->AddText(ImVec2(tx, ea_y0+3), IM_COL32(255,200,100,255), ti);
                        }
                    }
                    if(v.eid_tag_dragging && ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
                        v.eid_tag_dragging = false;
                        float dx = fabsf(v.eid_tag_drag_x1 - v.eid_tag_drag_x0);
                        if(dx > 5.f){
                            // 드래그 > 태그 생성
                            v.eid_push_undo();
                            float sx0 = std::max(ea_x0, std::min(v.eid_tag_drag_x0, v.eid_tag_drag_x1));
                            float sx1 = std::min(ea_x1, std::max(v.eid_tag_drag_x0, v.eid_tag_drag_x1));
                            double t0_tag = vt0 + ((sx0 - ea_x0) / ea_w) * vis_samp;
                            double t1_tag = vt0 + ((sx1 - ea_x0) / ea_w) * vis_samp;
                            static const ImU32 tag_colors[] = {
                                IM_COL32(255,180,60,200), IM_COL32(100,200,255,200),
                                IM_COL32(255,100,255,200), IM_COL32(100,255,180,200),
                                IM_COL32(255,255,100,200), IM_COL32(180,140,255,200),
                            };
                            FFTViewer::EidTag tag;
                            tag.s0 = std::max(0.0, t0_tag);
                            tag.s1 = std::min((double)v.eid_total_samples, t1_tag);
                            tag.color = tag_colors[v.eid_tags.size() % 6];
                            snprintf(tag.label, sizeof(tag.label), "Tag %d", (int)v.eid_tags.size() + 1);
                            v.eid_tags.push_back(tag);
                        v.eid_auto_analyze_tag(v.eid_tags.back());
                        } else {
                            // 단순 클릭: 아무 동작 없음 (Ctrl+Z로 undo)
                        }
                    }

                    // Home 키: 뷰 리셋 + 스택 클리어
                    if(mouse_in_eid && ImGui::IsKeyPressed(ImGuiKey_Home, false)){
                        v.eid_view_stack.clear();
                        v.eid_view_t0 = 0.0;
                        v.eid_view_t1 = (double)v.eid_total_samples;
                    }

                    // ── 뷰 모드 전환 (1~9 키, sa_btns 디스플레이 순서) ─ EID가 가장 상단일 때만
                    // 1=Spectrogram 2=Amp 3=Freq 4=Phase 5=I/Q 6=Const 7=Audio 8=Power 9=Bits
                    if(mouse_in_eid && top_ov() == 1){
                        if(ImGui::IsKeyPressed(ImGuiKey_1, false)) v.eid_view_mode = 0;
                        if(ImGui::IsKeyPressed(ImGuiKey_2, false)) v.eid_view_mode = 1;
                        if(ImGui::IsKeyPressed(ImGuiKey_3, false)) v.eid_view_mode = 2;
                        if(ImGui::IsKeyPressed(ImGuiKey_4, false)) v.eid_view_mode = 3;
                        if(ImGui::IsKeyPressed(ImGuiKey_5, false)) v.eid_view_mode = 4;
                        if(ImGui::IsKeyPressed(ImGuiKey_6, false)) v.eid_view_mode = 5;
                        if(ImGui::IsKeyPressed(ImGuiKey_7, false)) v.eid_view_mode = 8; // Audio
                        if(ImGui::IsKeyPressed(ImGuiKey_8, false)) v.eid_view_mode = 6; // Power
                        if(ImGui::IsKeyPressed(ImGuiKey_9, false)) v.eid_view_mode = 7; // Bits
                        // 좌우 방향키: 현재 화면 폭만큼 팬
                        if(!io.WantTextInput){
                            double span = vt1 - vt0;
                            double total = (double)v.eid_total_samples;
                            if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow, false)){
                                double t0 = vt0 - span;
                                if(t0 < 0) t0 = 0;
                                v.eid_view_t0 = t0;
                                v.eid_view_t1 = t0 + span;
                            }
                            if(ImGui::IsKeyPressed(ImGuiKey_RightArrow, false)){
                                double t1 = vt1 + span;
                                if(t1 > total) t1 = total;
                                v.eid_view_t0 = t1 - span;
                                v.eid_view_t1 = t1;
                            }
                        }
                        // Delete 키: 마우스 위치의 태그 삭제
                        if(ImGui::IsKeyPressed(ImGuiKey_Delete, false)){
                            double mouse_s = vt0 + ((mp.x - ea_x0) / ea_w) * vis_samp;
                            for(auto it = v.eid_tags.begin(); it != v.eid_tags.end(); ++it){
                                if(mouse_s >= it->s0 && mouse_s <= it->s1){
                                    v.eid_push_undo();
                                    v.eid_tags.erase(it); break;
                                }
                            }
                        }
                    }

                    // ── 커서 오버레이 ───────────────────────────────────────
                    if(mouse_in_eid){
                        // 십자선
                        dl->AddLine(ImVec2(mp.x, ea_y0), ImVec2(mp.x, ea_y1),
                                    IM_COL32(255,255,255,60));
                        dl->AddLine(ImVec2(ea_x0, mp.y), ImVec2(ea_x1, mp.y),
                                    IM_COL32(255,255,255,60));
                        // 정보 박스
                        double mouse_samp = vt0 + ((mp.x - ea_x0) / ea_w) * vis_samp;
                        float  mouse_val  = a_max - ((mp.y - ea_y0) / ea_h) * a_rng;
                        uint32_t sr = v.eid_sample_rate > 0 ? v.eid_sample_rate : 1;
                        double mouse_sec  = mouse_samp / sr;
                        char info1[48], info2[48], info3[48];
                        if(mouse_sec < 0.001)
                            snprintf(info1, sizeof(info1), "Time : %.3f us", mouse_sec * 1e6);
                        else if(mouse_sec < 1.0)
                            snprintf(info1, sizeof(info1), "Time : %.4f ms", mouse_sec * 1e3);
                        else
                            snprintf(info1, sizeof(info1), "Time : %.6f s", mouse_sec);
                        // 값 라벨: 모드별
                        if(eid_mode == 0)
                            snprintf(info2, sizeof(info2), "Amp  : %.5f", mouse_val);
                        else if(eid_mode == 1)
                            snprintf(info2, sizeof(info2), "Val  : %.5f", mouse_val);
                        else if(eid_mode == 2)
                            snprintf(info2, sizeof(info2), "Phase: %.4f rad", mouse_val);
                        else
                            snprintf(info2, sizeof(info2), "Freq : %.1f Hz", mouse_val);
                        snprintf(info3, sizeof(info3), "Samp : %lld", (long long)(int64_t)mouse_samp);
                        float fw = std::max({ImGui::CalcTextSize(info1).x,
                                             ImGui::CalcTextSize(info2).x,
                                             ImGui::CalcTextSize(info3).x});
                        float fh = ImGui::GetFontSize() * 3.8f;
                        float ox = ea_x1 - fw - 14.f, oy = ea_y0 + 8.f;
                        dl->AddRectFilled(ImVec2(ox-4,oy-2), ImVec2(ox+fw+4,oy+fh),
                                         IM_COL32(0,0,0,180), 4.f);
                        float lh = ImGui::GetFontSize() + 2.f;
                        dl->AddText(ImVec2(ox,oy),       IM_COL32(255,180,220,255), info1);
                        dl->AddText(ImVec2(ox,oy+lh),    IM_COL32(255,180,220,255), info2);
                        dl->AddText(ImVec2(ox,oy+lh*2),  IM_COL32(255,180,220,255), info3);
                    }
                    } // end scope for eid data access
                } else {
                    eid_skip:
                    const char* msg = "Load a WAV file (SA or right-click)";
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
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8,8));
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0,0,0,0));
                ImGui::Begin("##board_panel", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                    ImGuiWindowFlags_NoDecoration);

                ImGui::PushStyleColor(ImGuiCol_Tab,       ImVec4(0.12f,0.12f,0.16f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabHovered,ImVec4(0.20f,0.30f,0.45f,1.f));
                ImGui::PushStyleColor(ImGuiCol_TabActive, ImVec4(0.15f,0.40f,0.65f,1.f));
                if(ImGui::BeginTabBar("##board_tabs")){
                    if(ImGui::BeginTabItem("BOARD")) ImGui::EndTabItem();
                    ImGui::EndTabBar();
                }
                ImGui::PopStyleColor(3);

                ImGui::BeginChild("##board_scroll", ImVec2(0,0), false,
                    ImGuiWindowFlags_HorizontalScrollbar);

                // ── 0. System Status ──────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("System Status##board")){
                    ImGui::Indent(6.f);
                    draw_system_status(v, "board");
                    ImGui::Unindent(6.f);
                }
                ImGui::Spacing();

                // ── 1. Operators ─────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Operators")){
                    ImGui::Indent(6.f);
                    if(v.net_srv){
                        // HOST 모드: 자기 자신 먼저, 그 뒤 JOINs
                        ImGui::TextColored(ImVec4(0.4f,0.85f,1.f,1.f),
                            "[HOST] %s  [T%d]", login_get_id(), login_get_tier());
                        std::lock_guard<std::mutex> lk(s_relay_op_mtx);
                        int join_count = 0;
                        for(int i = 0; i < (int)s_relay_op_list.count; i++){
                            auto& op = s_relay_op_list.ops[i];
                            if(op.index == 0) continue;
                            ImGui::TextColored(ImVec4(0.7f,0.92f,0.7f,1.f),
                                "[JOIN] %s  [T%d]", op.name, op.tier);
                            join_count++;
                        }
                        if(s_relay_op_list.count == 0){
                            auto ops = v.net_srv->get_operators();
                            for(auto& op : ops){
                                ImGui::TextColored(ImVec4(0.7f,0.92f,0.7f,1.f),
                                    "[JOIN] %s  [T%d]", op.name, op.tier);
                                join_count++;
                            }
                        }
                        if(join_count == 0) ImGui::TextDisabled("  (no operators)");
                    } else if(v.net_cli){
                        // JOIN 모드: 서버 op_list 그대로 (HOST 먼저, JOINs 이후)
                        std::lock_guard<std::mutex> lk(v.net_cli->op_mtx);
                        for(int i=0;i<(int)v.net_cli->op_list.count;i++){
                            auto& op = v.net_cli->op_list.ops[i];
                            bool is_host = (op.index==0);
                            ImVec4 oc = is_host ? ImVec4(0.4f,0.85f,1.f,1.f)
                                                : ImVec4(0.7f,0.92f,0.7f,1.f);
                            ImGui::TextColored(oc, "%s %s  [T%d]",
                                is_host?"[HOST]":"[JOIN]", op.name, op.tier);
                        }
                    } else {
                        // LOCAL 모드
                        ImGui::TextColored(ImVec4(0.4f,0.85f,1.f,1.f),
                            "[LOCAL] %s", login_get_id());
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
                ImGui::Spacing();

                // ── 5. Report Feed ────────────────────────────────────────
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Report Feed")){
                    ImGui::Indent(6.f);
                    // report/ 폴더의 파일 수
                    static std::vector<std::string> board_report_files;
                    static float board_rpt_timer = 0;
                    board_rpt_timer -= io.DeltaTime;
                    if(board_rpt_timer <= 0){
                        board_rpt_timer = 3.0f;
                        board_report_files.clear();
                        auto scan_rpt = [](const std::string& dir, std::vector<std::string>& out){
                            DIR* d = opendir(dir.c_str()); if(!d) return;
                            struct dirent* e;
                            while((e = readdir(d))){
                                if(e->d_name[0]=='.') continue;
                                std::string n(e->d_name);
                                if(n.size()>4 && n.substr(n.size()-4)==".wav")
                                    out.push_back(n);
                            }
                            closedir(d);
                        };
                        scan_rpt(BEWEPaths::report_iq_dir(), board_report_files);
                        scan_rpt(BEWEPaths::report_audio_dir(), board_report_files);
                        std::sort(board_report_files.rbegin(), board_report_files.rend());
                    }
                    if(board_report_files.empty()){
                        ImGui::TextDisabled("  (no reports)");
                    } else {
                        int show = std::min(10, (int)board_report_files.size());
                        for(int ri2=0; ri2<show; ri2++){
                            auto& fn2 = board_report_files[ri2];
                            // .info 요약 읽기
                            std::string ip2 = BEWEPaths::report_iq_dir()+"/"+fn2+".info";
                            if(access(ip2.c_str(), F_OK)!=0)
                                ip2 = BEWEPaths::report_audio_dir()+"/"+fn2+".info";
                            char info_line[128] = "";
                            FILE* fi2 = fopen(ip2.c_str(), "r");
                            if(fi2){
                                char line[256];
                                while(fgets(line, sizeof(line), fi2)){
                                    char k[64]={}, val[128]={};
                                    if(sscanf(line, "%63[^:]: %127[^\n]", k, val)>=1){
                                        if(strcmp(k,"Operator")==0 && val[0])
                                            snprintf(info_line, sizeof(info_line), "by %s", val);
                                    }
                                }
                                fclose(fi2);
                            }
                            ImGui::TextColored(ImVec4(1.f,0.8f,0.4f,1.f), "%s", fn2.c_str());
                            if(info_line[0]){
                                ImGui::SameLine(0,6);
                                ImGui::TextDisabled("%s", info_line);
                            }
                        }
                    }
                    ImGui::Unindent(6.f);
                }
                ImGui::Spacing();

                // ── 6. Event Log ──────────────────────────────────────────
                ImGui::SetNextItemOpen(false, ImGuiCond_Once);
                if(ImGui::CollapsingHeader("Event Log")){
                    ImGui::Indent(6.f);
                    // 최근 bewe_log 메시지 표시 (v.log_buf[0])
                    if(!v.log_buf[0].empty()){
                        int cnt = 0;
                        for(int li=(int)v.log_buf[0].size()-1; li>=0 && cnt<30; li--,cnt++){
                            ImGui::TextDisabled("%s", v.log_buf[0][li].msg);
                        }
                    } else {
                        ImGui::TextDisabled("  (no events)");
                    }
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
                    std::thread([&v](){
                        char tmp[16] = "";
                        int  tmp_deg = 0;
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
                                    if(fscanf(ft, "%d", &milli) == 1){
                                        snprintf(tmp, sizeof(tmp), "%.0f\xC2\xB0""C",
                                                 milli / 1000.f);
                                        tmp_deg = milli / 1000;
                                    }
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
                                        if(fscanf(fv, "%d", &milli) == 1){
                                            snprintf(tmp, sizeof(tmp), "%.0f\xC2\xB0""C",
                                                     milli / 1000.f);
                                            tmp_deg = milli / 1000;
                                        }
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
                                if(fscanf(fv, "%d", &milli) == 1 && milli > 0){
                                    snprintf(tmp, sizeof(tmp), "%.0f\xC2\xB0""C",
                                             milli / 1000.f);
                                    tmp_deg = milli / 1000;
                                }
                                fclose(fv);
                            }
                        }
                        { std::lock_guard<std::mutex> lk(cpu_temp_mtx);
                          strncpy(cpu_temp_str, tmp, sizeof(cpu_temp_str)-1); }
                        v.sysmon_cpu_temp_c.store(tmp_deg);
                        cpu_fetching.store(false);
                    }).detach();
                }

                // SDR 온도
                // - LOCAL/HOST + BladeRF: libbladeRF 직접 쿼리 (3초마다)
                // - LOCAL/HOST + RTL-SDR: 온도 API 없음 > "00°C"
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

                // 중앙 하단: 시간만 표시 (CPU/SDR 온도는 STATUS 패널과 중복이라 생략)
                ImVec2 csz = ImGui::CalcTextSize(clock_str);
                dl->AddText(ImVec2((disp_w - csz.x) / 2.f, ty_b),
                            IM_COL32(200,200,200,255), clock_str);
            }

            // ── 좌측: 타임머신 오프셋 (TM 모드 + 오프셋 있을 때만 표시) ─
            if(v.tm_active.load() && v.tm_offset > 0.0f){
                char tm_txt[48];
                snprintf(tm_txt,sizeof(tm_txt),"-%.1f sec",v.tm_offset);
                dl->AddText(ImVec2(8,ty_b),IM_COL32(255,200,50,255),tm_txt);
            }

            // ── 우측: 상태 인디케이터 (오른쪽>왼쪽) ─────────────────────
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
            // JOIN: fft_seq가 최근 3초 이내에 변한 경우 > 실제 데이터 수신 중
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
                // host_state 2 = HOST 의도적 정지 > 노란, 연결 끊김/stall > 빨간, 수신 중 > 초록
                // JOIN 로컬 수직바(fft_panel_on=false) > FFT/WF 빨간
                // JOIN 로컬 수평바(wf_area_visible=false) > WF만 빨간
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

            // 오른쪽>왼쪽: TM IQ AUD WF FFT LINK SDR
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

            // WF (클릭 가능, 3색>초록/빨간)
            if(click_ind(rx,"WF", wf_led)){
                v.spectrum_pause.store(!v.spectrum_pause.load());
            }

            // FFT (클릭 가능, 3색>초록/빨간)
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

            // Signal Analysis > 독립 오버레이
            if(ImGui::Selectable("  Signal Analysis")){
                v.sa_temp_path = file_ctx.filepath;
                v.eid_panel_open = true;
                v.eid_view_mode = 1; // 기본 Amp
                // 기존 audio 재생 정지 + cursor 리셋
                v.audio_play_stop();
                v.eid_audio_cursor_sample = 0;
                // 시간 도메인 데이터 로드
                v.eid_cleanup();
                v.eid_start(file_ctx.filepath);
                // 스펙트로그램 데이터 로드 (새 파일 > 뷰 리셋)
                v.sa_cleanup();
                v.sa_mode = false;
                v.sa_view_x0=0.f; v.sa_view_x1=1.f;
                v.sa_view_y0=0.f; v.sa_view_y1=1.f;
                v.sa_start(file_ctx.filepath);
                file_ctx.open = false;
                // EID 분석 중 Del 키로 원본 파일이 삭제되는 것을 방지
                file_ctx.selected = false;
                file_ctx.filepath = "";
                file_ctx.filename = "";
            }

            // ── Rename ─────────────────────────────────────────────────
            if(ImGui::Selectable("  Rename")){
                // 디렉토리와 파일명 분리
                size_t slash = file_ctx.filepath.rfind('/');
                rename_modal.dir = (slash != std::string::npos) ? file_ctx.filepath.substr(0,slash) : ".";
                rename_modal.old_name = file_ctx.filename;
                strncpy(rename_modal.new_name, file_ctx.filename.c_str(), 255);
                rename_modal.open = true;
                rename_modal.focus_set = false;
                file_ctx.open = false;
            }

            // ── Info / Add Info ────────────────────────────────────────
            {
                std::string ip = file_ctx.filepath + ".info";
                bool has_info = (access(ip.c_str(), F_OK) == 0);
                if(ImGui::Selectable(has_info ? "  Info" : "  Add Info")){
                    info_modal.open = true;
                    info_modal.save_and_write = false;
                    info_modal.save_file_fn = nullptr;
                    info_modal.src_filepath.clear();
                    info_modal.filepath = file_ctx.filepath;
                    info_modal.info_path = ip;
                    info_modal.exists = has_info;
                    info_modal.utc_off_override = v.utc_offset_hours();
                    if(has_info){
                        memset(info_modal.fields, 0, sizeof(info_modal.fields));
                        info_modal.load();
                    }
                    else info_modal.autofill(file_ctx.filename);
                    info_modal.init_ext();
                    file_ctx.open = false;
                }
            }

            ImGui::Separator();

            // ── Report ────────────────────────────────────────────────
            if(ImGui::Selectable("  Report")){
                // 파일 복사 없음 — 제목 + .info만 Central에 전송
                {
                    char info_sum[256] = {};
                    std::string ip2 = file_ctx.filepath + ".info";
                    FILE* fis = fopen(ip2.c_str(), "r");
                    if(fis){
                        char line[256]; int pos2=0;
                        while(fgets(line,sizeof(line),fis) && pos2<250){
                            int l = (int)strlen(line);
                            if(pos2+l < 255){ memcpy(info_sum+pos2, line, l); pos2+=l; }
                        }
                        fclose(fis);
                    }
                    if(v.net_cli){
                        v.net_cli->cmd_report_add(file_ctx.filename.c_str(), info_sum);
                    } else if(v.net_srv && v.net_srv->cb.on_relay_broadcast){
                        // HOST: relay를 통해 Central에 전송
                        PktReportAdd ra{};
                        strncpy(ra.filename, file_ctx.filename.c_str(), 127);
                        strncpy(ra.reporter, login_get_id(), 31);
                        strncpy(ra.info_summary, info_sum, 255);
                        auto pkt = make_packet(PacketType::REPORT_ADD, &ra, sizeof(ra));
                        v.net_srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
                    }
                }
                file_ctx.open = false;
            }

            // ── Save DB (Central Server) ──────────────────────────────
            if(ImGui::Selectable("  Save DB")){
                // DB 저장: Central server의 ~/BE_WE/DataBase/ 에 저장
                // JOIN → HOST(relay) → Central, HOST → Central(relay)
                std::string op_name = login_get_id();
                std::string fp_cap = file_ctx.filepath;
                std::string fn_cap = file_ctx.filename;
                std::string op_cap = op_name;
                // 파일 크기 미리 측정 (진행률 표시용)
                uint64_t file_total = 0;
                {
                    FILE* fmeasure = fopen(fp_cap.c_str(),"rb");
                    if(fmeasure){
                        fseek(fmeasure,0,SEEK_END);
                        file_total = (uint64_t)ftell(fmeasure);
                        fclose(fmeasure);
                    }
                }
                // file_xfers에 등록 (UI에서 진행률 표시)
                {
                    std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                    FFTViewer::FileXfer xf{};
                    xf.filename = fn_cap;
                    xf.total_bytes = file_total;
                    xf.done_bytes = 0;
                    v.file_xfers.push_back(xf);
                }
                if(v.net_cli){
                    // JOIN: cmd_db_save (HOST→Central relay로 전달됨)
                    NetClient* cli_cap = v.net_cli;
                    auto* viewer = &v;
                    std::thread([cli_cap, fp_cap, op_cap, fn_cap, viewer](){
                        cli_cap->cmd_db_save(fp_cap.c_str(), op_cap.c_str());
                        // 완료 표시 (cmd_db_save 동기 — 끝나면 done)
                        std::lock_guard<std::mutex> lk(viewer->file_xfer_mtx);
                        for(auto& x : viewer->file_xfers)
                            if(x.filename == fn_cap && !x.finished){
                                x.finished = true;
                                x.done_bytes = x.total_bytes;
                                break;
                            }
                    }).detach();
                } else if(v.net_srv && v.net_srv->cb.on_relay_broadcast){
                    // HOST: DB_SAVE를 Central relay로 직접 전송
                    NetServer* srv_cap = v.net_srv;
                    auto* viewer = &v;
                    std::thread([srv_cap, fp_cap, fn_cap, op_cap, viewer](){
                        FILE* fp=fopen(fp_cap.c_str(),"rb");
                        if(!fp) return;
                        fseek(fp,0,SEEK_END); uint64_t fsz=(uint64_t)ftell(fp); fseek(fp,0,SEEK_SET);
                        char info_data[512]={};
                        { std::string ip=fp_cap+".info"; FILE* fi=fopen(ip.c_str(),"r");
                          if(fi){fread(info_data,1,511,fi);fclose(fi);} }
                        PktDbSaveMeta meta{};
                        strncpy(meta.filename,fn_cap.c_str(),127);
                        meta.total_bytes=fsz; meta.transfer_id=1;
                        strncpy(meta.operator_name,op_cap.c_str(),31);
                        strncpy(meta.info_data,info_data,511);
                        { auto pkt=make_packet(PacketType::DB_SAVE_META,&meta,sizeof(meta));
                          srv_cap->cb.on_relay_broadcast(pkt.data(),pkt.size(),true); }
                        const size_t CHUNK=64*1024;
                        std::vector<uint8_t> buf(sizeof(PktDbSaveData)+CHUNK);
                        uint64_t sent = 0;
                        while(true){
                            size_t n=fread(buf.data()+sizeof(PktDbSaveData),1,CHUNK,fp);
                            if(n==0) break;
                            auto* d=reinterpret_cast<PktDbSaveData*>(buf.data());
                            d->transfer_id=1; d->is_last=feof(fp)?1:0; d->chunk_bytes=(uint32_t)n;
                            auto pkt=make_packet(PacketType::DB_SAVE_DATA,buf.data(),(uint32_t)(sizeof(PktDbSaveData)+n));
                            srv_cap->cb.on_relay_broadcast(pkt.data(),pkt.size(),true);
                            sent += n;
                            // 진행률 갱신
                            std::lock_guard<std::mutex> lk(viewer->file_xfer_mtx);
                            for(auto& x : viewer->file_xfers)
                                if(x.filename == fn_cap && !x.finished){
                                    x.done_bytes = sent;
                                    break;
                                }
                        }
                        fclose(fp);
                        // 완료 표시
                        std::lock_guard<std::mutex> lk(viewer->file_xfer_mtx);
                        for(auto& x : viewer->file_xfers)
                            if(x.filename == fn_cap && !x.finished){
                                x.finished = true;
                                x.done_bytes = x.total_bytes;
                                break;
                            }
                    }).detach();
                } else {
                    // LOCAL: 로컬 저장 fallback (flat — operator는 .info 의 Operator: 필드로 보존)
                    mkdir(BEWEPaths::database_dir().c_str(), 0755);
                    std::string dst = BEWEPaths::database_dir() + "/" + fn_cap;
                    FILE* fin=fopen(fp_cap.c_str(),"rb");
                    FILE* fout=fopen(dst.c_str(),"wb");
                    if(fin&&fout){ char buf[65536]; size_t n;
                        while((n=fread(buf,1,sizeof(buf),fin))>0) fwrite(buf,1,n,fout); }
                    if(fin) fclose(fin); if(fout) fclose(fout);
                    std::string info_src = fp_cap + ".info";
                    if(access(info_src.c_str(), F_OK)==0){
                        std::string info_dst = dst + ".info";
                        FILE* fi2=fopen(info_src.c_str(),"rb");
                        FILE* fo2=fopen(info_dst.c_str(),"wb");
                        if(fi2&&fo2){ char b[4096]; size_t n; while((n=fread(b,1,sizeof(b),fi2))>0) fwrite(b,1,n,fo2); }
                        if(fi2) fclose(fi2); if(fo2) fclose(fo2);
                    }
                }
                file_ctx.open = false;
            }

            ImGui::Separator();

            // Delete > 파일 삭제 (Public은 소유자만 가능)
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
                    remove((file_ctx.filepath + ".info").c_str()); // 동반 info 삭제
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

            // 창 밖 클릭 또는 ESC > 닫기
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

        // ── Info 모달 ─────────────────────────────────────────────────
        if(info_modal.open){
            ImVec2 center(disp_w * 0.5f, disp_h * 0.5f);
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::SetNextWindowSize(ImVec2(500, 560), ImGuiCond_Appearing);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f,0.08f,0.12f,0.98f));
            const char* modal_title = info_modal.save_and_write
                ? "Save File##info_modal" : "File Info##info_modal";
            ImGui::Begin(modal_title, &info_modal.open,
                ImGuiWindowFlags_NoCollapse);

            // 엔터 감지: 이 모달 내 아무 InputText에서 Enter 시 Save 처리
            bool submit = false;

            for(int i=0; i<info_modal.N_FIELDS; i++){
                ImGui::Text("%-18s", info_modal.names[i]);
                ImGui::SameLine(140);
                char id[32]; snprintf(id, sizeof(id), "##inf_%d", i);
                // 파일명(0)은 옆에 확장자 라벨 표시
                if(i == 0 && info_modal.ext_buf[0]){
                    float ext_w = ImGui::CalcTextSize(info_modal.ext_buf).x
                                + ImGui::GetStyle().ItemSpacing.x + 6.f;
                    ImGui::SetNextItemWidth(-ext_w);
                    bool e = ImGui::InputText(id, info_modal.fields[i], 256,
                        ImGuiInputTextFlags_EnterReturnsTrue);
                    ImGui::SameLine();
                    ImGui::TextDisabled("%s", info_modal.ext_buf);
                    if(e) submit = true;
                } else if(i == 13){
                    // Notes: 멀티라인 (멀티라인은 엔터가 개행)
                    ImGui::SetNextItemWidth(-1);
                    ImGui::InputTextMultiline(id, info_modal.fields[i], 256,
                        ImVec2(-1, ImGui::GetTextLineHeight()*3));
                } else {
                    ImGui::SetNextItemWidth(-1);
                    bool e = ImGui::InputText(id, info_modal.fields[i], 256,
                        ImGuiInputTextFlags_EnterReturnsTrue);
                    if(e) submit = true;
                    // Up Time(2) / Down Time(3): 포커스 잃을 때 자동 포맷 변환
                    if((i == 2 || i == 3) && ImGui::IsItemDeactivatedAfterEdit())
                        fmt_time_field(info_modal.fields[i]);
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            // ── 버튼 중앙정렬 (Save/Add + Cancel) ────────────────────
            const char* primary_label = info_modal.save_and_write ? "Add" : "Save";
            float bw = 80.f*2 + ImGui::GetStyle().ItemSpacing.x;
            ImGui::SetCursorPosX((ImGui::GetWindowWidth() - bw) * 0.5f);
            bool pressed = ImGui::Button(primary_label, ImVec2(80,0));
            ImGui::SameLine();
            bool cancelled = ImGui::Button("Cancel", ImVec2(80,0));

            if(pressed || submit){
                // 0) Up/Down Time 필드 마지막 자동 포맷 (포커스 놓치지 않은 경우 대비)
                fmt_time_field(info_modal.fields[2]);
                fmt_time_field(info_modal.fields[3]);

                // 1) Rename 수행 (fields[0] stem이 바뀌었으면)
                std::string new_path = info_modal.filepath;
                {
                    size_t slash = info_modal.filepath.rfind('/');
                    std::string dir = (slash==std::string::npos) ? "."
                                     : info_modal.filepath.substr(0, slash);
                    std::string old_base = (slash==std::string::npos)
                                     ? info_modal.filepath
                                     : info_modal.filepath.substr(slash+1);
                    size_t dot = old_base.rfind('.');
                    std::string old_stem = (dot==std::string::npos) ? old_base
                                           : old_base.substr(0, dot);
                    std::string new_stem_s = info_modal.fields[0];
                    if(new_stem_s.empty()) new_stem_s = old_stem;
                    if(new_stem_s != old_stem){
                        auto p_exists = [](const std::string& p){
                            struct stat st; return ::stat(p.c_str(), &st) == 0;
                        };
                        std::string cand = dir + "/" + new_stem_s + info_modal.ext_buf;
                        for(int n=2; p_exists(cand); n++){
                            cand = dir + "/" + new_stem_s + "_" + std::to_string(n)
                                 + info_modal.ext_buf;
                            if(n>9999) break;
                        }
                        new_path = cand;
                    }
                }

                // 2) EID Save File 플로우
                if(info_modal.save_and_write && info_modal.save_file_fn){
                    std::string written = info_modal.save_file_fn(new_path);
                    if(!written.empty()) new_path = written;
                    info_modal.filepath = new_path;
                    info_modal.info_path = new_path + ".info";
                    // 실제 저장 후 File Name 필드도 최종 stem으로 맞춤
                    {
                        size_t s = new_path.rfind('/');
                        std::string bn = (s==std::string::npos) ? new_path : new_path.substr(s+1);
                        size_t d = bn.rfind('.');
                        std::string fs = (d==std::string::npos) ? bn : bn.substr(0, d);
                        strncpy(info_modal.fields[0], fs.c_str(), 255);
                    }
                } else if(new_path != info_modal.filepath){
                    if(rename(info_modal.filepath.c_str(), new_path.c_str()) == 0){
                        remove(info_modal.info_path.c_str());
                        info_modal.filepath = new_path;
                        info_modal.info_path = new_path + ".info";
                    }
                }

                // 3) .info 저장
                info_modal.save();
                info_modal.open = false;
                g_arch_cache_dirty = true;
            }
            if(cancelled){
                info_modal.open = false;
            }
            ImGui::End();
            ImGui::PopStyleColor();
        }

        // ── Rename 모달 ──────────────────────────────────────────────────
        if(rename_modal.open){
            ImGui::SetNextWindowSize(ImVec2(420.f, 0.f));
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x*0.5f-210.f, io.DisplaySize.y*0.35f), ImGuiCond_Appearing);
            ImGui::SetNextWindowBgAlpha(0.97f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f,0.10f,0.16f,1.f));
            ImGui::Begin("Rename##rename_modal", &rename_modal.open,
                ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoCollapse);
            if(!rename_modal.focus_set){ ImGui::SetKeyboardFocusHere(); rename_modal.focus_set=true; }
            ImGui::SetNextItemWidth(-FLT_MIN);
            bool enter = ImGui::InputText("##rename_input", rename_modal.new_name, 256,
                                          ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::Spacing();
            std::string nn(rename_modal.new_name);
            bool valid = !nn.empty() && nn != rename_modal.old_name && nn.find('/')==std::string::npos;
            float bw = 80.f*2 + ImGui::GetStyle().ItemSpacing.x;
            ImGui::SetCursorPosX((ImGui::GetWindowWidth() - bw) * 0.5f);
            if(!valid) ImGui::BeginDisabled();
            if(ImGui::Button("OK", ImVec2(80,0)) || (enter && valid)){
                std::string src = rename_modal.dir + "/" + rename_modal.old_name;
                std::string dst = rename_modal.dir + "/" + nn;
                if(::rename(src.c_str(), dst.c_str()) == 0){
                    std::string src_info = src + ".info";
                    std::string dst_info = dst + ".info";
                    ::rename(src_info.c_str(), dst_info.c_str());
                    g_arch_cache_dirty = true;
                    g_arch_rescan.store(true);
                    if(file_ctx.filepath == src){
                        file_ctx.filepath = dst;
                        file_ctx.filename = nn;
                    }
                }
                rename_modal.open = false;
            }
            if(!valid) ImGui::EndDisabled();
            ImGui::SameLine();
            if(ImGui::Button("Cancel", ImVec2(80,0)) || ImGui::IsKeyPressed(ImGuiKey_Escape,false)){
                rename_modal.open = false;
            }
            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }

        if(chat_open){
            const float CW=360.f, CH=320.f;
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x-CW-10,
                                           io.DisplaySize.y-CH-TOPBAR_H-10));
            ImGui::SetNextWindowSize(ImVec2(CW,CH));
            ImGui::SetNextWindowBgAlpha(0.92f);
            ImGui::SetNextWindowFocus();
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
                bool is_sys=(strcmp(from,"SYSTEM")==0);
                ImVec4 col = is_sys  ? ImVec4(1.f,0.3f,0.3f,1.f) :
                             is_me   ? ImVec4(0.3f,1.f,0.5f,1.f) :
                                       ImVec4(0.85f,0.85f,0.85f,1.f);
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
                        // SDR 종료 > 로그인 화면으로 (세션 삭제, 프로세스 재시작)
                        do_logout = true;
                        // glfwSetWindowShouldClose 없이 do_logout만으로 inner while 탈출
                        // outer do-while은 !do_main_menu이므로 탈출 > if(do_logout) execv

                    } else if(chat_str == "/main"){
                        // SDR 종료 > main(지구본)으로 (로그인 세션 유지)
                        do_main_menu = true;
                        // inner while의 !do_main_menu 조건으로 탈출
                        // outer do-while이 do_main_menu=true로 재진입 > 지구본부터

                    } else if(chat_str == "/chassis 1 reset"){
                        bewe_log_push(0, "[CMD:%s] /chassis 1 reset\n", login_get_id());
                        if(v.net_srv){
                            push_local("SYSTEM", "Chassis 1 reset ...", false);
                            v.net_srv->broadcast_chat("SYSTEM", "Chassis 1 reset ...");
                            v.net_srv->broadcast_heartbeat(1);
                            // SDR 연결 중일 때만 SDR 복구 경로 진입
                            if(v.is_running || cap.joinable()){
                                v.is_running = false;
                                v.sdr_stream_error.store(true);
                                v.tm_iq_on.store(false);
                                v.spectrum_pause.store(true);
                                usb_reset_pending = true;
                            } else {
                                push_local("SYSTEM", "No SDR connected - skip HW reset", false);
                            }
                        } else if(v.net_cli){
                            // JOIN: HOST에 명령 전달 + 로컬에 메시지 표시
                            push_local("SYSTEM", "Chassis 1 reset ...", false);
                            v.net_cli->cmd_chassis_reset();
                        } else {
                            // LOCAL: SDR 백그라운드 리셋 (UI 유지)
                            push_local("SYSTEM", "Chassis 1 reset ...", false);
                            if(v.is_running || cap.joinable()){
                                v.is_running = false;
                                v.sdr_stream_error.store(true);
                                v.tm_iq_on.store(false);
                                v.spectrum_pause.store(true);
                                usb_reset_pending = true;
                            } else {
                                push_local("SYSTEM", "No SDR connected - skip HW reset", false);
                            }
                        }

                    } else if(chat_str == "/chassis 2 reset"){
                        bewe_log_push(0, "[CMD:%s] /chassis 2 reset\n", login_get_id());
                        if(v.net_srv){
                            push_local("SYSTEM", "Chassis 2 reset ...", false);
                            v.net_srv->broadcast_chat("SYSTEM", "Chassis 2 reset ...");
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
                                srv_ptr->broadcast_chat("SYSTEM", "Chassis 2 stable ...");
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
                                        bewe_log_push(2,"[UI] Central reconnected after chassis 2 reset (chat)\n");
                                    }
                                } else if(central_ptr->is_central_connected()){
                                    central_ptr->send_net_reset(1);
                                }
                                std::lock_guard<std::mutex> lk(*log_mtx_ptr);
                                LocalChatMsg lm{}; lm.is_error = false;
                                strncpy(lm.from, "SYSTEM", 31);
                                strncpy(lm.msg,  "Chassis 2 stable ...", 255);
                                if((int)log_ptr->size() >= 200) log_ptr->erase(log_ptr->begin());
                                log_ptr->push_back(lm);
                            }).detach();
                        } else if(v.net_cli){
                            // JOIN: HOST에게 명령 전달 + 로컬에 메시지 표시
                            push_local("SYSTEM", "Chassis 2 reset ...", false);
                            v.net_cli->cmd_net_reset();
                        } else {
                            push_local("System", "/chassis 2 reset: not in HOST/JOIN mode.", true);
                        }

                    } else if(chat_str == "/rx stop"){
                        bewe_log_push(0, "[CMD:%s] /rx stop\n", login_get_id());
                        if(v.net_cli){
                            push_local("SYSTEM", "RX stop ...", false);
                            v.net_cli->cmd_rx_stop();
                        } else if(v.rx_stopped.load()){
                            push_local("System", "RX already stopped.", true);
                        } else if(!v.is_running && !cap.joinable()){
                            push_local("System", "No SDR running.", true);
                        } else {
                            // HOST / LOCAL: 직접 실행
                            push_local("SYSTEM", "RX stop", false);
                            if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "RX stop");
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
                            push_local("SYSTEM", "RX stopped.", false);
                        }

                    } else if(chat_str == "/rx start"){
                        bewe_log_push(0, "[CMD:%s] /rx start\n", login_get_id());
                        if(v.net_cli){
                            // JOIN: HOST에 명령 전달
                            push_local("SYSTEM", "RX start ...", false);
                            v.net_cli->cmd_rx_start();
                        } else if(!v.rx_stopped.load()){
                            push_local("System", "RX already running.", true);
                        } else {
                            // HOST / LOCAL: 직접 실행
                            push_local("SYSTEM", "RX start", false);
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
                                push_local("SYSTEM", "RX start", false);
                                if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "RX start");
                            } else {
                                v.is_running = false;
                                v.rx_stopped.store(true);
                                push_local("System", "RX start failed - SDR not found.", true);
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

        // ╔══════════════════════════════════════════════════════════════════╗
        // ║  Signal Analysis 독립 오버레이 (E키 토글)                        ║
        // ╚══════════════════════════════════════════════════════════════════╝
        if(v.eid_panel_open){
            // SA 텍스처 업로드 (right panel 없이 EID 오버레이만 열려있을 때도 동작)
            if(v.sa_pixel_ready.load()){ v.sa_upload_texture(); v.sa_anim_timer=0.0f; }
            ImGui::SetNextWindowPos(ImVec2(0,0));
            ImGui::SetNextWindowSize(ImVec2(disp_w, disp_h));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.047f,0.047f,0.07f,0.98f));
            ImGui::Begin("##sig_analysis", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);
            ImDrawList* fg = ImGui::GetWindowDrawList();
            const float SB_H = 22.f;  // 서브바 높이
            float ov_x0 = 0.f, ov_y0 = 0.f;
            float ov_x1 = disp_w, ov_y1 = disp_h;
            float ov_w = ov_x1 - ov_x0, ov_h = ov_y1 - ov_y0;

            // ── 서브바 ──────────────────────────────────────────────────
            float sb_y0 = ov_y0, sb_y1 = ov_y0 + SB_H;
            fg->AddRectFilled(ImVec2(ov_x0,sb_y0), ImVec2(ov_x1,sb_y1), IM_COL32(20,20,30,255));
            fg->AddLine(ImVec2(ov_x0,sb_y1), ImVec2(ov_x1,sb_y1), IM_COL32(60,60,80,255));

            // 서브바 버튼
            struct SaBtn { const char* lbl; int mode; ImU32 col; };
            SaBtn sa_btns[] = {
                {"Spectrogram", 0, IM_COL32(255,200,80,255)},
                {"Amp",         1, IM_COL32(80,255,140,255)},
                {"Freq",        2, IM_COL32(255,120,200,255)},
                {"Phase",       3, IM_COL32(255,220,80,255)},
                {"I/Q",         4, IM_COL32(80,180,255,255)},
                {"Const",       5, IM_COL32(200,80,255,255)},
                {"Audio",       8, IM_COL32(80,255,200,255)},
                {"Power",       6, IM_COL32(255,100,100,255)},
                {"Bits",        7, IM_COL32(120,220,255,255)},
            };
            float bx = ov_x0 + 8.f;
            float btn_ty = sb_y0 + 3.f;
            for(auto& b : sa_btns){
                ImVec2 tsz = ImGui::CalcTextSize(b.lbl);
                bool active = (v.eid_view_mode == b.mode);
                bool hov = (io.MousePos.x >= bx && io.MousePos.x <= bx + tsz.x + 2 &&
                            io.MousePos.y >= sb_y0 && io.MousePos.y < sb_y1);
                ImU32 col = active ? b.col : (hov ? IM_COL32(160,160,180,255) : IM_COL32(110,110,130,255));
                fg->AddText(ImVec2(bx, btn_ty), col, b.lbl);
                if(hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                    int prev_mode = v.eid_view_mode;
                    v.eid_view_mode = b.mode;
                    // 뷰 전환 시 줌 상태 동기화: 스펙트로그램 ↔ 시간 도메인
                    double eid_total = (double)v.eid_total_samples;
                    if(eid_total > 0){
                        if(prev_mode == 0 && b.mode != 0){
                            // 스펙트로그램 > 시간 도메인: UV > 샘플 인덱스
                            v.eid_view_t0 = v.sa_view_y0 * eid_total;
                            v.eid_view_t1 = v.sa_view_y1 * eid_total;
                        } else if(prev_mode != 0 && b.mode == 0){
                            // 시간 도메인 > 스펙트로그램: 샘플 인덱스 > UV
                            v.sa_view_y0 = (float)(v.eid_view_t0 / eid_total);
                            v.sa_view_y1 = (float)(v.eid_view_t1 / eid_total);
                            v.sa_view_y0 = std::max(0.f, std::min(1.f, v.sa_view_y0));
                            v.sa_view_y1 = std::max(0.f, std::min(1.f, v.sa_view_y1));
                        }
                    }
                }
                bx += tsz.x + 14.f;
            }

            // ── 서브바 닫기(×) 버튼 (우측 끝 항상 표시) ───────────────
            {
                const char* close_lbl = "×";
                ImVec2 csz = ImGui::CalcTextSize(close_lbl);
                float cx = ov_x1 - csz.x - 10.f;
                float cy = sb_y0 + (SB_H - csz.y) * 0.5f;
                bool chov = (io.MousePos.x >= cx-4 && io.MousePos.x <= cx+csz.x+4 &&
                             io.MousePos.y >= sb_y0 && io.MousePos.y < sb_y1);
                fg->AddText(ImVec2(cx, cy), chov ? IM_COL32(255,100,100,255) : IM_COL32(160,160,180,200), close_lbl);
                if(chov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                    v.eid_panel_open = false;
                    v.audio_play_stop(); // Audio 재생 중이었으면 정지
                }
            }

            // FFT size + Window 콤보 (Spectrogram 모드일 때만)
            if(v.eid_view_mode == 0){
                bool computing = v.sa_computing.load();
                float combo_y = sb_y0 + (SB_H - ImGui::GetFontSize() - 4)/2;

                // ── FFT size 콤보 (닫기 버튼 왼쪽) ──────────────────────
                static const int fft_sizes[] = {32,64,128,256,512,1024,2048,4096,8192};
                static const char* fft_labels[] = {"32","64","128","256","512","1024","2048","4096","8192"};
                char cur_fft[16]; snprintf(cur_fft,16,"%d",v.sa_fft_size);
                float fft_combo_w = 62;
                float fft_combo_x = ov_x1 - fft_combo_w - 30.f; // 닫기 버튼 공간 확보
                ImGui::SetCursorScreenPos(ImVec2(fft_combo_x, combo_y));
                ImGui::SetNextItemWidth(fft_combo_w);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4,2));
                if(computing) ImGui::BeginDisabled();
                if(ImGui::BeginCombo("##ov_fft", cur_fft)){
                    for(int i=0;i<9;i++){
                        bool sel = (v.sa_fft_size == fft_sizes[i]);
                        if(ImGui::Selectable(fft_labels[i], sel)){
                            if(v.sa_fft_size != fft_sizes[i]){
                                v.sa_fft_size = fft_sizes[i];
                                v.sa_recompute_from_iq(true);
                            }
                        }
                    }
                    ImGui::EndCombo();
                }
                if(computing) ImGui::EndDisabled();
                ImGui::PopStyleVar();
                fg->AddText(ImVec2(fft_combo_x - ImGui::CalcTextSize("FFT").x - 6, btn_ty),
                            IM_COL32(130,130,150,255), "FFT");

                // ── Window 콤보 (FFT 왼쪽) ───────────────────────────────
                static const char* win_labels[] = {"Blackman-Harris","Hann","Nuttall"};
                const char* cur_win = win_labels[v.sa_window_type < 3 ? v.sa_window_type : 0];
                float win_combo_w = 120;
                float win_combo_x = fft_combo_x - ImGui::CalcTextSize("FFT").x - win_combo_w - 20;
                ImGui::SetCursorScreenPos(ImVec2(win_combo_x, combo_y));
                ImGui::SetNextItemWidth(win_combo_w);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4,2));
                if(computing) ImGui::BeginDisabled();
                if(ImGui::BeginCombo("##ov_win", cur_win)){
                    for(int i=0;i<3;i++){
                        bool sel = (v.sa_window_type == i);
                        if(ImGui::Selectable(win_labels[i], sel)){
                            if(v.sa_window_type != i){
                                v.sa_window_type = i;
                                v.sa_recompute_from_iq(true);
                            }
                        }
                    }
                    ImGui::EndCombo();
                }
                if(computing) ImGui::EndDisabled();
                ImGui::PopStyleVar();
                fg->AddText(ImVec2(win_combo_x - ImGui::CalcTextSize("Window").x - 6, btn_ty),
                            IM_COL32(130,130,150,255), "Window");
            }

            // Phase 모드: Sweep line 슬라이더 (우측 정렬)
            if(v.eid_view_mode == 3){
                float max_hz = v.eid_sample_rate > 0 ? (float)v.eid_sample_rate * 0.5f : 10000.f;
                ImVec2 swp_tsz = ImGui::CalcTextSize("Sweep");
                float combo_w = 160;
                float combo_x = ov_x1 - combo_w - 8.f;
                float swp_lx = combo_x - swp_tsz.x - 8.f;
                float combo_y = sb_y0 + (SB_H - ImGui::GetFontSize() - 4)/2;
                fg->AddText(ImVec2(swp_lx, btn_ty), IM_COL32(255,200,80,255), "Sweep");
                ImGui::SetCursorScreenPos(ImVec2(combo_x, combo_y));
                ImGui::SetNextItemWidth(combo_w);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4,2));
                ImGui::SliderFloat("##sweep_hz", &v.eid_phase_detrend_hz, -max_hz, max_hz, "%.1f Hz");
                bool sweep_hovered = ImGui::IsItemHovered();
                if(sweep_hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    v.eid_phase_detrend_hz = 0.0f;
                if(sweep_hovered && io.MouseWheel != 0.f){
                    v.eid_phase_detrend_hz += io.MouseWheel * 1.0f;
                    v.eid_phase_detrend_hz = std::max(-max_hz, std::min(max_hz, v.eid_phase_detrend_hz));
                }
                if(sweep_hovered){
                    if(ImGui::IsKeyPressed(ImGuiKey_RightArrow,true))
                        v.eid_phase_detrend_hz = std::min(max_hz, v.eid_phase_detrend_hz + 1.0f);
                    if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow,true))
                        v.eid_phase_detrend_hz = std::max(-max_hz, v.eid_phase_detrend_hz - 1.0f);
                    if(ImGui::IsKeyPressed(ImGuiKey_UpArrow,true))
                        v.eid_phase_detrend_hz = std::min(max_hz, v.eid_phase_detrend_hz + 10.0f);
                    if(ImGui::IsKeyPressed(ImGuiKey_DownArrow,true))
                        v.eid_phase_detrend_hz = std::max(-max_hz, v.eid_phase_detrend_hz - 10.0f);
                }
                ImGui::PopStyleVar();
            }

            // 1~7 키로 뷰 전환 (뷰 동기화 포함) — EID가 가장 상단일 때만
            if(!io.WantTextInput && top_ov() == 1){
                int new_mode=-1;
                if(ImGui::IsKeyPressed(ImGuiKey_1,false)) new_mode=0;
                if(ImGui::IsKeyPressed(ImGuiKey_2,false)) new_mode=1;
                if(ImGui::IsKeyPressed(ImGuiKey_3,false)) new_mode=2;
                if(ImGui::IsKeyPressed(ImGuiKey_4,false)) new_mode=3;
                if(ImGui::IsKeyPressed(ImGuiKey_5,false)) new_mode=4;
                if(ImGui::IsKeyPressed(ImGuiKey_6,false)) new_mode=5;
                if(ImGui::IsKeyPressed(ImGuiKey_7,false)) new_mode=6;
                if(new_mode>=0 && new_mode!=v.eid_view_mode){
                    int prev=v.eid_view_mode;
                    v.eid_view_mode=new_mode;
                    double et=(double)v.eid_total_samples;
                    if(et>0){
                        if(prev==0 && new_mode!=0){
                            v.eid_view_t0=v.sa_view_y0*et;
                            v.eid_view_t1=v.sa_view_y1*et;
                        } else if(prev!=0 && new_mode==0){
                            v.sa_view_y0=(float)(v.eid_view_t0/et);
                            v.sa_view_y1=(float)(v.eid_view_t1/et);
                            v.sa_view_y0=std::max(0.f,std::min(1.f,v.sa_view_y0));
                            v.sa_view_y1=std::max(0.f,std::min(1.f,v.sa_view_y1));
                        }
                    }
                }
            }

            // ── 콘텐츠 영역 ────────────────────────────────────────────
            float ca_y0 = sb_y1;
            float ca_y1 = ov_y1;
            float ca_h  = ca_y1 - ca_y0;

            int eid_mode = v.eid_view_mode;

            // 태그 컨텍스트 메뉴 상태 (spectrogram/time-domain 공유)
            static struct {
                bool open=false; float x=0,y=0; int tag_idx=-1;
                bool renaming=false; bool rename_focused=false; char rename_buf[32]={};
                bool is_pending=false; // true=임시 영역 메뉴, false=확정 태그 메뉴
            } eid_tag_ctx;

            // Spectrogram 빈 영역 우클릭 > Save File 메뉴
            static struct { bool open=false; float x=0,y=0; } eid_save_ctx;
            static std::atomic<bool> eid_save_busy{false};

            // ── Ctrl+Z: Undo / Ctrl+Shift+Z: Redo ──────────────────────
            if(v.eid_data_ready.load() && !io.WantTextInput){
                if(io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false)){
                    v.eid_do_undo();
                    eid_tag_ctx.open=false;
                }
                if(io.KeyCtrl && io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false)){
                    v.eid_do_redo();
                    eid_tag_ctx.open=false;
                }
            }

            // ── 로딩 ────────────────────────────────────────────────────
            bool loading = v.eid_computing.load() || (eid_mode == 0 && v.sa_computing.load());
            if(loading){
                v.eid_anim_timer += io.DeltaTime;
                int dots = ((int)(v.eid_anim_timer / 0.5f) % 3) + 1;
                char msg[32]; snprintf(msg, sizeof(msg), "Loading %.*s", dots, "...");
                ImVec2 msz = ImGui::CalcTextSize(msg);
                fg->AddText(ImVec2(ov_x0+(ov_w-msz.x)/2, ca_y0+(ca_h-msz.y)/2),
                            IM_COL32(255,100,180,255), msg);

            // ── Spectrogram 모드 (mode 0) ─── 가로: X=시간, Y=주파수 ────
            } else if(eid_mode == 0){
                if(v.sa_texture){
                    const float LM = 60.f, RM = 10.f, TM = 24.f, BM = 30.f;
                    float ea_x0 = ov_x0 + LM, ea_y0 = ca_y0 + TM;
                    float ea_x1 = ov_x1 - RM, ea_y1 = ca_y1 - BM;
                    float ea_w = ea_x1 - ea_x0, ea_h = ea_y1 - ea_y0;

                    if(ea_w > 10.f && ea_h > 10.f){

                    // 파일 정보 헤더
                    {
                        uint32_t sr = v.sa_sample_rate > 0 ? v.sa_sample_rate : 1;
                        double total_time = (double)v.sa_total_rows * v.sa_actual_fft_n / sr;
                        double visible_time = (v.sa_view_y1 - v.sa_view_y0) * total_time;
                        char hdr[256];
                        if(v.sa_center_freq_hz > 0)
                            snprintf(hdr, sizeof(hdr), "Duration: %.4fs | CF: %.3f MHz", visible_time, v.sa_center_freq_hz/1e6);
                        else
                            snprintf(hdr, sizeof(hdr), "Duration: %.4fs", visible_time);
                        fg->AddText(ImVec2(ea_x0, ca_y0 + 4), IM_COL32(160,160,180,220), hdr);
                        // 선택된 태그 PRI/PRF (spectrogram)
                        {
                            float hx = ea_x0 + ImGui::CalcTextSize(hdr).x + 12.f;
                            uint32_t esr = v.eid_sample_rate > 0 ? v.eid_sample_rate : 1;
                            for(auto& tag : v.eid_tags){
                                if(!tag.selected) continue;
                                double tdur = (tag.s1-tag.s0)/(double)esr;
                                double tdur_us = tdur*1e6, trate = tdur>0?1.0/tdur:0;
                                char ti[128]; snprintf(ti,sizeof(ti),"| PRI:%.2fus PRF:%.0fHz PD:%.2fus Baud:%.0f",
                                                       tdur_us,trate,tdur_us,trate);
                                fg->AddText(ImVec2(hx,ca_y0+4),IM_COL32(255,200,100,255),ti);
                                hx+=ImGui::CalcTextSize(ti).x+10.f;
                                if(tag.auto_pulse_count>=2){
                                    char ai[96]; snprintf(ai,sizeof(ai),"[Auto] PRI:%.2fus PRF:%.0fHz Pulses:%d",
                                                          tag.auto_pri_us,tag.auto_prf_hz,tag.auto_pulse_count);
                                    fg->AddText(ImVec2(hx,ca_y0+4),IM_COL32(100,220,255,255),ai);
                                }
                                break;
                            }
                        }
                        if(!v.sa_temp_path.empty()){
                            const char* fn=v.sa_temp_path.c_str();
                            const char* sep=strrchr(fn,'/'); if(sep) fn=sep+1;
                            ImVec2 fsz=ImGui::CalcTextSize(fn);
                            fg->AddText(ImVec2(ea_x1-fsz.x, ca_y0+4), IM_COL32(160,160,180,220), fn);
                        }
                    }

                    // 배경
                    fg->AddRectFilled(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), IM_COL32(8,8,12,255));

                    // 스펙트로그램 텍스처
                    ImTextureID tid = (ImTextureID)(intptr_t)v.sa_texture;
                    float fy0 = v.sa_view_x0, fy1 = v.sa_view_x1;
                    float tx0 = v.sa_view_y0, tx1 = v.sa_view_y1;
                    ImVec2 uv_tl(fy1, tx0), uv_tr(fy1, tx1);
                    ImVec2 uv_br(fy0, tx1), uv_bl(fy0, tx0);
                    fg->AddImageQuad(tid,
                        ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y0),
                        ImVec2(ea_x1, ea_y1), ImVec2(ea_x0, ea_y1),
                        uv_tl, uv_tr, uv_br, uv_bl);

                    // Y축 라벨 (주파수 MHz) - top=고주파, bottom=저주파
                    {
                        double sr_d = v.sa_sample_rate > 0 ? (double)v.sa_sample_rate : 1.0;
                        double cf = v.sa_center_freq_hz > 0 ? (double)v.sa_center_freq_hz : sr_d * 0.5;
                        double freq_lo = cf - sr_d * 0.5;
                        double vis_flo = freq_lo + fy0 * sr_d;
                        double vis_fhi = freq_lo + fy1 * sr_d;
                        int n_divs = std::max(2, std::min(10, (int)(ea_h / 50.f)));
                        for(int i = 0; i <= n_divs; i++){
                            float frac = (float)i / n_divs;
                            float yy = ea_y0 + frac * ea_h;
                            double freq_val = vis_fhi - frac * (vis_fhi - vis_flo);
                            // 그리드 라인 제거 — 라벨만 유지
                            char lbl[32];
                            if(v.sa_center_freq_hz > 0)
                                snprintf(lbl, sizeof(lbl), "%.3f", freq_val / 1e6);
                            else
                                snprintf(lbl, sizeof(lbl), "%.1f", freq_val);
                            ImVec2 tsz = ImGui::CalcTextSize(lbl);
                            fg->AddText(ImVec2(ea_x0 - tsz.x - 4, yy - tsz.y * 0.5f), IM_COL32(150,150,180,255), lbl);
                        }
                        // (MHz/Hz 라벨 제거 - 파일명을 헤더에 표시)
                    }

                    // X축 라벨 (시간)
                    if(v.sa_total_rows > 0 && v.sa_actual_fft_n > 0 && v.sa_sample_rate > 0){
                        uint32_t sr = v.sa_sample_rate;
                        double row_sec = (double)v.sa_actual_fft_n / (double)sr;
                        double total_sec = v.sa_total_rows * row_sec;
                        double t0_sec = tx0 * total_sec;
                        double t1_sec = tx1 * total_sec;
                        double dt = t1_sec - t0_sec;

                        int n_divs = std::max(2, std::min(10, (int)(ea_w / 120.f)));
                        // nice step
                        double raw_step = dt / n_divs;
                        double mag = pow(10.0, floor(log10(raw_step)));
                        double nm = raw_step / mag;
                        double nice_step = (nm<=1.0)?1.0*mag : (nm<=2.0)?2.0*mag : (nm<=5.0)?5.0*mag : 10.0*mag;
                        double t_start = ceil(t0_sec / nice_step) * nice_step;

                        const char* unit = "s"; double ud = 1.0;
                        if(dt < 0.001){ unit = "us"; ud = 1e-6; }
                        else if(dt < 1.0){ unit = "ms"; ud = 1e-3; }

                        for(double ts = t_start; ts <= t1_sec + nice_step*0.1; ts += nice_step){
                            float frac_t = (float)((ts - t0_sec) / dt);
                            float xx = ea_x0 + frac_t * ea_w;
                            if(xx < ea_x0 || xx > ea_x1) continue;
                            // 그리드 라인 제거 — 라벨만 유지
                            char lbl[32]; snprintf(lbl, sizeof(lbl), "%.4g%s", ts / ud, unit);
                            ImVec2 tsz = ImGui::CalcTextSize(lbl);
                            fg->AddText(ImVec2(xx - tsz.x*0.5f, ea_y1+4), IM_COL32(130,130,160,255), lbl);
                        }
                    }

                    // 테두리
                    fg->AddRect(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), IM_COL32(60,60,80,255));

                    // 스크롤 줌 (시간축 = 화면 X)
                    ImVec2 mp = io.MousePos;
                    bool in_sa = (mp.x >= ea_x0 && mp.x < ea_x1 && mp.y >= ea_y0 && mp.y < ea_y1);
                    // 스펙트로그램 줌 > eid_view 동기화 헬퍼
                    auto sync_sa_to_eid = [&](){
                        double et = (double)v.eid_total_samples;
                        if(et > 0){ v.eid_view_t0 = v.sa_view_y0 * et; v.eid_view_t1 = v.sa_view_y1 * et; }
                    };

                    if(in_sa && io.MouseWheel != 0.f){
                        float zf = (io.MouseWheel > 0) ? 0.8f : 1.25f;
                        float frac = (mp.x - ea_x0) / ea_w;
                        float mt = v.sa_view_y0 + frac * (v.sa_view_y1 - v.sa_view_y0);
                        float new_range = (v.sa_view_y1 - v.sa_view_y0) * zf;
                        v.sa_view_y0 = mt - frac * new_range;
                        v.sa_view_y1 = mt + (1.0f - frac) * new_range;
                        if(v.sa_view_y0 < 0.f){ v.sa_view_y1 -= v.sa_view_y0; v.sa_view_y0 = 0.f; }
                        if(v.sa_view_y1 > 1.f){ v.sa_view_y0 -= (v.sa_view_y1 - 1.f); v.sa_view_y1 = 1.f; }
                        v.sa_view_y0 = std::max(0.f, v.sa_view_y0);
                        v.sa_view_y1 = std::min(1.f, v.sa_view_y1);
                        sync_sa_to_eid();
                    }

                    // Ctrl+좌클릭 드래그: Y축 (주파수) 줌 = BPF
                    static bool sa_ctrl_drag = false;
                    static float sa_ctrl_sy0 = 0, sa_ctrl_sy1 = 0;
                    if(in_sa && io.KeyCtrl && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        sa_ctrl_drag=true; sa_ctrl_sy0=mp.y; sa_ctrl_sy1=mp.y;
                    }
                    if(sa_ctrl_drag){
                        if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                            sa_ctrl_sy1=mp.y;
                            float sy0=std::max(ea_y0,std::min(sa_ctrl_sy0,sa_ctrl_sy1));
                            float sy1=std::min(ea_y1,std::max(sa_ctrl_sy0,sa_ctrl_sy1));
                            if(sy1-sy0>1.f){
                                fg->AddRectFilled(ImVec2(ea_x0,sy0),ImVec2(ea_x1,sy1),IM_COL32(100,200,255,40));
                                fg->AddRect(ImVec2(ea_x0,sy0),ImVec2(ea_x1,sy1),IM_COL32(100,200,255,160),0.f,0,1.f);
                                // 주파수 범위 표시
                                double sr_d=v.sa_sample_rate>0?(double)v.sa_sample_rate:1.0;
                                double cf=v.sa_center_freq_hz>0?(double)v.sa_center_freq_hz:sr_d*0.5;
                                double freq_lo_full=cf-sr_d*0.5;
                                float fy0v=v.sa_view_x0,fy1v=v.sa_view_x1;
                                double vis_flo=freq_lo_full+fy0v*sr_d, vis_fhi=freq_lo_full+fy1v*sr_d;
                                float ft=(sy0-ea_y0)/ea_h, fb=(sy1-ea_y0)/ea_h;
                                double f_hi=vis_fhi-ft*(vis_fhi-vis_flo), f_lo=vis_fhi-fb*(vis_fhi-vis_flo);
                                double bw_hz=f_hi-f_lo;
                                char fi[96];
                                if(bw_hz>=1e6) snprintf(fi,sizeof(fi),"BPF: %.3f - %.3f MHz  |  BW: %.3f MHz",f_lo/1e6,f_hi/1e6,bw_hz/1e6);
                                else if(bw_hz>=1e3) snprintf(fi,sizeof(fi),"BPF: %.3f - %.3f MHz  |  BW: %.1f kHz",f_lo/1e6,f_hi/1e6,bw_hz/1e3);
                                else snprintf(fi,sizeof(fi),"BPF: %.3f - %.3f MHz  |  BW: %.1f Hz",f_lo/1e6,f_hi/1e6,bw_hz);
                                ImVec2 tsz=ImGui::CalcTextSize(fi);
                                float tx2=ea_x0+(ea_w-tsz.x)*0.5f;
                                float ty2=sy0+(sy1-sy0-tsz.y)*0.5f;
                                fg->AddRectFilled(ImVec2(tx2-2,ty2-1),ImVec2(tx2+tsz.x+2,ty2+tsz.y+1),IM_COL32(0,0,0,180),3.f);
                                fg->AddText(ImVec2(tx2,ty2),IM_COL32(100,200,255,255),fi);
                            }
                        } else {
                            sa_ctrl_drag=false;
                            float sy0=std::min(sa_ctrl_sy0,sa_ctrl_sy1);
                            float sy1=std::max(sa_ctrl_sy0,sa_ctrl_sy1);
                            if(sy1-sy0>5.f){
                                v.eid_push_undo();
                                v.sa_view_history.push_back({v.sa_view_x0,v.sa_view_x1,v.sa_view_y0,v.sa_view_y1,true});
                                float frac_top=(sy0-ea_y0)/ea_h;
                                float frac_bot=(sy1-ea_y0)/ea_h;
                                float new_x1=v.sa_view_x1-frac_top*(v.sa_view_x1-v.sa_view_x0);
                                float new_x0=v.sa_view_x1-frac_bot*(v.sa_view_x1-v.sa_view_x0);
                                new_x0=std::max(0.f,new_x0);
                                new_x1=std::min(1.f,new_x1);
                                v.eid_apply_bpf(new_x0,new_x1);
                                v.sa_view_x0=0.f; v.sa_view_x1=1.f;
                            }
                        }
                    }

                    // 좌클릭 드래그 줌 (시간축 = X축)
                    static bool sa_sel_drag = false;
                    static float sa_sel_sx0 = 0, sa_sel_sx1 = 0;
                    if(in_sa && !io.KeyCtrl && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        sa_sel_drag=true; sa_sel_sx0=mp.x; sa_sel_sx1=mp.x;
                    }
                    if(sa_sel_drag){
                        if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                            sa_sel_sx1=mp.x;
                            float sx0=std::max(ea_x0,std::min(sa_sel_sx0,sa_sel_sx1));
                            float sx1=std::min(ea_x1,std::max(sa_sel_sx0,sa_sel_sx1));
                            if(sx1-sx0>1.f){
                                fg->AddRectFilled(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,50));
                                fg->AddRect(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,160),0.f,0,1.f);
                            }
                        } else {
                            sa_sel_drag=false;
                            float sx0=std::min(sa_sel_sx0,sa_sel_sx1);
                            float sx1=std::max(sa_sel_sx0,sa_sel_sx1);
                            if(sx1-sx0>5.f){
                                v.eid_push_undo();
                                v.sa_view_history.push_back({v.sa_view_x0,v.sa_view_x1,v.sa_view_y0,v.sa_view_y1,false});
                                float f0=v.sa_view_y0+((sx0-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                                float f1=v.sa_view_y0+((sx1-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                                v.sa_view_y0=std::max(0.f,f0);
                                v.sa_view_y1=std::min(1.f,f1);
                                sync_sa_to_eid();
                            }
                        }
                    }

                    // 우클릭: 드래그>임시 선택 영역, 클릭>컨텍스트 메뉴
                    static bool sa_tag_drag = false;
                    static float sa_tag_dx0 = 0, sa_tag_dx1 = 0;
                    static float sa_prev_y0 = 0, sa_prev_y1 = 1;
                    if(in_sa && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !eid_tag_ctx.open){
                        sa_tag_drag=true; sa_tag_dx0=mp.x; sa_tag_dx1=mp.x;
                    }
                    if(sa_tag_drag && ImGui::IsMouseDown(ImGuiMouseButton_Right)){
                        sa_tag_dx1=mp.x;
                        float sx0=std::max(ea_x0,std::min(sa_tag_dx0,sa_tag_dx1));
                        float sx1=std::min(ea_x1,std::max(sa_tag_dx0,sa_tag_dx1));
                        if(sx1-sx0>1.f){
                            fg->AddRectFilled(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,40));
                            fg->AddRect(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,160),0.f,0,1.f);
                        }
                    }
                    if(sa_tag_drag && ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
                        sa_tag_drag=false;
                        float dx=fabsf(sa_tag_dx1-sa_tag_dx0);
                        if(dx>5.f){
                            // 임시 선택 영역으로 저장 (태그 즉시 생성 안 함)
                            float sx0=std::max(ea_x0,std::min(sa_tag_dx0,sa_tag_dx1));
                            float sx1=std::min(ea_x1,std::max(sa_tag_dx0,sa_tag_dx1));
                            double total=(double)v.eid_total_samples;
                            if(total>0){
                                float uv0=v.sa_view_y0+((sx0-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                                float uv1=v.sa_view_y0+((sx1-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                                v.eid_pending_s0=std::max(0.0,(double)uv0*total);
                                v.eid_pending_s1=std::min(total,(double)uv1*total);
                                v.eid_pending_active=true;
                            }
                        } else {
                            // 짧은 클릭: 임시 영역 내부 → 컨텍스트 메뉴, 태그 위 → 태그 메뉴, 밖 → undo
                            double total=(double)v.eid_total_samples;
                            if(total>0){
                                float uv=v.sa_view_y0+((mp.x-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                                double click_s=uv*total;
                                // 임시 영역 내부 클릭 확인
                                bool in_pending = v.eid_pending_active &&
                                                  click_s>=v.eid_pending_s0 && click_s<=v.eid_pending_s1;
                                if(in_pending){
                                    eid_tag_ctx.open=true; eid_tag_ctx.x=mp.x; eid_tag_ctx.y=mp.y;
                                    eid_tag_ctx.tag_idx=-1; eid_tag_ctx.renaming=false;
                                    eid_tag_ctx.is_pending=true;
                                } else {
                                    int hit_tag=-1;
                                    for(int ti=0;ti<(int)v.eid_tags.size();ti++)
                                        if(click_s>=v.eid_tags[ti].s0&&click_s<=v.eid_tags[ti].s1){ hit_tag=ti; break; }
                                    if(hit_tag>=0){
                                        eid_tag_ctx.open=true; eid_tag_ctx.x=mp.x; eid_tag_ctx.y=mp.y;
                                        eid_tag_ctx.tag_idx=hit_tag; eid_tag_ctx.renaming=false;
                                        eid_tag_ctx.is_pending=false;
                                        strncpy(eid_tag_ctx.rename_buf,v.eid_tags[hit_tag].label,31);
                                    } else {
                                        // 빈 영역 단순 클릭: Save File 메뉴
                                        v.eid_pending_active=false;
                                        eid_save_ctx.open=true;
                                        eid_save_ctx.x=mp.x; eid_save_ctx.y=mp.y;
                                    }
                                }
                            } else {
                                v.eid_pending_active=false;
                            }
                        }
                    }
                    // 임시 선택 영역 렌더링 (드래그 중 아닐 때도 유지)
                    if(v.eid_pending_active && !sa_tag_drag){
                        double total=(double)v.eid_total_samples;
                        if(total>0){
                            float uv0=(float)(v.eid_pending_s0/total);
                            float uv1=(float)(v.eid_pending_s1/total);
                            float px0=ea_x0+((uv0-v.sa_view_y0)/(v.sa_view_y1-v.sa_view_y0))*ea_w;
                            float px1=ea_x0+((uv1-v.sa_view_y0)/(v.sa_view_y1-v.sa_view_y0))*ea_w;
                            px0=std::max(ea_x0,std::min(ea_x1,px0));
                            px1=std::max(ea_x0,std::min(ea_x1,px1));
                            if(px1>px0+1.f){
                                fg->AddRectFilled(ImVec2(px0,ea_y0),ImVec2(px1,ea_y1),IM_COL32(200,200,220,30));
                                fg->AddRect(ImVec2(px0,ea_y0),ImVec2(px1,ea_y1),IM_COL32(200,200,220,140),0.f,0,1.f);
                            }
                        }
                    }

                    // Home 키: 전체 보기
                    if(in_sa && ImGui::IsKeyPressed(ImGuiKey_Home,false)){
                        v.sa_view_y0=0.f; v.sa_view_y1=1.f;
                        v.sa_view_x0=0.f; v.sa_view_x1=1.f;
                        v.sa_view_history.clear();
                        if(v.eid_bpf_active) v.eid_undo_bpf();
                        sync_sa_to_eid();
                    }

                    // Delete 키 / 더블클릭: 임시 영역 또는 태그 삭제 (마우스 위치 기반)
                    auto sa_delete_at = [&](double ms){
                        if(v.eid_pending_active && ms>=v.eid_pending_s0 && ms<=v.eid_pending_s1){
                            v.eid_push_undo();
                            v.eid_pending_active=false;
                        } else {
                            for(auto it=v.eid_tags.begin();it!=v.eid_tags.end();++it){
                                if(ms>=it->s0&&ms<=it->s1){ v.eid_push_undo(); v.eid_tags.erase(it); break; }
                            }
                        }
                    };
                    if(in_sa && ImGui::IsKeyPressed(ImGuiKey_Delete,false)){
                        double total=(double)v.eid_total_samples;
                        if(total>0){
                            float uv=v.sa_view_y0+((mp.x-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                            sa_delete_at(uv*total);
                        }
                    }
                    if(in_sa && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                        double total=(double)v.eid_total_samples;
                        if(total>0){
                            float uv=v.sa_view_y0+((mp.x-ea_x0)/ea_w)*(v.sa_view_y1-v.sa_view_y0);
                            sa_delete_at(uv*total);
                        }
                        sa_sel_drag=false; // 더블클릭 시 줌 드래그 시작 방지
                    }

                    // 커서 오버레이 (십자선 + Freq/Time 정보)
                    if(in_sa){
                        fg->AddLine(ImVec2(mp.x,ea_y0),ImVec2(mp.x,ea_y1),IM_COL32(255,255,255,60));
                        fg->AddLine(ImVec2(ea_x0,mp.y),ImVec2(ea_x1,mp.y),IM_COL32(255,255,255,60));

                        if(v.sa_center_freq_hz > 0 && v.sa_actual_fft_n > 0 && v.sa_sample_rate > 0){
                            float fx = (mp.x - ea_x0) / ea_w;
                            float fy = (mp.y - ea_y0) / ea_h;
                            float tv = v.sa_view_y0 + fx * (v.sa_view_y1 - v.sa_view_y0);
                            float fu = v.sa_view_x1 - fy * (v.sa_view_x1 - v.sa_view_x0);
                            double freq_hz = (double)v.sa_center_freq_hz - (double)v.sa_sample_rate*0.5 + fu*(double)v.sa_sample_rate;
                            double row_sec = (double)v.sa_actual_fft_n / (double)v.sa_sample_rate;
                            double t_off = tv * v.sa_total_rows * row_sec;
                            time_t ta = (time_t)(v.sa_start_time + (int64_t)t_off);
                            struct tm* tmv = localtime(&ta);
                            char tb[16]="--:--:--";
                            if(tmv) strftime(tb, sizeof(tb), "%H:%M:%S", tmv);
                            char fb[32]; snprintf(fb, sizeof(fb), "Freq : %.3fMHz", freq_hz/1e6);
                            char tt[32]; snprintf(tt, sizeof(tt), "Time : %s", tb);
                            float fw = std::max(ImGui::CalcTextSize(fb).x, ImGui::CalcTextSize(tt).x);
                            float fh = ImGui::GetFontSize() * 2.5f;
                            float ox = ea_x1 - fw - 14.f, oy = ea_y0 + 8.f;
                            fg->AddRectFilled(ImVec2(ox-4,oy-2), ImVec2(ox+fw+4,oy+fh), IM_COL32(0,0,0,180), 4.f);
                            fg->AddText(ImVec2(ox,oy), IM_COL32(200,230,255,255), fb);
                            fg->AddText(ImVec2(ox,oy+ImGui::GetFontSize()+2.f), IM_COL32(200,230,255,255), tt);
                        }
                    }

                    } // end if(ea_w > 10 && ea_h > 10)
                } else {
                    const char* msg = "No spectrogram data";
                    ImVec2 msz = ImGui::CalcTextSize(msg);
                    fg->AddText(ImVec2(ov_x0+(ov_w-msz.x)/2, ca_y0+(ca_h-msz.y)/2),
                                IM_COL32(100,100,120,255), msg);
                }

            // ── Constellation 모드 (mode 5) ─────────────────────────────
            } else if(eid_mode == 5 && v.eid_data_ready.load()){
                const float LM=60.f, RM=10.f, TM=24.f, BM=50.f; // BM 늘려서 슬라이더 공간
                float ea_x0=ov_x0+LM, ea_y0=ca_y0+TM;
                float ea_x1=ov_x1-RM, ea_y1=ca_y1-BM;
                float ea_w=ea_x1-ea_x0, ea_h=ea_y1-ea_y0;
                if(ea_w>10.f && ea_h>10.f){
                    uint32_t sr=v.eid_sample_rate>0?v.eid_sample_rate:1;

                    // 뷰 범위 사용 (다른 모드와 동기화)
                    int64_t vw0=(int64_t)v.eid_view_t0;
                    int64_t vw1=(int64_t)v.eid_view_t1;
                    if(vw0<0) vw0=0;
                    if(vw1>v.eid_total_samples) vw1=v.eid_total_samples;
                    if(vw1<=vw0){ vw0=0; vw1=v.eid_total_samples; }
                    int64_t view_n=vw1-vw0;

                    // 서브윈도우: 뷰 범위 내에서 eid_const_win 크기
                    v.eid_const_win=std::max(64,std::min(v.eid_const_win,(int)view_n));

                    // 자동 재생
                    if(v.eid_const_playing){
                        float speed=(float)sr*0.05f;
                        v.eid_const_pos+=speed*io.DeltaTime*10.0;
                        if(v.eid_const_pos+v.eid_const_win>=(double)vw1)
                            v.eid_const_pos=(double)vw0;
                    }
                    // 클램프 (뷰 범위 내)
                    v.eid_const_pos=std::max((double)vw0,std::min(v.eid_const_pos,
                        (double)(vw1-v.eid_const_win)));

                    // 헤더
                    {
                        double t0_ms=v.eid_const_pos/(double)sr*1e3;
                        double t1_ms=(v.eid_const_pos+v.eid_const_win)/(double)sr*1e3;
                        char hdr[256];
                        snprintf(hdr,sizeof(hdr),"%.1f-%.1fms (%d samp) | CF: %.3f MHz",
                            t0_ms,t1_ms,v.eid_const_win,v.eid_center_freq_hz/1e6);
                        fg->AddText(ImVec2(ea_x0,ca_y0+4),IM_COL32(160,160,180,220),hdr);
                        if(!v.sa_temp_path.empty()){
                            const char* fn=v.sa_temp_path.c_str();
                            const char* sep=strrchr(fn,'/'); if(sep) fn=sep+1;
                            ImVec2 fsz=ImGui::CalcTextSize(fn);
                            fg->AddText(ImVec2(ea_x1-fsz.x,ca_y0+4),IM_COL32(160,160,180,220),fn);
                        }
                    }
                    // 정사각형 플롯 영역
                    float side=std::min(ea_w,ea_h);
                    float plot_cx=ea_x0+ea_w*0.5f, plot_cy=ea_y0+ea_h*0.5f;
                    float px0=plot_cx-side*0.5f, py0=plot_cy-side*0.5f;
                    float px1=plot_cx+side*0.5f, py1=plot_cy+side*0.5f;

                    // 윈도우 범위 & 데시메이션
                    int64_t w0=(int64_t)v.eid_const_pos;
                    int64_t w1=std::min((int64_t)(w0+v.eid_const_win),v.eid_total_samples);
                    int64_t win_n=w1-w0;
                    int64_t step=std::max((int64_t)1,win_n/30000);

                    // Phase 탭의 Sweep 값을 캐리어 오프셋으로 사용
                    double phase_inc=2.0*M_PI*v.eid_phase_detrend_hz/(double)sr;

                    // 자동 스케일: 현재 윈도우의 IQ 최대 진폭 계산
                    float max_amp=0.0f;
                    for(int64_t s=w0;s<w1;s+=step){
                        float ri=v.eid_ch_i[s], rq=v.eid_ch_q[s];
                        float amp=ri*ri+rq*rq;
                        if(amp>max_amp) max_amp=amp;
                    }
                    max_amp=sqrtf(max_amp);
                    if(max_amp<1e-9f) max_amp=1.0f;

                    float auto_scale=side*0.45f/max_amp;
                    float scale;
                    if(v.eid_const_zoom>0.0f)
                        scale=side*0.5f*v.eid_const_zoom;
                    else
                        scale=auto_scale;

                    // 배경
                    fg->AddRectFilled(ImVec2(px0,py0),ImVec2(px1,py1),IM_COL32(8,8,12,255));
                    // 십자 축선
                    fg->AddLine(ImVec2(px0,plot_cy),ImVec2(px1,plot_cy),IM_COL32(60,60,80,180));
                    fg->AddLine(ImVec2(plot_cx,py0),ImVec2(plot_cx,py1),IM_COL32(60,60,80,180));
                    // 격자 (0.25 간격)
                    for(int g=1;g<=3;g++){
                        float off=g*0.25f*side*0.5f;
                        fg->AddLine(ImVec2(plot_cx+off,py0),ImVec2(plot_cx+off,py1),IM_COL32(40,40,55,100));
                        fg->AddLine(ImVec2(plot_cx-off,py0),ImVec2(plot_cx-off,py1),IM_COL32(40,40,55,100));
                        fg->AddLine(ImVec2(px0,plot_cy+off),ImVec2(px1,plot_cy+off),IM_COL32(40,40,55,100));
                        fg->AddLine(ImVec2(px0,plot_cy-off),ImVec2(px1,plot_cy-off),IM_COL32(40,40,55,100));
                    }
                    // 단위원
                    fg->AddCircle(ImVec2(plot_cx,plot_cy),side*0.5f,IM_COL32(80,80,120,120),64,1.f);
                    // 축 라벨
                    fg->AddText(ImVec2(px1+4,plot_cy-7),IM_COL32(150,150,180,255),"I");
                    fg->AddText(ImVec2(plot_cx-4,py0-16),IM_COL32(150,150,180,255),"Q");

                    // 데이터 플로팅 (캐리어 제거)
                    fg->PushClipRect(ImVec2(px0,py0),ImVec2(px1,py1),true);
                    for(int64_t s=w0;s<w1;s+=step){
                        float ri=v.eid_ch_i[s], rq=v.eid_ch_q[s];
                        double theta=-phase_inc*(double)(s-w0);
                        float ct=(float)cos(theta), st=(float)sin(theta);
                        float iv=ri*ct-rq*st;
                        float qv=ri*st+rq*ct;
                        float sx=plot_cx+iv*scale;
                        float sy=plot_cy-qv*scale;
                        fg->AddCircleFilled(ImVec2(sx,sy),1.5f,IM_COL32(80,255,140,120));
                    }
                    fg->PopClipRect();
                    // 테두리
                    fg->AddRect(ImVec2(px0,py0),ImVec2(px1,py1),IM_COL32(60,60,80,255));

                    // ── 하단 컨트롤: Play/Pause + 슬라이더 + Window 크기 ──
                    float ctrl_y=ea_y1+6.f;
                    ImGui::SetCursorScreenPos(ImVec2(ea_x0,ctrl_y));
                    if(ImGui::SmallButton(v.eid_const_playing?"||":" > ")){
                        v.eid_const_playing=!v.eid_const_playing;
                    }
                    ImGui::SameLine(0,8);
                    // 위치 슬라이더 (뷰 범위 내에서)
                    float slider_w=ea_w-240.f;
                    if(slider_w<50.f) slider_w=50.f;
                    ImGui::SetNextItemWidth(slider_w);
                    float pos_f=(float)v.eid_const_pos;
                    float pos_min_f=(float)vw0;
                    float pos_max_f=(float)std::max((int64_t)1,vw1-(int64_t)v.eid_const_win);
                    if(ImGui::SliderFloat("##const_pos",&pos_f,pos_min_f,pos_max_f,"")){
                        v.eid_const_pos=(double)pos_f;
                    }
                    if(ImGui::IsItemActive()) v.eid_const_playing=false;
                    ImGui::SameLine(0,8);
                    // Window 크기 콤보
                    ImGui::Text("Win:"); ImGui::SameLine(0,4);
                    {
                        const int wsizes[]={64,128,256,512,1024,2048,4096,8192,16384,32768,65536};
                        int cur_idx=4;
                        for(int i=0;i<11;i++) if(wsizes[i]==v.eid_const_win) cur_idx=i;
                        char wl[16]; snprintf(wl,sizeof(wl),"%d",v.eid_const_win);
                        ImGui::SetNextItemWidth(80.f);
                        if(ImGui::BeginCombo("##const_win",wl,ImGuiComboFlags_NoArrowButton)){
                            for(int i=0;i<11;i++){
                                if(wsizes[i]>(int)view_n) break;
                                char ll[16]; snprintf(ll,sizeof(ll),"%d",wsizes[i]);
                                bool is_sel=(wsizes[i]==v.eid_const_win);
                                if(ImGui::Selectable(ll,is_sel))
                                    v.eid_const_win=wsizes[i];
                                if(is_sel) ImGui::SetItemDefaultFocus();
                            }
                            ImGui::EndCombo();
                        }
                    }
                    // 마우스 휠로 줌 조절 (플롯 위에서)
                    ImVec2 mp2=io.MousePos;
                    if(mp2.x>=px0&&mp2.x<=px1&&mp2.y>=py0&&mp2.y<=py1){
                        if(io.MouseWheel!=0.f){
                            if(v.eid_const_zoom<=0.0f)
                                v.eid_const_zoom=auto_scale/(side*0.5f);
                            float factor=io.MouseWheel>0?1.25f:0.8f;
                            v.eid_const_zoom*=factor;
                            v.eid_const_zoom=std::max(0.001f,std::min(v.eid_const_zoom,10000.f));
                        }
                        if(ImGui::IsMouseDoubleClicked(0))
                            v.eid_const_zoom=0.0f;
                    }
                }

            // ── Audio 모드 (mode 8): WAV 재생 + bipolar waveform + cursor ──
            } else if(eid_mode == 8 && v.eid_data_ready.load()){
                const float LM=20.f, RM=10.f, TM=24.f, BM=30.f;
                float ea_x0=ov_x0+LM, ea_y0=ca_y0+TM;
                float ea_x1=ov_x1-RM, ea_y1=ca_y1-BM;
                float ea_w=ea_x1-ea_x0, ea_h=ea_y1-ea_y0;
                if(ea_w>10.f && ea_h>10.f){
                    uint32_t sr=v.eid_sample_rate; if(sr==0) sr=1;
                    ImDrawList* fg2=ImGui::GetForegroundDrawList();

                    // 배경
                    fg2->AddRectFilled(ImVec2(ea_x0,ea_y0), ImVec2(ea_x1,ea_y1),
                                       IM_COL32(22,22,28,255));
                    fg2->AddRect(ImVec2(ea_x0,ea_y0), ImVec2(ea_x1,ea_y1),
                                 IM_COL32(60,60,80,255));

                    double vt0=v.eid_view_t0, vt1=v.eid_view_t1;
                    double vis_samp=vt1-vt0; if(vis_samp<1.0) vis_samp=1.0;
                    int pixels=(int)ea_w; if(pixels<1) pixels=1;
                    double spp=vis_samp/pixels;

                    float y_mid  = (ea_y0 + ea_y1) * 0.5f;
                    float half_h = (ea_y1 - ea_y0) * 0.5f;
                    // 파형이 쓰는 실제 반높이 — 상하 20%씩 헤드룸 (여유)
                    float usable_h = half_h * 0.6f;

                    // 시간 포맷 헬퍼: MM:SS.ss 또는 H:MM:SS.ss
                    auto fmt_time = [](double t_sec, char* buf, size_t bufsz, bool subsec){
                        if(t_sec < 0) t_sec = 0;
                        int h = (int)(t_sec / 3600.0);
                        int m = (int)((t_sec - h*3600.0) / 60.0);
                        double s = t_sec - h*3600.0 - m*60.0;
                        if(h > 0){
                            if(subsec) snprintf(buf,bufsz,"%d:%02d:%05.2f",h,m,s);
                            else       snprintf(buf,bufsz,"%d:%02d:%02d",h,m,(int)s);
                        } else if(subsec){
                            snprintf(buf,bufsz,"%02d:%05.2f",m,s);
                        } else {
                            snprintf(buf,bufsz,"%02d:%02d",m,(int)s);
                        }
                    };

                    // 중앙선 (0 기준선)
                    fg2->AddLine(ImVec2(ea_x0,y_mid), ImVec2(ea_x1,y_mid),
                                 IM_COL32(70,70,90,255));

                    // X축 시간 그리드 + MM:SS 라벨 (상단)
                    {
                        double t0s=(double)vt0/sr, t1s=(double)vt1/sr, dts=t1s-t0s;
                        double rs = dts / std::max(2.0, (double)(pixels/120));
                        double mg = pow(10.0, floor(log10(rs)));
                        double nm = rs/mg;
                        double ns = (nm<=1.0)?1.0*mg:(nm<=2.0)?2.0*mg
                                  :(nm<=5.0)?5.0*mg:10.0*mg;
                        if(ns < 0.001) ns = 0.001; // 최소 1ms
                        double su = ceil(t0s/ns)*ns;
                        bool subsec = (dts < 10.0);
                        for(double ts=su; ts<=t1s+ns*0.5; ts+=ns){
                            float xx=ea_x0+(float)((ts*sr-vt0)/vis_samp)*ea_w;
                            if(xx<ea_x0||xx>ea_x1) continue;
                            fg2->AddLine(ImVec2(xx,ea_y0), ImVec2(xx,ea_y1),
                                         IM_COL32(40,40,55,255));
                            char lbl[32]; fmt_time(ts, lbl, sizeof(lbl), subsec);
                            ImVec2 tsz=ImGui::CalcTextSize(lbl);
                            fg2->AddText(ImVec2(xx-tsz.x*0.5f, ea_y1+4),
                                         IM_COL32(130,130,160,255), lbl);
                        }
                    }

                    // Waveform (Audacity 스타일) — Peak(반투명) + RMS(진함)
                    // 픽셀당 min/max(peak)와 RMS를 동시에 계산해서 이중 렌더링
                    fg2->PushClipRect(ImVec2(ea_x0,ea_y0), ImVec2(ea_x1,ea_y1), true);
                    {
                        const auto& ch = v.eid_ch_i;
                        int64_t total = (int64_t)ch.size();
                        if(total > 0){
                            const ImU32 col_rms = IM_COL32(240,70,70,255); // RMS 진한 빨강
                            if(spp<=1.0){
                                // 픽셀당 샘플 1개 이하 — 그냥 라인 한 줄
                                std::vector<ImVec2> pts;
                                int64_t s0c=std::max((int64_t)0,(int64_t)vt0);
                                int64_t s1c=std::min(total,(int64_t)ceil(vt1)+1);
                                pts.reserve((size_t)(s1c-s0c));
                                for(int64_t s=s0c; s<s1c; s++){
                                    float val = ch[s];
                                    if(val<-1.f) val=-1.f;
                                    if(val>1.f)  val=1.f;
                                    float yy = y_mid - val*usable_h;
                                    float xx = ea_x0 + (float)((s-vt0)/vis_samp)*ea_w;
                                    pts.push_back(ImVec2(xx,yy));
                                }
                                if(pts.size()>=2)
                                    fg2->AddPolyline(pts.data(),(int)pts.size(),
                                                     col_rms,ImDrawFlags_None,1.2f);
                            } else {
                                // 픽셀당 여러 샘플 — Peak + RMS 이중 렌더링
                                for(int px=0; px<pixels; px++){
                                    int64_t s0=(int64_t)(vt0+px*spp);
                                    int64_t s1=(int64_t)(vt0+(px+1)*spp);
                                    s0=std::max((int64_t)0,std::min(s0,total-1));
                                    s1=std::max(s0+1,std::min(s1,total));
                                    double sum_sq=0.0; int64_t n=0;
                                    for(int64_t s=s0; s<s1; s++){
                                        float val=ch[s];
                                        sum_sq += (double)val*val;
                                        n++;
                                    }
                                    float rms = (n>0) ? (float)sqrt(sum_sq/(double)n) : 0.f;
                                    if(rms>1.f) rms=1.f;

                                    // RMS 수직 막대 — 말하는 구간이 두툼하게 부풀어 오름
                                    float y_hi_rms = y_mid - rms*usable_h;
                                    float y_lo_rms = y_mid + rms*usable_h;
                                    fg2->AddRectFilled(ImVec2((float)(ea_x0+px),     y_hi_rms),
                                                       ImVec2((float)(ea_x0+px+1.f), y_lo_rms),
                                                       col_rms);
                                }
                            }
                        }
                    }

                    // 재생 중이면 cursor = 현재 재생 위치로 실시간 동기화
                    if(v.audio_play_active()){
                        double pos_sec = (double)v.audio_play_pos_sec();
                        v.eid_audio_cursor_sample = (int64_t)(pos_sec * sr);
                    }

                    // Cursor 밴드 + 중심선 (노란색) — 재생 중이면 자연스럽게 움직임
                    {
                        int64_t cs = v.eid_audio_cursor_sample;
                        if(cs >= (int64_t)vt0 && cs <= (int64_t)vt1){
                            float cxp = ea_x0 + (float)(((double)cs-vt0)/vis_samp)*ea_w;
                            fg2->AddRectFilled(ImVec2(cxp-3,ea_y0),
                                               ImVec2(cxp+3,ea_y1),
                                               IM_COL32(255,235,80,40));
                            fg2->AddLine(ImVec2(cxp,ea_y0), ImVec2(cxp,ea_y1),
                                         IM_COL32(255,235,80,255), 1.5f);
                        }
                    }
                    fg2->PopClipRect();

                    // 상태 헤더 (PLAY/PAUSE + cursor - total)
                    {
                        bool is_playing = v.audio_play_active() && !v.audio_play_paused();
                        const char* state = is_playing ? "PLAY" : "PAUSE";
                        ImU32 scol = is_playing ? IM_COL32(80,255,140,255)
                                                : IM_COL32(255,180,80,255);
                        double cs_sec    = (double)v.eid_audio_cursor_sample / sr;
                        double total_sec = (double)v.eid_total_samples / sr;
                        char ct[32]; fmt_time(cs_sec,    ct, sizeof(ct), true);
                        char tt[32]; fmt_time(total_sec, tt, sizeof(tt), true);
                        char info[128];
                        snprintf(info, sizeof(info), "%s   %s - %s", state, ct, tt);
                        fg2->AddText(ImVec2(ov_x0+8, ca_y0+4), scol, info);
                    }

                    // ── 인터랙션 ──
                    bool hov = (io.MousePos.x >= ea_x0 && io.MousePos.x <= ea_x1
                             && io.MousePos.y >= ea_y0 && io.MousePos.y <= ea_y1);

                    // 마우스 휠 → X축 줌 (마우스 위치 기준)
                    if(hov && io.MouseWheel != 0.f){
                        double zoom_f = (io.MouseWheel > 0) ? 0.8 : 1.25;
                        double mouse_t = vt0 + ((io.MousePos.x - ea_x0) / ea_w) * vis_samp;
                        double frac = (mouse_t - vt0) / vis_samp;
                        if(frac < 0.0) frac = 0.0;
                        if(frac > 1.0) frac = 1.0;
                        double new_vis = vis_samp * zoom_f;
                        double nt0 = mouse_t - new_vis * frac;
                        double nt1 = nt0 + new_vis;
                        if(nt0 < 0){ nt1 -= nt0; nt0 = 0; }
                        if(nt1 > (double)v.eid_total_samples){
                            nt0 -= (nt1 - (double)v.eid_total_samples);
                            nt1 = (double)v.eid_total_samples;
                        }
                        nt0 = std::max(0.0, nt0);
                        nt1 = std::min((double)v.eid_total_samples, nt1);
                        if(nt1 - nt0 < 32.0){
                            double mid = (nt0 + nt1) * 0.5;
                            nt0 = mid - 16.0; nt1 = mid + 16.0;
                        }
                        v.eid_view_t0 = nt0;
                        v.eid_view_t1 = nt1;
                    }

                    // 좌클릭 → cursor 설정 (재생 중이면 정지 후 새 위치 대기)
                    if(hov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        double sx = vt0 + ((io.MousePos.x - ea_x0) / ea_w) * vis_samp;
                        if(sx<0) sx=0;
                        if(sx>(double)v.eid_total_samples) sx=(double)v.eid_total_samples;
                        v.eid_audio_cursor_sample = (int64_t)sx;
                        if(v.audio_play_active()) v.audio_play_stop();
                    }

                    // 스페이스 → 재생/일시정지 토글
                    if(!io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Space, false)){
                        if(!v.audio_play_active()){
                            double off_sec = (double)v.eid_audio_cursor_sample / sr;
                            if(!v.sa_temp_path.empty())
                                v.audio_play_start(v.sa_temp_path, off_sec);
                        } else if(v.audio_play_paused()){
                            v.audio_play_resume();
                        } else {
                            v.audio_play_pause();
                        }
                    }
                }

            // ── M-th Power Spectrum 모드 (mode 6) ────────────────────────
            } else if(eid_mode == 6 && v.eid_data_ready.load()){
                const float LM=60.f, RM=10.f, TM=24.f, BM=50.f;
                float ea_x0=ov_x0+LM, ea_y0=ca_y0+TM;
                float ea_x1=ov_x1-RM, ea_y1=ca_y1-BM;
                float ea_w=ea_x1-ea_x0, ea_h=ea_y1-ea_y0;
                if(ea_w>10.f && ea_h>10.f){
                    uint32_t sr=v.eid_sample_rate;
                    if(sr==0) sr=1;
                    ImDrawList* fg=ImGui::GetForegroundDrawList();

                    // 현재 뷰 범위 사용 (다른 모드에서 줌한 상태 그대로)
                    int64_t pw0=(int64_t)v.eid_view_t0;
                    int64_t pw1=(int64_t)v.eid_view_t1;
                    if(pw0<0) pw0=0;
                    if(pw1>v.eid_total_samples) pw1=v.eid_total_samples;
                    if(pw1<=pw0){ pw0=0; pw1=v.eid_total_samples; }
                    int64_t pwin=pw1-pw0;

                    int fft_n=v.eid_power_fft_n;
                    int M=v.eid_power_order;

                    // 헤더 정보
                    {
                        double t0_ms=pw0/(double)sr*1000.0;
                        double t1_ms=pw1/(double)sr*1000.0;
                        char hdr[256];
                        snprintf(hdr,sizeof(hdr),"M=%d  |  FFT: %d  |  %.1f-%.1fms (%lld samp)  |  CF: %.3f MHz",
                            M, fft_n, t0_ms, t1_ms, (long long)pwin,
                            v.eid_center_freq_hz/1e6);
                        fg->AddText(ImVec2(ea_x0,ca_y0+4),IM_COL32(200,200,220,255),hdr);
                        if(!v.sa_temp_path.empty()){
                            const char* fn=v.sa_temp_path.c_str();
                            const char* sep=strrchr(fn,'/'); if(sep) fn=sep+1;
                            ImVec2 fsz=ImGui::CalcTextSize(fn);
                            fg->AddText(ImVec2(ea_x1-fsz.x,ca_y0+4),IM_COL32(160,160,180,220),fn);
                        }
                    }

                    // M-th power FFT 계산
                    std::vector<float> psd(fft_n,0.f);
                    int n_avg=0;
                    if(pwin>=fft_n){
                        std::vector<float> win(fft_n);
                        for(int i=0;i<fft_n;i++){
                            double x=2.0*M_PI*i/(fft_n-1);
                            win[i]=(float)(0.35875-0.48829*cos(x)+0.14128*cos(2*x)-0.01168*cos(3*x));
                        }
                        fftwf_complex* fin=(fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*fft_n);
                        fftwf_complex* fout=(fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*fft_n);
                        fftwf_plan plan=fftwf_plan_dft_1d(fft_n,fin,fout,FFTW_FORWARD,FFTW_ESTIMATE);
                        int hop=fft_n/2; if(hop<1)hop=1;
                        for(int64_t seg=pw0; seg+fft_n<=pw1; seg+=hop){
                            for(int i=0;i<fft_n;i++){
                                float ri=v.eid_ch_i[seg+i], rq=v.eid_ch_q[seg+i];
                                float zr=ri, zi=rq;
                                for(int p=1;p<M;p++){
                                    float nr=zr*ri-zi*rq;
                                    float ni=zr*rq+zi*ri;
                                    zr=nr; zi=ni;
                                }
                                fin[i][0]=zr*win[i];
                                fin[i][1]=zi*win[i];
                            }
                            fftwf_execute(plan);
                            for(int i=0;i<fft_n;i++){
                                float mag=fout[i][0]*fout[i][0]+fout[i][1]*fout[i][1];
                                psd[i]+=mag;
                            }
                            n_avg++;
                        }
                        fftwf_destroy_plan(plan);
                        fftwf_free(fin);
                        fftwf_free(fout);
                    }

                    // 평균 & dB 변환
                    float psd_max=-999.f;
                    std::vector<float> psd_db(fft_n);
                    for(int i=0;i<fft_n;i++){
                        float val=n_avg>0? psd[i]/n_avg : 1e-20f;
                        psd_db[i]=10.f*log10f(val+1e-20f);
                        if(psd_db[i]>psd_max) psd_max=psd_db[i];
                    }
                    psd_max+=10.f;
                    float psd_min=psd_max-80.f;

                    // 배경
                    fg->AddRectFilled(ImVec2(ea_x0,ea_y0),ImVec2(ea_x1,ea_y1),IM_COL32(8,8,12,255));

                    // X축 격자 & 라벨
                    int n_grid=8;
                    for(int g=0;g<=n_grid;g++){
                        float frac=(float)g/n_grid;
                        float gx=ea_x0+frac*ea_w;
                        fg->AddLine(ImVec2(gx,ea_y0),ImVec2(gx,ea_y1),IM_COL32(40,40,55,100));
                        float bin_freq=((float)g/n_grid-0.5f)*(float)sr;
                        char flbl[32];
                        if(fabsf(bin_freq)>=1000.f)
                            snprintf(flbl,sizeof(flbl),"%.1fk",bin_freq/1000.f);
                        else
                            snprintf(flbl,sizeof(flbl),"%.0f",bin_freq);
                        ImVec2 tsz=ImGui::CalcTextSize(flbl);
                        fg->AddText(ImVec2(gx-tsz.x*0.5f,ea_y1+2),IM_COL32(140,140,160,255),flbl);
                    }
                    // Y축 격자 (dB)
                    for(int g=0;g<=4;g++){
                        float frac=(float)g/4;
                        float gy=ea_y0+frac*ea_h;
                        fg->AddLine(ImVec2(ea_x0,gy),ImVec2(ea_x1,gy),IM_COL32(40,40,55,100));
                        float db_val=psd_max-(psd_max-psd_min)*frac;
                        char dlbl[16]; snprintf(dlbl,sizeof(dlbl),"%.0fdB",db_val);
                        ImVec2 tsz=ImGui::CalcTextSize(dlbl);
                        fg->AddText(ImVec2(ea_x0-tsz.x-4,gy-tsz.y*0.5f),IM_COL32(140,140,160,255),dlbl);
                    }

                    // PSD 그리기 (FFT shift: DC 중앙)
                    fg->PushClipRect(ImVec2(ea_x0,ea_y0),ImVec2(ea_x1,ea_y1),true);
                    float db_range=psd_max-psd_min;
                    if(db_range<1.f) db_range=1.f;
                    ImVec2 prev_pt;
                    for(int i=0;i<fft_n;i++){
                        int si=(i+fft_n/2)%fft_n;
                        float fx=ea_x0+((float)i/(fft_n-1))*ea_w;
                        float fy=ea_y0+(1.f-(psd_db[si]-psd_min)/db_range)*ea_h;
                        fy=std::max(ea_y0,std::min(ea_y1,fy));
                        ImVec2 pt(fx,fy);
                        if(i>0)
                            fg->AddLine(prev_pt,pt,IM_COL32(255,100,100,220),1.2f);
                        prev_pt=pt;
                    }
                    fg->PopClipRect();
                    fg->AddRect(ImVec2(ea_x0,ea_y0),ImVec2(ea_x1,ea_y1),IM_COL32(60,60,80,255));

                    // ── 하단 컨트롤: M 버튼 + FFT size 콤보 ──
                    float ctrl_y=ea_y1+20.f;
                    ImGui::SetCursorScreenPos(ImVec2(ea_x0,ctrl_y));

                    // M 값 선택 버튼
                    ImGui::Text("M:"); ImGui::SameLine(0,4);
                    for(int mv : {1,2,4,8}){
                        char ml[8]; snprintf(ml,sizeof(ml),"%d",mv);
                        bool sel=(v.eid_power_order==mv);
                        if(sel) ImGui::PushStyleColor(ImGuiCol_Button,ImVec4(0.6f,0.2f,0.2f,1.f));
                        if(ImGui::SmallButton(ml)) v.eid_power_order=mv;
                        if(sel) ImGui::PopStyleColor();
                        ImGui::SameLine(0,4);
                    }
                    ImGui::SameLine(0,16);

                    // FFT size 콤보
                    ImGui::Text("FFT:"); ImGui::SameLine(0,4);
                    {
                        const int fft_sizes[]={256,512,1024,2048,4096,8192,16384,32768,65536};
                        const char* fft_labels[]={"256","512","1024","2048","4096","8192","16384","32768","65536"};
                        int cur_idx=4; // default 4096
                        for(int i=0;i<9;i++) if(fft_sizes[i]==v.eid_power_fft_n) cur_idx=i;
                        ImGui::SetNextItemWidth(80.f);
                        if(ImGui::BeginCombo("##pw_fft",fft_labels[cur_idx],ImGuiComboFlags_NoArrowButton)){
                            for(int i=0;i<9;i++){
                                bool is_sel=(i==cur_idx);
                                if(ImGui::Selectable(fft_labels[i],is_sel))
                                    v.eid_power_fft_n=fft_sizes[i];
                                if(is_sel) ImGui::SetItemDefaultFocus();
                            }
                            ImGui::EndCombo();
                        }
                    }

                    // 뷰 범위 안내
                    ImGui::SameLine(0,16);
                    {
                        char vinfo[128];
                        if(pwin<fft_n)
                            snprintf(vinfo,sizeof(vinfo),"View: %lld samp (need >= %d, zoom out)",(long long)pwin,fft_n);
                        else
                            snprintf(vinfo,sizeof(vinfo),"View: %lld samp (%d avg)",(long long)pwin,n_avg);
                        ImGui::TextDisabled("%s",vinfo);
                    }

                    // 마우스 호버 시 주파수 표시
                    ImVec2 mpos=io.MousePos;
                    if(mpos.x>=ea_x0&&mpos.x<=ea_x1&&mpos.y>=ea_y0&&mpos.y<=ea_y1){
                        float frac=(mpos.x-ea_x0)/ea_w;
                        float hz=(frac-0.5f)*(float)sr;
                        float real_hz=hz/(float)M;
                        int bin=(int)(frac*fft_n);
                        bin=std::max(0,std::min(fft_n-1,bin));
                        int sbin=(bin+fft_n/2)%fft_n;
                        float db=psd_db[sbin];
                        char tip[128];
                        snprintf(tip,sizeof(tip),"%.1f Hz (real: %.1f Hz)  %.1f dB",hz,real_hz,db);
                        fg->AddText(ImVec2(mpos.x+12,mpos.y-18),IM_COL32(255,255,200,255),tip);
                        fg->AddLine(ImVec2(mpos.x,ea_y0),ImVec2(mpos.x,ea_y1),
                            IM_COL32(255,200,100,120));
                    }
                }

            // ── Time-domain 모드 (Amp/Freq/Phase/IQ: mode 1-4) ──────────
            } else if(eid_mode <= 4 && v.eid_data_ready.load()){
                const float LM = 60.f, RM = 10.f, TM = 24.f, BM = 30.f;
                float ea_x0 = ov_x0 + LM, ea_y0 = ca_y0 + TM;
                float ea_x1 = ov_x1 - RM, ea_y1 = ca_y1 - BM;
                float ea_w = ea_x1 - ea_x0, ea_h = ea_y1 - ea_y0;
                if(ea_w > 10.f && ea_h > 10.f){

                // 파일 정보 헤더
                {
                    uint32_t sr = v.eid_sample_rate > 0 ? v.eid_sample_rate : 1;
                    double visible_dur = (v.eid_view_t1 - v.eid_view_t0) / sr;
                    char hdr[256];
                    if(v.eid_center_freq_hz > 0)
                        snprintf(hdr, sizeof(hdr), "Duration: %.4fs | CF: %.3f MHz", visible_dur, v.eid_center_freq_hz/1e6);
                    else
                        snprintf(hdr, sizeof(hdr), "Duration: %.4fs", visible_dur);
                    fg->AddText(ImVec2(ea_x0, ca_y0 + 4), IM_COL32(160,160,180,220), hdr);
                    // 선택된 태그 PRI/PRF 표시
                    float hdr_next_x = ea_x0 + ImGui::CalcTextSize(hdr).x + 12.f;
                    for(auto& tag : v.eid_tags){
                        if(!tag.selected) continue;
                        double tdur = (tag.s1-tag.s0) / (double)sr;
                        double tdur_us = tdur * 1e6;
                        double trate = (tdur > 0) ? 1.0/tdur : 0;
                        char ti[128]; snprintf(ti,sizeof(ti),"| PRI:%.2fus PRF:%.0fHz PD:%.2fus Baud:%.0f",
                                               tdur_us, trate, tdur_us, trate);
                        fg->AddText(ImVec2(hdr_next_x, ca_y0+4), IM_COL32(255,200,100,255), ti);
                        hdr_next_x += ImGui::CalcTextSize(ti).x + 10.f;
                        // 자동 분석 결과
                        if(tag.auto_pulse_count >= 2){
                            char ai[96]; snprintf(ai,sizeof(ai),"[Auto] PRI:%.2fus PRF:%.0fHz Pulses:%d",
                                                  tag.auto_pri_us, tag.auto_prf_hz, tag.auto_pulse_count);
                            fg->AddText(ImVec2(hdr_next_x, ca_y0+4), IM_COL32(100,220,255,255), ai);
                            hdr_next_x += ImGui::CalcTextSize(ai).x + 10.f;
                        }
                        break; // 첫 번째 선택 태그만
                    }
                    // Baud 모드 표시
                    if(v.eid_baud_mode && v.eid_baud_s0>=0 && v.eid_baud_s1>=0){
                        double baud_interval=v.eid_baud_s1-v.eid_baud_s0;
                        if(baud_interval>0){
                            double baud_rate=(double)sr/baud_interval;
                            char bi[64]; snprintf(bi,sizeof(bi),"| Baud: %.0f",baud_rate);
                            fg->AddText(ImVec2(hdr_next_x,ca_y0+4),IM_COL32(255,255,0,255),bi);
                            hdr_next_x+=ImGui::CalcTextSize(bi).x+10.f;
                        }
                    }
                    if(!v.sa_temp_path.empty()){
                        const char* fn=v.sa_temp_path.c_str();
                        const char* sep=strrchr(fn,'/'); if(sep) fn=sep+1;
                        ImVec2 fsz=ImGui::CalcTextSize(fn);
                        fg->AddText(ImVec2(ea_x1-fsz.x, ca_y0+4), IM_COL32(160,160,180,220), fn);
                    }
                }

                // 배경
                fg->AddRectFilled(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), IM_COL32(8,8,12,255));
                fg->AddRect(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), IM_COL32(60,60,80,255));

                // 내부 모드 매핑: eid_view_mode 1=Amp, 2=Freq, 3=Phase, 4=I/Q
                // > 내부: 0=envelope, 1=I/Q, 2=Phase, 3=Freq
                int imode;
                if(eid_mode == 1) imode = 0;      // Amp > envelope
                else if(eid_mode == 2) imode = 3;  // Freq > inst_freq
                else if(eid_mode == 3) imode = 2;  // Phase > phase
                else imode = 1;                     // I/Q > ch_i/ch_q

                double vt0 = v.eid_view_t0, vt1 = v.eid_view_t1;

                // Y축 범위: 수동 스케일 사용 (Ctrl+휠로 조절)
                float a_min = v.eid_y_min[imode];
                float a_max = v.eid_y_max[imode];
                float a_rng = a_max - a_min;
                if(a_rng < 1e-6f) a_rng = 1e-6f;
                double vis_samp = vt1 - vt0;
                if(vis_samp < 1.0) vis_samp = 1.0;
                int pixels = (int)ea_w;
                if(pixels < 1) pixels = 1;
                double spp = vis_samp / pixels;

                // Y축 그리드
                {
                    int n_divs = std::max(2, std::min(10, (int)(ea_h / 40.f)));
                    for(int i = 0; i <= n_divs; i++){
                        float frac = (float)i / n_divs;
                        float yy = ea_y0 + frac * ea_h;
                        float amp_val = a_max - frac * a_rng;
                        char lbl[32];
                        if(imode == 0) snprintf(lbl, sizeof(lbl), "%.3f", amp_val);
                        else if(imode == 2) snprintf(lbl, sizeof(lbl), "%.0f\xc2\xb0", amp_val*(180.f/(float)M_PI));
                        else if(imode == 3){
                            float fabs_val = fabsf(amp_val);
                            if(fabs_val >= 1e6f) snprintf(lbl, sizeof(lbl), "%.3fMHz", amp_val/1e6f);
                            else snprintf(lbl, sizeof(lbl), "%.1fkHz", amp_val/1e3f);
                        }
                        else snprintf(lbl, sizeof(lbl), "%.3f", amp_val);
                        ImVec2 tsz = ImGui::CalcTextSize(lbl);
                        fg->AddText(ImVec2(ea_x0 - tsz.x - 4, yy - tsz.y * 0.5f), IM_COL32(130,130,160,255), lbl);
                    }
                }

                // X축 그리드 (시간)
                {
                    uint32_t sr = v.eid_sample_rate; if(sr == 0) sr = 1;
                    double t0s = vt0 / sr, t1s = vt1 / sr, dts = t1s - t0s;
                    const char* unit = "s"; double ud = 1.0;
                    if(dts < 0.001){ unit = "us"; ud = 1e-6; }
                    else if(dts < 1.0){ unit = "ms"; ud = 1e-3; }
                    double ru = dts / ud;
                    double rs = ru / std::max(2.0, (double)(pixels / 120));
                    double mg = pow(10.0, floor(log10(rs)));
                    double nm = rs / mg;
                    double ns = (nm<=1.0)?1.0*mg : (nm<=2.0)?2.0*mg : (nm<=5.0)?5.0*mg : 10.0*mg;
                    double su = ceil((t0s/ud)/ns)*ns;
                    for(double tu = su; tu*ud <= t1s + ns*0.5; tu += ns){
                        double ts = tu * ud;
                        float xx = ea_x0 + (float)((ts*sr - vt0) / vis_samp) * ea_w;
                        if(xx < ea_x0 || xx > ea_x1) continue;
                        char lbl[32]; snprintf(lbl, sizeof(lbl), "%.4g%s", tu, unit);
                        ImVec2 tsz = ImGui::CalcTextSize(lbl);
                        fg->AddText(ImVec2(xx - tsz.x*0.5f, ea_y1+4), IM_COL32(130,130,160,255), lbl);
                    }
                }

                // Phase detrend slope (rad/sample)
                float detrend_slope = 0.0f;
                if(imode == 2 && v.eid_phase_detrend_hz != 0.0f && v.eid_sample_rate > 0)
                    detrend_slope = 2.0f * (float)M_PI * v.eid_phase_detrend_hz / (float)v.eid_sample_rate;
                auto wrap_pi = [](float x) -> float {
                    x = fmodf(x + (float)M_PI, 2.0f * (float)M_PI);
                    if(x < 0.0f) x += 2.0f * (float)M_PI;
                    return x - (float)M_PI;
                };

                // 파형 렌더링
                fg->PushClipRect(ImVec2(ea_x0, ea_y0), ImVec2(ea_x1, ea_y1), true);
                {
                    struct ECh { const std::vector<float>* d; ImU32 c; };
                    ECh chs[2]; int nch = 0;
                    if(imode == 0){ chs[0]={&v.eid_envelope, IM_COL32(80,255,140,255)}; nch=1; }
                    else if(imode == 1){ chs[0]={&v.eid_ch_i, IM_COL32(80,255,140,255)};
                                         chs[1]={&v.eid_ch_q, IM_COL32(80,140,255,255)}; nch=2; }
                    else if(imode == 2){ chs[0]={&v.eid_phase, IM_COL32(255,200,80,255)}; nch=1; }
                    else { chs[0]={&v.eid_inst_freq, IM_COL32(255,120,200,255)}; nch=1; }

                    for(int ci = 0; ci < nch; ci++){
                        const auto& dat = *chs[ci].d;
                        ImU32 col = chs[ci].c;
                        int64_t total = (int64_t)dat.size();
                        if(total < 1) continue;
                        if(spp <= 1.0){
                            std::vector<ImVec2> pts;
                            int64_t s0 = std::max((int64_t)0, (int64_t)vt0);
                            int64_t s1 = std::min(total, (int64_t)ceil(vt1)+1);
                            pts.reserve(s1-s0);
                            for(int64_t s = s0; s < s1; s++){
                                float val = dat[s];
                                if(detrend_slope != 0.0f)
                                    val = wrap_pi(val - detrend_slope * (float)s);
                                float xx = ea_x0 + (float)((s-vt0)/vis_samp)*ea_w;
                                float yy = ea_y0 + (1.0f-(val-a_min)/a_rng)*ea_h;
                                pts.push_back(ImVec2(xx,yy));
                            }
                            if(pts.size()>=2)
                                fg->AddPolyline(pts.data(),(int)pts.size(),col,ImDrawFlags_None,1.5f);
                        } else {
                            ImU32 bc=(col&0x00FFFFFF)|0xC8000000;
                            float prev_lo=FLT_MAX, prev_hi=-FLT_MAX;
                            for(int px = 0; px < pixels; px++){
                                int64_t s0=(int64_t)(vt0+px*spp), s1=(int64_t)(vt0+(px+1)*spp);
                                s0=std::max((int64_t)0,std::min(s0,total-1));
                                s1=std::max(s0+1,std::min(s1,total));
                                float iv=dat[s0];
                                if(detrend_slope!=0.0f) iv=wrap_pi(iv-detrend_slope*(float)s0);
                                float lo=iv, hi=iv;
                                for(int64_t s=s0+1;s<s1;s++){
                                    float v2=dat[s];
                                    if(detrend_slope!=0.0f) v2=wrap_pi(v2-detrend_slope*(float)s);
                                    if(v2<lo)lo=v2; if(v2>hi)hi=v2;
                                }
                                // 이전 픽셀과 gap이 생기면 브릿지 (원본 lo/hi 기준)
                                float draw_lo=lo, draw_hi=hi;
                                if(prev_lo!=FLT_MAX){
                                    if(prev_hi<lo) draw_lo=prev_hi;
                                    else if(prev_lo>hi) draw_hi=prev_lo;
                                }
                                prev_lo=lo; prev_hi=hi;
                                float xx=ea_x0+px;
                                float yl=ea_y0+(1.0f-(draw_hi-a_min)/a_rng)*ea_h;
                                float yh=ea_y0+(1.0f-(draw_lo-a_min)/a_rng)*ea_h;
                                if(yl>yh) std::swap(yl,yh);
                                if(yh-yl<1.0f) yh=yl+1.0f;
                                fg->AddLine(ImVec2(xx,yl),ImVec2(xx,yh),bc);
                            }
                        }
                    }

                    // 노이즈 레벨 라인 (Amp 모드만)
                    if(imode == 0 && v.eid_noise_level > a_min && v.eid_noise_level < a_max){
                        float ny = ea_y0 + (1.0f - (v.eid_noise_level - a_min) / a_rng) * ea_h;
                        for(float nx = ea_x0; nx < ea_x1; nx += 8.f)
                            fg->AddLine(ImVec2(nx,ny), ImVec2(std::min(nx+4.f,ea_x1),ny), IM_COL32(255,60,60,150));
                        char nl[24]; snprintf(nl, sizeof(nl), "Noise %.4f", v.eid_noise_level);
                        fg->AddText(ImVec2(ea_x1-ImGui::CalcTextSize(nl).x-4, ny-14), IM_COL32(255,60,60,180), nl);
                    }

                    // 태그 영역
                    for(auto& tag : v.eid_tags){
                        float tx0 = ea_x0 + (float)((tag.s0-vt0)/vis_samp)*ea_w;
                        float tx1 = ea_x0 + (float)((tag.s1-vt0)/vis_samp)*ea_w;
                        tx0=std::max(tx0,ea_x0); tx1=std::min(tx1,ea_x1);
                        if(tx1<=tx0) continue;
                        ImU32 fill_alpha = tag.selected ? 0x50000000 : 0x28000000;
                        fg->AddRectFilled(ImVec2(tx0,ea_y0),ImVec2(tx1,ea_y1),(tag.color&0x00FFFFFF)|fill_alpha);
                        fg->AddLine(ImVec2(tx0,ea_y0),ImVec2(tx0,ea_y1),tag.color,tag.selected?2.5f:1.5f);
                        fg->AddLine(ImVec2(tx1,ea_y0),ImVec2(tx1,ea_y1),tag.color,tag.selected?2.5f:1.5f);
                        { ImVec2 lsz=ImGui::CalcTextSize(tag.label);
                          float cx=tx0+(tx1-tx0-lsz.x)*0.5f; cx=std::max(cx,tx0+2.f);
                          fg->AddText(ImVec2(cx,ea_y0+2),tag.color,tag.label); }
                    }
                }

                // 비트 구분 격자 (baud mode)
                if(v.eid_baud_mode && v.eid_baud_s0>=0){
                    float bx0=ea_x0+(float)((v.eid_baud_s0-vt0)/vis_samp)*ea_w;
                    // s1 설정 시 격자 + 밴드 채우기 + s1 선
                    if(v.eid_baud_s1>=0){
                        double interval=v.eid_baud_s1-v.eid_baud_s0;
                        if(interval>0){
                            float bx1=ea_x0+(float)((v.eid_baud_s1-vt0)/vis_samp)*ea_w;
                            // 두 메인 선 사이 채우기 (드래그 가능 영역 표시)
                            float rx0=std::max(bx0,ea_x0), rx1=std::min(bx1,ea_x1);
                            if(rx1>rx0)
                                fg->AddRectFilled(ImVec2(rx0,ea_y0),ImVec2(rx1,ea_y1),IM_COL32(255,255,0,28));
                            // 격자
                            for(double s=v.eid_baud_s0+interval; s<=vt1; s+=interval){
                                float xx=ea_x0+(float)((s-vt0)/vis_samp)*ea_w;
                                if(xx>=ea_x0&&xx<=ea_x1)
                                    fg->AddLine(ImVec2(xx,ea_y0),ImVec2(xx,ea_y1),IM_COL32(255,255,0,80),1.f);
                            }
                            for(double s=v.eid_baud_s0-interval; s>=vt0; s-=interval){
                                float xx=ea_x0+(float)((s-vt0)/vis_samp)*ea_w;
                                if(xx>=ea_x0&&xx<=ea_x1)
                                    fg->AddLine(ImVec2(xx,ea_y0),ImVec2(xx,ea_y1),IM_COL32(255,255,0,80),1.f);
                            }
                            if(bx1>=ea_x0&&bx1<=ea_x1)
                                fg->AddLine(ImVec2(bx1,ea_y0),ImVec2(bx1,ea_y1),IM_COL32(255,255,0,255),2.f);
                        }
                    }
                    // s0 선 (s1보다 위에 그려 항상 보이게)
                    if(bx0>=ea_x0&&bx0<=ea_x1)
                        fg->AddLine(ImVec2(bx0,ea_y0),ImVec2(bx0,ea_y1),IM_COL32(255,255,0,255),2.f);
                }

                // 비트 판단선 렌더링 (baud mode + baseline 설정 시)
                if(v.eid_baud_mode && v.eid_baseline_active){
                    float a_rng_ = v.eid_y_max[imode] - v.eid_y_min[imode];
                    if(a_rng_ > 1e-9f){
                        float by = ea_y0 + (1.f - (v.eid_baseline_val - v.eid_y_min[imode]) / a_rng_) * ea_h;
                        if(by >= ea_y0 && by <= ea_y1){
                            fg->AddLine(ImVec2(ea_x0, by), ImVec2(ea_x1, by), IM_COL32(255,80,80,220), 1.5f);
                            fg->AddText(ImVec2(ea_x0+4, by-14), IM_COL32(255,80,80,220), "Baseline");
                        }
                    }
                }

                fg->PopClipRect();

                // 마우스 인터랙션
                ImVec2 mp = io.MousePos;
                bool mouse_in = (mp.x >= ea_x0 && mp.x < ea_x1 && mp.y >= ea_y0 && mp.y < ea_y1);

                // eid_view > sa_view 역동기화 헬퍼
                auto sync_eid_to_sa = [&](){
                    double et = (double)v.eid_total_samples;
                    if(et > 0){
                        v.sa_view_y0 = (float)(v.eid_view_t0 / et);
                        v.sa_view_y1 = (float)(v.eid_view_t1 / et);
                        v.sa_view_y0 = std::max(0.f, std::min(1.f, v.sa_view_y0));
                        v.sa_view_y1 = std::max(0.f, std::min(1.f, v.sa_view_y1));
                    }
                };

                // 스크롤 줌 (마우스 위치 고정) / Ctrl+휠 = Y축 줌
                if(mouse_in && io.MouseWheel != 0.f){
                    if(io.KeyCtrl){
                        // Y축 줌: 마우스 Y값 기준
                        float a_rng_ = v.eid_y_max[imode] - v.eid_y_min[imode];
                        if(a_rng_ < 1e-9f) a_rng_ = 1e-9f;
                        float mouse_val = v.eid_y_max[imode] - ((mp.y - ea_y0) / ea_h) * a_rng_;
                        float frac_y = (mouse_val - v.eid_y_min[imode]) / a_rng_;
                        float zf = (io.MouseWheel > 0) ? 0.8f : 1.25f;
                        float new_rng = a_rng_ * zf;
                        if(new_rng < 1e-9f) new_rng = 1e-9f;
                        v.eid_y_min[imode] = mouse_val - frac_y * new_rng;
                        v.eid_y_max[imode] = mouse_val + (1.f - frac_y) * new_rng;
                    } else {
                        v.eid_view_stack.push_back({vt0,vt1});
                        double zf = (io.MouseWheel > 0) ? 0.8 : 1.25;
                        double frac = (double)(mp.x-ea_x0)/ea_w;
                        double mt = vt0 + frac * vis_samp;
                        double new_vis = vis_samp * zf;
                        v.eid_view_t0 = mt - frac * new_vis;
                        v.eid_view_t1 = mt + (1.0 - frac) * new_vis;
                        if(v.eid_view_t0<0){ v.eid_view_t1-=v.eid_view_t0; v.eid_view_t0=0; }
                        if(v.eid_view_t1>(double)v.eid_total_samples){
                            v.eid_view_t0-=(v.eid_view_t1-(double)v.eid_total_samples);
                            v.eid_view_t1=(double)v.eid_total_samples;
                        }
                        v.eid_view_t0=std::max(0.0,v.eid_view_t0);
                        v.eid_view_t1=std::min((double)v.eid_total_samples,v.eid_view_t1);
                        if(v.eid_view_t1-v.eid_view_t0<32.0){
                            double mid=(v.eid_view_t0+v.eid_view_t1)*0.5;
                            v.eid_view_t0=mid-16; v.eid_view_t1=mid+16;
                        }
                        sync_eid_to_sa();
                    }
                }

                // (태그 토글은 좌클릭 드래그 release에서 처리 — 드래그<5px일 때)

                // Ctrl+좌클릭 드래그: Y축 영역 선택 (추후 기능 확장용)
                static bool eid_ctrl_drag = false;
                static float eid_ctrl_sy0 = 0, eid_ctrl_sy1 = 0;
                if(mouse_in && io.KeyCtrl && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                    eid_ctrl_drag=true; eid_ctrl_sy0=mp.y; eid_ctrl_sy1=mp.y;
                }
                if(eid_ctrl_drag){
                    if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                        eid_ctrl_sy1=mp.y;
                        float sy0=std::max(ea_y0,std::min(eid_ctrl_sy0,eid_ctrl_sy1));
                        float sy1=std::min(ea_y1,std::max(eid_ctrl_sy0,eid_ctrl_sy1));
                        if(sy1-sy0>1.f){
                            fg->AddRectFilled(ImVec2(ea_x0,sy0),ImVec2(ea_x1,sy1),IM_COL32(100,200,255,40));
                            fg->AddRect(ImVec2(ea_x0,sy0),ImVec2(ea_x1,sy1),IM_COL32(100,200,255,160),0.f,0,1.f);
                            float val_top=a_max-((sy0-ea_y0)/ea_h)*a_rng;
                            float val_bot=a_max-((sy1-ea_y0)/ea_h)*a_rng;
                            char yi[96];
                            if(imode==0){ // Amp: 두 지점의 실제 dB 차
                                float db_top = (val_top > 0.f) ? 20.f*log10f(val_top) : -120.f;
                                float db_bot = (val_bot > 0.f) ? 20.f*log10f(val_bot) : -120.f;
                                float db_rng = fabsf(db_top - db_bot);
                                snprintf(yi,sizeof(yi),"Amp: %.2f dB",db_rng);
                            } else if(imode==1){ // I/Q: amplitude range
                                float rng=fabsf(val_top-val_bot);
                                snprintf(yi,sizeof(yi),"I/Q: %.4f ~ %.4f  |  \xce\x94: %.4f",val_bot,val_top,rng);
                            } else if(imode==2){ // Phase: delta degrees only
                                float deg_top=val_top*(180.f/(float)M_PI);
                                float deg_bot=val_bot*(180.f/(float)M_PI);
                                float deg_rng=fabsf(deg_top-deg_bot);
                                snprintf(yi,sizeof(yi),"Phase: %.1f\xc2\xb0",deg_rng);
                            } else { // Freq: BW + Resolution
                                float bw_hz=fabsf(val_top-val_bot);
                                float res_m=(bw_hz>0)?299792458.0f/(2.0f*bw_hz):0;
                                if(bw_hz>=1e6f) snprintf(yi,sizeof(yi),"BW: %.2f MHz | Res: %.1fm",bw_hz/1e6f,res_m);
                                else if(bw_hz>=1e3f) snprintf(yi,sizeof(yi),"BW: %.1f kHz | Res: %.1fm",bw_hz/1e3f,res_m);
                                else snprintf(yi,sizeof(yi),"BW: %.1f Hz | Res: %.1fm",bw_hz,res_m);
                            }
                            ImVec2 tsz=ImGui::CalcTextSize(yi);
                            float tx2=ea_x0+(ea_w-tsz.x)*0.5f;
                            float ty2=sy0+(sy1-sy0-tsz.y)*0.5f;
                            fg->AddRectFilled(ImVec2(tx2-2,ty2-1),ImVec2(tx2+tsz.x+2,ty2+tsz.y+1),IM_COL32(0,0,0,180),3.f);
                            fg->AddText(ImVec2(tx2,ty2),IM_COL32(100,200,255,255),yi);
                        }
                    } else {
                        eid_ctrl_drag=false;
                        // 추후 기능 확장용 - 현재는 시각적 선택만
                    }
                }

                // 좌클릭 드래그 줌 (baud 모드가 아닐 때만)
                if(mouse_in && !io.KeyCtrl && !v.eid_baud_mode && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                    v.eid_sel_active=true; v.eid_sel_x0=mp.x; v.eid_sel_x1=mp.x;
                }
                if(v.eid_sel_active){
                    if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                        v.eid_sel_x1=mp.x;
                        float sx0=std::max(ea_x0,std::min(v.eid_sel_x0,v.eid_sel_x1));
                        float sx1=std::min(ea_x1,std::max(v.eid_sel_x0,v.eid_sel_x1));
                        if(sx1-sx0>1.f){
                            fg->AddRectFilled(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,50));
                            fg->AddRect(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,160),0.f,0,1.f);
                            double ss0=vt0+((sx0-ea_x0)/ea_w)*vis_samp;
                            double ss1=vt0+((sx1-ea_x0)/ea_w)*vis_samp;
                            int64_t ns=(int64_t)ss1-(int64_t)ss0;
                            uint32_t sr=v.eid_sample_rate>0?v.eid_sample_rate:1;
                            char si[64]; snprintf(si,sizeof(si),"Sel: %lld samp | %.3fms",(long long)ns,(double)ns/sr*1000);
                            ImVec2 tsz2=ImGui::CalcTextSize(si);
                            float tx=sx0+(sx1-sx0-tsz2.x)*0.5f; tx=std::max(tx,sx0+2.f);
                            fg->AddRectFilled(ImVec2(tx-2,ea_y0+2),ImVec2(tx+tsz2.x+2,ea_y0+tsz2.y+4),IM_COL32(0,0,0,180),3.f);
                            fg->AddText(ImVec2(tx,ea_y0+3),IM_COL32(200,220,255,255),si);
                        }
                    } else {
                        v.eid_sel_active=false;
                        float sx0=std::min(v.eid_sel_x0,v.eid_sel_x1);
                        float sx1=std::max(v.eid_sel_x0,v.eid_sel_x1);
                        if(sx1-sx0>5.f){
                            v.eid_view_stack.push_back({vt0,vt1});
                            v.eid_view_t0=std::max(0.0,vt0+((sx0-ea_x0)/ea_w)*vis_samp);
                            v.eid_view_t1=std::min((double)v.eid_total_samples,vt0+((sx1-ea_x0)/ea_w)*vis_samp);
                            if(v.eid_view_t1-v.eid_view_t0<32.0){
                                double mid=(v.eid_view_t0+v.eid_view_t1)*0.5;
                                v.eid_view_t0=mid-16; v.eid_view_t1=mid+16;
                            }
                            sync_eid_to_sa();
                        } else {
                            // 드래그<5px = 클릭 → 태그 선택 토글
                            double click_s=vt0+((mp.x-ea_x0)/ea_w)*vis_samp;
                            for(auto& tag : v.eid_tags)
                                if(click_s>=tag.s0&&click_s<=tag.s1){ tag.selected=!tag.selected; break; }
                        }
                    }
                }

                // 우클릭: 드래그>임시 선택 영역, 클릭>컨텍스트 메뉴
                if(mouse_in && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !eid_tag_ctx.open){
                    v.eid_tag_dragging=true; v.eid_tag_drag_x0=mp.x; v.eid_tag_drag_x1=mp.x;
                }
                if(v.eid_tag_dragging && ImGui::IsMouseDown(ImGuiMouseButton_Right)){
                    v.eid_tag_drag_x1=mp.x;
                    float sx0=std::max(ea_x0,std::min(v.eid_tag_drag_x0,v.eid_tag_drag_x1));
                    float sx1=std::min(ea_x1,std::max(v.eid_tag_drag_x0,v.eid_tag_drag_x1));
                    if(sx1-sx0>1.f){
                        fg->AddRectFilled(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,40));
                        fg->AddRect(ImVec2(sx0,ea_y0),ImVec2(sx1,ea_y1),IM_COL32(200,200,220,160),0.f,0,1.f);
                    }
                }
                if(v.eid_tag_dragging && ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
                    v.eid_tag_dragging=false;
                    float dx=fabsf(v.eid_tag_drag_x1-v.eid_tag_drag_x0);
                    if(dx>5.f){
                        // 임시 선택 영역으로 저장
                        float sx0=std::max(ea_x0,std::min(v.eid_tag_drag_x0,v.eid_tag_drag_x1));
                        float sx1=std::min(ea_x1,std::max(v.eid_tag_drag_x0,v.eid_tag_drag_x1));
                        v.eid_pending_s0=std::max(0.0,vt0+((sx0-ea_x0)/ea_w)*vis_samp);
                        v.eid_pending_s1=std::min((double)v.eid_total_samples,vt0+((sx1-ea_x0)/ea_w)*vis_samp);
                        v.eid_pending_active=true;
                    } else {
                        // 짧은 클릭: 임시 영역 내부 → 컨텍스트 메뉴, 태그 → 태그 메뉴, 밖 → undo
                        double click_s=vt0+((mp.x-ea_x0)/ea_w)*vis_samp;
                        bool in_pending = v.eid_pending_active &&
                                          click_s>=v.eid_pending_s0 && click_s<=v.eid_pending_s1;
                        if(in_pending){
                            eid_tag_ctx.open=true; eid_tag_ctx.x=mp.x; eid_tag_ctx.y=mp.y;
                            eid_tag_ctx.tag_idx=-1; eid_tag_ctx.renaming=false;
                            eid_tag_ctx.is_pending=true;
                        } else {
                            int hit_tag=-1;
                            for(int ti=0;ti<(int)v.eid_tags.size();ti++)
                                if(click_s>=v.eid_tags[ti].s0&&click_s<=v.eid_tags[ti].s1){ hit_tag=ti; break; }
                            if(hit_tag>=0){
                                eid_tag_ctx.open=true; eid_tag_ctx.x=mp.x; eid_tag_ctx.y=mp.y;
                                eid_tag_ctx.tag_idx=hit_tag; eid_tag_ctx.renaming=false;
                                eid_tag_ctx.is_pending=false;
                                strncpy(eid_tag_ctx.rename_buf,v.eid_tags[hit_tag].label,31);
                            } else {
                                // 단순 클릭 빈 영역: 아무 동작 없음 (Ctrl+Z로 undo)
                                v.eid_pending_active=false;
                            }
                        }
                    }
                }
                // 임시 선택 영역 렌더링 (드래그 중 아닐 때도 유지)
                if(v.eid_pending_active && !v.eid_tag_dragging){
                    float px0=ea_x0+((float)((v.eid_pending_s0-vt0)/vis_samp))*ea_w;
                    float px1=ea_x0+((float)((v.eid_pending_s1-vt0)/vis_samp))*ea_w;
                    px0=std::max(ea_x0,std::min(ea_x1,px0));
                    px1=std::max(ea_x0,std::min(ea_x1,px1));
                    if(px1>px0+1.f){
                        fg->AddRectFilled(ImVec2(px0,ea_y0),ImVec2(px1,ea_y1),IM_COL32(200,200,220,30));
                        fg->AddRect(ImVec2(px0,ea_y0),ImVec2(px1,ea_y1),IM_COL32(200,200,220,140),0.f,0,1.f);
                    }
                }

                // Home 키
                if(mouse_in && ImGui::IsKeyPressed(ImGuiKey_Home,false)){
                    v.eid_view_stack.clear(); v.eid_view_t0=0; v.eid_view_t1=(double)v.eid_total_samples;
                    sync_eid_to_sa();
                }

                // Delete 키 / 더블클릭: 임시 영역 또는 태그 삭제
                auto eid_delete_at = [&](double ms){
                    if(v.eid_pending_active && ms>=v.eid_pending_s0 && ms<=v.eid_pending_s1){
                        v.eid_push_undo();
                        v.eid_pending_active=false;
                    } else {
                        for(auto it=v.eid_tags.begin();it!=v.eid_tags.end();++it){
                            if(ms>=it->s0&&ms<=it->s1){ v.eid_push_undo(); v.eid_tags.erase(it); break; }
                        }
                    }
                };
                if(mouse_in && ImGui::IsKeyPressed(ImGuiKey_Delete,false)){
                    eid_delete_at(vt0+((mp.x-ea_x0)/ea_w)*vis_samp);
                }
                // 좌우 방향키: 현재 화면 폭만큼 팬
                if(mouse_in && !io.WantTextInput){
                    double span = vt1 - vt0;
                    double total = (double)v.eid_total_samples;
                    if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow, false)){
                        double t0 = vt0 - span;
                        if(t0 < 0) t0 = 0;
                        v.eid_view_t0 = t0;
                        v.eid_view_t1 = t0 + span;
                        sync_eid_to_sa();
                    }
                    if(ImGui::IsKeyPressed(ImGuiKey_RightArrow, false)){
                        double t1 = vt1 + span;
                        if(t1 > total) t1 = total;
                        v.eid_view_t0 = t1 - span;
                        v.eid_view_t1 = t1;
                        sync_eid_to_sa();
                    }
                }
                if(mouse_in && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                    eid_delete_at(vt0+((mp.x-ea_x0)/ea_w)*vis_samp);
                    v.eid_sel_active=false; // 더블클릭 시 줌 드래그 시작 방지
                }

                // B키: 비트 구분 모드 토글 (비트 판단선도 함께 초기화)
                if(mouse_in && ImGui::IsKeyPressed(ImGuiKey_B,false) && !io.WantTextInput){
                    v.eid_push_undo();
                    v.eid_baud_mode=!v.eid_baud_mode;
                    v.eid_baud_s0=-1; v.eid_baud_s1=-1; v.eid_baud_click=0;
                    v.eid_baud_drag=-1; v.eid_baud_drag_band=false;
                    v.eid_baseline_active=false;
                }

                // 비트 구분 모드: 좌클릭 인터랙션 (팝업 열린 동안 차단)
                if(v.eid_baud_mode && mouse_in && !io.KeyCtrl
                   && !ImGui::IsPopupOpen("##baud_ctx")){
                    const float SNAP_PX=6.f;
                    double mouse_s=vt0+((mp.x-ea_x0)/ea_w)*vis_samp;
                    uint32_t baud_sr=v.eid_sample_rate>0?v.eid_sample_rate:1;

                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)
                       && v.eid_baud_drag<0 && !v.eid_baud_drag_band){
                        // 1순위: 메인 선 근처 → 선 드래그
                        for(int li=0;li<2;li++){
                            double ls=(li==0)?v.eid_baud_s0:v.eid_baud_s1;
                            if(ls<0) continue;
                            float lx=ea_x0+(float)((ls-vt0)/vis_samp)*ea_w;
                            if(fabsf(mp.x-lx)<SNAP_PX){ v.eid_baud_drag=li; break; }
                        }
                        // 2순위: 밴드 안쪽 → 밴드 전체 드래그
                        if(v.eid_baud_drag<0 && v.eid_baud_s0>=0 && v.eid_baud_s1>=0){
                            float bx0b=ea_x0+(float)((v.eid_baud_s0-vt0)/vis_samp)*ea_w;
                            float bx1b=ea_x0+(float)((v.eid_baud_s1-vt0)/vis_samp)*ea_w;
                            if(mp.x>bx0b+SNAP_PX && mp.x<bx1b-SNAP_PX){
                                v.eid_baud_drag_band=true;
                                v.eid_baud_band_drag_offset=mouse_s-v.eid_baud_s0;
                            }
                        }
                        // 3순위: 새 점 설정
                        if(v.eid_baud_drag<0 && !v.eid_baud_drag_band){
                            if(v.eid_baud_click==0){
                                v.eid_baud_s0=mouse_s; v.eid_baud_s1=-1; v.eid_baud_click=1;
                            } else {
                                v.eid_baud_s1=mouse_s; v.eid_baud_click=2;
                                if(v.eid_baud_s1<v.eid_baud_s0) std::swap(v.eid_baud_s0,v.eid_baud_s1);
                            }
                        }
                    }
                    // 선 개별 드래그
                    if(v.eid_baud_drag>=0 && ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                        if(v.eid_baud_drag==0) v.eid_baud_s0=mouse_s;
                        else v.eid_baud_s1=mouse_s;
                        if(v.eid_baud_s0>v.eid_baud_s1) std::swap(v.eid_baud_s0,v.eid_baud_s1);
                    }
                    if(v.eid_baud_drag>=0 && ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                        v.eid_baud_drag=-1;
                    }
                    // 밴드 전체 드래그 (100-baud 스냅)
                    if(v.eid_baud_drag_band && ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                        double interval=v.eid_baud_s1-v.eid_baud_s0;
                        if(interval>0){
                            // 현재 baud를 100 단위로 스냅
                            double baud=(double)baud_sr/interval;
                            double snapped_baud=round(baud/100.0)*100.0;
                            if(snapped_baud<100.0) snapped_baud=100.0;
                            double snapped_interval=(double)baud_sr/snapped_baud;
                            double new_s0=mouse_s-v.eid_baud_band_drag_offset;
                            v.eid_baud_s0=new_s0;
                            v.eid_baud_s1=new_s0+snapped_interval;
                        }
                    }
                    if(v.eid_baud_drag_band && ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                        v.eid_baud_drag_band=false;
                    }
                    // 우클릭 컨텍스트 메뉴: Make Baseline
                    if(mouse_in && ImGui::IsMouseClicked(ImGuiMouseButton_Right)
                       && !v.eid_baud_drag_band && v.eid_baud_drag<0){
                        ImGui::OpenPopup("##baud_ctx");
                        // 클릭 시점의 Y값과 모드를 저장
                        float a_rng_ = a_max - a_min;
                        v.eid_baseline_val = a_max - ((mp.y - ea_y0) / ea_h) * a_rng_;
                        v.eid_baseline_imode = imode;
                    }
                }
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16, 8));
                if(ImGui::BeginPopup("##baud_ctx")){
                    if(ImGui::MenuItem("Make Baseline")){
                        v.eid_push_undo();
                        v.eid_baseline_active = true;
                    }
                    ImGui::EndPopup();
                }
                ImGui::PopStyleVar();

                // 커서 오버레이
                if(mouse_in){
                    fg->AddLine(ImVec2(mp.x,ea_y0),ImVec2(mp.x,ea_y1),IM_COL32(255,255,255,60));
                    fg->AddLine(ImVec2(ea_x0,mp.y),ImVec2(ea_x1,mp.y),IM_COL32(255,255,255,60));
                    double ms=vt0+((mp.x-ea_x0)/ea_w)*vis_samp;
                    float mv=a_max-((mp.y-ea_y0)/ea_h)*a_rng;
                    uint32_t sr=v.eid_sample_rate>0?v.eid_sample_rate:1;
                    double msec=ms/sr;
                    char i1[48],i2[48];
                    if(msec<0.001) snprintf(i1,sizeof(i1),"Time : %.3f us",msec*1e6);
                    else if(msec<1.0) snprintf(i1,sizeof(i1),"Time : %.4f ms",msec*1e3);
                    else snprintf(i1,sizeof(i1),"Time : %.6f s",msec);
                    if(imode==0) snprintf(i2,sizeof(i2),"Amp  : %.5f",mv);
                    else if(imode==1) snprintf(i2,sizeof(i2),"Val  : %.5f",mv);
                    else if(imode==2){
                        if(detrend_slope!=0.0f){
                            int64_t si=(int64_t)ms;
                            if(si>=0&&si<(int64_t)v.eid_phase.size())
                                mv=wrap_pi(v.eid_phase[si]-detrend_slope*(float)si);
                        }
                        float deg_mv = mv * (180.f / (float)M_PI);
                        snprintf(i2,sizeof(i2),"Phase: %.2f\xc2\xb0",deg_mv);
                    } else {
                        float fabs_mv = fabsf(mv);
                        if(fabs_mv >= 1e6f) snprintf(i2,sizeof(i2),"Freq : %.3f MHz",mv/1e6f);
                        else snprintf(i2,sizeof(i2),"Freq : %.3f kHz",mv/1e3f);
                    }
                    float fw=std::max(ImGui::CalcTextSize(i1).x,ImGui::CalcTextSize(i2).x);
                    float fh=ImGui::GetFontSize()*2.6f;
                    float ox=ea_x1-fw-14.f, oy=ea_y0+8.f;
                    fg->AddRectFilled(ImVec2(ox-4,oy-2),ImVec2(ox+fw+4,oy+fh),IM_COL32(0,0,0,180),4.f);
                    float lh=ImGui::GetFontSize()+2.f;
                    fg->AddText(ImVec2(ox,oy),IM_COL32(255,180,220,255),i1);
                    fg->AddText(ImVec2(ox,oy+lh),IM_COL32(255,180,220,255),i2);
                }

                } // end if(ea_w > 10 && ea_h > 10)

            // ── Bit Viewer (mode 7) ──────────────────────────────────────
            } else if(eid_mode == 7 && v.eid_data_ready.load()){
                const float LM=10.f, RM=10.f, TM=24.f, BM=10.f;
                float ea_x0=ov_x0+LM, ea_y0=ca_y0+TM;
                float ea_x1=ov_x1-RM, ea_y1=ca_y1-BM;
                float ea_w=ea_x1-ea_x0, ea_h=ea_y1-ea_y0;

                if(!v.eid_baud_mode || v.eid_baud_s0<0 || v.eid_baud_s1<0 || !v.eid_baseline_active){
                    {
                        char hdr[256]; snprintf(hdr,sizeof(hdr),"[Bits]");
                        fg->AddText(ImVec2(ea_x0, ca_y0+4), IM_COL32(160,160,180,220), hdr);
                        if(!v.sa_temp_path.empty()){
                            const char* fn=v.sa_temp_path.c_str();
                            const char* sep2=strrchr(fn,'/'); if(sep2) fn=sep2+1;
                            ImVec2 fsz=ImGui::CalcTextSize(fn);
                            fg->AddText(ImVec2(ea_x1-fsz.x, ca_y0+4), IM_COL32(160,160,180,220), fn);
                        }
                    }
                    fg->AddRectFilled(ImVec2(ea_x0,ea_y0),ImVec2(ea_x1,ea_y1),IM_COL32(8,8,12,255));
                    fg->AddRect(ImVec2(ea_x0,ea_y0),ImVec2(ea_x1,ea_y1),IM_COL32(60,60,80,255));
                    const char* hint = "Set baud lines (B) and baseline (right-click > Make Baseline) in Amp/Freq/Phase tab";
                    ImVec2 hsz=ImGui::CalcTextSize(hint);
                    fg->AddText(ImVec2(ea_x0+(ea_w-hsz.x)*0.5f, ea_y0+(ea_h-hsz.y)*0.5f),
                                IM_COL32(120,120,140,255), hint);
                } else {
                    // ── 비트 추출 ──
                    uint32_t sr=v.eid_sample_rate>0?v.eid_sample_rate:1;
                    double interval=v.eid_baud_s1-v.eid_baud_s0;
                    if(interval>0){
                        const std::vector<float>* src_data = nullptr;
                        switch(v.eid_baseline_imode){
                            case 0: src_data = v.eid_envelope.empty()   ? nullptr : &v.eid_envelope;  break;
                            case 1: src_data = v.eid_ch_i.empty()       ? nullptr : &v.eid_ch_i;      break;
                            case 2: src_data = v.eid_phase.empty()      ? nullptr : &v.eid_phase;     break;
                            case 3: src_data = v.eid_inst_freq.empty()  ? nullptr : &v.eid_inst_freq; break;
                            default: src_data = nullptr; break;
                        }

                        if(src_data && !src_data->empty()){
                            int64_t total=(int64_t)src_data->size();
                            float baseline=v.eid_baseline_val;

                            std::vector<uint8_t> bits;
                            // LoRa CSS 복조 결과가 있으면 그 비트열을 우선 사용
                            if(!v.eid_decoded_bits.empty()){
                                bits = v.eid_decoded_bits;
                            } else {
                                double first_edge = v.eid_baud_s0;
                                while(first_edge - interval >= 0) first_edge -= interval;
                                for(double edge = first_edge; edge + interval <= (double)total; edge += interval){
                                    double mid = edge + interval * 0.5;
                                    int64_t si = (int64_t)mid;
                                    if(si < 0 || si >= total) continue;
                                    bits.push_back((*src_data)[si] > baseline ? 1 : 0);
                                }
                            }

                            double baud_rate=(double)sr/interval;
                            double sig_len_s = (baud_rate > 0) ? (double)bits.size() / baud_rate : 0;
                            int n_bits=(int)bits.size();
                            int BPR = v.eid_bits_per_row;
                            int bit_off = std::max(0, std::min(v.eid_bits_offset, n_bits-1));

                            // ── 헤더 바 (다른 탭과 동일 위치) ──
                            {
                                char sig_len_str[32];
                                if(sig_len_s >= 1.0) snprintf(sig_len_str,sizeof(sig_len_str),"%.1f s",sig_len_s);
                                else snprintf(sig_len_str,sizeof(sig_len_str),"%.1f ms",sig_len_s*1000.0);
                                char hdr[320];
                                if(!v.eid_decoded_label.empty()){
                                    snprintf(hdr,sizeof(hdr), "%s | Bits: %d",
                                        v.eid_decoded_label.c_str(), n_bits);
                                } else {
                                    snprintf(hdr,sizeof(hdr),
                                        "Baud: %.0f | Bits: %d | Bytes: %d | %s",
                                        baud_rate, n_bits, n_bits/8, sig_len_str);
                                }
                                fg->AddText(ImVec2(ea_x0, ca_y0+4), IM_COL32(160,160,180,220), hdr);
                                if(!v.sa_temp_path.empty()){
                                    const char* fn=v.sa_temp_path.c_str();
                                    const char* sep2=strrchr(fn,'/'); if(sep2) fn=sep2+1;
                                    ImVec2 fsz=ImGui::CalcTextSize(fn);
                                    fg->AddText(ImVec2(ea_x1-fsz.x, ca_y0+4), IM_COL32(160,160,180,220), fn);
                                }
                            }

                            // ── 컨트롤 바 (TM 영역 활용) ──
                            float ctrl_y = ca_y0 + 4.f;
                            float ctrl_h = TM - 6.f;
                            float font_h = ImGui::GetFontSize();
                            // 컨트롤을 헤더 바 아래 공간에 배치 (ea_y0 바로 위)
                            float ctrl_bar_y = ea_y0 - 1.f;

                            // 컨트롤 바 배경
                            fg->AddRectFilled(ImVec2(ea_x0, ctrl_bar_y), ImVec2(ea_x1, ctrl_bar_y+font_h+6.f),
                                              IM_COL32(18,18,28,255));
                            float cby = ctrl_bar_y + 3.f;
                            float cbx = ea_x0 + 8.f;

                            // (View 토글 제거 - BIN/HEX/BITMAP 버튼으로 대체)
                            // [Bits/Row: N] - 클릭으로 순환
                            {
                                char rlbl[32]; snprintf(rlbl,sizeof(rlbl),"Bits/Row: %d", BPR);
                                ImVec2 rsz = ImGui::CalcTextSize(rlbl);
                                bool rhov = io.MousePos.x>=cbx && io.MousePos.x<=cbx+rsz.x &&
                                            io.MousePos.y>=cby && io.MousePos.y<=cby+font_h;
                                fg->AddText(ImVec2(cbx,cby), rhov?IM_COL32(255,255,255,255):IM_COL32(140,180,220,255), rlbl);
                                if(rhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                    static const int bpr_opts[]={8,16,32,64,128,256,512};
                                    int ci=0; for(int i=0;i<7;i++) if(bpr_opts[i]==BPR) ci=i;
                                    v.eid_bits_per_row = bpr_opts[(ci+1)%7];
                                    v.eid_bits_scroll = 0;
                                }
                                if(rhov && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                                    static const int bpr_opts[]={8,16,32,64,128,256,512};
                                    int ci=0; for(int i=0;i<7;i++) if(bpr_opts[i]==BPR) ci=i;
                                    v.eid_bits_per_row = bpr_opts[(ci+6)%7];
                                    v.eid_bits_scroll = 0;
                                }
                                cbx += rsz.x + 16.f;
                            }
                            // [Offset: +/-] 수동 비트 오프셋
                            {
                                char olbl[32]; snprintf(olbl,sizeof(olbl),"Offset: %d", v.eid_bits_offset);
                                ImVec2 osz = ImGui::CalcTextSize(olbl);
                                fg->AddText(ImVec2(cbx,cby), IM_COL32(140,180,220,255), olbl);
                                cbx += osz.x + 4.f;
                                // [-] 버튼
                                {
                                    const char* ml = "-";
                                    ImVec2 ms = ImGui::CalcTextSize(ml);
                                    bool mh = io.MousePos.x>=cbx && io.MousePos.x<=cbx+ms.x+4 &&
                                              io.MousePos.y>=cby && io.MousePos.y<=cby+font_h;
                                    fg->AddText(ImVec2(cbx,cby), mh?IM_COL32(255,100,100,255):IM_COL32(180,180,200,255), ml);
                                    if(mh && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && v.eid_bits_offset>0)
                                        v.eid_bits_offset--;
                                    cbx += ms.x + 4.f;
                                }
                                // [+] 버튼
                                {
                                    const char* pl = "+";
                                    ImVec2 ps = ImGui::CalcTextSize(pl);
                                    bool ph = io.MousePos.x>=cbx && io.MousePos.x<=cbx+ps.x+4 &&
                                              io.MousePos.y>=cby && io.MousePos.y<=cby+font_h;
                                    fg->AddText(ImVec2(cbx,cby), ph?IM_COL32(100,255,100,255):IM_COL32(180,180,200,255), pl);
                                    if(ph && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                                        v.eid_bits_offset++;
                                    cbx += ps.x + 16.f;
                                }
                            }
                            // [Copy] 버튼
                            static float copy_flash = 0.f;
                            {
                                const char* clbl = copy_flash>0 ? "Copied!" : "Copy";
                                ImVec2 csz = ImGui::CalcTextSize(clbl);
                                bool chov = io.MousePos.x>=cbx && io.MousePos.x<=cbx+csz.x &&
                                            io.MousePos.y>=cby && io.MousePos.y<=cby+font_h;
                                ImU32 ccol = copy_flash>0 ? IM_COL32(80,255,140,255) :
                                             (chov ? IM_COL32(255,255,255,255) : IM_COL32(140,180,220,255));
                                fg->AddText(ImVec2(cbx,cby), ccol, clbl);
                                if(chov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                    // 비트열을 문자열로 변환 후 클립보드 복사
                                    std::string bstr; bstr.reserve(n_bits+n_bits/8);
                                    for(int i=bit_off;i<n_bits;i++){
                                        bstr += (bits[i]?'1':'0');
                                        if((i-bit_off+1)%8==0 && i+1<n_bits) bstr += ' ';
                                    }
                                    ImGui::SetClipboardText(bstr.c_str());
                                    copy_flash = 1.5f;
                                }
                                if(copy_flash > 0) copy_flash -= io.DeltaTime;
                            }

                            float data_y0 = ctrl_bar_y + font_h + 7.f;

                            // 배경
                            fg->AddRectFilled(ImVec2(ea_x0,data_y0),ImVec2(ea_x1,ea_y1),IM_COL32(8,8,12,255));
                            fg->AddRect(ImVec2(ea_x0,data_y0),ImVec2(ea_x1,ea_y1),IM_COL32(60,60,80,255));

                            // 마우스 호버 영역
                            bool mouse_in_bits = io.MousePos.x>=ea_x0 && io.MousePos.x<=ea_x1 &&
                                                 io.MousePos.y>=data_y0 && io.MousePos.y<=ea_y1;

                            // 비트 오프셋 적용
                            int disp_bits = n_bits - bit_off;
                            if(disp_bits < 0) disp_bits = 0;
                            int total_rows = (disp_bits + BPR - 1) / BPR;

                            if(v.eid_bits_view == 0 || v.eid_bits_view == 1){
                                // ══════ 텍스트 뷰 (BIN 또는 HEX) ══════
                                ImFont* bfnt = g_bits_font ? g_bits_font : ImGui::GetFont();
                                float bfh = bfnt->LegacySize;
                                float line_h = bfh + 4.f;
                                float cx=ea_x0+8.f;
                                float b_ch_w = bfnt->CalcTextSizeA(bfh,FLT_MAX,-1.f,"0").x;

                                int GROUPS = BPR / 8;
                                if(GROUPS < 1) GROUPS = 1;
                                float content_w=ea_w-16.f;
                                float zoom = v.eid_bits_zoom;
                                float bit_cw, hex_byte_w, grp_gap;
                                if(v.eid_bits_view == 0){
                                    // BIN: 고정 간격 * 줌
                                    bit_cw = b_ch_w * zoom;
                                    grp_gap = b_ch_w * 0.5f * zoom;
                                    hex_byte_w = 0;
                                } else {
                                    // HEX: 고정 간격 * 줌
                                    grp_gap = 0;
                                    hex_byte_w = b_ch_w * 3.0f * zoom;
                                    bit_cw = 0;
                                }
                                // 전체 행 폭 계산 (스크롤 범위용)
                                float total_row_w = (v.eid_bits_view == 0)
                                    ? BPR * bit_cw + (GROUPS-1) * grp_gap
                                    : GROUPS * hex_byte_w;

                                // 헤더: [AIS] [ADS-B] [UAV] 디코더 버튼
                                float cy = data_y0 + 2.f;
                                {
                                    float bx_btn = cx;
                                    const char* dec_btns[] = {"AIS","ADS-B","UAV"};
                                    for(int di=0;di<3;di++){
                                        ImVec2 dsz = bfnt->CalcTextSizeA(bfh, FLT_MAX, -1.f, dec_btns[di]);
                                        bool dsel = (v.eid_decode_mode == di+1);
                                        bool dhov = io.MousePos.x>=bx_btn && io.MousePos.x<=bx_btn+dsz.x &&
                                                    io.MousePos.y>=cy && io.MousePos.y<=cy+bfh;
                                        ImU32 dcol = dsel ? IM_COL32(80,255,140,255) :
                                                     dhov ? IM_COL32(255,255,255,255) : IM_COL32(140,180,220,255);
                                        fg->AddText(bfnt,bfh,ImVec2(bx_btn,cy),dcol,dec_btns[di]);
                                        if(dhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                                            v.eid_decode_mode = dsel ? 0 : (di+1);
                                        bx_btn += dsz.x + 24.f;
                                    }
                                    // BIN / HEX / BITMAP (우측 정렬)
                                    {
                                        const char* fmt_btns[] = {"BIN","HEX","BITMAP"};
                                        float rx = ea_x1 - 8.f;
                                        for(int fi2=2;fi2>=0;fi2--){
                                            ImVec2 fsz2 = bfnt->CalcTextSizeA(bfh, FLT_MAX, -1.f, fmt_btns[fi2]);
                                            float bx2 = rx - fsz2.x;
                                            bool fsel = (v.eid_bits_view == fi2 && v.eid_decode_mode == 0);
                                            bool fhov = io.MousePos.x>=bx2 && io.MousePos.x<=bx2+fsz2.x &&
                                                        io.MousePos.y>=cy && io.MousePos.y<=cy+bfh;
                                            ImU32 fcol = fsel ? IM_COL32(80,255,140,255) :
                                                         fhov ? IM_COL32(255,255,255,255) : IM_COL32(140,180,220,255);
                                            fg->AddText(bfnt,bfh,ImVec2(bx2,cy),fcol,fmt_btns[fi2]);
                                            if(fhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                                v.eid_bits_view = fi2;
                                                v.eid_decode_mode = 0;
                                            }
                                            rx = bx2 - 20.f;
                                        }
                                    }
                                }
                                cy+=line_h;
                                fg->AddLine(ImVec2(ea_x0+4,cy-1),ImVec2(ea_x1-4,cy-1),IM_COL32(50,50,70,255));

                                if(v.eid_decode_mode == 1){
                                    // ══════ AIS 디코더 모드 (Python 호출) ══════
                                    auto ais_r = ais_decode::decode(bits, bit_off);
                                    if(ais_r.ok && !ais_r.lines.empty()){
                                        int total_lines = (int)ais_r.lines.size();
                                        int vis_lines = (int)((ea_y1 - cy) / line_h);
                                        if(vis_lines < 1) vis_lines = 1;
                                        int max_scroll = std::max(0, total_lines - vis_lines);
                                        if(mouse_in_bits && io.MouseWheel != 0)
                                            v.eid_decode_scroll -= (int)(io.MouseWheel * 3);
                                        v.eid_decode_scroll = std::max(0, std::min(v.eid_decode_scroll, max_scroll));

                                        // 스크롤바
                                        if(total_lines > vis_lines){
                                            float sb_x = ea_x1 - 6.f;
                                            float sb_h = ea_y1 - cy;
                                            float thumb_h = std::max(20.f, sb_h * (float)vis_lines / (float)total_lines);
                                            float thumb_y = cy + (sb_h - thumb_h) * (float)v.eid_decode_scroll / (float)max_scroll;
                                            fg->AddRectFilled(ImVec2(sb_x,cy),ImVec2(sb_x+4,ea_y1),IM_COL32(30,30,45,255));
                                            fg->AddRectFilled(ImVec2(sb_x,thumb_y),ImVec2(sb_x+4,thumb_y+thumb_h),
                                                              IM_COL32(80,80,120,255),2.f);
                                        }

                                        fg->PushClipRect(ImVec2(ea_x0,cy),ImVec2(ea_x1,ea_y1),true);
                                        for(int li=v.eid_decode_scroll; li<total_lines && cy<ea_y1-line_h; li++){
                                            auto& ln = ais_r.lines[li];
                                            ImU32 lc = (ln.find("===")!=std::string::npos || ln.find("---")!=std::string::npos)
                                                       ? IM_COL32(100,100,130,255)
                                                       : IM_COL32(200,220,255,255);
                                            fg->AddText(bfnt,bfh,ImVec2(cx,cy),lc,ln.c_str());
                                            cy+=line_h;
                                        }
                                        fg->PopClipRect();
                                    } else {
                                        fg->PushClipRect(ImVec2(ea_x0,cy),ImVec2(ea_x1,ea_y1),true);
                                        const char* msg = ais_r.error.empty() ? "No valid AIS frame found" : ais_r.error.c_str();
                                        fg->AddText(bfnt,bfh,ImVec2(cx,cy),IM_COL32(255,100,100,255),msg);
                                        fg->PopClipRect();
                                    }
                                } else if(v.eid_decode_mode >= 2){
                                    // ══════ ADS-B / UAV — 추후 구현 ══════
                                    fg->PushClipRect(ImVec2(ea_x0,cy),ImVec2(ea_x1,ea_y1),true);
                                    fg->AddText(bfnt,bfh,ImVec2(cx,cy),IM_COL32(180,180,100,255),"Not implemented yet");
                                    fg->PopClipRect();
                                } else {
                                    // ══════ 기본 Binary+Hex 뷰 ══════
                                    int vis_rows = (int)((ea_y1 - cy) / line_h);
                                    if(vis_rows < 1) vis_rows = 1;
                                    int max_scroll = std::max(0, total_rows - vis_rows);

                                    // 스크롤바
                                    if(total_rows > vis_rows){
                                        float sb_x = ea_x1 + 2.f;
                                        float sb_h = ea_y1 - cy;
                                        float thumb_h = std::max(20.f, sb_h * (float)vis_rows / (float)total_rows);
                                        float thumb_y = cy + (sb_h - thumb_h) * (float)v.eid_bits_scroll / (float)max_scroll;
                                        fg->AddRectFilled(ImVec2(sb_x,cy),ImVec2(sb_x+4,ea_y1),IM_COL32(30,30,45,255));
                                        fg->AddRectFilled(ImVec2(sb_x,thumb_y),ImVec2(sb_x+4,thumb_y+thumb_h),
                                                          IM_COL32(80,80,120,255),2.f);
                                    }

                                    // Ctrl+휠: 줌 / 일반 휠: 수직 스크롤
                                    if(mouse_in_bits && io.MouseWheel != 0){
                                        if(io.KeyCtrl){
                                            float zf = (io.MouseWheel > 0) ? 1.15f : 0.87f;
                                            v.eid_bits_zoom = std::max(0.2f, std::min(5.0f, v.eid_bits_zoom * zf));
                                        } else {
                                            v.eid_bits_scroll -= (int)(io.MouseWheel * 3);
                                        }
                                    }
                                    v.eid_bits_scroll = std::max(0, std::min(v.eid_bits_scroll, max_scroll));

                                    // 수평 스크롤 범위
                                    float max_hscroll = std::max(0.f, total_row_w - content_w);
                                    v.eid_bits_hscroll = std::max(0.f, std::min(v.eid_bits_hscroll, max_hscroll));
                                    float draw_ox = cx - v.eid_bits_hscroll;

                                    // 데이터 렌더링
                                    fg->PushClipRect(ImVec2(ea_x0,cy),ImVec2(ea_x1,ea_y1),true);
                                    for(int row=v.eid_bits_scroll; row<total_rows && cy<ea_y1-line_h; row++){
                                        int bi = bit_off + row * BPR;
                                        if(v.eid_bits_view == 0){
                                            float bx2=draw_ox;
                                            for(int gi=0;gi<BPR&&bi+gi<n_bits;gi+=8){
                                                for(int k=0;k<8&&bi+gi+k<n_bits;k++){
                                                    char c[2]={bits[bi+gi+k]?'1':'0',0};
                                                    ImU32 bc=bits[bi+gi+k]?IM_COL32(100,220,255,255):IM_COL32(90,90,110,255);
                                                    fg->AddText(bfnt,bfh,ImVec2(bx2+k*bit_cw,cy),bc,c);
                                                }
                                                bx2+=8*bit_cw+grp_gap;
                                            }
                                        } else {
                                            float hx=draw_ox;
                                            for(int gi=0;gi<BPR&&bi+gi<n_bits;gi+=8){
                                                uint8_t byte_val=0;
                                                int valid=0;
                                                for(int k=0;k<8&&bi+gi+k<n_bits;k++){
                                                    byte_val=(byte_val<<1)|bits[bi+gi+k];
                                                    valid++;
                                                }
                                                if(valid==8){
                                                    char hstr[4]; snprintf(hstr,sizeof(hstr),"%02X",byte_val);
                                                    fg->AddText(bfnt,bfh,ImVec2(hx,cy),IM_COL32(120,200,255,255),hstr);
                                                }
                                                hx+=hex_byte_w;
                                            }
                                        }
                                        cy+=line_h;
                                    }
                                    fg->PopClipRect();

                                    // 수평 스크롤바 (하단)
                                    if(max_hscroll > 0){
                                        float hsb_y = ea_y1 + 2.f;
                                        float hsb_w = ea_x1 - ea_x0;
                                        float hthumb_w = std::max(20.f, hsb_w * content_w / total_row_w);
                                        float hthumb_x = ea_x0 + (hsb_w - hthumb_w) * v.eid_bits_hscroll / max_hscroll;
                                        fg->AddRectFilled(ImVec2(ea_x0,hsb_y),ImVec2(ea_x1,hsb_y+4),IM_COL32(30,30,45,255));
                                        fg->AddRectFilled(ImVec2(hthumb_x,hsb_y),ImVec2(hthumb_x+hthumb_w,hsb_y+4),
                                                          IM_COL32(80,80,120,255),2.f);
                                        // 수평 스크롤바 드래그
                                        static bool hsb_drag = false;
                                        static float hsb_drag_start = 0;
                                        if(io.MousePos.y >= hsb_y-4 && io.MousePos.y <= hsb_y+8 &&
                                           io.MousePos.x >= ea_x0 && io.MousePos.x <= ea_x1){
                                            if(ImGui::IsMouseClicked(0)){
                                                hsb_drag = true;
                                                hsb_drag_start = io.MousePos.x;
                                            }
                                        }
                                        if(hsb_drag){
                                            if(ImGui::IsMouseDown(0)){
                                                float dx = io.MousePos.x - hsb_drag_start;
                                                hsb_drag_start = io.MousePos.x;
                                                v.eid_bits_hscroll += dx * (max_hscroll / (hsb_w - hthumb_w + 1.f));
                                                v.eid_bits_hscroll = std::max(0.f, std::min(v.eid_bits_hscroll, max_hscroll));
                                            } else hsb_drag = false;
                                        }
                                    }

                                    // hover 비트번호 (BIN/HEX 공통)
                                    if(mouse_in_bits){
                                        // cy_start는 데이터 시작 Y (헤더+구분선 아래)
                                        // 헤더줄 = data_y0+2 ~ data_y0+2+line_h, 구분선, 데이터시작
                                        float data_start_y = data_y0 + 2.f + line_h + 1.f; // 헤더+구분선 다음
                                        if(io.MousePos.y >= data_start_y){
                                            int hr = v.eid_bits_scroll + (int)((io.MousePos.y - data_start_y) / line_h);
                                            float mx_rel = io.MousePos.x - cx + v.eid_bits_hscroll;
                                            int hc = -1;
                                            if(v.eid_bits_view == 0){
                                                // BIN: 비트 단위
                                                for(int gi=0;gi<GROUPS;gi++){
                                                    float gx0 = gi*(8*bit_cw+grp_gap);
                                                    float gx1 = gx0 + 8*bit_cw;
                                                    if(mx_rel >= gx0 && mx_rel < gx1){
                                                        hc = gi*8 + (int)((mx_rel-gx0)/bit_cw);
                                                        break;
                                                    }
                                                }
                                            } else {
                                                // HEX: 바이트 단위 → 바이트*8
                                                int byte_idx = (int)(mx_rel / hex_byte_w);
                                                if(byte_idx >= 0 && byte_idx < GROUPS) hc = byte_idx * 8;
                                            }
                                            if(hc >= 0 && hr >= 0 && hr < total_rows){
                                                int hbi = bit_off + hr * BPR + hc;
                                                if(hbi >= 0 && hbi < n_bits){
                                                    char tip[32]; snprintf(tip,sizeof(tip),"%d",hbi+1);
                                                    ImVec2 tsz=ImGui::CalcTextSize(tip);
                                                    float tx=io.MousePos.x+12.f, ty=io.MousePos.y-tsz.y-4.f;
                                                    fg->AddRectFilled(ImVec2(tx-3,ty-2),ImVec2(tx+tsz.x+3,ty+tsz.y+2),
                                                                      IM_COL32(0,0,0,200),3.f);
                                                    fg->AddText(ImVec2(tx,ty),IM_COL32(255,255,255,255),tip);
                                                }
                                            }
                                        }
                                    }
                                }

                            } else if(v.eid_bits_view == 2){
                                // ══════ BITMAP 뷰 ══════
                                // 헤더 버튼 (BIN/HEX/BITMAP 전환용)
                                {
                                    ImFont* bfnt = g_bits_font ? g_bits_font : ImGui::GetFont();
                                    float bfh2 = bfnt->LegacySize;
                                    float cy_hdr = data_y0 + 2.f;
                                    float cx_hdr = ea_x0 + 8.f;
                                    // AIS/ADS-B/UAV (좌측)
                                    const char* dec_btns[] = {"AIS","ADS-B","UAV"};
                                    for(int di=0;di<3;di++){
                                        ImVec2 dsz = bfnt->CalcTextSizeA(bfh2, FLT_MAX, -1.f, dec_btns[di]);
                                        bool dsel = (v.eid_decode_mode == di+1);
                                        bool dhov = io.MousePos.x>=cx_hdr && io.MousePos.x<=cx_hdr+dsz.x &&
                                                    io.MousePos.y>=cy_hdr && io.MousePos.y<=cy_hdr+bfh2;
                                        ImU32 dcol = dsel ? IM_COL32(80,255,140,255) :
                                                     dhov ? IM_COL32(255,255,255,255) : IM_COL32(140,180,220,255);
                                        fg->AddText(bfnt,bfh2,ImVec2(cx_hdr,cy_hdr),dcol,dec_btns[di]);
                                        if(dhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                                            v.eid_decode_mode = dsel ? 0 : (di+1);
                                        cx_hdr += dsz.x + 24.f;
                                    }
                                    // BIN/HEX/BITMAP (우측)
                                    const char* fmt_btns[] = {"BIN","HEX","BITMAP"};
                                    float rx2 = ea_x1 - 8.f;
                                    for(int fi2=2;fi2>=0;fi2--){
                                        ImVec2 fsz2 = bfnt->CalcTextSizeA(bfh2, FLT_MAX, -1.f, fmt_btns[fi2]);
                                        float bx2 = rx2 - fsz2.x;
                                        bool fsel = (v.eid_bits_view == fi2 && v.eid_decode_mode == 0);
                                        bool fhov = io.MousePos.x>=bx2 && io.MousePos.x<=bx2+fsz2.x &&
                                                    io.MousePos.y>=cy_hdr && io.MousePos.y<=cy_hdr+bfh2;
                                        ImU32 fcol = fsel ? IM_COL32(80,255,140,255) :
                                                     fhov ? IM_COL32(255,255,255,255) : IM_COL32(140,180,220,255);
                                        fg->AddText(bfnt,bfh2,ImVec2(bx2,cy_hdr),fcol,fmt_btns[fi2]);
                                        if(fhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                                            v.eid_bits_view = fi2;
                                            v.eid_decode_mode = 0;
                                        }
                                        rx2 = bx2 - 20.f;
                                    }
                                }

                                float avail_w = ea_w - 8.f;
                                float cell_w = avail_w / (float)BPR;
                                if(cell_w < 1.f) cell_w = 1.f;
                                float cell_h = cell_w; // 정사각형
                                if(cell_h > 12.f) cell_h = 12.f;
                                if(cell_h < 2.f) cell_h = 2.f;
                                float line_h_v = cell_h + 1.f;

                                ImFont* bfnt_bm = g_bits_font ? g_bits_font : ImGui::GetFont();
                                float cy = data_y0 + bfnt_bm->LegacySize + 8.f; // 헤더 아래

                                // 스크롤 처리
                                int vis_rows = (int)((ea_y1 - cy) / line_h_v);
                                if(vis_rows < 1) vis_rows = 1;
                                int max_scroll = std::max(0, total_rows - vis_rows);
                                if(mouse_in_bits && io.MouseWheel != 0){
                                    v.eid_bits_scroll -= (int)(io.MouseWheel * 5);
                                }
                                v.eid_bits_scroll = std::max(0, std::min(v.eid_bits_scroll, max_scroll));

                                // 스크롤바
                                if(total_rows > vis_rows){
                                    float sb_x = ea_x1 + 2.f;
                                    float sb_h = ea_y1 - cy;
                                    float thumb_h = std::max(20.f, sb_h * (float)vis_rows / (float)total_rows);
                                    float thumb_y = cy + (sb_h - thumb_h) * (float)v.eid_bits_scroll / (float)max_scroll;
                                    fg->AddRectFilled(ImVec2(sb_x,cy),ImVec2(sb_x+4,ea_y1),IM_COL32(30,30,45,255));
                                    fg->AddRectFilled(ImVec2(sb_x,thumb_y),ImVec2(sb_x+4,thumb_y+thumb_h),
                                                      IM_COL32(80,80,120,255),2.f);
                                }

                                // 비트맵 렌더링
                                float cx = ea_x0 + 4.f;
                                fg->PushClipRect(ImVec2(ea_x0,cy),ImVec2(ea_x1,ea_y1),true);
                                for(int row=v.eid_bits_scroll; row<total_rows && cy<ea_y1; row++){
                                    int bi = bit_off + row * BPR;
                                    for(int k=0; k<BPR && bi+k<n_bits; k++){
                                        float x0 = cx + k * cell_w;
                                        float x1 = x0 + cell_w - 0.5f;
                                        ImU32 col = bits[bi+k] ?
                                            IM_COL32(220,240,255,255) :  // 1: 밝은 흰색
                                            IM_COL32(12,12,20,255);      // 0: 거의 검은색
                                        fg->AddRectFilled(ImVec2(x0,cy),ImVec2(x1,cy+cell_h),col);
                                    }
                                    cy += line_h_v;
                                }
                                fg->PopClipRect();

                                // 마우스 호버 비트번호
                                float bm_start_y = data_y0 + bfnt_bm->LegacySize + 8.f;
                                if(mouse_in_bits && io.MousePos.y >= bm_start_y){
                                    int hover_row = v.eid_bits_scroll + (int)((io.MousePos.y - bm_start_y) / line_h_v);
                                    int hover_col = (int)((io.MousePos.x - ea_x0 - 4.f) / cell_w);
                                    if(hover_col>=0 && hover_col<BPR && hover_row>=0 && hover_row<total_rows){
                                        int hover_bi = bit_off + hover_row * BPR + hover_col;
                                        if(hover_bi >= 0 && hover_bi < n_bits){
                                            char tip[32]; snprintf(tip,sizeof(tip),"%d",hover_bi+1);
                                            ImVec2 tsz = ImGui::CalcTextSize(tip);
                                            float tx = io.MousePos.x + 12.f;
                                            float ty = io.MousePos.y - tsz.y - 4.f;
                                            fg->AddRectFilled(ImVec2(tx-3,ty-2),ImVec2(tx+tsz.x+3,ty+tsz.y+2),
                                                              IM_COL32(0,0,0,200),3.f);
                                            fg->AddText(ImVec2(tx,ty),IM_COL32(255,255,255,255),tip);
                                        }
                                    }
                                }
                            }

                            // Home/End 키
                            if(mouse_in_bits){
                                if(ImGui::IsKeyPressed(ImGuiKey_Home,false)) v.eid_bits_scroll=0;
                                if(ImGui::IsKeyPressed(ImGuiKey_End,false)) v.eid_bits_scroll=999999;
                                // 방향키: Up/Down = Bits/Row +-1, Left/Right = Offset +-1
                                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow,true)){
                                    v.eid_bits_per_row = std::min(512, v.eid_bits_per_row + 1);
                                    v.eid_bits_scroll = 0;
                                }
                                if(ImGui::IsKeyPressed(ImGuiKey_DownArrow,true)){
                                    v.eid_bits_per_row = std::max(1, v.eid_bits_per_row - 1);
                                    v.eid_bits_scroll = 0;
                                }
                                if(ImGui::IsKeyPressed(ImGuiKey_RightArrow,true)){
                                    v.eid_bits_offset = std::max(0, v.eid_bits_offset - 1);
                                }
                                if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow,true)){
                                    v.eid_bits_offset++;
                                }
                            }
                        }
                    }
                }

                // 데이터 없음
            } else {
                const char* msg = "Load a WAV file from STAT (right-click > Signal Analysis)";
                ImVec2 msz = ImGui::CalcTextSize(msg);
                fg->AddText(ImVec2(ov_x0+(ov_w-msz.x)/2, ca_y0+(ca_h-msz.y)/2),
                            IM_COL32(100,100,120,255), msg);
            }
            // ── 태그 컨텍스트 메뉴 ────────────────────────────────────
            bool ctx_valid = eid_tag_ctx.open &&
                (eid_tag_ctx.is_pending ? v.eid_pending_active
                                        : (eid_tag_ctx.tag_idx>=0 && eid_tag_ctx.tag_idx<(int)v.eid_tags.size()));
            if(ctx_valid){
                ImGui::SetNextWindowPos(ImVec2(eid_tag_ctx.x,eid_tag_ctx.y));
                ImGui::SetNextWindowSize(ImVec2(180.f,0.f));
                ImGui::SetNextWindowBgAlpha(0.95f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,6.f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(6.f,6.f));
                ImGui::PushStyleColor(ImGuiCol_WindowBg,ImVec4(0.10f,0.12f,0.18f,1.f));
                ImGui::Begin("##eid_tag_ctx",nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                    ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoDecoration);

                // 현재 메뉴 대상의 샘플 범위
                double ctx_s0 = eid_tag_ctx.is_pending ? v.eid_pending_s0 : v.eid_tags[eid_tag_ctx.tag_idx].s0;
                double ctx_s1 = eid_tag_ctx.is_pending ? v.eid_pending_s1 : v.eid_tags[eid_tag_ctx.tag_idx].s1;

                if(eid_tag_ctx.renaming){
                    // Rename 입력창 (창 너비에 맞춤)
                    if(!eid_tag_ctx.rename_focused){
                        ImGui::SetKeyboardFocusHere();
                        eid_tag_ctx.rename_focused=true;
                    }
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                    if(ImGui::InputText("##tag_rename",eid_tag_ctx.rename_buf,32,
                                        ImGuiInputTextFlags_EnterReturnsTrue)){
                        strncpy(v.eid_tags[eid_tag_ctx.tag_idx].label,eid_tag_ctx.rename_buf,31);
                        eid_tag_ctx.renaming=false; eid_tag_ctx.open=false;
                    }
                    if(ImGui::IsKeyPressed(ImGuiKey_Escape)){
                        eid_tag_ctx.renaming=false; eid_tag_ctx.open=false;
                    }
                } else if(eid_tag_ctx.is_pending){
                    // ── 임시 선택 영역 메뉴 ──
                    if(ImGui::Selectable("  Make Tag")){
                        v.eid_push_undo();
                        static const ImU32 tc[]={IM_COL32(255,180,60,200),IM_COL32(100,200,255,200),
                            IM_COL32(255,100,255,200),IM_COL32(100,255,180,200),
                            IM_COL32(255,255,100,200),IM_COL32(180,140,255,200)};
                        FFTViewer::EidTag tag;
                        tag.s0=ctx_s0; tag.s1=ctx_s1;
                        tag.color=tc[v.eid_tags.size()%6];
                        snprintf(tag.label,sizeof(tag.label),"Tag %d",(int)v.eid_tags.size()+1);
                        v.eid_tags.push_back(tag);
                        v.eid_auto_analyze_tag(v.eid_tags.back());
                        v.eid_pending_active=false;
                        eid_tag_ctx.open=false;
                    }
                    if(ImGui::Selectable("  Remove Tag")){
                        v.eid_pending_active=false;
                        eid_tag_ctx.open=false;
                    }
                    // LoRa CSS 복조: Freq 탭(mode 2)에서만 표시
                    if(v.eid_view_mode == 2){
                    ImGui::Separator();
                    if(ImGui::Selectable("  Demod LoRa CSS")){
                        int64_t s0 = (int64_t)ctx_s0;
                        int64_t s1 = (int64_t)ctx_s1;
                        if(s0 > s1) std::swap(s0, s1);
                        if(s0 < 0) s0 = 0;
                        if(s1 > v.eid_total_samples) s1 = v.eid_total_samples;
                        int64_t n = s1 - s0;
                        uint32_t sr = v.eid_sample_rate;
                        if(n < 4096 || sr == 0){
                            bewe_log_push(2,"[LoRa] 선택 영역이 너무 짧음 (n=%lld, sr=%u)\n", (long long)n, sr);
                        } else if(v.eid_lora_busy.load()){
                            bewe_log_push(2,"[LoRa] 이미 복조 중입니다\n");
                        } else {
                            v.eid_lora_busy.store(true);
                            v.eid_decoded_bits.clear();
                            v.eid_decoded_label = "Decoding LoRa CSS...";
                            v.eid_pending_active = false;
                            std::thread([&v, s0, s1, n, sr](){
                                // eid_ch_i/q 접근은 mutex로 보호
                                std::vector<float> local_i, local_q;
                                {
                                    std::lock_guard<std::mutex> lk(v.eid_data_mtx);
                                    if((int64_t)v.eid_ch_i.size() < s1 || (int64_t)v.eid_ch_q.size() < s1){
                                        v.eid_lora_busy.store(false);
                                        v.eid_decoded_label.clear();
                                        bewe_log_push(2,"[LoRa] eid IQ 버퍼 부족\n");
                                        return;
                                    }
                                    local_i.assign(v.eid_ch_i.begin()+s0, v.eid_ch_i.begin()+s1);
                                    local_q.assign(v.eid_ch_q.begin()+s0, v.eid_ch_q.begin()+s1);
                                }
                                LoraDemodResult info;
                                std::vector<uint8_t> bits;
                                bool ok = lora_demod_auto(local_i.data(), local_q.data(),
                                                          n, sr, bits, info);
                                if(ok){
                                    v.eid_decoded_bits = std::move(bits);
                                    v.eid_decoded_label = info.detail;
                                    bewe_log_push(0,"[LoRa] %s\n", info.detail.c_str());
                                } else {
                                    v.eid_decoded_bits.clear();
                                    v.eid_decoded_label = std::string("LoRa decode FAIL: ") + info.detail;
                                    bewe_log_push(2,"[LoRa] decode failed: %s\n", info.detail.c_str());
                                }
                                v.eid_lora_busy.store(false);
                            }).detach();
                        }
                        eid_tag_ctx.open = false;
                    }
                    } // end if(eid_view_mode == 2)
                    ImGui::Separator();
                    if(ImGui::Selectable("  Select Samples")){
                        v.eid_push_undo();
                        eid_tag_ctx.open=false;
                        v.eid_pending_active=false;
                        v.eid_select_samples(ctx_s0,ctx_s1);
                    }
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f,0.35f,0.35f,1.0f));
                    if(ImGui::Selectable("  Remove Samples")){
                        ImGui::PopStyleColor();
                        v.eid_push_undo();
                        eid_tag_ctx.open=false;
                        v.eid_pending_active=false;
                        v.eid_remove_samples(ctx_s0,ctx_s1);
                    } else { ImGui::PopStyleColor(); }
                } else {
                    // ── 확정 태그 메뉴 ──
                    if(ImGui::Selectable("  Rename Tag")){
                        v.eid_push_undo();
                        eid_tag_ctx.renaming=true; eid_tag_ctx.rename_focused=false;
                    }
                    if(ImGui::Selectable("  Remove Tag")){
                        v.eid_push_undo();
                        v.eid_tags.erase(v.eid_tags.begin()+eid_tag_ctx.tag_idx);
                        eid_tag_ctx.open=false;
                    }
                    ImGui::Separator();
                    if(ImGui::Selectable("  Select Samples")){
                        v.eid_push_undo();
                        eid_tag_ctx.open=false;
                        v.eid_select_samples(ctx_s0,ctx_s1);
                    }
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f,0.35f,0.35f,1.0f));
                    if(ImGui::Selectable("  Remove Samples")){
                        ImGui::PopStyleColor();
                        v.eid_push_undo();
                        eid_tag_ctx.open=false;
                        v.eid_remove_samples(ctx_s0,ctx_s1);
                    } else { ImGui::PopStyleColor(); }
                }

                // 메뉴 바깥 클릭 시 닫기
                if(!eid_tag_ctx.renaming &&
                   !ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
                   (ImGui::IsMouseClicked(ImGuiMouseButton_Left)||ImGui::IsMouseClicked(ImGuiMouseButton_Right))){
                    eid_tag_ctx.open=false;
                }
                ImGui::End();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar(2);
            }

            // ── Spectrogram 빈 영역 우클릭 Save File 메뉴 ─────────────────
            if(eid_save_ctx.open){
                ImGui::SetNextWindowPos(ImVec2(eid_save_ctx.x, eid_save_ctx.y));
                ImGui::SetNextWindowSize(ImVec2(180.f, 0.f));
                ImGui::SetNextWindowBgAlpha(0.95f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.f,6.f));
                ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f,0.12f,0.18f,1.f));
                ImGui::Begin("##eid_save_ctx", nullptr,
                    ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                    ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
                    ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoDecoration);

                // Save File: 중앙정렬 텍스트 셀렉터블
                const char* lbl = "Save File";
                float avail_w = ImGui::GetContentRegionAvail().x;
                ImVec2 ts = ImGui::CalcTextSize(lbl);
                float row_h = ts.y + ImGui::GetStyle().FramePadding.y * 2.f;
                ImVec2 cp = ImGui::GetCursorScreenPos();
                bool clicked = ImGui::Selectable("##eid_save_sel", false, 0, ImVec2(avail_w, row_h));
                ImGui::GetWindowDrawList()->AddText(
                    ImVec2(cp.x + (avail_w - ts.x) * 0.5f,
                           cp.y + (row_h  - ts.y) * 0.5f),
                    ImGui::GetColorU32(ImGuiCol_Text), lbl);

                if(clicked){
                    // InfoModal을 EID Save File 플로우로 열기
                    std::string default_path = v.eid_default_filtered_path();
                    if(!default_path.empty()){
                        info_modal.open = true;
                        info_modal.save_and_write = true;
                        info_modal.src_filepath = v.sa_temp_path;
                        info_modal.filepath = default_path;
                        info_modal.info_path = default_path + ".info";
                        info_modal.exists = false;
                        info_modal.utc_off_override = v.utc_offset_hours();
                        // pre-fill: 원본 .info 있으면 그 내용으로, 없으면 autofill
                        std::string src_info = v.sa_temp_path + ".info";
                        if(access(src_info.c_str(), F_OK) == 0){
                            std::string dst_ip = info_modal.info_path;
                            info_modal.info_path = src_info;
                            memset(info_modal.fields, 0, sizeof(info_modal.fields));
                            info_modal.load();
                            info_modal.info_path = dst_ip;
                        } else {
                            const char* bn = strrchr(default_path.c_str(), '/');
                            bn = bn ? bn+1 : default_path.c_str();
                            info_modal.autofill(std::string(bn));
                        }
                        info_modal.init_ext();
                        // 저장 콜백: InfoModal Add > new_path에 WAV 생성
                        info_modal.save_file_fn = [&v](const std::string& out_path) -> std::string {
                            return v.eid_save_filtered_to(out_path);
                        };
                    }
                    eid_save_ctx.open=false;
                }

                // 메뉴 바깥 클릭 시 닫기
                if(!ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
                   (ImGui::IsMouseClicked(ImGuiMouseButton_Left)||ImGui::IsMouseClicked(ImGuiMouseButton_Right))){
                    eid_save_ctx.open=false;
                }
                ImGui::End();
                ImGui::PopStyleColor();
                ImGui::PopStyleVar(2);
            }

            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        } // end Signal Analysis overlay

        // ╔══════════════════════════════════════════════════════════════════╗
        // ║  DIGITAL DECODE 오버레이 (Q키 토글)                              ║
        // ╚══════════════════════════════════════════════════════════════════╝
        if(v.digi_decode_panel_open){
            ImGui::SetNextWindowPos(ImVec2(0,0));
            ImGui::SetNextWindowSize(ImVec2(disp_w, disp_h));
            if(top_ov() == 3) ImGui::SetNextWindowFocus();
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.03f,0.03f,0.05f,0.97f));
            ImGui::Begin("##digi_overlay", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

            ImDrawList* dfg = ImGui::GetWindowDrawList();
            const float DB_H = 22.f;

            // 서브바 배경
            dfg->AddRectFilled(ImVec2(0,0), ImVec2(disp_w, DB_H), IM_COL32(25,25,35,255));
            dfg->AddLine(ImVec2(0,DB_H-1), ImVec2(disp_w,DB_H-1), IM_COL32(50,50,65,255));

            // 타이틀
            dfg->AddText(ImVec2(8, 3), IM_COL32(200,200,220,255), "DIGITAL DECODE");

            // 탭 바
            float tab_y = DB_H;
            static int digi_tab = 0;  // 0=AIS, 1=ADS-B, 2=UAV, 3=DEMOD
            static const char* tab_names[] = {"AIS", "ADS-B", "UAV", "DEMOD"};
            static const ImU32 tab_colors[] = {
                IM_COL32(80,220,255,255),   // AIS: 시안
                IM_COL32(255,160,80,255),   // ADS-B: 주황
                IM_COL32(120,255,120,255),  // UAV: 녹색
                IM_COL32(255,100,200,255),  // DEMOD: 핑크
            };

            float tab_w = disp_w / 4.0f;
            for(int t = 0; t < 4; t++){
                float tx0 = t * tab_w;
                float tx1 = tx0 + tab_w;
                bool selected = (t == digi_tab);
                ImU32 bg = selected ? IM_COL32(35,35,50,255) : IM_COL32(18,18,25,255);
                dfg->AddRectFilled(ImVec2(tx0, tab_y), ImVec2(tx1, tab_y+24), bg);
                if(selected)
                    dfg->AddLine(ImVec2(tx0, tab_y+23), ImVec2(tx1, tab_y+23), tab_colors[t], 2.f);
                ImVec2 tsz = ImGui::CalcTextSize(tab_names[t]);
                dfg->AddText(ImVec2(tx0 + (tab_w-tsz.x)/2, tab_y+4),
                    selected ? tab_colors[t] : IM_COL32(100,100,120,255), tab_names[t]);
                // 클릭 감지
                if(ImGui::IsMouseClicked(0)){
                    ImVec2 mp = ImGui::GetMousePos();
                    if(mp.x >= tx0 && mp.x < tx1 && mp.y >= tab_y && mp.y < tab_y+24)
                        digi_tab = t;
                }
            }

            // 콘텐츠 영역
            float content_y = tab_y + 26;
            float content_h = disp_h - content_y;

            ImGui::SetCursorScreenPos(ImVec2(0, content_y));
            ImGui::BeginChild("##digi_content", ImVec2(disp_w, content_h), false);
            {
                std::lock_guard<std::mutex> lk(v.digi_log_mtx);
                if(v.digi_log_buf[digi_tab].empty()){
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f,0.4f,0.5f,1.f));
                    if(digi_tab == 0) ImGui::TextWrapped("  AIS: No decoded messages yet. Set channel filter and press D > AIS to start.");
                    else if(digi_tab == 1) ImGui::TextWrapped("  ADS-B decoder: not implemented");
                    else if(digi_tab == 2) ImGui::TextWrapped("  UAV decoder: not implemented");
                    else ImGui::TextWrapped("  DEMOD: No data yet. Set channel filter, press D > DEMOD, select modulation and baud rate.");
                    ImGui::PopStyleColor();
                } else {
                    for(auto& e : v.digi_log_buf[digi_tab]){
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.75f,0.85f,0.9f,1.f));
                        ImGui::Selectable(e.msg, false, ImGuiSelectableFlags_AllowDoubleClick);
                        ImGui::PopStyleColor();
                        // 우클릭으로 복사
                        if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(1)){
                            ImGui::SetClipboardText(e.msg);
                        }
                    }
                }
            }
            if(v.digi_log_scroll[digi_tab]){
                ImGui::SetScrollHereY(1.f);
                v.digi_log_scroll[digi_tab] = false;
            }
            ImGui::EndChild();

            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        } // end DIGITAL DECODE overlay

        // ╔══════════════════════════════════════════════════════════════════╗
        // ║  LOG 오버레이 (L키 토글)                                        ║
        // ╚══════════════════════════════════════════════════════════════════╝
        if(v.log_panel_open){
            ImGui::SetNextWindowPos(ImVec2(0,0));
            ImGui::SetNextWindowSize(ImVec2(disp_w, disp_h));
            if(top_ov() == 2) ImGui::SetNextWindowFocus();
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.03f,0.03f,0.05f,0.97f));
            ImGui::Begin("##log_overlay", nullptr,
                ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
                ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

            ImDrawList* lfg = ImGui::GetWindowDrawList();
            const float LB_H = 22.f;
            float lb_y0 = 0, lb_y1 = LB_H;

            // 서브바 배경
            lfg->AddRectFilled(ImVec2(0, lb_y0), ImVec2(disp_w, lb_y1), IM_COL32(25,25,35,255));
            lfg->AddLine(ImVec2(0,lb_y1-1), ImVec2(disp_w,lb_y1-1), IM_COL32(50,50,65,255));

            // 타이틀
            float lbl_ty = lb_y0 + 3.f;
            lfg->AddText(ImVec2(8, lbl_ty), IM_COL32(200,200,220,255), "LOG");

            // 3칸 영역 계산
            float col_w = disp_w / 2.0f;
            float ca_y0 = lb_y1;
            float ca_h = disp_h - lb_y1;
            static const char* col_names[] = {"HOST", "SERVER"};
            static const ImU32 col_colors[] = {
                IM_COL32(80,255,160,255),  // HOST: 녹색
                IM_COL32(255,200,80,255),  // SERVER: 주황
            };

            // 네트워크 속도 계산 (1초 간격)
            static double net_last_time = 0;
            static uint64_t net_last_tx = 0, net_last_rx = 0;
            static float net_tx_speed = 0, net_rx_speed = 0;
            double now_t = ImGui::GetTime();
            if(now_t - net_last_time >= 1.0){
                uint64_t cur_tx = 0, cur_rx = 0;
                if(v.net_srv){
                    auto st = v.net_srv->collect_stats();
                    cur_tx = st.tx_bytes; cur_rx = st.rx_bytes;
                }
                if(v.net_cli){
                    auto st = v.net_cli->collect_stats();
                    cur_tx += st.tx_bytes; cur_rx += st.rx_bytes;
                }
                double dt = now_t - net_last_time;
                if(dt > 0 && net_last_time > 0){
                    net_tx_speed = (float)((cur_tx - net_last_tx) / dt);
                    net_rx_speed = (float)((cur_rx - net_last_rx) / dt);
                }
                net_last_tx = cur_tx; net_last_rx = cur_rx;
                net_last_time = now_t;
            }

            for(int c = 0; c < 2; c++){
                float cx0 = c * col_w;
                float cx1 = cx0 + col_w;

                // 칸 구분선
                if(c > 0)
                    lfg->AddLine(ImVec2(cx0, ca_y0), ImVec2(cx0, disp_h), IM_COL32(50,50,65,255));

                // 칸 헤더
                lfg->AddRectFilled(ImVec2(cx0, ca_y0), ImVec2(cx1, ca_y0+20), IM_COL32(20,20,30,255));
                ImVec2 nsz = ImGui::CalcTextSize(col_names[c]);
                lfg->AddText(ImVec2(cx0 + (col_w-nsz.x)/2, ca_y0+2), col_colors[c], col_names[c]);

                // 네트워크 속도 (하단)
                float bot_y = disp_h - 20.f;
                lfg->AddRectFilled(ImVec2(cx0, bot_y), ImVec2(cx1, disp_h), IM_COL32(15,15,22,255));
                lfg->AddLine(ImVec2(cx0, bot_y), ImVec2(cx1, bot_y), IM_COL32(50,50,65,255));
                auto fmt_speed = [](float bps) -> std::string {
                    char b[32];
                    if(bps >= 1e6f) snprintf(b,sizeof(b),"%.1f MB/s", bps/1e6f);
                    else if(bps >= 1e3f) snprintf(b,sizeof(b),"%.1f KB/s", bps/1e3f);
                    else snprintf(b,sizeof(b),"%.0f B/s", bps);
                    return b;
                };
                if(c == 0){ // HOST
                    if(v.net_srv){
                        auto s = fmt_speed(net_tx_speed);
                        char line[64]; snprintf(line,sizeof(line),"TX %s  RX %s", s.c_str(), fmt_speed(net_rx_speed).c_str());
                        lfg->AddText(ImVec2(cx0+6, bot_y+3), IM_COL32(120,120,140,255), line);
                    } else if(v.net_cli){
                        auto s = fmt_speed(net_rx_speed);
                        char line[64]; snprintf(line,sizeof(line),"RX %s  TX %s", s.c_str(), fmt_speed(net_tx_speed).c_str());
                        lfg->AddText(ImVec2(cx0+6, bot_y+3), IM_COL32(120,120,140,255), line);
                    }
                }

                // 로그 내용 (스크롤 가능 자식 창)
                float log_y0 = ca_y0 + 22;
                float log_h = bot_y - log_y0;
                char child_id[16]; snprintf(child_id,sizeof(child_id),"##log_c%d",c);
                ImGui::SetCursorScreenPos(ImVec2(cx0, log_y0));
                ImGui::BeginChild(child_id, ImVec2(col_w, log_h), false);
                {
                    std::lock_guard<std::mutex> lk(v.log_mtx);
                    for(auto& e : v.log_buf[c]){
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f,0.7f,0.75f,1.f));
                        ImGui::Selectable(e.msg, false, ImGuiSelectableFlags_AllowDoubleClick);
                        ImGui::PopStyleColor();
                        // 우클릭으로 복사
                        if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(1)){
                            ImGui::SetClipboardText(e.msg);
                        }
                    }
                }
                if(v.log_scroll[c]){
                    ImGui::SetScrollHereY(1.f);
                    v.log_scroll[c] = false;
                }
                ImGui::EndChild();
            }

            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        } // end LOG overlay

        ImGui::Render();
        int dw2,dh2; glfwGetFramebufferSize(win,&dw2,&dh2);
        glViewport(0,0,dw2,dh2);
        glClearColor(0.1f,0.1f,0.1f,1); glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    } // end if(!do_logout) - skip SDR init+main loop when logout from globe

    // 재연결 스레드 완료 대기 (central_cli 참조하는 detached 스레드 보호)
    for(int w=0; w<100 && reconn_busy.load(); w++)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    v.is_running = false;
    // RTL-SDR: async read 즉시 취소 > cap thread 블로킹 해제
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
    if(v.net_srv){ v.net_srv->stop(); delete v.net_srv; v.net_srv=nullptr; }
    if(v.net_cli){ v.net_cli->disconnect(); delete v.net_cli; v.net_cli=nullptr; }
    if(!v.remote_mode && cap.joinable()) cap.join();
    if(v.dev_blade){
        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
    }
    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
    if(v.waterfall_texture) glDeleteTextures(1,&v.waterfall_texture);
    v.sa_cleanup();
    v.eid_cleanup();      // eid_thread join 보장
    v.ais_pipe_stop();    // ais_pipe_reader_thr + python 자식 정리 (no-op if not alive)

    // ── record/ > private/ 이동 (세션 종료 시) ─────────────────────────
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

    g_log_viewer = nullptr; // detached thread에서의 use-after-free 방지
    } while(do_main_menu && !glfwWindowShouldClose(win)); // ── 모드선택 outer 루프 끝

    if(do_logout){
        // 로그인 화면으로 돌아가기 (프로세스 재시작, 세션 삭제)
        bewe_log_push(2,"Logout: restarting to login screen...\n");
        ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
        char* argv0[] = {(char*)"/proc/self/exe", nullptr};
        execv("/proc/self/exe", argv0);
        _exit(0);
    }

    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(); glfwTerminate();
    bewe_log_push(2,"Closed\n");
}