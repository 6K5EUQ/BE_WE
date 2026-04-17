#include "fft_viewer.hpp"
#include "net_server.hpp"
#include "bewe_paths.hpp"
#include "login.hpp"
#include <ctime>
#include <chrono>
#include <thread>
#include <cstdio>

// ── sched 리스트를 SCHED_SYNC 패킷으로 변환 ──────────────────────────────
// 호출자는 sched_mtx를 잡은 상태여야 함
static PktSchedSync build_sched_sync_pkt(const std::vector<FFTViewer::SchedEntry>& list){
    PktSchedSync pkt{};
    int n = (int)list.size();
    if(n > MAX_SCHED_ENTRIES) n = MAX_SCHED_ENTRIES;
    pkt.count = (uint8_t)n;
    for(int i=0; i<n; i++){
        const auto& e = list[i];
        auto& se = pkt.entries[i];
        se.valid        = 1;
        se.status       = (uint8_t)e.status;
        se.op_index     = e.op_index;
        se.start_time   = (int64_t)e.start_time;
        se.duration_sec = e.duration_sec;
        se.freq_mhz     = e.freq_mhz;
        se.bw_khz       = e.bw_khz;
        strncpy(se.operator_name, e.operator_name, sizeof(se.operator_name)-1);
    }
    return pkt;
}

// 외부에서 호출 가능한 헬퍼 (cli_host.cpp / ui.cpp에서 사용)
// sched_mtx는 이 함수가 직접 잡음
void FFTViewer::broadcast_sched_list(){
    if(!net_srv) return;
    PktSchedSync pkt;
    {
        std::lock_guard<std::mutex> lk(sched_mtx);
        pkt = build_sched_sync_pkt(sched_entries);
    }
    net_srv->broadcast_sched_sync(pkt);
}

// Overlap 검사 — [start, start+dur) 구간이 기존 WAITING/RECORDING entry와 겹치는지
bool FFTViewer::sched_has_overlap(time_t start, float dur) const {
    time_t a0 = start;
    time_t a1 = start + (time_t)dur;
    for(const auto& e : sched_entries){
        if(e.status == SchedEntry::DONE || e.status == SchedEntry::FAILED) continue;
        time_t b0 = e.start_time;
        time_t b1 = b0 + (time_t)e.duration_sec;
        if(a0 < b1 && b0 < a1) return true;
    }
    return false;
}

void FFTViewer::sched_tick(){
    std::lock_guard<std::mutex> lk(sched_mtx);
    time_t now = time(nullptr);

    // Active recording: check if duration expired
    if(sched_active_idx >= 0 && sched_active_idx < (int)sched_entries.size()){
        auto& e = sched_entries[sched_active_idx];
        if(e.status == SchedEntry::RECORDING){
            float elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - e.rec_started).count();
            if(elapsed >= e.duration_sec)
                sched_stop_entry(sched_active_idx);
        }
    }

    // No active recording: find next WAITING entry
    if(sched_active_idx < 0){
        for(int i = 0; i < (int)sched_entries.size(); i++){
            auto& e = sched_entries[i];
            if(e.status != SchedEntry::WAITING) continue;
            // Entirely missed?
            if(now > e.start_time + (time_t)e.duration_sec){
                e.status = SchedEntry::FAILED;
                bewe_log_push(0, "[SCHED] Entry %d missed (time passed)\n", i);
                continue;
            }
            // Time to start?
            if(now >= e.start_time){
                sched_start_entry(i);
                break;
            }
        }
    }
}

void FFTViewer::sched_start_entry(int idx){
    auto& e = sched_entries[idx];

    // Validate
    if(remote_mode){ e.status=SchedEntry::FAILED; bewe_log_push(0,"[SCHED] Failed: JOIN mode\n"); return; }
    if(!dev_blade && !dev_rtl){ e.status=SchedEntry::FAILED; bewe_log_push(0,"[SCHED] Failed: no SDR\n"); return; }

    // Save current frequency
    sched_saved_cf = (float)(header.center_frequency / 1e6);

    // Change frequency
    set_frequency(e.freq_mhz);
    bewe_log_push(0, "[SCHED] Freq > %.3f MHz\n", e.freq_mhz);

    // Find free channel slot
    int slot = -1;
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(!channels[i].filter_active){ slot = i; break; }
    }
    if(slot < 0){
        e.status = SchedEntry::FAILED;
        set_frequency(sched_saved_cf);
        bewe_log_push(0, "[SCHED] Failed: no free channel slot\n");
        return;
    }

    // Create temporary channel — owner = 예약자 이름
    float half_bw = e.bw_khz / 2000.0f;
    channels[slot].reset_slot();
    channels[slot].s = e.freq_mhz - half_bw;
    channels[slot].e = e.freq_mhz + half_bw;
    channels[slot].filter_active = true;
    const char* owner_src = (e.operator_name[0]) ? e.operator_name : "SCHED";
    strncpy(channels[slot].owner, owner_src, 31);
    channels[slot].audio_mask.store(0x1);
    e.temp_ch_idx = slot;

    // Start demodulation (FM by default for squelch gate to work)
    start_dem(slot, Channel::DM_FM);

    // Wait briefly for demod thread to initialize and squelch to calibrate
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 예약 녹음: squelch 무관하게 전 구간 녹음
    channels[slot].iq_rec_force_all.store(true);
    // Start IQ recording
    start_iq_rec(slot);

    e.status = SchedEntry::RECORDING;
    e.rec_started = std::chrono::steady_clock::now();
    sched_active_idx = idx;

    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    bewe_log_push(0, "[SCHED] Recording started: CH%d %.3f MHz BW=%.0f kHz dur=%.0fs\n",
                  slot, e.freq_mhz, e.bw_khz, e.duration_sec);
}

void FFTViewer::sched_stop_entry(int idx){
    auto& e = sched_entries[idx];
    int slot = e.temp_ch_idx;

    // 녹음 경로와 메타정보 스냅샷 (stop 이후 채널 reset되기 전에 캡처)
    std::string iq_path;
    if(slot >= 0 && slot < MAX_CHANNELS)
        iq_path = channels[slot].iq_rec_path;
    char entry_op[32] = {};
    strncpy(entry_op, e.operator_name, sizeof(entry_op)-1);
    time_t entry_start = e.start_time;
    float  entry_dur   = e.duration_sec;
    float  entry_freq  = e.freq_mhz;
    float  entry_bw    = e.bw_khz;

    // Stop IQ recording
    if(slot >= 0 && slot < MAX_CHANNELS)
        stop_iq_rec(slot);

    // Tear down temporary channel
    if(slot >= 0 && slot < MAX_CHANNELS){
        stop_dem(slot);
        stop_digi(slot);
        channels[slot].reset_slot();
    }

    // Restore frequency
    set_frequency(sched_saved_cf);
    bewe_log_push(0, "[SCHED] Freq restored > %.3f MHz\n", sched_saved_cf);

    e.status = SchedEntry::DONE;
    sched_active_idx = -1;

    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    bewe_log_push(0, "[SCHED] Recording complete: entry %d\n", idx);

    // 자동 DB 업로드 (별도 스레드) — 파일 finalize 대기 후 전송
    if(!iq_path.empty() && sched_db_upload_fn){
        auto upload_fn = sched_db_upload_fn;
        std::thread([upload_fn, iq_path, entry_op, entry_start, entry_dur, entry_freq, entry_bw](){
            // 파일 flush/close 마무리 대기
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            char info[512];
            char tbuf[64] = {};
            struct tm tm_utc; gmtime_r(&entry_start, &tm_utc);
            strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
            snprintf(info, sizeof(info),
                "This is a scheduled recording file.\n"
                "Scheduled by: %s\n"
                "Start time (UTC): %s\n"
                "Duration: %.0f sec\n"
                "Center frequency: %.4f MHz\n"
                "Bandwidth: %.1f kHz\n",
                entry_op[0] ? entry_op : "?", tbuf, entry_dur, entry_freq, entry_bw);
            upload_fn(iq_path, entry_op, info);
        }).detach();
    }
}
