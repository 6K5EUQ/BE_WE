#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include <ctime>
#include <chrono>

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

    // Create temporary channel
    float half_bw = e.bw_khz / 2000.0f;
    channels[slot].reset_slot();
    channels[slot].s = e.freq_mhz - half_bw;
    channels[slot].e = e.freq_mhz + half_bw;
    channels[slot].filter_active = true;
    strncpy(channels[slot].owner, "SCHED", 31);
    channels[slot].audio_mask.store(0x1);
    e.temp_ch_idx = slot;

    // Start demodulation (FM by default for squelch gate to work)
    start_dem(slot, Channel::DM_FM);

    // Wait briefly for demod thread to initialize and squelch to calibrate
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

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
}
