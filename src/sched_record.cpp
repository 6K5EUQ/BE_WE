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

// Overlap 검사 — pre-arm 윈도우 포함 [start - PRE_ARM, start+dur) 가 기존 active entry와 겹치는지
bool FFTViewer::sched_has_overlap(time_t start, float dur) const {
    time_t a0 = start - (time_t)SCHED_PRE_ARM_SEC;
    time_t a1 = start + (time_t)dur;
    for(const auto& e : sched_entries){
        if(e.status == SchedEntry::DONE || e.status == SchedEntry::FAILED) continue;
        time_t b0 = e.start_time - (time_t)SCHED_PRE_ARM_SEC;
        time_t b1 = e.start_time + (time_t)e.duration_sec;
        if(a0 < b1 && b0 < a1) return true;
    }
    return false;
}

void FFTViewer::sched_tick(){
    std::lock_guard<std::mutex> lk(sched_mtx);
    time_t now = time(nullptr);

    // Active slot (ARMED or RECORDING)
    if(sched_active_idx >= 0 && sched_active_idx < (int)sched_entries.size()){
        auto& e = sched_entries[sched_active_idx];
        if(e.status == SchedEntry::ARMED){
            if(now >= e.start_time)
                sched_begin_rec(sched_active_idx);
        } else if(e.status == SchedEntry::RECORDING){
            float elapsed = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - e.rec_started).count();
            if(elapsed >= e.duration_sec)
                sched_stop_entry(sched_active_idx);
        }
    }

    // No active slot: find next WAITING entry eligible for pre-arm
    if(sched_active_idx < 0){
        for(int i = 0; i < (int)sched_entries.size(); i++){
            auto& e = sched_entries[i];
            if(e.status != SchedEntry::WAITING) continue;
            // Entirely missed?
            if(now > e.start_time + (time_t)e.duration_sec){
                e.status = SchedEntry::FAILED;
                broadcast_sched_list_locked();
                bewe_log_push(0, "[SCHED] Entry %d missed (time passed)\n", i);
                continue;
            }
            // Pre-arm window reached?
            if(now >= e.start_time - (time_t)SCHED_PRE_ARM_SEC){
                sched_arm_entry(i);
                break;
            }
        }
    }
}

// Pre-arm: start_time - SCHED_PRE_ARM_SEC 시점에 호출됨.
// SDR 튠(DC 오프셋 적용), 채널 할당, demod 시작만 수행. IQ 기록은 아직.
// 이 구간 동안 PLL lock / IIR 과도응답 / squelch 보정이 안정화됨.
void FFTViewer::sched_arm_entry(int idx){
    auto& e = sched_entries[idx];

    if(remote_mode){ e.status=SchedEntry::FAILED; broadcast_sched_list_locked(); bewe_log_push(0,"[SCHED] Failed: JOIN mode\n"); return; }
    bool sdr_ok = (dev_blade != nullptr)
               || (dev_rtl   != nullptr)
               || (hw.type == HWType::PLUTO && pluto_ctx != nullptr);
    if(!sdr_ok){ e.status=SchedEntry::FAILED; broadcast_sched_list_locked(); bewe_log_push(0,"[SCHED] Failed: no SDR\n"); return; }

    sched_saved_cf = (float)(header.center_frequency / 1e6);

    // DC 오프셋: SDR CF를 target + offset 로 이동 → DC 스파이크가 채널 베이스밴드에서 -offset 위치로
    // 밀려 채널 LPF 바깥이 되어 제거됨.
    float bw_mhz     = e.bw_khz / 1000.0f;
    float sdr_sr_mhz = (float)(header.sample_rate) / 1e6f;
    float min_off    = bw_mhz;                                    // 채널 필터 바깥으로 밀어냄
    float max_off    = sdr_sr_mhz * 0.45f - bw_mhz * 0.5f;        // SDR 유효 대역 헤드룸
    float offset;
    if(max_off <= 0.f){
        offset = 0.f;
        bewe_log_push(0,"[SCHED] Warn: BW exceeds SDR headroom; DC may intrude\n");
    } else {
        offset = (max_off < min_off) ? max_off : min_off;
    }
    float sdr_cf = e.freq_mhz + offset;

    set_frequency(sdr_cf);
    bewe_log_push(0, "[SCHED] ARM: SDR CF %.3f MHz (target %.3f + DC offset %.3f)\n",
                  sdr_cf, e.freq_mhz, offset);

    int slot = -1;
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(!channels[i].filter_active){ slot = i; break; }
    }
    if(slot < 0){
        e.status = SchedEntry::FAILED;
        set_frequency(sched_saved_cf);
        broadcast_sched_list_locked();
        bewe_log_push(0, "[SCHED] Failed: no free channel slot\n");
        return;
    }

    // 채널 [s, e]는 target 기준 (demod 오실레이터가 SDR CF에서 target까지 믹싱)
    float half_bw = e.bw_khz / 2000.0f;
    channels[slot].reset_slot();
    channels[slot].s = e.freq_mhz - half_bw;
    channels[slot].e = e.freq_mhz + half_bw;
    channels[slot].filter_active = true;
    const char* owner_src = (e.operator_name[0]) ? e.operator_name : "SCHED";
    strncpy(channels[slot].owner, owner_src, 31);
    channels[slot].audio_mask.store(0x1);
    e.temp_ch_idx = slot;

    // 기록되는 I/Q는 discriminator 전의 채널 베이스밴드. demod 스레드는 샘플 공급자로서 필수.
    start_dem(slot, Channel::DM_FM);
    channels[slot].iq_rec_force_all.store(true);

    e.status = SchedEntry::ARMED;
    sched_active_idx = idx;

    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    broadcast_sched_list_locked();
    bewe_log_push(0, "[SCHED] ARMED: CH%d %.3f MHz BW=%.0f kHz dur=%.0fs (T-%.1fs)\n",
                  slot, e.freq_mhz, e.bw_khz, e.duration_sec, SCHED_PRE_ARM_SEC);
}

// sched_mtx를 이미 잡은 상태에서 호출 가능한 브로드캐스트 (내부 잠금 없음)
void FFTViewer::broadcast_sched_list_locked(){
    if(!net_srv) return;
    PktSchedSync pkt = build_sched_sync_pkt(sched_entries);
    net_srv->broadcast_sched_sync(pkt);
}

// start_time 도달 시 호출 — ARM된 엔트리의 IQ 기록만 실제로 시작.
void FFTViewer::sched_begin_rec(int idx){
    auto& e = sched_entries[idx];
    int slot = e.temp_ch_idx;
    if(slot < 0 || slot >= MAX_CHANNELS){
        e.status = SchedEntry::FAILED;
        sched_active_idx = -1;
        broadcast_sched_list_locked();
        return;
    }
    start_iq_rec(slot);
    if(!channels[slot].iq_rec_on.load()){
        bewe_log_push(0, "[SCHED] REC start failed: CH%d\n", slot);
        stop_dem(slot);
        channels[slot].reset_slot();
        set_frequency(sched_saved_cf);
        e.status = SchedEntry::FAILED;
        sched_active_idx = -1;
        if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
        broadcast_sched_list_locked();
        return;
    }
    e.status      = SchedEntry::RECORDING;
    e.rec_started = std::chrono::steady_clock::now();
    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    broadcast_sched_list_locked();
    bewe_log_push(0, "[SCHED] REC start: CH%d %.3f MHz dur=%.0fs\n",
                  slot, e.freq_mhz, e.duration_sec);
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

    // 1) demod 스레드를 먼저 join → maybe_rec_iq() 호출이 더 이상 일어나지 않음
    //    (이 순서가 아니면 stop_iq_rec와 fwrite 사이 race로 frame 카운트 어긋남)
    if(slot >= 0 && slot < MAX_CHANNELS){
        stop_dem(slot);
        stop_digi(slot);
    }
    // 2) IQ 녹음 finalize (헤더 갱신 + .info Duration 갱신)
    if(slot >= 0 && slot < MAX_CHANNELS)
        stop_iq_rec(slot);

    if(slot >= 0 && slot < MAX_CHANNELS)
        channels[slot].reset_slot();

    // Restore frequency
    set_frequency(sched_saved_cf);
    bewe_log_push(0, "[SCHED] Freq restored > %.3f MHz\n", sched_saved_cf);

    e.status = SchedEntry::DONE;
    sched_active_idx = -1;

    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
    broadcast_sched_list_locked();
    bewe_log_push(0, "[SCHED] Recording complete: entry %d\n", idx);

    // 자동 DB 업로드 (별도 스레드) — 파일 finalize 대기 후 전송
    if(!iq_path.empty() && sched_db_upload_fn){
        auto upload_fn = sched_db_upload_fn;
        std::thread([upload_fn, iq_path, entry_op, entry_start, entry_dur, entry_freq, entry_bw](){
            // stop_iq_rec()가 동기로 fclose+.info Duration 갱신까지 끝냈으므로 sleep 불필요.

            // stop_iq_rec()가 이미 표준 Key:Value 형식 .info를 생성/갱신했음.
            // 그 내용을 그대로 read해서 업로드 (free-form 텍스트 대신 표준 포맷 보장)
            std::string info_str;
            std::string ipath = iq_path + ".info";
            FILE* fi = fopen(ipath.c_str(), "r");
            if(fi){
                char buf[2048];
                size_t n = fread(buf, 1, sizeof(buf)-1, fi);
                buf[n] = 0;
                info_str = buf;
                fclose(fi);
            }
            // sched 특유 메타를 Notes에 추가 (info_str이 비었거나 Notes 라인 비어있으면 채움)
            char tbuf[64] = {};
            struct tm tm_utc; gmtime_r(&entry_start, &tm_utc);
            strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
            char sched_note[256];
            snprintf(sched_note, sizeof(sched_note),
                "Notes: Scheduled by %s @ %s, dur=%.0fs, BW=%.1fkHz\n",
                entry_op[0] ? entry_op : "?", tbuf, entry_dur, entry_bw);
            // Notes: 라인 교체 (없으면 끝에 append)
            std::string out;
            bool note_replaced = false;
            size_t pos = 0;
            while(pos < info_str.size()){
                size_t eol = info_str.find('\n', pos);
                if(eol == std::string::npos) eol = info_str.size();
                std::string line = info_str.substr(pos, eol - pos);
                if(!note_replaced && line.rfind("Notes:", 0) == 0){
                    out += sched_note;
                    note_replaced = true;
                } else {
                    out += line;
                    if(eol < info_str.size()) out += "\n";
                }
                pos = eol + 1;
            }
            if(!note_replaced) out += sched_note;
            (void)entry_freq; // freq는 표준 .info의 Freq 필드에 이미 들어있음
            upload_fn(iq_path, entry_op, out);
        }).detach();
    }
}
