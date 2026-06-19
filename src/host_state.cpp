#include "host_state.hpp"
#include "fft_viewer.hpp"
#include "channel.hpp"
#include "bewe_paths.hpp"
#include "json_scan.hpp"
#include "module_api.hpp"   // 디코드 모듈 on/off 상태 영속화 (bewe_modules / host_mask)
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <errno.h>

namespace HostState {

std::string file_path(const std::string& station){
    std::string s = station.empty() ? "default" : station;
    for(char& c : s) if(c=='/' || c=='\\') c='_';  // 경로 구분자 방지
    return BEWEPaths::data_dir() + "/host_state_" + s + ".json";
}

static void json_escape(std::string& out, const char* s){
    for(; *s; s++){
        char c = *s;
        if(c=='"' || c=='\\'){ out += '\\'; out += c; }
        else if((unsigned char)c < 0x20) {}
        else out += c;
    }
}

// Holding(범위 밖) 중엔 보존된 mode 가 진짜 의도된 mode.
static int effective_mode(const Channel& ch){
    if(ch.dem_paused.load() && ch.dem_paused_mode != Channel::DM_NONE)
        return (int)ch.dem_paused_mode;
    return (int)ch.mode;
}

void save(const FFTViewer& v, const std::string& station){
    char buf[256];
    float cf = (float)(v.header.center_frequency / 1e6);
    float sr = (float)(v.header.sample_rate / 1e6);
    std::string out;
    snprintf(buf, sizeof(buf),
        "{\n  \"cf_mhz\":%.6f,\n  \"sr_msps\":%.6f,\n  \"gain_db\":%.2f,\n  \"channels\": [\n",
        cf, sr, v.gain_db);
    out += buf;

    bool first = true;
    for(int i=0;i<MAX_CHANNELS;i++){
        const Channel& ch = v.channels[i];
        if(!ch.filter_active) continue;
        if(!first) out += ",\n";
        first = false;
        char own[33]={}; strncpy(own, ch.owner, 31);
        std::string oesc; json_escape(oesc, own);
        snprintf(buf, sizeof(buf),
            "    {\"s\":%.6f,\"e\":%.6f,\"mode\":%d,\"audio_mask\":%u,\"pan\":%d,\"sq\":%.2f,\"owner\":\"",
            ch.s, ch.e, effective_mode(ch),
            (unsigned)ch.audio_mask.load(), ch.pan, ch.sq_threshold.load());
        out += buf;
        out += oesc;
        // 이 채널에 켜진 디코드 모듈 id (host_mask 비트) — 재시작 시 디코드 재개용
        std::string dmods;
        for(const auto& m : bewe_modules()){
            if(!m.target_modes) continue;
            if(bewe_mod_host_mask(m.id) & (1ull<<i)){
                if(!dmods.empty()) dmods += ',';
                dmods += m.id;
            }
        }
        out += "\",\"decode_mods\":\"";
        out += dmods;          // 모듈 id 는 안전 문자 (escaping 불필요)
        out += "\"}";
    }
    out += "\n  ]\n}\n";

    std::string dir = BEWEPaths::data_dir();
    mkdir(dir.c_str(), 0755);
    std::string path = file_path(station);
    std::string tmp  = path + ".tmp";
    FILE* fp = fopen(tmp.c_str(), "w");
    if(!fp){ fprintf(stderr,"[HostState] save: cannot open %s errno=%d\n", tmp.c_str(), errno); return; }
    size_t wrote = fwrite(out.data(), 1, out.size(), fp);
    bool ok = (wrote == out.size());
    if(fflush(fp) != 0) ok = false;
    if(fclose(fp) != 0) ok = false;
    if(!ok){  // 디스크 풀 등 부분 기록 — 기존 good 파일 보존, 빈 파일로 덮지 않음
        fprintf(stderr,"[HostState] save: write incomplete, keeping old %s\n", path.c_str());
        remove(tmp.c_str());
        return;
    }
    rename(tmp.c_str(), path.c_str());  // 원자적 교체 — 부분 기록 파일 복원 방지
}

Snapshot load(const std::string& station){
    Snapshot st;
    std::string path = file_path(station);
    FILE* fp = fopen(path.c_str(), "r");
    if(!fp) return st;  // 파일 없음 → ok=false
    fseek(fp,0,SEEK_END); long sz = ftell(fp); fseek(fp,0,SEEK_SET);
    if(sz <= 0){ fclose(fp); return st; }
    std::string body(sz, 0);
    if(fread(&body[0], 1, sz, fp) != (size_t)sz){ fclose(fp); return st; }
    fclose(fp);

    JScan js{body.data(), body.data()+body.size()};
    if(!js.consume('{')) return st;
    std::string key;
    while(js.read_key(key)){
        if(key=="cf_mhz"){ double d=0; js.read_number(d); st.cf_mhz=(float)d; }
        else if(key=="sr_msps"){ double d=0; js.read_number(d); st.sr_msps=(float)d; }
        else if(key=="gain_db"){ double d=0; js.read_number(d); st.gain_db=(float)d; st.has_gain=true; }
        else if(key=="channels"){
            if(!js.consume('[')) break;
            while(!js.peek(']')){
                if(!js.consume('{')) break;
                ChanSnap c;
                std::string k;
                while(js.read_key(k)){
                    if(k=="s"){ double d=0; js.read_number(d); c.s=(float)d; }
                    else if(k=="e"){ double d=0; js.read_number(d); c.e=(float)d; }
                    else if(k=="mode"){ double d=0; js.read_number(d); c.mode=(int)d; }
                    else if(k=="audio_mask"){ double d=0; js.read_number(d); c.audio_mask=(uint32_t)d; }
                    else if(k=="pan"){ double d=0; js.read_number(d); c.pan=(int)d; }
                    else if(k=="sq"){ double d=0; js.read_number(d); c.sq=(float)d; }
                    else if(k=="owner"){ std::string s; js.read_string(s); strncpy(c.owner, s.c_str(), 31); }
                    else if(k=="decode_mods"){ std::string s; js.read_string(s); strncpy(c.decode_mods, s.c_str(), sizeof(c.decode_mods)-1); }
                    else {  // 미지 키 — 문자열/숫자/배열/객체 모두 스킵
                        if(js.peek('"')){ std::string t; js.read_string(t); }
                        else if(js.consume('[') || js.consume('{')){
                            int depth=1;
                            while(js.p<js.end && depth>0){
                                if(*js.p=='['||*js.p=='{') depth++;
                                else if(*js.p==']'||*js.p=='}') depth--;
                                js.p++;
                            }
                        } else { double d; js.read_number(d); }
                    }
                    if(js.peek('}')) break;
                }
                js.consume('}');
                if(st.n_chans < MAX_CHANNELS && c.e > c.s)
                    st.chans[st.n_chans++] = c;
            }
            js.consume(']');
        } else {
            // 미지 키 스킵
            if(js.peek('"')){ std::string t; js.read_string(t); }
            else if(js.consume('[') || js.consume('{')){
                int depth=1;
                while(js.p<js.end && depth>0){
                    if(*js.p=='['||*js.p=='{') depth++;
                    else if(*js.p==']'||*js.p=='}') depth--;
                    js.p++;
                }
            } else { double d; js.read_number(d); }
        }
    }
    st.ok = true;
    return st;
}

static inline uint64_t mix(uint64_t h, uint64_t x){
    h ^= x + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
    return h;
}
static inline uint64_t f2u(float f){ uint32_t u; memcpy(&u,&f,4); return u; }

uint64_t fingerprint(const FFTViewer& v){
    uint64_t h = 1469598103934665603ULL;
    h = mix(h, (uint64_t)v.header.center_frequency);
    h = mix(h, (uint64_t)v.header.sample_rate);
    h = mix(h, f2u(v.gain_db));
    for(int i=0;i<MAX_CHANNELS;i++){
        const Channel& ch = v.channels[i];
        if(!ch.filter_active) continue;
        h = mix(h, (uint64_t)i);
        h = mix(h, f2u(ch.s));
        h = mix(h, f2u(ch.e));
        h = mix(h, (uint64_t)effective_mode(ch));
        h = mix(h, (uint64_t)ch.audio_mask.load());
        h = mix(h, (uint64_t)(uint32_t)ch.pan);
        h = mix(h, f2u(ch.sq_threshold.load()));
        for(int k=0;k<32 && ch.owner[k]; k++) h = mix(h, (uint64_t)(unsigned char)ch.owner[k]);
    }
    // 디코드 모듈 on/off 변경도 감지 → 토글 시 저장 트리거
    for(const auto& m : bewe_modules()){
        if(!m.target_modes) continue;
        h = mix(h, (uint64_t)bewe_mod_host_mask(m.id));
    }
    return h;
}

void apply_channels(FFTViewer& v, const Snapshot& st){
    for(int i=0;i<st.n_chans && i<MAX_CHANNELS;i++){
        const ChanSnap& c = st.chans[i];
        int slot = i;
        v.stop_dem(slot);
        v.channels[slot].reset_slot();
        v.channels[slot].s = c.s;
        v.channels[slot].e = c.e;
        v.channels[slot].filter_active = true;
        strncpy(v.channels[slot].owner, c.owner, 31);
        v.channels[slot].audio_mask.store(c.audio_mask);
        v.channels[slot].pan = c.pan;
        v.channels[slot].sq_threshold.store(c.sq);
        v.local_ch_out[slot] = 3;
        Channel::DemodMode dm = (c.mode>=0 && c.mode<=2) ? (Channel::DemodMode)c.mode
                                                         : Channel::DM_NONE;
        v.channels[slot].mode = dm;
        if(dm != Channel::DM_NONE) v.start_dem(slot, dm);
        // 저장된 디코드 모듈 재개 (필터처럼 복원 — 재시작 후에도 디코드 유지).
        // LOCAL 경로 → host_apply_set → 워커 재기동 + host_mask 복구 + STATE 브로드캐스트.
        if(c.decode_mods[0]){
            char tmp[64]; strncpy(tmp, c.decode_mods, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
            for(char* tok=strtok(tmp,","); tok; tok=strtok(nullptr,","))
                bewe_mod_set_target(v, tok, bewe_mod_my_station(), slot, true);
        }
    }
    // 범위 밖 채널은 Holding 으로 (복조 중이면 mode 보존 + stop)
    v.update_dem_by_freq(v.header.center_frequency/1e6f);
}

} // namespace HostState
