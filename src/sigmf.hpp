#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// SigMF (Signal Metadata Format) IO for BE_WE IQ recordings.
//
//   data : <stem>.sigmf-data = raw interleaved int16 I,Q little-endian
//          (SigMF datatype "ci16_le"). Header-less; samples start at byte 0.
//   meta : <stem>.sigmf-meta = JSON {global, captures, annotations}.
//          BE_WE-specific fields live under the "bewe:" namespace.
//
// Replaces the legacy pair:
//   <name>.wav  (RIFF/WAVE stereo int16 + custom "bewe" chunk)
//   <name>.wav.info  (plain-text key:value sidecar)
//
// Readers (SA / EID / playback) call open_source(), which transparently handles
// both the new SigMF files and the legacy .wav (+bewe chunk) recorded earlier.
// ─────────────────────────────────────────────────────────────────────────────
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <ctime>

namespace SigMF {

// Metadata carried for one IQ recording (was bewe chunk + .info sidecar).
struct Meta {
    uint32_t    sample_rate    = 0;
    uint64_t    center_freq_hz = 0;
    int64_t     start_unix     = 0;   // capture datetime, UTC seconds
    double      bandwidth_hz   = 0;
    double      duration_s     = 0;
    std::string station;              // .info "Location"
    std::string modulation;
    std::string op;                   // .info "Operator"
    std::string target;
    std::string protocol;
    std::string recorder;
    std::string notes;
    std::string tags;
    std::string priority;
    std::string signal_strength;
};

// ── path helpers ─────────────────────────────────────────────────────────────
inline std::string strip_iq_ext(const std::string& p){
    size_t dot = p.find_last_of('.');
    if(dot == std::string::npos) return p;
    std::string e = p.substr(dot);
    if(e == ".sigmf-data" || e == ".sigmf-meta" || e == ".wav") return p.substr(0, dot);
    return p;
}
inline std::string data_path(const std::string& any){ return strip_iq_ext(any) + ".sigmf-data"; }
inline std::string meta_path(const std::string& any){ return strip_iq_ext(any) + ".sigmf-meta"; }
inline bool is_sigmf_data(const std::string& p){
    size_t dot = p.find_last_of('.');
    return dot != std::string::npos && p.compare(dot, std::string::npos, ".sigmf-data") == 0;
}

// ── JSON write ───────────────────────────────────────────────────────────────
inline void json_esc(std::string& o, const std::string& s){
    for(char c : s){
        switch(c){
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:   o += c;
        }
    }
}

// Write <stem>.sigmf-meta for the recording identified by any_path
// (.sigmf-data / .sigmf-meta / .wav all map to the same stem).
inline bool write_meta(const std::string& any_path, const Meta& m){
    std::string mp = meta_path(any_path);
    FILE* f = fopen(mp.c_str(), "w");
    if(!f) return false;

    char dt[40] = {};
    if(m.start_unix > 0){
        time_t t = (time_t)m.start_unix;
        struct tm g; gmtime_r(&t, &g);
        strftime(dt, sizeof(dt), "%Y-%m-%dT%H:%M:%SZ", &g);
    }

    std::string o;
    char num[80];
    auto kv_str = [&](const char* k, const std::string& v){
        o += "    \""; o += k; o += "\": \""; json_esc(o, v); o += "\",\n";
    };
    o += "{\n  \"global\": {\n";
    o += "    \"core:datatype\": \"ci16_le\",\n";
    snprintf(num, sizeof(num), "    \"core:sample_rate\": %u,\n", m.sample_rate); o += num;
    o += "    \"core:version\": \"1.0.0\",\n";
    if(!m.recorder.empty()){ o += "    \"core:recorder\": \""; json_esc(o, m.recorder); o += "\",\n"; }
    kv_str("bewe:station",         m.station);
    kv_str("bewe:modulation",      m.modulation);
    kv_str("bewe:operator",        m.op);
    kv_str("bewe:target",          m.target);
    kv_str("bewe:protocol",        m.protocol);
    kv_str("bewe:signal_strength", m.signal_strength);
    kv_str("bewe:tags",            m.tags);
    kv_str("bewe:priority",        m.priority);
    kv_str("bewe:notes",           m.notes);
    snprintf(num, sizeof(num), "    \"bewe:bandwidth_hz\": %.1f,\n", m.bandwidth_hz); o += num;
    snprintf(num, sizeof(num), "    \"bewe:duration_s\": %.1f\n",    m.duration_s);   o += num;
    o += "  },\n  \"captures\": [\n    {\n      \"core:sample_start\": 0,\n";
    snprintf(num, sizeof(num), "      \"core:frequency\": %llu",
             (unsigned long long)m.center_freq_hz); o += num;
    if(dt[0]){ o += ",\n      \"core:datetime\": \""; o += dt; o += "\"\n"; }
    else       o += "\n";
    o += "    }\n  ],\n  \"annotations\": []\n}\n";

    fwrite(o.data(), 1, o.size(), f);
    fclose(f);
    return true;
}

// ── JSON read (flat scan — our keys are unique across the doc) ────────────────
inline bool meta_find_num(const std::string& s, const char* key, double& out){
    std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);                 if(p == std::string::npos) return false;
    p = s.find(':', p + k.size());        if(p == std::string::npos) return false;
    out = atof(s.c_str() + p + 1);        return true;
}
inline bool meta_find_str(const std::string& s, const char* key, std::string& out){
    std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);                 if(p == std::string::npos) return false;
    p = s.find(':', p + k.size());        if(p == std::string::npos) return false;
    p = s.find('"', p + 1);               if(p == std::string::npos) return false;
    std::string v; size_t q = p + 1;
    while(q < s.size() && s[q] != '"'){
        if(s[q] == '\\' && q + 1 < s.size()){
            char c = s[++q];
            v += (c=='n') ? '\n' : (c=='r') ? '\r' : (c=='t') ? '\t' : c;
        } else v += s[q];
        q++;
    }
    out = v; return true;
}

// Read <stem>.sigmf-meta. Returns false if the meta file is absent.
inline bool read_meta(const std::string& any_path, Meta& m){
    std::string mp = meta_path(any_path);
    FILE* f = fopen(mp.c_str(), "rb");
    if(!f) return false;
    std::string buf; char tmp[4096]; size_t r;
    while((r = fread(tmp, 1, sizeof(tmp), f)) > 0) buf.append(tmp, r);
    fclose(f);

    double d;
    if(meta_find_num(buf, "core:sample_rate", d)) m.sample_rate    = (uint32_t)d;
    if(meta_find_num(buf, "core:frequency",   d)) m.center_freq_hz = (uint64_t)d;
    if(meta_find_num(buf, "bewe:bandwidth_hz",d)) m.bandwidth_hz   = d;
    if(meta_find_num(buf, "bewe:duration_s",  d)) m.duration_s     = d;
    std::string dts;
    if(meta_find_str(buf, "core:datetime", dts) && !dts.empty()){
        struct tm g; memset(&g, 0, sizeof(g));
        if(strptime(dts.c_str(), "%Y-%m-%dT%H:%M:%SZ", &g)) m.start_unix = (int64_t)timegm(&g);
    }
    meta_find_str(buf, "core:recorder",        m.recorder);
    meta_find_str(buf, "bewe:station",         m.station);
    meta_find_str(buf, "bewe:modulation",      m.modulation);
    meta_find_str(buf, "bewe:operator",        m.op);
    meta_find_str(buf, "bewe:target",          m.target);
    meta_find_str(buf, "bewe:protocol",        m.protocol);
    meta_find_str(buf, "bewe:signal_strength", m.signal_strength);
    meta_find_str(buf, "bewe:tags",            m.tags);
    meta_find_str(buf, "bewe:priority",        m.priority);
    meta_find_str(buf, "bewe:notes",           m.notes);
    return true;
}

// Rewrite only duration_s in an existing meta (other fields preserved by
// re-reading then re-writing). Called when a live recording closes.
inline void update_duration(const std::string& any_path, double duration_s){
    Meta m;
    if(!read_meta(any_path, m)) return;
    m.duration_s = duration_s;
    write_meta(any_path, m);
}

// ── unified IQ/audio source open (readers) ───────────────────────────────────
// On success f is positioned at the first sample. nch: 2=IQ(ci16/stereo),
// 1=mono audio (legacy demod .wav). data_size is in bytes.
struct Source {
    FILE*    f              = nullptr;
    uint32_t sample_rate    = 0;
    uint64_t center_freq_hz = 0;
    int64_t  start_unix     = 0;
    int      nch            = 2;
    long     data_offset    = 0;
    long     data_size      = 0;
};

inline bool open_source(const std::string& path, Source& s){
    if(is_sigmf_data(path)){
        Meta m;
        if(!read_meta(path, m)) return false;          // meta required for raw data
        FILE* f = fopen(path.c_str(), "rb");
        if(!f) return false;
        fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
        s.f = f; s.sample_rate = m.sample_rate; s.center_freq_hz = m.center_freq_hz;
        s.start_unix = m.start_unix; s.nch = 2; s.data_offset = 0; s.data_size = sz;
        return s.sample_rate > 0 && sz > 0;
    }

    // ── legacy RIFF/WAVE (+ optional "bewe" chunk, sibling or inside data) ──
    FILE* f = fopen(path.c_str(), "rb");
    if(!f) return false;
    char riff[4] = {}, wave[4] = {};
    if(fread(riff, 1, 4, f) != 4){ fclose(f); return false; }
    fseek(f, 4, SEEK_CUR);
    if(fread(wave, 1, 4, f) != 4 ||
       strncmp(riff, "RIFF", 4) != 0 || strncmp(wave, "WAVE", 4) != 0){ fclose(f); return false; }

    uint16_t ch = 0, bits = 0; uint32_t sr = 0;
    uint64_t cf = 0; int64_t tm = 0; long doff = 0, dsz = 0;
    while(true){
        char id[4] = {}; uint32_t csz = 0;
        if(fread(id, 1, 4, f) != 4) break;
        if(fread(&csz, 4, 1, f) != 1) break;
        long pos = ftell(f);
        if(strncmp(id, "fmt ", 4) == 0 && csz >= 16){
            uint16_t fmt = 0;
            fread(&fmt, 2, 1, f); fread(&ch, 2, 1, f); fread(&sr, 4, 1, f);
            fseek(f, 6, SEEK_CUR); fread(&bits, 2, 1, f);
        } else if(strncmp(id, "bewe", 4) == 0 && csz >= 20){
            uint32_t bsr = 0;
            fread(&cf, 8, 1, f); fread(&tm, 8, 1, f); fread(&bsr, 4, 1, f);
            if(sr == 0) sr = bsr;
        } else if(strncmp(id, "data", 4) == 0){
            doff = pos; dsz = (long)csz;
            char peek[4] = {};
            if(fread(peek, 1, 4, f) == 4 && strncmp(peek, "bewe", 4) == 0){
                uint32_t bsz = 0, bsr = 0;
                fread(&bsz, 4, 1, f);
                fread(&cf, 8, 1, f); fread(&tm, 8, 1, f); fread(&bsr, 4, 1, f);
                if(sr == 0) sr = bsr;
                long newoff = pos + 4 + 4 + (long)bsz + ((long)bsz & 1);
                dsz  -= (newoff - doff);
                doff  = newoff;
            }
        }
        long next = pos + (long)csz + ((long)csz & 1);
        if(fseek(f, next, SEEK_SET) != 0) break;
    }
    if(dsz <= 0){                                  // no data chunk → estimate
        fseek(f, 0, SEEK_END); long fsz = ftell(f);
        doff = 44; dsz = fsz - 44;
    }
    if(ch == 0) ch = 2;
    if(dsz <= 0){ fclose(f); return false; }
    fseek(f, doff, SEEK_SET);
    s.f = f; s.sample_rate = sr; s.center_freq_hz = cf; s.start_unix = tm;
    s.nch = (ch >= 2) ? 2 : 1; s.data_offset = doff; s.data_size = dsz;
    return true;
}

} // namespace SigMF
