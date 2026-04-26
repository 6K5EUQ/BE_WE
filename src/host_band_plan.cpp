#include "host_band_plan.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <sys/stat.h>
#include <errno.h>

namespace HostBandPlan {

std::mutex                g_mtx;
std::vector<PktBandEntry> g_segments;
std::vector<uint8_t>      g_cached_pkt;

std::string file_path(){
    return BEWEPaths::data_dir() + "/band_plan.json";
}

static void json_escape(std::string& out, const char* s){
    for(; *s; s++){
        char c = *s;
        if(c == '"' || c == '\\'){ out += '\\'; out += c; }
        else if(c == '\n') out += "\\n";
        else if(c == '\r') out += "\\r";
        else if((unsigned char)c < 0x20) {}
        else out += c;
    }
}

namespace {
struct JScan {
    const char* p;
    const char* end;
    void skip_ws(){ while(p<end && (*p==' '||*p=='\n'||*p=='\r'||*p=='\t'||*p==',')) p++; }
    bool consume(char c){ skip_ws(); if(p<end && *p==c){ p++; return true; } return false; }
    bool peek(char c){ skip_ws(); return p<end && *p==c; }
    bool read_string(std::string& out){
        skip_ws();
        if(p>=end || *p != '"') return false;
        p++; out.clear();
        while(p<end && *p != '"'){
            if(*p == '\\' && p+1<end){ p++;
                if(*p=='n') out+='\n';
                else if(*p=='r') out+='\r';
                else out += *p;
                p++;
            } else { out += *p++; }
        }
        if(p<end && *p=='"') p++;
        return true;
    }
    bool read_number(double& v){
        skip_ws();
        const char* s = p;
        while(p<end && (*p=='-'||*p=='+'||*p=='.'||(*p>='0'&&*p<='9')||*p=='e'||*p=='E')) p++;
        if(s==p) return false;
        std::string tmp(s, p-s);
        v = atof(tmp.c_str());
        return true;
    }
    bool read_key(std::string& k){
        if(!read_string(k)) return false;
        skip_ws();
        if(p<end && *p==':'){ p++; return true; }
        return false;
    }
};
} // anon

void save_to_file(){
    // Caller must NOT hold g_mtx (we take it).
    std::lock_guard<std::mutex> lk(g_mtx);
    std::string out = "{\n  \"bands\": [\n";
    bool first = true;
    for(auto& e : g_segments){
        if(!e.valid) continue;
        if(!first) out += ",\n";
        first = false;
        char buf[512];
        char lbl[33]={}; memcpy(lbl, e.label, 24);
        char dsc[200]={}; memcpy(dsc, e.description, 128);
        snprintf(buf, sizeof(buf),
            "    {\"freq_lo\":%.4f,\"freq_hi\":%.4f,\"category\":%u,\"label\":\"",
            e.freq_lo_mhz, e.freq_hi_mhz, (unsigned)e.category);
        out += buf;
        json_escape(out, lbl);
        out += "\",\"description\":\"";
        json_escape(out, dsc);
        out += "\"}";
    }
    out += "\n  ]\n}\n";

    // ensure ~/BE_WE exists
    std::string dir = BEWEPaths::data_dir();
    mkdir(dir.c_str(), 0755);

    std::string path = file_path();
    FILE* fp = fopen(path.c_str(), "w");
    if(fp){ fwrite(out.data(), 1, out.size(), fp); fclose(fp); }
    else fprintf(stderr, "[HostBandPlan] save: cannot open %s errno=%d\n",
                 path.c_str(), errno);
}

void load_from_file(){
    std::lock_guard<std::mutex> lk(g_mtx);
    g_segments.clear();
    std::string path = file_path();
    FILE* fp = fopen(path.c_str(), "r");
    if(!fp){ printf("[HostBandPlan] no file at %s — starting empty\n", path.c_str()); return; }
    fseek(fp,0,SEEK_END); long sz = ftell(fp); fseek(fp,0,SEEK_SET);
    if(sz <= 0){ fclose(fp); printf("[HostBandPlan] %s empty — starting empty\n", path.c_str()); return; }
    std::string body(sz, 0);
    if(fread(&body[0], 1, sz, fp) != (size_t)sz){ fclose(fp); return; }
    fclose(fp);

    JScan js{body.data(), body.data()+body.size()};
    if(!js.consume('{')) return;
    std::string key;
    int loaded = 0;
    while(js.read_key(key)){
        if(key != "bands"){
            if(js.peek('"')){ std::string tmp; js.read_string(tmp); }
            else if(js.consume('[') || js.consume('{')){
                int depth = 1;
                while(js.p<js.end && depth>0){
                    if(*js.p=='['||*js.p=='{') depth++;
                    else if(*js.p==']'||*js.p=='}') depth--;
                    js.p++;
                }
            }
            continue;
        }
        if(!js.consume('[')) break;
        while(!js.peek(']')){
            if(!js.consume('{')) break;
            PktBandEntry e{};
            e.valid = 1;
            std::string k;
            while(js.read_key(k)){
                if(k=="freq_lo"){ double v=0; js.read_number(v); e.freq_lo_mhz=(float)v; }
                else if(k=="freq_hi"){ double v=0; js.read_number(v); e.freq_hi_mhz=(float)v; }
                else if(k=="category"){ double v=0; js.read_number(v); e.category=(uint8_t)v; }
                else if(k=="label"){ std::string s; js.read_string(s);
                    strncpy(e.label, s.c_str(), sizeof(e.label)-1); }
                else if(k=="description"){ std::string s; js.read_string(s);
                    strncpy(e.description, s.c_str(), sizeof(e.description)-1); }
                else { if(js.peek('"')){ std::string t; js.read_string(t); } else { double v; js.read_number(v); } }
                if(js.peek('}')) break;
            }
            js.consume('}');
            if((int)g_segments.size() < MAX_BAND_SEGMENTS && e.freq_hi_mhz > e.freq_lo_mhz){
                g_segments.push_back(e);
                loaded++;
            }
        }
        js.consume(']');
        break;
    }
    printf("[HostBandPlan] loaded %d segments from %s\n", loaded, path.c_str());
}

void rebuild_cache(){
    // Caller must NOT hold g_mtx.
    std::lock_guard<std::mutex> lk(g_mtx);
    int n = (int)std::min<size_t>(g_segments.size(), (size_t)MAX_BAND_SEGMENTS);
    uint32_t plen = 4 + (uint32_t)n * sizeof(PktBandEntry);  // count(2)+pad(2)+entries
    std::vector<uint8_t> body(plen, 0);
    uint16_t count16 = (uint16_t)n;
    memcpy(body.data(), &count16, 2);
    if(n > 0){
        memcpy(body.data() + 4, g_segments.data(), (size_t)n * sizeof(PktBandEntry));
    }
    g_cached_pkt = make_packet(PacketType::BAND_PLAN_SYNC, body.data(), plen);
}

static bool entries_equal(const PktBandEntry& a, const PktBandEntry& b){
    if(std::abs(a.freq_lo_mhz - b.freq_lo_mhz) > 1e-4f) return false;
    if(std::abs(a.freq_hi_mhz - b.freq_hi_mhz) > 1e-4f) return false;
    return true;
}

bool apply_add(const PktBandEntry& in){
    std::lock_guard<std::mutex> lk(g_mtx);
    if(in.freq_hi_mhz <= in.freq_lo_mhz) return false;
    for(auto& e : g_segments){
        if(entries_equal(e, in)){
            e = in; e.valid = 1;
            return true;
        }
    }
    if((int)g_segments.size() >= MAX_BAND_SEGMENTS) return false;
    PktBandEntry e = in; e.valid = 1;
    g_segments.push_back(e);
    return true;
}

bool apply_update(const PktBandEntry& in){
    std::lock_guard<std::mutex> lk(g_mtx);
    if(in.freq_hi_mhz <= in.freq_lo_mhz) return false;
    for(auto& e : g_segments){
        if(entries_equal(e, in)){
            e = in; e.valid = 1;
            return true;
        }
    }
    // Not found: treat as add
    if((int)g_segments.size() >= MAX_BAND_SEGMENTS) return false;
    PktBandEntry e = in; e.valid = 1;
    g_segments.push_back(e);
    return true;
}

bool apply_remove(const PktBandRemove& rm){
    std::lock_guard<std::mutex> lk(g_mtx);
    auto it = std::remove_if(g_segments.begin(), g_segments.end(),
        [&](const PktBandEntry& e){
            return std::abs(e.freq_lo_mhz - rm.freq_lo_mhz) < 1e-4f
                && std::abs(e.freq_hi_mhz - rm.freq_hi_mhz) < 1e-4f;
        });
    if(it == g_segments.end()) return false;
    g_segments.erase(it, g_segments.end());
    return true;
}

void snapshot_pkt(PktBandPlan& out){
    std::lock_guard<std::mutex> lk(g_mtx);
    memset(&out, 0, sizeof(out));
    int n = (int)std::min<size_t>(g_segments.size(), (size_t)MAX_BAND_SEGMENTS);
    out.count = (uint16_t)n;
    for(int i=0; i<n; i++) out.entries[i] = g_segments[i];
}

bool host_local_add(const PktBandEntry& in){
    if(!apply_add(in)) return false;
    save_to_file(); rebuild_cache(); return true;
}
bool host_local_update(const PktBandEntry& in){
    if(!apply_update(in)) return false;
    save_to_file(); rebuild_cache(); return true;
}
bool host_local_remove(const PktBandRemove& rm){
    if(!apply_remove(rm)) return false;
    save_to_file(); rebuild_cache(); return true;
}

} // namespace HostBandPlan
