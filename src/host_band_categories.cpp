#include "host_band_categories.hpp"
#include "bewe_paths.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <sys/stat.h>
#include <errno.h>

namespace HostBandCategories {

std::mutex                    g_mtx;
std::vector<PktBandCategory>  g_cats;
std::vector<uint8_t>          g_cached_pkt;

std::string file_path(){
    return BEWEPaths::data_dir() + "/band_categories.json";
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
    const char* p; const char* end;
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

// Builtin defaults — match BAND_COLORS in ui.cpp (rendered with alpha 110).
struct BuiltinCat { uint8_t id; const char* name; uint8_t r, g, b; };
static const BuiltinCat BUILTIN_CATS[] = {
    {0,  "Broadcast",     255,180, 80},
    {1,  "Aero",           80,180,255},
    {2,  "Marine",         80,255,180},
    {3,  "Amateur",       255,255, 80},
    {4,  "Cell",          180, 80,255},
    {5,  "ISM",           255, 80,180},
    {6,  "WiFi-BT",       120,200,255},
    {7,  "Mil",           200,100,100},
    {8,  "Public-Safety", 150,180,200},
    {9,  "Government",    180,150,200},
    {10, "Other",         160,160,160},
};

static void seed_builtins_locked(){
    g_cats.clear();
    for(auto& b : BUILTIN_CATS){
        PktBandCategory c{};
        c.id = b.id; c.valid = 1;
        c.r = b.r; c.g = b.g; c.b = b.b;
        strncpy(c.name, b.name, sizeof(c.name)-1);
        g_cats.push_back(c);
    }
}

static void write_json_locked(){
    std::string out = "{\n  \"categories\": [\n";
    bool first = true;
    for(auto& c : g_cats){
        if(!c.valid) continue;
        if(!first) out += ",\n";
        first = false;
        char buf[256];
        char nm[33]={}; memcpy(nm, c.name, 24);
        snprintf(buf, sizeof(buf),
            "    {\"id\":%u,\"r\":%u,\"g\":%u,\"b\":%u,\"name\":\"",
            (unsigned)c.id, (unsigned)c.r, (unsigned)c.g, (unsigned)c.b);
        out += buf;
        json_escape(out, nm);
        out += "\"}";
    }
    out += "\n  ]\n}\n";
    std::string dir = BEWEPaths::data_dir();
    mkdir(dir.c_str(), 0755);
    std::string path = file_path();
    FILE* fp = fopen(path.c_str(), "w");
    if(fp){ fwrite(out.data(), 1, out.size(), fp); fclose(fp); }
    else fprintf(stderr, "[HostBandCategories] save failed: %s errno=%d\n",
                 path.c_str(), errno);
}

void save_to_file(){
    std::lock_guard<std::mutex> lk(g_mtx);
    write_json_locked();
}

void load_from_file(){
    std::lock_guard<std::mutex> lk(g_mtx);
    g_cats.clear();
    std::string path = file_path();
    FILE* fp = fopen(path.c_str(), "r");
    if(!fp){
        printf("[HostBandCategories] no file at %s — seeding %zu builtins\n",
               path.c_str(), sizeof(BUILTIN_CATS)/sizeof(BUILTIN_CATS[0]));
        seed_builtins_locked();
        write_json_locked();
        return;
    }
    fseek(fp,0,SEEK_END); long sz = ftell(fp); fseek(fp,0,SEEK_SET);
    if(sz <= 0){ fclose(fp); seed_builtins_locked(); write_json_locked(); return; }
    std::string body(sz, 0);
    if(fread(&body[0], 1, sz, fp) != (size_t)sz){ fclose(fp); seed_builtins_locked(); return; }
    fclose(fp);

    JScan js{body.data(), body.data()+body.size()};
    if(!js.consume('{')){ seed_builtins_locked(); return; }
    std::string key;
    int loaded = 0;
    while(js.read_key(key)){
        if(key != "categories"){
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
            PktBandCategory c{};
            c.valid = 1;
            std::string k;
            while(js.read_key(k)){
                if(k=="id"){ double v=0; js.read_number(v); c.id=(uint8_t)v; }
                else if(k=="r"){ double v=0; js.read_number(v); c.r=(uint8_t)v; }
                else if(k=="g"){ double v=0; js.read_number(v); c.g=(uint8_t)v; }
                else if(k=="b"){ double v=0; js.read_number(v); c.b=(uint8_t)v; }
                else if(k=="name"){ std::string s; js.read_string(s);
                    strncpy(c.name, s.c_str(), sizeof(c.name)-1); }
                else { if(js.peek('"')){ std::string t; js.read_string(t); } else { double v; js.read_number(v); } }
                if(js.peek('}')) break;
            }
            js.consume('}');
            if((int)g_cats.size() < MAX_BAND_CATEGORIES){
                g_cats.push_back(c);
                loaded++;
            }
        }
        js.consume(']');
        break;
    }
    if(loaded == 0){
        seed_builtins_locked();
        write_json_locked();
        printf("[HostBandCategories] empty file — re-seeded builtins\n");
    } else {
        printf("[HostBandCategories] loaded %d categories from %s\n", loaded, path.c_str());
    }
}

void rebuild_cache(){
    std::lock_guard<std::mutex> lk(g_mtx);
    PktBandCatSync pkt{};
    int n = (int)std::min<size_t>(g_cats.size(), (size_t)MAX_BAND_CATEGORIES);
    pkt.count = (uint16_t)n;
    for(int i=0;i<n;i++) pkt.entries[i] = g_cats[i];
    g_cached_pkt = make_packet(PacketType::BAND_CAT_SYNC, &pkt, sizeof(pkt));
}

bool apply_upsert(const PktBandCategory& in){
    std::lock_guard<std::mutex> lk(g_mtx);
    if(!in.name[0]) return false;  // empty name rejected
    for(auto& c : g_cats){
        if(c.id == in.id){
            c = in; c.valid = 1;
            return true;
        }
    }
    if((int)g_cats.size() >= MAX_BAND_CATEGORIES) return false;
    PktBandCategory c = in; c.valid = 1;
    g_cats.push_back(c);
    return true;
}

bool apply_delete(uint8_t id){
    std::lock_guard<std::mutex> lk(g_mtx);
    auto it = std::remove_if(g_cats.begin(), g_cats.end(),
        [&](const PktBandCategory& c){ return c.id == id; });
    if(it == g_cats.end()) return false;
    g_cats.erase(it, g_cats.end());
    return true;
}

void snapshot_pkt(PktBandCatSync& out){
    std::lock_guard<std::mutex> lk(g_mtx);
    memset(&out, 0, sizeof(out));
    int n = (int)std::min<size_t>(g_cats.size(), (size_t)MAX_BAND_CATEGORIES);
    out.count = (uint16_t)n;
    for(int i=0;i<n;i++) out.entries[i] = g_cats[i];
}

bool lookup(uint8_t id, PktBandCategory& out){
    std::lock_guard<std::mutex> lk(g_mtx);
    for(auto& c : g_cats) if(c.id == id && c.valid){ out = c; return true; }
    return false;
}

std::string name_of(uint8_t id){
    std::lock_guard<std::mutex> lk(g_mtx);
    for(auto& c : g_cats) if(c.id == id && c.valid) return std::string(c.name);
    return "Other";
}

bool host_local_upsert(const PktBandCategory& in){
    if(!apply_upsert(in)) return false;
    save_to_file(); rebuild_cache(); return true;
}
bool host_local_delete(uint8_t id){
    if(!apply_delete(id)) return false;
    save_to_file(); rebuild_cache(); return true;
}

} // namespace HostBandCategories
