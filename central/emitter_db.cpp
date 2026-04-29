#include "emitter_db.hpp"
#include "info_parse.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <random>
#include <sstream>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace BeweCentral {

// ── Local utils (anon namespace) ───────────────────────────────────────
namespace {

// FNV-1a 64-bit → 12 hex chars. Deterministic, no crypto deps.
std::string fnv1a_hex12(const std::string& s){
    uint64_t h = 0xcbf29ce484222325ULL;
    for(unsigned char c : s){
        h ^= c;
        h *= 0x100000001b3ULL;
    }
    char buf[20];
    snprintf(buf, sizeof(buf), "%012llx",
             (unsigned long long)(h & 0xffffffffffffULL));
    return std::string(buf);
}

std::string rand_hex8(){
    static thread_local std::mt19937_64 rng{std::random_device{}()};
    uint64_t v = rng() & 0xffffffffULL;
    char buf[12];
    snprintf(buf, sizeof(buf), "%08llx", (unsigned long long)v);
    return std::string(buf);
}

std::string js_escape(const std::string& s){
    std::string out;
    out.reserve(s.size() + 8);
    for(char c : s){
        switch(c){
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if((unsigned char)c < 0x20){
                    char hex[8]; snprintf(hex, sizeof(hex), "\\u%04x", (int)(unsigned char)c);
                    out += hex;
                } else out += c;
        }
    }
    return out;
}

struct JR {
    const char* p;
    const char* end;
    void skip_ws(){ while(p<end && (*p==' '||*p=='\n'||*p=='\r'||*p=='\t')) p++; }
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
                else if(*p=='t') out+='\t';
                else if(*p=='"') out+='"';
                else if(*p=='\\') out+='\\';
                else if(*p=='u' && p+4<end){
                    // skip \uXXXX (decode best-effort: only ASCII)
                    char hex[5] = {p[1],p[2],p[3],p[4],0};
                    int code = (int)strtol(hex, nullptr, 16);
                    if(code < 0x80) out += (char)code;
                    p += 4;
                }
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
    bool read_int64(int64_t& v){
        double d; if(!read_number(d)) return false;
        v = (int64_t)d; return true;
    }
    bool skip_value(){
        skip_ws();
        if(p>=end) return false;
        if(*p == '"'){ std::string s; return read_string(s); }
        if(*p == '['){
            p++; int depth = 1;
            while(p<end && depth>0){
                if(*p == '"'){ std::string s; read_string(s); continue; }
                if(*p == '[') depth++;
                else if(*p == ']') depth--;
                p++;
            }
            return true;
        }
        if(*p == '{'){
            p++; int depth = 1;
            while(p<end && depth>0){
                if(*p == '"'){ std::string s; read_string(s); continue; }
                if(*p == '{') depth++;
                else if(*p == '}') depth--;
                p++;
            }
            return true;
        }
        double d; return read_number(d);
    }
};

bool dir_exists(const std::string& p){
    struct stat st{};
    return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}
bool ensure_dir_path(const std::string& p){
    if(dir_exists(p)) return true;
    return mkdir(p.c_str(), 0755) == 0;
}

bool read_file_all(const std::string& path, std::string& out){
    FILE* f = fopen(path.c_str(), "rb");
    if(!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    out.resize(sz > 0 ? (size_t)sz : 0);
    if(sz > 0) fread(&out[0], 1, (size_t)sz, f);
    fclose(f);
    return true;
}

bool write_file_atomic(const std::string& path, const std::string& content){
    std::string tmp = path + ".tmp";
    FILE* f = fopen(tmp.c_str(), "wb");
    if(!f) return false;
    if(!content.empty()){
        if(fwrite(content.data(), 1, content.size(), f) != content.size()){
            fclose(f); unlink(tmp.c_str()); return false;
        }
    }
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    if(rename(tmp.c_str(), path.c_str()) != 0){
        unlink(tmp.c_str()); return false;
    }
    return true;
}

// ── Serialization ──────────────────────────────────────────────────────

std::string emitter_to_jsonl(const Emitter& e){
    std::string s = "{";
    bool first = true;
    auto add_str = [&](const char* k, const std::string& v){
        if(!first) s += ",";
        first = false;
        s += "\""; s += k; s += "\":\""; s += js_escape(v); s += "\"";
    };
    auto add_num = [&](const char* k, double v){
        if(!first) s += ",";
        first = false;
        s += "\""; s += k; s += "\":";
        char buf[32]; snprintf(buf, sizeof(buf), "%.6g", v);
        s += buf;
    };
    auto add_int = [&](const char* k, int64_t v){
        if(!first) s += ",";
        first = false;
        s += "\""; s += k; s += "\":";
        char buf[32]; snprintf(buf, sizeof(buf), "%lld", (long long)v);
        s += buf;
    };
    auto add_arr = [&](const char* k, const std::vector<std::string>& v){
        if(!first) s += ",";
        first = false;
        s += "\""; s += k; s += "\":[";
        bool fa = true;
        for(auto& x : v){
            if(!fa) s += ",";
            fa = false;
            s += "\""; s += js_escape(x); s += "\"";
        }
        s += "]";
    };
    add_str("uid",     e.emitter_uid);
    add_str("name",    e.display_name);
    add_num("f_mhz",   e.freq_center_mhz);
    add_num("tol_khz", e.freq_tolerance_khz);
    add_num("bw_khz",  e.bw_khz);
    add_str("mod",     e.modulation);
    add_str("proto",   e.protocol);
    add_str("tags_id", e.tags_id);
    add_int("first",   e.first_seen_utc);
    add_int("last",    e.last_seen_utc);
    add_int("count",   (int64_t)e.sighting_count);
    add_arr("stations",e.contributing_stations);
    add_str("notes",   e.operator_notes);
    add_arr("members", e.member_sighting_ids);
    add_str("creator", e.created_by);
    add_int("created", e.created_utc);
    s += "}";
    return s;
}

bool emitter_from_jsonl(const std::string& line, Emitter& e){
    JR jr{line.data(), line.data() + line.size()};
    if(!jr.consume('{')) return false;
    while(!jr.peek('}')){
        std::string k;
        if(!jr.read_string(k)) return false;
        if(!jr.consume(':')) return false;
        if(k == "uid"){ jr.read_string(e.emitter_uid); }
        else if(k == "name"){ jr.read_string(e.display_name); }
        else if(k == "f_mhz"){ double d; jr.read_number(d); e.freq_center_mhz=(float)d; }
        else if(k == "tol_khz"){ double d; jr.read_number(d); e.freq_tolerance_khz=(float)d; }
        else if(k == "bw_khz"){ double d; jr.read_number(d); e.bw_khz=(float)d; }
        else if(k == "mod"){ jr.read_string(e.modulation); }
        else if(k == "proto"){ jr.read_string(e.protocol); }
        else if(k == "tags_id"){ jr.read_string(e.tags_id); }
        else if(k == "first"){ jr.read_int64(e.first_seen_utc); }
        else if(k == "last"){ jr.read_int64(e.last_seen_utc); }
        else if(k == "count"){ int64_t v; jr.read_int64(v); e.sighting_count=(uint32_t)v; }
        else if(k == "stations"){
            jr.consume('[');
            e.contributing_stations.clear();
            while(!jr.peek(']') && jr.p < jr.end){
                std::string sx; if(!jr.read_string(sx)) break;
                e.contributing_stations.push_back(sx);
                if(jr.peek(',')) jr.consume(',');
            }
            jr.consume(']');
        }
        else if(k == "notes"){ jr.read_string(e.operator_notes); }
        else if(k == "members"){
            jr.consume('[');
            e.member_sighting_ids.clear();
            while(!jr.peek(']') && jr.p < jr.end){
                std::string sx; if(!jr.read_string(sx)) break;
                e.member_sighting_ids.push_back(sx);
                if(jr.peek(',')) jr.consume(',');
            }
            jr.consume(']');
        }
        else if(k == "creator"){ jr.read_string(e.created_by); }
        else if(k == "created"){ jr.read_int64(e.created_utc); }
        else { jr.skip_value(); }
        if(jr.peek(',')) jr.consume(',');
    }
    jr.consume('}');
    return !e.emitter_uid.empty();
}

std::string sighting_to_jsonl(const Sighting& s){
    std::string out = "{";
    bool first = true;
    auto add_str = [&](const char* k, const std::string& v){
        if(!first) out += ",";
        first = false;
        out += "\""; out += k; out += "\":\""; out += js_escape(v); out += "\"";
    };
    auto add_num = [&](const char* k, double v){
        if(!first) out += ",";
        first = false;
        out += "\""; out += k; out += "\":";
        char buf[32]; snprintf(buf, sizeof(buf), "%.6g", v);
        out += buf;
    };
    auto add_int = [&](const char* k, int64_t v){
        if(!first) out += ",";
        first = false;
        out += "\""; out += k; out += "\":";
        char buf[32]; snprintf(buf, sizeof(buf), "%lld", (long long)v);
        out += buf;
    };
    add_str("sid",      s.sighting_id);
    add_str("file",     s.filename);
    add_str("reporter", s.reporter);
    add_str("station",  s.station);
    add_num("f_mhz",    s.freq_mhz);
    add_num("bw_khz",   s.bw_khz);
    add_str("mod",      s.modulation);
    add_str("proto",    s.protocol);
    add_str("target",   s.target);
    add_str("op",       s.operator_name);
    add_str("tags",     s.tags);
    add_int("start",    s.start_utc);
    add_int("dur",      (int64_t)s.duration_s);
    add_str("euid",     s.emitter_uid);
    add_int("status",   (int64_t)s.match_status);
    out += "}";
    return out;
}

bool sighting_from_jsonl(const std::string& line, Sighting& s){
    JR jr{line.data(), line.data() + line.size()};
    if(!jr.consume('{')) return false;
    while(!jr.peek('}')){
        std::string k;
        if(!jr.read_string(k)) return false;
        if(!jr.consume(':')) return false;
        if(k == "sid"){ jr.read_string(s.sighting_id); }
        else if(k == "file"){ jr.read_string(s.filename); }
        else if(k == "reporter"){ jr.read_string(s.reporter); }
        else if(k == "station"){ jr.read_string(s.station); }
        else if(k == "f_mhz"){ double d; jr.read_number(d); s.freq_mhz=(float)d; }
        else if(k == "bw_khz"){ double d; jr.read_number(d); s.bw_khz=(float)d; }
        else if(k == "mod"){ jr.read_string(s.modulation); }
        else if(k == "proto"){ jr.read_string(s.protocol); }
        else if(k == "target"){ jr.read_string(s.target); }
        else if(k == "op"){ jr.read_string(s.operator_name); }
        else if(k == "tags"){ jr.read_string(s.tags); }
        else if(k == "start"){ jr.read_int64(s.start_utc); }
        else if(k == "dur"){ int64_t v; jr.read_int64(v); s.duration_s=(uint32_t)v; }
        else if(k == "euid"){ jr.read_string(s.emitter_uid); }
        else if(k == "status"){ int64_t v; jr.read_int64(v); s.match_status=(uint8_t)v; }
        else { jr.skip_value(); }
        if(jr.peek(',')) jr.consume(',');
    }
    jr.consume('}');
    return !s.sighting_id.empty();
}

} // anon

std::string sighting_id_from(const std::string& filename, const std::string& reporter){
    return fnv1a_hex12(filename + "|" + reporter);
}

std::string new_emitter_uid(){
    return std::string("e_") + rand_hex8();
}

// ── EmitterDb members ──────────────────────────────────────────────────

bool EmitterDb::ensure_dirs_(){
    if(!ensure_dir_path(base_dir_)) return false;
    if(!ensure_dir_path(base_dir_ + "/_emitters")) return false;
    if(!ensure_dir_path(base_dir_ + "/_sightings")) return false;
    return true;
}

bool EmitterDb::load(const std::string& base_dir){
    std::lock_guard<std::mutex> lk(mtx_);
    base_dir_ = base_dir;
    if(!ensure_dirs_()) return false;
    emitters_.clear();
    sightings_.clear();
    load_emitters_jsonl_();
    load_sightings_jsonl_();
    return true;
}

bool EmitterDb::load_emitters_jsonl_(){
    std::string path = base_dir_ + "/_emitters/emitters.jsonl";
    std::string text;
    if(!read_file_all(path, text)) return false;
    std::istringstream iss(text);
    std::string line;
    while(std::getline(iss, line)){
        if(line.empty()) continue;
        Emitter e;
        if(emitter_from_jsonl(line, e)) emitters_[e.emitter_uid] = e;
    }
    return true;
}

bool EmitterDb::load_sightings_jsonl_(){
    std::string path = base_dir_ + "/_sightings/index.jsonl";
    std::string text;
    if(!read_file_all(path, text)) return false;
    std::istringstream iss(text);
    std::string line;
    while(std::getline(iss, line)){
        if(line.empty()) continue;
        Sighting s;
        if(sighting_from_jsonl(line, s)) sightings_[s.sighting_id] = s;
    }
    return true;
}

bool EmitterDb::save_emitters_atomic_unlocked_(){
    std::string path = base_dir_ + "/_emitters/emitters.jsonl";
    std::string out;
    out.reserve(emitters_.size() * 256);
    for(auto& kv : emitters_){
        out += emitter_to_jsonl(kv.second);
        out += "\n";
    }
    return write_file_atomic(path, out);
}

bool EmitterDb::save_sightings_atomic_unlocked_(){
    std::string path = base_dir_ + "/_sightings/index.jsonl";
    std::string out;
    out.reserve(sightings_.size() * 256);
    for(auto& kv : sightings_){
        out += sighting_to_jsonl(kv.second);
        out += "\n";
    }
    return write_file_atomic(path, out);
}

float EmitterDb::score_match_(const Sighting& s, const Emitter& e) const {
    // Frequency must be within emitter's tolerance.
    float df_khz = std::fabs(s.freq_mhz - e.freq_center_mhz) * 1000.0f;
    if(df_khz > e.freq_tolerance_khz) return 0.f;

    constexpr float w_freq  = 0.6f;
    constexpr float w_mod   = 0.2f;
    constexpr float w_proto = 0.15f;
    constexpr float w_tags  = 0.05f;

    float total_w = w_freq;
    float score   = w_freq;

    bool s_mod = !s.modulation.empty();
    bool e_mod = !e.modulation.empty();
    if(s_mod && e_mod){
        total_w += w_mod;
        if(InfoParse::lower(s.modulation) == InfoParse::lower(e.modulation))
            score += w_mod;
    }
    bool s_pr = !s.protocol.empty();
    bool e_pr = !e.protocol.empty();
    if(s_pr && e_pr){
        total_w += w_proto;
        if(InfoParse::lower(s.protocol) == InfoParse::lower(e.protocol))
            score += w_proto;
    }
    bool s_tg = !s.tags.empty();
    bool e_tg = !e.tags_id.empty();
    if(s_tg && e_tg){
        total_w += w_tags;
        std::string lt = InfoParse::lower(s.tags);
        std::string et = InfoParse::lower(e.tags_id);
        if(lt.find(et) != std::string::npos) score += w_tags;
    }

    if(total_w <= 0.f) return 0.f;
    float normalized = score / total_w;

    // 운영자가 .info의 변조/protocol/tags를 다 비웠으면 freq만으로 매칭됨 → 정규화하면 1.0이지만
    // 자동합치기 막기 위해 0.6으로 캡. = pending 단계에서 사람이 결정.
    if(total_w == w_freq) return std::min(0.6f, normalized);
    return normalized;
}

Emitter EmitterDb::make_emitter_from_(const Sighting& s, const std::string& creator){
    Emitter e;
    e.emitter_uid = new_emitter_uid();
    char namebuf[128];
    if(!s.modulation.empty())
        snprintf(namebuf, sizeof(namebuf), "%.4f MHz %s", s.freq_mhz, s.modulation.c_str());
    else
        snprintf(namebuf, sizeof(namebuf), "%.4f MHz", s.freq_mhz);
    e.display_name = namebuf;
    e.freq_center_mhz = s.freq_mhz;
    e.freq_tolerance_khz = 1.0f;
    e.bw_khz = s.bw_khz;
    e.modulation = s.modulation;
    e.protocol = s.protocol;
    e.first_seen_utc = s.start_utc;
    e.last_seen_utc = s.start_utc;
    e.sighting_count = 0;
    e.created_by = creator;
    e.created_utc = (int64_t)time(nullptr);
    return e;
}

void EmitterDb::recompute_emitter_aggregate_(Emitter& e){
    e.sighting_count = (uint32_t)e.member_sighting_ids.size();
    int64_t first = INT64_MAX, last = 0;
    std::vector<std::string> stations;
    for(auto& sid : e.member_sighting_ids){
        auto it = sightings_.find(sid);
        if(it == sightings_.end()) continue;
        const Sighting& s = it->second;
        if(s.start_utc > 0 && s.start_utc < first) first = s.start_utc;
        if(s.start_utc > last) last = s.start_utc;
        if(!s.station.empty() &&
           std::find(stations.begin(), stations.end(), s.station) == stations.end()){
            stations.push_back(s.station);
        }
    }
    if(first == INT64_MAX) first = 0;
    e.first_seen_utc = first;
    e.last_seen_utc  = last;
    e.contributing_stations = stations;
}

EmitterDb::MatchResult EmitterDb::ingest_sighting(Sighting& s){
    std::lock_guard<std::mutex> lk(mtx_);
    MatchResult mr;

    bool freq_ok = (s.freq_mhz > 0.f);

    // Dedup: same sighting_id already known → preserve user's manual link, refresh fields only.
    auto exist = sightings_.find(s.sighting_id);
    if(exist != sightings_.end()){
        std::string saved_euid = exist->second.emitter_uid;
        uint8_t saved_status = exist->second.match_status;
        s.emitter_uid  = saved_euid;
        s.match_status = saved_status;
        sightings_[s.sighting_id] = s;
        save_sightings_atomic_unlocked_();
        mr.emitter_uid = saved_euid;
        mr.status = saved_status;
        return mr;
    }

    if(!freq_ok){
        s.match_status = MS_PENDING;
        s.emitter_uid.clear();
        sightings_[s.sighting_id] = s;
        save_sightings_atomic_unlocked_();
        mr.status = MS_PENDING;
        return mr;
    }

    // 가장 점수 높은 emitter 1개 찾기.
    float best_score = -1.f;
    std::string best_uid;
    for(auto& kv : emitters_){
        float sc = score_match_(s, kv.second);
        if(sc > best_score){ best_score = sc; best_uid = kv.first; }
    }

    if(best_score >= 0.85f){
        s.emitter_uid = best_uid;
        s.match_status = MS_AUTO_HIGH;
        sightings_[s.sighting_id] = s;
        Emitter& e = emitters_[best_uid];
        e.member_sighting_ids.push_back(s.sighting_id);
        recompute_emitter_aggregate_(e);
        mr.emitter_uid = best_uid; mr.score = best_score; mr.status = MS_AUTO_HIGH;
    } else if(best_score >= 0.55f){
        s.emitter_uid = best_uid;
        s.match_status = MS_PENDING;
        sightings_[s.sighting_id] = s;
        Emitter& e = emitters_[best_uid];
        e.member_sighting_ids.push_back(s.sighting_id);
        recompute_emitter_aggregate_(e);
        mr.emitter_uid = best_uid; mr.score = best_score; mr.status = MS_PENDING;
    } else {
        Emitter ne = make_emitter_from_(s, s.reporter);
        ne.member_sighting_ids.push_back(s.sighting_id);
        s.emitter_uid = ne.emitter_uid;
        s.match_status = MS_AUTO_LOW;
        sightings_[s.sighting_id] = s;
        recompute_emitter_aggregate_(ne);
        emitters_[ne.emitter_uid] = ne;
        mr.emitter_uid = ne.emitter_uid;
        mr.score = best_score < 0 ? 0.f : best_score;
        mr.status = MS_AUTO_LOW;
    }
    save_sightings_atomic_unlocked_();
    save_emitters_atomic_unlocked_();
    return mr;
}

bool EmitterDb::upsert_emitter(Emitter& e){
    std::lock_guard<std::mutex> lk(mtx_);
    if(e.emitter_uid.empty()){
        e.emitter_uid = new_emitter_uid();
        if(e.created_utc == 0) e.created_utc = (int64_t)time(nullptr);
    } else {
        auto it = emitters_.find(e.emitter_uid);
        if(it != emitters_.end()){
            // 자동 관리 필드는 보존 (사용자가 수정 못 함).
            e.first_seen_utc        = it->second.first_seen_utc;
            e.last_seen_utc         = it->second.last_seen_utc;
            e.sighting_count        = it->second.sighting_count;
            e.contributing_stations = it->second.contributing_stations;
            e.member_sighting_ids   = it->second.member_sighting_ids;
            e.created_by            = it->second.created_by;
            e.created_utc           = it->second.created_utc;
        }
    }
    if(e.display_name.empty()){
        char buf[64]; snprintf(buf, sizeof(buf), "%.4f MHz", e.freq_center_mhz);
        e.display_name = buf;
    }
    if(e.freq_tolerance_khz <= 0.f) e.freq_tolerance_khz = 1.0f;
    emitters_[e.emitter_uid] = e;
    return save_emitters_atomic_unlocked_();
}

bool EmitterDb::delete_emitter(const std::string& uid){
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = emitters_.find(uid);
    if(it == emitters_.end()) return false;
    for(auto& sid : it->second.member_sighting_ids){
        auto sit = sightings_.find(sid);
        if(sit != sightings_.end()){
            sit->second.emitter_uid.clear();
            sit->second.match_status = MS_PENDING;
        }
    }
    emitters_.erase(it);
    save_sightings_atomic_unlocked_();
    save_emitters_atomic_unlocked_();
    return true;
}

bool EmitterDb::link_sighting(const std::string& sid,
                              const std::string& target_uid,
                              uint8_t action){
    std::lock_guard<std::mutex> lk(mtx_);
    auto sit = sightings_.find(sid);
    if(sit == sightings_.end()) return false;
    Sighting& s = sit->second;

    auto remove_from_emitter = [&](const std::string& uid){
        if(uid.empty()) return;
        auto eit = emitters_.find(uid);
        if(eit == emitters_.end()) return;
        auto& v = eit->second.member_sighting_ids;
        v.erase(std::remove(v.begin(), v.end(), sid), v.end());
        recompute_emitter_aggregate_(eit->second);
    };
    auto add_to_emitter = [&](const std::string& uid){
        auto eit = emitters_.find(uid);
        if(eit == emitters_.end()) return;
        auto& v = eit->second.member_sighting_ids;
        if(std::find(v.begin(), v.end(), sid) == v.end()) v.push_back(sid);
        recompute_emitter_aggregate_(eit->second);
    };

    switch(action){
    case 0: // confirm
        if(!s.emitter_uid.empty()) s.match_status = MS_CONFIRMED;
        break;
    case 1: { // reject — 분리해서 단독 emitter로
        std::string old_uid = s.emitter_uid;
        remove_from_emitter(old_uid);
        Emitter ne = make_emitter_from_(s, s.reporter);
        ne.member_sighting_ids.push_back(sid);
        s.emitter_uid = ne.emitter_uid;
        s.match_status = MS_MANUAL;
        recompute_emitter_aggregate_(ne);
        emitters_[ne.emitter_uid] = ne;
        break;
    }
    case 2: // move
        remove_from_emitter(s.emitter_uid);
        s.emitter_uid = target_uid;
        s.match_status = MS_MANUAL;
        if(!target_uid.empty()) add_to_emitter(target_uid);
        break;
    case 3: { // split_to_new
        remove_from_emitter(s.emitter_uid);
        Emitter ne = make_emitter_from_(s, s.reporter);
        ne.member_sighting_ids.push_back(sid);
        s.emitter_uid = ne.emitter_uid;
        s.match_status = MS_MANUAL;
        recompute_emitter_aggregate_(ne);
        emitters_[ne.emitter_uid] = ne;
        break;
    }
    default:
        return false;
    }
    save_sightings_atomic_unlocked_();
    save_emitters_atomic_unlocked_();
    return true;
}

void EmitterDb::list_emitters(uint16_t off, uint16_t lim,
                              std::vector<Emitter>& out, uint16_t& total){
    std::lock_guard<std::mutex> lk(mtx_);
    size_t sz = emitters_.size();
    total = (uint16_t)std::min<size_t>(UINT16_MAX, sz);
    out.clear();
    if(off >= total) return;
    uint16_t end = (uint16_t)std::min<int>((int)off + (int)lim, (int)total);
    out.reserve(end - off);
    uint16_t i = 0;
    for(auto& kv : emitters_){
        if(i >= off && i < end) out.push_back(kv.second);
        i++;
        if(i >= end) break;
    }
}

void EmitterDb::list_sightings(const std::string& filter,
                               uint16_t off, uint16_t lim,
                               std::vector<Sighting>& out, uint16_t& total){
    std::lock_guard<std::mutex> lk(mtx_);
    out.clear();
    std::vector<const Sighting*> cands;
    cands.reserve(sightings_.size());
    for(auto& kv : sightings_){
        if(filter.empty() || kv.second.emitter_uid == filter)
            cands.push_back(&kv.second);
    }
    total = (uint16_t)std::min<size_t>(UINT16_MAX, cands.size());
    if(off >= total) return;
    uint16_t end = (uint16_t)std::min<int>((int)off + (int)lim, (int)total);
    out.reserve(end - off);
    for(uint16_t i = off; i < end; i++) out.push_back(*cands[i]);
}

bool EmitterDb::find_emitter(const std::string& uid, Emitter& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = emitters_.find(uid);
    if(it == emitters_.end()) return false;
    out = it->second;
    return true;
}
bool EmitterDb::find_sighting(const std::string& sid, Sighting& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = sightings_.find(sid);
    if(it == sightings_.end()) return false;
    out = it->second;
    return true;
}

size_t EmitterDb::emitter_count() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return emitters_.size();
}
size_t EmitterDb::sighting_count() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return sightings_.size();
}

bool EmitterDb::migrate_from_reports(const std::string& reports_dir){
    DIR* dir = opendir(reports_dir.c_str());
    if(!dir) return false;
    struct dirent* de;
    int n = 0;
    while((de = readdir(dir)) != nullptr){
        std::string name = de->d_name;
        if(name == "." || name == "..") continue;
        if(name.size() < 5 || name.substr(name.size() - 5) != ".info") continue;
        std::string path = reports_dir + "/" + name;
        std::string text;
        if(!read_file_all(path, text)) continue;
        auto kv = InfoParse::parse(text);
        Sighting s;
        s.filename = kv.count("File Name") ? kv["File Name"] : name.substr(0, name.size()-5);
        s.reporter = kv.count("Operator") ? kv["Operator"] : "";
        s.station  = kv.count("Location") ? kv["Location"] : "";
        if(kv.count("Frequency")) InfoParse::extract_freq_mhz(kv["Frequency"], s.freq_mhz);
        if(kv.count("Bandwidth")) InfoParse::extract_bw_khz(kv["Bandwidth"], s.bw_khz);
        s.modulation    = kv.count("Modulation") ? kv["Modulation"] : "";
        s.protocol      = kv.count("Protocol") ? kv["Protocol"] : "";
        s.target        = kv.count("Target") ? kv["Target"] : "";
        s.operator_name = s.reporter;
        s.tags          = kv.count("Tags") ? kv["Tags"] : "";
        if(kv.count("Day") && kv.count("Up Time"))
            InfoParse::extract_start_utc(kv["Day"], kv["Up Time"], s.start_utc);
        if(kv.count("Duration")) InfoParse::extract_duration_s(kv["Duration"], s.duration_s);
        s.sighting_id = sighting_id_from(s.filename, s.reporter);
        ingest_sighting(s);
        n++;
    }
    closedir(dir);
    (void)n;
    return true;
}

} // namespace BeweCentral
