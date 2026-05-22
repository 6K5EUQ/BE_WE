#include "host_band_plan.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"
#include "json_scan.hpp"
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

// 첫 부팅 시 band_plan.json 이 없으면 기본 band plan 을 자동 생성.
// 각 HOST 가 독립적으로 누적 편집 (broadcast 로 JOIN 에 sync) — git 추적 X.
// ITU/ARRL/KMA 일반 주파수 할당 기준.
static const char* DEFAULT_BAND_PLAN_JSON = R"json({
  "bands": [
    {"freq_lo":0.526,"freq_hi":1.606,"label":"AM","category":0,"description":"AM Broadcast (MW)"},
    {"freq_lo":1.800,"freq_hi":2.000,"label":"160m","category":3,"description":"Amateur 160m (HF)"},
    {"freq_lo":3.500,"freq_hi":3.900,"label":"80m","category":3,"description":"Amateur 80m (HF)"},
    {"freq_lo":7.000,"freq_hi":7.300,"label":"40m","category":3,"description":"Amateur 40m (HF)"},
    {"freq_lo":10.100,"freq_hi":10.150,"label":"30m","category":3,"description":"Amateur 30m (HF, CW/Digi only)"},
    {"freq_lo":13.553,"freq_hi":13.567,"label":"ISM","category":5,"description":"ISM 13.56 MHz (RFID/NFC)"},
    {"freq_lo":14.000,"freq_hi":14.350,"label":"20m","category":3,"description":"Amateur 20m (HF)"},
    {"freq_lo":18.068,"freq_hi":18.168,"label":"17m","category":3,"description":"Amateur 17m (HF)"},
    {"freq_lo":21.000,"freq_hi":21.450,"label":"15m","category":3,"description":"Amateur 15m (HF)"},
    {"freq_lo":24.890,"freq_hi":24.990,"label":"12m","category":3,"description":"Amateur 12m (HF)"},
    {"freq_lo":26.957,"freq_hi":27.283,"label":"ISM/CB","category":5,"description":"ISM 27.12 / CB Radio"},
    {"freq_lo":28.000,"freq_hi":29.700,"label":"10m","category":3,"description":"Amateur 10m (HF)"},
    {"freq_lo":40.660,"freq_hi":40.700,"label":"ISM","category":5,"description":"ISM 40.68 MHz"},
    {"freq_lo":50.000,"freq_hi":54.000,"label":"6m","category":3,"description":"Amateur 6m (VHF Low)"},
    {"freq_lo":76.000,"freq_hi":88.000,"label":"TV","category":0,"description":"VHF TV Low (legacy)"},
    {"freq_lo":88.000,"freq_hi":108.000,"label":"FM","category":0,"description":"FM Broadcast"},
    {"freq_lo":108.000,"freq_hi":117.975,"label":"VOR/ILS","category":1,"description":"Aero VOR / ILS Localizer"},
    {"freq_lo":118.000,"freq_hi":137.000,"label":"Aero","category":1,"description":"Aero Voice (AM, ATC)"},
    {"freq_lo":137.000,"freq_hi":138.000,"label":"WX-Sat","category":1,"description":"NOAA APT / Weather satellite downlink"},
    {"freq_lo":144.000,"freq_hi":148.000,"label":"2m","category":3,"description":"Amateur 2m (VHF)"},
    {"freq_lo":156.000,"freq_hi":162.025,"label":"Marine","category":2,"description":"Marine VHF + AIS (161.975/162.025)"},
    {"freq_lo":162.400,"freq_hi":162.550,"label":"WX","category":8,"description":"NOAA Weather Radio"},
    {"freq_lo":174.000,"freq_hi":216.000,"label":"TV/DAB","category":0,"description":"VHF TV High / DAB"},
    {"freq_lo":225.000,"freq_hi":328.600,"label":"Mil Aero","category":7,"description":"Military Aero (UHF)"},
    {"freq_lo":380.000,"freq_hi":400.000,"label":"P-Safety","category":8,"description":"Public Safety / Trunked (TETRA)"},
    {"freq_lo":403.000,"freq_hi":406.000,"label":"WX-Sonde","category":8,"description":"Radiosonde / Weather telemetry"},
    {"freq_lo":420.000,"freq_hi":450.000,"label":"70cm","category":3,"description":"Amateur 70cm (UHF)"},
    {"freq_lo":462.000,"freq_hi":467.000,"label":"FRS/GMRS","category":8,"description":"Family/General Mobile Radio"},
    {"freq_lo":470.000,"freq_hi":698.000,"label":"DTV","category":0,"description":"UHF Digital TV"},
    {"freq_lo":824.000,"freq_hi":894.000,"label":"Cell800","category":4,"description":"Cellular 800/850 (LTE B5/B26)"},
    {"freq_lo":902.000,"freq_hi":928.000,"label":"ISM915","category":5,"description":"ISM 915 (LoRa US, Wi-SUN)"},
    {"freq_lo":925.000,"freq_hi":960.000,"label":"GSM900","category":4,"description":"GSM/LTE 900 DL"},
    {"freq_lo":1227.000,"freq_hi":1230.000,"label":"GPS L2","category":9,"description":"GPS L2 1227.6 MHz"},
    {"freq_lo":1240.000,"freq_hi":1300.000,"label":"23cm","category":3,"description":"Amateur 23cm"},
    {"freq_lo":1525.000,"freq_hi":1559.000,"label":"L-Sat DL","category":1,"description":"Inmarsat / Aero-Sat downlink"},
    {"freq_lo":1559.000,"freq_hi":1610.000,"label":"GNSS L1","category":9,"description":"GPS/Galileo/GLONASS L1"},
    {"freq_lo":1710.000,"freq_hi":1880.000,"label":"DCS/LTE","category":4,"description":"GSM1800 / LTE B3"},
    {"freq_lo":1880.000,"freq_hi":1920.000,"label":"DECT","category":6,"description":"DECT cordless phone"},
    {"freq_lo":1920.000,"freq_hi":2170.000,"label":"UMTS","category":4,"description":"UMTS / LTE B1"},
    {"freq_lo":2300.000,"freq_hi":2400.000,"label":"WiBro","category":4,"description":"LTE B40 / WiBro"},
    {"freq_lo":2400.000,"freq_hi":2483.500,"label":"WiFi/BT","category":6,"description":"Wi-Fi 2.4 GHz / Bluetooth / Zigbee / ISM"},
    {"freq_lo":2500.000,"freq_hi":2690.000,"label":"LTE B7","category":4,"description":"LTE B7 / B41 (TDD)"},
    {"freq_lo":3300.000,"freq_hi":3800.000,"label":"5G n78","category":4,"description":"5G NR n77/n78 (mid-band)"},
    {"freq_lo":5150.000,"freq_hi":5875.000,"label":"WiFi5","category":6,"description":"Wi-Fi 5/6 (5 GHz UNII)"}
  ]
}
)json";

static void seed_default_band_plan_locked(){
    std::string path = file_path();
    FILE* fw = fopen(path.c_str(), "w");
    if(fw){
        fwrite(DEFAULT_BAND_PLAN_JSON, 1, strlen(DEFAULT_BAND_PLAN_JSON), fw);
        fclose(fw);
        printf("[HostBandPlan] seeded default at %s\n", path.c_str());
    } else {
        fprintf(stderr, "[HostBandPlan] seed FAIL: cannot create %s errno=%d\n",
                path.c_str(), errno);
    }
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
    if(!fp){
        // 첫 부팅 — 기본 band plan 자동 생성 후 다시 read
        seed_default_band_plan_locked();
        fp = fopen(path.c_str(), "r");
        if(!fp){ printf("[HostBandPlan] seed/open failed — starting empty\n"); return; }
    }
    fseek(fp,0,SEEK_END); long sz = ftell(fp); fseek(fp,0,SEEK_SET);
    if(sz <= 0){
        fclose(fp);
        // 빈 파일도 seed 로 보충
        seed_default_band_plan_locked();
        fp = fopen(path.c_str(), "r");
        if(!fp) return;
        fseek(fp,0,SEEK_END); sz = ftell(fp); fseek(fp,0,SEEK_SET);
        if(sz <= 0){ fclose(fp); return; }
    }
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
