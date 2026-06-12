#include "module_api.hpp"
#include "net_protocol.hpp"
#include <cstring>
#include <vector>

// ── 모듈 레지스트리 + 공용 파이프 (GUI/CLI 공통, 모듈 무관) ─────────────────

static std::vector<BeweModule>& reg(){
    static std::vector<BeweModule> v;   // static-init 순서 안전 (Meyers singleton)
    return v;
}

void bewe_register_module(const BeweModule& m){ reg().push_back(m); }
const std::vector<BeweModule>& bewe_modules(){ return reg(); }

// ── 송신 백엔드 ──
static std::function<bool(const void*, uint32_t)> g_send_host;
static std::function<bool(const void*, uint32_t)> g_broadcast;
void bewe_mod_set_send_to_host(std::function<bool(const void*, uint32_t)> f){ g_send_host = std::move(f); }
void bewe_mod_set_broadcast(std::function<bool(const void*, uint32_t)> f){ g_broadcast = std::move(f); }

static std::vector<uint8_t> build_pipe(const char* mod_id, uint8_t kind, const void* d, size_t n){
    std::vector<uint8_t> buf(sizeof(PktModulePipe) + n);
    auto* h = reinterpret_cast<PktModulePipe*>(buf.data());
    memset(h, 0, sizeof(*h));
    strncpy(h->mod_id, mod_id, sizeof(h->mod_id)-1);
    h->kind = kind;
    h->data_len = (uint32_t)n;
    if(n) memcpy(buf.data()+sizeof(PktModulePipe), d, n);
    return buf;
}

bool bewe_mod_send_to_host(const char* mod_id, uint8_t kind, const void* d, size_t n){
    if(!g_send_host) return false;
    auto buf = build_pipe(mod_id, kind, d, n);
    return g_send_host(buf.data(), (uint32_t)buf.size());
}

bool bewe_mod_broadcast(const char* mod_id, uint8_t kind, const void* d, size_t n){
    if(!g_broadcast) return false;
    auto buf = build_pipe(mod_id, kind, d, n);
    return g_broadcast(buf.data(), (uint32_t)buf.size());
}

void bewe_mod_route(FFTViewer& v, bool host_side, const uint8_t* payload, size_t len){
    if(len < sizeof(PktModulePipe)) return;
    auto* h = reinterpret_cast<const PktModulePipe*>(payload);
    if(sizeof(PktModulePipe) + h->data_len > len) return;
    char id[9]={}; memcpy(id, h->mod_id, 8);
    const uint8_t* d = payload + sizeof(PktModulePipe);
    for(auto& m : reg()){
        if(strcmp(m.id, id)!=0) continue;
        if(host_side){ if(m.on_pipe_host) m.on_pipe_host(v, h->kind, d, h->data_len); }
        else         { if(m.on_pipe_join) m.on_pipe_join(v, h->kind, d, h->data_len); }
        return;
    }
    // 미설치 모듈 데이터 → 조용히 무시 (구매 안 한 사용자)
}
