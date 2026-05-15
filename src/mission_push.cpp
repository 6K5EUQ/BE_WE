// Mission File Push worker (Phase 2, v3.8.0)
// HOST → Central 파일 업로드 + ACK 기반 로컬 unlink.
#include "mission_push.hpp"
#include "fft_viewer.hpp"
#include "central_client.hpp"
#include "net_protocol.hpp"
#include "bewe_paths.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>

namespace MissionPush {

namespace {

struct QueueItem {
    std::string path;       // 절대 경로
    uint8_t     subdir;     // MFS_IQ/MFS_AUDIO/MFS_HIST
    int         year;
    std::string code;
    std::string filename;
};

struct AckEntry {
    bool                 received = false;
    PktMissionFilePushAck ack{};
};

FFTViewer*     g_v   = nullptr;
CentralClient* g_cli = nullptr;

std::mutex              q_mtx;
std::condition_variable q_cv;
std::deque<QueueItem>   q;

// transfer_id → ack state (worker가 wait, mux_loop가 setter)
std::mutex                              ack_mtx;
std::condition_variable                 ack_cv;
std::unordered_map<uint8_t, AckEntry>   ack_map;

std::atomic<bool> running{false};
std::thread       worker_thr;
std::atomic<uint8_t> next_transfer_id{1};

const char* subdir_name(uint8_t s){
    switch(s){
        case MFS_IQ:    return "iq";
        case MFS_AUDIO: return "audio";
        case MFS_HIST:  return "hist";
        default:        return "";
    }
}

// path = ~/BE_WE/recordings/missions/<YYYY>/<code>/<sub>/<filename>
// 위 형식이면 year/code/filename 파싱 후 true. 아니면 false.
bool parse_mission_path(const std::string& full, uint8_t subdir,
                        int& year_out, std::string& code_out, std::string& fname_out){
    std::string root = BEWEPaths::missions_root() + "/";
    if(full.compare(0, root.size(), root) != 0) return false;
    std::string rest = full.substr(root.size());  // YYYY/code/<sub>/filename
    size_t s1 = rest.find('/'); if(s1 == std::string::npos) return false;
    size_t s2 = rest.find('/', s1 + 1); if(s2 == std::string::npos) return false;
    size_t s3 = rest.find('/', s2 + 1); if(s3 == std::string::npos) return false;
    std::string ys = rest.substr(0, s1);
    std::string code = rest.substr(s1 + 1, s2 - s1 - 1);
    std::string subs = rest.substr(s2 + 1, s3 - s2 - 1);
    std::string fn   = rest.substr(s3 + 1);
    if(ys.empty() || code.empty() || fn.empty()) return false;
    if(strcmp(subs.c_str(), subdir_name(subdir)) != 0) return false;
    year_out = atoi(ys.c_str());
    if(year_out < 1970 || year_out > 3000) return false;
    code_out = code;
    fname_out = fn;
    return true;
}

uint8_t alloc_transfer_id(){
    // 1..255 순환 (0은 예약)
    for(int i = 0; i < 256; i++){
        uint8_t id = next_transfer_id.fetch_add(1, std::memory_order_relaxed);
        if(id == 0) id = next_transfer_id.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lk(ack_mtx);
        if(ack_map.find(id) == ack_map.end()) return id;
    }
    return 1;  // overflow fallback (collision 무시)
}

// Build BEWE packet (magic + type + len + payload) for MUX
std::vector<uint8_t> make_bewe(uint8_t bewe_type, const void* payload, uint32_t plen){
    std::vector<uint8_t> out(9 + plen);
    out[0]='B'; out[1]='E'; out[2]='W'; out[3]='E';
    out[4] = bewe_type;
    memcpy(out.data() + 5, &plen, 4);
    if(plen && payload) memcpy(out.data() + 9, payload, plen);
    return out;
}

// 한 파일 push 시도. 성공 시 true(+ unlink), 실패 시 false(+ 파일 유지).
bool push_one(const QueueItem& it){
    if(!g_cli) return false;
    FILE* fp = fopen(it.path.c_str(), "rb");
    if(!fp){
        fprintf(stderr, "[MissionPush] fopen FAIL %s errno=%d\n",
                it.path.c_str(), errno);
        return false;
    }
    fseeko(fp, 0, SEEK_END);
    uint64_t total = (uint64_t)ftello(fp);
    fseeko(fp, 0, SEEK_SET);

    // station_name 결정
    std::string station;
    if(g_v){
        // FFTViewer의 mission_station_name (string-like getter 없음 → 직접 접근 필요)
        // 대안: 활성 미션 entry에서 가져오기
        // 안전책: 그냥 host_band station_name 캐시 (FFTViewer.station_name).
        // 일단 viewer에서 추출하는 helper가 필요. 임시로 hostname or empty.
        // 정확한 station: mission이 시작될 때 set된 mission_station_name.
        // 여기선 viewer가 active mission entry에서 station_name을 알려주는 가정.
        // Phase 1 메모리: station[64]는 mission_station_name과 동일.
        station = g_v->mission_active_station_name();
    }
    if(station.empty()){
        fprintf(stderr, "[MissionPush] no station_name (mission inactive?) — skip %s\n",
                it.path.c_str());
        fclose(fp);
        return false;
    }

    uint8_t tid = alloc_transfer_id();

    // .info sidecar 읽기
    std::string info_text;
    {
        std::string info_path = it.path + ".info";
        FILE* fi = fopen(info_path.c_str(), "r");
        if(fi){
            char buf[512];
            size_t r = fread(buf, 1, sizeof(buf)-1, fi);
            buf[r] = 0;
            info_text = buf;
            fclose(fi);
        }
    }

    // META
    PktMissionFilePushMeta meta{};
    strncpy(meta.key.station, station.c_str(), sizeof(meta.key.station)-1);
    meta.key.year = (uint16_t)it.year;
    meta.key.subdir = it.subdir;
    strncpy(meta.key.code, it.code.c_str(), sizeof(meta.key.code)-1);
    strncpy(meta.key.filename, it.filename.c_str(), sizeof(meta.key.filename)-1);
    meta.total_bytes = total;
    meta.transfer_id = tid;
    meta.mode = 0;  // replace
    if(!info_text.empty())
        strncpy(meta.info_data, info_text.c_str(), sizeof(meta.info_data)-1);

    {
        std::lock_guard<std::mutex> lk(ack_mtx);
        ack_map[tid] = AckEntry{};
    }

    auto meta_pkt = make_bewe(0x4E /*MISSION_FILE_PUSH_META*/, &meta, sizeof(meta));
    g_cli->enqueue_relay_broadcast(meta_pkt.data(), meta_pkt.size(), /*no_drop=*/true);

    fprintf(stderr, "[MissionPush] PUSH start tid=%u %s (%lu bytes) -> %s/%04d/%s/%s\n",
            tid, it.path.c_str(), (unsigned long)total,
            station.c_str(), it.year, it.code.c_str(), it.filename.c_str());

    // DATA chunks
    constexpr uint32_t CHUNK = 256 * 1024;
    std::vector<uint8_t> chunk(CHUNK);
    uint64_t off = 0;
    bool ok = true;
    bool sent_any = false;
    while(off < total || !sent_any){
        if(!running.load()){ ok = false; break; }
        uint32_t want = (uint32_t)std::min((uint64_t)CHUNK, total - off);
        if(want > 0){
            size_t got = fread(chunk.data(), 1, want, fp);
            if(got != want){
                fprintf(stderr, "[MissionPush] read short tid=%u want=%u got=%zu\n",
                        tid, want, got);
                ok = false; break;
            }
        }
        // PUSH_DATA + raw chunk (BEWE payload = struct + raw)
        PktMissionFilePushData hd{};
        hd.transfer_id = tid;
        hd.is_last = (off + want >= total) ? 1 : 0;
        hd.offset = off;
        hd.chunk_bytes = want;
        std::vector<uint8_t> bewe_payload(sizeof(hd) + want);
        memcpy(bewe_payload.data(), &hd, sizeof(hd));
        if(want > 0) memcpy(bewe_payload.data() + sizeof(hd), chunk.data(), want);
        auto pkt = make_bewe(0x4F /*MISSION_FILE_PUSH_DATA*/,
                              bewe_payload.data(), (uint32_t)bewe_payload.size());
        g_cli->enqueue_relay_broadcast(pkt.data(), pkt.size(), /*no_drop=*/true);
        off += want;
        sent_any = true;
        if(want == 0) break;  // 빈 파일 1 chunk만
    }
    fclose(fp);
    if(!ok){
        std::lock_guard<std::mutex> lk(ack_mtx);
        ack_map.erase(tid);
        return false;
    }

    // ACK 대기 (최대 60s)
    bool ack_ok = false;
    PktMissionFilePushAck ack{};
    {
        std::unique_lock<std::mutex> lk(ack_mtx);
        ack_cv.wait_for(lk, std::chrono::seconds(60), [&]{
            auto it2 = ack_map.find(tid);
            return it2 == ack_map.end() || it2->second.received || !running.load();
        });
        auto it2 = ack_map.find(tid);
        if(it2 != ack_map.end() && it2->second.received){
            ack = it2->second.ack;
            ack_ok = (ack.status == 0);
        }
        ack_map.erase(tid);
    }
    if(!running.load()){
        fprintf(stderr, "[MissionPush] aborted tid=%u (worker stopped)\n", tid);
        return false;
    }
    if(!ack_ok){
        fprintf(stderr, "[MissionPush] tid=%u FAIL (ack received=%d status=%u msg='%s')\n",
                tid, ack.transfer_id != 0, ack.status, ack.error_msg);
        return false;
    }
    fprintf(stderr, "[MissionPush] tid=%u DONE (%lu bytes on Central) — unlink %s\n",
            tid, (unsigned long)ack.total_bytes, it.path.c_str());
    unlink(it.path.c_str());
    unlink((it.path + ".info").c_str());
    return true;
}

void worker_loop(){
    while(running.load()){
        QueueItem it;
        {
            std::unique_lock<std::mutex> lk(q_mtx);
            q_cv.wait_for(lk, std::chrono::seconds(2), [&]{
                return !running.load() || !q.empty();
            });
            if(!running.load()) break;
            if(q.empty()) continue;
            it = std::move(q.front());
            q.pop_front();
        }
        bool ok = push_one(it);
        if(!ok && running.load()){
            // 실패 시 후미에 재추가 + 짧은 백오프
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::lock_guard<std::mutex> lk(q_mtx);
            q.push_back(std::move(it));
        }
    }
    fprintf(stderr, "[MissionPush] worker exit\n");
}

} // anonymous

void start(FFTViewer* v, CentralClient* cli){
    if(running.load()) return;
    g_v = v;
    g_cli = cli;
    if(!cli) return;
    // ACK callback: mux_loop에서 호출
    cli->set_on_central_mf_push_ack(
        [](const uint8_t* bewe, size_t len){
            if(len < 9 + sizeof(PktMissionFilePushAck)) return;
            const auto* a = reinterpret_cast<const PktMissionFilePushAck*>(bewe + 9);
            std::lock_guard<std::mutex> lk(ack_mtx);
            auto it = ack_map.find(a->transfer_id);
            if(it == ack_map.end()){
                fprintf(stderr, "[MissionPush] stray ACK tid=%u (no waiting xfer)\n",
                        a->transfer_id);
                return;
            }
            it->second.received = true;
            it->second.ack = *a;
            ack_cv.notify_all();
        });
    running.store(true);
    worker_thr = std::thread(worker_loop);
    fprintf(stderr, "[MissionPush] started\n");
}

void stop(){
    if(!running.load()) return;
    running.store(false);
    q_cv.notify_all();
    ack_cv.notify_all();
    if(worker_thr.joinable()) worker_thr.join();
    g_v = nullptr;
    g_cli = nullptr;
}

void enqueue(const std::string& path, uint8_t subdir){
    if(!running.load()) return;
    QueueItem it;
    it.path = path;
    it.subdir = subdir;
    if(!parse_mission_path(path, subdir, it.year, it.code, it.filename)){
        fprintf(stderr, "[MissionPush] enqueue invalid path '%s' subdir=%u (skip)\n",
                path.c_str(), subdir);
        return;
    }
    // 파일 존재 확인
    struct stat st{};
    if(stat(path.c_str(), &st) != 0){
        // 이미 삭제됨 (성공한 push 이후 호출되면 정상)
        return;
    }
    if(!S_ISREG(st.st_mode)) return;
    std::lock_guard<std::mutex> lk(q_mtx);
    // 중복 enqueue 방지
    for(auto& e : q) if(e.path == path) return;
    q.push_back(std::move(it));
    q_cv.notify_one();
}

void scan_mission_dir_enqueue(int year, const char* code){
    if(!running.load() || !code || !code[0]) return;
    auto scan = [&](uint8_t sub){
        std::string dir;
        switch(sub){
            case MFS_IQ:    dir = BEWEPaths::mission_iq_dir(year, code); break;
            case MFS_AUDIO: dir = BEWEPaths::mission_audio_dir(year, code); break;
            case MFS_HIST:  dir = BEWEPaths::mission_hist_dir(year, code); break;
            default: return;
        }
        DIR* d = opendir(dir.c_str());
        if(!d) return;
        struct dirent* ent;
        while((ent = readdir(d)) != nullptr){
            const char* n = ent->d_name;
            if(!n || n[0] == '.') continue;
            size_t nlen = strlen(n);
            if(nlen >= 5 && strcmp(n + nlen - 5, ".info") == 0) continue;
            // -LIVE.bewehist 는 아직 stream 진행 중일 수 있어 제외
            if(sub == MFS_HIST && strstr(n, "-LIVE.")) continue;
            std::string full = dir + "/" + n;
            enqueue(full, sub);
        }
        closedir(d);
    };
    scan(MFS_IQ);
    scan(MFS_AUDIO);
    scan(MFS_HIST);
}

int pending_count(){
    std::lock_guard<std::mutex> lk(q_mtx);
    return (int)q.size();
}

} // namespace MissionPush
