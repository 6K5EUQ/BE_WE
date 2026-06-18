// Mission File Archive (Phase 1, v3.8.0)
// Layout: ~/BE_WE/DataBase/missions/<station>/<year>/<code>/{iq,audio,hist}/
// HOST PUSH (IQ/audio):
//   PUSH_META(transfer_id, mode, total, info)  ->  archive_dir/filename + .info
//   PUSH_DATA(offset, chunk) xN
//   PUSH_DATA(is_last=1)     -> Central close + PUSH_ACK(status=0)
// HIST: PUSH 없이 LWF_LIVE_START/ROW/STOP 스트림 tap.
#include "central_server.hpp"
#include "../src/net_protocol.hpp"
#include "../src/sigmf.hpp"
#include "../src/long_waterfall.hpp"   // build_hist_filename_finalize
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <vector>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>

namespace {
// 파일명/경로 컴포넌트 sanitization.
// 슬래시 '/' 와 ".." 컴포넌트 차단 (디렉토리 escape 방지).
// 빈 문자열·null 입력 → false.
bool path_component_safe(const char* s, size_t maxlen){
    if(!s || !s[0]) return false;
    size_t n = strnlen(s, maxlen);
    if(n == 0 || n >= maxlen) return false;
    if(strcmp(s, ".") == 0 || strcmp(s, "..") == 0) return false;
    for(size_t i=0;i<n;i++){
        unsigned char c = (unsigned char)s[i];
        if(c == '/' || c == '\\' || c == 0) return false;
        if(c < 0x20) return false;     // 제어문자 금지
    }
    return true;
}

void mkdir_p(const std::string& p){
    if(p.empty()) return;
    std::string cur;
    cur.reserve(p.size());
    for(size_t i=0;i<p.size();i++){
        cur.push_back(p[i]);
        if(p[i] == '/' && cur.size() > 1){
            mkdir(cur.c_str(), 0755);
        }
    }
    mkdir(p.c_str(), 0755);
}

void rm_rf(const std::string& path){
    DIR* d = opendir(path.c_str());
    if(!d){ unlink(path.c_str()); return; }
    struct dirent* ent;
    while((ent = readdir(d)) != nullptr){
        const char* n = ent->d_name;
        if(!n || (n[0]=='.' && (n[1]==0 || (n[1]=='.' && n[2]==0)))) continue;
        std::string full = path + "/" + n;
        struct stat st;
        if(stat(full.c_str(), &st) != 0) continue;
        if(S_ISDIR(st.st_mode)) rm_rf(full);
        else                    unlink(full.c_str());
    }
    closedir(d);
    rmdir(path.c_str());
}

const char* subdir_name(uint8_t s){
    switch(s){
        case MFS_IQ:    return "iq";
        case MFS_AUDIO: return "audio";
        case MFS_HIST:  return "hist";
        default:        return nullptr;
    }
}
} // anonymous namespace

std::string CentralServer::archive_root() const {
    const char* home = getenv("HOME");
    std::string base = home ? std::string(home) + "/BE_WE/DataBase"
                            : std::string("/tmp/BE_WE/DataBase");
    return base + "/missions";
}

std::string CentralServer::archive_dir(const char* station, uint16_t year,
                                       const char* code, uint8_t subdir) const {
    const char* sub = subdir_name(subdir);
    if(!sub) return std::string();
    if(!path_component_safe(station, 64)) return std::string();
    if(!path_component_safe(code, 8))     return std::string();
    if(year < 1970 || year > 3000)        return std::string();
    char y[8]; snprintf(y, sizeof(y), "%04u", (unsigned)year);
    return archive_root() + "/" + station + "/" + y + "/" + code + "/" + sub;
}

void CentralServer::archive_wipe_mission(const char* station, uint16_t year,
                                         const char* code){
    if(!path_component_safe(station, 64)) return;
    if(!path_component_safe(code, 8))     return;
    if(year < 1970 || year > 3000)        return;
    char y[8]; snprintf(y, sizeof(y), "%04u", (unsigned)year);
    std::string dir = archive_root() + "/" + station + "/" + y + "/" + code;
    rm_rf(dir);
    printf("[Central][Archive] WIPE %s\n", dir.c_str());
}

// ── HOST → Central: PUSH_META ────────────────────────────────────────────
void CentralServer::handle_mission_file_push_meta(std::shared_ptr<HostRoom> room,
                                                   const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFilePushMeta)){
        printf("[Central][Archive] PUSH_META short payload=%zu < %zu\n",
               plen, sizeof(PktMissionFilePushMeta));
        return;
    }
    const auto* m = reinterpret_cast<const PktMissionFilePushMeta*>(payload);

    // key 유효성 검증
    char station[65]={}; memcpy(station, m->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    m->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   m->key.filename,128); fname[128]=0;

    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(m->key.subdir) == nullptr){
        printf("[Central][Archive] PUSH_META invalid key station='%s' year=%u code='%s' subdir=%u file='%s'\n",
               station, m->key.year, code, m->key.subdir, fname);
        MissionFileKey k = m->key;
        send_push_ack(room, k, m->transfer_id, 3, 0, "invalid key");
        return;
    }

    std::string dir = archive_dir(station, m->key.year, code, m->key.subdir);
    if(dir.empty()){
        send_push_ack(room, m->key, m->transfer_id, 3, 0, "bad path");
        return;
    }
    mkdir_p(dir);

    std::string fullpath = dir + "/" + fname;

    // 기존 transfer 있으면 닫음 (id 충돌 → 이전 transfer 폐기)
    auto it = room->mission_xfers.find(m->transfer_id);
    if(it != room->mission_xfers.end() && it->second.fp){
        fclose(it->second.fp);
        room->mission_xfers.erase(it);
    }

    // mode==1(append) 일 때 + 기존 파일 있으면 append, 없으면 new
    // mode==0(replace) 일 때 truncate
    const char* fmode = (m->mode == 1) ? "ab" : "wb";
    FILE* fp = fopen(fullpath.c_str(), fmode);
    if(!fp){
        printf("[Central][Archive] PUSH_META fopen FAIL '%s' errno=%d (%s)\n",
               fullpath.c_str(), errno, strerror(errno));
        send_push_ack(room, m->key, m->transfer_id, 1, 0, strerror(errno));
        return;
    }

    MissionFileTransfer xf;
    xf.key = m->key;
    xf.fp = fp;
    xf.expected_bytes = m->total_bytes;
    xf.written_bytes = 0;
    xf.high_water = 0;
    xf.archive_path = fullpath;
    xf.info_data.assign(m->info_data, strnlen(m->info_data, sizeof(m->info_data)));
    auto ins = room->mission_xfers.emplace(m->transfer_id, std::move(xf));

    // info sidecar 즉시 작성 (있으면)
    if(!ins.first->second.info_data.empty()){
        FILE* fi = fopen(SigMF::sidecar_path(fullpath).c_str(), "w");
        if(fi){
            const auto& s = ins.first->second.info_data;
            fwrite(s.data(), 1, s.size(), fi);
            fclose(fi);
        }
    }

    printf("[Central][Archive] PUSH_META xfer=%u mode=%u total=%lu → %s\n",
           m->transfer_id, m->mode, (unsigned long)m->total_bytes, fullpath.c_str());
}

// ── HOST → Central: PUSH_DATA ────────────────────────────────────────────
void CentralServer::handle_mission_file_push_data(std::shared_ptr<HostRoom> room,
                                                   const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFilePushData)) return;
    const auto* d = reinterpret_cast<const PktMissionFilePushData*>(payload);
    const uint8_t* raw = payload + sizeof(PktMissionFilePushData);
    size_t raw_avail = plen - sizeof(PktMissionFilePushData);
    if(d->chunk_bytes > raw_avail){
        printf("[Central][Archive] PUSH_DATA truncated xfer=%u chunk=%u avail=%zu\n",
               d->transfer_id, d->chunk_bytes, raw_avail);
        return;
    }

    auto it = room->mission_xfers.find(d->transfer_id);
    if(it == room->mission_xfers.end() || !it->second.fp){
        printf("[Central][Archive] PUSH_DATA unknown xfer=%u (no META)\n", d->transfer_id);
        return;
    }
    auto& xf = it->second;

    // offset 위치로 seek 후 write. append 모드라도 fseek 가능.
    if(fseeko(xf.fp, (off_t)d->offset, SEEK_SET) != 0){
        printf("[Central][Archive] PUSH_DATA seek FAIL xfer=%u offset=%lu errno=%d\n",
               d->transfer_id, (unsigned long)d->offset, errno);
        send_push_ack(room, xf.key, d->transfer_id, 1, xf.written_bytes, "seek failed");
        fclose(xf.fp); xf.fp = nullptr;
        room->mission_xfers.erase(it);
        return;
    }
    if(d->chunk_bytes > 0){
        size_t w = fwrite(raw, 1, d->chunk_bytes, xf.fp);
        if(w != d->chunk_bytes){
            printf("[Central][Archive] PUSH_DATA write short xfer=%u want=%u got=%zu\n",
                   d->transfer_id, d->chunk_bytes, w);
            send_push_ack(room, xf.key, d->transfer_id, 1, xf.written_bytes, "short write");
            fclose(xf.fp); xf.fp = nullptr;
            room->mission_xfers.erase(it);
            return;
        }
        xf.written_bytes += w;
        uint64_t hw = d->offset + d->chunk_bytes;
        if(hw > xf.high_water) xf.high_water = hw;
    }

    if(d->is_last){
        fflush(xf.fp);
        fclose(xf.fp); xf.fp = nullptr;
        // 디스크 commit 후 ack
        struct stat st{}; uint64_t disk_bytes = 0;
        if(stat(xf.archive_path.c_str(), &st) == 0) disk_bytes = (uint64_t)st.st_size;
        printf("[Central][Archive] PUSH_DATA last xfer=%u written=%lu disk=%lu → ACK\n",
               d->transfer_id, (unsigned long)xf.written_bytes, (unsigned long)disk_bytes);
        send_push_ack(room, xf.key, d->transfer_id, 0, disk_bytes, "");
        room->mission_xfers.erase(it);
    }
}

void CentralServer::send_push_ack(std::shared_ptr<HostRoom> room,
                                   const MissionFileKey& key, uint8_t transfer_id,
                                   uint8_t status, uint64_t total_bytes,
                                   const char* err){
    PktMissionFilePushAck ack{};
    ack.key = key;
    ack.transfer_id = transfer_id;
    ack.status = status;
    ack.total_bytes = total_bytes;
    if(err) strncpy(ack.error_msg, err, sizeof(ack.error_msg)-1);
    auto bewe = CentralServer::make_bewe_packet(BEWE_TYPE_MISSION_FILE_PUSH_ACK,
                                                  &ack, sizeof(ack));
    enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                      bewe.data(), (uint32_t)bewe.size());
}

// ── LIST_REQ ─────────────────────────────────────────────────────────────
// 스캔 후 PktMissionFileList 페이지(들) 전송. count > MAX 면 다중 page.
// requester != nullptr → JOIN, == nullptr → HOST.
void CentralServer::handle_mission_file_list_req(std::shared_ptr<HostRoom> room,
                                                  std::shared_ptr<JoinEntry> requester,
                                                  const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFileListReq)) return;
    const auto* r = reinterpret_cast<const PktMissionFileListReq*>(payload);
    char fstation[65]={}; memcpy(fstation, r->station, 64); fstation[64]=0;
    char fcode[9]={};     memcpy(fcode,    r->code,    8);  fcode[8]=0;
    uint16_t fyear = r->year;
    uint8_t  fsub  = r->subdir;

    std::string root = archive_root();
    std::vector<MissionFileEntry> rows;

    auto add_if = [&](const std::string& station_s, uint16_t y, const std::string& code_s,
                      uint8_t sub){
        std::string dir = archive_dir(station_s.c_str(), y, code_s.c_str(), sub);
        if(dir.empty()) return;
        DIR* d = opendir(dir.c_str());
        if(!d) return;
        struct dirent* ent;
        while((ent = readdir(d)) != nullptr){
            const char* n = ent->d_name;
            if(!n || n[0] == '.') continue;
            // sidecar(.info / .sigmf-meta) 는 list에서 제외
            size_t nlen = strlen(n);
            if(nlen >= 5  && strcmp(n + nlen - 5,  ".info")       == 0) continue;
            if(nlen >= 11 && strcmp(n + nlen - 11, ".sigmf-meta") == 0) continue;
            std::string full = dir + "/" + n;
            struct stat st;
            if(stat(full.c_str(), &st) != 0) continue;
            if(!S_ISREG(st.st_mode)) continue;
            // Stale -LIVE.bewehist: mtime 이 30초 이상 안 변경됐으면 host stream 끊긴 것으로 판단.
            // 자동 unlink + LIST 에서 제외 — 다른 사용자에게 보이지 않게 유지.
            if(strstr(n, "-LIVE.bewehist")){
                time_t now = time(nullptr);
                if(st.st_mtime + 30 < now){
                    printf("[Central][Archive] purge stale LIVE %s (mtime=%ld now=%ld)\n",
                           full.c_str(), (long)st.st_mtime, (long)now);
                    unlink(full.c_str());
                    unlink(SigMF::sidecar_path(full).c_str());
                    continue;
                }
            }
            MissionFileEntry e{};
            strncpy(e.station, station_s.c_str(), sizeof(e.station)-1);
            e.year = y;
            e.subdir = sub;
            strncpy(e.code, code_s.c_str(), sizeof(e.code)-1);
            strncpy(e.filename, n, sizeof(e.filename)-1);
            e.size_bytes = (uint64_t)st.st_size;
            e.mtime_unix = (int64_t)st.st_mtime;
            // .info sidecar 의 "Operator:" 추출 — DB 탭과 동일 표시용.
            FILE* fi = fopen(SigMF::sidecar_path(full).c_str(), "r");
            if(fi){
                char buf[1024]; size_t br = fread(buf, 1, sizeof(buf)-1, fi); buf[br]=0;
                fclose(fi);
                const char* p = buf;
                while(p && *p){
                    char k[64]={}, val[128]={};
                    if(sscanf(p, "%63[^:]: %127[^\n]", k, val) >= 2){
                        if(strcmp(k, "Operator") == 0){
                            strncpy(e.operator_name, val, sizeof(e.operator_name)-1);
                            break;
                        }
                    }
                    const char* nl = strchr(p, '\n');
                    if(!nl) break;
                    p = nl + 1;
                }
                // 파일별 note (IQ=bewe:notes / 그 외=Note: 라인) — 호버 툴팁용
                std::string nt = SigMF::note_from_text(std::string(buf, br));
                strncpy(e.note, nt.c_str(), sizeof(e.note)-1);
            }
            rows.push_back(e);
        }
        closedir(d);
    };

    auto enum_dir = [](const std::string& path, std::vector<std::string>& out){
        DIR* d = opendir(path.c_str());
        if(!d) return;
        struct dirent* ent;
        while((ent = readdir(d)) != nullptr){
            const char* n = ent->d_name;
            if(!n || n[0] == '.') continue;
            std::string full = path + "/" + n;
            struct stat st;
            if(stat(full.c_str(), &st) != 0) continue;
            if(S_ISDIR(st.st_mode)) out.push_back(n);
        }
        closedir(d);
    };

    // station 트리 walk: fstation 빈 → 모든 station
    std::vector<std::string> stations;
    if(fstation[0] && path_component_safe(fstation, 64)){
        stations.push_back(fstation);
    } else {
        enum_dir(root, stations);
    }
    for(auto& st_name : stations){
        std::vector<std::string> years;
        if(fyear){
            char yb[8]; snprintf(yb, sizeof(yb), "%04u", (unsigned)fyear);
            years.push_back(yb);
        } else {
            enum_dir(root + "/" + st_name, years);
        }
        for(auto& ys : years){
            uint16_t y = (uint16_t)atoi(ys.c_str());
            if(y < 1970 || y > 3000) continue;
            std::vector<std::string> codes;
            if(fcode[0] && path_component_safe(fcode, 8)){
                codes.push_back(fcode);
            } else {
                enum_dir(root + "/" + st_name + "/" + ys, codes);
            }
            for(auto& cs : codes){
                if(fsub){
                    add_if(st_name, y, cs, fsub);
                } else {
                    add_if(st_name, y, cs, MFS_IQ);
                    add_if(st_name, y, cs, MFS_AUDIO);
                    add_if(st_name, y, cs, MFS_HIST);
                }
            }
        }
    }

    printf("[Central][Archive] LIST_REQ station='%s' year=%u code='%s' sub=%u → %zu rows\n",
           fstation, fyear, fcode, fsub, rows.size());

    // 페이지 분할 전송
    size_t per_pkt = (size_t)MAX_MISSION_FILES_PER_PKT;
    size_t total = rows.size();
    size_t pages = total == 0 ? 1 : (total + per_pkt - 1) / per_pkt;
    for(size_t pg = 0; pg < pages; pg++){
        PktMissionFileList list{};
        size_t off = pg * per_pkt;
        size_t n = std::min(per_pkt, total - off);
        list.count = (uint16_t)n;
        list.is_last_page = (pg + 1 == pages) ? 1 : 0;
        for(size_t i = 0; i < n; i++) list.entries[i] = rows[off + i];
        auto bewe = CentralServer::make_bewe_packet(
            BEWE_TYPE_MISSION_FILE_LIST, &list, sizeof(list));
        if(requester){
            requester->enqueue_ctrl(bewe.data(), bewe.size());
        } else {
            enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                              bewe.data(), (uint32_t)bewe.size());
        }
    }
}

// ── DL_REQ ───────────────────────────────────────────────────────────────
void CentralServer::handle_mission_file_dl_req(std::shared_ptr<HostRoom> room,
                                                std::shared_ptr<JoinEntry> requester,
                                                const uint8_t* payload, size_t plen){
    if(plen < sizeof(PktMissionFileDlReq)) return;
    const auto* r = reinterpret_cast<const PktMissionFileDlReq*>(payload);

    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;

    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] DL_REQ invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string full = dir + "/" + fname;
    FILE* fp = fopen(full.c_str(), "rb");
    if(!fp){
        printf("[Central][Archive] DL_REQ open FAIL %s errno=%d\n", full.c_str(), errno);
        // 빈 응답 (chunk_bytes=0, is_first=1, is_last=1) → caller가 실패 인지
        PktMissionFileDlData head{};
        head.key = r->key;
        head.is_first = 1; head.is_last = 1;
        head.total_bytes = 0; head.offset = 0; head.chunk_bytes = 0;
        auto bewe = CentralServer::make_bewe_packet(
            BEWE_TYPE_MISSION_FILE_DL_DATA, &head, sizeof(head));
        if(requester) requester->enqueue_ctrl(bewe.data(), bewe.size());
        else          enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                                        bewe.data(), (uint32_t)bewe.size());
        return;
    }
    fseeko(fp, 0, SEEK_END); uint64_t total = (uint64_t)ftello(fp);
    uint64_t start = r->start_offset;
    if(start > total) start = total;
    fseeko(fp, (off_t)start, SEEK_SET);

    // sidecar(.sigmf-meta/.info) — start==0(처음부터 받기) 일 때만 동봉. resume 시엔 이미 받았다고 간주.
    char info[1024] = {};
    if(start == 0){
        FILE* fi = fopen(SigMF::sidecar_path(full).c_str(), "r");
        if(fi){ fread(info, 1, sizeof(info)-1, fi); fclose(fi); }
    }

    // 이미 받은 양이 현재 size 와 같거나 더 큰 경우: 빈 응답 (chunk_bytes=0, is_first/last=1)
    if(start >= total){
        PktMissionFileDlData head{};
        head.key = r->key;
        head.total_bytes = total;
        head.offset = start;
        head.chunk_bytes = 0;
        head.is_first = 1; head.is_last = 1;
        auto bewe = CentralServer::make_bewe_packet(
            BEWE_TYPE_MISSION_FILE_DL_DATA, &head, sizeof(head));
        if(requester) requester->enqueue_ctrl(bewe.data(), bewe.size());
        else          enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                                        bewe.data(), (uint32_t)bewe.size());
        fclose(fp);
        printf("[Central][Archive] DL_REQ %s up-to-date (start=%lu total=%lu)\n",
               full.c_str(), (unsigned long)start, (unsigned long)total);
        return;
    }

    constexpr uint32_t CHUNK = 256 * 1024;  // 256 KB
    uint64_t off = start;
    bool first = true;
    while(off < total){
        uint32_t want = (uint32_t)std::min((uint64_t)CHUNK, total - off);
        PktMissionFileDlData head{};
        head.key = r->key;
        head.total_bytes = total;
        head.offset = off;
        head.chunk_bytes = want;
        head.is_first = first ? 1 : 0;
        head.is_last  = (off + want >= total) ? 1 : 0;
        if(first && start == 0) memcpy(head.info_data, info, sizeof(head.info_data));
        // BEWE 패킷에 직접 빌드 — 중간 chunk/payload 버퍼 복사 제거 (fread → 패킷 직행)
        uint32_t pkt_plen = (uint32_t)(sizeof(head) + want);
        std::vector<uint8_t> bewe(BEWE_HDR_SIZE + pkt_plen);
        memcpy(bewe.data(), "BEWE", 4);
        bewe[4] = BEWE_TYPE_MISSION_FILE_DL_DATA;
        memcpy(bewe.data()+5, &pkt_plen, 4);
        memcpy(bewe.data()+BEWE_HDR_SIZE, &head, sizeof(head));
        size_t got = fread(bewe.data()+BEWE_HDR_SIZE+sizeof(head), 1, want, fp);
        if(got != want){
            printf("[Central][Archive] DL_REQ read short off=%lu want=%u got=%zu\n",
                   (unsigned long)off, want, got);
            break;
        }
        if(requester){
            requester->enqueue_file(bewe.data(), bewe.size());
        } else {
            enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA,
                              bewe.data(), (uint32_t)bewe.size());
        }
        off += want;
        first = false;
    }
    fclose(fp);
    printf("[Central][Archive] DL_REQ done %s start=%lu sent=%lu total=%lu\n",
           full.c_str(), (unsigned long)start, (unsigned long)(off - start),
           (unsigned long)total);
}

// ── DELETE ───────────────────────────────────────────────────────────────
void CentralServer::handle_mission_file_delete(std::shared_ptr<HostRoom> room,
                                                std::shared_ptr<JoinEntry> requester,
                                                const uint8_t* payload, size_t plen){
    (void)room; (void)requester;
    if(plen < sizeof(PktMissionFileDelete)) return;
    const auto* r = reinterpret_cast<const PktMissionFileDelete*>(payload);
    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;
    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] DELETE invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string full = dir + "/" + fname;
    int rc1 = unlink(full.c_str());
    int rc2 = unlink(SigMF::sidecar_path(full).c_str());
    printf("[Central][Archive] DELETE %s (file=%d info=%d)\n", full.c_str(), rc1, rc2);
}

// ── RENAME ───────────────────────────────────────────────────────────────
void CentralServer::handle_mission_file_rename(std::shared_ptr<HostRoom> room,
                                                std::shared_ptr<JoinEntry> requester,
                                                const uint8_t* payload, size_t plen){
    (void)room; (void)requester;
    if(plen < sizeof(PktMissionFileRename)) return;
    const auto* r = reinterpret_cast<const PktMissionFileRename*>(payload);
    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;
    char newfn[129]={}; memcpy(newfn,   r->new_filename,128); newfn[128]=0;
    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128)  || !path_component_safe(newfn, 128) ||
       subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] RENAME invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string src = dir + "/" + fname;
    std::string dst = dir + "/" + newfn;
    int rc1 = rename(src.c_str(), dst.c_str());
    int rc2 = rename(SigMF::sidecar_path(src).c_str(), SigMF::sidecar_path(dst).c_str());
    printf("[Central][Archive] RENAME %s → %s (file=%d info=%d)\n",
           src.c_str(), newfn, rc1, rc2);
}

// ── SET_NOTE: 파일별 메모를 사이드카에 기입 (IQ=bewe:notes / 그 외=Note: 라인) ──
// 클라가 다음 LIST_REQ(2초 폴링)에서 갱신된 note 를 받아 호버 툴팁 반영.
void CentralServer::handle_mission_file_set_note(std::shared_ptr<HostRoom> room,
                                                 std::shared_ptr<JoinEntry> requester,
                                                 const uint8_t* payload, size_t plen){
    (void)room; (void)requester;
    if(plen < sizeof(PktMissionFileSetNote)) return;
    const auto* r = reinterpret_cast<const PktMissionFileSetNote*>(payload);
    char station[65]={}; memcpy(station, r->key.station, 64); station[64]=0;
    char code[9]={};    memcpy(code,    r->key.code,    8);  code[8]=0;
    char fname[129]={}; memcpy(fname,   r->key.filename,128); fname[128]=0;
    char note[257]={};  memcpy(note,    r->note,        256); note[256]=0;
    if(!path_component_safe(station, 64) || !path_component_safe(code, 8) ||
       !path_component_safe(fname, 128) || subdir_name(r->key.subdir) == nullptr){
        printf("[Central][Archive] SET_NOTE invalid key\n");
        return;
    }
    std::string dir = archive_dir(station, r->key.year, code, r->key.subdir);
    if(dir.empty()) return;
    std::string full = dir + "/" + fname;
    bool ok = SigMF::update_note(full, note);
    printf("[Central][Archive] SET_NOTE %s (ok=%d, %zu chars)\n", full.c_str(), (int)ok, strlen(note));
}

// ── LWF live stream → HIST archive tap ───────────────────────────────────
void CentralServer::archive_hist_on_live_start(std::shared_ptr<HostRoom> room,
                                                const PktLwfLiveStart& ls){
    if(!room->active_mission_valid) return;  // 미션 활성 아니면 archive 안 함
    char fname[65]={}; memcpy(fname, ls.filename, 64); fname[64]=0;
    if(!path_component_safe(fname, 128)){
        printf("[Central][Archive] HIST live_start invalid filename\n");
        return;
    }
    std::string dir = archive_dir(room->active_mission_station,
                                  room->active_mission_year,
                                  room->active_mission_code, MFS_HIST);
    if(dir.empty()) return;
    mkdir_p(dir);
    std::string full = dir + "/" + fname;

    // 기존 stream이 같은 filename으로 열려있으면 닫음
    auto it = room->hist_streams.find(fname);
    if(it != room->hist_streams.end() && it->second.fp){
        fclose(it->second.fp);
        room->hist_streams.erase(it);
    }

    FILE* fp = fopen(full.c_str(), "wb");
    if(!fp){
        printf("[Central][Archive] HIST live_start open FAIL %s errno=%d\n",
               full.c_str(), errno);
        return;
    }
    // FileHeader 작성 (long_waterfall::FileHeader v3 128B와 동일 layout 재현)
    // 이걸 직접 헤더 import 없이 inline으로 박는다 — 대신 fields 만큼 정확히.
    struct __attribute__((packed)) LwfHeader128 {
        char     magic[4];          // "BWWF"
        uint16_t version;           // 0x0003
        uint32_t fft_size;
        uint64_t sample_rate_hz;
        uint64_t center_freq_hz;
        float    row_rate_hz;
        float    db_min;
        float    db_max;
        uint64_t start_utc_unix;
        float    station_lon;
        uint32_t fft_input_size;
        int32_t  utc_offset_hours;
        uint8_t  reserved_v2[6];
        char     station_name[32];
        float    station_lat;
        uint8_t  reserved_v3[28];
    } h{};
    static_assert(sizeof(LwfHeader128) == 128, "LwfHeader128 must be 128 bytes");
    memcpy(h.magic, "BWWF", 4);
    h.version = 0x0003;
    h.fft_size = ls.fft_size;
    h.sample_rate_hz = ls.sample_rate_hz;
    h.center_freq_hz = ls.center_freq_hz;
    h.row_rate_hz = ls.row_rate_hz;
    h.db_min = ls.db_min;
    h.db_max = ls.db_max;
    h.start_utc_unix = ls.start_utc_unix;
    h.station_lon = ls.station_lon;
    h.fft_input_size = ls.fft_input_size;
    h.utc_offset_hours = ls.utc_offset_hours;
    memcpy(h.station_name, ls.station_name, sizeof(h.station_name));
    h.station_lat = ls.station_lat;
    fwrite(&h, 1, sizeof(h), fp);
    fflush(fp);

    MissionHistStream st;
    st.archive_path = full;
    st.fp = fp;
    strncpy(st.station, room->active_mission_station, sizeof(st.station)-1);
    st.year = room->active_mission_year;
    strncpy(st.code, room->active_mission_code, sizeof(st.code)-1);
    st.fft_size = ls.fft_size;
    st.rows_written = 0;
    room->hist_streams.emplace(fname, std::move(st));

    printf("[Central][Archive] HIST stream OPEN %s (fft=%u, %.3fMHz)\n",
           full.c_str(), ls.fft_size, ls.center_freq_hz / 1e6);
}

void CentralServer::archive_hist_on_live_row(std::shared_ptr<HostRoom> room,
                                              const PktLwfLiveRowHdr& hdr,
                                              const uint8_t* row, uint32_t row_bytes){
    char fname[65]={}; memcpy(fname, hdr.filename, 64); fname[64]=0;
    auto it = room->hist_streams.find(fname);
    if(it == room->hist_streams.end() || !it->second.fp) return;
    auto& st = it->second;
    if(row_bytes == 0) return;
    fwrite(row, 1, row_bytes, st.fp);
    st.rows_written++;
    // 매 row fflush — row_rate 5Hz 라 부담 없음. UI LIST_REQ 가 stat() 으로 size
    // 가져올 때 libc user-buffer 가 비어있어야 정확한 progress 표시.
    fflush(st.fp);
}

void CentralServer::archive_hist_on_live_stop(std::shared_ptr<HostRoom> room,
                                               const PktLwfLiveStop& stop){
    char fname[65]={}; memcpy(fname, stop.filename, 64); fname[64]=0;
    auto it = room->hist_streams.find(fname);
    if(it == room->hist_streams.end()) return;
    if(it->second.fp){
        fflush(it->second.fp);
        fclose(it->second.fp);
        it->second.fp = nullptr;
    }
    // Empty file (header-only) — discard instead of finalize. Matches host-side
    // guard in long_waterfall.cpp::close_file_locked. Avoids 128B 0000-0000.bewehist
    // generated by midnight rollover triple-rotate.
    if(it->second.rows_written == 0){
        printf("[Central][Archive] HIST discard empty %s (no rows)\n",
               it->second.archive_path.c_str());
        unlink(it->second.archive_path.c_str());
        unlink((it->second.archive_path + ".info").c_str());
        room->hist_streams.erase(it);
        return;
    }
    // -LIVE → finalize 이름으로 rename. Host long_waterfall.cpp 와 동일 규칙.
    // UTC offset = 0 (Central wall-clock 기준; host 측 offset 도 wallclock 과 매우 가까움).
    std::string base = fname;
    std::string fin  = LongWaterfall::build_hist_filename_finalize(
                        base, (uint64_t)time(nullptr), 0);
    if(fin != base){
        auto slash = it->second.archive_path.find_last_of('/');
        std::string dir2 = (slash == std::string::npos) ? ""
                         : it->second.archive_path.substr(0, slash + 1);
        std::string finalp = dir2 + fin;
        if(rename(it->second.archive_path.c_str(), finalp.c_str()) == 0){
            printf("[Central][Archive] HIST finalize %s -> %s\n", base.c_str(), fin.c_str());
        } else {
            printf("[Central][Archive] HIST finalize rename FAIL %s -> %s errno=%d (%s)\n",
                   it->second.archive_path.c_str(), finalp.c_str(),
                   errno, strerror(errno));
        }
    }
    printf("[Central][Archive] HIST stream CLOSE %s (%u rows)\n",
           it->second.archive_path.c_str(), it->second.rows_written);
    room->hist_streams.erase(it);
}

// ── MISSION_SYNC.active shadow ───────────────────────────────────────────
void CentralServer::update_active_mission_shadow(std::shared_ptr<HostRoom> room,
                                                  const uint8_t* bewe_pkt, size_t bewe_len){
    if(bewe_len < BEWE_HDR_SIZE + sizeof(PktMissionSync)) {
        room->active_mission_valid = false;
        return;
    }
    const auto* m = reinterpret_cast<const PktMissionSync*>(bewe_pkt + BEWE_HDR_SIZE);
    if(!m->active_valid || m->active.valid == 0){
        if(room->active_mission_valid){
            printf("[Central][Archive] active mission CLEAR (was %04u/%s)\n",
                   room->active_mission_year, room->active_mission_code);
        }
        room->active_mission_valid = false;
        room->active_mission_year = 0;
        memset(room->active_mission_code, 0, sizeof(room->active_mission_code));
        memset(room->active_mission_station, 0, sizeof(room->active_mission_station));
        return;
    }
    room->active_mission_valid = true;
    room->active_mission_year = m->active.year;
    memcpy(room->active_mission_code,    m->active.code,         sizeof(room->active_mission_code));
    memcpy(room->active_mission_station, m->active.station_name, sizeof(room->active_mission_station));
    room->active_mission_code[sizeof(room->active_mission_code)-1] = 0;
    room->active_mission_station[sizeof(room->active_mission_station)-1] = 0;
    printf("[Central][Archive] active mission = %04u/%s @ %s\n",
           room->active_mission_year, room->active_mission_code,
           room->active_mission_station);
}

