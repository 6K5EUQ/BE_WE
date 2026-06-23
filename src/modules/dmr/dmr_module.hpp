#pragma once
// ── DMR 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ──────────────────────
#include "dmr_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

struct FFTViewer;

namespace dmr_mod {

extern std::mutex              mtx;
extern std::vector<DmrRecord>  log;
extern char                    filter[64];
constexpr int LOG_MAX = 100000;

// 워커 슬롯 접근자 (dmr_module.cpp)
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// HOST 워커 (dmr_decode.cpp)
void worker(FFTViewer& v, int ch_idx);
// 워커 → 스탬프 + 호스트 아카이브 + framework emit
void host_emit(FFTViewer& v, DmrRecord m);

// 공통 (dmr_module.cpp)
void append_log(const DmrRecord& m);
void store_append(const DmrRecord& m);          // ~/BE_WE/modules/dmr/dmr_YYYYMMDD.jsonl
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<DmrRecord>& out);

// 온디맨드 녹음 WAV 페치 (dmr_module.cpp) — 뷰에서 폴링/정리
int  dmr_rec_state(uint64_t rec_id, std::string& path);   // 0=대기, 2=ready(path), 3=파일없음
void dmr_rec_cleanup();                                   // bewe 종료 시 임시파일 제거

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void local_load_today(FFTViewer& v);
#endif

} // namespace dmr_mod
