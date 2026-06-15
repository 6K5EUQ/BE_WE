#pragma once
// ── AIS 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ──────────────────────
#include "ais_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

struct FFTViewer;

namespace ais_mod {

extern std::mutex              mtx;
extern std::vector<AisRecord>  log;
extern char                    filter[64];
constexpr int LOG_MAX = 100000;

// 워커 슬롯 접근자 (ais_module.cpp)
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// HOST 워커 (ais_decode.cpp)
void worker(FFTViewer& v, int ch_idx);
// 워커 → 스탬프 + 호스트 아카이브 + framework emit
void host_emit(FFTViewer& v, AisRecord m);

// 공통 (ais_module.cpp)
void append_log(const AisRecord& m);
void store_append(const AisRecord& m);          // ~/BE_WE/modules/ais/ais_YYYYMMDD.jsonl
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<AisRecord>& out);

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void local_load_today(FFTViewer& v);
#endif

} // namespace ais_mod
