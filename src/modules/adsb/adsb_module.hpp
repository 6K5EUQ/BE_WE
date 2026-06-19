#pragma once
// ── ADS-B 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ─────────────────────
#include "adsb_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

struct FFTViewer;

namespace adsb_mod {

extern std::mutex               mtx;
extern std::vector<AdsbRecord>  log;
extern char                     filter[64];
constexpr int LOG_MAX = 100000;

// 워커 슬롯 접근자 (adsb_module.cpp)
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// HOST 워커 (adsb_decode.cpp)
void worker(FFTViewer& v, int ch_idx);
// 워커 → 스탬프 + 호스트 아카이브 + framework emit
void host_emit(FFTViewer& v, AdsbRecord m);

// 공통 (adsb_module.cpp)
void append_log(const AdsbRecord& m);
void store_append(const AdsbRecord& m);          // ~/BE_WE/modules/adsb/adsb_YYYYMMDD.jsonl
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<AdsbRecord>& out);

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void local_load_today(FFTViewer& v);
#endif

} // namespace adsb_mod
