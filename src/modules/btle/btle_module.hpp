#pragma once
// ── BLE 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ─────────────────────────
#include "btle_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

struct FFTViewer;

namespace btle_mod {

extern std::mutex               mtx;
extern std::vector<BtleRecord>  log;
extern char                     filter[64];
constexpr int LOG_MAX = 100000;

// 워커 슬롯 접근자 (btle_module.cpp)
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

// HOST 워커 (btle_decode.cpp)
void worker(FFTViewer& v, int ch_idx);
// 워커 → 스탬프 + 호스트 아카이브 + framework emit
void host_emit(FFTViewer& v, BtleRecord m);

// 공통 (btle_module.cpp)
void append_log(const BtleRecord& m);
void store_append(const BtleRecord& m);          // ~/BE_WE/modules/btle/btle_YYYYMMDD.jsonl
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<BtleRecord>& out);

#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void local_load_today(FFTViewer& v);
#endif

} // namespace btle_mod
