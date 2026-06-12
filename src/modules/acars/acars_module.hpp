#pragma once
// ── ACARS 모듈 내부 공유 선언 (모듈 밖에서 include 금지) ─────────────────────
#include "acars_meta.hpp"
#include "config.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct FFTViewer;

namespace acars_mod {

// 데이터 레코드 wire 포맷 (framework BEWE_MK_DATA payload; station 은 MpData 봉투)
struct __attribute__((packed)) WireMsg {
    int64_t t_ms; float freq;
    uint8_t ch, crc_ok, downlink;
    char mode, block, ack;
    char reg[10], flight[8], label[3];
    char text[256];
};

// ── 디코드 로그 (표시) ──
extern std::mutex            mtx;
extern std::vector<AcarsMsg> msglog;
extern bool                  scroll;
extern char                  filter[64];
constexpr int LOG_MAX = 5000;

// ── HOST 워커 (acars_decode.cpp) ──
void worker(FFTViewer& v, int ch_idx);
void host_emit(FFTViewer& v, AcarsMsg m);    // 워커 → 스탬프 + 호스트 아카이브 + framework emit

// ── 공통 (acars_module.cpp) ──
void append_log(const AcarsMsg& m);
void msg_to_wire(const AcarsMsg& m, WireMsg& w);
void wire_to_msg(const WireMsg& w, AcarsMsg& m);
// 호스트 일 단위 아카이브: ~/BE_WE/modules/acars/acars_YYYYMMDD.jsonl (KST)
void store_append(const AcarsMsg& m);
bool store_read_today(std::string& out);
void store_parse_jsonl(const char* data, size_t n, std::vector<AcarsMsg>& out);

// ── GUI (acars_view.cpp) ──
#ifndef BEWE_HEADLESS
void draw_content(FFTViewer& v, bool just_opened);
void local_load_today(FFTViewer& v);   // LOCAL: 오늘 아카이브 직접 로드
#endif

} // namespace acars_mod
