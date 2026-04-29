#pragma once
// .info 파일(Key: Value 텍스트) 파싱 + 단위·시각 추출 헬퍼.
// 운영자 자유 입력에 관용적: 잘못된 줄은 스킵, 단위 누락도 best-effort.

#include <string>
#include <map>
#include <cstdint>

namespace InfoParse {

// 전체 텍스트 → key→value 맵. 키와 값 둘 다 trim. 중복 키는 마지막 값 유지.
std::map<std::string, std::string> parse(const std::string& text);

// "145.230 MHz", "145230 kHz", "0.145 GHz", "145.230" → MHz.
// 단위 누락 시 MHz로 가정. 숫자 못 뽑으면 false + out 미변경.
bool extract_freq_mhz(const std::string& v, float& out);

// "151.2 kHz", "0.151 MHz", "151200 Hz" → kHz. 단위 누락 시 kHz 가정.
bool extract_bw_khz(const std::string& v, float& out);

// "2.0 s", "2:34", "01:23:45", "5 min" → 초. 못 뽑으면 false.
bool extract_duration_s(const std::string& v, uint32_t& out);

// "Apr 29, 2026" + "19:41:21 (UTC+9)" → unix epoch (UTC). 못 뽑으면 false.
bool extract_start_utc(const std::string& day, const std::string& time_str, int64_t& out);

// 양끝 공백 제거.
std::string trim(const std::string& s);

// ASCII 소문자.
std::string lower(const std::string& s);

} // namespace InfoParse
