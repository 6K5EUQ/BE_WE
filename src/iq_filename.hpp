#pragma once
// 파일명 기반 IQ vs Demod 분류 헬퍼.
// 규칙: 파일명 어디든 "IQ" 가 들어 있으면 IQ 원본, 그 외엔 Demod (복조 결과물).
// 새 prefix 추가 시 이 한 곳만 고치면 됨.

#include <cstring>
#include <string>

inline bool is_iq_filename(const char* fn){
    if(!fn) return false;
    return std::strstr(fn, "IQ") != nullptr;
}
inline bool is_iq_filename(const std::string& s){
    return is_iq_filename(s.c_str());
}
