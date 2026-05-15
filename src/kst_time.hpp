#pragma once
// KST (Korea Standard Time, UTC+9) helpers — force KST regardless of system TZ.
// All timestamp displays and filenames in BE_WE use KST via these helpers.
//
// Internal storage continues to use time_t / unix epoch (UTC seconds);
// these helpers only shift for display/filename rendering and for
// "what KST hour is it now" rotation checks.

#include <ctime>

namespace KST {

static constexpr int OFFSET_HOURS = 9;
static constexpr int OFFSET_SEC   = OFFSET_HOURS * 3600;

// Fill `out` with KST broken-down time. tm_gmtoff/tm_zone are NOT set (gmtime_r doesn't).
inline void to_tm(time_t t, struct tm& out){
    time_t shifted = t + OFFSET_SEC;
    gmtime_r(&shifted, &out);
}

// "Now" in KST.
inline void now_tm(struct tm& out){
    time_t now = time(nullptr);
    to_tm(now, out);
}

// Convenience: KST tm as a value.
inline struct tm to_tm(time_t t){
    struct tm out{};
    to_tm(t, out);
    return out;
}

} // namespace KST
