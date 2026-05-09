#pragma once
#include <string>

// Child-process boot args parsed from CLI in main.cpp.
// When mode_set is true, ui.cpp skips the globe / mode-selection loop and
// jumps straight into HOST or JOIN operation mode.
struct SessionArgs {
    bool        mode_set = false;
    std::string mode;          // "host" or "join"
    std::string station_id;    // Central room id (JOIN)
    std::string station_name;  // display name (both)
    float       station_lat = 0.f;
    float       station_lon = 0.f;
};

extern SessionArgs g_session_args;
