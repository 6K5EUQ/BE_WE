#pragma once
#include <ctime>

class GlobeRenderer;
struct ImGuiIO;

// Refresh assets/tle/sot_tle.txt and assets/tle/starlink_tle.txt from
// celestrak.org via curl. Skips work if the local files are <1 week old.
// Silent on failure (existing file kept). Call once before sat_view_init().
void sat_tle_fetch();

void sat_view_init();                 // load sot_tle.txt + starlink_tle.txt + SOI_tle.txt
void sat_view_draw(GlobeRenderer& globe, ImGuiIO& io, time_t now_utc);

// Returns true if a satellite marker was hit (and selection updated).
// Caller should skip station/pick logic when true.
bool sat_view_handle_click(GlobeRenderer& globe, float mx, float my);
