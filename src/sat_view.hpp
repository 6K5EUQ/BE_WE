#pragma once
#include <ctime>

class GlobeRenderer;
struct ImGuiIO;

// Refresh assets/tle/sot_tle.txt and assets/tle/starlink_tle.txt from
// celestrak.org via curl. Skips work if the local files are <1 week old.
// Silent on failure (existing file kept). Call once before sat_view_init().
// `force=true` bypasses the staleness throttle (used by the manual Update
// button so users can refresh on demand). `false` honours the throttle.
void sat_tle_fetch(bool force = false);

// Refresh SOI_tle.txt only (per-satellite CATNR query, 24h throttle).
// Lighter than sat_tle_fetch() since it touches just the SOI list.
void sat_tle_refresh_soi(bool force = false);

void sat_view_init();                 // load sot_tle.txt + starlink_tle.txt + SOI_tle.txt
void sat_view_draw(GlobeRenderer& globe, ImGuiIO& io, time_t now_utc);

// Returns true if a satellite marker was hit (and selection updated).
// Caller should skip station/pick logic when true.
bool sat_view_handle_click(GlobeRenderer& globe, float mx, float my);
