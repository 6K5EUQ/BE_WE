#pragma once
#include <ctime>

class GlobeRenderer;
struct ImGuiIO;

void sat_view_init();                 // load assets/tle/sample.txt
void sat_view_draw(GlobeRenderer& globe, ImGuiIO& io, time_t now_utc);

// Returns true if a satellite marker was hit (and selection updated).
// Caller should skip station/pick logic when true.
bool sat_view_handle_click(GlobeRenderer& globe, float mx, float my);
