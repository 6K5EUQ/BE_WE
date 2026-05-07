#include "sat_view.hpp"
#include "sat_tle.hpp"
#include "globe.hpp"
#include "bewe_paths.hpp"
#include "imgui.h"
#include <vector>
#include <set>
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
    enum SatMode { SAT_OFF = 0, SAT_FAVORITE = 1, SAT_ALL = 2 };

    std::vector<TleElem> g_sats;
    std::set<int>        g_favorites;   // catalog_num set, populated later
    int                  g_selected = -1;
    int                  g_mode     = SAT_ALL;
    const double         R_EARTH_KM = 6378.137;

    bool sat_visible(size_t i) {
        if (g_mode == SAT_OFF) return false;
        if (g_mode == SAT_ALL) return true;
        return g_favorites.count(g_sats[i].catalog_num) > 0;
    }

    // lat/lon/alt → globe-local xyz (unit sphere = radius 1).
    // Matches globe.cpp project() convention: lat is negated to map north → -y.
    void latlonalt_to_world(double lat_deg, double lon_deg, double alt_km,
                            float& x, float& y, float& z) {
        double r = 1.0 + alt_km / R_EARTH_KM;
        double lat = -lat_deg * M_PI / 180.0;   // negate (mirrors globe.cpp:402)
        double lon = lon_deg * M_PI / 180.0;
        x = (float)(r * cos(lat) * cos(lon));
        y = (float)(r * sin(lat));
        z = (float)(r * cos(lat) * sin(lon));
    }
}

void sat_view_init() {
    g_sats.clear();
    g_selected = -1;
    std::string path = BEWEPaths::assets_dir() + "/tle/sample.txt";
    if (!tle_load(path, g_sats)) {
        fprintf(stderr, "[sat_view] failed to load TLE: %s\n", path.c_str());
        return;
    }
    fprintf(stderr, "[sat_view] loaded %d satellites from %s\n",
            (int)g_sats.size(), path.c_str());
}

void sat_view_draw(GlobeRenderer& globe, ImGuiIO& io, time_t now_utc) {
    if (g_sats.empty()) return;

    // ── Bottom-left control: Off / Favorite / All ────────────────────────
    {
        ImVec2 ds = io.DisplaySize;
        ImGui::SetNextWindowPos(ImVec2(8, ds.y - 8),
                                ImGuiCond_Always, ImVec2(0.f, 1.f));
        ImGui::SetNextWindowBgAlpha(0.55f);
        ImGui::Begin("##sat_ctrl", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::TextUnformatted("Sat");
        ImGui::SameLine();
        ImGui::RadioButton("Off",      &g_mode, SAT_OFF);     ImGui::SameLine();
        ImGui::RadioButton("Favorite", &g_mode, SAT_FAVORITE);ImGui::SameLine();
        ImGui::RadioButton("All",      &g_mode, SAT_ALL);
        ImGui::End();
    }
    if (g_mode == SAT_OFF) return;

    ImDrawList* fdl = ImGui::GetForegroundDrawList();

    // ── Selected satellite: orbit polyline (next ~1 period) ──────────────
    if (g_selected >= 0 && g_selected < (int)g_sats.size()
        && sat_visible((size_t)g_selected)) {
        const TleElem& e = g_sats[g_selected];
        double period_sec = 86400.0 / e.mean_motion_revs_per_day;
        const int K = 120;
        bool prev_ok = false;
        float prev_sx = 0.f, prev_sy = 0.f;
        for (int i = 0; i <= K; i++) {
            time_t t = now_utc + (time_t)((double)i / K * period_sec);
            double lat, lon, alt;
            tle_propagate(e, t, lat, lon, alt);
            float wx, wy, wz;
            latlonalt_to_world(lat, lon, alt, wx, wy, wz);
            float sx, sy;
            bool ok = globe.project_world(wx, wy, wz, sx, sy);
            if (ok && prev_ok) {
                fdl->AddLine(ImVec2(prev_sx, prev_sy), ImVec2(sx, sy),
                             IM_COL32(120, 220, 255, 200), 2.0f);
            }
            prev_ok = ok;
            prev_sx = sx;
            prev_sy = sy;
        }
    }

    // ── All satellite markers ────────────────────────────────────────────
    for (size_t i = 0; i < g_sats.size(); i++) {
        if (!sat_visible(i)) continue;
        const TleElem& e = g_sats[i];
        double lat, lon, alt;
        tle_propagate(e, now_utc, lat, lon, alt);
        float wx, wy, wz;
        latlonalt_to_world(lat, lon, alt, wx, wy, wz);
        float sx, sy;
        if (!globe.project_world(wx, wy, wz, sx, sy)) continue;

        bool selected = ((int)i == g_selected);
        ImU32 outer = selected ? IM_COL32(255, 200,  60, 100)
                               : IM_COL32(255, 200,  60,  60);
        ImU32 inner = selected ? IM_COL32(255, 220, 100, 240)
                               : IM_COL32(240, 180,  40, 220);
        ImU32 core  = IM_COL32(255, 250, 220, 255);

        fdl->AddCircle      (ImVec2(sx,sy), 12.f, outer, 32, 3.f);
        fdl->AddCircleFilled(ImVec2(sx,sy),  6.f, inner);
        fdl->AddCircleFilled(ImVec2(sx,sy),  3.f, core);

        // Hover tooltip: name + alt
        float dx = sx - io.MousePos.x, dy = sy - io.MousePos.y;
        if (dx*dx + dy*dy < 196.f) {
            fdl->AddText(ImVec2(sx + 12, sy - 8),
                         IM_COL32(255, 240, 200, 255), e.name.c_str());
            char buf[64];
            snprintf(buf, sizeof buf, "alt %.0f km", alt);
            fdl->AddText(ImVec2(sx + 12, sy + 4),
                         IM_COL32(220, 200, 160, 220), buf);
        }
    }
}

bool sat_view_handle_click(GlobeRenderer& globe, float mx, float my) {
    if (g_sats.empty() || g_mode == SAT_OFF) return false;
    time_t now = time(nullptr);
    int best_idx = -1;
    float best_d2 = 25.f * 25.f;   // 25px hit radius
    for (size_t i = 0; i < g_sats.size(); i++) {
        if (!sat_visible(i)) continue;
        double lat, lon, alt;
        tle_propagate(g_sats[i], now, lat, lon, alt);
        float wx, wy, wz;
        latlonalt_to_world(lat, lon, alt, wx, wy, wz);
        float sx, sy;
        if (!globe.project_world(wx, wy, wz, sx, sy)) continue;
        float dx = sx - mx, dy = sy - my;
        float d2 = dx*dx + dy*dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = (int)i;
        }
    }
    if (best_idx < 0) return false;
    g_selected = (g_selected == best_idx) ? -1 : best_idx;   // toggle
    return true;
}
