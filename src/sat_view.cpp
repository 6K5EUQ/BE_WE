#include "sat_view.hpp"
#include "sat_tle.hpp"
#include "globe.hpp"
#include "bewe_paths.hpp"
#include "imgui.h"
#include <vector>
#include <set>
#include <array>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
    enum SatMode { SAT_OFF = 0, SAT_SOI = 1, SAT_ALL = 2 };

    std::vector<TleElem> g_sats;
    std::set<int>        g_soi_ids;       // catalog_nums listed in SOI_tle.txt
    std::set<int>        g_seen_ids;      // catalog_nums already in g_sats (dedupe)
    bool                 g_all_loaded = false; // starlink+etc loaded yet?
    int                  g_selected = -1;
    int                  g_mode     = SAT_SOI;
    const double         R_EARTH_KM = 6378.137;

    // Cached propagator output, reused while the UTC second is unchanged.
    struct PosCache {
        time_t valid_at = 0;
        double lat = 0.0, lon = 0.0, alt = 0.0;
        float  wx = 0.f, wy = 0.f, wz = 0.f;
    };
    std::vector<PosCache> g_pos_cache;

    // Frozen orbit polyline; future segments turn solid as time advances.
    struct OrbitPath {
        int    sat_idx = -1;
        time_t t_start = 0;
        time_t t_end   = 0;
        std::vector<std::array<float,3>> wpts;
    };
    OrbitPath g_orbit;
    const int ORBIT_K = 720;

    ImU32 band_color_rgb(double alt_km) {
        if (alt_km < 2000.0)  return IM_COL32(255,  90,  90, 0);
        if (alt_km < 35000.0) return IM_COL32(255, 220,  60, 0);
        return                       IM_COL32(120, 200, 255, 0);
    }
    ImU32 with_alpha(ImU32 c, unsigned a) {
        return (c & 0x00FFFFFFu) | ((a & 0xFFu) << 24);
    }

    void latlonalt_to_world(double lat_deg, double lon_deg, double alt_km,
                            float& x, float& y, float& z) {
        double r   = 1.0 + alt_km / R_EARTH_KM;
        double lat = -lat_deg * M_PI / 180.0;
        // lon 반전: globe 텍스처 매핑 컨벤션에 맞추기 (East가 음수 자리에 그려짐)
        double lon = -lon_deg * M_PI / 180.0;
        x = (float)(r * cos(lat) * cos(lon));
        y = (float)(r * sin(lat));
        z = (float)(r * cos(lat) * sin(lon));
    }

    bool sat_visible(size_t i) {
        if (g_mode == SAT_OFF) return false;
        if (g_mode == SAT_ALL) return true;
        return g_soi_ids.count(g_sats[i].catalog_num) > 0;
    }

    void update_position(size_t i, time_t now_utc) {
        PosCache& c = g_pos_cache[i];
        if (c.valid_at == now_utc) return;
        tle_propagate(g_sats[i], now_utc, c.lat, c.lon, c.alt);
        latlonalt_to_world(c.lat, c.lon, c.alt, c.wx, c.wy, c.wz);
        c.valid_at = now_utc;
    }

    void orbit_cache_clear() { g_orbit = OrbitPath{}; }

    void orbit_cache_build(int sat_idx, time_t t_click) {
        orbit_cache_clear();
        if (sat_idx < 0 || sat_idx >= (int)g_sats.size()) return;
        const TleElem& e = g_sats[sat_idx];
        if (e.mean_motion_revs_per_day <= 0.0) return;
        double period = 86400.0 / e.mean_motion_revs_per_day;
        g_orbit.sat_idx = sat_idx;
        g_orbit.t_start = t_click - (time_t)(period * 0.5);
        g_orbit.t_end   = t_click + (time_t)(period * 0.5);
        int N = ORBIT_K + 1;
        g_orbit.wpts.resize(N);
        double dur = (double)(g_orbit.t_end - g_orbit.t_start);
        for (int i = 0; i < N; i++) {
            double frac = (double)i / (N - 1);
            time_t t = g_orbit.t_start + (time_t)(frac * dur);
            double lat, lon, alt;
            tle_propagate(e, t, lat, lon, alt);
            float wx, wy, wz;
            latlonalt_to_world(lat, lon, alt, wx, wy, wz);
            g_orbit.wpts[i] = {wx, wy, wz};
        }
    }

    bool file_age_under(const std::string& path, time_t max_age_sec) {
        struct stat st;
        if (stat(path.c_str(), &st) != 0) return false;
        return (time(nullptr) - st.st_mtime) < max_age_sec;
    }
}

void sat_tle_fetch() {
    const time_t WEEK = 7 * 86400;
    std::string dir = BEWEPaths::assets_dir() + "/tle";

    // sot_tle.txt + SOI_tle.txt are user-managed (never auto-fetched).
    // starlink_tle.txt = GROUP=starlink. etc_tle.txt = GROUP=gnss + GROUP=geo.
    bool starlink_fresh = file_age_under(dir + "/starlink_tle.txt", WEEK);
    bool etc_fresh      = file_age_under(dir + "/etc_tle.txt",      WEEK);
    if (starlink_fresh && etc_fresh) {
        fprintf(stderr, "[sat_tle] cached files <1 week old, skipping fetch\n");
        return;
    }

    auto fetch_single = [&](const char* group, const char* fname) {
        std::string final_path = dir + "/" + fname;
        std::string tmp_path   = final_path + ".tmp";
        char cmd[1024];
        snprintf(cmd, sizeof cmd,
            "curl --max-time 30 -fsS -A 'Mozilla/5.0' -o '%s' "
            "'https://celestrak.org/NORAD/elements/gp.php?GROUP=%s&FORMAT=tle' "
            "&& mv '%s' '%s' || rm -f '%s'",
            tmp_path.c_str(), group,
            tmp_path.c_str(), final_path.c_str(),
            tmp_path.c_str());
        int rc = system(cmd);
        fprintf(stderr, "[sat_tle] fetch GROUP=%s rc=%d\n", group, rc);
    };

    auto fetch_combined = [&](const char* g1, const char* g2, const char* fname) {
        std::string final_path = dir + "/" + fname;
        std::string t1 = final_path + ".g1.tmp";
        std::string t2 = final_path + ".g2.tmp";
        char cmd[2048];
        snprintf(cmd, sizeof cmd,
            "curl --max-time 30 -fsS -A 'Mozilla/5.0' -o '%s' "
            "'https://celestrak.org/NORAD/elements/gp.php?GROUP=%s&FORMAT=tle' && "
            "curl --max-time 30 -fsS -A 'Mozilla/5.0' -o '%s' "
            "'https://celestrak.org/NORAD/elements/gp.php?GROUP=%s&FORMAT=tle' && "
            "cat '%s' '%s' > '%s' && rm -f '%s' '%s' || rm -f '%s' '%s'",
            t1.c_str(), g1,
            t2.c_str(), g2,
            t1.c_str(), t2.c_str(), final_path.c_str(),
            t1.c_str(), t2.c_str(),
            t1.c_str(), t2.c_str());
        int rc = system(cmd);
        fprintf(stderr, "[sat_tle] fetch GROUP=%s+%s rc=%d\n", g1, g2, rc);
    };

    if (!starlink_fresh) fetch_single  ("starlink",       "starlink_tle.txt");
    if (!etc_fresh)      fetch_combined("gnss", "geo",    "etc_tle.txt");
}

void sat_view_init() {
    g_sats.clear();
    g_pos_cache.clear();
    g_soi_ids.clear();
    g_seen_ids.clear();
    g_all_loaded = false;
    g_selected = -1;
    orbit_cache_clear();

    // SOI-only fast startup. starlink + etc (the big sets) are loaded
    // lazily when the user picks ALL — see ensure_all_loaded().
    std::string dir = BEWEPaths::assets_dir() + "/tle";

    std::vector<TleElem> sot, soi;
    tle_load(dir + "/sot_tle.txt", sot);
    tle_load(dir + "/SOI_tle.txt", soi);

    for (auto& e : soi) g_soi_ids.insert(e.catalog_num);

    g_sats.reserve(sot.size() + soi.size());
    int sot_kept = 0, soi_kept = 0;
    for (auto& e : sot) {
        if (g_seen_ids.insert(e.catalog_num).second) {
            g_sats.push_back(std::move(e)); sot_kept++;
        }
    }
    for (auto& e : soi) {
        if (g_seen_ids.insert(e.catalog_num).second) {
            g_sats.push_back(std::move(e)); soi_kept++;
        }
    }

    g_pos_cache.assign(g_sats.size(), PosCache{});
    fprintf(stderr, "[sat_view] init: %d sot + %d SOI-only; %zu in SOI set; %zu total\n",
            sot_kept, soi_kept, g_soi_ids.size(), g_sats.size());
}

namespace {
    void ensure_all_loaded() {
        if (g_all_loaded) return;
        g_all_loaded = true;

        sat_tle_fetch();   // refresh starlink+etc if cache is stale (>1 week)

        std::string dir = BEWEPaths::assets_dir() + "/tle";
        std::vector<TleElem> starlink, etc;
        tle_load(dir + "/starlink_tle.txt", starlink);
        tle_load(dir + "/etc_tle.txt",      etc);
        for (auto& e : starlink) e.is_starlink = true;

        g_sats.reserve(g_sats.size() + starlink.size() + etc.size());
        int sl_kept = 0, etc_kept = 0;
        for (auto& e : starlink) {
            if (g_seen_ids.insert(e.catalog_num).second) {
                g_sats.push_back(std::move(e)); sl_kept++;
            }
        }
        for (auto& e : etc) {
            if (g_seen_ids.insert(e.catalog_num).second) {
                g_sats.push_back(std::move(e)); etc_kept++;
            }
        }
        g_pos_cache.assign(g_sats.size(), PosCache{});
        fprintf(stderr, "[sat_view] ALL: +%d starlink +%d etc; %zu total\n",
                sl_kept, etc_kept, g_sats.size());
    }
}

void sat_view_draw(GlobeRenderer& globe, ImGuiIO& io, time_t now_utc) {
    // ── Bottom-left control: ALL / SOI / OFF ─────────────────────────────
    {
        ImVec2 ds = io.DisplaySize;
        ImGui::SetNextWindowPos(ImVec2(8, ds.y - 8),
                                ImGuiCond_Always, ImVec2(0.f, 1.f));
        ImGui::SetNextWindowSize(ImVec2(200, 0), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.55f);
        ImGui::Begin("##sat_ctrl", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoSavedSettings);
        // Title centered
        const char* title = "Satellite Tracker";
        ImVec2 ts = ImGui::CalcTextSize(title);
        float avail = ImGui::GetContentRegionAvail().x;
        if (avail > ts.x)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - ts.x) * 0.5f);
        ImGui::TextUnformatted(title);
        ImGui::Separator();
        // Center the radio row.
        float frame_h    = ImGui::GetFrameHeight();
        float inner_sp   = ImGui::GetStyle().ItemInnerSpacing.x;
        float item_sp    = ImGui::GetStyle().ItemSpacing.x;
        float label_w    = ImGui::CalcTextSize("ALL").x;     // ALL/SOI/OFF same width
        float row_w      = (frame_h + inner_sp + label_w) * 3 + item_sp * 2;
        float avail_w    = ImGui::GetContentRegionAvail().x;
        if (avail_w > row_w)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail_w - row_w) * 0.5f);
        if (ImGui::RadioButton("ALL", &g_mode, SAT_ALL)) ensure_all_loaded();
        ImGui::SameLine();
        ImGui::RadioButton("SOI", &g_mode, SAT_SOI); ImGui::SameLine();
        ImGui::RadioButton("OFF", &g_mode, SAT_OFF);
        ImGui::End();
    }
    if (g_mode == SAT_OFF) return;
    if (g_sats.empty()) return;

    ImDrawList* fdl = ImGui::GetForegroundDrawList();

    // ── Selected satellite orbit (frozen at click time) ──────────────────
    if (g_selected >= 0 && g_selected == g_orbit.sat_idx
        && sat_visible((size_t)g_selected) && !g_orbit.wpts.empty()) {
        const TleElem& e = g_sats[g_selected];
        double sel_alt   = e.semi_major_km - R_EARTH_KM;
        ImU32  col_orbit = with_alpha(band_color_rgb(sel_alt), 220);

        int N = (int)g_orbit.wpts.size();
        double dur = (double)(g_orbit.t_end - g_orbit.t_start);
        double t_norm = (dur > 0.0)
            ? (((double)now_utc - (double)g_orbit.t_start) / dur) : 0.0;
        if (t_norm < 0.0) t_norm = 0.0;
        if (t_norm > 1.0) t_norm = 1.0;
        int now_idx = (int)(t_norm * (N - 1));

        std::vector<float> sx_arr(N), sy_arr(N);
        std::vector<unsigned char> ok_arr(N, 0);
        for (int i = 0; i < N; i++) {
            ok_arr[i] = globe.project_world(
                g_orbit.wpts[i][0], g_orbit.wpts[i][1], g_orbit.wpts[i][2],
                sx_arr[i], sy_arr[i]) ? 1 : 0;
        }
        for (int i = 1; i < N; i++) {
            if (!ok_arr[i-1] || !ok_arr[i]) continue;
            bool past   = (i <= now_idx);
            bool render = past || (i & 1);
            if (render) {
                fdl->AddLine(ImVec2(sx_arr[i-1], sy_arr[i-1]),
                             ImVec2(sx_arr[i],   sy_arr[i]),
                             col_orbit, 2.0f);
            }
        }
    }

    // ── Satellite markers (1-second propagate cache) ─────────────────────
    for (size_t i = 0; i < g_sats.size(); i++) {
        if (!sat_visible(i)) continue;
        update_position(i, now_utc);
        const PosCache& pc = g_pos_cache[i];
        float sx, sy;
        if (!globe.project_world(pc.wx, pc.wy, pc.wz, sx, sy)) continue;

        bool   selected = ((int)i == g_selected);
        // Starlink overrides altitude band → plain white dot regardless of LEO.
        ImU32  c        = g_sats[i].is_starlink
                        ? IM_COL32(255, 255, 255, 0)
                        : band_color_rgb(pc.alt);
        const int LAYERS = 8;
        float scale = selected ? 2.0f : 1.0f;
        float r_out = 4.5f * scale;
        float r_in  = 0.75f * scale;
        for (int k = LAYERS - 1; k >= 0; k--) {
            float t = (float)k / (float)(LAYERS - 1);
            float r = r_in + (r_out - r_in) * t;
            unsigned a = (unsigned)(240.f * (1.f - t) * (1.f - t) + 15.f);
            fdl->AddCircleFilled(ImVec2(sx, sy), r, with_alpha(c, a), 16);
        }
        fdl->AddCircleFilled(ImVec2(sx, sy), 0.75f * scale,
                             IM_COL32(255, 255, 255, 230), selected ? 16 : 8);

        // Show label on hover, while selected, or always under SOI mode
        // (SOI list is small enough that labels never crowd the screen).
        float dx = sx - io.MousePos.x, dy = sy - io.MousePos.y;
        if (g_mode == SAT_SOI || selected || dx*dx + dy*dy < 196.f) {
            const TleElem& e = g_sats[i];
            fdl->AddText(ImVec2(sx + 12, sy - 8),
                         IM_COL32(255, 240, 200, 255), e.name.c_str());
            char buf[64];
            snprintf(buf, sizeof buf, "%.0f km", pc.alt);
            fdl->AddText(ImVec2(sx + 12, sy + 4),
                         IM_COL32(220, 200, 160, 220), buf);
        }
    }
}

bool sat_view_handle_click(GlobeRenderer& globe, float mx, float my) {
    if (g_sats.empty() || g_mode == SAT_OFF) return false;
    time_t now = time(nullptr);
    int best_idx = -1;
    float best_d2 = 25.f * 25.f;
    for (size_t i = 0; i < g_sats.size(); i++) {
        if (!sat_visible(i)) continue;
        update_position(i, now);
        const PosCache& pc = g_pos_cache[i];
        float sx, sy;
        if (!globe.project_world(pc.wx, pc.wy, pc.wz, sx, sy)) continue;
        float dx = sx - mx, dy = sy - my;
        float d2 = dx*dx + dy*dy;
        if (d2 < best_d2) { best_d2 = d2; best_idx = (int)i; }
    }
    if (best_idx < 0) return false;
    if (g_selected == best_idx) {
        g_selected = -1;
        orbit_cache_clear();
    } else {
        g_selected = best_idx;
        orbit_cache_build(best_idx, now);
    }
    return true;
}
