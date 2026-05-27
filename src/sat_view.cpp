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
    enum SatMode { SAT_SOI = 0, SAT_LEO = 1, SAT_ALL = 2 };

    std::vector<TleElem> g_sats;
    std::set<int>        g_soi_ids;       // catalog_nums listed in SOI_tle.txt
    std::set<int>        g_seen_ids;      // catalog_nums already in g_sats (dedupe)
    bool                 g_all_loaded = false; // starlink+etc loaded yet?
    bool                 g_leo_loaded = false; // leo_tle loaded yet?
    int                  g_selected = -1;
    int                  g_mode     = SAT_SOI;   // default
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

    // Priority: Starlink=white > SOI=red > LEO(<2000km)=blue > MEO=yellow > GEO=light-blue
    ImU32 sat_color(size_t i, double alt_km) {
        const TleElem& e = g_sats[i];
        if (e.is_starlink)                  return IM_COL32(255, 255, 255, 0);
        if (g_soi_ids.count(e.catalog_num)) return IM_COL32(255,  60,  60, 0);
        if (alt_km < 2000.0)                return IM_COL32( 80, 160, 255, 0);
        if (alt_km < 35000.0)               return IM_COL32(255, 220,  60, 0);
        return                                     IM_COL32(120, 200, 255, 0);
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
        if (g_mode == SAT_ALL) return true;
        if (g_mode == SAT_LEO) {
            const TleElem& e = g_sats[i];
            if (e.is_starlink) return false;
            double alt_km = e.semi_major_km - 6378.137;
            return alt_km < 2000.0;
        }
        return g_soi_ids.count(g_sats[i].catalog_num) > 0;  // SAT_SOI
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
        if (e.semi_major_km <= 0.0) return;
        double n_rps = sqrt(398600.4418 / (e.semi_major_km * e.semi_major_km * e.semi_major_km));
        double period = 2.0 * M_PI / n_rps;  // seconds
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

void sat_tle_fetch(bool force) {
    const time_t WEEK = 7 * 86400;
    std::string dir = BEWEPaths::assets_dir() + "/tle";

    // sot_tle.txt + SOI_tle.txt are user-managed (never auto-fetched).
    // starlink_tle.txt = GROUP=starlink. etc_tle.txt = GROUP=gnss + GROUP=geo.
    bool starlink_fresh = !force && file_age_under(dir + "/starlink_tle.txt", WEEK);
    bool etc_fresh      = !force && file_age_under(dir + "/etc_tle.txt",      WEEK);
    bool leo_fresh      = !force && file_age_under(dir + "/leo_tle.txt",      WEEK);
    if (starlink_fresh && etc_fresh && leo_fresh) {
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
        fprintf(stderr, "[sat_tle] fetching GROUP=%s ...\n", group);
        int rc = system(cmd);
        if (rc == 0)
            fprintf(stderr, "[sat_tle] update complete: %s (GROUP=%s)\n",
                    fname, group);
        else
            fprintf(stderr, "[sat_tle] update FAILED: %s (GROUP=%s, rc=%d)\n",
                    fname, group, rc);
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
        fprintf(stderr, "[sat_tle] fetching GROUP=%s+%s ...\n", g1, g2);
        int rc = system(cmd);
        if (rc == 0)
            fprintf(stderr, "[sat_tle] update complete: %s (GROUP=%s+%s)\n",
                    fname, g1, g2);
        else
            fprintf(stderr, "[sat_tle] update FAILED: %s (GROUP=%s+%s, rc=%d)\n",
                    fname, g1, g2, rc);
    };

    if (!starlink_fresh) fetch_single  ("starlink",       "starlink_tle.txt");
    if (!etc_fresh)      fetch_combined("gnss", "geo",    "etc_tle.txt");
    if (!leo_fresh)      fetch_single  ("active",         "leo_tle.txt");
}

// Refresh SOI_tle.txt every 24h via per-satellite CATNR queries (parallel).
// NORAD updates LEO 1–3×/day, so 24h granularity is the practical sweet spot.
void sat_tle_refresh_soi(bool force) {
    const time_t DAY = 24 * 3600;
    std::string dir = BEWEPaths::assets_dir() + "/tle";
    std::string soi_path = dir + "/SOI_tle.txt";
    struct stat st;
    if (!force && stat(soi_path.c_str(), &st) == 0 &&
        (time(nullptr) - st.st_mtime) < DAY) {
        fprintf(stderr, "[sat_tle] SOI <24h old, skipping refresh\n");
        return;
    }
    std::vector<TleElem> existing;
    if (!tle_load(soi_path, existing) || existing.empty()) {
        fprintf(stderr, "[sat_tle] SOI missing/empty, skip refresh\n");
        return;
    }
    // Build a single shell command: parallel curl -Z, then cat & mv.
    std::string cmd = "curl --max-time 30 -fsS -A 'Mozilla/5.0' -Z";
    std::vector<std::string> tmp;
    for (size_t i = 0; i < existing.size(); i++) {
        char buf[64];
        snprintf(buf, sizeof buf, "%s/.soi_%zu.tmp", dir.c_str(), i);
        tmp.emplace_back(buf);
        char part[512];
        snprintf(part, sizeof part,
            " -o '%s' 'https://celestrak.org/NORAD/elements/"
            "gp.php?CATNR=%d&FORMAT=tle'",
            tmp.back().c_str(), existing[i].catalog_num);
        cmd += part;
    }
    std::string out_tmp = soi_path + ".new";
    cmd += " && cat";
    for (auto& t : tmp) cmd += " '" + t + "'";
    cmd += " > '" + out_tmp + "' && mv '" + out_tmp + "' '" + soi_path + "'";
    cmd += "; rm -f";
    for (auto& t : tmp) cmd += " '" + t + "'";
    cmd += " '" + out_tmp + "'";
    fprintf(stderr, "[sat_tle] refreshing SOI list (%zu sats) ...\n",
            existing.size());
    int rc = system(cmd.c_str());
    if (rc == 0)
        fprintf(stderr, "[sat_tle] update complete: SOI_tle.txt (%zu sats)\n",
                existing.size());
    else
        fprintf(stderr, "[sat_tle] update FAILED: SOI_tle.txt (rc=%d)\n", rc);
}

void sat_view_init() {
    // No auto-fetch on startup — user triggers via the Update button in the
    // bottom-left panel. Loads whatever TLE files currently exist on disk.
    g_sats.clear();
    g_pos_cache.clear();
    g_soi_ids.clear();
    g_seen_ids.clear();
    g_all_loaded = false;
    g_leo_loaded = false;
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
        // No auto-fetch — load whatever starlink+etc files already exist.
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

    void ensure_leo_loaded() {
        if (g_leo_loaded) return;
        g_leo_loaded = true;
        std::string dir = BEWEPaths::assets_dir() + "/tle";

        // leo_tle.txt (GROUP=active) contains Starlink too. Skip those —
        // they're either loaded separately by ensure_all_loaded() or hidden.
        std::set<int> starlink_ids;
        {
            std::vector<TleElem> sl;
            tle_load(dir + "/starlink_tle.txt", sl);
            for (auto& s : sl) starlink_ids.insert(s.catalog_num);
        }

        std::vector<TleElem> leo;
        tle_load(dir + "/leo_tle.txt", leo);
        int kept = 0, skipped_sl = 0;
        g_sats.reserve(g_sats.size() + leo.size());
        for (auto& e : leo) {
            if (starlink_ids.count(e.catalog_num)) { skipped_sl++; continue; }
            if (g_seen_ids.insert(e.catalog_num).second) {
                e.is_leo = true;
                g_sats.push_back(std::move(e)); kept++;
            }
        }
        g_pos_cache.assign(g_sats.size(), PosCache{});
        fprintf(stderr, "[sat_view] LEO: +%d sats (-%d starlink filtered); %zu total\n",
                kept, skipped_sl, g_sats.size());
    }
}

void sat_view_update_tle() {
    sat_tle_fetch(true);
    sat_tle_refresh_soi(true);
    sat_view_init();
    if (g_mode == SAT_ALL) { ensure_all_loaded(); ensure_leo_loaded(); }
    if (g_mode == SAT_LEO) ensure_leo_loaded();
    fprintf(stderr, "[sat_view] TLE manually updated\n");
}

void sat_view_draw(GlobeRenderer& globe, ImGuiIO& io, time_t now_utc) {
    // ── Bottom-left control: ALL / SOI / OFF ─────────────────────────────
    {
        ImVec2 ds = io.DisplaySize;
        ImGui::SetNextWindowPos(ImVec2(16, ds.y - 16),
                                ImGuiCond_Always, ImVec2(0.f, 1.f));
        ImGui::SetNextWindowSize(ImVec2(200, 0), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.85f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 6.f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f,0.10f,0.16f,1.f));
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
        // Single row: SOI / LEO / ALL
        float frame_h  = ImGui::GetFrameHeight();
        float inner_sp = ImGui::GetStyle().ItemInnerSpacing.x;
        float item_sp  = ImGui::GetStyle().ItemSpacing.x;
        float lw_soi   = ImGui::CalcTextSize("SOI").x;
        float lw_leo   = ImGui::CalcTextSize("LEO").x;
        float lw_all   = ImGui::CalcTextSize("ALL").x;
        float row_w    = (frame_h + inner_sp) * 3
                       + lw_soi + lw_leo + lw_all + item_sp * 2;
        float avail_w  = ImGui::GetContentRegionAvail().x;
        if (avail_w > row_w)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail_w - row_w) * 0.5f);
        ImGui::RadioButton("SOI", &g_mode, SAT_SOI);
        ImGui::SameLine();
        if (ImGui::RadioButton("LEO", &g_mode, SAT_LEO)) ensure_leo_loaded();
        ImGui::SameLine();
        if (ImGui::RadioButton("ALL", &g_mode, SAT_ALL)) { ensure_all_loaded(); ensure_leo_loaded(); }

        // (Update TLE 버튼 제거 — chat 입력 `/Update TLEs` 명령어로 트리거)
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }
    if (g_sats.empty()) return;
    ImDrawList* fdl = ImGui::GetForegroundDrawList();

    // ── Selected satellite orbit (frozen at click time) ──────────────────
    if (g_selected >= 0 && g_selected == g_orbit.sat_idx
        && sat_visible((size_t)g_selected) && !g_orbit.wpts.empty()) {
        const TleElem& e = g_sats[g_selected];
        double sel_alt   = e.semi_major_km - R_EARTH_KM;
        ImU32  col_orbit = with_alpha(sat_color((size_t)g_selected, sel_alt), 220);

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
        ImU32  c        = sat_color(i, pc.alt);
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
    if (g_sats.empty()) return false;
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
