#pragma once
#include <string>
#include <vector>
#include <ctime>

// Two-Line Element parser + simple Kepler propagator (e=0 assumed).
// Sufficient for visualization with synthetic TLEs; replace with SGP4 later.

struct TleElem {
    std::string name;
    int    catalog_num    = 0;
    time_t epoch_utc      = 0;       // TLE epoch as UTC time_t
    double inclination_deg = 0.0;
    double raan_deg        = 0.0;
    double arg_perigee_deg = 0.0;
    double mean_anomaly_deg = 0.0;
    double mean_motion_revs_per_day = 0.0;
    double semi_major_km   = 0.0;    // derived from mean motion (mu / n^2)^(1/3)
    bool   is_starlink     = false;  // tagged at load time (from starlink_tle.txt)
};

bool tle_load(const std::string& path, std::vector<TleElem>& out);

// e=0 circular Kepler propagation. Outputs sub-satellite point + altitude.
void tle_propagate(const TleElem& e, time_t now_utc,
                   double& lat_deg, double& lon_deg, double& alt_km);
