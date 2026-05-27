#pragma once
#include "SGP4.h"
#include <string>
#include <vector>
#include <ctime>

struct TleElem {
    std::string name;
    int         catalog_num     = 0;
    double      semi_major_km   = 0.0;  // derived: visualisation / LEO filter
    bool        is_starlink     = false;
    bool        is_leo          = false;
    elsetrec    satrec;                 // SGP4 initialised record
};

bool tle_load(const std::string& path, std::vector<TleElem>& out);

void tle_propagate(const TleElem& e, time_t now_utc,
                   double& lat_deg, double& lon_deg, double& alt_km);
