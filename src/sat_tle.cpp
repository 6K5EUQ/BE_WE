#include "sat_tle.hpp"
#include "MathTimeLib.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Parse a fixed-width decimal field from a TLE line (1-indexed columns → 0-indexed here)
static double pf(const char* line, int start, int width) {
    char buf[32];
    if (width >= (int)sizeof buf) width = (int)sizeof buf - 1;
    memcpy(buf, line + start, width);
    buf[width] = '\0';
    return atof(buf);
}
static int pfi(const char* line, int start, int width) {
    char buf[32];
    if (width >= (int)sizeof buf) width = (int)sizeof buf - 1;
    memcpy(buf, line + start, width);
    buf[width] = '\0';
    return atoi(buf);
}

// TLE decimal-point-implied exponent field: " 12345-3" → 0.12345e-3
static double tle_exp_field(const char* line, int start, int width) {
    char buf[32];
    if (width >= (int)sizeof buf) width = (int)sizeof buf - 1;
    memcpy(buf, line + start, width);
    buf[width] = '\0';
    char* p = buf;
    while (*p == ' ') p++;                // skip leading spaces
    int sign = 1;
    if (*p == '+') { p++; } else if (*p == '-') { sign = -1; p++; }
    char mant[16] = "0.";
    int mi = 2;
    while (*p && *p >= '0' && *p <= '9') mant[mi++] = *p++;
    mant[mi] = '\0';
    int exp = atoi(p);                    // p now at sign of exponent
    return sign * atof(mant) * pow(10.0, (double)exp);
}

bool tle_load(const std::string& path, std::vector<TleElem>& out) {
    out.clear();
    FILE* fp = fopen(path.c_str(), "r");
    if (!fp) return false;

    char l0[256], l1[256], l2[256];
    while (fgets(l0, sizeof l0, fp)) {
        // skip blank/comment lines
        if (l0[0] == '#' || l0[0] == '\n' || l0[0] == '\r') continue;
        if (!fgets(l1, sizeof l1, fp)) break;
        if (!fgets(l2, sizeof l2, fp)) break;
        if (l1[0] != '1' || l2[0] != '2') continue;
        if ((int)strlen(l1) < 69 || (int)strlen(l2) < 69) continue;

        // ── name ──────────────────────────────────────────────────────────
        TleElem e;
        {
            std::string nm(l0);
            while (!nm.empty() && (nm.back()=='\n'||nm.back()=='\r'||
                                   nm.back()==' ' ||nm.back()=='\t'))
                nm.pop_back();
            e.name = nm;
        }

        e.catalog_num = pfi(l1, 2, 5);

        // ── epoch → Julian date ──────────────────────────────────────────
        int    ep_yy  = pfi(l1, 18, 2);
        double ep_doy = pf (l1, 20, 12);
        int    year   = (ep_yy < 57) ? 2000 + ep_yy : 1900 + ep_yy;
        int    mon, day, hr, minute; double sec;
        MathTimeLib::days2mdhms(year, ep_doy, mon, day, hr, minute, sec);
        double jd, jdF;
        MathTimeLib::jday(year, mon, day, hr, minute, sec, jd, jdF);
        double epoch_days = (jd + jdF) - 2433281.5;  // days since 1949-12-31 00:00 UT

        // ── TLE fields. xpdotp converts rev/day ↔ rad/min ───────────────
        const double xpdotp = 1440.0 / (2.0 * M_PI);
        double bstar   = tle_exp_field(l1, 53, 8);
        double ndot    = pf(l1, 33, 10) / (xpdotp * 1440.0);
        double nddot   = tle_exp_field(l1, 44, 8) / (xpdotp * 1440.0 * 1440.0);
        double inclo   = pf(l2,  8, 8) * M_PI / 180.0;
        double nodeo   = pf(l2, 17, 8) * M_PI / 180.0;
        double ecco    = pf(l2, 26, 7) * 1e-7;
        double argpo   = pf(l2, 34, 8) * M_PI / 180.0;
        double mo      = pf(l2, 43, 8) * M_PI / 180.0;
        double no_koz  = pf(l2, 52, 11) / xpdotp;     // rev/day → rad/min

        // satnum: 5-char string (left-padded with zeros)
        char satn[10];
        snprintf(satn, sizeof satn, "%05d", e.catalog_num);

        // sgp4init does NOT populate jdsatepoch — caller must set it.
        memset(&e.satrec, 0, sizeof(e.satrec));
        e.satrec.jdsatepoch  = jd;
        e.satrec.jdsatepochF = jdF;
        e.satrec.classification = 'U';

        SGP4Funcs::sgp4init(wgs72, 'i', satn, epoch_days,
                            bstar, ndot, nddot, ecco, argpo,
                            inclo, mo, no_koz, nodeo, e.satrec);

        // semi_major for LEO detection (µ=398600.4418, n_rad/s)
        if (no_koz > 0.0) {
            double n_rps = no_koz / 60.0;
            e.semi_major_km = pow(398600.4418 / (n_rps * n_rps), 1.0/3.0);
        }

        if (e.satrec.error == 0)
            out.push_back(std::move(e));
    }
    fclose(fp);
    return true;
}

void tle_propagate(const TleElem& e, time_t now_utc,
                   double& lat_deg, double& lon_deg, double& alt_km) {
    // tsince = minutes since TLE epoch
    // jdsatepoch + jdsatepochF = epoch in Julian days
    double jd_now = 2440587.5 + (double)now_utc / 86400.0;
    double tsince  = (jd_now - (e.satrec.jdsatepoch + e.satrec.jdsatepochF)) * 1440.0;

    elsetrec satrec = e.satrec;   // SGP4 mutates satrec.t — copy per call
    double r[3], v[3];
    if (!SGP4Funcs::sgp4(satrec, tsince, r, v)) {
        lat_deg = lon_deg = alt_km = 0.0;
        return;
    }

    // ECI (km) → geodetic lat/lon/alt
    // GMST via SGP4's own gstime
    double jdF_now = (jd_now - (long)jd_now);
    double gst = SGP4Funcs::gstime_SGP4(jd_now);

    double xe =  r[0]*cos(gst) + r[1]*sin(gst);
    double ye = -r[0]*sin(gst) + r[1]*cos(gst);
    double ze =  r[2];

    double p   = sqrt(xe*xe + ye*ye);
    const double a_e = 6378.137, f = 1.0/298.257223563;
    double e2  = 2*f - f*f;
    double lat = atan2(ze, p);
    for (int i = 0; i < 5; i++) {
        double N = a_e / sqrt(1.0 - e2*sin(lat)*sin(lat));
        lat = atan2(ze + e2*N*sin(lat), p);
    }
    double N   = a_e / sqrt(1.0 - e2*sin(lat)*sin(lat));
    alt_km     = (p / cos(lat)) - N;
    lat_deg    = lat * 180.0 / M_PI;
    lon_deg    = atan2(ye, xe) * 180.0 / M_PI;
}
