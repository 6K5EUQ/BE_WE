#include "sat_tle.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern "C" time_t timegm(struct tm*);  // glibc

static time_t doy_frac_to_utc(int yy, double doy_frac) {
    int year = (yy < 57) ? 2000 + yy : 1900 + yy;
    int doy_int = (int)doy_frac;
    double frac = doy_frac - (double)doy_int;
    struct tm tmv;
    memset(&tmv, 0, sizeof tmv);
    tmv.tm_year = year - 1900;
    tmv.tm_mon  = 0;
    tmv.tm_mday = doy_int;       // tm_mday > 31 is normalized by timegm
    time_t t = timegm(&tmv);
    return t + (time_t)(frac * 86400.0);
}

static double gmst_rad(time_t utc) {
    // J2000.0 = 2000-01-01 12:00:00 UTC = unix 946728000
    double d = ((double)utc - 946728000.0) / 86400.0;
    double gmst_deg = 280.46061837 + 360.98564736629 * d;
    gmst_deg = fmod(gmst_deg, 360.0);
    if (gmst_deg < 0) gmst_deg += 360.0;
    return gmst_deg * M_PI / 180.0;
}

static double parse_field(const char* line, int len, int start, int width) {
    if (start + width > len) return 0.0;
    char buf[32];
    if (width >= (int)sizeof buf) width = (int)sizeof buf - 1;
    memcpy(buf, line + start, width);
    buf[width] = 0;
    return atof(buf);
}

static int parse_field_int(const char* line, int len, int start, int width) {
    if (start + width > len) return 0;
    char buf[32];
    if (width >= (int)sizeof buf) width = (int)sizeof buf - 1;
    memcpy(buf, line + start, width);
    buf[width] = 0;
    return atoi(buf);
}

bool tle_load(const std::string& path, std::vector<TleElem>& out) {
    out.clear();
    FILE* fp = fopen(path.c_str(), "r");
    if (!fp) return false;
    char l0[128], l1[128], l2[128];
    while (fgets(l0, sizeof l0, fp)) {
        if (l0[0] == '#' || l0[0] == '\n' || l0[0] == '\r') continue;
        if (!fgets(l1, sizeof l1, fp)) break;
        if (!fgets(l2, sizeof l2, fp)) break;
        if (l1[0] != '1' || l2[0] != '2') continue;
        int len1 = (int)strlen(l1);
        int len2 = (int)strlen(l2);
        if (len1 < 64 || len2 < 64) continue;

        TleElem e;
        std::string nm(l0);
        while (!nm.empty() && (nm.back()=='\n'||nm.back()=='\r'||nm.back()==' '||nm.back()=='\t'))
            nm.pop_back();
        e.name = nm;

        e.catalog_num    = parse_field_int(l1, len1, 2, 5);
        int yy           = parse_field_int(l1, len1, 18, 2);
        double doy       = parse_field(l1, len1, 20, 12);
        e.epoch_utc      = doy_frac_to_utc(yy, doy);

        e.inclination_deg          = parse_field(l2, len2, 8, 8);
        e.raan_deg                 = parse_field(l2, len2, 17, 8);
        e.arg_perigee_deg          = parse_field(l2, len2, 34, 8);
        e.mean_anomaly_deg         = parse_field(l2, len2, 43, 8);
        e.mean_motion_revs_per_day = parse_field(l2, len2, 52, 11);

        if (e.mean_motion_revs_per_day <= 0.0) continue;
        const double mu = 398600.4418;
        double n_rps = e.mean_motion_revs_per_day * 2.0 * M_PI / 86400.0;
        e.semi_major_km = pow(mu / (n_rps * n_rps), 1.0/3.0);
        out.push_back(e);
    }
    fclose(fp);
    return true;
}

void tle_propagate(const TleElem& e, time_t now_utc,
                   double& lat_deg, double& lon_deg, double& alt_km) {
    double minutes = ((double)now_utc - (double)e.epoch_utc) / 60.0;
    double M_deg = e.mean_anomaly_deg + e.mean_motion_revs_per_day * 0.25 * minutes;
    M_deg = fmod(M_deg, 360.0);
    if (M_deg < 0) M_deg += 360.0;
    double M = M_deg * M_PI / 180.0;
    double a = e.semi_major_km;

    // Orbit plane (e=0: r=a, true anomaly = mean anomaly)
    double xp = a * cos(M);
    double yp = a * sin(M);

    // Rotate by argp around Z
    double argp = e.arg_perigee_deg * M_PI / 180.0;
    double ca = cos(argp), sa = sin(argp);
    double x1 = ca*xp - sa*yp;
    double y1 = sa*xp + ca*yp;
    double z1 = 0.0;

    // Rotate by inclination around X
    double inc = e.inclination_deg * M_PI / 180.0;
    double ci = cos(inc), si = sin(inc);
    double x2 = x1;
    double y2 = ci*y1 - si*z1;
    double z2 = si*y1 + ci*z1;

    // Rotate by RAAN around Z → ECI
    double raan = e.raan_deg * M_PI / 180.0;
    double cr = cos(raan), sr = sin(raan);
    double xeci = cr*x2 - sr*y2;
    double yeci = sr*x2 + cr*y2;
    double zeci = z2;

    // ECI → ECEF: rotate by -GMST around Z
    double th = gmst_rad(now_utc);
    double cT = cos(th), sT = sin(th);
    double xe =  cT*xeci + sT*yeci;
    double ye = -sT*xeci + cT*yeci;
    double ze = zeci;

    double r = sqrt(xe*xe + ye*ye + ze*ze);
    lat_deg = asin(ze / r) * 180.0 / M_PI;
    lon_deg = atan2(ye, xe) * 180.0 / M_PI;
    alt_km  = r - 6378.137;
}
