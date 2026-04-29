#include "info_parse.hpp"
#include <cctype>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sstream>
#include <strings.h>

namespace InfoParse {

std::string trim(const std::string& s){
    size_t a = 0, b = s.size();
    while(a < b && std::isspace((unsigned char)s[a])) a++;
    while(b > a && std::isspace((unsigned char)s[b-1])) b--;
    return s.substr(a, b-a);
}

std::string lower(const std::string& s){
    std::string out = s;
    for(auto& c : out) c = (char)std::tolower((unsigned char)c);
    return out;
}

std::map<std::string, std::string> parse(const std::string& text){
    std::map<std::string, std::string> m;
    std::istringstream iss(text);
    std::string line;
    while(std::getline(iss, line)){
        if(!line.empty() && line.back() == '\r') line.pop_back();
        if(line.empty()) continue;
        size_t colon = line.find(':');
        if(colon == std::string::npos) continue;
        std::string key = trim(line.substr(0, colon));
        std::string val = trim(line.substr(colon+1));
        if(key.empty()) continue;
        m[key] = val;
    }
    return m;
}

bool extract_freq_mhz(const std::string& v, float& out){
    if(v.empty()) return false;
    double num = 0;
    char suffix[16] = {0};
    int n = sscanf(v.c_str(), " %lf %15s", &num, suffix);
    if(n < 1) return false;
    std::string s = lower(suffix);
    double mult = 1.0; // MHz default
    if(s.find("ghz") != std::string::npos) mult = 1000.0;
    else if(s.find("khz") != std::string::npos) mult = 0.001;
    else if(s.find("mhz") != std::string::npos) mult = 1.0;
    else if(s.find("hz") != std::string::npos) mult = 1e-6;
    out = (float)(num * mult);
    return true;
}

bool extract_bw_khz(const std::string& v, float& out){
    if(v.empty()) return false;
    double num = 0;
    char suffix[16] = {0};
    int n = sscanf(v.c_str(), " %lf %15s", &num, suffix);
    if(n < 1) return false;
    std::string s = lower(suffix);
    double mult = 1.0; // kHz default
    if(s.find("mhz") != std::string::npos) mult = 1000.0;
    else if(s.find("khz") != std::string::npos) mult = 1.0;
    else if(s.find("hz") != std::string::npos) mult = 0.001;
    out = (float)(num * mult);
    return true;
}

bool extract_duration_s(const std::string& v, uint32_t& out){
    if(v.empty()) return false;
    int h=0, m=0, s=0;
    if(sscanf(v.c_str(), " %d:%d:%d", &h, &m, &s) == 3){
        out = (uint32_t)(h*3600 + m*60 + s);
        return true;
    }
    if(sscanf(v.c_str(), " %d:%d", &m, &s) == 2){
        out = (uint32_t)(m*60 + s);
        return true;
    }
    double sec = 0;
    char suffix[16] = {0};
    int n = sscanf(v.c_str(), " %lf %15s", &sec, suffix);
    if(n < 1) return false;
    std::string sx = lower(suffix);
    double mult = 1.0;
    if(sx.find("min") != std::string::npos || sx == "m") mult = 60.0;
    else if(sx == "h" || sx.find("hour") != std::string::npos) mult = 3600.0;
    out = (uint32_t)(sec * mult + 0.5);
    return true;
}

bool extract_start_utc(const std::string& day, const std::string& time_str, int64_t& out){
    out = 0;
    if(day.empty() || time_str.empty()) return false;
    char mon[8] = {0};
    int dd=0, yy=0;
    if(sscanf(day.c_str(), " %3s %d , %d", mon, &dd, &yy) != 3 &&
       sscanf(day.c_str(), " %3s %d %d", mon, &dd, &yy) != 3){
        return false;
    }
    static const char* months[] = {"Jan","Feb","Mar","Apr","May","Jun",
                                   "Jul","Aug","Sep","Oct","Nov","Dec"};
    int mon_idx = -1;
    for(int i=0; i<12; i++){
        if(strncasecmp(mon, months[i], 3) == 0){ mon_idx = i; break; }
    }
    if(mon_idx < 0) return false;
    int hh=0, mm=0, ss=0;
    if(sscanf(time_str.c_str(), " %d:%d:%d", &hh, &mm, &ss) != 3) return false;

    struct tm tm{};
    tm.tm_year = yy - 1900;
    tm.tm_mon  = mon_idx;
    tm.tm_mday = dd;
    tm.tm_hour = hh;
    tm.tm_min  = mm;
    tm.tm_sec  = ss;

    // Parse UTC offset suffix "(UTC+9)" / "(UTC-3)" — int hours only for now.
    int utc_off_h = 0;
    auto p = time_str.find("UTC");
    if(p != std::string::npos){
        const char* sp = time_str.c_str() + p + 3;
        int off_h = 0;
        if(sscanf(sp, "%d", &off_h) == 1) utc_off_h = off_h;
    }

    time_t t = timegm(&tm);
    if(t == (time_t)-1) return false;
    out = (int64_t)t - (int64_t)utc_off_h * 3600;
    return true;
}

} // namespace InfoParse
