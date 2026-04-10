#pragma once
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>

// AIS decoder — calls ~/AIS_Decorder/ais_decoder.py via popen
// Caches result per bit hash to avoid repeated subprocess calls

namespace ais_decode {

struct Result {
    bool ok = false;
    std::string error;
    std::vector<std::string> lines;  // decoded output lines
};

// Convert EID bits to binary string (space-separated 8-bit groups)
static inline std::string bits_to_binstr(const std::vector<uint8_t>& bits, int offset){
    std::string r;
    r.reserve((bits.size()-offset)*2);
    for(int i=offset;i<(int)bits.size();i++){
        r += (bits[i] ? '1' : '0');
        if((i-offset+1)%8==0 && i+1<(int)bits.size()) r += ' ';
    }
    return r;
}

// Hash bits for cache invalidation
static inline uint64_t bits_hash(const std::vector<uint8_t>& bits, int offset){
    uint64_t h = 14695981039346656037ULL;
    for(int i=offset;i<(int)bits.size();i++){
        h ^= bits[i];
        h *= 1099511628211ULL;
    }
    h ^= (uint64_t)(bits.size()-offset);
    return h;
}

static inline Result decode(const std::vector<uint8_t>& bits, int offset = 0){
    // Cache: avoid re-running Python on every frame
    static uint64_t cached_hash = 0;
    static Result   cached_result;

    uint64_t h = bits_hash(bits, offset);
    if(h == cached_hash && cached_hash != 0) return cached_result;

    int n = (int)bits.size() - offset;
    if(n < 16){
        cached_hash = h;
        cached_result = {false, "Too few bits", {}};
        return cached_result;
    }

    // Build binary string (no spaces, compact)
    std::string binstr;
    binstr.reserve(n);
    for(int i=offset;i<(int)bits.size();i++) binstr += (bits[i]?'1':'0');

    // Find decoder path: repo 내부 decoder/AIS/ 우선, fallback ~/AIS_Decorder/
    std::string decoder_path;
    {
        // 실행 파일 기준 상대 경로
        char exe[512]={}; ssize_t rl = readlink("/proc/self/exe", exe, sizeof(exe)-1);
        if(rl > 0){
            std::string ep(exe, rl);
            auto sl = ep.rfind('/');
            if(sl != std::string::npos){
                std::string dir = ep.substr(0, sl);
                // build/ 또는 build_headless/ 에서 실행 → 상위의 decoder/AIS/
                std::string try1 = dir + "/../decoder/AIS/ais_decoder.py";
                if(access(try1.c_str(), F_OK)==0) decoder_path = try1;
            }
        }
        if(decoder_path.empty()){
            const char* home = getenv("HOME");
            decoder_path = home ? std::string(home) + "/AIS_Decorder/ais_decoder.py"
                                : "/home/dsa/AIS_Decorder/ais_decoder.py";
        }
    }

    // Build command — pass bitstring as argument
    std::string cmd = "python3 \"" + decoder_path + "\" --no-crc \"" + binstr + "\" 2>/dev/null";

    FILE* fp = popen(cmd.c_str(), "r");
    if(!fp){
        cached_hash = h;
        cached_result = {false, "Failed to run decoder", {}};
        return cached_result;
    }

    Result r;
    char buf[512];
    while(fgets(buf, sizeof(buf), fp)){
        std::string line(buf);
        while(!line.empty() && (line.back()=='\n'||line.back()=='\r')) line.pop_back();
        if(!line.empty()) r.lines.push_back(std::move(line));
    }
    int rc = pclose(fp);

    r.ok = !r.lines.empty() && rc == 0;
    if(!r.ok && r.lines.empty()) r.error = "No valid AIS frame found";

    cached_hash = h;
    cached_result = r;
    return cached_result;
}

} // namespace ais_decode
