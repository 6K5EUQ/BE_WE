#pragma once
// Minimal JSON scanner for fixed-schema configs.
// Whitespace + commas are skipped. No strict syntax validation.
// Extracted from previously-duplicated definitions in
// host_band_plan.cpp, host_band_categories.cpp, central/central_server.cpp.

#include <string>
#include <cstdlib>

struct JScan {
    const char* p;
    const char* end;
    void skip_ws(){ while(p<end && (*p==' '||*p=='\n'||*p=='\r'||*p=='\t'||*p==',')) p++; }
    bool consume(char c){ skip_ws(); if(p<end && *p==c){ p++; return true; } return false; }
    bool peek(char c){ skip_ws(); return p<end && *p==c; }
    bool read_string(std::string& out){
        skip_ws();
        if(p>=end || *p != '"') return false;
        p++; out.clear();
        while(p<end && *p != '"'){
            if(*p == '\\' && p+1<end){ p++;
                if(*p=='n') out+='\n';
                else if(*p=='r') out+='\r';
                else out += *p;
                p++;
            } else { out += *p++; }
        }
        if(p<end && *p=='"') p++;
        return true;
    }
    bool read_number(double& v){
        skip_ws();
        const char* s = p;
        while(p<end && (*p=='-'||*p=='+'||*p=='.'||(*p>='0'&&*p<='9')||*p=='e'||*p=='E')) p++;
        if(s==p) return false;
        std::string tmp(s, p-s);
        v = atof(tmp.c_str());
        return true;
    }
    bool read_key(std::string& k){
        if(!read_string(k)) return false;
        skip_ws();
        if(p<end && *p==':'){ p++; return true; }
        return false;
    }
};
