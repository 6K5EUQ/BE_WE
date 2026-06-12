#pragma once
// ── ACARS message record + registration/callsign enrichment ───────────────
#include <cstdint>
#include <cstring>
#include "acars_db.hpp"

struct AcarsMsg {
    int64_t t_ms=0;        // wall time (caller stamps)
    int     ch=0;
    float   freq=0.f;      // MHz (caller stamps)
    bool    crc_ok=false;
    bool    downlink=false; // 항공기→지상 (편명/MSN 구조 존재 시)
    char    mode=0;
    char    block=0;
    char    ack=0;
    char    station[16]={};// 복조한 기지 표시명 (DGS-2 / LOCAL)
    char    reg[10]={};    // aircraft registration (tail)
    char    flight[8]={};  // flight id / callsign (best-effort)
    char    label[3]={};
    char    text[256]={};
};

// ── registration prefix → country (longest-prefix match) ──────────────────
inline const char* acars_country(const char* reg){
    struct P{ const char* pfx; const char* cc; };
    static const P T[]={
        {"HL","Korea"},{"JA","Japan"},{"B","China"},{"N","USA"},{"C","Canada"},
        {"VH","Australia"},{"ZK","NewZealand"},{"VT","India"},{"9V","Singapore"},
        {"HS","Thailand"},{"PK","Indonesia"},{"RP","Philippines"},{"9M","Malaysia"},
        {"VN","Vietnam"},{"XU","Cambodia"},{"A6","UAE"},{"A7","Qatar"},{"HZ","SaudiArabia"},
        {"9K","Kuwait"},{"4X","Israel"},{"TC","Turkey"},{"EP","Iran"},{"AP","Pakistan"},
        {"4R","SriLanka"},{"S2","Bangladesh"},{"G","UK"},{"D","Germany"},{"F","France"},
        {"I","Italy"},{"EC","Spain"},{"CS","Portugal"},{"PH","Netherlands"},{"OO","Belgium"},
        {"OE","Austria"},{"HB","Switzerland"},{"SE","Sweden"},{"LN","Norway"},{"OY","Denmark"},
        {"EI","Ireland"},{"SP","Poland"},{"OK","Czechia"},{"SX","Greece"},{"RA","Russia"},
        {"UR","Ukraine"},{"ZS","SouthAfrica"},{"5Y","Kenya"},{"ET","Ethiopia"},{"SU","Egypt"},
        {"XA","Mexico"},{"PP","Brazil"},{"PR","Brazil"},{"PT","Brazil"},{"LV","Argentina"},{"CC","Chile"},
    };
    if(!reg||!reg[0]) return "";
    for(int len=3; len>=1; len--){
        for(auto& p : T){ if((int)strlen(p.pfx)==len && strncmp(reg,p.pfx,len)==0) return p.cc; }
    }
    return "";
}

// ── flight callsign prefix → airline (IATA 2-char / ICAO 3-char) ───────────
inline const char* acars_airline(const char* fl){
    struct P{ const char* pfx; const char* name; };
    static const P T[]={
        {"KAL","Korean Air"},{"KE","Korean Air"},{"AAR","Asiana"},{"OZ","Asiana"},
        {"JJA","Jeju Air"},{"7C","Jeju Air"},{"JNA","Jin Air"},{"LJ","Jin Air"},
        {"TWB","Tway Air"},{"TW","Tway Air"},{"ABL","Air Busan"},{"BX","Air Busan"},
        {"ASV","Air Seoul"},{"RS","Air Seoul"},{"ESR","Eastar Jet"},{"ZE","Eastar Jet"},
        {"JAL","Japan Airlines"},{"JL","Japan Airlines"},{"ANA","All Nippon"},{"NH","All Nippon"},
        {"APJ","Peach"},{"MM","Peach"},{"SKY","Skymark"},{"BC","Skymark"},
        {"CCA","Air China"},{"CA","Air China"},{"CES","China Eastern"},{"MU","China Eastern"},
        {"CSN","China Southern"},{"CZ","China Southern"},{"CXA","XiamenAir"},{"MF","XiamenAir"},
        {"CPA","Cathay Pacific"},{"CX","Cathay Pacific"},{"HDA","Cathay"},{"UO","HK Express"},
        {"SIA","Singapore Air"},{"SQ","Singapore Air"},{"THA","Thai Airways"},{"TG","Thai Airways"},
        {"MAS","Malaysia"},{"MH","Malaysia"},{"GIA","Garuda"},{"GA","Garuda"},
        {"PAL","Philippine"},{"PR","Philippine"},{"HVN","Vietnam Air"},{"VN","Vietnam Air"},
        {"AIC","Air India"},{"AI","Air India"},{"UAE","Emirates"},{"EK","Emirates"},
        {"QTR","Qatar"},{"QR","Qatar"},{"ETD","Etihad"},{"EY","Etihad"},
        {"QFA","Qantas"},{"QF","Qantas"},{"ANZ","Air NZ"},{"NZ","Air NZ"},
        {"AAL","American"},{"AA","American"},{"DAL","Delta"},{"DL","Delta"},
        {"UAL","United"},{"UA","United"},{"FDX","FedEx"},{"FX","FedEx"},{"UPS","UPS"},{"5X","UPS"},
        {"BAW","British Airways"},{"BA","British Airways"},{"DLH","Lufthansa"},{"LH","Lufthansa"},
        {"AFR","Air France"},{"AF","Air France"},{"KLM","KLM"},{"KL","KLM"},
    };
    if(!fl||!fl[0]) return "";
    for(int len=3; len>=2; len--){
        for(auto& p : T){ if((int)strlen(p.pfx)==len && strncmp(fl,p.pfx,len)==0) return p.name; }
    }
    return "";
}

// 항공사: 등록부호 DB(operator) 우선, 없으면 콜사인 프리픽스
inline const char* acars_airline_of(const AcarsMsg& m){
    const char* op = acars_db_operator(m.reg);
    if(op && op[0]) return op;
    return acars_airline(m.flight);
}
