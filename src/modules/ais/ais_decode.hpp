#pragma once
// ── AIS HDLC/NRZI 프레임 디코더 (ITU-R M.1371) ─────────────────────────────
// 입력 = NRZI 디코드된 비트열(0/1) 한 비트씩. GMSK 복조/비트동기는 워커(ais_decode.cpp).
// 프레임 구조: 프리앰블(0101..) → 플래그 0x7E → 데이터 → FCS(CRC-16) → 플래그 0x7E.
//   - 비트스터핑: 연속 1 다섯 개 뒤 삽입된 0 제거.
//   - HDLC 옥텟은 LSB-first 전송 → buffer(수신순) → rbuffer(옥텟 MSB-first 재배열).
//   - FCS: CRC-16-CCITT reflected(0x8408), init 0xFFFF; 프레임 전체 잔차 == 0x0F47 면 OK.
// 알고리즘 골격은 GNU AIS(protodec, GPLv2, R.Undheim/H.Hannikainen)에서 검증된 구조를 차용.
#include <cstdint>
#include <cstring>
#include <functional>
#include "ais_meta.hpp"

class AisDecoder {
public:
    std::function<void(const AisRecord&)> on_record;
    std::function<void(bool)>             on_gate;   // payload(ST_DATA) 구간 알림 (RF 지문 게이팅). true=시작 false=종결

    void reset_all(){ if(on_gate) on_gate(false); reset_frame(); state=ST_SKURR; nskurr=0; npreamble=0; last=0; }

    // NRZI 디코드된 데이터 비트 1개 투입 (0/1)
    void feed_bit(uint8_t in){
        switch(state){
        case ST_DATA:
            if(bitstuff){
                if(in==1){ state=ST_STOPSIGN; bitstuff=0; }   // 다섯 1 뒤 1 → 플래그 시작
                else     { bitstuff=0; }                       // 스터핑된 0 → 폐기
            } else {
                if(in==last && in==1){
                    if(++antallenner==4){ bitstuff=1; antallenner=0; }  // 연속 1 다섯 개
                } else antallenner=0;
                if(bufferpos<BUFSZ) buffer[bufferpos++]=in;
                if(bufferpos>=BUFSZ-1) reset_all();
            }
            break;

        case ST_SKURR:                                         // 잡음/탐색: 프리앰블(교번) 누적
            if(in!=last) antallpreamble++; else antallpreamble=0;
            if(antallpreamble>14 && in==0){ state=ST_PREAMBLE; nskurr=0; antallpreamble=0; }
            nskurr++;
            break;

        case ST_PREAMBLE:                                      // 교번 끝 → 플래그의 연속 1 검출
            if(in!=last && nstartsign==0){ antallpreamble++; }
            else {
                if(in==1){
                    if(nstartsign==0){ nstartsign=3; last=in; }
                    else if(nstartsign==5){ nstartsign++; npreamble=0; antallpreamble=0; state=ST_STARTSIGN; }
                    else nstartsign++;
                } else {
                    if(nstartsign==0) nstartsign=1; else reset_all();
                }
            }
            npreamble++;
            break;

        case ST_STARTSIGN:                                     // 플래그 닫는 0 → 데이터 시작
            if(nstartsign>=7){
                if(in==0){ state=ST_DATA; nstartsign=0; antallenner=0;
                           memset(buffer,0,sizeof(buffer)); bufferpos=0;
                           if(on_gate) on_gate(true); }   // 페이로드 시작 → RF 지문 누산 개시
                else reset_all();
            } else if(in==0) reset_all();
            nstartsign++;
            break;

        case ST_STOPSIGN: {                                    // 닫는 플래그 → 프레임 종결
            int bits = bufferpos - 6 - 16;                     // 데이터 비트 수(플래그6 + FCS16 제거)
            if(in==0 && bits>0 && calc_crc(bits)) emit(bits);
            reset_all();
            break;
        }
        }
        last=in;
    }

private:
    static constexpr int ST_SKURR=1, ST_PREAMBLE=2, ST_STARTSIGN=3, ST_DATA=4, ST_STOPSIGN=5;
    static constexpr int BUFSZ=520;

    int     state=ST_SKURR;
    uint8_t buffer[BUFSZ]={};    // 수신순 비트(옥텟 LSB-first)
    uint8_t rbuffer[BUFSZ]={};   // 옥텟 MSB-first 재배열 비트 (henten 입력)
    int     bufferpos=0;
    uint8_t last=0;
    int     antallpreamble=0, nstartsign=0, antallenner=0, bitstuff=0, nskurr=0, npreamble=0;

    void reset_frame(){ bufferpos=0; antallenner=0; bitstuff=0; nstartsign=0; antallpreamble=0; }

    // ── CRC-16 (HDLC FCS): reflected poly 0x8408, init 0xFFFF, 결과 반전 ──
    static uint16_t sdlc_crc(const uint8_t* data, int len){
        uint16_t crc=0xFFFF;
        while(len--)
            for(unsigned c=0x100+*data++; c>1; c>>=1)
                crc = ((crc^c)&1) ? (crc>>1)^0x8408 : (crc>>1);
        return ~crc;
    }
    // 데이터 bits + FCS(2바이트) 를 옥텟으로 묶어 CRC 확인. OK 면 rbuffer 채움.
    bool calc_crc(int length_bits){
        int length_bytes = length_bits/8;
        int buflen = length_bytes + 2;                  // + FCS 2바이트
        if(buflen<=0 || (buflen*8) > BUFSZ) return false;
        uint8_t buf[BUFSZ/8 + 4];
        for(int j=0;j<buflen;j++){
            uint8_t tmp=0;
            for(int i=0;i<8;i++) tmp |= (uint8_t)(buffer[i+8*j] << i);  // LSB-first → 옥텟
            buf[j]=tmp;
        }
        if(sdlc_crc(buf, buflen) != 0x0F47) return false;
        memset(rbuffer,0,sizeof(rbuffer));
        for(int j=0;j<length_bytes;j++)
            for(int i=0;i<8;i++) rbuffer[j*8+i] = (buf[j]>>(7-i))&1;     // 옥텟 MSB-first
        return true;
    }

    // rbuffer 에서 size 비트 MSB-first 부호없는 정수
    unsigned long henten(int from, int size) const {
        unsigned long t=0;
        for(int i=0;i<size;i++) t |= (unsigned long)rbuffer[from+i] << (size-1-i);
        return t;
    }
    // size 비트 부호있는(2의 보수) 정수
    static long sext(unsigned long v, int bits){
        return (v & (1UL<<(bits-1))) ? (long)(v - (1UL<<bits)) : (long)v;
    }
    void sixstr(int from, int chars, char* out){
        int o=0;
        for(int k=0;k<chars;k++){ out[o++]=ais_sixbit_ascii((unsigned)henten(from+6*k,6)); }
        out[o]=0;
        for(int i=o-1;i>=0;i--){ if(out[i]==' '||out[i]==0) out[i]=0; else break; }  // 뒤 공백 트림
    }

    void emit(int bits){
        AisRecord m{};
        m.crc_ok = true;
        m.msg_type = (int)henten(0,6);
        if(m.msg_type<1 || m.msg_type>27) return;
        m.mmsi = (uint32_t)henten(8,30);

        auto set_pos = [&](int lon_off, int lat_off){
            unsigned long rlon=henten(lon_off,28), rlat=henten(lat_off,27);
            if(rlon==0x6791AC0UL || rlat==0x3412140UL) return;       // n/a 센티넬
            m.lon = sext(rlon,28)/600000.0;
            m.lat = sext(rlat,27)/600000.0;
            if(m.lon>=-180.0&&m.lon<=180.0&&m.lat>=-90.0&&m.lat<=90.0) m.has_pos=true;
        };
        auto set_sog = [&](int off){ unsigned long s=henten(off,10); if(s!=1023) m.sog=(float)s/10.f; };
        auto set_cog = [&](int off){ unsigned long c=henten(off,12); if(c!=3600&&c<3600) m.cog=(float)c/10.f; };
        auto set_hdg = [&](int off){ m.heading=(int)henten(off,9); };

        switch(m.msg_type){
        case 1: case 2: case 3:
            if(bits<144) break;
            m.nav_status=(int)henten(38,4);
            set_sog(50); set_pos(61,89); set_cog(116); set_hdg(128);
            break;
        case 4:                                                    // 기지국 보고 (위치)
            if(bits<144) break;
            set_pos(79,107);
            break;
        case 5:                                                    // 정적/항해 데이터
            if(bits<420) break;
            m.imo=(uint32_t)henten(40,30);                         // IMO 번호 (0=n/a)
            sixstr(70,7,m.callsign);
            sixstr(112,20,m.name);
            m.ship_type=(int)henten(232,8);
            m.eta_mon =(uint8_t)henten(274,4);                     // ETA 월(0=n/a)/일/시(24=n/a)/분(60=n/a)
            m.eta_day =(uint8_t)henten(278,5);
            m.eta_hour=(uint8_t)henten(283,5);
            m.eta_min =(uint8_t)henten(288,6);
            { unsigned long d=henten(294,8); if(d) m.draught=(float)d/10.f; }  // 흘수 0.1m 단위
            sixstr(302,20,m.dest);                                 // 목적지
            break;
        case 18:                                                   // Class B 위치
            if(bits<144) break;
            set_sog(46); set_pos(57,85); set_cog(112); set_hdg(124);
            break;
        case 19:                                                   // Class B 확장 위치+정적
            if(bits<306) break;
            set_sog(46); set_pos(57,85); set_cog(112); set_hdg(124);
            sixstr(143,20,m.name);
            m.ship_type=(int)henten(263,8);
            break;
        case 24: {                                                 // Class B 정적 (Part A/B)
            int part=(int)henten(38,2);
            if(part==0){ if(bits>=160) sixstr(40,20,m.name); }
            else if(part==1){ if(bits>=162){ m.ship_type=(int)henten(40,8); sixstr(90,7,m.callsign); } }
            break;
        }
        default: break;
        }
        if(on_record) on_record(m);
    }
};
