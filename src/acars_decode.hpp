#pragma once
// ── ACARS (ARINC 618 Mode-A) streaming decoder ────────────────────────────
// Input: real AM-demodulated audio at a known rate (e.g. 48000 Hz).
// Coherent MSK demodulation (the only thing that actually works for ACARS):
//   1800 Hz VCO down-mix → half-cosine matched filter → alternating I/Q-axis
//   bit decision (MSK = OQPSK) → carrier/timing PLL → byte assembly →
//   SYN/~SYN/SOH sync (polarity auto-resolved) → 7-bit + odd parity →
//   CRC-16/X-25 (BCS) → on_msg() callback.
// MSK is NOT plain 2-FSK: data rides alternately on the real/imag axis with a
// 2-bit sign rotation, so a naive FM discriminator yields structured-but-wrong
// bits. This implements the standard coherent MSK receiver.
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <functional>

class AcarsDecoder {
public:
    std::function<void(const char*)> on_msg;

    void reset(float sample_rate, int ch_idx){
        fs = sample_rate>1.f ? sample_rate : 48000.f;
        ch = ch_idx;
        FLEN = (int)(fs/1200.f) + 1; if(FLEN>FLEN_MAX) FLEN=FLEN_MAX; if(FLEN<2) FLEN=2;
        FLENO = FLEN*MFLTOVER + 1;
        for(int i=0;i<FLENO;i++){
            double x = std::cos(2.0*M_PI*600.0/fs/MFLTOVER*(i-(FLENO-1)/2.0));
            h[i] = (float)(x>0 ? x : 0);
        }
        for(int i=0;i<FLEN_MAX;i++){ inb_re[i]=inb_im[i]=0; }
        idx=0; MskPhi=0; MskClk=0; MskDf=0; MskS=0;
        outbits=0; nbits=1; st=WSYN; txtlen=0;
        dfw=0; dlacc=0; dbits=0; dmsg=0;
    }

    void feed(float audio){
        // 진단 누적 (~4초마다 rms / bit수 / 메시지수)
        dfw++; dlacc += (double)audio*audio;
        if(dfw >= (long)(fs*4.f)){
            char db[120];
            snprintf(db,sizeof(db),"ACARS[%d] diag rms=%.4f bits/4s=%ld msgs=%ld",
                     ch, std::sqrt(dlacc/(double)dfw), dbits, dmsg);
            if(on_msg) on_msg(db);
            dfw=0; dlacc=0; dbits=0;
        }

        // ── VCO @ 1800 Hz (+ PLL freq correction) ──
        double s = 1800.0/fs*2.0*M_PI + MskDf;
        MskPhi += s; if(MskPhi >= 2.0*M_PI) MskPhi -= 2.0*M_PI;
        // ── mixer: down-convert to baseband ──
        inb_re[idx] =  (float)( audio*std::cos(MskPhi));
        inb_im[idx] =  (float)(-audio*std::sin(MskPhi));
        idx = (idx+1)%FLEN;

        // ── bit clock (VCO advances 3π/2 per bit @ 2400 baud) ──
        MskClk += s;
        if(MskClk >= 3.0*M_PI/2.0 - s/2.0){
            MskClk -= 3.0*M_PI/2.0;
            int o = (int)(MFLTOVER*(MskClk/s + 0.5));
            if(o>MFLTOVER) o=MFLTOVER; if(o<0) o=0;
            // ── matched filter (half-cosine pulse) ──
            double vr=0, vi=0;
            for(int j=0;j<FLEN;j++){
                int oo = o + j*MFLTOVER; if(oo>=FLENO) oo=FLENO-1;
                int k = (j+idx)%FLEN;
                vr += (double)h[oo]*inb_re[k];
                vi += (double)h[oo]*inb_im[k];
            }
            double lvl = std::sqrt(vr*vr+vi*vi) + 1e-8;
            vr/=lvl; vi/=lvl;
            // ── alternating I/Q-axis decision (MSK) + off-axis phase error ──
            double vo, dphi;
            if(MskS&1){ vo=vi; dphi=(vo>=0)? -vr : vr; }
            else      { vo=vr; dphi=(vo>=0)?  vi : -vi; }
            double pv = (MskS&2)? -vo : vo;
            outbits = (uint8_t)((outbits>>1) | (pv>0 ? 0x80 : 0x00));
            MskS++;
            // ── carrier/timing PLL ──
            MskDf = PLLC*MskDf + (1.0-PLLC)*PLLG*dphi;
            // ── byte clock ──
            dbits++;
            if(--nbits<=0) process_byte();
        }
    }

private:
    static constexpr int    MFLTOVER=12;
    static constexpr int    FLEN_MAX=64;
    static constexpr int    FLENO_MAX=FLEN_MAX*MFLTOVER+1;
    static constexpr double PLLG=0.004, PLLC=0.5;
    static constexpr uint8_t SYN=0x16, SOH=0x01, ETX=0x83, ETB=0x97;

    float  fs=48000.f; int ch=0;
    int    FLEN=41, FLENO=493;
    float  h[FLENO_MAX];
    float  inb_re[FLEN_MAX], inb_im[FLEN_MAX];
    int    idx=0;
    double MskPhi=0, MskClk=0, MskDf=0; int MskS=0;
    uint8_t outbits=0; int nbits=1;
    enum St{ WSYN, SYN2, SOH1, TXT, CRC1, CRC2 } st=WSYN;
    uint8_t txt[300]; int txtlen=0; uint8_t crcb[2]={0,0};
    // diag
    long dfw=0; double dlacc=0; long dbits=0, dmsg=0;

    void process_byte(){
        uint8_t r = outbits;
        switch(st){
        case WSYN:
            if(r==SYN){ st=SYN2; nbits=8; }
            else if(r==(uint8_t)~SYN){ MskS^=2; st=SYN2; nbits=8; }  // inverted preamble → flip polarity
            else nbits=1;
            return;
        case SYN2:
            if(r==SYN){ st=SOH1; nbits=8; }
            else if(r==(uint8_t)~SYN){ MskS^=2; nbits=8; }
            else { st=WSYN; nbits=1; }
            return;
        case SOH1:
            if(r==SOH){ st=TXT; txtlen=0; nbits=8; }
            else { st=WSYN; nbits=1; }
            return;
        case TXT:
            if(txtlen<(int)sizeof(txt)) txt[txtlen++]=r;
            if(r==ETX || r==ETB){ st=CRC1; nbits=8; }
            else if(txtlen>=250){ st=WSYN; nbits=1; }
            else nbits=8;
            return;
        case CRC1: crcb[0]=r; st=CRC2; nbits=8; return;
        case CRC2: crcb[1]=r; finalize(); st=WSYN; nbits=1; return;
        }
    }

    // ACARS BCS: CRC-16, reflected poly 0x8408, init 0, no final XOR (over raw 8-bit bytes)
    static uint16_t crc_acars(const uint8_t* p, int n){
        uint16_t crc=0;
        for(int i=0;i<n;i++){ crc^=p[i]; for(int k=0;k<8;k++) crc=(crc&1)?(crc>>1)^0x8408:(crc>>1); }
        return crc;
    }
    static char pr(uint8_t c){ c&=0x7F; return (c>=0x20&&c<0x7F)?(char)c:'.'; }

    void finalize(){
        if(txtlen<13){ return; }
        uint16_t crc=crc_acars(txt,txtlen);   // raw 8-bit bytes (parity 포함, 와이어 그대로)
        uint8_t lo=crc&0xFF, hi=(crc>>8)&0xFF;
        bool ok=(lo==crcb[0]&&hi==crcb[1]);
        char reg[10]={0}; int ri=0;
        for(int i=1;i<=7;i++){ uint8_t c=txt[i]&0x7F; if((c=='.'||c==' ')&&ri==0)continue; reg[ri++]=pr(c); }
        char label[3]={ pr(txt[9]), pr(txt[10]), 0 };
        char tb[256]; int ti=0;
        if(txtlen>13 && (txt[12]&0x7F)==0x02)
            for(int i=13;i<txtlen-1 && ti<255;i++) tb[ti++]=pr(txt[i]);
        tb[ti]=0;
        char out[400];
        snprintf(out,sizeof(out),"ACARS[%d] reg=%s mode=%c lbl=%s blk=%c CRC=%s | %s",
                 ch, reg[0]?reg:"------", pr(txt[0]), label, pr(txt[11]), ok?"OK":"FAIL", tb);
        dmsg++;
        if(on_msg) on_msg(out);
    }
};
