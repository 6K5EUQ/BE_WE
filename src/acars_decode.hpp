#pragma once
// ── ACARS (ARINC 618 "Plain Old ACARS" / Mode-A) streaming decoder ─────────
// Self-contained C++ implementation — no external library, only <cmath> etc.
// Decodes from real-valued AM-demodulated audio at a known rate (e.g. 48 kHz).
//
// ACARS physical layer (ARINC 618):
//   • 2400 bit/s, MSK (continuous-phase, modulation index 0.5).
//   • Two audio tones 1200 Hz and 2400 Hz around a 1800 Hz center
//     (shift = ±600 Hz = bitrate/4), carried as DSB-AM.
//   • 7-bit ASCII, LSB first, odd parity (8th bit).
//   • Block: pre-key → SYN SYN SOH → fields → ETX/ETB → 2-byte BCS (CRC).
//
// MSK is OQPSK with half-sine pulses, so it is NOT a plain 2-FSK: the data bit
// rides on an axis that rotates 90° every symbol. This receiver therefore does
// a coherent MSK detection (carrier down-mix → matched filter → rotating-axis
// decision → decision-directed carrier/timing loop), which is the standard way
// to demodulate MSK. A naive FM/frequency discriminator does NOT recover ACARS.
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <functional>
#include "acars_meta.hpp"

class AcarsDecoder {
public:
    std::function<void(const char*)>       on_msg;     // diagnostic line (console)
    std::function<void(const AcarsMsg&)>   on_record;  // structured decoded message

    void reset(float sample_rate, int ch_idx){
        sr = sample_rate>1.f ? sample_rate : 48000.f;
        ch = ch_idx;
        // matched filter = one MSK half-sine pulse, ~one symbol-pair long.
        mf_taps = (int)(sr/1200.f) + 1;                 // ≈ 2 bit periods worth
        if(mf_taps>TAPS_MAX) mf_taps=TAPS_MAX; if(mf_taps<2) mf_taps=2;
        mf_len  = mf_taps*MF_OS + 1;
        const double shift = 600.0;                     // half the tone spacing
        for(int i=0;i<mf_len;i++){
            double x = std::cos(2.0*M_PI*shift/sr/MF_OS*(i-(mf_len-1)/2.0));
            mf[i] = (float)(x>0 ? x : 0);               // half-cosine, clipped
        }
        for(int i=0;i<TAPS_MAX;i++){ bb_re[i]=bb_im[i]=0; }
        bb_pos=0; vco=0; sym_ph=0; pll=0; sym_n=0;
        shifter=0; need=1; fr=WAIT_SYN; flen=0;
        d_n=0; d_e=0; d_bits=0; d_msgs=0;
    }

    void feed(float a){
        // diagnostics (~4 s of fed audio)
        d_n++; d_e += (double)a*a;
        if(d_n >= (long)(sr*4.f)){
            char db[120];
            snprintf(db,sizeof(db),"ACARS[%d] diag rms=%.4f bits/4s=%ld msgs=%ld",
                     ch, std::sqrt(d_e/(double)d_n), d_bits, d_msgs);
            if(on_msg) on_msg(db);
            d_n=0; d_e=0; d_bits=0;
        }

        // ── carrier NCO @ 1800 Hz (+ loop correction) ──
        double w = 1800.0/sr*2.0*M_PI + pll;
        vco += w; if(vco >= 2.0*M_PI) vco -= 2.0*M_PI;
        // mix audio down to complex baseband, store in the matched-filter delay line
        bb_re[bb_pos] = (float)( a*std::cos(vco));
        bb_im[bb_pos] = (float)(-a*std::sin(vco));
        bb_pos = (bb_pos+1)%mf_taps;

        // ── symbol clock: NCO advances 3π/2 per symbol (1800/2400 = ¾ cycle) ──
        sym_ph += w;
        if(sym_ph < 3.0*M_PI/2.0 - w/2.0) return;
        sym_ph -= 3.0*M_PI/2.0;

        // matched-filter correlation at the current fractional symbol phase
        int o0 = (int)(MF_OS*(sym_ph/w + 0.5));
        if(o0>MF_OS) o0=MF_OS; if(o0<0) o0=0;
        double yr=0, yi=0;
        for(int j=0;j<mf_taps;j++){
            int t = o0 + j*MF_OS; if(t>=mf_len) t=mf_len-1;
            int k = (j+bb_pos)%mf_taps;
            yr += (double)mf[t]*bb_re[k];
            yi += (double)mf[t]*bb_im[k];
        }
        double mag = std::sqrt(yr*yr+yi*yi) + 1e-8;
        yr/=mag; yi/=mag;

        // ── rotating-axis decision ──
        // MSK carrier turns 90° per symbol, so the data axis cycles +R,+I,-R,-I.
        // bit  = sign of the on-axis projection.
        // perr = off-axis projection (≈0 when locked) → carrier/timing loop input.
        double bit_proj, perr;
        switch(sym_n & 3){
            case 0: bit_proj= yr; perr=(yr>=0)?  yi : -yi; break;  // +real
            case 1: bit_proj= yi; perr=(yi>=0)? -yr :  yr; break;  // +imag
            case 2: bit_proj=-yr; perr=(yr>=0)?  yi : -yi; break;  // -real
            default:bit_proj=-yi; perr=(yi>=0)? -yr :  yr; break;  // -imag
        }
        sym_n++;
        // LSB-first into the byte shifter
        shifter = (uint8_t)((shifter>>1) | (bit_proj>0 ? 0x80 : 0x00));
        // decision-directed loop (1st-order): pull carrier/clock to null perr
        pll = LOOP_LEAK*pll + (1.0-LOOP_LEAK)*LOOP_GAIN*perr;

        d_bits++;
        if(--need <= 0) take_byte(shifter);
    }

private:
    static constexpr int    MF_OS   = 12;            // matched-filter oversampling
    static constexpr int    TAPS_MAX= 64;
    static constexpr int    MFLEN_MAX = TAPS_MAX*MF_OS + 1;
    static constexpr double LOOP_GAIN = 0.004, LOOP_LEAK = 0.5;
    static constexpr uint8_t C_SYN=0x16, C_SOH=0x01, C_ETX=0x83, C_ETB=0x97;

    float  sr=48000.f; int ch=0;
    int    mf_taps=41, mf_len=493;
    float  mf[MFLEN_MAX];
    float  bb_re[TAPS_MAX], bb_im[TAPS_MAX];
    int    bb_pos=0;
    double vco=0, sym_ph=0, pll=0; int sym_n=0;
    uint8_t shifter=0; int need=1;
    enum Fr{ WAIT_SYN, GOT_SYN1, WAIT_SOH, BODY, BCS_HI, BCS_LO } fr=WAIT_SYN;
    uint8_t frame[300]; int flen=0; uint8_t bcs[2]={0,0};
    // diagnostics
    long d_n=0; double d_e=0; long d_bits=0, d_msgs=0;

    // byte-level framing (slides bit-by-bit until SYN, then byte-aligned).
    void take_byte(uint8_t r){
        switch(fr){
        case WAIT_SYN:
            if(r==C_SYN){ fr=GOT_SYN1; need=8; }
            else if(r==(uint8_t)~C_SYN){ sym_n^=2; fr=GOT_SYN1; need=8; } // inverted preamble → 180° polarity flip
            else need=1;
            return;
        case GOT_SYN1:
            if(r==C_SYN){ fr=WAIT_SOH; need=8; }
            else if(r==(uint8_t)~C_SYN){ sym_n^=2; need=8; }
            else { fr=WAIT_SYN; need=1; }
            return;
        case WAIT_SOH:
            if(r==C_SOH){ fr=BODY; flen=0; need=8; }
            else { fr=WAIT_SYN; need=1; }
            return;
        case BODY:
            if(flen<(int)sizeof(frame)) frame[flen++]=r;
            if(r==C_ETX || r==C_ETB){ fr=BCS_HI; need=8; }
            else if(flen>=250){ fr=WAIT_SYN; need=1; }
            else need=8;
            return;
        case BCS_HI: bcs[0]=r; fr=BCS_LO; need=8; return;
        case BCS_LO: bcs[1]=r; emit(); fr=WAIT_SYN; need=1; return;
        }
    }

    // ACARS BCS: 16-bit CRC, reflected polynomial 0x8408, zero init, no final XOR,
    // computed over the on-wire bytes (parity bit included), SOH excluded,
    // through and including the ETX/ETB terminator.
    static uint16_t bcs_crc(const uint8_t* p, int n){
        uint16_t c=0;
        for(int i=0;i<n;i++){ c^=p[i]; for(int k=0;k<8;k++) c=(c&1)?(c>>1)^0x8408:(c>>1); }
        return c;
    }
    static char pc(uint8_t c){ c&=0x7F; return (c>=0x20&&c<0x7F)?(char)c:'.'; }

    void emit(){
        if(flen<13) return;                              // shorter than a valid header
        uint16_t c = bcs_crc(frame, flen);
        // fields: [0]=mode [1..7]=registration [8]=ack [9..10]=label [11]=block
        //         [12]=STX (if any text) [13..]=text  [last]=ETX/ETB
        AcarsMsg m;
        m.ch = ch;
        m.crc_ok = ((c&0xFF)==bcs[0]) && ((c>>8)==bcs[1]);
        m.mode = pc(frame[0]);
        m.ack  = pc(frame[8]);
        m.block= pc(frame[11]);
        m.label[0]=pc(frame[9]); m.label[1]=pc(frame[10]); m.label[2]=0;
        int ri=0;
        for(int i=1;i<=7;i++){ uint8_t v=frame[i]&0x7F; if((v=='.'||v==' ')&&ri==0)continue; if(ri<9) m.reg[ri++]=pc(v); }
        m.reg[ri]=0;
        int ti=0;
        if(flen>13 && (frame[12]&0x7F)==0x02)
            for(int i=13;i<flen-1 && ti<255;i++) m.text[ti++]=pc(frame[i]);
        m.text[ti]=0;
        // best-effort flight id (downlink text starts with 4-char MSN + 6-char flight)
        if(ti>=10){
            bool letter=false, alnum=true;
            for(int i=4;i<10;i++){ char cc=m.text[i];
                if(!((cc>='A'&&cc<='Z')||(cc>='0'&&cc<='9'))){ alnum=false; break; }
                if(cc>='A'&&cc<='Z') letter=true; }
            if(alnum && letter){ for(int i=0;i<6;i++) m.flight[i]=m.text[4+i]; m.flight[6]=0; m.downlink=true; }
        }
        d_msgs++;
        if(on_record) on_record(m);
        if(on_msg){
            char out[400];
            snprintf(out,sizeof(out),"ACARS[%d] reg=%s mode=%c lbl=%s blk=%c CRC=%s | %s",
                     ch, m.reg[0]?m.reg:"------", m.mode, m.label, m.block, m.crc_ok?"OK":"FAIL", m.text);
            on_msg(out);
        }
    }
};
