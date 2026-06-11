#pragma once
// ── ACARS (ARINC 618 Mode-A) streaming decoder ────────────────────────────
// Input: real-valued AM-demodulated audio samples at a known rate.
// Pipeline: 1800 Hz complex down-mix → LPF → FM discriminator (tone = 1200/2400)
//           → bit integrate&dump w/ transition-tracking clock → sym stream
//           → 4-hypothesis sync search (covers disc-sign + NRZI polarity)
//           → byte assembly (7-bit ASCII LSB-first, odd parity) → frame parse
//           → CRC-16/X-25 (BCS) verify → on_msg() callback.
// Self-contained: no BE_WE deps. Polarity/NRZI convention resolved empirically
// by matching SYN SYN SOH (0x16 0x16 0x01) + BCS, per the verified ACARS spec.
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <vector>
#include <functional>

class AcarsDecoder {
public:
    std::function<void(const char*)> on_msg;   // one decoded line (owner -> log)

    void reset(float sample_rate, int ch_idx){
        fs    = sample_rate>1.f ? sample_rate : 48000.f;
        ch    = ch_idx;
        spb   = (double)fs / 2400.0;
        mixst = 2.0*M_PI*1800.0/fs;
        lpfa  = 1.0 - std::exp(-2.0*M_PI*1100.0/fs);
        mixph = 0; li1=li2=lq1=lq2=0; pbi=pbq=0;
        t=0; bacc=0; prevds=0;
        lastsym=0; for(int k=0;k<4;k++) hist[k]=0;
        unlock();
    }

    void feed(float audio){
        // 1800 Hz complex down-mix
        double c=std::cos(mixph), s=std::sin(mixph);
        mixph += mixst; if(mixph>2.0*M_PI) mixph-=2.0*M_PI;
        double bi=audio*c, bq=-audio*s;
        // 2-stage one-pole LPF (isolate ±600 Hz tones, kill images)
        li1+=lpfa*(bi-li1); li2+=lpfa*(li1-li2);
        lq1+=lpfa*(bq-lq1); lq2+=lpfa*(lq1-lq2);
        double I=li2, Q=lq2;
        // FM discriminator: sign indicates which tone (absolute sign ambiguous → ok)
        double disc=I*pbq - Q*pbi; pbi=I; pbq=Q;
        bacc+=disc;
        // transition-tracking bit clock: nudge boundary toward tone sign-changes
        int ds = (disc>0)?1:-1;
        if(prevds!=0 && ds!=prevds){
            double err = (t < spb*0.5) ? t : (t - spb);
            t -= 0.10*err;
        }
        prevds=ds;
        t += 1.0;
        if(t>=spb){
            t-=spb;
            int sym = (bacc>0)?1:0;     // 1 = one tone, 0 = the other (mapping resolved at sync)
            bacc=0;
            push_sym(sym);
        }
    }

private:
    // ── config / DSP state ──
    float  fs=48000.f; int ch=0;
    double spb=20.0, mixst=0, lpfa=0;
    double mixph=0, li1=0,li2=0,lq1=0,lq2=0, pbi=0,pbq=0;
    double t=0, bacc=0; int prevds=0;
    // ── sync search ──
    int      lastsym=0;
    uint32_t hist[4]={0,0,0,0};            // 24-bit histories for 4 bit-derivations
    static constexpr uint32_t SYNC = 0x686880u; // SYN SYN SOH, wire bits MSB=first-tx
    // ── frame assembly (after lock) ──
    bool locked=false; int hyp=0;
    int  curbyte=0, bytebit=0;
    enum { ST_TEXT, ST_BCS } state=ST_TEXT;
    std::vector<uint8_t> frame;            // 7-bit chars: mode..ETX/ETB inclusive
    uint8_t bcs[2]={0,0}; int bcs_n=0;

    void unlock(){
        locked=false; hyp=0; curbyte=0; bytebit=0; state=ST_TEXT;
        bcs_n=0; frame.clear();
    }

    void push_sym(int sym){
        int d[4];
        d[0]=sym;                d[1]=sym^1;
        d[2]=(sym^lastsym)^1;    d[3]=(sym^lastsym);
        lastsym=sym;
        if(!locked){
            for(int k=0;k<4;k++){
                hist[k]=((hist[k]<<1)|(uint32_t)d[k]) & 0xFFFFFFu;
                if(hist[k]==SYNC){
                    locked=true; hyp=k; curbyte=0; bytebit=0;
                    state=ST_TEXT; bcs_n=0; frame.clear();
                    break;
                }
            }
        } else {
            assemble_bit(d[hyp]);
        }
    }

    void assemble_bit(int b){
        curbyte |= (b & 1) << bytebit;      // LSB first
        if(++bytebit>=8){
            process_byte((uint8_t)curbyte);
            curbyte=0; bytebit=0;
        }
    }

    void process_byte(uint8_t wb){
        if(state==ST_BCS){
            bcs[bcs_n++]=wb;                // BCS bytes carry no parity (full 8 bits)
            if(bcs_n>=2) finalize();
            return;
        }
        uint8_t c7 = wb & 0x7F;             // strip odd-parity bit
        frame.push_back(c7);
        if(c7==0x03 || c7==0x17){           // ETX / ETB
            state=ST_BCS; bcs_n=0; return;
        }
        if(frame.size()>260) unlock();      // runaway / false lock
    }

    static uint16_t crc_x25(const uint8_t* p, size_t n){
        uint16_t crc=0xFFFF;
        for(size_t i=0;i<n;i++){
            crc ^= p[i];
            for(int k=0;k<8;k++)
                crc = (crc&1) ? (crc>>1)^0x8408 : (crc>>1);
        }
        return crc ^ 0xFFFF;
    }

    void finalize(){
        // CRC over chars after SOH through ETX/ETB inclusive (7-bit values)
        uint16_t crc = crc_x25(frame.data(), frame.size());
        uint8_t lo=crc&0xFF, hi=(crc>>8)&0xFF;
        bool ok = (lo==bcs[0]&&hi==bcs[1]) || (hi==bcs[0]&&lo==bcs[1]);

        if(frame.size()>=12){
            char reg[12]={0}; int ri=0;
            for(int i=1;i<=7;i++){ uint8_t c=frame[i];
                if((c=='.'||c==' ') && ri==0) continue;        // strip leading pad
                reg[ri++]=(c>=0x20&&c<0x7F)?(char)c:'.'; }
            char label[3]={ pr(frame[9]), pr(frame[10]), 0 };
            char mode = pr(frame[0]);
            char bid  = pr(frame[11]);
            // text: STX(0x02) at [12], text follows until suffix (last frame elem)
            char txt[256]; int ti=0;
            if(frame.size()>13 && frame[12]==0x02){
                for(size_t i=13;i+1<frame.size() && ti<255;i++)
                    txt[ti++] = pr(frame[i]);
            }
            txt[ti]=0;
            char buf[400];
            snprintf(buf,sizeof(buf),
                "ACARS[%d] reg=%s mode=%c lbl=%s blk=%c CRC=%s | %s",
                ch, reg[0]?reg:"------", mode, label, bid, ok?"OK":"FAIL", txt);
            if(on_msg) on_msg(buf);
        }
        unlock();
    }

    static char pr(uint8_t c){ return (c>=0x20 && c<0x7F) ? (char)c : '.'; }
};
