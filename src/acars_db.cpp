#include "acars_db.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

// File format (little-endian):
//   "ABDB" | u32 nrec | u32 nop | records[nrec] | op_offsets[nop] | op_blob
//   record = reg[8] (UPPER alnum, null-padded) + type[4] + u16 op_id  (14 bytes)
//   op_offsets[i] = u32 byte offset into op_blob of operator i's C-string
namespace {
    std::vector<uint8_t> g_buf;
    const uint8_t* g_recs = nullptr;   // record array (stride 14)
    uint32_t       g_nrec = 0;
    const uint8_t* g_offs = nullptr;   // op_offsets (u32 each)
    uint32_t       g_nop  = 0;
    const char*    g_blob = nullptr;
    bool           g_ready = false;
    constexpr int  REC = 14;

    void norm(const char* reg, char out[8]){
        int n=0;
        for(const char* p=reg; *p && n<8; ++p){
            char c=*p;
            if(c>='a'&&c<='z') c=(char)(c-'a'+'A');
            if((c>='A'&&c<='Z')||(c>='0'&&c<='9')) out[n++]=c;
        }
        for(; n<8; ++n) out[n]=0;
    }
    uint32_t rd_u32(const uint8_t* p){ return p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24); }

    const uint8_t* find(const char* reg){
        if(!g_ready || !g_nrec) return nullptr;
        char key[8]; norm(reg,key);
        if(!key[0]) return nullptr;
        int lo=0, hi=(int)g_nrec-1;
        while(lo<=hi){
            int mid=(lo+hi)/2;
            const uint8_t* r=g_recs+(size_t)mid*REC;
            int c=memcmp(r,key,8);
            if(c==0) return r;
            if(c<0) lo=mid+1; else hi=mid-1;
        }
        return nullptr;
    }
}

bool acars_db_load(const char* path){
    g_ready=false;
    FILE* f=fopen(path,"rb"); if(!f) return false;
    fseek(f,0,SEEK_END); long sz=ftell(f); fseek(f,0,SEEK_SET);
    if(sz<12){ fclose(f); return false; }
    g_buf.resize((size_t)sz);
    size_t got=fread(g_buf.data(),1,(size_t)sz,f); fclose(f);
    if(got!=(size_t)sz) return false;
    const uint8_t* p=g_buf.data();
    if(memcmp(p,"ABDB",4)!=0) return false;
    g_nrec=rd_u32(p+4); g_nop=rd_u32(p+8);
    g_recs=p+12;
    size_t off_recs = 12 + (size_t)g_nrec*REC;
    if(off_recs + (size_t)g_nop*4 > (size_t)sz) return false;
    g_offs=p+off_recs;
    g_blob=(const char*)(p+off_recs+(size_t)g_nop*4);
    g_ready=true;
    return true;
}

bool acars_db_ready(){ return g_ready; }

const char* acars_db_type(const char* reg){
    const uint8_t* r=find(reg); if(!r) return "";
    static thread_local char t[5];
    memcpy(t, r+8, 4); t[4]=0; return t;
}

const char* acars_db_operator(const char* reg){
    const uint8_t* r=find(reg); if(!r) return "";
    uint16_t id = (uint16_t)(r[12] | (r[13]<<8));
    if(id>=g_nop) return "";
    uint32_t o = rd_u32(g_offs + (size_t)id*4);
    return g_blob + o;
}
