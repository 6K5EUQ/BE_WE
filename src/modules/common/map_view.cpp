// ── 공용 2D 지도 위젯 구현 (GUI 전용, *_view.cpp → CLI 빌드 제외, 1회 컴파일) ─
// 순수 ImGui ImDrawList. 등거리원통 투영 + Natural Earth 110m 해안선 폴리라인 +
// 위경도 격자 + 항적 꼬리 + 침로 마커 + 커서고정 휠줌 + 드래그 팬 + auto-fit.
#include "modview_map.hpp"
#include "../../world_map_data.hpp"   // static const WORLD_MAP_DATA[] — 이 TU 에서만 include
#include "../../korea_map_data.hpp"   // GSHHG 한국: KR_COAST_DATA / KR_LAND_TRI_FULL / KR_LAND_TRI_HALF
#include "../../korea_osm_data.hpp"   // OSM 한국(별도 소스, 제거 시 이 헤더 + kr_mode==3 분기만 삭제)
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace modview_map {

// 지도 표시 한계 — 한국 bbox. 카메라(줌아웃/팬)를 이 영역에 가둠. 데이터도 이 밖은 안 그림.
static const double MAP_LON0=124.0, MAP_LON1=132.0, MAP_LAT0=33.0, MAP_LAT1=39.0;

// 현재 카메라(v.lat/lon)를 MAP bbox 안으로 클램프. 줌아웃이 bbox 보다 커지면 bbox 에 맞춰 멈춤.
static void clamp_to_bbox(MapView& v, float W, float H){
    // span 이 bbox 보다 크면 bbox span 으로 축소 (한 축이라도 넘으면 줌아웃 한계)
    double lonsp=v.lon1-v.lon0, latsp=v.lat1-v.lat0;
    double maxlon=MAP_LON1-MAP_LON0, maxlat=MAP_LAT1-MAP_LAT0;
    if(lonsp>maxlon){ double c=(v.lon0+v.lon1)*0.5; v.lon0=c-maxlon*0.5; v.lon1=c+maxlon*0.5; lonsp=maxlon; }
    if(latsp>maxlat){ double c=(v.lat0+v.lat1)*0.5; v.lat0=c-maxlat*0.5; v.lat1=c+maxlat*0.5; latsp=maxlat; }
    // 경계 밖으로 나간 만큼 시프트해 되넣음 (span 유지)
    if(v.lon0<MAP_LON0){ v.lon1+=MAP_LON0-v.lon0; v.lon0=MAP_LON0; }
    if(v.lon1>MAP_LON1){ v.lon0-=v.lon1-MAP_LON1; v.lon1=MAP_LON1; }
    if(v.lat0<MAP_LAT0){ v.lat1+=MAP_LAT0-v.lat0; v.lat0=MAP_LAT0; }
    if(v.lat1>MAP_LAT1){ v.lat0-=v.lat1-MAP_LAT1; v.lat1=MAP_LAT1; }
}

// lat 스팬 기준으로 lon 스팬을 종횡비+cos(lat_c) 로 재계산 (isotropic 유지) + lat 클램프.
static void normalize(MapView& v, float W, float H){
    double latc=(v.lat0+v.lat1)*0.5, lonc=(v.lon0+v.lon1)*0.5;
    double latspan=v.lat1-v.lat0;
    if(latspan<0.002) latspan=0.002;
    if(latspan>170.0) latspan=170.0;
    if(latc-latspan*0.5 < -85.0) latc=-85.0+latspan*0.5;
    if(latc+latspan*0.5 >  85.0) latc= 85.0-latspan*0.5;
    double cl=std::cos(latc*M_PI/180.0); if(cl<0.05) cl=0.05;
    double lonspan = (double)W/(double)H * latspan / cl;
    v.lat0=latc-latspan*0.5; v.lat1=latc+latspan*0.5;
    v.lon0=lonc-lonspan*0.5; v.lon1=lonc+lonspan*0.5;
}

static void fit_to_points(MapView& v, const std::vector<MapPoint>& pts, float W, float H){
    double a=90,b=-90,c=180,d=-180; int n=0;
    for(const auto& p : pts){ if(p.lat<a)a=p.lat; if(p.lat>b)b=p.lat; if(p.lon<c)c=p.lon; if(p.lon>d)d=p.lon; n++; }
    if(n<1){ v.lat0=-80; v.lat1=80; v.lon0=-180; v.lon1=180; normalize(v,W,H); return; }
    double dlat=b-a, dlon=d-c;
    if(dlat<0.02) dlat=0.02; if(dlon<0.02) dlon=0.02;
    double mlat=(a+b)*0.5, mlon=(c+d)*0.5;
    v.lat0=mlat-dlat*0.575; v.lat1=mlat+dlat*0.575;   // +15% 패딩
    v.lon0=mlon-dlon*0.575; v.lon1=mlon+dlon*0.575;
    normalize(v,W,H);
}

// LAND_TRI_DATA(단위구 XYZ, 9 floats/삼각형) → lat,lon 정점 배열로 1회 변환 후 캐시.
static const float* land_latlon(int& nverts){
    static std::vector<float> g; static bool done=false;
    if(!done){
        done=true;
        g.reserve(LAND_TRI_COUNT/3*2);
        for(int i=0;i+2<LAND_TRI_COUNT;i+=3){
            float x=LAND_TRI_DATA[i], y=LAND_TRI_DATA[i+1], z=LAND_TRI_DATA[i+2];
            if(y>1.f)y=1.f; if(y<-1.f)y=-1.f;
            // LAND_TRI 생성기 좌표계: y=sin(lat), z=-cos(lat)*sin(lon) (globe latlon_to_xyz 와 z 부호 반대).
            // 평면지도는 raw 데이터를 직접 변환 → lat=asin(y), lon=atan2(-z,x). (해안선 정점과 99.6% 일치 실측)
            // globe 는 LAND_TRI raw 를 GPU 업로드 후 자체 transform → 이 함수 안 거침.
            g.push_back(std::asin(y)*57.29578f);
            g.push_back(std::atan2(-z,x)*57.29578f);
        }
    }
    nverts=(int)g.size()/2;
    return g.data();
}

MapResult draw_map(const char* id, MapView& v, const std::vector<MapPoint>& pts,
                   ImVec2 size, bool do_fit){
    MapResult r{};
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float W = size.x>0 ? size.x : avail.x;
    float H = size.y>0 ? size.y : avail.y;
    if(W<16) W=16; if(H<16) H=16;
    ImGui::InvisibleButton(id, ImVec2(W,H),
        ImGuiButtonFlags_MouseButtonLeft|ImGuiButtonFlags_MouseButtonRight);
    bool hovered = ImGui::IsItemHovered();
    ImVec2 p1 = ImVec2(p0.x+W, p0.y+H);

    if(!v.initialized && !pts.empty()){ fit_to_points(v, pts, W, H); v.initialized=true; }
    if(do_fit){ fit_to_points(v, pts, W, H); }
    clamp_to_bbox(v, W, H);   // 카메라를 한국 bbox 안으로 제한 (줌아웃/팬 한계)

    // ── 커서고정 휠줌 ──
    if(hovered && io.MouseWheel!=0.f){
        double zf = io.MouseWheel>0 ? 0.8 : 1.25;
        double fx=(io.MousePos.x-p0.x)/(double)W, fy=(io.MousePos.y-p0.y)/(double)H;
        double curlon=v.lon0+fx*(v.lon1-v.lon0);
        double curlat=v.lat1-fy*(v.lat1-v.lat0);
        double latspan=(v.lat1-v.lat0)*zf;
        if(latspan<0.002) latspan=0.002; if(latspan>170.0) latspan=170.0;
        v.lat1=curlat+fy*latspan; v.lat0=v.lat1-latspan;
        double latc=(v.lat0+v.lat1)*0.5, cl=std::cos(latc*M_PI/180.0); if(cl<0.05)cl=0.05;
        double lonspan=(double)W/(double)H*latspan/cl;
        v.lon0=curlon-fx*lonspan; v.lon1=v.lon0+lonspan;
        normalize(v,W,H);
        clamp_to_bbox(v,W,H);
    }
    // ── 드래그 팬 (3px 이상 끌면) ──
    bool panned=false;
    if(hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f)){
        double dlon=-(double)io.MouseDelta.x/W*(v.lon1-v.lon0);
        double dlat= (double)io.MouseDelta.y/H*(v.lat1-v.lat0);
        v.lon0+=dlon; v.lon1+=dlon; v.lat0+=dlat; v.lat1+=dlat;
        clamp_to_bbox(v,W,H);
        panned=true;
    }

    // 투영 람다 (현재 bounds)
    auto LL2PX=[&](double lat,double lon)->ImVec2{
        return ImVec2((float)(p0.x+(lon-v.lon0)/(v.lon1-v.lon0)*W),
                      (float)(p0.y+(v.lat1-lat)/(v.lat1-v.lat0)*H));
    };

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(14,22,34,255));     // 바다 배경

    // 한국 GSHHG bbox — kr_mode>0 일 때 이 영역은 world(110m) 대신 KR 고해상만 그림(이중선 방지).
    const float KRX0=124.f,KRX1=132.f,KRY0=33.f,KRY1=39.f;
    auto in_kr=[&](float la,float lo){ return lo>=KRX0&&lo<=KRX1&&la>=KRY0&&la<=KRY1; };
    bool kr_active = v.kr_mode>0 && v.lon1>=KRX0 && v.lon0<=KRX1 && v.lat1>=KRY0 && v.lat0<=KRY1;
    // 소스별 coast/tri 포인터 선택. 1=HALF 2=FULL 3=OSM (0=SIMPLE 은 kr_active=false).
    const float* KR_COAST = KR_COAST_DATA; int KR_COAST_N = KR_COAST_COUNT;
    const float* KR_TRI   = KR_LAND_TRI_HALF; int KR_TRI_N = KR_LAND_TRI_HALF_COUNT;
    if(v.kr_mode==2){ KR_TRI=KR_LAND_TRI_FULL; KR_TRI_N=KR_LAND_TRI_FULL_COUNT; }
    else if(v.kr_mode==3){ KR_COAST=KR_OSM_COAST; KR_COAST_N=KR_OSM_COAST_COUNT;
                           KR_TRI=KR_OSM_TRI;     KR_TRI_N=KR_OSM_TRI_COUNT; }
    // 모든 모드: 한국 bbox 만 표시 (그 밖은 클리핑해 빈 바다). OSM 은 추가로 world 통째 skip.
    bool kr_only = true;
    if(kr_only){
        // KR bbox 를 화면 픽셀로 → 그 밖 영역에 바다색 덮어 world/데이터 가림
        ImVec2 c0=LL2PX(KRY1,KRX0), c1=LL2PX(KRY0,KRX1);   // (lat1,lon0)=좌상, (lat0,lon1)=우하
        float bx0=std::max(p0.x,std::min(c0.x,c1.x)), bx1=std::min(p1.x,std::max(c0.x,c1.x));
        float by0=std::max(p0.y,std::min(c0.y,c1.y)), by1=std::min(p1.y,std::max(c0.y,c1.y));
        ImU32 sea=IM_COL32(14,22,34,255);
        if(by0>p0.y) dl->AddRectFilled(p0,ImVec2(p1.x,by0),sea);          // 상단
        if(by1<p1.y) dl->AddRectFilled(ImVec2(p0.x,by1),p1,sea);          // 하단
        if(bx0>p0.x) dl->AddRectFilled(ImVec2(p0.x,by0),ImVec2(bx0,by1),sea);  // 좌
        if(bx1<p1.x) dl->AddRectFilled(ImVec2(bx1,by0),p1,sea);           // 우
        dl->PushClipRect(ImVec2(bx0,by0),ImVec2(bx1,by1),true);          // KR bbox 로 클립
    }

    // ── 육지 채움 (LAND_TRI_DATA 재활용; lat/lon 1회 캐시, antimeridian/화면밖 컬링) ──
    bool world_off = (v.kr_mode==3);   // OSM 모드는 한국 자체 데이터라 world 통째 skip
    if(v.show_land && !world_off){
        int lv; const float* L=land_latlon(lv);
        ImU32 lc=IM_COL32(34,46,40,255);
        for(int t=0;t+2<lv;t+=3){
            float lo0=L[t*2+1], lo1=L[(t+1)*2+1], lo2=L[(t+2)*2+1];
            float la0=L[t*2],   la1=L[(t+1)*2],   la2=L[(t+2)*2];
            float lomin=std::min(lo0,std::min(lo1,lo2)), lomax=std::max(lo0,std::max(lo1,lo2));
            if(lomax-lomin>180.f) continue;                          // 자오선 가로질러 = 가짜 삼각형
            float lamin=std::min(la0,std::min(la1,la2)), lamax=std::max(la0,std::max(la1,la2));
            if(lomax<(float)v.lon0||lomin>(float)v.lon1||lamax<(float)v.lat0||lamin>(float)v.lat1) continue;
            // KR bbox 와 겹치는 world 삼각형은 skip (KR 고해상이 대신 채움)
            if(kr_active && lomax>=KRX0&&lomin<=KRX1&&lamax>=KRY0&&lamin<=KRY1) continue;
            dl->AddTriangleFilled(LL2PX(la0,lo0),LL2PX(la1,lo1),LL2PX(la2,lo2), lc);
        }
    }

    // ── 한국 GSHHG 고해상 채움 오버레이 (kr_active 시; world 위에 덧그림) ──
    if(v.show_land && kr_active){
        ImU32 lc=IM_COL32(34,46,40,255);
        for(int t=0;t+5<KR_TRI_N;t+=6){
            float la0=KR_TRI[t],   lo0=KR_TRI[t+1];
            float la1=KR_TRI[t+2], lo1=KR_TRI[t+3];
            float la2=KR_TRI[t+4], lo2=KR_TRI[t+5];
            float lomin=std::min(lo0,std::min(lo1,lo2)), lomax=std::max(lo0,std::max(lo1,lo2));
            float lamin=std::min(la0,std::min(la1,la2)), lamax=std::max(la0,std::max(la1,la2));
            if(lomax<(float)v.lon0||lomin>(float)v.lon1||lamax<(float)v.lat0||lamin>(float)v.lat1) continue;
            dl->AddTriangleFilled(LL2PX(la0,lo0),LL2PX(la1,lo1),LL2PX(la2,lo2), lc);
        }
    }

    // ── 해안선 (WORLD_MAP_DATA 1패스, NAN 끊김, 화면밖 컬링; 점당 trig 없음) ──
    if(v.show_coast && !world_off){
        bool have=false; ImVec2 prev; float prevlon=0;
        for(int i=0;i+1<WORLD_MAP_DATA_COUNT;i+=2){
            float lat=WORLD_MAP_DATA[i], lon=WORLD_MAP_DATA[i+1];
            if(std::isnan(lat)){ have=false; continue; }
            // KR bbox 안 점은 world 해안선 건너뜀 (KR 고해상이 대신; 이중선 방지)
            if(kr_active && in_kr(lat,lon)){ have=false; continue; }
            ImVec2 cur=LL2PX(lat,lon);
            if(have && std::fabs(lon-prevlon)<=180.f){
                bool out=(prev.x<p0.x&&cur.x<p0.x)||(prev.x>p1.x&&cur.x>p1.x)||
                         (prev.y<p0.y&&cur.y<p0.y)||(prev.y>p1.y&&cur.y>p1.y);
                if(!out) dl->AddLine(prev,cur, IM_COL32(120,150,175,255), 1.2f);
            }
            prev=cur; prevlon=lon; have=true;
        }
    }

    // ── 한국 GSHHG 해안선 오버레이 (world 위에 더 정밀하게 덧그림) ──
    if(v.show_coast && kr_active){
        bool have=false; ImVec2 prev;
        for(int i=0;i+1<KR_COAST_N;i+=2){
            float lat=KR_COAST[i], lon=KR_COAST[i+1];
            if(std::isnan(lat)){ have=false; continue; }
            ImVec2 cur=LL2PX(lat,lon);
            if(have){
                bool out=(prev.x<p0.x&&cur.x<p0.x)||(prev.x>p1.x&&cur.x>p1.x)||
                         (prev.y<p0.y&&cur.y<p0.y)||(prev.y>p1.y&&cur.y>p1.y);
                if(!out) dl->AddLine(prev,cur, IM_COL32(120,150,175,255), 1.2f);
            }
            prev=cur; have=true;
        }
    }
    if(kr_only) dl->PopClipRect();   // KR bbox 클립 해제 (격자/마커는 전체 캔버스에)

    // ── 위경도 격자 ──
    if(v.show_grid){
        const double steps[]={0.05,0.1,0.25,0.5,1,2,5,10,30};
        double latspan=v.lat1-v.lat0, step=30;
        for(double s : steps){ if(latspan/s <= 7){ step=s; break; } }
        ImU32 gc=IM_COL32(45,60,78,140), tc=IM_COL32(120,140,160,200);
        char lab[24];
        for(double lon=std::ceil(v.lon0/step)*step; lon<=v.lon1; lon+=step){
            ImVec2 a=LL2PX(v.lat0,lon), b=LL2PX(v.lat1,lon);
            dl->AddLine(ImVec2(a.x,p0.y),ImVec2(b.x,p1.y), gc, 1.0f);
            snprintf(lab,sizeof(lab),"%.2f%c", std::fabs(lon), lon>=0?'E':'W');
            dl->AddText(ImVec2(a.x+2,p1.y-14), tc, lab);
        }
        for(double lat=std::ceil(v.lat0/step)*step; lat<=v.lat1; lat+=step){
            ImVec2 a=LL2PX(lat,v.lon0);
            dl->AddLine(ImVec2(p0.x,a.y),ImVec2(p1.x,a.y), gc, 1.0f);
            snprintf(lab,sizeof(lab),"%.2f%c", std::fabs(lat), lat>=0?'N':'S');
            dl->AddText(ImVec2(p0.x+2,a.y+2), tc, lab);
        }
    }

    // ── 항적 + 마커 ── (최근접 마커 추적 → hover/click)
    int best=-1; float bestd=1e9f; ImVec2 mp=io.MousePos;
    for(size_t i=0;i<pts.size();i++){
        const MapPoint& pt=pts[i];
        // 항적 꼬리 (oldest→newest, alpha ramp)
        if(v.show_trails && pt.trail && pt.trail_n>1){
            ImU32 base = pt.color & 0x00FFFFFFu;
            ImVec2 tp = LL2PX(pt.trail[0], pt.trail[1]);
            for(int k=1;k<pt.trail_n;k++){
                ImVec2 tc=LL2PX(pt.trail[k*2], pt.trail[k*2+1]);
                int a = 40 + (int)(140.0*k/(pt.trail_n-1));
                dl->AddLine(tp, tc, base|((ImU32)a<<24), pt.selected?2.0f:1.2f);
                tp=tc;
            }
        }
        ImVec2 s=LL2PX(pt.lat, pt.lon);
        if(s.x<p0.x||s.x>p1.x||s.y<p0.y||s.y>p1.y) continue;   // 화면밖 컬링
        if(pt.heading>=0.f){
            float a=pt.heading*(float)M_PI/180.f;
            float ux=std::sin(a), uy=-std::cos(a);             // 전방 (북=위)
            float rx=-uy, ry=ux;                               // 우현
            float L=pt.selected?9.f:6.f, Wd=pt.selected?5.f:3.5f;
            ImVec2 tip{s.x+ux*L, s.y+uy*L};
            ImVec2 bl {s.x-ux*L*0.6f-rx*Wd, s.y-uy*L*0.6f-ry*Wd};
            ImVec2 br {s.x-ux*L*0.6f+rx*Wd, s.y-uy*L*0.6f+ry*Wd};
            dl->AddTriangleFilled(tip,bl,br, pt.color);
            if(pt.selected) dl->AddTriangle(tip,bl,br, IM_COL32(255,255,255,255), 1.5f);
        } else {
            float rad=pt.selected?5.f:3.f;
            dl->AddCircleFilled(s, rad, pt.color, 10);
            if(pt.selected) dl->AddCircle(s, rad+2, IM_COL32(255,255,255,255), 12, 1.5f);
        }
        if(v.show_labels && pt.label && (pt.selected || true))
            dl->AddText(ImVec2(s.x+8,s.y-6), pt.selected ? IM_COL32(255,240,200,255) : IM_COL32(200,220,255,180), pt.label);
        if(hovered){ float dx=s.x-mp.x, dy=s.y-mp.y, d=dx*dx+dy*dy; if(d<bestd){ bestd=d; best=(int)i; } }
    }

    // ── 스케일 바 (우하단; 거리 감각) ──
    {
        double latc=(v.lat0+v.lat1)*0.5;
        double mpp=(v.lon1-v.lon0)/W*111320.0*std::cos(latc*M_PI/180.0);   // m/px
        if(mpp>0){
            double target=mpp*(W*0.22);
            double pw=std::pow(10.0,std::floor(std::log10(target)));
            double nn=target/pw, nice = nn<1.5?1.0 : nn<3.5?2.0 : nn<7.5?5.0:10.0;
            double meters=nice*pw; float px=(float)(meters/mpp);
            float bx=p1.x-16-px, by=p1.y-12; ImU32 sc=IM_COL32(235,235,245,235);
            dl->AddLine(ImVec2(bx,by),ImVec2(bx+px,by),sc,2.f);
            dl->AddLine(ImVec2(bx,by-4),ImVec2(bx,by+4),sc,2.f);
            dl->AddLine(ImVec2(bx+px,by-4),ImVec2(bx+px,by+4),sc,2.f);
            char sl[24]; if(meters>=1000) snprintf(sl,sizeof(sl),"%.0f km",meters/1000.0); else snprintf(sl,sizeof(sl),"%.0f m",meters);
            dl->AddText(ImVec2(bx,by-15),sc,sl);
        }
    }

    // ── 지도 소스 선택 (우측 상단 세로 버튼: SIMPLE / HALF / FULL / OSM) ──
    // 라디오 선택. kr_mode: 0=SIMPLE 1=HALF 2=FULL 3=OSM. LAYERS 토글 패널은 제거됨
    // (LAND/COAST/GRID/TRAILS/LABELS 는 항상 켜짐 — MapView 기본값 true 고정).
    {
        const char* names[4] = {"SIMPLE","HALF","FULL","OSM"};
        const int N=4;
        const float BW=58.f, BH=18.f, PAD=4.f, MX=6.f, MY=6.f;
        float bx = p1.x - BW - MX;
        for(int i=0;i<N;i++){
            float by = p0.y + MY + i*(BH+PAD);
            ImVec2 b0(bx,by), b1(bx+BW,by+BH);
            bool sel = (v.kr_mode==i);
            bool bhov = hovered && io.MousePos.x>=b0.x && io.MousePos.x<=b1.x
                                && io.MousePos.y>=b0.y && io.MousePos.y<=b1.y;
            ImU32 bg = sel ? (bhov?IM_COL32(40,110,160,235):IM_COL32(30,90,140,225))
                           :(bhov?IM_COL32(45,60,80,210):IM_COL32(18,30,46,200));
            dl->AddRectFilled(b0,b1,bg,3.f);
            if(sel) dl->AddRect(b0,b1,IM_COL32(120,200,255,255),3.f,0,1.5f);
            ImU32 tc = sel ? IM_COL32(220,240,255,255) : IM_COL32(150,165,185,210);
            ImVec2 ts=ImGui::CalcTextSize(names[i]);
            dl->AddText(ImVec2(b0.x+(BW-ts.x)*0.5f, b0.y+(BH-ts.y)*0.5f), tc, names[i]);
            if(bhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) v.kr_mode=i;
        }
    }

    dl->PopClipRect();

    // ── hover 툴팁 + 클릭(드래그 아님) 선택 ──
    if(hovered && best>=0 && bestd<=169.f){
        const MapPoint& pt=pts[best];
        r.hovered_id=pt.id;
        if(pt.tip_l1||pt.tip_l2){
            ImGui::BeginTooltip();
            if(pt.tip_l1) ImGui::TextUnformatted(pt.tip_l1);
            if(pt.tip_l2) ImGui::TextUnformatted(pt.tip_l2);
            ImGui::EndTooltip();
        }
        ImVec2 dr=ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
        if(!panned && pt.id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)
           && std::fabs(dr.x)+std::fabs(dr.y) < 4.f)
            r.clicked_id=pt.id;
    }
    return r;
}

} // namespace modview_map
