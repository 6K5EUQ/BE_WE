// ── 공용 2D 지도 위젯 구현 (GUI 전용, *_view.cpp → CLI 빌드 제외, 1회 컴파일) ─
// 순수 ImGui ImDrawList. 등거리원통 투영 + OSM 한국 육지/해안선 +
// 위경도 격자 + 항적 꼬리 + 침로 마커 + 커서고정 휠줌 + 드래그 팬 + auto-fit.
#include "modview_map.hpp"
#include "../../korea_osm_data.hpp"   // OSM 한국 해안선 (KR_OSM_COAST) — 육지/해안선 유일 소스
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace modview_map {

// 지도 표시 한계 — 한국 bbox. 카메라(줌아웃/팬)를 이 영역에 가둠. 데이터도 이 밖은 안 그림.
static const double MAP_LON0=123.0, MAP_LON1=132.0, MAP_LAT0=32.0, MAP_LAT1=39.0;

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

// 육지 채움용 엣지 — 해안선(KR_OSM_COAST) 닫힌 링만 모아 lat 오름차순 정렬 (1회 캐시).
// 화면 scanline even-odd 로 "해안선이 둘러싼 영역"을 그대로 채우는 paint-bucket 용.
struct FillEdge { float latlo, lathi, lonlo, slope; };  // lonlo=latlo 에서의 lon, slope=dlon/dlat
static const std::vector<FillEdge>& fill_edges(){
    static std::vector<FillEdge> E; static bool done=false;
    if(!done){
        done=true;
        std::vector<float> rla, rlo;   // 현재 링 (lat,lon)
        auto flush=[&](){
            size_t n=rla.size();
            if(n>=3 && std::fabs(rla.front()-rla.back())<0.002f
                    && std::fabs(rlo.front()-rlo.back())<0.002f){   // 닫힌 링만
                for(size_t i=0;i<n;i++){
                    float la1=rla[i], lo1=rlo[i];
                    float la2=rla[(i+1)%n], lo2=rlo[(i+1)%n];
                    if(la1==la2) continue;
                    if(la1>la2){ std::swap(la1,la2); std::swap(lo1,lo2); }
                    E.push_back({la1,la2,lo1,(lo2-lo1)/(la2-la1)});
                }
            }
            rla.clear(); rlo.clear();
        };
        for(int i=0;i+1<KR_OSM_COAST_COUNT;i+=2){
            float lat=KR_OSM_COAST[i], lon=KR_OSM_COAST[i+1];
            if(std::isnan(lat)){ flush(); continue; }
            rla.push_back(lat); rlo.push_back(lon);
        }
        flush();
        std::sort(E.begin(),E.end(),[](const FillEdge&a,const FillEdge&b){ return a.latlo<b.latlo; });
    }
    return E;
}

MapResult draw_map(const char* id, MapView& v, const std::vector<MapPoint>& pts,
                   ImVec2 size, bool do_fit, const std::vector<MapStation>* stations,
                   const std::vector<MapLink>* links){
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

    // 최초 1회만 auto-fit. 탭 재진입(do_fit) 시엔 카메라(줌/위치) 유지 — 초기화 안 함.
    if(!v.initialized && !pts.empty()){ fit_to_points(v, pts, W, H); v.initialized=true; }
    else if(do_fit && !v.initialized){ fit_to_points(v, pts, W, H); }
    // 캔버스 크기(전체화면 토글 등)가 바뀌면 종횡비 재보정 — 안 하면 첫 프레임 지도 비율 깨짐.
    else if(v.initialized && v._fl_W>0 && (std::fabs(v._fl_W-W)>0.5f || std::fabs(v._fl_H-H)>0.5f))
        normalize(v, W, H);
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

    // 한국 bbox — OSM 데이터는 이 영역만 커버. 카메라가 겹칠 때만 그림.
    const float KRX0=123.f,KRX1=132.f,KRY0=32.f,KRY1=39.f;
    bool kr_active = v.lon1>=KRX0 && v.lon0<=KRX1 && v.lat1>=KRY0 && v.lat0<=KRY1;
    // KR bbox 밖은 클리핑해 빈 바다로. bbox 안만 OSM 육지/해안선 표시.
    {
        // KR bbox 를 화면 픽셀로 → 그 밖 영역에 바다색 덮음
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

    // ── OSM 한국 육지 채움 (해안선이 둘러싼 영역을 화면 픽셀행 even-odd scanline 으로 채움) ──
    // paint-bucket: 흰 해안선과 동일 데이터(닫힌 링)를 화면 1px 행마다 채워 줌 무관 픽셀 단위 매끈.
    // 카메라(bbox)/크기 변경 시만 재계산 → v._fill 막대(p0 상대 px) 캐시, 정지 프레임은 재방출만.
    if(v.show_land && kr_active){
        bool changed = v.lat0!=v._fl_lat0 || v.lat1!=v._fl_lat1
                    || v.lon0!=v._fl_lon0 || v.lon1!=v._fl_lon1
                    || W!=v._fl_W || H!=v._fl_H;
        if(changed){
            v._fl_lat0=v.lat0; v._fl_lat1=v.lat1; v._fl_lon0=v.lon0; v._fl_lon1=v.lon1; v._fl_W=W; v._fl_H=H;
            v._fill.clear();
            const std::vector<FillEdge>& E = fill_edges();
            double latspan=v.lat1-v.lat0, lonspan=v.lon1-v.lon0;
            size_t NE=E.size(), ei=0;
            static std::vector<const FillEdge*> active; active.clear();
            static std::vector<float> xs;
            int Hi=(int)H;
            for(int yy=Hi-1; yy>=0; --yy){                       // 아래(저위도)→위 = lat 오름차순 sweep
                double latc = v.lat1 - ((yy+0.5)/(double)Hi)*latspan;
                while(ei<NE && E[ei].latlo<=latc){ active.push_back(&E[ei]); ++ei; }   // 새 엣지 활성화
                for(size_t a=0;a<active.size();){                 // 만료 엣지 제거 (swap-pop)
                    if(active[a]->lathi < latc){ active[a]=active.back(); active.pop_back(); }
                    else ++a;
                }
                xs.clear();
                for(const FillEdge* e : active)
                    if(e->latlo<=latc && latc<e->lathi)
                        xs.push_back(e->lonlo + (float)(latc-e->latlo)*e->slope);   // 교차 lon
                if(xs.size()<2) continue;
                std::sort(xs.begin(), xs.end());
                for(size_t k=0;k+1<xs.size();k+=2){               // even-odd: 쌍 사이 채움
                    float dx0=(float)((xs[k]  -v.lon0)/lonspan*W);
                    float dx1=(float)((xs[k+1]-v.lon0)/lonspan*W);
                    v._fill.push_back(dx0); v._fill.push_back((float)yy); v._fill.push_back(dx1);
                }
            }
        }
        ImU32 lc=IM_COL32(34,46,40,255);
        for(size_t i=0;i+2<v._fill.size();i+=3)
            dl->AddRectFilled(ImVec2(p0.x+v._fill[i],   p0.y+v._fill[i+1]),
                              ImVec2(p0.x+v._fill[i+2], p0.y+v._fill[i+1]+1.0f), lc);
    }

    // ── OSM 한국 해안선 (KR_OSM_COAST: interleaved lat,lon, NAN 끊김, 화면밖 컬링) ──
    if(v.show_coast && kr_active){
        bool have=false; ImVec2 prev;
        for(int i=0;i+1<KR_OSM_COAST_COUNT;i+=2){
            float lat=KR_OSM_COAST[i], lon=KR_OSM_COAST[i+1];
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
    dl->PopClipRect();   // KR bbox 클립 해제 (격자/마커는 전체 캔버스에)

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
    bool any_sel=false; for(const auto& p : pts) if(p.selected){ any_sel=true; break; }  // 선택 배 있으면 그 배만 꼬리
    for(size_t i=0;i<pts.size();i++){
        const MapPoint& pt=pts[i];
        // 항적 꼬리 (oldest→newest, alpha ramp) — 선택 배 있으면 비선택 배 꼬리 숨김(그 배만 돋보이게)
        if(v.show_trails && pt.trail && pt.trail_n>1 && (!any_sel || pt.selected)){
            ImU32 base = pt.color & 0x00FFFFFFu;
            ImVec2 tp = LL2PX(pt.trail[0], pt.trail[1]);
            if(pt.selected) dl->AddCircleFilled(tp, 1.25f, base|0xC0000000u, 8);  // GPS 기록점
            for(int k=1;k<pt.trail_n;k++){
                ImVec2 tc=LL2PX(pt.trail[k*2], pt.trail[k*2+1]);
                int a = 40 + (int)(140.0*k/(pt.trail_n-1));
                dl->AddLine(tp, tc, base|((ImU32)a<<24), pt.selected?2.0f:1.2f);
                if(pt.selected) dl->AddCircleFilled(tc, 1.25f, base|0xC0000000u, 8);  // 각 기록 위치에 점
                // 항적선 클릭 판정: 마우스↔선분 최단거리 (꼬리 눌러도 선박 선택)
                if(hovered && pt.id){
                    float vx=tc.x-tp.x, vy=tc.y-tp.y, wx=mp.x-tp.x, wy=mp.y-tp.y;
                    float L2=vx*vx+vy*vy, t=(L2>1e-3f)?((wx*vx+wy*vy)/L2):0.f;
                    if(t<0)t=0; if(t>1)t=1;
                    float cxp=tp.x+t*vx-mp.x, cyp=tp.y+t*vy-mp.y, d=cxp*cxp+cyp*cyp;
                    if(d<bestd){ bestd=d; best=(int)i; }
                }
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

    // ── 기지→선박 점선 (선택 기지가 수신한 선박 연결; 마커 아래에 깔림) ──
    if(links){
        for(const MapLink& ln : *links){
            ImVec2 a=LL2PX(ln.lat0,ln.lon0), b=LL2PX(ln.lat1,ln.lon1);
            bool out=(a.x<p0.x&&b.x<p0.x)||(a.x>p1.x&&b.x>p1.x)||(a.y<p0.y&&b.y<p0.y)||(a.y>p1.y&&b.y>p1.y);
            if(out) continue;
            // 점선: 짧은 선분 반복
            float dx=b.x-a.x, dy=b.y-a.y, len=std::sqrt(dx*dx+dy*dy);
            if(len<1.f) continue;
            float ux=dx/len, uy=dy/len; const float dash=6.f, gap=5.f;
            for(float t=0; t<len; t+=dash+gap){
                float t2=std::min(t+dash,len);
                dl->AddLine(ImVec2(a.x+ux*t,a.y+uy*t), ImVec2(a.x+ux*t2,a.y+uy*t2), IM_COL32(255,120,120,140), 1.0f);
            }
        }
    }

    // ── 수신소(기지) 마커 + 이름 — 실제 복조한 기지 위치 오버레이 ──
    int st_hit=-1;
    if(stations){
        int si=-1;
        for(const MapStation& st : *stations){
            ++si;
            ImVec2 s=LL2PX(st.lat, st.lon);
            if(s.x<p0.x||s.x>p1.x||s.y<p0.y||s.y>p1.y) continue;
            // 클릭 감지 (마커 ~12px 반경)
            if(hovered){ float dx=io.MousePos.x-s.x, dy=io.MousePos.y-s.y;
                if(dx*dx+dy*dy<=144.f && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) st_hit=si; }
            ImU32 sc = st.selected ? IM_COL32(255,200,80,255) : IM_COL32(255,90,90,255);
            // 안테나(수신소) 아이콘: 마스트 + 베이스 + 송신 호(전파)
            float mh=8.f;                                   // 마스트 높이
            ImVec2 top(s.x, s.y-mh), base(s.x, s.y);
            dl->AddLine(top, base, sc, 1.6f);               // 마스트
            dl->AddLine(ImVec2(s.x-4,s.y),ImVec2(s.x+4,s.y), sc, 1.6f);  // 베이스
            dl->AddCircleFilled(top, 1.8f, sc, 8);          // 안테나 팁
            for(int rr=2;rr<=3;rr++){                        // 송신 전파 호(좌우)
                float rad=rr*2.4f;
                dl->PathArcTo(top, rad, -2.10f, -1.04f, 8); dl->PathStroke(sc, 0, 1.2f);  // 우측 호
                dl->PathArcTo(top, rad, -2.10f+3.1416f, -1.04f+3.1416f, 8); dl->PathStroke(sc, 0, 1.2f);  // 좌측 호
            }
            if(st.name && st.name[0]){
                // 안테나 밑 중앙 — 투명 배경, 그림자만 살짝(가독성)
                ImVec2 ts=ImGui::CalcTextSize(st.name);
                ImVec2 lp(s.x-ts.x*0.5f, s.y+3.f);
                dl->AddText(ImVec2(lp.x+1,lp.y+1), IM_COL32(0,0,0,150), st.name);  // 그림자
                dl->AddText(lp, IM_COL32(255,190,190,255), st.name);
            }
        }
    }
    r.clicked_station = st_hit;

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

    // ── 마우스 좌표 (좌하단; hover 시 실시간 lat/lon) ──
    if(hovered){
        double mlon=v.lon0+(double)(io.MousePos.x-p0.x)/W*(v.lon1-v.lon0);
        double mlat=v.lat1-(double)(io.MousePos.y-p0.y)/H*(v.lat1-v.lat0);
        char cl[40];
        snprintf(cl,sizeof(cl),"%.5f%c  %.5f%c",
                 std::fabs(mlat), mlat>=0?'N':'S', std::fabs(mlon), mlon>=0?'E':'W');
        ImVec2 ts=ImGui::CalcTextSize(cl);
        float tx=p0.x+6, ty=p1.y-ts.y-5;
        dl->AddRectFilled(ImVec2(tx-3,ty-2),ImVec2(tx+ts.x+3,ty+ts.y+2),IM_COL32(0,0,0,150),3.f);
        dl->AddText(ImVec2(tx,ty),IM_COL32(235,235,245,235),cl);
    }

    // ── 크게보기/작게하기 토글 (좌측 상단, 아이콘). 호출자가 v.big 읽어 표 숨기고 지도 전폭. ──
    {
        const float SZ=20.f, MX=6.f, MY=6.f;
        ImVec2 b0(p0.x+MX, p0.y+MY), b1(p0.x+MX+SZ, p0.y+MY+SZ);
        bool bhov = hovered && io.MousePos.x>=b0.x && io.MousePos.x<=b1.x
                            && io.MousePos.y>=b0.y && io.MousePos.y<=b1.y;
        // 배경/테두리 없음 — 아이콘만 (hover 시 살짝 밝게)
        ImU32 ic= bhov?IM_COL32(255,255,255,255):IM_COL32(210,225,245,235);
        float cx=(b0.x+b1.x)*0.5f, cy=(b0.y+b1.y)*0.5f; float a=5.f;
        if(!v.big){
            // 바깥쪽 4방향 화살촉 (확대)
            ImVec2 c[4]={{b0.x+3,b0.y+3},{b1.x-3,b0.y+3},{b1.x-3,b1.y-3},{b0.x+3,b1.y-3}};
            int dx[4]={1,-1,-1,1}, dy[4]={1,1,-1,-1};
            for(int k=0;k<4;k++){
                dl->AddLine(c[k], ImVec2(c[k].x+dx[k]*a, c[k].y), ic, 1.6f);
                dl->AddLine(c[k], ImVec2(c[k].x, c[k].y+dy[k]*a), ic, 1.6f);
            }
        } else {
            // 안쪽 4방향 화살촉 (축소)
            ImVec2 c[4]={{cx-a-1,cy-a-1},{cx+a+1,cy-a-1},{cx+a+1,cy+a+1},{cx-a-1,cy+a+1}};
            int dx[4]={1,-1,-1,1}, dy[4]={1,1,-1,-1};
            for(int k=0;k<4;k++){
                dl->AddLine(c[k], ImVec2(c[k].x+dx[k]*a, c[k].y), ic, 1.6f);
                dl->AddLine(c[k], ImVec2(c[k].x, c[k].y+dy[k]*a), ic, 1.6f);
            }
        }
        if(bhov && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) v.big=!v.big;
    }

    dl->PopClipRect();

    // ── hover 툴팁 + 클릭(드래그 아님) 선택 ──
    if(hovered && best>=0 && bestd<=169.f){
        const MapPoint& pt=pts[best];
        r.hovered_id=pt.id;
        if(pt.tip_l1||pt.tip_l2){
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10,8));  // 툴팁 여백 (글씨 벽에 안 끼게)
            ImGui::BeginTooltip();
            float bodyw = pt.tip_l2 ? ImGui::CalcTextSize(pt.tip_l2).x : 0.f;  // 본문 최대폭
            if(pt.tip_l1){
                float tw1=ImGui::CalcTextSize(pt.tip_l1).x;
                if(tw1<bodyw) ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(bodyw-tw1)*0.5f);  // MMSI 중앙
                ImGui::TextUnformatted(pt.tip_l1);
                ImGui::Separator();   // MMSI ↔ 값 구분선
            }
            if(pt.tip_l2) ImGui::TextUnformatted(pt.tip_l2);
            ImGui::EndTooltip();
            ImGui::PopStyleVar();
        }
        ImVec2 dr=ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
        if(!panned && pt.id && ImGui::IsMouseReleased(ImGuiMouseButton_Left)
           && std::fabs(dr.x)+std::fabs(dr.y) < 4.f)
            r.clicked_id=pt.id;
    }
    return r;
}

} // namespace modview_map
