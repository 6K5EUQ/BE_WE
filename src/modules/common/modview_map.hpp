#pragma once
// ── 해독 모듈 공용 경량 2D 지도 위젯 (GUI 전용) ─────────────────────────────
// 위치형 해독 모듈(AIS/ADS-B 등)이 좌표+항적을 ImGui ImDrawList 만으로 등거리원통도법
// 2D 지도에 그린다. OpenGL/타일/네트워크 없음. 해안선은 src/world_map_data.hpp 재사용
// (구현부 map_view.cpp 에서 1회 include). 모듈은 MapPoint 리스트만 만들어 draw_map 호출.
//   - *_view.cpp 에서만 include (GUI 빌드 전용). world_map_data.hpp 는 절대 직접 include 금지.
#include <imgui.h>
#include <cstdint>
#include <vector>

namespace modview_map {

struct MapPoint {
    double   lat = 0.0, lon = 0.0;   // 머리(최신) 위치
    float    heading = -1.f;         // deg [0,360); <0 = 점(dot)
    ImU32    color = IM_COL32(80,200,255,255);
    bool     selected = false;
    uint64_t id = 0;                 // MMSI / ICAO 등. 0 = 클릭 불가
    const char* label  = nullptr;    // 선택 시 표시할 짧은 이름 (옵션)
    const char* tip_l1 = nullptr;    // hover 툴팁 1행 (호출자 소유, 프레임 동안 유효)
    const char* tip_l2 = nullptr;
    const float* trail = nullptr;    // interleaved lat,lon (oldest→newest), 옵션
    int      trail_n = 0;            // trail 점 개수
};

// 영속 카메라 상태 (모듈이 1개 보유). 등거리원통 도(degree) 경계.
struct MapView {
    double lat0=-80, lat1=80, lon0=-180, lon1=180;  // 현재 가시 창 (deg)
    bool   initialized = false;                     // false = 첫 비어있지않은 프레임에 auto-fit
    bool   show_grid      = true;
    bool   show_land      = true;    // 육지 채움
    bool   show_coast     = true;    // 해안선
    bool   show_trails    = true;    // 항적 꼬리
    bool   show_labels    = true;    // 마커 이름 라벨
    int    kr_mode        = 3;       // 지도 소스: 0=SIMPLE(world 110m) 1=HALF(GSHHG 큰섬) 2=FULL(GSHHG 전체) 3=OSM(기본)
    bool   big            = false;   // 크게보기: 지도가 패널 전폭 차지 (호출자가 읽어 표 숨김). 좌상단 버튼으로 토글
};

// 수신소(기지) 마커 — 실제 복조한 기지 위치+이름 오버레이용.
struct MapStation {
    double      lat=0.0, lon=0.0;
    const char* name=nullptr;   // 호출자 소유 (프레임 동안 유효)
    bool        selected=false; // 선택됨(클릭) — 강조 표시
};

// 기지→선박 점선 (선택 기지가 수신한 선박 연결). 호출자가 좌표쌍으로 빌드.
struct MapLink { double lat0,lon0, lat1,lon1; };

struct MapResult {
    uint64_t clicked_id = 0;       // 이번 프레임 클릭된 선박 마커 id (없으면 0)
    uint64_t hovered_id = 0;
    int      clicked_station = -1; // 이번 프레임 클릭된 수신소 인덱스 (없으면 -1)
};

// 현재 윈도우/자식 안에 캔버스를 예약하고 지도를 그린다. size.x/y<=0 이면 남은 영역 사용.
// pts 는 읽기전용 (호출자가 매 프레임 캐시에서 싸게 재구성). do_fit=true 면 이번 프레임 fit.
// stations: 수신소 마커. links: 기지→선박 점선(선택 기지 수신선박). 없으면 nullptr.
MapResult draw_map(const char* id, MapView& view, const std::vector<MapPoint>& pts,
                   ImVec2 size = ImVec2(0,0), bool do_fit = false,
                   const std::vector<MapStation>* stations = nullptr,
                   const std::vector<MapLink>* links = nullptr);

} // namespace modview_map
