#pragma once
// Signal Library overlay (slot 4, 단축키 M).
// Central에 누적된 emitter/sighting을 검색·필터·확정·메모.
//
// FFTViewer 멤버:
//   bool sig_lib_panel_open
//   std::vector<PktEmitterEntry>  sig_lib_emitters
//   std::vector<PktSightingEntry> sig_lib_sightings
//   bool sig_lib_dirty             — 오버레이 진입 시 list_req 보내기
//
// NetClient 콜백:
//   on_emitter_list / on_sighting_list — 받아서 sig_lib_emitters/sightings에 머지(클라이언트 측 캐시).

class FFTViewer;
class NetClient;

namespace SigLibView {

// 매 프레임 호출 (다른 오버레이가 활성이면 자동 무시).
void draw_overlay(FFTViewer& v, NetClient* cli);

// NetClient 콜백 등록 (한 번만). 받은 entry를 FFTViewer 캐시에 merge.
void register_callbacks_once(FFTViewer& v, NetClient* cli);

} // namespace
