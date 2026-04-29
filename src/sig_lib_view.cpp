#include "sig_lib_view.hpp"
#include "fft_viewer.hpp"
#include "net_client.hpp"
#include "net_protocol.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstring>
#include <ctime>
#include <string>

namespace SigLibView {

namespace {

constexpr float kBottomBarH = 32.0f;

// "yyyy-mm-dd HH:MM" 짧은 형식.
std::string fmt_short_local(int64_t utc){
    if(utc <= 0) return "-";
    time_t t = (time_t)utc;
    struct tm tm; localtime_r(&t, &tm);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tm);
    return buf;
}

// "<N>m / <N>h / <N>d ago"
std::string fmt_ago(int64_t utc){
    if(utc <= 0) return "-";
    int64_t now = (int64_t)time(nullptr);
    int64_t d = now - utc;
    if(d < 60)        return std::to_string(d) + "s ago";
    if(d < 3600)      return std::to_string(d/60) + "m ago";
    if(d < 86400)     return std::to_string(d/3600) + "h ago";
    if(d < 86400*30)  return std::to_string(d/86400) + "d ago";
    return fmt_short_local(utc);
}

const char* status_label(uint8_t s){
    switch(s){
    case 0: return "auto-high";
    case 1: return "auto-low";
    case 2: return "pending";
    case 3: return "confirmed";
    case 4: return "rejected";
    case 5: return "manual";
    default: return "?";
    }
}
ImU32 status_color(uint8_t s){
    switch(s){
    case 0: return IM_COL32(180,255,180,255); // auto-high green
    case 1: return IM_COL32(200,200,200,255); // auto-low gray
    case 2: return IM_COL32(255,210, 80,255); // pending yellow
    case 3: return IM_COL32(120,255,200,255); // confirmed teal
    case 4: return IM_COL32(255,140,140,255); // rejected red
    case 5: return IM_COL32( 90,200,255,255); // manual blue
    default: return IM_COL32(180,180,180,255);
    }
}

bool case_insensitive_contains(const std::string& hay, const std::string& needle){
    if(needle.empty()) return true;
    if(needle.size() > hay.size()) return false;
    auto lower_eq = [](char a, char b){
        return std::tolower((unsigned char)a) == std::tolower((unsigned char)b);
    };
    for(size_t i=0; i + needle.size() <= hay.size(); i++){
        bool ok = true;
        for(size_t j=0; j<needle.size(); j++){
            if(!lower_eq(hay[i+j], needle[j])){ ok=false; break; }
        }
        if(ok) return true;
    }
    return false;
}

bool emitter_matches_filter(FFTViewer& v, const PktEmitterEntry& e){
    // status filter is applied via sightings (each sighting has status, not emitter).
    // For now skip status filter at emitter level — show all emitters that match search.
    std::string name(e.display_name), mod(e.modulation), proto(e.protocol),
                tags(e.tags_id), stations(e.contributing_stations);
    std::string q = v.sig_lib_search;
    if(q.empty()) return true;
    if(case_insensitive_contains(name, q)) return true;
    if(case_insensitive_contains(mod, q)) return true;
    if(case_insensitive_contains(proto, q)) return true;
    if(case_insensitive_contains(tags, q)) return true;
    if(case_insensitive_contains(stations, q)) return true;
    char fbuf[32]; snprintf(fbuf,sizeof(fbuf),"%.4f",e.freq_center_mhz);
    if(case_insensitive_contains(fbuf, q)) return true;
    return false;
}

void draw_emitter_table(FFTViewer& v, NetClient* cli){
    (void)cli;
    std::lock_guard<std::mutex> lk(v.sig_lib_mtx);
    if(v.sig_lib_emitters.empty()){
        ImGui::TextDisabled("(no emitters yet — waiting for Central data)");
        return;
    }
    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                          | ImGuiTableFlags_Resizable | ImGuiTableFlags_Sortable
                          | ImGuiTableFlags_ScrollY;
    if(!ImGui::BeginTable("##emitters", 5, flags, ImVec2(-1, -1))) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch, 0.40f);
    ImGui::TableSetupColumn("Freq (MHz)", ImGuiTableColumnFlags_WidthFixed, 100.f);
    ImGui::TableSetupColumn("Mod", ImGuiTableColumnFlags_WidthFixed, 60.f);
    ImGui::TableSetupColumn("N", ImGuiTableColumnFlags_WidthFixed, 40.f);
    ImGui::TableSetupColumn("Last seen", ImGuiTableColumnFlags_WidthStretch, 0.25f);
    ImGui::TableHeadersRow();

    // Build filtered + sorted view.
    std::vector<int> rows;
    rows.reserve(v.sig_lib_emitters.size());
    for(int i=0; i<(int)v.sig_lib_emitters.size(); i++){
        if(emitter_matches_filter(v, v.sig_lib_emitters[i]))
            rows.push_back(i);
    }
    if(auto* sort = ImGui::TableGetSortSpecs()){
        if(sort->SpecsCount > 0){
            int col = sort->Specs[0].ColumnIndex;
            bool asc = sort->Specs[0].SortDirection == ImGuiSortDirection_Ascending;
            auto& src = v.sig_lib_emitters;
            std::sort(rows.begin(), rows.end(), [&](int a, int b){
                int sign = asc ? 1 : -1;
                switch(col){
                case 0: return sign * strcmp(src[a].display_name, src[b].display_name) < 0;
                case 1: return sign * (src[a].freq_center_mhz < src[b].freq_center_mhz ? -1 : 1) < 0;
                case 2: return sign * strcmp(src[a].modulation, src[b].modulation) < 0;
                case 3: return sign * (src[a].sighting_count < src[b].sighting_count ? -1 : 1) < 0;
                case 4: return sign * (src[a].last_seen_utc < src[b].last_seen_utc ? -1 : 1) < 0;
                }
                return false;
            });
        }
    }

    for(int idx : rows){
        const auto& e = v.sig_lib_emitters[idx];
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        bool is_sel = (v.sig_lib_selected_uid == e.emitter_uid);
        if(ImGui::Selectable(e.display_name, is_sel,
                             ImGuiSelectableFlags_SpanAllColumns)){
            v.sig_lib_selected_uid = e.emitter_uid;
            // 새로 선택된 emitter의 sightings 받기.
            if(cli) cli->cmd_sighting_list_req(e.emitter_uid, 0, MAX_SIGHTINGS_PER_PKT);
        }
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.4f", e.freq_center_mhz);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%s", e.modulation[0] ? e.modulation : "-");
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%u", e.sighting_count);
        ImGui::TableSetColumnIndex(4);
        ImGui::TextUnformatted(fmt_ago(e.last_seen_utc).c_str());
    }
    ImGui::EndTable();
}

void draw_detail_panel(FFTViewer& v, NetClient* cli){
    std::lock_guard<std::mutex> lk(v.sig_lib_mtx);
    const PktEmitterEntry* sel = nullptr;
    for(auto& e : v.sig_lib_emitters){
        if(v.sig_lib_selected_uid == e.emitter_uid){ sel = &e; break; }
    }
    if(!sel){
        ImGui::TextDisabled("(select an emitter from the list)");
        return;
    }

    static char name_buf[EMITTER_NAME_LEN];
    static char notes_buf[EMITTER_NOTES_LEN];
    static char tags_buf[EMITTER_TAGS_LEN];
    static std::string last_loaded_uid;
    if(last_loaded_uid != sel->emitter_uid){
        last_loaded_uid = sel->emitter_uid;
        memset(name_buf, 0, sizeof(name_buf));
        memset(notes_buf, 0, sizeof(notes_buf));
        memset(tags_buf, 0, sizeof(tags_buf));
        strncpy(name_buf,  sel->display_name,    sizeof(name_buf)-1);
        strncpy(notes_buf, sel->operator_notes,  sizeof(notes_buf)-1);
        strncpy(tags_buf,  sel->tags_id,         sizeof(tags_buf)-1);
    }

    ImGui::TextDisabled("UID: %s", sel->emitter_uid);
    ImGui::Separator();

    ImGui::Text("Name");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##name", name_buf, sizeof(name_buf));

    char freq_buf[32]; snprintf(freq_buf, sizeof(freq_buf), "%.4f", sel->freq_center_mhz);
    ImGui::Text("Frequency:  %s MHz   Tol: %.2f kHz   BW: %.1f kHz",
                freq_buf, sel->freq_tolerance_khz, sel->bw_khz);
    ImGui::Text("Modulation: %s   Protocol: %s",
                sel->modulation[0] ? sel->modulation : "(blank)",
                sel->protocol[0]   ? sel->protocol   : "(blank)");
    ImGui::Text("First seen: %s", fmt_short_local(sel->first_seen_utc).c_str());
    ImGui::Text("Last  seen: %s", fmt_short_local(sel->last_seen_utc).c_str());
    ImGui::Text("Sightings:  %u", sel->sighting_count);
    if(sel->contributing_stations[0])
        ImGui::TextWrapped("Stations: %s", sel->contributing_stations);

    ImGui::Spacing();
    ImGui::Text("Tags ID (e.g. \"MMSI:1234\")");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##tags", tags_buf, sizeof(tags_buf));

    ImGui::Text("Operator notes");
    ImGui::InputTextMultiline("##notes", notes_buf, sizeof(notes_buf),
                              ImVec2(-1, 80.f));

    if(ImGui::Button("Save changes") && cli){
        PktEmitterUpsert up{};
        strncpy(up.emitter_uid,    sel->emitter_uid,    sizeof(up.emitter_uid)-1);
        strncpy(up.display_name,   name_buf,            sizeof(up.display_name)-1);
        up.freq_center_mhz    = sel->freq_center_mhz;
        up.freq_tolerance_khz = sel->freq_tolerance_khz;
        up.bw_khz             = sel->bw_khz;
        strncpy(up.modulation,     sel->modulation,     sizeof(up.modulation)-1);
        strncpy(up.protocol,       sel->protocol,       sizeof(up.protocol)-1);
        strncpy(up.tags_id,        tags_buf,            sizeof(up.tags_id)-1);
        strncpy(up.operator_notes, notes_buf,           sizeof(up.operator_notes)-1);
        // editor stamp = my_name (NetClient에서 채움)
        cli->cmd_emitter_upsert(up);
    }
    ImGui::SameLine();
    if(ImGui::Button("Delete emitter") && cli){
        ImGui::OpenPopup("##del_confirm");
    }
    if(ImGui::BeginPopupModal("##del_confirm", nullptr,
                              ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoTitleBar)){
        ImGui::Text("Delete '%s'? Children sightings will be unlinked.", sel->display_name);
        if(ImGui::Button("Confirm delete", ImVec2(160, 0)) && cli){
            cli->cmd_emitter_delete(sel->emitter_uid);
            v.sig_lib_selected_uid.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if(ImGui::Button("Cancel", ImVec2(80, 0))){
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Sightings");

    // Sighting list (filtered by emitter_uid).
    if(v.sig_lib_sightings_filter_uid != sel->emitter_uid){
        // Cache mismatch — show hint until incoming list refreshes.
        ImGui::TextDisabled("(loading sightings...)");
        if(cli) cli->cmd_sighting_list_req(sel->emitter_uid, 0, MAX_SIGHTINGS_PER_PKT);
    } else {
        ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY;
        if(ImGui::BeginTable("##sightings", 5, flags, ImVec2(-1, 200.f))){
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("Time",      ImGuiTableColumnFlags_WidthFixed, 130.f);
            ImGui::TableSetupColumn("Station",   ImGuiTableColumnFlags_WidthFixed, 100.f);
            ImGui::TableSetupColumn("Reporter",  ImGuiTableColumnFlags_WidthFixed, 100.f);
            ImGui::TableSetupColumn("Status",    ImGuiTableColumnFlags_WidthFixed, 90.f);
            ImGui::TableSetupColumn("Action",    ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for(auto& s : v.sig_lib_sightings){
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(fmt_short_local(s.start_utc).c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%s", s.station[0] ? s.station : "-");
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%s", s.reporter[0] ? s.reporter : "-");
                ImGui::TableSetColumnIndex(3);
                ImGui::PushStyleColor(ImGuiCol_Text, status_color(s.match_status));
                ImGui::Text("%s", status_label(s.match_status));
                ImGui::PopStyleColor();
                ImGui::TableSetColumnIndex(4);
                ImGui::PushID(s.sighting_id);
                if(s.match_status == 2 /*pending*/){
                    if(ImGui::SmallButton("Confirm") && cli)
                        cli->cmd_sighting_link(s.sighting_id, "", 0, nullptr);
                    ImGui::SameLine();
                    if(ImGui::SmallButton("Reject") && cli)
                        cli->cmd_sighting_link(s.sighting_id, "", 1, nullptr);
                } else {
                    if(ImGui::SmallButton("Split") && cli)
                        cli->cmd_sighting_link(s.sighting_id, "", 3, nullptr);
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }
}

void merge_emitter_into_cache(FFTViewer& v,
                              const PktEmitterList& hdr,
                              const std::vector<PktEmitterEntry>& list){
    std::lock_guard<std::mutex> lk(v.sig_lib_mtx);
    // 단건 또는 페이지 모두 처리: uid로 갱신/삽입.
    for(auto& ne : list){
        bool found = false;
        for(auto& ee : v.sig_lib_emitters){
            if(strncmp(ee.emitter_uid, ne.emitter_uid, EMITTER_UID_LEN) == 0){
                ee = ne;
                found = true;
                break;
            }
        }
        if(!found) v.sig_lib_emitters.push_back(ne);
    }
    // 단건 broadcast(EMITTER_LIST count=1)에서 freq=0 + name 빈 엔트리 = delete signal:
    // emitter_uid만 있고 다른 필드 비어있으면 (freq==0 && name[0]==0) 캐시에서 제거.
    for(auto& ne : list){
        if(ne.freq_center_mhz == 0.f && ne.display_name[0] == 0
           && ne.sighting_count == 0 && ne.first_seen_utc == 0){
            v.sig_lib_emitters.erase(
                std::remove_if(v.sig_lib_emitters.begin(), v.sig_lib_emitters.end(),
                    [&](const PktEmitterEntry& x){
                        return strncmp(x.emitter_uid, ne.emitter_uid, EMITTER_UID_LEN) == 0
                            && (x.freq_center_mhz == 0.f);
                    }),
                v.sig_lib_emitters.end());
        }
    }
    (void)hdr;
}

void merge_sighting_into_cache(FFTViewer& v,
                               const PktSightingList& hdr,
                               const std::vector<PktSightingEntry>& list){
    std::lock_guard<std::mutex> lk(v.sig_lib_mtx);
    // 필터가 다르면 캐시 교체. 같으면 merge by sighting_id.
    char filter_buf[EMITTER_UID_LEN+1] = {};
    memcpy(filter_buf, hdr.emitter_uid_filter, EMITTER_UID_LEN);
    std::string new_filter = filter_buf;
    if(v.sig_lib_sightings_filter_uid != new_filter && hdr.offset == 0){
        v.sig_lib_sightings = list;
        v.sig_lib_sightings_filter_uid = new_filter;
    } else {
        for(auto& ns : list){
            bool found = false;
            for(auto& ss : v.sig_lib_sightings){
                if(strncmp(ss.sighting_id, ns.sighting_id, SIGHTING_ID_LEN) == 0){
                    ss = ns;
                    found = true;
                    break;
                }
            }
            if(!found) v.sig_lib_sightings.push_back(ns);
        }
    }
}

bool s_cb_registered = false;

} // anon

void register_callbacks_once(FFTViewer& v, NetClient* cli){
    if(s_cb_registered || !cli) return;
    s_cb_registered = true;
    cli->on_emitter_list = [&v](const PktEmitterList& hdr,
                                const std::vector<PktEmitterEntry>& list){
        merge_emitter_into_cache(v, hdr, list);
    };
    cli->on_sighting_list = [&v](const PktSightingList& hdr,
                                  const std::vector<PktSightingEntry>& list){
        merge_sighting_into_cache(v, hdr, list);
    };
}

void draw_overlay(FFTViewer& v, NetClient* cli){
    if(!v.sig_lib_panel_open) return;
    register_callbacks_once(v, cli);

    // 첫 진입 시 emitter 목록 요청.
    if(v.sig_lib_dirty){
        v.sig_lib_dirty = false;
        if(cli) cli->cmd_emitter_list_req(0, MAX_EMITTERS_PER_PKT);
    }

    ImGuiIO& io = ImGui::GetIO();
    float modal_h = io.DisplaySize.y - kBottomBarH;
    if(modal_h < 100.f) modal_h = 100.f;
    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, modal_h));
    ImGui::SetNextWindowBgAlpha(0.97f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f,0.07f,0.12f,1.f));
    ImGui::Begin("##sig_lib_overlay", &v.sig_lib_panel_open,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
        ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar);

    bool focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if(focused && ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        v.sig_lib_panel_open = false;

    // ── Header bar ───────────────────────────────────────────────────────
    ImGui::Indent(8.f);
    ImGui::Dummy(ImVec2(0,4));
    ImGui::Text("SIGNAL LIBRARY  [M to toggle, ESC to close]");
    ImGui::Dummy(ImVec2(0,2));
    ImGui::SetNextItemWidth(360.f);
    ImGui::InputTextWithHint("##search", "Search name / mod / proto / tags / station / freq",
                             v.sig_lib_search, sizeof(v.sig_lib_search));
    ImGui::SameLine();
    if(ImGui::Button("Refresh") && cli){
        cli->cmd_emitter_list_req(0, MAX_EMITTERS_PER_PKT);
        if(!v.sig_lib_selected_uid.empty())
            cli->cmd_sighting_list_req(v.sig_lib_selected_uid.c_str(), 0, MAX_SIGHTINGS_PER_PKT);
    }
    ImGui::SameLine(); ImGui::TextDisabled("|");
    ImGui::SameLine();
    {
        std::lock_guard<std::mutex> lk(v.sig_lib_mtx);
        ImGui::TextDisabled("emitters: %zu", v.sig_lib_emitters.size());
    }
    ImGui::Unindent(8.f);
    ImGui::Separator();

    // ── 2-pane: left table | right detail ────────────────────────────────
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float left_w = std::max(280.f, avail.x * 0.55f);
    ImGui::BeginChild("##siglib_left", ImVec2(left_w, avail.y), false);
    draw_emitter_table(v, cli);
    ImGui::EndChild();

    ImGui::SameLine();
    ImGui::BeginChild("##siglib_right", ImVec2(0, avail.y), true);
    draw_detail_panel(v, cli);
    ImGui::EndChild();

    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

} // namespace SigLibView
