// ── WiFi 모듈 GUI (draw_content) — GUI 빌드 전용 (*_view.cpp) ───────────────
// M1: 진단행(capture 상태) 표시 + Recv/Clear. M2+: SSID/보안/PHY 비콘 테이블.
#include "wifi_module.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <ctime>

namespace wifi_mod {

static void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}

void draw_content(FFTViewer& v, bool just_opened){
    bool remote = bewe_mod_my_station()[0] != 0;   // station 있으면 JOIN/원격
    if(just_opened && !remote) local_load_today(v);

    // ── 헤더: Recv(원격) / Clear / count ──
    if(remote){
        bool rcv = bewe_mod_recv("wifi");
        ImVec4 col = rcv ? ImVec4(0.2f,0.7f,0.3f,1.f) : ImVec4(0.7f,0.2f,0.2f,1.f);
        ImGui::PushStyleColor(ImGuiCol_Button, col);
        if(ImGui::Button(rcv?"Recv ON":"Recv OFF")){
            bewe_mod_set_recv(v, "wifi", !rcv);
            if(rcv){ std::lock_guard<std::mutex> lk(mtx); log.clear(); }
        }
        ImGui::PopStyleColor();
        if(bewe_mod_hist_loading("wifi")){ ImGui::SameLine(); ImGui::TextDisabled("(loading…)"); }
        ImGui::SameLine();
    }
    if(ImGui::Button("Clear")){ std::lock_guard<std::mutex> lk(mtx); log.clear(); }
    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    ImGui::SameLine(); ImGui::Text("%d rec", total);
    ImGui::Separator();

    ImGuiTableFlags fl = ImGuiTableFlags_ScrollY|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##wifi", 7, fl)){
        ImGui::TableSetupScrollFreeze(0,1);
        ImGui::TableSetupColumn("Time");
        ImGui::TableSetupColumn("Freq");
        ImGui::TableSetupColumn("SSID");
        ImGui::TableSetupColumn("Ch");
        ImGui::TableSetupColumn("Sec");
        ImGui::TableSetupColumn("Peak");
        ImGui::TableSetupColumn("Bursts/SR");
        ImGui::TableHeadersRow();

        std::lock_guard<std::mutex> lk(mtx);
        ImGuiListClipper clip; clip.Begin((int)log.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const WifiRecord& m = log[r];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms, ts);
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(ts);
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f", m.freq);
            ImGui::TableSetColumnIndex(2);
            if(m.ssid[0]) ImGui::TextUnformatted(m.ssid);
            else          ImGui::TextDisabled("(capturing)");
            ImGui::TableSetColumnIndex(3); if(m.wch) ImGui::Text("%d", m.wch);
            ImGui::TableSetColumnIndex(4); if(m.sec[0]) ImGui::TextUnformatted(m.sec);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.0f dB", m.peak_dbfs);
            ImGui::TableSetColumnIndex(6); ImGui::Text("%d / %u M", m.burst_count, m.out_sr/1000000);
        }
        ImGui::EndTable();
    }
}

} // namespace wifi_mod
