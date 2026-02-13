#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

struct FFTHeader {
    char magic[4];
    uint32_t version;
    uint32_t fft_size;
    uint32_t sample_rate;
    uint64_t center_frequency;
    uint32_t num_ffts;
    uint32_t time_average;
    float power_min;
    float power_max;
    float reserved[8];
};

class FFTViewer {
public:
    FFTHeader header;
    std::vector<int8_t> fft_data;
    
    int current_fft_idx = 0;
    float freq_zoom = 1.0f;
    float freq_pan = 0.0f;
    float display_power_min = 0.0f;
    float display_power_max = 0.0f;
    float spectrum_height_ratio = 0.4f;
    std::string window_title;

    bool load_file(const char *filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            fprintf(stderr, "Failed to open file: %s\n", filename);
            return false;
        }

        file.read(reinterpret_cast<char*>(&header), sizeof(FFTHeader));
        if (std::string(header.magic, 4) != "FFTD") {
            fprintf(stderr, "Invalid FFT data file\n");
            return false;
        }

        printf("FFT Header: num_ffts=%u, fft_size=%u\n", header.num_ffts, header.fft_size);
        printf("Power range: %.1f ~ %.1f dB\n", header.power_min, header.power_max);

        size_t expected_size = static_cast<size_t>(header.num_ffts) * header.fft_size;
        fft_data.resize(expected_size);
        file.read(reinterpret_cast<char*>(fft_data.data()), expected_size * sizeof(int8_t));
        file.close();
        
        char title[256];
        snprintf(title, sizeof(title), "FFT Viewer - %.2f MHz", header.center_frequency / 1e6f);
        window_title = title;
        
        display_power_min = header.power_min;
        display_power_max = header.power_max;
        return true;
    }

    float get_freq_from_bin(int bin, float sr_mhz) {
        int n = header.fft_size;
        if (bin == 0) return 0.0f;
        if (bin <= n / 2) {
            return bin * sr_mhz / n;
        } else {
            return (bin - n) * sr_mhz / n;
        }
    }

    int get_bin_from_freq(float freq, float sr_mhz) {
        int n = header.fft_size;
        if (freq >= 0) {
            int bin = static_cast<int>(freq * n / sr_mhz + 0.5f);
            return std::max(0, std::min(n / 2, bin));
        } else {
            int bin = static_cast<int>(freq * n / sr_mhz + 0.5f);
            return std::max(n / 2 + 1, std::min(n - 1, n + bin));
        }
    }

    void draw_spectrum(float canvas_width, float canvas_height) {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size(canvas_width, canvas_height);

        ImVec2 plot_pos(canvas_pos.x + 40, canvas_pos.y);
        ImVec2 plot_size(canvas_size.x - 40, canvas_size.y - 25);

        draw_list->AddRectFilled(plot_pos, ImVec2(plot_pos.x + plot_size.x, plot_pos.y + plot_size.y), 
                                 IM_COL32(20, 20, 20, 255));

        if (current_fft_idx >= static_cast<int>(header.num_ffts)) return;

        const int8_t *spectrum = fft_data.data() + current_fft_idx * header.fft_size;

        float nyquist = header.sample_rate / 2.0f / 1e6f;
        float total_range = 2.0f * nyquist;
        
        float disp_start = -nyquist + freq_pan * total_range;
        float disp_width = total_range / freq_zoom;
        float disp_end = disp_start + disp_width;
        
        disp_start = std::max(-nyquist, disp_start);
        disp_end = std::min(nyquist, disp_end);

        float power_range = display_power_max - display_power_min;
        
        draw_power_grid(draw_list, plot_pos, plot_size, power_range);
        draw_freq_grid(draw_list, plot_pos, plot_size, disp_start, disp_end);

        float sr_mhz = header.sample_rate / 1e6f;
        int start_bin = get_bin_from_freq(disp_start, sr_mhz);
        int end_bin = get_bin_from_freq(disp_end, sr_mhz);
        
        if (start_bin > end_bin) std::swap(start_bin, end_bin);
        end_bin = std::min(end_bin + 1, static_cast<int>(header.fft_size));

        for (int bin = start_bin; bin < end_bin - 1; bin++) {
            float freq1 = get_freq_from_bin(bin, sr_mhz);
            float freq2 = get_freq_from_bin(bin + 1, sr_mhz);
            
            float norm_freq1 = (freq1 - disp_start) / (disp_end - disp_start);
            float norm_freq2 = (freq2 - disp_start) / (disp_end - disp_start);
            
            norm_freq1 = std::max(0.0f, std::min(1.0f, norm_freq1));
            norm_freq2 = std::max(0.0f, std::min(1.0f, norm_freq2));

            float p1 = dequantize(spectrum[bin]);
            float p2 = dequantize(spectrum[bin + 1]);
            float np1 = (p1 - display_power_min) / power_range;
            float np2 = (p2 - display_power_min) / power_range;
            np1 = std::max(0.0f, std::min(1.0f, np1));
            np2 = std::max(0.0f, std::min(1.0f, np2));

            float x1 = plot_pos.x + norm_freq1 * plot_size.x;
            float x2 = plot_pos.x + norm_freq2 * plot_size.x;
            float y1 = plot_pos.y + plot_size.y * (1.0f - np1);
            float y2 = plot_pos.y + plot_size.y * (1.0f - np2);

            draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), get_color(np1), 2.0f);
        }

        ImGui::InvisibleButton("spectrum_canvas", canvas_size);
        if (ImGui::IsItemHovered()) {
            ImGuiIO& io = ImGui::GetIO();
            ImVec2 mouse = ImGui::GetMousePos();
            float mx = (mouse.x - plot_pos.x) / plot_size.x;
            mx = std::max(0.0f, std::min(1.0f, mx));
            float freq_mouse = disp_start + mx * (disp_end - disp_start);
            
            if (io.MouseWheel != 0.0f) {
                freq_zoom *= (1.0f + io.MouseWheel * 0.1f);
                freq_zoom = std::max(1.0f, std::min(10.0f, freq_zoom));
                
                float new_width = total_range / freq_zoom;
                float new_start = freq_mouse - (mx * new_width);
                freq_pan = (new_start + nyquist) / total_range;
                freq_pan = std::max(0.0f, std::min(1.0f - 1.0f / freq_zoom, freq_pan));
            }
        }
    }

    void draw_power_grid(ImDrawList *draw_list, ImVec2 pos, ImVec2 size, float range) {
        ImU32 col = IM_COL32(150, 150, 150, 150);
        for (int i = 0; i <= 12; i++) {
            float ny = i * 10.0f / range;
            if (ny > 1.0f) break;
            float y = pos.y + size.y * (1.0f - ny);
            draw_list->AddLine(ImVec2(pos.x, y), ImVec2(pos.x + size.x, y), col, 1.0f);
            char label[16];
            snprintf(label, sizeof(label), "%.0f", display_power_min + i * 10.0f);
            draw_list->AddText(ImVec2(pos.x - 35, y - 8), IM_COL32(0, 255, 0, 255), label);
        }
    }

    void draw_freq_grid(ImDrawList *draw_list, ImVec2 pos, ImVec2 size, float start, float end) {
        ImU32 col = IM_COL32(150, 150, 150, 150);
        float range = end - start;
        
        int tick_start = static_cast<int>(std::floor(start));
        int tick_end = static_cast<int>(std::ceil(end));
        
        for (int tick = tick_start; tick <= tick_end; tick++) {
            float nx = (tick - start) / range;
            if (nx < -0.05f || nx > 1.05f) continue;
            nx = std::max(0.0f, std::min(1.0f, nx));
            float x = pos.x + nx * size.x;
            draw_list->AddLine(ImVec2(x, pos.y), ImVec2(x, pos.y + size.y), col, 1.0f);
            float abs_f = tick + header.center_frequency / 1e6f;
            char label[16];
            snprintf(label, sizeof(label), "%.0f", abs_f);
            draw_list->AddText(ImVec2(x - 15, pos.y + size.y + 5), IM_COL32(0, 255, 0, 255), label);
        }
    }

    void draw_waterfall_canvas(ImDrawList *draw_list, ImVec2 plot_pos, ImVec2 plot_size) {
        draw_list->AddRectFilled(plot_pos, ImVec2(plot_pos.x + plot_size.x, plot_pos.y + plot_size.y), 
                                 IM_COL32(10, 10, 10, 255));

        float nyquist = header.sample_rate / 2.0f / 1e6f;
        float total_range = 2.0f * nyquist;
        float disp_start = -nyquist + freq_pan * total_range;
        float disp_width = total_range / freq_zoom;
        float disp_end = disp_start + disp_width;
        
        disp_start = std::max(-nyquist, disp_start);
        disp_end = std::min(nyquist, disp_end);

        int display_rows = std::min(static_cast<int>(header.num_ffts), static_cast<int>(plot_size.y / 2));
        int start_row = current_fft_idx - display_rows + 1;
        if (start_row < 0) start_row = 0;

        float sr_mhz = header.sample_rate / 1e6f;
        int start_bin = get_bin_from_freq(disp_start, sr_mhz);
        int end_bin = get_bin_from_freq(disp_end, sr_mhz);
        
        if (start_bin > end_bin) std::swap(start_bin, end_bin);
        end_bin = std::min(end_bin + 1, static_cast<int>(header.fft_size));

        float bin_to_x_scale = plot_size.x / (end_bin - start_bin);

        for (int row = 0; row < display_rows; row++) {
            int fft_idx = start_row + row;
            if (fft_idx < 0 || fft_idx >= static_cast<int>(header.num_ffts)) continue;

            const int8_t *spec = fft_data.data() + fft_idx * header.fft_size;
            float row_y = plot_pos.y + (display_rows - 1 - row) * plot_size.y / display_rows;
            float row_h = plot_size.y / display_rows;

            for (int bin = start_bin; bin < end_bin; bin++) {
                float p = dequantize(spec[bin]);
                float power_range = display_power_max - display_power_min;
                float np = (p - display_power_min) / power_range;
                np = std::max(0.0f, std::min(1.0f, np));

                float bx = plot_pos.x + (bin - start_bin) * bin_to_x_scale;
                float bw = bin_to_x_scale + 1.0f;
                draw_list->AddRectFilled(ImVec2(bx, row_y), ImVec2(bx + bw, row_y + row_h), get_color(np));
            }
        }
    }

    ImU32 get_color(float v) {
        v = std::max(0.0f, std::min(1.0f, v));
        float r, g, b;
        if (v < 0.25f) { r=0; g=0; b=v/0.25f; }
        else if (v < 0.5f) { r=0; g=(v-0.25f)/0.25f; b=1.0f; }
        else if (v < 0.75f) { r=(v-0.5f)/0.25f; g=1.0f; b=1.0f-(v-0.5f)/0.25f; }
        else { r=1.0f; g=1.0f-(v-0.75f)/0.25f; b=0; }
        return IM_COL32(r*255, g*255, b*255, 255);
    }

    float dequantize(int8_t v) {
        return header.power_min + (v / 127.0f) * (header.power_max - header.power_min);
    }
};

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <fftdata_file>\n", argv[0]);
        return 1;
    }

    FFTViewer viewer;
    if (!viewer.load_file(argv[1])) return 1;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(1400, 900, viewer.window_title.c_str(), nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    glewInit();

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);

        if (ImGui::Begin("FFT Viewer", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
            if (ImGui::SliderInt("FFT Index", &viewer.current_fft_idx, 0, viewer.header.num_ffts - 1)) {}
            ImGui::Separator();

            float w = ImGui::GetContentRegionAvail().x;
            float total_h = ImGui::GetIO().DisplaySize.y - 140;
            float divider_h = 10.0f;
            float h1 = (total_h - divider_h) * viewer.spectrum_height_ratio;
            float h2 = (total_h - divider_h) * (1.0f - viewer.spectrum_height_ratio);

            if (ImGui::CollapsingHeader("Power Spectrum", ImGuiTreeNodeFlags_DefaultOpen)) {
                viewer.draw_spectrum(w, h1);
            }
            
            ImVec2 divider_pos = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton("divider", ImVec2(w, divider_h));
            
            ImDrawList *draw_list = ImGui::GetWindowDrawList();
            draw_list->AddLine(ImVec2(divider_pos.x, divider_pos.y + divider_h/2), 
                              ImVec2(divider_pos.x + w, divider_pos.y + divider_h/2), 
                              IM_COL32(100, 100, 100, 200), 2.0f);
            
            if (ImGui::IsItemHovered()) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
            }
            
            if (ImGui::IsItemActive()) {
                ImGuiIO& io = ImGui::GetIO();
                float delta = io.MouseDelta.y;
                viewer.spectrum_height_ratio += delta / total_h;
                viewer.spectrum_height_ratio = std::max(0.2f, std::min(0.8f, viewer.spectrum_height_ratio));
            }
            
            if (ImGui::CollapsingHeader("Waterfall", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushItemWidth(25);
                ImGui::VSliderFloat("##max", ImVec2(25, h2 * 0.45f), &viewer.display_power_max, 
                                   viewer.header.power_min, viewer.header.power_max, "");
                ImGui::SameLine();
                
                ImDrawList *wf_draw = ImGui::GetWindowDrawList();
                ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
                ImVec2 canvas_size(w - 40, h2);
                
                viewer.draw_waterfall_canvas(wf_draw, canvas_pos, canvas_size);
                ImGui::InvisibleButton("waterfall", canvas_size);
                
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() - h2 * 0.45f);
                ImGui::VSliderFloat("##min", ImVec2(25, h2 * 0.45f), &viewer.display_power_min, 
                                   viewer.header.power_min, viewer.header.power_max, "");
                ImGui::PopItemWidth();
            }
            ImGui::End();
        }

        ImGui::Render();
        int dw, dh;
        glfwGetFramebufferSize(window, &dw, &dh);
        glViewport(0, 0, dw, dh);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();
    return 0;
}