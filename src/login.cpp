#include "login.hpp"
#include "bewe_paths.hpp"
#include <GL/glew.h>
#include <imgui.h>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <png.h>

static GLuint bg_tex[3] = {0, 0, 0};
static int    bg_w  [3] = {0, 0, 0};
static int    bg_h  [3] = {0, 0, 0};
static std::string BG_PATH_STRS[3];
static const char*  BG_PATHS[3] = {nullptr,nullptr,nullptr};
static void init_bg_paths(){
    static bool done=false; if(done) return; done=true;
    std::string a=BEWEPaths::assets_dir();
    BG_PATH_STRS[0]=a+"/login_bg_Tier_1.png";
    BG_PATH_STRS[1]=a+"/login_bg_Tier_2.png";
    BG_PATH_STRS[2]=a+"/login_bg_Tier_3.png";
    for(int i=0;i<3;i++) BG_PATHS[i]=BG_PATH_STRS[i].c_str();
}

static bool load_png(const char* path, GLuint& tex, int& w, int& h){
    FILE* fp = fopen(path,"rb");
    if(!fp) return false;
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING,nullptr,nullptr,nullptr);
    if(!png){ fclose(fp); return false; }
    png_infop info = png_create_info_struct(png);
    if(!info){ png_destroy_read_struct(&png,nullptr,nullptr); fclose(fp); return false; }
    if(setjmp(png_jmpbuf(png))){ png_destroy_read_struct(&png,&info,nullptr); fclose(fp); return false; }
    png_init_io(png,fp);
    png_read_info(png,info);
    w=(int)png_get_image_width(png,info);
    h=(int)png_get_image_height(png,info);
    png_byte color_type=png_get_color_type(png,info);
    png_byte bit_depth =png_get_bit_depth(png,info);
    if(bit_depth==16) png_set_strip_16(png);
    if(color_type==PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if(color_type==PNG_COLOR_TYPE_GRAY && bit_depth<8) png_set_expand_gray_1_2_4_to_8(png);
    if(png_get_valid(png,info,PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);
    if(color_type==PNG_COLOR_TYPE_RGB||color_type==PNG_COLOR_TYPE_GRAY||
       color_type==PNG_COLOR_TYPE_PALETTE) png_set_filler(png,0xFF,PNG_FILLER_AFTER);
    if(color_type==PNG_COLOR_TYPE_GRAY||color_type==PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png);
    png_read_update_info(png,info);
    unsigned char* pixels=new unsigned char[w*h*4];
    png_bytep* rows=new png_bytep[h];
    for(int y=0;y<h;y++) rows[y]=pixels+y*w*4;
    png_read_image(png,rows);
    delete[] rows;
    png_destroy_read_struct(&png,&info,nullptr);
    fclose(fp);
    glGenTextures(1,&tex);
    glBindTexture(GL_TEXTURE_2D,tex);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,pixels);
    glBindTexture(GL_TEXTURE_2D,0);
    delete[] pixels;
    return true;
}

bool draw_login_screen(int win_w, int win_h){
    static char id_buf[64]  ={};
    static char pw_buf[64]  ={};
    static int  tier        =1;
    static bool failed      =false;
    static float fail_timer =0.0f;
    init_bg_paths();
    static bool bg_tried[3] ={false,false,false};

    // ── 이미지 페이드 전환 ────────────────────────────────────────────────
    static int  prev_ti     = 0;   // 이전 티어 인덱스
    static float fade_alpha = 1.0f;// 새 이미지 불투명도 (0→1 페이드인)
    static bool fading      = false;

    int ti = tier - 1; // 0~2

    // 티어 변경 감지 → 페이드 시작
    if(ti != prev_ti){
        if(!fading){ fading=true; fade_alpha=0.0f; }
    }
    if(fading){
        fade_alpha += ImGui::GetIO().DeltaTime * 3.0f; // ~0.33초
        if(fade_alpha >= 1.0f){ fade_alpha=1.0f; fading=false; prev_ti=ti; }
    }

    // 현재/이전 티어 이미지 로드
    for(int i=0;i<3;i++){
        if(i==ti || i==prev_ti){
            if(!bg_tried[i]){
                bg_tried[i]=true;
                if(!load_png(BG_PATHS[i],bg_tex[i],bg_w[i],bg_h[i]))
                    printf("Login: bg not found: %s\n",BG_PATHS[i]);
            }
        }
    }

    // 배경 이미지 렌더 헬퍼
    auto draw_bg=[&](int idx, float alpha){
        if(!bg_tex[idx]||bg_w[idx]<=0) return;
        float ia=(float)bg_w[idx]/(float)bg_h[idx], wa=(float)win_w/(float)win_h;
        float dx,dy,dw,dh;
        if(wa>ia){ dw=(float)win_w; dh=dw/ia; dx=0; dy=((float)win_h-dh)*0.5f; }
        else      { dh=(float)win_h; dw=dh*ia; dy=0; dx=((float)win_w-dw)*0.5f; }
        ImU32 col=IM_COL32(255,255,255,(int)(alpha*255));
        ImGui::GetWindowDrawList()->AddImage((ImTextureID)(intptr_t)bg_tex[idx],
            ImVec2(dx,dy),ImVec2(dx+dw,dy+dh),ImVec2(0,0),ImVec2(1,1),col);
    };

    // ── 전체 배경 ────────────────────────────────────────────────────────
    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2((float)win_w,(float)win_h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,   ImVec2(0,0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("##login_bg",nullptr,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoInputs);
    ImDrawList* dl=ImGui::GetWindowDrawList();

    // 폴백 배경 (이미지 없을 때)
    bool any_img = (bg_tex[ti]&&bg_w[ti]>0)||(bg_tex[prev_ti]&&bg_w[prev_ti]>0);
    if(!any_img){
        dl->AddRectFilled(ImVec2(0,0),ImVec2((float)win_w,(float)win_h),IM_COL32(12,18,35,255));
        for(int x=0;x<win_w;x+=60)
            dl->AddLine(ImVec2((float)x,0),ImVec2((float)x,(float)win_h),IM_COL32(20,35,65,160),1);
        for(int y=0;y<win_h;y+=60)
            dl->AddLine(ImVec2(0,(float)y),ImVec2((float)win_w,(float)y),IM_COL32(20,35,65,160),1);
    }
    // 이전 이미지 (1-fade) → 새 이미지 (fade) 크로스페이드
    if(fading){
        draw_bg(prev_ti, 1.0f - fade_alpha);
        draw_bg(ti,      fade_alpha);
    } else {
        draw_bg(ti, 1.0f);
    }

    ImGui::End();
    ImGui::PopStyleVar(2);

    // ── 로그인 패널 ──────────────────────────────────────────────────────
    const float PW_=330.0f, PH_=208.0f, PAD=28.0f;
    ImGui::SetNextWindowPos(ImVec2((float)win_w-PW_-PAD,(float)win_h-PH_-PAD));
    ImGui::SetNextWindowSize(ImVec2(PW_,PH_));
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,  4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,    ImVec2(8,9));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,1.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.06f,0.08f,0.14f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,       ImVec4(0.10f,0.13f,0.22f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,ImVec4(0.14f,0.18f,0.30f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f,0.36f,0.70f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f,0.50f,0.90f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.10f,0.28f,0.58f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_CheckMark,     ImVec4(0.40f,0.80f,1.0f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ImVec4(0.22f,0.32f,0.58f,0.9f));
    ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(0.88f,0.92f,1.00f,1.0f));

    bool login_done=false;
    ImGui::Begin("##login_panel",nullptr,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoCollapse);

    // ── 티어별 타이틀 ────────────────────────────────────────────────────
    static const char* TIER_TITLES[3]={
        "Behind Everyone We Hear Everything",
        "BlackWave",
        "SHADE"
    };
    { // 타이틀 영역: 1줄 고정
      ImGui::SetWindowFontScale(1.3f);
      float lh = ImGui::GetTextLineHeight();
      float win_pad_y = ImGui::GetStyle().WindowPadding.y;
      const float TITLE_H = lh + 12.0f; // 1줄+위아래패딩
      float base_y = ImGui::GetCursorPosY();
      ImGui::Dummy(ImVec2(PW_-16.0f, TITLE_H));
      ImGui::SetCursorPosY(base_y);
      const char* t = TIER_TITLES[ti];
      float tw = ImGui::CalcTextSize(t).x;
      float offset = (TITLE_H - lh - win_pad_y) * 0.5f;
      float oy = base_y + std::max(0.0f, offset);
      ImGui::SetCursorPos(ImVec2(std::max(4.0f,(PW_-tw)*0.5f), oy));
      ImGui::TextColored(ImVec4(0.50f,0.78f,1.00f,1.0f),"%s",t);
      ImGui::SetWindowFontScale(1.0f);
      ImGui::SetCursorPosY(base_y + TITLE_H); }
    ImGui::Separator(); ImGui::Spacing();

    // ── 티어 버튼 중앙정렬 (Security Tier 텍스트 제거) ───────────────────
    {
        float r1w=ImGui::CalcTextSize("Tier 1").x+20.0f; // RadioButton 폭 근사
        float r2w=ImGui::CalcTextSize("Tier 2").x+20.0f;
        float r3w=ImGui::CalcTextSize("Tier 3").x+20.0f;
        float spacing=ImGui::GetStyle().ItemSpacing.x;
        float total_w=r1w+r2w+r3w+spacing*2;
        float rx=(PW_-total_w)*0.5f;
        if(rx<4)rx=4;
        ImGui::SetCursorPosX(rx);
    }
    if(ImGui::RadioButton("Tier 1",tier==1)) tier=1;
    ImGui::SameLine();
    if(ImGui::RadioButton("Tier 2",tier==2)) tier=2;
    ImGui::SameLine();
    if(ImGui::RadioButton("Tier 3",tier==3)) tier=3;

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    ImGui::SetNextItemWidth(PW_-16.0f);
    ImGui::InputTextWithHint("##id","Enter ID",id_buf,sizeof(id_buf));
    ImGui::Spacing();

    ImGui::SetNextItemWidth(PW_-16.0f);
    bool enter_pw=ImGui::InputTextWithHint("##pw","Enter Password",pw_buf,sizeof(pw_buf),
        ImGuiInputTextFlags_Password|ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::Spacing();

    if(failed){
        fail_timer-=ImGui::GetIO().DeltaTime;
        if(fail_timer<=0.0f) failed=false;
        ImGui::TextColored(ImVec4(1.0f,0.35f,0.35f,1.0f),"ID or PW cannot be empty.");
    } else {
        ImGui::Dummy(ImVec2(0,13));
    }

    ImGui::SetCursorPosX((PW_-110.0f)*0.5f);
    bool do_login=ImGui::Button("LOGIN",ImVec2(110,26))||enter_pw;
    if(do_login){
        if(id_buf[0]=='\0'||pw_buf[0]=='\0'){ failed=true; fail_timer=2.5f; }
        else login_done=true;
    }

    ImGui::End();
    ImGui::PopStyleColor(9);
    ImGui::PopStyleVar(4);
    return login_done;
}