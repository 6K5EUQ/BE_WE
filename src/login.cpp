#include "login.hpp"
#include "bewe_paths.hpp"
#include <GL/glew.h>
#include <imgui.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <png.h>

// ── 계정 getter ───────────────────────────────────────────────────────────
static char g_login_id  [64] = {};
static char g_login_pw  [64] = {};
static int  g_login_tier     = 1;

const char* login_get_id()   { return g_login_id; }
const char* login_get_pw()   { return g_login_pw; }
int         login_get_tier() { return g_login_tier; }

// ── 배경 텍스처 (티어별) ──────────────────────────────────────────────────
static GLuint bg_tex[3] = {0,0,0};
static int    bg_w  [3] = {0,0,0};
static int    bg_h  [3] = {0,0,0};
static std::string BG_PATH_STRS[3];
static const char* BG_PATHS[3] = {nullptr,nullptr,nullptr};

static void init_bg_paths(){
    static bool done=false; if(done) return; done=true;
    std::string a=BEWEPaths::assets_dir();
    BG_PATH_STRS[0]=a+"/login_bg_Tier_1.png";
    BG_PATH_STRS[1]=a+"/login_bg_Tier_2.png";
    BG_PATH_STRS[2]=a+"/login_bg_Tier_3.png";
    for(int i=0;i<3;i++) BG_PATHS[i]=BG_PATH_STRS[i].c_str();
}

static bool load_png(const char* path, GLuint& tex, int& w, int& h){
    FILE* fp=fopen(path,"rb"); if(!fp) return false;
    png_structp png=png_create_read_struct(PNG_LIBPNG_VER_STRING,nullptr,nullptr,nullptr);
    if(!png){fclose(fp);return false;}
    png_infop info=png_create_info_struct(png);
    if(!info){png_destroy_read_struct(&png,nullptr,nullptr);fclose(fp);return false;}
    if(setjmp(png_jmpbuf(png))){png_destroy_read_struct(&png,&info,nullptr);fclose(fp);return false;}
    png_init_io(png,fp);
    png_read_info(png,info);
    w=(int)png_get_image_width(png,info);
    h=(int)png_get_image_height(png,info);
    png_byte ct=png_get_color_type(png,info);
    png_byte bd=png_get_bit_depth(png,info);
    if(bd==16) png_set_strip_16(png);
    if(ct==PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if(ct==PNG_COLOR_TYPE_GRAY&&bd<8) png_set_expand_gray_1_2_4_to_8(png);
    if(png_get_valid(png,info,PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);
    if(ct==PNG_COLOR_TYPE_RGB||ct==PNG_COLOR_TYPE_GRAY||ct==PNG_COLOR_TYPE_PALETTE)
        png_set_filler(png,0xFF,PNG_FILLER_AFTER);
    if(ct==PNG_COLOR_TYPE_GRAY||ct==PNG_COLOR_TYPE_GRAY_ALPHA)
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
    static int  tier        =3;
    static bool failed      =false;
    static float fail_timer =0.0f;
    static bool bg_tried[3] ={false,false,false};
    static int  prev_ti     =2;
    static float fade_alpha =1.0f;
    static bool fading      =false;
    static bool auto_checked =false;
    init_bg_paths();

    // /restart로 재실행된 경우: 환경변수로 자동 로그인
    if(!auto_checked){
        auto_checked = true;
        const char* a_id   = getenv("BEWE_AUTO_ID");
        const char* a_pw   = getenv("BEWE_AUTO_PW");
        const char* a_tier = getenv("BEWE_AUTO_TIER");
        if(a_id && a_id[0]){
            strncpy(g_login_id, a_id, 63);
            strncpy(g_login_pw, a_pw ? a_pw : "", 63);
            g_login_tier = a_tier ? atoi(a_tier) : 3;
            unsetenv("BEWE_AUTO_ID"); unsetenv("BEWE_AUTO_PW"); unsetenv("BEWE_AUTO_TIER");
            return true; // 로그인 화면 건너뜀
        }
    }

    int ti=tier-1;
    if(!bg_tried[ti]){
        bg_tried[ti]=true;
        if(!load_png(BG_PATHS[ti],bg_tex[ti],bg_w[ti],bg_h[ti]))
            printf("Login: bg not found: %s\n",BG_PATHS[ti]);
    }
    if(ti!=prev_ti&&!fading){ fading=true; fade_alpha=0.0f; prev_ti=ti; }
    if(fading){ fade_alpha+=ImGui::GetIO().DeltaTime*3.0f; if(fade_alpha>=1.0f){fade_alpha=1.0f;fading=false;} }

    // ── 배경 ─────────────────────────────────────────────────────────────
    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2((float)win_w,(float)win_h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(0,0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,0.0f);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("##login_bg",nullptr,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|
        ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoInputs);
    ImDrawList* dl=ImGui::GetWindowDrawList();
    auto draw_bg=[&](int idx,float alpha){
        if(bg_tex[idx]&&bg_w[idx]>0){
            float ia=(float)bg_w[idx]/(float)bg_h[idx], wa=(float)win_w/(float)win_h;
            float dx,dy,dw,dh;
            if(wa>ia){dw=(float)win_w;dh=dw/ia;dx=0;dy=((float)win_h-dh)*0.5f;}
            else     {dh=(float)win_h;dw=dh*ia;dy=0;dx=((float)win_w-dw)*0.5f;}
            dl->AddImage((ImTextureID)(intptr_t)bg_tex[idx],
                ImVec2(dx,dy),ImVec2(dx+dw,dy+dh),
                ImVec2(0,0),ImVec2(1,1),IM_COL32(255,255,255,(int)(alpha*255)));
        } else {
            dl->AddRectFilled(ImVec2(0,0),ImVec2((float)win_w,(float)win_h),IM_COL32(12,18,35,255));
            for(int x=0;x<win_w;x+=60)
                dl->AddLine(ImVec2((float)x,0),ImVec2((float)x,(float)win_h),IM_COL32(20,35,65,160),1);
            for(int y=0;y<win_h;y+=60)
                dl->AddLine(ImVec2(0,(float)y),ImVec2((float)win_w,(float)y),IM_COL32(20,35,65,160),1);
        }
    };
    draw_bg(ti,fade_alpha);
    ImGui::End();
    ImGui::PopStyleVar(2);

    // ── 로그인 패널 ──────────────────────────────────────────────────────
    bool is_t3 = (tier == 3);
    const float PW_=290.0f,PH_=(is_t3?220.0f:262.0f),PAD=28.0f;
    ImGui::SetNextWindowPos(ImVec2((float)win_w-PW_-PAD,(float)win_h-PH_-PAD));
    ImGui::SetNextWindowSize(ImVec2(PW_,PH_));
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,4.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,ImVec2(8,9));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,1.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg,     ImVec4(0.06f,0.08f,0.14f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,      ImVec4(0.10f,0.13f,0.22f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,ImVec4(0.14f,0.18f,0.30f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_Button,       ImVec4(0.16f,0.36f,0.70f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,ImVec4(0.26f,0.50f,0.90f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.10f,0.28f,0.58f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_CheckMark,    ImVec4(0.40f,0.80f,1.0f,1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border,       ImVec4(0.22f,0.32f,0.58f,0.9f));
    ImGui::PushStyleColor(ImGuiCol_Text,         ImVec4(0.88f,0.92f,1.00f,1.0f));

    bool login_done=false;
    ImGui::Begin("##login_panel",nullptr,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
        ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoCollapse);

    {const char* t="B E W E";
     ImGui::SetCursorPosX((PW_-ImGui::CalcTextSize(t).x)*0.5f);
     ImGui::TextColored(ImVec4(0.50f,0.78f,1.00f,1.0f),"%s",t);}
    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    ImGui::TextColored(ImVec4(0.6f,0.7f,0.9f,1.0f),"Security Tier");
    float rx=(PW_-(ImGui::CalcTextSize("Tier 1").x+
                   ImGui::CalcTextSize("Tier 2").x+
                   ImGui::CalcTextSize("Tier 3").x+64.0f))*0.5f;
    if(rx<4)rx=4;
    ImGui::SetCursorPosX(rx);
    if(ImGui::RadioButton("Tier 1",tier==1)) tier=1;
    ImGui::SameLine();
    if(ImGui::RadioButton("Tier 2",tier==2)) tier=2;
    ImGui::SameLine();
    if(ImGui::RadioButton("Tier 3",tier==3)) tier=3;

    ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

    ImGui::Text("ID");
    ImGui::SetNextItemWidth(PW_-16.0f);
    ImGui::InputText("##id",id_buf,sizeof(id_buf));
    ImGui::Spacing();

    bool enter_pw = false;
    if(!is_t3){
        ImGui::Text("PW");
        ImGui::SetNextItemWidth(PW_-16.0f);
        enter_pw=ImGui::InputText("##pw",pw_buf,sizeof(pw_buf),
            ImGuiInputTextFlags_Password|ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::Spacing();
    } else {
        // Tier 3: Enter on ID field acts as login trigger
        enter_pw = ImGui::IsKeyPressed(ImGuiKey_Enter,false) ||
                   ImGui::IsKeyPressed(ImGuiKey_KeypadEnter,false);
        pw_buf[0] = '\0'; // no password for Tier 3
    }

    if(failed){
        fail_timer-=ImGui::GetIO().DeltaTime;
        if(fail_timer<=0.0f) failed=false;
        ImGui::TextColored(ImVec4(1.0f,0.35f,0.35f,1.0f),"ID cannot be empty.");
    } else {
        ImGui::Dummy(ImVec2(0,13));
    }

    ImGui::SetCursorPosX((PW_-110.0f)*0.5f);
    bool do_login=ImGui::Button("LOGIN",ImVec2(110,26))||enter_pw;
    if(do_login){
        if(id_buf[0]=='\0'){ failed=true; fail_timer=2.5f; }
        else if(!is_t3 && pw_buf[0]=='\0'){ failed=true; fail_timer=2.5f; }
        else {
            strncpy(g_login_id, id_buf, 63);
            strncpy(g_login_pw, pw_buf, 63);
            g_login_tier=tier;
            login_done=true;
        }
    }

    ImGui::End();
    ImGui::PopStyleColor(9);
    ImGui::PopStyleVar(4);
    return login_done;
}