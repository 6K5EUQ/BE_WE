#pragma once
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>
#include <limits.h>

// ── BE_WE 런타임 경로 ─────────────────────────────────────────────────────
// assets : 실행 파일 옆 (AppImage 내부 or 설치 경로)
// data   : $HOME/.local/share/BE_WE/ (녹음, 임시파일)
namespace BEWEPaths {

// 실행 파일의 절대 경로 디렉터리 반환
static inline std::string exe_dir(){
    char buf[PATH_MAX]={};
    ssize_t n=readlink("/proc/self/exe",buf,sizeof(buf)-1);
    if(n<=0) return ".";
    std::string p(buf,n);
    auto pos=p.rfind('/');
    return (pos==std::string::npos)?".":p.substr(0,pos);
}

// AppImage: APPDIR env 설정됨 → assets는 $APPDIR/usr/share/BE_WE/assets
// 일반 실행: exe 옆 assets/ → 없으면 ../assets → 없으면 ~/BE_WE/assets 순으로 탐색
static inline std::string assets_dir(){
    const char* appdir=getenv("APPDIR");
    if(appdir){
        return std::string(appdir)+"/usr/share/BE_WE/assets";
    }
    // 후보 경로 순서대로 탐색
    std::string candidates[]={
        exe_dir()+"/assets",          // 실행파일 옆
        exe_dir()+"/../assets",       // build/ 한 단계 위 (cmake 빌드)
        exe_dir()+"/../../assets",    // 두 단계 위
    };
    for(auto& p : candidates){
        struct stat st{};
        if(stat(p.c_str(),&st)==0 && S_ISDIR(st.st_mode))
            return p;
    }
    // 최후 폴백: ~/BE_WE/assets
    const char* home=getenv("HOME");
    if(home) return std::string(home)+"/BE_WE/assets";
    return exe_dir()+"/assets";
}

// 사용자 데이터: $HOME/.local/share/BE_WE/
static inline std::string data_dir(){
    const char* home=getenv("HOME");
    std::string base=(home?std::string(home):std::string("/tmp"))+
                     "/.local/share/BE_WE";
    return base;
}

static inline std::string recordings_dir(){
    return data_dir()+"/recordings";
}

static inline std::string time_temp_dir(){
    return recordings_dir()+"/Time_temp";
}

static inline std::string sa_temp_dir(){
    return recordings_dir()+"/SA_Temp";
}

// 디렉터리 없으면 자동 생성
static inline void ensure_dirs(){
    auto mk=[](const std::string& p){
        mkdir(p.c_str(),0755);
    };
    mk(data_dir());
    mk(recordings_dir());
    mk(time_temp_dir());
    mk(sa_temp_dir());
}

} // namespace BEWEPaths