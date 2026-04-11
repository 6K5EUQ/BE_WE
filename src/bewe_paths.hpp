#pragma once
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>
#include <limits.h>

// ── BE_WE 런타임 경로 ─────────────────────────────────────────────────────
// assets : 실행 파일 옆 (AppImage 내부 or 설치 경로)
// data   : $HOME/.local/share/BE_WE/ (녹음, 임시파일)
//
// 폴더 구조:
//   recordings/
//     record/iq/       ← 세션 중 실시간 녹음 (IQ)
//     record/audio/    ← 세션 중 실시간 녹음 (Audio)
//     private/iq/      ← 이전 세션 녹음 (프로그램 종료 시 record에서 이동)
//     private/audio/
//     public/iq/       ← Public으로 올린 파일 (서버 공유)
//     public/audio/
//     share/iq/        ← Public에서 다운받은 파일 (JOIN)
//     share/audio/
//     Time_temp/       ← TM IQ rolling (기존 유지)

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
    std::string candidates[]={
        exe_dir()+"/assets",
        exe_dir()+"/../assets",
        exe_dir()+"/../../assets",
    };
    for(auto& p : candidates){
        struct stat st{};
        if(stat(p.c_str(),&st)==0 && S_ISDIR(st.st_mode))
            return p;
    }
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
    const char* home=getenv("HOME");
    return (home?std::string(home):std::string("/tmp"))+"/BE_WE/recordings";
}

// ── 세션 중 실시간 녹음 폴더 ─────────────────────────────────────────────
static inline std::string record_dir()       { return recordings_dir()+"/record"; }
static inline std::string record_iq_dir()    { return record_dir()+"/iq"; }
static inline std::string record_audio_dir() { return record_dir()+"/audio"; }

// ── 이전 세션 녹음 폴더 (프로그램 종료 시 record에서 이동) ──────────────
static inline std::string private_dir()       { return recordings_dir()+"/private"; }
static inline std::string private_iq_dir()    { return private_dir()+"/iq"; }
static inline std::string private_audio_dir() { return private_dir()+"/audio"; }

// ── Public 공유 폴더 (서버 업로드) ───────────────────────────────────────
static inline std::string public_dir()       { return recordings_dir()+"/public"; }
static inline std::string public_iq_dir()    { return public_dir()+"/iq"; }
static inline std::string public_audio_dir() { return public_dir()+"/audio"; }

// ── 다운로드된 Public 파일 (JOIN 수신) ───────────────────────────────────
static inline std::string share_dir()       { return recordings_dir()+"/share"; }
static inline std::string share_iq_dir()    { return share_dir()+"/iq"; }
static inline std::string share_audio_dir() { return share_dir()+"/audio"; }

// ── Report 파일 (전 오퍼레이터 공유) ─────────────────────────────────────
static inline std::string report_dir()       { return recordings_dir()+"/report"; }
static inline std::string report_iq_dir()    { return report_dir()+"/iq"; }
static inline std::string report_audio_dir() { return report_dir()+"/audio"; }

// ── Database (Central Server 로컬 저장) ──────────────────────────────────
// Central server의 ./BE_WE/DataBase/{operator}/ 에 저장
// 클라이언트 측에서도 로컬 DB 캐시로 사용
static inline std::string database_dir(){
    const char* home = getenv("HOME");
    return home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";
}

// ── 기존 임시 폴더 (로직 유지) ───────────────────────────────────────────
static inline std::string time_temp_dir(){
    return recordings_dir()+"/Time_temp";
}

// 디렉터리 없으면 자동 생성
static inline void ensure_dirs(){
    auto mk=[](const std::string& p){ mkdir(p.c_str(),0755); };
    mk(data_dir());
    mk(recordings_dir());
    mk(record_dir());    mk(record_iq_dir());    mk(record_audio_dir());
    mk(private_dir());   mk(private_iq_dir());   mk(private_audio_dir());
    // public/share 폴더는 더 이상 사용하지 않으므로 자동 생성하지 않음
    mk(report_dir());   mk(report_iq_dir());   mk(report_audio_dir());
    mk(database_dir());
    mk(time_temp_dir());
}

} // namespace BEWEPaths
