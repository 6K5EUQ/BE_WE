#include "session_spawn.hpp"
#include "login.hpp"
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

extern char** environ;

static std::string self_exe_path(){
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf)-1);
    if(n <= 0) return "BE_WE";
    buf[n] = '\0';
    return std::string(buf);
}

pid_t spawn_session_child(const std::string& mode,
                          const std::string& station_id,
                          const std::string& station_name,
                          float              station_lat,
                          float              station_lon){
    std::string exe = self_exe_path();
    char lat_buf[32], lon_buf[32];
    snprintf(lat_buf, sizeof(lat_buf), "%.6f", station_lat);
    snprintf(lon_buf, sizeof(lon_buf), "%.6f", station_lon);

    // Build argv. Strings live until posix_spawn returns.
    std::string a_mode  = "--session-mode=" + mode;
    std::string a_sid   = "--station-id="   + station_id;
    std::string a_sname = "--station-name=" + station_name;
    std::string a_slat  = "--station-lat="  + std::string(lat_buf);
    std::string a_slon  = "--station-lon="  + std::string(lon_buf);

    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(exe.c_str()));
    argv.push_back(const_cast<char*>(a_mode.c_str()));
    if(!station_id.empty())
        argv.push_back(const_cast<char*>(a_sid.c_str()));
    argv.push_back(const_cast<char*>(a_sname.c_str()));
    argv.push_back(const_cast<char*>(a_slat.c_str()));
    argv.push_back(const_cast<char*>(a_slon.c_str()));
    argv.push_back(nullptr);

    // Forward login token via env vars (login.cpp reads BEWE_AUTO_*).
    setenv("BEWE_AUTO_ID",     login_get_id()     ? login_get_id()     : "", 1);
    setenv("BEWE_AUTO_PW",     login_get_pw()     ? login_get_pw()     : "", 1);
    char tier_buf[8];
    snprintf(tier_buf, sizeof(tier_buf), "%d", login_get_tier());
    setenv("BEWE_AUTO_TIER",   tier_buf, 1);
    setenv("BEWE_AUTO_SERVER", login_get_server() ? login_get_server() : "", 1);

    pid_t pid = -1;
    int rc = posix_spawn(&pid, exe.c_str(), nullptr, nullptr,
                         argv.data(), environ);
    if(rc != 0){
        fprintf(stderr, "[BEWE] spawn_session_child failed: %s\n", strerror(rc));
        return -1;
    }
    return pid;
}

void reap_finished_children(std::vector<ChildSession>& sessions,
                            pid_t&                     active_host_pid){
    for(;;){
        int status = 0;
        pid_t pid = waitpid(-1, &status, WNOHANG);
        if(pid <= 0) break;
        for(auto it = sessions.begin(); it != sessions.end(); ++it){
            if(it->pid == pid){
                if(active_host_pid == pid) active_host_pid = 0;
                sessions.erase(it);
                break;
            }
        }
    }
}

void kill_all_children(const std::vector<ChildSession>& sessions){
    for(const auto& s : sessions){
        if(s.pid > 0) kill(s.pid, SIGTERM);
    }
}
