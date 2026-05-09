#pragma once
#include <string>
#include <vector>
#include <sys/types.h>

// Tracks a child process spawned by the parent globe to host an
// independent JOIN/HOST session in a separate OS-level window.
struct ChildSession {
    pid_t       pid = 0;
    std::string mode;          // "host" or "join"
    std::string station_name;  // display name
    std::string station_id;    // Central room id (JOIN); empty for HOST
};

// posix_spawn the same binary with --session-mode flags. Login credentials
// are forwarded via BEWE_AUTO_* env vars (login.cpp picks them up).
// Returns child pid on success, or -1 on failure.
pid_t spawn_session_child(const std::string& mode,
                          const std::string& station_id,
                          const std::string& station_name,
                          float              station_lat,
                          float              station_lon);

// Reap any child processes that have exited (non-blocking). Removes them
// from `sessions` and clears `active_host_pid` if the dead child was HOST.
void reap_finished_children(std::vector<ChildSession>& sessions,
                            pid_t&                     active_host_pid);

// Send SIGTERM to all live children. Used when the parent globe exits.
void kill_all_children(const std::vector<ChildSession>& sessions);
