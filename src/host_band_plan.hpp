#pragma once
// Host-owned band plan store. Each HOST machine owns ~/BE_WE/band_plan.json.
// On HOST start: load_from_file() + rebuild_cache().
// On JOIN connect (Central CONN_OPEN): broadcast g_cached_pkt to that conn (or all).
// On JOIN/HOST edit: apply_*() → save_to_file() → rebuild_cache() → broadcast.
//
// Thread safety: take g_mtx for any access to g_segments / g_cached_pkt.
// Do not hold g_mtx while broadcasting via NetServer (separate mutex inside).

#include <vector>
#include <mutex>
#include <string>
#include "net_protocol.hpp"

namespace HostBandPlan {

extern std::mutex                g_mtx;
extern std::vector<PktBandEntry> g_segments;     // master in-memory copy
extern std::vector<uint8_t>      g_cached_pkt;   // BEWE-framed BAND_PLAN_SYNC

std::string file_path();           // ~/BE_WE/band_plan.json
void load_from_file();             // populate g_segments; missing → empty
void save_to_file();               // write g_segments → disk (same JSON shape Central used)
void rebuild_cache();              // g_segments → g_cached_pkt

// Mutators. Caller must NOT hold g_mtx — these take it internally.
// Return true if state changed (so caller can persist + broadcast).
bool apply_add   (const PktBandEntry&  in);
bool apply_update(const PktBandEntry&  in);
bool apply_remove(const PktBandRemove& rm);

// Snapshot current state into a PktBandPlan (for broadcast).
void snapshot_pkt(PktBandPlan& out);

// Convenience for in-process host edits (GUI host menu, future CLI command).
// Applies, persists, rebuilds cache. Caller is responsible for broadcast
// (since broadcast needs NetServer access, kept out of this module).
// Returns true if state changed.
bool host_local_add   (const PktBandEntry&  in);
bool host_local_update(const PktBandEntry&  in);
bool host_local_remove(const PktBandRemove& rm);

} // namespace HostBandPlan
