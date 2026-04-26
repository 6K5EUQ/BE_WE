#pragma once
// Host-owned band category list. Each HOST owns ~/BE_WE/band_categories.json.
// 11 builtins (id 0..10) seeded if file missing. User can add (id 11..255),
// rename, recolor, delete (used-by entries strung to id=10 Other).
//
// Same lifecycle as HostBandPlan: load on startup, broadcast on CONN_OPEN,
// apply edits + persist + rebroadcast.

#include <vector>
#include <mutex>
#include <string>
#include "net_protocol.hpp"

namespace HostBandCategories {

extern std::mutex                    g_mtx;
extern std::vector<PktBandCategory>  g_cats;        // master in-memory copy
extern std::vector<uint8_t>          g_cached_pkt;  // BEWE-framed BAND_CAT_SYNC

std::string file_path();           // ~/BE_WE/band_categories.json
void load_from_file();             // missing → seed 11 builtins + save
void save_to_file();
void rebuild_cache();

bool apply_upsert(const PktBandCategory& in);  // by id (insert or update)
bool apply_delete(uint8_t id);                 // returns true if removed

void snapshot_pkt(PktBandCatSync& out);

// Lookup helpers (take g_mtx internally).
bool        lookup(uint8_t id, PktBandCategory& out);
std::string name_of(uint8_t id);

// In-process host edits (UI menu): apply + save + rebuild.
bool host_local_upsert(const PktBandCategory& in);
bool host_local_delete(uint8_t id);

} // namespace HostBandCategories
