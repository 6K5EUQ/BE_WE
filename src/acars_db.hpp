#pragma once
// ── Offline aircraft registry (reg → ICAO type / operator) ────────────────
// Binary built from OpenSky aircraftDatabase (see tools/build_aircraft_db.py).
// Loaded once at startup; lookups are read-only (no lock needed afterwards).
bool        acars_db_load(const char* path);   // true if loaded ok
bool        acars_db_ready();
const char* acars_db_type(const char* reg);     // ICAO type designator, "" if unknown
const char* acars_db_operator(const char* reg); // operator/airline name, "" if unknown
