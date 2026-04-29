// Standalone unit test for InfoParse + EmitterDb.
// Build: cd central/build && make bewe_emitter_db_test && ./bewe_emitter_db_test
//
// Verifies:
//   - .info parsing (key:value, freq/bw/duration/utc extraction)
//   - sighting ingest with deterministic IDs (dedup)
//   - auto-high merge when modulation matches
//   - pending when only frequency matches (operator left mod blank)
//   - new emitter when frequency far from existing
//   - manual link operations (confirm/reject/move/split)
//   - JSONL persistence round-trip

#include "info_parse.hpp"
#include "emitter_db.hpp"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <sys/stat.h>

using namespace BeweCentral;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond) do { \
    if(cond){ g_pass++; printf("  PASS: %s\n", #cond); } \
    else    { g_fail++; printf("  FAIL: %s  (line %d)\n", #cond, __LINE__); } \
} while(0)

static std::string make_tmp_dir(){
    char tmpl[] = "/tmp/bewe_emitter_test_XXXXXX";
    char* p = mkdtemp(tmpl);
    if(!p){ perror("mkdtemp"); exit(1); }
    return std::string(p);
}

static void remove_dir_recursive(const std::string& d){
    std::string cmd = "rm -rf '" + d + "'";
    (void)!system(cmd.c_str());
}

static void test_info_parse(){
    printf("\n[test_info_parse]\n");
    std::string text =
        "File Name: test_recording_1\n"
        "Day: Apr 29, 2026\n"
        "Up Time: 19:41:21 (UTC+9)\n"
        "Duration: 2.0 s\n"
        "Frequency: 145.2300 MHz\n"
        "Modulation: FM\n"
        "Bandwidth: 12.5 kHz\n"
        "Protocol: \n"
        "Operator: testop\n"
        "Tags: \n";
    auto kv = InfoParse::parse(text);
    CHECK(kv.size() == 10);
    CHECK(kv["File Name"] == "test_recording_1");
    CHECK(kv["Modulation"] == "FM");
    CHECK(kv["Protocol"] == "");
    float f = 0;
    CHECK(InfoParse::extract_freq_mhz(kv["Frequency"], f));
    CHECK(std::abs(f - 145.23f) < 0.001f);
    float bw = 0;
    CHECK(InfoParse::extract_bw_khz(kv["Bandwidth"], bw));
    CHECK(std::abs(bw - 12.5f) < 0.001f);
    uint32_t dur = 0;
    CHECK(InfoParse::extract_duration_s(kv["Duration"], dur));
    CHECK(dur == 2);
    int64_t utc = 0;
    CHECK(InfoParse::extract_start_utc(kv["Day"], kv["Up Time"], utc));
    // Sanity: 2026-ish epoch range. Exact value not critical (timegm + offset arithmetic).
    CHECK(utc > 1700000000 && utc < 1900000000);
    // Roundtrip: same wall-clock + UTC+0 vs UTC+9 should differ by 9h.
    int64_t utc0 = 0;
    CHECK(InfoParse::extract_start_utc(kv["Day"], "19:41:21 (UTC+0)", utc0));
    CHECK(utc0 - utc == 9*3600);
}

static void test_unit_freq_bw(){
    printf("\n[test_unit_freq_bw]\n");
    float f;
    CHECK(InfoParse::extract_freq_mhz("0.145 GHz", f) && std::abs(f - 145.0f) < 1e-3);
    CHECK(InfoParse::extract_freq_mhz("145230 kHz", f) && std::abs(f - 145.230f) < 1e-3);
    CHECK(InfoParse::extract_freq_mhz("145.23", f) && std::abs(f - 145.23f) < 1e-3);
    CHECK(InfoParse::extract_freq_mhz("145000000 Hz", f) && std::abs(f - 145.0f) < 1e-3);
    float bw;
    CHECK(InfoParse::extract_bw_khz("0.5 MHz", bw) && std::abs(bw - 500.0f) < 0.1);
    CHECK(InfoParse::extract_bw_khz("500", bw) && std::abs(bw - 500.0f) < 0.1);
}

static Sighting mk_sighting(const std::string& fname, const std::string& reporter,
                            float freq_mhz, const std::string& mod,
                            const std::string& proto = "", int64_t start = 1700000000){
    Sighting s;
    s.filename   = fname;
    s.reporter   = reporter;
    s.station    = reporter;
    s.freq_mhz   = freq_mhz;
    s.bw_khz     = 12.5f;
    s.modulation = mod;
    s.protocol   = proto;
    s.start_utc  = start;
    s.duration_s = 5;
    s.sighting_id = sighting_id_from(s.filename, s.reporter);
    return s;
}

static void test_db_basic(){
    printf("\n[test_db_basic]\n");
    std::string dir = make_tmp_dir();
    {
        EmitterDb db;
        CHECK(db.load(dir));
        CHECK(db.emitter_count() == 0);

        // 1) First sighting → AUTO_LOW (new emitter)
        Sighting s1 = mk_sighting("rec1", "alpha", 145.230f, "FM");
        auto r1 = db.ingest_sighting(s1);
        CHECK(r1.status == MS_AUTO_LOW);
        CHECK(!r1.emitter_uid.empty());
        CHECK(db.emitter_count() == 1);

        // 2) Same freq + same mod → AUTO_HIGH merge
        Sighting s2 = mk_sighting("rec2", "bravo", 145.2305f, "FM");
        auto r2 = db.ingest_sighting(s2);
        CHECK(r2.status == MS_AUTO_HIGH);
        CHECK(r2.emitter_uid == r1.emitter_uid);
        CHECK(db.emitter_count() == 1);

        // 3) Different freq (>1 kHz tolerance) → new emitter
        Sighting s3 = mk_sighting("rec3", "alpha", 145.500f, "FM");
        auto r3 = db.ingest_sighting(s3);
        CHECK(r3.status == MS_AUTO_LOW);
        CHECK(r3.emitter_uid != r1.emitter_uid);
        CHECK(db.emitter_count() == 2);

        // 4) Same freq as #1 but blank modulation → PENDING (only freq matched, capped at 0.6)
        Sighting s4 = mk_sighting("rec4", "charlie", 145.230f, "");
        auto r4 = db.ingest_sighting(s4);
        CHECK(r4.status == MS_PENDING);
        CHECK(r4.emitter_uid == r1.emitter_uid);
        CHECK(db.emitter_count() == 2);

        // 5) Re-ingest same sighting (idempotent) — count must not change
        size_t prev = db.sighting_count();
        auto r4b = db.ingest_sighting(s4);
        CHECK(db.sighting_count() == prev);
        CHECK(r4b.status == r4.status);

        // 6) Aggregate state on emitter #1: 3 sightings (s1, s2, s4)
        Emitter e1;
        CHECK(db.find_emitter(r1.emitter_uid, e1));
        CHECK(e1.sighting_count == 3);
        CHECK(e1.contributing_stations.size() >= 2);  // alpha, bravo, charlie
    }
    // Reload — verify persistence
    {
        EmitterDb db2;
        CHECK(db2.load(dir));
        CHECK(db2.emitter_count() == 2);
        CHECK(db2.sighting_count() == 4);
    }
    remove_dir_recursive(dir);
}

static void test_link_actions(){
    printf("\n[test_link_actions]\n");
    std::string dir = make_tmp_dir();
    EmitterDb db; db.load(dir);

    Sighting s1 = mk_sighting("rec1", "alpha", 145.230f, "FM");
    auto r1 = db.ingest_sighting(s1);
    Sighting s2 = mk_sighting("rec2", "bravo", 145.230f, "");  // pending
    auto r2 = db.ingest_sighting(s2);
    CHECK(r2.status == MS_PENDING);
    CHECK(db.emitter_count() == 1);

    // Confirm s2 → status=CONFIRMED, still attached.
    CHECK(db.link_sighting(s2.sighting_id, "", 0));
    Sighting s2b; CHECK(db.find_sighting(s2.sighting_id, s2b));
    CHECK(s2b.match_status == MS_CONFIRMED);
    CHECK(s2b.emitter_uid == r1.emitter_uid);

    // Reject s2 → split into new emitter, status=MANUAL
    CHECK(db.link_sighting(s2.sighting_id, "", 1));
    Sighting s2c; CHECK(db.find_sighting(s2.sighting_id, s2c));
    CHECK(s2c.match_status == MS_MANUAL);
    CHECK(s2c.emitter_uid != r1.emitter_uid);
    CHECK(db.emitter_count() == 2);

    // Move s2 back to emitter #1
    CHECK(db.link_sighting(s2.sighting_id, r1.emitter_uid, 2));
    Sighting s2d; CHECK(db.find_sighting(s2.sighting_id, s2d));
    CHECK(s2d.emitter_uid == r1.emitter_uid);

    remove_dir_recursive(dir);
}

static void test_upsert_and_delete(){
    printf("\n[test_upsert_and_delete]\n");
    std::string dir = make_tmp_dir();
    EmitterDb db; db.load(dir);

    Emitter e;
    e.display_name = "Test Sat-Uplink";
    e.freq_center_mhz = 437.85f;
    e.freq_tolerance_khz = 5.0f;
    e.modulation = "GMSK";
    e.created_by = "manual_test";
    CHECK(db.upsert_emitter(e));
    CHECK(!e.emitter_uid.empty());
    CHECK(db.emitter_count() == 1);

    // Update name only
    Emitter e2 = e;
    e2.display_name = "Sat-Uplink (renamed)";
    CHECK(db.upsert_emitter(e2));
    Emitter back; CHECK(db.find_emitter(e.emitter_uid, back));
    CHECK(back.display_name == "Sat-Uplink (renamed)");
    CHECK(back.created_by == "manual_test");  // preserved

    // Delete
    CHECK(db.delete_emitter(e.emitter_uid));
    CHECK(db.emitter_count() == 0);

    remove_dir_recursive(dir);
}

int main(){
    test_info_parse();
    test_unit_freq_bw();
    test_db_basic();
    test_link_actions();
    test_upsert_and_delete();
    printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
