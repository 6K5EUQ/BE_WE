#pragma once
// Signal Library / Emitter DB — Central 측 데이터 모델 + 매칭 엔진 + JSONL 영속화.
// 운영자가 .info에 직접 적은 값(또는 녹음 시점에 시스템이 자동 기재한 값)만 매칭에 사용.
// 자동 디코더 결과(AIS/ADS-B/PRI)는 절대 매칭에 반영 안 함 — 운영자가 신뢰할 만하다고 판단해
// .info에 직접 옮겨 적은 경우에만 신뢰.

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <cstdint>

namespace BeweCentral {

// match_status (PktSightingEntry.match_status와 동일 값).
constexpr uint8_t MS_AUTO_HIGH = 0;
constexpr uint8_t MS_AUTO_LOW  = 1;
constexpr uint8_t MS_PENDING   = 2;
constexpr uint8_t MS_CONFIRMED = 3;
constexpr uint8_t MS_REJECTED  = 4;
constexpr uint8_t MS_MANUAL    = 5;

// 한 번의 관측(.info 1개 = 1 sighting).
struct Sighting {
    std::string sighting_id;        // fnv1a(filename|reporter)[:12] — 멱등성 키
    std::string filename;
    std::string reporter;
    std::string station;            // .info "Location"
    float       freq_mhz = 0.f;
    float       bw_khz = 0.f;
    std::string modulation;
    std::string protocol;
    std::string target;
    std::string operator_name;
    std::string tags;
    int64_t     start_utc = 0;
    uint32_t    duration_s = 0;
    std::string emitter_uid;        // 묶인 emitter (없으면 빈)
    uint8_t     match_status = MS_PENDING;
};

// 같은 신호로 묶인 sighting의 집계.
struct Emitter {
    std::string emitter_uid;            // "e_<8hex>" — 발급 후 불변
    std::string display_name;
    float       freq_center_mhz = 0.f;
    float       freq_tolerance_khz = 1.0f;
    float       bw_khz = 0.f;
    std::string modulation;
    std::string protocol;
    std::string tags_id;                // 운영자 명시적 ID 토큰 (예: "MMSI:1234")
    int64_t     first_seen_utc = 0;
    int64_t     last_seen_utc = 0;
    uint32_t    sighting_count = 0;
    std::vector<std::string> contributing_stations;
    std::string operator_notes;
    std::vector<std::string> member_sighting_ids;
    std::string created_by;
    int64_t     created_utc = 0;
};

class EmitterDb {
public:
    // base_dir = ~/BE_WE/DataBase. _emitters/, _sightings/ 자동 생성.
    bool load(const std::string& base_dir);

    // _reports/ 안의 .info 파일을 모두 스캔해 ingest. 부팅 시 마이그레이션용.
    // 이미 ingest된 sighting_id는 dedup.
    bool migrate_from_reports(const std::string& reports_dir);

    struct MatchResult {
        std::string emitter_uid;        // 빈 = 매칭 실패(주파수 누락)
        float       score = 0.f;
        uint8_t     status = MS_PENDING;
    };
    // 새 sighting 1건 처리. s.emitter_uid + s.match_status 갱신 후 영속화.
    MatchResult ingest_sighting(Sighting& s);

    // 신규/갱신. e.emitter_uid 비어있으면 새 uid 발급.
    bool upsert_emitter(Emitter& e);

    // emitter 삭제. 자식 sighting의 emitter_uid는 비우고 status=PENDING으로.
    bool delete_emitter(const std::string& uid);

    // filename에 해당하는 모든 sighting을 삭제. 부모 emitter의 멤버에서 제거하고,
    // 멤버가 0이면 emitter도 삭제. 변경된 emitter uid를 affected_out, 삭제된 uid를
    // orphaned_out에 추가 (호출자가 broadcast_emitter_changed로 전파).
    bool delete_sighting_by_filename(const std::string& filename,
                                     std::vector<std::string>& affected_out,
                                     std::vector<std::string>& orphaned_out);

    // action: 0=confirm, 1=reject, 2=move(target), 3=split_to_new
    bool link_sighting(const std::string& sighting_id,
                       const std::string& target_emitter_uid,
                       uint8_t action);

    // 페이지네이션. total = 전체 개수.
    void list_emitters(uint16_t off, uint16_t lim,
                       std::vector<Emitter>& out, uint16_t& total);
    // emitter_uid_filter 빈 = 전체.
    void list_sightings(const std::string& emitter_uid_filter,
                        uint16_t off, uint16_t lim,
                        std::vector<Sighting>& out, uint16_t& total);

    bool find_emitter(const std::string& uid, Emitter& out) const;
    bool find_sighting(const std::string& sid, Sighting& out) const;

    size_t emitter_count() const;
    size_t sighting_count() const;

private:
    bool ensure_dirs_();
    bool save_emitters_atomic_unlocked_();
    bool save_sightings_atomic_unlocked_();
    bool load_emitters_jsonl_();
    bool load_sightings_jsonl_();

    Emitter make_emitter_from_(const Sighting& s, const std::string& creator);
    float   score_match_(const Sighting& s, const Emitter& e) const;
    void    recompute_emitter_aggregate_(Emitter& e);

    mutable std::mutex mtx_;
    std::map<std::string, Emitter>  emitters_;
    std::map<std::string, Sighting> sightings_;
    std::string base_dir_;
};

// Util — 단위 테스트용으로 외부 노출.
std::string sighting_id_from(const std::string& filename, const std::string& reporter);
std::string new_emitter_uid();

} // namespace BeweCentral
