#pragma once
// Mission File Push worker (Phase 2, v3.8.0)
//
// HOST 측 background worker: 미션 dir 안의 IQ/audio/hist 완료 파일을
// Central로 push (MISSION_FILE_PUSH_META + DATA), ACK 받으면 로컬 unlink.
//
// 사용:
//   MissionPush::start(viewer, central_client);   // HOST 모드 시작 시
//   MissionPush::enqueue(path, MFS_IQ|MFS_AUDIO|MFS_HIST);  // 파일 close 후 호출
//   MissionPush::stop();                           // HOST 모드 종료 시
//
// 정책: ACK(status=0) 받으면 로컬 unlink (+.info도). 실패/timeout 시 다시 queue 후미로.

#include <cstdint>
#include <string>

class FFTViewer;
class CentralClient;

namespace MissionPush {

void start(FFTViewer* v, CentralClient* cli);
void stop();

// 완료된 파일을 push queue 후미에 추가. subdir = MFS_IQ/MFS_AUDIO/MFS_HIST.
// path는 ~/BE_WE/recordings/missions/<year>/<code>/<subdir>/<filename> 형식이어야 함.
// 잘못된 path면 silent drop.
void enqueue(const std::string& path, uint8_t subdir);

// 활성 미션 dir 안 모든 닫힌 파일을 일괄 enqueue (mission_end 직후 호출 권장)
void scan_mission_dir_enqueue(int year, const char* code);

// 디버그 / status: 대기 중 transfer 수
int pending_count();

} // namespace MissionPush
