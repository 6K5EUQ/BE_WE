#pragma once

#ifndef BEWE_HEADLESS
bool draw_login_screen(int win_w, int win_h);
#endif

const char* login_get_id();
const char* login_get_pw();
int         login_get_tier();
const char* login_get_server();  // Central Server 주소

// CLI login: 직접 전역 로그인 변수 설정
bool cli_login(const char* id, const char* pw, int tier, const char* server);