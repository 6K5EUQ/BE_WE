#pragma once

bool draw_login_screen(int win_w, int win_h);

const char* login_get_id();
const char* login_get_pw();
int         login_get_tier();
const char* login_get_server();  // Central Server 주소