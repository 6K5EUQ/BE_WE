// ── AIS 모듈 (예정) — placeholder 등록만. 기능 미구현. ──────────────────────
// 폴더가 존재하면 런처 목록에 "AIS (soon)" 비활성 항목으로 표시. 추후 구현.
#include "module_api.hpp"

namespace {
bool s_reg = [](){
    BeweModule m{};
    m.id      = "ais";
    m.label   = "AIS";
    m.planned = true;
    bewe_register_module(m);
    return true;
}();
}
