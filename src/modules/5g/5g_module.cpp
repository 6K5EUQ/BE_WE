// ── 5G 모듈 (예정) — placeholder 등록만. 기능 미구현. ───────────────────────
// 폴더가 존재하면 런처 목록에 "5G (soon)" 비활성 항목으로 표시. 추후 구현.
#include "module_api.hpp"

namespace {
bool s_reg = [](){
    BeweModule m{};
    m.id      = "5g";
    m.label   = "5G";
    m.planned = true;
    bewe_register_module(m);
    return true;
}();
}
