#!/usr/bin/env bash
# Install BEWE as a desktop application (icon theme + .desktop launcher) so it shows
# up in the GNOME app grid / dock like a normal installed app (Chrome-style).
# Idempotent, per-user, no sudo. Run once per PC:  ~/BE_WE/packaging/install-app.sh
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"          # repo root (packaging/ -> ..)
ICON_SRC="$REPO/assets/icon_round.png"
EXEC="$REPO/build/BE_WE"
[ -f "$ICON_SRC" ] || { echo "ERROR: $ICON_SRC 없음 (먼저 빌드/pull)"; exit 1; }

# 1) themed icon (여러 크기 — 독/그리드에서 선명하게)
for s in 512 256 128 64 48; do
  d="$HOME/.local/share/icons/hicolor/${s}x${s}/apps"; mkdir -p "$d"
  if command -v convert >/dev/null 2>&1; then convert "$ICON_SRC" -resize ${s}x${s} "$d/bewe.png"
  else cp "$ICON_SRC" "$d/bewe.png"; fi
done

# 2) .desktop launcher
APPDIR="$HOME/.local/share/applications"; mkdir -p "$APPDIR"
cat > "$APPDIR/bewe.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=BEWE
GenericName=SDR / SIGINT
Comment=BEWE distributed SDR/SIGINT monitor
Exec=$EXEC
Icon=bewe
Terminal=false
StartupWMClass=BEWE
StartupNotify=true
Categories=Network;HamRadio;
Keywords=SDR;RTL;spectrum;radio;BEWE;
EOF

# 3) refresh caches
update-desktop-database "$APPDIR" 2>/dev/null || true
gtk-update-icon-cache -f -t "$HOME/.local/share/icons/hicolor" 2>/dev/null || true

echo "OK: BEWE 앱 등록됨 (Exec=$EXEC, Icon=bewe)"
echo "→ Activities(앱 그리드)에서 BEWE 검색, 독 아이콘 우클릭 → 즐겨찾기에 추가 로 고정."
