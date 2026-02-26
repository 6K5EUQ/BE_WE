#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# BE_WE AppImage 빌드 스크립트
# 사용법: bash build_appimage.sh
# 빌드 결과: ./BEWE-x86_64.AppImage
# ─────────────────────────────────────────────────────────────────────────────
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
APPDIR="$PROJECT_DIR/AppDir"

echo "=== [1/6] 빌드 ==="
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr
make -j$(nproc)

echo "=== [2/6] AppDir 구성 ==="
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin"
mkdir -p "$APPDIR/usr/share/BE_WE/assets"
mkdir -p "$APPDIR/usr/share/applications"
mkdir -p "$APPDIR/usr/share/icons/hicolor/256x256/apps"
mkdir -p "$APPDIR/usr/lib"

# 실행 파일 복사
cp "$BUILD_DIR/BE_WE" "$APPDIR/usr/bin/BE_WE"

# assets 복사 (로그인 배경 이미지 등)
if [ -d "$PROJECT_DIR/assets" ]; then
    cp -r "$PROJECT_DIR/assets/"* "$APPDIR/usr/share/BE_WE/assets/"
fi

# .desktop 파일
cat > "$APPDIR/usr/share/applications/bewe.desktop" << 'DESK'
[Desktop Entry]
Name=BEWE
Exec=BE_WE
Icon=bewe
Type=Application
Categories=Utility;
DESK
cp "$APPDIR/usr/share/applications/bewe.desktop" "$APPDIR/bewe.desktop"

# 아이콘 (없으면 빈 PNG 생성)
if [ -f "$PROJECT_DIR/assets/icon.png" ]; then
    cp "$PROJECT_DIR/assets/icon.png" "$APPDIR/usr/share/icons/hicolor/256x256/apps/bewe.png"
    cp "$PROJECT_DIR/assets/icon.png" "$APPDIR/bewe.png"
else
    # Python으로 유효한 256x256 PNG 생성
    python3 -c "
import struct, zlib

def make_png(w, h, r, g, b):
    def chunk(name, data):
        c = struct.pack('>I', len(data)) + name + data
        return c + struct.pack('>I', zlib.crc32(name + data) & 0xffffffff)
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    raw = b''.join(b'\x00' + bytes([r,g,b]*w) for _ in range(h))
    idat = zlib.compress(raw)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) + chunk(b'IDAT', idat) + chunk(b'IEND', b'')

png = make_png(256, 256, 30, 60, 120)
open('$APPDIR/usr/share/icons/hicolor/256x256/apps/bewe.png','wb').write(png)
import shutil
shutil.copy('$APPDIR/usr/share/icons/hicolor/256x256/apps/bewe.png','$APPDIR/bewe.png')
"
fi

# AppRun 스크립트
cat > "$APPDIR/AppRun" << 'APPRUN'
#!/bin/bash
SELF=$(readlink -f "$0")
HERE=$(dirname "$SELF")
export APPDIR="$HERE"
export PATH="$HERE/usr/bin:$PATH"
export LD_LIBRARY_PATH="$HERE/usr/lib:$LD_LIBRARY_PATH"

# --install-udev 옵션: udev 규칙 설치 후 종료
if [ "$1" = "--install-udev" ]; then
    if [ "$EUID" -ne 0 ]; then
        echo "root 권한 필요: sudo $SELF --install-udev"
        exit 1
    fi
    echo "SUBSYSTEM=="usb", ATTR{idVendor}=="2cf0", MODE="0664", GROUP="plugdev""         > /etc/udev/rules.d/88-nuand.rules
    echo "SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2832", GROUP="plugdev", MODE="0664""         > /etc/udev/rules.d/20-rtlsdr.rules
    echo "SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", GROUP="plugdev", MODE="0664""         >> /etc/udev/rules.d/20-rtlsdr.rules
    udevadm control --reload-rules
    udevadm trigger
    ACTUAL_USER="${SUDO_USER:-$USER}"
    usermod -aG plugdev "$ACTUAL_USER"
    echo "Done. 재로그인 후 SDR 장치 사용 가능합니다."
    exit 0
fi

# udev 규칙 확인
if [ ! -f /etc/udev/rules.d/88-nuand.rules ] && [ ! -f /lib/udev/rules.d/88-nuand.rules ]; then
    echo "[BEWE] BladeRF udev rules not found."
    echo "       Run: sudo $SELF --install-udev"
fi
exec "$HERE/usr/bin/BE_WE" "$@"
APPRUN
chmod +x "$APPDIR/AppRun"

echo "=== [3/6] 의존 라이브러리 복사 ==="
copy_lib(){
    local lib=$(ldconfig -p | grep "$1" | grep "x86-64" | head -1 | awk '{print $NF}')
    if [ -z "$lib" ]; then
        lib=$(find /usr/lib /usr/local/lib -name "$1*" 2>/dev/null | head -1)
    fi
    if [ -n "$lib" ] && [ -f "$lib" ]; then
        cp -L "$lib" "$APPDIR/usr/lib/" 2>/dev/null || true
        echo "  copied: $lib"
    else
        echo "  [WARN] not found: $1"
    fi
}

# 핵심 의존성
for lib in \
    libbladeRF.so \
    librtlsdr.so \
    libfftw3f.so \
    libasound.so \
    libmpg123.so \
    libmbe.so \
    libpng16.so \
    libGLEW.so \
    libglfw.so \
    libusb-1.0.so \
; do
    copy_lib "$lib"
done

# BladeRF FPGA/펌웨어 파일 복사 (있으면)
for BLADE_DATA in \
    /usr/share/Nuand/bladeRF \
    /usr/local/share/Nuand/bladeRF \
    /usr/lib/firmware/nuand \
; do
    if [ -d "$BLADE_DATA" ]; then
        mkdir -p "$APPDIR/usr/share/Nuand/bladeRF"
        cp -r "$BLADE_DATA/"* "$APPDIR/usr/share/Nuand/bladeRF/" 2>/dev/null || true
        echo "  BladeRF firmware: $BLADE_DATA"
        break
    fi
done

echo "=== [4/6] linuxdeploy 다운로드 ==="
if [ ! -f "$PROJECT_DIR/linuxdeploy-x86_64.AppImage" ]; then
    wget -q --show-progress \
        "https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage" \
        -O "$PROJECT_DIR/linuxdeploy-x86_64.AppImage"
    chmod +x "$PROJECT_DIR/linuxdeploy-x86_64.AppImage"
fi

echo "=== [5/6] AppImage 패키징 ==="
cd "$PROJECT_DIR"
ARCH=x86_64 ./linuxdeploy-x86_64.AppImage \
    --appdir "$APPDIR" \
    --output appimage \
    --executable "$APPDIR/usr/bin/BE_WE" \
    --desktop-file "$APPDIR/bewe.desktop" \
    --icon-file "$APPDIR/bewe.png"

echo "=== [6/6] udev 설치 스크립트 생성 ==="
cat > "$PROJECT_DIR/bewe-install-udev.sh" << 'UDEV'
#!/bin/bash
# BE_WE udev 규칙 설치 (BladeRF + RTL-SDR)
# 실행: sudo bash bewe-install-udev.sh
set -e

# BladeRF
cat > /etc/udev/rules.d/88-nuand.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2cf0", MODE="0664", GROUP="plugdev"
EOF

# RTL-SDR
cat > /etc/udev/rules.d/20-rtlsdr.rules << 'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2832", GROUP="plugdev", MODE="0664"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", GROUP="plugdev", MODE="0664"
EOF

udevadm control --reload-rules
udevadm trigger

# 현재 유저를 plugdev 그룹에 추가
usermod -aG plugdev "$SUDO_USER"
echo "Done. 재로그인 후 SDR 장치 사용 가능합니다."
UDEV
chmod +x "$PROJECT_DIR/bewe-install-udev.sh"

echo ""
echo "=========================================="
echo " 빌드 완료!"
echo " AppImage : $(ls $PROJECT_DIR/BEWE*.AppImage 2>/dev/null | head -1)"
echo " udev 설치: sudo bash bewe-install-udev.sh  (최초 1회)"
echo "=========================================="
