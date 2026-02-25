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
