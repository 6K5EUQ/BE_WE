# BEWE — Installation and Deployment

This document covers system requirements, dependency installation, receiver
permissions, build procedures for the operator workstation and headless
collection node, and Central Server provisioning.

For operator-facing material (key bindings, daily workflows, troubleshooting),
see [`OPERATOR.md`](OPERATOR.md).

---

## 1. Supported Platforms

| Role | Tested Platform | Build Profile |
|---|---|---|
| Operator workstation (GUI) | Ubuntu 24.04 LTS, x86_64 | Full GUI + analyzer |
| Headless collection node | Raspberry Pi OS 64-bit, ARM64 | CLI host only (`-DCLI=ON`) |
| Central Server | Ubuntu 24.04 LTS, x86_64 | Server binary only |

GPU acceleration (OpenGL 3+) is required on the operator workstation for the
waterfall renderer. Software-rendered fallback (`llvmpipe`) is detected and
warned at startup.

---

## 2. Dependencies (Ubuntu 24.04)

### Operator workstation (GUI + analyzer)

```bash
sudo apt install -y build-essential cmake pkg-config \
  libbladerf-dev librtlsdr-dev libiio-dev libad9361-dev \
  libfftw3-dev libasound2-dev libmpg123-dev libvolk-dev \
  libglew-dev libglfw3-dev libgl-dev libpng-dev libstb-dev
```

### Headless collection node (no GPU)

```bash
sudo apt install -y build-essential cmake pkg-config \
  libbladerf-dev librtlsdr-dev libfftw3-dev libasound2-dev \
  libmpg123-dev libvolk-dev libpng-dev
```

---

## 3. Receiver Permissions

On a clean Linux install, USB device nodes are root-owned. The receiver drivers
will fail to open the device with *insufficient permissions* until udev rules
are installed and the operator account is added to the `plugdev` group.

```bash
# BladeRF (2.0 micro PID 5250 and original PID 5246)
sudo tee /etc/udev/rules.d/88-bladerf.rules >/dev/null <<'EOF'
ATTR{idVendor}=="2cf0", ATTR{idProduct}=="5250", MODE="0660", GROUP="plugdev"
ATTR{idVendor}=="2cf0", ATTR{idProduct}=="5246", MODE="0660", GROUP="plugdev"
EOF

# ADALM-Pluto
sudo tee /etc/udev/rules.d/53-adalm-pluto.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="0456", ATTR{idProduct}=="b673", MODE="0660", GROUP="plugdev"
EOF

# RTL-SDR — also blacklist the kernel DVB driver that claims the dongle
sudo tee /etc/udev/rules.d/20-rtlsdr.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2832", MODE="0660", GROUP="plugdev"
EOF
echo "blacklist dvb_usb_rtl28xxu" | sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf

# Apply
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER
```

Log out and back in (or reboot) for the group change to take effect, then
re-plug the receiver.

---

## 4. Build Procedure

### 4.1 Operator workstation (full GUI)

```bash
git clone https://github.com/6K5EUQ/BE_WE.git
cd BE_WE && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
./BE_WE
```

### 4.2 Headless collection node (CLI host)

```bash
git clone https://github.com/6K5EUQ/BE_WE.git
cd BE_WE && mkdir build_cli && cd build_cli
cmake -DCLI=ON -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
./BE_WE
```

The CLI binary runs without a display server. Interactive prompts at startup
collect operator credentials and site preset (DGS-1 / DGS-2 / DGS-3 / ETC). Once
running, the process accepts text commands (`/status`, `/clients`, `/shutdown`,
`/Update TLEs`); any other input is broadcast as chat to connected analyst
workstations.

---

## 5. Central Server

The Central Server is a separate executable that relays HOST↔JOIN traffic,
maintains the permanent archive, and hosts the cross-station emitter database.
It binds a single TCP port (default 7700).

```bash
cd central && mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
./bewe_central
```

The server runs in the foreground and logs to stdout. For unattended
operation, deploy under `systemd` (a unit file is not bundled — operations
group provisions per site policy).

### Storage layout

The Central Server creates the following on first run under `~/BE_WE/DataBase/`:

| Directory | Purpose |
|---|---|
| `missions/<station>/<year>/<code>/{iq,audio,hist}/` | Per-mission archive (permanent) |
| `iq/`, `audio/`, `hist/` | Operator-uploaded shared library |
| `_emitters/`, `_sightings/`, `_reports/` | Cross-station emitter database |
| `schedules.json` | Persisted recording schedule |
| `missions.json` | Mission metadata index |

Sizing guidance: storage requirement is dominated by continuous HIST capture.
A single collection node at 20 MS/s produces approximately 7–9 GB per 24 hours
of waterfall archive (post-quantization). Plan retention accordingly.

---

## 6. Raspberry Pi 5 Deployment

The Raspberry Pi 5 is qualified as a forward-deployed collection node when
paired with an RTL-SDR or BladeRF. Performance tuning is required for stable
sustained capture.

```bash
# 1. Dependencies (CLI-only build)
sudo apt install -y build-essential cmake pkg-config \
  libbladerf-dev librtlsdr-dev libfftw3-dev libasound2-dev \
  libmpg123-dev libvolk-dev libpng-dev

# 2. Build
cd ~ && git clone https://github.com/6K5EUQ/BE_WE.git
cd BE_WE && mkdir build_cli && cd build_cli
cmake -DCLI=ON -DCMAKE_BUILD_TYPE=Release ..
make -j4

# 3. System tuning
sudo bash ~/BE_WE/setup_pi_performance.sh && sudo reboot

# 4. Start
cd ~/BE_WE/build_cli && ./BE_WE
```

The tuning script locks the CPU governor to `performance`, disables WiFi power
management and USB autosuspend, and enlarges network buffers. Settings
persist across reboots. Use a short USB extension cable to keep the receiver
away from the Pi's heat envelope.

---

## 7. Verification

After installation, confirm the following:

1. `./BE_WE` (or CLI variant) starts without `llvmpipe` warning on the
   operator workstation. Software rendering will produce a usable but slow
   waterfall.
2. The configured receiver enumerates at startup (look for
   `HW: BladeRF detected` / `HW: Pluto detected` / `HW: RTL-SDR detected`).
3. The Central Server logs `[Central] listening on port 7700` and accepts the
   first HOST connection.
4. From an operator workstation, the 3D globe view populates with the live
   station marker(s).

---

## 8. Network Considerations

- **Single port**: The Central Server uses one TCP port (default 7700) for all
  HOST and JOIN traffic. No additional port forwarding is required.
- **Mesh VPN recommended**: For multi-site deployments across public networks,
  a mesh VPN (Tailscale, WireGuard) provides stable site-to-Central
  connectivity without static IP allocation.
- **Bandwidth**: A single station consumes approximately 145 KB/s of
  continuous traffic to Central after quantization (configurable). Operator
  workstation bandwidth scales with the number of stations under observation.

---

## 9. Build Options

| CMake option | Purpose |
|---|---|
| `-DCLI=ON` | Headless collection node build (no GPU, no GUI) |
| `-DCMAKE_BUILD_TYPE=Release` | Optimized build (default for deployment) |
| `-DCMAKE_BUILD_TYPE=Debug` | Symbols and asserts for development |

---

## 10. Licensing

Contact for licensing inquiries.
