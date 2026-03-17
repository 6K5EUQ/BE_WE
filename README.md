# BE_WE

> Multi-user SDR spectrum analyzer with real-time network streaming, digital decoding, and 3D station discovery.

![Main Interface](assets/BEWE.png)

---

## Table of Contents

- [Overview](#overview)
- [Screenshots](#screenshots)
- [Features](#features)
- [Supported Hardware](#supported-hardware)
- [Architecture](#architecture)
- [Build](#build)
- [Usage](#usage)
- [Network](#network)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)

---

## Overview

BE_WE is a Linux-native SDR (Software Defined Radio) application built with C++17, OpenGL, and ImGui. A single HOST captures RF spectrum from an SDR device and streams it in real-time to multiple JOIN clients over TCP. Every operator sees the same live waterfall, can create channels, demodulate signals, chat, and share recordings — all from separate machines.

---

## Screenshots

| Login | Globe (Station Discovery) |
|:---:|:---:|
| ![Login](assets/Login.png) | ![Globe](assets/Main_Page.png) |

| Spectrum + Waterfall | Signal Analyzer |
|:---:|:---:|
| ![Monitor](assets/Monitor.png) | ![Screen](assets/Screen.png) |

---

## Features

### Spectrum & Waterfall
- Real-time FFT with configurable size (default 8192)
- GPU-accelerated waterfall display with 2500-row history (~60 s)
- Frequency zoom / pan / drag-scroll
- Auto-scale and manual power range control
- Time-stamped event tags on waterfall (5 s interval)

### Demodulation
- **Analog** — AM, FM, MAGIC (auto-detect AM/FM/DSB/SSB/CW)
- **Digital** — AIS (marine vessel tracking), DMR (AMBE+2 voice via mbelib)
- Up to 10 simultaneous channels with independent mode selection

### Audio
- 48 kHz stereo output via ALSA
- Per-channel pan control (L / Center / R / Mute)
- Per-operator audio routing (32-bit bitmask)
- Squelch with auto-calibration and gate hold
- 5 noise reduction algorithms: Spectral Subtraction, Spectral Gate, Wiener Filter, MMSE-STSA, Log-MMSE

### Time Machine
- 60-second rolling IQ recording to disk (toggle with `T`)
- Freeze & seek through waterfall history (`Space`)
- Region selection (Ctrl+Right-drag) for IQ export

### Network Streaming (HOST / JOIN)
- HOST broadcasts FFT + audio over TCP
- JOIN clients receive with jitter buffer for smooth playback
- Per-client async send queues — slow clients don't block the stream
- Tier-based authentication (Tier 1 / 2 / 3)
- Bi-directional commands: JOIN can tune frequency, create channels, control gain

### Station Discovery & Globe
- 3D interactive globe with Blue Marble texture
- UDP broadcast discovery on LAN
- Optional relay server for WAN connections
- Local relay for same-subnet optimization
- Click globe to set your station location

### Collaboration
- Real-time chat between all connected operators
- File sharing (upload / download recordings via public directory)
- Operator list with tier and connection status
- Channel ownership tracking

### Signal Analyzer
- Open exported WAV/IQ files for offline spectral analysis
- Zoom / pan / region selection on SA waterfall
- Demodulate and play back selected regions

---

## Supported Hardware

| Device | Frequency Range | Gain | Format |
|--------|----------------|------|--------|
| **BladeRF** | 47 MHz – 6 GHz | 0 – 60 dB | SC16_Q11 |
| **RTL-SDR** | 500 kHz – 1.766 GHz | 0 – 49.6 dB (29 steps) | uint8 offset binary |

Hardware is auto-detected at startup. If no SDR is found, you can still JOIN a remote HOST.

---

## Architecture

```
                        ┌─────────────────────────────────┐
                        │           RELAY SERVER          │
                        └──────┬──────────────┬───────────┘
                               │   WAN        │
                 ┌─────────────┴───┐    ┌─────┴───────────┐
                 │   HOST (SDR)    │    │   JOIN Client    │
                 │                 │    │   (no hardware)  │
                 │  ┌───────────┐  │    └──────────────────┘
                 │  │ BladeRF / │  │         LAN
                 │  │ RTL-SDR   │  ├─── UDP Discovery ───── JOIN Client
                 │  └───────────┘  │
                 └─────────────────┘
```

- **HOST** — Runs the SDR, computes FFT, broadcasts to all JOINs
- **JOIN** — Connects to HOST, receives spectrum + audio, sends commands
- **Relay** — Optional WAN bridge when HOST and JOIN are on different networks

---

## Build

### Requirements

- Linux (tested on Ubuntu 24.04)
- CMake 3.16+
- C++17 compiler (GCC 9+ / Clang 10+)

### System Dependencies

```bash
# Build tools
sudo apt install -y build-essential cmake pkg-config

# SDR libraries
sudo apt install -y libbladerf-dev librtlsdr-dev

# DSP / Audio
sudo apt install -y libfftw3-dev libasound2-dev libmpg123-dev

# Graphics
sudo apt install -y libglew-dev libglfw3-dev libgl-dev libpng-dev

# Image loading
sudo apt install -y libstb-dev
```

#### mbelib (manual build required)

`libmbe` is not available in Ubuntu's default repositories. Build from source:

```bash
cd /tmp
git clone https://github.com/szechyjs/mbelib.git
cd mbelib
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### One-liner (all deps)

```bash
sudo apt install -y build-essential cmake pkg-config \
  libbladerf-dev librtlsdr-dev libfftw3-dev libasound2-dev \
  libmpg123-dev libglew-dev libglfw3-dev libgl-dev libpng-dev libstb-dev
```

### Compile

```bash
git clone https://github.com/<your-username>/BE_WE.git
cd BE_WE
mkdir build && cd build
cmake ..
make -j$(nproc)
./BE_WE
```

---

## Usage

### HOST Mode

1. Launch `./BE_WE`
2. Log in with ID / Password and select a Tier (`Ctrl+1` / `Ctrl+2` / `Ctrl+3`)
3. Click your location on the 3D globe, then press **HOST**
4. SDR starts automatically — spectrum and waterfall appear
5. Other users can now JOIN your station

### JOIN Mode

1. Launch `./BE_WE` on a different machine (no SDR required)
2. Log in and select a Tier
3. Stations appear on the globe via UDP discovery or relay
4. Click a station marker and press **JOIN**
5. Live spectrum, audio, and channels stream in real-time

### Key Bindings

| Key | Action |
|-----|--------|
| `T` | Toggle IQ rolling recording (Time Machine) |
| `Space` | Freeze waterfall and enter Time Machine view |
| `Scroll` | Zoom frequency axis |
| `Ctrl+Right-drag` | Select region for IQ export |
| `D` | Toggle digital decode panel for selected channel |
| `Ctrl+1/2/3` | Select Tier on login screen |

---

## Network

- HOST ↔ JOIN communication uses a custom binary protocol over TCP
- LAN stations are discovered automatically via UDP broadcast
- Optional relay server available for WAN connections
- See `src/net_protocol.hpp` for internal details

---

## Troubleshooting

### WiFi: Choppy / stuttering spectrum on JOIN

If the waterfall scrolls in bursts instead of smoothly, WiFi power management is likely batching incoming packets.

**Check:**
```bash
iwconfig 2>/dev/null | grep "Power Management"
```

**Fix (immediate):**
```bash
sudo iwconfig <interface> power off
```

**Fix (permanent):**
```bash
sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf <<'EOF'
[connection]
wifi.powersave = 2
EOF
sudo systemctl restart NetworkManager
```

### `glxinfo` shows software renderer (`llvmpipe`)

You're running without GPU acceleration. Install the proper driver:

```bash
# NVIDIA
sudo apt install nvidia-driver-535

# Intel / AMD
sudo apt install mesa-vulkan-drivers
```

### BladeRF not detected

```bash
# Check USB connection
lsusb | grep Nuand

# Permissions: add udev rule
sudo tee /etc/udev/rules.d/88-bladerf.rules <<'EOF'
ATTR{idVendor}=="2cf0", ATTR{idProduct}=="5246", MODE="0660", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
# Re-plug the device
```

### RTL-SDR claimed by kernel DVB driver

```bash
sudo modprobe -r dvb_usb_rtl28xxu
# To make permanent:
echo "blacklist dvb_usb_rtl28xxu" | sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf
```

### Build error: package not found

See [System Dependencies](#system-dependencies) above. All required packages with their `apt` names:

| CMake package | apt package |
|---|---|
| `libbladeRF` | `libbladerf-dev` |
| `fftw3f` | `libfftw3-dev` |
| `alsa` | `libasound2-dev` |
| `libmpg123` | `libmpg123-dev` |
| `librtlsdr` | `librtlsdr-dev` |
| `libmbe` | [build from source](#mbelib-manual-build-required) |
| `libpng` | `libpng-dev` |
| OpenGL | `libgl-dev` |
| GLEW | `libglew-dev` |
| glfw3 | `libglfw3-dev` |

---

## Project Structure

```
BE_WE/
├── src/
│   ├── main.cpp              # Entry point
│   ├── ui.cpp                # ImGui UI (login, globe, spectrum, panels)
│   ├── fft_viewer.hpp/cpp    # FFT computation & waterfall rendering
│   ├── bladerf_io.cpp        # BladeRF SDR capture
│   ├── rtlsdr_io.cpp         # RTL-SDR capture
│   ├── hw_detect.cpp         # Hardware auto-detection
│   ├── demod.cpp             # AM / FM / MAGIC demodulation
│   ├── audio.cpp             # ALSA output + 5 noise reduction algorithms
│   ├── ais.cpp               # AIS digital decoder
│   ├── dmr.cpp               # DMR digital decoder (AMBE+2)
│   ├── net_protocol.hpp      # Binary protocol specification
│   ├── net_server.hpp/cpp    # HOST-side TCP server
│   ├── net_client.hpp/cpp    # JOIN-side TCP client
│   ├── net_stream.cpp        # Stream utilities
│   ├── udp_discovery.hpp/cpp # LAN station broadcast
│   ├── relay_client.hpp/cpp  # WAN relay client
│   ├── local_relay_server.*  # LAN local relay
│   ├── globe.hpp/cpp         # 3D Earth renderer (OpenGL 3.3)
│   ├── timemachine.cpp       # IQ rolling record & playback
│   ├── region_save.cpp       # Region IQ export
│   ├── sa_compute.cpp        # Signal Analyzer offline FFT
│   ├── iq_record.cpp         # IQ / audio recording
│   ├── login.cpp             # Authentication & tier selection
│   ├── channel.hpp           # Per-channel state (freq, mode, squelch)
│   ├── config.hpp            # Global constants
│   └── world_map_data.hpp    # Embedded vector map for globe
├── relay/
│   ├── relay_main.cpp        # Standalone relay server
│   ├── relay_server.hpp/cpp  # Relay implementation
│   └── relay_proto.hpp       # Relay protocol
├── libs/
│   └── imgui/                # Dear ImGui (embedded)
├── assets/
│   ├── BEWE.png              # Logo / main screenshot
│   ├── earth.jpg             # Blue Marble texture
│   ├── login_bg_Tier_*.png   # Tier-specific login backgrounds
│   └── *.png                 # UI screenshots
└── CMakeLists.txt            # Build configuration
```

---

## License

TBD
