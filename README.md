# BE_WE

> Multi-user SDR spectrum analyzer with real-time network streaming, digital decoding, and 3D station discovery.

![Main Interface](assets/BEWE.png)

---

## Table of Contents

- [Overview](#overview)
- [Why BE_WE](#why-be_we)
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

## Why BE_WE

Most SDR applications are designed for a single operator sitting in front of a single machine. BE_WE is built from the ground up for **distributed, multi-operator environments** where the SDR hardware may be remote and bandwidth is limited.

### Multi-User Real-Time Collaboration

Unlike any other desktop SDR software, BE_WE supports multiple operators working on the same spectrum simultaneously. Each operator can independently create channels, tune demodulators, control audio routing, and record — all in real-time over the network. Built-in chat, file sharing, and an operator list with tier-based permissions make it a true **multiplayer SDR platform**.

### 3D Globe Station Discovery

Stations running BE_WE appear as markers on an interactive 3D globe. Operators can visually browse available stations worldwide and connect with a single click — no IP addresses or manual configuration needed. LAN stations are discovered automatically; WAN stations are reachable through an optional relay server.

### Time Machine — Rewind the Spectrum

BE_WE continuously records a 60-second rolling IQ buffer to disk. When something interesting appears on the waterfall, press `Space` to freeze and scroll back in time. Missed a signal 30 seconds ago? It's still there. Select a region and export the IQ — no need to have been "recording" at the time.

### Selective Region IQ Export

Instead of capturing the entire wideband IQ stream (which can be tens of MB/s), operators can `Ctrl+Right-drag` on the waterfall to select a specific time-frequency region and export only that portion. This is critical in bandwidth-constrained remote environments where full IQ transfer is impractical. Combined with Time Machine, operators can retroactively extract only the signal of interest — saving storage, network bandwidth, and analysis time.

### Per-Operator Audio Routing

Each demodulated channel can be routed to specific operators using a per-channel bitmask. Operator A monitors channel 1, operator B monitors channels 2 and 3 — each with independent pan and volume control. This enables team-based division of labor across the spectrum.

### Distributed Architecture (HOST / JOIN / Central Relay)

Any machine with an SDR can HOST a station. Any machine without an SDR can JOIN and operate as if the hardware were local. A central relay server bridges stations across different networks using a multiplexed (MUX) protocol over a single port — HOST connects once and the relay fans out to all JOINs. LAN-priority connection logic minimizes latency when operators share the same subnet. The entire system is designed for real-world deployment where operators and hardware are rarely in the same room.

---

## Screenshots

| Login | Globe (Station Discovery) |
|:---:|:---:|
| ![Login](assets/Login.png) | ![Globe](assets/Main_Page.png) |

| Spectrum + Waterfall | Wideband Overview |
|:---:|:---:|
| ![Monitor](assets/Monitor.png) | ![Screen](assets/Screen.png) |

| Time Machine + Region Export |
|:---:|
| ![Screen2](assets/Screen2.png) |

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
- IQ_CHUNK transfer: WAN-compatible file streaming from HOST to JOIN via MUX relay
- Real-time transfer progress broadcast (REC → Transferring → Done)

### Network Streaming (HOST / JOIN)
- HOST broadcasts FFT + audio over TCP
- JOIN clients receive with jitter buffer for smooth playback
- Per-client async send queues with priority (control > FFT > audio) — slow clients don't block the stream
- Tier-based authentication (Tier 1 / 2 / 3)
- Bi-directional commands: JOIN can tune frequency, create channels, control gain
- Dynamic FFT size and sample rate changes from JOIN
- Remote chassis reset and RX start/stop commands
- IQ_CHUNK streaming: WAN-compatible file transfer via MUX relay

### Station Discovery & Globe
- 3D interactive globe with Blue Marble texture
- UDP broadcast discovery on LAN
- Central relay server for WAN connections (single port 7700)
- LAN-priority connection: auto-detects same-subnet relay and connects locally
- Continuous station list polling with live updates
- Click globe to set your station location

### Collaboration
- Real-time chat between all connected operators
- File sharing (upload / download recordings via public directory)
- Operator list with tier and connection status
- Channel ownership tracking
- Public file deletion by owner

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
                        ┌──────────────────────────────────┐
                        │        CENTRAL RELAY SERVER       │
                        │   port 7700 (MUX)  │  7702 (IQ)  │
                        └──────┬──────────────┬────────────┘
                               │   WAN        │
                 ┌─────────────┴───┐    ┌─────┴───────────┐
                 │   HOST (SDR)    │    │   JOIN Client    │
                 │   port 7701     │    │   (no hardware)  │
                 │  ┌───────────┐  │    └──────────────────┘
                 │  │ BladeRF / │  │         LAN
                 │  │ RTL-SDR   │  ├─── UDP Discovery ───── JOIN Client
                 │  └───────────┘  │
                 └─────────────────┘
```

- **HOST** — Runs the SDR, computes FFT, streams to JOINs (direct on LAN, or via relay on WAN)
- **JOIN** — Connects to HOST, receives spectrum + audio, sends commands
- **Central Relay** — WAN bridge: HOST opens a room, JOINs connect through a single multiplexed port (7700). IQ file transfers use a dedicated pipe port (7702)

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

### Ports

| Port | Role | Description |
|------|------|-------------|
| **7700** | Central Relay | MUX relay — HOST registers station, JOINs connect via relay |
| **7701** | HOST ↔ JOIN | Direct BEWE protocol (LAN or relay-forwarded) |
| **7702** | IQ Pipe | Dedicated IQ file transfer between HOST and JOIN |

### Protocol

- HOST ↔ JOIN communication uses a custom binary protocol (`BEWE` magic) over TCP
- Central relay uses a separate protocol (`BRLY` magic) with MUX headers to multiplex multiple JOINs over one HOST connection
- LAN stations are discovered automatically via UDP broadcast
- WAN stations are reachable through the central relay server
- See `src/net_protocol.hpp` and `central/central_proto.hpp` for internal details

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
│   ├── hw_config.hpp         # Runtime hardware parameters (SR, freq range, gain mapping)
│   ├── demod.cpp             # AM / FM / MAGIC demodulation
│   ├── audio.hpp/cpp         # ALSA output + 5 noise reduction algorithms
│   ├── ais.cpp               # AIS digital decoder
│   ├── dmr.cpp               # DMR digital decoder (AMBE+2)
│   ├── net_protocol.hpp      # Binary protocol specification (BEWE)
│   ├── net_server.hpp/cpp    # HOST-side TCP server
│   ├── net_client.hpp/cpp    # JOIN-side TCP client
│   ├── net_stream.cpp        # Stream utilities
│   ├── central_client.hpp/cpp # Central relay client (HOST/JOIN WAN relay)
│   ├── iq_pipe_server.hpp/cpp # IQ file transfer server (port 7702)
│   ├── udp_discovery.hpp/cpp # LAN station broadcast
│   ├── globe.hpp/cpp         # 3D Earth renderer (OpenGL 3.3)
│   ├── timemachine.cpp       # IQ rolling record & playback
│   ├── region_save.cpp       # Region IQ export
│   ├── sa_compute.cpp        # Signal Analyzer offline FFT
│   ├── iq_record.cpp         # IQ / audio recording
│   ├── login.hpp/cpp         # Authentication & tier selection
│   ├── channel.hpp           # Per-channel state (freq, mode, squelch)
│   ├── config.hpp            # Global constants
│   ├── bewe_paths.hpp        # Path management (recordings, assets, data dirs)
│   └── world_map_data.hpp    # Embedded vector map for globe
├── central/
│   ├── central_main.cpp      # Standalone central relay server
│   ├── central_server.hpp/cpp # Central relay implementation
│   ├── central_proto.hpp     # Central relay protocol (BRLY + MUX)
│   └── CMakeLists.txt        # Build configuration for bewe_central
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

## Data Directory

BE_WE stores recordings and data files under `$HOME/BE_WE/`:

```
$HOME/BE_WE/
├── recordings/
│   ├── record/          # Current session recordings
│   │   ├── iq/
│   │   └── audio/
│   ├── private/         # Previous session recordings
│   │   ├── iq/
│   │   └── audio/
│   ├── public/          # Files shared via server
│   │   ├── iq/
│   │   └── audio/
│   ├── share/           # Files downloaded from public
│   │   ├── iq/
│   │   └── audio/
│   └── Time_temp/       # Time Machine rolling IQ buffer
└── assets/              # Fallback asset directory
```

Directories are created automatically on startup.

---

## License

TBD
