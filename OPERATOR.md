# BEWE — Operator Guide

This document covers operator workflows, the complete keybinding reference,
and resolution of common operational issues. For installation and platform
provisioning, see [`INSTALL.md`](INSTALL.md).

---

## 1. First-Time Login

1. Launch the workstation client (`./BE_WE`).
2. Provide operator ID and password.
3. Select operator tier (`Ctrl+1` / `Ctrl+2` / `Ctrl+3`). Tier governs the set
   of commands the operator is permitted to issue to a remote HOST (re-tune,
   change FFT size, change receiver, delete archive, etc.).
4. The 3D globe view loads. Live station markers populate as collection nodes
   announce themselves to Central.

To assume control of a site, click the corresponding globe marker:

- **JOIN** — observe and operate the site (multiple operators may join the
  same site concurrently).
- **HOST** — begin collection at the workstation's own location (requires a
  receiver attached to the workstation; only one HOST per machine).

Each operation opens in a separate OS window. The globe remains available as
the master station selector.

---

## 2. Standard Operating Procedure

### 2.1 Site Selection

- The globe shows all active collection nodes as illuminated markers with
  station labels.
- Hover for live coordinates; click an empty location to host a new site at
  that position.
- Multiple operation windows may be open simultaneously and dragged to
  separate monitors.

### 2.2 Channel Deployment

In the operation window:

- Right-click on the waterfall to deploy a demodulator channel at that
  frequency.
- Up to 10 channels may run concurrently (AM / FM modes supported).
- Re-tune off a channel and it slides into a **Holding** list; the squelch
  timer is preserved. Return to the same frequency to reattach.
- Channels and re-tune commands originating from an authorized JOIN are
  applied at the HOST.

### 2.3 Mission Workflow

Press **`M`** to open the Mission archive modal.

- **Start** — A new mission is created with an auto-generated code
  (alphabetical month + day; for example `J15` = October 15).
- **Active operation** — All IQ captures, demodulated audio, and waterfall
  segments produced during the mission are tagged with the mission code and
  consolidated under
  `recordings/missions/<station>/<year>/<code>/{iq,audio,hist}/`.
- **End** — On mission end (manual `End` button, `Ctrl+C` on the host, or
  midnight UTC rollover), remaining files are pushed to Central archive
  automatically.
- **Recovery** — Past missions are visible in the left tree of the Mission
  modal. The tree consolidates host-local missions, in-memory history, and
  the Central archive. Selecting a past mission populates the right pane with
  the full file inventory, available for download.

### 2.4 Recording and Export

| Action | Method |
|---|---|
| Per-channel IQ recording (squelch-gated) | Press `I` on the active channel |
| Region IQ export | `Ctrl+Right-drag` on waterfall to mark a time-frequency region; box displays live `BW / Duration`; press `R` to save |
| Time Machine (60-second rolling IQ) | Press `T` to begin; `Space` to freeze and scroll back |
| File metadata | The **File Info** modal auto-populates frequency, bandwidth, modulation, recorder identity, and UTC up/down times for every recording |

### 2.5 Forensic Inspection (Signal Analyzer)

Press **`E`** to open the Signal Analyzer on the most recent recording, or
right-click any archived file → **Signal Analysis**.

Domain views (`1`–`9` keys to switch):

| Key | View | Use |
|---|---|---|
| `1` | Spectrogram | FFT-vs-time with region select |
| `2` | Amp | Envelope / burst structure |
| `3` | Freq | Instantaneous frequency; auto PRI/PRF/PD/Baud |
| `4` | Phase | Instantaneous phase; carrier sweep |
| `5` | I/Q | Raw baseband |
| `6` | Const | Constellation with carrier recovery |
| `7` | Audio | Demodulated playback |
| `8` | Power | M-th power spectrum (cyclostationary analysis) |
| `9` | Bits | Demodulated bit stream (BIN / HEX / 2D bitmap) |

### 2.6 Cross-Station Emitter Library

Press **`L`** to open the Signal Library overlay.

- On any recording, **Report** ships the `.info` metadata to Central.
- Central aggregates sightings by operator-entered fields (frequency within
  tolerance, modulation, protocol, explicit ID tokens such as `MMSI:...` or
  `callsign:...`).
- Sightings without identifying tags drop into a **Pending** queue for human
  confirm/reject — no silent merges.
- The overlay provides sortable lists, full-text filter, an editable detail
  pane, and a sightings timeline with per-row confirm / reject / split
  actions.

### 2.7 HISTORY Overlay (Long-Term Waterfall Archive)

Press **`H`** to open the HISTORY overlay.

- Each HOST records a continuous waterfall archive segmented by hour.
- Files are mission-coded and embed station name, lat/lon, and UTC range in
  the header.
- Frequency changes automatically rotate to a fresh segment so each file
  represents a single center frequency.
- `Ctrl+Right-drag` measures any region (`BW / Duration`).
- An opt-in live mode lets a JOIN operator observe the archive grow row-by-row
  on a remote site.

---

## 3. Key Bindings Reference

Overlays (`E`, `H`, `L`, `M`) are mutually exclusive — opening one closes
whichever is currently up.

| Key | Action |
|---|---|
| `T` | Start / stop Time Machine rolling capture |
| `Space` | Freeze waterfall (Time Machine scroll-back) |
| `Ctrl+Right-drag` | Mark region on waterfall; press `R` to save as IQ |
| `Scroll wheel` | Zoom frequency axis |
| `I` | Start / stop per-channel IQ recording |
| `P` | Pause / resume spectrum display (FFT freeze) |
| `E` | Toggle Signal Analyzer overlay |
| `H` | Toggle HISTORY (long-waterfall archive) |
| `L` | Toggle Signal Library overlay |
| `M` | Toggle Mission archive modal |
| `S` | Toggle STATUS right panel |
| `B` | Toggle BAND overlay (or baud-mode lines when SA is open) |
| `RightShift` | Toggle chat panel |
| `1`–`9` | Switch domain inside the Signal Analyzer |
| `F11` | Fullscreen / windowed |
| `←` / `→` | Carrier sweep ±1 Hz (SA Phase / Freq) |
| `↑` / `↓` | Carrier sweep ±10 Hz (SA Phase / Freq) |
| `Ctrl+1` / `Ctrl+2` / `Ctrl+3` | Select operator tier at login |

---

## 4. Troubleshooting

### 4.1 Waterfall stutters when joining over WiFi

WiFi power saving aggressively suspends the radio between packets, dropping
spectrum frames.

```bash
sudo iwconfig <interface> power off

# Permanent
sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf <<'EOF'
[connection]
wifi.powersave = 2
EOF
sudo systemctl restart NetworkManager
```

### 4.2 Receiver not detected

**BladeRF**:

```bash
sudo tee /etc/udev/rules.d/88-bladerf.rules <<'EOF'
ATTR{idVendor}=="2cf0", ATTR{idProduct}=="5246", MODE="0660", GROUP="plugdev"
ATTR{idVendor}=="2cf0", ATTR{idProduct}=="5250", MODE="0660", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
```

**ADALM-Pluto** — confirm enumeration:

```bash
iio_info -s
```

(`Bad URI: 'usb:'` with no device attached is normal output.)

**RTL-SDR claimed by kernel DVB driver**:

```bash
sudo modprobe -r dvb_usb_rtl28xxu
echo "blacklist dvb_usb_rtl28xxu" | sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf
```

### 4.3 GPU reports `llvmpipe` (software renderer)

The waterfall will render but at degraded performance.

```bash
# NVIDIA
sudo apt install nvidia-driver-535

# Intel / AMD
sudo apt install mesa-vulkan-drivers
```

### 4.4 Central connection drops periodically

The HOST automatically reconnects every 5 seconds on disconnect. During the
disconnected interval:

- All IQ, demodulated audio, and waterfall data continue recording to local
  disk without interruption.
- The currently active HIST file is flagged as "dirty"; on successful
  reconnect, the file is pushed to Central on next mission rotation to fill
  the gap in the row stream.
- No operator action is required.

To diagnose persistent disconnects, inspect the HOST log for repeated
`Central auto-reconnect attempt N` lines. A growing attempt count indicates
the Central Server is unreachable.

### 4.5 HOST disk space exhausted

The HOST local archive uses a 2-month rotating policy keyed to the mission
code (alphabetic month). On the first mission of month `C`, all month `A`
mission directories are purged from the local disk. The Central archive
retains the data permanently.

If the disk fills before the auto-purge cycle (for example, after extended
high-bandwidth collection), purge older missions manually:

```bash
rm -rf ~/BE_WE/recordings/missions/<station>/<year>/A*
```

This affects only the local copy; the Central archive is unaffected.

### 4.6 Mission archive shows no past missions

If the left tree is empty after a fresh HOST or Central restart:

1. Confirm the operator's site selection matches the station whose missions
   are sought (the tree filters by current station).
2. Wait 30 seconds for the Central archive broadcast to populate the tree.
3. If the tree remains empty, manually press **Refresh** on the file-list
   tab. The tree consolidates host-local missions, in-memory history, and
   the Central archive.

---

## 5. Operator Tiers

Operator tiers govern the privileges an analyst can exert against a HOST.

| Tier | Privileges |
|---|---|
| 1 | Full command (re-tune, change FFT size, swap receiver, delete archive, end mission) |
| 2 | Operational command (re-tune, channel deployment, mission start/end, recording) |
| 3 | Observation only (view waterfall, listen to demodulated channels) |

Tier is selected at login (`Ctrl+1` / `Ctrl+2` / `Ctrl+3`) and is enforced at
the HOST. Unauthorized commands are silently dropped.
