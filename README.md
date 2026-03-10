# Mode 5 IFF Communication System

**IFF (Identification Friend or Foe)** implementation using software-defined radios.

| Role | Hardware | IP | Frequency |
|------|----------|----|-----------|
| Ground Station (Interrogator) | ADALM Pluto SDR | `ip:192.168.2.1` | TX 1030 MHz / RX 1090 MHz |
| Aircraft (Transponder) | ZedBoard + FMCOMMS3 (AD9361) | `192.168.1.10` | RX 1030 MHz / TX 1090 MHz |

**Connection:** Direct SMA cable between Pluto and ZedBoard.

---

## Prerequisites

### Software
- MATLAB R2023b or newer
- Communications Toolbox
- Communications Toolbox Support Package for Analog Devices ADALM-Pluto Radio
- Communications Toolbox Support Package for Xilinx Zynq-Based Radio (for ZedBoard/FMCOMMS3)

### Hardware Setup
1. **ADALM Pluto SDR** connected via USB to your PC
2. **ZedBoard + FMCOMMS3** connected via:
   - Ethernet cable to your PC (for data/control)
   - USB serial cable to your PC (for IP configuration, COM8)
   - SMA cable from FMCOMMS3 TX/RX to Pluto TX/RX
3. **PC Network Configuration:**
   - Pluto: Automatic (shows up as `192.168.2.1` over USB)
   - ZedBoard Ethernet adapter on your PC: Static IP `192.168.1.1`, Subnet `255.255.255.0`

---

## Quick Start

### Step 1 — Power on and configure ZedBoard IP

Every time ZedBoard is power-cycled, its IP must be reset:

```matlab
>> run('matlab_codes/reset_zedboard_ip.m')
```

Verify connectivity:
```matlab
>> !ping -n 2 192.168.1.10
```
You should see replies. If not, check Ethernet cable and PC adapter settings.

### Step 2 — Verify Pluto is connected

```matlab
>> !ping -n 2 192.168.2.1
```

### Step 3 — Run the IFF system

**You need two MATLAB windows** (one per device).

#### MATLAB Window 1 — Aircraft Transponder (ZedBoard)
```matlab
>> run('matlab_codes/test_adsb_zedboard.m')
```
This starts listening on 1030 MHz. It will wait for an interrogation.

#### MATLAB Window 2 — Ground Station (Pluto)
```matlab
>> run('matlab_codes/test_adsb_pluto.m')
```
This sends an interrogation on 1030 MHz, then listens on 1090 MHz for the aircraft's reply.

**Start ZedBoard FIRST, then Pluto within 15 seconds.**

### Expected Output

**Ground Station (Pluto) console:**
```
=== Interrogation 1/5 ===
  [TX] Interrogation on 1030 MHz for 3.0 s ...
  [TX] Interrogation sent.
  [RX] Listening for reply on 1090 MHz (6.0 s) ...
  ┌──────────────────────────────────────────┐
  │  AIRCRAFT REPLY RECEIVED                 │
  │  ICAO addr  : 4840D6                     │
  │  STATUS     : AIRCRAFT IDENTIFIED        │
  └──────────────────────────────────────────┘
```

**Aircraft Transponder (ZedBoard) console:**
```
=== Listening — Cycle 1/5 ===
  [RX] INTERROGATION RECEIVED
       Ground ID  : 1A2B
       Challenge  : D2CE21DA
  [TX] Replying on 1090 MHz ...
       ICAO addr  : 4840D6  (this aircraft)
  [TX] Reply sent — aircraft identity transmitted.
```

---

## File Descriptions

| File | Purpose |
|------|---------|
| `matlab_codes/test_adsb_pluto.m` | **IFF Ground Station** — Pluto interrogates on 1030 MHz, receives reply on 1090 MHz |
| `matlab_codes/test_adsb_zedboard.m` | **IFF Aircraft Transponder** — ZedBoard listens on 1030 MHz, replies on 1090 MHz |
| `matlab_codes/test_adsb_link.m` | Combined single-script ADS-B link test (both directions in one MATLAB session) |
| `matlab_codes/demo_comm.m` | Mode 5 IFF with BPSK + CRC-16 (next-step: DSSS + MAC to be re-added) |
| `matlab_codes/reset_zedboard_ip.m` | Reset ZedBoard IP to `192.168.1.10` via serial port (COM8, 115200 baud) |
| `matlab_codes/slow_simulation_IFF.m` | ZedBoard-only loopback test (legacy) |
| `matlab_codes/test5.m` | Early test script (legacy) |
| `documentation.doc` | Full technical report: 5-bug analysis + project evolution log |

---

## IFF Protocol Flow

```
Ground Station (Pluto)                    Aircraft (ZedBoard)
──────────────────                        ───────────────────

TX interrogation on 1030 MHz              Listening on 1030 MHz ...
[DF=B0][ID=1A2B][Mode=05]     ─────────►  Decode → validate DF=B0
[Challenge][CRC]                           Extract station ID, challenge

Switch to RX on 1090 MHz                  Build reply:
Listening for reply ...                    [DF=8D][ICAO=4840D6]
                               ◄─────────  [Squawk/IFF][CRC]
Decode reply                               TX reply on 1090 MHz
Extract ICAO = 4840D6
"AIRCRAFT IDENTIFIED"
```

---

## Gain Settings (SMA Cable)

| Parameter | Value | Notes |
|-----------|-------|-------|
| TX Pluto | -20 dB | |
| TX ZedBoard | -20 dB | |
| RX ZedBoard | +10 to +25 dB | Adjust if max(abs) too low/high |
| RX Pluto | +10 dB | |
| Target max(abs(rx)) | 0.2 – 0.6 | Linear range, no clipping |

> **Warning:** max(abs) ≥ 1.0 means ADC clipping. Reduce gains immediately.  
> max(abs) < 0.02 means no signal. Check cable and increase gains.

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Failed to create context for uri: ip:192.168.1.10` | ZedBoard IP not configured | Run `reset_zedboard_ip.m`, verify with ping |
| `Input X must be complex` | PlutoSDR rejects real waveform | Already fixed in code (`sig + 1i*1e-12`) |
| NO REPLY on ground station | Timing mismatch or ZedBoard offline | Start ZedBoard first, then Pluto within 15s |
| BER = 50% with high SNR | LO phase ambiguity (BPSK only) | Use complex xcorr phase recovery (in demo_comm.m) |
| All 1-bits decoded as 0 | highpass() on envelope | Use movmean DC removal (already fixed) |
| ADC clipping (max > 1.0) | Gain too high for SMA cable | TX=-20 dB, RX=+10 dB |

---

## IP Core Targets (Vivado 2023.2)

Three functions in `test_adsb_zedboard.m` are designed for MATLAB HDL Coder:

| Function | HDL Implementation | Description |
|----------|-------------------|-------------|
| `adsb_preprocess()` | Sliding-window accumulator + max tracker | Envelope extraction + DC removal |
| `adsb_framesync()` | 96-tap FIR with ROM coefficients | Bipolar preamble matched filter |
| `adsb_decode_bits()` | Adder + comparator (no divider) | PPM half-period energy comparison |

All three have fixed-size I/O, no persistent state, and no floating-point division.

---

## Roadmap

- [x] RF link validation (test_adsb_link.m — 0% BER)
- [x] Split into ground station + aircraft transponder
- [x] Proper IFF protocol (interrogation → reply → identification)
- [ ] BPSK + CRC-16 validation working end-to-end (demo_comm.m)
- [ ] Re-add DSSS spreading (Gold-31 code)
- [ ] Re-add MAC authentication layer
- [ ] HDL Coder → Vivado 2023.2 IP core generation for ZedBoard
- [ ] Full Mode 5 IFF with crypto running autonomously on FPGA

---

## Repository

- **GitHub:** https://github.com/fragilepotato/mode-5
- **Branch:** master
