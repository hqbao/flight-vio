# Per-stage VIO profile — Raspberry Pi 5 (real hardware)

Session: `sessions/gold/push_straight_fast_15s` (299 frames, native 640x400, motion-rich).
Harness: `verification/stage_profile.py` (lean flight runtime: numpy + numba, no cv2/scipy).
Measured on `bao@192.168.1.72` (Pi 5, 4-core Cortex-A76 @2.4GHz, aarch64, py3.13), 139 timed frames.

The numbers below are the **serial per-stage wall-clock in ONE process**. The live flight
stack runs these stages in **separate OS processes** (capture / vio / slam), so the achievable
frame rate is set by the **busiest PROCESS**, not the serial sum (see "Per-process pipeline").

## Per-stage cost (ms)

### LOOSE 320x200 — frontend cfg win=11 lvl=2 corners=200 ; SGM downscale=2 ndisp=48
| stage | n | mean ms | p50 | p95 | /frame ms | % |
|---|---|---|---|---|---|---|
| SGM stereo depth | 139 | 9.00 | 9.00 | 9.15 | 9.00 | 17.3% |
| **Frontend (KLT+PnP)** | 139 | **32.35** | 31.67 | 39.92 | **32.35** | **62.3%** |
| Windowed BA loose (1/5 frames) | 27 | 48.37 | 27.34 | 99.00 | 9.39 | 18.1% |
| IMU preintegration (1/5) | 27 | 6.03 | 6.03 | 6.09 | 1.17 | 2.3% |
| **TOTAL (serial)** | | | | | **51.91** | → **19.3 fps** |

### TIGHT 320x200
| stage | n | mean ms | p50 | p95 | /frame ms | % |
|---|---|---|---|---|---|---|
| SGM stereo depth | 139 | 13.30 | 9.17 | 26.36 | 13.30 | 9.7% |
| Frontend (KLT+PnP) | 139 | 34.04 | 32.49 | 43.01 | 34.04 | 24.9% |
| Windowed BA loose (1/5) | 27 | 48.39 | 27.90 | 98.76 | 9.40 | 6.9% |
| IMU preintegration (1/5) | 27 | 6.02 | 6.02 | 6.07 | 1.17 | 0.9% |
| **Tight optimize_vio (1/5)** | 27 | **404.57** | 275.80 | 926.02 | **78.59** | **57.6%** |
| **TOTAL (serial)** | | | | | **136.50** | → **7.3 fps** |

### LOOSE 160x100 — frontend cfg win=7 lvl=1 corners=100 bucketed ; SGM downscale=1 ndisp=32
| stage | n | mean ms | p50 | p95 | /frame ms | % |
|---|---|---|---|---|---|---|
| SGM stereo depth | 139 | 7.75 | 7.74 | 7.90 | 7.75 | 22.6% |
| **Frontend (KLT+PnP)** | 139 | **19.97** | 20.61 | 23.56 | **19.97** | **58.3%** |
| Windowed BA loose (1/5) | 27 | 27.74 | 31.90 | 41.46 | 5.39 | 15.7% |
| IMU preintegration (1/5) | 27 | 5.92 | 5.93 | 5.98 | 1.15 | 3.4% |
| **TOTAL (serial)** | | | | | **34.26** | → **29.2 fps** |

### TIGHT 160x100
| stage | n | mean ms | p50 | p95 | /frame ms | % |
|---|---|---|---|---|---|---|
| SGM stereo depth | 139 | 12.53 | 11.35 | 25.88 | 12.53 | 13.1% |
| Frontend (KLT+PnP) | 139 | 21.25 | 21.18 | 28.77 | 21.25 | 22.3% |
| Windowed BA loose (1/5) | 27 | 27.52 | 31.71 | 41.13 | 5.35 | 5.6% |
| IMU preintegration (1/5) | 27 | 5.86 | 5.88 | 5.94 | 1.14 | 1.2% |
| **Tight optimize_vio (1/5)** | 27 | **284.15** | 297.90 | 400.76 | **55.19** | **57.8%** |
| **TOTAL (serial)** | | | | | **95.46** | → **10.5 fps** |

## Per-process pipeline (the number that actually matters)

The live launcher runs capture / vio / slam as separate `Popen` processes. Group the stages
by the process they run in and the achievable fps = 1000 / (busiest process ms/frame):

### LOOSE 320x200, `--worker` ON (current Pi default — BA in its own process)
- capture proc = SGM 9.0 ms → **111 fps**
- vio proc = frontend 32.4 + IMU 1.2 = **33.5 ms → 30 fps**  ← bottleneck
- BA worker = 48.4 ms/call, fires every 5 vio frames (≈167 ms) → keeps up ✓
→ **pipeline ≈ 30 fps achievable** (vs 19.3 fps serial). The frontend KLT+PnP is the wall.
- This is why `deploy/pi-run.sh` now defaults `--worker` ON (opt out with `--no-worker`).

### LOOSE 320x200, `--worker` OFF (`pi-run.sh --no-worker`)
- vio proc = frontend 32.4 ms every frame **+ 48.4 ms BA burst every 5th frame** (same core,
  GIL-contended) → 80 ms hitch 1-in-5 frames = visible stutter, ≈19 fps effective.
→ This is the stall the default `--worker` avoids — keep it on for loose 320x200.

### TIGHT 320x200
- tight optimize_vio = **404 ms/call** every 5th frame. Even in its own worker process the
  KF period at 20 fps is 250 ms < 404 ms → the worker **cannot keep up**. Tight is not viable
  at 320x200 without rewriting the finite-difference IMU Jacobians (the 404 ms is ~2600
  pure-Python so3/se3 calls per solve).

## Bottleneck summary
1. **Frontend (KLT track + RGB-D PnP)** is the dominant per-frame stage at every resolution
   (58–62% loose). It is the vio-process wall. The KLT pyramid-reuse change
   (`sky/front/klt.py`/`frontend.py`: 1 pyramid build/frame instead of 4) trims the loose
   320×200 frontend ~32.4 → ~28.7 ms (−11%); the table above pre-dates that change.
2. **Loose windowed BA** is a bursty 48 ms/call (1-in-5 frames) → stalls the vio core when
   in-thread; `--worker` fixes it (now the `deploy/pi-run.sh` default; opt out `--no-worker`).
3. **Tight optimize_vio** is catastrophic (284–404 ms/call) — the FD IMU-Jacobian build.
4. **SGM depth** is cheap and not the bottleneck (9–13 ms, its own capture core).
5. With no `NUMBA_NUM_THREADS` set, overlapping SGM+KLT bursts oversubscribe (8 numba
   threads on 4 cores). `--cap-numba-threads` (now the `pi-run.sh` default) pins them per
   process — capture=2, vio=2, slam=1 on the Pi5 — a zero-code, gap-safe lever.
