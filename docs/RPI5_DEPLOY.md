# RPi5 Deploy Runbook — oak-d FLIGHT runtime

Deploy the from-scratch RGB-D VIO/SLAM flight stack
(`imu_camera → vio → slam`) on a **Raspberry Pi 5 (Debian, aarch64)**, headless.

> **Honesty contract.** Everything below the **"✅ validated on mac dev"** line
> was proven on the macOS development box and is reproduced by named gates in
> this repo. Everything under **"⚠️ MUST verify on the Pi"** is a
> hardware-/architecture-specific unknown that *only the board can answer* —
> aarch64 wheels and real-time throughput. Those are **not** claimed as tested.
> Do not treat a ⚠️ item as done until you have run it on the Pi.

---

## 1. Prerequisites (apt)

The flight runtime targets **Python 3.13** and (when a wheel must build from
source) needs the toolchain + dev headers:

```bash
sudo apt update && sudo apt install -y \
    python3.13 python3.13-venv python3.13-dev build-essential
```

`scripts/setup_pi.sh` **checks** for these and prints this exact line if any are
missing — it never runs `sudo` itself.

> If Raspberry Pi OS does not ship `python3.13` in its repos for your release,
> see **Troubleshooting → Python 3.13 on the Pi**.

---

## 2. Bootstrap

### Option A — one script (recommended)

```bash
git clone <repo> oak-d && cd oak-d        # or copy the repo onto the Pi
./scripts/setup_pi.sh
```

It is **idempotent** (re-running reuses an existing `.venv`) and does:

1. check the apt prerequisites (prints the `sudo apt install` line if missing);
2. create `.venv` with `python3.13`, `pip install -U pip`,
   `pip install -r requirements-flight.txt`;
3. run the **validation smoke** — codec round-trip + a headless `--no-ui` replay
   + the cv2-absent litmus — and print PASS/FAIL + next steps.

`./scripts/setup_pi.sh --no-smoke` bootstraps only (skips the validation run).

### Option B — manual venv

```bash
python3.13 -m venv .venv
.venv/bin/pip install -U pip
.venv/bin/pip install -r requirements-flight.txt
```

`requirements-flight.txt` is the **lean flight install**: `numpy`, `numba`,
`pyserial`, `depthai` — **no OpenCV, no Qt**. (Use the full `requirements.txt`
only on a dev box that needs the Qt UI / calibration wizard / parity tooling;
the Pi does not.)

---

## 3. Run

The Pi runs **headless** — always pass `--no-ui`. The UI is optional and can run
remotely on a dev box (it consumes the same abstract IPC topics).

```bash
# Headless flight replay (no hardware needed — validates the full stack):
./run.sh --no-ui --session sessions/gold/lab_loop_30s

# Live flight, once the OAK-D / VL53 ToF is attached (the 54×42 ToF recipe):
./run.sh --no-ui --vl53l9cx --direct

# Live capture only, to confirm the device opens:
./run.sh --no-ui --vl53l9cx
```

`--no-ui` spawns exactly the three flight processes
(`imu_camera.main`, `vio.main`, `slam.main`) and **never** the Qt UI — verified
in the gate below. `--vl53l9cx` selects the VL53-class ToF source (downsample to
54×42); `--direct` selects the dense direct photometric VO front-end tuned for
that low-res ToF recipe.

### 3a. Remote UI over WiFi (`netbridge`)

The UI can run **live on a Mac** against the Pi's flight stack, over TCP/WiFi —
the `netbridge` project bridges the Pi's local IPC to the Mac. The UI is
**byte-for-byte unchanged**: the Mac re-serves the same `oak.capture` / `oak.vio`
/ `oak.slam` endpoints the UI already consumes.

```bash
# --- on the Pi: run the flight stack WITH the bridge (additive to --no-ui) ---
export OAKD_NETBRIDGE_KEY=$(openssl rand -hex 32)   # shared secret (see below)
./run.sh --no-ui --vl53l9cx --direct --forward 0.0.0.0:8787

# --- on the Mac: run the UI against the Pi (SAME secret) ---
export OAKD_NETBRIDGE_KEY=<the-same-secret-from-the-Pi>
./run-ui-remote.sh --connect <pi-host>:8787
```

`--forward HOST:PORT` spawns `netbridge.forward` as one more managed flight
subprocess (torn down with the rest). On the Mac, `run-ui-remote.sh` starts
`netbridge.receive` (which sizes its rings from the forwarded `calib.bundle`, so a
54×42 ToF run is re-served at 54×42, not 640×400), waits for the re-served
sockets, then runs the unchanged `ui.main`.

**Security (HONEST):** `OAKD_NETBRIDGE_KEY` is a shared HMAC secret that
**authenticates** the peer — a wrong/missing key is refused, no silent open
socket. It does **not encrypt** the stream (LAN threat model). For an untrusted
network, tunnel it: `ssh -L 8787:localhost:8787 pi@<pi-host>` then
`./run-ui-remote.sh --connect 127.0.0.1:8787` (or run over Wireguard) — the tunnel
provides the encryption and netbridge sees only loopback.

See `netbridge/README.md` for the full design + the gate
(`verification/netbridge_loopback_selftest.py`).

---

## 4. BOARD-ARRIVAL VALIDATION CHECKLIST

Ordered, copy-pasteable. Run **top to bottom**. The split is deliberate: the ✅
items are already proven on the dev box (re-run them to confirm the transfer was
clean); the ⚠️ items are the genuine Pi-only unknowns.

### ✅ Validated on mac dev (already proven — re-run to confirm the port copied clean)

These are gated in-repo on macOS; they exercise pure-software paths (IPC, codec,
headless flight, byte-parity) that are architecture-independent in design. Cited
gate in parentheses.

```bash
# [✅-1] gap=0 byte-parity oracle — the split math reproduces the frozen baseline.
.venv/bin/python verification/oracle_replay_selftest.py
#   (gate: verification/oracle_replay_selftest.py — PASS on mac, gap=0.000e+00)

# [✅-2] IPC codec round-trip + cross-copy byte-parity digests.
.venv/bin/python -m imu_camera.tests.codec_roundtrip_selftest
#   (gate: imu_camera/tests/codec_roundtrip_selftest.py — 26/26 vectors on mac)

# [✅-3] headless --no-ui replay runs the flight stack with NO Qt import.
./run.sh --no-ui --session sessions/gold/lab_loop_30s --max-frames 30
#   (gate: launcher --no-ui path spawns only imu_camera/vio/slam — rc=0 on mac)

# [✅-4] cv2-free flight — full --vl53l9cx --direct flight runs with cv2 BLOCKED.
.venv/bin/python -m verification.cv2_absent_flight_litmus --max-frames 30
#   (gate: verification/cv2_absent_flight_litmus.py — LITMUS PASSED on mac)
```

> Note: the litmus passing on the Pi additionally proves the *aarch64* install is
> genuinely cv2-free at runtime — i.e. the lean `requirements-flight.txt` is
> sufficient on the board, not just on the dev box.

### ⚠️ MUST verify on the Pi (hardware / arch — only the board can answer)

These cannot be answered on the macOS dev box. Treat each as open until it passes
on the actual Pi 5.

```bash
# [⚠️-1] aarch64 flight-deps install — esp. numba/llvmlite AND depthai wheels.
.venv/bin/pip install -r requirements-flight.txt
#   OPEN UNKNOWN: do prebuilt aarch64 wheels exist for your numba/llvmlite and
#   depthai versions? If not, they build from source (needs build-essential +
#   python3.13-dev) or depthai may have no aarch64 wheel at all.
#   → see Troubleshooting (numba aarch64 / depthai aarch64).

# [⚠️-2] cv2-absent litmus rc=0 ON aarch64.
.venv/bin/python -m verification.cv2_absent_flight_litmus
#   Expect rc=0. Proven on mac; re-prove on the Pi's CPython/arch.

# [⚠️-3] headless replay rc=0 ON aarch64.
./run.sh --no-ui --session sessions/gold/lab_loop_30s
#   Expect rc=0 and slam keyframe count growing.

# [⚠️-4] REAL-TIME PERF — does the live ToF pipeline hold ~20 Hz?  *** THE open
#        unknown the dev box CANNOT answer. ***
./run.sh --no-ui --vl53l9cx --direct        # OAK-D / ToF attached
#   MEASURE ms/frame in the logs across imu_camera → vio → slam. Target ≈ 20 Hz
#   (≤ ~50 ms/frame end-to-end). If numba has no aarch64 wheel and the pure-NumPy
#   fallback is in use, expect this to be SLOWER — this is exactly the number the
#   Pi must produce. If it under-runs 20 Hz: confirm numba is active, then
#   consider lowering --fps.

# [⚠️-5] live OAK-D capture opens on the Pi.
./run.sh --no-ui --vl53l9cx
#   Confirm the device enumerates over USB3 and frames flow (depthai + udev).
```

---

## 5. Troubleshooting

**depthai has no aarch64 wheel / fails to install.**
`depthai` is the OAK-D device driver — needed **only** for `--live` capture.
Because of the project's VL53-ToF pivot, depthai is **optional**: headless
**replay** (`--session ...`) and any non-OAK source run **without it**. Options,
in order: (a) install only when you actually attach the OAK-D; (b) build depthai
from source for aarch64 (needs the build toolchain); (c) drop the `depthai` line
from `requirements-flight.txt` for a replay-only board. If you skip it, the
import is only reached on the live path, so replay validation still runs.

**numba / llvmlite has no aarch64 wheel / build is heavy.**
numba only **JIT-accelerates** the pure-NumPy hot paths (SGM cost volume,
optical flow, etc.). The runtime has a **pure-NumPy fallback** and runs
correctly without numba — just **slower** (this is noted in
`requirements-flight.txt`). If the wheel won't install: ensure
`python3.13-dev` + `build-essential` are present for a source build, or remove
`numba` from the install to run the fallback. Functionally identical;
performance only — which directly feeds the ⚠️-4 real-time measurement.

**Python 3.13 not in the Pi OS repos.**
If your Raspberry Pi OS release doesn't package `python3.13`, install it via the
`deadsnakes` PPA (if available for your base), `pyenv`, or build from source.
The stack requires 3.13 (it uses 3.13-era syntax/stdlib). Once `python3.13`,
`python3.13-venv`, and `python3.13-dev` resolve, `scripts/setup_pi.sh` proceeds
unchanged.

**Qt / display errors.**
Don't install Qt on the Pi and **always** pass `--no-ui`. The flight processes
never import PyQt6 on the headless path (the launcher imports a constant from
`ui.main`, but PyQt6 is imported lazily *inside* `run_ui`, which `--no-ui` never
calls). Run the UI remotely on a dev box if you want visualisation.

---

## 6. Why the core is already portable (audit)

The portability audit concluded the **code** is Linux/aarch64-ready by
construction — these are the load-bearing facts the reader can trust without
re-deriving:

- **IPC codec is endian-safe.** The wire codec encodes with explicit
  **big-endian** byte order (network order), so x86-64 (mac/dev) ↔ aarch64 (Pi)
  produce identical bytes — frozen and gated by the cross-copy digests in
  `imu_camera/tests/codec_roundtrip_selftest.py` (`codec_vectors.json`).
- **Shared-memory names ≤ 30 chars.** POSIX shm names stay within the portable
  limit, so `/dev/shm` on the Pi accepts them.
- **AF_UNIX socket paths < 104 chars.** The IPC endpoint sockets live under a
  short tmp dir, inside the `sun_path` limit on Linux.
- **Headless by design.** UI is a separate process consuming abstract IPC
  topics; `--no-ui` runs the flight stack with no windowing system and no Qt
  import in the flight processes.
- **cv2-free flight runtime.** The frontend is library-free (own KLT/PnP/corners,
  own ORB loop closure, own SGM stereo + dense direct VO, pure-Python PNG codec,
  factory calibration). OpenCV is **not** in `requirements-flight.txt` — proven
  by `verification/cv2_absent_flight_litmus.py` (the full `--vl53l9cx --direct`
  replay runs at rc=0 with `import cv2` blocked).

What remains genuinely Pi-only is therefore **not the code** but the **arch
wheels** (numba/llvmlite, depthai) and the **real-time throughput** — captured
as the ⚠️ items in §4.

---

## 7. Running the baseline on the Pi

`baseline/` is the **independent DepthAI/Basalt reference pipeline** — it runs
the OAK-D's **on-chip** BasaltVIO + RTABMapSLAM blobs over `depthai` and renders
their pose in a Qt 3D viewer (it imports **no** `ours`/`sky`, so it is not on the
gap=0 oracle). Like the flight stack it is **cv2-free**: the recorder reads/writes
frames with the pure-Python PNG codec (`baseline/capture/pngio.py`), and the
offline session viewer (`baseline/tools/viz_session.py`) colourises depth with a
NumPy Turbo LUT — no OpenCV anywhere.

### Install

```bash
.venv/bin/pip install -r requirements-baseline.txt
```

`requirements-baseline.txt` is the **lean baseline install**: `numpy` + `depthai`
(the real dependency — the OAK-D on-chip Basalt source) plus the viewer trio
`PyQt6` + `pyqtgraph` + `PyOpenGL`. **No OpenCV.**

- **Headless / FC-only board** → you only need **`depthai` + `numpy`**. Drop the
  Qt trio (PyQt6 / pyqtgraph / PyOpenGL). Run the **live pose stream with NO UI**:

  ```bash
  ./run-baseline.sh --no-ui --source oak     # Basalt VIO, headless; pose -> stdout
  ```

  `--no-ui` runs the Basalt source headless and streams the pose to stdout — the
  **FC-output path**: wire the MAVLink `VISION_POSITION_ESTIMATE` send into
  `_on_pose()` in `baseline/tools/view_pose3d.py`. It imports no Qt, and Ctrl+C /
  SIGTERM tears the OAK-D down cleanly (no firmware crash on quit). Record-only
  (`baseline/tools/record_session.py`) is likewise Qt-free.
- **Viewer board (dev)** → add the Qt trio for the live 3D pose viewer
  (`./run-baseline.sh`, no `--no-ui`) and the offline session inspector
  (`baseline/tools/viz_session.py`).

### Validate (cv2-free, runs on the dev box too)

```bash
.venv/bin/python -m verification.cv2_absent_baseline_litmus
#   (gate: verification/cv2_absent_baseline_litmus.py — LITMUS PASSED)
```

This imports the whole baseline surface (sources/capture/ui/tools.viz_session)
and runs two cv2-free runtime slices (`FakePoseSource` + the `viz_session` PNG
decode → NumPy Turbo depth) with `import cv2` **blocked**. depthai/OAK-D is not
exercised here (no device on the dev box); it is reached only on the live path.

> ### ⚠️ depthai aarch64 wheel — same caveat as the flight stack
> `depthai` is the **only** native device dependency and the **same open unknown**
> as in §4/§5: do prebuilt **aarch64** wheels exist for your `depthai` version? If
> not it must build from source (needs `build-essential` + `python3.13-dev`), or
> there may be no aarch64 wheel at all. A **replay/viewer-only** baseline board
> can drop the `depthai` line entirely — `depthai` is imported **lazily** inside
> the Basalt sources' worker thread, so `FakePoseSource`, `viz_session`, and the
> Qt viewer all run without it. PyQt6/PyOpenGL aarch64 wheels are the other arch
> unknown for a viewer board; a headless record-only board avoids them.
