# RPi5 Deploy Runbook — flight-vio FLIGHT runtime

Deploy the from-scratch RGB-D VIO/SLAM flight stack
(`imu_camera → vio → slam`) on a **Raspberry Pi 5 (Debian, aarch64)**, headless.

> **Honesty contract.** Everything below the **"✅ validated on mac dev"** line
> was proven on the macOS development box and is reproduced by named gates in
> this repo. Everything under **"⚠️ MUST verify on the Pi"** is a
> hardware-/architecture-specific unknown that *only the board can answer* —
> aarch64 wheels and real-time throughput. Do not treat a ⚠️ item as done until
> you have run it on the Pi.
>
> **aarch64 wheels: CONFIRMED (2026-06-15)** on a real Pi 5 / Debian 13 (trixie) /
> Python 3.13 — `numpy 2.4.6`, `numba 0.65.1`, `llvmlite 0.47.0`, `pyserial 3.5`,
> `depthai 3.7.1` all install from prebuilt manylinux-aarch64 wheels (no source
> build), import cleanly, and the IPC codec round-trips. **Still open:** live
> real-time throughput (the ~20 Hz target) — measure on the board.

---

## 0. Deploy from the Mac (recommended) — `deploy/`

The whole lifecycle is driven from the Mac; **you never log into the Pi**. The
scripts in `deploy/` share one cached, key-authenticated connection (you enter the
Pi user/host/password ONCE):

```bash
./deploy/pi-discover.sh                      # 1. find the Pi, authenticate once, cache it
./deploy/pi-deploy.sh                        # 2. rsync the repo (code only) + build the aarch64 venv
./deploy/pi-optimize.sh --reboot             # 3. cut boot time (disable unneeded services), reboot, re-measure
./deploy/pi-run.sh --vl53l9cx --direct       # 4. run the flight VIO, headless + detached
./deploy/pi-run.sh --logs                    #    tail the live log
./deploy/pi-stop.sh                          #    stop it
```

Watch the live UI on the Mac over WiFi (`netbridge`):

```bash
export OAKD_NETBRIDGE_KEY=<shared-secret>    # same secret on both ends (authenticated)
./deploy/pi-run.sh --ui --vl53l9cx --direct  # start the stack on the Pi WITH the bridge
./deploy/pi-ui.sh                            # on the Mac: UI auto-connects to the cached Pi
```

> **No secret to manage?** On a trusted home LAN you can skip the key entirely:
> **leave `OAKD_NETBRIDGE_KEY` unset** and both ends fall back to the same built-in
> **default key**, so `pi-run.sh --ui` and `pi-ui.sh` connect with no setup (each
> prints a one-line note). That default is public (it's in the source) — convenience,
> not a secret — so export a real `OAKD_NETBRIDGE_KEY` on both hosts for security on
> an untrusted network. The stream is unencrypted either way (tunnel over SSH if the
> network is untrusted).

Maintenance: `./deploy/pi-discover.sh --reset` (forget the Pi / re-discover after a
DHCP change), `./deploy/pi-discover.sh --status` (show the cached connection),
`./deploy/pi-uninstall.sh [--restore-boot] [--forget]` (remove the stack; optionally
revert the boot optimisation and the SSH key). The connection cache lives in the
gitignored `.cache/pi_connection.env` (0600).

The sections below document the **on-Pi** scripts (`deploy/pi/setup_pi.sh`,
`deploy/pi/optimize_pi.sh`) those host-side commands drive, and the manual path.

---

## 1. Prerequisites (apt)

The flight runtime targets **Python 3.13** and (when a wheel must build from
source) needs the toolchain + dev headers:

```bash
sudo apt update && sudo apt install -y \
    python3.13 python3.13-venv python3.13-dev build-essential
```

`deploy/pi/setup_pi.sh` **checks** for these and prints this exact line if any are
missing — it never runs `sudo` itself.

> If Raspberry Pi OS does not ship `python3.13` in its repos for your release,
> see **Troubleshooting → Python 3.13 on the Pi**.

---

## 2. Bootstrap

### Option A — one script (recommended)

```bash
git clone git@github.com:hqbao/flight-vio.git && cd flight-vio   # or copy the repo onto the Pi
./deploy/pi/setup_pi.sh
```

It is **idempotent** (re-running reuses an existing `.venv`) and does:

1. check the apt prerequisites (prints the `sudo apt install` line if missing);
2. create `.venv` with `python3.13`, `pip install -U pip`,
   `pip install -r requirements-flight.txt`;
3. run the **validation smoke** — codec round-trip + a headless `--no-ui` replay
   + the cv2-absent litmus — and print PASS/FAIL + next steps.

`./deploy/pi/setup_pi.sh --no-smoke` bootstraps only (skips the validation run).

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

**Clean stop.** A single `Ctrl-C` (or `SIGTERM`) shuts the whole stack down
cleanly — each process handles `SIGINT`/`SIGTERM` identically (set a stop flag,
short-circuit the drain, release SHM rings + close the OAK-D in an order the
firmware watchdog tolerates), and the launcher ignores further `Ctrl-C` while
tearing down so a second press can never abort cleanup or print a traceback.

### 3a. FC output — the pose stream

`--no-ui` **is** the FC-output path. The launcher attaches a pose logger
(`_start_pose_logger` → `_on_pose` in `launcher/main.py`) that subscribes to the
VIO's `pose.odom` topic and prints each pose, throttled to ~2 Hz:

```
pose: pos WORLD=(+0.005 -0.027 -0.003) m  quat wxyz=(+0.003 +0.017 -0.025 +0.999)  sig_pos=0.076m  n=14
```

| field | meaning |
|---|---|
| `pos WORLD` | position in the VIO world frame, metres (`wm.T_world_cam[:3,3]`) |
| `quat wxyz` | orientation quaternion — the FC derives heading from it |
| `sig_pos`   | **position noise σ (m)** for the FC ESKF: `R ≈ sig_pos²`. Only on `--direct` (`wm.info["pos_sigma_m"]`); σ ∝ Z/√N, clamped [0.05, 3.0] m. High at startup / few features / far scene; ~0.07 m when tracking well |
| `n`         | cumulative pose-message counter (the print is throttled, not every msg) |

**Wiring the real FC link (TODO — not done yet).** The log line is a *preview*.
To feed the FC, send a MAVLink `VISION_POSITION_ESTIMATE` from inside `_on_pose`
(marked `# === FC OUTPUT HOOK`): pose from `wm.T_world_cam`, covariance from
`wm.info["pos_sigma_m"]`. Two caveats before flight:

- **Frame.** `WORLD` is gravity-aligned *optical* (+Y down), **not** NED. Rotate
  the position *and* the 3×3 covariance into the FC frame before sending — don't
  ship raw axes.
- **Loop closure is a JUMP, not a measurement.** When SLAM closes a loop the pose
  steps discontinuously; signal it with the MAVLink `reset_counter` (+ re-anchor
  the ESKF), never as a fused position update — a fused jump injects phantom
  velocity. VIO is odometry and drifts, so prefer fusing velocity where the FC
  supports it. (`sig_pos` is the deliberately-simple first model — commit
  `7f9769a`; the principled upgrade is the inverse-Hessian marginal covariance
  with NEES calibration, left as future work since the FC floors the value.)

### 3b. Remote UI over WiFi (`netbridge`)

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
./deploy/pi-ui.sh --connect <pi-host>:8787
```

`--forward HOST:PORT` spawns `netbridge.forward` as one more managed flight
subprocess (torn down with the rest). On the Mac, `deploy/pi-ui.sh` starts
`netbridge.receive` (which sizes its rings from the forwarded `calib.bundle`, so a
54×42 ToF run is re-served at 54×42, not 640×400), waits for the re-served
sockets, then runs the unchanged `ui.main`.

**Security (HONEST):** `OAKD_NETBRIDGE_KEY` is a shared HMAC secret that
**authenticates** the peer at connect time (one-time handshake, not per frame) — a
wrong key is refused. If it is **unset**, both ends use a built-in **default key**
(public, in the source) so the bridge connects with no setup on a trusted LAN — that
is convenience auth, not a secret, so export a real key on both hosts for security.
It does **not encrypt** the stream (LAN threat model). For an untrusted network,
tunnel it: `ssh -L 8787:localhost:8787 pi@<pi-host>` then
`./deploy/pi-ui.sh --connect 127.0.0.1:8787` (or run over Wireguard) — the tunnel
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

# [✅-5] netbridge copied clean — TCP bridge round-trips on loopback (no 2nd box).
.venv/bin/python verification/netbridge_loopback_selftest.py
#   (gate: verification/netbridge_loopback_selftest.py — confirms 3b will work)
```

> Note: the litmus passing on the Pi additionally proves the *aarch64* install is
> genuinely cv2-free at runtime — i.e. the lean `requirements-flight.txt` is
> sufficient on the board, not just on the dev box.

### ⚠️ MUST verify on the Pi (hardware / arch — only the board can answer)

These cannot be answered on the macOS dev box. Treat each as open until it passes
on the actual Pi 5.

```bash
# [✅-1] aarch64 flight-deps install — CONFIRMED 2026-06-15 (Pi 5 / Debian 13 /
#        py3.13). numpy 2.4.6, numba 0.65.1, llvmlite 0.47.0, pyserial 3.5,
#        depthai 3.7.1 ALL install from prebuilt manylinux-aarch64 wheels (no
#        source build), import cleanly, and the IPC codec round-trips. Driven by:
./deploy/pi-deploy.sh          # rsync + venv + the import/codec validation
#   (The Troubleshooting fallbacks below apply only if YOUR pinned versions have
#    no aarch64 wheel — the current pins do.)

# [⚠️-2] cv2-absent litmus rc=0 ON aarch64.  (codec round-trip already PASSES on
#        the Pi; the litmus replays a gold session, which the lean deploy does NOT
#        ship — sync one session to run it, or rely on the live run below.)
.venv/bin/python -m verification.cv2_absent_flight_litmus --session <a-synced-session>
#   Expect rc=0. Proven on mac; re-prove on the Pi's CPython/arch.

# [⚠️-3] headless replay rc=0 ON aarch64 (also needs a synced gold session).
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
> Note: `depthai 3.7.1` **does** ship a prebuilt aarch64 (py3.13) wheel — confirmed
> installing on the Pi 2026-06-15. This fallback applies only if you pin a version
> that lacks one.

`depthai` is the OAK-D device driver — needed **only** for `--live` capture.
Because of the project's VL53-ToF pivot, depthai is **optional**: headless
**replay** (`--session ...`) and any non-OAK source run **without it**. Options,
in order: (a) install only when you actually attach the OAK-D; (b) build depthai
from source for aarch64 (needs the build toolchain); (c) drop the `depthai` line
from `requirements-flight.txt` for a replay-only board. If you skip it, the
import is only reached on the live path, so replay validation still runs.

**numba / llvmlite has no aarch64 wheel / build is heavy.**
> Note: `numba 0.65.1` + `llvmlite 0.47.0` **do** ship prebuilt aarch64 (py3.13)
> wheels — confirmed on the Pi 2026-06-15. This applies only if you pin versions
> without one.

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
`python3.13-venv`, and `python3.13-dev` resolve, `deploy/pi/setup_pi.sh` proceeds
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

> ### depthai aarch64 wheel — RESOLVED for the flight stack
> `depthai` is the **only** native device dependency. As in §4/§5 it now has a
> prebuilt aarch64 (py3.13) wheel (`3.7.1`, confirmed on the Pi 2026-06-15), so the
> flight install needs no source build. If you pin a version without one it must
> build from source (`build-essential` + `python3.13-dev`); a **replay/viewer-only**
> baseline board can drop the `depthai` line entirely — it is imported **lazily**
> inside the Basalt sources' worker thread, so `FakePoseSource`, `viz_session`, and
> the Qt viewer all run without it. **Still open for a baseline VIEWER board:**
> PyQt6/PyOpenGL aarch64 wheels (the flight Pi is headless, so it avoids them).
