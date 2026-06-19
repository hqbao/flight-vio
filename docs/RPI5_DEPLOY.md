# RPi5 Deploy Runbook — flight-vio FLIGHT runtime

Deploy the from-scratch RGB-D VIO/SLAM flight stack
(`imu_camera → vio → ba → slam`) on a **Raspberry Pi 5 (Debian, aarch64)**, headless.

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

## 1b. OAK-D camera on the Pi5 (depthai pin + power + Lite vs W)

**⚠️ OAK-D Lite crash-loop = depthai 3.7.x firmware bug — pin `depthai<3.7`.**
Symptom in `run.log` (the Lite "doesn't work", UI stays empty):
```
[depthai][error] Couldn't read data from stream '__x_..' (X_LINK_ERROR)
[depthai][error] Device <MxId> has crashed. Crash dump stored in ...
```
right after `live src=... pub=...`, repeating. The crash dump's real cause is a
device-FIRMWARE fault:
```
RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE  in  PlgSrcMipi.cpp (the MIPI camera source plugin)
```
**depthai 3.7.x ships a firmware regression that crashes the OAK-D Lite's
OV7251 camera stream on start, at ANY resolution.** It is **NOT** a power/cable/USB
issue — proven: the device idles stable (0 disconnects), a bare `dai.Device()`
open holds for 10 s, and only **camera streaming** trips it. **depthai 3.6.x
streams the Lite fine** (the OAK-D **W** happens to survive 3.7.x — this bites the
Lite specifically). `requirements*.txt` pin `depthai>=3.6,<3.7`; if a board already
got 3.7: `.venv/bin/pip install 'depthai==3.6.1'`. Confirm the fix: `imu_camera.main
--live` should print `live src=...@20` with **no** `X_LINK_ERROR` / crash.

**USB power (separate good-practice, not the crash above).** The Pi5 default-caps
total USB current at 600 mA; an OAK under load is happier with the cap lifted and a
USB3 link. `deploy/pi/optimize_pi.sh` sets `usb_max_current_enable=1` in
`/boot/firmware/config.txt` (idempotent, `--rollback` removes it; needs a 5V/5A PSU,
keep `vcgencmd get_throttled`=`0x0`); prefer a **USB3 (blue) port + USB3 cable**
(`lsusb -t` shows `5000M`, not `480M`). Reboot to apply. (This did **not** fix the
Lite crash — that was the depthai pin above.)

**OAK-D Lite vs OAK-D W — fast-motion VIO.** The mono FOVs differ a lot:
**W ≈ 97° HFOV** (the "W" = Wide, designed for VIO/SLAM) vs **Lite ≈ 70°**. On a
**fast push** the Lite's narrower FOV whips features out of frame → the KLT/PnP
frontend loses tracks → **loose pose stalls** (sticks in one place) and the **tight
backend's window is under-constrained → it lurches then snaps back** (moves a stretch
then jerks backward). This is largely a **hardware-FOV limit**, not a regression:
prefer the **OAK-D W for fast-motion** work; the Lite is fine for slower motion.

**OAK-D Lite IMU→cam extrinsic.** The Lite's BMI270 EEPROM ships a **wrong nominal
`Rx(90°)`** IMU→cam rotation (the W's `diag(1,−1,−1)` is correct); a wrong
extrinsic corrupts IMU-assisted tracking. Verify/correct per device with the
`imu_camera.tools.imu_cam_calib` pose wizard (stores a `calib_store` override that
`live_calib` applies over the EEPROM) or an EEPROM flash. Device
`19443010A12F157E00` here was flashed to the correct `diag(1,−1,−1)` — confirm
with `cal.getImuToCameraExtrinsics(CAM_B)` on the live device.

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

# Live flight + stream the pose to the drone FC over UART (dblink); see §3a:
./run.sh --no-ui --vl53l9cx --direct --fc /dev/ttyAMA0

# Live capture only, to confirm the device opens:
./run.sh --no-ui --vl53l9cx
```

`--no-ui` spawns the flight processes
(`imu_camera.main`, `vio.main`, `ba.main`, `slam.main`) and **never** the Qt UI —
verified in the gate below. The windowed bundle adjustment is its own `ba`
process (`pose.refined`/`ba.state` are re-emitted onto the VIO endpoint via the
pass-through, so the UI is unchanged). `--no-ba` drops the `ba` process and
`--no-slam` drops `slam` — the **lean flight config** for the 4-core Pi (see the
saturation note below). `--vl53l9cx` selects the VL53-class ToF source (downsample to
54×42); `--direct` selects the dense direct photometric VO front-end tuned for
that low-res ToF recipe. `--fc PORT[:BAUD]` (e.g. `/dev/ttyAMA0`) additionally streams
the VIO pose to the drone FC over UART — see §3a.

**Clean stop.** A single `Ctrl-C` (or `SIGTERM`) shuts the whole stack down
cleanly — each process handles `SIGINT`/`SIGTERM` identically (set a stop flag,
short-circuit the drain, release SHM rings + close the OAK-D in an order the
firmware watchdog tolerates), and the launcher ignores further `Ctrl-C` while
tearing down so a second press can never abort cleanup or print a traceback.

### 3a. FC output — `--fc` (dblink UART)

The real FC output is the **`--fc PORT[:BAUD]`** flag: it spawns the consumer-only
`fc` process (after `slam`), which subscribes the VIO's `pose.odom`, converts each
pose to the FC's **NED** earth frame via the shared SSOT, and writes it to the serial
port as a **dblink `DB_CMD_VISION_POSE`** frame — the in-house FC protocol
(`../flight-controller`), **not** MAVLink. It is additive + **non-fatal** (a bad /
missing port logs + exits without taking the stack down) and independent of `--no-ui`.

```bash
# Pi flight + FC UART output. --direct makes pos_sigma_m usable (see below);
# --fc-rate clamps to [10,50] Hz; --fc-mount is the R_body_cam extrinsic.
./run.sh --no-ui --vl53l9cx --direct --fc /dev/ttyAMA0
./run.sh --no-ui --vl53l9cx --direct --fc /dev/ttyUSB0:921600 --fc-rate 50
```

**The full wire contract — dblink frame, the 38-byte payload (fields/units/ranges),
the `reset_counter` edges, the safety floors, and the `age` time model (and why the
FC's constant `C` must absorb the pipeline-latency floor) — lives in
[fc/README.md](../fc/README.md).** Two flight-critical points:

- **`pos_sigma_m` is `--direct`-only.** On the loose default path it is absent, so
  `fc` inflates the sigma to 100 m and the **FC ignores VIO position**. A usable FC
  *position* fix needs `--vl53l9cx --direct --fc ...` (attitude is sent regardless).
- **Loop closure / re-lock is a JUMP, not a measurement.** `fc` bumps the dblink
  `reset_counter` on a sensor-gap re-lock and an fc-local position jump so the FC ESKF
  **resets its origin** instead of fusing the discontinuity; a degraded / non-finite
  pose goes out advertised INVALID with the sigma inflated (never NaN on the wire).

A separate read-only pose *logger* (`_start_pose_logger` → `_on_pose` in
`launcher/main.py`) prints a **preview** of `pose.odom`, throttled to ~2 Hz — useful
on `--no-ui` to confirm the pose is flowing, but it is **not** the FC link (`--fc`
is):

```
pose: pos WORLD=(+0.005 -0.027 -0.003) m  quat wxyz=(+0.003 +0.017 -0.025 +0.999)  sig_pos=0.076m  n=14
```

| field | meaning |
|---|---|
| `pos WORLD` | position in the VIO world frame, metres (`wm.T_world_cam[:3,3]`) |
| `quat wxyz` | orientation quaternion — the FC derives heading from it |
| `sig_pos`   | **position noise σ (m)** for the FC ESKF: `R ≈ sig_pos²`. Only on `--direct` (`wm.info["pos_sigma_m"]`); σ ∝ Z/√N, clamped [0.05, 3.0] m. High at startup / few features / far scene; ~0.07 m when tracking well |
| `n`         | cumulative pose-message counter (the print is throttled, not every msg) |

> The logger prints the raw **WORLD** pose (gravity-aligned *optical*, +Y down). The
> `--fc` path does **not** ship those raw axes: the SSOT rotates the position **and**
> the attitude into the FC's NED / FRD frame before packing (see fc/README.md). The
> `sig_pos` column here is the same `pos_sigma_m` the `--fc` frame carries — the
> deliberately-simple first model (commit `7f9769a`, σ ∝ Z/√N); the principled
> upgrade is the inverse-Hessian marginal covariance with NEES calibration, left as
> future work since the FC floors the value.

**Still open (FC side):** the FC-side dblink vision *receiver* + EKF fusion does not
exist yet — that is separate work in `../flight-controller`. The `DB_CMD_VISION_POSE`
value (`0x0C`) is proposed; the FC header owns the final value. HIL on the Pi is
pending.

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

**Bandwidth mode — pose-only by default.** The bridge defaults to **pose-only**:
only the small pose/map/overlay POD + retained topics cross the WiFi, NOT the heavy
camera/depth/keyframe image topics. The launcher's `build_forward_args`
(`launcher/main.py`) appends `--pose-only` to `netbridge.forward` unless
`--bridge-frames` is set; `deploy/pi-ui.sh` passes `--pose-only` to
`netbridge.receive` to match. Both ends select the topic set via
`netbridge/topics_allowlist.py` `all_topics(role, include_images=False)`. The
trajectory + map UI works fully; only the opt-in camera Visualize windows have no
frames in this mode. Pass `--bridge-frames` (raw `run.sh`) / `--frames`
(`pi-run.sh` and `pi-ui.sh`) on **both** ends to include the camera frames — see
§3c.

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

### 3c. Lean-Pi run defaults (what `pi-run.sh` adds for you)

`deploy/pi-run.sh` is the host-driven wrapper around `run.sh --no-ui`; it injects
a few 4-core-Pi defaults so the operator doesn't have to. None change the math
(the byte-parity oracle stays `gap=0`); they only affect process scheduling and
which diagnostic captures run.

| `pi-run.sh` injects | default on the Pi | opt out | why |
|---|---|---|---|
| `--cap-numba-threads` | **on** (always passed) | — | per-process numba thread caps so capture+vio don't oversubscribe 4 cores |
| `--no-frontend-viz --no-ba-window` | on (with `--ui`) | `--viz` | the UI diagnostic captures run in the vio process and drag it below real-time |
| pose-only bridge | **on** (with `--ui`) | `--frames` | only the small pose/map/overlay topics cross the WiFi — the heavy uncompressed camera/depth/keyframe frames the main UI never displays would saturate the 2.4 GHz link |

- **The windowed BA and SLAM each run in their own process (always, in-process).**
  Every project is one process that runs its solve in-process — there is no internal
  worker child and **no `--worker` flag**. The bursty ~48 ms/keyframe BA solve used
  to share the vio/frontend core (a 1-in-5 ~80 ms stutter); it is now the standalone
  `ba` project, GIL-isolated from the frontend by construction. SLAM likewise runs
  its loop-closure + pose-graph solve on its own thread in the `slam` process; its
  IPC recv is a separate thread feeding a latest-only inbox, so a brief block during
  a heavy PGO just drops stale keyframes (the inbox is built to) and the live map
  stays current.

- **Lean flight config on the 4-core Pi.** HIL (2026-06-18): the FULL stack
  `capture + vio + ba + slam` at **640×400 `--tight`** saturates the Pi (load ~4.2,
  `ba` ~135 % CPU, live `pose.odom` throttled to ~5 Hz) — the split correctly keeps
  the slow BA off the frontend (vio runs free; `pose.refined` flows at ~1 Hz via the
  pass-through), but four heavy processes still oversubscribe four cores. For flight
  use **320×200** and drop what the mission does not need: `--no-ba` (no
  pose.refined / no bias feed-forward) and/or `--no-slam` (no loop closure) — both
  are launcher spawn gates, so the dropped process is simply never started. With
  `--ui`, the netbridge forward **skips the slam bridge** under `--no-slam` (it used
  to pass the real `oak.slam` endpoint anyway, block on the missing socket, and
  crash the whole bridge after the 30 s connect timeout — taking the remote UI down;
  fixed: `--no-slam` empties the forward's slam endpoint, like it does for vio).

- **`--cap-numba-threads` is always passed.** The flight stack runs capture / vio /
  slam as separate OS processes; with nothing set, **each** spins a numba pool of
  all cores, so an overlapping SGM (capture) + KLT (vio) burst puts ~2× the core
  count of runnable threads on 4 cores → oversubscription thrash. The flag pins
  `NUMBA_NUM_THREADS` per process, derived from `os.cpu_count()`
  (`_numba_thread_caps` in `launcher/main.py`): on the 4-core Pi5 **capture=2,
  vio=2, slam=1**. The launcher logs it on boot:
  ```
  launcher: capping numba threads per process {'imu_camera': 2, 'vio': 2, 'slam': 1} (ncores=4, --cap-numba-threads)
  ```
  A **user-set `NUMBA_NUM_THREADS` in the environment always wins** (the cap never
  overrides an explicit choice). The flag is a no-op / off by default on big dev
  hosts (full cores) — it is only injected by `pi-run.sh`.

- **Pose-only bridge defaults ON (with `--ui`).** The Mac↔Pi 2.4 GHz WiFi link
  (ch6, congested) delivers only **~1.6 Mbit/s** effective, but bridging the
  uncompressed 320×200 camera/depth/keyframe frames pushes **~51 Mbit/s** — frames
  the main 3D UI never shows (it draws only the trajectory + SLAM map). Forwarding
  them saturates the link, the pose stream backs up, and the UI lags. So
  `pi-run.sh --ui` defaults to **pose-only**: it drops the image topics
  (`frame.depth` / `cam.sync` / `imucam.sample` / `keyframe`) from the bridge and
  forwards only the small pose/map/overlay POD + retained topics. **Both ends must
  match** — run `pi-ui.sh` (default pose-only) against `pi-run.sh --ui` (default
  pose-only), or `pi-ui.sh --frames` against `pi-run.sh --ui --frames`.

  Measured on the real Pi 5 @320×200 (HIL A/B, single Mac consumer, `wlan0` TX
  deltas):

  | mode | WiFi TX | `pose.odom` | `slam.map` | UI |
  |---|---|---|---|---|
  | **pose-only** (default) | 0.38–0.87 Mbit/s (scene-dependent) | 19.9 Hz steady (real-time) | 4 Hz | trajectory + map fully work; 45–75% link headroom |
  | `--frames` | 13.9 Mbit/s burst + **4.1 MB TCP send-Q backlog** | 9.3 Hz (halved / backed up) | — | live camera image, but the weak link is saturated |

  To watch the live camera image remotely, run **both** `pi-run.sh --frames` and
  `pi-ui.sh --frames` — best on 5 GHz / a clear channel, where the ~51 Mbit/s fits.

### 3d. Profiling — where the time goes

Per-stage wall-clock was measured on the real Pi 5 with
`verification/stage_profile.py` (lean numpy+numba runtime, session
`sessions/gold/push_straight_fast_15s`); the full table is in
`verification/STAGE_PROFILE_RESULTS.md`. Headline findings:

- **The frontend (KLT track + RGB-D PnP) is the per-frame wall** — 58–62% of the
  loose serial budget at every resolution, and the single bottleneck of the vio
  process. At loose 320×200 it is ~29 ms (after the KLT pyramid-reuse change in
  `sky/front/klt.py`/`frontend.py`: one pyramid build/frame instead of four).
- **The live frame rate is the busiest PROCESS, not the serial sum.** Because `ba`
  and `slam` are their own processes (each runs its solve in-process), loose 320×200
  is set by the vio process (~33 ms/frame → ~30 fps); capture (SGM ~9 ms) and the
  `ba` solve run concurrently.
- **Tight `--tight` on the Pi (optimisation chain, 2026-06-16).** The tight
  `optimize_vio` solve now ships the four-link chain — **landmark Schur complement**
  (exact, ~1.4–1.8×), an **absolute-velocity gauge regulariser**, an always-on
  **divergence guard**, and the **njit IMU-Jacobian kernel** (default ON, coupled to
  the guard). All four are ON by default inside `--tight`. This makes `--tight`
  **bounded** (the guard caps a diverged keyframe instead of letting it run away) and
  faster than the pre-chain pure-Python solve. **It is still slower than loose** and
  does **not** reach 20 fps @ 320 — the keyframe solve was a design-estimate ~6–9 fps,
  and the real Pi `--tight` fps measurement is improved but load-noisy. **Loose is
  still the recommended Pi flight path; `--tight` is opt-in.**
  - **`divergence_guard` is an always-on flight invariant** (`WindowedVIOConfig`,
    default True). Do NOT disable it on a flight build: it is what keeps the published
    tight pose bounded on a bad/blurred frame, and the njit kernel is validated ONLY
    with it on.
  - **njit ↔ guard coupling + override.** The njit IMU-Jacobian kernel is default ON
    only when the guard is on; if the guard is off the kernel force-disables itself and
    logs why. `SKY_VIO_IMU_NJIT=0` always forces the pure-Python FD build (the explicit
    kill switch — use it for an A/B or a no-numba host); the env is otherwise unset on a
    normal run. A host without numba (`HAVE_NUMBA=False`) silently runs pure-Python.

> **Replay-vs-live caveat (read before quoting 320×200 numbers).** The launcher
> **replay** path forces capture to the session's *native* resolution
> (`imu_camera/main.py` overrides `--width`/`--height`), so you **cannot** get a
> deterministic 320×200 replay of a 640×400 gold session. The 320×200 figures above
> come from the `stage_profile.py` harness (which models capture-at-resolution by
> area-downsampling). The live 320×200 win from the per-process split (`ba`/`slam`
> off the vio core) + `--cap-numba-threads` must be confirmed with a **live camera
> run** at that resolution
> (`./deploy/pi-run.sh --width 320 --height 200`), not a replay.

### 3e. Reading `run.log` — heartbeat, freeze + overload detection

`run.log` (the headless stack's stdout; tail it with `./deploy/pi-run.sh --logs`)
carries a continuous pulse so neither a **freeze** (frames stop) nor an
**overload** (too slow to keep up) is ever silent on the console — both used to
look identical: the UI froze and the log said nothing.

- **capture heartbeat** — every 5 s: `capture: 20.0 fps (frame N)`. A frozen OAK
  does NOT kill the capture worker — it leaves it BLOCKED on its inbox (alive but
  starved), which used to spin silently (runs a bit, then freezes, console says
  nothing). Now the instant frames stop you get a LOUD, throttled warning:
  ```
  capture: STALLED -- no frame for 3.0s (last frame 432, read-thread alive=True). OAK crashed/hung? check `dmesg` + .cache/depthai/crashdumps
  ```
  and `capture: RECOVERED after Xs` when frames resume. So `grep STALLED run.log`
  pins exactly WHEN it froze + the cause to chase (the OAK firmware crash under
  streaming — see §1b; idle-stable but crashes under MIPI load).
- **pose stream** — the `--no-ui` FC-output logger prints `pose: pos WORLD=... n=N`
  at ~2 Hz; the `n=` counter is the live pose count, so it stalls when the pipeline
  does. (Its IPC client now waits up to 30 s for VIO to come up, so `--no-ui` always
  gets the pose log — it used to give up at 5 s, before VIO's server existed.)
- **overload watchdog** — a freeze is frames STOPPING; an overload is frames still
  flowing but **too slow**. At too high a resolution for the box (e.g. 640×400 on
  the 4-core Pi) nothing errors — the pipeline just can't keep real-time and the
  (remote) UI looks frozen / "laggy". The launcher compares the live `pose.odom`
  rate to `--fps`; when it stays below ~60 % of target for a few seconds (live
  only — replay is paced, not a "can't keep up" signal) it logs:
  ```
  pipeline OVERLOADED: pose.odom only ~6.5 Hz vs 20 target at 640x400 -- the box can't keep up at this resolution; lower it (e.g. --width 320 --height 200).
  ```
  So `grep OVERLOADED run.log` says the box can't sustain the chosen resolution —
  drop to **320×200** and/or shed `--no-ba` / `--no-slam` (§3c). pose.odom is the
  LAST pipeline stage, so this one signal catches a bottleneck in EITHER capture or
  vio.

Keep a freeze/overload log for later by copying `run.log` before the next run (it is
truncated each start). Post-mortem:
`grep -E "STALLED|OVERLOADED|crashed|X_LINK" run.log`, then for a freeze
`dmesg | grep -iE "usb|movidius|luxonis"` + the newest `.cache/depthai/crashdumps/*.tar.gz`.

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

**"no OAK device found" on the Pi even though the camera is plugged in.**
On Linux, depthai needs a **udev rule** for the login user to access the OAK over
USB — without it the device is visible only to `root` (`sudo python -c "import
depthai; print(depthai.Device.getAllAvailableDevices())"` sees it, the normal user
sees `[]`). `pi-deploy.sh` installs the rule automatically; to do it by hand:
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger   # (or just re-plug the OAK)
```
Verify the USB enumeration first with `lsusb | grep 03e7` (`03e7` = the Movidius
vendor). If `lsusb` shows nothing, it is a cable/power issue, not udev (the OAK-D
Lite especially needs a USB3 data cable + enough power — a Y-cable on a weak port).

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
