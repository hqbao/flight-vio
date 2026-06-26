# VIO latency budget (camera → FC), measured

End-to-end latency of the VIO pose as the **flight controller** consumes it. This
is the dominant cause of the FC's mode-0 position-hold wobble (the FC steers on a
pose that is ~one tenth of a second old). Numbers measured on real hardware, not
estimated.

- **Rig:** RP5 (companion) + OAK-D W, dblink `DB_CMD_VIO_POSE` over `/dev/ttyAMA0`
  (115200 8N1) → STM32H7 FC (h7v1).
- **Config:** `--width 320 --height 200 --no-ba --no-slam` (the lightest live path).
- **Date:** 2026-06-26.

## Budget @ 20 fps (the stable operating point)

| Stage | Latency | Notes |
|---|---|---|
| Camera → host read | **47 ms** | OAK exposure + on-Myriad stereo + USB. ≈ one 20 fps frame. Rock-constant (p50=p95=47). |
| Host VIO solve (frame → pose ready) | **~40 ms** | RP5 odometry, `--no-slam --no-ba`. |
| UART transport + FC decode/fuse | ~5 ms | 46 B @ 115200 ≈ 4 ms. |
| **Total camera → FC fuses pose** | **~92 ms** | Steady-state jitter only ±3 ms. |

So the FC always acts on a position that reflects the world **~92 ms ago**. For
reference, DJI-class flow+IMU is ~5–10 ms — this rig is ~10× slower, which is why
it cannot hold position as tightly.

## fps does NOT help — the RP5 is throughput-bound (~28 pose/s)

Raising the camera fps lowers the **camera** stage but the **host solve** cannot
keep up, so the pose queue backs up unboundedly:

| @ 60 fps | Result |
|---|---|
| Camera → host | **20 ms** (down from 47 — higher fps *does* cut camera latency) |
| Camera → pose | **1.4 s → 3.6 s and climbing** (frames queue faster than solved) |
| Pipeline | `OVERLOADED: pose.odom only ~28 Hz vs 60 target` |
| FC | **nav_src = HOLD, 0 VIO frames** — poses are seconds stale, the FC age-gate rejects them all |

**The RP5 solves ~28 pose/s max.** Feeding 60 fps overloads it and breaks VIO
entirely (worse than 20 fps). Practical max sustainable ≈ 25 fps; the latency win
over 20 fps is small and risks the overload edge. **Do not raise fps as a wobble
fix.**

## Accuracy vs fps (it is fine — this is STEREO)

Depth/scale come from the **fixed stereo baseline between the two cameras**, per
frame — NOT from temporal parallax between consecutive frames. So frame spacing
(fps) does **not** affect depth accuracy or cause faster drift. (That worry is a
*monocular* VIO concern, where depth needs inter-frame motion.) Higher fps would,
if the box could keep up, *improve* odometry (shorter exposure → less motion
blur, smaller inter-frame displacement → easier tracking). The binding constraint
is RP5 **compute**, not accuracy.

## What actually reduces the latency

1. **Cut the host solve cost** (the ~40 ms / ~28 Hz ceiling) so a higher,
   lower-latency fps becomes sustainable — offload more to the Myriad VPU
   (see `RP5_ACCELERATION_AUDIT.md` in the flight-controller side), trim the
   odometry front-end.
2. **VIO measurement-delay compensation on the FC (SE1):** timestamp each pose
   and fuse it at its true past instant (now − ~92 ms) instead of "now". Then
   trusting VIO no longer injects the delay into the control loop → the FC can
   trust VIO (no drift) *and* not wobble. This is the DJI/PX4/VINS approach and
   the real fix; it makes the 92 ms not matter to the loop.

The FC-side trade-off this latency creates (trust-accel ⇒ wobble↓ but drift↑;
trust-VIO ⇒ drift↓ but wobble↑) is documented on the flight-controller side
(SE1 `sigma_accel` / `R_pos` tuning).

## How it was measured (reproduce)

Both probes use DepthAI's **host-synced** frame timestamp (`getTimestamp()`),
which removes the device↔host clock-offset problem (the on-wire `age_us` is
floor-subtracted and under-reports, so it can't be used for the absolute number):

- **Camera → host:** in `imu_camera/modules/read_cam.py` `LiveCamSource.read()`,
  `dai.Clock.now() - ld.getTimestamp()` per frame.
- **Camera → pose:** carry the device→host offset
  `O = getTimestamp() - getTimestampDevice()` (written once by read_cam) and in
  `launcher/main.py` `_on_pose`, `now - (wm.ts_ns*1e-9 + O)`.

Probes are temporary (reverted after measuring). Watch `~/flight-vio/run.log`;
the `pipeline OVERLOADED` warning already flags the throughput ceiling without
any probe.
