"""Live RGB-D VIO from the OAK-D using *our* from-scratch odometry.

Unlike :mod:`oakd.sources.depthai_vio` (which reads poses out of DepthAI's
built-in ``BasaltVIO`` node), this source runs **our own** frame-to-frame
RGB-D PnP odometry (:class:`oakd.vio.RGBDVisualOdometry`) on the live
rectified-left + depth stream. It exists so we can watch our VIO drive the 3D
viewer in real time and eyeball its quality against Basalt *before* we add
sliding-window bundle adjustment.

Pipeline (same front-end as the recorder):
    Camera CAM_B/CAM_C -> StereoDepth (depthAlign=left)
        -> rectifiedLeft (uint8 gray) + depth (uint16 mm)

Our odometry produces poses in the standard OpenCV **camera optical** frame
(x right, y down, z forward), world = first frame (assumed level at start, since
vision-only VO has no gravity reference). For the NED 3D viewer we remap optical
-> NED with the textbook mapping::

    North = +z_opt (forward),  East = +x_opt (right),  Down = +y_opt (down)

i.e. ``M_opt->ned = [[0,0,1],[1,0,0],[0,1,0]]``. Combined with the attitude
column reorder ``P`` below this gives an *identity* startup attitude
(roll=pitch=yaw=0) and a physically self-consistent display: moving the camera
up moves the marker up, moving it forward moves it North, moving it right moves
it East. (An earlier flipped mapping ``[[0,0,-1],[1,0,0],[0,-1,0]]`` baked in a
spurious 180 roll -- the symptom was the green 'right' arrow pointing left and
upward camera motion showing as downward marker motion.)

Note: absolute North/Down here are *not* gravity-aligned (no IMU leveling); the
frame is anchored to the first camera pose. Trajectory accuracy (ATE) is
Umeyama-aligned so it is unaffected by this convention choice.

Startup attitude: at launch we average the accelerometer over a short static
window and gravity-level the initial pose (``RGBDVisualOdometry.align_to_gravity``)
so the world "down" is real gravity and the reported roll/pitch reflect the
camera's actual tilt -- not an assumed-level identity start. Yaw stays at the
starting heading (no magnetometer). If the device reports no IMU, we fall back to
an identity start.

Note: the gyro rotation prior is a measured no-op on well-synced data (see
``oakd/vio/imu.py``), so this live source runs pure vision for simplicity.
"""
from __future__ import annotations

import threading
import time

import numpy as np

from ..pose import Pose
from ..frames import quat_to_rpy
from ..vio import (
    FrontendConfig, KLTFrontend, OdometryConfig, RGBDVisualOdometry,
    gravity_aligned_R0, level_attitude,
)
from .base import PoseSource


# Standard OpenCV optical (x right, y down, z forward) -> world NED.
# North = +z (forward), East = +x (right), Down = +y (down). With the column
# reorder P below this yields an identity startup attitude (roll=pitch=yaw=0)
# and a self-consistent display (up->up, forward->N, right->E).
_M_OPT_TO_NED = np.array(
    [[0.0, 0.0, 1.0],
     [1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0]]
)

# Measured mounted-pose attitude (camera->world), used only as a FALLBACK when
# the device reports no accelerometer. Captured 2026-06-02 with the camera in
# its drone-mount pose via ``tools/measure_mount_attitude.py`` (avg of 760 raw
# accel samples, per-axis std ~0.02 m/s^2 -> tilt good to <0.05 deg): the mount
# is essentially level (roll +0.1, pitch -0.5, lens looking horizontally
# forward). The live path still prefers a fresh accel measurement at startup, so
# this only matters on IMU-less devices. Re-run the tool and update this if the
# physical mount changes.
_MOUNT_R0 = np.array(
    [[+0.999998, -0.002003, +0.000000],
     [+0.002003, +0.999956, +0.009146],
     [-0.000018, -0.009146, +0.999958]]
)

# Accel leveling only runs when the camera is at rest, detected from the residual
# of the raw accelerometer against its EMA (recent motion energy, m/s^2). Below
# this threshold the camera is still and the accel reads pure gravity; above it
# there is translation/rotation whose lateral linear acceleration would bias the
# gravity DIRECTION (a magnitude gate cannot catch that), so leveling is skipped
# and vision holds the attitude. ~0.35 m/s^2 sits above the sensor noise floor
# (~0.02-0.15) and below the accel of deliberate handheld motion.
_REST_MOTION_THRESH = 0.35

# Column reorder optical (right, down, fwd) -> body FRD (fwd, right, down).
# The viewer triad expects the attitude columns to be [forward, right, down],
# but our VO's rotation columns are the optical axes [right, down, fwd]. The
# body attitude in NED is therefore the camera axes mapped to NED with the
# columns picked as [optical_z, optical_x, optical_y] -> M @ R_opt @ P. Using
# the naive conjugation M @ R_opt @ M.T leaves the forward+down arrows 180
# off (only the right axis happens to line up). Verified vs Basalt (all body
# axes +0.97..+1.0 cos).
_P_OPT_TO_FRD = np.array(
    [[0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0],
     [1.0, 0.0, 0.0]]
)


def _ease_se3(C_cur: np.ndarray, C_tgt: np.ndarray, alpha: float) -> np.ndarray:
    """Move ``C_cur`` a fraction ``alpha`` toward ``C_tgt`` (smooth correction).

    Rotation eases along the geodesic (scaled axis-angle); translation eases
    linearly. Keeps the applied correction continuous so BA updates never snap
    the displayed trajectory.
    """
    R_cur, R_tgt = C_cur[:3, :3], C_tgt[:3, :3]
    dR = R_tgt @ R_cur.T
    ang = np.arccos(np.clip((np.trace(dR) - 1.0) * 0.5, -1.0, 1.0))
    out = np.eye(4)
    if ang < 1e-8:
        out[:3, :3] = R_tgt
    else:
        axis = np.array([dR[2, 1] - dR[1, 2],
                         dR[0, 2] - dR[2, 0],
                         dR[1, 0] - dR[0, 1]]) / (2.0 * np.sin(ang))
        a = alpha * ang
        K_ = np.array([[0, -axis[2], axis[1]],
                       [axis[2], 0, -axis[0]],
                       [-axis[1], axis[0], 0]])
        R_step = np.eye(3) + np.sin(a) * K_ + (1.0 - np.cos(a)) * (K_ @ K_)
        out[:3, :3] = R_step @ R_cur
    out[:3, 3] = (1.0 - alpha) * C_cur[:3, 3] + alpha * C_tgt[:3, 3]
    return out


def _rot_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a (w, x, y, z) unit quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


class OakOursVioSource(PoseSource):
    """OAK-D + *our* RGB-D odometry -> NED pose stream.

    ``backend='f2f'`` runs the plain frame-to-frame PnP VO; ``backend='ba'``
    runs the sliding-window bundle-adjustment VO. Both share the same KLT
    frontend and depth, so switching backends isolates exactly what BA adds.

    ``backend='slam'`` runs the f2f VO for display and a **background loop-closure
    SLAM** thread (:class:`oakd.vio.SlamMap`): every few frames a keyframe (the
    raw f2f pose + its image + depth) is handed to the SLAM map, which recognises
    revisited places (ORB + fundamental-matrix + PnP geometric verification) and
    runs SE(3) pose-graph optimisation. The resulting world-frame correction is
    eased onto the displayed trajectory exactly like the BA correction, so loop
    closures snap out the accumulated drift smoothly. The SLAM map is fed the
    *raw* f2f poses (a fixed world frame), so its odometry edges stay
    self-consistent over the whole trajectory; gravity leveling is still applied
    as the final display step (the ordering rule: loop closure owns position+yaw,
    accel re-levels tilt last).

    **Gyro complementary fusion:** the gyroscope is integrated each frame into an
    inter-frame rotation prior that is handed to the odometry (``gyro_fuse``).
    Vision (PnP) corrects this rotation weighted by its inlier confidence, so a
    fast yaw turn that makes the KLT tracker lose features no longer under-rotates
    the pose. On a healthy frame the fusion collapses to pure vision (no accuracy
    cost on good data). Translation stays vision-only.
    """

    def __init__(self, width: int = 640, height: int = 400, fps: int = 20,
                 backend: str = "f2f", slam_kf_every: int = 5,
                 slam_radius_m: float = 0.0, ba_window: int = 6,
                 ba_kf_every: int = 5, ba_iters: int = 5,
                 use_own_klt: bool = False, slam_kf_min_trans: float = 0.0,
                 slam_kf_min_rot: float = 0.0, slam_max_kf: int = 0) -> None:
        super().__init__()
        self.width = int(width)
        self.height = int(height)
        self.cam_fps = int(fps)
        self.backend = backend
        # SLAM update cadence: insert a keyframe (and run loop detection) every
        # ``slam_kf_every`` frames. This is the main lever for the SLAM update
        # rate -- fewer keyframes = more responsive loop closure AND a smaller
        # pose graph (cheaper PGO). ``slam_radius_m`` optionally spatially gates
        # loop candidates (0 = check all, the default): measured to help little
        # at ~200 keyframes because the ORB appearance gate already rejects
        # distant keyframes cheaply, but it bounds cost on very long runs.
        self.slam_kf_every = int(slam_kf_every)
        self.slam_radius_m = float(slam_radius_m)
        # Keyframe budget for long runs. The motion gate (min translation /
        # rotation since the last keyframe) makes the map grow with TRAJECTORY
        # length instead of run TIME -- a hovering/stationary drone stops piling
        # up redundant keyframes, the main cause of unbounded memory + PGO cost.
        # ``slam_max_kf`` is an absolute safety cap (drops the oldest keyframe
        # when exceeded; 0 = unlimited). Both default to off so behaviour is
        # unchanged unless requested.
        self.slam_kf_min_trans = float(slam_kf_min_trans)
        self.slam_kf_min_rot = float(slam_kf_min_rot)
        self.slam_max_kf = int(slam_max_kf)

        # Sliding-window BA tuning (backend='ba'): window size, keyframe cadence,
        # and BA iterations per solve. Smaller = cheaper/faster, larger = more
        # accurate but heavier on the background thread.
        self.ba_window = int(ba_window)
        self.ba_kf_every = int(ba_kf_every)
        self.ba_iters = int(ba_iters)
        # Optical-flow + corner backend for the display front-end. Our own
        # pure-NumPy pyramidal LK + Shi-Tomasi (library-free) tracks the same
        # corners to sub-pixel agreement with cv2, BUT costs ~104 ms/frame --
        # ~2x over the 50 ms budget at 20 fps. Running it in this SYNCHRONOUS
        # read loop makes the viewer fall behind: the drain-to-latest logic then
        # skips frames, the inter-frame motion jumps, KLT's forward-backward
        # check fails and tracking is lost. So LIVE defaults to cv2 (3 ms/frame,
        # smooth real time); pass use_own_klt=True (viewer --own-klt) only to
        # watch the library-free path live and accept the lag. Offline scoring
        # (tools/vio_run.py) always uses our own -- there time does not matter
        # and ATE is at parity (lab_loop f2f 1.18 vs 1.27%).
        self.use_own_klt = bool(use_own_klt)

        # --- live SLAM overlay (read by the 3D viewer) ----------------------
        # Thread-safe snapshot of the SLAM map for the UI: keyframe dots, the
        # matched (revisited) keyframes, and the loop-closure links. All in NED
        # so the viewer only has to apply its NED->ENU display transform. These
        # are REAL SlamMap outputs (corrected keyframe poses + confirmed loop
        # events), not a parallel/derived pipeline. Empty for non-SLAM backends.
        self._slam_lock = threading.Lock()
        self._slam_kf_ned = np.zeros((0, 3), dtype=np.float32)
        self._slam_match_ned = np.zeros((0, 3), dtype=np.float32)
        self._slam_loop_ned: list[np.ndarray] = []
        # Flash counter: bumped each time a NEW loop closes, so the viewer can
        # detect a fresh teleport and play a short fade-out (instead of drawing
        # the whole accumulated loop history, which turns into a magenta mess).
        self._slam_flash_id = 0

        # Set by the UI "clear keyframes" button; the read loop picks it up,
        # wipes the SLAM map + the loop-closure correction + the overlay, so a
        # test run can be restarted without relaunching the pipeline.
        self._slam_reset = threading.Event()

        # Gyro zero-rate bias (rad/s, IMU frame), measured over the static
        # startup window by ``_collect_startup_accel``. None until startup runs
        # (or if the device has no IMU), in which case gyro integration uses a
        # zero bias.
        self._gyro_bias: np.ndarray | None = None

    def slam_overlay_snapshot(
        self,
    ) -> tuple[np.ndarray, np.ndarray, list[np.ndarray], int]:
        """Latest SLAM overlay for the viewer (all positions in NED).

        Returns ``(kf_ned, match_ned, loop_segs, flash_id)`` where ``kf_ned`` is
        every keyframe position (Nx3), ``match_ned`` the keyframes revisited by
        the MOST RECENT loop closure (Mx3), ``loop_segs`` its ``[cur, old]``
        teleport segments, and ``flash_id`` a counter the viewer watches to know
        a new loop just closed (so it can flash then fade the link). The match /
        loop fields hold only the latest closure, not the full history.
        """
        with self._slam_lock:
            return (self._slam_kf_ned.copy(),
                    self._slam_match_ned.copy(),
                    [s.copy() for s in self._slam_loop_ned],
                    self._slam_flash_id)

    def clear_slam_map(self) -> None:
        """Forget every SLAM keyframe (UI "clear keyframes" button).

        Signals the read loop, which wipes the map worker-side and resets the
        loop-closure correction + overlay. Safe to call from the UI thread; a
        no-op for non-SLAM backends. Does NOT touch the displayed trajectory or
        the f2f/gyro odometry — only the SLAM keyframe map and its corrections.
        """
        self._slam_reset.set()

    def _run(self) -> None:
        import cv2
        import depthai as dai  # lazy: --source fake works without depthai/device

        left_socket = dai.CameraBoardSocket.CAM_B
        right_socket = dai.CameraBoardSocket.CAM_C

        with dai.Pipeline() as p:
            left = p.create(dai.node.Camera).build(left_socket, sensorFps=self.cam_fps)
            right = p.create(dai.node.Camera).build(right_socket, sensorFps=self.cam_fps)
            stereo = p.create(dai.node.StereoDepth)
            imu = p.create(dai.node.IMU)

            stereo.setExtendedDisparity(False)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(False)
            stereo.setRectifyEdgeFillColor(0)
            stereo.enableDistortionCorrection(True)
            stereo.initialConfig.setLeftRightCheckThreshold(10)
            stereo.setDepthAlign(left_socket)

            # Accelerometer (gravity leveling) + gyroscope (the inter-frame
            # rotation prior for the complementary fusion). The gyro is what
            # keeps yaw correct through fast turns where vision under-rotates;
            # accel cannot recover yaw. 200 Hz gyro >> the ~20 fps frame rate so
            # each frame integrates ~10 samples.
            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,
                                 dai.IMUSensor.GYROSCOPE_RAW], 200)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)

            left.requestOutput((self.width, self.height)).link(stereo.left)
            right.requestOutput((self.width, self.height)).link(stereo.right)

            # Non-blocking queues so a slow consumer never stalls the device
            # (a stalled XLink read is what triggers X_LINK_ERROR). We keep a
            # small buffer and always consume the *latest* frame below.
            q_left = stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=False)
            q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
            q_imu = imu.out.createOutputQueue(maxSize=50, blocking=False)

            p.start()

            # Pull rectified-left intrinsics for the metric back-projection.
            ch = p.getDefaultDevice().readCalibration()
            K = np.array(
                ch.getCameraIntrinsics(left_socket, self.width, self.height),
                dtype=np.float64,
            )
            # IMU->left-camera rotation (for bringing accel into the optical
            # frame). depthai returns the extrinsic with translation in cm; we
            # only need the 3x3 rotation here.
            try:
                R_imu_cam = np.array(
                    ch.getImuToCameraExtrinsics(left_socket), dtype=np.float64
                )[:3, :3]
            except Exception:
                R_imu_cam = np.eye(3)

            # The displayed pose is ALWAYS produced by the fast frame-to-frame
            # VO, so the read loop never blocks on BA and the UI stays smooth.
            # When the user opts into our own library-free frontend live
            # (use_own_klt), pick the config by whether Numba is available:
            #   * with Numba, the JIT core tracks the FULL-quality config in
            #     ~15 ms/frame (well under the 50 ms budget at 20 fps), so use it.
            #   * without Numba the pure-NumPy path costs ~140 ms/frame, so fall
            #     back to the lighter ``live_own`` preset (~38-58 ms) to stay
            #     roughly real time.
            # cv2 (use_own_klt False) stays the default at ~3 ms/frame.
            if self.use_own_klt:
                from ..vio.klt_numba import HAVE_NUMBA
                fe_cfg = (FrontendConfig(use_own_klt=True) if HAVE_NUMBA
                          else FrontendConfig.live_own())
            else:
                fe_cfg = FrontendConfig(use_own_klt=False)
            vo = RGBDVisualOdometry(
                K, OdometryConfig(gyro_fuse=True), frontend=KLTFrontend(fe_cfg))

            # Gravity-level the initial attitude: average the accelerometer over
            # a short static startup window, rotate it into the camera optical
            # frame, and seed the VO world frame so its "down" is real gravity.
            # If the device has no accelerometer, fall back to the measured
            # mounted pose ``_MOUNT_R0`` so we still start from a known attitude.
            accel_cam = self._collect_startup_accel(q_imu, R_imu_cam)
            if accel_cam is not None:
                vo.align_to_gravity(accel_cam)
                # Sanity-log how the live measurement compares to the recorded
                # mount baseline. The per-frame ``correct_tilt`` below keeps the
                # attitude pinned to gravity at any orientation, so a startup
                # offset (or even an upside-down hold) self-corrects within a few
                # frames -- no need to gate on it.
                dR = vo.pose[:3, :3] @ _MOUNT_R0.T
                ang = np.degrees(np.arccos(
                    np.clip((np.trace(dR) - 1.0) * 0.5, -1.0, 1.0)))
                print(f"[ours-vio] gravity-leveled startup; "
                      f"{ang:.1f} deg from recorded mount baseline")
            else:
                vo.pose = np.eye(4)
                vo.pose[:3, :3] = _MOUNT_R0
                print("[ours-vio] no accelerometer; using recorded mount R0")

            # In BA mode, a background *process* refines a sliding window of
            # keyframes and publishes a world-frame correction ``C``. We ease
            # the applied correction toward the latest ``C`` so updates never
            # snap the trajectory, and apply it as P_disp = C @ P_f2f. The BA
            # map is fed the (already gravity-aligned) f2f poses, so its
            # correction is frame-consistent without extra bookkeeping.
            ba_state = None
            C_applied = np.eye(4)
            C_target = np.eye(4)
            if self.backend == "ba":
                ba_state = self._start_ba_worker(K)

            # In SLAM mode, a background thread keeps a persistent keyframe map,
            # recognises revisited places and runs pose-graph optimisation; it
            # publishes a world-frame loop-closure correction that we ease onto
            # the displayed f2f trajectory the same way as the BA correction.
            slam_state = None
            if self.backend == "slam":
                slam_state = self._start_slam_worker(K)



            t0 = time.monotonic()
            prev_pos_ned = np.zeros(3)
            prev_t: float | None = None
            frames = 0
            kf_count = 0
            last_fps_t = t0
            accel_n = 0
            accel_used = 0
            last_tilt_log = t0
            accel_ema: np.ndarray | None = None
            grav_corr = np.eye(3)   # stateful display-frame gravity correction

            # Gyro complementary-fusion state. ``gyro_bias`` is the mean gyro over
            # the static startup window (rad/s, IMU frame); each frame we
            # integrate the gyro samples drained from the IMU queue into an
            # inter-frame rotation and hand it to ``vo.process`` as the rotation
            # prior. ``so3_exp`` is the same exponential map the offline
            # GyroPreintegrator uses, so live and offline share one convention.
            from ..vio.imu import so3_exp
            gyro_bias = (self._gyro_bias if self._gyro_bias is not None
                         else np.zeros(3))
            gyro_last_ts: float | None = None

            while not self._stop.is_set() and p.isRunning():
                # Drain each queue to its most recent frame; drop the backlog so
                # that if anything briefly stalls we skip stale frames instead of
                # falling progressively further behind (stays real time).
                ld = q_left.tryGet()
                while True:
                    nxt = q_left.tryGet()
                    if nxt is None:
                        break
                    ld = nxt
                dd = q_depth.tryGet()
                while True:
                    nxt = q_depth.tryGet()
                    if nxt is None:
                        break
                    dd = nxt
                if ld is None or dd is None:
                    time.sleep(0.002)
                    continue

                gray = ld.getCvFrame()
                if gray.ndim == 3:
                    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
                depth_mm = dd.getCvFrame()
                depth_m = depth_mm.astype(np.float32) / 1000.0

                # Drain the IMU queue ONCE per frame, BEFORE odometry, so we have
                # this frame's gyro rotation prior ready for PnP. We both (a)
                # average the accelerometer (gravity leveling, below) and (b)
                # integrate the gyro into an inter-frame rotation. Averaging the
                # ~10 accel samples/frame rejects per-sample noise; integrating
                # every gyro sample with its own dt preserves fast rotation.
                acc_sum = np.zeros(3)
                acc_cnt = 0
                R_imu_accum = np.eye(3)
                gyro_cnt = 0
                imsg = q_imu.tryGet()
                while imsg is not None:
                    for pkt in imsg.packets:
                        a = pkt.acceleroMeter
                        v = (a.x, a.y, a.z)
                        # Reject NaN/inf sentinel packets: a single bad sample
                        # would poison the EMA permanently (it never recovers from
                        # NaN), which then corrupts the pose into NaN forever.
                        if np.all(np.isfinite(v)):
                            acc_sum += v
                            acc_cnt += 1
                        g = pkt.gyroscope
                        w = np.array([g.x, g.y, g.z], dtype=np.float64)
                        if np.all(np.isfinite(w)):
                            try:
                                ts = g.getTimestampDevice().total_seconds()
                            except Exception:
                                ts = None
                            if ts is not None:
                                if gyro_last_ts is not None:
                                    dt = ts - gyro_last_ts
                                    if 0.0 < dt < 0.1:   # skip gaps/duplicates
                                        R_imu_accum = R_imu_accum @ so3_exp(
                                            (w - gyro_bias) * dt)
                                        gyro_cnt += 1
                                gyro_last_ts = ts
                    imsg = q_imu.tryGet()
                accel_raw = None if acc_cnt == 0 else R_imu_cam @ (acc_sum / acc_cnt)

                # Inter-frame gyro rotation in the camera frame (prev<-cur
                # convention, matching GyroPreintegrator). None until we have a
                # spanned interval, so the very first frame stays pure vision.
                R_prior = (R_imu_cam @ R_imu_accum @ R_imu_cam.T
                           if gyro_cnt > 0 else None)

                vo.process(gray, depth_m, R_prior=R_prior)  # camera-optical world


                # --- accelerometer leveling, gated on the camera being at rest --
                # We only trust accel for leveling when the camera is actually at
                # rest, because a magnitude gate cannot reject lateral linear
                # acceleration (a sideways push barely changes |accel| yet tilts
                # the gravity direction). The motion signal is the residual of the
                # batch-averaged sample against its EMA: tiny at rest, large during
                # any translation/rotation. When moving we skip leveling and let
                # vision hold the attitude; when still, accel pulls roll/pitch back
                # to true gravity.
                accel_cam = None
                at_rest = False
                motion = 0.0
                if accel_raw is not None:
                    if accel_ema is None:
                        accel_ema = accel_raw.copy()
                    else:
                        accel_ema += 0.2 * (accel_raw - accel_ema)
                    accel_cam = accel_ema
                    motion = float(np.linalg.norm(accel_raw - accel_ema))
                    at_rest = motion < _REST_MOTION_THRESH
                    # Track the true gravity magnitude from the at-rest samples.
                    # The startup g_ref can be captured during motion (it read
                    # 10.15 vs a real ~8.9 here), which would skew the inner
                    # magnitude gate; refresh it whenever we are actually still.
                    if at_rest:
                        na = float(np.linalg.norm(accel_cam))
                        if vo._g_ref is None:
                            vo._g_ref = na
                        else:
                            vo._g_ref += 0.05 * (na - vo._g_ref)

                # Level the f2f world frame too (only at rest), so the BA map is
                # fed gravity-consistent poses over the long term.
                if accel_cam is not None and at_rest:
                    vo.correct_tilt(accel_cam)

                pose = vo.pose.copy()  # camera-optical world

                # Set True when a loop-closure correction slews the pose this
                # frame (a teleport while the camera is ~still). Coloured
                # distinctly in the path so the jump reads as a map correction.
                teleport = False

                if ba_state is not None:
                    # Submit a keyframe snapshot every kf_every frames. Drop if
                    # the worker is busy (non-blocking) — never stall the loop.
                    kf_count += 1
                    if kf_count >= ba_state["kf_every"]:
                        kf_count = 0
                        st = vo.frontend.tracks
                        ba_state["submit"](
                            np.linalg.inv(pose),          # T_cw (world->cam)
                            st.ids.copy(), st.points.copy(),
                            depth_m.copy(),
                            # Only hand BA a gravity measurement when the camera
                            # is at rest; during motion lateral acceleration would
                            # bias the gravity direction, so the keyframe carries
                            # no gravity prior (None) and BA levels it from its
                            # at-rest neighbours in the window.
                            accel_cam.copy() if (accel_cam is not None
                                                 and at_rest) else None,
                        )
                    # Pull the latest correction from the worker (drain to last).
                    newC = ba_state["poll"]()
                    if newC is not None:
                        C_target = newC
                    # Ease toward the target so the correction never snaps.
                    C_applied = _ease_se3(C_applied, C_target, 0.15)
                    pose = C_applied @ pose

                if slam_state is not None:
                    # UI "clear keyframes": wipe the map worker-side and snap our
                    # local loop-closure correction + overlay buffers back to
                    # empty so the displayed path detaches from the old map at
                    # once (no eased slew back through a stale correction).
                    if self._slam_reset.is_set():
                        self._slam_reset.clear()
                        slam_state["reset_map"]()
                        C_target = np.eye(4)
                        C_applied = np.eye(4)
                        kf_count = 0
                        with self._slam_lock:
                            self._slam_kf_ned = np.zeros((0, 3), np.float32)
                            self._slam_match_ned = np.zeros((0, 3), np.float32)
                            self._slam_loop_ned = []
                            self._slam_flash_id += 1
                    # Hand a keyframe (raw f2f pose + image + depth) to the SLAM
                    # map every kf_every frames; drop if the worker is still busy
                    # on the previous one (non-blocking) — never stall the loop.
                    kf_count += 1
                    if kf_count >= slam_state["kf_every"]:
                        kf_count = 0
                        slam_state["submit"](pose.copy(), gray.copy(),
                                             depth_m.copy())
                    # Pull the latest loop-closure correction (drain to last) and
                    # ease it on, exactly like the BA correction.
                    newC = slam_state["poll"]()
                    if newC is not None:
                        C_target = newC
                    C_prev = C_applied
                    C_applied = _ease_se3(C_applied, C_target, 0.15)
                    # Teleport displacement = how far THIS pose moves purely from
                    # the correction slewing (same vo pose, only the correction
                    # changed). Real camera motion never enters this delta, so a
                    # non-trivial value means the loop-closure correction is
                    # dragging the displayed point back onto the remembered place.
                    corr_step = float(np.linalg.norm(
                        (C_applied @ pose)[:3, 3] - (C_prev @ pose)[:3, 3]))
                    teleport = corr_step > 0.01   # 1 cm/frame from correction
                    pose = C_applied @ pose

                # FINAL display leveling -- accel is the "trum cuoi" (last word)
                # on the shown roll/pitch. The Phase-4 in-BA gravity prior keeps
                # the keyframe MAP from tilt-drifting, but the BA correction
                # ``C_applied`` can still leave a residual tilt on the displayed
                # ``C_applied @ vo.pose`` (reprojection in a low-parallax view is
                # blind to absolute tilt, so the latest keyframe attitude need not
                # be perfectly level). So we keep a STATEFUL world-frame correction
                # ``grav_corr`` and apply ``pose = grav_corr @ pose`` as the last
                # step. At rest we nudge ``grav_corr`` by a small adaptive gain
                # toward cancelling the residual tilt (stateful => a partial gain
                # converges, unlike a partial gain on the freshly-rebuilt pose);
                # when moving we freeze it so nothing jumps at the rest-gate
                # boundary. Because BA now also levels the map, grav_corr stays
                # small. Re-orthonormalise (SVD project onto SO(3)) each update so
                # the repeated matrix products never drift out of SO(3) into NaN.
                # yaw is untouched (level_attitude only rotates about horizontal).
                R_pre = pose[:3, :3]
                R_disp = grav_corr @ R_pre
                tilt_deg = 0.0
                if accel_cam is not None and at_rest:
                    R_lvl, used, tilt_deg = level_attitude(
                        R_disp, accel_cam, g_ref=vo._g_ref,
                        alpha=0.05, alpha_max=0.25)
                    if used:
                        grav_corr = (R_lvl @ R_disp.T) @ grav_corr
                        U, _, Vt = np.linalg.svd(grav_corr)
                        grav_corr = U @ Vt
                        if np.linalg.det(grav_corr) < 0:
                            U[:, -1] *= -1.0
                            grav_corr = U @ Vt
                        R_disp = grav_corr @ R_pre
                # ``grav_corr`` is a world-frame rotation, so it MUST be applied
                # to BOTH the attitude and the position trajectory -- otherwise
                # the triad rotates by the leveling angle but the path does not,
                # and camera motion stops lining up with the body axes (moving
                # "forward" no longer tracks the red arrow; the symptom of
                # only-rotate-attitude). Rotating position by the same grav_corr
                # keeps displacement and triad consistent.
                pose[:3, :3] = R_disp
                pose[:3, 3] = grav_corr @ pose[:3, 3]

                # Refresh the SLAM overlay for the viewer when the worker has a
                # new map snapshot. Apply the SAME world-frame transform as the
                # displayed path (grav_corr in optical, then optical->NED) so the
                # keyframe dots and loop links line up with the trajectory.
                if slam_state is not None:
                    ov = slam_state["overlay"]()
                    if ov is not None:
                        kf_opt, match_opt, loop_pairs, has_new = ov

                        def _to_ned(p):
                            return (_M_OPT_TO_NED @ (grav_corr @ p)).astype(
                                np.float32)

                        kf_ned = (np.array([_to_ned(p) for p in kf_opt],
                                           dtype=np.float32)
                                  if len(kf_opt) else
                                  np.zeros((0, 3), np.float32))
                        with self._slam_lock:
                            self._slam_kf_ned = kf_ned
                            # Only refresh the flash (matched dot + teleport
                            # link) when a NEW loop actually closed; otherwise
                            # leave the previous flash to fade out in the viewer.
                            if has_new:
                                self._slam_match_ned = (
                                    np.array([_to_ned(p) for p in match_opt],
                                             dtype=np.float32)
                                    if len(match_opt) else
                                    np.zeros((0, 3), np.float32))
                                self._slam_loop_ned = [
                                    np.stack([_to_ned(a), _to_ned(b)])
                                    for a, b in loop_pairs]
                                self._slam_flash_id += 1

                # Accelerometer-ONLY attitude (gravity-leveled, yaw=0) for live
                # side-by-side comparison in the UI -- computed every frame
                # regardless of the rest gate, so we can see what accel "wants"
                # even when leveling is being withheld.
                accel_q_ned = None
                if accel_cam is not None:
                    R0_opt = gravity_aligned_R0(accel_cam)        # cam->world, yaw=0
                    R_acc_ned = _M_OPT_TO_NED @ R0_opt @ _P_OPT_TO_FRD
                    accel_q_ned = _rot_to_quat_wxyz(R_acc_ned)

                # Rate-limited diagnostics so we can see, on the device, whether
                # the rest gate is firing and what accel is reporting.
                if accel_cam is not None:
                    accel_n += 1
                    if at_rest:
                        accel_used += 1
                    if time.monotonic() - last_tilt_log >= 1.0:
                        rate = accel_used / max(accel_n, 1)
                        ar, ap, _ = (np.degrees(
                            quat_to_rpy(accel_q_ned)) if accel_q_ned is not None
                            else (0.0, 0.0, 0.0))
                        print(f"[ours-vio] accel r/p={ar:+5.1f}/{ap:+5.1f} "
                              f"tilt={tilt_deg:4.1f} at_rest={100*rate:3.0f}% "
                              f"motion={motion:.2f} n={acc_cnt} "
                              f"|a|={np.linalg.norm(accel_cam):.2f} "
                              f"g_ref={vo._g_ref or 0:.2f}")
                        accel_n = 0
                        accel_used = 0
                        last_tilt_log = time.monotonic()

                pos_opt = pose[:3, 3]
                R_opt = pose[:3, :3]

                pos_ned = _M_OPT_TO_NED @ pos_opt
                # Body axes [forward, right, down] in NED for the viewer triad.
                R_ned = _M_OPT_TO_NED @ R_opt @ _P_OPT_TO_FRD
                q_ned = _rot_to_quat_wxyz(R_ned)

                now = time.monotonic()
                t = now - t0
                if prev_t is None:
                    vel_ned = np.zeros(3)
                else:
                    dt = max(now - prev_t, 1e-6)
                    vel_ned = (pos_ned - prev_pos_ned) / dt
                prev_pos_ned = pos_ned
                prev_t = now

                ok = bool(vo.last_info.get("ok", False))
                self._emit(Pose(
                    t=t,
                    pos_ned=pos_ned,
                    vel_ned=vel_ned,
                    quat_wxyz=q_ned,
                    tracking_ok=ok,
                    teleport=teleport,
                    accel_quat_wxyz=accel_q_ned,
                ))

                frames += 1
                if now - last_fps_t >= 0.5:
                    self.fps = frames / (now - last_fps_t)
                    frames = 0
                    last_fps_t = now

            if ba_state is not None:
                ba_state["stop"].set()
                ba_state["event"].set()
                ba_state["thread"].join(timeout=1.0)

            if slam_state is not None:
                slam_state["stop"].set()
                slam_state["event"].set()
                slam_state["thread"].join(timeout=1.0)

    # ----------------------------------------------------------------------- #
    def _collect_startup_accel(self, q_imu, R_imu_cam: np.ndarray,
                               window_s: float = 0.4,
                               timeout_s: float = 2.0) -> np.ndarray | None:
        """Average the accelerometer over a short static startup window.

        Returns the mean specific-force vector rotated into the camera optical
        frame (ready for :meth:`RGBDVisualOdometry.align_to_gravity`), or ``None``
        if no IMU samples arrived within ``timeout_s`` (older device / no IMU) —
        in which case the caller falls back to an identity (unleveled) start.

        As a side effect, the mean gyro over the same static window is stored in
        ``self._gyro_bias`` (rad/s, IMU frame). The device is assumed motionless
        at startup, so this is the gyro zero-rate offset; subtracting it before
        integration keeps the rotation prior from drifting when the camera is
        still. Left ``None`` if no IMU samples arrived.
        """
        samples: list[np.ndarray] = []
        gyro: list[np.ndarray] = []
        t_start = time.monotonic()
        t_first: float | None = None
        while time.monotonic() - t_start < timeout_s:
            msg = q_imu.tryGet()
            if msg is None:
                time.sleep(0.005)
                continue
            for pkt in msg.packets:
                a = pkt.acceleroMeter
                samples.append(np.array([a.x, a.y, a.z], dtype=np.float64))
                g = pkt.gyroscope
                w = np.array([g.x, g.y, g.z], dtype=np.float64)
                if np.all(np.isfinite(w)):
                    gyro.append(w)
            if t_first is None:
                t_first = time.monotonic()
            elif time.monotonic() - t_first >= window_s:
                break
        if gyro:
            self._gyro_bias = np.mean(gyro, axis=0)
        if not samples:
            return None
        accel_imu = np.mean(samples, axis=0)
        return R_imu_cam @ accel_imu

    # ----------------------------------------------------------------------- #
    def _start_ba_worker(self, K: np.ndarray) -> dict:
        """Spawn the background sliding-window BA thread.

        Returns a state dict with ``submit(T_cw, ids, pts, depth)`` and
        ``poll() -> C | None``. The worker keeps the BA map in the *raw f2f*
        world frame (it is always fed raw f2f poses), so the published
        correction ``C = inv(T_ba) @ T_cw`` maps that frame onto the BA-refined
        one. Because the BA core is vectorised NumPy (which releases the GIL on
        its heavy linear-algebra), a thread is enough to keep the device read
        loop responsive — no separate process needed.
        """
        import threading

        from ..vio import WindowedBAMap, WindowedConfig
        from ..vio.bundle import BAConfig

        # use_gravity=True adds the accelerometer leveling prior INSIDE the
        # sliding-window BA, so the optimised map keeps its roll/pitch pinned to
        # real gravity (no display-side correction needed). Only at-rest accel
        # samples are submitted per keyframe (see the read loop), so a moving
        # keyframe simply carries no gravity constraint.
        cfg = WindowedConfig(window=self.ba_window, kf_every=self.ba_kf_every,
                             ba=BAConfig(max_iters=self.ba_iters, huber_px=2.0,
                                         use_gravity=True))
        ba_map = WindowedBAMap(K, cfg)

        snap_lock = threading.Lock()
        out_lock = threading.Lock()
        event = threading.Event()
        stop = threading.Event()
        state = {
            "event": event,
            "stop": stop,
            "kf_every": cfg.kf_every,
            "_pending": None,
            "_corr": None,
        }

        def submit(T_cw, ids, pts, depth_m, accel):
            with snap_lock:
                state["_pending"] = (T_cw, ids, pts, depth_m, accel)
            event.set()

        def poll():
            with out_lock:
                C = state["_corr"]
                state["_corr"] = None
            return C

        def worker():
            while not stop.is_set():
                event.wait()
                event.clear()
                if stop.is_set():
                    break
                with snap_lock:
                    snap = state["_pending"]
                    state["_pending"] = None
                if snap is None:
                    continue
                T_cw, ids, pts, depth_m, accel = snap
                ba_map.add_keyframe(T_cw, ids, pts, depth_m, accel_cam=accel)
                post = ba_map.run_ba()
                if post is not None:
                    with out_lock:
                        state["_corr"] = np.linalg.inv(post) @ T_cw

        th = threading.Thread(target=worker, name="OursBAWorker", daemon=True)
        th.start()
        state["thread"] = th
        state["submit"] = submit
        state["poll"] = poll
        return state

    # ----------------------------------------------------------------------- #
    def _start_slam_worker(self, K: np.ndarray) -> dict:
        """Spawn the background loop-closure SLAM thread.

        Returns a state dict with ``submit(T_wc, gray, depth_m)`` and
        ``poll() -> C | None``. The worker keeps a persistent
        :class:`oakd.vio.SlamMap` of every keyframe (in the *raw f2f* world
        frame, so its odometry edges stay self-consistent), recognises revisited
        places and runs pose-graph optimisation when a loop is confirmed. After
        each keyframe it publishes the world-frame correction for the **latest**
        keyframe, ``C = kf_pose[last] · inv(kf_orig[last])``; the read loop eases
        that onto the current f2f pose (the current frame hangs off the latest
        keyframe, so this maps it into the loop-corrected world). Until the first
        loop closes the correction is identity, so the display is plain f2f.

        ORB detection + matching against the growing keyframe set is the slow
        part, which is exactly why it lives on this background thread; the device
        read loop keeps running fast f2f for the display.
        """
        import threading

        from ..vio import SlamMap
        from ..vio.slam import SlamConfig

        # Spatial gating keeps loop detection bounded as the map grows so the
        # configured keyframe cadence stays sustainable on the background thread.
        slam = SlamMap(K, SlamConfig(
            loop_search_radius_m=self.slam_radius_m,
            loop_max_odom_rot_deg=30.0,
            kf_min_trans_m=self.slam_kf_min_trans,
            kf_min_rot_deg=self.slam_kf_min_rot,
            max_keyframes=self.slam_max_kf))

        snap_lock = threading.Lock()
        out_lock = threading.Lock()
        event = threading.Event()
        stop = threading.Event()
        reset = threading.Event()
        state = {
            "event": event,
            "stop": stop,
            "reset": reset,
            "kf_every": self.slam_kf_every,
            "_pending": None,
            "_corr": None,
            "_overlay": None,
        }

        def submit(T_wc, gray, depth_m):
            with snap_lock:
                state["_pending"] = (T_wc, gray, depth_m)
            event.set()

        def reset_map():
            # Ask the worker to forget every keyframe on its next wake.
            reset.set()
            event.set()

        def poll():
            with out_lock:
                C = state["_corr"]
                state["_corr"] = None
            return C

        def overlay():
            with out_lock:
                ov = state["_overlay"]
                state["_overlay"] = None
            return ov

        def worker():
            from ..vio.posegraph import se3_inv

            while not stop.is_set():
                event.wait()
                event.clear()
                if stop.is_set():
                    break
                if reset.is_set():
                    reset.clear()
                    slam.reset()
                    with snap_lock:
                        state["_pending"] = None
                    with out_lock:
                        # Identity correction (no loop) + an empty overlay marked
                        # "new" so the read loop drops the keyframe dots and the
                        # loop flash on its next refresh.
                        state["_corr"] = np.eye(4)
                        state["_overlay"] = (np.zeros((0, 3)),
                                             np.zeros((0, 3)), [], True)
                    continue
                with snap_lock:
                    snap = state["_pending"]
                    state["_pending"] = None
                if snap is None:
                    continue
                T_wc, gray, depth_m = snap
                events = slam.add_keyframe(T_wc, gray, depth_m)
                if events:
                    slam.optimize()
                    for ev in events:
                        print(f"[ours-slam] loop closed: kf {ev['cur']} <-> "
                              f"{ev['old']} ({ev['inliers']} inliers)")
                # Publish the correction for the latest keyframe (identity until
                # a loop has closed). The current display pose hangs off it.
                last = len(slam.kf_orig) - 1
                C = None
                if last >= 0:
                    C = slam.kf_pose[last] @ se3_inv(slam.kf_orig[last])
                # Snapshot the map for the UI overlay: every keyframe position,
                # the revisited (matched) keyframes to highlight, and the loop
                # links as [cur, old] segments. Positions are the CURRENT
                # (PGO-corrected) keyframe poses in the camera-optical world
                # frame; the read loop maps them to NED. These are real SlamMap
                # outputs, so the dots/links always reflect the actual graph.
                kf_opt = (
                    np.array([p[:3, 3] for p in slam.kf_pose], dtype=np.float64)
                    if slam.kf_pose else np.zeros((0, 3)))
                # Flash ONLY the loops confirmed at THIS keyframe (``events``),
                # i.e. the teleport that just happened -- not the whole history.
                match_opt = (
                    np.array([slam.kf_pose[ev["old"]][:3, 3] for ev in events],
                             dtype=np.float64)
                    if events else np.zeros((0, 3)))
                loop_pairs = [
                    (slam.kf_pose[ev["cur"]][:3, 3].copy(),
                     slam.kf_pose[ev["old"]][:3, 3].copy())
                    for ev in events]
                with out_lock:
                    state["_corr"] = C
                    state["_overlay"] = (kf_opt, match_opt, loop_pairs,
                                         bool(events))

        th = threading.Thread(target=worker, name="OursSlamWorker", daemon=True)
        th.start()
        state["thread"] = th
        state["submit"] = submit
        state["poll"] = poll
        state["overlay"] = overlay
        state["reset_map"] = reset_map
        return state

