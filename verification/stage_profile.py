"""Per-algorithm-stage profiler for the from-scratch RGB-D VIO/SLAM pipeline.

Run it on the Raspberry Pi 5 (or any host) to see WHERE the per-frame time goes
so the 320x200 real-time (>=20 fps) target can be attacked stage by stage. It
replays a recorded gold session, downsamples each native-640x400 stereo pair to
the requested ``--res`` (exactly what live capture at ``--width/--height`` would
hand the matcher), and times each pipeline stage independently:

  (1) SGM stereo depth         -- sky.depth.stereo.SGMStereoMatcher (live cfg)
  (2) frontend                 -- KLT track + RGB-D PnP (sky.front.odometry)
  (3) windowed BA  (every 5th) -- sky.vio.window.WindowedVIOMap.run_ba (loose)
  (4) IMU preintegration       -- sky.vio.imu.preintegrate_imu (per-keyframe seg)
  (5) tight BA     (--tight)   -- sky.vio.window.optimize_vio / build_system

It runs on the LEAN flight runtime (numpy + numba ONLY -- no cv2, no scipy):
the SGM ``live()`` preset's median/speckle post-filters fall back to their
pure-numba implementations when cv2 is absent, and every np.linalg call here is
plain NumPy (host LAPACK), never inside an njit kernel -- so nothing needs scipy.

Numba JIT compile is warmed (5 untimed frames) before timing so the LLVM compile
never lands in a measured stage.

NOTES / honesty caveats
-----------------------
* The harness models capturing AT ``--res``: it builds a scaled StereoCalib
  (fx,cx by W/640 ; fy,cy by H/400 ; dist unchanged -- it is in normalised
  coords ; baseline extrinsic unchanged -- metric) and runs SGM directly on the
  downsampled pair. This is NOT the ToF "compute-high-downsample" path; it is the
  honest "the sensor outputs ``--res``" path the 320x200 goal asks about.
* Stage (1) recomputes the dense depth via ``dense_depth`` (the matcher's own
  rectify-right + SGM). Stage (2) reuses that depth so the frontend timing is
  pure track+PnP, not double-counted depth.
* Stage (5) times the WHOLE ``optimize_vio`` call: ``build_system`` is a nested
  closure inside it and cannot be called in isolation, so this is the closest
  real call. The IMU-factor finite-difference Jacobian cost the profiler flagged
  lives inside this same timed region.
* Stages are timed per frame and accumulated; BA/IMU stages only fire on
  keyframes (every 5th frame), so their sample count is smaller -- the table
  reports each stage's own n.
"""
from __future__ import annotations

import argparse
import copy
import sys
import time
from dataclasses import dataclass, field

import numpy as np

# --- project APIs (read against the real source, see module docstring) ------ #
from imu_camera.io.reader import SessionReader
from imu_camera.comms.lib.config.resolution import ResolutionProfile
from imu_camera.resolution_build import sgm_config
from imu_camera.modules.tof_downsample import _area_resize_gray

# The VIO project owns the resolution-scaled frontend/odometry tuning (its own
# vendored ResolutionProfile copy). We build the frontend stage EXACTLY as the
# live vio process does (vio/main.py) -- numba-aware window/pyramid/corner budget
# -- so the profiled per-frame cost matches the deployed Pi config, not an
# unscaled FrontendConfig() default (which does ~2-4x more KLT work).
from vio.comms.lib.config.resolution import ResolutionProfile as ResolutionProfileVio
from vio.resolution_build import frontend_config

from sky.depth.stereo import SGMStereoMatcher
from sky.front.odometry import RGBDVisualOdometry, OdometryConfig
from sky.front.frontend import KLTFrontend

try:
    from sky.front.klt_numba import HAVE_NUMBA
except Exception:          # numba absent -> pure-NumPy KLT (capped window/budget)
    HAVE_NUMBA = False
from sky.vio.imu import (
    integrate_gyro_camera,
    preintegrate_imu,
)
# LOOSE backend = the visual-only sliding-window BA the live non-tight path runs
# (vio/engine/__init__.make_ba_engine -> WindowedBAMap). NOT WindowedVIOMap --
# that is the TIGHT joint visual+IMU optimiser (make_vi_engine), a heavier
# different solver. Profiling loose with WindowedVIOMap inflates the BA cost.
from sky.backend.windowed import WindowedBAMap, WindowedConfig
from sky.backend.bundle import BAConfig
from sky.vio.window import WindowedVIOMap, WindowedVIOConfig

BASELINE_W = 640
BASELINE_H = 400
# Live backend window/iters (vio/main.py defaults backend_window/backend_iters,
# threaded into WindowedConfig at vio/modules/pipeline.py:543).
BACKEND_WINDOW = 6
BACKEND_ITERS = 5
KF_EVERY = 5                       # windowed BA / IMU stages fire every 5th frame
# Warm enough frames that the JIT compile of EVERY stage (incl. the BA/IMU/tight
# kernels, which only do real work once the window holds >=2 keyframes) lands
# OUTSIDE the timed region. 2*KF_EVERY+1 guarantees >=2 warmup keyframes -> the
# first real run_ba / optimize_vio / preintegrate_imu solve compiles while still
# warming, never in a measured sample.
WARMUP_FRAMES = 2 * KF_EVERY + 1


# --------------------------------------------------------------------------- #
# Resolution scaling: build a StereoCalib at the target res (capture-at-res)
# --------------------------------------------------------------------------- #
def _scale_camera_calib(cam, sx: float, sy: float, out_w: int, out_h: int):
    """Anisotropically scale a CameraCalib's intrinsics to the target grid.

    fx,cx scale by sx = W/640 ; fy,cy scale by sy = H/400. Distortion is in
    NORMALISED image coordinates (the rectifier applies it before the K matmul),
    so it is resolution-invariant -- copied unchanged. Width/height are set to
    the target so the rectifier builds its remap grid at ``--res``.
    """
    c = copy.deepcopy(cam)
    c.fx = float(cam.fx) * sx
    c.cx = float(cam.cx) * sx
    c.fy = float(cam.fy) * sy
    c.cy = float(cam.cy) * sy
    c.width = int(out_w)
    c.height = int(out_h)
    return c


def _scale_stereo_calib(calib, out_w: int, out_h: int):
    """Return a deep copy of the StereoCalib scaled to ``out_w x out_h``.

    The left intrinsic is what the SGM matcher uses as ``K`` (fx*baseline/disp);
    both left and right intrinsics are scaled so the rectifier's right->left
    remap stays self-consistent at the new grid. The metric baseline extrinsic
    (``T_left_right``) and the IMU<->cam extrinsics are unchanged (metres, not
    pixels).
    """
    sx = out_w / BASELINE_W
    sy = out_h / BASELINE_H
    sc = copy.deepcopy(calib)
    sc.left = _scale_camera_calib(calib.left, sx, sy, out_w, out_h)
    sc.right = _scale_camera_calib(calib.right, sx, sy, out_w, out_h)
    return sc


# --------------------------------------------------------------------------- #
# Per-frame inputs prepared once (downsample is NOT a timed pipeline stage)
# --------------------------------------------------------------------------- #
@dataclass
class PreparedFrame:
    seq: int
    ts_ns: int
    gray_left: np.ndarray         # (H, W) uint8, area-resized to target res
    gray_right: np.ndarray        # (H, W) uint8, area-resized to target res


def _prepare_frames(reader: SessionReader, out_w: int, out_h: int,
                    n: int) -> list[PreparedFrame]:
    """Load + area-resize the first ``n`` stereo frames to the target res.

    Uses the project's cv2-free area resize (``_area_resize_gray``, bit-exact vs
    cv2 INTER_AREA) on BOTH the rectified-left and the raw-right -- the same
    operation live capture at ``--width/--height`` performs. Done up front and
    EXCLUDED from the stage timings (it models the camera/ISP output, not a
    pipeline stage).
    """
    frames: list[PreparedFrame] = []
    count = min(n, len(reader))
    for i in range(count):
        f = reader.load_frame(i, load_right=True)
        if f.gray_right is None:
            raise RuntimeError(
                f"session frame {i} has no right image; cannot run SGM stage")
        gl = _area_resize_gray(f.gray_left, out_h, out_w)
        gr = _area_resize_gray(f.gray_right, out_h, out_w)
        frames.append(PreparedFrame(f.seq, f.ts_ns, gl, gr))
    return frames


# --------------------------------------------------------------------------- #
# IMU helpers: per-frame gyro prior + per-interval raw segment (camera frame)
# --------------------------------------------------------------------------- #
def _imu_startup(reader: SessionReader):
    """``(R_imu_cam, accel_align, gyro_bias)`` -- mirrors _replay_imu_startup."""
    if not reader.calib.has_imu_extrinsics:
        return None, None, None
    imu = reader.load_imu()
    ts = imu["ts_ns"]
    if ts.size <= 1:
        return None, None, None
    R_imu_cam = reader.calib.T_imu_left[:3, :3]
    t0 = int(ts[0])
    gwin = ts <= t0 + int(1.0e9)
    gyro_bias = imu["gyro"][gwin].mean(axis=0) if gwin.any() else np.zeros(3)
    awin = ts <= t0 + int(0.3e9)
    accel_align = (R_imu_cam @ imu["accel"][awin].mean(axis=0)
                   if awin.any() else None)
    return R_imu_cam, accel_align, gyro_bias


def _imu_block(imu: dict, t0_ns: int, t1_ns: int):
    """Raw IMU samples whose ts lie in (t0, t1]; returns (ts, gyro, accel)."""
    ts = imu["ts_ns"]
    lo = np.searchsorted(ts, t0_ns, side="right")
    hi = np.searchsorted(ts, t1_ns, side="right")
    sl = slice(lo, hi)
    return ts[sl], imu["gyro"][sl], imu["accel"][sl]


# --------------------------------------------------------------------------- #
# Timing accumulator
# --------------------------------------------------------------------------- #
@dataclass
class StageTimer:
    name: str
    samples: list = field(default_factory=list)   # ms per call

    def add(self, ms: float) -> None:
        self.samples.append(ms)

    @property
    def n(self) -> int:
        return len(self.samples)

    def stats(self) -> tuple[float, float, float]:
        if not self.samples:
            return 0.0, 0.0, 0.0
        a = np.asarray(self.samples, dtype=np.float64)
        return float(a.mean()), float(np.percentile(a, 50)), float(
            np.percentile(a, 95))


# --------------------------------------------------------------------------- #
# Main profiling loop
# --------------------------------------------------------------------------- #
def run(session: str, out_w: int, out_h: int, n_frames: int,
        tight: bool) -> int:
    print(f"[stage_profile] session={session} res={out_w}x{out_h} "
          f"frames={n_frames} tight={tight}")

    reader = SessionReader(session)
    src_w = int(reader.calib.left.width)
    src_h = int(reader.calib.left.height)
    if (src_w, src_h) != (BASELINE_W, BASELINE_H):
        print(f"[warn] session native res is {src_w}x{src_h}, not "
              f"{BASELINE_W}x{BASELINE_H}; K scaling assumes 640x400 baseline.")

    # Scaled calib + matcher (capture-at-res model). The gold right frame is RAW,
    # so the matcher rectifies the right internally (rectify_left=False, as the
    # replay path does); the left is already the chip's rectified-left.
    scaled_calib = _scale_stereo_calib(reader.calib, out_w, out_h)
    res = ResolutionProfile.for_resolution(out_w, out_h)
    sgm_cfg = sgm_config(res, fast=True)            # SGMConfig.live() + res ndisp
    matcher = SGMStereoMatcher.from_calib(scaled_calib, sgm_cfg)
    K = scaled_calib.left.K
    print(f"[stage_profile] sgm: downscale={sgm_cfg.downscale} "
          f"ndisp={sgm_cfg.num_disparities} paths={sgm_cfg.num_paths} "
          f"census_r={sgm_cfg.census_radius} K_fx={K[0, 0]:.1f} K_fy={K[1, 1]:.1f}")

    # Prepare frames (downsample once, off the clock).
    frames = _prepare_frames(reader, out_w, out_h, n_frames)
    if len(frames) < WARMUP_FRAMES + 2:
        print(f"[error] only {len(frames)} frames available; need at least "
              f"{WARMUP_FRAMES + 2}.")
        return 2

    # IMU stream + startup references.
    imu = reader.load_imu()
    R_imu_cam, accel_align, gyro_bias = _imu_startup(reader)
    if gyro_bias is None:
        gyro_bias = np.zeros(3)

    # --- Stage objects --------------------------------------------------- #
    # Frontend odometry: build the config EXACTLY as the live vio process does
    # (vio/main.py) so the profiled per-frame cost is the deployed Pi cost, not
    # an inflated default. frontend_config(res, numba=HAVE_NUMBA) gives the
    # resolution-scaled, numba-aware KLT window / pyramid depth / corner budget
    # (at 320x200 with numba: full res-scaled; without numba: the capped live_own
    # budget). odom uses gyro_fuse=True (an IMU session is present) -- the same
    # gyro-seeded RGB-D PnP the live frontend runs.
    res_vio = ResolutionProfileVio.for_resolution(out_w, out_h)
    fe_cfg = frontend_config(res_vio, numba=HAVE_NUMBA)
    odo_cfg = OdometryConfig(gyro_fuse=(R_imu_cam is not None))
    print(f"[stage_profile] frontend: numba={HAVE_NUMBA} "
          f"win={fe_cfg.win_size} levels={fe_cfg.max_level} "
          f"corners={fe_cfg.max_corners} bucketed={fe_cfg.bucketed} "
          f"reproj={odo_cfg.ransac_reproj_px}px")
    vo = RGBDVisualOdometry(K, odo_cfg, KLTFrontend(fe_cfg))
    if accel_align is not None:
        vo.align_to_gravity(accel_align)

    # Loose windowed BA map (stage 3): the EXACT live loose backend --
    # WindowedBAMap with the live window=6 / max_iters=5 config (vio/main.py
    # defaults). This is the visual-only projection+depth BA the non-tight flight
    # path runs on every keyframe (also under --direct: only --tight swaps it for
    # the joint VIO map).
    loose_cfg = WindowedConfig(
        window=BACKEND_WINDOW,
        ba=BAConfig(max_iters=BACKEND_ITERS, huber_px=2.0))
    loose_map = WindowedBAMap(K, cfg=loose_cfg)

    # Tight windowed VIO map (stage 5): IMU factors on, velocity stabilised --
    # the --tight backend whose optimize_vio/build_system is the profiled wall.
    tight_map = None
    if tight:
        # Full IMU stream rotated into the camera optical frame (the frame the
        # tight map's preintegration expects), as imu_prior.preintegrate_prior
        # supplies it live.
        if R_imu_cam is None:
            print("[warn] session has no IMU extrinsics; --tight stage skipped.")
            tight = False
        else:
            R_ic = np.asarray(R_imu_cam, np.float64)
            gyro_cam = imu["gyro"] @ R_ic.T
            accel_cam = imu["accel"] @ R_ic.T
            # Bare `--tight` config: the live path is `WindowedVIOConfig()` with
            # only `vio.imu_info_weight = True` (vio/modules/pipeline.py:519-520);
            # stabilize_velocity / depth_icp stay OFF unless their flags are given.
            # window=8 / vio.max_iters=12 are the WindowedVIOConfig defaults the
            # live tight backend uses -- so this times the real `--tight` solve.
            tight_cfg = WindowedVIOConfig()
            tight_cfg.vio.imu_info_weight = True
            tight_map = WindowedVIOMap(
                K, ts_ns=imu["ts_ns"], gyro_cam=gyro_cam, accel_cam=accel_cam,
                bg0=gyro_bias, ba0=np.zeros(3), cfg=tight_cfg)

    timers = {
        "sgm_depth": StageTimer("SGM stereo depth"),
        "frontend": StageTimer("Frontend (KLT track + RGB-D PnP)"),
        "windowed_ba": StageTimer("Windowed BA loose (every 5th)"),
        "imu_preint": StageTimer("IMU preintegration (per-KF segment)"),
    }
    if tight:
        timers["tight_ba"] = StageTimer("Tight optimize_vio/build_system")

    prev_kf_ts = None         # for the loose-map keyframe IMU preint segment

    total_frames_timed = 0
    for fi, fr in enumerate(frames):
        warming = fi < WARMUP_FRAMES
        gl, gr = fr.gray_left, fr.gray_right

        # --- per-frame gyro prior (camera frame) for the frontend solve ----- #
        R_prior = None
        if R_imu_cam is not None and fi > 0:
            t_prev = frames[fi - 1].ts_ns
            its, ig, _ia = _imu_block(imu, t_prev, fr.ts_ns)
            if its.size >= 2:
                ig_unbiased = ig - gyro_bias
                R_prior = integrate_gyro_camera(its, ig_unbiased, R_imu_cam)

        # ===== Stage 1: SGM stereo depth ===================================
        t0 = time.perf_counter()
        depth = matcher.dense_depth(gl, gr)
        dt = (time.perf_counter() - t0) * 1e3
        if not warming:
            timers["sgm_depth"].add(dt)

        # ===== Stage 2: frontend (KLT track + RGB-D PnP) ===================
        # vo.process == vo.track (KLT, the numba-parallel section) then
        # vo.estimate (pure-NumPy RGB-D PnP), exactly the two vio frontend steps.
        t0 = time.perf_counter()
        pose = vo.process(gl, depth, R_prior=R_prior)
        dt = (time.perf_counter() - t0) * 1e3
        if not warming:
            timers["frontend"].add(dt)

        is_kf = (fi % KF_EVERY == 0)
        tracks = vo.frontend.tracks      # live {ids, points} snapshot

        if is_kf:
            # ----- Stage 4: IMU preintegration over the keyframe interval --- #
            # Preintegrate the raw IMU block (camera frame) spanning since the
            # previous keyframe -- the per-keyframe segment the tight backend's
            # add_keyframe builds. Timed standalone so the IMU-factor build cost
            # is visible apart from the BA solve.
            if R_imu_cam is not None and prev_kf_ts is not None:
                its, ig, ia = _imu_block(imu, prev_kf_ts, fr.ts_ns)
                if its.size >= 2:
                    R_ic = np.asarray(R_imu_cam, np.float64)
                    ig_cam = ig @ R_ic.T
                    ia_cam = ia @ R_ic.T
                    t0 = time.perf_counter()
                    preintegrate_imu(its, ig_cam, ia_cam, gyro_bias,
                                     np.zeros(3))
                    dt = (time.perf_counter() - t0) * 1e3
                    if not warming:
                        timers["imu_preint"].add(dt)
            prev_kf_ts = fr.ts_ns

            # ----- Stage 3: windowed BA (loose) ----------------------------- #
            # Register the keyframe (pose + track snapshot + depth) then solve.
            # use_imu=False so this times the visual projection+depth BA only.
            # add_keyframe expects T_cw (world->camera); vo.process returns
            # self.pose = T_world_cur (camera->world), so invert -- exactly what
            # the live drivers do (sky/backend/windowed.py:350, window.py:1394).
            # Passing the un-inverted pose back-projects every landmark to a wrong
            # world point and the BA solve degenerates (its timing unrepresentative).
            T_cw = np.linalg.inv(pose)
            # WindowedBAMap.add_keyframe(T_cw, ids, pts, depth_m, accel_cam=None)
            # -- no ts_ns (visual-only); matches vio.engine.steps.ba_step.
            loose_map.add_keyframe(
                T_cw, tracks.ids, tracks.points, depth)
            t0 = time.perf_counter()
            loose_map.run_ba()
            dt = (time.perf_counter() - t0) * 1e3
            if not warming and len(loose_map.keyframes) >= 2:
                timers["windowed_ba"].add(dt)

            # ----- Stage 5: tight optimize_vio (--tight) -------------------- #
            if tight and tight_map is not None:
                tight_map.add_keyframe(
                    T_cw, tracks.ids, tracks.points, depth, fr.ts_ns)
                t0 = time.perf_counter()
                tight_map.run_ba()
                dt = (time.perf_counter() - t0) * 1e3
                if not warming and len(tight_map.keyframes) >= 2:
                    timers["tight_ba"].add(dt)

        if not warming:
            total_frames_timed += 1

    _report(timers, total_frames_timed, out_w, out_h, tight)
    return 0


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #
def _report(timers: dict, n_frames: int, out_w: int, out_h: int,
            tight: bool) -> None:
    # Per-FRAME mean for each stage (BA/IMU fire only on keyframes, so their
    # per-frame amortised cost = sum / total_frames_timed). The %-of-total and
    # the achievable-fps use the AMORTISED per-frame budget, which is the honest
    # number for "can the pipeline keep up at 20 fps".
    print()
    print(f"=== Per-stage timing  ({out_w}x{out_h}, {n_frames} timed frames) ===")
    header = (f"{'stage':<34}{'n':>5}{'mean ms':>10}{'p50':>9}{'p95':>9}"
              f"{'/frame ms':>11}{'% total':>9}")
    print(header)
    print("-" * len(header))

    per_frame = {}
    for key, t in timers.items():
        amortised = (sum(t.samples) / n_frames) if n_frames else 0.0
        per_frame[key] = amortised
    total_pf = sum(per_frame.values())

    order = ["sgm_depth", "frontend", "windowed_ba", "imu_preint"]
    if tight:
        order.append("tight_ba")
    for key in order:
        t = timers.get(key)
        if t is None:
            continue
        mean, p50, p95 = t.stats()
        pf = per_frame[key]
        pct = (100.0 * pf / total_pf) if total_pf > 0 else 0.0
        print(f"{t.name:<34}{t.n:>5}{mean:>10.2f}{p50:>9.2f}{p95:>9.2f}"
              f"{pf:>11.2f}{pct:>8.1f}%")

    print("-" * len(header))
    print(f"{'TOTAL (amortised per frame)':<34}{'':>5}{'':>10}{'':>9}{'':>9}"
          f"{total_pf:>11.2f}{100.0:>8.1f}%")

    fps = (1000.0 / total_pf) if total_pf > 0 else float("inf")
    clears = fps >= 20.0
    print()
    # This is the SERIAL sum of per-stage wall-clocks measured in ONE process
    # (each stage's own kernels may already be numba-parallel; what is NOT
    # modelled here is cross-STAGE pipelining across the live 4 OS processes --
    # which is exactly the parallel headroom the 320x200 goal targets).
    print(f"Implied max throughput: {fps:6.2f} fps "
          f"(serial per-stage sum, no cross-stage pipelining, "
          f"{total_pf:.2f} ms/frame)")
    print(f"Clears 20 fps target:   {'YES' if clears else 'NO'}")
    if not clears:
        # Rank the stages by amortised cost so the next optimisation target is
        # obvious (the whole point of this harness).
        ranked = sorted(per_frame.items(), key=lambda kv: kv[1], reverse=True)
        worst = ", ".join(f"{timers[k].name.split(' (')[0]} "
                          f"({v:.1f} ms)" for k, v in ranked[:2] if v > 0)
        print(f"Top cost: {worst}")
    print()
    print("NOTE: BA/IMU stages fire every 5th frame; their 'n' is the keyframe "
          "count and '/frame ms' is the amortised per-frame share. SGM + "
          "frontend run EVERY frame. 'mean/p50/p95' are per-CALL (per keyframe "
          "for BA/IMU). Stage 5 times the whole optimize_vio (build_system is a "
          "nested closure, not isolable).")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Per-stage profiler for the RGB-D VIO/SLAM pipeline.")
    ap.add_argument("--session", default="sessions/gold/push_straight_fast_15s",
                    help="recorded gold session dir (native 640x400).")
    ap.add_argument("--res", default="320x200",
                    help="target capture resolution WxH (e.g. 320x200).")
    ap.add_argument("--frames", type=int, default=200,
                    help="number of frames to profile (incl. 5 warmup).")
    ap.add_argument("--tight", action="store_true",
                    help="also profile the tight optimize_vio/build_system.")
    args = ap.parse_args(argv)

    try:
        out_w, out_h = (int(x) for x in args.res.lower().split("x"))
    except ValueError:
        print(f"[error] bad --res '{args.res}'; expected WxH like 320x200.")
        return 2

    return run(args.session, out_w, out_h, args.frames, args.tight)


if __name__ == "__main__":
    sys.exit(main())
