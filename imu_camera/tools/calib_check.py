#!/usr/bin/env python3
"""Calibration check tool -- a pre-flight / CI gate for a session's calibration.

WHAT THIS IS
------------
Before a run (live or replay) the camera/stereo/IMU calibration is the silent
foundation everything else trusts: the VIO solves with ``K``, the stereo depth
scales with the baseline, the gyro prior rotates by ``R_imu_cam``. A malformed or
implausible calib does not crash -- it quietly poisons the trajectory. This tool
validates the *parsed* :class:`~imu_camera.io.reader.StereoCalib` (the exact object
the live pipeline consumes) against physical sanity bands and flags problems
BEFORE they cost a run, so it can sit in a pre-run check or a CI gate.

It does NOT re-parse ``calib.json`` -- it reuses the project's own loader
(:class:`~imu_camera.io.reader.SessionReader` / ``StereoCalib.from_json``), so the
values it checks are byte-identical to what the pipeline will actually use. In
particular the loader converts the ``T_left_right`` translation from centimetres
to metres; this tool validates the *metres* value (OAK-D baseline ~= 0.075 m), and
specifically catches the case where that cm->m conversion was skipped or doubled.

It is multi-chip-generic: it validates the abstract ``StereoCalib`` / pinhole
intrinsics, not OAK-D-specific JSON quirks (beyond the documented cm->m note).

USAGE
-----
Validate a recorded session (primary -- can also check recorded-data consistency)::

    python -m imu_camera.tools.calib_check --session sessions/gold/lab_loop_30s

Validate a bare ``calib.json`` (secondary -- no recorded-frame checks)::

    python -m imu_camera.tools.calib_check --calib path/to/calib.json

Treat WARN as a hard failure for the exit code (strict CI gate)::

    python -m imu_camera.tools.calib_check --session <dir> --strict

EXIT CODE
---------
* ``0`` -- no FAIL (WARN is allowed unless ``--strict``).
* nonzero -- any FAIL, or (under ``--strict``) any WARN. Suitable as a gate.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from imu_camera.io.reader import (  # noqa: E402
    CameraCalib,
    SessionReader,
    StereoCalib,
)

# --------------------------------------------------------------------------- #
# Status vocabulary + a single check result row.
# --------------------------------------------------------------------------- #
PASS = "PASS"   # measured value is squarely in the expected band
WARN = "WARN"   # outside the ideal band but not provably broken (off-centre lens,
                # wide-but-physical FOV, ...) -- gate fails on it only under --strict
FAIL = "FAIL"   # provably wrong / non-physical / inconsistent -- always gates
INFO = "INFO"   # a valid-but-notable state (e.g. no IMU extrinsics) -- never gates


@dataclass
class CheckResult:
    """One row of the report: a named check, what it measured vs expected, status."""

    name: str
    measured: str
    expected: str
    status: str
    note: str = ""  # human explanation, shown for every non-PASS row


# --------------------------------------------------------------------------- #
# Thresholds (tuned so a known-good OAK-D W gold calib stays entirely PASS/INFO;
# values that triggered tuning are called out inline).
# --------------------------------------------------------------------------- #
_ASPECT_WARN = 0.01          # |fx-fy|/fx < 1% PASS, < 5% WARN, else FAIL
_ASPECT_FAIL = 0.05
_PP_CENTRE_PASS = 0.10       # principal point within 10% of centre = PASS,
_PP_CENTRE_WARN = 0.20       #   < 20% WARN, else WARN-with-note (never FAIL: off-centre lenses exist)
_HFOV_SANE_LO = 30.0         # horizontal FOV sane band (deg); OAK-D W is wide ~95-110
_HFOV_SANE_HI = 150.0
_DIST_HUGE = 1e3             # any |dist coeff| above this is suspicious (WARN)
_KNOWN_DIST_LENS = {0, 4, 5, 8, 12, 14}  # recognised OpenCV distortion-model lengths
_SO3_TOL = 1e-3              # ||R Rt - I|| above this => not a rotation (FAIL)
_DET_TOL = 1e-2              # |det(R) - 1| above this => reflection/scale (FAIL)
_STEREO_ANG_PASS = 2.0       # inter-camera rotation: < 2deg PASS, < 5deg WARN, else WARN-note
_STEREO_ANG_WARN = 5.0
_BASELINE_LO = 0.02          # plausible stereo baseline band (m)
_BASELINE_HI = 0.30
_IMU_LEVER_PASS = 0.10       # IMU<->cam lever-arm (m): < 10cm PASS, else WARN
_DEPTH_LO = 0.2              # plausible indoor median depth band (m)
_DEPTH_HI = 15.0
_DEPTH_MIN_VALID_FRAC = 0.05  # a frame needs >=5% valid pixels before its median is
                              # trusted (skips stereo warm-up frames pinned at the far rail)


# --------------------------------------------------------------------------- #
# Small numeric helpers (kept local + pure; no dependency on the VIO math libs so
# this gate stays standalone and importable anywhere).
# --------------------------------------------------------------------------- #
def _rotation_angle_deg(R: np.ndarray) -> float:
    """Geodesic angle of a (near-)rotation matrix, in degrees (== ||log(R)||)."""
    # arccos((tr R - 1) / 2) is the SO(3) angle; clip guards tiny numeric overshoot.
    cos_theta = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0))))


def _all_finite(a: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(a)))


# --------------------------------------------------------------------------- #
# Intrinsics checks (run for BOTH left and right).
# --------------------------------------------------------------------------- #
def check_intrinsics(cam: CameraCalib, side: str) -> list[CheckResult]:
    """Validate one pinhole camera's intrinsics; ``side`` labels the rows (L/R)."""
    out: list[CheckResult] = []
    p = side  # row-name prefix, e.g. "L" / "R"

    # 1. fx, fy finite and > 0.
    fx, fy = cam.fx, cam.fy
    fxy_ok = np.isfinite(fx) and np.isfinite(fy) and fx > 0 and fy > 0
    out.append(CheckResult(
        f"{p} fx,fy positive", f"fx={fx:.2f} fy={fy:.2f}", "finite, >0",
        PASS if fxy_ok else FAIL,
        "" if fxy_ok else "focal length non-finite or <=0 -- intrinsics unusable"))
    # If focal lengths are broken the aspect/FOV checks below would divide by junk;
    # guard them so the report stays meaningful instead of raising.
    fxy_safe = fxy_ok

    # 2. Pixel aspect: |fx-fy|/fx.
    if fxy_safe:
        aspect = abs(fx - fy) / fx
        if aspect < _ASPECT_WARN:
            st, note = PASS, ""
        elif aspect < _ASPECT_FAIL:
            st, note = WARN, "non-square pixels (>1%) -- check fx/fy or a bad resize"
        else:
            st, note = FAIL, "strongly non-square pixels (>5%) -- intrinsics likely wrong"
        out.append(CheckResult(f"{p} pixel aspect", f"{aspect * 100:.3f}%",
                               f"<{_ASPECT_WARN * 100:.0f}%", st, note))

    # 3. Principal point inside image + near centre.
    w, h = cam.width, cam.height
    cx, cy = cam.cx, cam.cy
    inside = (0 < cx < w) and (0 < cy < h) and w > 0 and h > 0
    if not inside:
        out.append(CheckResult(
            f"{p} principal point", f"cx={cx:.1f} cy={cy:.1f}", f"in 0..{w} x 0..{h}",
            FAIL, "principal point falls outside the image -- intrinsics/size mismatch"))
    else:
        off = max(abs(cx - w / 2) / w, abs(cy - h / 2) / h)
        if off < _PP_CENTRE_PASS:
            st, note = PASS, ""
        elif off < _PP_CENTRE_WARN:
            st, note = WARN, "principal point >10% off centre -- unusual but possible"
        else:
            st, note = WARN, "principal point >20% off centre -- verify this is intended"
        out.append(CheckResult(f"{p} principal point", f"{off * 100:.1f}% off centre",
                               f"<{_PP_CENTRE_PASS * 100:.0f}% off", st, note))

    # 4. K property consistent with fx,fy,cx,cy (exact -- the loader builds K from
    #    these fields, so any drift means the dataclass was mutated inconsistently).
    K = cam.K
    expected_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    k_ok = np.array_equal(K, expected_K)
    out.append(CheckResult(
        f"{p} K matches fx/fy/cx/cy", "exact" if k_ok else "MISMATCH", "exact",
        PASS if k_ok else FAIL,
        "" if k_ok else "K property disagrees with fx/fy/cx/cy -- inconsistent calib object"))

    # 5. width, height > 0 (the L==R equality is a stereo-level check, see check_sizes_match).
    size_ok = w > 0 and h > 0
    out.append(CheckResult(
        f"{p} image size", f"{w}x{h}", ">0 x >0",
        PASS if size_ok else FAIL,
        "" if size_ok else "image width/height <= 0 -- calib has no valid resolution"))

    # 6. Horizontal FOV from fx + width.
    if fxy_safe and size_ok:
        hfov = np.degrees(2.0 * np.arctan(w / (2.0 * fx)))
        if hfov <= 0.0 or hfov >= 180.0:
            st, note = FAIL, "horizontal FOV non-physical (<=0 or >=180deg)"
        elif _HFOV_SANE_LO <= hfov <= _HFOV_SANE_HI:
            st, note = PASS, ""
        else:
            st, note = WARN, "horizontal FOV outside the typical 30-150deg band"
        out.append(CheckResult(f"{p} horizontal FOV", f"{hfov:.1f} deg",
                               f"{_HFOV_SANE_LO:.0f}-{_HFOV_SANE_HI:.0f} deg", st, note))

    # 7. Distortion coefficients: finite, known length, sane magnitude.
    dist = np.asarray(cam.dist, dtype=np.float64)
    n = int(dist.size)
    if not _all_finite(dist):
        out.append(CheckResult(f"{p} dist coeffs", f"len={n}, has NaN/inf",
                               "all finite", FAIL,
                               "distortion has NaN/inf -- undistort/rectify would blow up"))
    else:
        if n not in _KNOWN_DIST_LENS:
            st, note = FAIL, (f"unrecognised distortion length {n} "
                              f"(known: {sorted(_KNOWN_DIST_LENS)})")
        elif n and np.abs(dist).max() > _DIST_HUGE:
            st, note = WARN, (f"a distortion coeff is huge "
                              f"(|max|={np.abs(dist).max():.1f}) -- verify the model")
        else:
            st, note = PASS, ""
        out.append(CheckResult(f"{p} dist coeffs", f"len={n}, |max|="
                               f"{(np.abs(dist).max() if n else 0.0):.2f}",
                               "finite, known len", st, note))

    return out


# --------------------------------------------------------------------------- #
# Stereo-level checks (sizes match, extrinsic R, baseline, baseline axis).
# --------------------------------------------------------------------------- #
def check_sizes_match(calib: StereoCalib) -> CheckResult:
    """Check 5 (stereo half): left and right report the same resolution."""
    lw, lh = calib.left.width, calib.left.height
    rw, rh = calib.right.width, calib.right.height
    same = (lw == rw) and (lh == rh)
    return CheckResult(
        "L/R size equal", f"L {lw}x{lh}  R {rw}x{rh}", "equal",
        PASS if same else FAIL,
        "" if same else "left and right resolutions differ -- not a coherent stereo pair")


def check_stereo_extrinsic(calib: StereoCalib) -> list[CheckResult]:
    """Checks 8-11: T_left_right rotation in SO(3), angle, baseline, baseline axis."""
    out: list[CheckResult] = []
    T = np.asarray(calib.T_left_right, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]

    # 8. R in SO(3): orthonormal AND det ~= +1.
    if not _all_finite(R):
        out.append(CheckResult("stereo R orthonormal", "non-finite", "in SO(3)",
                               FAIL, "T_left_right rotation has NaN/inf"))
        out.append(CheckResult("stereo R det", "non-finite", "~= +1",
                               FAIL, "T_left_right rotation has NaN/inf"))
    else:
        ortho = float(np.linalg.norm(R @ R.T - np.eye(3)))
        ortho_ok = ortho < _SO3_TOL
        out.append(CheckResult(
            "stereo R orthonormal", f"||RRt-I||={ortho:.2e}", f"<{_SO3_TOL:.0e}",
            PASS if ortho_ok else FAIL,
            "" if ortho_ok else "stereo rotation is not orthonormal -- corrupt extrinsics"))
        det = float(np.linalg.det(R))
        det_ok = abs(det - 1.0) < _DET_TOL
        out.append(CheckResult(
            "stereo R det", f"det(R)={det:.4f}", "~= +1",
            PASS if det_ok else FAIL,
            "" if det_ok else ("stereo rotation det != +1 "
                               "(reflection/scale) -- not a proper rotation")))

        # 9. Inter-camera rotation angle (stereo cameras should be ~parallel).
        ang = _rotation_angle_deg(R) if ortho_ok else float("nan")
        if not np.isfinite(ang):
            out.append(CheckResult("stereo rotation angle", "n/a (R not SO(3))",
                                   f"<{_STEREO_ANG_PASS:.0f} deg", WARN,
                                   "skipped: rotation is not orthonormal"))
        elif ang < _STEREO_ANG_PASS:
            out.append(CheckResult("stereo rotation angle", f"{ang:.2f} deg",
                                   f"<{_STEREO_ANG_PASS:.0f} deg", PASS))
        elif ang < _STEREO_ANG_WARN:
            out.append(CheckResult("stereo rotation angle", f"{ang:.2f} deg",
                                   f"<{_STEREO_ANG_PASS:.0f} deg", WARN,
                                   "stereo cameras >2deg from parallel -- check rectification"))
        else:
            out.append(CheckResult("stereo rotation angle", f"{ang:.2f} deg",
                                   f"<{_STEREO_ANG_PASS:.0f} deg", WARN,
                                   "stereo cameras >5deg from parallel -- verify extrinsics"))

    # 10. Baseline magnitude (parsed = metres). Outside the plausible band is a FAIL
    #     with a unit-conversion hint -- the classic cm->m bug this tool exists to catch.
    base = float(np.linalg.norm(t))
    if _BASELINE_LO <= base <= _BASELINE_HI:
        out.append(CheckResult("stereo baseline", f"{base * 100:.2f} cm",
                               f"{_BASELINE_LO * 100:.0f}-{_BASELINE_HI * 100:.0f} cm", PASS))
    else:
        hint = ""
        if base > 1.0:
            hint = " (~7.5 => cm-to-m conversion skipped; raw cm leaked through)"
        elif 0.0 < base < _BASELINE_LO * 0.1:
            hint = " (~7.5e-4 => baseline double-converted cm->m->cm)"
        out.append(CheckResult(
            "stereo baseline", f"{base:.5f} m",
            f"{_BASELINE_LO:.2f}-{_BASELINE_HI:.2f} m", FAIL,
            "implausible stereo baseline" + hint))

    # 11. Baseline dominantly along camera X (side-by-side rig). |t_x| should be
    #     the largest component; otherwise the rig axis assumption is off.
    ax = np.abs(t)
    x_dominant = ax[0] >= ax[1] and ax[0] >= ax[2]
    axis_split = f"|t|=({ax[0]*100:.2f},{ax[1]*100:.2f},{ax[2]*100:.2f}) cm"
    out.append(CheckResult(
        "baseline along X", axis_split, "|t_x| largest",
        PASS if x_dominant else WARN,
        "" if x_dominant else "baseline is not dominantly along camera-X -- not a side-by-side rig?"))

    return out


# --------------------------------------------------------------------------- #
# IMU<->camera extrinsic + IMU noise checks.
# --------------------------------------------------------------------------- #
def check_imu_extrinsic(calib: StereoCalib) -> list[CheckResult]:
    """Check 12: IMU<->cam rotation in SO(3) + small lever-arm, or INFO if absent."""
    out: list[CheckResult] = []
    if not calib.has_imu_extrinsics:
        out.append(CheckResult(
            "IMU extrinsics", "none", "optional", INFO,
            "no IMU extrinsics recorded -- gyro prior disabled (valid for vision-only)"))
        return out

    T = np.asarray(calib.T_imu_left, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    if not _all_finite(R):
        out.append(CheckResult("IMU->cam rotation", "non-finite", "in SO(3)",
                               FAIL, "T_imu_left rotation has NaN/inf"))
    else:
        ortho = float(np.linalg.norm(R @ R.T - np.eye(3)))
        det = float(np.linalg.det(R))
        so3_ok = ortho < _SO3_TOL and abs(det - 1.0) < _DET_TOL
        out.append(CheckResult(
            "IMU->cam rotation", f"||RRt-I||={ortho:.2e} det={det:.4f}", "in SO(3)",
            PASS if so3_ok else FAIL,
            "" if so3_ok else "IMU->cam rotation is not a proper rotation -- corrupt extrinsics"))

    # Lever-arm: IMU and camera are physically a few cm apart at most on these rigs.
    lever = float(np.linalg.norm(t))
    if lever < _IMU_LEVER_PASS:
        out.append(CheckResult("IMU->cam lever-arm", f"{lever * 100:.2f} cm",
                               f"<{_IMU_LEVER_PASS * 100:.0f} cm", PASS))
    else:
        out.append(CheckResult("IMU->cam lever-arm", f"{lever * 100:.2f} cm",
                               f"<{_IMU_LEVER_PASS * 100:.0f} cm", WARN,
                               "IMU->cam lever-arm >10cm -- large for a handheld rig, verify units"))
    return out


def check_imu_noise(calib: StereoCalib) -> CheckResult:
    """Check 13: IMU noise densities finite + positive, or INFO if absent."""
    noise = calib.imu_noise
    if not isinstance(noise, dict) or not noise:
        return CheckResult("IMU noise model", "none", "optional", INFO,
                           "no IMU noise model recorded (gyro/accel densities unset)")
    # Collect every numeric density value; all must be finite and > 0.
    vals = [v for v in noise.values() if isinstance(v, (int, float))]
    bad = [k for k, v in noise.items()
           if isinstance(v, (int, float)) and (not np.isfinite(v) or v <= 0.0)]
    if not vals:
        return CheckResult("IMU noise model", "no numeric densities", "finite, >0",
                           WARN, "imu_noise present but holds no numeric density values")
    ok = not bad
    return CheckResult(
        "IMU noise model", f"{len(vals)} densities", "finite, >0",
        PASS if ok else WARN,
        "" if ok else f"non-positive/non-finite IMU noise density: {bad}")


# --------------------------------------------------------------------------- #
# Consistency with recorded data (session-only: needs actual frames).
# --------------------------------------------------------------------------- #
def check_recorded_consistency(reader: SessionReader) -> list[CheckResult]:
    """Checks 14-15: calib size == recorded frame shape; recorded depth in band."""
    out: list[CheckResult] = []
    try:
        frame = reader.load_frame(0, load_right=False)
    except (IndexError, FileNotFoundError, ValueError) as exc:
        out.append(CheckResult("recorded frame shape", "no frame", "match calib",
                               INFO, f"could not load frame 0 ({exc}) -- skipped"))
        return out

    # 14. calib width/height must equal the actual recorded frame shape.
    fh, fw = frame.gray_left.shape[:2]
    cw, ch = reader.calib.left.width, reader.calib.left.height
    match = (fw == cw) and (fh == ch)
    out.append(CheckResult(
        "calib vs recorded shape", f"calib {cw}x{ch}  frame {fw}x{fh}", "equal",
        PASS if match else FAIL,
        "" if match else "calib resolution != recorded frame shape -- wrong calib for this recording"))

    # 15. Median recorded depth in a plausible indoor band (catches a depth scale/unit
    #     error). The chip's StereoDepth needs a few frames to warm up: the first
    #     non-empty frames can carry a HANDFUL of pixels all pinned at the far disparity
    #     rail (~21 m on OAK-D), whose median is a warm-up artefact, not the scene. So we
    #     skip sparse frames and use the first with MEANINGFUL coverage; the median then
    #     reflects real geometry. Fall back to any valid frame if none is dense enough.
    valid = np.empty(0, dtype=np.float64)
    fallback = np.empty(0, dtype=np.float64)
    n_probe = min(len(reader), 30)
    for i in range(n_probe):
        try:
            d = np.asarray(reader.load_frame(i, load_right=False).depth_m,
                           dtype=np.float64)
        except (IndexError, FileNotFoundError, ValueError):
            continue
        v = d[(d > 0.0) & np.isfinite(d)]
        if v.size == 0:
            continue
        if fallback.size == 0:
            fallback = v
        if v.size >= _DEPTH_MIN_VALID_FRAC * d.size:
            valid = v
            break
    if valid.size == 0:
        valid = fallback  # no dense frame found -- use the sparse one we did see
    if valid.size == 0:
        out.append(CheckResult("recorded depth median", "no valid depth", "0.2-15 m",
                               INFO, f"no valid depth in first {n_probe} frames -- depth check skipped"))
    else:
        med = float(np.median(valid))
        if _DEPTH_LO <= med <= _DEPTH_HI:
            out.append(CheckResult("recorded depth median", f"{med:.2f} m",
                                   f"{_DEPTH_LO:.1f}-{_DEPTH_HI:.0f} m", PASS))
        else:
            out.append(CheckResult("recorded depth median", f"{med:.2f} m",
                                   f"{_DEPTH_LO:.1f}-{_DEPTH_HI:.0f} m", WARN,
                                   "median recorded depth outside indoor band -- depth scale/unit error?"))
    return out


# --------------------------------------------------------------------------- #
# Orchestration: run every check on a StereoCalib (+ optional reader).
# --------------------------------------------------------------------------- #
def run_checks(calib: StereoCalib,
               reader: SessionReader | None = None) -> list[CheckResult]:
    """Run the full check suite; ``reader`` enables the recorded-data checks (14-15)."""
    results: list[CheckResult] = []
    results += check_intrinsics(calib.left, "L")
    results += check_intrinsics(calib.right, "R")
    results.append(check_sizes_match(calib))
    results += check_stereo_extrinsic(calib)
    results += check_imu_extrinsic(calib)
    results.append(check_imu_noise(calib))
    if reader is not None:
        results += check_recorded_consistency(reader)
    return results


# --------------------------------------------------------------------------- #
# Reporting (aligned plaintext table, mirroring loose_vs_tight_bench's style).
# --------------------------------------------------------------------------- #
def _counts(results: list[CheckResult]) -> dict[str, int]:
    c = {PASS: 0, WARN: 0, FAIL: 0, INFO: 0}
    for r in results:
        c[r.status] += 1
    return c


def render_report(results: list[CheckResult], title: str) -> str:
    """Render the aligned CHECK | MEASURED | EXPECTED | STATUS table + summary."""
    name_w = max([len("CHECK")] + [len(r.name) for r in results])
    meas_w = max([len("MEASURED")] + [len(r.measured) for r in results])
    exp_w = max([len("EXPECTED")] + [len(r.expected) for r in results])

    lines: list[str] = []
    lines.append("=" * (name_w + meas_w + exp_w + 24))
    lines.append(f"calib_check -- {title}")
    lines.append("=" * (name_w + meas_w + exp_w + 24))
    hdr = (f"{'CHECK':<{name_w}}  {'MEASURED':<{meas_w}}  "
           f"{'EXPECTED':<{exp_w}}  {'STATUS':<6}")
    lines.append(hdr)
    lines.append("-" * len(hdr))
    for r in results:
        lines.append(f"{r.name:<{name_w}}  {r.measured:<{meas_w}}  "
                     f"{r.expected:<{exp_w}}  {r.status:<6}")
        # One-line human explanation for every non-PASS row.
        if r.status != PASS and r.note:
            lines.append(f"{'':<{name_w}}  -> {r.note}")
    lines.append("-" * len(hdr))
    c = _counts(results)
    lines.append(f"summary: {c[PASS]} pass / {c[WARN]} warn / {c[FAIL]} fail "
                 f"/ {c[INFO]} info")
    return "\n".join(lines)


def exit_code(results: list[CheckResult], strict: bool) -> int:
    """0 when no FAIL (WARN allowed); nonzero on any FAIL, or any WARN under --strict."""
    c = _counts(results)
    if c[FAIL] > 0:
        return 1
    if strict and c[WARN] > 0:
        return 2
    return 0


# --------------------------------------------------------------------------- #
# CLI.
# --------------------------------------------------------------------------- #
def _load(session: str | None, calib_path: str | None
          ) -> tuple[StereoCalib, SessionReader | None, str]:
    """Resolve --session / --calib into a (calib, reader, title) triple."""
    if session:
        reader = SessionReader(Path(session))
        return reader.calib, reader, f"session {session}"
    # --calib: parse a bare calib.json through the SAME loader (no reader -> skip 14-15).
    import json
    data = json.loads(Path(calib_path).read_text())
    return StereoCalib.from_json(data), None, f"calib {calib_path}"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--session", help="recorded session dir (validates its calib.json "
                                        "+ checks recorded-data consistency)")
    src.add_argument("--calib", help="bare calib.json path (no recorded-data checks)")
    ap.add_argument("--strict", action="store_true",
                    help="treat WARN as failure for the exit code (strict CI gate)")
    args = ap.parse_args(argv)

    calib, reader, title = _load(args.session, args.calib)
    results = run_checks(calib, reader)
    print(render_report(results, title))
    return exit_code(results, args.strict)


if __name__ == "__main__":
    raise SystemExit(main())
