#!/usr/bin/env python3
"""LITMUS: the BASELINE pipeline imports + runs with OpenCV (cv2) UNINSTALLED.

`baseline/` is the INDEPENDENT DepthAI/Basalt reference pipeline (it imports NO
``ours``/``sky``, so it is NOT on the gap=0 oracle). Its Pi install
(``requirements-baseline.txt``) deliberately omits the heavy ``opencv-python``
wheel. This harness PROVES baseline is cv2-free -- WITHOUT uninstalling cv2 from
the dev venv -- by injecting a ``sitecustomize.py`` (on ``PYTHONPATH``) that
makes ``import cv2`` raise ``ImportError`` in every spawned interpreter.
``sitecustomize`` auto-runs at interpreter startup (before any baseline code), so
the block is total.

The real device dependency (``depthai`` / the OAK-D's on-chip Basalt blobs) is
NOT present on the mac dev box, so this exercises the cv2-free paths that ARE
runnable here -- two child interpreters, both with cv2 blocked:

* **import probe** -- imports every baseline subpackage that has no native-device
  / no-Qt-display requirement at import time
  (``baseline.sources``, ``baseline.capture``, ``baseline.frames``,
  ``baseline.pose``, ``baseline.ui.theme``, and crucially
  ``baseline.tools.viz_session`` -- the only module that used cv2). Asserts NO
  ``import cv2`` anywhere in that surface.

* **runtime slice** -- runs two cv2-free runtime paths end to end:
  (1) ``FakePoseSource`` produces real ``Pose`` samples (the pure-Python pose
  generator the UI bring-up uses); (2) the ``viz_session`` data path on a
  recorded session -- ``pngio.imread_gray`` decodes the stereo PNGs and the
  NumPy ``turbo_rgb`` LUT colourises the depth frame -- proving the cv2->NumPy
  swap produces a sane RGB depth image with cv2 absent.

Run::

    python -m verification.cv2_absent_baseline_litmus
    python -m verification.cv2_absent_baseline_litmus --session sessions/fast_push_15s
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

# The sitecustomize.py body that blocks cv2 in every child interpreter. Placed on
# PYTHONPATH so CPython auto-imports it at startup, before any baseline module.
_BLOCKER_SRC = '''\
"""Auto-loaded cv2 blocker (simulate opencv-python UNINSTALLED on the Pi)."""
import sys, importlib.abc


class _BlockCv2(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name == "cv2" or name.startswith("cv2."):
            raise ImportError(
                "cv2 blocked by cv2_absent_baseline_litmus (simulated absent): "
                + name)
        return None


for _m in [m for m in list(sys.modules) if m == "cv2" or m.startswith("cv2.")]:
    del sys.modules[_m]
sys.meta_path.insert(0, _BlockCv2())
'''

# Child #1: import the whole cv2-relevant baseline surface (must not import cv2).
_IMPORT_PROBE = '''\
import cv2_guard_assert  # raises if cv2 is importable
import baseline
import baseline.frames
import baseline.pose
import baseline.sources               # FakePoseSource + lazy depthai sources
import baseline.capture               # SessionRecorder + pngio
import baseline.capture.pngio
import baseline.ui.theme              # theme has no Qt import at module load
import baseline.tools.viz_session     # the ONLY module that used cv2
print("IMPORT_PROBE_OK")
'''

# A tiny helper module the probe imports first to assert cv2 is truly blocked.
_GUARD_SRC = '''\
try:
    import cv2  # noqa: F401
except ImportError:
    pass
else:
    raise SystemExit("FAIL: cv2 was importable in the baseline litmus child")
'''

# Child #2: run two cv2-free runtime slices (FakePoseSource + viz_session data).
_RUNTIME_SLICE = '''\
import cv2_guard_assert  # raises if cv2 is importable
import json, time
from pathlib import Path
import numpy as np

# --- slice 1: FakePoseSource emits real poses (pure-Python generator) ---
from baseline.sources.fake import FakePoseSource
poses = []
src = FakePoseSource(rate_hz=200.0)
src.start(lambda p: poses.append(p))
time.sleep(0.4)
src.stop()
assert len(poses) >= 10, f"FakePoseSource produced too few poses: {len(poses)}"
p = poses[-1]
assert p.pos_ned.shape == (3,) and np.isfinite(p.pos_ned).all(), "bad pose pos"
assert abs(np.linalg.norm(p.quat_wxyz) - 1.0) < 1e-6, "pose quat not unit"
print(f"FAKE_POSE_OK n={len(poses)}")

# --- slice 2: viz_session data path (PNG decode + NumPy Turbo depth) ---
from baseline.capture.pngio import imread_gray
from baseline.tools.viz_session import turbo_rgb

session = Path(SESSION)
frames = []
with (session / "input" / "frames.jsonl").open() as f:
    for line in f:
        line = line.strip()
        if line:
            frames.append(json.loads(line))
assert frames, f"no frames in {session}"
rec = frames[0]
base = session / "input"
h, w = int(rec["height"]), int(rec["width"])

left = imread_gray(base / rec["left_path"])
right = imread_gray(base / rec["right_path"])
assert left.shape == (h, w) and left.dtype == np.uint8, f"bad left {left.shape}"
assert right.shape == (h, w), f"bad right {right.shape}"

depth = np.fromfile(base / rec["depth_path"], dtype="<u2").reshape(h, w)
valid = depth > 0
vmax = float(depth[valid].max())
norm = np.zeros_like(depth, dtype=np.uint8)
norm[valid] = np.clip(depth[valid].astype(np.float32) / vmax * 255.0,
                      0, 255).astype(np.uint8)
rgb = turbo_rgb(norm)
rgb[~valid] = 0
assert rgb.shape == (h, w, 3) and rgb.dtype == np.uint8, f"bad rgb {rgb.shape}"
assert rgb[valid].any(), "depth colourisation is all black on valid pixels"
n_distinct = len(np.unique(rgb[valid].reshape(-1, 3), axis=0))
assert n_distinct >= 8, f"depth colour ramp too flat: {n_distinct} colours"
print(f"VIZ_DATA_OK left={left.shape} depth_valid={int(valid.sum())}/{w*h} "
      f"rgb={rgb.shape} colours={n_distinct}")
print("RUNTIME_SLICE_OK")
'''


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/fast_push_15s",
                    help="a recorded baseline session (input/frames.jsonl + "
                         "input/img/*.png + *.raw16) for the viz_session slice")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    session = (repo / args.session) if not Path(args.session).is_absolute() \
        else Path(args.session)
    if not (session / "input" / "frames.jsonl").exists():
        print(f"  [FAIL] no recorded session at {session} "
              "(need input/frames.jsonl)", file=sys.stderr)
        return 1

    py = sys.executable

    with tempfile.TemporaryDirectory(prefix="cv2block_baseline_") as blockdir:
        (Path(blockdir) / "sitecustomize.py").write_text(_BLOCKER_SRC)
        (Path(blockdir) / "cv2_guard_assert.py").write_text(_GUARD_SRC)
        # Prepend the blocker dir AND the repo to PYTHONPATH so every child both
        # blocks cv2 (sitecustomize) and can import the baseline package.
        env = dict(os.environ)
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = os.pathsep.join(
            [blockdir, str(repo)] + ([existing] if existing else []))
        # Force Qt offscreen in case anything pulls a platform plugin.
        env.setdefault("QT_QPA_PLATFORM", "offscreen")

        print("cv2_absent_baseline_litmus (BASELINE imports+runs with cv2 BLOCKED)")
        print(f"  session={args.session}")
        print(f"  cv2 blocker: {Path(blockdir) / 'sitecustomize.py'}")

        # Sanity: the blocker really blocks cv2 in a child interpreter.
        probe = subprocess.run(
            [py, "-c", "import cv2"], env=env, capture_output=True, text=True)
        _check(probe.returncode != 0 and "blocked" in probe.stderr,
               "cv2 is BLOCKED in spawned interpreters "
               "(import cv2 -> ImportError)")

        # --- child #1: import the whole cv2-relevant baseline surface ---
        imp = subprocess.run([py, "-c", _IMPORT_PROBE], env=env,
                             capture_output=True, text=True)
        if imp.returncode != 0:
            print(imp.stderr, file=sys.stderr)
        _check(imp.returncode == 0 and "IMPORT_PROBE_OK" in imp.stdout,
               "baseline imports clean with cv2 absent "
               "(sources/capture/ui.theme/tools.viz_session)")

        # --- child #2: run the cv2-free runtime slices ---
        slice_src = f"SESSION = {str(session)!r}\n" + _RUNTIME_SLICE
        run = subprocess.run([py, "-c", slice_src], env=env,
                             capture_output=True, text=True)
        if run.returncode != 0:
            print(run.stderr, file=sys.stderr)
        for line in run.stdout.splitlines():
            if line.endswith("_OK") or "_OK " in line:
                print(f"    {line}")
        _check("FAKE_POSE_OK" in run.stdout,
               "FakePoseSource emits real poses (cv2 absent)")
        _check("VIZ_DATA_OK" in run.stdout,
               "viz_session PNG decode + NumPy Turbo depth -> sane RGB "
               "(cv2 absent)")
        _check(run.returncode == 0 and "RUNTIME_SLICE_OK" in run.stdout,
               "baseline runtime slice rc=0 with cv2 absent")

    print("\nLITMUS PASSED: baseline imports + a runtime slice run with cv2 "
          "ABSENT (rc=0)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
