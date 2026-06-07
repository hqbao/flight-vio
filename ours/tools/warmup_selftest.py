"""Self-test for the JIT warmup (``ours.lib.warmup``).

Asserts the warmup actually compiles the KLT + SGM numba kernels (their numba
dispatchers gain a signature), is idempotent, never raises, and -- the point of
it -- that a COLD-cache warmup makes the subsequent first KLT/SGM call cheap. The
cold-cache timing runs in a child process with an empty ``NUMBA_CACHE_DIR`` so it
is hermetic and does not disturb the repo's warm cache.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def test_compiles_signatures() -> None:
    print("warmup compiles the KLT + SGM kernels")
    from ours.lib.frontend.klt_numba import HAVE_NUMBA, _track_level
    from ours.lib.stereo.stereo import _census, _cost_volume
    from ours.lib.misc.warmup import warmup_jit

    if not HAVE_NUMBA:
        _check(warmup_jit() is False, "numba absent -> warmup is a safe no-op")
        return

    ok = warmup_jit()
    _check(ok is True, "warmup_jit reported success")
    _check(len(_track_level.signatures) > 0, "KLT _track_level got compiled")
    _check(len(_census.signatures) > 0, "SGM _census got compiled")
    _check(len(_cost_volume.signatures) > 0, "SGM _cost_volume got compiled")
    # Idempotent: a second call must not raise and stays compiled.
    _check(warmup_jit() is True, "warmup_jit is idempotent")


def test_never_raises_on_bad_cfg() -> None:
    print("warmup never raises (swallows failures)")
    from ours.lib.misc.warmup import warmup_jit

    class _Bad:
        win_size = "not-an-int"   # forces an internal failure
        max_level = 2
    _check(warmup_jit(klt_cfg=_Bad()) is False,
           "a broken config returns False instead of raising")


_COLD_CHILD = r"""
import os, sys, time
sys.path.insert(0, {root!r})
from ours.lib.frontend.frontend import FrontendConfig
from ours.lib.stereo.stereo import SGMConfig, sgm_disparity
from ours.lib.frontend.klt import calc_optical_flow_pyr_lk
from ours.lib.misc.warmup import warmup_jit
import numpy as np
rng = np.random.default_rng(1)
L = rng.integers(0, 255, (64, 96)).astype(np.float32)
R = np.roll(L, -2, axis=1).astype(np.float32)
pts = np.array([[30., 30.], [50., 25.]], np.float32)
mode = sys.argv[1]
if mode == "warm":
    warmup_jit()
t0 = time.perf_counter()
calc_optical_flow_pyr_lk(L, R, pts, win_size=21, max_level=3)
sgm_disparity(L, R, SGMConfig())
print(time.perf_counter() - t0)
"""


def _cold_first_call_s(mode: str, cache_dir: str) -> float:
    env = dict(os.environ)
    env["NUMBA_CACHE_DIR"] = cache_dir          # hermetic, empty -> cold compile
    src = _COLD_CHILD.format(root=ROOT)
    out = subprocess.run([sys.executable, "-c", src, mode],
                         capture_output=True, text=True, env=env, cwd=ROOT)
    if out.returncode != 0:
        print(out.stderr[-2000:])
        raise SystemExit(1)
    return float(out.stdout.strip().splitlines()[-1])


def test_cold_cache_warmup_helps() -> None:
    print("cold-cache warmup makes the first frame cheap")
    from ours.lib.frontend.klt_numba import HAVE_NUMBA
    if not HAVE_NUMBA:
        _check(True, "numba absent -> skip timing")
        return
    with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
        plain = _cold_first_call_s("plain", d1)   # frame one pays the JIT
        warm = _cold_first_call_s("warm", d2)     # warmup already paid it
    print(f"     cold first-call: plain={plain*1e3:.0f} ms  after-warmup={warm*1e3:.0f} ms")
    # After warmup the first real call is just execution (no compile) -> a large
    # margin. Require it to be at least 3x cheaper to prove the compile moved.
    _check(warm * 3.0 < plain,
           f"warmed first call >=3x cheaper ({plain/max(warm,1e-9):.1f}x)")


def main() -> int:
    test_compiles_signatures()
    test_never_raises_on_bad_cfg()
    test_cold_cache_warmup_helps()
    print("\nRESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
