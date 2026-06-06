"""Pre-compile the Numba JIT kernels (KLT + SGM) off the critical path.

The first call to each ``@njit`` kernel pays a one-time LLVM compile (KLT inner
loop + the SGM census/cost/aggregation/WTA kernels, ~1-3 s in total). They are
declared ``cache=True`` so that cost is paid only on a COLD cache (the first run
after a code change clears it); but on that first run it lands on the very first
live frame and stalls the viewer for seconds while the OAK-D is already
streaming.

:func:`warmup_jit` triggers those compiles with tiny synthetic inputs. Kicked on
a background thread at device-open time, it overlaps the LLVM compile with the
OAK-D boot + the startup IMU still-window (~1-2 s of dead time anyway), so by the
time frame one arrives the kernels are already machine code. The numba
dispatchers are module-level singletons, so compiling them in the warmup thread
makes the SAME functions the frame path calls instant. With a warm disk cache it
is a sub-millisecond no-op. It NEVER changes results -- it only compiles; a
failure (e.g. numba absent) is swallowed so it can never break a run.
"""
from __future__ import annotations

import numpy as np

from .flow.runtime import NUMBA_PARALLEL_LOCK


def warmup_jit(klt_cfg=None, sgm_cfg=None) -> bool:
    """Compile the KLT + SGM numba kernels via tiny dummy calls.

    ``klt_cfg`` / ``sgm_cfg`` should be the configs the live path will use so the
    compiled type signatures match (numba specialises per argument *type*, which
    these tiny calls reproduce; the config *values* only need to exercise the
    same code paths, e.g. SGM downscale). Both default to the library defaults.
    Returns ``True`` if the kernels compiled, ``False`` if numba is unavailable
    or anything went wrong (always safe -- the real path then just compiles on
    frame one exactly as before).
    """
    try:
        from .frontend.klt_numba import HAVE_NUMBA
        if not HAVE_NUMBA:
            return False
        from .frontend.frontend import FrontendConfig
        from .frontend.klt import calc_optical_flow_pyr_lk
        from .stereo.stereo import SGMConfig, sgm_disparity

        klt = klt_cfg or FrontendConfig()
        sgm = sgm_cfg or SGMConfig()

        # A small textured pair so the KLT solver and SGM matcher actually run
        # their inner loops (a flat image would short-circuit before the kernel).
        rng = np.random.default_rng(0)
        left = rng.integers(0, 255, (64, 96)).astype(np.float32)
        right = np.roll(left, -2, axis=1).astype(np.float32)
        pts = np.array([[30.0, 30.0], [50.0, 25.0]], dtype=np.float32)

        with NUMBA_PARALLEL_LOCK:
            calc_optical_flow_pyr_lk(
                left, right, pts,
                win_size=int(klt.win_size), max_level=int(klt.max_level))
            sgm_disparity(left, right, sgm)
        return True
    except Exception:                                    # noqa: BLE001
        return False
