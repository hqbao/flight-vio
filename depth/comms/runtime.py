"""Process-wide runtime guards for the threaded module pipeline.

Numba's default ``workqueue`` threading layer (the only one available when
neither Intel TBB nor OpenMP is installed) is **not** safe to enter from two
Python threads at once. In the module graph the only ``parallel=True`` regions
are the SGM depth matcher (depth module) and the KLT tracker (odometry module),
which run on different threads. :data:`NUMBA_PARALLEL_LOCK` serializes just those
two sections so they never launch a parallel region concurrently; every other
module (pure-NumPy back-end / SLAM) keeps running freely.

If a threadsafe layer is available (``pip install tbb`` / ``intel-openmp`` and
``NUMBA_THREADING_LAYER=tbb``) this lock is uncontended and can be ignored.
"""
from __future__ import annotations

import threading

#: Held around numba ``parallel=True`` calls made from module threads.
NUMBA_PARALLEL_LOCK = threading.Lock()
