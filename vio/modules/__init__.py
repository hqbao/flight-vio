"""``vio.modules`` -- the VIO pipeline (odometry + windowed BA), procedural.

Data flow, one frame at a time::

    frame --> frontend --> imu_prior --> backend --> publishers
                  (the worker in pipeline.py orchestrates the chain)

The pipeline is plain procedural Python (the old class-heavy Step/Module reactive
framework was dissolved): each stage is a module of single-purpose FUNCTIONS that
take their dependencies explicitly and hand one frame's state to the next via the
small carrier records in :mod:`carriers`.

Files (read in pipeline order)
------------------------------
* :mod:`carriers`      -- the per-frame dataclass records (Tracked / Primed / Step)
                          threaded between stages; never go on the bus.
* :mod:`frontend`      -- sparse visual VO: KLT track -> RGB-D PnP (+ gyro fusion).
* :mod:`imu_prior`     -- IMU prior + gravity chain: preintegrate the per-frame
                          prior, one-shot gravity align, IMU<->vision join, at-rest
                          tilt correction.
* :mod:`backend`       -- keyframe EMISSION (``emit_keyframe``, on the odometry
                          thread). The windowed BA itself moved to the ``ba`` process.
* :mod:`publishers`    -- the thin "emit one result on a bus topic" steps.
* :mod:`pipeline`      -- the orchestration: the odometry worker thread
                          (:class:`~vio.modules.pipeline.OdometryWorker` joins the
                          IMU + depth edges and runs the per-frame chain). THE ENTRY
                          POINT.
* :mod:`direct_odometry` -- the ``--direct`` ALT odometry (dense direct RGB-D VO).
* :mod:`propagate_imu` -- the live per-frame IMU dead-reckoning (nav state).
* :mod:`loop_inbox`    -- the SLAM loop-closure + ba.state bias feedback inboxes.

``OdometryModule`` is kept as a public alias for the worker (``vio.main`` + the
vio/verification selftests import it). The windowed-BA worker now lives in the
``ba`` project (``ba.modules``).
"""
from .pipeline import OdometryModule, OdometryWorker

__all__ = ["OdometryModule", "OdometryWorker"]
