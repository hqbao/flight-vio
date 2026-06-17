"""``ba.modules`` -- the windowed-BA backend over the keyframe stream, procedural.

The back-end half of the old in-VIO pipeline, extracted into the ``ba`` process.
``ba`` is a pure CONSUMER of the ``keyframe`` stream (``emit_keyframe`` stays in
``vio`` on its odometry thread); these modules ingest each keyframe and run the
sliding-window solve.

Files
-----
* :mod:`backend`    -- :func:`~ba.modules.backend.run_ba`, the per-keyframe submit +
                       refined-pose extraction (the SAME frozen solve as in-VIO).
* :mod:`publishers` -- the two backend taps:
                       :func:`~ba.modules.publishers.publish_refined` (``pose.refined``)
                       and :func:`~ba.modules.publishers.publish_ba_window`
                       (``ba.window``, opt-in).
* :mod:`pipeline`   -- the orchestration: :func:`~ba.modules.pipeline.process_kf`
                       per keyframe + the worker thread
                       :class:`~ba.modules.pipeline.BackendWorker` (legacy alias
                       :data:`~ba.modules.pipeline.BackendModule`). THE ENTRY POINT.
"""
from .pipeline import BackendModule, BackendWorker

__all__ = ["BackendModule", "BackendWorker"]
