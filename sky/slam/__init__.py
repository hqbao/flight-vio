"""``sky.slam`` -- the shared loop-closure SLAM (ORB + detector + pose-graph + map).

This is the ONE canonical home for the library-free loop-closure SLAM the
``slam`` process runs. It used to live in ``slam/mathlib/loop/``; ``slam`` was
the only consumer, so the four modules were RELOCATED here (R4) to make the SLAM
process a thin IPC shell (frame ingest + ``loop.correction`` publish) that just
calls into :mod:`sky.slam`.

* :mod:`sky.slam.orb` -- :class:`~sky.slam.orb.ORB`, the from-scratch ORB
  detector/descriptor + ratio-test mutual matcher + fundamental-matrix RANSAC the
  appearance/geometric loop check needs (pure NumPy, no cv2 runtime dependency).
* :mod:`sky.slam.loopclosure` -- :class:`~sky.slam.loopclosure.LoopDetector`, the
  three-stage loop detector (appearance -> epipolar -> metric PnP via
  :func:`sky.front.pnp.solve_pnp_ransac`) + :class:`~sky.slam.loopclosure.LoopConfig`
  / :class:`~sky.slam.loopclosure.LoopMatchCapture`.
* :mod:`sky.slam.posegraph` -- :class:`~sky.slam.posegraph.PoseGraph`, the SE(3)
  pose-graph optimiser (Gauss-Newton over relative-pose edges) that cancels the
  accumulated drift once a loop is confirmed (uses :mod:`sky.math` Lie helpers).
* :mod:`sky.slam.slam` -- :class:`~sky.slam.slam.SlamMap` / :class:`~sky.slam.slam.SlamConfig`,
  the persistent-keyframe orchestrator: it ingests keyframes, runs the loop
  detector, triggers the pose-graph optimisation and emits the loop correction.
  It is process-free -- the caller hands it data as arguments, so it stays a leaf.

Every module imports only ``numpy`` + other :mod:`sky.*` (``sky.math`` Lie
helpers, ``sky.front.pnp``) -- no process / comms / io module -- so the package
stays a leaf and movable (maps onto the C ``libskyslam`` / loop layer in
``docs/C_PORT_PLAN.md``).
"""
