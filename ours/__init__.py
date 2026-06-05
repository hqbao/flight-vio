"""Our from-scratch RGB-D visual-inertial pipeline (library-free).

This root package holds everything we implement ourselves while replacing the
DepthAI BasaltVIO + RTABMap black boxes one module at a time:

  * ``ours.vio``                 — the algorithm library (KLT, corners, PnP, IMU
                                   preintegration, stereo SGM, windowed BA, pose
                                   graph + loop closure, the synced-input bundle)
  * ``ours.depthai_ours_vio``    — the live OAK-D source driving ``ours.vio``
  * ``ours.tools``               — offline scoring, self-tests and inspectors
  * ``ours.{pose,frames,pngio}`` — our own pose types, frames math, PNG codec
  * ``ours.sources`` / ``ours.ui`` — our own pose-source base + Qt 3D viewer

This package is fully self-contained: it imports nothing from ``oakd`` (the
baseline's core) so the two pipelines share no code. We accept the small
duplication (pose/frames/pngio/sources/ui) in exchange for a clean split. The
library baseline we are replacing lives in ``baseline`` (with ``oakd`` as its
core).
"""
