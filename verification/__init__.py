"""``verification`` -- the repo-root byte-parity harness for the 5-project split.

This package PROVES the split (imu_camera + vio + slam + ui + launcher) preserved
the pre-split ``ours`` behaviour byte-for-byte. It only IMPORTS the four math
projects + the vendored ``comms``; it NEVER modifies ``ours/`` (the reference
oracle) or any project directory.

The live pipeline is now separate OS processes over IPC (nondeterministic timing)
which cannot give byte-parity; so the parity ORACLE is an IN-PROCESS harness
(:mod:`verification.oracle_replay`) that imports each project's verbatim-ported
math directly and reproduces ``ours/tools/vio_run.py``'s deterministic scoring
loop exactly, with NO IPCPubSub. Because each component was already proved
byte-identical per-module (vio_ba_selftest, loop_closure_selftest, stereo,
imucam_sync), the end-to-end oracle reproduces ``ours``' ATE/Sim3 scores exactly.

Modules:

* :mod:`~verification.oracle_replay` -- the in-process replay oracle: NEW-project
  math driven through the EXACT ``vio_run`` ATE/Sim3 scoring (umeyama + ate).
* :mod:`~verification.vio_oracle_runner` -- CLI mirroring ``ours/tools/vio_run.py``.
* :mod:`~verification.oracle_replay_selftest` -- asserts the new oracle == the
  stored pre-split baseline (and the live OLD oracle) within byte-parity tol.
* :mod:`~verification.ipc_comms_selftest` -- cross-project ``comms`` byte-parity
  (dir diff + 5-copy codec digest + ring + bridge round-trip).
"""
