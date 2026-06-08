"""``launcher`` -- the top-level 4-process pipeline launcher (proc4).

Boots the four split projects -- :mod:`imu_camera`, :mod:`vio`, :mod:`slam`,
:mod:`ui` -- as one pipeline: ``imu_camera`` (capture) and ``vio`` / ``slam`` run
in the background; ``ui`` runs in the FOREGROUND so the Qt event loop owns the
GUI focus and a clean window-close / Ctrl-C / Restart tears everything down.

The launcher's only job is process lifecycle management (spawn order, endpoint
naming, orphan SHM / socket reclaim, the Restart respawn loop, and SIGTERM
forwarding). It is a behaviour-for-behaviour port of the pre-split
``ours.proc.launcher`` retargeted onto the new ``<project>.main`` entrypoints.

:mod:`launcher.comms` is a byte-identical vendored copy of
:mod:`imu_camera.comms` (CI diffs the copies). The launcher only needs
``SharedArrayRing.cleanup_stale`` + ``ring_registry`` for orphan reclaim, but the
full copy is vendored for consistency with the other projects + the ``diff -r``
gate. See ``docs/PROC4_ARCHITECTURE.md``.
"""
