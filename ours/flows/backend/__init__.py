"""backend flow: sliding-window bundle adjustment over keyframes.

Subscribes ``keyframe`` and publishes ``pose.refined``. Mirrors the live BA
worker thread: each keyframe's track snapshot (``T_cw``, ids, pixels, depth) is
fed to a :class:`~ours.lib.backend.windowed.WindowedBAMap`; when the window
optimises, the refined latest pose is published.
"""
from .backend_flow import BackendFlow

__all__ = ["BackendFlow"]
