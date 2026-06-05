"""depth flow: turn a raw stereo pair into a metric depth map.

Subscribes ``frame.raw`` and publishes ``frame.depth``. The depth is ALWAYS our
own from-scratch SGM matcher (:class:`~ours.lib.stereo.stereo.SGMStereoMatcher`)
run on the rectified left + raw right frames -- the same portable depth the live
pipeline and ``vio_run --depth ours`` use. The matcher rectifies the right frame
internally.
"""
from .depth_flow import DepthFlow

__all__ = ["DepthFlow"]
