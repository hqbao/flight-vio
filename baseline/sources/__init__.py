"""Pose source implementations for the DepthAI/Basalt baseline.

Re-exports the public source API so callers can do ``from baseline.sources
import OakBasaltVioSource`` without knowing the module layout. The Basalt
sources import ``depthai`` lazily (only inside their ``_run`` thread body), so
importing this package does NOT pull DepthAI — ``--source fake`` and the tools'
``--help`` work on machines without a device or the depthai wheel.
"""
from ..pose import Pose
from .base import PoseSource
from .fake import FakePoseSource
from .basalt_vio import OakBasaltVioSource
from .basalt_slam import OakBasaltSlamSource

__all__ = [
    "Pose",
    "PoseSource",
    "FakePoseSource",
    "OakBasaltVioSource",
    "OakBasaltSlamSource",
]
