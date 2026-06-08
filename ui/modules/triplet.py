"""ui triplet module: forward the synced (image, depth, IMU) unit to a callback.

The sink that drives the image|depth|IMU triplet window
(:class:`~ui.qt.synced_window.SyncedViewWindow`). It subscribes the two topics
the ``imu_cam`` module already publishes -- ``frame.depth`` (rectified-left image
+ metric depth) and ``imucam.sample`` (the CALIBRATED IMU rows for the frame
interval, exactly what the VIO consumes) -- and joins them by ``seq`` into one
unit handed to an ``on_triplet`` callback, the same subscribe pattern
:class:`~ui.modules.render.UiRenderModule` uses for ``pose.odom``.

No parallel pipeline, no second device: the triplet shown is literally the live
acquisition front-end's output. The join is exact because, per frame, the
``imu_cam`` chain publishes ``imucam.sample`` then ``frame.depth`` for the same
``seq`` in order (see :mod:`ui.modules.stash_imucam` /
:mod:`ui.modules.render_triplet`).
"""
from __future__ import annotations

from typing import Callable

from ui.comms import LocalPubSub, Module, topics
from .render_triplet import RenderTriplet
from .stash_imucam import StashImuCam

# on_triplet(seq, ts_ns, gray_left, depth_m, gyro_rows, accel_rows) -> None
TripletCallback = Callable[..., None]


class UiTripletModule(Module):
    """Sink module that joins ``frame.depth`` + ``imucam.sample`` by seq."""

    def __init__(self, bus: LocalPubSub, on_triplet: TripletCallback) -> None:
        super().__init__("ui", bus)
        self.ctx.state["on_triplet"] = on_triplet
        self.ctx.state["imu_rows"] = {}
        self.expected_ends = 2          # frame.depth + imucam.sample both end
        self.on(topics.IMUCAM_SAMPLE, [StashImuCam()])
        self.on(topics.FRAME_DEPTH, [RenderTriplet()])
