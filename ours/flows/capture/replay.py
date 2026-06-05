"""Replay capture: stream a recorded session onto the bus.

This is the offline drop-in for the live OAK-D camera. It reads a session with
:class:`~ours.lib.io.reader.SessionReader` and publishes exactly what the live
camera would:

* one :class:`~ours.lib.messages.ImuChunk` up front (so the odometry flow can
  build its gyro preintegrator and gravity-align), then
* one :class:`~ours.lib.messages.RawFrame` per recorded frame.

It is a :class:`~ours.lib.flow.SourceFlow`: ``produce`` yields those messages and
the single publish task routes each to its topic. When the session is exhausted
the base class emits ``END`` on ``frame.raw`` so the graph drains.
"""
from __future__ import annotations

from ...lib import topics
from ...lib.flow import SourceFlow
from ...lib.io.reader import SessionReader
from ...lib.messages import ImuChunk, RawFrame
from ...lib.pubsub import Bus
from ...lib.task import Task


class _PublishCapture(Task):
    """Route a produced message to its topic by type."""

    name = "publish_capture"

    def run(self, ctx, msg):
        if isinstance(msg, ImuChunk):
            ctx.bus.publish(topics.IMU_SAMPLE, msg)
        elif isinstance(msg, RawFrame):
            ctx.bus.publish(topics.FRAME_RAW, msg)
        return None


class ReplayCaptureFlow(SourceFlow):
    def __init__(self, bus: Bus, reader: SessionReader,
                 load_right: bool = True, max_frames: int = 0) -> None:
        super().__init__("capture", bus, [_PublishCapture()])
        self.reader = reader
        self.load_right = load_right
        self.max_frames = max_frames
        # END travels only down the primary frame path (imu.sample is auxiliary).
        self.forwards_to(topics.FRAME_RAW)

    def produce(self):
        r = self.reader
        if r.calib.has_imu_extrinsics:
            imu = r.load_imu()
            if imu["ts_ns"].size > 1:
                yield ImuChunk(imu["ts_ns"], imu["gyro"], imu["accel"],
                               r.calib.T_imu_left, True)
        n = len(r) if self.max_frames <= 0 else min(self.max_frames, len(r))
        for i in range(n):
            f = r.load_frame(i, load_right=self.load_right)
            yield RawFrame(f.seq, f.ts_ns, f.gray_left,
                           f.gray_right if self.load_right else None)
