"""read_cam module: pull stereo on a schedule, trigger the IMU pack.

One half of the acquisition front-end (``imu_cam`` is the other). It owns the
*schedule*: one stereo pair per scheduler tick (``fps`` Hz). For each pair it
publishes a single :class:`~imu_camera.comms.messages.CamSync` (the frames + their
device timestamp) on ``cam.sync`` -- the trigger the
:class:`~imu_camera.modules.pipeline.ImuCamModule` reacts to.

This module is exactly ONE source module: the :class:`ReadCamModule` plus its
publish step and the pull-based frame sources (replay offline / live OAK-D).
depthai is only touched by :class:`LiveCamSource`, imported lazily, so the offline
path never pulls the device library.

A source is pull-based -- :meth:`CamSource.read` returns the next
``(seq, ts_ns, gray_left, gray_right)`` or ``None`` when exhausted -- because the
camera module, unlike the free-running IMU, decides *when* to grab a frame.
"""
from __future__ import annotations

import time

import numpy as np

from imu_camera.comms import LocalPubSub, SourceModule, Step, topics
from imu_camera.comms.messages import CamSync
from imu_camera.io.reader import SessionReader


# --------------------------------------------------------------------------- #
# Frame sources
# --------------------------------------------------------------------------- #
class CamSource:
    """Pull-based stereo source."""

    def open(self) -> None:
        """Acquire the source (open files / device). Optional."""

    def read(self):
        """Return the next ``(seq, ts_ns, gray_left, gray_right)`` or ``None``."""
        raise NotImplementedError

    def close(self) -> None:
        """Release the source. Optional."""


class ReplayCamSource(CamSource):
    """Yields a recorded session's stereo frames in order (offline, deterministic)."""

    def __init__(self, reader: SessionReader, *, load_right: bool = True,
                 max_frames: int = 0) -> None:
        self._reader = reader
        self._load_right = bool(load_right)
        n = len(reader)
        self._n = n if max_frames <= 0 else min(max_frames, n)
        self._i = 0

    def read(self):
        if self._i >= self._n:
            return None
        f = self._reader.load_frame(self._i, load_right=self._load_right)
        self._i += 1
        return (int(f.seq), int(f.ts_ns), f.gray_left,
                f.gray_right if self._load_right else None)


class LiveCamSource(CamSource):
    """Grabs synced stereo pairs from a shared OAK-D (raw left + raw right).

    Reads the mono pair off a
    :class:`~imu_camera.mathlib.device.oak_live.SharedLiveDevice` (the OAK-D is
    single-client, so the camera and IMU readers must share ONE device/pipeline).
    It pairs left/right by sequence number -- the cameras are hardware-synced, so a
    shared ``seq`` is a true same-instant pair -- and tags the pair with the left
    frame's device timestamp, the clock the IMU module drains against. depthai is
    pulled lazily by the shared device; hardware-only.
    """

    def __init__(self, device) -> None:
        self.device = device
        self._pend_l: dict[int, object] = {}
        self._pend_r: dict[int, object] = {}

    def open(self) -> None:
        self.device.acquire()

    @staticmethod
    def _seq(msg) -> int:
        try:
            return int(msg.getSequenceNum())
        except Exception:
            return -1

    @staticmethod
    def _gray(frame) -> np.ndarray:
        g = frame.getCvFrame()
        if g.ndim == 3:                                  # BGR -> luminance (601)
            g = (g[..., 0] * 0.114 + g[..., 1] * 0.587
                 + g[..., 2] * 0.299).astype(np.uint8)
        return g

    def read(self):
        dev = self.device
        while dev.is_running():
            ld = dev.poll("left")
            while True:
                nxt = dev.poll("left")
                if nxt is None:
                    break
                ld = nxt
            if ld is not None:
                self._pend_l[self._seq(ld)] = ld
            while True:
                nxt = dev.poll("right")
                if nxt is None:
                    break
                self._pend_r[self._seq(nxt)] = nxt
            common = self._pend_l.keys() & self._pend_r.keys()
            if not common:
                for buf in (self._pend_l, self._pend_r):
                    if len(buf) > 8:
                        for k in sorted(buf)[:-8]:
                            buf.pop(k, None)
                time.sleep(0.002)
                continue
            seq = max(common)
            ld = self._pend_l.pop(seq)
            rd = self._pend_r.pop(seq)
            for k in [k for k in self._pend_l if k < seq]:
                self._pend_l.pop(k, None)
            for k in [k for k in self._pend_r if k < seq]:
                self._pend_r.pop(k, None)
            try:
                ts_ns = int(ld.getTimestampDevice().total_seconds() * 1e9)
            except Exception:
                ts_ns = time.monotonic_ns()
            return seq, ts_ns, self._gray(ld), self._gray(rd)
        return None

    def close(self) -> None:
        self.device.release()


# --------------------------------------------------------------------------- #
# Steps
# --------------------------------------------------------------------------- #
class PublishCamSyncStep(Step):
    """publish_cam_sync step: emit one stereo pair as the IMU sync trigger."""

    name = "publish_cam_sync"

    def run(self, ctx, msg: CamSync):
        ctx.bus.publish(topics.CAM_SYNC, msg)
        return None


# --------------------------------------------------------------------------- #
# Module
# --------------------------------------------------------------------------- #
class ReadCamModule(SourceModule):
    """Source module: emit one :class:`~imu_camera.comms.messages.CamSync` per frame.

    ``fps`` sets the schedule; ``realtime`` paces ticks to it (live-like) versus
    running free (deterministic offline replay). ``source`` supplies the frames
    (``ReplayCamSource`` offline, ``LiveCamSource`` on the bench).
    """

    def __init__(self, bus: LocalPubSub, source: CamSource, *, fps: int = 20,
                 realtime: bool = False) -> None:
        super().__init__("cam", bus, [PublishCamSyncStep()])
        self.source = source
        self.fps = max(1, int(fps))
        self.realtime = bool(realtime)
        self.error: str | None = None
        self.forwards_to(topics.CAM_SYNC)

    def produce(self):
        try:
            self.source.open()
        except Exception as e:                                    # noqa: BLE001
            # e.g. the OAK-D is absent (X_LINK_DEVICE_NOT_FOUND). Record the
            # reason and return cleanly so the module still emits END -- the graph
            # drains and the UI can surface the failure instead of hanging.
            self.error = f"camera open failed: {e}"
            return
        period = 1.0 / self.fps
        try:
            next_tick = time.monotonic()
            while not self._stop.is_set():
                if self.realtime:
                    now = time.monotonic()
                    if now < next_tick:
                        time.sleep(next_tick - now)
                    next_tick += period
                try:
                    item = self.source.read()
                except Exception as e:                            # noqa: BLE001
                    self.error = f"camera read failed: {e}"
                    break
                if item is None:
                    break
                seq, ts_ns, gray_left, gray_right = item
                yield CamSync(seq=seq, ts_ns=ts_ns,
                              gray_left=gray_left, gray_right=gray_right)
        finally:
            self.source.close()
