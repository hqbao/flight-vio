"""The :class:`CamReaderFlow` -- emit one ``CamSync`` per scheduled stereo pair."""
from __future__ import annotations

import time

from ...lib.flow import Bus, SourceFlow, topics
from ...lib.flow.messages import CamSync
from .publish_cam_sync import PublishCamSync
from .sources import CamSource


class CamReaderFlow(SourceFlow):
    """Source flow: emit one :class:`~ours.lib.flow.messages.CamSync` per frame.

    ``fps`` sets the schedule; ``realtime`` paces ticks to it (live-like) versus
    running free (deterministic offline replay). ``source`` supplies the frames
    (``ReplayCamSource`` offline, ``LiveCamSource`` on the bench).
    """

    def __init__(self, bus: Bus, source: CamSource, *, fps: int = 20,
                 realtime: bool = False) -> None:
        super().__init__("cam-reader", bus, [PublishCamSync()])
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
            # reason and return cleanly so the flow still emits END -- the graph
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
