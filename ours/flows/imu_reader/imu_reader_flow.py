"""The :class:`ImuReaderFlow` -- buffer IMU on a side thread, pack per trigger."""
from __future__ import annotations

from ...lib.flow import Bus, Flow, topics
from ...lib.imu.timed_buffer import TimedImuBuffer
from .pack_imucam import PackImuCam
from .publish_imucam import PublishImuCam
from .sources import ImuSource


class ImuReaderFlow(Flow):
    """Reactive flow: buffers IMU on a side thread, packs it per camera trigger.

    ``source`` supplies the raw IMU (``ReplayImuSource`` offline,
    ``LiveImuSource`` on the bench). ``wait_timeout`` bounds how long packing a
    frame waits for the IMU stream to cover its timestamp before draining what is
    available (so the run never hangs on the final frame).

    Note on threads: the *flow* owns one thread (it drains the inbox and runs the
    pack/publish chain). The injected ``source`` runs the continuous high-rate
    IMU read on its OWN I/O thread -- a hardware producer, not a flow, the same
    pattern the calibration ``ImuStream`` uses. No flow logic runs on that
    thread; it only fills the thread-safe buffer.
    """

    def __init__(self, bus: Bus, source: ImuSource, *,
                 buffer_capacity: int = 8192, wait_timeout: float = 0.5) -> None:
        super().__init__("imu-reader", bus)
        self.source = source
        self.buffer = TimedImuBuffer(capacity=buffer_capacity)
        self.forwards_to(topics.IMUCAM_SAMPLE)
        self.on(topics.CAM_SYNC,
                [PackImuCam(self.buffer, wait_timeout), PublishImuCam()])

    def run(self) -> None:
        # Continuous IMU read on the source's own I/O thread; close the buffer
        # when a replay source exhausts so any pending wait_until returns at once.
        self.source.start(self.buffer.append, on_exhausted=self.buffer.close)
        try:
            super().run()
        finally:
            self.source.stop()
            self.buffer.close()
