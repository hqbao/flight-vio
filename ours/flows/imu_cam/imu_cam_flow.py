"""The :class:`ImuCamFlow` -- buffer IMU, pack per trigger, compute depth."""
from __future__ import annotations

from collections.abc import Callable

from ...lib.flow import Bus, Flow, topics
from ...lib.imu.imu_calib import ImuCalibration
from ...lib.imu.timed_buffer import TimedImuBuffer
from ...lib.stereo.stereo import SGMStereoMatcher
from .admission import Admission, AdmitAll
from .admit_frame import AdmitFrame
from .apply_calibration import ApplyCalibration
from .complete_admission import CompleteAdmission
from .compute_depth import ComputeDepth
from .pack_imucam import PackImuCam
from .publish_depth import PublishDepth
from .publish_imu_raw import PublishImuRaw
from .publish_imucam import PublishImuCam
from .sources import ImuSource


class ImuCamFlow(Flow):
    """Reactive flow: buffer IMU, pack per camera trigger, compute dense depth.

    ``source`` supplies the raw IMU (``ReplayImuSource`` offline,
    ``LiveImuSource`` on the bench). ``wait_timeout`` bounds how long packing a
    frame waits for the IMU stream to cover its timestamp before draining what is
    available (so the run never hangs on the final frame).

    For every camera trigger the flow publishes the uncalibrated samples on
    ``topics.IMU_RAW`` (honest: exactly what the sensor reported) and, on
    ``topics.IMUCAM_SAMPLE``, the frames bundled with the CALIBRATED IMU.
    ``calibration`` (or the lazy ``calibration_provider``, used on the live path
    where the device id is known only after the device opens) supplies the
    per-device correction; with none, the calibrated packet equals the raw one.

    ``matcher`` makes depth a task IN this flow: when supplied, the chain also
    runs SGM on the same stereo pair and publishes ``topics.FRAME_DEPTH`` -- depth
    is just a transform of the pair this flow already produces, so it lives here
    rather than in a separate flow (it still runs on this flow's thread, in
    parallel with the odometry thread that consumes the result). The
    camera/IMU visualiser passes ``matcher=None`` (it only wants the synced
    packet, no depth).

    ``admission`` is the realtime backpressure gate (see
    :mod:`~ours.flows.imu_cam.admission`). The default
    :class:`~ours.flows.imu_cam.admission.AdmitAll` admits every frame (replay
    determinism); the live path injects a
    :class:`~ours.flows.imu_cam.admission.BudgetAdmission` so at most ``N`` frames
    are in flight. The gate is the FIRST task in the camera chain (it runs before
    the IMU is drained, so a skip folds that interval into the next frame), and
    ``topics.FRAME_DONE`` frees a credit when the odometry tail reports a frame
    finished.

    Note on threads: the *flow* owns one thread (it drains the inbox and runs the
    pack/publish/depth chain). The injected ``source`` runs the continuous
    high-rate IMU read on its OWN I/O thread -- a hardware producer, not a flow,
    the same pattern the calibration ``ImuStream`` uses. No flow logic runs on
    that thread; it only fills the thread-safe buffer.
    """

    def __init__(self, bus: Bus, source: ImuSource, *,
                 matcher: SGMStereoMatcher | None = None,
                 buffer_capacity: int = 8192, wait_timeout: float = 0.5,
                 calibration: ImuCalibration | None = None,
                 calibration_provider:
                     Callable[[], ImuCalibration | None] | None = None,
                 admission: Admission | None = None) -> None:
        super().__init__("imu-cam", bus)
        self.source = source
        self.buffer = TimedImuBuffer(capacity=buffer_capacity)
        self.admission = admission or AdmitAll()

        chain = [
            AdmitFrame(self.admission),
            PackImuCam(self.buffer, wait_timeout),
            PublishImuRaw(),
            ApplyCalibration(calibration, provider=calibration_provider),
            PublishImuCam(),
        ]
        downstream = [topics.IMU_RAW, topics.IMUCAM_SAMPLE]
        if matcher is not None:
            self.ctx.state["matcher"] = matcher
            chain += [ComputeDepth(), PublishDepth()]
            downstream.append(topics.FRAME_DEPTH)

        self.forwards_to(*downstream)
        self.on(topics.CAM_SYNC, chain)
        # Backpressure control: free a credit per finished frame. Not END-bearing
        # (odometry never forwards END here), so it does not affect drain.
        self.on(topics.FRAME_DONE, [CompleteAdmission(self.admission)])

    def run(self) -> None:
        # Continuous IMU read on the source's own I/O thread; close the buffer
        # when a replay source exhausts so any pending wait_until returns at once.
        self.source.start(self.buffer.append, on_exhausted=self.buffer.close)
        try:
            super().run()
        finally:
            self.source.stop()
            self.buffer.close()
