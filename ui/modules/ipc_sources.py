"""IPC adapters so the EXISTING ``ui.qt`` windows + calib dialogs run in proc4.

The single-process windows drive their views off worker threads that build an
in-process acquisition / odometry graph on a private
:class:`~ui.comms.LocalPubSub` and tap it with a UI sink (see
``ui.qt.synced_window`` / ``ui.qt.keypoints_window``). In the 4-process
``./run.sh --proc`` topology there is no in-process graph: the data already lives
on the capture / VIO IPC servers. This module provides three drop-in adapters
that subscribe those IPC topics and republish them onto the very same local bus
the UNCHANGED UI sinks read -- so the windows + dialogs work identically without
any edit to ``ui.qt`` or ``ui.main``.

Device-agnostic by contract
---------------------------
This module is part of the proc4 UI plumbing, which must stay generic for a
future multi-chip port: it consumes only the abstract IPC topics + Wire POD
types and NEVER imports depthai (no device/chip library) -- that device-agnostic
guarantee is the one the multi-chip port depends on. It does pull PyQt6
transitively (the ``TripletWorker`` / ``KeypointWorker`` base classes live in
the Qt window modules), which is expected -- the UI is a Qt app; "generic" here
means independent of the camera/SoC, not of the GUI toolkit. It does NOT import
``ui.main`` (no import cycle), so ``ui.main`` can import it lazily inside
``run_ui`` to keep its own module import Qt-free.

What each adapter feeds
-----------------------
* :class:`IpcImuRawSource` -- duck-types the calib dialogs' default IMU stream
  for the gyro / accel calib dialogs. Subscribes capture's RAW IMU (``imu.raw``)
  and re-emits one ``(3,)`` sample at a time (the shape the dialog's stillness
  gate / six-face collector expect).
* :class:`IpcTripletWorker` -- a :class:`~ui.qt.synced_window.TripletWorker`
  whose ``_drive`` republishes capture's ``imucam.sample`` + ``frame.depth`` so
  the UNCHANGED :class:`~ui.qt.synced_window.SyncedViewWindow` sink renders the
  triplet.
* :class:`IpcKeypointWorker` -- a
  :class:`~ui.qt.keypoints_window.KeypointWorker` whose ``_drive`` republishes
  capture's ``frame.depth`` plus VIO's ``frame.tracks`` + ``frame.inliers`` (two
  endpoints) so the UNCHANGED
  :class:`~ui.qt.keypoints_window.KeypointTrackWindow` sink renders the
  keypoint overlay.

IPC client error model (how connect failures are surfaced)
----------------------------------------------------------
:meth:`ui.comms.IPCPubSub.start` (role="client") RAISES (``TimeoutError`` /
``ConnectionError``) when the socket never appears within ``connect_timeout_s``;
once connected, a runtime receive error instead sets its ``.error`` attribute.
:class:`~ui.comms.IPCSubscriber` swallows the ``start`` exception inside its own
``run`` (it logs and returns), so to surface a connect failure to the polling
window we (a) check each client's ``.error`` every loop tick and (b) detect a
subscriber that died at start (the client never connected) and report a connect
failure. The base worker ``run`` catches any exception we raise in ``_drive``
into ``self.error`` for the window to display.
"""
from __future__ import annotations

import numpy as np

from ui.comms import topics
from ui.comms.messages import END
from ui.comms import IPCPubSub, IPCSubscriber, RingRegistry
from ui.comms.converters import to_local
from ui.comms.ring_registry import default_capture_specs
from ui.qt.keypoints_window import KeypointWorker
from ui.qt.synced_window import TripletWorker


def _attach_capture_rings(endpoint: str, width: int, height: int) -> RingRegistry:
    """Attach capture's consumer-side shared-memory rings.

    The rings only exist while the capture process is running, so a failure here
    almost always means capture is down. Re-raise it as a clear, device-agnostic
    reason (the base worker ``run`` lifts it onto ``self.error`` for the window)
    instead of leaking a raw ``/<endpoint>.gray_left`` shared-memory path.
    """
    try:
        return RingRegistry().attach_all(default_capture_specs(
            endpoint=endpoint, width=int(width), height=int(height)))
    except FileNotFoundError as e:
        raise RuntimeError(
            f"capture stream not available on {endpoint!r} "
            f"(is capture running?)") from e


# --------------------------------------------------------------------------- #
# (1) IMU source for the calibration dialogs
# --------------------------------------------------------------------------- #
class IpcImuRawSource:
    """Duck-typed IMU stream over capture's RAW IMU IPC topic.

    The gyro / accel calibration dialogs (:mod:`ui.qt.calib_dialogs`) drive a
    stream object with exactly four touch-points -- ``start(callback)``,
    ``stop()``, ``.error`` and ``.device_id`` -- and feed each ``(3,)`` sample to
    a stillness gate / six-face collector. This adapter offers the same surface
    but sources the samples from capture's retained ``imu.raw`` topic instead of
    opening a device, so the SAME dialogs work unchanged in the 4-process UI.

    NOT an ``ImuStream`` subclass: it shares no implementation, only the duck
    type the dialogs rely on.

    ``imu.raw`` is the RAW, uncalibrated IMU (capture's ``_DATA_TOPICS`` publishes
    it) -- exactly what a calibration must consume (calibrating off an
    already-calibrated stream would be circular).
    """

    def __init__(self, capture_endpoint: str, *,
                 device_id: str = "default",
                 connect_timeout_s: float = 30.0) -> None:
        self._endpoint = capture_endpoint
        self._connect_timeout_s = float(connect_timeout_s)
        # Public attrs the dialog reads (mirror the IMU stream's contract).
        self.device_id: str = device_id
        self.error: str | None = None

        self._client: IPCPubSub | None = None
        # ``imu.raw`` is pure POD (no shared-memory ring), so a bare registry is
        # enough for the converter -- the ``rings`` arg is unused for this topic.
        self._rings = RingRegistry()
        # cb(gyro:(3,), accel:(3,), t_s_seconds) -> None
        self._cb = None

    # ------------------------------------------------------------------ #
    def start(self, callback) -> None:
        """Connect to capture and stream per-sample IMU rows to ``callback``.

        On connect failure set :attr:`error` and return (do NOT raise): the
        dialog polls :attr:`error` on its UI timer and surfaces it itself.
        """
        self._cb = callback
        client = IPCPubSub(self._endpoint, role="client",
                           connect_timeout_s=self._connect_timeout_s)
        client.subscribe(topics.IMU_RAW, self._on_imu)
        try:
            client.start()
        except Exception as e:                                     # noqa: BLE001
            # start() raises on connect timeout / refusal -- surface it for the
            # dialog's poll loop rather than crashing the UI thread.
            self.error = f"capture IMU stream connect failed: {e}"
            return
        self._client = client

    def _on_imu(self, wm) -> None:
        """Receive thread: split a wire IMU batch into per-sample callbacks."""
        if wm is END:
            return
        # IMU_RAW is pure POD; the rings arg is unused for this topic.
        imu = to_local(topics.IMU_RAW, wm, self._rings)
        if imu is END:                                # WireEnd -> local END
            return
        gyro = np.asarray(imu.gyro, dtype=np.float64).reshape(-1, 3)
        accel = np.asarray(imu.accel, dtype=np.float64).reshape(-1, 3)
        imu_ts = np.asarray(imu.imu_ts, dtype=np.int64).reshape(-1)
        m = int(min(gyro.shape[0], accel.shape[0], imu_ts.shape[0]))
        if m == 0:                                    # no samples this interval
            return
        cb = self._cb
        if cb is None:
            return
        # The dialog's collector takes ONE (3,) sample at a time with a float
        # SECONDS timestamp (it computes window_s = t_last - t_start and needs
        # >=80 gyro samples over >=1.0 s). The wire batch is (M, 3) in ns, so
        # emit row-by-row in seconds.
        for i in range(m):
            cb(gyro[i], accel[i], float(imu_ts[i]) * 1e-9)

    def stop(self) -> None:
        """Close the IPC client (idempotent; swallow teardown errors)."""
        client = self._client
        self._client = None
        if client is not None:
            try:
                client.stop()
            except Exception:                                      # noqa: BLE001
                pass


# --------------------------------------------------------------------------- #
# (1b) Gyro-fusion source for the strip-chart window
# --------------------------------------------------------------------------- #
class IpcGyroFuseSource:
    """Duck-typed gyro-fusion stream over VIO's ``frame.gyrofuse`` IPC topic.

    The "Gyro fusion" strip-chart window (:mod:`ui.qt.gyrofuse_window`) drives a
    stream object with exactly three touch-points -- ``start(callback)``,
    ``stop()`` and ``.error`` -- and feeds each per-frame
    :class:`~ui.comms.messages.FrameGyroFuse` to its chart. This adapter offers
    the same surface but sources the records from VIO's ``frame.gyrofuse`` topic
    (pure POD, no shared-memory ring), so the window needs no device handle.

    ``frame.gyrofuse`` is published ONLY on gyro-fused frames (the VIO publisher
    self-skips when gyro is off / PnP failed), so every record the callback sees
    is a genuine fusion observation -- the chart never gets a garbage frame.
    Mirrors :class:`IpcImuRawSource`'s connect-error model: ``start`` swallows a
    connect timeout onto :attr:`error` (the window polls it) rather than raising.
    """

    def __init__(self, vio_endpoint: str, *,
                 connect_timeout_s: float = 30.0) -> None:
        self._endpoint = vio_endpoint
        self._connect_timeout_s = float(connect_timeout_s)
        self.error: str | None = None
        self._client: IPCPubSub | None = None
        # frame.gyrofuse is pure POD (no ring), so a bare registry suffices for
        # the converter -- the ``rings`` arg is unused for this topic.
        self._rings = RingRegistry()
        self._cb = None

    def start(self, callback) -> None:
        """Connect to VIO and stream each FrameGyroFuse record to ``callback``."""
        self._cb = callback
        client = IPCPubSub(self._endpoint, role="client",
                           connect_timeout_s=self._connect_timeout_s)
        client.subscribe(topics.FRAME_GYROFUSE, self._on_msg)
        try:
            client.start()
        except Exception as e:                                     # noqa: BLE001
            self.error = f"VIO gyro-fusion stream connect failed: {e}"
            return
        self._client = client

    def _on_msg(self, wm) -> None:
        if wm is END:
            return
        msg = to_local(topics.FRAME_GYROFUSE, wm, self._rings)
        if msg is END:                                # WireEnd -> local END
            return
        cb = self._cb
        if cb is not None:
            cb(msg)

    def stop(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            try:
                client.stop()
            except Exception:                                      # noqa: BLE001
                pass


# --------------------------------------------------------------------------- #
# (2) Triplet worker (image | depth | IMU) for SyncedViewWindow
# --------------------------------------------------------------------------- #
class IpcTripletWorker(TripletWorker):
    """Drive :class:`~ui.qt.synced_window.SyncedViewWindow` over IPC.

    Republishes capture's ``imucam.sample`` + ``frame.depth`` onto the local bus
    that the window's :class:`~ui.modules.triplet.UiTripletModule` sink joins by
    ``seq`` -- so the window renders the exact same triplet it would from the
    in-process front-end, without any edit to the window.
    """

    mode = "IPC"

    def __init__(self, capture_endpoint: str, width: int, height: int, *,
                 connect_timeout_s: float = 10.0) -> None:
        super().__init__()
        self._cap_ep = capture_endpoint
        self._w = int(width)
        self._h = int(height)
        self._connect_timeout_s = float(connect_timeout_s)

    def _drive(self, bus, sink) -> None:
        # Attach capture's shared-memory rings (consumer side) so the subscriber
        # bridge can ``read_copy`` the frame + depth arrays out of them. The
        # rings only exist while capture is up, so a missing ring == capture not
        # running -- surface that as a clear reason, not a raw shm-path error.
        cap_rings = _attach_capture_rings(self._cap_ep, self._w, self._h)
        client = IPCPubSub(self._cap_ep, role="client",
                           connect_timeout_s=self._connect_timeout_s)
        sub = IPCSubscriber(bus, client, cap_rings,
                            [topics.IMUCAM_SAMPLE, topics.FRAME_DEPTH])
        # Mirror the Replay/Live worker lifecycle: start the sink first, then the
        # source bridge; loop until stopped while surfacing the first error.
        sink.start()
        sub.start()
        try:
            while not self._stop.is_set():
                self._stop.wait(0.05)
                err = self._connect_or_runtime_error(client, sub)
                if err is not None:
                    self.error = err
                    break
        finally:
            sub.stop()
            sink.stop()
            cap_rings.close()

    @staticmethod
    def _connect_or_runtime_error(client: IPCPubSub,
                                  sub: IPCSubscriber) -> str | None:
        """First fatal reason from a client, or None.

        ``IPCPubSub.start`` raises on a failed connect; ``IPCSubscriber``
        catches that inside its ``run`` and returns, so a dead subscriber thread
        means the client never connected. A runtime receive error instead lands
        on ``client.error``.
        """
        if client.error:
            return client.error
        if not sub.is_alive():
            return f"capture stream connect failed ({client.endpoint})"
        return None


# --------------------------------------------------------------------------- #
# (3) Keypoint worker (frame + KLT tracks) for KeypointTrackWindow
# --------------------------------------------------------------------------- #
class IpcKeypointWorker(KeypointWorker):
    """Drive :class:`~ui.qt.keypoints_window.KeypointTrackWindow` over IPC.

    The overlay needs three streams from TWO endpoints: ``frame.depth`` (the
    rectified-left image + metric depth) comes from CAPTURE, while
    ``frame.tracks`` + ``frame.inliers`` (the KLT ids/pixels + PnP inliers) come
    from VIO. We republish all three onto the local bus the window's
    :class:`~ui.modules.tracks.UiTracksModule` sink reads.
    """

    mode = "IPC"
    #: Realtime live view -- keep latency bounded (latest-only sink).
    latest_only = True

    def __init__(self, capture_endpoint: str, vio_endpoint: str,
                 width: int, height: int, *,
                 connect_timeout_s: float = 10.0) -> None:
        super().__init__()
        self._cap_ep = capture_endpoint
        self._vio_ep = vio_endpoint
        self._w = int(width)
        self._h = int(height)
        self._connect_timeout_s = float(connect_timeout_s)

    def _drive(self, bus, sink) -> None:
        # Capture's depth ring must be attached so its frame.depth converts; VIO's
        # tracks/inliers are pure POD (no ring) so a bare registry suffices there.
        # A missing ring == capture not running -> surface a clear reason.
        cap_rings = _attach_capture_rings(self._cap_ep, self._w, self._h)
        cap_client = IPCPubSub(self._cap_ep, role="client",
                               connect_timeout_s=self._connect_timeout_s)
        vio_client = IPCPubSub(self._vio_ep, role="client",
                               connect_timeout_s=self._connect_timeout_s)
        # Depth first (per-seq image+depth); tracks/inliers (POD) from VIO.
        cap_sub = IPCSubscriber(bus, cap_client, cap_rings,
                                [topics.FRAME_DEPTH])
        vio_sub = IPCSubscriber(bus, vio_client, RingRegistry(),
                                [topics.FRAME_TRACKS, topics.FRAME_INLIERS])
        sink.start()
        cap_sub.start()
        vio_sub.start()
        try:
            while not self._stop.is_set():
                self._stop.wait(0.05)
                err = self._first_error(((cap_client, cap_sub),
                                         (vio_client, vio_sub)))
                if err is not None:
                    self.error = err
                    break
        finally:
            vio_sub.stop()
            cap_sub.stop()
            sink.stop()
            cap_rings.close()

    @staticmethod
    def _first_error(pairs) -> str | None:
        """First fatal reason across ``(client, sub)`` pairs, or None."""
        for client, sub in pairs:
            if client.error:
                return client.error
            if not sub.is_alive():
                return f"stream connect failed ({client.endpoint})"
        return None


# --------------------------------------------------------------------------- #
# (4) Factory helpers -- the windows want a zero-arg ``worker_factory``
# --------------------------------------------------------------------------- #
def ipc_triplet_factory(capture_endpoint: str, width: int, height: int):
    """Return a zero-arg factory building an :class:`IpcTripletWorker`."""
    return lambda: IpcTripletWorker(capture_endpoint, width, height)


def ipc_keypoint_factory(capture_endpoint: str, vio_endpoint: str,
                         width: int, height: int):
    """Return a zero-arg factory building an :class:`IpcKeypointWorker`."""
    return lambda: IpcKeypointWorker(capture_endpoint, vio_endpoint,
                                     width, height)
