"""fc process: stream the VIO earth-frame pose to a drone FC over UART.

Subscribes (over IPC) to the ``vio`` endpoint for ``pose.odom`` + the retained
``calib.bundle`` (used ONLY as a readiness barrier -- proof VIO is up), converts
each pose to the FC's NED earth frame via the shared SSOT
(:func:`sky.fc.fc_earth_pose.earth_pose_from_T_world_cam`), and writes it to the
serial port as a ``dblink`` ``DB_CMD_VIO_POSE`` frame (the in-house FC wire
protocol, :mod:`sky.fc.dblink`).

This is the FLIGHT-SAFETY output seam, so its structure is deliberate and the
safety floors below are NON-NEGOTIABLE:

Latest-wins, UART OFF the IPC callback
--------------------------------------
The IPC recv callback does ONLY one thing: store ``(wire_pose, recv_monotonic)``
in a 1-slot holder under a lock, then return immediately. It NEVER touches the
serial port. A dedicated daemon UART thread (:class:`UartSender`) loops at a fixed
cadence (``rate_hz``, clamped ``[10, 50]``), reads the freshest stored pose, and
does the convert+pack+write. So a slow / blocked UART can NEVER back-pressure the
flight pipeline (the callback always returns fast) and a write error / stale pose
NEVER crashes the run (logged + skipped).

Measurement age (instead of an absolute timestamp)
--------------------------------------------------
The wire carries ``age_us`` = a measure of how long ago the pose was captured
(capture -> send elapsed), not an absolute timestamp. age is a *duration*, so the
FC anchors it to its OWN clock -- ``valid_at_fc = fc_rx_time - age - C`` -- and the
module's absolute clock never has to be synchronised with the FC's. The capture
instant is ``pose.ts_ns``, the DEVICE (camera) clock. We recover the device->host
clock offset ``O`` on the UART thread by a running MINIMUM of
``(recv_host_s - ts_device_s)`` and then report ``age = send_host_s -
(ts_device_s + O)``, floored at 0.

WHAT age ACTUALLY MEASURES (honest property -- read before tuning C on the FC):
because the running-min ``O_est = O + min(capture->fc pipeline latency)``, the
reported age is biased YOUNGER than the true capture->send age by approximately
that minimum pipeline-latency floor. That floor is NOT sub-millisecond -- it
includes the VIO compute floor (tens of ms), the IPC hop and the sender's queue
wait. So ``age`` is the VARIABLE latency ABOVE the floor (fc queue wait + pipeline
jitter), with the roughly-constant floor subtracted out. The only guarantees are:
age is floored at 0 (never negative) and it UNDER-reports the absolute capture->
send age by ~that constant floor. The FC's fixed constant ``C`` MUST therefore
absorb the floor: ``C = UART_transport + pipeline_latency_floor`` (NOT just the
~4 ms UART transport). With C calibrated that way, ``fc_rx_time - age - C`` lands
on the true capture instant. A slow upward relaxation (``_OFFSET_RELAX_PER_S``)
lets ``O_est`` climb so a drifting device clock can't pin the estimate forever, and
a single anomalous (corrupt/future) ``ts_ns`` is rejected from the running-min so
it cannot latch the estimate low. (FUTURE "Muc 2": once ``imu_camera`` stamps a
host capture time, age can be the full absolute capture->send age and ``C`` reduces
to UART transport only.) If ``ts_ns`` is unset (0; the loose path shouldn't hit
this live) we fall back to the queue age ``send - recv`` only.

Safety floors (from the very first send)
----------------------------------------
* STALENESS: a stored pose older than ``_STALE_S`` (250 ms) is treated as stale --
  not sent (the FC must not fuse a stale fix as if it were fresh).
* POSITION SIGMA: ``pos_sigma_m`` is taken from ``info["pos_sigma_m"]`` only on a
  clean frame; whenever the fix must NOT be trusted at face value (``info`` missing,
  ``pos_sigma_m`` absent, ``vio_degraded`` set, or a ``sensor_gap_s`` re-lock marker)
  it is INFLATED to ``_SIGMA_DEGRADED`` -- a large finite metre value -- so the FC
  down-weights the fix to ~zero gain. We NEVER put NaN on the wire (some FC float
  parsers choke) and NEVER hand the FC an over-confident sigma.
* NON-FINITE POSE: this stack genuinely produces exploding / NaN poses (``--tight``
  on shake, ``--direct`` divergence). A non-finite position/quaternion is sent as an
  explicitly INVALID, degraded frame -- the broken field is zeroed/identity-ed, the
  validity bit (pos_valid / att_valid) is CLEARED and the degraded bit SET -- so the
  FC never fuses a broken pose as a real fix. The ``sky.fc.dblink`` leaf is also
  authoritative: it can never raise and never emit NaN/inf regardless of the caller.
* CAPTURE-AGE CEILING: a frame whose measured capture age exceeds ``_AGE_CEIL_US``
  (1 s) is dropped -- a defence distinct from the 250 ms QUEUE-staleness gate.
* reset_counter (owned here, a plain int on the UART thread): bumped on the RISING
  EDGE of a sensor-gap re-lock (``sensor_gap_s`` present this frame but not last)
  AND on an fc-local position-JUMP (a single-frame NED position delta exceeding a
  generous threshold). It is NOT keyed off ``loop.correction`` -- that is tight-only
  and blended, invisible on the loose / ``--direct`` default path (see PLAN). A bump
  tells the FC ESKF to reset its origin instead of fusing the discontinuity.
* flags: bit0 pos_valid (the solve's ``info["ok"]``), bit1 att_valid (the
  quaternion is valid once tracking), bit2 degraded (``vio_degraded`` /
  ``sensor_gap_s`` / ``inertial_dr``). The FC gates fusion on these.

Heading is RELATIVE (carried in the quaternion)
-----------------------------------------------
There is no magnetometer fusion: the optical world's gravity-aligned X axis
defines "North", so attitude is relative to the heading at VIO init. The wire
carries the FULL body->NED quaternion -- the FC extracts heading (and roll/pitch)
from it itself, which is gimbal-lock-free -- so no Euler/heading scalar is imposed
on the link; the FC must just treat the recovered heading as a RELATIVE source.

Calibration handshake
---------------------
Same as SLAM -- a dedicated calib client blocks until the retained
``calib.bundle`` arrives on the VIO endpoint. Receiving it proves VIO is up and
publishing; ``fc`` does not need the intrinsics themselves (it only sends pose).

Bundled downward range (optional)
---------------------------------
When ``--lidar-endpoint`` is given, ``fc`` ALSO opens a read-only client on the
``lidar`` process's endpoint, subscribes ``lidar.range``, and BUNDLES the freshest
gated reading into each VIO-pose frame (the trailing ``range_m`` + the
``range_valid`` flag bit in :mod:`sky.fc.dblink` -- it is NOT a separate dblink
message). This is best-effort + freshness-gated: a stale (> ``_RANGE_STALE_S``),
sensor-rejected, or absent reading sends ``range_valid=0``, and a missing / down
lidar process never blocks or delays the pose link (no calib barrier on the lidar
endpoint). So the VIO send is unaffected whether or not the rangefinder is present.

CONSUMER-ONLY: ``fc`` opens NO IPC server and publishes nothing. It is a pure sink.

Run::

    python -m fc.main --port /dev/ttyAMA0
    python -m fc.main --vio-endpoint oak.vio.test --port /dev/ttyUSB0 --baud 921600
    python -m fc.main --port /dev/ttyAMA0 --lidar-endpoint oak.lidar   # +range
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fc.comms import IPCPubSub, topics                              # noqa: E402
from fc.comms.messages import END                                  # noqa: E402
from fc.comms.wire import WireCalibBundle, WireEnd                 # noqa: E402
from sky.fc.dblink import pack_vio_pose                            # noqa: E402
from sky.fc.fc_earth_pose import earth_pose_from_T_world_cam       # noqa: E402

LOG = logging.getLogger("fc.main")

DEFAULT_VIO_ENDPOINT = "oak.vio"
#: Default IPC endpoint the ``lidar`` process serves ``lidar.range`` on. The fc
#: sender opens a read-only client here (when --lidar-endpoint is given) to bundle
#: the downward range into the VIO-pose frame.
DEFAULT_LIDAR_ENDPOINT = "oak.lidar"
DEFAULT_BAUD = 115200
DEFAULT_RATE_HZ = 30.0

#: A stored pose older than this (seconds) is STALE -> not sent (never fuse a
#: stale fix as fresh). Matches the propagate_imu sensor-gap guard's 250 ms.
_STALE_S = 0.25
#: A stored downward-range reading older than this (seconds) is treated as STALE ->
#: the VIO-pose frame goes out with range_valid=0 (the FC must not act on a stale
#: AGL range). The lidar process reads at ~50 Hz, so 200 ms is several missed reads
#: -- generous enough to ride a scheduling hiccup, tight enough to drop a dead
#: sensor. The range rides the pose frame; this only gates the range field, never
#: the pose itself (a stale / absent range never blocks the pose send).
_RANGE_STALE_S = 0.20
#: UART send cadence clamp (Hz). Below 10 the FC fusion starves; above 50 a slow
#: link (115200 baud ~= one 46-byte dblink frame per ~4 ms) can't keep up.
_RATE_MIN_HZ, _RATE_MAX_HZ = 10.0, 50.0
#: Small serial write timeout (seconds): a blocked UART must time out, not hang
#: the sender thread (the next loop simply tries again with the freshest pose).
_WRITE_TIMEOUT_S = 0.05
#: reset_counter wraps mod 256 (the dblink field is a u8).
_RESET_WRAP = 256
#: fc-local position-JUMP floor (m): a single-frame NED delta above
#: ``max(_JUMP_SIGMA_K * pos_sigma_m, _JUMP_FLOOR_M)`` bumps reset_counter. The
#: sigma term scales the gate with the reported uncertainty; the floor is the
#: generous absolute backstop when sigma is tiny / absent.
_JUMP_SIGMA_K = 5.0
_JUMP_FLOOR_M = 0.5
#: Inflated position sigma (m) sent when the fix must NOT be trusted at face value
#: (no info / no pos_sigma_m / vio_degraded / sensor_gap_s / non-finite pose). Large
#: + FINITE so the FC down-weights the fix to ~zero gain -- NEVER NaN (some FC float
#: parsers choke). ASSUMPTION (safety): the FC consumes pos_sigma_m as sqrt(R) with
#: NO internal floor that re-trusts a large sigma -- i.e. a bigger sigma must
#: monotonically REDUCE the Kalman gain toward zero, never clamp back up to a
#: confident value. If the FC ever floored R, this down-weighting defence is void
#: and a degraded fix would be fused as if trusted. (Cross-checked with the FC EKF.)
_SIGMA_DEGRADED = 100.0
#: Hard capture-age ceiling (us): a frame whose measured age exceeds this (1 s) is
#: NOT sent. Defence-in-depth BEYOND the 250 ms queue-staleness gate (which bounds
#: queue wait, a DIFFERENT quantity): this bounds the CAPTURE age, catching a stale
#: fix even if some future upstream stopped emitting the sensor_gap_s / inertial_dr
#: markers that drive the degraded path.
_AGE_CEIL_US = 1_000_000
#: Clock-offset outlier reject (s): a device-ts candidate ``cand = recv - ts`` more
#: than this BELOW the current running-min is treated as a corrupt / future ts_ns
#: and excluded from the running-min update, so one bad sample can't latch ``o_est``
#: low forever (which would inflate every subsequent age). age is still computed
#: from the existing ``o_est`` for that frame.
_OFFSET_OUTLIER_S = 0.5
#: dblink VIO-pose flag bits (matched on the FC side).
_FLAG_POS_VALID = 1 << 0
_FLAG_ATT_VALID = 1 << 1
_FLAG_DEGRADED = 1 << 2
#: Clock-offset relaxation rate (s of upward drift allowed per s of wall time): lets
#: the device->host offset estimate climb slowly so a one-off short transport sample
#: (which pins the running-min low) cannot under-estimate the offset forever.
_OFFSET_RELAX_PER_S = 1e-4


def _clamp_rate(rate_hz: float) -> float:
    """Clamp the requested UART cadence into the safe ``[10, 50]`` Hz band."""
    return float(min(max(float(rate_hz), _RATE_MIN_HZ), _RATE_MAX_HZ))


class LatestPose:
    """A 1-slot, lock-guarded holder for the freshest VIO pose.

    The IPC recv callback ``set()``s; the UART thread ``get()``s. Storing the
    whole wire message (not a derived product) keeps the callback's work to a
    single assignment so it returns instantly -- latest-wins, never back-pressures
    the flight pipeline.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._wm = None
        self._recv_t = 0.0

    def set(self, wm, recv_t: float) -> None:
        with self._lock:
            self._wm = wm
            self._recv_t = recv_t

    def get(self):
        """Return ``(wire_pose_or_None, recv_monotonic)`` -- a cheap snapshot."""
        with self._lock:
            return self._wm, self._recv_t


class LatestRange:
    """A 1-slot, lock-guarded holder for the freshest downward-range reading.

    Mirrors :class:`LatestPose`: the lidar IPC recv callback ``set()``s
    ``(range_m, valid, recv_monotonic)``; the UART thread ``get()``s the snapshot
    and applies its own freshness gate (against the local ``recv_t``, NOT the
    reading's device ts -- no cross-process clock assumption). Absent / never-set ->
    ``get()`` returns ``valid=0`` so a missing lidar process simply yields no range.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._range_m = 0.0
        self._valid = 0
        self._recv_t = 0.0

    def set(self, range_m: float, valid: int, recv_t: float) -> None:
        with self._lock:
            self._range_m = float(range_m)
            self._valid = int(valid)
            self._recv_t = recv_t

    def get(self):
        """Return ``(range_m, valid, recv_monotonic)`` -- a cheap snapshot."""
        with self._lock:
            return self._range_m, self._valid, self._recv_t


def _is_degraded(info: dict | None) -> bool:
    """True iff the fix must NOT be trusted at face value.

    Degraded == the solve flagged ``vio_degraded``, a ``sensor_gap_s`` re-lock
    marker is present (camera/IMU dropout just ended), or the pose is being purely
    inertially dead-reckoned (``inertial_dr``). Drives BOTH the inflated sigma and
    the ``degraded`` flag bit so the FC down-weights / gates the fix.
    """
    if not isinstance(info, dict):
        return False
    return bool(info.get("vio_degraded")) or ("sensor_gap_s" in info) \
        or bool(info.get("inertial_dr"))


def _sigma_for(info: dict | None) -> float:
    """The position 1-sigma (m) for this frame -- inflated, never NaN.

    SAFETY FLOOR: return the real ``info["pos_sigma_m"]`` ONLY on a clean frame
    with an explicit sigma; otherwise (``info`` missing, ``pos_sigma_m`` absent, or
    :func:`_is_degraded`) return the large finite ``_SIGMA_DEGRADED`` so the FC
    down-weights the fix to ~zero gain. We NEVER hand the FC an over-confident
    sigma and NEVER put NaN on the wire.
    """
    if not isinstance(info, dict) or _is_degraded(info):
        return _SIGMA_DEGRADED
    sig = info.get("pos_sigma_m")
    if sig is None:
        return _SIGMA_DEGRADED
    return float(sig)


def _flags_for(info: dict | None, *, att_valid: bool = True) -> int:
    """Assemble the dblink VIO-pose flags byte for this frame.

    * bit0 pos_valid -- the solve's ``info["ok"]`` (default True when unstated).
    * bit1 att_valid -- the body->NED quaternion is valid once tracking; the caller
      may pass ``att_valid=False`` to gate it (e.g. before the first pose).
    * bit2 degraded  -- :func:`_is_degraded` (vio_degraded / sensor_gap_s /
      inertial_dr); tells the FC to gate / heavily down-weight the fix.
    """
    info = info if isinstance(info, dict) else {}
    flags = 0
    if info.get("ok", True):
        flags |= _FLAG_POS_VALID
    if att_valid:
        flags |= _FLAG_ATT_VALID
    if _is_degraded(info):
        flags |= _FLAG_DEGRADED
    return flags


class UartSender(threading.Thread):
    """Daemon thread: fixed-cadence latest-wins VIO-pose -> dblink frame -> serial.

    Owns the serial port, the ``reset_counter`` (a plain int, mutated only here),
    the rising-edge sensor-gap detector, the fc-local position-jump detector and
    the device->host clock-offset estimate behind ``age_us``. It is deliberately
    decoupled from :func:`run_fc` and from the real ``serial.Serial`` so the SIL
    test can drive it against a pty: pass any object with a ``write(bytes)`` method
    as ``ser`` and any ``LatestPose`` as ``latest``.
    """

    def __init__(self, latest: LatestPose, ser, *, rate_hz: float = DEFAULT_RATE_HZ,
                 mount_extrinsic: np.ndarray | None = None,
                 latest_range: "LatestRange | None" = None) -> None:
        super().__init__(name="fc.uart_sender", daemon=True)
        self._latest = latest
        self._latest_range = latest_range
        self._ser = ser
        self._period = 1.0 / _clamp_rate(rate_hz)
        self._R_body_cam = mount_extrinsic
        self._stop = threading.Event()
        # reset_counter + its edge-detector state (all OWNED on this thread).
        self.reset_counter = 0
        self._prev_gap_present = False     # for the sensor-gap rising edge
        self._prev_pos_ned = None          # for the position-jump detector
        # Device->host clock-offset estimate for age_us (running-min of recv - ts,
        # see _age_us). None until the first pose carrying a device ts is seen.
        self._o_est: float | None = None
        self._last_ts_s = 0.0              # device ts of the last offset update
        # Lightweight counters the SIL test / logs can inspect (no hot-path cost).
        self.n_sent = 0
        self.n_stale = 0
        self.n_write_err = 0
        self.n_nonfinite = 0   # frames with a non-finite pose -> sent INVALID
        self.n_range_valid = 0  # frames that carried a fresh, valid downward range

    def stop(self) -> None:
        self._stop.set()

    # ---- edge detectors (pure given the per-frame inputs) ------------------ #
    def _bump_reset(self) -> None:
        self.reset_counter = (self.reset_counter + 1) % _RESET_WRAP

    def _update_reset_counter(self, info: dict | None, pos_ned: np.ndarray) -> None:
        """Bump reset_counter on a sensor-gap RISING EDGE or an fc-local jump.

        Both are debounced to ONE bump per event: the sensor-gap edge fires only
        on the not-present -> present transition of ``sensor_gap_s``; the jump
        fires once when a single-frame NED delta exceeds the generous gate. Keying
        off ``sensor_gap_s`` (re-lock after a dropout) + the fc-local jump -- NOT
        ``loop.correction`` -- because the loop correction is tight-only + blended
        and invisible on the loose / ``--direct`` default path (see PLAN).
        """
        info = info if isinstance(info, dict) else {}
        gap_present = "sensor_gap_s" in info
        # (a) sensor-gap re-lock rising edge: was-not -> is.
        if gap_present and not self._prev_gap_present:
            self._bump_reset()
            LOG.warning("fc: sensor-gap re-lock (gap=%.3fs) -> reset_counter=%d",
                        float(info.get("sensor_gap_s", 0.0)), self.reset_counter)
        self._prev_gap_present = gap_present
        # (b) fc-local position JUMP: a single-frame NED delta over the gate. The
        # gate scales with the reported sigma (when present) but never drops below
        # the generous absolute floor.
        if self._prev_pos_ned is not None:
            sig = info.get("pos_sigma_m")
            sig_term = _JUMP_SIGMA_K * float(sig) if sig is not None else 0.0
            gate = max(sig_term, _JUMP_FLOOR_M)
            delta = float(np.linalg.norm(np.asarray(pos_ned) - self._prev_pos_ned))
            # Don't double-count a gap-driven jump: the gap edge above already
            # bumped, and the re-anchor legitimately moves the pose.
            if delta > gate and not gap_present:
                self._bump_reset()
                LOG.warning("fc: position JUMP %.2fm > %.2fm gate -> "
                            "reset_counter=%d", delta, gate, self.reset_counter)
        self._prev_pos_ned = np.asarray(pos_ned, dtype=np.float64).copy()

    # ---- measurement age (device->host clock offset) ----------------------- #
    def _age_us(self, ts_ns: int, recv_t: float, send_t: float) -> int:
        """Measurement age (capture -> send) in microseconds, clamped to u32.

        ``ts_ns`` is the DEVICE-clock capture time of this pose; ``recv_t`` /
        ``send_t`` are host ``monotonic`` seconds. We estimate the device->host
        offset ``O`` as a running MINIMUM of ``recv - ts_device`` -- which converges
        to ``O + min(pipeline_latency)`` -- with a slow upward relaxation
        (``_OFFSET_RELAX_PER_S``) so a single unusually fast sample can't pin the
        estimate low forever. Then ``age = send - (ts_device + O_est)``, floored at 0.

        NOTE the reported age UNDER-reports the true capture->send age by ~the
        (roughly constant) minimum pipeline-latency floor baked into ``O_est``; the
        FC's ``C`` constant must absorb that floor (see the module docstring). The
        only hard properties are: age >= 0 and age carries the VARIABLE latency above
        the floor.

        FALLBACK: if ``ts_ns == 0`` (loose-path unset -- shouldn't happen live), use
        the queue age ``send - recv`` only (we have no capture time to anchor to).
        """
        if ts_ns <= 0:
            return int(max(0.0, send_t - recv_t) * 1e6)
        ts_s = ts_ns * 1e-9
        cand = recv_t - ts_s
        if self._o_est is None:
            self._o_est = cand
        elif cand < self._o_est - _OFFSET_OUTLIER_S:
            # Outlier reject: a candidate this far below the running-min implies a
            # corrupt / future ts_ns; don't let it latch o_est low forever. Skip the
            # update and age this frame off the existing (good) o_est.
            pass
        else:
            dt = max(0.0, ts_s - self._last_ts_s)
            self._o_est = min(cand, self._o_est + _OFFSET_RELAX_PER_S * dt)
            self._last_ts_s = ts_s
        age_s = send_t - (ts_s + self._o_est)
        return int(max(0.0, age_s) * 1e6)

    # ---- downward range (bundled into the VIO frame) ----------------------- #
    def _range_for(self, now: float) -> tuple[float, bool]:
        """The (range_m, range_valid) to BUNDLE into this frame's VIO-pose payload.

        Reads the latest downward-range reading and gates it on BOTH the sensor's
        own validity flag AND local freshness (``now - recv_t <= _RANGE_STALE_S``):
        a stale or sensor-rejected reading -> ``(0.0, False)`` so the FC sees
        range_valid=0. When no lidar holder is wired (no ``--lidar-endpoint`` / the
        lidar process is absent) this is always ``(0.0, False)`` -- the range field
        is simply never advertised valid, and the pose send is unaffected.
        """
        if self._latest_range is None:
            return 0.0, False
        range_m, valid, recv_t = self._latest_range.get()
        if not valid or (now - recv_t) > _RANGE_STALE_S:
            return 0.0, False
        return float(range_m), True

    # ---- the one-frame send (isolated so the SIL test can call it) --------- #
    def send_once(self) -> bool:
        """Read the latest pose; if fresh, convert + pack + write ONE dblink frame.

        Returns True iff a frame was written. Never raises: a stale pose is skipped,
        a NON-FINITE pose (this codebase genuinely produces exploding / NaN poses --
        ``--tight`` on shake, ``--direct`` divergence) is sent as an explicitly
        INVALID, degraded frame (never as a real fix), and any pack/write error is
        logged + swallowed -- the flight run must never die on the FC link, and the
        UART thread must NEVER terminate on an exception.
        """
        wm, recv_t = self._latest.get()
        if wm is None:
            return False
        now = time.monotonic()
        if now - recv_t > _STALE_S:
            self.n_stale += 1
            return False
        info = wm.info if isinstance(wm.info, dict) else {}
        pos_ned, q_ned, _ = earth_pose_from_T_world_cam(wm.T_world_cam,
                                                        self._R_body_cam)
        # reset_counter edge detection runs on EVERY fresh frame (before the send)
        # so a jump / re-lock is reflected in the very packet that carries it. Run
        # before the finiteness guard: a jump INTO a huge-but-finite / inf pose has
        # an inf delta (> the gate) so it ALSO bumps reset_counter -- correct, an
        # exploding pose IS a discontinuity. A NaN delta compares False (no bump),
        # which is fine: the finiteness guard below still flags that frame INVALID.
        self._update_reset_counter(info, pos_ned)
        # The wire carries the FULL body->NED quaternion (q_ned, the SSOT's second
        # return); the FC extracts heading itself (gimbal-lock-free) -- no Euler.
        sigma = _sigma_for(info)
        flags = _flags_for(info)
        age_us = self._age_us(int(getattr(wm, "ts_ns", 0) or 0), recv_t, now)
        # CAPTURE-AGE ceiling (defence-in-depth, distinct from the queue-staleness
        # gate above): a too-old capture is not a usable fix -- skip the send.
        if age_us > _AGE_CEIL_US:
            self.n_stale += 1
            return False
        # FINITENESS GUARD (safety): a non-finite pose must go out advertised
        # INVALID, never fused as a real fix. Zero/identity the broken field, force
        # the degraded sigma, and CLEAR the corresponding validity bit (+ set
        # degraded). The dblink leaf would already neutralise NaN/inf on the wire,
        # but the FLAGS are what tell the FC not to fuse it -- so we set them here.
        if not np.all(np.isfinite(pos_ned)):
            pos_ned = np.zeros(3, dtype=np.float64)
            sigma = _SIGMA_DEGRADED
            flags = (flags & ~_FLAG_POS_VALID) | _FLAG_DEGRADED
            self.n_nonfinite += 1
            if self.n_nonfinite <= 3 or self.n_nonfinite % 100 == 0:
                LOG.warning("fc: NON-FINITE position -> sent INVALID+degraded "
                            "(count=%d)", self.n_nonfinite)
        if not np.all(np.isfinite(q_ned)):
            q_ned = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            flags = (flags & ~_FLAG_ATT_VALID) | _FLAG_DEGRADED
            self.n_nonfinite += 1
            if self.n_nonfinite <= 3 or self.n_nonfinite % 100 == 0:
                LOG.warning("fc: NON-FINITE quaternion -> identity + att_valid "
                            "cleared (count=%d)", self.n_nonfinite)
        # BUNDLE the downward range into THIS frame (NOT a second frame): grab the
        # freshest gated reading; a stale / rejected / absent one -> range_valid=0.
        # The packer owns the range_valid flag bit + zeroes the field when invalid.
        range_m, range_valid = self._range_for(now)
        try:
            # pack is INSIDE the try: the leaf is engineered not to raise, but an
            # unforeseen error here must never escape run() and kill the thread.
            frame = pack_vio_pose(pos_ned, q_ned, sigma, age_us,
                                  self.reset_counter, flags,
                                  range_m=range_m, range_valid=range_valid)
            self._ser.write(frame)
        except Exception as e:                                      # noqa: BLE001
            self.n_write_err += 1
            # Throttle: a persistently bad UART would otherwise flood the log.
            if self.n_write_err <= 3 or self.n_write_err % 100 == 0:
                LOG.warning("fc: pack/serial write failed (%s); continuing", e)
            return False
        self.n_sent += 1
        if range_valid:
            self.n_range_valid += 1
        return True

    def run(self) -> None:
        next_t = time.monotonic()
        while not self._stop.is_set():
            self.send_once()
            next_t += self._period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                self._stop.wait(timeout=sleep_s)
            else:
                # Fell behind (a slow write / scheduling hiccup): re-anchor the
                # cadence so we don't burst to catch up.
                next_t = time.monotonic()


# --------------------------------------------------------------------------- #
def _await_calib_bundle(endpoint: str, timeout_s: float) -> WireCalibBundle:
    """Open a dedicated client, block until the retained calib bundle arrives.

    Mirrors ``slam._await_calib_bundle``: receiving VIO's retained
    ``calib.bundle`` is the readiness barrier proving VIO is up + publishing
    before we subscribe ``pose.odom``.
    """
    bundle: list[WireCalibBundle | None] = [None]
    got = threading.Event()

    def on_calib(wm: WireCalibBundle) -> None:
        bundle[0] = wm
        got.set()

    client = IPCPubSub(endpoint, role="client", connect_timeout_s=timeout_s)
    client.subscribe("calib.bundle", on_calib)
    client.start()
    try:
        if not got.wait(timeout=timeout_s):
            raise TimeoutError(
                f"fc: no calib.bundle from {endpoint!r} in {timeout_s}s")
    finally:
        client.stop()
    assert bundle[0] is not None
    return bundle[0]


def run_fc(*,
           vio_endpoint: str = DEFAULT_VIO_ENDPOINT,
           port: str,
           baud: int = DEFAULT_BAUD,
           rate_hz: float = DEFAULT_RATE_HZ,
           mount_extrinsic: np.ndarray | None = None,
           lidar_endpoint: str | None = None,
           calib_timeout_s: float = 30.0) -> int:
    """Run the FC UART-output process until END / SIGTERM / Ctrl-C.

    Opens the serial ``port`` (``write_timeout`` so a stuck UART can't hang the
    sender), barriers on VIO's calib bundle, subscribes ``pose.odom`` (CONSUMER-
    ONLY: no IPC server, no publish), and runs the latest-wins :class:`UartSender`.
    A failed serial open returns non-zero (the launcher treats that as non-fatal
    -- the rest of the stack keeps running).

    ``lidar_endpoint`` (optional): when given, also open a read-only client on the
    ``lidar`` process's endpoint, subscribe ``lidar.range``, and feed the freshest
    reading into the sender so the downward range is BUNDLED into each VIO-pose
    frame. The lidar client is BEST-EFFORT + non-blocking: if the lidar process is
    absent / never comes up, the sender simply emits range_valid=0 and the pose
    send is completely unaffected (no calib barrier on the lidar endpoint -- we do
    NOT block the FC link on the rangefinder).
    """
    import serial  # lazy: only the fc process needs pyserial

    rate_hz = _clamp_rate(rate_hz)
    LOG.info("fc: opening UART %s @ %d baud (rate=%.1f Hz, stale>%.0fms)",
             port, baud, rate_hz, _STALE_S * 1e3)
    try:
        ser = serial.Serial(port, baud, write_timeout=_WRITE_TIMEOUT_S)
    except Exception as e:                                          # noqa: BLE001
        # NON-FATAL to the stack: log + exit non-zero. The launcher mirrors how a
        # failed --forward connect never takes the pipeline down.
        LOG.error("fc: could NOT open serial port %r (%s) -- fc exiting; the "
                  "rest of the stack is unaffected", port, e)
        return 1

    # Barrier: block until VIO's retained calib bundle arrives (proves VIO is up).
    LOG.info("fc: waiting for calib.bundle on %s ...", vio_endpoint)
    try:
        _await_calib_bundle(vio_endpoint, calib_timeout_s)
    except Exception as e:                                          # noqa: BLE001
        LOG.error("fc: calib barrier failed (%s) -- fc exiting", e)
        ser.close()
        return 1
    LOG.info("fc: VIO up; subscribing to %s for pose.odom", topics.POSE_ODOM)

    latest = LatestPose()
    finished = threading.Event()

    def _on_pose(wm) -> None:
        # CONSUMER callback: store-and-return ONLY. END (the wire-level WireEnd or
        # the local END sentinel) ends the run; everything else is the freshest
        # pose. NEVER write serial here.
        if wm is END or isinstance(wm, WireEnd):
            finished.set()
            return
        latest.set(wm, time.monotonic())

    # CONSUMER-ONLY: a single read client on the VIO endpoint, no server/publisher.
    in_client = IPCPubSub(vio_endpoint, role="client")
    in_client.subscribe(topics.POSE_ODOM, _on_pose)

    # Optional downward-range client (best-effort): subscribe lidar.range on the
    # lidar endpoint and store the freshest reading, latest-wins, store-and-return.
    # NOT a calib-barriered dependency -- a missing lidar process must never block
    # or delay the FC link; the sender just sends range_valid=0.
    latest_range: LatestRange | None = None
    lidar_client = None
    if lidar_endpoint:
        latest_range = LatestRange()

        def _on_range(wm) -> None:
            # CONSUMER callback: store-and-return ONLY. A WireRange carries
            # (range_m, valid); END just stops feeding (the pose path owns the run
            # lifetime). NEVER write serial here.
            if wm is END or isinstance(wm, WireEnd):
                return
            latest_range.set(getattr(wm, "range_m", 0.0),
                             getattr(wm, "valid", 0), time.monotonic())

        # connect_timeout_s is short + non-fatal: if the lidar endpoint never comes
        # up the client keeps retrying in the background; the sender meanwhile sends
        # range_valid=0. We do NOT block run_fc on it.
        lidar_client = IPCPubSub(lidar_endpoint, role="client")
        lidar_client.subscribe(topics.LIDAR_RANGE, _on_range)
        LOG.info("fc: bundling downward range from %s (lidar.range)",
                 lidar_endpoint)

    sender = UartSender(latest, ser, rate_hz=rate_hz,
                        mount_extrinsic=mount_extrinsic,
                        latest_range=latest_range)

    stop = [False]

    def _on_sigterm(_signo, _frame):
        stop[0] = True
    # Ctrl-C (SIGINT) + launcher SIGTERM both request the same clean stop -- never
    # abort on a raw traceback.
    signal.signal(signal.SIGTERM, _on_sigterm)
    signal.signal(signal.SIGINT, _on_sigterm)

    sender.start()
    in_client.start()
    if lidar_client is not None:
        # Best-effort: start the range client AFTER the pose client so a slow lidar
        # connect can never delay the pose link coming up. Its own recv thread
        # retries the connect; a failure here is swallowed (range stays invalid).
        try:
            lidar_client.start()
        except Exception as e:                                      # noqa: BLE001
            LOG.warning("fc: lidar.range client could not start (%s); sending "
                        "range_valid=0", e)
            lidar_client = None
    LOG.info("fc[%s] streaming dblink DB_CMD_VIO_POSE -> %s (quaternion attitude; "
             "RELATIVE heading, no mag)", vio_endpoint, port)

    sender_died_logged = False
    try:
        while not stop[0] and not finished.is_set():
            time.sleep(0.1)
            # Belt-and-suspenders: the UART thread is engineered to NEVER die on an
            # exception (send_once swallows everything), but if it ever does, make it
            # LOUD instead of silently starving the FC of pose. Log once; keep the
            # run alive so the rest of the stack is unaffected (fc is non-fatal).
            if not sender.is_alive() and not sender_died_logged:
                sender_died_logged = True
                LOG.error("fc: UART sender thread is DEAD -- the FC is no longer "
                          "receiving pose (this should be impossible; investigate)")
    finally:
        sender.stop()
        sender.join(timeout=2.0)
        try:
            in_client.stop()
        except Exception:                                          # noqa: BLE001
            pass
        if lidar_client is not None:
            try:
                lidar_client.stop()
            except Exception:                                      # noqa: BLE001
                pass
        try:
            ser.close()
        except Exception:                                          # noqa: BLE001
            pass
        LOG.info("fc: shutdown complete (sent=%d stale=%d write_err=%d "
                 "nonfinite=%d range_valid=%d reset_counter=%d)", sender.n_sent,
                 sender.n_stale, sender.n_write_err, sender.n_nonfinite,
                 sender.n_range_valid, sender.reset_counter)
    return 0


# --------------------------------------------------------------------------- #
def _parse_mount(spec: str | None) -> np.ndarray | None:
    """Parse the optional ``--mount`` extrinsic: 9 comma-separated R_body_cam
    values (row-major 3x3). ``None`` -> the default identity (nominal forward
    mount). Raises ValueError on a malformed spec (fail fast, don't fly a bad
    mount silently)."""
    if not spec:
        return None
    vals = [float(v) for v in spec.replace(" ", "").split(",") if v != ""]
    if len(vals) != 9:
        raise ValueError(
            f"--mount needs 9 row-major R_body_cam values, got {len(vals)}")
    return np.asarray(vals, dtype=np.float64).reshape(3, 3)


def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vio-endpoint", default=DEFAULT_VIO_ENDPOINT,
                    help=f"VIO IPC endpoint (default: {DEFAULT_VIO_ENDPOINT!r})")
    ap.add_argument("--port", required=True,
                    help="serial port to the FC (e.g. /dev/ttyAMA0, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help=f"UART baud rate (default: {DEFAULT_BAUD})")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                    help=f"dblink VIO-pose send cadence in Hz, clamped "
                         f"[{int(_RATE_MIN_HZ)},{int(_RATE_MAX_HZ)}] "
                         f"(default: {DEFAULT_RATE_HZ:g})")
    ap.add_argument("--mount", default=None,
                    help="optional R_body_cam mount extrinsic: 9 comma-separated "
                         "row-major values (OpenCV-camera body -> FRD airframe body, "
                         "relative to the nominal forward mount). Default = identity "
                         "(camera faces forward). Heading is RELATIVE (no mag).")
    ap.add_argument("--lidar-endpoint", default=None,
                    help="optional: the lidar process's IPC endpoint (e.g. "
                         f"{DEFAULT_LIDAR_ENDPOINT!r}). When given, subscribe "
                         "lidar.range and BUNDLE the downward range into each "
                         "dblink VIO-pose frame. Best-effort: a missing lidar "
                         "process simply yields range_valid=0 (the pose send is "
                         "unaffected). Omit to never bundle a range.")
    ap.add_argument("--calib-timeout", type=float, default=30.0,
                    help="seconds to wait for the calib.bundle on boot")
    args = ap.parse_args()

    return run_fc(
        vio_endpoint=args.vio_endpoint,
        port=args.port,
        baud=args.baud,
        rate_hz=args.rate,
        mount_extrinsic=_parse_mount(args.mount),
        lidar_endpoint=args.lidar_endpoint,
        calib_timeout_s=args.calib_timeout,
    )


if __name__ == "__main__":
    # os._exit so a lingering non-daemon thread (IPCSubscriber's recv loop) cannot
    # keep the process alive past shutdown -- mirrors slam.main / vio.main.
    import os as _os
    _rc = main()
    LOG.info("fc: main returned, calling os._exit(%d)", int(_rc))
    logging.shutdown()
    _os.sys.stdout.flush()
    _os.sys.stderr.flush()
    _os._exit(int(_rc))
