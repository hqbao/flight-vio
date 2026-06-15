"""Full-ndarray (ref-free) <-> local-dataclass conversion for the TCP boundary.

The in-host bridge keeps large arrays in shared memory: the wire ``Wire*`` classes
for image topics (``WireCamSync`` / ``WireDepthFrame`` / ``WireImuCamPacket`` /
``WireKeyframe``) carry ``SharedArrayRef`` METADATA in their image/depth slots, and
the pixels are ``read_copy``-ed out of a ``SharedArrayRing`` by the subscriber.
That is impossible across the network -- the Mac cannot ``read_copy`` the Pi's
shared memory -- so netbridge ships the FULL ndarray inline.

This module is the two halves of that ref-free codec:

* :func:`local_to_full_wire` (forward, on the Pi) -- given a LOCAL dataclass (whose
  arrays are already REAL ndarrays, resolved by ``comms.converters.to_local``),
  build the topic's ``Wire*`` instance with the ndarrays placed DIRECTLY in the
  former ref slots. The shared ``comms.codec`` then encodes those as full 0x08
  ndarray frames (it dispatches by Python type), so NO ``SharedArrayRef`` (0x09)
  ever reaches the wire.

* :func:`full_wire_to_local` (receive, on the Mac) -- given a ``Wire*`` decoded off
  the TCP stream (full ndarrays in the former ref slots), rebuild the LOCAL
  dataclass. The receive bridge then hands that to the standard
  ``comms.bridge.IPCPublisher`` (``comms.converters.to_wire``), which writes the
  arrays into the MAC-LOCAL rings -> 0x09 refs over AF_UNIX -> the UI reads them
  exactly as if it were on the Pi.

POD + retained topics need no ref handling: their ``to_wire``/``to_local`` already
ship every array inline. For those, forward calls ``comms.converters.to_wire``
with NO ring writes (a sentinel registry), and receive calls
``comms.converters.to_local`` -- both are pass-through for inline arrays. So this
module only special-cases the FOUR ref-bearing image topics; everything else flows
through the unchanged ``comms.converters``.
"""
from __future__ import annotations

from typing import Any

from netbridge.comms import topics
from netbridge.comms.messages import CamSync, DepthFrame, ImuCamPacket, Keyframe
from netbridge.comms.wire import (
    WireCamSync, WireDepthFrame, WireImuCamPacket, WireKeyframe,
)

#: The four topics whose ``Wire*`` carry ``SharedArrayRef`` slots (image/depth).
#: Everything else is pure POD and rides the standard converters unchanged.
REF_BEARING_TOPICS = frozenset({
    topics.CAM_SYNC, topics.FRAME_DEPTH, topics.IMUCAM_SAMPLE, topics.KEYFRAME,
})


# --------------------------------------------------------------------------- #
# forward (Pi): LOCAL dataclass (real ndarrays) -> ref-free Wire* (ndarrays inline)
# --------------------------------------------------------------------------- #
def local_to_full_wire(topic: str, msg: Any) -> Any:
    """Build the topic's ``Wire*`` with REAL ndarrays in the former ref slots.

    Only the four ref-bearing image topics are handled here; the caller routes
    every other (POD) topic through ``comms.converters.to_wire``. The returned
    wire object holds ndarrays where the in-host wire would hold ``SharedArrayRef``
    -- so ``comms.codec.encode`` emits full 0x08 ndarray frames, never 0x09.
    """
    if topic == topics.CAM_SYNC:
        m: CamSync = msg
        return WireCamSync(seq=int(m.seq), ts_ns=int(m.ts_ns),
                           gray_left_ref=m.gray_left,
                           gray_right_ref=m.gray_right)
    if topic == topics.FRAME_DEPTH:
        m = msg                                   # type: ignore[assignment]
        return WireDepthFrame(seq=int(m.seq), ts_ns=int(m.ts_ns),
                              gray_left_ref=m.gray_left, depth_ref=m.depth_m)
    if topic == topics.IMUCAM_SAMPLE:
        m = msg                                   # type: ignore[assignment]
        return WireImuCamPacket(seq=int(m.seq), ts_ns=int(m.ts_ns),
                                gray_left_ref=m.gray_left,
                                gray_right_ref=m.gray_right,
                                imu_ts=m.imu_ts, gyro=m.gyro, accel=m.accel)
    if topic == topics.KEYFRAME:
        m = msg                                   # type: ignore[assignment]
        return WireKeyframe(seq=int(m.seq), T_world_cam=m.T_world_cam,
                            gray_left_ref=m.gray_left, depth_ref=m.depth_m,
                            track_ids=m.track_ids, track_px=m.track_px,
                            accel=m.accel, inlier_ids=m.inlier_ids)
    raise KeyError(f"local_to_full_wire: {topic!r} is not a ref-bearing topic")


# --------------------------------------------------------------------------- #
# receive (Mac): ref-free Wire* (ndarrays inline) -> LOCAL dataclass
# --------------------------------------------------------------------------- #
def full_wire_to_local(topic: str, wm: Any) -> Any:
    """Rebuild the LOCAL dataclass from a ref-free ``Wire*`` decoded off TCP.

    The ``Wire*`` here was encoded by :func:`local_to_full_wire`, so its former ref
    slots hold full ndarrays (decoded as 0x08). The result is the standard local
    dataclass the unchanged ``comms.converters.to_wire`` expects, so the receive
    bridge can write the arrays into the Mac-local rings.
    """
    if topic == topics.CAM_SYNC:
        return CamSync(seq=int(wm.seq), ts_ns=int(wm.ts_ns),
                       gray_left=wm.gray_left_ref,
                       gray_right=wm.gray_right_ref)
    if topic == topics.FRAME_DEPTH:
        return DepthFrame(seq=int(wm.seq), ts_ns=int(wm.ts_ns),
                          gray_left=wm.gray_left_ref, depth_m=wm.depth_ref)
    if topic == topics.IMUCAM_SAMPLE:
        return ImuCamPacket(seq=int(wm.seq), ts_ns=int(wm.ts_ns),
                            gray_left=wm.gray_left_ref,
                            gray_right=wm.gray_right_ref,
                            imu_ts=wm.imu_ts, gyro=wm.gyro, accel=wm.accel)
    if topic == topics.KEYFRAME:
        return Keyframe(seq=int(wm.seq), T_world_cam=wm.T_world_cam,
                        gray_left=wm.gray_left_ref, depth_m=wm.depth_ref,
                        track_ids=wm.track_ids, track_px=wm.track_px,
                        accel=wm.accel, inlier_ids=wm.inlier_ids)
    raise KeyError(f"full_wire_to_local: {topic!r} is not a ref-bearing topic")
