"""The single source of truth for WHICH topics cross the network.

Forward (Pi) and receive (Mac) MUST agree on the same allowlist, the same
image-vs-POD split, and the same retained set, so this lives in one module both
import. The allowlist is UI-needed topics only -- the bridge forwards exactly what
a Mac UI consumes, nothing more (no internal VIO-path FIFO traffic the UI never
reads).

Three classes, per the design:

* **POD / full-ndarray** -- pose + overlay + diagnostic topics whose arrays ride
  the codec inline (no shared-memory ring). Reliable delivery.
* **image (shm)** -- topics whose large arrays live in a ``SharedArrayRing`` on the
  producer. On the Pi these are ``SharedArrayRef`` locally; the forward bridge
  resolves them to REAL ndarrays before they hit the wire (the 0x09 -> 0x08
  re-materialisation). Forwarded latest-wins (drop stale on a WiFi stall).
* **retained** -- one-shot config (``calib.bundle`` / ``calib.stereo`` / ``vio.map``)
  the TCP server caches + replays to a late subscriber. Reliable delivery.

Endpoint mapping: each topic is produced by exactly one of the three canonical
endpoints (capture / vio / slam). Forward subscribes the local endpoints; receive
re-serves the SAME canonical endpoint names so the UI attaches unchanged.
"""
from __future__ import annotations

from netbridge.comms import topics

# --------------------------------------------------------------------------- #
# Per-endpoint topic sets. The endpoint name is the LOGICAL role; forward maps it
# to the Pi-local endpoint and receive maps it to the Mac-served endpoint.
# --------------------------------------------------------------------------- #

#: capture endpoint (``oak.capture``): the synced camera/IMU sample, the rectified
#: stereo pair, the depth frame, the retained calib, and the raw IMU stream the
#: calib dialogs read.
CAPTURE_POD: tuple[str, ...] = (
    topics.IMU_RAW,                 # gyro/accel dialogs
)
CAPTURE_IMAGE: tuple[str, ...] = (
    topics.IMUCAM_SAMPLE,           # triplet + stereo-calib + epipolar (left+right)
    topics.CAM_SYNC,                # stereo pair
    topics.FRAME_DEPTH,             # triplet + keypoint depth overlay
)
CAPTURE_RETAINED: tuple[str, ...] = (
    topics.CALIB_BUNDLE,            # sizes the Mac rings (W/H) -- MUST arrive first
    topics.CALIB_STEREO,            # epipolar / rectification window
)

#: vio endpoint (``oak.vio``): the trajectory lines + per-frame overlays + the
#: keyframe stream + the opt-in diagnostic snapshots, plus the retained vio.map.
VIO_POD: tuple[str, ...] = (
    topics.POSE_ODOM,               # live VIO marker + trail
    topics.POSE_VO,                 # pure-vision VO line
    topics.POSE_REFINED,            # windowed-BA line
    topics.FRAME_TRACKS,            # keypoint tracker overlay
    topics.FRAME_INLIERS,           # keypoint tracker inlier marks
    topics.FRAME_GYROFUSE,          # gyro-fusion strip chart
    topics.BA_WINDOW,               # BA Window visualiser (opt-in --ba-window)
    topics.FRAME_FRONTEND,          # Frontend Internals (opt-in --frontend-viz)
)
VIO_IMAGE: tuple[str, ...] = (
    topics.KEYFRAME,                # SLAM map room view + loop-closure grays
)
VIO_RETAINED: tuple[str, ...] = (
    # The UI awaits ``calib.bundle`` on the VIO endpoint (vio.main republishes the
    # capture bundle); it MUST be re-served here too or ui._await_calib_bundle
    # times out. calib FIRST so it leads the retained replay.
    topics.CALIB_BUNDLE,
    topics.VIO_MAP,                 # refined-map snapshot read directly by the UI
)

#: slam endpoint (``oak.slam``): the live keyframe overlay + loop funnel + the
#: loop-correction event. All pure POD.
SLAM_POD: tuple[str, ...] = (
    topics.SLAM_MAP,                # continuous keyframe overlay (cyan line + dots)
    topics.SLAM_LOOP,              # loop-candidate match funnel
    topics.LOOP_CORRECTION,         # loop-closure correction event
)
SLAM_IMAGE: tuple[str, ...] = ()
SLAM_RETAINED: tuple[str, ...] = (
    # The UI awaits ``calib.bundle`` on the SLAM endpoint too (slam.main
    # re-broadcasts VIO's bundle); re-serve it here or the UI handshake stalls.
    topics.CALIB_BUNDLE,
)


# --------------------------------------------------------------------------- #
# Aggregations the forward/receive code iterates over.
# --------------------------------------------------------------------------- #
#: Logical role -> (pod, image, retained) topic tuples.
BY_ROLE: dict[str, tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...]]] = {
    "capture": (CAPTURE_POD, CAPTURE_IMAGE, CAPTURE_RETAINED),
    "vio":     (VIO_POD, VIO_IMAGE, VIO_RETAINED),
    "slam":    (SLAM_POD, SLAM_IMAGE, SLAM_RETAINED),
}

#: Topics that have NO ``comms.converters`` entry: they travel as their ``Wire*``
#: form DIRECTLY (capture/vio publish them with ``server.publish(topic, wire)`` and
#: the UI reads them straight off the wire -- ``to_local`` / ``to_wire`` would
#: KeyError on them). These are exactly the retained one-shot config topics. The
#: bridge must therefore forward + re-serve them as raw wire, never through the
#: converter-based ``IPCSubscriber`` / ``IPCPublisher`` path.
DIRECT_WIRE_TOPICS: frozenset[str] = frozenset(
    CAPTURE_RETAINED + VIO_RETAINED + SLAM_RETAINED)


def all_topics(role: str) -> list[str]:
    """Every allowlisted topic for ``role`` (pod + image + retained), in order.

    Order matters for retained replay: retained topics are declared so the server
    can replay calib FIRST. The full list is what forward subscribes locally and
    what receive subscribes over TCP.
    """
    pod, image, retained = BY_ROLE[role]
    # retained first so calib leads the replay; then image, then the rest of POD.
    return list(retained) + list(image) + list(pod)


def image_topics(role: str) -> set[str]:
    """The image (shm-backed) topics for ``role`` -- forwarded latest-wins."""
    return set(BY_ROLE[role][1])


def retained_topics(role: str) -> set[str]:
    """The retained (one-shot config) topics for ``role`` -- cached + replayed."""
    return set(BY_ROLE[role][2])
