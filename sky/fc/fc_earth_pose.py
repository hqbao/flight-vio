"""The ONE earth-frame pose conversion: optical-world ``T_world_cam`` -> NED.

This is the single source of truth (SSOT) for turning the VIO pose -- expressed
as ``T_world_cam`` in the pipeline's gravity-aligned OPTICAL world frame (camera
OpenCV axes: X=right, Y=down, Z=forward) -- into the FC's NED earth frame
(X=North, Y=East, Z=Down) plus the body attitude in the FRD airframe convention
(X=Forward, Y=Right, Z=Down). Both the UI viewer (``ui/main.py``) and the ``fc``
UART sender consume it, so they can never drift apart.

It is PURE and STATELESS: pose in, earth-pose out. NO time, NO I/O, NO counters,
NO sequence numbers -- those live in the consumers (the UI's velocity
finite-difference, the FC sender's ``reset_counter`` / staleness). That keeps
this a leaf of the ``sky.*`` library (numpy + :mod:`sky.math` only) and trivially
testable against known poses.

Frame algebra
-------------
Two fixed matrices, carried VERBATIM (byte-identical) from the verified UI path:

* ``_M_OPT_TO_NED`` = the WORLD rotation ``C_ned_optw``: it re-expresses a vector
  given in the optical-world basis in the NED basis. The optical world is
  gravity-aligned with X=right, Y=down, Z=forward, so this maps
  forward->North, right->East, down->Down.
* ``_P_OPT_TO_FRD`` = the NOMINAL camera-OpenCV -> FRD body axis swap. It maps an
  FRD body vector to its OpenCV-camera-body representation (its columns are the
  FRD basis axes written in OpenCV coords). For the nominal forward-facing mount
  this is exactly ``R_body_cam_nominal.T``.

Position::    pos_ned = M @ pos_opt

Attitude::    R_ned = M @ R_opt @ P @ R_body_cam.T

  where ``R_opt`` is the rotation part of ``T_world_cam`` (the OpenCV-camera body
  axes written in the optical world). ``M @ R_opt`` re-expresses those body axes
  in NED; ``@ P`` swaps the body convention from OpenCV-camera to the NOMINAL FRD
  airframe; ``@ R_body_cam.T`` applies the operator's EXTRA physical mount tilt of
  the camera relative to that nominal forward mount.

Where ``R_body_cam`` enters (default identity)
----------------------------------------------
``R_body_cam`` is the EXTRA mount-tilt rotation that maps the actual camera-OpenCV
body axes to the FRD airframe body axes ``v_frd = R_body_cam @ v_cam`` -- i.e. the
rotation from "where the camera nominally points (forward, level)" to "where it is
physically bolted". The default ``R_body_cam = I`` means the camera sits in the
nominal forward-facing mount, and then ``R_ned = M @ R_opt @ P`` reproduces the UI
path BIT-FOR-BIT. A non-identity ``R_body_cam`` (e.g. a 20-degree pitched-down
gimbal mount) right-multiplies as ``P @ R_body_cam.T`` so the reported FRD attitude
is rotated by the mount. (The full opencv<->FRD swap stays baked into ``P``; the
operator only supplies the delta from the nominal mount, hence the identity
default -- a forward-facing camera needs no config.)
"""
from __future__ import annotations

import numpy as np

from sky.math import rot_to_quat

# Camera optical (X=right, Y=down, Z=forward) world -> NED earth, carried VERBATIM
# from ui/main.py so the two code paths display / send the SAME convention.
# _M_OPT_TO_NED = C_ned_optw (world basis change); _P_OPT_TO_FRD reorders the body
# attitude columns from OpenCV-camera to the nominal FRD airframe.
_M_OPT_TO_NED = np.array([[0.0, 0.0, 1.0],
                          [1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0]])
_P_OPT_TO_FRD = np.array([[0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0],
                          [1.0, 0.0, 0.0]])

#: Reused identity for the default (nominal forward-facing) mount.
_I3 = np.eye(3, dtype=np.float64)


def earth_pose_from_T_world_cam(
    T_world_cam: np.ndarray,
    R_body_cam: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert ``T_world_cam`` (optical world) to the FC's NED earth pose.

    Args:
        T_world_cam: ``(4, 4)`` rigid transform, camera->optical-world (the VIO
            pose). The translation column is the camera position in the
            gravity-aligned optical world; the rotation block is ``R_opt`` (the
            OpenCV-camera body axes expressed in that world).
        R_body_cam: ``(3, 3)`` EXTRA mount-tilt rotation, OpenCV-camera body ->
            FRD airframe body (``v_frd = R_body_cam @ v_cam``), relative to the
            NOMINAL forward-facing mount. ``None`` / identity (the default) ==
            nominal mount, and the result is byte-identical to the UI path.

    Returns:
        ``(pos_ned, q_ned_wxyz, R_ned)`` -- position in NED metres ``(3,)``, the
        FRD-body attitude as a unit quaternion ``(w, x, y, z)`` ``(4,)``, and the
        same attitude as a ``(3, 3)`` rotation matrix (body->NED). Heading
        (yaw) is RELATIVE: the optical world's gravity-aligned X axis defines
        North, so without a magnetometer "North" is the heading at init.
    """
    T = np.asarray(T_world_cam, dtype=np.float64)
    R_bc = _I3 if R_body_cam is None else np.asarray(R_body_cam, dtype=np.float64)

    pos_opt = T[:3, 3]
    R_opt = T[:3, :3]
    pos_ned = _M_OPT_TO_NED @ pos_opt
    # R_body_cam is the EXTRA tilt on top of the nominal opencv->FRD swap P, so it
    # right-multiplies as P @ R_body_cam.T (R_body_cam=I -> the verified UI form
    # M @ R_opt @ P exactly).
    R_ned = _M_OPT_TO_NED @ R_opt @ _P_OPT_TO_FRD @ R_bc.T
    q_ned = rot_to_quat(R_ned)
    return pos_ned, q_ned, R_ned


# --------------------------------------------------------------------------- #
# Camera-mount extrinsic constructor (the R_body_cam argument above)
# --------------------------------------------------------------------------- #
# Derived + verified by math-reviewer (roll-invariance certified to 5.7e-14 deg
# over 3000 random attitudes x 6 rolls x 7 mounts). R_body_cam is the EXTRA mount
# rotation, expressed in the FRD body frame, on top of the nominal forward mount
# baked into P. The nominal optical axis = FRD +X (fwd), image-right = +Y, image-
# down = +Z. Three elementary rotations IN THE FRD FRAME, in this exact order:
#     R_bc = Rz(azimuth) @ Ry(-down_tilt) @ Rx(image_roll)
# NOTE the NEGATIVE down_tilt (about FRD +Y/right, +rotation lifts the axis UP, so
# "down" needs -tilt). Returned R_bc is passed DIRECTLY to earth_pose_from_T_world_cam
# (which transposes it as P @ R_bc.T) -- do NOT pre-transpose.

def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def R_body_cam_from_angles(azimuth_deg: float, down_tilt_deg: float,
                           image_roll_deg: float = 0.0) -> np.ndarray:
    """Build ``R_body_cam`` from intuitive mount angles (all in the FRD body frame).

    Args:
        azimuth_deg: horizontal direction the optical axis points, about FRD +Z
            (down), CLOCKWISE viewed from above: 0=forward, +90=right, 180=back,
            270(=-90)=left.
        down_tilt_deg: depression of the optical axis BELOW horizontal: 0=level,
            90=straight down.
        image_roll_deg: rotation about the optical axis (which way is image-"up").
            At down_tilt≈90 the optical axis is vertical and azimuth aliases into
            this DOF -- set azimuth=0 and put the mount yaw here.

    Returns:
        ``(3, 3)`` ``R_body_cam`` (OpenCV-camera body -> FRD airframe body, the
        EXTRA rotation vs the nominal forward mount). Pass directly to
        :func:`earth_pose_from_T_world_cam`. Makes the recovered heading
        roll-invariant for any physical mount (math-reviewer APPROVE).
    """
    az = np.deg2rad(float(azimuth_deg))
    tilt = np.deg2rad(float(down_tilt_deg))
    roll = np.deg2rad(float(image_roll_deg))
    return _rz(az) @ _ry(-tilt) @ _rx(roll)


#: Named mount presets -> (azimuth_deg, down_tilt_deg, image_roll_deg). Build the
#: matrix via :func:`R_body_cam_from_angles`. "down" leaves azimuth at 0 (it would
#: alias into image_roll); set image_roll if a specific image heading is wanted.
MOUNT_PRESETS: dict[str, tuple[float, float, float]] = {
    "forward":          (0.0,   0.0, 0.0),
    "backward":         (180.0, 0.0, 0.0),
    "forward-down-45":  (0.0,  45.0, 0.0),
    "backward-down-45": (180.0, 45.0, 0.0),
    "down":             (0.0,  90.0, 0.0),
}
