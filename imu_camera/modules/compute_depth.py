"""``compute_depth`` step: run the SGM matcher on a raw stereo pair."""
from __future__ import annotations

import numpy as np

from imu_camera.comms.messages import DepthFrame, ImuCamPacket
from imu_camera.comms.runtime import NUMBA_PARALLEL_LOCK
from sky.depth.stereo import SGMStereoMatcher


def compute_depth(matcher: SGMStereoMatcher, msg: ImuCamPacket) -> DepthFrame:
    """Dense SGM depth for the packet's stereo pair.

    Was ``ComputeDepthStep(Step)``; the matcher (was ``ctx.state["matcher"]``) is
    now an explicit arg. Logic byte-identical.
    """
    with NUMBA_PARALLEL_LOCK:        # SGM uses numba parallel=True
        # Returns the tracking-grid left + metric depth on the SAME grid.
        # For a replay matcher (rectify_left off) the left passes through
        # unchanged (uint8); for the live matcher (rectify_left on, raw
        # cameras) `LeftRectifier.rectify` returns FLOAT32 (bilinear-warped),
        # which doesn't fit the uint8 `gray_left` ring + KLT expects uint8.
        # Cast it back here -- the math precision is already gone after the
        # bilinear interp, the storage just needs to match the contract.
        gray_track, depth = matcher.dense_depth_rectified_left(
            msg.gray_left, msg.gray_right)
    if gray_track.dtype != np.uint8:
        gray_track = np.clip(gray_track, 0.0, 255.0).astype(np.uint8)
    return DepthFrame(msg.seq, msg.ts_ns, gray_track, depth)
