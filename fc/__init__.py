"""``fc`` -- the flight-controller UART output project (consumer-only sibling).

The sixth sibling process. It subscribes to the ``vio`` process over IPC
(``pose.odom`` + the retained ``calib.bundle`` readiness barrier), converts the
VIO earth-frame pose to NED via the shared SSOT
(:func:`sky.fc.fc_earth_pose.earth_pose_from_T_world_cam`), and streams it to a
drone flight controller over UART as a ``dblink`` ``DB_CMD_VISION_POSE`` frame
(the FC's in-house wire protocol, packed by :mod:`sky.fc.dblink`).

CONSUMER-ONLY: unlike every other project, ``fc`` opens NO IPC server and
PUBLISHES nothing -- it is a pure sink. It still vendors the FULL ``comms``
contract bit-identically (a ``diff -r`` gate enforces byte-parity vs
``imu_camera/comms``); it simply never instantiates the server / publisher half.

Built by replicating the PROVEN ``slam`` template (calib-barrier on the vio
endpoint, then subscribe), with the SLAM worker / pose-graph replaced by a
latest-wins UART sender thread. The conversion lives in the shared leaf
:mod:`sky.fc`, NOT in the frozen ``comms/lib/misc/frames.py``.
"""
