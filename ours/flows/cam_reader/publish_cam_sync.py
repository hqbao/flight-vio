"""``publish_cam_sync`` task: emit one stereo pair as the IMU sync trigger."""
from __future__ import annotations

from ...lib.flow import topics
from ...lib.flow.messages import CamSync
from ...lib.flow.task import Task


class PublishCamSync(Task):
    name = "publish_cam_sync"

    def run(self, ctx, msg: CamSync):
        ctx.bus.publish(topics.CAM_SYNC, msg)
        return None
