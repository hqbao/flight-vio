"""capture flow: ingest sensor data and publish it onto the bus.

Two interchangeable sources publish the *same* topics so every downstream flow
is identical live or offline:

* :class:`ReplayCaptureFlow` -- replays a recorded session (offline validation).
* :class:`LiveCaptureFlow`   -- the OAK-D device (live; validated on hardware).
"""
from .replay import ReplayCaptureFlow

__all__ = ["ReplayCaptureFlow"]
