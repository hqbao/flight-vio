"""``comms.lib.misc`` -- shared, dependency-light helpers (vendored).

These are the small cross-cutting utilities that are neither a VIO algorithm nor
part of the module architecture, grouped here so the library root holds only
packages:

    frames    NED/FRD/optical frame conventions + rigid-body transforms
    geometry  RGB-D back-projection primitives (pure numpy)
    pose      Pose dataclass + fixed-size trajectory ring buffer
    pngio     stdlib-only 8-bit grayscale PNG codec (record/replay frames)

Import the submodules directly, e.g. ``from comms.lib.misc.pose import Pose`` or
``from comms.lib.misc import frames``. These modules are numpy + stdlib only --
import is guarded so it never pulls math (or breaks a headless comms import).
"""
