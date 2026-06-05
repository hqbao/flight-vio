"""``ours.legacy`` -- transitional, pre-flow code kept until superseded.

Holds the monolithic ``OakOursVioSource`` (``depthai_ours_vio``) -- the original
single-file live OAK-D VIO that predates the flow decomposition in
``ours.flows``. It is NOT dead: tools still expose it as ``--source ours-legacy``
and it serves as the rotation-prior convention oracle that ``ours.flows.capture``
is validated against. It will be removed once the live flow graph is verified on
the bench.
"""
