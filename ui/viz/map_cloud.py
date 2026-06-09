"""UI-only SLAM-map cloud helpers (NOT in the vendored comms package).

The vendored ``ui.comms.lib.misc.geometry`` is byte-identical across all five
projects and must NOT be edited, so any UI-specific cloud post-processing lives
here. The SLAM 3D-map viewer shows ONE point per LANDMARK (KLT track id) that was
a PnP INLIER across a run of SUCCESSIVE keyframes -- i.e. a sparse, ID-based
landmark map (like a real SLAM map), not a dense-depth reconstruction. The single
primitive that gate needs is the longest run of consecutive keyframe indices a
landmark was an inlier in, which this module provides.

Why a consecutive-run gate
--------------------------
A landmark seen in scattered, non-adjacent keyframes (or only a keyframe or two)
is an uncertain, transient feature. The viewer instead wants *confirmed*,
motion-validated landmarks: a track that survived as a PnP inlier across a run of
SUCCESSIVE keyframes. :func:`longest_consecutive_run` reduces a landmark's set of
inlier-keyframe indices to the length of its longest such run, so the caller can
keep only landmarks whose run reaches a threshold.
"""
from __future__ import annotations


def longest_consecutive_run(sorted_unique_ints) -> int:
    """Length of the longest run of consecutive integers in a sorted unique seq.

    ``sorted_unique_ints`` is an ascending sequence of DISTINCT integers (e.g. the
    set of 0-based keyframe indices in which a landmark was a PnP inlier). A "run"
    is a maximal stretch of integers each exactly one greater than the last; this
    returns the length of the longest such run.

    Examples (the caller's persistence semantics):

    * ``{0, 1, ..., 24}``  -> 25  (a 25-keyframe consecutive streak).
    * ``{0, 5, 40}``       -> 1   (no two are adjacent).
    * ``{0, 1, ..., 18}``  -> 19  (just short of a 20-threshold).
    * ``{0..9, 20..29}``   -> 10  (two runs of 10; the longest sub-run wins).
    * ``{}``               -> 0   (empty -> no run).

    Pure Python (the input is at most ``#keyframes`` integers per landmark, so a
    single linear scan is more than fast enough -- no numpy needed). The caller is
    responsible for passing a SORTED, DEDUPED sequence (a keyframe that hit the
    landmark twice must count once); duplicates would break the +1 step counting.
    """
    longest = 0
    run = 0
    prev = None
    for v in sorted_unique_ints:
        v = int(v)
        if prev is not None and v == prev + 1:
            run += 1                       # extends the current consecutive streak
        else:
            run = 1                        # starts a fresh run (gap or first value)
        if run > longest:
            longest = run
        prev = v
    return longest
