"""Fixed-shape ring of shared-memory slots for one image / depth stream.

The IPC bus (:mod:`comms.ipc`) carries only metadata across the wire; large
numpy arrays travel through a :class:`SharedArrayRing` -- a single pre-allocated
shared-memory segment holding ``slots`` identically-shaped frames at fixed byte
offsets, which the producer fills in rotation and the consumer reads by slot
index.

One segment per ring, slot = byte offset
----------------------------------------
The ring is ONE ``SharedMemory`` segment of size ``slots * nbytes``; slot ``i``
is the offset view ``[i * nbytes : (i + 1) * nbytes]``. This keeps the file-
descriptor cost at a small CONSTANT per ring, independent of ``slots`` -- the
earlier design allocated one ``SharedMemory`` segment PER slot, so a 64-slot
ring opened 64 segments and the fd cost scaled linearly with slots; an attaching
consumer of 3 rings could trip macOS's 256-fd default (``shm_open`` -> EMFILE,
``OSError: [Errno 24] Too many open files``) at boot. (CPython's
``SharedMemory(create=True)`` holds a small constant number of fds per segment
on macOS -- roughly two -- so the win is that fd/segment cost no longer scales
with slot count, not that a ring costs literally one fd.) Alignment is exact:
``nbytes`` is always a multiple of the dtype itemsize (uint8 -> 1; float32 ->
H*W*4 divisible by 4), so ``i * nbytes`` lands every slot view on a correctly
aligned boundary with no per-slot page padding (identical total RAM as the old
per-slot design).

Why a ring and not one-shot blocks
----------------------------------
A live OAK-D stream publishes ~20 frames/s into 3-4 subscribers (VIO, SLAM, UI,
maybe a tool). Allocating + unlinking a fresh ``SharedMemory`` per frame per
subscriber is far too slow (each ``shared_memory.SharedMemory("name", create=True)``
takes ~1-5 ms on macOS). A pre-allocated ring of ``N=8`` slots gives 0.4 s of
slack at 20 fps -- well above the 50-60 ms latest-only inbox cadence downstream,
so the producer rotation never catches a still-reading consumer (the consumer
copies out within its inbox handler, then the slot is free to reuse).

Concurrency model
-----------------
SINGLE-PRODUCER, MULTI-CONSUMER (publishers are processes, not threads -- the
capture process is the only writer to each ring). No locks: the producer
advances ``slot = seq % N`` monotonically. Consumers receive ``(slot, shape,
dtype)`` in the wire metadata; the bus publishes the metadata only AFTER the
slot has been fully written, so by the time a consumer reads the slot it is
already coherent (writes happen-before the encoded send on the socket; the
recv-side decode happens-after, all under the connection's TCP-like ordering
guarantee on the local socket).

Worst case: an extremely slow consumer is N frames behind, reads stale data.
That matches the downstream "latest-only" inbox semantics already used by the
live pipeline (the keypoints/triplet/odometry latest_only inboxes coalesce
backlog on purpose), so a stale read just drops a frame the consumer would have
discarded anyway.

Lifecycle
---------
The producer creates the ring with ``SharedArrayRing.create(name, slots, shape,
dtype)`` -- this allocates the single underlying :class:`SharedMemory` block and
returns a handle that knows how to close + unlink it. Consumers attach to an
existing ring with ``SharedArrayRing.attach(name, slots, shape, dtype)`` -- they
do NOT unlink, only close. On Linux/macOS shared memory persists until every
attached process closes it AND the creator unlinks it.

The owning ``capture`` process is responsible for ``unlink`` at shutdown; if it
crashes the OS cleans up shared memory at the next reboot (acceptable for a
desktop dev tool; production would add an atexit fallback).
"""
from __future__ import annotations

import atexit
import struct
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class SharedArrayRef:
    """Wire reference to one slot of a :class:`SharedArrayRing`.

    Travels inside an IPC wire message; the receiver uses it to copy the slot's
    contents out of shared memory into a private ``np.ndarray`` before any
    downstream step runs.
    """

    ring_name: str
    slot: int
    shape: tuple[int, ...]
    dtype: str               # numpy dtype name, e.g. "uint8" / "float32"


class SharedArrayRing:
    """A ring of ``slots`` identically-shaped frames in ONE shared-memory segment.

    Slot ``i`` is the byte range ``[i * nbytes : (i + 1) * nbytes]`` of the single
    backing segment (see the module docstring for the constant-fd-per-ring
    rationale -- one segment per ring, fd cost independent of ``slots``).
    Use :meth:`create` on the producer side, :meth:`attach` on each consumer.
    """

    def __init__(self, name: str, slots: int, shape: tuple[int, ...],
                 dtype: np.dtype, _shm, _owner: bool) -> None:
        self.name = name
        self.slots = int(slots)
        self.shape = tuple(int(s) for s in shape)
        self.dtype = np.dtype(dtype)
        self._shm = _shm                  # single SharedMemory block, size slots*nbytes
        self._owner = bool(_owner)
        #: Set after the first successful :meth:`unlink` so a stray atexit
        #: callback (defence in depth -- see :meth:`create`) is a no-op.
        self._unlinked = False
        # Pre-build np.ndarray views (one per slot) as fixed-offset windows into
        # the single block, so the hot publish/poll path only does a memcpy, no
        # fresh ndarray construction. nbytes is a multiple of the dtype itemsize,
        # so `i * nbytes` keeps every view correctly aligned.
        nbytes = int(np.prod(self.shape)) * self.dtype.itemsize
        self._views: list[np.ndarray] = [
            np.ndarray(self.shape, dtype=self.dtype,
                       buffer=self._shm.buf, offset=i * nbytes)
            for i in range(self.slots)
        ]

    # ------------------------------------------------------------------ #
    # Factories
    # ------------------------------------------------------------------ #
    @classmethod
    def create(cls, name: str, slots: int, shape: Iterable[int],
               dtype) -> "SharedArrayRing":
        """Allocate the single ``slots * nbytes`` shared block. Producer-side.

        The block's :class:`SharedMemory` is named exactly ``name`` (no per-slot
        suffix); slot ``i`` is the offset view ``[i * nbytes : (i + 1) * nbytes]``
        so consumers attach once and index by offset.
        """
        shape = tuple(int(s) for s in shape)
        dt = np.dtype(dtype)
        nbytes = int(np.prod(shape)) * dt.itemsize
        # macOS limits POSIX shared-memory names to ~31 chars (PSHMNAMLEN, incl.
        # the leading '/'). The name is now just `{name}` (no `.{i}` suffix), so
        # gate on the bare name. Fail loudly here rather than getting a cryptic
        # ENAMETOOLONG from shm_open. Linux is much higher (NAME_MAX 255), so
        # this is the lower of the two and the right gate.
        if len(name) > 30:
            raise ValueError(
                f"shared-memory name {name!r} too long: {len(name)} chars; "
                f"macOS limit is 30. Use a shorter endpoint / stream name.")
        try:
            shm = shared_memory.SharedMemory(
                name=name, create=True, size=slots * nbytes)
        except FileExistsError as e:
            # Stale ring from a previous run that crashed without unlink.
            # Re-raise so the caller can cleanup_stale + retry. (Nothing partial
            # to clean up now -- there is just the one block, and shm_open never
            # created it on the EEXIST path.)
            raise RuntimeError(
                f"shared memory ring {name!r} already exists -- "
                f"call SharedArrayRing.cleanup_stale({name!r}, {slots}) first") from e
        ring = cls(name, slots, shape, dt, shm, _owner=True)
        # Defence in depth: register an atexit fallback so an unhandled exception
        # path (or any creator-side teardown that forgets `unlink`) still frees
        # the shared blocks instead of leaking them as
        # `resource_tracker: There appear to be N leaked shared_memory objects`.
        # The fallback can't save us from SIGKILL (atexit doesn't run there) --
        # only the clean SIGTERM / exception paths -- but combined with the
        # SIGTERM handlers in the process mains this closes the window.
        # `_safe_unlink` is idempotent (guarded by `_unlinked`) so it is a no-op
        # when the caller has already unlinked explicitly.
        atexit.register(ring._safe_unlink)
        return ring

    @classmethod
    def attach(cls, name: str, slots: int, shape: Iterable[int],
               dtype) -> "SharedArrayRing":
        """Open an existing ring created by another process. Consumer-side.

        Passes ``track=False`` to :class:`SharedMemory` so the attaching
        process's :mod:`multiprocessing.resource_tracker` does NOT claim
        ownership of the block (the creator already does). Without this, the
        attacher would print "leaked shared_memory" warnings on exit even
        though only the creator should unlink -- a long-standing footgun in
        Python's stdlib (`issue38119 <https://bugs.python.org/issue38119>`_,
        fixed in 3.13 via the ``track`` parameter).
        """
        shape = tuple(int(s) for s in shape)
        dt = np.dtype(dtype)
        shm = shared_memory.SharedMemory(name=name, create=False, track=False)
        return cls(name, slots, shape, dt, shm, _owner=False)

    @staticmethod
    def cleanup_stale(name: str, slots: int) -> None:
        """Unlink the leftover shared block from a previous run that crashed.

        ``slots`` is retained for API compatibility but no longer used for naming
        (the block is named exactly ``name``). Best-effort; a missing block is
        silently skipped (the normal case).
        """
        del slots                          # kept for API compat; not used now
        try:
            shm = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            return
        try:
            shm.close()
            shm.unlink()
        except Exception:                                          # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    # Producer / consumer ops
    # ------------------------------------------------------------------ #
    def slot_for(self, seq: int) -> int:
        """The slot index the producer uses for a given monotonic ``seq``."""
        return int(seq) % self.slots

    def write(self, slot: int, arr: np.ndarray) -> SharedArrayRef:
        """Copy ``arr`` into slot ``slot``; return the wire reference.

        ``arr`` must already be the ring's shape + dtype (the producer is
        expected to allocate the camera/depth at that shape, no per-frame
        reshaping). Raises ``ValueError`` otherwise so a wiring bug is caught at
        the boundary, not silently corrupted in shared memory.
        """
        if arr.shape != self.shape:
            raise ValueError(
                f"ring {self.name!r} shape {self.shape} != arr {arr.shape}")
        if arr.dtype != self.dtype:
            raise ValueError(
                f"ring {self.name!r} dtype {self.dtype} != arr {arr.dtype}")
        np.copyto(self._views[int(slot)], arr, casting="no")
        return SharedArrayRef(self.name, int(slot), self.shape, str(self.dtype))

    def read_copy(self, ref: SharedArrayRef) -> np.ndarray:
        """Return a private copy of the slot referenced by ``ref``.

        Always copies -- the caller owns the result and may keep it past the
        next producer rotation. Cheap (~0.1 ms for 640x400 uint8).
        """
        if ref.ring_name != self.name:
            raise ValueError(
                f"ref ring {ref.ring_name!r} != this ring {self.name!r}")
        return self._views[int(ref.slot)].copy()

    # ------------------------------------------------------------------ #
    # Teardown
    # ------------------------------------------------------------------ #
    def close(self) -> None:
        """Detach this process from the shared block (idempotent).

        Does NOT unlink -- only :meth:`unlink` (creator) destroys the memory.
        Consumers always call :meth:`close` only.
        """
        try:
            self._shm.close()
        except Exception:                                          # noqa: BLE001
            pass

    def unlink(self) -> None:
        """Destroy the underlying shared block. Creator-only, idempotent.

        After :meth:`unlink` no further reads / writes succeed. Always pair
        :meth:`close` after :meth:`unlink` in the creator to free the local
        handle. Sets ``self._unlinked`` so the atexit fallback registered in
        :meth:`create` becomes a no-op once the caller has cleaned up.
        """
        if not self._owner or self._unlinked:
            return
        self._unlinked = True
        try:
            self._shm.unlink()
        except FileNotFoundError:
            pass
        except Exception:                                          # noqa: BLE001
            pass

    def _safe_unlink(self) -> None:
        """atexit fallback: unlink wrapped in try/except so interpreter teardown
        never raises out of the registered callback.

        Idempotent via :attr:`_unlinked`; called automatically when the
        interpreter exits normally (clean exit, SIGTERM with finally-block,
        unhandled exception). Does NOT run on SIGKILL or os._exit -- those paths
        rely on the OS / next reboot to reclaim the shared blocks.
        """
        try:
            self.unlink()
        except Exception:                                          # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Helpers: SharedArrayRef is a plain dataclass. The ring itself is process-local
# and never sent across the wire; the producer creates one, the consumer attaches
# its own (same name + dims) and they exchange only refs (encoded by the codec).
# --------------------------------------------------------------------------- #
def pack_ref(ref: SharedArrayRef) -> bytes:
    """Pack a ref into a compact binary (for cases that want to avoid the codec).

    Layout: 1B name-len | name | 4B slot | 2B ndim | ndim x 4B shape | 1B dtype-len | dtype.
    Used by callers when bandwidth is a concern; the default IPC path just lets
    the codec encode ``SharedArrayRef`` and that is fast enough.
    """
    name = ref.ring_name.encode("utf-8")
    dt = ref.dtype.encode("utf-8")
    shape = ref.shape
    parts = [
        struct.pack("!B", len(name)),
        name,
        struct.pack("!I", int(ref.slot)),
        struct.pack("!H", len(shape)),
        *(struct.pack("!I", int(s)) for s in shape),
        struct.pack("!B", len(dt)),
        dt,
    ]
    return b"".join(parts)
