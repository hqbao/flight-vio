"""Runtime capability detection + device selection for the live OAK-D path.

The shipped flight stack must run unchanged across THREE physical devices that
present different hardware:

* OAK-D W           -- has an IMU (BNO086), OV9282 mono @ 1280x800,
* OAK-D Lite retail -- has an IMU (BMI270), OV7251 mono @ 640x480,
* OAK-D Lite Kickstarter -- has NO IMU at all.

Two facts make a fixed pipeline wrong for at least one of them:

1. ``imu.enableIMUSensor(...)`` on a device WITHOUT an IMU does NOT raise -- it
   silently produces no data and the first blocking read HANGS forever
   (luxonis/depthai #598). So a try/except around the IMU enable is useless; we
   must DETECT IMU presence up front (:func:`probe_capabilities`) and build the
   IMU node only when it is really there.
2. The requested mono resolution (``--width``/``--height``, defaulting to the
   OAK-D W's 640x400 working point) can exceed what a Lite's OV7251 supports.
   We clamp the request to the connected sensor's advertised maximum.

:func:`select_device` adds the operator-facing ``--model`` selector for the
multi-device case (several OAK devices plugged into one host): it enumerates the
available devices and opens exactly the one the operator named (by ``deviceId``
or by a product-name substring), or fails with a clear list of what it saw.

``depthai`` is imported LAZILY inside every function, so importing this module on
the offline path never pulls the device library -- the same discipline as
:mod:`imu_camera.device.oak_live`. Both public functions take injection seams
(``enumerator`` / ``opener``) so the selection logic is exercised fully offline
with fake device handles, no hardware required.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class DeviceCapabilities:
    """What the LIVE pipeline builder needs to know about the open device.

    Read once from an already-connected handle by :func:`probe_capabilities`:

    * ``name``      -- product model (e.g. ``"OAK-D-LITE"``), best-effort.
    * ``imu_type``  -- the connected IMU string (``"BMI270"`` / ``"BNO086"`` /
      ``""``); raw from ``getConnectedIMU`` for diagnostics/logging.
    * ``has_imu``   -- whether to build the IMU node at all (the load-bearing
      bit: see the module docstring on the no-IMU hang).
    * ``mono_max``  -- ``(width, height)`` the left/CAM_B mono sensor supports;
      the requested resolution is clamped to this.
    * ``device_id`` -- the unique mxid, the per-device calibration cache key.
    """

    name: str
    imu_type: str
    has_imu: bool
    mono_max: tuple[int, int]
    device_id: str


def _str_attr(dev, attr: str) -> str:
    """Best-effort string read of a no-arg device method; "" if unavailable.

    Defensive on purpose: a fake handle in the offline tests may not implement
    every method, and a real handle may raise on a transient link error -- in
    both cases an empty string is the safe, non-crashing default.
    """
    fn = getattr(dev, attr, None)
    if not callable(fn):
        return ""
    try:
        val = fn()
    except Exception:                                              # noqa: BLE001
        return ""
    return "" if val is None else str(val)


def _mono_max(dev) -> tuple[int, int]:
    """Max ``(width, height)`` the left/CAM_B mono sensor advertises.

    Reads ``getConnectedCameraFeatures()`` and, for the CAM_B socket, takes the
    largest resolution across that camera's ``configs`` (falling back to the
    feature's own ``width``/``height`` when no per-config list is exposed). Any
    failure -- no such method, no CAM_B entry, an empty config list -- falls back
    to ``(640, 400)``, the OAK-D W working point the pipeline used before this
    code existed, so a probe that can't read features never regresses behaviour.
    """
    fallback = (640, 400)
    try:
        import depthai as dai
        cam_b = dai.CameraBoardSocket.CAM_B
    except Exception:                                              # noqa: BLE001
        # No depthai (pure offline test with a fake handle): the fake exposes its
        # CAM_B socket value directly, so compare by the attribute below instead.
        cam_b = None

    get_feats = getattr(dev, "getConnectedCameraFeatures", None)
    if not callable(get_feats):
        return fallback
    try:
        feats = get_feats()
    except Exception:                                              # noqa: BLE001
        return fallback

    best: tuple[int, int] | None = None
    for f in feats or []:
        # Match the left mono socket. With depthai present compare against the
        # real CAM_B enum; offline (fake handle) compare against the fake's own
        # ``cam_b`` marker if it exposes one, else accept the single feature.
        sock = getattr(f, "socket", None)
        if cam_b is not None:
            if sock != cam_b:
                continue
        elif sock is not None and getattr(dev, "cam_b", sock) != sock:
            continue
        # Largest of the per-config resolutions; fall back to the feature dims.
        configs = getattr(f, "configs", None) or []
        cand: list[tuple[int, int]] = [
            (int(c.width), int(c.height)) for c in configs
            if getattr(c, "width", None) and getattr(c, "height", None)]
        if not cand:
            w, h = getattr(f, "width", None), getattr(f, "height", None)
            if w and h:
                cand = [(int(w), int(h))]
        for wh in cand:
            if best is None or wh[0] * wh[1] > best[0] * best[1]:
                best = wh
    return best if best is not None else fallback


def probe_capabilities(dev) -> DeviceCapabilities:
    """Read the runtime capabilities of an OPEN depthai device handle.

    ``dev`` is an already-connected ``dai.Device``. Everything is read
    defensively so a partial/fake handle (offline tests) yields sane defaults
    rather than raising:

    * ``has_imu`` -- true unless ``getConnectedIMU()`` is empty or ``"NONE"``
      (case-insensitive, whitespace-stripped). This is the gate the pipeline
      builder uses to decide whether to create the IMU node at all.
    * ``mono_max`` -- the left/CAM_B sensor's largest resolution (see
      :func:`_mono_max`), used to clamp the requested ``--width``/``--height``.
    """
    imu_type = _str_attr(dev, "getConnectedIMU").strip()
    has_imu = imu_type.upper() not in ("", "NONE")
    name = _str_attr(dev, "getDeviceName")
    device_id = (_str_attr(dev, "getDeviceId")
                 or _str_attr(dev, "getMxId") or "default")
    return DeviceCapabilities(name=name, imu_type=imu_type, has_imu=has_imu,
                              mono_max=_mono_max(dev), device_id=device_id)


def _close_quiet(dev) -> None:
    """Close a probe-only device handle, swallowing any teardown error."""
    close = getattr(dev, "close", None)
    if callable(close):
        try:
            close()
        except Exception:                                          # noqa: BLE001
            pass


def _info_id(info) -> str:
    """The pre-connect mxid of a ``DeviceInfo`` (``.deviceId``; "" if absent)."""
    val = getattr(info, "deviceId", None)
    return "" if val is None else str(val)


def select_device(model: str | None, *, enumerator=None, opener=None):
    """Enumerate the available OAK devices and OPEN exactly the requested one.

    Returns ``(open_device, seen)`` where ``open_device`` is the connected
    ``dai.Device`` to drive and ``seen`` is a diagnostic
    ``list[{"device_id", "name"}]`` describing every device considered (handy for
    logging which one was picked and what else was attached).

    Injection seams (both default to the real depthai entry points, both
    overridable so the whole selection tree is offline-testable with fakes):

    * ``enumerator()`` -> ``list[DeviceInfo]`` (default
      ``dai.Device.getAllAvailableDevices``).
    * ``opener(arg)``  -> open ``dai.Device`` (default the ``dai.Device``
      constructor); called with a ``DeviceInfo`` to target a specific device.

    Selection rules:

    * no devices found                 -> ``RuntimeError``.
    * ``model is None`` and exactly one -> open and return it (the common case).
    * ``model is None`` and many        -> open each to read its name/IMU for the
      message, close them all, ``RuntimeError`` listing ``name@device_id`` per
      device and instructing the operator to pass ``--model``.
    * ``model`` given                   -> first try an exact (case-insensitive)
      ``deviceId`` match and open just that one (cheap). Otherwise open each
      device in turn and keep the FIRST whose product name CONTAINS ``model``
      (case-insensitive); close the rest. No match -> close all, ``RuntimeError``
      listing what was seen. (A substring that matches several devices keeps the
      first and logs a warning naming the others -- the substring is the
      operator's explicit choice.)

    Every device opened only for probing is CLOSED via ``try/finally`` so a
    selection that ends up rejecting a device never leaks an open handle; only
    the chosen device stays open.
    """
    if enumerator is None or opener is None:
        import depthai as dai
        if enumerator is None:
            enumerator = dai.Device.getAllAvailableDevices
        if opener is None:
            opener = dai.Device

    infos = list(enumerator() or [])
    if not infos:
        raise RuntimeError("no OAK device found")

    # ---- model is None -------------------------------------------------- #
    if model is None:
        if len(infos) == 1:
            dev = opener(infos[0])
            seen = [{"device_id": _info_id(infos[0]),
                     "name": _str_attr(dev, "getDeviceName")}]
            return dev, seen
        # Several devices, no selector: probe each just enough to name it, then
        # close them all and tell the operator to disambiguate with --model.
        seen = []
        for info in infos:
            dev = opener(info)
            try:
                seen.append({"device_id": (_str_attr(dev, "getDeviceId")
                                           or _info_id(info)),
                             "name": _str_attr(dev, "getDeviceName")})
            finally:
                _close_quiet(dev)
        listing = ", ".join(f"{s['name'] or '?'}@{s['device_id'] or '?'}"
                            for s in seen)
        raise RuntimeError(
            f"{len(infos)} OAK devices connected ({listing}); "
            f"pass --model NAME (product-name substring or deviceId) to pick one")

    # ---- model given: exact deviceId fast path -------------------------- #
    want = model.strip()
    for info in infos:
        if _info_id(info).lower() == want.lower():
            dev = opener(info)
            seen = [{"device_id": (_str_attr(dev, "getDeviceId")
                                   or _info_id(info)),
                     "name": _str_attr(dev, "getDeviceName")}]
            return dev, seen

    # ---- model given: product-name substring scan ----------------------- #
    selected = None
    seen = []
    extra_matches: list[str] = []
    for info in infos:
        dev = opener(info)
        name = _str_attr(dev, "getDeviceName")
        dev_id = _str_attr(dev, "getDeviceId") or _info_id(info)
        seen.append({"device_id": dev_id, "name": name})
        is_match = want.lower() in name.lower()
        if is_match and selected is None:
            selected = dev                    # keep the FIRST match open
        else:
            if is_match:
                extra_matches.append(f"{name or '?'}@{dev_id or '?'}")
            _close_quiet(dev)

    if selected is None:
        listing = ", ".join(f"{s['name'] or '?'}@{s['device_id'] or '?'}"
                            for s in seen)
        raise RuntimeError(
            f"--model {model!r} matched no connected OAK device (saw: {listing})")
    if extra_matches:
        # Substring matched more than one device: the first is used, but name the
        # others so the operator can tighten --model if they meant a different one.
        print(f"[probe] --model {model!r} matched multiple devices; using the "
              f"first, ignored: {', '.join(extra_matches)}", file=sys.stderr)
    return selected, seen
