# ADR 0001 — Extract the VIO windowed-BA backend into a 6th process (`ba/`)

- **Status:** Accepted — shipped 2026-06-18 (uncommitted). Supersedes the in-VIO
  `BackendModule` layout described in `docs/PROC4_ARCHITECTURE.md` §5.2 / §5.5.
- **Tier:** T2 (touches the tight feed-forward → flight-safety pose; gated as such).
- **Deciders:** architecture-reviewer (REQUEST_CHANGES → corrected design APPROVED).

> First ADR in the repo. Format: Context → Decision → Consequences → Rejected
> alternatives → Status, per the team doc policy.

---

## Context

The live stack was **5 projects** (`imu_camera`, `vio`, `slam`, `ui` + `launcher`,
plus the inline `depth/` source tree and the optional `netbridge/`). The windowed
bundle adjustment ran INSIDE the `vio` process as a `BackendModule` on the VIO local
bus: `OdometryModule` produced keyframes, `BackendModule` (`run_ba`) solved the
sliding window — loose `sky.backend.windowed.WindowedBAMap` or tight
`sky.vio.window.WindowedVIOMap` — and published `pose.refined` (the VIO-BA line) on
the VIO endpoint. SLAM was already a separate process consuming the same `keyframe`.

Three forces pushed BA out of `vio`:

1. **Architectural cleanliness / port boundary.** The five projects each map onto a
   `libsky*` port target; the backend solver is a distinct concern from the live
   front-end (KLT → PnP → gyro fuse → dead-reckon). Co-locating them in one process
   blurs that boundary.
2. **Fault isolation.** A diverging or wedged BA solve should not be able to stall
   the VIO front-end (which owns the live `pose.odom` the FC consumes).
3. **A clean lifecycle.** BA runs on the keyframe cadence, not the frame cadence; it
   deserves its own process boundary, drain, and health surface.

**This is NOT a performance decision.** `--worker` already ran the BA solve GIL-free
in a subprocess, so the GIL-escape motivation was already satisfied. Critically, the
architecture-reviewer **falsified the original premise** that extraction risked the
offline byte-parity oracle: the ATE oracle (`verification/oracle_replay.py`) drives
`sky.backend.windowed` / `sky.vio.window` DIRECTLY and **never imported** the live
`vio.engine` / `run_ba` / `Keyframe` path. The live backend is therefore decoupled
from the frozen oracle path — the engine can move out wholesale and `gap = 0` holds.

## Decision

Extract the windowed-BA backend into a **6th independent project, `ba/`** (a sibling
of `imu_camera`, `vio`, `slam`, `ui`, `launcher`):

- **`ba/main.py` `run_ba_proc`** — its own process on endpoint `oak.ba`. Subscribes
  the VIO endpoint for `keyframe` (+ the retained `calib.bundle` as a readiness
  barrier), attaches VIO's `kf_gray` / `kf_depth` rings, runs the windowed solve, and
  publishes on its OWN endpoint.
- **`ba/comms/`** — a 7th byte-identical vendored `comms/` copy (diff-identical to
  `imu_camera/comms/`), added to the cross-copy parity gate.
- **`ba/engine/` + `ba/modules/`** — `vio/engine/*` moved WHOLESALE; `run_ba` /
  `process_kf` / `BackendWorker` moved to `ba/modules/`. **`vio/engine/` was DELETED.**
- **Both backends extracted, selected by `--tight`** (now a `ba/` flag): loose
  `WindowedBAMap` (`ba_step`) or tight `WindowedVIOMap` (`vio_step`). `Keyframe` STAYS
  in shared `comms` — `vio` still produces it on its odometry thread (`emit_keyframe`)
  and it already crosses IPC to SLAM.
- **In-process engine only.** `ba/` IS its own process, so the GIL is already escaped;
  the in-VIO `--worker` SubprocessEngine is retired for BA. `--worker` is accepted as
  a **logged no-op** on `ba` and stays live for `slam`.

**New topology: `capture → vio → ba → slam → ui`.** Launcher spawn order is
capture → vio → ba → slam, UI last. `vio` still PRODUCES `keyframe`, consumed by
**both** `ba` and `slam`.

### Pass-through design (keeps the UI + netbridge UNCHANGED)

The hard constraint: do **not** add a 4th UI endpoint. `ba` publishes on `oak.ba`, but
the UI (and netbridge) read `pose.refined` from the **VIO** endpoint. So `vio` keeps a
thin **pass-through** (a direct mirror of how it re-hydrates SLAM's `loop.correction`):

- `vio` opens a **read-only client on `--ba-endpoint`** and bridges these topics back
  onto its own local bus, where its existing `IPCPublisher` re-emits them on the VIO
  endpoint:
  - **`pose.refined`** — re-emitted on the VIO endpoint → the UI's VIO-BA line is
    unchanged; netbridge keeps `pose.refined` in `VIO_POD` (no BA_POD needed).
  - **`ba.window`** — the opt-in `--ba-window` solve snapshot, re-emitted the same way.
- Under `--tight`, `ba` also publishes **`ba.state`** (the optimised bias: `seq`, `bg`,
  `ba`, `degraded`) on `oak.ba`. `vio` feeds it into `propagate_imu` via the existing
  `BackendStateInbox` — the tight **bias feed-forward**, now an IPC analog of SLAM's
  `loop.correction` feedback. The carried `seq` survives the wire so the consumer's
  staleness gate (built for the async `--worker` path) makes the IPC hop tolerable.

When `--ba-endpoint` is unset (the lean `--no-ba` path), `vio` runs with no refined
pose and an inert bias feed — `pose.odom` (live VIO) is unaffected, since it never
consumed the backend.

### Flag routing

- **`--no-ba`** is now a **launcher SPAWN gate** (mirror of `--no-slam`): the launcher
  simply does not spawn `ba`, and omits `--ba-endpoint` from `vio`. It is no longer a
  `vio` flag.
- The backend knobs **`--stabilize-velocity` / `--depth-icp` / `--backend-window` /
  `--backend-iters` / `--ba-window`** route to `ba` (`build_ba_args` → `ba.main`) and
  were REMOVED from `vio.main` / `build_vio_args` (they were inert on VIO once the
  backend left). Same gating as before: `--stabilize-velocity` / `--depth-icp` are
  forwarded only under `--tight`; `--ba-window` is loose-only.
  - *Side effect found & fixed:* `--backend-window` / `--backend-iters` were **never
    launcher-forwarded** pre-split (dead end-to-end). They are now added to the
    launcher argparse + `build_ba_args`, so they are operator-reachable for the first
    time.

## Consequences

- **6th project + 7th comms copy.** One more standalone, independently portable
  package; one more vendored `comms/` copy under the diff-identical parity gate
  (`verification/ipc_comms_selftest.py` COPIES now lists `ba`).
- **`ba.state` is a real cross-process feedback edge.** The tight bias now traverses
  IPC `ba → vio`. It is health-gated and seq-staleness-gated, so a diverging BA is
  never adopted (mirror of `loop.correction`).
- **Offline path untouched.** The oracle drives `sky.*` directly and never imported
  the live engine, so `gap = 0` is intact; `--tight` / loose behaviour is locked.
- **A backend-less window risk during the cut** was avoided by landing the vio-removal
  and the launcher-spawn together (the `vio/engine` COPY existed first), so
  `tight_live_regression` never went red.
- **One transitional gate artifact removed.** While `BACKEND_STATE` still lived in
  `vio/comms` (additive chunk), a scoped `_diff_is_only_backend_state` tolerance kept
  the comms source-parity gate green; deleting `BACKEND_STATE` from `vio/comms` made
  parity CLEAN again and that tolerance was removed.
- **Stale docs.** `docs/PROC4_ARCHITECTURE.md` and `README.md` describe "5 projects"
  and the in-VIO `BackendModule`; both are updated alongside this ADR.

## Rejected alternative

**A 4th UI endpoint (UI subscribes `oak.ba` directly for `pose.refined`).** Rejected:
it would force the UI's IPC sources AND netbridge (`receive.py`'s hardcoded role
tuples, an extra `EndpointServer`) to learn a new endpoint — a change that ripples
into the byte-for-byte-unchanged UI contract for no benefit. The pass-through re-emit
keeps the UI reading a single VIO endpoint exactly as before, at the cost of one thin
read-only bridge in `vio` that already exists in spirit for `loop.correction`.

## References

- Implementation record: `PLAN.md` → "TASK: Extract VIO windowed-BA into a 6th
  project `ba/`" + Chunk 1/2/3.
- `ba/main.py` (`run_ba_proc`), `vio/main.py` (`--ba-endpoint` pass-through),
  `launcher/main.py` (`build_ba_args`, `spawn_ba`, spawn order).
- Topology + diagram: `docs/PROC4_ARCHITECTURE.md` §2.
- Tight feed-forward design + health gate: `PLAN.md` P1/P2; `docs/TIGHT_COUPLED_PLAN.md`.
