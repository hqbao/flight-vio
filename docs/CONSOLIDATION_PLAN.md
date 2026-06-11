# Algorithm Consolidation Plan — duplicated `*/mathlib/` → shared `sky.*`

Status: **IN PROGRESS — S0–S5 DONE, S6 SKIPPED, R1–R6 DONE** (`sky/` scaffolded, `skymath` re-homed
as `sky.math`, SGM stereo → `sky.depth.stereo`, PnP → `sky.front.pnp`, IMU gyro/accel
calib → `sky.sensors`, loose IMU preint → `sky.imu.imu`, BA bundle core → `sky.backend.bundle`;
then the R-pass relocated the remaining SINGLE-COPY algorithm code — vio frontend +
odometry → `sky.front`, vio loose backend → `sky.backend`, slam loop → `sky.slam`, ui
camera-calib math → `sky.calib`, imu_camera inertial filters → `sky.imu`;
oracle `gap=0` preserved at every step). S6 (engine) deliberately SKIPPED as low/negative
port-value (see S6 below). S7 deferred until Phase 4 freezes. Goal: pull the DUPLICATED
algorithm code out of the 5 projects into one shared `sky.*` library so each process
is a thin shell (IPC wiring + calls into `sky.*`). Builds on the completed `skymath/`
(now `sky.math`, Step 1). Same discipline: byte-identical numerics, **`gap=0` gated at
every step**, divergent behavior preserved under distinct names (never silently
unified). Maps 1:1 onto the C `libsky*` layering (`docs/C_PORT_PLAN.md`) → this is the
Python precursor to the port.

## Measured drift map (diffed this session — better than feared)
| Target | Copies | Diff | Verdict |
|---|---|---|---|
| SGM `stereo.py` | imu_camera, depth | 3 hunks, **docstring-only** (code-identical; a `diff -r` gate already locks them) | **clean dedup** |
| PnP `odometry/pnp.py` | vio, slam | **0 diff** (byte-identical) | ✅ **DONE → `sky.front.pnp`** (S2) |
| BA `backend/bundle.py` | vio, slam | 5 lines, **comment-only** (optimizer core identical; VIO-vs-loop factors live in the CALLERS, not here; slam's copy was DEAD) | ✅ **DONE → `sky.backend.bundle`** (S5) |
| IMU calib `{accel_calib,calib_collect,calib_store,imu_calib}.py` | imu_camera, ui | accel/collect 0 diff; store/imu_calib docstring-path only | ✅ **DONE → `sky.sensors`** (S3) |
| IMU preint `imu/imu.py` | imu_camera, slam, **vio** | imu_camera **== slam (0 diff)**; vio = 707-line tight-VIO superset (438 diff) | ✅ **DONE → `sky.imu` (imu_camera+slam); DEFER vio (Phase 4)** (S4) |
| Engine `engine/{base,inprocess,steps,subprocess}.py` | vio, slam | base/inprocess small; steps 43 / subprocess 77 diff — genuine structural divergence | **extract-common-keep-variant (hardest; last)** |
| `resolution_build.py`, `warmup.py` | 2–3 | each builds a DIFFERENT config for the lib that project owns (deliberate) | **NOT a dedup target — leave per-project** |

## Target structure: ONE `sky/` package at repo root, with sub-packages
The shared library is a SINGLE top-level package `sky/` (importable as `import sky`);
each domain is a sub-package under it. `skymath` was re-homed INTO it as `sky.math`
(one library, not a sibling). Done so far: `sky.math`, `sky.depth`, `sky.front`,
`sky.sensors`, `sky.imu`.
`sky.math` (DONE) · `sky.depth` (SGM, DONE) · `sky.front` (PnP DONE; later KLT/corners) ·
`sky.sensors` (IMU calib, DONE) · `sky.imu` (loose preint, DONE) · `sky.backend` (bundle/marginalize/
windowed, loose) · `sky.engine` (Module/Step glue) · `sky.slam` (loop, re-home) ·
**`sky.vio` (tight VIO) = DEFERRED**. `sky.*` MUST stay free of process/comms/ui/io
imports (numpy + already-used cv2/numba only) so it's movable — enforce with an import-lint.

## Sequenced rollout (one domain/step, each `gap=0`-gated; dev → tester(gap=0 + ./run.sh replay) → docs)
- **S0 ✅ DONE** scaffolded `sky/` as one package; re-homed `skymath/` → `sky/math/` via the clean repoint (no shim — moved the 3 files, repointed all 27 `skymath` import sites to `sky.math`, deleted old `skymath/`). Added `sky.assert_import_clean()` (the import-lint: `sky.*` must pull NO process/comms/io module). Oracle stayed `gap=0`.
- **S1 ✅ DONE** `sky.depth` (SGM): moved imu_camera's canonical `stereo.py` → `sky/depth/stereo.py`, generalized the 3 docstring refs (`io.reader.StereoCalib`), repointed all 15 call-sites in both projects, deleted BOTH old `mathlib/stereo/` copies, and retired the prose `diff -r` lock-step gate (it was documentation in `depth/__init__.py` / `depth/mathlib/__init__.py` / the stereo `__init__.py` files — no executable gate existed). No SGM numerics touched; oracle stayed `gap=0`; both stereo selftests PASS; full 4-process replay clean.
- **S2 ✅ DONE** `sky.front.pnp`: moved the byte-identical `pnp.py` (one canonical copy) → `sky/front/pnp.py`, repointed both real call-sites (vio `odometry.py`, slam `loopclosure.py`) + the doc cross-refs, deleted both old copies; slam's `mathlib/odometry/` held only `pnp.py` so the now-empty forced-vendor package was removed. (Found: pnp's old transitive pull of `slam.mathlib.imu` for SO(3) is gone — pnp imports `sky.math` directly; this left slam's `imu.py` code-dead, harvested in S4.) Oracle `gap=0`; `vio.tests.odometry_selftest` + `slam.tests.loop_closure_selftest` PASS; pyflakes clean.
- **S3 ✅ DONE** `sky.sensors` (IMU gyro/accel calib): moved the 4 files (`accel_calib`, `calib_collect`, `calib_store`, `imu_calib`) → `sky/sensors/`, generalized the docstrings, fixed `calib_store`'s cache-dir depth (`parents[3]`→`parents[2]` for the new location), repointed all imu_camera + ui call-sites (incl. the gyro/accel `ui.qt.calib_dialogs`), deleted the ui dups + the now-empty `ui/mathlib/imu/`. SCOPE: the stereo-CAMERA calib (single-copy) left untouched. Oracle `gap=0`; imu_camera calib selftests + all 9 ui offscreen-Qt selftests PASS; offscreen smoke confirms the calib dialogs construct + bind `sky.sensors`; pyflakes clean.
- **S4 ✅ DONE** `sky.imu` (loose preint, **imu_camera+slam ONLY**): moved the byte-identical `imu.py` → `sky/imu/imu.py`, repointed the doc cross-refs, removed slam's now-empty `mathlib/imu/`. vio's 707-line tight-VIO SUPERSET (`vio/mathlib/imu/imu.py`) left untouched (Phase-4). Oracle `gap=0` (it feeds preint via vio's untouched copy); `imu_camera.tests.imucam_sync_selftest` + `slam.tests.loop_closure_selftest` + `vio.tests.imu_preint_cov_selftest` PASS; a parity smoke proves `sky.imu`'s preint API is byte-identical to vio's superset; pyflakes clean.
- **S5 ✅ DONE** `sky.backend.bundle`: investigation confirmed the two `backend/bundle.py` were token-identical (code + all string literals; only ~3 comment lines differed) and the `optimize()` core is FACTOR-AGNOSTIC — reprojection / depth / gravity / marginalization-prior / VO-relative factors are all passed in as arrays, and the VIO-vs-loop divergence lives in the CALLERS (vio's `windowed.py` builds them), never in `bundle.py` → **clean dedup**. Extra finding: slam's `bundle.py` was DEAD CODE (nothing in slam imported it; slam loop closure runs its own pose-graph in `slam.mathlib.loop.posegraph`, Lie helpers from `sky.math`). `git mv` vio's canonical copy → `sky/backend/bundle.py` (+ `sky/backend/__init__.py`), generalized its docstring/comments, repointed vio's 3 call-sites (`windowed.py`, `marginalize.py`, `modules/pipeline.py`), deleted slam's dead copy + fixed slam's stale FORCED-VENDOR `backend` doc-note. SCOPE: `bundle.py` only — `vio_window.py` (Phase-4) and `windowed.py`/`marginalize.py` content untouched (only their bundle import repointed). Oracle `gap=0`; `vio.tests.vio_ba_selftest` + `slam.tests.loop_closure_selftest` PASS; sky.* leaf (`assert_import_clean` OK); `-W error` import clean; pyflakes clean (the one pre-existing unused-local in `marginalize.py` is out of scope — not introduced here).
- **S6 ⏭ SKIPPED (deliberate, low value)** `sky.engine`: assessment found the genuine common core is SMALL and ENTANGLED, not worth extracting. (a) Even ignoring docstrings, `base.py`/`inprocess.py` are NOT token-identical — slam carries an extra `poll_loops()` on BOTH (vio lacks it); a shared base would either pollute vio's contract with a SLAM-only loop-capture method or keep a slam override anyway (≈nothing gained). (b) `steps.py` (43-diff) + `subprocess.py` (77-diff) are irreducibly per-project. (c) vio's engine is directly coupled to the **Phase-4 `vio_window.py` surface** (`make_vi_engine`→`WindowedVIOMap`, `vio_step`/`vio_overlay`) — consolidating would entangle S6 with the frozen Phase-4 surface; the `make_*_engine` factories are per-project by construction. (d) **Zero port-value**: `docs/C_PORT_PLAN.md`'s libsky layer table has NO `libskyengine` — the in-process/subprocess split is a Python-GIL artifact the C IPC process model replaces wholesale. Best case an extraction saves ~40 lines while adding a Python-only abstraction the port immediately discards. Per the plan's own escape clause → **left per-project**.
- **S7 DEFERRED** `sky.vio` (tight VIO `vio_window.py` + vio's `imu.py` superset + `propagate_imu.py`) — **blocked until Phase 4 freezes** (`phase4_tight_diverge_diag.py` clean). Hard rule: stabilize-in-Python-then-consolidate. (The slam *loop* re-home originally listed here was NOT tight-coupled and was done in R4, below.)

### R-pass — relocate the remaining SINGLE-COPY algorithm code (not dedup)
After the S0–S5 *dedup* of the duplicated code, the R-pass relocates the modules that
existed in only ONE project's `mathlib/` into `sky.*` so each process is maximally thin.
Same discipline as S0–S5 (leaf-check → `git mv` → repoint → `gap=0` → commit), one domain
at a time. **Single-copy moves — no reconciliation.**
- **R1 ✅ DONE** `sky.front` (vio frontend): `git mv` the 4 leaf-clean modules `vio/mathlib/frontend/{klt,corners,frontend,klt_numba}.py` → `sky/front/` (joining `pnp.py`). Repointed every call-site (`vio.main`, `vio.modules.pipeline`, `vio.mathlib.{warmup,resolution_build,backend.windowed,backend.vio_window}`, `vio.tests.odometry_selftest` + doc cross-refs in `slam.mathlib.loop.orb` / `ui.viz.keypoint_overlay`); removed the now-empty `vio/mathlib/frontend/`. Oracle `gap=0`; `vio.tests.odometry_selftest` + `ui.tests.ui_dataflow_selftest` PASS; sky.* leaf; pyflakes 0; `-W error` clean. (commit `3701e67`)
- **R2 ✅ DONE** `sky.front.odometry` (vio odometry): `git mv` `vio/mathlib/odometry/odometry.py` → `sky/front/odometry.py` (already used `sky.front.pnp` + `sky.math`). Its one coupling — `align_to_gravity` imported `gravity_aligned_R0` from the Phase-4 `vio.mathlib.imu.imu` SUPERSET — was DECOUPLED by repointing to `sky.imu.imu`, after VERIFYING that function is byte-identical there (S4); odometry is now leaf-clean and numerics unchanged. Repointed all 20 call-sites; removed the now-empty `vio/mathlib/odometry/`. Oracle `gap=0`; `vio.tests.{odometry,gyrofuse,reproj_stub,vio_ba,tight_smoke,closed_loop_drift,tight_live_pose}_selftest` PASS; sky.* leaf; pyflakes 0; `-W error` clean. (commit `22b77bd`)
- **R3 ✅ DONE** `sky.backend` (vio loose backend): `git mv` `vio/mathlib/backend/{windowed,marginalize}.py` → `sky/backend/` (joining `bundle.py`). The tight Phase-4 `vio_window.py` STAYS per-project (it references windowed only in docstrings — no code import — so the move is safe). Repointed `verification.{oracle_replay,loose_vs_tight_bench}`, `vio.mathlib.engine.{__init__,subprocess}`, `vio.modules.pipeline`, `vio.tests.tight_smoke_selftest`; updated the `sky.backend` + `vio.mathlib` package docstrings. Oracle `gap=0`; `vio.tests.{vio_ba,tight_smoke}_selftest` + `ui.tests.ui_dataflow_selftest` PASS; sky.* leaf; pyflakes 0; `-W error` clean. (commit `a45a539`)
- **R4 ✅ DONE** `sky.slam` (slam loop): `git mv` `slam/mathlib/loop/{orb,loopclosure,posegraph,slam}.py` → the new `sky/slam/` package. All four are leaf-clean (numpy + `sky.math` + `sky.front.pnp` + intra-package); the `SlamMap` orchestrator (`slam.py`) is process-free (takes data as args), so it moved cleanly — no coupling found. Repointed `slam.main`, `slam.modules.{pipeline,__init__}`, `slam.mathlib.{resolution_build,engine.__init__,engine.subprocess}`, `slam.tests.{loop_closure,loop_capture}_selftest`, `verification.oracle_replay`, `vio.tests.closed_loop_drift_selftest`, `ui.viz.loop_render` + doc cross-refs; removed the now-empty `slam/mathlib/loop/`. Oracle `gap=0`; `slam.tests.{loop_closure,loop_capture,proc3_smoke}_selftest` PASS; sky.* leaf; pyflakes 0; `-W error` clean. (commit `2433f81`)
- **R5 ✅ DONE** `sky.calib` (ui camera-calib math): `git mv` the 4 leaf-clean modules `ui/mathlib/calib/{detect,collector,solve,writer}.py` → `sky/calib/` (cv2 is lazy-imported, so importing `sky.calib` loads no OpenCV). `checkerboard.py` was COUPLED (`save_checkerboard` imports `ui.comms.lib.misc.pngio`; `_show_fullscreen` imports PyQt6 — both FORBIDDEN in `sky`) → **SPLIT**: the PURE numpy generators (`make_checkerboard` + `square_px_from_mm`) moved to `sky/calib/checkerboard.py`; the PNG-save + Qt preview + CLI stay per-project in `ui/mathlib/calib/checkerboard.py`, now a thin wrapper that re-exports the pure generators (so call-sites + CLI are unchanged). Repointed `ui.qt.camera_calib_dialog`, `ui.tests.{calib_solve,camera_calib_dialog}_selftest`, `imu_camera.mathlib.device.camera_calib_store` doc ref. Oracle `gap=0`; `ui.tests.{checkerboard,calib_solve,camera_calib_dialog}_selftest` PASS (offscreen; incl the cv2-free-import + writer round-trip + full wizard solve/save); sky.* leaf; pyflakes 0; `-W error` clean. (commit `044f8c5`)
- **R6 ✅ DONE** `sky.imu` (imu_camera inertial filters): `git mv` `imu_camera/mathlib/imu/{inertial_filter,timed_buffer}.py` → `sky/imu/` (joining `imu.py`). Both leaf-clean (numpy + stdlib threading/time/collections). Repointed `imu_camera.modules.{pipeline,pack_synced}` (`TimedImuBuffer`) + `read_imu` doc ref; updated the `sky.imu` + `imu_camera.mathlib.imu` package docstrings. The depthai `decode.py` (device driver) STAYS — it is now the ONLY module left in `imu_camera.mathlib.imu`. Oracle `gap=0`; `imu_camera.tests.{imucam_sync,codec_roundtrip,calib_check,calib_status,camera_calib_store,stereo_sgm}_selftest` PASS; sky.* leaf; pyflakes 0; `-W error` clean. (commit `f3fff5a`)
  - *Coupling note (R6):* `inertial_filter` has no current consumers repo-wide and no test coverage; it was RELOCATED faithfully (single-copy move), not purged — a dead-code review is a separate task.
  - *Pre-existing issues observed, left out of scope:* (a) `vio/mathlib/engine/` has a broken lazy `from ..loop.slam import SlamMap` (vio has no `loop/` dir; `make_slam_engine` is defined-but-never-called in vio) — already broken, engine = S6-skipped. (b) `slam/mathlib/backend/__init__.py` is an empty dead package (nothing imports `slam.mathlib.backend`; left from S5's deleted dead `bundle.py`).

## Stays per-project (not consolidated)
All `*/comms/` (vendored wire contract); `resolution_build.py`/`warmup.py` (per-project
config builders); each process `main.py`/`modules/`/`io/` (orchestration + IPC + drivers);
vio's tight-VIO surface (Phase 4); `baseline/` (Basalt ref); `ui/` Qt/viz (only its calib
math joins `sky.sensors`).

## Effort + risks
~**3–5 weeks** for the stable set S0–S6 (S7 unschedulable until Phase 4). S0–S3 ~1 wk
(mechanical, low-risk — most are docstring/0-diff so `gap=0` is near-automatic); S4 ~3-4d
(oracle-feeding); S5 ~3-5d (the factor-location investigation is the risk); S6 ~1 wk (only
genuine extract-common). Risks: per-step byte-parity (primary gate); engine drift
reconciliation; import-cycle/movability (lint `sky.*`); collision with in-flight calib
work (S3); the bundle factor-location assumption (S5). Honest note: this is INFRASTRUCTURE
(thinner processes, no drift) — it does NOT advance the algorithm/Phase 4; it makes future
algorithm work easier + is the port precursor.

## Recommended first step
**S0 + S1** (scaffold `sky/` then dedup SGM → `sky.depth`): safest, oracle-covered, and
the direct precursor to the C port's first real process. Establishes the reusable pattern
(move → repoint/shim → retire redundant gate → gap=0) that S2–S6 follow mechanically.
