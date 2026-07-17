# Experimental Simulation Package — Implementation Plan

*Rev 2 — after adversarial review (2026-07-17). Major changes from rev 1: gravity-
convention decision, URDF preprocessing task, typed interfaces, conventions
section, closed-loop parity testing, external-wrench primitive moved to Phase 2,
config dataclasses from the start, per-controller control rates.*

Part of the document set: [requirements.md](requirements.md) (what/why, REQ-IDs) →
[design.md](design.md) (how, DD-IDs) → [verification.md](verification.md)
(proof, TC-IDs + traceability matrix) → **plan.md** (when/order) →
[execution/](execution/index.md) (dispatchable work packages WP-01..24 for
subagents, with the dispatch protocol). Phase exits are gated by the L3/L4 test
cases mapped to each phase in verification.md §3.

## Goal

Evolve this repo from two standalone demo scripts into an **experimental simulation
package** for robot-arm control analysis. Impedance control becomes one test among
many. Target capabilities (in build order): real contact physics, a controller zoo
with comparisons, robustness/disturbance batch sweeps, and system-ID/estimation
testbeds.

## Decisions (agreed 2026-07-17, extended after review)

| Topic | Decision |
|---|---|
| Physics backend | **MuJoCo** (`mujoco==3.10.0`, pip) simulates the world; **Pinocchio** stays for control-side model terms (M, C, g, Jacobians) |
| Model source | One shared URDF (example-robot-data) loaded into *both* engines. UR5 first; other example-robot-data arms pluggable by name |
| **Gravity convention** | **The plant is real: MuJoCo applies gravity, no hidden compensation.** Every controller must output *total* torque including its own gravity/nle compensation (via `ctrl_model`). The legacy script's plant `aba(q, dq, tau + g)` (line 291) secretly cancels gravity — ports must be reformulated, not copied |
| Control rate | Per-controller/per-task property, not global. Legacy variant 2 was designed at 1 kHz (`wn`=250 rad/s — line 95); contact tasks default to 1 kHz. 100 Hz only for free-space variants 3/4 |
| Viewer | MuJoCo native viewer is **authoritative** (shows the true physics scene). Meshcat is an optional robot-mirror (pretty DAE visuals; scene props mirrored best-effort only). Selectable per run |
| Consumption | Interactive-first (live viewer + matplotlib at end). `Recorder` opt-in, required for batch sweeps |
| First analysis | Contact interaction tasks (wall contact / surface slide) |
| Repo shape | Python package `icbench/` + thin runnable scripts in `experiments/`. Legacy `controllers/*.py` stay untouched and working |

## Conventions (single source of truth — violations are bugs)

- **Units**: SI throughout (m, s, rad, N, N·m). No degrees anywhere.
- **Canonical joint order**: the URDF/pinocchio order. `World` owns the
  mujoco↔canonical remap; `Controller.compute` and `Viewer.sync` always receive
  canonical-order state. The remap is data (an index map built in `models.py`),
  never assumed to be identity — pinocchio uses 2 qpos entries for continuous
  joints, so `pin.nq != mj.nq` for some arms.
- **Frames**: end-effector frame is the URDF `ee_link` frame; Jacobians and
  task-space quantities in `LOCAL_WORLD_ALIGNED` (world-aligned axes at the ee).
  `World.ee_wrench()` returns the wrench **rotated into world-aligned axes**
  (MuJoCo F/T sensors report in the body-fixed site frame — must be rotated by
  site `xmat`). MuJoCo `mj_contactForce` uses a contact frame whose **X axis is
  the normal** — document at every use site.
- **Orientation error**: `rpy(R_des @ R.T)` as in the legacy script for ported
  controllers (documented limitation: not a proper Lie-group error; revisit if
  large-rotation tasks appear).
- **Seeds**: every experiment script takes `--seed`; every random test fixes and
  records its RNG seed.
- **Wrench sign**: force *exerted by the robot on the environment* is positive in
  task metrics. Verified by a Phase 1 static test (payload hanging at rest must
  read +m·g in world −z on the sensor, sign-mapped accordingly).

## Key technical facts (verified / to verify early)

- **MuJoCo does not resolve `package://` URIs** (all 14 UR5 mesh references use
  them — verified). `models.py` must preprocess the URDF: rewrite
  `package://example-robot-data/` to the absolute share path pinocchio uses.
  Also verify visual DAE handling (MuJoCo can't load DAE): rely on
  `discardvisual` default for URDF or strip `<visual>` elements in the same
  preprocessing pass; MuJoCo uses the STL *collision* meshes.
- **MuJoCo compiles URDF but produces no actuators** → attach one `<motor>` per
  joint via `mjSpec`, with `forcerange` taken from URDF `<limit effort>`
  (UR5: ≤150 N·m base, ≤28 N·m wrist). Unbounded motors flatter every controller
  and would invalidate Phase 7 conclusions. Opt-out flag for idealized studies.
- **Inertia processing**: compile with inertia balancing disabled
  (`balanceinertia=false`); if the URDF inertias violate the triangle inequality,
  that's a finding to document, not something the compiler should silently "fix"
  (it would make pin and MuJoCo simulate different models).
- **Legacy port traps (verified in `controllers/impedance_6dof.py`)**:
  1. Plant is gravity-compensated (line 291) — see Decisions.
  2. Lines 262/270: `(h - g - M @ Ji @ dJ) @ Ji @ v_desired` subtracts a 6-vector
     from a 6×6 matrix — silent numpy broadcast, mathematically meaningless.
     Benign in legacy only because `v_desired = 0` unless `dynamic_ref`. The port
     must use the correct Coriolis-matrix form (`pin.computeCoriolisMatrix`) or
     drop the term with documented justification.
  3. `np.linalg.inv(J)` with no singularity handling → port uses damped
     least-squares pseudoinverse with a manipulability guard.
- **URDF `<dynamics>` damping/friction**: MuJoCo honors them (`qfrc_passive`),
  `pin.aba` ignores them. UR5 has all zeros (verified) — the consistency test
  asserts `qfrc_passive == 0` per robot; a nonzero case requires replicating
  damping on the pinocchio side before that robot is supported.
- Timing: physics dt 1–2 ms; control at its own dt (see Decisions); viewer syncs
  ~real-time interactive, unconstrained headless.
- Ubuntu 20.04 + Python 3.12 venv (`.venv`). Headless rendering (batch videos)
  uses EGL — verify once in Phase 0, don't build on it before.

## Interfaces (defined now to avoid Phase-4/6 rework)

```python
# state: canonical-order dataclass
State:      q, dq, tau_applied, t                      # np arrays, canonical order
Reference:  pose (SE3 or None), twist, accel,          # task-space targets
            q_des, dq_des, ddq_des (or None),          # joint-space targets
            wrench_des (or None)                       # force targets (contact tasks)
LogSchema:  dict[str, np.ndarray]                      # fixed channel names + units,
                                                       # defined once in Phase 2; shared
                                                       # by runner, Task.metrics, Recorder

Controller: __init__(ctrl_model, config_dataclass)     # model injected, params are data
            reset(state0)
            compute(state, ref, t, dt) -> tau          # TOTAL torque (incl. gravity comp)

Task:       build_scene(spec)                          # owns ALL env geometry incl. floor
            reference(t) -> Reference
            metrics(log: LogSchema) -> dict

World:      reset(q0); step(tau, n_substeps)
            state() -> State
            ee_wrench() -> np.ndarray                  # world-aligned, sign per Conventions
            contacts() -> list                         # raw mj contacts (X-normal frame)
            apply_external_wrench(frame, wrench)       # disturbance primitive (Phase 2)

Viewer:     sync(state); close()
```

Config: every controller/task gets a small `@dataclass` config constructible from a
plain dict, from Phase 2 onward. Phase 7's YAML layer only builds these dicts —
no controller refactor at sweep time.

## Package layout

```
icbench/
  models.py            # registry; URDF preprocessing (package:// → abs paths);
                       #   builds pin RobotWrapper + mjSpec robot; joint index map
  sim/world.py         # World (see interface); mujoco↔canonical remap lives here
  sim/scene.py         # mjSpec helpers: actuators+forcerange, ee site + F/T sensor,
                       #   floor/wall builders (called only by Tasks)
  control/base.py      # Controller ABC
  control/ctrl_model.py# pin wrapper: M, nle, g, coriolis matrix, frame J/dJ, FK
  control/impedance.py # task-space impedance (reformulated variants 2/3/4)
  control/joint_pd.py  # (phase 6) computed_torque.py, admittance.py
  tasks/base.py        # Task ABC; free_space.py; (phase 4) wall_contact.py, surface_slide.py
  viz/base.py          # mujoco_viewer.py; meshcat_viewer.py + meshcat_patch.py
  runner.py            # Experiment loop; owns LogSchema assembly
  recording.py         # (phase 5)
  analysis/            # (phases 7-8)
experiments/
  _common.py           # shared argparse: --seed --viewer --record --duration
  impedance_free_space.py, impedance_wall_contact.py, ...
tests/
```

## Phases

Commit at every checkbox. `./.venv/bin/pytest tests/ -q` before each commit from
Phase 1 on. All spike findings and validation numbers go to **one** file:
`docs/design/sim-framework/findings.md`.

### Phase 0 — Scaffold

- [ ] `requirements.txt`: add `mujoco==3.10.0`, `pytest` (other deps already pinned); install
- [ ] Package skeleton + `tests/test_smoke.py` (imports pass)
- [ ] Spike (throwaway code, findings committed to findings.md): preprocess UR5 URDF
      (package:// rewrite), compile via mjSpec, print joint names/order, confirm STL
      meshes + `discardvisual` behavior, confirm EGL headless renders one frame

### Phase 1 — Models + World core

- [ ] `models.py`: URDF preprocessing + `load_robot(name) -> RobotBundle` (pin wrapper,
      preprocessed URDF path, joint index map). Test: UR5 loads; **explicit assertion
      of the joint-order map itself**, not just quantities computed through it
- [ ] `scene.py`: actuators with `forcerange` from URDF effort limits; `ee` site +
      F/T sensor; **no floor here** — env geometry belongs to Tasks
- [ ] `sim/world.py`: `World` per interface; remap lives here. Tests: free fall
      (no floor!) matches gravity; torque on one joint accelerates it; static
      payload test fixes wrench sign & frame (+m·g in world −z)
- [ ] `apply_external_wrench()` primitive + test (constant wrench deflects ee as
      predicted quasi-statically)
- [ ] **Cross-engine tests** (`tests/test_cross_engine.py`, seeded RNG, relative
      tol ~1e-6): (a) `pin g(q)` vs `qfrc_bias(q, 0)`; (b) `pin nle(q, dq)` vs
      `qfrc_bias(q, dq)` at nonzero dq — catches Coriolis/order errors statics miss;
      (c) `pin M(q)` vs `mj_fullM`; (d) `qfrc_passive == 0`

### Phase 2 — Control layer + runner (free-space)

- [ ] `ctrl_model.py`: pin wrapper incl. `coriolis(q, dq)` matrix. Test vs direct pin calls
- [ ] `control/base.py` ABC + config dataclasses
- [ ] `impedance.py`: **reformulated** variants 2/3/4 — explicit gravity/nle comp,
      correct Coriolis-matrix term (replaces broadcast bug), damped-pinv J inverse.
      Unit tests: multi-state golden fixtures **including dq≠0 states** for the
      terms that are ports; documented derivation for the reformulated terms
- [ ] **Closed-loop parity test**: replicate the legacy plant *exactly* in a test
      harness (`pin.aba(q, dq, tau + g)`, explicit Euler, legacy dt) and assert the
      ported controller class reproduces the legacy script's `q(t)` trajectory.
      This validates the port; MuJoCo behavior is then validated separately for
      stability (finite RMSE, no divergence) — it will differ, that's physics
- [ ] `tasks/base.py` + `free_space.py` (fixed + sinusoidal reference; sine ee-force
      disturbance via `apply_external_wrench` — parity with legacy `fe_amp` behavior)
- [ ] `runner.py` + LogSchema (channel names/units fixed here, shared contract)
- [ ] `experiments/_common.py` + `impedance_free_space.py` (`--variant --duration
      --seed --viewer none`); README section: package vs legacy scripts

### Phase 3 — Viewers

- [ ] `viz/mujoco_viewer.py` (`launch_passive`, real-time sync) — authoritative
- [ ] Move `_patch_meshcat_viewer_bundle()` from legacy script into
      `icbench/viz/meshcat_patch.py` (legacy imports from there; **warn loudly**
      when the bundle matches neither the broken nor the fixed pattern — silent
      no-op only when already patched)
- [ ] `viz/meshcat_viewer.py`: robot mirror (pin DAE visuals synced to MuJoCo state);
      scene props mirrored best-effort from task metadata, explicitly not guaranteed
- [ ] `--viewer {none,mujoco,meshcat}` wired through; manual visual check both paths

### Phase 4 — Contact tasks

- [ ] `scene.py`: `add_wall(spec, pose, friction)` (called from Task.build_scene)
- [ ] `tasks/wall_contact.py`: approach → press to `wrench_des`; control dt 1 kHz;
      metrics from **summed `mj_contactForce`** (ground truth; X-normal convention
      documented at the use site) — F/T-sensor-based force feedback kept separate
      as "what a real robot would measure" (bias-compensated) for admittance later
- [ ] `tasks/surface_slide.py`: constant-normal-force slide; force RMSE along path
- [ ] `experiments/impedance_wall_contact.py`: variants 2/3/4, force/position plots
- [ ] Validation: stiffness sweep → force/penetration trend documented in findings.md;
      README updated with contact experiments

### Phase 5 — Recorder

- [ ] `recording.py`: opt-in, serializes the LogSchema + config dataclasses + metrics
      to timestamped `data/runs/<ts>/`; add `data/runs/` to `.gitignore` (README
      already bans committing data)
- [ ] `--record` in `_common.py`

### Phase 6 — Controller zoo + comparison

- [ ] `joint_pd.py` (legacy 1/5 reformulated), `computed_torque.py` — computed torque
      **before** admittance (it's the admittance inner loop)
- [ ] `admittance.py`: outer loop (wrench → motion reference) composed with inner
      joint controller at 1 kHz; low-pass on measured wrench (cutoff well below
      inner-loop bandwidth); uses F/T-sensor path, not contact ground truth
- [ ] `experiments/controller_comparison.py`: same task × N controllers → metric
      table + overlay plots

### Phase 7 — Robustness & disturbance batch sweeps

- [ ] `analysis/sweep.py`: YAML → config-dataclass dicts (no controller changes);
      grids over payload mass, wall friction, gains; headless parallel runs;
      aggregate `metrics.parquet` + report
- [ ] Disturbance *scheduling* (time-profiles over the existing Phase-2 wrench
      primitive)

### Phase 8 — System ID / estimation testbed

- [ ] `analysis/sysid/payload_estimation.py`: payload mass/inertia from joint
      torques + motion (regressor least squares / RLS), sim ground truth from
      Phase-7 payload machinery. (Same problem family as the OKS lifter
      weight-estimation work — port lessons, not code)

## Findings

→ `docs/design/sim-framework/findings.md` (single location; Phase 0 spike results first).

## Non-goals (YAGNI, revisit only when hit)

- ROS integration, real-robot interfaces
- GPU/parallel sim (MJX), RL training
- Upstream-PR compatibility of the package (legacy scripts stay compatible; the
  package itself is ours)
- Guaranteed meshcat scene fidelity (MuJoCo viewer is authoritative)
