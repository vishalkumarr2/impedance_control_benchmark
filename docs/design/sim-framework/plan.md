# Experimental Simulation Package — Implementation Plan

## Goal

Evolve this repo from two standalone demo scripts into an **experimental simulation
package** for robot-arm control analysis. Impedance control becomes one test among
many. Target capabilities (in build order): real contact physics, a controller zoo
with comparisons, robustness/disturbance batch sweeps, and system-ID/estimation
testbeds.

## Decisions (agreed 2026-07-17)

| Topic | Decision |
|---|---|
| Physics backend | **MuJoCo** (`mujoco==3.10.0`, pip) simulates the world; **Pinocchio** stays for control-side model terms (M, C, g, Jacobians) |
| Model source | One shared URDF (example-robot-data) loaded into *both* engines → no model mismatch between sim and controller. UR5 first; any example-robot-data arm pluggable by name |
| Viewer | Both backends, selectable per run: MuJoCo native viewer *and* meshcat (browser). Meshcat reuses the `merge_geometries` bundle patch |
| Consumption | Interactive-first (live viewer + matplotlib at end). `Recorder` is optional, needed later for batch sweeps |
| First analysis | Contact interaction tasks (wall contact / surface slide) |
| Repo shape | Python package `icbench/` + thin runnable scripts in `experiments/`. Legacy `controllers/*.py` stay untouched and working |

## Key technical facts (verified / to verify early)

- **MuJoCo cannot load DAE meshes** (STL/OBJ/MSH only). example-robot-data ships
  *collision* meshes as STL → MuJoCo uses those for both collision and its own
  rendering. Meshcat viewer keeps the pretty DAE visuals via the pinocchio path.
  This split is a feature: physics never depends on visual meshes.
- **MuJoCo compiles URDF directly** but produces no actuators; use the `mjSpec`
  API (mujoco ≥3.2) to attach one `<motor>` per joint (ctrl = joint torque) and to
  add scene elements (floor, wall, ee force sensor) around the compiled robot.
- **Cross-engine consistency is testable**: at rest, pinocchio `g(q)` must equal
  MuJoCo `qfrc_bias` (same URDF, same joint order). This is the anchor unit test
  that catches 90% of integration mistakes (joint ordering, frame conventions,
  inertia parsing).
- Timing: physics dt 1–2 ms; control runs at its own dt (default 100 Hz to match
  the current script; configurable to 1 kHz); viewer syncs ~real-time in
  interactive mode, unconstrained headless.
- Ubuntu 20.04 + Python 3.12 venv (`.venv`) is the target env. Headless rendering
  (batch phase) uses EGL/osmesa — verify once in Phase 0, don't build on it before.

## Package layout

```
icbench/
  __init__.py
  models.py            # registry: name -> URDF path/meshes; builds (pin.RobotWrapper, mjSpec robot)
  sim/
    __init__.py
    world.py           # World: mjModel/mjData, step(tau), state(), contacts(), ee_wrench()
    scene.py           # mjSpec helpers: floor, wall, ee site + force sensor, actuators
  control/
    __init__.py
    base.py            # Controller ABC: reset(model_state), compute(state, t) -> tau
    ctrl_model.py      # Pinocchio wrapper: M, nle, g, frame J/dJ, FK (control-side model;
                       #   later: deliberately perturbed params for robustness studies)
    impedance.py       # task-space impedance (ported variants 2/3/4 from legacy script)
    joint_pd.py        # (phase 6) variants 1/5
    admittance.py      # (phase 6)
    computed_torque.py # (phase 6)
  tasks/
    __init__.py
    base.py            # Task ABC: build_scene(spec), reference(t), metrics(log) -> dict
    free_space.py      # regulation + moving reference (port of current demo behaviour)
    wall_contact.py    # (phase 4) approach + press against wall, force target
    surface_slide.py   # (phase 4) slide along surface at constant normal force
  viz/
    __init__.py
    base.py            # Viewer ABC: sync(state), close()
    mujoco_viewer.py   # mujoco.viewer.launch_passive
    meshcat_viewer.py  # pin MeshcatVisualizer + the merge_geometries bundle patch
  runner.py            # Experiment(robot, task, controller, viewer, dt...): interactive loop
  recording.py         # (phase 5) Recorder -> timestamped run dir: config, npz, metrics, plots
  analysis/            # (phases 7-8) sweep driver, aggregate reports, sysid estimators
experiments/           # thin argparse scripts, one per study — read like today's demos
tests/                 # pytest; consistency + unit tests, no GUI needed
```

## Phases

Commit at every checkbox. Each task lists its files; a task should be a few minutes
of work. Run `./.venv/bin/pytest tests/ -q` before each commit from Phase 1 on.

### Phase 0 — Scaffold (foundation, ~30 min)

- [ ] Add `mujoco==3.10.0` and `pytest` to `requirements.txt`; `pip install -r requirements.txt`
- [ ] Create package skeleton: `icbench/__init__.py` (+ empty subpackage `__init__.py`s), `tests/`, `experiments/`
- [ ] Smoke test `tests/test_smoke.py`: `import mujoco; import pinocchio; import icbench` passes
- [ ] One-off spike (throwaway, do not commit code — commit findings to this plan):
      compile the UR5 URDF with `mujoco.MjSpec.from_file`, print joint names/order,
      confirm STL meshes resolve. Record any path quirks below in *Findings*.

### Phase 1 — Models + World core

- [ ] `icbench/models.py`: `load_robot(name) -> RobotBundle` — pin RobotWrapper
      (visual DAE model) + URDF/mesh paths for MuJoCo, via example-robot-data.
      Test: UR5 loads, `nq == 6`, joint names match between engines *in order*.
- [ ] `icbench/sim/scene.py`: `build_spec(urdf_path, mesh_dir)` → mjSpec with robot,
      floor plane, torque `<motor>` per joint, `ee` site + force/torque sensor.
      Test: compiled model has 6 actuators; sensor exists.
- [ ] `icbench/sim/world.py`: `World` — `reset(q0)`, `step(tau, n_substeps)`,
      `state() -> (q, dq)`, `ee_wrench()`, `contacts()`.
      Test: free fall matches gravity; torque input accelerates the expected joint.
- [ ] **Consistency test** `tests/test_cross_engine.py`: for 20 random q,
      `pin g(q) ≈ mujoco qfrc_bias(q, dq=0)` (tol 1e-8 after sign/order mapping);
      pin `M(q)` ≈ mujoco `mj_fullM` (tol 1e-6). This test gates everything after it.

### Phase 2 — Control layer + runner (free space parity with legacy demo)

- [ ] `icbench/control/ctrl_model.py`: pin wrapper — `M, nle, g, J(frame), dJ, oMf`.
      Test: values equal direct pin calls on the legacy script's pose.
- [ ] `icbench/control/base.py`: `Controller` ABC (`reset`, `compute(state, t) -> tau`).
- [ ] `icbench/control/impedance.py`: port legacy controllers 2/3/4 as
      `TaskSpaceImpedance(variant=...)`. Test: given the legacy script's exact
      state snapshot, tau matches the legacy computation (regression fixture).
- [ ] `icbench/tasks/base.py` + `tasks/free_space.py`: fixed & sinusoidal reference
      (port `dynamic_ref` logic), `metrics()` = tracking RMSE.
- [ ] `icbench/runner.py`: `Experiment.run(duration)` loop — control dt vs physics
      substeps, no viewer yet. Test: 2 s headless run completes, RMSE finite.
- [ ] `experiments/impedance_free_space.py`: argparse (`--variant --duration --viewer none`),
      prints metrics — behavioural sanity check vs legacy demo.

### Phase 3 — Viewers (both, selectable)

- [ ] `icbench/viz/base.py` + `mujoco_viewer.py` (`launch_passive`, real-time sync).
- [ ] `meshcat_viewer.py`: move `_patch_meshcat_viewer_bundle()` out of the legacy
      script into `icbench/viz/meshcat_patch.py`; legacy script imports it back from
      there (single source of truth). Meshcat shows pin visual model synced to
      MuJoCo state.
- [ ] Wire `--viewer {none,mujoco,meshcat}` into runner + experiment scripts.
      Manual check: same run looks identical in both viewers.

### Phase 4 — Contact tasks (first real analysis payoff)

- [ ] `scene.py`: `add_wall(spec, pose, friction)`; wall visible in both viewers
      (meshcat: mirror as a box from task description).
- [ ] `tasks/wall_contact.py`: approach → press to target normal force; metrics:
      steady-state force error, overshoot, settling time.
- [ ] `tasks/surface_slide.py`: constant-force slide; metrics: force RMSE along path.
- [ ] `experiments/impedance_wall_contact.py`: run variants 2/3/4 against the wall,
      matplotlib force/position plots at end (interactive-first).
- [ ] Validation: impedance stiffness sweep shows expected force/penetration trend
      (document numbers in `docs/design/sim-framework/findings.md`).

### Phase 5 — Recorder (enabler for batch work)

- [ ] `icbench/recording.py`: opt-in `Recorder` — timestamped dir under `data/runs/`:
      `config.json`, `timeseries.npz`, `metrics.json`, saved PNGs.
- [ ] `--record` flag in experiment scripts. Legacy `data/` files untouched.

### Phase 6 — Controller zoo + comparison

- [ ] Port joint-space PD (legacy 1/5) → `joint_pd.py`; add `admittance.py`
      (position-based, needs measured ee wrench from World) and `computed_torque.py`.
- [ ] `experiments/controller_comparison.py`: same task × N controllers → metric
      table + overlay plots.

### Phase 7 — Robustness & disturbance batch sweeps

- [ ] `analysis/sweep.py`: config-driven (YAML) parameter grids — payload mass on
      ee, wall friction, controller gains; headless parallel runs (verify EGL
      headless rendering only if videos wanted); aggregate `metrics.parquet` + report.
- [ ] Disturbance injection hooks in `World` (external wrench schedules — replaces
      the legacy hardcoded sine force).

### Phase 8 — System ID / estimation testbed

- [ ] `analysis/sysid/payload_estimation.py`: estimate ee payload mass from
      joint torques + motion (RLS / least squares on regressor), sim = ground truth.
      (Same problem family as the OKS lifter weight-estimation work — port lessons,
      not code.)

## Findings (append as work proceeds)

- (Phase 0 spike results go here.)

## Non-goals (YAGNI, revisit only when hit)

- ROS integration, real-robot interfaces
- GPU/parallel sim (MJX), RL training
- Upstream-PR compatibility of the package (legacy scripts stay compatible; the
  package itself is our own)
