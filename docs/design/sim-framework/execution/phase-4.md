# Phase 4 Work Packages

## WP-16 — Wall scene + wall-contact task + experiment

**Implements**: REQ-TSK-003, REQ-SIM-004. **Acceptance**: TC-SYS-003.
**Depends on**: WP-13.

**Read first**: design.md §3.6 (WallContact), §3.3 (`contacts()` accessor,
X-normal warning); verification.md TC-SYS-003.

**Deliverables**: finish `add_wall` in `icbench/sim/scene.py`, add
`World.ee_contact_wrench()` (ground-truth channel), `icbench/tasks/wall_contact.py`,
`experiments/impedance_wall_contact.py`, `tests/test_wall_contact.py`.

**Specification**
- `add_wall(spec, pos, size, friction)`: static box geom; default: vertical wall
  0.15 m in front of the ee home position (compute from FK at build time), size
  (0.02, 0.5, 0.5) m, friction 0.8.
- `World.ee_contact_wrench()`: sum `mj_contactForce` over contacts involving any
  robot geom and the wall geom, mapped to world axes. **The MuJoCo contact frame
  has the normal along X — document this in the accessor docstring and handle
  the rotation via the contact frame matrix.** Sign: robot-on-wall positive.
  Log channel name: `ee_wrench_contact` (contract, design.md §2.2).
- `WallContact(config)`: phases — approach (Cartesian ramp to a point 2 cm
  *behind* the wall surface → guarantees contact), press (hold pose ref;
  `wrench_des` ramps 0→20 N normal over 0.5 s then holds). `reference(t)`
  returns pose + wrench_des. `metrics(log)`: from `ee_wrench_contact` normal
  component after contact: steady-state error (last 1 s mean vs target),
  overshoot (%), settling time (into ±10% band). `build_scene`: floor + wall.
  `meshcat_props()`: the wall box.
- Control default: 1 kHz (REQ-CTL-004) — pass through config.
- `experiments/impedance_wall_contact.py`: `--variant {2,3,4}`, plots: normal
  force (ft + contact channels overlaid) vs t, ee position vs t.

**Tests** (TC-SYS-003, headless, variant 4, 4 s):
- run completes; contact established (max normal force > 5 N);
- steady-state |error| < 2 N vs 20 N target;
- metrics dict has exactly {steady_state_error_n, overshoot_pct, settling_time_s};
- `ee_wrench_contact` ≈ `ee_wrench_ft` normal components within 20% during
  steady press (cross-validates the two sensing paths).

**Commit**: `feat: wall-contact task with ground-truth contact wrench`

---

## WP-17 — Surface-slide task `[parallel-ok]`

**Implements**: REQ-TSK-004. **Acceptance**: TC-SYS-004. **Depends on**: WP-16.

**Deliverables**: `icbench/tasks/surface_slide.py`,
`experiments/impedance_surface_slide.py`, `tests/test_surface_slide.py`.

**Specification**
- Horizontal table surface (reuse `add_wall` with horizontal pose or add
  `add_table`); ee presses down (wrench_des = 10 N normal) while pose ref
  traverses a 0.2 m straight line in 3 s.
- Metrics: normal-force RMSE during the slide, path-following RMSE (tangential),
  path completion fraction.
- Test (TC-SYS-004): variant 4, run completes; force RMSE finite and
  < recorded baseline (record on first green run, comment in test); completion
  > 95%.

**Commit**: `feat: constant-force surface-slide task`

---

## WP-18 — Validation: stiffness sweep + effort-limit study `[parallel-ok]`

**Implements**: validation of REQ-TSK-003, REQ-MDL-005. **Acceptance**:
TC-VAL-001, TC-VAL-003 (Analysis — findings, not pytest). **Depends on**: WP-16.

**Deliverables**: `experiments/validation_stiffness_sweep.py`; findings.md
sections "Stiffness/penetration trend (TC-VAL-001)" and "Effort-limit effect
(TC-VAL-003)".

**Specification**
- Sweep variant-4 normal stiffness Kd_n ∈ {300, 900, 2500, 5000} N/m on
  WallContact (headless, fixed seed); collect steady-state penetration
  (ee position past wall surface plane) and force error.
- Assert-in-script (not pytest): penetration strictly decreasing with Kd_n;
  print table; append a markdown table + 3-sentence interpretation to
  findings.md. Note any contact chatter (R-3) and, if observed, the
  solref/solimp values that resolved it.
- Effort-limit study: WallContact with `use_effort_limits` True vs False,
  Kd_n=5000: report max |tau| per joint and whether clipping engaged
  (`tau` vs actuator forcerange in log); 3 sentences in findings.md.

**Commit**: `docs: contact validation studies (stiffness trend, effort limits)`
