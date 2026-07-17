# Phase 6–8 Work Packages — ELABORATE BEFORE DISPATCH

These briefs are intentionally lighter: they depend on findings from Phases 0–4
(contact behavior, RTF, solver settings). Before dispatching any WP below, the
dispatcher expands it to the Phase 1–5 level of detail (same format: Read first /
Deliverables / Specification / Tests / Commit), folding in findings.md.

## WP-20 — Joint PD + computed torque `[parallel-ok]`

**Implements**: REQ-CTL-006 (part). **Depends on**: WP-12.

Scope when elaborating:
- `joint_pd.py`: legacy variants 1/5 reformulated (legacy line 252:
  `tau = M·ddq_des + h + M·(Kp·e + Kd·ė)` — note the legacy plant hid `g`; the
  port keeps `h = nle` which already contains g, so verify against TC-CTL-001
  gravity hold). Uses `Reference.q_des/dq_des/ddq_des`.
- `computed_torque.py`: `tau = M·(ddq_des + Kp·e + Kd·ė) + nle` — this is the
  admittance inner loop; must hold TC-CTL-001 and track a joint-space chirp
  (new unit test, tolerance recorded on first green run).
- Golden-fixture tests per WP-09 pattern; config dataclasses per WP-08.

## WP-21 — Admittance (outer/inner composition)

**Implements**: REQ-CTL-006. **Acceptance**: TC-CTL-008. **Depends on**: WP-16, WP-20.

Scope when elaborating:
- Outer loop: virtual dynamics `M_v·ẍ + D_v·ẋ + K_v·(x − x_ref) = f_meas`
  integrated at control rate → motion reference; `f_meas` = `ee_wrench_ft`
  through a 1st-order low-pass (config cutoff, default ≤ 1/10 of inner-loop
  bandwidth — compute and document).
- Inner: `ComputedTorque` at 1 kHz on the outer loop's reference (IK via
  damped_pinv velocity-level integration).
- TC-CTL-008: wall task force convergence ±10%, no oscillation growth
  (amplitude of force oscillation non-increasing over last 2 s), spectrum check
  confirms low-pass active.

## WP-22 — Controller comparison experiment

**Implements**: REQ-NFR-007 (demonstration). **Depends on**: WP-20/21.

Scope when elaborating:
- `experiments/controller_comparison.py`: same task (free-space sinusoid AND
  wall contact) × {impedance 2/3/4, joint PD, computed torque, admittance} →
  metrics table (markdown to stdout + optional record), overlay plots.
- Demonstrates REQ-NFR-007: adding a controller touched only its module +
  config — verify by inspection, note in findings.md.

## WP-23 — Sweep driver

**Implements**: REQ-ANA-001/002. **Acceptance**: TC-ANA-001. **Depends on**: WP-19.

Scope when elaborating:
- `analysis/sweep.py`: YAML → grid of config dicts (controller/task dataclasses
  via `from_dict` — zero controller changes, DD-6); `multiprocessing.Pool`
  headless runs (each with Recorder); aggregate metrics + config columns into
  `metrics.parquet` (pandas, add to requirements) + markdown report.
- Disturbance schedules (REQ-ANA-002): time-profiled wrench configs
  (const/sine/step) as sweepable task parameters over the WP-05 primitive.
- Payload-mass sweep mechanism: scene_hook adding point mass at ee (reuse
  TC-SIM-003 fixture machinery) — this is also WP-24's ground-truth knob.
- TC-ANA-001: 2×2 grid, 4 rows, seeded rerun reproduces metrics bitwise.

## WP-24 — Payload estimation testbed

**Implements**: REQ-ANA-003. **Acceptance**: TC-ANA-002. **Depends on**: WP-23.

Scope when elaborating:
- `analysis/sysid/payload_estimation.py`: excitation trajectory (joint-space
  chirp via computed torque); regressor formulation for ee payload
  (mass [+ com, then inertia] on last link) using CtrlModel terms; batch least
  squares first, then RLS variant; report estimate vs ground truth (the WP-23
  payload knob), error %, convergence time.
- TC-ANA-002: known 2 kg payload estimated within 5% after the documented
  excitation; test tolerance justified in findings.md.
- Context note for the implementer: same problem family as Rapyuta lifter
  weight estimation — the value here is a clean testbed with perfect ground
  truth; keep estimator code independent of any Rapyuta internals.
