# Verification & Validation Plan — Experimental Simulation Package (`icbench`)

Rev 1 — 2026-07-17. Verifies [requirements.md](requirements.md) against
[design.md](design.md); executed per the [plan.md](plan.md) schedule.

## 1. Strategy

**Levels** (V-model right side):

| Level | What | How | Gate |
|---|---|---|---|
| L1 Unit | One function/class in isolation | pytest, no GUI | every commit |
| L2 Integration | Cross-engine consistency; component pairs | pytest, seeded | every commit from Phase 1 |
| L3 System | Closed-loop runs end-to-end | pytest (headless) | phase exit |
| L4 Validation | "Does the physics/controller behave like physics/controllers" | scripted demonstrations + analysis in findings.md | phase exit |

**Principles**

- Every automated test with randomness uses a fixed, recorded seed.
- Tolerances: cross-engine comparisons *relative* 1e-6 (REQ-NFR-001);
  closed-loop parity tolerances stated per test and justified.
- Manual demonstrations (viewers) have written step lists and expected
  observations — subjective "looks right" is not a pass criterion; specific
  observables are.
- A failing L2 cross-engine test blocks all downstream work (it means the two
  engines disagree about the model — nothing built on top is meaningful).

## 2. Test Case Specifications

IDs: `TC-<area>-<nnn>`. Each maps to requirements (→ §3 traceability).

### Cross-engine (the anchor set)

**TC-CE-001 — Gravity vector equivalence** (L2)
For 20 seeded random q within joint limits: `pin.g(q)` vs MuJoCo
`qfrc_bias(q, dq=0)` mapped to canonical order; rel. tol 1e-6.
*Verifies REQ-NFR-001, REQ-MDL-002/003.*

**TC-CE-002 — Bias-force equivalence at speed** (L2)
Same states with seeded random dq ≠ 0: `pin.nle(q,dq)` vs `qfrc_bias(q,dq)`.
Catches Coriolis/ordering errors invisible at rest.
*Verifies REQ-NFR-001.*

**TC-CE-003 — Mass matrix equivalence** (L2)
`pin.M(q)` vs `mj_fullM`; symmetric, rel. tol 1e-6.
*Verifies REQ-NFR-001.*

**TC-CE-004 — No passive forces** (L2)
`qfrc_passive == 0` for the loaded robot (URDF damping/friction all zero, and no
compiler-injected passive terms). *Verifies REQ-MDL-004.*

### Modeling

**TC-MDL-001 — Robot loads** (L1) UR5 via `load_robot("ur5")`: nq=nv=6, meshes
resolved (no missing-file errors), preprocessed URDF has no `package://`.
*Verifies REQ-MDL-001/002.*

**TC-MDL-002 — Joint map asserted directly** (L1) The mj↔canonical map is
verified by joint *names* on both sides, not through derived quantities;
round-trip q→mj→canonical is identity. *Verifies REQ-MDL-003.*

**TC-MDL-003 — Unsupported robot rejected** (L1) A synthetic URDF with nonzero
joint damping raises a clear error naming the joint. *Verifies REQ-MDL-004.*

**TC-MDL-004 — Actuator force range** (L1) Compiled model has 6 motors;
`forcerange` equals URDF effort limits; opt-out flag removes clamping.
*Verifies REQ-MDL-005.*

### Simulation / World

**TC-SIM-001 — Free fall** (L2) Empty scene (no floor — DD-8), zero torque:
joint accelerations at t=0 equal `M⁻¹·(−g_bias)` from pinocchio; energy drift
over 1 s within integrator expectations (documented bound).
*Verifies REQ-SIM-001/006, REQ-TSK-001 (no phantom floor).*

**TC-SIM-002 — Torque causality** (L2) Constant torque on one joint (others
gravity-held via pin g(q) feedforward): that joint accelerates in the expected
direction, others stay bounded. *Verifies REQ-SIM-001/002.*

**TC-SIM-003 — Wrench frame & sign** (L2) Static known payload m attached at ee,
arm at rest (position-held): `ee_wrench()` reads +m·g in world −z within 2%;
sensor-frame rotation verified at a second, rotated pose.
*Verifies REQ-SIM-003.*

**TC-SIM-004 — External wrench primitive** (L2) Constant world-frame force at ee
deflects the gravity-compensated arm in the force direction; removing it returns
near the original pose. *Verifies REQ-SIM-005.*

**TC-SIM-005 — Determinism** (L3) Two runs, same config+seed: LogSchema arrays
bitwise identical. *Verifies REQ-SIM-007, REQ-NFR-003.*

### Control

**TC-CTL-001 — Gravity hold** (L3) Each controller variant, regulation reference
at the legacy home pose, no disturbance: ee position error stays < 5 mm for 5 s
(controller self-compensates gravity — the DD-2 acid test; a legacy-copied
controller sags and fails this immediately). *Verifies REQ-CTL-001/003.*

**TC-CTL-002 — CtrlModel correctness** (L1) M, nle, g, J, dJ, FK against direct
pinocchio calls at fixture states. Coriolis matrix property:
`Ṁ − 2C` skew-symmetric at seeded states. *Verifies REQ-CTL-002.*

**TC-CTL-003 — Golden fixtures** (L1) Ported (unchanged) controller terms match
recorded legacy values at ≥3 states including dq≠0; reformulated terms match
their documented derivation computed independently in the test.
*Verifies REQ-CTL-003.*

**TC-CTL-004 — Legacy closed-loop parity** (L3) Test harness replicating the
legacy plant exactly (`pin.aba(q,dq,τ+g)`, explicit Euler, legacy dt): ported
variant-4 controller minus its added g-compensation reproduces the legacy
script's q(t) over 2 s within 1e-9 (same math, same integrator).
*Verifies REQ-CTL-003, REQ-NFR-006 (port faithfulness).*

**TC-CTL-005 — Singularity robustness** (L1) `damped_pinv` at a singular J:
bounded output, smooth λ transition; manipulability guard activates.
*Verifies REQ-CTL-005.*

**TC-CTL-006 — Control-rate declaration** (L1) Variant 2 config defaults to
1 kHz; runner rejects non-integer dt_ctrl/dt_phys. *Verifies REQ-CTL-004,
REQ-SIM-001.*

**TC-CTL-007 — Config round-trip** (L1) Every controller/task config:
dict → dataclass → dict is lossless. *Verifies REQ-CTL-007.*

**TC-CTL-008 — Admittance composition** (L3, Phase 6) Admittance over inner
computed-torque against the wall task: contact force converges to target ±10%
without oscillation growth; wrench low-pass verified active (spectrum check on
logged wrench). *Verifies REQ-CTL-006.*

### Tasks & System

**TC-SYS-001 — Free-space tracking** (L3) Variant 3/4 on sinusoidal reference
(no disturbance): finite RMSE below a recorded baseline; no NaN; run completes.
Baseline captured once, asserted as non-regression afterwards.
*Verifies REQ-TSK-002, REQ-EXP-001.*

**TC-SYS-002 — Free-space disturbance parity** (L3) Sine ee-force via
REQ-SIM-005 primitive: response qualitatively matches legacy demo (force/error
phase relationship; documented comparison in findings.md).
*Verifies REQ-TSK-002, REQ-SIM-005.*

**TC-SYS-003 — Wall contact metrics** (L3) Variant 4 pressing to 20 N target:
steady-state |error| < 2 N, metrics dict contains the three specified fields,
computed from `ee_wrench_contact` (ground truth channel).
*Verifies REQ-TSK-003, REQ-SIM-004.*

**TC-SYS-004 — Surface slide** (L3) Constant-force slide: force RMSE finite and
below recorded baseline; path completed. *Verifies REQ-TSK-004.*

### Validation (L4 — physics behaves like physics)

**TC-VAL-001 — Stiffness/penetration trend** Impedance stiffness sweep against
the wall: steady-state penetration decreases monotonically with Kd; steady-state
force error trend documented. Numbers in findings.md.
*Validates REQ-TSK-003, A-2.*

**TC-VAL-002 — Real-time factor** 10 s free-space headless run timed; RTF ≥ 1
recorded in findings.md. *Verifies REQ-NFR-002 (A).*

**TC-VAL-003 — Effort-limit effect** Same task with/without force ranges:
saturated case shows clipped tau in log; documented.
*Validates REQ-MDL-005.*

### Visualization & Experiments

**TC-VIZ-001 — MuJoCo viewer demo** (D) Scripted: run wall-contact experiment
`--viewer mujoco`; observe arm, wall, contact — no missing geometry; sim-time/
wall-time ratio displayed ≈ 1. *Verifies REQ-VIZ-001.*

**TC-VIZ-002 — Meshcat mirror demo** (D) Same run `--viewer meshcat`: full-mesh
robot (regression check vs the merge_geometries bug: shoulder/wrist housings
present), wall box present, motion matches MuJoCo viewer run.
*Verifies REQ-VIZ-002.*

**TC-VIZ-003 — Patch module behavior** (L1) `meshcat_patch` on: pristine bundle
→ patches; patched bundle → silent no-op; unknown bundle → loud warning, no
modification. Legacy script imports it (inspection).
*Verifies REQ-VIZ-004.*

**TC-VIZ-004 — Headless** (L3) `--viewer none` completes with no display server
access (unset DISPLAY in test env). *Verifies REQ-VIZ-003.*

**TC-EXP-001 — LogSchema contract** (L1) Runner output contains exactly the
documented channels with documented shapes; `Task.metrics` and `Recorder`
consume the same object without adaptation. *Verifies REQ-EXP-001.*

**TC-REC-001 — Recorder round-trip** (L1) Record a short run; reload
`timeseries.npz` + `config.json`; arrays equal in-memory log; config
reconstructs dataclasses; `data/runs/` gitignored (inspection).
*Verifies REQ-REC-001/002.*

**TC-ANA-001 — Sweep integrity** (L3, Phase 7) 2×2 YAML grid runs headless in
parallel; aggregate contains 4 rows with correct config columns; a seeded rerun
reproduces metrics. *Verifies REQ-ANA-001, REQ-NFR-003.*

**TC-ANA-002 — Payload estimation sanity** (L3, Phase 8) Known 2 kg payload:
estimate within 5% after documented excitation trajectory; error reported vs
ground truth. *Verifies REQ-ANA-003.*

**TC-LEG-001 — Legacy untouched** (L3/D) `echo "" | python controllers/impedance_6dof.py`
exits 0, "Simulation ended" printed; meshcat shows full arm (spot demo per
release). *Verifies REQ-NFR-006.*

## 3. Traceability Matrix (requirement → design → tests → phase)

| Requirement | Design | Test cases | Phase |
|---|---|---|---|
| REQ-MDL-001 | §3.1 | TC-MDL-001 | 1 |
| REQ-MDL-002 | §3.1, DD-1 | TC-MDL-001, TC-CE-001..003 | 1 |
| REQ-MDL-003 | §3.1/3.3, DD-3 | TC-MDL-002, TC-CE-001..003 | 1 |
| REQ-MDL-004 | §3.1 | TC-MDL-003, TC-CE-004 | 1 |
| REQ-MDL-005 | §3.2, DD-9 | TC-MDL-004, TC-VAL-003 | 1 / 4 |
| REQ-SIM-001 | §3.3 | TC-SIM-001/002, TC-CTL-006 | 1 |
| REQ-SIM-002 | §3.3, DD-3 | TC-SIM-002 | 1 |
| REQ-SIM-003 | §3.3 | TC-SIM-003 | 1 |
| REQ-SIM-004 | §3.3 | TC-SYS-003 | 4 |
| REQ-SIM-005 | §3.3 | TC-SIM-004, TC-SYS-002 | 1–2 |
| REQ-SIM-006 | DD-2, §3.3 | TC-SIM-001, TC-CTL-001 | 1–2 |
| REQ-SIM-007 | §3.8 | TC-SIM-005 | 2 |
| REQ-CTL-001 | DD-2, §3.5 | TC-CTL-001 | 2 |
| REQ-CTL-002 | §3.4 | TC-CTL-002 | 2 |
| REQ-CTL-003 | §3.5 | TC-CTL-003/004, TC-CTL-001 | 2 |
| REQ-CTL-004 | §3.5/3.8 | TC-CTL-006 | 2 |
| REQ-CTL-005 | §3.5 | TC-CTL-005 | 2 |
| REQ-CTL-006 | §3.5 | TC-CTL-008 | 6 |
| REQ-CTL-007 | DD-6 | TC-CTL-007 | 2 |
| REQ-TSK-001 | DD-8, §3.6 | TC-SIM-001, inspection | 1–2 |
| REQ-TSK-002 | §3.6 | TC-SYS-001/002 | 2 |
| REQ-TSK-003 | §3.6 | TC-SYS-003, TC-VAL-001 | 4 |
| REQ-TSK-004 | §3.6 | TC-SYS-004 | 4 |
| REQ-VIZ-001 | §3.7, DD-7 | TC-VIZ-001 | 3 |
| REQ-VIZ-002 | §3.7, DD-7 | TC-VIZ-002 | 3 |
| REQ-VIZ-003 | §3.7 | TC-VIZ-004 | 3 |
| REQ-VIZ-004 | §3.7 | TC-VIZ-003 | 3 |
| REQ-EXP-001 | §3.8, DD-5 | TC-EXP-001, TC-SYS-001 | 2 |
| REQ-EXP-002 | experiments/_common.py | inspection | 2 |
| REQ-EXP-003 | §3.8 | demo w/ TC-VIZ-001 | 2–4 |
| REQ-REC-001 | §3.9, §2.3 | TC-REC-001 | 5 |
| REQ-REC-002 | .gitignore | TC-REC-001 (I) | 5 |
| REQ-ANA-001 | §3.9, DD-6 | TC-ANA-001 | 7 |
| REQ-ANA-002 | §3.9 | TC-ANA-001 variant | 7 |
| REQ-ANA-003 | §3.9 | TC-ANA-002 | 8 |
| REQ-NFR-001 | DD-1/3 | TC-CE-001..004 | 1 |
| REQ-NFR-002 | — | TC-VAL-002 | 2 |
| REQ-NFR-003 | §3.8 | TC-SIM-005, TC-ANA-001 | 2 / 7 |
| REQ-NFR-004 | requirements.txt | inspection | 0 |
| REQ-NFR-005 | tests/ | the suite itself + inspection | all |
| REQ-NFR-006 | untouched legacy | TC-LEG-001, TC-CTL-004 | all |
| REQ-NFR-007 | DD-4/5/6 | inspection at Phase 6 | 6 |

**Coverage check**: every REQ row has ≥1 verification activity; every TC verifies
≥1 REQ. Update this table in the same commit as any requirement or test change.

## 4. Regression & Release Discipline

- `./.venv/bin/pytest tests/ -q` green before every commit (Phase ≥1); a phase
  exits only with its L3/L4 items done and findings.md updated.
- Golden baselines (TC-SYS-001/004) are recorded once, committed as fixtures,
  and only changed with a justification note in the commit message.
- Demonstrations (TC-VIZ-001/002, TC-LEG-001) re-run at each phase exit that
  touches their area; observations noted in findings.md.
- Test seeds recorded inside the tests; never time- or entropy-derived.
