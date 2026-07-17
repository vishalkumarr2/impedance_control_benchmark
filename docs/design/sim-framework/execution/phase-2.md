# Phase 2 Work Packages

## WP-07 — `icbench/control/ctrl_model.py` `[parallel-ok]`

**Implements**: REQ-CTL-002. **Acceptance**: TC-CTL-002. **Depends on**: WP-06.

**Read first**: design.md §3.4; legacy `controllers/impedance_6dof.py` lines
130–200 (what quantities controllers consume).

**Deliverables**: `icbench/control/ctrl_model.py`, `tests/test_ctrl_model.py`.

**Specification**
```python
class CtrlModel:
    def __init__(self, bundle, ee_frame: str = "ee_link"): ...
    def mass(self, q) -> np.ndarray            # pin.crba, symmetrized
    def nle(self, q, dq) -> np.ndarray         # pin.nonLinearEffects
    def gravity(self, q) -> np.ndarray         # pin.computeGeneralizedGravity
    def coriolis(self, q, dq) -> np.ndarray    # pin.computeCoriolisMatrix (matrix!)
    def frame_jacobian(self, q) -> np.ndarray  # LOCAL_WORLD_ALIGNED, ee frame
    def frame_jacobian_dot(self, q, dq) -> np.ndarray
    def fk(self, q) -> "pin.SE3"
```
All canonical order. Stateless between calls (each method does its own
`forwardKinematics`/compute as needed — correctness over micro-perf).

**Tests**
- Each method vs direct pinocchio calls at 3 fixture states (one = legacy home
  pose `[0, -1, 1.2, -(π+0.2), -π/2, 0]`), exact match.
- Coriolis property: `Ṁ − 2C` skew-symmetric at 5 seeded random (q, dq):
  `M_dot ≈ (M(q + dq·h) − M(q))/h` with h=1e-7, assert
  `‖(Ṁ − 2C) + (Ṁ − 2C)ᵀ‖ < 1e-4` (loose tol: finite difference).

**Commit**: `feat: control-side pinocchio model wrapper`

---

## WP-08 — Controller ABC + config dataclasses `[parallel-ok]`

**Implements**: REQ-CTL-001, REQ-CTL-007. **Acceptance**: TC-CTL-007.
**Depends on**: WP-06.

**Deliverables**: `icbench/control/base.py`, `tests/test_control_base.py`.

**Specification**
```python
class ControllerConfig:  # base for all controller configs
    @classmethod
    def from_dict(cls, d: dict): ...   # dataclasses.fields-driven; unknown keys -> ValueError
    def to_dict(self) -> dict: ...

class Controller(ABC):
    control_rate_hz: float             # class attr, overridable via config
    def __init__(self, ctrl_model, config): ...
    @abstractmethod
    def reset(self, state0: State) -> None: ...
    @abstractmethod
    def compute(self, state: State, ref: Reference, t: float, dt: float) -> np.ndarray:
        """Returns TOTAL joint torque (canonical order) INCLUDING gravity/nle
        compensation. The plant is real — see DD-2."""
```

**Tests**: TC-CTL-007 — a sample config: dict → dataclass → dict lossless;
unknown key raises with the key named.

**Commit**: `feat: controller ABC and config dataclass machinery`

---

## WP-09 — Task-space impedance (reformulated) + damped pinv

**Implements**: REQ-CTL-003/004/005. **Acceptance**: TC-CTL-003/005/006.
**Depends on**: WP-07, WP-08.

**Read first**: legacy `controllers/impedance_6dof.py` lines 236–276 (the three
variants); design.md §3.5; the two verified legacy bugs in plan.md
§"Key technical facts".

**Deliverables**: `icbench/control/impedance.py`,
`icbench/control/kinematics.py` (damped pinv), `tests/test_impedance.py`,
derivation notes appended to findings.md.

**Specification**

`kinematics.py`:
```python
def damped_pinv(J, lam0=0.05, w_threshold=0.02) -> np.ndarray:
    # J^T (J J^T + λ² I)^-1 ; λ = lam0 * (1 - w/w_threshold) if w < w_threshold else ~0
    # w = sqrt(det(J J^T)) manipulability
```

`impedance.py` — `TaskSpaceImpedance(ctrl_model, config)`, `variant ∈ {2,3,4}`:
- Error terms exactly as legacy (lines 236–237): `x_err = [x_des − x;
  rpy(R_des·Rᵀ)]`, `v_err = v_des − J·dq`.
- Variant 4 (`control_rate_hz=100` default): `tau = Jᵀ(Kd·x_err + Dd·v_err) + g(q)`
  — the `+g(q)` replaces the legacy plant's hidden compensation (DD-2).
- Variant 3 (100 Hz): as legacy `inverse_dyn + interaction_port` BUT: gravity
  handled explicitly (net effect documented in the derivation), the legacy term
  `(h − g − M·Ji·dJ) @ Ji @ v_des` (a vector−matrix broadcast bug, legacy lines
  262/270) replaced by `(C(q,dq) − M·J⁺·dJ)·J⁺·v_des` with `C` the Coriolis
  MATRIX; all `inv(J)` → `damped_pinv(J)`.
- Variant 2 (`control_rate_hz=1000`): same corrections + the `Md`/inertia-ratio
  port from legacy lines 96–102, 255–260.
- Gains in `ImpedanceConfig` (defaults = legacy values per variant).
- Write the variant-by-variant derivation (legacy form → reformulated form,
  including where `g` moves and why the Coriolis term changes) into findings.md
  §"Impedance reformulation (WP-09)". This is a REQUIRED deliverable (REQ-CTL-003
  is verified partly by Analysis).

**Tests**
- TC-CTL-003 golden fixtures: at ≥3 states incl. dq≠0 (fixture file with
  hardcoded arrays): unchanged terms (x_err, v_err, Jᵀ·F mapping for variant 4
  minus the g term) equal values computed by transliterating the legacy lines
  inside the test; reformulated terms equal an independent in-test computation
  of the corrected formula (do not import the implementation's own helper for
  the expected value — write the formula out).
- TC-CTL-005: singular pose (elbow straight): `damped_pinv` bounded
  (‖J⁺‖ < 1e3), continuous around the threshold (sweep w through w_threshold,
  no jump > 10% between adjacent samples).
- TC-CTL-006: variant 2 config default rate 1000 Hz; variants 3/4 → 100 Hz.

**Commit**: `feat: reformulated task-space impedance variants with damped pinv`

---

## WP-10 — Legacy closed-loop parity test

**Implements**: REQ-CTL-003 (port faithfulness), REQ-NFR-006. **Acceptance**:
TC-CTL-004. **Depends on**: WP-09.

**Deliverables**: `tests/test_legacy_parity.py`.

**Specification** — replicate the legacy PLANT exactly inside the test (this is
the one place the hidden-gravity plant is intentionally reproduced):
```python
# legacy plant replica: ddq = pin.aba(model, data, q, dq, tau + g(q)); Euler, dt=0.01
```
Run 200 steps (2 s) from the legacy home pose with regulation reference:
1. Reference trajectory: transliterate legacy variant-4 controller inline in the
   test (lines 273–276: `tau = Jᵀ(Kd·x_err + Dd·v_err)`, legacy gains).
2. Candidate: `TaskSpaceImpedance(variant=4).compute(...)` **minus**
   `ctrl_model.gravity(q)` (removing the package's explicit g-comp recovers the
   legacy torque in the legacy plant).
Assert `‖q_legacy(t) − q_candidate(t)‖∞ < 1e-9` at every step (identical math,
identical integrator ⇒ near-machine agreement).

**Commit**: `test: closed-loop parity of ported impedance vs legacy plant replica`

---

## WP-11 — Task ABC + free-space task `[parallel-ok]`

**Implements**: REQ-TSK-001/002. **Depends on**: WP-08 (Reference type from WP-05's types.py).

**Read first**: design.md §3.6; legacy lines 88–95 (dt), 172–186 (dynamic_ref),
229–233 (disturbance).

**Deliverables**: `icbench/tasks/base.py`, `icbench/tasks/free_space.py`,
`tests/test_tasks.py`.

**Specification**
```python
class Task(ABC):
    def build_scene(self, spec) -> None: ...      # ALL env geometry (incl. floor)
    @abstractmethod
    def reference(self, t: float) -> Reference: ...
    @abstractmethod
    def metrics(self, log: dict[str, np.ndarray]) -> dict: ...
    def disturbance(self, t: float) -> np.ndarray | None:  # world wrench at ee or None
        return None

@dataclass
class FreeSpaceConfig: mode: str = "regulation"  # or "sinusoid"
    # sinusoid: legacy ref_freq=0.3 Hz, ref_ampl=0.10 m on y/z (lines 173-185)
    # disturbance: legacy fe_amp = 9.80665*5.0 N, fe_angf = 3.0 (lines 89-91, 231-233)
    disturbance_on: bool = False
```
`FreeSpace.build_scene` adds NOTHING (empty world — DD-8). `metrics`: tracking
RMSE of `ee_pos` vs `ref_pos` (skip first 0.2 s), max abs error.

**Tests**: reference continuity (‖ref(t+h)−ref(t)‖ → 0 as h→0 at 10 seeded t);
regulation mode returns constant pose; disturbance returns legacy sine values at
t = 0.5 s (hand-computed expected value in test).

**Commit**: `feat: task ABC and free-space task with legacy-parity disturbance`

---

## WP-12 — Runner + LogSchema + determinism

**Implements**: REQ-EXP-001, REQ-SIM-007, REQ-NFR-003. **Acceptance**:
TC-EXP-001, TC-SIM-005, TC-SYS-001, TC-SYS-002, TC-CTL-001.
**Depends on**: WP-09, WP-11 (and WP-05).

**Read first**: design.md §2.2 (LogSchema — channel names are a CONTRACT), §3.8.

**Deliverables**: `icbench/runner.py`, `tests/test_runner.py`,
`tests/test_system_freespace.py`.

**Specification**
```python
class Experiment:
    def __init__(self, bundle, task, controller, viewer=None, dt_phys=0.002, seed=0): ...
    def run(self, duration: float) -> tuple[dict[str, np.ndarray], dict]:
        # returns (log, metrics)
```
- dt_ctrl = 1/controller.control_rate_hz; `n_substeps = round(dt_ctrl/dt_phys)`;
  raise if not integer within 1e-9.
- Loop exactly as design.md §5 sequence diagram; task.disturbance(t) applied via
  `World.apply_external_wrench` each control step (or cleared when None).
- Log: preallocate/append ALL channels of design.md §2.2 with those exact names.
- Viewer optional (None = headless); `viewer.sync(state)` best-effort.

**Tests**
- TC-EXP-001: log has exactly the documented channels & shapes; a
  `FreeSpace.metrics(log)` call consumes it unmodified.
- TC-CTL-001 gravity hold: variant 4, regulation at home pose, 5 s headless:
  max ‖ee_pos − ref‖ < 5 mm after the first 0.5 s. (THE acid test for DD-2.)
- TC-SYS-001: variant 3 and 4, sinusoid mode, 4 s: RMSE finite, < recorded
  baseline (record baseline on first green run into the test as a constant with
  a comment; treat later regressions > 20% as failures).
- TC-SYS-002: disturbance_on: log `ee_wrench_ft` shows the injected sine
  (correlate: peak frequency ≈ fe_angf/2π Hz).
- TC-SIM-005 determinism: two identical runs (same seed) → all log arrays
  `np.array_equal` (bitwise).

**Commit**: `feat: experiment runner with fixed log schema; system tests (gravity hold, tracking, determinism)`

---

## WP-13 — experiments/_common.py + free-space script + README

**Implements**: REQ-EXP-002/003, REQ-NFR-002 (measurement). **Acceptance**:
TC-VAL-002 + inspection. **Depends on**: WP-12.

**Deliverables**: `experiments/_common.py`, `experiments/impedance_free_space.py`,
README section, findings.md RTF entry.

**Specification**
- `_common.py`: `build_argparser()` with `--seed int=0`, `--viewer
  {none,mujoco,meshcat}=none` (choices exist now; mujoco/meshcat raise
  NotImplementedError until Phase 3), `--record` (stored, unused until Phase 5),
  `--duration float`.
- `impedance_free_space.py`: `--variant {2,3,4}`, builds Experiment, runs,
  prints metrics dict, shows matplotlib figure (ee error + wrench channels vs t)
  unless `--no-plot`.
- Time a 10 s headless run (`time.perf_counter` around `run`) and append the
  real-time factor to findings.md §"RTF (WP-13)" (TC-VAL-002).
- README.md: new section "Simulation package (icbench)" — one paragraph +
  run example command; state legacy scripts unchanged.

**Acceptance**: script runs green end-to-end headless; README updated;
RTF recorded. **Commit**: `feat: free-space experiment script and shared CLI conventions`
