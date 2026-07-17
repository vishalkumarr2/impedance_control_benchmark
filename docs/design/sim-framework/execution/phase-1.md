# Phase 1 Work Packages

Prerequisite for all WPs here: read `docs/design/sim-framework/findings.md`
§"Phase 0 spike" — it overrides any assumption below that it contradicts.

## WP-03 — `icbench/models.py`: URDF preprocessing, registry, joint map

**Implements**: REQ-MDL-001..004. **Acceptance tests**: TC-MDL-001/002/003.
**Depends on**: WP-02.

**Read first**: design.md §3.1; verification.md TC-MDL-001..003; findings.md.

**Deliverables**: `icbench/models.py`, `tests/test_models.py`.

**Specification**
```python
@dataclass(frozen=True)
class RobotBundle:
    name: str
    pin_robot: "pinocchio.RobotWrapper"   # with visual (DAE) model for meshcat
    urdf_mj_path: str                     # preprocessed URDF for MuJoCo
    mesh_dir: str
    joint_names: list[str]                # canonical (pinocchio) order, movable joints only
    mj_from_canon: np.ndarray             # index arrays, see below
    canon_from_mj: np.ndarray
    effort_limits: np.ndarray             # (nv,) from URDF <limit effort>

def load_robot(name: str = "ur5") -> RobotBundle: ...
```
- Locate URDF + share dir as in WP-02 (make the lookup a helper; don't hardcode
  the venv path — derive from `example_robot_data`'s installed location or by
  importing pinocchio's example-robot-data loader package).
- Preprocess to a cached temp file (`tempfile.mkdtemp` per process is fine):
  rewrite `package://example-robot-data/` → share path; strip `<visual>` blocks
  iff findings.md says MuJoCo needs that.
- Build pin robot via `example_robot_data.load(name)` (gives visual model too).
- Build MuJoCo model using the API findings.md validated; disable inertia
  balancing per findings.
- Joint map: match by joint NAME between `pin_robot.model.names` (skip
  "universe") and MuJoCo joint names. Raise `ValueError` naming the joint on any
  mismatch. Raise on `pin.nq != pin.nv` (continuous joints unsupported).
- Effort limits: parse URDF `<limit effort="...">` per joint (canonical order).
- Guard (REQ-MDL-004): parse URDF `<dynamics>`; raise if damping/friction ≠ 0.

**Tests** (write first)
- TC-MDL-001: `load_robot("ur5")`: nq==nv==6; preprocessed file contains no
  `package://`; MuJoCo model compiles (import the MuJoCo build here).
- TC-MDL-002: joint-name lists on both sides match under the map;
  `canon_from_mj[mj_from_canon] == arange(6)` (round-trip identity).
- TC-MDL-003: synthetic minimal URDF (write in test tmpdir) with
  `<dynamics damping="0.5">` → `ValueError` naming the joint.

**Commit**: `feat: robot model registry with URDF preprocessing and joint map`

---

## WP-04 — `icbench/sim/scene.py`: actuators + ee sensor `[parallel-ok]`

**Implements**: REQ-MDL-005. **Acceptance**: TC-MDL-004. **Depends on**: WP-03.

**Read first**: design.md §3.2; findings.md (mjSpec editing capabilities).

**Deliverables**: `icbench/sim/scene.py`, `tests/test_scene.py`.

**Specification**
```python
def attach_actuators(spec, bundle, use_effort_limits: bool = True) -> None:
    # one <motor> per movable joint; gear=1; ctrl = joint torque [N·m]
    # forcerange = ±effort_limits[i] when use_effort_limits (DD-9)

def add_ee_site_and_ft_sensor(spec, body_name: str = "ee_link", site_name: str = "ee") -> None:
    # site at the ee body frame origin; force + torque sensors on that site

def add_floor(spec) -> None: ...        # called ONLY by Tasks (DD-8)
def add_wall(spec, pos, size, friction) -> None: ...  # stub now, used in WP-16
```
- Work on the mjSpec level (edit → `spec.compile()`); if findings.md says mjSpec
  URDF editing is unavailable, the documented fallback is: compile URDF →
  `mj_saveLastXML` → parse MJCF string → inject XML elements → recompile. Follow
  findings.md.

**Tests**
- TC-MDL-004: compiled model: `model.nu == 6`; `actuator_forcerange` rows equal
  ±effort limits; with `use_effort_limits=False` ranges are unbounded (0,0 or
  ±inf per MuJoCo convention — assert actual behavior and document in a comment).
- Sensor: model has force+torque sensors on site "ee"; floor absent unless
  `add_floor` called.

**Commit**: `feat: scene builders (torque actuators with effort limits, ee F/T sensor)`

---

## WP-05 — `icbench/types.py` + `icbench/sim/world.py`: World core

**Implements**: REQ-SIM-001/002/003/005/006. **Acceptance**: TC-SIM-001..004.
**Depends on**: WP-03, WP-04.

**Read first**: design.md §2.1, §3.3; verification.md TC-SIM-001..004;
Conventions in plan.md.

**Deliverables**: `icbench/types.py` (State, Reference dataclasses exactly per
design.md §2.1), `icbench/sim/world.py`, `tests/test_world.py`.

**Specification**
```python
class World:
    def __init__(self, bundle, scene_hook=None, dt_phys=0.002):
        # scene_hook: callable(spec) -> None, applied before compile (Tasks use this)
    def reset(self, q0: np.ndarray) -> None: ...          # canonical order
    def step(self, tau: np.ndarray, n_substeps: int) -> None:
        # canonical -> mj order -> data.ctrl; mj_step * n_substeps
        # re-apply any active external wrench each substep
    def state(self) -> State: ...                          # canonical
    def ee_wrench(self) -> np.ndarray:
        # (6,) [force, torque]; sensor frame -> world axes via site xmat;
        # sign: robot-on-environment positive (flip sensor sign accordingly —
        # MuJoCo F/T sensors report child-on-parent; VERIFY empirically in the
        # static test and document the flip in a comment)
    def contacts(self) -> list:  # raw mjContact view; docstring MUST state
        # MuJoCo contact frame: X axis is the contact normal
    def apply_external_wrench(self, wrench: np.ndarray, body: str = "ee_link") -> None:
        # sets data.xfrc_applied[body_id]; persists until cleared
    def clear_external_wrench(self) -> None: ...
```
- No gravity logic anywhere (REQ-SIM-006). No floor by default (DD-8).

**Tests** (empty scene unless stated)
- TC-SIM-001 free fall: `reset(q_home)`, zero torque, one step: `ddq` from
  finite-difference matches `pin: aba(q, 0, 0)` within 1e-3 rel. (uses pin as
  oracle); 1 s of falling produces no NaN and no contacts.
- TC-SIM-002 torque causality: gravity-feedforward `tau = pin.g(q)` holds the
  arm (‖dq‖ < 1e-2 after 0.5 s); adding +5 N·m on joint 0 accelerates joint 0
  positive.
- TC-SIM-003 wrench frame/sign: attach a 2 kg point mass to the ee body in the
  scene_hook; hold arm with gravity feedforward computed for the *unloaded*
  model plus a position-holding high-gain joint PD (keep it crude and stiff —
  this is a fixture, not a controller); after settling, `ee_wrench()` force ≈
  (0, 0, −2·9.81) world axes within 5% → robot-on-env positive means the sensor
  reads the load's weight with the documented sign. Repeat at a 90°-rotated
  wrist pose: still world-aligned (this catches missing xmat rotation).
- TC-SIM-004 external wrench: gravity-held arm; `apply_external_wrench`
  (+20 N world x at ee) → ee moves +x; `clear_external_wrench()` → returns
  toward start (‖err‖ shrinking for 1 s).

**Commit**: `feat: World core (canonical remap, wrench sensing, external wrench)`

---

## WP-06 — Cross-engine anchor tests

**Implements**: REQ-NFR-001 (+ REQ-MDL-004 via TC-CE-004). **Acceptance**:
TC-CE-001..004. **Depends on**: WP-05.

**Read first**: verification.md §TC-CE-*; design.md DD-3.

**Deliverables**: `tests/test_cross_engine.py`.

**Specification** — all with `rng = np.random.default_rng(20260717)`; 20 states
sampled within joint limits; comparisons in canonical order; relative tolerance
1e-6 (`np.testing.assert_allclose(..., rtol=1e-6, atol=1e-10)`).
- TC-CE-001: `pin.computeGeneralizedGravity(q)` vs mj `qfrc_bias` with
  `data.qvel[:]=0` after `mj_forward` (map order; mind sign conventions —
  MuJoCo `qfrc_bias` = C(q,dq)dq + g(q) with *positive* sign; pinocchio
  `nle`/`g` likewise; assert equality, and if a systematic sign flip appears,
  STOP and report rather than absorbing it silently).
- TC-CE-002: nonzero seeded `dq`: `pin.nonLinearEffects(q, dq)` vs `qfrc_bias`.
- TC-CE-003: `pin.crba(q)` (symmetrize) vs `mj_fullM` dense.
- TC-CE-004: `data.qfrc_passive == 0` exactly, at all sampled states.

**Acceptance**: suite green; runtime of these tests < 10 s.
**Commit**: `test: cross-engine consistency anchors (gravity, nle, mass matrix, passive)`
