# Phase 0 Work Packages

## WP-01 — Scaffold, dependencies, smoke test

**Implements**: REQ-NFR-004, REQ-NFR-005 (foundation). **Depends on**: none.

**Read first**: `requirements.txt`, `docs/design/sim-framework/design.md` §"Package layout" (in plan.md).

**Deliverables**
- `requirements.txt`: append `mujoco==3.10.0` and `pytest==8.*` (keep existing pins untouched)
- Run `./.venv/bin/pip install -r requirements.txt` and confirm success
- Directories with empty `__init__.py`: `icbench/`, `icbench/sim/`, `icbench/control/`, `icbench/tasks/`, `icbench/viz/`, `icbench/analysis/`
- `experiments/` and `tests/` directories (no `__init__.py` needed in `experiments/`)
- `tests/test_smoke.py`:

```python
def test_imports():
    import mujoco
    import pinocchio
    import icbench
    assert mujoco.__version__ == "3.10.0"
```

**Acceptance**: `./.venv/bin/pytest tests/ -q` → 1 passed.
**Commit**: `feat: package scaffold, mujoco dependency, smoke test`

---

## WP-02 — Spike: URDF → MuJoCo compile, EGL headless (throwaway code, durable findings)

**Implements**: resolves design risks R-1, R-2, R-5. **Depends on**: WP-01.

**Read first**: `docs/design/sim-framework/design.md` §3.1 and §6 (risks).

**Context**: MuJoCo cannot resolve `package://` mesh URIs or load DAE meshes.
The UR5 URDF lives inside the venv (find it):
`./.venv/lib/python3.12/site-packages/cmeel.prefix/share/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf`
(verify exact name with `find .venv -name "ur5*.urdf"`). All mesh references are
`package://example-robot-data/...`; the share root is
`.../cmeel.prefix/share/example-robot-data/` — the path prefix pinocchio's
loaders use.

**Steps** (script in scratch space or `/tmp`, NOT committed):
1. Read the URDF; rewrite `package://example-robot-data/` → absolute share path;
   write to a temp file.
2. Try `spec = mujoco.MjSpec.from_file(tmp_urdf)`; if the mjSpec URDF path fails,
   try `mujoco.MjModel.from_xml_path(tmp_urdf)` and note which works.
3. Report: does compilation succeed with DAE `<visual>` entries present
   (discardvisual behavior)? If not, strip `<visual>` elements and retry.
4. Check compiler behavior with inertia balancing disabled if the API exposes it
   on this path (`spec.compiler.balanceinertia = False` or MJCF equivalent);
   note whether UR5 inertias compile without "fixing".
5. Print and record: joint names in MuJoCo order; `model.nq`, `model.nv`;
   number of actuators (expect 0); mesh count.
6. EGL: `MUJOCO_GL=egl ./.venv/bin/python -c "import mujoco; r=mujoco.Renderer(model); ..."`
   render one offscreen frame from the compiled model; note pass/fail.

**Deliverables**: `docs/design/sim-framework/findings.md` — new section
"Phase 0 spike (WP-02)" answering ALL of: which load API works for URDF; visual/DAE
handling; balanceinertia behavior + whether UR5 inertias are consistent; MuJoCo
joint order vs URDF order (list both); EGL result. If any answer contradicts a
design assumption (design.md §3.1), say so explicitly in the findings — do not
silently adapt.

**Acceptance**: findings.md section exists and answers all six questions;
no code committed other than findings.md.
**Commit**: `docs: phase 0 spike findings (URDF->MuJoCo, EGL)`
