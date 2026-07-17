# Phase 3 Work Packages

## WP-14 — MuJoCo native viewer `[parallel-ok]`

**Implements**: REQ-VIZ-001, REQ-VIZ-003. **Acceptance**: TC-VIZ-001 (demo),
TC-VIZ-004. **Depends on**: WP-12.

**Read first**: design.md §3.7; `icbench/runner.py` (viewer call sites).

**Deliverables**: `icbench/viz/base.py`, `icbench/viz/mujoco_viewer.py`,
runner/`_common.py` wiring, `tests/test_viz.py` (headless parts only).

**Specification**
```python
class Viewer(ABC):
    @abstractmethod
    def sync(self, state: State) -> None: ...
    @abstractmethod
    def close(self) -> None: ...

class MujocoViewer(Viewer):
    def __init__(self, world):  # mujoco.viewer.launch_passive(world.model, world.data)
    # sync(): viewer.sync(); pace to wall clock: sleep max(0, t_sim - t_wall);
    # never slow physics below data availability — drop frames instead.
```
- Wire `--viewer mujoco` in `_common.py` → construct MujocoViewer;
  `--viewer none` stays default.
- TC-VIZ-004 (automated): run a 1 s experiment with `--viewer none` in an env
  with `DISPLAY` and `WAYLAND_DISPLAY` unset (subprocess with scrubbed env) —
  exit 0.

**Manual demo (TC-VIZ-001)** — document the steps + expected observations at the
end of `docs/design/sim-framework/findings.md` §"Viewer demos", leave a checkbox
for the human: full arm visible, motion smooth, no missing geometry.

**Commit**: `feat: MuJoCo passive viewer backend`

---

## WP-15 — meshcat patch module + meshcat viewer `[parallel-ok]`

**Implements**: REQ-VIZ-002, REQ-VIZ-004. **Acceptance**: TC-VIZ-002 (demo),
TC-VIZ-003. **Depends on**: WP-12. **This WP MAY edit the legacy script** (import
switch only — explicitly allowed here, overriding the common rule).

**Read first**: `controllers/impedance_6dof.py` lines 27–52 (the existing patch
function `_patch_meshcat_viewer_bundle` and viewer init); design.md §3.7, DD-7.

**Deliverables**: `icbench/viz/meshcat_patch.py`, `icbench/viz/meshcat_viewer.py`,
edit to `controllers/impedance_6dof.py` (replace its inline patch function with
`from icbench.viz.meshcat_patch import patch_meshcat_viewer_bundle` — keep
behavior identical), `tests/test_meshcat_patch.py`.

**Specification**

`meshcat_patch.py` — move the existing function, upgrading its behavior:
```python
BROKEN = "i.push(t.geometry),n.push(t.material)"
FIXED  = "i.push(t.geometry),n.push(Array.isArray(t.material)?t.material[0]:t.material)"
def patch_meshcat_viewer_bundle() -> str:  # returns "patched"|"already-patched"|"unknown-bundle"
    # unknown-bundle => print LOUD warning (REQ-VIZ-004): bundle matches neither
    # pattern; meshes may render partially; do not modify the file.
```

`meshcat_viewer.py`:
- Calls `patch_meshcat_viewer_bundle()` on init.
- Builds pin `MeshcatVisualizer` from `bundle.pin_robot` (DAE visual model),
  `initViewer(open=False)`, `viewer.delete()` then `loadViewerModel()` (stale
  scene clear — same rationale as legacy), atexit server cleanup (mirror legacy
  lines 46–53 behavior).
- `sync(state)`: `viz.display(q)` (canonical q is what pin expects).
- Scene mirroring: read optional `task.meshcat_props() -> list[dict]` (boxes
  only: pos/size/rgba); render via `viewer["props/i"].set_object(Box…)`;
  document best-effort status (DD-7).

**Tests** (TC-VIZ-003, no browser needed — operate on a copied bundle file):
- fixture: copy the real `main.min.js` to tmp; monkeypatch the module's path
  discovery to the copy.
- pristine copy (restore BROKEN string first) → returns "patched", file contains
  FIXED; second call → "already-patched", file unchanged (hash equal).
- bundle with neither string (write garbage file) → "unknown-bundle", file
  unchanged, warning on stderr (capsys).
- Inspection item: legacy script still runs (`echo "" | ./.venv/bin/python
  controllers/impedance_6dof.py` exit 0) — include as an automated subprocess
  test.

**Manual demo (TC-VIZ-002)**: findings.md checklist — meshcat shows FULL arm
(shoulder/wrist housings present — regression vs the merge_geometries bug),
motion matches MuJoCo viewer for the same run.

**Commit**: `feat: meshcat viewer backend with centralized bundle patch`
