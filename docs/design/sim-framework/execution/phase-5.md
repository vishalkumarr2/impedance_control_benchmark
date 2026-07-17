# Phase 5 Work Package

## WP-19 — Recorder + gitignore

**Implements**: REQ-REC-001, REQ-REC-002. **Acceptance**: TC-REC-001.
**Depends on**: WP-12 (LogSchema contract).

**Read first**: design.md §2.3 (run artifact layout — exact), §3.9.

**Deliverables**: `icbench/recording.py`, `.gitignore` entry `data/runs/`,
`--record` wiring in `experiments/_common.py`, `tests/test_recording.py`.

**Specification**
```python
class Recorder:
    def save(self, experiment_name: str, config: dict, log: dict, metrics: dict,
             figures: list = ()) -> Path:
        # data/runs/<UTC yyyymmddTHHMMSSZ>_<experiment_name>/
        #   config.json   — config dicts + seed + git SHA (subprocess: git rev-parse HEAD;
        #                    "unknown" on failure, never crash)
        #   timeseries.npz — np.savez_compressed(**log)
        #   metrics.json
        #   fig_<i>.png   — for each matplotlib figure passed
```
- Config dict comes from the dataclass `to_dict()`s (WP-08); include
  `{"controller": ..., "task": ..., "seed": ..., "git_sha": ...}`.
- `--record` in `_common.py`: after run, call Recorder with the same figures the
  interactive path shows; print the run dir path.

**Tests** (TC-REC-001)
- Record a 0.5 s headless free-space run into a tmp-redirected `data/runs`
  (monkeypatch base dir): reload npz — every array equal to in-memory log;
  config.json round-trips to the same dataclasses via `from_dict`; metrics.json
  equals metrics.
- Inspection-as-test: assert `data/runs/` appears in `.gitignore`.

**Commit**: `feat: opt-in run recorder with reproducible artifacts`
