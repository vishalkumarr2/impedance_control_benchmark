# Execution Work Packages — Dispatch Index

Rev 1 — 2026-07-17. Fifth document of the set (requirements → design →
verification → plan → **execution**). Each work package (WP) is a self-contained
brief executable by an agent with **zero conversation context** (Sonnet-class).

## Dispatch protocol

To dispatch a WP to a subagent, give it exactly:

1. The **Common Briefing** below (paste verbatim),
2. The WP section from the phase file (e.g. `phase-1.md` § WP-03),
3. Nothing else. The WP lists every file the agent must read.

Rules for the dispatcher:
- Respect `Depends on:` — never dispatch a WP before its dependencies are merged.
- WPs within a phase marked `[parallel-ok]` may run concurrently in separate
  worktrees; all others are sequential.
- After each WP: review the diff, run the acceptance commands yourself, then merge.

## Common Briefing (paste into every subagent prompt)

```
You are implementing one work package of the `icbench` experimental robot-arm
simulation package inside the repo at ~/ai/test/impedance_control_benchmark.

Environment:
- Python: ./.venv/bin/python   Tests: ./.venv/bin/pytest tests/ -q
- All deps are pip-installed in .venv; do not install anything not listed in
  your work package.

Project conventions (violations are bugs):
- SI units (m, s, rad, N, N·m). Canonical joint order = URDF/pinocchio order;
  the MuJoCo<->canonical index map is DATA (built in icbench/models.py), never
  assumed identity. World owns the remap; controllers/viewers only ever see
  canonical order.
- Task-space quantities: pinocchio LOCAL_WORLD_ALIGNED frame. Wrenches:
  world-aligned axes, robot-on-environment positive.
- GRAVITY CONVENTION: the plant is real (MuJoCo applies gravity; no hidden
  compensation anywhere in World). Every controller returns TOTAL torque
  including its own gravity/nle compensation. The legacy script
  controllers/impedance_6dof.py line 291 uses aba(q,dq,tau+g) — a hidden
  compensation you must NOT replicate in the package.
- Config: every controller/task takes a @dataclass config constructible from a
  plain dict.
- Seeds: any randomness uses a fixed, recorded seed.

Hard rules:
- NEVER modify controllers/*.py (legacy demos) unless the WP explicitly says so.
- NEVER modify files outside the WP's "Deliverables" list.
- Write tests FIRST where the WP specifies tests; run the full suite before
  finishing; everything green.
- Commit style: conventional commits (feat:/test:/docs:), one commit per WP
  unless the WP says otherwise. Do not push.
- If you hit a contradiction between your WP and the code you find, STOP and
  report it instead of improvising.

Reference docs in docs/design/sim-framework/ (read only the sections your WP
cites): requirements.md (REQ-IDs), design.md (DD-IDs, interfaces), verification.md
(TC-IDs = your acceptance tests), plan.md (context).
```

## WP index and requirement mapping

| WP | Phase file | Title | Implements | Acceptance | Depends on | Parallel |
|---|---|---|---|---|---|---|
| WP-01 | phase-0.md | Scaffold, deps, smoke test | REQ-NFR-004/005 (part) | TC (smoke) | — | no |
| WP-02 | phase-0.md | Spike: URDF→MuJoCo, EGL | resolves R-1/R-2/R-5 | findings.md entries | WP-01 | no |
| WP-03 | phase-1.md | models.py: preprocess, registry, joint map | REQ-MDL-001..004 | TC-MDL-001..003 | WP-02 | no |
| WP-04 | phase-1.md | scene.py: actuators, ee sensor | REQ-MDL-005 | TC-MDL-004 | WP-03 | yes |
| WP-05 | phase-1.md | types.py + world.py: World core | REQ-SIM-001..003/005/006 | TC-SIM-001..004 | WP-03, WP-04 | no |
| WP-06 | phase-1.md | Cross-engine anchor tests | REQ-NFR-001 | TC-CE-001..004 | WP-05 | no |
| WP-07 | phase-2.md | ctrl_model.py | REQ-CTL-002 | TC-CTL-002 | WP-06 | yes |
| WP-08 | phase-2.md | Controller ABC + configs | REQ-CTL-001/007 | TC-CTL-007 | WP-06 | yes |
| WP-09 | phase-2.md | Impedance (reformulated) + damped pinv | REQ-CTL-003/004/005 | TC-CTL-003/005/006 | WP-07, WP-08 | no |
| WP-10 | phase-2.md | Legacy closed-loop parity test | REQ-CTL-003, REQ-NFR-006 | TC-CTL-004 | WP-09 | no |
| WP-11 | phase-2.md | Task ABC + free_space | REQ-TSK-001/002 | (via WP-12 TCs) | WP-08 | yes |
| WP-12 | phase-2.md | runner.py + LogSchema + determinism | REQ-EXP-001, REQ-SIM-007, REQ-NFR-003 | TC-EXP-001, TC-SIM-005, TC-SYS-001/002, TC-CTL-001 | WP-09, WP-11 | no |
| WP-13 | phase-2.md | experiments/_common + free-space script + README | REQ-EXP-002/003, REQ-NFR-002 | TC-VAL-002, inspection | WP-12 | no |
| WP-14 | phase-3.md | MuJoCo viewer | REQ-VIZ-001/003 | TC-VIZ-001/004 | WP-12 | yes |
| WP-15 | phase-3.md | meshcat_patch module + meshcat viewer | REQ-VIZ-002/004 | TC-VIZ-002/003 | WP-12 | yes |
| WP-16 | phase-4.md | Wall scene + wall_contact task + experiment | REQ-TSK-003, REQ-SIM-004 | TC-SYS-003 | WP-13 | no |
| WP-17 | phase-4.md | surface_slide task | REQ-TSK-004 | TC-SYS-004 | WP-16 | yes |
| WP-18 | phase-4.md | Validation: stiffness sweep + effort-limit study | REQ-MDL-005 (val) | TC-VAL-001/003 | WP-16 | yes |
| WP-19 | phase-5.md | Recorder + gitignore | REQ-REC-001/002 | TC-REC-001 | WP-12 | no |
| WP-20 | phase-6-8.md | joint_pd + computed_torque | REQ-CTL-006 (part) | zoo unit tests | WP-12 | yes |
| WP-21 | phase-6-8.md | Admittance (outer/inner) | REQ-CTL-006 | TC-CTL-008 | WP-16, WP-20 | no |
| WP-22 | phase-6-8.md | Controller comparison experiment | REQ-NFR-007 (demo) | inspection | WP-20/21 | no |
| WP-23 | phase-6-8.md | Sweep driver (YAML, parallel) | REQ-ANA-001/002 | TC-ANA-001 | WP-19 | no |
| WP-24 | phase-6-8.md | Payload estimation testbed | REQ-ANA-003 | TC-ANA-002 | WP-23 | no |

Reverse map (requirement → WP): every REQ appears in exactly one WP's
"Implements" column above except cross-cutting REQ-NFR-005 (test discipline —
every WP) and REQ-NFR-006 (legacy untouched — enforced by the Common Briefing
hard rule + TC-LEG-001 run at phase exits by the dispatcher).

**Phases 6–8 briefs are intentionally lighter** (marked ELABORATE-BEFORE-DISPATCH):
they depend on findings from phases 0–4; the dispatcher expands them (same format)
before dispatching, using accumulated findings.md.
