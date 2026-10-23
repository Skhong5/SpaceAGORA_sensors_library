# Author verification: coupled multirate integration

Prepared 2026-10-23. This is author verification, not an independent review.

## Scope and base

The user explicitly included constellation-plan C-6 in 0.7.0 and authorized
continued implementation. The isolated branch starts from station/data PR #518
at `dc2edc342e038dac4454dbaab42d0b1774b84e34` and merges event-output PR #515
at `aea9fa928cf04220411084bac0b3d6ae6a011a09`. Their combined base is
`f4f138ef3c0c90d99a9de047a381787bfb292862`. This work belongs after 0.6.1.

The existing Strang route refuses interlinks and batteries, omits their rates,
and revisits callback intervals. The new opt-in `:multirate_mri` mode uses
OrdinaryDiffEqMultirate 3.0.3's MRIGARKERK45a. It advances one coupled state with
one callback timeline. Dynamic forces and kinematics are slow; control,
interlink forces, battery energy, data stores and ground-station counters are
fast. Both derivative buffers are fully initialized. Initial aerodynamic
output is published after its slow-force evaluation.

The original solver modes retain their implementations and defaults. MRI is
explicit, with RK4 microsteps per stage; the original slow/fast solver selectors
apply only to Strang. Stiff problems still need an appropriate full-state solver.
Articulated, compliant, robot-arm and externally propagated configurations remain
unsupported by MRI. No speedup or scientific-baseline acceptance is claimed.

## Changed Files

- `src/orchestration/simulation/config/solver_settings.jl`: admit and document the opt-in mode.
- `src/orchestration/simulation/engine/setup.jl`: import the MRI algorithm.
- `src/orchestration/simulation/engine/dynamics_rhs.jl`: complete fast/slow wrappers, including power, data and interlinks.
- `src/orchestration/simulation/engine/execution.jl`: fast-first MRI problem, robot-arm refusal and snapshot of environment-derived solver settings.
- `src/orchestration/simulation/engine/solver_policy.jl`: one adaptive integrator with the configured macro-step cap and truthful solver metadata.
- `src/orchestration/simulation/callbacks/density_callbacks/assembly.jl`: preserve the macro cap across phase changes; also contains the inherited #515 saver repair.
- `src/orchestration/simulation/callbacks/event_callbacks.jl`: inherited #515 incoming-output repair; no additional C-6 edit.
- Root and test projects plus the root manifest: add OrdinaryDiffEqMultirate. The manifest was resolved with its existing Julia 1.12.3 runtime, retaining unrelated standard-library pins.
- Solver, interlink and power documentation; unit runner; coupled multirate and initial-force output tests.

## Tests Added/Updated

All execution below used Julia 1.12.1 with `--compiled-modules=existing` and the
supported unit-shard harness with the pinned EDG/HYPR providers, except the
initial standalone feasibility probe.

- Standalone data/battery probe: six assertions pass. Fast plus slow equals the
  full derivative exactly; final-state maximum difference from DP8 is 1.49e-8.
- Coupled unit file: 65 assertions pass. Covers complete derivatives for active
  and inactive spacecraft; data, laser and power trajectories against DP8;
  analytic data and battery values; strictly increasing scheduler history;
  a manufactured coupled ODE with one continuous event and unique periodic ticks;
  data-store boundary output with and without retained solution history;
  ground-station counters; mounted sensor measurements; atmosphere macro caps;
  interrupted checkpoint continuation on the same mission horizon; and the
  environment-selected mode with its effective step cap.
- Five selected files pass: coupled multirate, solver configuration, event output,
  direct RHS assembly and existing initial-force output. The configuration file
  passes 161 assertions. Log: `/tmp/spaceagora-c6-broader.log`; report:
  `/tmp/spaceagora-c6-broader-report/unit.toml`.
- All 33 PR contract gates pass: `/tmp/spaceagora-c6-gates.log`.
- Follow-up validation passes both complete files, with zero failures:
  `/tmp/spaceagora-c6-final.log` and `/tmp/spaceagora-c6-final-report/unit.toml`.
  This rerun includes all 65 coupled assertions and MRI-specific initial
  aerodynamic output in both saver orders (56 assertions across all tested
  first-order solver paths).

An earlier expanded run had 40 passes and three fixture failures: the default
laser policy selected the endpoint whose semimajor axis would decrease, so no
link activated; and the checkpoint check expected the resumed history instead
of the final returned segment. The corrected fixture explicitly targets the
gaining endpoint and checks the serialized checkpoint at 6 s separately from
the returned 9-to-10 s final segment. No tolerance was loosened for those fixes.

## P1 Assessment and Residual Risk

No P1 finding identified in author verification. Independent source review,
hosted checks and final integration with the release/prerequisite branches are
still required. The local checks are not a whole-unit-tree or performance claim.
The first installation attempt stopped at the missing EDG provider before
running the new file; the recorded successful harness run used installed
providers. Do not confuse this feature with the separate companion-interface
cleanup item also named C6.

Unresolved P1: No
