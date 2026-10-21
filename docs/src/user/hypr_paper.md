# Run the HyPR paper algorithm

!!! note "Optional package"
    Requires the optional `SpaceAGORAHYPR` package and the HYPR planner; see
    [Install and run](#Install-and-run).

These examples implement the algorithm described in the final author-shared
2026 iSpaRo paper, *HyPR: Safety- and Fuel-Aware Motion Planning and Replanning
for Autonomous SmallSat Servicing*. They run through SpaceAGORA's nonlinear
simulator and retain failed outcomes. They reconstruct missing experiment inputs;
they do not reproduce the original Tables II and III datasets or promise the
same numerical results. The selected paper PDF has SHA-256
`aa7e59492ebfad7f0cc8d7dace83c345c3ddee5f6ee67e965aa3bf0d9aa516e8`.

## Install and run

From the SpaceAGORA checkout containing this guide, create a separate environment.
The setup helper installs the immutable HYPR source revision recorded in
`packages/SpaceAGORAHYPR/HYPRSource.toml`, SpaceAGORA and its companion package.
Record the SpaceAGORA commit, the HYPR pin, Julia version, hardware and thread
count with any results. An older HYPR release is not a substitute for this pin.

```sh
julia --startup-file=no scripts/setup_hypr.jl "$HOME/AgentScratch/hypr-paper-env"
JULIA_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  HYPR_OUTPUT_ROOT="$HOME/AgentScratch/hypr-paper-static" \
  julia --startup-file=no --project="$HOME/AgentScratch/hypr-paper-env" \
  examples/Earth_RPO_HyPR_FinalISpaRo_Reconstruction.jl
```

This executes one newly generated static case, including planning and full
tracking. `HYPR_CASE_IDS=4` selects case 4 directly. To compare all seven methods,
set `HYPR_PLANNERS=hypr,pso_unrefined,rrt_connect,rrt_connect_bezier,rrt_star,chomp,stomp`.
RRT-Connect must precede its Bezier variant: both use exactly the same search
result. A failed search is saved as `planning_failed`, never executed as a valid
fallback. If HYPR planning fails, dependent comparisons are recorded as
`blocked_baseline` rather than run with an unlimited budget. A plan the run's
validation refuses, for example below the 0.5 m clearance, is saved as `refused`
with the refusal: it is not flown, so it has no command log or trace. A tracking
failure, or a refused HYPR plan, after valid HYPR planning retains the measured
planning allowance. CHOMP and STOMP use the same obstacle weight multiplier, 1.0.
Their wall-clock budgets depend on the machine. Large paper-sized comparisons
are optional and are not needed to try or verify the implementation.

## CI smoke scope

The example suite launches the paper entry point in the dedicated HYPR example
environment. With `SPACEAGORA_EXAMPLE_SMOKE=1`, it loads the actual packages and
configuration without running a numerical experiment or writing scientific
results. The optional HYPR integration job separately exercises the planner,
replanning contracts and failed-run evidence handling. The unit test
`test/unit/gnc/rpo_examples_planner_contract_tests.jl` replays the saved case-4
baseline below through `RetainedPathRPOPlanner` when the SPICE kernels in
`data/GRAMSuite.jl/GRAM Suite 2.0/SPICE` are present, and skips the replay without
them. A smoke pass does not establish maneuver success or paper-table
reproduction. Leave the smoke flag unset
for the scientific runs above and below.

## The original reconstructed dynamic case

The repository includes the saved baseline used for the reconstructed case 4:
`examples/support/hypr_final_isparo_case4.json`. It was generated during the
October 13, 2026 reconstruction, not by the original paper experiment.
Its SHA-256 is
`889c2df68edf946d7c097a7b95068f40809afb90aa003fe6af984393293843f5`.
It contains the endpoints, fitted path, reference, effective configuration and
search history. Geometry sampling uses seed 740; the initial case uses seed 744.
Replacement planning uses seed 1744.

!!! note "Reproduce with SpaceAGORA 0.4.0"
    The dynamic results below were produced with `RPOGuidanceModel`, the HYPR
    guidance model that SpaceAGORA 0.5.0 removed, through the replanning study
    `examples/Earth_RPO_HyPR_FinalISpaRo_Replanning.jl`, which was removed with it.
    Reproduce them from SpaceAGORA v0.4.0, the last release that has the model and
    the study: use that release's `Manifest.toml` and the HYPR revision pinned in
    its `packages/SpaceAGORAHYPR/HYPRSource.toml`. The study was delivered with
    these results at commit `b165c0528b782316db65142f19def374c181b47f`. For
    disturbances on the planner contract, see
    `examples/Earth_RPO_Planner_Disturbances.jl`
    ([Run an RPO case with interchangeable planners](rpo_planner_pilot.md)).

!!! warning "TODO: rerun from v0.4.0 pending"
    The rerun of these results from SpaceAGORA v0.4.0 has not been done yet. It
    follows the v0.4.0 tag; until then, that they are reproducible from v0.4.0 is
    unchecked.

Use a separate checkout of the v0.4.0 tag for this historical case. From that
checkout's root, create a distinct environment so the current-release setup
above keeps its own core and HYPR pins:

```sh
julia --startup-file=no scripts/setup_hypr.jl "$HOME/AgentScratch/hypr-paper-v040-env"
JULIA_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  HYPR_OUTPUT_ROOT="$HOME/AgentScratch/hypr-paper-dynamic" \
  HYPR_DYNAMIC_SCENARIOS=directly_ahead HYPR_DYNAMIC_MAX_TIME_S=1200 \
  julia --startup-file=no --project="$HOME/AgentScratch/hypr-paper-v040-env" \
  examples/Earth_RPO_HyPR_FinalISpaRo_Replanning.jl
```

This retains the original reconstructed obstacle placement and appearance.
The longer observation ceiling lets the replacement route continue; it changes
neither the obstacle nor the guidance/controller policy. Omit the ceiling to use
the historical `max(600, 4 * initial_reference_duration)` rule. Neither ceiling
comes from the paper. Planning wall time is consumed as simulated hold latency,
so measured hold latency and total completion time can vary with the computer.

`HYPR_DYNAMIC_SCENARIOS=baseline,directly_ahead` also runs the undisturbed case.
Other choices are `far_obstacle`, `near_path_feasible`, `partial_blockage`,
`goal_change`, `displacement_retime`, `displacement_replan`,
`sensor_false_positive` and `obstructed_goal_negative_control`. The last is an
extra negative control, outside Table III. `HYPR_BASELINE_RECORD` can select
another saved baseline; changing it creates a different experiment.

## How the implementation maps to the paper

| Paper mechanism | Implementation |
| --- | --- |
| RRT-Connect warm start, Bezier particle search and manuscript adaptation | HYPR `ext/rpo/pso_path_planning.jl`, `hypr_mode=:manuscript` |
| Absolute coefficient endpoints `(0.455, 1.05, 1.75)`, first-half cubic schedule, ten-iteration culling and evaluated final updates | HYPR manuscript swarm; `test/final_isparo_swarm.jl` |
| Clearance/curvature retiming and zero departure/arrival reference speed | HYPR `ext/rpo/rpo_reference_trajectory.jl` |
| Algorithm 1: continue, retime or brake/hold and replan | SpaceAGORA `rpo_replanning_decision`; the brake/hold lifecycle was `RPOGuidanceModel`'s (`maybe_update_rpo_replanning!`, through v0.4.0) |
| Algorithm 1 remaining route and measured-state splice | `rpo_retime_existing_plan`, retained geometric polyline and timed connector |
| Track the reference in the nonlinear plant and record delivered force | Full SpaceAGORA demo, LQ-MPC and thruster command log |

The thresholds remain 0.5 m safety clearance, 0.6 m retime clearance, 0.75 m
retime tracking error and 1.1 m replan tracking error. The detection memory is
0.02 times the original reference duration. Tracking uses 10 Hz control, a
60-step horizon, `Q=diag(20,20,20,2,2,2)`, `R=0.1I`, and `Qf=20Q`.
Retiming uses 0.00625 m/s², 0.25 s reaction time and speed factor 0.5.

Reconstruction choices that the paper does not fully fix include exact geometry
and obstacle draws, event placement/time, whole-particle uniform stagnation
trials, particle growth factor 1.5, interpreting 5 kg as wet mass, fixed inertial
attitude and hold release tolerances of 0.02 m and 0.002 m/s. The 20% activation
setting suppresses small average-throttle commands; it is not a valve-level PWM
simulation. Clearance is measured against a buffered sampled CAD point cloud,
with 0.05 s swept trajectory chords, not a watertight collision certificate.

## What the dynamic failure means

The paper's Section VI-B includes directly-ahead spherical obstacles. It does
not provide the exact appearance time, radius, distance ahead or stopping-margin
selection used in the original trials. Table III reports 100% HYPR success for
that category. **The failure retained here is not a failure reported by the paper.**

Our reconstructed sphere has radius 0.25 m and appears at 25% of the baseline
reference duration. Its centre is the first route sample at least 1.16 m from
both the trigger reference position and the goal. That is a geometric rule,
not a stopping-feasibility rule. In the retained run it appears at 21.1 s with
0.513011587 m clearance, only 13 mm above the required 0.5 m.

A model-conditional reachability calculation gives at most 0.482285 m clearance
one second later even allowing instantaneous, arbitrarily directed acceleration
up to 0.02 m/s². The conservative combined thrust and relative-dynamics bound is approximately
0.01857 m/s².
The saved calculation, inputs and assumptions are in
`examples/support/hypr_dynamic_braking_bound.json`. Thus, under this two-body, circular-target, fixed-RTN-sphere model, no available
control can preserve the 0.5 m margin after that appearance. This explains the
early clearance violation; it is not a claim of physical collision or of
universal safety for other encounters. Braking and a successful replacement
route cannot retroactively turn the whole maneuver into a safety pass.

The retained nine-case primary reconstruction had eight passes and this failure.
The original 600 s run also ended before completing its replacement route.
Separate post-release qualifications showed that a longer route can finish with
adequate clearance; those do not erase the early violation or prove the original
full run passed. The runnable example reports these questions separately.

A fresh full delivery run with the same reconstructed event and a 1200 s ceiling
completed at 362.2 s, with 0.0001906 m terminal position error. All 3,622 QP
commands were solved; delivered-force fuel consumption was 4.95575 g. Clearance
after release was at least 0.512911 m, with no violations. Whole-run minimum
clearance was 0.436911 m, so overall success remained false. These are bounded
reconstruction results on Julia 1.12.1, two Julia threads and one BLAS thread;
they are not Table III measurements or a new universal safety guarantee.

## Inspect the evidence

`*_results.json` retains overall `success`, `completion_reached`,
`clearance_satisfied`, `terminal_satisfied`, `control_satisfied`,
`fuel_satisfied`, the observation ceiling, event inputs, phase changes and
post-release clearance. A time cutoff is different from `hold_failed` and from
an early clearance violation. Overall success requires all applicable criteria;
failed cases stay in the denominator. Reaching the target alone is insufficient.

`*_trace.csv` records the executed relative state, mass and phase.
`*_commands.json` records times, forces, QP status, terminal state and mass so
fuel can be recomputed from delivered impulse. Static runs also retain each
planner's path and reference. Compare fuel only over common successful case IDs;
keep planner proxy cost separate from consumed propellant. Output files use the
selected prefix and can be overwritten: use a fresh output directory per run.
