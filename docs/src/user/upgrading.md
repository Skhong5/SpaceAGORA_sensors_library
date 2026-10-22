# Upgrading

Use this page when you move a checkout, or a project that uses SpaceAGORA, to a
newer release. Each section lists what you must change for that release.

Renamed and retired extension hooks have their own page,
[Migrating Extension Hooks](../reference/migrating_extension_hooks.md), and
[Public API and Compatibility](../public_api_policy.md) describes the
deprecation rules.

## 0.6.0

### The examples run in the examples environment

Plots and Roots are no longer dependencies of SpaceAGORA. The examples, the
plotting scripts and the benchmark studies now run in the examples environment,
`examples/Project.toml`, which takes SpaceAGORA from the checkout and adds the
packages they use. SpaceAGORA's own functions are unchanged.

After updating a checkout, install it and prepare the examples environment.
Terminal, from the repository root:

```text
julia --project=. -e "using Pkg; Pkg.instantiate(julia_version_strict=true)"
julia --startup-file=no examples/setup.jl
```

Then:

- Run the examples with `--project=examples`, for example
  `julia --project=examples examples/AGORA_Basic_Quickstart.jl`. An example
  that includes `examples/common.jl` and is started with `--project=.`
  continues in the examples environment once it is prepared, and otherwise
  stops with the setup command. The others, such as the cloth-panel and
  solar-panel demos, need `--project=examples`.
- The command-line tool still runs with `--project=.`. Its `run`, `telemetry`
  and `benchmark` commands start their scripts in the examples environment,
  and stop with the setup command until it is prepared.
- Scripts of your own that you ran with `--project=.` and that use Plots or
  Roots: run them with `--project=examples`, or move them to a project of your
  own that declares those packages (`Pkg.add("Plots")`).
- Run `examples/setup.jl` again after a pull that changes the root
  `Manifest.toml`, so the examples follow the versions it records.
- Contributors: the tests start example and CLI children in the examples
  environment, so prepare it beside the test environment
  (`julia --startup-file=no test/setup.jl`).

[Installation & Environment](installation_environment.md#Examples-environment)
describes the environment.

### The RPO Plotly figures need `using PlotlyJS`

PlotlyJS is no longer a dependency of SpaceAGORA. The RPO Plotly figures come
from a PlotlyJS package extension, which loads when PlotlyJS does:
`rpo_path_plot`, `rpo_tracking_plot`, and the planner-comparison plots and
their HTML reports. SpaceAGORA 0.5.1 announced the change the first time a
script made one of these figures.

- In a project of your own, add PlotlyJS and load it before making a figure.
  Julia REPL, in that project:

  ```julia
  using Pkg
  Pkg.add("PlotlyJS")
  using SpaceAGORA, PlotlyJS
  ```

  The order of the two packages does not matter. The examples environment
  already provides PlotlyJS, and the RPO examples load it.
- Without PlotlyJS loaded, the figure functions throw
  `PlotlyJSUnavailableError`, whose message gives both steps. So does a planner
  comparison asked for figures: `rpo_run_planner_comparison_batch` refuses
  before it plans anything, and `rpo_write_planner_comparison_outputs` before
  it writes anything.
- To write a planner comparison's CSV without figures and without PlotlyJS, set
  both `write_plotly_outputs=false` and `write_failed_path_outputs=false` in
  `RPOPlannerComparisonConfig`, or pass them to
  `rpo_write_planner_comparison_outputs`.
- The qualified module bindings `GuidanceHooks.PlotlyJS` and
  `GuidanceHooks.RPOComparisonReports.PlotlyJS` are gone: use `PlotlyJS`
  itself.
- The full sysimage (`build/build_sysimage.jl`) no longer includes PlotlyJS.

### Runs with guidance, navigation or control models do not resume 0.5.1 checkpoints

A checkpoint now records which spacecraft each guidance, navigation and control
model acts on, and a resumed run refuses one whose models act on other
spacecraft, or are another number in a group. A checkpoint written by
SpaceAGORA 0.5.1 or earlier holds no such record, so a run with any guidance,
navigation or control model refuses it with an `ArgumentError` before anything
is restored.

- Finish such a run with SpaceAGORA 0.5.1, or run it again from the start after
  updating.
- A run without those models resumes from a 0.5.1 checkpoint as before. Since
  that checkpoint holds no results rows, the resume warns that the earlier rows
  cannot be included, and writes its results to `resumed_at_<t>s` inside
  `results_directory`, leaving the earlier files as they were.

[Simulation Outputs](outputs.md#Checkpointing) lists what a checkpoint records
and what a resumed run refuses.

### Machine calibration constants are measured again

The machine calibration no longer reads past its tables, and the cost constants
it saves move to schema 5. Constants saved by SpaceAGORA 0.5.1 or earlier are no
longer read:

- A `SolverConfig(parallel=true)` run measures them again by itself, once in
  each working directory, as on a machine without constants.
- On a machine that uses the predictive campaign planner any other way (profile
  R7, or `SPACEAGORA_CAMPAIGN_PLANNER=predictive` set by hand), rerun the
  calibration after updating, with the thread count you run with. Until then
  that planner runs without machine constants. Terminal, from the repository
  root:

  ```text
  julia --project=. --threads=<T> scripts/calibration/calibrate_machine.jl
  ```

[Parallel Execution](parallel_execution.md#Turning-on-parallel-execution) says
where the constants are kept.

!!! note "TODO(0.6.0 release PR): sections still to add"
    Added on the release branch as each pull request merges: the typed interlink
    scheduling policies (#478, #480), the renamed interlink states (#486), and
    the companion versions of contract 3.0.0 (HYPR, SpaceAGORAEDG and
    SpaceAGORAMuJoCo 0.5.0).
