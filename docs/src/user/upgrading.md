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


### Load HYPR directly

The `SpaceAGORAHYPR` compatibility loader is removed. In a project that uses the
optional planner, replace `using SpaceAGORAHYPR` with:

```julia
using SpaceAGORA, HYPR
```

Both load orders activate the same HYPR extension. SpaceAGORA alone still does
not load HYPR, and planner preflight still refuses an unavailable or incompatible
provider. The external HYPR repository and its immutable pin are retained; the
standalone pin has moved to `packages/HYPRSource.toml`.

Run the HYPR setup helper again for an existing environment. It removes the
retired loader's dependency and source entries while preserving unrelated project
settings. A project you maintain yourself should remove the loader dependency
and declare HYPR directly. The loader's `RPO`, `RobotArm`, `SwarmPolicy` and
`PlannerAdapter` module aliases are removed; application code should use the
public planner interfaces, and companion code should use the documented
`HYPRServices` contract. This removal does not narrow that service contract.
