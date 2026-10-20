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
  started with `--project=.` continues in the examples environment once it is
  prepared, and otherwise stops with the setup command.
- The command-line tool still runs with `--project=.`. Its `run`, `telemetry`
  and `benchmark` commands start their scripts in the examples environment,
  and stop with the setup command until it is prepared.
- Scripts of your own that you ran with `--project=.` and that use Plots or
  Roots: run them with `--project=examples`, or move them to a project of your
  own that declares those packages (`Pkg.add("Plots")`).
- Run `examples/setup.jl` again after a pull that changes the root
  `Manifest.toml`, so the examples keep the versions it pins.
- Contributors: the tests start example and CLI children in the examples
  environment, so prepare it beside the test environment
  (`julia --startup-file=no test/setup.jl`).

[Installation & Environment](installation_environment.md#Examples-environment)
describes the environment.
