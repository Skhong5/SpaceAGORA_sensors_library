# Companion boundaries: private paths and checks

This page is for maintainers of SpaceAGORA and of its companion packages: HYPR,
SpaceAGORAEDG and SpaceAGORAMuJoCo. Use it when you change a boundary between
SpaceAGORA and a companion, or when you move a companion's test or research
script off a private path. It lists:

- the private SpaceAGORA paths that companion tests and research scripts still use;
- the rules for keeping and replacing those paths;
- the checks to run when you change a boundary.

Users and companion authors find the supported boundaries on the published
pages: the [optional adapter standard](../src/reference/adapter_contract.md) and the
[HYPR](../src/reference/hypr_services.md),
[EDG](../src/reference/edg_adapter_boundary.md) and
[external propagation](../src/reference/external_propagation_services.md) service
contracts. Release evidence for an adapter is in
[adapter maintenance](adapter_maintenance.md).

## Terms

- **Service module (facade):** a module inside SpaceAGORA, such as
  `SpaceAGORA.HYPRServices`, that lists by name the SpaceAGORA types and functions
  a companion may use and the functions it may add methods to. Names in these
  lists are the supported boundary.
- **Private path:** any other way a companion reaches SpaceAGORA code, such as an
  internal module, an internal name or the engine's raw storage. Private paths are
  not supported and can change without notice.
- **Compatibility package (shim):** `SpaceAGORAHYPR`, at
  `packages/SpaceAGORAHYPR/`. It keeps older code that loads it working by
  forwarding to HYPR; it holds no planner of its own.
- **Compatibility bridge:** EDG's private forwarding list, `COMPATIBILITY_FUNCTIONS`
  in `src/models/gnc/guidance/aerobraking/typed_edg/algorithms.jl`. It forwards
  older helper names to the EDG companion.

## How the integration checks catch a boundary change

A boundary change can start on either side, so both repositories test the pair:

- **Each companion's CI** installs the companion with a released SpaceAGORA. Its
  `core-release-pin` job checks that the pinned core commit is the commit of a
  SpaceAGORA release tag, inside the companion's compatibility range. A companion
  change that breaks the boundary fails there.
- **SpaceAGORA's CI** installs each companion at its gitlink and runs the
  companion's own integration tests against the SpaceAGORA change under review,
  in the `optional-hypr-installation`, `optional-edg-installation` and
  `optional-mujoco-installation` jobs. A SpaceAGORA change that breaks a
  companion fails there, before it merges.

Every companion's tests cover the same five things:

| What is checked | HYPR | EDG | MuJoCo |
|---|---|---|---|
| Both package load orders, each in a fresh process | `test/integration.jl`, `test/service_boundary.jl` and `test/precompiled_loading.jl`, each in both orders | `test/load_order.jl core-first` and `edg-first` | `test/load_order.jl core-first` and `companion-first` |
| The refusal when the provider is missing | `test/service_boundary.jl core-first`: a HYPR planner before HYPR loads | `test/load_order.jl core-first`: an EDG run before the companion loads | `test/missing_native.jl`: a scene without the MuJoCo library |
| The refusal of another contract version, before any method is defined | `test/incompatible_contract.jl` | `test/compatibility_tests.jl` | `test/incompatible_core.jl` |
| The service module's shape and exact version check (`Conformance.test_service_facade`) | `test/service_boundary.jl` | `test/service_boundary.jl` | `test/load_order.jl` |
| The companion uses only names its service module lists (MuJoCo: and public SpaceAGORA names) | `test/service_boundary.jl` | `test/service_boundary.jl` | `test/service_boundary.jl` |

SpaceAGORA's own tests add the core side of the same checks:
`test/optional/hypr/core.jl`, `test/optional/edg/core.jl` and the companion's
`test/core_only.jl` load SpaceAGORA without the companion, and
`test/unit/adapter_contract_tests.jl` checks every service module's shape.

The service-name check is what makes a contract narrowing safe. If a service
module stops listing a name that a companion still uses, the companion's check
fails in SpaceAGORA's CI. If a companion starts using a name its service module
does not list, the check fails in the companion's CI.

## Rules for private paths

1. **Keep a private path working** until every caller that uses it has moved to a
   supported call, or has been retired on purpose.
2. **Move a caller only when the replacement keeps what the caller checks.** Each
   section below says which assertions a replacement must keep.
3. **Do not add a name to a service module just because a test uses it.** Write
   down the private access that remains instead. Do not expose the engine's raw
   storage to make a migration easier.

## HYPR

### Private paths in HYPR's tests

HYPR's tests use `SpaceAGORA.HYPRServices` and SpaceAGORA's public names, except:

| HYPR file | Private SpaceAGORA access | Before you replace it |
|---|---|---|
| `test/integration.jl`, `test/sampling_compatibility.jl`, `test/rrt_callback_contract.jl` and `test/custom_objective.jl` | `SimulationModel.RPOStationGeometry` and `SimulationModel.RPOCubeSatGeometry`, to build the station and chaser of an `RPOReferenceGeometry`. No public or service name builds them. | Move them when a supported name builds this geometry. |
| `test/extension_coverage.jl` | Runs SpaceAGORA's own HYPR probes and unit tests from the paired checkout; those SpaceAGORA files use its internal modules | Keep the coverage it measures. The files are SpaceAGORA's tests, not copies. |
| `test/precompiled_conflicts.jl` | `SimulationModel.HYPRSupport`: the activation protocol of the old companion, which the test calls on purpose | Keep a test, in a fresh Julia process, that a conflicting HYPR is refused. |
| `test/jakob_comparison.jl` | The pinned historical comparison fixture, and `GuidanceHooks.RPOLQMPCTrackingSettings` | Check any replacement against the pinned fixture before you remove the access. |

`test/custom_objective.jl` also calls HYPR's own extension module; that is HYPR's
code, not a SpaceAGORA path.

### Who uses the HYPR names

In HYPR, `ext/HYPRSpaceAGORAExt.jl` imports the groups of names, then includes the
domain files. `ext/rpo_planner_execution.jl` uses planner requests and results,
and `ext/robot/swarm_and_retiming.jl` also uses the Cloth services. In
SpaceAGORA, the RPO planner adapter, the comparison planners and the examples call
these names too. So a name that one HYPR file does not use can still be needed by
the contract or by another caller.

### Before you remove an import from HYPR

The compatibility package's `RPO` and `RobotArm` aliases point at HYPR's
extension namespaces. Code that loads the compatibility package can reach every
name those namespaces import, even a name HYPR itself never uses. So when you
clean up imports in HYPR:

- check the names reachable through those two aliases, as well as the callers in
  each repository;
- keep every name in SpaceAGORA's contract lists;
- run the checks below, not only a search: a search finds imports, not the code
  that reaches names through the aliases.

### Checks for a HYPR boundary change

Run SpaceAGORA's `test/unit/gnc/hypr_services_tests.jl`, HYPR's
`test/source_boundaries.jl`, `test/integration.jl`, `test/service_boundary.jl`,
`test/incompatible_contract.jl` and `test/precompiled_loading.jl`, and the tests of
the domain you changed.
SpaceAGORA's `optional-hypr-installation` job runs HYPR's integration tests at the
pinned HYPR revision, as the EDG and MuJoCo jobs run their companions' tests, so a
SpaceAGORA change that breaks one of them fails before it merges. Keep these covered:

- both package load orders;
- the refusal of a planner call without an explicit `rng`;
- the identity of the shared bindings.

## EDG

### Private paths around EDG

| Caller | Private access | Before you replace it |
|---|---|---|
| SpaceAGORA's EDG regression tests | The bridge helpers, called directly or through model-typed wrappers in `targeting_control.jl` (`_edg_recompute_switches!`, `_edg_base_alpha`, `_edg_command_alpha!`), `edg_compatibility.jl` and `guidance_models.jl` | SpaceAGORA's runtime calls only the two decisions; these wrappers and helpers serve the regression tests. Keep them until those tests move to the companion, which owns the kernels. |
| The companion's `test/runtests.jl` | Private fixtures for the prediction context and kernels, and checks of the compatibility bindings, next to the public analytic-run tests | Keep the assertions on lease expiry, on observation copies and on the real dispatch path. |
| The companion's `benchmarks/native_reference/refresh_comparison.jl` and `observer_comparison_cases.jl` | Private model, engine and control aliases, `ODEParams` construction and integrator access | These research scripts belong to the companion's v0.1.0 source pair (core `05de98e`) and do not run on a newer core. A refresh needs a new comparison, with an explicit replacement for these paths and a check that the results are unchanged. |

### The compatibility bridge

The bridge, `COMPATIBILITY_FUNCTIONS` in `typed_edg/algorithms.jl`, lists the two
decisions and the helpers that SpaceAGORA's EDG regression tests still call.
SpaceAGORA's runtime calls only the two decisions through it. Its helpers are not
extra public hooks. The bridge is separate from `EDGServices`, so shrinking
`EDGServices` leaves it in place. Keep the bridge until each maintained caller has
moved to a supported call or has been retired on purpose. [Adapter maintenance](adapter_maintenance.md#edg-extraction-and-source-ownership)
describes how it forwards to the companion.

### Checks for an EDG boundary change

Run the companion's checks in `test/runtests.jl`: the service module and its
signatures, the names the companion uses, the prediction-context lease, the
observation copies and the analytic run. Run `test/load_order.jl` in both orders,
and SpaceAGORA's EDG callback and control tests.

- Go through the real path, from SpaceAGORA through the extension to the
  companion, and include a lease that expires after an exception.
- Run both the public simulation tests and the private fixture tests: they check
  different things.
- These checks show that the interface works. Before you make a scientific claim,
  run the scientific cases themselves.

## External propagation and MuJoCo

### Private paths in SpaceAGORAMuJoCo

MuJoCo's tests and its cross-validation driver use SpaceAGORA's public names,
except three internals that `test/engine_tests.jl` reaches through
`SimulationModel` (defined in `test/common.jl`):

- `SimulationCallbacks.PeriodicCallback`, a no-op periodic callback that aligns
  saved rows with the scene's steps;
- `QuaternionMath.rot`, to compare MuJoCo's body frames with SpaceAGORA's
  quaternion convention;
- `project_unit_quaternion`, to check saved attitudes bitwise against SpaceAGORA's
  own projection.

They are maintenance dependencies, not supported adapter calls. Move them only when
a public replacement keeps these assertions on synchronization and frames.

### Checks for an external-propagation boundary change

Run SpaceAGORA's `test/unit/external_propagation_tests.jl` and the companion's
`test/load_order.jl` (which includes `test/service_boundary.jl`),
`test/incompatible_core.jl`, `test/missing_native.jl` and `test/engine_tests.jl`.
Keep both load orders covered, and keep the compatibility refusal before any native
library loads or any method is defined.

Record whether an actual native run was exercised. The service-module checks and
the configuration preflight do not run MuJoCo; if no native run was part of your
checks, report the result as an interface and configuration check.

## Checks for the other integrations

`test/unit/adapter_contract_tests.jl` checks what the three service modules share:
that a wrong contract version is refused, that the lists of names have the right
shape, and that every listed name resolves. For method signatures, units, state
lifetime and numerical results, run the domain tests above.

| Integration | Tests to run | What to state with the result |
|---|---|---|
| Visualizer | `test/optional/visualizer/optional.jl`, both load orders in `julia-ci.yml`, and the companion's payload, schema and CLI tests | These tests cover the file format and the renderer; test propagation separately. |
| GRAMSuite | `test/smoke/gram_package_readiness.jl` and `test/smoke/gram_dependency_integration_tests.jl` | The readiness matrix checks loading, identity and the installed hooks. The dependency suite runs the real boundary without the native library. Run a native atmosphere case before you report one as tested. |
| PythonCall | `test/integration/interlink_python.jl` and both load orders in `julia-ci.yml` | Cover the refusal when the backend is missing, the meaning of edges and weights, and a successful match. |
| Generated packages and companions | The starter smoke tests, the contributor-environment checks and the generated package's own tests | Test against the SpaceAGORA revision you selected. A demonstration package pinned to an older revision is evidence for that revision only. |
