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
- **Compatibility bridge:** EDG's private forwarding list, `COMPATIBILITY_FUNCTIONS`
  in `src/models/gnc/guidance/aerobraking/typed_edg/algorithms.jl`. It forwards
  older helper names to the EDG companion.

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

| HYPR file | Private SpaceAGORA access | Before you replace it |
|---|---|---|
| `test/custom_objective.jl` and `test/extension_coverage.jl` | Private core model and control bindings, or extension bindings, used by test fixtures | Keep the tested objective and the coverage it measures. |
| `test/precompiled_conflicts.jl` | The private machinery that activates HYPR | Keep a test, in a fresh Julia process, that a conflicting HYPR is refused. |
| `test/jakob_comparison.jl` | Private access to the comparison fixture | Check the replacement against the pinned fixture before you remove the access. |

### Who uses the HYPR names

In HYPR, `ext/HYPRSpaceAGORAExt.jl` imports the groups of names, then includes the
domain files. `ext/rpo_planner_execution.jl` uses planner requests and results,
and `ext/robot/swarm_and_retiming.jl` also uses the Cloth services. In
SpaceAGORA, the RPO planner adapter, the comparison planners and the examples call
these names too. So a name that one HYPR file does not use can still be needed by
the contract or by another caller.

### Before you remove an import from HYPR

HYPR's extension namespaces remain implementation details. Removing the old
loader does not narrow the service contract or move shared bindings. When you
clean up imports in HYPR:

- check direct extension accesses and callers in each repository;
- keep every name in SpaceAGORA's contract lists;
- run the checks below, not only a search: a search finds imports, not every
  method or alias that reaches those bindings.

### Checks for a HYPR boundary change

Run SpaceAGORA's `test/unit/gnc/hypr_services_tests.jl`, HYPR's
`test/source_boundaries.jl`, `test/integration.jl` and
`test/precompiled_loading.jl`, and the tests of the domain you changed.
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
| SpaceAGORA's targeting control | `_edg_recompute_switches!`, `_edg_base_alpha` and `_edg_command_alpha!`, through the compatibility bridge | This coupling is outside the supported decision and query lists. Keep it on the bridge until the targeting control has a supported route. |
| The companion's `test/runtests.jl` | Private fixtures for the prediction context and kernels, and checks of the compatibility bindings, next to the public analytic-run tests | Keep the assertions on lease expiry, on observation copies and on the real dispatch path. |
| The companion's `benchmarks/native_reference/refresh_comparison.jl` and `observer_comparison_cases.jl` | Private model, engine and control aliases, `ODEParams` construction and integrator access | Give these research scripts an explicit replacement, and check that their results are unchanged, before you remove their private paths. |

### The compatibility bridge

The bridge, `COMPATIBILITY_FUNCTIONS` in `typed_edg/algorithms.jl`, lists the two
decisions and the helpers that SpaceAGORA, the regression fixtures and the
research scripts still call. Its helpers are not extra public hooks. The bridge
is separate from `EDGServices`, so shrinking `EDGServices` leaves it in place.
Keep the bridge until each maintained caller has moved to a supported call or has
been retired on purpose. [Adapter maintenance](adapter_maintenance.md#edg-extraction-and-source-ownership)
describes how it forwards to the companion.

### Checks for an EDG boundary change

Run the companion's checks in `test/runtests.jl`: the service module and its
signatures, the prediction-context lease, the observation copies and the analytic
run. Run them with SpaceAGORA's EDG callback and control tests.

- Go through the real path, from SpaceAGORA through the extension to the
  companion, and include a lease that expires after an exception.
- Run both the public simulation tests and the private fixture tests: they check
  different things.
- These checks show that the interface works. Before you make a scientific claim,
  run the scientific cases themselves.

## External propagation and MuJoCo

### Private paths in SpaceAGORAMuJoCo

MuJoCo's `test/common.jl`, `test/engine_tests.jl` and
`benchmarks/basilisk_mujoco_xval/sagora_driver.jl` use private SpaceAGORA paths
for test fixtures and research. They are maintenance dependencies, not supported
adapter calls. Move them only when a public replacement keeps their assertions on
synchronization, frames, spacecraft indices and isolation between runs. If some
private access must remain, write it down here.

### Checks for an external-propagation boundary change

Run SpaceAGORA's `test/unit/external_propagation_tests.jl` and the companion's
`test/load_order.jl`, `test/incompatible_core.jl` and `test/engine_tests.jl`. Keep
both load orders covered, and keep the compatibility refusal before any native
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
