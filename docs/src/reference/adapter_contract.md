# Optional adapter standard

Optional integrations use a versioned SpaceAGORA service facade. The adapter owns
its implementation and dependencies; SpaceAGORA owns shared configuration and
result types, simulation lifecycle, validation and the services listed by the
facade. Loading SpaceAGORA alone must not load an optional provider or its native
library.

This standard applies to new adapters and changes to existing adapters. Domains
retain their own protocol: a dynamics owner advances state, a planner produces a
validated reference, and a guidance adapter computes its domain's outputs. They
share the rules below, not a universal callback or provider registry.

## Supported boundaries

| Integration | Implementation owner | Supported core boundary | Lifecycle |
|---|---|---|---|
| HYPR planning | `HYPR.jl` extension; `SpaceAGORAHYPR` is a compatibility shim | [`SpaceAGORA.HYPRServices`](hypr_services.md) | Explicit provider activation; per-request planner state |
| External propagation, including MuJoCo | Independent [SpaceAGORAMuJoCo.jl companion](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl) for MuJoCo scenes; each other adapter owns its domain implementation | [`SpaceAGORA.ExternalPropagationServices`](external_propagation_services.md) | Configured owners; separate mutable runtime for every simulation |
| Experimental EDG guidance | Independent `SpaceAGORAEDG.jl` companion | [`SpaceAGORA.EDGServices`](edg_adapter_boundary.md), contract `v"2.0.0"` | Core-owned callbacks and per-run state; copied observations and call-scoped prediction queries |

EDG remains experimental; interface conformance does not establish scientific
qualification. The HYPR compatibility shim stays at `packages/SpaceAGORAHYPR/`;
the implementation remains owned by the external HYPR repository.

## Choose a boundary

For a new algorithm, navigation model or controller, start with the
[GNC student guide](../user/gnc_algorithm.md) and the
[package, project and companion choices](../extensibility.md). An ordinary
GNC package uses the public GNC hooks; it need not implement an optional-provider
facade. The generated `companion` starter is specifically an external-propagation
owner. SpaceAGORAKitDemo is a public GNC package, not such an owner.

| Boundary | Provider or consumer | Where to find ownership and checks |
|---|---|---|
| Planning methods and shared geometry | HYPR extension, core planner adapters and comparison workflows | [HYPR domains and callers](hypr_services.md#Domains-and-callers) |
| EDG decisions and prediction queries | EDG companion, core guidance/control, provider tests and research | [EDG callers and private forwarding](edg_adapter_boundary.md#Callers-and-private-forwarding) |
| External state advancement | MuJoCo, other external owners, the `companion` starter and telemetry callbacks | [External-owner users](external_propagation_services.md#Owners-and-users) |
| Saved JSON and HTML | Core output producers and the independent Visualizer | [Saved-output boundary](#Saved-output-consumers) and [visualization guide](../user/visualization.md) |
| Atmosphere queries and native resources | GRAMSuite and the core GRAM extension | [Atmosphere models](../user/atmosphere_models.md) and [GRAMSuite setup](../user/gramsuite_setup.md) |
| Optional interlink matching | PythonCall extension and NetworkX backend | [Interlinks](../user/interlinks.md) |

SpaceAGORA owns the atmosphere wrapper types. Its GRAM extension connects them
to GRAMSuite, installs hooks in both directions, and reconstructs serialized
native wrappers with fresh locks. It is not one of the three service facades above. PythonCall/NetworkX is an
optional matching backend, not a separately versioned SpaceAGORA companion.
WamIPEDensity is currently a standalone density package without a SpaceAGORA
adapter; adding one would require a new integration design.

## Versions, source pins and evidence

Keep three identifiers separate when recording a supported pair:

- **Package version:** the resolver's compatibility input in `Project.toml`.
- **Contract or format version:** the protocol checked by the facade or renderer.
- **Source revision:** the exact core and companion commits exercised by a check.

The checked-out core's gitlinks and `packages/VisualizerSource.toml` select
optional source revisions. A companion's default branch or release tag may point
elsewhere. Record both tested commits rather than treating a package version or
an updated documentation pin as new execution evidence. In particular, a
source-level review of a newer companion does not qualify its native runtime.

Use the domain pages below to choose behavioral checks. The generic checks in
`test/unit/adapter_contract_tests.jl` establish version refusal, inventory shape
and binding resolution; they do not establish every method signature, unit,
state lifetime or numerical result. For the other integrations:

| Integration | Existing validation route | Limit to retain in the result |
|---|---|---|
| Visualizer | `test/optional/visualizer/optional.jl`, both load orders in `julia-ci.yml`, companion payload/schema and CLI tests | File-format and renderer behavior do not certify propagation. |
| GRAMSuite | `test/smoke/gram_package_readiness.jl` and `test/smoke/gram_dependency_integration_tests.jl` | The readiness matrix checks loading, identity and injected hooks; the dependency suite exercises a real native-free boundary. Neither alone certifies a native atmosphere run. |
| PythonCall | `test/integration/interlink_python.jl` and both load orders in `julia-ci.yml` | Keep the missing-backend refusal and edge/weight semantics, as well as successful matching. |
| Generated packages and companions | Starter smoke tests, contributor-environment checks and the generated package's own tests | Validate the selected core revision; an older demonstration package pin is not evidence for the current release. |

Some provider tests and research scripts intentionally still use private core
paths. The domain pages name these maintenance dependencies separately from
supported adapter calls. Keep them working until their users have migrated; do
not promote a private path into the public contract simply because a test uses it.

## Facade shape and compatibility

Every facade provides:

- `CONTRACT_VERSION::VersionNumber`, independent of package release versions.
- `require_version(expected)` returning `nothing` for the exact supported version
  and throwing the facade's `CompatibilityError` otherwise.
- `IMPLEMENTED_FUNCTIONS`, the explicit methods the adapter is allowed to supply.
- `CONSUMED_SERVICES`, the explicit core types and operations it may consume.

Inventory entries are a `Symbol` for a direct binding or a tuple of symbols for
a path through facade namespaces. Supplemental inventories must be named and
documented; HYPR's `CLOTH_SERVICES` is relative to its `Cloth` namespace. Aliases
preserve the original type and function identities. Adapter-specific core access
uses explicit facade bindings; the documented root public API remains available
for ordinary configuration and simulation operations. Whole internal modules and
raw engine storage are outside the contract.

Check compatibility before defining adapter methods or loading native libraries,
then again in runtime initialization. A rejected version must leave the adapter
unavailable. Start a fresh Julia process after a failed package load; method
definitions already evaluated by Julia cannot be assumed to roll back.

Contract versions are matched exactly, including additive changes. Changes to
signatures, units, frames, result fields, mutability, ownership or failure
semantics require a new reviewed contract version. A package compatibility range is an outer resolver
constraint, not evidence that every release in the range has been tested.

## Contract history

Each facade's reference page describes its current contract. A new contract
version is listed here with what changed.

| Facade | Contract | Change |
|---|---|---|
| `ExternalPropagationServices` | 2.0.0 | `external_state` and `external_acceleration` take `(rt, t, slot)`, time before the owner-local position, instead of `(rt, k, t)`. The attitude quaternion is defined by formula. No numerical change. SpaceAGORAMuJoCo implements it from version 0.2.0. |
| `HYPRServices` | 2.0.0 | The planners HYPR supplies require an explicit `rng::AbstractRNG` keyword instead of defaulting to the global stream. HYPR implements it from version 0.2.0; this source accepts HYPR 0.4.0. No binding changes. |
| `EDGServices` | 2.0.0 | `guidance_decision!` and `control_decision!` take the EDG state before the configuration; `_edg_environment_state(p, u, t, spacecraft_index)` takes the context first; `_edg_targeting_control_clock(p)` returns the clock the engine reported to the call's EDG state instead of searching the configuration. No numerical change. The EDG guidance family of the GNC algorithm contract is served by this contract. SpaceAGORAEDG implements it from version 0.2.0. |
| `HYPRServices`, `ExternalPropagationServices`, `EDGServices` | 1.0.0 | First reviewed contracts. |

## Package and installation rules

An adapter may be a standalone package depending on SpaceAGORA, or a Julia
extension activated by a weak dependency. SpaceAGORA must not acquire a mandatory
dependency on an optional adapter. Keep one authoritative implementation; a
compatibility shim only forwards to it and checks availability.

The domain layout uses exact Git submodule revisions under `packages/robotics/`
for HYPR and SpaceAGORAMuJoCo, and under `packages/gnc/` for SpaceAGORAEDG.
These are source checkouts, not copies of their implementations or new required
dependencies. The shared
[optional-package helper](../user/installation_environment.md#Optional-package-commands)
verifies the selected gitlink revision, package UUID/version and clean checkout
before passing its path to the existing installer. It initializes only the
selected repository. Keep this orchestration separate from provider activation,
runtime compatibility checks and scientific acceptance. A successful source
check is not evidence that a native simulation has passed.

Each repository declares its own runtime and test dependencies, owns its native
artifacts and supplies an installation path that works outside a SpaceAGORA
checkout. Do not rely on a parent `Manifest.toml`, sibling source paths or an
unrelated adapter's setup script. For unregistered or unreleased integrations,
record exact source revisions for the qualified pair and explicitly install the
core revision before the adapter. Package `[sources]` entries do not propagate
into arbitrary consumer environments.

## Ownership and scientific semantics

Document who owns configuration, mutable runtime, state, random-number streams,
native resources and cleanup. State reused as a template must not become shared
mutable state between runs or workers. Specify units, coordinate frames,
quaternion convention, index mapping, callback ordering and time synchronization
where relevant. Use core services for shared calculations rather than copying
them into the adapter.

Refuse unsupported modes explicitly. Do not silently substitute an integrator,
drop a configured feature, relax an accuracy threshold or treat a failed native
test as a successful skip. Core validation and adapter validation complement each
other; a configuration-only preflight does not certify every execution mode.

## Saved-output consumers

The independent SpaceAGORAVisualizer package consumes a versioned data format,
not simulation services. SpaceAGORA owns scene metadata and saved-row conversion;
the companion owns HTML assembly, browser code and third-party assets. Their
renderer protocol and portable payload schema are both version 1. The extension
checks the renderer protocol before delegation, and the companion checks the
payload schema before rendering. These integer format versions are separate from
the VersionNumber service-facade contracts above.

The integration uses `ext/SpaceAGORAVisualizerExt.jl` and a weak dependency.
Loading core alone leaves the renderer absent; loading both packages in either
order enables HTML export. A missing renderer fails before a requested run or
campaign creates outputs. Scene and JSON export remain available without it.
The companion also opens saved JSON independently of SpaceAGORA.

`scripts/setup_visualizer.jl` installs the revision pinned in
`packages/VisualizerSource.toml` into a separate environment. Installation and
both load orders are supported. The companion does not read raw
engine state, install a dynamics provider or change propagation settings.
See the [user guide](../user/visualization.md) for supported commands.
