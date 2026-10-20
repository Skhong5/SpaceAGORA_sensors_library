# EDG package boundary

`SpaceAGORAEDG.jl` is the optional, experimental implementation of the typed
energy-depletion guidance used by SpaceAGORA. The core facade is
`SpaceAGORA.EDGServices`, contract `v"2.0.0"`. This separation follows the
[optional adapter standard](adapter_contract.md). It changes where numerical
kernels live; it does not establish manuscript equivalence or make the
controller flight-qualified. The [user guide](../user/edg_guidance.md) explains
the scientific policy and its current limits.

## Ownership

| Responsibility | Owner |
|---|---|
| Public configuration, state, guidance/control model and angle-command types | SpaceAGORA; existing type identities and constructors are retained. |
| Heat-rate and heat-load decisions, structural restriction, targeting, candidate planning, passage heat, guidance and control decisions | SpaceAGORAEDG numerical implementation. |
| Observation copies, atmosphere, epoch, frames, gravity and vehicle queries | SpaceAGORA, behind the versioned facade. |
| Integrator, heat integrals, event roots, solver masks, callback order, output and panel actuation | SpaceAGORA. |
| Fresh-run initialization, per-passage bookkeeping and shared guidance/control state | Core lifecycle hooks, operating on the run's own EDG state. |
| Older E-EDG/T-EDG strategy routes | Removed from core; SpaceAGORAEDG is the only EDG implementation. |

## Callers and private forwarding

The public facade inventory in `src/models/gnc/companions/edg/services.jl`
separates two provider decisions from core-supplied types and queries. Core's
`ext/SpaceAGORAEDGExt.jl` delegates calls to the companion by allowed name;
this differs from an external propagator adding methods to the same core generic.

| Consumer | Access | Maintenance boundary |
|---|---|---|
| Companion decision implementations | Explicit `EDGServices` imports in `src/SpaceAGORAEDG.jl` | Supported facade calls; prediction contexts are supplied by core during a decision. |
| Core guidance and control | Delegation through the extension and core-owned lifecycle | Core retains callbacks, pairing, panel application and per-run state. |
| Core targeting control | `_edg_recompute_switches!`, `_edg_base_alpha` and `_edg_command_alpha!` through the compatibility bridge | Existing private coupling, outside the public decision/query inventory. |
| Companion `test/runtests.jl` | Private context/kernel fixtures and compatibility-binding checks, alongside public analytic-run tests | Preserve lease-expiry, observation-copy and real dispatch-path assertions when migrating fixtures. |
| Companion `benchmarks/native_reference/refresh_comparison.jl` and `observer_comparison_cases.jl` | Private model/engine/control aliases, `ODEParams` construction and integrator access | Research callers need an explicit replacement and correctness check before their private paths can be removed. |

The private allowlist lives in
`src/models/gnc/guidance/aerobraking/typed_edg/algorithms.jl`. Its retained helpers
are not extra public hooks. It includes the two decisions as well as helpers
used by core, regression fixtures and research. Shrinking the formal facade
would not remove this separate coupling. Keep the bridge until each maintained
caller has migrated or has been explicitly retired.

For changes here, use the companion's facade/signature, context-lease, copy and
analytic-run checks in `test/runtests.jl`, together with core EDG callback/control
tests. Exercise the real core-to-extension-to-provider path, including expiry
after exceptions. Public simulation tests and private fixture tests answer
different questions; interface conformance does not establish scientific validity.

## Supported calls and values

`IMPLEMENTED_FUNCTIONS` names the two decision services the companion
implements:

- `guidance_decision!(state, config, u, p, t, spacecraft_index)`;
- `control_decision!(state, config, controlled_panel_links, u, p, t, spacecraft_index)`.

Core invokes them with the run-owned mutable EDG state first, which the
companion updates, then the immutable configuration, the controlled panel links
(control only), a copied observation `u`, a borrowed prediction context `p`, the
elapsed time `t` in seconds and the spacecraft's position in the run,
`spacecraft_index`, from 1. `CONSUMED_SERVICES` lists the exact types and query
operations supplied to the companion by this contract. The retained helper inventory supports
compatibility routes and regression fixtures; it is not a general-purpose
simulation API.

`control_decision!` returns the core-owned `EDGControlDecision`: an angle command
in radians, an `apply` flag, a switch/cache action and diagnostics. The flag
distinguishes an invalid-index no-op from a valid zero-angle command. A cache
hit or a returned command does not certify scientific feasibility. Core applies
the command to the configured non-root panel links.

Observations contain inertial position in metres, inertial velocity in m/s,
mass in kg and copied cumulative per-link heat in J/cm². Spacecraft indices and
controlled-link indices are 1-based. Position and velocity are immutable
`SVector` values; heat and link collections are tuples. Geometry queries return
only the masses, dimensions, reference areas, reflection coefficients and angles
consumed by the kernels. Mutable physical geometry is never passed to the provider.

| Service | Meaning |
|---|---|
| `planet_constants`, `thermal_parameters`, `entry_altitude_m`, `spacecraft_geometry(p, spacecraft_index)` | Immutable value projections, with entry altitude in metres. |
| `_edg_environment_state(p, u, t, spacecraft_index)` | Current geodetic altitude, atmosphere and air-relative motion for a copied spacecraft observation. |
| `_edg_targeting_prediction_environment` | The same physical conventions at an explicit inertial state and elapsed simulation time. |
| `_edg_sample_prediction_atmosphere` | Altitude-only prediction at zero latitude/longitude; it is not interchangeable with a full trajectory query. |
| `_edg_ephemeris_time`, `_edg_planet_frame_lpi`, `frame_state` | Configured epoch and frame services; `l_pi` maps inertial components to planet-fixed components. |
| `gravity_acceleration` | The configured central/J2 and N-body acceleration, preserving the existing predictor's arithmetic. |
| `density_scale_height` | Existing density-model fallback. |
| `_edg_targeting_control_clock(p)` | The period and origin (s) of the EDG control ticks, on which targeting predicts its switch times. |

Every query takes the prediction context first. The control clock is the one the
engine reports to the EDG state (`gnc_clock_started!`) when the run starts,
before the models' initializers, and again at each solve's start: the context of a
provider call carries the clock of the EDG state passed to that call, and core no
longer searches the run's configuration for the EDG controller. A call without an
EDG state, or in a run without the EDG control model, has no clock, and the query
raises an `ArgumentError`.

Environment values use metres, kg/m³, kelvin, m/s and pascals; molecular speed
ratio is dimensionless. Heat rate is W/cm² and specific energy is J/kg. Frame
queries use the configured epoch plus elapsed time. Atmosphere queries use
elapsed time. Winds are east/north/up; airspeed uses spacecraft velocity
relative to the rotating planet minus atmospheric velocity, including the
rotation transport term.

## Prediction capability and state lifetime

`AbstractPredictionContext` is an opaque, synchronous, call-scoped capability.
Only the facade query functions are supported. Ordinary property access is
refused, and the query closure checks expiry before accessing the core. Core
revokes each lease in a `finally` block, including when a kernel throws. Retaining
a handle does not extend its lifetime. Julia reflection into private fields or
modules is outside this interface; the facade is an ownership boundary, not a
security sandbox.

The capability does not expose `ODEParams`, `shared_buffers`, the live state,
mutable geometry, an atmosphere reseeding operation or solver controls. Existing
atmosphere caches and RNG behavior remain core-owned and use the configured
query path. Packaging introduces no new prediction instance, seed or cache
policy. Any future change to those semantics needs its own scientific review.

EDG may update its own run-owned plan, counters and diagnostic state. Under the
default `run_simulation` isolation, core deep-copies the configuration while
preserving the shared state between paired guidance and control. Separate runs
receive separate state. `isolate_state=false` means caller-owned state with fresh
initialization; it does not resume a prior passage. Checkpoint writing and resume
remain refused because they do not persist the complete EDG state.

Core owns normal callback periods, lifecycle scheduling and physical integration.
The geodetic EDG entry/exit observer owns passage heat baselines; the solver's
spherical atmosphere phase boundary remains a separate concept. Neither the
package move nor a diagnostic snapshot changes the physical cumulative heat
integral, callback ordering, root settings, limits or 3 Hz scientific cases.

## Installation and errors

The [installation guide](../user/installation_environment.md) describes the
optional `edg` target and the exact pair recorded in package metadata. Load
`SpaceAGORAEDG` before running EDG. Non-EDG simulations can still load and run
with core alone. Selecting an EDG model without its provider raises
`EDGUnavailableError` during preflight, before output or stepping; it never
silently selects another guidance law.

The companion checks that the facade exists and requires exactly contract
`v"2.0.0"` before including its algorithms. The extension repeats the version
check during initialization and dispatch. An incompatible contract raises
`CompatibilityError`; install a supported pair and start a fresh process.
Core retains existing configuration, state pairing and panel-mapping validation.
Prediction failures, unreachable targets and observed constraint violations keep
their existing diagnostics and scientific meaning.

## The EDG guidance family

The guidance decision is also an algorithm of the
[GNC algorithm contract](rpo_planner_contract.md#The-general-algorithm-contract), so the generic binding
`AlgorithmGuidance` can run it in place of
`AerobrakingEnergyDepletionGuidanceModel`:

```julia
config = AerobrakingEnergyDepletionConfig(guidance_modes = (:max_energy_depletion,))
state = AerobrakingEnergyDepletionState(num_sats = 1)
guidance = AlgorithmGuidance(algorithm = EDGGuidanceAlgorithm(), spacecraft_index = 1,
    settings = EDGGuidanceSettings(config = config, state = state), replan_interval_s = 1 / 3)
control = AerobrakingEnergyDepletionControlModel(config, state)
```

| Part | The EDG guidance family (`EDGGuidanceFamily`) |
|---|---|
| Algorithm | `EDGGuidanceAlgorithm()`: the companion's `guidance_decision!`. Capabilities (`EDGGuidanceCapabilities`): the `:max_energy_depletion` and `:targeting` guidance modes. It draws no random numbers (`consumes_rng` is `false`) and needs the companion to be loaded. |
| Request | `EDGGuidanceRequest`: copies of the observation, the configuration and the EDG state (with its control clock), and an opaque handle from which each provider call borrows its prediction context. |
| Product | `EDGGuidanceDecision`: the spacecraft's guidance fields of the EDG state. The provider decides on a copy of the state; a proposal fails if it changes anything else. |
| Validation | `validate_edg_guidance_decision`: the request's identity and spacecraft, a known mode that the configuration allows (`:inactive` and `:safe_low_drag` always), and a bracketing count that has not decreased. |
| Install target | The EDG state of the settings, which the EDG control model reads. |
| On rejection | `:fail`, as when the provider raises. |
| Contract version | `family_contract_version` is this facade's contract version. |

Requests come at the binding's guidance ticks (`replan_interval_s`, with one at
the run's start), and the binding ticks before the same tick's control. The
binding runs one spacecraft; a run with several uses the EDG guidance model.
The binding keeps a record of every request, which costs memory over long
campaigns.

## Changes from contract 1.0.0

Contract 2.0.0 follows SpaceAGORA's [extension conventions](extension_conventions.md):

- `guidance_decision!` and `control_decision!` take the EDG state first, then the
  configuration (1.0.0: configuration first).
- `_edg_environment_state(p, u, t, spacecraft_index)` takes the context first
  (1.0.0: `(u, p, t, i)`).
- `_edg_targeting_control_clock(p)` takes the context alone and returns the clock
  of the call's EDG state (1.0.0: `(config, p)`, which searched the configuration
  for the control model holding `config`). The EDG state gained
  `control_clock_period_s`.
- Spacecraft indices are named `spacecraft_index`.
- The other queries, `require_version`, `require_provider` and the errors are
  unchanged, and so are the numerical kernels. The values of the queries do not
  change.

SpaceAGORAEDG implements contract 2.0.0 from version 0.2.0. A companion built for
1.0.0 is refused at load.

## Evidence and limits

Interface conformance and installation checks do not qualify native reference
campaigns or their scientific settings.

The initial package is experimental. Exact journal reproduction, continuous-family
optimality, flight use, new checkpoint support and new concurrency guarantees
are not established by this extraction. Existing observed violations remain
violations; no threshold or scientific setting is relaxed to pass packaging tests.
