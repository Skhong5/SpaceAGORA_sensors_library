# GNC Aerobraking Boundary Contract

## Canonical ownership
- `src/models/gnc/guidance/aerobraking/*` owns aerobraking strategy planning logic (`E-EDG`, `T-EDG`).
- `src/models/gnc/control/aerobraking/*` owns algorithm-agnostic tracking/execution only.
- `src/orchestration/mission/operations/aerobraking_policy/*` owns strategy selection policy interfaces.

## Required boundaries
- Guidance files must not include control source files directly.
- Control files must not define targeting/switch-planning strategy solvers.
- Strategy selection must be typed and mission-owned (`AerobrakingStrategyKind`, selector contract).
- DRL integration point is a stub contract only (`DRLPolicyAdapterStub`), with explicit `Not implemented` behavior.
- Propulsive maneuver coupling must cross the typed command boundary (`PropulsiveManeuverCommand`, `AerobrakingControlCommand`) rather than `DynamicEffectors` or direct thruster-state mutation from guidance.

## Canonical interfaces
- `AerobrakingGuidanceInput`
- `AerobrakingGuidanceOutput`
- `PropulsiveManeuverCommand`
- `AerobrakingControlCommand`
- `AbstractAerobrakingStrategy`
- `AerobrakingPolicyConfig`
- `AbstractAerobrakingPolicySelector`
- `AerobrakingStrategyKind` (`E_EDG`, `T_EDG`)


## Typed EDG prediction and panel commands

The optional `SpaceAGORAEDG.jl` package owns the typed numerical kernels.
Core's `SimulationModel.EDGAlgorithms`, under
`src/models/gnc/guidance/aerobraking/typed_edg/`, retains compatibility dispatch
rather than a second implementation. `SpaceAGORA.EDGServices` is the versioned
boundary. The [EDG package boundary](@ref)
defines the supported calls, units, state ownership, installation and errors.

Guidance and control model types remain in `GuidanceHooks` and `ControlHooks`.
Core owns lifecycle, scheduling, physical integration and panel actuation. The
provider receives configuration, run-owned EDG state, copied observations and
call-scoped prediction queries. `ODEParams`, live physical state and mutable
vehicle geometry remain inside core. Legacy E-EDG/T-EDG wrappers retain their
routes; shared typed numerical calls forward to the companion.

The control hook requests `control_decision!` at its existing control times and
applies the returned `EDGControlDecision.command` only when `apply` is true.
Prediction uses `config.controlled_panel_links`, which may differ from the
control effector's links. An invalid index produces no command application or
environment query. A `:solved` status records execution, not scientific acceptance.

Numerical decisions may update the run's EDG plan, cache and diagnostic state;
they do not move panels or physical heat integrals. Exceptions still propagate
without rollback: a decision can update mode before an atmosphere error, and
completed telemetry can precede an actuator error. Control-first scheduling,
passage bookkeeping and panel application order remain core responsibilities.

Targeting retains its trajectory-based, wind-relative atmosphere query. Heat-load
prediction retains its altitude-only, zero-latitude/longitude query. These are
different scientific assumptions, not interchangeable implementations. Packaging
does not change query order, callback timing, constraints or acceptance criteria.

The GNC boundary and source-completeness gates reject numerical implementations
added back to core. Focused ownership tests cover command/application parity,
invalid-index laziness, switch endpoints, panel-selection mismatch and query
conventions. Package installation and ownership checks do not replace integrated
mission or scientific validation.
