# External propagation service contract

[`SpaceAGORA.ExternalPropagationServices`](@ref) is the supported facade for
optional external spacecraft integrators.

This facade follows the [optional adapter standard](adapter_contract.md). Contract
1.0.0 exposes the existing external-owner protocol without changing the engine's
numerical behavior. It has no global provider registry: independently configured
owners may coexist, provided they do not claim the same spacecraft.

Use `with_external_propagators` to attach owners to a configuration,
`external_initial_state` in preparation, and `external_runtimes` in telemetry
callbacks. These helpers keep adapter code independent of configuration-copy and
engine-storage implementation details. Borrowed runtimes must not be stepped or
mutated from telemetry callbacks.

Configuration preflight checks ownership and solver compatibility. Execution
still checks routing, the flat-RHS guard and other runtime restrictions. This
facade does not add checkpoint/resume, ensemble/process-pool support, actuator
coupling or a higher-order native step.

## Version and inventory

```@docs
SpaceAGORA.ExternalPropagationServices.CONTRACT_VERSION
SpaceAGORA.ExternalPropagationServices.CompatibilityError
SpaceAGORA.ExternalPropagationServices.require_version
SpaceAGORA.ExternalPropagationServices.IMPLEMENTED_FUNCTIONS
SpaceAGORA.ExternalPropagationServices.CONSUMED_SERVICES
```

## Adapter implementation hooks

```@docs
SpaceAGORA.ExternalPropagationServices.AbstractExternalPropagator
SpaceAGORA.ExternalPropagationServices.external_spacecraft
SpaceAGORA.ExternalPropagationServices.external_step
SpaceAGORA.ExternalPropagationServices.external_preflight
SpaceAGORA.ExternalPropagationServices.external_prepare
SpaceAGORA.ExternalPropagationServices.external_sync!
SpaceAGORA.ExternalPropagationServices.external_time
SpaceAGORA.ExternalPropagationServices.external_state
SpaceAGORA.ExternalPropagationServices.external_acceleration
```

## Core services

```@docs
SpaceAGORA.ExternalPropagationServices.with_external_propagators
SpaceAGORA.ExternalPropagationServices.validate_external_propagators
SpaceAGORA.ExternalPropagationServices.external_runtimes
SpaceAGORA.ExternalPropagationServices.external_initial_state
SpaceAGORA.ExternalPropagationServices.sample_planet_frame_with_lpi
SpaceAGORA.ExternalPropagationServices.gravity_backbone_relative_acceleration_ii
```
