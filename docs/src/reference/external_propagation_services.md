# External propagation service contract

[`SpaceAGORA.ExternalPropagationServices`](@ref) is the supported facade for
optional external spacecraft integrators.

This facade follows the [optional adapter standard](adapter_contract.md). This
page describes contract 2.0.0, which applies SpaceAGORA's hook conventions to the
external-owner protocol: time before index, an owner-local position named
`slot`, and one definition of the attitude quaternion. It has no global provider
registry: independently configured owners may coexist, provided they do not claim
the same spacecraft.

Use `with_external_propagators` to attach owners to a configuration,
`external_initial_state` in preparation, and `external_runtimes` in telemetry
callbacks. These helpers keep adapter code independent of configuration-copy and
engine-storage implementation details. Borrowed runtimes must not be stepped or
mutated from telemetry callbacks.

Configuration preflight checks ownership and solver compatibility. Execution
still checks routing, the flat-RHS guard and other runtime restrictions. This
facade does not add checkpoint/resume, ensemble/process-pool support, actuator
coupling or a higher-order native step.

## Conventions

- **Argument order.** `external_state(rt, t, slot)` and
  `external_acceleration(rt, t, slot)` take the elapsed engine time `t` [s]
  before `slot`. `slot` is the 1-based position in the owner's
  `external_spacecraft(ep)` list, not a run index. `external_initial_state(u0,
  spacecraft_index)` takes a run index: the position in the configuration's
  spacecraft list.
- **Frames and units.** Positions, velocities and accelerations are inertial, in
  m, m/s and m/s². Angular velocity is in body axes, in rad/s.
- **Attitude quaternion.** `q = (x, y, z, w)` is scalar last and gives
  `v_body = rot(q) * v_inertial`, with
  `rot(q) = (w^2 - e⋅e) I + 2 e eᵀ - 2 w [e×]` for `e = (x, y, z)` and `[e×]`
  the cross-product matrix of `e`. The engine's state, `StateSample.q_ib` and the
  saved `sc<i>_q_*` columns use the same convention.

## Changes from contract 1.0.0

Contract 2.0.0 changes signatures and wording only; the engine's numerical
behavior is unchanged.

- `external_state(rt, k, t)` became `external_state(rt, t, slot)`, and
  `external_acceleration(rt, k, t)` became `external_acceleration(rt, t, slot)`.
  An adapter swaps the last two arguments of both methods and requires
  `v"2.0.0"`. The exact version check refuses an adapter built for 1.0.0 before
  it defines any method. Update the methods and the required version together:
  a method typed `(rt, k::Int, t::Float64)` matches no 2.0.0 call, but an untyped
  one would receive the arguments swapped.
- The quaternion is defined by the formula above. The 1.0.0 text called it a
  scalar-last body-to-inertial quaternion; the numbers are the same.
- `StateSample`, which `sample_planet_frame_with_lpi` accepts, gained a
  `spacecraft_index` field while the contract was 1.0.0. Contract 2.0.0 records
  this additive change.

The SpaceAGORAMuJoCo companion implements contract 2.0.0 from its version 0.2.0.

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

`sample_planet_frame_with_lpi` accepts either a `StateSample` or a named tuple
`(pos_ii=..., vel_ii=...)` with `SVector{3,Float64}` values: inertial position in
metres and inertial velocity in metres per second. The named-tuple form is part of
this service contract and requires no mass or attitude fields. In both forms, the
supplied `l_pi` maps inertial components to planet-fixed components, and the
returned velocity includes the planet-rotation transport term.

```@docs
SpaceAGORA.ExternalPropagationServices.with_external_propagators
SpaceAGORA.ExternalPropagationServices.validate_external_propagators
SpaceAGORA.ExternalPropagationServices.external_runtimes
SpaceAGORA.ExternalPropagationServices.external_initial_state
SpaceAGORA.ExternalPropagationServices.sample_planet_frame_with_lpi
SpaceAGORA.ExternalPropagationServices.gravity_backbone_relative_acceleration_ii
```
