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

## Owners and users

The source inventory in
`src/orchestration/simulation/external_propagation_services.jl` defines the
boundary. The same function and type identities are exposed through the facade.

| User | Responsibility | Supported access |
|---|---|---|
| MuJoCo `src/engine_coupling.jl` and other adapter implementations | Declare owned spacecraft; prepare a fresh runtime; synchronize and report state | Methods on the eight inventoried external-owner hooks. |
| Configuration builders and the generated `companion` starter | Attach owners and perform configuration preflight | `with_external_propagators` and `validate_external_propagators`. |
| Adapter preparation | Read each spacecraft's initial state | `external_initial_state` returns copied samples using the run index. |
| Telemetry callbacks | Observe owner runtimes without changing them | `external_runtimes` returns borrowed runtimes; callbacks must not step them. |
| MuJoCo `src/proximity_scene.jl` | Obtain common frame and gravity quantities | Facade frame/gravity services and documented root API, including `StateSample` and `EnvironmentSample`. |

Configuration updates through `with_external_propagators` are shallow. They do
not clone arbitrary adapter-owned objects. Per-run preparation must establish
separate mutable runtimes. Preflight is a configuration check; runtime routing
and unsupported-mode refusals remain necessary.

MuJoCo's `test/common.jl`, `test/engine_tests.jl` and
`benchmarks/basilisk_mujoco_xval/sagora_driver.jl` also use private core paths for
fixtures or research. Those are maintenance dependencies, not supported adapter
APIs. Migrate them only when a public replacement preserves their synchronization,
frame, spacecraft-index and isolation assertions; document any remaining private
access rather than exposing raw engine storage for convenience.

Use core `test/unit/external_propagation_tests.jl` and the companion's
`test/load_order.jl`, `test/incompatible_core.jl` and `test/engine_tests.jl` for
boundary changes. Preserve both load orders and compatibility refusal before
native loading or method installation. Record whether an actual native run was
exercised; facade checks and configuration preflight alone do not certify it.

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
