# Migrating Extension Hooks

SpaceAGORA's extension hooks follow one set of [conventions](extension_conventions.md):
time before the spacecraft index, a state sample for right-hand-side hooks, and
positive consumption. Ten hook names, and one form of the plume model's
`wrench`, did not, and are deprecated. This page lists each with its
replacement, shows how to move a method, and gives the schedule.

Your old methods keep working until the names are removed. Before each run, the
pre-run check warns once per model type and old name. Run with
`julia --depwarn=error` to turn the warnings into errors, so that a test run
finds every method still to move. To find them in your code:

```bash
grep -rnE "calcForceTorque|calcControl(ForceTorque|MassFlowRate|Effect!)|calcReactionWheelTorque|calc(Guidance|Navigation)Effect!|planet_frame_lpi|ephemerides_(sun_direction_ii|time_seconds)" src test
```

## Force and torque models

| Deprecated | Replacement | What changes |
|---|---|---|
| `calcForceTorque(model, x, p, i)` | `wrench(model, x::StateSample, env::EnvironmentSample, t::Float64)` | a state sample instead of the raw view; the environment the engine sampled instead of `p`; the time as an argument and the index in `x.spacecraft_index` |
| `wrench(model::PlumeSurfaceInteractionModel, x, env, t, i)` | `wrench(model, x, env, t)` | the index in `x.spacecraft_index` instead of an argument |

A model declares what it reads from the environment with
`environment_requirements(model)`; the engine fills `env` once per evaluation.
[Adding a Force or Torque of Your Own](../user/custom_effector.md) walks through
a complete model.

```julia
# Before
SpaceAGORA.calcForceTorque(m::Push, x, p, i) =
    (m.force_n * x.vel / norm(x.vel), zero(SVector{3, Float64}))

# After
SpaceAGORA.wrench(m::Push, x::SpaceAGORA.StateSample, env::SpaceAGORA.EnvironmentSample, t::Float64) =
    (m.force_n * x.vel_ii / norm(x.vel_ii), zero(SVector{3, Float64}))
```

## Control outputs

| Deprecated | Replacement | What changes |
|---|---|---|
| `calcControlForceTorque(model, u, p, i, t)` | `control_wrench(model, x::StateSample, p, t::Float64)` | a state sample instead of the raw view, the time before the index, the index in `x.spacecraft_index` |
| `calcControlMassFlowRate(model, u, p, i, t)` | `control_propellant_rate(model, x::StateSample, p, t::Float64)` | the same, and **the sign** |
| `calcReactionWheelTorque(model, u, p, i, t)` | `control_wheel_torque(model, x::StateSample, p, t::Float64)` | the same as for the wrench |

!!! warning "The sign changes"
    `calcControlMassFlowRate` returned the spacecraft's mass derivative,
    **negative** while propellant is used. `control_propellant_rate` returns the
    propellant used, **positive**. Flip the sign when you rename the method: a
    method renamed without it makes the vehicle gain mass.

```julia
# Before
SpaceAGORA.calcControlForceTorque(m::Burn, u, p, i, t) =
    i == m.spacecraft_index ? (m.force_ii, zero(SVector{3, Float64})) : (zero(SVector{3, Float64}), zero(SVector{3, Float64}))
SpaceAGORA.calcControlMassFlowRate(m::Burn, u, p, i, t) =
    i == m.spacecraft_index ? -m.propellant_kg_s : 0.0

# After
SpaceAGORA.control_wrench(m::Burn, x::SpaceAGORA.StateSample, p, t::Float64) =
    x.spacecraft_index == m.spacecraft_index ? (m.force_ii, zero(SVector{3, Float64})) : (zero(SVector{3, Float64}), zero(SVector{3, Float64}))
SpaceAGORA.control_propellant_rate(m::Burn, x::SpaceAGORA.StateSample, p, t::Float64) =
    x.spacecraft_index == m.spacecraft_index ? m.propellant_kg_s : 0.0
```

The frames and units are unchanged: the force is inertial, in N, and the
torques are in the body frame, in N·m. An effector that uses no propellant, or
drives no wheel, needs no `control_propellant_rate` or `control_wheel_torque`.
Each hook is routed on its own, so an effector can move one hook at a time.

## GNC ticks

| Deprecated | Replacement | What changes |
|---|---|---|
| `calcControlEffect!(model, u, p, t, i)` | `update_control!(model, u, p, t, spacecraft_index)` | the name only |
| `calcGuidanceEffect!(model, u, p, t, i)` | `update_guidance!(model, u, p, t, spacecraft_index)` | the name only |
| `calcNavigationEffect!(model, u, p, t, i)` | `update_navigation!(model, u, p, t, spacecraft_index)` | the name only |

The arguments, their order and the times the engine calls each tick are
unchanged, so moving a tick is a rename:

```julia
# Before
function SpaceAGORA.calcControlEffect!(m::Burn, u, p, t::Float64, spacecraft_index::Int)
    # ...
end

# After
function SpaceAGORA.update_control!(m::Burn, u, p, t::Float64, spacecraft_index::Int)
    # ...
end
```

## Ephemerides models

| Deprecated | Replacement | What changes |
|---|---|---|
| `planet_frame_lpi(planet, et, model)` | `ephemerides_frame_lpi(model, planet, et)` | the model first |
| `ephemerides_sun_direction_ii(planet, et, model)` | `ephemerides_sun_ii(model, planet, et)` | the model first, and the name |
| `ephemerides_time_seconds(initial_time, model)` | `ephemerides_epoch_seconds(model, initial_time)` | the model first, and the name |

The values the hooks return are unchanged. A model of your own subtypes
`AbstractEphemeridesModel`; [Ephemerides models](../user/simulation_configuration.md#Ephemerides-models)
lists all five hooks, with `ephemerides_requires_spice` and
`ephemerides_cache_key`.

```julia
# Before
SpaceAGORA.planet_frame_lpi(planet, et, m::FixedFrame) = m.L
SpaceAGORA.ephemerides_time_seconds(initial_time, m::FixedFrame) = 0.0

# After
SpaceAGORA.ephemerides_frame_lpi(m::FixedFrame, planet, et) = m.L
SpaceAGORA.ephemerides_epoch_seconds(m::FixedFrame, initial_time) = 0.0
```

## Other deprecations in the 0.3.0 cycle

| Item | Replacement | Status | Removed in |
|---|---|---|---|
| `RPOGuidanceModel` | `make_rpo_configuration` with a planner, for example `planner=HYPRRPOPlanner(RPOPSOConfig(...))`, which configures a `PlannerGuidance` | warns before the run, once per session; the warning does not stop a run under `--depwarn=error` | a later release |
| index fields and keywords `sat_idx`, `spacecraft_idx`, `chaser_idx`, `target_idx` | `spacecraft_index`, `chaser_index`, `target_index` | warn from 0.3.0 | 0.5.0 |
| selecting an RPO planner by a Symbol, such as `:rrt_star` | a planner object | warns from 0.3.0 | 0.5.0 |
| a default random stream in the GNC planning functions | an explicit `rng` keyword | required now, without a window | |

## Schedule

- **0.3.0.** The new names, with warnings for the old ones.
  `ExternalPropagationServices` and `HYPRServices` are at contract 2.0.0, with
  SpaceAGORAMuJoCo 0.2.0 and HYPR 0.2.0; `EDGServices` 2.0.0 follows with
  SpaceAGORAEDG 0.2.0.

  !!! note "TODO(0.3.0 PR 2): EDGServices 2.0.0"
      When the EDGServices 2.0.0 pair merges, say that all three facades are at
      contract 2.0.0, with SpaceAGORAEDG 0.2.0.
- **0.4.0.** The companion packages use the new names.
- **0.5.0, or 1.0 if that comes first.** The deprecated names are removed, and
  `control_propellant_rate` and `control_wheel_torque` gain defaults, `0.0` and
  `nothing`. The removal waits until a tagged release has carried the warnings
  and the companion packages have moved.

Release numbers follow `version` in `Project.toml`. The older `v1.0.0` and
`v2.0.0` git tags predate it.

## Checklist

1. Run your tests with `julia --depwarn=error` and note each warning.
2. Rename each tick.
3. Move each output hook to its state-sample form, and flip the sign of the
   propellant rate.
4. Move each `calcForceTorque` method to `wrench`, with an
   `environment_requirements` method if it reads the atmosphere, the Sun or the
   planet-fixed frame.
5. Type the sample argument `x::StateSample`, or leave it untyped (see the
   [conventions](extension_conventions.md#The-supported-sample-argument)).
6. Run the tests again: no warning should remain.
