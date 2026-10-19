# Migrating Extension Hooks

SpaceAGORA's extension hooks follow one set of [conventions](extension_conventions.md):
time before the spacecraft index, a state sample for right-hand-side hooks, and
positive consumption. Ten hook names, and one form of the plume model's
`wrench`, did not. SpaceAGORA 0.3.0 deprecated them, 0.3.0 and 0.4.0 warned
about them before each run, and 0.5.0 removed them. This page lists each with
its replacement.

A method of a removed name no longer runs. One that names the package, such as
`SpaceAGORA.calcForceTorque(...) = ...`, fails when its file loads, because the
name is gone. A model left with no method of the replacement is refused by the
pre-run check, which names the hook it needs. To find the old names in your
code:

```bash
grep -rnE "calcForceTorque|calcControl(ForceTorque|MassFlowRate|Effect!)|calcReactionWheelTorque|calc(Guidance|Navigation)Effect!|planet_frame_lpi|ephemerides_(sun_direction_ii|time_seconds)" src test
```

## Removed hook names

| Removed in 0.5.0 | Replacement | What changes |
|---|---|---|
| `calcForceTorque(model, x, p, i)` | `wrench(model, x::StateSample, env::EnvironmentSample, t::Float64)` | a state sample instead of the raw view; the environment the engine sampled instead of `p`; the time as an argument and the index in `x.spacecraft_index` |
| `wrench(model::PlumeSurfaceInteractionModel, x, env, t, i)` | `wrench(model, x, env, t)` | the index in `x.spacecraft_index` instead of an argument |
| `calcControlForceTorque(model, u, p, i, t)` | `control_wrench(model, x::StateSample, p, t::Float64)` | a state sample instead of the raw view, the time before the index, the index in `x.spacecraft_index` |
| `calcControlMassFlowRate(model, u, p, i, t)` | `control_propellant_rate(model, x::StateSample, p, t::Float64)` | the same, and **the sign** |
| `calcReactionWheelTorque(model, u, p, i, t)` | `control_wheel_torque(model, x::StateSample, p, t::Float64)` | the same as for the wrench |
| `calcControlEffect!(model, u, p, t, i)` | `update_control!(model, u, p, t, spacecraft_index)` | the name only |
| `calcGuidanceEffect!(model, u, p, t, i)` | `update_guidance!(model, u, p, t, spacecraft_index)` | the name only |
| `calcNavigationEffect!(model, u, p, t, i)` | `update_navigation!(model, u, p, t, spacecraft_index)` | the name only |
| `planet_frame_lpi(planet, et, model)` | `ephemerides_frame_lpi(model, planet, et)` | the model first |
| `ephemerides_sun_direction_ii(planet, et, model)` | `ephemerides_sun_ii(model, planet, et)` | the model first, and the name |
| `ephemerides_time_seconds(initial_time, model)` | `ephemerides_epoch_seconds(model, initial_time)` | the model first, and the name |

!!! warning "The sign changes"
    `calcControlMassFlowRate` returned the spacecraft's mass derivative,
    **negative** while propellant is used. `control_propellant_rate` returns the
    propellant used, **positive**. Flip the sign when you rename the method: a
    method renamed without it makes the vehicle gain mass.

The frames, units and values are otherwise unchanged. An effector that uses no
propellant, or drives no wheel, needs no `control_propellant_rate` or
`control_wheel_torque`: their defaults return `0.0` and `nothing`. A force or
torque model declares what it reads from the environment with
`environment_requirements(model)`. [Adding a Force or Torque of Your Own](../user/custom_effector.md)
and [Extending SpaceAGORA](../extensibility.md) show complete methods of the
replacements.

## Other deprecations in the 0.3.0 cycle

| Item | Replacement | Status | Removed in |
|---|---|---|---|
| `RPOGuidanceModel` | `make_rpo_configuration` with a planner, for example `planner=HYPRRPOPlanner(RPOPSOConfig(...))`, which configures a `PlannerGuidance` | warns before the run, once per session; the warning does not stop a run under `--depwarn=error` | a later release |
| index fields and keywords `sat_idx`, `spacecraft_idx`, `chaser_idx`, `target_idx`, and `RobotArmReactionEffector.spacecraft_idx` | `spacecraft_index`, `chaser_index`, `target_index` | warn from 0.3.0; `RobotArmReactionEffector`'s from 0.3.1 | 0.5.0 |
| selecting an RPO planner by a Symbol, such as `:rrt_star` | a planner object | warns from 0.3.0 | 0.5.0 |
| a default random stream in the GNC planning functions | an explicit `rng` keyword | required now, without a window | |

## Schedule

- **0.3.0.** The new names, with warnings for the old ones.
  `ExternalPropagationServices`, `HYPRServices` and `EDGServices` are at
  contract 2.0.0, with SpaceAGORAMuJoCo 0.2.0, HYPR 0.2.0 and SpaceAGORAEDG 0.2.0.
- **0.3.1.** `RobotArmReactionEffector.spacecraft_idx` becomes `spacecraft_index`,
  with a warning for the old name.
- **0.4.0.** The companion packages use the new names.
- **0.5.0, or 1.0 if that comes first.** The deprecated names are removed, and
  `control_propellant_rate` and `control_wheel_torque` gain defaults, `0.0` and
  `nothing`. The removal waits until a tagged release has carried the warnings
  and the companion packages have moved.

Release numbers follow `version` in `Project.toml`. The older `v1.0.0` and
`v2.0.0` git tags predate it.

## Checklist

1. Find each method of an old name with the command above. On 0.4.0, a test
   run with `julia --depwarn=error` also stops at each one.
2. Rename each tick.
3. Move each output hook to its state-sample form, and flip the sign of the
   propellant rate.
4. Move each `calcForceTorque` method to `wrench`, with an
   `environment_requirements` method if it reads the atmosphere, the Sun or the
   planet-fixed frame.
5. Type the sample argument `x::StateSample`, or leave it untyped (see the
   [conventions](extension_conventions.md#The-supported-sample-argument)).
6. Run the tests again: the pre-run check accepts every model.
