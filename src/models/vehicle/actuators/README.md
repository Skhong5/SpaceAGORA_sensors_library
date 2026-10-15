# Actuators

Hardware mounted on a spacecraft link that acts on the vehicle when commanded.
Each kind describes its hardware once; the engine derives the force, torque,
propellant use and firing levels from the actuators mounted on the vehicle.
The extensibility page and `templates/actuator_template.jl` show how to add a
kind.

| File | Holds |
|---|---|
| `actuators.jl` | the contract (`Actuators`): the methods every kind implements, the evaluation context, the controller hooks `actuation_mode` and `actuator_levels`, and `allocate_actuators` |
| `mounting.jl` | mounting and the mounted set (`ActuatorMounting`): `add_actuator!`, the mounting order, `allocation_matrix`, `mounted_wrench` |
| `thruster.jl` | the thruster kind: the `Thruster` record (its `direction` is the thrust direction), its contract methods and `six_axis_thrusters` |
| `laser.jl` | the laser-terminal kind: the `LaserTerminal` record (level: the transmitted optical power; the vehicle recoils with `level / c`), its contract methods, and the link parameters it gives the interlink model |
| `hinge.jl` | the hinge kind: the `PanelHinge` record (level: the commanded angle of the link it turns; angle and rate limits), applied at control ticks |
| `thruster_models.jl` | `BaseThrusterModel` (a burn schedule per spacecraft) and `SixAxisThrusterModel` (six fixed jets) |
| `thruster_models_module.jl` | the `ThrusterModels` owner module |
| `thruster_hooks.jl` | thruster firing dynamics over a link's jets: torque allocation, Schmitt trigger, impulse integration |
| `thruster_allocation.jl` | the RPO six-axis force allocation and its wrench |
| `reaction_wheel.jl` | the reaction-wheel kind: the `ReactionWheel` record (level: the torque on the body about its spin axis; torque and momentum limits, unlimited by default), its contract methods, and the root link's `ReactionWheelAssembly` whose momenta the engine integrates |
| `magnetorquer.jl` | `Magnet`, a permanent dipole the torque-rod effector applies, and the torque-rod kind: the `Magnetorquer` record (level: the commanded dipole along its axis), its contract methods |

Records stay owned by `Components` (`src/models/vehicle/spacecraft/components.jl`),
which includes the kind files, so serialized spacecraft keep their type
identity.
