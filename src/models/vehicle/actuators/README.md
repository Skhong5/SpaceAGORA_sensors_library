# Actuators

Hardware that acts on the vehicle. One file per actuator kind; the thruster is
the template.

| File | Holds |
|---|---|
| `thruster_record.jl` | `Thruster`: one mounted jet (location, exhaust direction, rating, Isp, firing dynamics constants) |
| `thruster_models.jl` | `BaseThrusterModel` (burn schedule per spacecraft) and `SixAxisThrusterModel` (six fixed jets) |
| `thruster_models_module.jl` | the `ThrusterModels` owner module |
| `thruster_hooks.jl` | torque allocation over mounted jets, Schmitt trigger and impulse integration |
| `thruster_allocation.jl` | per-axis allocation of a body force over six fixed jets and its wrench |
| `reaction_wheel_record.jl` | `ReactionWheelAssembly`: wheel Jacobian, limits and stored momentum |
| `magnetorquer_record.jl` | `Magnet`: a body-fixed dipole |

The records are included and exported by `Components`
(`src/models/vehicle/spacecraft/components.jl`) until the actuator contract
lands; see the actuator-layer plan.
