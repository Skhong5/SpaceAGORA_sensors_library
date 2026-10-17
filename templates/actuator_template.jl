module ExampleActuatorTemplate

# A new actuator kind and a controller that commands it. The kind says what the
# hardware does at a commanded level; the engine derives the force, torque and
# propellant use on the vehicle from the actuators mounted on it.

using LinearAlgebra
using StaticArrays
using SpaceAGORA

# 1. The hardware record: one cold-gas nozzle pushing along a fixed axis of
#    the link it is mounted on. The level is its thrust in newtons.
struct ExampleColdGasNozzle <: SpaceAGORA.AbstractActuator
    axis::SVector{3, Float64}          # thrust direction in the link frame (unit vector)
    location::SVector{3, Float64}      # nozzle position in the link frame, m
    max_thrust_n::Float64
    isp_s::Float64
end

# 2. Required: the level bounds and the wrench at a (saturated) level, both in
#    the spacecraft body frame, the torque about the body reference point.
SpaceAGORA.actuator_limits(n::ExampleColdGasNozzle) = (0.0, n.max_thrust_n)

function SpaceAGORA.actuator_wrench(n::ExampleColdGasNozzle, level, ctx::SpaceAGORA.ActuatorContext)
    force_body = ctx.link_to_body * (level * n.axis)
    arm_body = ctx.link_to_body * n.location + ctx.link_r_body
    return force_body, cross(arm_body, force_body)
end

# 3. Optional: propellant use (kg/s, positive) and power (W) at a level.
SpaceAGORA.actuator_consumption(n::ExampleColdGasNozzle, level, ctx) =
    (mass_rate_kg_s = level / (n.isp_s * 9.80665), power_w = 0.0)

# 4. A controller in `:mounted` actuation: it keeps one level per mounted
#    actuator of its spacecraft, updated at each control tick. It writes no
#    force, mass-flow or firing-level hook; the engine derives them.
mutable struct ExampleNozzleController <: SpaceAGORA.AbstractControlEffectorModel
    sat_idx::Int
    levels::Vector{Float64}
end

SpaceAGORA.actuation_mode(::ExampleNozzleController) = :mounted
SpaceAGORA.actuator_levels(c::ExampleNozzleController, spacecraft_index::Int) =
    spacecraft_index == c.sat_idx ? c.levels : nothing

function SpaceAGORA.update_control!(c::ExampleNozzleController, u, p, t::Float64, spacecraft_index::Int)
    # Compute and store the levels here (they are held until the next tick).
    return nothing
end

# Mounting, for a spacecraft `sc` and one of its links:
#     add_actuator!(sc, sc.root, ExampleColdGasNozzle(SVector(1.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0), 0.5, 60.0))
# and the controller goes in ControlModel(control_effectors=(ExampleNozzleController(1, [0.1]),), ...).

end # module ExampleActuatorTemplate
