# The thruster actuator kind. Included by `Components`
# (src/models/vehicle/spacecraft/components.jl), which owns and exports the
# record, so serialized spacecraft keep their type identity; the actuator
# contract it implements is in `actuators.jl`.

# ---- Record ---------------------------------------------------------------

"""
    Thruster(; max_thrust, location, direction, Isp, ...)

One jet mounted on a spacecraft link. `direction` is the direction of the
thrust it puts on the vehicle (not the direction of its exhaust), in the link
frame; `location` is the nozzle position in the link frame, relative to the
link's centre of mass, metres; `max_thrust` is its rating in newtons and
`Isp` its specific impulse in seconds. The remaining fields describe its
firing dynamics and hold its state during a run.

A `Thruster` is an [`AbstractActuator`](@ref) whose level is a thrust in
newtons, from zero to `max_thrust`. Mount it with `add_actuator!`.
"""
@kwdef mutable struct Thruster <: AbstractActuator
    max_thrust::Float64 = 1.0 # Maximum thrust, N
    location::MVector{3, Float64} = MVector{3, Float64}(zeros(3)) # Location in the link frame, relative to the CoM of the link, m
    direction::MVector{3, Float64} = MVector{3, Float64}(zeros(3)) # Unit vector direction of thrust in the link frame, n/d
    Isp::Float64 = 0.0 # Specific impulse of the thruster, s
    cutoff_frequency::Float64 = 1.0 # Governing parameter for thruster ramp-up/ramp-down, rad/s.
    min_firing_time::Float64 = 0.0 # Minimum time for the thruster to be firing, s.
    level_on::Float64 = 0.75 # Upper threshold for the Schmitt trigger, nd
    level_off::Float64 = 0.25 # Lower threshold for the Schmitt trigger, nd
    κ::Float64 = 0.0 # Thrust factor, nd.
    thrust::Float64 = 0.0 # Current thrust magnitude, to be updated during simulation.
    stop_firing_time::Float64 = 0.0 # Time at which the thruster should stop firing, s.
end

# ---- State ----------------------------------------------------------------
#
# The mutable fields `thrust`, `κ` and `stop_firing_time` above hold the firing
# state; `run_simulation(isolate_state=true)` copies them per run. The firing
# dynamics that update them (Schmitt trigger, impulse integration, ramp
# factor) are in `thruster_hooks.jl`, which needs the link types.

# ---- Methods of the actuator contract --------------------------------------

actuator_limits(thruster::Thruster) = (0.0, Float64(thruster.max_thrust))

"""
    actuator_wrench(thruster::Thruster, level, ctx) -> (force_body, torque_body)

A thrust of `level` newtons along the thruster's `direction`, applied at its
`location`: the force in the body frame and its torque about the body
reference point. A zero or non-finite direction gives no wrench.
"""
@inline function actuator_wrench(thruster::Thruster, level::Real, ctx::ActuatorContext)
    direction = SVector{3, Float64}(thruster.direction)
    n = norm(direction)
    (isfinite(n) && n > 0.0) || return (SVector{3, Float64}(0.0, 0.0, 0.0), SVector{3, Float64}(0.0, 0.0, 0.0))
    force_body = ctx.link_to_body * ((Float64(level) / n) * direction)
    arm_body = ctx.link_to_body * SVector{3, Float64}(thruster.location) + ctx.link_r_body
    return force_body, cross(arm_body, force_body)
end

"""
    actuator_consumption(thruster::Thruster, level, ctx) -> (mass_rate_kg_s, power_w)

Propellant use at a thrust of `level` newtons, `level / (Isp g0)`; zero when
`Isp` is not positive. No electrical power.
"""
@inline function actuator_consumption(thruster::Thruster, level::Real, ::ActuatorContext)
    isp = Float64(thruster.Isp)
    thrust = Float64(level)
    rate = (isfinite(isp) && isp > 0.0 && isfinite(thrust) && thrust > 0.0) ?
        thrust / (isp * STANDARD_GRAVITY_MPS2) : 0.0
    return (mass_rate_kg_s = rate, power_w = 0.0)
end

# ---- Builders ---------------------------------------------------------------

"""
    six_axis_thrusters(model::SixAxisThrusterModel) -> Vector{Thruster}

The six jets of a six-axis thruster model as mountable thrusters, in the
model's column order: each one's thrust direction, location, rating, specific
impulse and minimum firing time. Mount them on the vehicle with
`add_actuator!` to drive them in `:mounted` actuation.
"""
function six_axis_thrusters(model::SixAxisThrusterModel)::Vector{Thruster}
    return [Thruster(
        max_thrust=model.max_thrust_n[j],
        location=MVector{3, Float64}(model.locations_body[:, j]),
        direction=MVector{3, Float64}(model.directions_body[:, j]),
        Isp=model.isp_s[j],
        min_firing_time=model.min_firing_time_s,
    ) for j in 1:6]
end
