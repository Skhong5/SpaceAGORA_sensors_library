# Included by `Components` (src/models/vehicle/spacecraft/components.jl), which
# owns and exports this record. The actuator layer takes ownership in the
# contract step of the actuator plan.

@kwdef mutable struct Thruster
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
