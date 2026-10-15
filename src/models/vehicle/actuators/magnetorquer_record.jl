# Included by `Components` (src/models/vehicle/spacecraft/components.jl), which
# owns and exports this record. The actuator layer takes ownership in the
# contract step of the actuator plan.

@kwdef mutable struct Magnet
    m::MVector{3, Float64} = MVector{3, Float64}(zeros(3)) # Magnetic dipole moment in the body frame, A*m^2
    location::MVector{3, Float64} = MVector{3, Float64}(zeros(3)) # Location in the link frame, relative to the CoM of the link, m
end
