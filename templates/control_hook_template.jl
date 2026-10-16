module ExampleControlHookTemplate

using StaticArrays
using ComponentArrays
using SpaceAGORA

struct ExampleControlHook <: SpaceAGORA.AbstractControlEffectorModel
    force_n::SVector{3, Float64}
    torque_n_m::SVector{3, Float64}
    mass_flow_kg_s::Float64
end

function SpaceAGORA.calcControlEffect!(
    model::ExampleControlHook,
    u::ComponentVector,
    p,
    t::Float64,
    spacecraft_index::Int,
)
    return nothing
end

# The held command: inertial force (N) and bus-frame torque (N·m).
function SpaceAGORA.calcControlForceTorque(
    model::ExampleControlHook,
    u::AbstractVector,
    p,
    spacecraft_index::Int,
    t::Float64,
)
    return model.force_n, model.torque_n_m
end

# The mass derivative in kg/s, negative while propellant is used. An effector
# that uses no propellant can leave this method out: the fallback returns 0.0.
function SpaceAGORA.calcControlMassFlowRate(
    model::ExampleControlHook,
    u::AbstractVector,
    p,
    spacecraft_index::Int,
    t::Float64,
)
    return model.mass_flow_kg_s
end

end # module ExampleControlHookTemplate
