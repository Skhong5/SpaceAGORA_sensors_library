module ExampleControlHookTemplate

using StaticArrays
using ComponentArrays
using SpaceAGORA

struct ExampleControlHook <: SpaceAGORA.AbstractControlEffectorModel
    force_n::SVector{3, Float64}
    torque_n_m::SVector{3, Float64}
    propellant_kg_s::Float64
end

function SpaceAGORA.update_control!(
    model::ExampleControlHook,
    u::ComponentVector,
    p,
    t::Float64,
    spacecraft_index::Int,
)
    return nothing
end

# The held command: inertial force (N) and bus-frame torque (N·m). `x` is the
# spacecraft's state sample; its position in the run is `x.spacecraft_index`.
function SpaceAGORA.control_wrench(
    model::ExampleControlHook,
    x::SpaceAGORA.StateSample,
    p,
    t::Float64,
)
    return model.force_n, model.torque_n_m
end

# The propellant used in kg/s, positive while propellant is used. An effector
# that uses no propellant can leave this method out.
function SpaceAGORA.control_propellant_rate(
    model::ExampleControlHook,
    x::SpaceAGORA.StateSample,
    p,
    t::Float64,
)
    return model.propellant_kg_s
end

end # module ExampleControlHookTemplate
