"""
Mounting actuators on spacecraft links and evaluating the mounted set.

The mounting order is the order every per-actuator vector follows (levels,
allocation columns): first every thruster, link by link in `model.links`
order and within a link in `link.thrusters` order (the order of the
`thruster_level` result columns and of the viewer's plumes), then every
actuator of a user-defined kind, link by link in `link.custom_actuators`
order.
"""
module ActuatorMounting

using LinearAlgebra
using StaticArrays
using ..AbstractTypes: AbstractActuator
using ..QuaternionMath: rot
using ..Actuators
using ..Components: Thruster
using ..SpacecraftModels: Link, SpacecraftModel
using ..Assembly: add_thruster!

export add_actuator!, mounted_actuators, mounted_actuator_count, mounted_thruster_count
export actuator_context, allocation_matrix, mounted_wrench, mounted_thruster_fractions

const _ZERO3 = SVector{3, Float64}(0.0, 0.0, 0.0)
const _IDENTITY3 = SMatrix{3, 3, Float64, 9}(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
const _IDENTITY_Q = SVector{4, Float64}(0.0, 0.0, 0.0, 1.0)

"""
    add_actuator!(model::SpacecraftModel, link::Link, actuator) -> actuator

Mount `actuator` on `link`, one of `model`'s links. A [`Thruster`](@ref) joins
`link.thrusters` (its direction is normalized and the link's thruster torque
Jacobian gains its column); an actuator of any other [`AbstractActuator`](@ref)
kind joins `link.custom_actuators`. Returns the actuator.
"""
function add_actuator!(model::SpacecraftModel, link::Link, thruster::Thruster)
    add_thruster!(model, link, thruster)
    return thruster
end

function add_actuator!(model::SpacecraftModel, link::Link, actuator::AbstractActuator)
    push!(link.custom_actuators, actuator)
    return actuator
end

"""
    mounted_thruster_count(model::SpacecraftModel) -> Int

The number of thrusters mounted on `model`'s links.
"""
function mounted_thruster_count(model::SpacecraftModel)::Int
    n = 0
    for link in model.links
        n += length(link.thrusters)
    end
    return n
end

"""
    mounted_actuator_count(model::SpacecraftModel) -> Int

The number of actuators mounted on `model`'s links, of every kind.
"""
function mounted_actuator_count(model::SpacecraftModel)::Int
    n = mounted_thruster_count(model)
    for link in model.links
        n += length(link.custom_actuators)
    end
    return n
end

"""
    mounted_actuators(model::SpacecraftModel) -> Vector{NamedTuple{(:link, :actuator)}}

Every actuator mounted on `model`, with the link it is mounted on, in mounting
order: the thrusters first, then the actuators of user-defined kinds.
"""
function mounted_actuators(model::SpacecraftModel)
    out = NamedTuple{(:link, :actuator), Tuple{Link, AbstractActuator}}[]
    for link in model.links, thruster in link.thrusters
        push!(out, (link=link, actuator=thruster))
    end
    for link in model.links, actuator in link.custom_actuators
        push!(out, (link=link, actuator=actuator))
    end
    return out
end

"""
    actuator_context(link::Link; mass_kg=NaN, q_ib=identity, ω_body=zero) -> ActuatorContext

The evaluation context of an actuator mounted on `link`: the link's configured
pose in the body frame (its `r` and, for a non-root link, its attitude `q`)
and the vehicle's mass, attitude and rate.
"""
actuator_context(link::Link; mass_kg::Real=NaN, q_ib=_IDENTITY_Q, ω_body=_ZERO3) =
    actuator_context(link, Float64(mass_kg), SVector{4, Float64}(q_ib), SVector{3, Float64}(ω_body))

@inline function actuator_context(link::Link, mass_kg::Float64, q_ib::SVector{4, Float64}, ω_body::SVector{3, Float64})
    # Same link-to-body map as the thruster torque Jacobian (Kinematics.rotate_to_body).
    link_to_body = link.root ? _IDENTITY3 : SMatrix{3, 3, Float64, 9}(rot(SVector{4, Float64}(link.q))')
    return ActuatorContext(SVector{3, Float64}(link.r), link_to_body, mass_kg, q_ib, ω_body)
end

"""
    allocation_matrix(model::SpacecraftModel) -> Matrix{Float64}

The 6 × n allocation matrix of the actuators mounted on `model`, in mounting
order: column `k` stacks the force (rows 1 to 3, N) and the torque about the
body reference point (rows 4 to 6, N m) of actuator `k` at level one, both in
the body frame (see [`actuator_unit_wrench`](@ref)). Use it with
[`allocate_actuators`](@ref).
"""
function allocation_matrix(model::SpacecraftModel)::Matrix{Float64}
    mounted = mounted_actuators(model)
    B = zeros(Float64, 6, length(mounted))
    for (k, entry) in enumerate(mounted)
        force, torque = actuator_unit_wrench(entry.actuator, actuator_context(entry.link))
        B[1:3, k] .= force
        B[4:6, k] .= torque
    end
    return B
end

@inline function _add_actuator(actuator, level, ctx::ActuatorContext,
                               force::SVector{3, Float64}, torque::SVector{3, Float64}, mass_rate::Float64)
    x = actuator_saturate(actuator, level)
    f, tq = actuator_wrench(actuator, x, ctx)
    use = actuator_consumption(actuator, x, ctx)
    return force + SVector{3, Float64}(f), torque + SVector{3, Float64}(tq), mass_rate + Float64(use.mass_rate_kg_s)
end

# Function barrier for user-defined kinds held in an abstractly typed list.
@noinline _add_custom_actuator(actuator::AbstractActuator, level, ctx, force, torque, mass_rate) =
    _add_actuator(actuator, level, ctx, force, torque, mass_rate)

"""
    mounted_wrench(model::SpacecraftModel, levels; mass_kg=NaN, q_ib=identity, ω_body=zero)
        -> (force_body, torque_body, mass_rate_kg_s)

The wrench of every actuator mounted on `model` at its level in `levels` (one
per mounted actuator, in mounting order), each level first saturated: the
total force and torque in the body frame, the torque about the body reference
point, and the total propellant mass rate (kg/s, positive while consuming).
"""
mounted_wrench(model::SpacecraftModel, levels::AbstractVector; mass_kg::Real=NaN, q_ib=_IDENTITY_Q, ω_body=_ZERO3) =
    mounted_wrench(model, levels, Float64(mass_kg), SVector{4, Float64}(q_ib), SVector{3, Float64}(ω_body))

function mounted_wrench(model::SpacecraftModel, levels::AbstractVector, mass_kg::Float64,
                        q_ib::SVector{4, Float64}, ω_body::SVector{3, Float64})
    n = mounted_actuator_count(model)
    length(levels) == n || throw(ArgumentError(
        "Mounted actuation: $(length(levels)) actuator levels for the $(n) actuators mounted on spacecraft $(model.id)."))
    force = _ZERO3
    torque = _ZERO3
    mass_rate = 0.0
    k = 0
    @inbounds for link in model.links
        isempty(link.thrusters) && continue
        ctx = actuator_context(link, mass_kg, q_ib, ω_body)
        for thruster in link.thrusters
            k += 1
            force, torque, mass_rate = _add_actuator(thruster, levels[k], ctx, force, torque, mass_rate)
        end
    end
    @inbounds for link in model.links
        isempty(link.custom_actuators) && continue
        ctx = actuator_context(link, mass_kg, q_ib, ω_body)
        for actuator in link.custom_actuators
            k += 1
            # The list is abstractly typed: assert the result so the loop stays concretely typed.
            force, torque, mass_rate = _add_custom_actuator(actuator, levels[k], ctx, force, torque, mass_rate)::Tuple{SVector{3, Float64}, SVector{3, Float64}, Float64}
        end
    end
    return force, torque, mass_rate
end

"""
    mounted_thruster_fractions(model::SpacecraftModel, levels) -> Vector{Float64}

The firing level, from 0 to 1, of each thruster mounted on `model`, from the
first entries of `levels` (mounting order): each saturated level as a fraction
of the thruster's rating.
"""
function mounted_thruster_fractions(model::SpacecraftModel, levels::AbstractVector)::Vector{Float64}
    out = Float64[]
    k = 0
    for link in model.links, thruster in link.thrusters
        k += 1
        x = k <= length(levels) ? actuator_saturate(thruster, levels[k]) : 0.0
        push!(out, actuator_fraction(thruster, x))
    end
    return out
end

end # module ActuatorMounting
