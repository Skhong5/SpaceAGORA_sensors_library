"""Robot-arm reaction effector module that couples joint-load feedback into spacecraft force and torque models."""
module RobotArmReactionEffectors

using StaticArrays
using ...AbstractTypes: AbstractForceTorqueModel
using ...ClothMultibody: CompliantJointActuator
using ...RobotArmPlanning: RobotArmPlan, robot_arm_plan_sample
using ...EffectorSampling: StateSample, EnvironmentSample
import ..DynamicEffectors: wrench
using ...HookDeprecation: _renamed_field_warning, _renamed_keyword, _NOT_PASSED

export RobotArmReactionEffector

"""
Force-torque effector that feeds robot-arm joint reaction loads back to the base of
the spacecraft at run position `spacecraft_index`. `spacecraft_idx`, its old name,
still works with a deprecation warning until SpaceAGORA 0.5.0.
"""
mutable struct RobotArmReactionEffector <: AbstractForceTorqueModel
    spacecraft_index::Int
    plan::Union{Nothing, RobotArmPlan}
    updated_at_s::Float64
    force_scale::Float64
    torque_scale::Float64
    k_translation_n_m::Any
    c_translation_n_s_m::Any
    k_rotation_n_m_rad::Any
    c_rotation_n_m_s_rad::Any
    joint_actuators::Vector{CompliantJointActuator}
end
# The deprecated name forwards, as a keyword and as a property.
function RobotArmReactionEffector(; spacecraft_index=_NOT_PASSED, spacecraft_idx=_NOT_PASSED, plan=nothing,
        updated_at_s=0.0, force_scale=0.0, torque_scale=0.0, k_translation_n_m=5.0e3, c_translation_n_s_m=30.0,
        k_rotation_n_m_rad=15.0, c_rotation_n_m_s_rad=0.5, joint_actuators=CompliantJointActuator[])
    T = RobotArmReactionEffector
    return T(_renamed_keyword(T, :spacecraft_idx, spacecraft_idx, :spacecraft_index, spacecraft_index, 1),
        plan, updated_at_s, force_scale, torque_scale, k_translation_n_m, c_translation_n_s_m, k_rotation_n_m_rad,
        c_rotation_n_m_s_rad, joint_actuators)
end
@inline _reaction_effector_field(name::Symbol) = name === :spacecraft_idx ?
    _renamed_field_warning(RobotArmReactionEffector, :spacecraft_idx, :spacecraft_index) : name
@inline Base.getproperty(m::RobotArmReactionEffector, name::Symbol) = getfield(m, _reaction_effector_field(name))
@inline function Base.setproperty!(m::RobotArmReactionEffector, name::Symbol, x)
    field = _reaction_effector_field(name)
    return setfield!(m, field, convert(fieldtype(RobotArmReactionEffector, field), x))
end

# The robot-arm reaction force and torque on the spacecraft at `x.spacecraft_index`.
function wrench(model::RobotArmReactionEffector, x::StateSample, env::EnvironmentSample, t::Float64)
    zero3 = SVector{3, Float64}(0.0, 0.0, 0.0)
    x.spacecraft_index == model.spacecraft_index || return zero3, zero3
    model.plan === nothing && return zero3, zero3
    # Placeholder hook for the coupled backend: the feedforward path is present,
    # but reaction wrench estimation belongs with compliant multibody/RNEA dynamics.
    return zero3, zero3
end


end # module RobotArmReactionEffectors
