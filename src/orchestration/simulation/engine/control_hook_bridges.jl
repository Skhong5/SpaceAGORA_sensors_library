# The control output hooks of an `:ideal` control effector, as the dynamics call
# them: control_wrench, control_propellant_rate and control_wheel_torque when
# the effector's type implements them, and otherwise the deprecated
# calcControlForceTorque, calcControlMassFlowRate and calcReactionWheelTorque.
# Each hook is routed on its own, so a type may implement some of the new hooks
# and still rely on the old form, or on the default, for the others.
#
# A bridge takes the old call's arguments and is not marked `@inline`: the
# dynamics call it where and as they called the old hook, and `hasmethod` folds
# in the bridge's specialization on the effector's type. The state sample is
# built only on the new branch, inside the bridge, so nothing extra is boxed.
# The new hook is called through a call-site `@noinline`, so it is compiled on
# its own, as a run-time-dispatched hook is, and LLVM fuses its StaticArrays
# arithmetic the same way in every caller (see `_evaluate_dynamic_effector`).
# The query uses `StateSample`, as the pre-run check does: a method typed
# `x::StateSample`, or with `x` untyped, is the one the dynamics call.

@inline _control_state_sample(sc_view, p, sat_idx::Int)::StateSample =
    build_state_sample(sc_view, p.args.dynamics_model.spacecraft[sat_idx],
        p.args.mission_configuration.orientation_sim, sat_idx)

# Typed on `F` so that it specializes on the hook, which it only passes on.
@inline _has_control_hook(f::F, model, p) where {F} =
    hasmethod(f, Tuple{typeof(model), SimulationModel.StateSample, typeof(p), Float64})

# (force_ii, torque_body)
function _control_wrench(effector, sc_view, p, sat_idx::Int, t::Float64)
    if _has_control_hook(SimulationModel.control_wrench, effector, p)
        x = _control_state_sample(sc_view, p, sat_idx)
        return @noinline SimulationModel.control_wrench(effector, x, p, t)
    end
    return SimulationModel.calcControlForceTorque(effector, sc_view, p, sat_idx, t)
end

# The propellant used, kg/s, positive. The old hook returns the mass
# derivative, so its value is negated: the accumulation subtracts this rate,
# and `m - (-r)` is `m + r` bit for bit.
function _control_propellant_rate(effector, sc_view, p, sat_idx::Int, t::Float64)
    if _has_control_hook(SimulationModel.control_propellant_rate, effector, p)
        x = _control_state_sample(sc_view, p, sat_idx)
        return @noinline SimulationModel.control_propellant_rate(effector, x, p, t)
    end
    return -SimulationModel.calcControlMassFlowRate(effector, sc_view, p, sat_idx, t)
end

# nothing, or the wheels' torque in the bus frame
function _control_wheel_torque(effector, sc_view, p, sat_idx::Int, t::Float64)
    if _has_control_hook(SimulationModel.control_wheel_torque, effector, p)
        x = _control_state_sample(sc_view, p, sat_idx)
        return @noinline SimulationModel.control_wheel_torque(effector, x, p, t)
    end
    return SimulationModel.calcReactionWheelTorque(effector, sc_view, p, sat_idx, t)
end

# Direct calls of the deprecated names. An effector type's own method of an old
# hook takes precedence over these untyped fallbacks; they serve every other
# type. On a type that implements the new hook, the call warns once per type
# that the old name is deprecated (under --depwarn=error it throws before
# evaluating anything) and returns the new hook's result, the mass rate
# negated. Otherwise calcControlForceTorque raises a MethodError naming
# control_wrench, which the pre-run check reports before a run, and the other
# two return their defaults, 0.0 and nothing, without a warning. The dynamics
# call these only through the bridges' old branches, which run only for types
# without the new hook, so no warning is reachable from the right-hand side.
function SimulationModel.calcControlForceTorque(model, u, p, i, t)
    _has_control_hook(SimulationModel.control_wrench, model, p) ||
        throw(MethodError(SimulationModel.control_wrench, (model, u, p, t)))
    SimulationModel.HookDeprecation._hook_deprecation((typeof(model), :calcControlForceTorque),
        _direct_control_hook_deprecation(typeof(model), :calcControlForceTorque, :control_wrench))
    return SimulationModel.control_wrench(model, _control_state_sample(u, p, Int(i)), p, Float64(t))
end

function SimulationModel.calcControlMassFlowRate(model, u, p, i, t)
    _has_control_hook(SimulationModel.control_propellant_rate, model, p) || return 0.0
    SimulationModel.HookDeprecation._hook_deprecation((typeof(model), :calcControlMassFlowRate),
        _direct_control_hook_deprecation(typeof(model), :calcControlMassFlowRate, :control_propellant_rate))
    return -SimulationModel.control_propellant_rate(model, _control_state_sample(u, p, Int(i)), p, Float64(t))
end

function SimulationModel.calcReactionWheelTorque(model, u, p, i, t)
    _has_control_hook(SimulationModel.control_wheel_torque, model, p) || return nothing
    SimulationModel.HookDeprecation._hook_deprecation((typeof(model), :calcReactionWheelTorque),
        _direct_control_hook_deprecation(typeof(model), :calcReactionWheelTorque, :control_wheel_torque))
    return SimulationModel.control_wheel_torque(model, _control_state_sample(u, p, Int(i)), p, Float64(t))
end

@noinline function _direct_control_hook_deprecation(::Type{T}, old::Symbol, new::Symbol) where {T}
    msg = "$(old)(model, u, p, i, t) is deprecated and will be removed in SpaceAGORA 0.5.0; " *
        "it was called directly on a $(nameof(T)). Call $(new)(model, x::StateSample, p, t::Float64) " *
        "instead, with the spacecraft's index in x.spacecraft_index."
    new === :control_propellant_rate && (msg *= " It returns the propellant used, positive, " *
        "where $(old) returns the mass derivative, negative.")
    return msg
end
