# The GNC ticks, as the callbacks call them: update_navigation!,
# update_guidance! and update_control! when the effector's type implements them
# for the arguments the callback passes, and otherwise the deprecated
# calcNavigationEffect!, calcGuidanceEffect! and calcControlEffect!. Only the
# name changes, so each bridge passes its arguments on unchanged. `hasmethod`
# on the exact argument types is the lookup the call makes, and it folds in the
# bridge's specialization on the effector's type.
import ..NavigationHooks, ..GuidanceHooks, ..ControlHooks, ..HookDeprecation
using ..NavigationHooks: update_navigation!, calcNavigationEffect!
using ..GuidanceHooks: update_guidance!, calcGuidanceEffect!
using ..ControlHooks: update_control!, calcControlEffect!

@inline _has_tick(f::F, model, u, p, t, spacecraft_index) where {F} =
    hasmethod(f, Tuple{typeof(model), typeof(u), typeof(p), typeof(t), typeof(spacecraft_index)})

function _update_navigation!(model, u, p, t, spacecraft_index)
    _has_tick(update_navigation!, model, u, p, t, spacecraft_index) &&
        return update_navigation!(model, u, p, t, spacecraft_index)
    return calcNavigationEffect!(model, u, p, t, spacecraft_index)
end

function _update_guidance!(model, u, p, t, spacecraft_index)
    _has_tick(update_guidance!, model, u, p, t, spacecraft_index) &&
        return update_guidance!(model, u, p, t, spacecraft_index)
    return calcGuidanceEffect!(model, u, p, t, spacecraft_index)
end

function _update_control!(model, u, p, t, spacecraft_index)
    _has_tick(update_control!, model, u, p, t, spacecraft_index) &&
        return update_control!(model, u, p, t, spacecraft_index)
    return calcControlEffect!(model, u, p, t, spacecraft_index)
end

# Direct calls of the deprecated names. An effector type's own method of an old
# tick takes precedence over these untyped fallbacks; they serve every other
# type. On a type that implements the new tick for these arguments, the call
# warns once per type that the old name is deprecated (under --depwarn=error it
# throws before ticking) and runs the new tick. Otherwise it raises a
# MethodError naming the new tick, which the pre-run check reports before a run.
# The callbacks call these only through the bridges' old branches, which run
# only for types without the new tick, so no warning is reachable from a
# callback.
function NavigationHooks.calcNavigationEffect!(model, u, p, t, spacecraft_index)
    _has_tick(update_navigation!, model, u, p, t, spacecraft_index) ||
        throw(MethodError(update_navigation!, (model, u, p, t, spacecraft_index)))
    HookDeprecation._hook_deprecation((typeof(model), :calcNavigationEffect!),
        _direct_tick_deprecation(typeof(model), :calcNavigationEffect!, :update_navigation!))
    return update_navigation!(model, u, p, t, spacecraft_index)
end

function GuidanceHooks.calcGuidanceEffect!(model, u, p, t, spacecraft_index)
    _has_tick(update_guidance!, model, u, p, t, spacecraft_index) ||
        throw(MethodError(update_guidance!, (model, u, p, t, spacecraft_index)))
    HookDeprecation._hook_deprecation((typeof(model), :calcGuidanceEffect!),
        _direct_tick_deprecation(typeof(model), :calcGuidanceEffect!, :update_guidance!))
    return update_guidance!(model, u, p, t, spacecraft_index)
end

function ControlHooks.calcControlEffect!(model, u, p, t, spacecraft_index)
    _has_tick(update_control!, model, u, p, t, spacecraft_index) ||
        throw(MethodError(update_control!, (model, u, p, t, spacecraft_index)))
    HookDeprecation._hook_deprecation((typeof(model), :calcControlEffect!),
        _direct_tick_deprecation(typeof(model), :calcControlEffect!, :update_control!))
    return update_control!(model, u, p, t, spacecraft_index)
end

@noinline _direct_tick_deprecation(::Type{T}, old::Symbol, new::Symbol) where {T} =
    "$(old)(model, u, p, t, spacecraft_index) is deprecated and will be removed in SpaceAGORA 0.5.0; " *
    "it was called directly on a $(nameof(T)). Call $(new)(model, u, p, t, spacecraft_index) " *
    "instead: only the name changes."
