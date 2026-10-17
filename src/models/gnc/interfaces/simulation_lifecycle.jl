# Internal lifecycle extension points. Legacy guidance retains no-op defaults.
# The bind_spacecraft fallback names the effector's scope, a GNCRuntime declaration.
isdefined(@__MODULE__, :GNCRuntime) || Base.include(@__MODULE__, joinpath(@__DIR__, "gnc_runtime.jl"))
module SimulationLifecycle
using ..GNCRuntime: spacecraft_scope, BoundSpacecraft, CoupledSpacecraft
preflight_guidance(model, args; isolate_state) = nothing
# Run-start initialization: before the solve, the engine calls the navigation,
# then the guidance, then the control methods for every configured effector.
initialize_navigation!(model, u, p, t) = nothing
initialize_guidance!(model, u, p, t) = nothing
preflight_control(model, args; isolate_state) = nothing
initialize_control!(model, u, p, t) = nothing
# Called after the engine reconciles the full atmosphere mask, once per changed
# spacecraft. Model methods own their state; legacy models remain inert.
requires_atmosphere_events(model) = false
atmosphere_transition!(model, u, p, t, i, inside) = nothing
before_reference_control!(guidance, control, u, p, t) = nothing

"""
    bind_spacecraft(effector, sat_idx::Int)

Extension point for per-spacecraft GNC: return `effector` bound to spacecraft
`sat_idx` (a run index, 1-based). A `SpacecraftModel` can declare guidance,
navigation and control effectors; at the start of `run_simulation` each one is
passed through this function with the position of its spacecraft, and the result
joins the configuration-level GNC tuples.

Implement it for an effector that selects its vehicle by an index field: return a
shallow copy (sharing mutable members with the original) with that field set, or
`effector` itself when it already targets `sat_idx`. A member that holds one
spacecraft's run state (a held command, a controller's warm start) needs its own
copy, so spacecraft bound from one template do not share it; members indexed by
spacecraft, such as an `ApolloDescentState`, stay shared. The fallback throws an
`ArgumentError` that names the effector's `spacecraft_scope`: an effector that acts
on every spacecraft (`AllSpacecraft`) or couples several vehicles
(`CoupledSpacecraft`, for example the RPO chaser and target types) cannot belong
to one; declare it at configuration level instead.
"""
function bind_spacecraft(effector, sat_idx::Int)
    T = typeof(effector)
    scope = spacecraft_scope(effector)
    why = scope isa CoupledSpacecraft ?
        "it couples several spacecraft (its spacecraft_scope is CoupledSpacecraft)" :
        scope isa BoundSpacecraft ?
        "it is bound to spacecraft $(join(scope.indices, ", ")) (its spacecraft_scope) and has no method to bind another" :
        "it acts on every spacecraft (its spacecraft_scope is AllSpacecraft)"
    throw(ArgumentError(
        "Cannot declare $(nameof(T)) on a spacecraft: $why. Declare it at configuration level " *
        "(guidance_model/navigation_model/control_model), or implement SpaceAGORA.bind_spacecraft(::$(nameof(T)), ::Int)."))
end

# Shallow copy of a plain struct with all fields set by position (kwdef types included).
_shallow_copy(x::T) where {T} = T((getfield(x, i) for i in 1:fieldcount(T))...)
end
