# Spacecraft scope and inactive spacecraft, as the GNC tick loops apply them
# (GNCRuntime declarations). The loops still run over every spacecraft, so their
# threading decisions do not change; each spacecraft's step asks here whether the
# model acts on it.
using ...GNCRuntime: SpacecraftScope, AllSpacecraft, BoundSpacecraft, CoupledSpacecraft,
    spacecraft_scope, skip_inactive, release_command!

# Whether a model with this scope ticks for run position `spacecraft_index`.
@inline _ticks_for(::AllSpacecraft, spacecraft_index::Int) = true
@inline _ticks_for(scope::BoundSpacecraft, spacecraft_index::Int) = spacecraft_index in scope.indices
@inline _ticks_for(scope::CoupledSpacecraft, spacecraft_index::Int) = spacecraft_index == scope.primary

# The run positions a scope names (none for every spacecraft).
_scope_indices(::AllSpacecraft) = ()
_scope_indices(scope::BoundSpacecraft) = scope.indices
_scope_indices(scope::CoupledSpacecraft) = (scope.primary, scope.partners...)

# One spacecraft's step of a GNC tick loop. A model whose scope leaves spacecraft
# `sat_idx` out is not called, and the step returns `false`. Otherwise it returns
# `true` after the tick, or after `release_command!` in its place when the model
# declares `skip_inactive` and the spacecraft is inactive.
@inline function _gnc_tick!(tick!::F, model, u, p, t, sat_idx::Int) where {F}
    _ticks_for(spacecraft_scope(model), sat_idx) || return false
    if skip_inactive(model) && !p.is_active[sat_idx]
        release_command!(model, t, sat_idx)
    else
        tick!(model, u, p, t, sat_idx)
    end
    return true
end

"""
    check_spacecraft_scopes(args) -> nothing

Before a run: refuse a guidance, navigation or control model whose
`spacecraft_scope` is not a `SpacecraftScope` or whose `skip_inactive` is not a
`Bool`, and warn when a scope names a run position outside `1:n` for the run's
`n` spacecraft, since the model never acts on it. The warning is the shared
pre-run warning for configuration checks (`_preflight_warning`, log group
`:preflight`), once per model type, which throws under `--depwarn=error`.
SpaceAGORA 0.4.0 will refuse such a run.
"""
function check_spacecraft_scopes(args)
    warn = _simulation_model_module.HookDeprecation._preflight_warning
    n = length(args.dynamics_model.spacecraft)
    models = (args.guidance_model.guidance_effectors..., args.navigation_model.navigation_effectors...,
              args.control_model.control_effectors...)
    for model in models
        T = typeof(model)
        scope = spacecraft_scope(model)
        scope isa SpacecraftScope || throw(ArgumentError(
            "spacecraft_scope($(nameof(T))) must return AllSpacecraft(), BoundSpacecraft(indices) or " *
            "CoupledSpacecraft(primary, partners), not $(repr(scope))."))
        inactive = skip_inactive(model)
        inactive isa Bool || throw(ArgumentError(
            "skip_inactive($(nameof(T))) must return true or false, not $(repr(inactive))."))
        outside = [i for i in _scope_indices(scope) if !(1 <= i <= n)]
        isempty(outside) || warn((T, :spacecraft_scope),
            "$(nameof(T)) names spacecraft $(join(outside, ", ")) in its spacecraft_scope, outside the run's " *
            "positions 1 to $(n), so the model never acts there. Set its spacecraft indices to run positions; " *
            "SpaceAGORA 0.4.0 will refuse this run.")
    end
    return nothing
end
