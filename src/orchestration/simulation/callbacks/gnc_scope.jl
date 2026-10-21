# Spacecraft scope and inactive spacecraft, as the GNC tick loops apply them
# (GNCRuntime declarations). A periodic tick reads the model's scope once and
# visits only the run positions it names (`_tick_positions`), in ascending order,
# so a tick costs the size of the scope rather than the number of spacecraft.
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

# The run positions, among `n` spacecraft, that a model with this scope ticks for:
# the positions `_ticks_for` accepts, in ascending order and each once. That is
# every position, the `BoundSpacecraft` indices within `1:n`, or the
# `CoupledSpacecraft` primary. Indices already strictly increasing and within the
# run are returned as they are; any others are sorted, deduplicated and filtered
# into a new vector.
@inline _tick_positions(::AllSpacecraft, n::Int) = Base.OneTo(n)
@inline function _tick_positions(scope::BoundSpacecraft, n::Int)
    _increasing_within(scope.indices, n) && return scope.indices
    return sort!(unique!(Int[i for i in scope.indices if 1 <= i <= n]))
end
@inline _tick_positions(scope::CoupledSpacecraft, n::Int) = 1 <= scope.primary <= n ? (scope.primary,) : ()

@inline function _increasing_within(indices::Tuple, n::Int)
    previous = 0
    for i in indices
        previous < i <= n || return false
        previous = i
    end
    return true
end

# One spacecraft's step of a GNC tick loop, for a spacecraft in the model's
# scope: the tick, or `release_command!` in its place when the model declares
# `skip_inactive` and the spacecraft is inactive.
@inline function _gnc_tick_in_scope!(tick!::F, model, u, p, t, sat_idx::Int) where {F}
    if skip_inactive(model) && !p.is_active[sat_idx]
        release_command!(model, t, sat_idx)
    else
        tick!(model, u, p, t, sat_idx)
    end
    return nothing
end

# One spacecraft's step for any spacecraft, as the callbacks that visit one
# spacecraft at a time take it. A model whose scope leaves spacecraft `sat_idx`
# out is not called, and the step returns `false`. Otherwise it returns `true`
# after the step above.
@inline function _gnc_tick!(tick!::F, model, u, p, t, sat_idx::Int) where {F}
    _ticks_for(spacecraft_scope(model), sat_idx) || return false
    _gnc_tick_in_scope!(tick!, model, u, p, t, sat_idx)
    return true
end

# A periodic tick of `model` for every spacecraft in its scope, in ascending order.
@inline function _gnc_tick_scope!(tick!::F, model, u, p, t, num_sats::Int) where {F}
    @inbounds for sat_idx in _tick_positions(spacecraft_scope(model), num_sats)
        _gnc_tick_in_scope!(tick!, model, u, p, t, sat_idx)
    end
    return nothing
end

# Whether `model`'s command for spacecraft `sat_idx` is released at this time, as
# `_gnc_tick!` releases it: the spacecraft is in the model's scope and inactive, and
# the model declares `skip_inactive`. A path that writes the command outside the
# tick (the burn scheduler's scheduling and burn edges) releases it again after its
# bookkeeping, so nothing undoes a release.
@inline _released(model, p, sat_idx::Int) =
    _ticks_for(spacecraft_scope(model), sat_idx) && skip_inactive(model) && !p.is_active[sat_idx]

# Whether a tick of `model` at this time runs its tick for some spacecraft, rather
# than skipping or releasing every one: the condition for preparing it first
# (`before_reference_control!` for its linked producers).
function _ticks_any(model, p, num_sats::Int)
    scope, skip = spacecraft_scope(model), skip_inactive(model)
    for sat_idx in _tick_positions(scope, num_sats)
        !(skip && !p.is_active[sat_idx]) && return true
    end
    return false
end

"""
    check_spacecraft_scopes(args) -> nothing

Before a run: refuse it with an `ArgumentError` when a guidance, navigation or
control model's `spacecraft_scope` is not a `SpacecraftScope`, its
`skip_inactive` is not a `Bool`, or its scope names a run position outside `1:n`
for the run's `n` spacecraft, where the model would never act. (SpaceAGORA 0.3
warned about such a position instead.)
"""
function check_spacecraft_scopes(args)
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
        isempty(outside) || throw(ArgumentError(
            "$(nameof(T)) names spacecraft $(join(outside, ", ")) in its spacecraft_scope, outside the run's " *
            "positions 1 to $(n), so the model would never act there. Set its spacecraft indices to run positions."))
    end
    return nothing
end
