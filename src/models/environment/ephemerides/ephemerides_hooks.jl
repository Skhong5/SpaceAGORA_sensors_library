# The ephemerides hooks a model implements, model first, and the deprecated
# model-last forms they replace.

"""
    ephemerides_frame_lpi(model, planet, et) -> SMatrix{3, 3, Float64}

The rotation from the inertial (J2000) frame to `planet`'s planet-fixed frame at
ephemeris time `et`, so that `r_pf = L * r_ii`. The engine calls it wherever it
needs the planet-fixed frame, for example for the atmosphere, gravity-harmonics
and magnetic-field samples, guidance and the scene export. It returns the
rotation only; a caller that needs planet-relative velocity adds the transport
term with the planet's spin vector.

`et` is in seconds past the J2000 epoch on the model's own timeline: the run's
start, [`ephemerides_epoch_seconds`](@ref), plus the elapsed time.
`SpiceEphemeridesModel` asks SPICE for the transform; `SimpleEphemeridesModel`
turns the planet about its spin axis, by default Earth by its Greenwich mean
sidereal time.
"""
function ephemerides_frame_lpi end

"""
    ephemerides_sun_ii(model, planet, et) -> Union{Nothing, SVector{3, Float64}}

Unit vector from the planet's center to the Sun, in the inertial (J2000) frame
the integrated state uses, or `nothing` when this ephemeris backend cannot
resolve the Sun for this body. `et` is as for
[`ephemerides_frame_lpi`](@ref). The power callbacks and the saved Sun
direction call it.

`SpiceEphemeridesModel` asks SPICE for the Sun's position relative to the
planet, so it works for every body whose SPK is furnished. `SimpleEphemeridesModel`
carries no planetary ephemeris, so it answers only for Earth, from a
low-precision analytic Sun; it returns `nothing` for any other body rather than
inventing a direction.
"""
function ephemerides_sun_ii end

"""
    ephemerides_epoch_seconds(model, initial_time) -> Float64

The run's start epoch, `initial_time` (an `InitialTime`, UTC), in seconds past
the J2000 epoch on the model's own timeline. The engine calls it before the run
and adds the elapsed time to it for each ephemeris time `et` it passes to
[`ephemerides_frame_lpi`](@ref) and [`ephemerides_sun_ii`](@ref).

`SpiceEphemeridesModel` returns SPICE's ephemeris time (TDB);
`SimpleEphemeridesModel` returns leap-second-free UTC seconds past J2000 plus
its `reference_epoch_seconds`.
"""
function ephemerides_epoch_seconds end

"""
    ephemerides_requires_spice(model) -> Bool

Whether the model reads SPICE kernels. When it does, the planet-fixed position
and velocity transforms ask SPICE for the full state transform rather than
[`ephemerides_frame_lpi`](@ref), and the run counts the model's SPICE frame
calls.
"""
function ephemerides_requires_spice end

"""
    ephemerides_cache_key(model) -> Tuple

A key for the model's timeline and frames: the engine may reuse an ephemeris
cache built for one run in a later run whose model has an equal key. Two models
with equal keys must give the same frames and times.
"""
function ephemerides_cache_key end

# The hooks, as the engine calls them: the model-first name when the model's
# type implements it for these argument types, and otherwise the deprecated
# model-last name. `hasmethod` on the exact argument types is the lookup the call
# makes, and it folds in the bridge's specialization on the model's type.
@inline function _ephemerides_frame_lpi(model, planet, et)
    hasmethod(ephemerides_frame_lpi, Tuple{typeof(model), typeof(planet), typeof(et)}) &&
        return ephemerides_frame_lpi(model, planet, et)
    return planet_frame_lpi(planet, et, model)
end

@inline function _ephemerides_sun_ii(model, planet, et)
    hasmethod(ephemerides_sun_ii, Tuple{typeof(model), typeof(planet), typeof(et)}) &&
        return ephemerides_sun_ii(model, planet, et)
    return ephemerides_sun_direction_ii(planet, et, model)
end

@inline function _ephemerides_epoch_seconds(model, initial_time)
    hasmethod(ephemerides_epoch_seconds, Tuple{typeof(model), typeof(initial_time)}) &&
        return ephemerides_epoch_seconds(model, initial_time)
    return ephemerides_time_seconds(initial_time, model)
end

"""
    planet_frame_lpi(planet, et, model) -> SMatrix{3, 3, Float64}

Deprecated, use [`ephemerides_frame_lpi`](@ref)`(model, planet, et)`: the same
rotation with the model first. A model that implements only this name keeps
working, with a warning before the run. This name will be removed in SpaceAGORA
0.5.0.
"""
function planet_frame_lpi end

"""
    ephemerides_sun_direction_ii(planet, et, model) -> Union{Nothing, SVector{3, Float64}}

Deprecated, use [`ephemerides_sun_ii`](@ref)`(model, planet, et)`: the same
direction with the model first. A model that implements only this name keeps
working, with a warning before the run. This name will be removed in SpaceAGORA
0.5.0.
"""
function ephemerides_sun_direction_ii end

"""
    ephemerides_time_seconds(initial_time, model) -> Float64

Deprecated, use [`ephemerides_epoch_seconds`](@ref)`(model, initial_time)`: the
same epoch with the model first. A model that implements only this name keeps
working, with a warning before the run. This name will be removed in SpaceAGORA
0.5.0.
"""
function ephemerides_time_seconds end

# Direct calls of the deprecated names. A model type's own method of an old name
# takes precedence over these untyped fallbacks; they serve every other type. On
# a type that implements the model-first hook for these arguments, the call warns
# once per type that the old name is deprecated (under --depwarn=error it throws)
# and returns the new hook's value. Otherwise it raises a MethodError naming the
# new hook. The bridges call these only for types without the new hook, so no
# warning is reachable from the engine.
function planet_frame_lpi(planet, et, model)
    hasmethod(ephemerides_frame_lpi, Tuple{typeof(model), typeof(planet), typeof(et)}) ||
        throw(MethodError(ephemerides_frame_lpi, (model, planet, et)))
    _hook_deprecation((typeof(model), :planet_frame_lpi),
        _direct_ephemerides_deprecation(typeof(model), "planet_frame_lpi(planet, et, model)",
            "ephemerides_frame_lpi(model, planet, et)"))
    return ephemerides_frame_lpi(model, planet, et)
end

function ephemerides_sun_direction_ii(planet, et, model)
    hasmethod(ephemerides_sun_ii, Tuple{typeof(model), typeof(planet), typeof(et)}) ||
        throw(MethodError(ephemerides_sun_ii, (model, planet, et)))
    _hook_deprecation((typeof(model), :ephemerides_sun_direction_ii),
        _direct_ephemerides_deprecation(typeof(model), "ephemerides_sun_direction_ii(planet, et, model)",
            "ephemerides_sun_ii(model, planet, et)"))
    return ephemerides_sun_ii(model, planet, et)
end

function ephemerides_time_seconds(initial_time, model)
    hasmethod(ephemerides_epoch_seconds, Tuple{typeof(model), typeof(initial_time)}) ||
        throw(MethodError(ephemerides_epoch_seconds, (model, initial_time)))
    _hook_deprecation((typeof(model), :ephemerides_time_seconds),
        _direct_ephemerides_deprecation(typeof(model), "ephemerides_time_seconds(initial_time, model)",
            "ephemerides_epoch_seconds(model, initial_time)"))
    return ephemerides_epoch_seconds(model, initial_time)
end

@noinline _direct_ephemerides_deprecation(::Type{T}, old::String, new::String) where {T} =
    "$(old) is deprecated and will be removed in SpaceAGORA 0.5.0; it was called directly on a " *
    "$(nameof(T)). Call $(new) instead: the model comes first."
