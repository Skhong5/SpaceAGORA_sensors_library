module EphemeridesModels

using Dates
using StaticArrays
using LinearAlgebra
using SPICE

using ...RuntimeServices: SPICE_LOCK, tracked_lock
using ..AbstractTypes: AbstractEphemeridesModel
using ..HookDeprecation: _hook_deprecation

export SpiceEphemeridesModel, SimpleEphemeridesModel
export ephemerides_frame_lpi, ephemerides_sun_ii, ephemerides_epoch_seconds
export ephemerides_requires_spice, ephemerides_cache_key
# Deprecated: the model-last names, removed in SpaceAGORA 0.5.0.
export planet_frame_lpi, ephemerides_sun_direction_ii, ephemerides_time_seconds

include(joinpath(@__DIR__, "ephemerides_hooks.jl"))
include(joinpath(@__DIR__, "simple_ephemerides.jl"))

end # module EphemeridesModels
