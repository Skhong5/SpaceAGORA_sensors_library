# The link contract: what one registered interlink does between its two terminals.
# Each registered link is one value of a link kind; the engine asks it whether it
# can close and what it does to its two endpoints. Included by `InterLinkModels`
# (interlink_graph.jl).

"""
    AbstractInterlink

Supertype of the link kinds. One value describes one registered link between two
terminals: register it with `register_link!`. A kind implements `link_range_m`,
and the effects it has: `link_force` for a link that pushes its endpoints (the
default exerts none). `link_available` decides when the link can close (by
default, within `link_range_m`). The built-in kind is `LaserMomentumLink`.
"""
abstract type AbstractInterlink end

"""
    LinkContext(; time_s=NaN, planet=nothing)

What a link hook may read besides its two endpoints: the time (s into the run)
and the run's planet. The engine passes the run's values; a call outside a run
gets the defaults.
"""
struct LinkContext{P}
    time_s::Float64
    planet::P
end
LinkContext(; time_s::Real=NaN, planet=nothing) = LinkContext(Float64(time_s), planet)

"""
    link_range_m(link) -> Float64

The maximum separation in metres at which `link` can close. Every link kind
implements it.
"""
function link_range_m end

"""
    link_available(link, a::StateSample, b::StateSample, ctx::LinkContext) -> Bool

Whether `link` can close between its endpoints `a` and `b`, in the order of the
link's key (the lower `(spacecraft, terminal)` pair first). The engine asks at the start of the run and after every accepted step,
once the run's own rules (forbidden pairs, active spacecraft) allow the pair.
Default: the endpoints are apart and within `link_range_m(link)`.
"""
function link_available(link, a, b, ctx)
    separation = norm(a.pos_ii - b.pos_ii)
    return 0 < separation <= link_range_m(link)
end

const _ZERO_FORCE = SVector{3, Float64}(0.0, 0.0, 0.0)

"""
    link_force(link, a::StateSample, b::StateSample, ctx::LinkContext) -> (force_a_ii, force_b_ii)

The inertial forces (N) an active `link` puts on its endpoints `a` and `b`, in
the order of the link's key. The engine adds them to the endpoints' accelerations at every
right-hand-side evaluation, and the scheduling policies score candidate links by
them. Default: no force.
"""
link_force(link, a, b, ctx) = (_ZERO_FORCE, _ZERO_FORCE)

# The state sample of a link endpoint: spacecraft `index` of a run state
# `view`, with its model when known. A run state holds every field; a velocity
# or mass a caller's partial state lacks reads NaN.
const _NAN3 = SVector{3, Float64}(NaN, NaN, NaN)
function link_endpoint(view, spacecraft, index::Integer)
    vel = hasproperty(view, :vel) ? SVector{3, Float64}(view.vel) : _NAN3
    mass = hasproperty(view, :mass) ? Float64(view.mass) : NaN
    q = hasproperty(view, :q) ? SVector{4, Float64}(view.q) : nothing
    ω = hasproperty(view, :ω) ? SVector{3, Float64}(view.ω) : nothing
    return StateSample(SVector{3, Float64}(view.pos), vel, mass, q, ω, spacecraft, Int(index))
end

# ---- The laser momentum link ---------------------------------------------------

# The speed of light, the one the laser terminal's recoil uses.
const SPEED_OF_LIGHT_MPS = _SPEED_OF_LIGHT_MPS

"""
    LaserMomentumLink(; power_w=10_000.0, magnification=100.0, range_m=200e3)
    LaserMomentumLink(terminal::LaserTerminal)

An open-cavity laser link that pushes its two spacecraft apart: an idealized
central force of `magnification * power_w / c` on each endpoint, directed away
from the other, equal and opposite, at any separation up to `range_m`. It models
no pointing, occultation or beam losses. `power_w` (W) must be nonnegative,
`magnification` and `range_m` (m) positive. The terminal form takes a link
between two terminals of that design: its `max_power_w`, `magnification` and
`range_m`.
"""
struct LaserMomentumLink <: AbstractInterlink
    power_w::Float64
    magnification::Float64
    range_m::Float64

    function LaserMomentumLink(power_w::Real, magnification::Real, range_m::Real)
        isfinite(power_w) && power_w >= 0 || throw(ArgumentError("power_w must be finite and nonnegative."))
        isfinite(magnification) && magnification > 0 || throw(ArgumentError("magnification must be finite and positive."))
        isfinite(range_m) && range_m > 0 || throw(ArgumentError("range_m must be finite and positive."))
        new(Float64(power_w), Float64(magnification), Float64(range_m))
    end
end

function LaserMomentumLink(; power_w=_NOT_PASSED, magnification=_NOT_PASSED, range_m=_NOT_PASSED,
                           P=_NOT_PASSED, B=_NOT_PASSED, range=_NOT_PASSED)
    power = _laser_keyword(:P, P, :power_w, power_w, 10_000.0)
    gain = _laser_keyword(:B, B, :magnification, magnification, 100.0)
    reach = _laser_keyword(:range, range, :range_m, range_m, 200e3)
    return LaserMomentumLink(power, gain, reach)
end

LaserMomentumLink(terminal::LaserTerminal) =
    LaserMomentumLink(terminal.max_power_w, terminal.magnification, terminal.range_m)

link_range_m(link::LaserMomentumLink) = link.range_m

function link_force(link::LaserMomentumLink, a, b, ctx)
    separation = a.pos_ii - b.pos_ii
    distance = norm(separation)
    distance == 0 && return (zero(separation), zero(separation))
    force = (link.magnification * link.power_w / SPEED_OF_LIGHT_MPS) * (separation / distance)
    return (force, -force)
end

# The `InterLinkParameters` names of the fields, P, B and range: deprecated.
const _LASER_OLD_FIELDS = (P=:power_w, B=:magnification, range=:range_m)

function _laser_keyword(old::Symbol, old_value, new::Symbol, new_value, default)
    old_value === _NOT_PASSED && return new_value === _NOT_PASSED ? default : new_value
    new_value === _NOT_PASSED || throw(ArgumentError(
        "LaserMomentumLink: pass $(new) only; $(old) is its deprecated name."))
    _interlink_deprecation((LaserMomentumLink, old),
        "The InterLinkParameters keyword $(old) is deprecated and will be removed in SpaceAGORA 1.0; " *
        "use LaserMomentumLink($(new)=...) instead: only the name changes.")
    return old_value
end

@inline function Base.getproperty(link::LaserMomentumLink, name::Symbol)
    if name === :P || name === :B || name === :range
        new = _LASER_OLD_FIELDS[name]
        _interlink_deprecation((LaserMomentumLink, name),
            "LaserMomentumLink.$(name) (InterLinkParameters.$(name)) is deprecated and will be removed in " *
            "SpaceAGORA 1.0; use .$(new) instead: only the name changes.")
        return getfield(link, new)
    end
    return getfield(link, name)
end

Base.propertynames(::LaserMomentumLink, private::Bool=false) = (:power_w, :magnification, :range_m)

# Deprecated interlink names warn once per process and name, and throw under
# `julia --depwarn=error`.
@noinline _interlink_deprecation(key::Tuple, msg::String) = _hook_deprecation(key, msg)
