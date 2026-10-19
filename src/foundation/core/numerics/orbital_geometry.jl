module OrbitalGeometry
using StaticArrays

# Inertial axes to perifocal axes for radian orbital angles. Keep the original
# component expressions and their evaluation order; callers own vector scaling
# and any direction normalization. This is an internal, non-exported owner.
@inline function _perifocal_from_inertial(i::Float64, ω::Float64, Ω::Float64)::SMatrix{3, 3, Float64, 9}
    return @SMatrix [
        -sin(Ω)*cos(i)*sin(ω)+cos(Ω)*cos(ω)  cos(Ω)*cos(i)*sin(ω)+sin(Ω)*cos(ω)  sin(i)*sin(ω)
        -sin(Ω)*cos(i)*cos(ω)-cos(Ω)*sin(ω)  cos(Ω)*cos(i)*cos(ω)-sin(Ω)*sin(ω)  sin(i)*cos(ω)
         sin(Ω)*sin(i)                       -cos(Ω)*sin(i)                       cos(i)
    ]
end

# The true anomaly (rad, in [0, 2π)) at mean anomaly `M` (rad) on an ellipse of
# eccentricity 0 <= e < 1: Kepler's equation M = E - e sin E by Newton's method,
# started at M (or at π for a very eccentric orbit, where M is a poor start).
function _true_anomaly_from_mean(M::Float64, e::Float64)::Float64
    0.0 <= e < 1.0 || throw(ArgumentError("The mean anomaly needs an elliptic orbit, 0 <= e < 1; got e = $e."))
    M = mod(M, 2pi)
    E = e < 0.8 ? M : Float64(pi)
    for _ in 1:50
        step = (E - e * sin(E) - M) / (1.0 - e * cos(E))
        E -= step
        abs(step) <= 1e-14 && break   # Newton is quadratic: E is then exact to rounding
    end
    return mod(2.0 * atan(sqrt(1.0 + e) * sin(E / 2.0), sqrt(1.0 - e) * cos(E / 2.0)), 2pi)
end
end
