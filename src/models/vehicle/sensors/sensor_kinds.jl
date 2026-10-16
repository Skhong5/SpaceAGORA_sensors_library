# The built-in sensor kinds: rate gyro, star tracker, magnetometer. Included by
# `Components` (src/models/vehicle/spacecraft/components.jl), which owns the
# records; the contract they implement is the `Sensors` module (sensors.jl).

const _ZERO_V3 = SVector{3, Float64}(0.0, 0.0, 0.0)
const _NAN_V3 = SVector{3, Float64}(NaN, NaN, NaN)

@inline _gaussian3(rng) = SVector{3, Float64}(randn(rng), randn(rng), randn(rng))

"""
    RateGyro(; period_s=0.1, angle_random_walk=0.0, rate_random_walk=0.0, initial_bias=zero, seed=1)

A three-axis rate gyro: it measures the body rate in its link's frame, plus a
bias and white noise. `angle_random_walk` (rad/√s) sets the white noise,
`angle_random_walk / √period_s` per axis and sample; the bias starts at
`initial_bias` (rad/s) and walks with `rate_random_walk` (rad/s/√s). `seed`
selects its random stream. Zero noise measures the rate exactly.
"""
mutable struct RateGyro <: AbstractSensor
    period_s::Float64
    angle_random_walk::Float64
    rate_random_walk::Float64
    initial_bias::SVector{3, Float64}
    seed::Int
    # Run state: the current bias, the latest measurement and its time, the random stream.
    bias::SVector{3, Float64}
    latest::SVector{3, Float64}
    latest_time_s::Float64
    rng::Any
end

function RateGyro(; period_s::Real=0.1, angle_random_walk::Real=0.0, rate_random_walk::Real=0.0,
                  initial_bias=_ZERO_V3, seed::Integer=1)
    (isfinite(period_s) && period_s > 0.0) || throw(ArgumentError("RateGyro period_s must be finite and positive, got $(period_s)."))
    (isfinite(angle_random_walk) && angle_random_walk >= 0.0) || throw(ArgumentError("RateGyro angle_random_walk must be finite and nonnegative."))
    (isfinite(rate_random_walk) && rate_random_walk >= 0.0) || throw(ArgumentError("RateGyro rate_random_walk must be finite and nonnegative."))
    b = SVector{3, Float64}(initial_bias)
    return RateGyro(Float64(period_s), Float64(angle_random_walk), Float64(rate_random_walk), b, Int(seed), b, _NAN_V3, NaN, nothing)
end

sensor_dimension(::RateGyro) = 3

function sensor_measure(g::RateGyro, truth::SensorTruth, ctx::ActuatorContext, rng)
    rate = ctx.link_to_body' * truth.ω_body
    noise = g.angle_random_walk > 0.0 ? (g.angle_random_walk / sqrt(g.period_s)) * _gaussian3(rng) : _ZERO_V3
    measurement = rate + g.bias + noise
    if g.rate_random_walk > 0.0
        g.bias = g.bias + (g.rate_random_walk * sqrt(g.period_s)) * _gaussian3(rng)
    end
    return measurement
end

"""
    StarTracker(; period_s=1.0, cross_boresight_noise_rad=0.0, about_boresight_noise_rad=0.0, seed=1)

A star tracker whose boresight is its link's +z axis: it measures the
attitude quaternion (inertial to body, scalar last), perturbed by a small
rotation with `cross_boresight_noise_rad` about the two axes across the
boresight and `about_boresight_noise_rad` about it (one standard deviation,
rad). Zero noise measures the attitude exactly.
"""
mutable struct StarTracker <: AbstractSensor
    period_s::Float64
    cross_boresight_noise_rad::Float64
    about_boresight_noise_rad::Float64
    seed::Int
    latest::SVector{4, Float64}
    latest_time_s::Float64
    rng::Any
end

function StarTracker(; period_s::Real=1.0, cross_boresight_noise_rad::Real=0.0, about_boresight_noise_rad::Real=0.0, seed::Integer=1)
    (isfinite(period_s) && period_s > 0.0) || throw(ArgumentError("StarTracker period_s must be finite and positive, got $(period_s)."))
    (isfinite(cross_boresight_noise_rad) && cross_boresight_noise_rad >= 0.0) || throw(ArgumentError("StarTracker cross_boresight_noise_rad must be finite and nonnegative."))
    (isfinite(about_boresight_noise_rad) && about_boresight_noise_rad >= 0.0) || throw(ArgumentError("StarTracker about_boresight_noise_rad must be finite and nonnegative."))
    return StarTracker(Float64(period_s), Float64(cross_boresight_noise_rad), Float64(about_boresight_noise_rad), Int(seed),
                       SVector{4, Float64}(NaN, NaN, NaN, NaN), NaN, nothing)
end

sensor_dimension(::StarTracker) = 4

# The rotation matrix of the rotation vector θ (Rodrigues).
@inline function _rotation_matrix(θ::SVector{3, Float64})::SMatrix{3, 3, Float64, 9}
    a = norm(θ)
    a < 1e-300 && return SMatrix{3, 3, Float64, 9}(1, 0, 0, 0, 1, 0, 0, 0, 1)
    k = θ / a
    K = SMatrix{3, 3, Float64, 9}(0.0, k[3], -k[2], -k[3], 0.0, k[1], k[2], -k[1], 0.0)
    return SMatrix{3, 3, Float64, 9}(1, 0, 0, 0, 1, 0, 0, 0, 1) + sin(a) * K + (1 - cos(a)) * (K * K)
end

function sensor_measure(st::StarTracker, truth::SensorTruth, ctx::ActuatorContext, rng)
    q = truth.q_ib / norm(truth.q_ib)
    (st.cross_boresight_noise_rad == 0.0 && st.about_boresight_noise_rad == 0.0) && return q
    to_body = ctx.link_to_body
    n = _gaussian3(rng)
    # A small rotation of the body frame: across the boresight (link x, y) and about it (link z).
    θ = to_body * SVector{3, Float64}(st.cross_boresight_noise_rad * n[1], st.cross_boresight_noise_rad * n[2],
                                      st.about_boresight_noise_rad * n[3])
    measured = _rotation_matrix(θ)' * rot(q)        # rot(q) maps inertial to body
    out = dcm_to_quaternion(SMatrix{3, 3, Float64, 9}(measured))
    return dot(out, q) < 0.0 ? -out : out           # the same hemisphere as the truth
end

"""
    Magnetometer(; period_s=0.1, noise_t=0.0, bias_t=zero, seed=1)

A three-axis magnetometer: it measures the local magnetic field in its link's
frame (T), from the run's torque-rod field model (dipole or IGRF, Earth
only), plus a constant `bias_t` and white noise `noise_t` per axis and sample.
"""
mutable struct Magnetometer <: AbstractSensor
    period_s::Float64
    noise_t::Float64
    bias_t::SVector{3, Float64}
    seed::Int
    latest::SVector{3, Float64}
    latest_time_s::Float64
    rng::Any
end

function Magnetometer(; period_s::Real=0.1, noise_t::Real=0.0, bias_t=_ZERO_V3, seed::Integer=1)
    (isfinite(period_s) && period_s > 0.0) || throw(ArgumentError("Magnetometer period_s must be finite and positive, got $(period_s)."))
    (isfinite(noise_t) && noise_t >= 0.0) || throw(ArgumentError("Magnetometer noise_t must be finite and nonnegative."))
    return Magnetometer(Float64(period_s), Float64(noise_t), SVector{3, Float64}(bias_t), Int(seed), _NAN_V3, NaN, nothing)
end

sensor_dimension(::Magnetometer) = 3
sensor_requirements(::Magnetometer) = ActuatorRequirements(magnetic_field=true)

function sensor_measure(m::Magnetometer, truth::SensorTruth, ctx::ActuatorContext, rng)
    field = ctx.link_to_body' * truth.magnetic_field_body
    noise = m.noise_t > 0.0 ? m.noise_t * _gaussian3(rng) : _ZERO_V3
    return field + m.bias_t + noise
end
