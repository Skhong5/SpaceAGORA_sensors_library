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

"""
    GNSSReceiver(; period_s=1.0, position_noise_m=0.0, velocity_noise_mps=0.0, seed=1)

A GNSS receiver: it measures the position and velocity in the run's inertial
frame (m, m/s; six components, position first), with white noise
`position_noise_m` and `velocity_noise_mps` per axis and sample. The antenna
is taken at the centre of mass; there are no outages.
"""
mutable struct GNSSReceiver <: AbstractSensor
    period_s::Float64
    position_noise_m::Float64
    velocity_noise_mps::Float64
    seed::Int
    latest::SVector{6, Float64}
    latest_time_s::Float64
    rng::Any
end

function GNSSReceiver(; period_s::Real=1.0, position_noise_m::Real=0.0, velocity_noise_mps::Real=0.0, seed::Integer=1)
    (isfinite(period_s) && period_s > 0.0) || throw(ArgumentError("GNSSReceiver period_s must be finite and positive, got $(period_s)."))
    (isfinite(position_noise_m) && position_noise_m >= 0.0) || throw(ArgumentError("GNSSReceiver position_noise_m must be finite and nonnegative."))
    (isfinite(velocity_noise_mps) && velocity_noise_mps >= 0.0) || throw(ArgumentError("GNSSReceiver velocity_noise_mps must be finite and nonnegative."))
    return GNSSReceiver(Float64(period_s), Float64(position_noise_m), Float64(velocity_noise_mps), Int(seed),
                        SVector{6, Float64}(ntuple(_ -> NaN, 6)), NaN, nothing)
end

sensor_dimension(::GNSSReceiver) = 6

function sensor_measure(g::GNSSReceiver, truth::SensorTruth, ctx::ActuatorContext, rng)
    pos = g.position_noise_m > 0.0 ? truth.pos_ii + g.position_noise_m * _gaussian3(rng) : truth.pos_ii
    vel = g.velocity_noise_mps > 0.0 ? truth.vel_ii + g.velocity_noise_mps * _gaussian3(rng) : truth.vel_ii
    return SVector{6, Float64}(pos[1], pos[2], pos[3], vel[1], vel[2], vel[3])
end

"""
    SunSensor(; period_s=0.1, noise_rad=0.0, field_of_view_rad=π/2, min_lit_fraction=0.5, seed=1)

A sun sensor whose boresight is its link's +z axis: it measures the direction
to the Sun as a unit vector in its link's frame, turned by a small rotation
with `noise_rad` (one standard deviation) about each axis. It reports no
direction (NaN) when the Sun is more than `field_of_view_rad` from the
boresight or when less than `min_lit_fraction` of the Sun's disk is lit
(eclipse). The Sun's direction comes from the run's ephemerides, the lit
fraction from the eclipse model of solar radiation pressure.
"""
mutable struct SunSensor <: AbstractSensor
    period_s::Float64
    noise_rad::Float64
    field_of_view_rad::Float64
    min_lit_fraction::Float64
    seed::Int
    latest::SVector{3, Float64}
    latest_time_s::Float64
    rng::Any
end

function SunSensor(; period_s::Real=0.1, noise_rad::Real=0.0, field_of_view_rad::Real=π / 2, min_lit_fraction::Real=0.5, seed::Integer=1)
    (isfinite(period_s) && period_s > 0.0) || throw(ArgumentError("SunSensor period_s must be finite and positive, got $(period_s)."))
    (isfinite(noise_rad) && noise_rad >= 0.0) || throw(ArgumentError("SunSensor noise_rad must be finite and nonnegative."))
    (0.0 < field_of_view_rad <= π) || throw(ArgumentError("SunSensor field_of_view_rad must be in (0, π], got $(field_of_view_rad)."))
    (0.0 <= min_lit_fraction <= 1.0) || throw(ArgumentError("SunSensor min_lit_fraction must be in [0, 1], got $(min_lit_fraction)."))
    return SunSensor(Float64(period_s), Float64(noise_rad), Float64(field_of_view_rad), Float64(min_lit_fraction), Int(seed), _NAN_V3, NaN, nothing)
end

sensor_dimension(::SunSensor) = 3
sensor_requirements(::SunSensor) = ActuatorRequirements(solar=true)

function sensor_measure(s::SunSensor, truth::SensorTruth, ctx::ActuatorContext, rng)
    d = ctx.link_to_body' * truth.sun_direction_body
    (all(isfinite, d) && truth.sun_lit_fraction >= s.min_lit_fraction) || return _NAN_V3
    acos(clamp(d[3], -1.0, 1.0)) <= s.field_of_view_rad || return _NAN_V3
    s.noise_rad > 0.0 || return d
    turned = _rotation_matrix(s.noise_rad * _gaussian3(rng)) * d
    return turned / norm(turned)
end

"""
    Accelerometer(; period_s=0.1, noise_mps2=0.0, bias_mps2=zero, seed=1)

A three-axis accelerometer at the centre of mass: it measures the specific
force, the vehicle's acceleration less gravity (m/s²), in its link's frame,
plus a constant `bias_mps2` and white noise `noise_mps2` per axis and sample.
Gravity is the force of the run's effectors for which `is_gravitational` is
true, so in free fall it reads zero and under thrust, drag or radiation
pressure it reads their acceleration.
"""
mutable struct Accelerometer <: AbstractSensor
    period_s::Float64
    noise_mps2::Float64
    bias_mps2::SVector{3, Float64}
    seed::Int
    latest::SVector{3, Float64}
    latest_time_s::Float64
    rng::Any
end

function Accelerometer(; period_s::Real=0.1, noise_mps2::Real=0.0, bias_mps2=_ZERO_V3, seed::Integer=1)
    (isfinite(period_s) && period_s > 0.0) || throw(ArgumentError("Accelerometer period_s must be finite and positive, got $(period_s)."))
    (isfinite(noise_mps2) && noise_mps2 >= 0.0) || throw(ArgumentError("Accelerometer noise_mps2 must be finite and nonnegative."))
    return Accelerometer(Float64(period_s), Float64(noise_mps2), SVector{3, Float64}(bias_mps2), Int(seed), _NAN_V3, NaN, nothing)
end

sensor_dimension(::Accelerometer) = 3
sensor_needs_specific_force(::Accelerometer) = true

function sensor_measure(a::Accelerometer, truth::SensorTruth, ctx::ActuatorContext, rng)
    f = ctx.link_to_body' * truth.specific_force_body + a.bias_mps2
    return a.noise_mps2 > 0.0 ? f + a.noise_mps2 * _gaussian3(rng) : f
end
