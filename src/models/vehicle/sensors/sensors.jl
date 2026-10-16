"""
    Sensors

The sensor contract: a sensor kind is a mutable record subtyping
`AbstractSensor` with methods of `sensor_measure` and `sensor_dimension`;
mounted on a link with `add_sensor!`, it is sampled at its period from the
accepted state, and navigation effectors read its latest measurement with
`sensor_measurement`.
"""
module Sensors

using StaticArrays
using ..AbstractTypes: AbstractSensor
using ..Actuators: ActuatorContext, ActuatorRequirements

export SensorTruth, sensor_measure, sensor_dimension, sensor_requirements, sensor_period_s

"""
    SensorTruth

What a sensor measures from, at a sample: the inertial position and velocity
(m, m/s), the attitude quaternion (inertial to body, scalar last), the body
rate (rad/s), the mass (kg), the time (s into the run), and the local
magnetic field in the body frame (T; NaN unless a mounted sensor asks for
it). Without attitude propagation the attitude is the identity and the rate
zero.
"""
struct SensorTruth
    pos_ii::SVector{3, Float64}
    vel_ii::SVector{3, Float64}
    q_ib::SVector{4, Float64}
    ω_body::SVector{3, Float64}
    mass_kg::Float64
    t_s::Float64
    magnetic_field_body::SVector{3, Float64}
end

"""
    sensor_measure(sensor, truth::SensorTruth, ctx::ActuatorContext, rng) -> measurement

Required: one measurement (an `SVector` or a number) of `truth` by `sensor`,
mounted where `ctx` says (`ctx.link_to_body` maps the sensor's frame, its
link's, to the body frame). Draw any noise from `rng`, the sensor's own
random stream. Called once per sample, so a sensor may advance its own state
(a bias random walk) here.
"""
function sensor_measure end

"""
    sensor_dimension(sensor) -> Int

Required: the number of components of the sensor's measurement, which sets
its result columns.
"""
function sensor_dimension end

"""
    sensor_requirements(sensor) -> ActuatorRequirements

Optional: what the sensor needs sampled. Sensors may ask for the magnetic
field; other environment requests are refused at the start of a run.
Default: nothing.
"""
@inline sensor_requirements(::Any) = ActuatorRequirements()

"""
    sensor_period_s(sensor) -> Float64

Optional: the sampling period, s. Default: the sensor's `period_s` field.

Besides its parameters, a sensor kind holds its run state in four fields the
engine fills: `seed` (an integer the user sets, which selects the random
stream), `rng` (the stream), `latest` (the latest measurement: a vector of
`sensor_dimension` components, or a number for a one-component sensor; NaN
before the first sample) and `latest_time_s` (its time, s into the run). The
period must be finite and positive.
"""
@inline sensor_period_s(sensor) = Float64(sensor.period_s)

end # module Sensors
