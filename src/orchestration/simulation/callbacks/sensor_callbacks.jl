# Sensor sampling during a run (see Sensors, SensorMounting): each sensor takes
# a measurement from the accepted state at the run's start and every period
# after it, and keeps it for navigation effectors to read; the sensor columns.
# Sensors do not act on the dynamics.

# The field source of a magnetometer: the run's MagneticTorqueRodModel, or its
# default (the tilted dipole) when the run has none.
function _sensor_field_source(effectors)
    index = findfirst(e -> e isa MagneticTorqueRodModel, effectors)
    return index === nothing ? MagneticTorqueRodModel() : effectors[index]
end

# The truth a sensor of spacecraft `i` measures at the accepted state.
function _sensor_truth(integrator, i::Int, wants_field::Bool)
    p = integrator.p
    sc_view = integrator.u.sc[i]
    pos = SVector{3, Float64}(sc_view.pos[1], sc_view.pos[2], sc_view.pos[3])
    vel = SVector{3, Float64}(sc_view.vel[1], sc_view.vel[2], sc_view.vel[3])
    q = hasproperty(sc_view, :q) ? SVector{4, Float64}(sc_view.q[1], sc_view.q[2], sc_view.q[3], sc_view.q[4]) :
        SVector{4, Float64}(0.0, 0.0, 0.0, 1.0)
    ω = hasproperty(sc_view, :ω) ? SVector{3, Float64}(sc_view.ω[1], sc_view.ω[2], sc_view.ω[3]) : SVector{3, Float64}(0.0, 0.0, 0.0)
    field = SVector{3, Float64}(NaN, NaN, NaN)
    if wants_field
        # The field model reads the time from the shared buffer the right-hand
        # side keeps: set it to the sample's time, then restore it.
        clock = p.shared_buffers.current_time
        previous = clock[]
        clock[] = Float64(integrator.t)
        field = rod_field_body(_sensor_field_source(p.args.dynamics_model.dynamic_effectors), pos, q, p)
        clock[] = previous
    end
    return SensorTruth(pos, vel, q, ω, Float64(sc_view.mass), Float64(integrator.t), field)
end

function _sample_sensors!(integrator, members::Vector{Tuple{Int, Int}})
    spacecraft = integrator.p.args.dynamics_model.spacecraft
    for (i, k) in members
        entry = mounted_sensors(spacecraft[i])[k]
        s = entry.sensor
        truth = _sensor_truth(integrator, i, sensor_requirements(s).magnetic_field)
        s.latest = sensor_measure(s, truth, actuator_context(entry.link), s.rng)
        s.latest_time_s = Float64(integrator.t)
    end
    return nothing
end

"""
    get_sensor_callbacks(num_sats, args) -> Tuple

For runs with sensors: one callback per distinct sampling period, which
samples its sensors at the run's start and then at every multiple of the
period after it, each a solver stop. The sampling times follow the run's
start, not the solve's, so a checkpointed run samples at the same times as a
run in one piece. Runs with sensors cannot resume from a checkpoint file,
which holds neither the sensors' random streams nor their sampling times.
Empty without sensors.
"""
function get_sensor_callbacks(num_sats::Int, args::SimulationConfiguration)
    spacecraft = args.dynamics_model.spacecraft
    groups = Dict{Float64, Vector{Tuple{Int, Int}}}()
    for i in eachindex(spacecraft), (k, entry) in enumerate(mounted_sensors(spacecraft[i]))
        push!(get!(groups, sensor_period_s(entry.sensor), Tuple{Int, Int}[]), (i, k))
    end
    isempty(groups) && return ()
    args.simulation_settings.resume_from_checkpoint && throw(ArgumentError(
        "Sensors do not support resume_from_checkpoint: the checkpoint holds neither their random streams, " *
        "a gyro's bias nor their sampling times. Run from the start."))
    sensor_sats = [i for i in eachindex(spacecraft) if !isempty(mounted_sensors(spacecraft[i]))]
    reset_done = Ref(false)
    callbacks = Any[]
    for period in sort!(collect(keys(groups)))
        members = groups[period]
        # Run state, kept across checkpoint segments: the run's start and the
        # number of the next sample, due at start + next * period.
        start = Ref(NaN)
        next = Ref(0)
        due() = start[] + next[] * period
        function sample_now!(integrator)
            _sample_sensors!(integrator, members)
            next[] += 1
            while due() <= integrator.t
                next[] += 1
            end
            _maybe_add_control_tstop!(integrator, due())
            return nothing
        end
        function initialize!(cb, u, t, integrator)
            if isnan(start[])                       # the run's first solve
                if !reset_done[]
                    for i in sensor_sats
                        reset_sensors!(integrator.p.args.dynamics_model.spacecraft[i])
                    end
                    reset_done[] = true
                end
                start[] = Float64(t)
                next[] = 0
            end
            # The run's first sample, or one due where the previous segment ended.
            Float64(t) >= due() ? sample_now!(integrator) : _maybe_add_control_tstop!(integrator, due())
            return nothing
        end
        affect! = integrator -> begin
            sample_now!(integrator)
            _mark_discontinuity!(integrator, false)        # sensors only observe
            return nothing
        end
        push!(callbacks, DiscreteCallback((u, t, integrator) -> t >= due(), affect!;
            initialize=initialize!, save_positions=(false, false)))
    end
    return Tuple(callbacks)
end

"""
    sensor_save_fields(args) -> Vector{SaveField}

The `sensor` field when a spacecraft carries sensors: per spacecraft every
sensor's latest measurement, sensors in mounting order and their components
in order (`sc{i}_sensor_{k}`), NaN before a sensor's first sample.
"""
function sensor_save_fields(args::SimulationConfiguration)
    any(sc -> !isempty(mounted_sensors(sc)), args.dynamics_model.spacecraft) || return SaveField[]
    num_sats = length(args.dynamics_model.spacecraft)
    getter = (u, t, integrator) -> [Float64[x for entry in mounted_sensors(integrator.p.args.dynamics_model.spacecraft[i])
                                             for x in entry.sensor.latest] for i in 1:num_sats]
    return SaveField[SaveField(:sensor, getter; per_satellite=true, column_prefix="sensor")]
end
