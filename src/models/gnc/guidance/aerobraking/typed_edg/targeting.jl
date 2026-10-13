
function _edg_orbit_metrics_from_rv(pos::SVector{3, Float64}, vel::SVector{3, Float64}, mass::Float64, planet)
    radius = norm(pos)
    energy = 0.5 * dot(vel, vel) - planet.μ / radius
    if !(isfinite(energy) && energy < 0.0)
        return (energy=energy, periapsis=NaN, apoapsis=Inf)
    end

    oe = rvtoorbitalelement(pos, vel, mass, planet)
    a, e = oe[1], oe[2]
    if !(isfinite(a) && isfinite(e))
        return (energy=energy, periapsis=NaN, apoapsis=NaN)
    end
    return (energy=energy, periapsis=a * (1.0 - e), apoapsis=a * (1.0 + e))
end

function _edg_target_energy_from_apoapsis(planet, target_apoapsis_radius_m::Float64, periapsis_radius_m::Float64)
    if !(isfinite(target_apoapsis_radius_m) && target_apoapsis_radius_m > 0.0 &&
            isfinite(periapsis_radius_m) && periapsis_radius_m > 0.0)
        return NaN
    end
    return -planet.μ / (target_apoapsis_radius_m + periapsis_radius_m)
end

function _edg_corrected_target_energy_from_apoapsis(
    planet,
    target_apoapsis_radius_m::Float64,
    periapsis_radius_m::Float64,
    perturbation_energy_change_jkg::Float64,
)
    target_energy = _edg_target_energy_from_apoapsis(
        planet,
        target_apoapsis_radius_m,
        periapsis_radius_m,
    )
    return target_energy - perturbation_energy_change_jkg
end

function _edg_vacuum_apoapsis_correction(
    p::ODEParams,
    pos0::SVector{3, Float64},
    vel0::SVector{3, Float64},
    mass::Float64,
    t0::Float64,
)
    planet = p.args.environment_model.planet
    initial_energy = 0.5 * dot(vel0, vel0) - planet.μ / norm(pos0)
    pos = pos0
    vel = vel0
    radial_velocity = dot(pos, vel) / norm(pos)
    dt = 1.0
    initial_metrics = _edg_orbit_metrics_from_rv(pos0, vel0, mass, planet)
    semi_major_axis = isfinite(initial_metrics.energy) && initial_metrics.energy < 0.0 ?
        -planet.μ / (2.0 * initial_metrics.energy) : NaN
    half_period = isfinite(semi_major_axis) && semi_major_axis > 0.0 ?
        pi * sqrt(semi_major_axis^3 / planet.μ) : 0.0
    max_duration = max(20_000.0, 1.25 * half_period)
    elapsed = 0.0
    event_reached = false

    while elapsed < max_duration
        acceleration(r, v, tau) = _edg_prediction_gravity_acceleration(p, r, v, mass, t0 + tau)
        a1 = acceleration(pos, vel, elapsed)
        k1r, k1v = vel, a1
        a2 = acceleration(pos + 0.5dt * k1r, vel + 0.5dt * k1v, elapsed + 0.5dt)
        k2r, k2v = vel + 0.5dt * k1v, a2
        a3 = acceleration(pos + 0.5dt * k2r, vel + 0.5dt * k2v, elapsed + 0.5dt)
        k3r, k3v = vel + 0.5dt * k2v, a3
        a4 = acceleration(pos + dt * k3r, vel + dt * k3v, elapsed + dt)
        k4r, k4v = vel + dt * k3v, a4
        next_pos = pos + dt * (k1r + 2.0k2r + 2.0k3r + k4r) / 6.0
        next_vel = vel + dt * (k1v + 2.0k2v + 2.0k3v + k4v) / 6.0
        next_radial_velocity = dot(next_pos, next_vel) / norm(next_pos)
        if radial_velocity > 0.0 && next_radial_velocity <= 0.0
            pos, vel, event_dt = _edg_prediction_event(acceleration,
                (r, v, tau) -> dot(r, v), pos, vel, elapsed, dt)
            elapsed += event_dt
            event_reached = true
            break
        end
        elapsed += dt
        pos, vel = next_pos, next_vel
        radial_velocity = next_radial_velocity
    end

    metrics = _edg_orbit_metrics_from_rv(pos, vel, mass, planet)
    final_energy = 0.5 * dot(vel, vel) - planet.μ / norm(pos)
    return (
        energy_change_jkg=final_energy - initial_energy,
        periapsis_radius_m=metrics.periapsis,
        apoapsis_radius_m=metrics.apoapsis,
        propagation_time_s=elapsed,
        event_reached=event_reached,
        position=pos,
        velocity=vel,
    )
end

function _edg_vacuum_drag_passage_exit(
    p::ODEParams,
    pos0::SVector{3, Float64},
    vel0::SVector{3, Float64},
    mass::Float64,
    t0::Float64,
)
    planet = p.args.environment_model.planet
    exit_altitude_m = 1e3 * Float64(p.args.environment_model.EI)
    pos = pos0
    vel = vel0
    dt = 1.0
    elapsed = 0.0
    max_duration = 2_000.0
    passed_periapsis = dot(pos, vel) >= 0.0
    event_reached = false

    while elapsed < max_duration
        acceleration(r, v, tau) = _edg_prediction_gravity_acceleration(p, r, v, mass, t0 + tau)
        a1 = acceleration(pos, vel, elapsed)
        k1r, k1v = vel, a1
        a2 = acceleration(pos + 0.5dt * k1r, vel + 0.5dt * k1v, elapsed + 0.5dt)
        k2r, k2v = vel + 0.5dt * k1v, a2
        a3 = acceleration(pos + 0.5dt * k2r, vel + 0.5dt * k2v, elapsed + 0.5dt)
        k3r, k3v = vel + 0.5dt * k2v, a3
        a4 = acceleration(pos + dt * k3r, vel + dt * k3v, elapsed + dt)
        k4r, k4v = vel + dt * k3v, a4
        pos = pos + dt * (k1r + 2.0k2r + 2.0k3r + k4r) / 6.0
        vel = vel + dt * (k1v + 2.0k2v + 2.0k3v + k4v) / 6.0
        elapsed += dt
        passed_periapsis |= dot(pos, vel) >= 0.0

        if passed_periapsis
            et = _edg_ephemeris_time(p, t0 + elapsed)
            pos_pp, _ = r_intor_p!(pos, vel, planet, et, p.args.environment_model.ephemerides_model)
            altitude_m = rtolatlong(pos_pp, planet)[1]
            if altitude_m >= exit_altitude_m
                event_reached = true
                break
            end
        end
    end
    return (position=pos, velocity=vel, propagation_time_s=elapsed, event_reached=event_reached)
end

function _edg_prediction_gravity_acceleration(
    p::ODEParams,
    r::SVector{3, Float64},
    v::SVector{3, Float64},
    mass::Float64,
    t_abs::Float64,
)
    planet = p.args.environment_model.planet
    ephemerides_model = p.args.environment_model.ephemerides_model
    et = _edg_ephemeris_time(p, t_abs)
    pos_pp, _ = r_intor_p!(r, v, planet, et, ephemerides_model)
    dynamic_effectors = p.args.dynamics_model.dynamic_effectors
    use_j2 = any(nameof(typeof(effector)) == :InverseSquaredJ2GravityModel for effector in dynamic_effectors)
    acceleration_ii = if use_j2
        l_pi = _edg_planet_frame_lpi(p, t_abs)
        SVector{3, Float64}(l_pi' * _inverse_squared_j2_gravity_accel(pos_pp, planet))
    else
        _inverse_squared_gravity_accel(r, planet)
    end
    et = _edg_ephemeris_time(p, t_abs)
    for effector in dynamic_effectors
        if effector isa NBodyGravityModel
            acceleration_ii += nbody_acceleration_ii_at_epoch(effector, r, p, et)
        end
    end
    return acceleration_ii
end

function _edg_targeting_constrained_alpha(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    env,
    base_alpha::Float64,
    alpha_past::Float64;
    heat_rate_control::Bool,
    structural_control::Bool,
)::Float64
    base = clamp(base_alpha, config.min_alpha_rad, config.max_alpha_rad)
    base <= config.min_alpha_rad + 1e-8 && return config.min_alpha_rad

    alpha_hr = base
    alpha_struct = base
    if heat_rate_control
        thermal_model = p.args.environment_model.thermal_model
        taf = hasproperty(thermal_model, :thermal_accomodation_factor) ? Float64(thermal_model.thermal_accomodation_factor) : 1.0
        planet = p.args.environment_model.planet
        alpha_hr = _energy_depletion_heatrate_root_alpha(
            taf=taf,
            rho=env.rho,
            T_p=env.temperature,
            R=planet.R,
            gamma=planet.γ,
            S=env.molecular_speed_ratio,
            max_alpha=base,
            min_alpha=config.min_alpha_rad,
            heat_rate_limit=config.heat_rate_limit_w_cm2,
            alpha_past=alpha_past,
        )
    end
    if structural_control
        alpha_struct = _energy_depletion_struct_load_root_alpha(
            config,
            env,
            spacecraft,
            config.controlled_panel_links,
            base,
        )
    end
    return clamp(min(alpha_hr, alpha_struct), config.min_alpha_rad, config.max_alpha_rad)
end

# PeriodicCallback uses its solve-start clock, including when a prediction starts
# between control ticks. Never stretch the command grid to fit its horizon.

_edg_targeting_control_period(config, p) = _edg_targeting_control_clock(config, p).period

@inline function _edg_switch_due(t::Float64, switch_s::Float64)
    # Equal callback ticks may differ by a few floating-point ulps.
    return isfinite(switch_s) && t >= switch_s - 8eps(max(abs(t), abs(switch_s), 1.0))
end

@inline function _edg_at_control_tick(t::Float64, period::Float64, origin::Float64=0.0)
    return abs(t - (origin + round((t - origin) / period) * period)) <= 8eps(max(abs(t), period, 1.0))
end

function _edg_targeting_prediction_time_grid(duration::Float64; t0::Float64=0.0, period::Float64=1.0, origin::Float64=0.0)
    isfinite(duration) && duration > 0.0 || throw(ArgumentError("Prediction duration must be positive and finite."))
    isfinite(period) && period > 0.0 || throw(ArgumentError("Control period must be positive and finite."))
    isfinite(t0) || throw(ArgumentError("Prediction start time must be finite."))
    isfinite(origin) || throw(ArgumentError("Control-clock origin must be finite."))
    first_tick = floor(Int, (t0 - origin) / period) + 1
    while _edg_switch_due(t0, origin + first_tick * period)
        first_tick += 1
    end
    last_tick = floor(Int, (t0 + duration - origin) / period)
    last_tick - first_tick < 100_000 || throw(ArgumentError("EDG prediction exceeds 100000 control intervals."))
    times = Float64[0.0]
    for tick in first_tick:last_tick
        tau = origin + tick * period - t0
        0.0 < tau < duration && push!(times, tau)
    end
    push!(times, duration)
    return times
end

function _edg_prediction_step(acceleration, r, v, t, dt)
    k1r, k1v = v, acceleration(r, v, t)
    k2r = v + 0.5dt * k1v
    k2v = acceleration(r + 0.5dt * k1r, k2r, t + 0.5dt)
    k3r = v + 0.5dt * k2v
    k3v = acceleration(r + 0.5dt * k2r, k3r, t + 0.5dt)
    k4r = v + dt * k3v
    k4v = acceleration(r + dt * k3r, k4r, t + dt)
    return (r + dt * (k1r + 2k2r + 2k3r + k4r) / 6,
        v + dt * (k1v + 2k2v + 2k3v + k4v) / 6)
end

# Control holding and integration accuracy are separate. Slow controllers must
# not force a coarse RK4 step; subdivision never updates the held command.
function _edg_prediction_advance(acceleration, r, v, t, dt)
    count = max(1, ceil(Int, dt))
    step = dt / count
    for k in 0:count-1
        r, v = _edg_prediction_step(acceleration, r, v, t + k * step, step)
    end
    return r, v
end

function _edg_prediction_event(acceleration, event, r, v, t, dt)
    left, right = 0.0, dt
    initial = event(r, v, t)
    for _ in 1:48
        middle = (left + right) / 2
        rm, vm = _edg_prediction_advance(acceleration, r, v, t, middle)
        value = event(rm, vm, t + middle)
        if signbit(value) == signbit(initial)
            left = middle
        else
            right = middle
        end
        right - left <= 1e-9 && break
    end
    elapsed = (left + right) / 2
    re, ve = _edg_prediction_advance(acceleration, r, v, t, elapsed)
    return re, ve, elapsed
end

function _edg_exact_targeting_exit(config, p, spacecraft, mass, t, track)
    altitude = 1e3 * Float64(p.args.environment_model.EI)
    if abs(track.h[1] - altitude) <= 1e-6 && dot(track.positions[1], track.velocities[1]) > 0.0
        return (position=track.positions[1], velocity=track.velocities[1], time=track.time[1])
    end
    j = findfirst(k -> track.h[k] < altitude <= track.h[k + 1] &&
        dot(track.positions[k + 1], track.velocities[k + 1]) > 0.0, 1:length(track.time)-1)
    j === nothing && error("EDG targeting prediction did not reach outbound EI within its horizon.")
    alpha = track.alpha_profile[j]
    acceleration(r, v, tau) = _edg_prediction_gravity_acceleration(p, r, v, mass, t + tau) +
        _edg_targeting_aero_acceleration(config, p, spacecraft, r, v, mass, t + tau, alpha)
    event(r, v, tau) = _edg_targeting_prediction_environment(p, r, v, t + tau).altitude_m - altitude
    r, v, dt = _edg_prediction_event(acceleration, event, track.positions[j],
        track.velocities[j], track.time[j], track.time[j + 1] - track.time[j])
    return (position=r, velocity=v, time=track.time[j] + dt)
end

function _edg_outbound_exit_index(track, p::ODEParams)::Int
    periapsis_index = argmin(track.h)
    exit_altitude_m = 1e3 * Float64(p.args.environment_model.EI)
    relative_index = findfirst(>=(exit_altitude_m), @view track.h[periapsis_index:end])
    return relative_index === nothing ? lastindex(track.h) : periapsis_index + relative_index - 1
end

function _edg_targeting_aero_acceleration(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    r::SVector{3, Float64},
    v::SVector{3, Float64},
    mass::Float64,
    t_abs::Float64,
    alpha::Float64,
)::SVector{3, Float64}
    env = _edg_targeting_prediction_environment(p, r, v, t_abs)
    return _edg_targeting_aero_acceleration(config, spacecraft, mass, alpha, env)
end

function _edg_targeting_aero_acceleration(
    config::AerobrakingEnergyDepletionConfig,
    spacecraft,
    mass::Float64,
    alpha::Float64,
    env,
)::SVector{3, Float64}
    if !(env.rho > 0.0 && env.speed > eps(Float64))
        return SVector{3, Float64}(0.0, 0.0, 0.0)
    end

    h_pp = cross(env.pos_pp, env.vel_pp)
    h_norm = norm(h_pp)
    h_norm <= eps(Float64) && return SVector{3, Float64}(0.0, 0.0, 0.0)
    vel_hat = env.vel_pp_rw / env.speed
    lift_vec = cross(h_pp / h_norm, vel_hat)
    lift_norm = norm(lift_vec)
    lift_hat = lift_norm > eps(Float64) ? lift_vec / lift_norm : SVector{3, Float64}(0.0, 0.0, 0.0)
    drag_hat = -vel_hat
    q = env.dynamic_pressure
    cl, cd = _edg_dynamic_spacecraft_aero_coefficients(
        spacecraft,
        env.temperature,
        max(env.molecular_speed_ratio, eps(Float64)),
        config.controlled_panel_links,
        alpha,
    )
    area = _edg_total_ref_area(spacecraft)
    force_pp = q * area * (cd * drag_hat + cl * lift_hat)
    return SVector{3, Float64}(env.l_pi' * force_pp) / mass
end

function _edg_integrated_targeting_trajectory(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos0::SVector{3, Float64},
    vel0::SVector{3, Float64},
    mass::Float64,
    t::Float64,
    times::Vector{Float64},
    switch_time_s::Float64;
    heat_rate_control::Bool,
    structural_control::Bool,
)
    planet = p.args.environment_model.planet
    n = length(times)
    positions = Vector{SVector{3, Float64}}(undef, n)
    velocities = Vector{SVector{3, Float64}}(undef, n)
    alpha_profile = Vector{Float64}(undef, n)
    positions[1] = pos0
    velocities[1] = vel0
    alpha_past = Float64(spacecraft.links[first(config.controlled_panel_links)].α)
    control_clock = _edg_targeting_control_clock(config, p)

    function acceleration(r, v, tau, alpha)
        gravity = _edg_prediction_gravity_acceleration(p, r, v, mass, t + tau)
        aero = _edg_targeting_aero_acceleration(config, p, spacecraft, r, v, mass, t + tau, alpha)
        return gravity + aero
    end

    for j in 1:(n - 1)
        tau = times[j]
        r = positions[j]
        v = velocities[j]
        base_alpha = _edg_switch_due(t + tau, switch_time_s) ? config.min_alpha_rad : config.max_alpha_rad
        env = _edg_targeting_prediction_environment(p, r, v, t + tau)
        alpha = _edg_targeting_constrained_alpha(
            config,
            p,
            spacecraft,
            env,
            base_alpha,
            alpha_past;
            heat_rate_control=heat_rate_control,
            structural_control=structural_control,
        )
        if j == 1 && !_edg_at_control_tick(t, control_clock.period, control_clock.origin)
            alpha = alpha_past
        end
        alpha_profile[j] = alpha
        alpha_past = alpha

        dt = times[j + 1] - times[j]
        positions[j + 1], velocities[j + 1] = _edg_prediction_advance(
            (rr, vv, tt) -> acceleration(rr, vv, tt, alpha), r, v, tau, dt)
    end

    final_tau = times[end]
    final_base = _edg_switch_due(t + final_tau, switch_time_s) ? config.min_alpha_rad : config.max_alpha_rad
    final_env = _edg_targeting_prediction_environment(p, positions[end], velocities[end], t + final_tau)
    alpha_profile[end] = _edg_targeting_constrained_alpha(
        config,
        p,
        spacecraft,
        final_env,
        final_base,
        alpha_past;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    if !_edg_at_control_tick(t + final_tau, control_clock.period, control_clock.origin)
        alpha_profile[end] = alpha_past
    end

    h = Vector{Float64}(undef, n)
    gamma = Vector{Float64}(undef, n)
    speed = Vector{Float64}(undef, n)
    rho = Vector{Float64}(undef, n)
    temperature = Vector{Float64}(undef, n)
    speed_ratio = Vector{Float64}(undef, n)
    for j in eachindex(times)
        env = _edg_targeting_prediction_environment(p, positions[j], velocities[j], t + times[j])
        radius = max(norm(positions[j]), eps(Float64))
        vel_norm = max(norm(velocities[j]), eps(Float64))
        h[j] = env.altitude_m
        gamma[j] = asin(clamp(dot(positions[j], velocities[j]) / max(radius * vel_norm, eps(Float64)), -1.0, 1.0))
        speed[j] = env.speed
        rho[j] = env.rho
        temperature[j] = env.temperature
        speed_ratio[j] = env.molecular_speed_ratio
    end
    return (
        time=times,
        h=h,
        gamma=gamma,
        speed=speed,
        rho=rho,
        temperature=temperature,
        speed_ratio=speed_ratio,
        positions=positions,
        velocities=velocities,
        alpha_profile=alpha_profile,
    )
end

function _edg_integrated_max_energy_depletion_trajectory(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos0::SVector{3, Float64},
    vel0::SVector{3, Float64},
    mass::Float64,
    t::Float64,
    times::Vector{Float64},
    heat_load_switches::NTuple{2, Float64};
    heat_rate_control::Bool,
    structural_control::Bool,
)
    planet = p.args.environment_model.planet
    n = length(times)
    positions = Vector{SVector{3, Float64}}(undef, n)
    velocities = Vector{SVector{3, Float64}}(undef, n)
    alpha_profile = Vector{Float64}(undef, n)
    positions[1] = pos0
    velocities[1] = vel0
    alpha_past = Float64(spacecraft.links[first(config.controlled_panel_links)].α)
    control_clock = _edg_targeting_control_clock(config, p)

    function acceleration(r, v, tau, alpha)
        gravity = _edg_prediction_gravity_acceleration(p, r, v, mass, t + tau)
        aero = _edg_targeting_aero_acceleration(config, p, spacecraft, r, v, mass, t + tau, alpha)
        return gravity + aero
    end

    function max_energy_alpha(r, v, tau)
        t_abs = t + tau
        heat_load_low_drag = (:heat_load in config.max_energy_submodes) &&
            isfinite(heat_load_switches[1]) &&
            isfinite(heat_load_switches[2]) &&
            heat_load_switches[1] < heat_load_switches[2] &&
            heat_load_switches[1] <= t_abs <= heat_load_switches[2]
        heat_load_low_drag && return config.min_alpha_rad

        base_alpha = config.max_alpha_rad
        env = _edg_targeting_prediction_environment(p, r, v, t_abs)
        alpha = _edg_targeting_constrained_alpha(
            config,
            p,
            spacecraft,
            env,
            base_alpha,
            alpha_past;
            heat_rate_control=heat_rate_control,
            structural_control=structural_control,
        )
        return alpha
    end

    for j in 1:(n - 1)
        tau = times[j]
        r = positions[j]
        v = velocities[j]
        alpha = max_energy_alpha(r, v, tau)
        if j == 1 && !_edg_at_control_tick(t, control_clock.period, control_clock.origin)
            alpha = alpha_past
        end
        alpha_profile[j] = alpha
        alpha_past = alpha

        dt = times[j + 1] - times[j]
        positions[j + 1], velocities[j + 1] = _edg_prediction_advance(
            (rr, vv, tt) -> acceleration(rr, vv, tt, alpha), r, v, tau, dt)
    end

    final_tau = times[end]
    alpha_profile[end] = _edg_at_control_tick(t + final_tau, control_clock.period, control_clock.origin) ?
        max_energy_alpha(positions[end], velocities[end], final_tau) : alpha_past

    h = Vector{Float64}(undef, n)
    gamma = Vector{Float64}(undef, n)
    speed = Vector{Float64}(undef, n)
    rho = Vector{Float64}(undef, n)
    temperature = Vector{Float64}(undef, n)
    speed_ratio = Vector{Float64}(undef, n)
    for j in eachindex(times)
        env = _edg_targeting_prediction_environment(p, positions[j], velocities[j], t + times[j])
        radius = max(norm(positions[j]), eps(Float64))
        vel_norm = max(norm(velocities[j]), eps(Float64))
        h[j] = env.altitude_m
        gamma[j] = asin(clamp(dot(positions[j], velocities[j]) / max(radius * vel_norm, eps(Float64)), -1.0, 1.0))
        speed[j] = env.speed
        rho[j] = env.rho
        temperature[j] = env.temperature
        speed_ratio[j] = env.molecular_speed_ratio
    end
    return (
        time=times,
        h=h,
        gamma=gamma,
        speed=speed,
        rho=rho,
        temperature=temperature,
        speed_ratio=speed_ratio,
        positions=positions,
        velocities=velocities,
        alpha_profile=alpha_profile,
    )
end

function _edg_predict_targeting_outcome(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass_state::Float64,
    t::Float64,
    switch_time_s::Float64;
    heat_rate_control::Bool,
    structural_control::Bool,
)
    mass = _edg_predict_mass(spacecraft, mass_state)
    duration = min(
        _edg_drag_passage_duration(config, p, pos, vel, mass) + 120.0,
        config.planning_horizon_s,
    )
    clock = _edg_targeting_control_clock(config, p)
    times = _edg_targeting_prediction_time_grid(duration; t0=t, period=clock.period, origin=clock.origin)
    track = _edg_integrated_targeting_trajectory(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        times,
        switch_time_s;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    planet = p.args.environment_model.planet
    exit = _edg_exact_targeting_exit(config, p, spacecraft, mass, t, track)
    final_metrics = _edg_orbit_metrics_from_rv(exit.position, exit.velocity, mass, planet)
    return (
        switch_time_s=switch_time_s,
        duration_s=exit.time,
        energy_jkg=final_metrics.energy,
        periapsis_radius_m=final_metrics.periapsis,
        apoapsis_radius_m=final_metrics.apoapsis,
        exit_position=exit.position,
        exit_velocity=exit.velocity,
        track=track,
        alpha_profile=track.alpha_profile,
    )
end

function _edg_predict_max_energy_depletion_outcome(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass_state::Float64,
    t::Float64,
    env,
    heat_load_j_cm2::Float64;
    heat_rate_control::Bool,
    structural_control::Bool,
)
    mass = _edg_predict_mass(spacecraft, mass_state)
    duration = min(
        _edg_drag_passage_duration(config, p, pos, vel, mass) + 120.0,
        config.planning_horizon_s,
    )
    clock = _edg_targeting_control_clock(config, p)
    times = _edg_targeting_prediction_time_grid(duration; t0=t, period=clock.period, origin=clock.origin)
    heat_load_switches = (:heat_load in config.max_energy_submodes) ?
        _edg_solve_heat_load_switches(
            config,
            p,
            spacecraft,
            pos,
            vel,
            mass,
            env,
            heat_load_j_cm2,
            t;
            heat_rate_control=heat_rate_control,
            structural_control=structural_control,
        ) :
        (Inf, Inf)
    track = _edg_integrated_max_energy_depletion_trajectory(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        times,
        heat_load_switches;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    planet = p.args.environment_model.planet
    exit = _edg_exact_targeting_exit(config, p, spacecraft, mass, t, track)
    final_metrics = _edg_orbit_metrics_from_rv(exit.position, exit.velocity, mass, planet)
    return (
        switch_time_s=NaN,
        heat_load_switches_s=heat_load_switches,
        duration_s=exit.time,
        energy_jkg=final_metrics.energy,
        periapsis_radius_m=final_metrics.periapsis,
        apoapsis_radius_m=final_metrics.apoapsis,
        exit_position=exit.position,
        exit_velocity=exit.velocity,
        track=track,
        alpha_profile=track.alpha_profile,
    )
end

function _edg_targeting_switch_outcomes(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass::Float64,
    t::Float64;
    heat_rate_control::Bool,
    structural_control::Bool,
)
    low_drag = _edg_predict_targeting_outcome(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        t;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    high_drag = _edg_predict_targeting_outcome(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        t + low_drag.duration_s + 1.0;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    return low_drag, high_drag
end

function _edg_targeting_bracket_outcomes(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass::Float64,
    t::Float64;
    heat_load_j_cm2::Float64=0.0,
    heat_rate_control::Bool,
    structural_control::Bool,
)
    low_drag = _edg_predict_targeting_outcome(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        t;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    env = _edg_targeting_prediction_environment(p, pos, vel, t)
    max_energy_depletion = _edg_predict_max_energy_depletion_outcome(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        env,
        heat_load_j_cm2;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    return low_drag, max_energy_depletion
end

function _edg_targeting_switch_solution(
    config::AerobrakingEnergyDepletionConfig,
    state::AerobrakingEnergyDepletionState,
    p::ODEParams,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass::Float64,
    t::Float64,
    i::Int;
    heat_load_j_cm2::Float64=0.0,
    heat_rate_control::Bool,
    structural_control::Bool,
)
    target_energy = state.target_energy_jkg[i]
    isfinite(target_energy) || return (switch_s=Inf, residual_jkg=NaN, status=:invalid_target, left_switch_s=NaN, right_switch_s=NaN, left_residual_jkg=NaN, right_residual_jkg=NaN)

    clock = _edg_targeting_control_clock(config, p)
    period, origin = clock.period, clock.origin
    next_tick = floor(Int, (t - origin) / period) + 1
    while _edg_switch_due(t, origin + next_tick * period)
        next_tick += 1
    end
    first_tick = 0
    on_tick = _edg_at_control_tick(t, period, origin)
    last_tick = floor(Int, (t + 2_000.0 - origin) / period) - next_tick + (on_tick ? 1 : 0)
    last_tick >= 0 || throw(ArgumentError("No executable EDG control tick within the 2000 s switch horizon."))
    # Index zero denotes the first executable command. Between callback ticks,
    # retain the current held angle and wait until the next callback.
    switch_at(k) = k == 0 ? (on_tick ? t : origin + next_tick * period) :
        origin + (next_tick + k - (on_tick ? 1 : 0)) * period
    cache = Dict{Int, Float64}()
    function residual(k)
        return get!(cache, k) do
            outcome = _edg_predict_targeting_outcome(config, p, spacecraft,
                pos, vel, mass, t, switch_at(k);
                heat_rate_control=heat_rate_control, structural_control=structural_control)
            value = outcome.energy_jkg - target_energy
            isfinite(value) || error("EDG targeting prediction returned a nonfinite energy residual.")
            value
        end
    end
    left, right = first_tick, last_tick
    fleft, fright = residual(left), residual(right)
    state.bracket_min_energy_jkg[i], state.bracket_max_energy_jkg[i] =
        extrema((fleft + target_energy, fright + target_energy))
    if fleft != 0.0 && fright != 0.0 && signbit(fleft) != signbit(fright)
        while right - left > 1
            middle = left + (right - left) ÷ 2
            fmiddle = residual(middle)
            if fmiddle == 0.0
                left = right = middle
                fleft = fright = fmiddle
            elseif signbit(fmiddle) == signbit(fleft)
                left, fleft = middle, fmiddle
            else
                right, fright = middle, fmiddle
            end
        end
    end
    # A held command generally has no exact continuous root. Retain and compare
    # the actual adjacent residuals, rather than interpolating an unevaluated time.
    chosen = abs(fleft) <= abs(fright) ? left : right
    selected_residual = residual(chosen)
    status = selected_residual == 0.0 ? :exact_sample :
        (signbit(fleft) != signbit(fright) ? :quantized_bracket : :endpoint_fallback)
    return (switch_s=switch_at(chosen), residual_jkg=selected_residual, status=status,
        left_switch_s=switch_at(left), right_switch_s=switch_at(right),
        left_residual_jkg=fleft, right_residual_jkg=fright)
end

_edg_solve_targeting_switch(args...; kwargs...) = _edg_targeting_switch_solution(args...; kwargs...).switch_s

# Retained compatibility helper; active journal planning uses its own certificate.

function _edg_targeting_outcome_with_heat_load(
    config::AerobrakingEnergyDepletionConfig,
    p::ODEParams,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass::Float64,
    t::Float64,
    switch_time_s::Float64,
    accumulated_heat_load_j_cm2::Float64;
    heat_rate_control::Bool,
    structural_control::Bool,
)
    outcome = _edg_predict_targeting_outcome(
        config,
        p,
        spacecraft,
        pos,
        vel,
        mass,
        t,
        switch_time_s;
        heat_rate_control=heat_rate_control,
        structural_control=structural_control,
    )
    future_heat_load = _edg_profile_heat_load(
        config,
        p,
        outcome.track,
        outcome.alpha_profile;
        heat_rate_control=false,
    )
    return merge(outcome, (heat_load_j_cm2=accumulated_heat_load_j_cm2 + future_heat_load,))
end

# Retained compatibility helper; active journal planning uses its own certificate.

function _edg_certify_targeting_candidates(
    candidate_times::AbstractVector{<:Real},
    evaluate_candidate;
    heat_load_limit_j_cm2::Real,
    energy_order_tolerance_jkg::Real,
    heat_load_tolerance_j_cm2::Real,
)
    isempty(candidate_times) && throw(ArgumentError("candidate_times must not be empty."))
    energy_tolerance = Float64(energy_order_tolerance_jkg)
    heat_tolerance = Float64(heat_load_tolerance_j_cm2)
    heat_limit = Float64(heat_load_limit_j_cm2)
    energy_tolerance >= 0.0 || throw(ArgumentError("energy_order_tolerance_jkg must be >= 0.0."))
    heat_tolerance >= 0.0 || throw(ArgumentError("heat_load_tolerance_j_cm2 must be >= 0.0."))

    first_time = Float64(first(candidate_times))
    first_outcome = evaluate_candidate(first_time)
    certified_times = Float64[]
    certified_outcomes = typeof(first_outcome)[]

    if !isfinite(first_outcome.energy_jkg)
        return (times=certified_times, outcomes=certified_outcomes, failure=:nonfinite_energy, failure_time_s=first_time)
    end
    if !(isfinite(first_outcome.heat_load_j_cm2) && first_outcome.heat_load_j_cm2 <= heat_limit + heat_tolerance)
        return (times=certified_times, outcomes=certified_outcomes, failure=:heat_load, failure_time_s=first_time)
    end

    push!(certified_times, first_time)
    push!(certified_outcomes, first_outcome)
    previous_energy = first_outcome.energy_jkg

    for candidate_time in Iterators.drop(candidate_times, 1)
        time_s = Float64(candidate_time)
        outcome = evaluate_candidate(time_s)
        if !isfinite(outcome.energy_jkg)
            return (times=certified_times, outcomes=certified_outcomes, failure=:nonfinite_energy, failure_time_s=time_s)
        end
        if !(isfinite(outcome.heat_load_j_cm2) && outcome.heat_load_j_cm2 <= heat_limit + heat_tolerance)
            return (times=certified_times, outcomes=certified_outcomes, failure=:heat_load, failure_time_s=time_s)
        end
        if !(outcome.energy_jkg < previous_energy - energy_tolerance)
            return (times=certified_times, outcomes=certified_outcomes, failure=:energy_order, failure_time_s=time_s)
        end
        push!(certified_times, time_s)
        push!(certified_outcomes, outcome)
        previous_energy = outcome.energy_jkg
    end

    return (times=certified_times, outcomes=certified_outcomes, failure=:none, failure_time_s=Inf)
end

# Retained compatibility helper; active journal planning uses its own certificate.

function _edg_disable_uncertified_targeting!(
    config::AerobrakingEnergyDepletionConfig,
    state::AerobrakingEnergyDepletionState,
    i::Int;
    prefer_max_energy_depletion::Bool,
)
    state.targeting_active[i] = false
    if prefer_max_energy_depletion && (:max_energy_depletion in config.guidance_modes)
        state.selected_mode[i] = :max_energy_depletion
        state.safe_low_drag[i] = false
    else
        state.selected_mode[i] = :safe_low_drag
        state.safe_low_drag[i] = true
    end
    return Inf
end
