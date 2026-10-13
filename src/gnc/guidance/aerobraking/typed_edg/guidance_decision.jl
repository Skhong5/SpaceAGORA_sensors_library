function _edg_interpolate_bracket_value(exit_energy::Float64, energy_min::Float64, energy_max::Float64, value_at_min::Float64, value_at_max::Float64)
    width = energy_max - energy_min
    abs(width) < eps(Float64) && return 0.5 * (value_at_min + value_at_max)
    fraction = (exit_energy - energy_min) / width
    return value_at_min + fraction * (value_at_max - value_at_min)
end

function _edg_target_energy_from_reachable_bracket(
    planet,
    target_apoapsis_radius_m::Float64,
    energy_min::Float64,
    energy_max::Float64,
    periapsis_at_min::Float64,
    periapsis_at_max::Float64,
)
    function residual(exit_energy)
        periapsis = _edg_interpolate_bracket_value(exit_energy, energy_min, energy_max, periapsis_at_min, periapsis_at_max)
        desired_energy = _edg_target_energy_from_apoapsis(planet, target_apoapsis_radius_m, periapsis)
        return exit_energy - desired_energy
    end

    residual_min = residual(energy_min)
    residual_max = residual(energy_max)
    if isfinite(residual_min) && isfinite(residual_max) && residual_min * residual_max <= 0.0
        return Roots.find_zero(residual, (energy_min, energy_max), Roots.Brent(); rtol=1e-10)
    elseif isfinite(residual_min) && isfinite(residual_max) && abs(residual_max - residual_min) > eps(Float64)
        return energy_min - residual_min * (energy_max - energy_min) / (residual_max - residual_min)
    end
    return abs(residual_min) <= abs(residual_max) ? energy_min : energy_max
end

function _edg_set_targeting_fallback!(
    config::AerobrakingEnergyDepletionConfig,
    state::AerobrakingEnergyDepletionState,
    i::Int,
)
    state.targeting_active[i] = false
    state.safe_low_drag[i] = !(:max_energy_depletion in config.guidance_modes)
    state.selected_mode[i] = (:max_energy_depletion in config.guidance_modes) ? :max_energy_depletion : :safe_low_drag
    return nothing
end

function _edg_run_target_energy_bracketing!(
    config::AerobrakingEnergyDepletionConfig, state::AerobrakingEnergyDepletionState,
    u,
    p::ODEParams,
    t::Float64,
    i::Int,
)
    if state.energy_bracketing_evaluated[i]
        # Journal VI.D: replan only before the applied switch, at 0.5 Hz by default.
        state.targeting_active[i] || return nothing
        _edg_switch_due(t, state.targeting_switch_s[i]) && return nothing
        _edg_replan_due(t, state.last_switch_solve_t[i], config.switch_recompute_interval_s) || return nothing
    end
    env = _edg_environment_state(u, p, t, i)
    _edg_in_drag_passage(p, env) || return nothing
    sc = _edg_control_sat_state(u, i)
    pos, vel, mass = _edg_control_pos_vel_mass(sc)
    heat_load = _edg_pass_heat_load_for_links(sc, config.controlled_panel_links, state, i)
    plan = _edg_journal_targeting_plan(config, p, p.args.dynamics_model.spacecraft[i],
        pos, vel, mass, t; heat_load_j_cm2=heat_load)
    state.energy_bracketing_evaluated[i] = true
    state.energy_bracketing_count[i] += 1
    state.target_energy_jkg[i] = plan.target_energy_jkg
    state.target_perturbation_energy_change_jkg[i] = plan.coast_energy_change_jkg
    state.bracket_min_energy_jkg[i] = plan.lower_energy_jkg
    state.bracket_max_energy_jkg[i] = plan.upper_energy_jkg
    state.targeting_active[i] = plan.mode == :targeting
    state.safe_low_drag[i] = plan.mode == :safe_low_drag
    state.selected_mode[i] = plan.mode
    state.targeting_switch_s[i] = plan.switch_s
    state.last_switch_solve_t[i] = t
    state.targeting_diagnostics[i] = plan
    return nothing
end

function guidance_decision!(
    config::AerobrakingEnergyDepletionConfig, state::AerobrakingEnergyDepletionState,
    u,
    p::ODEParams,
    t::Float64,
    i::Int64,
)
    1 <= i <= length(state.selected_mode) || return nothing

    if :targeting in config.guidance_modes
        _edg_run_target_energy_bracketing!(config, state, u, p, Float64(t), i)
    elseif :max_energy_depletion in config.guidance_modes
        state.selected_mode[i] = :max_energy_depletion
        state.targeting_active[i] = false
        state.safe_low_drag[i] = false
        state.energy_bracketing_evaluated[i] = false
    else
        state.selected_mode[i] = :safe_low_drag
        state.targeting_active[i] = false
        state.safe_low_drag[i] = true
    end
    return nothing
end
