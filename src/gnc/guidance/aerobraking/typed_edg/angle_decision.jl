
function _edg_recompute_switches!(
    config::AerobrakingEnergyDepletionConfig, state::AerobrakingEnergyDepletionState,
    p::ODEParams,
    env,
    spacecraft,
    pos::SVector{3, Float64},
    vel::SVector{3, Float64},
    mass::Float64,
    heat_load_j_cm2::Float64,
    t::Float64,
    i::Int,
)
    action = :not_required
    if state.selected_mode[i] == :targeting && state.targeting_active[i]
        return :cached
    end
    # Guidance owns the complete targeting plan, its certificate and replan
    # clock. Control only consumes that plan; the fixed-energy utility solver
    # must not replace it or consume a due guidance update.
    if state.selected_mode[i] == :max_energy_depletion && (:heat_load in config.max_energy_submodes)
        action = :cached
        if !_edg_in_drag_passage(p, env)
            if state.heat_load_drag_passage_active[i]
                state.heat_load_switch_solved[i] = false
                state.heat_load_drag_passage_active[i] = false
                state.heat_load_switches_s[i] = (Inf, Inf)
                state.heat_load_entry_time_s[i] = NaN
                state.heat_load_last_reevaluation_s[i] = -Inf
                state.heat_load_security_active[i] = false
                state.heat_load_previous_j_cm2[i] = 0.0
            end
            return :outside_pass
        end
        heat_rate_control = :heat_rate in config.max_energy_submodes
        structural_control = :structural_load in config.max_energy_submodes
        if !state.heat_load_drag_passage_active[i]
            state.heat_load_drag_passage_active[i] = true
            state.heat_load_entry_time_s[i] = t
            state.heat_load_last_reevaluation_s[i] = t
            state.heat_load_previous_j_cm2[i] = heat_load_j_cm2
        end
        if !state.heat_load_switch_solved[i]
            solve_altitude_m = 1e3 * Float64(p.args.environment_model.EI) - 500.0
            env.altitude_m <= solve_altitude_m || return :awaiting_entry
            t - state.last_switch_solve_t[i] >= 1.0 || return :awaiting_cadence
            switches = _edg_solve_heat_load_switches(
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
            )
            state.heat_load_switches_s[i] = switches
            # A finite zero-width result is a completed no-switch plan.
            state.heat_load_switch_solved[i] = all(isfinite, switches) && switches[1] <= switches[2]
            state.last_switch_solve_t[i] = t
            action = :solved
        else
            switches = state.heat_load_switches_s[i]
            remaining = switches[2] - t
            passage_switch_time = switches[2] - state.heat_load_entry_time_s[i]
            ascending = dot(pos, vel) > 0.0
            start_reevaluation = ascending ||
                (isfinite(remaining) && isfinite(passage_switch_time) && abs(remaining) < 0.2 * max(passage_switch_time, eps(Float64)))
            since_reevaluation = t - state.heat_load_last_reevaluation_s[i]
            after_first_switch = t - switches[1] > 1.0
            reevaluation_due = after_first_switch && (
                (since_reevaluation > 10.0 && remaining > 0.0) ||
                (3.0 < remaining < 50.0 && since_reevaluation > 3.0) ||
                (0.0 < remaining <= 3.0 && since_reevaluation > 0.8)
            )
            # Numerical plans remain cached for the passage; only closed form re-evaluates.
            if config.second_switch_reevaluation && config.heat_load_switch_solver == :closed_form &&
                    start_reevaluation && reevaluation_due &&
                    !state.heat_load_security_active[i]
                reevaluation_mode = since_reevaluation > 3.0 ? 1 : 2
                state.heat_load_switches_s[i] = _edg_recompute_second_heat_load_switch(
                    config,
                    p,
                    spacecraft,
                    pos,
                    vel,
                    mass,
                    env,
                    heat_load_j_cm2,
                    t,
                    switches;
                    passage_entry_time_s=state.heat_load_entry_time_s[i],
                    reevaluation_mode=reevaluation_mode,
                    heat_rate_control=heat_rate_control,
                    structural_control=structural_control,
                )
                state.heat_load_last_reevaluation_s[i] = t
                action = :reevaluated

            end
        end
        state.heat_load_previous_j_cm2[i] = heat_load_j_cm2
    end
    return action
end

@inline function _edg_base_alpha(config::AerobrakingEnergyDepletionConfig, state::AerobrakingEnergyDepletionState, t::Float64, i::Int)::Float64
    mode = state.selected_mode[i]
    if mode == :safe_low_drag || state.safe_low_drag[i] ||
            (mode == :max_energy_depletion && state.heat_load_security_active[i])
        return config.min_alpha_rad
    elseif mode == :targeting && state.targeting_active[i]
        return _edg_switch_due(t, state.targeting_switch_s[i]) ? config.min_alpha_rad : config.max_alpha_rad
    elseif mode == :max_energy_depletion
        if _edg_heat_load_low_drag_active(config, state, t, i)
            return config.min_alpha_rad
        end
    end
    return config.max_alpha_rad
end

# These are instantaneous model observations, not trajectory feasibility proofs.
function _edg_constraint_status(enabled::Bool, limit::Float64, value::Float64, minimum_value::Float64)
    enabled || return :disabled
    limit == Inf && return :unbounded
    isfinite(limit) && limit > 0.0 || return :invalid_limit
    isfinite(value) && isfinite(minimum_value) || return :unobserved
    value <= limit && return :within_limit
    return minimum_value > limit ? :above_limit_at_minimum_angle : :command_above_limit
end

function _edg_heat_budget_status(config, heat_load_j_cm2::Float64)
    :heat_load in config.max_energy_submodes || return :disabled
    limit = config.heat_load_limit_j_cm2
    limit == Inf && return :unbounded
    isfinite(limit) && limit > 0.0 || return :invalid_limit
    isfinite(heat_load_j_cm2) || return :unobserved
    return heat_load_j_cm2 >= limit ? :exhausted : :available
end

function _edg_command_alpha!(
    config::AerobrakingEnergyDepletionConfig, state::AerobrakingEnergyDepletionState,
    p::ODEParams,
    controlled_panel_links::Tuple{Vararg{Int}},
    env,
    spacecraft,
    base_alpha::Float64,
    heat_load_j_cm2::Float64,
    heat_load_low_drag_active::Bool,
    i::Int;
    pass_heat_load_j_cm2::Float64=heat_load_j_cm2,
)
    alpha = clamp(base_alpha, config.min_alpha_rad, config.max_alpha_rad)
    alpha_hr = alpha
    alpha_struct = alpha
    heat_rate_limit = config.heat_rate_limit_w_cm2
    heat_rate_active = (:heat_rate in config.max_energy_submodes) &&
        !heat_load_low_drag_active
    if heat_rate_active
        alpha_past = isfinite(state.last_alpha_rad[i]) ? state.last_alpha_rad[i] : alpha
        alpha_hr = _edg_heat_rate_alpha(config, p, env, alpha; limit_override=heat_rate_limit, alpha_past=alpha_past)
    end
    structural_active = (:structural_load in config.max_energy_submodes) &&
        !heat_load_low_drag_active
    if structural_active
        alpha_struct = _edg_structural_alpha(
            config,
            p,
            env,
            spacecraft,
            controlled_panel_links,
            alpha,
        )
    end
    if heat_rate_active &&
            structural_active
        alpha = min(alpha_hr, alpha_struct)
    elseif heat_rate_active
        alpha = alpha_hr
    elseif structural_active
        alpha = alpha_struct
    end
    alpha = clamp(alpha, config.min_alpha_rad, config.max_alpha_rad)
    # Diagnostic only. The existing switch plan and optional predictive security
    # mode own heat-load commands; cumulative telemetry is not a flight policy.
    budget_status = _edg_heat_budget_status(config, pass_heat_load_j_cm2)
    state.last_pass_heat_load_j_cm2[i] = pass_heat_load_j_cm2
    state.last_heat_budget_status[i] = budget_status
    state.last_alpha_rad[i] = alpha
    state.last_alpha_heat_rate_rad[i] = alpha_hr
    state.last_alpha_structural_rad[i] = alpha_struct
    state.last_heat_rate_w_cm2[i] = _edg_maxwellian_heat_rate(p, env, alpha)
    state.last_heat_load_j_cm2[i] = heat_load_j_cm2
    state.last_dynamic_pressure_pa[i] = env.dynamic_pressure
    reference_drag_area = _energy_depletion_struct_drag_area(
        spacecraft,
        env.temperature,
        env.molecular_speed_ratio,
        controlled_panel_links,
        config.max_alpha_rad,
        config,
    )
    controlled_drag_area = _energy_depletion_struct_drag_area(
        spacecraft,
        env.temperature,
        env.molecular_speed_ratio,
        controlled_panel_links,
        alpha,
        config,
    )
    state.last_structural_load_pa[i] = env.dynamic_pressure * controlled_drag_area /
        max(reference_drag_area, eps(Float64))
    minimum_drag_area = _energy_depletion_struct_drag_area(
        spacecraft, env.temperature, env.molecular_speed_ratio,
        controlled_panel_links, config.min_alpha_rad, config,
    )
    state.last_minimum_heat_rate_w_cm2[i] = _edg_maxwellian_heat_rate(p, env, config.min_alpha_rad)
    state.last_minimum_structural_load_pa[i] = env.dynamic_pressure * minimum_drag_area /
        max(reference_drag_area, eps(Float64))
    state.last_heat_rate_status[i] = _edg_constraint_status(
        :heat_rate in config.max_energy_submodes, config.heat_rate_limit_w_cm2,
        state.last_heat_rate_w_cm2[i], state.last_minimum_heat_rate_w_cm2[i],
    )
    state.last_structural_load_status[i] = _edg_constraint_status(
        :structural_load in config.max_energy_submodes, config.structural_load_limit_pa,
        state.last_structural_load_pa[i], state.last_minimum_structural_load_pa[i],
    )
    return alpha
end

"""
Private result of a control-time EDG decision. The angle command is in radians;
apply distinguishes an invalid-index no-op from a valid zero angle. Diagnostics
reflect state already computed before actuation. switch_action describes cache/
pass/solve execution, not scientific acceptance of a solution.
"""
struct EDGControlDecision{D}
    command::AerobrakingControlCommand
    apply::Bool
    switch_action::Symbol
    diagnostics::D
end

function control_decision!(
    config::AerobrakingEnergyDepletionConfig, state::AerobrakingEnergyDepletionState,
    controlled_panel_links::Tuple{Vararg{Int}},
    u,
    p::ODEParams,
    t::Float64,
    i::Int64,
)
    1 <= i <= length(state.selected_mode) || return EDGControlDecision(AerobrakingControlCommand(), false, :invalid_index, nothing)
    if state.selected_mode[i] == :inactive
        if !(:targeting in config.guidance_modes)
            state.selected_mode[i] = (:max_energy_depletion in config.guidance_modes) ? :max_energy_depletion : :safe_low_drag
        end
    end
    sc = _edg_control_sat_state(u, i)
    env = _edg_environment_state(u, p, Float64(t), i)
    spacecraft = p.args.dynamics_model.spacecraft[i]
    pos, vel, mass = _edg_control_pos_vel_mass(sc)
    heat_load = _edg_max_heat_load_for_links(sc, controlled_panel_links)
    pass_heat_load = _edg_pass_heat_load_for_links(sc, controlled_panel_links, state, i)
    switch_action = _edg_recompute_switches!(config, state, p, env, spacecraft, pos, vel, mass, pass_heat_load, Float64(t), i)
    _edg_update_heat_load_security!(config, state, p, env, spacecraft, pos, vel, mass, pass_heat_load, Float64(t), i)
    heat_load_low_drag_active = _edg_heat_load_low_drag_active(config, state, Float64(t), i)
    base_alpha = _edg_base_alpha(config, state, Float64(t), i)
    alpha = _edg_command_alpha!(config, state, p, controlled_panel_links, env, spacecraft, base_alpha, heat_load, heat_load_low_drag_active, i;
        pass_heat_load_j_cm2=pass_heat_load)
    diagnostics = (mode=state.selected_mode[i], heat_load_j_cm2=state.last_heat_load_j_cm2[i],
        alpha_heat_rate_rad=state.last_alpha_heat_rate_rad[i],
        alpha_structural_rad=state.last_alpha_structural_rad[i],
        dynamic_pressure_pa=state.last_dynamic_pressure_pa[i],
        targeting_switch_s=state.targeting_switch_s[i], heat_load_switches_s=state.heat_load_switches_s[i],
        heat_rate_w_cm2=state.last_heat_rate_w_cm2[i], pass_heat_load_j_cm2=pass_heat_load)
    return EDGControlDecision(AerobrakingControlCommand(alpha), true, switch_action, diagnostics)
end
