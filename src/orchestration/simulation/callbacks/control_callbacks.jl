@inline function _maybe_add_control_tstop!(integrator, tstop::Float64)
    if !(isfinite(tstop) && tstop > integrator.t)
        return nothing
    end
    current_max = try
        DiffEqBase.get_tstops_max(integrator)
    catch
        Inf
    end
    if tstop <= current_max + 1e-9 && applicable(DiffEqBase.add_tstop!, integrator, tstop)
        DiffEqBase.add_tstop!(integrator, tstop)
    end
    return nothing
end

@inline function _register_control_tstops!(integrator, control_model::BaseThrusterModel, sat_idx::Int)
    if sat_idx < 1 || sat_idx > length(control_model.start_burn_time)
        return nothing
    end
    # The effective window: the scheduler's own, or else the plan installed in
    # its burn-plan buffer (by the burn-plan family).
    start_s, stop_s = _effective_burn_window(control_model, integrator.p, sat_idx)
    _maybe_add_control_tstop!(integrator, start_s)
    _maybe_add_control_tstop!(integrator, stop_s)
    return nothing
end

# After the control tick of an effector in `:mounted` actuation, the engine moves
# the spacecraft's mounted hinges and thruster gimbals toward the levels the
# effector holds.
@inline function _apply_mounted_hinges!(control_model, p, sat_idx::Int, t)
    actuation_mode(control_model) === :mounted || return nothing
    spacecraft = p.args.dynamics_model.spacecraft[sat_idx]
    (mounted_hinge_count(spacecraft) == 0 && mounted_gimbal_count(spacecraft) == 0) && return nothing
    levels = actuator_levels(control_model, sat_idx)
    levels === nothing && return nothing
    applied = faulted_levels(spacecraft, levels)
    apply_hinge_levels!(spacecraft, applied, t)
    apply_gimbal_levels!(spacecraft, applied, t)
    return nothing
end

# After the control tick of an effector in `:mounted` actuation, each mounted
# thruster with firing dynamics schedules the average thrust it delivers until
# the next tick: the command as an on-time within the control period, through
# its minimum on-time, Schmitt trigger and thrust ramp. The schedule follows
# the command; the thruster's faults act on the thrust it delivers
# (`ActuatorMounting._delivered_thrust`), so they switch at their own times.
function _apply_mounted_thruster_firing!(control_model, p, sat_idx::Int, t, period::Float64)
    actuation_mode(control_model) === :mounted || return nothing
    spacecraft = p.args.dynamics_model.spacecraft[sat_idx]
    any(link -> any(th -> th.firing_dynamics, link.thrusters), spacecraft.links) || return nothing
    levels = actuator_levels(control_model, sat_idx)
    (levels === nothing || length(levels) != mounted_actuator_count(spacecraft)) && return nothing
    k = 0
    for link in spacecraft.links, thruster in link.thrusters
        k += 1
        thruster.firing_dynamics || continue
        thrust_calculation_schmitt_trigger!(link, thruster, actuator_saturate(thruster, levels[k]), Float64(t); period=period)
    end
    return nothing
end

@inline function _apply_mounted_tick!(control_model, p, sat_idx::Int, t, period::Float64)
    _apply_mounted_hinges!(control_model, p, sat_idx, t)
    _apply_mounted_thruster_firing!(control_model, p, sat_idx, t, period)
    return nothing
end

# Whether the engine ticks `control_model` with a periodic callback (internal;
# the model declares it with `tick_schedule`).
@inline control_requires_periodic_callback(control_model) = tick_schedule(control_model) === :periodic

@inline function _run_guidance_for_thruster_schedule!(integrator, sat_idx::Int)
    @inbounds for guidance_model in integrator.p.args.guidance_model.guidance_effectors
        _gnc_tick!(_update_guidance!, guidance_model, integrator.u, integrator.p, integrator.t, sat_idx)
    end
    return nothing
end

# The burn's bookkeeping for spacecraft `sat_idx` (its window, burn flag and trace)
# goes on while the spacecraft is inactive, but a released command stays released
# (`_released`): no scheduling pass or burn edge turns its mounted engine back on.
@inline function _refresh_burn_keeping_release!(control_model::BaseThrusterModel, p, sat_idx::Int, t)
    refresh_burn!(control_model, p, sat_idx, t)
    _released(control_model, p, sat_idx) && release_command!(control_model, t, sat_idx)
    return nothing
end

@inline function _schedule_thruster_control!(integrator, control_model::BaseThrusterModel, sat_idx::Int)
    _run_guidance_for_thruster_schedule!(integrator, sat_idx)
    _gnc_tick!(_update_control!, control_model, integrator.u, integrator.p, integrator.t, sat_idx)
    _register_control_tstops!(integrator, control_model, sat_idx)
    _refresh_burn_keeping_release!(control_model, integrator.p, sat_idx, integrator.t)
    return nothing
end

# A burn scheduler in `:mounted` actuation also ticks periodically: it points the
# engine with the mounted wheels and applies the mounted hinges and firing
# dynamics, as other mounted effectors do after their ticks.
function _mounted_burn_tick_callback(control_model::BaseThrusterModel, period::Float64, num_sats::Int)
    tick! = integrator -> begin
        @inbounds for sat_idx in 1:num_sats
            # As every periodic tick (`_gnc_tick!`): an inactive spacecraft's command
            # is released instead, and the mounted actuators then apply it.
            _gnc_tick!(mounted_burn_tick!, control_model, integrator.u, integrator.p, integrator.t, sat_idx) ||
                continue
            _apply_mounted_tick!(control_model, integrator.p, sat_idx, integrator.t, period)
        end
        return nothing
    end
    return _run_clock_periodic_callback(tick!, period)
end

# A burn switches only between integration steps: the solver stops at the
# window's ends (registered as stops when the burn is set) and this callback
# switches the burn there. A discrete callback counts as a derivative
# discontinuity unless it clears the flag, so the solver evaluates the
# derivative again, with the new thrust, before the next step; the callback
# must not call `derivative_discontinuity!(integrator, false)`, which would keep
# the stale one. The callback also takes up a plan installed in the burn-plan
# buffer since the scheduler's last pass: it adds the plan's window ends as stops
# and records it, at the end of the step it was installed in.
function _burn_edge_callback(control_model::BaseThrusterModel, num_sats::Int)
    condition = (u, t, integrator) -> burn_edge(control_model, t) || _burn_window_pending(control_model, integrator.p)
    affect! = integrator -> begin
        @inbounds for sat_idx in 1:num_sats
            _burn_window_changed(control_model, integrator.p, sat_idx) &&
                _register_control_tstops!(integrator, control_model, sat_idx)
            _refresh_burn_keeping_release!(control_model, integrator.p, sat_idx, integrator.t)
        end
        return nothing
    end
    return DiscreteCallback(condition, affect!; save_positions=(false, false))
end

# When spacecraft `sat_idx` leaves the atmosphere, the event-driven control
# models plan again; the burn scheduler schedules its next burn.
_reschedule_event_driven!(integrator, control_model, sat_idx::Int) = nothing
_reschedule_event_driven!(integrator, control_model::BaseThrusterModel, sat_idx::Int) =
    _schedule_thruster_control!(integrator, control_model, sat_idx)

function schedule_event_driven_thruster_controls!(integrator, sat_idx::Int)
    @inbounds for control_model in integrator.p.args.control_model.control_effectors
        if tick_schedule(control_model) === :event_driven
            _reschedule_event_driven!(integrator, control_model, sat_idx)
        end
    end
    return nothing
end

function _thruster_schedule_callbacks(control_model::BaseThrusterModel, num_sats::Int, args::SimulationConfiguration)
    n_slots = length(control_model.thrust)
    if n_slots != num_sats
        throw(ArgumentError(
            "BaseThrusterModel vector length ($n_slots) must match number of spacecraft ($num_sats). " *
            "Use one shared model with per-spacecraft vectors."
        ))
    end

    function schedule_all!(integrator)
        @inbounds for sat_idx in 1:num_sats
            _schedule_thruster_control!(integrator, control_model, sat_idx)
        end
        return nothing
    end

    init_callback = DiscreteCallback(
        (u, t, integrator) -> false,
        schedule_all!;
        initialize=(cb, u, t, integrator) -> begin
            schedule_all!(integrator)
            # A burn already on at the start: the first derivative must thrust.
            if any(control_model.burning) && applicable(DiffEqBase.derivative_discontinuity!, integrator, true)
                DiffEqBase.derivative_discontinuity!(integrator, true)
            end
        end,
        save_positions=(false, false)
    )
    return (init_callback,)
end

# The run-time declarations of the built-in control models (GNCRuntime). The
# burn scheduler is event-driven: its callbacks schedule each burn, switch it at
# the window's ends and, in `:mounted` actuation, tick the mounted actuators at
# the control rate.
GNCRuntime.tick_schedule(::BaseThrusterModel) = :event_driven
function GNCRuntime.gnc_callbacks(control_model::BaseThrusterModel, num_sats, args, period_s)
    callbacks = Any[_thruster_schedule_callbacks(control_model, num_sats, args)...]
    push!(callbacks, _burn_edge_callback(control_model, num_sats))
    actuation_mode(control_model) === :mounted &&
        push!(callbacks, _mounted_burn_tick_callback(control_model, Float64(period_s), num_sats))
    return Tuple(callbacks)
end
GNCRuntime.register_tstops!(integrator, control_model::BaseThrusterModel, spacecraft_index) =
    _register_control_tstops!(integrator, control_model, spacecraft_index)
# The EDG controller keeps the clock of its own ticks in the EDG state, where its
# predictions read it; recording the actual start matches the periodic callback's
# phase on resumes.
function GNCRuntime.gnc_clock_started!(control_model::_simulation_model_module.AerobrakingEnergyDepletionControlModel,
                                       period_s, origin_s)
    control_model.state.control_clock_period_s = Float64(period_s)
    control_model.state.control_clock_origin_s = Float64(origin_s)
    return nothing
end
# The two model-predictive controllers tick at their control step.
GNCRuntime.gnc_period_s(control_model::_simulation_model_module.RPOMPCControlModel) = control_model.control_dt_s
GNCRuntime.gnc_period_s(control_model::_simulation_model_module.RobotArmControlEffector) = control_model.control_dt_s

# The step a control model's controller was discretized with, when it has one.
_discretization_step(control_model) = nothing
_discretization_step(control_model::_simulation_model_module.RPOMPCControlModel) =
    control_model.controller isa _simulation_model_module.RpoLQMPCController ? control_model.controller.dt_s : nothing
_discretization_step(control_model::_simulation_model_module.RobotArmControlEffector) =
    control_model.controller isa _simulation_model_module.RobotArmJointMPCController ? control_model.controller.dt_s : nothing

"""
    check_control_periods(args) -> nothing

Before a run: refuse it with an `ArgumentError` when a periodic control model's
declared period (`gnc_period_s`) differs from the rate of its callback, or when
its controller was discretized with a different step. Equality is exact, since
builders set both from one value. (SpaceAGORA 0.3 warned instead.)
"""
function check_control_periods(args)
    models, rates = args.control_model.control_effectors, args.control_model.control_rates
    for (i, control_model) in enumerate(models)
        tick_schedule(control_model) === :periodic || continue
        T = typeof(control_model)
        period = gnc_period_s(control_model)
        period !== nothing && Float64(period) != Float64(rates[i]) && throw(ArgumentError(
            "$(nameof(T)) declares a $(period) s period (gnc_period_s) but its callback runs every " *
            "$(rates[i]) s (control_rates[$(i)]). Set both from one value."))
        step = _discretization_step(control_model)
        period !== nothing && step !== nothing && step != Float64(period) && throw(ArgumentError(
            "$(nameof(T)) ticks every $(period) s but its controller was discretized with a $(step) s step. " *
            "Build the controller with the model's control step."))
    end
    return nothing
end

# Links of the built-in models (GNCRuntime). The RPO plan buffer is written by the
# planner (its method lives with the planner) or the deprecated RPOGuidanceModel,
# and read by the RPO MPC controller. The energy-depletion state is shared by the
# EDG guidance and control; the link initializes it once per run.
GNCRuntime.gnc_links(g::_simulation_model_module.RPOGuidanceModel) = (Produces(g.plan_buffer),)
GNCRuntime.gnc_links(c::_simulation_model_module.RPOMPCControlModel) = (Consumes(c.plan_buffer),)
GNCRuntime.gnc_links(g::_simulation_model_module.AerobrakingEnergyDepletionGuidanceModel) = (Produces(g.state),)
GNCRuntime.gnc_links(c::_simulation_model_module.AerobrakingEnergyDepletionControlModel) = (Consumes(c.state),)
GNCRuntime.link_initialize!(s::_simulation_model_module.AerobrakingEnergyDepletionState, producers, consumers, u, p, t) =
    _simulation_model_module.GuidanceHooks._edg_initialize_state!(s)

"""
    gnc_link_groups(args) -> Vector

The run's links, in configuration order: for each shared object that a guidance,
navigation or control model declares in `gnc_links`, the models that produce it
and those that consume it, each in configuration order. Links are grouped by
identity.
"""
function gnc_link_groups(args)
    order = Any[]
    members = IdDict()                       # link => (producers, consumers), by identity
    models = (args.guidance_model.guidance_effectors..., args.navigation_model.navigation_effectors...,
              args.control_model.control_effectors...)
    for model in models, declared in gnc_links(model)
        declared isa Union{Produces, Consumes} || throw(ArgumentError(
            "gnc_links($(nameof(typeof(model)))) must return Produces and Consumes wrappers, not $(typeof(declared))."))
        link = declared.link
        haskey(members, link) || (members[link] = (Any[], Any[]); push!(order, link))
        push!(declared isa Produces ? members[link][1] : members[link][2], model)
    end
    return [(link=link, producers=members[link][1], consumers=members[link][2]) for link in order]
end

"""
    check_gnc_links(args) -> nothing

Before a run: refuse a link whose numbers of producers and consumers fall outside
its `link_cardinality`, then run each link's `check_link`.
"""
function check_gnc_links(args)
    for group in gnc_link_groups(args)
        limits = link_cardinality(group.link)
        T = nameof(typeof(group.link))
        for (role, models, allowed) in (("produce", group.producers, limits.producers),
                                        ("consume", group.consumers, limits.consumers))
            length(models) in allowed || throw(ArgumentError(
                "$(length(models)) models $(role) one $(T); $(allowed) may: " *
                join(unique(string.(nameof.(typeof.(models)))), ", ") * "."))
        end
        check_link(group.link, group.producers, group.consumers, args)
    end
    return nothing
end

# At the run's start, before the models' own initializers: each periodic control
# effector learns the clock its callback reports at the solve's start (its configured
# rate, from the grid's origin `origin`), so an initializer that predicts before the
# solve has it (the EDG guidance family's first request).
function _report_gnc_clocks!(args, origin)
    model = args.control_model
    for i in eachindex(model.control_effectors)
        control = model.control_effectors[i]
        tick_schedule(control) === :periodic || continue
        gnc_clock_started!(control, Float64(model.control_rates[i]), Float64(origin))
    end
    return nothing
end

"""
    initialize_gnc_links!(args, u, p, t) -> nothing

At a run's start, before the models' own initializers: each link's
`link_initialize!`, once.
"""
function initialize_gnc_links!(args, u, p, t)
    for group in gnc_link_groups(args)
        link_initialize!(group.link, group.producers, group.consumers, u, p, t)
    end
    return nothing
end

# The models that produce a link `control_model` consumes, in configuration order.
function _linked_producers(control_model, args)
    consumed = [declared.link for declared in gnc_links(control_model) if declared isa Consumes]
    isempty(consumed) && return ()
    producers = Any[]
    for group in gnc_link_groups(args), producer in group.producers
        any(link -> link === group.link, consumed) || continue
        any(model -> model === producer, producers) || push!(producers, producer)
    end
    return Tuple(producers)
end

function get_control_callbacks(num_sats::Int, args::SimulationConfiguration)
    # Perform the control effects' calculations at specific rates given by the control_rates field in the ControlModel
    control_models = args.control_model.control_effectors
    control_rates = args.control_model.control_rates
    callbacks = Any[]
    for i in eachindex(control_models)
        control_model = control_models[i]
        control_rate = control_rates[i]
        schedule_kind = tick_schedule(control_model)
        schedule_kind in (:periodic, :event_driven) || throw(ArgumentError(
            "tick_schedule($(nameof(typeof(control_model)))) must return :periodic or :event_driven, not $(repr(schedule_kind))."))
        if schedule_kind === :event_driven
            append!(callbacks, gnc_callbacks(control_model, num_sats, args, control_rate))
            continue
        end
        # Each control effector callback runs at its own rate and updates
        # all spacecraft states. The spacecraft index is passed explicitly
        # to avoid conflating effector-index with spacecraft-index.
        # A spacecraft outside the effector's scope gets neither the tick nor the
        # mounted actuators' update (`_gnc_tick!`).
        apply_control! = (integrator, sat_idx) -> begin
            _gnc_tick!(_update_control!, control_model, integrator.u, integrator.p, integrator.t, sat_idx) ||
                return nothing
            _apply_mounted_tick!(control_model, integrator.p, sat_idx, integrator.t, Float64(control_rate))
        end
        # The models that write what this one reads (its links) prepare it first,
        # once per tick and on this thread: the RPO planner checks its reference.
        # Not when the tick releases or skips every spacecraft (`_ticks_any`).
        producers = _linked_producers(control_model, args)
        control_func = (integrator) -> begin
            if !isempty(producers) && _ticks_any(control_model, integrator.p, num_sats)
                for producer in producers
                    SimulationLifecycle.before_reference_control!(producer, control_model, integrator.u, integrator.p,
                        integrator.t)
                end
            end
            decision = _control_callback_thread_decision(integrator.p, control_model, num_sats)
            use_threads = decision.use_threads
            started_ns = time_ns()
            if use_threads
                ParallelPolicy.threaded_foreach_persistent(:control_callback, num_sats, decision.allotment) do sat_idx
                    @inbounds apply_control!(integrator, sat_idx)
                end
            else
                @inbounds for sat_idx in 1:num_sats
                    apply_control!(integrator, sat_idx)
                end
            end
            # The solver's stop list belongs to the integrator that all spacecraft
            # share, so stops are registered one spacecraft at a time, after the
            # ticks, even when the ticks ran on several threads.
            @inbounds for sat_idx in 1:num_sats
                register_tstops!(integrator, control_model, sat_idx)
            end
            if decision.policy_applied
                ParallelPolicy.record_policy_observation!(
                    :control_callback;
                    mode=decision.mode,
                    num_items=num_sats,
                    use_threads=use_threads,
                    elapsed_ns=(time_ns() - started_ns),
                    env=_policy_env_config(integrator.p),
                    ctx=ParallelPolicy.policy_context_hint(integrator.p)
                )
            end
        end
        # At each solve's start the model learns its clock's period and origin:
        # the solve's start, or the run's in a checkpointed or resumed run.
        start_clock = (origin, integrator) -> gnc_clock_started!(control_model, Float64(control_rate), origin)
        push!(callbacks, _run_clock_periodic_callback(control_func, Float64(control_rate); on_clock=start_clock))
    end
    return callbacks
end
