@inline _checkpoint_directory(args) = SimulationModel.IOConfig._checkpoint_directory(args)
@inline _checkpoint_paths(args) = SimulationModel.IOConfig._checkpoint_paths(args)

# Keep feature identity and lifecycle knowledge in the simulation engine. The
# file layer stores this plain-data snapshot without importing interlink models.
function _interlink_checkpoint_configuration(args)
    model = args.interlink_model
    graph = model.graph
    policy = args.scheduling_policy_model
    return (
        link_type=model.constellation_link_type,
        vertices=copy(graph.vertices),
        candidates=[(
            endpoints=map(vertex -> graph.vertices[vertex], edge),
            P=graph.edge_properties[index].P,
            B=graph.edge_properties[index].B,
            range=graph.edge_properties[index].range,
        ) for (index, edge) in enumerate(graph.edges)],
        forbidden_pairs=sort!(collect(model.forbidden_pairs)),
        policy=policy.policy,
        target_idx=policy.target_idx,
        target_idxs=policy.target_idxs === nothing ? nothing : copy(policy.target_idxs),
        terminal_counts=[spacecraft.n_terminal for spacecraft in args.dynamics_model.spacecraft],
    )
end

function _interlink_checkpoint_runtime_state(args, is_active)
    args.interlink_model === nothing && return nothing
    count = length(args.dynamics_model.spacecraft)
    is_active isa AbstractVector{Bool} && length(is_active) == count ||
        throw(ArgumentError("Writing an interlink checkpoint requires one activity flag per spacecraft."))
    return (interlinks=(
        version=1,
        configuration=_interlink_checkpoint_configuration(args),
        is_active=Vector{Bool}(is_active),
    ),)
end

function _restore_interlink_checkpoint_state!(args, checkpoint, is_active)
    runtime = get(checkpoint, :runtime_state, nothing)
    snapshot = runtime !== nothing && hasproperty(runtime, :interlinks) ? runtime.interlinks : nothing
    if args.interlink_model === nothing
        snapshot === nothing || throw(ArgumentError(
            "An interlink checkpoint cannot resume with interlink_model=nothing."))
        return nothing
    end
    snapshot !== nothing && hasproperty(snapshot, :version) && snapshot.version === 1 ||
        throw(ArgumentError("Interlink checkpoint runtime metadata version 1 is required; " *
            "older checkpoints without this metadata cannot safely resume an interlinked run."))
    hasproperty(snapshot, :configuration) &&
        isequal(snapshot.configuration, _interlink_checkpoint_configuration(args)) ||
        throw(ArgumentError("Interlink configuration differs from the checkpoint. Resume with the same " *
            "candidate topology, laser parameters, forbidden pairs, policy targets and terminal counts."))
    count = length(args.dynamics_model.spacecraft)
    hasproperty(snapshot, :is_active) && snapshot.is_active isa AbstractVector{Bool} &&
        length(snapshot.is_active) == count && is_active isa AbstractVector{Bool} &&
        length(is_active) == count ||
        throw(ArgumentError("Interlink checkpoint activity flags must match the spacecraft count."))
    hasproperty(checkpoint.u, :sc) && length(checkpoint.u.sc) == count ||
        throw(ArgumentError("Interlink checkpoint state must match the spacecraft count."))
    copyto!(is_active, snapshot.is_active)
    return nothing
end

# Which spacecraft are active: an impact or a touchdown deactivates one for the
# rest of the run. Written with every checkpoint and restored before a resumed
# run's initializers, so a spacecraft inactive at the checkpoint stays inactive:
# its state is not integrated, the GNC models that skip inactive spacecraft do
# not tick for it, and its impact and touchdown do not fire again (both act on
# active spacecraft only, and the touchdown's start check skips inactive ones).
# An interlink checkpoint also carries the flags in its own entry; the two must
# agree. A checkpoint without the entry, written by an earlier SpaceAGORA,
# resumes with every spacecraft active, except in an interlinked run, which
# restores its entry's flags.
_activity_checkpoint_state(is_active) = (version=1, is_active=Vector{Bool}(is_active))

function _restore_activity_checkpoint_state!(args, checkpoint, is_active)
    runtime = get(checkpoint, :runtime_state, nothing)
    entry = runtime !== nothing && hasproperty(runtime, :activity) ? runtime.activity : nothing
    entry === nothing && return nothing
    hasproperty(entry, :version) && entry.version === 1 ||
        throw(ArgumentError("Spacecraft activity checkpoint state version 1 is required to resume this run."))
    count = length(args.dynamics_model.spacecraft)
    hasproperty(entry, :is_active) && entry.is_active isa AbstractVector{Bool} &&
        length(entry.is_active) == count && length(is_active) == count ||
        throw(ArgumentError("The checkpoint's spacecraft activity flags must match the spacecraft count."))
    interlinks = hasproperty(runtime, :interlinks) ? runtime.interlinks : nothing
    interlinks === nothing || !hasproperty(interlinks, :is_active) || interlinks.is_active == entry.is_active ||
        throw(ArgumentError("The checkpoint's spacecraft activity flags differ from its interlink entry's."))
    copyto!(is_active, entry.is_active)
    return nothing
end

# The run state the callbacks keep on the spacecraft records: the batteries'
# flags, the faults' activity and held levels, the sensors' streams, latest
# measurements and gyro biases, and the hinges' and gimbals' angles. A plain-data
# snapshot written with every checkpoint and restored, after checking it belongs
# to the same configuration, before a resumed run's callbacks initialize.
_record_faults(sc) = sc.root.faults.faults
_record_hinges(sc) = [(link, h) for link in sc.links for h in link.hinges]
_record_gimbals(sc) = [(link, g) for link in sc.links for g in link.gimbals]

# Runs whose resume needs the snapshot (their resume was refused before it
# existed), and runs with articulated actuators whose angles it also carries.
_needs_callback_state(sc) = SimulationModel.battery(sc) !== nothing || !isempty(_record_faults(sc)) ||
    !isempty(SimulationModel.mounted_sensors(sc))
_has_callback_state(sc) = _needs_callback_state(sc) || !isempty(_record_hinges(sc)) || !isempty(_record_gimbals(sc))

# Each record with where it sits: a fault with its actuator's position in the
# mounting order (as `resolve_faults!` finds it), a sensor, hinge or gimbal with
# its link's position, a gimbal also with its thruster's on that link, so a
# record moved to another device of the same kind refuses the snapshot.
_link_position(sc, link) = findfirst(l -> l === link, sc.links)
_mounting_position(sc, actuator) = findfirst(e -> e.actuator === actuator, SimulationModel.mounted_actuators(sc))

function _callback_checkpoint_configuration(args)
    return [(
        battery=(b = SimulationModel.battery(sc); b === nothing ? nothing : (b.capacity_wh, b.recovery_soc)),
        faults=[(nameof(typeof(f.actuator)), _mounting_position(sc, f.actuator), f.kind, f.start_s, f.stop_s, f.value)
                for f in _record_faults(sc)],
        sensors=[(nameof(typeof(e.sensor)), _link_position(sc, e.link), e.sensor.period_s, e.sensor.seed)
                 for e in SimulationModel.mounted_sensors(sc)],
        hinges=[_link_position(sc, link) for (link, _) in _record_hinges(sc)],
        gimbals=[(_link_position(sc, link), findfirst(t -> t === g.thruster, link.thrusters)) for (link, g) in _record_gimbals(sc)],
    ) for sc in args.dynamics_model.spacecraft]
end

function _callback_checkpoint_state(args)
    spacecraft = args.dynamics_model.spacecraft
    any(_has_callback_state, spacecraft) || return nothing
    return (
        version=1,
        configuration=_callback_checkpoint_configuration(args),
        spacecraft=[(
            battery=(b = SimulationModel.battery(sc); b === nothing ? nothing : (full=b.full, empty=b.empty, shed_tier=b.shed_tier)),
            faults=[(active=f.active, held_level=f.held_level) for f in _record_faults(sc)],
            sensors=[(rng=(s = e.sensor; s.rng === nothing ? nothing : copy(s.rng)), latest=deepcopy(e.sensor.latest),
                      latest_time_s=e.sensor.latest_time_s, bias=hasfield(typeof(e.sensor), :bias) ? e.sensor.bias : nothing)
                     for e in SimulationModel.mounted_sensors(sc)],
            hinges=[(angle_rad=h.angle_rad, last_update_s=h.last_update_s) for (_, h) in _record_hinges(sc)],
            gimbals=[(angle_rad=g.angle_rad, last_update_s=g.last_update_s, previous_angle_rad=g.previous_angle_rad)
                     for (_, g) in _record_gimbals(sc)],
        ) for sc in spacecraft],
    )
end

function _restore_callback_checkpoint_state!(args, checkpoint, p)
    spacecraft = args.dynamics_model.spacecraft
    runtime = get(checkpoint, :runtime_state, nothing)
    snapshot = runtime !== nothing && hasproperty(runtime, :callbacks) ? runtime.callbacks : nothing
    # A snapshot is always checked, so a run without the devices it holds
    # refuses it rather than dropping them.
    if snapshot === nothing
        any(_has_callback_state, spacecraft) || return nothing
        any(_needs_callback_state, spacecraft) && throw(ArgumentError(
            "This run has batteries, actuator faults or sensors, whose run state this checkpoint does not hold " *
            "(it was written by an earlier SpaceAGORA or by another configuration). Run from the start."))
        @warn "The checkpoint holds no hinge or gimbal angles; the resumed run starts them from their current values."
        return nothing
    end
    hasproperty(snapshot, :version) && snapshot.version === 1 ||
        throw(ArgumentError("Callback checkpoint state version 1 is required to resume this run."))
    hasproperty(snapshot, :configuration) && isequal(snapshot.configuration, _callback_checkpoint_configuration(args)) ||
        throw(ArgumentError("The batteries, actuator faults, sensors, hinges or gimbals differ from the checkpoint's. " *
            "Resume with the same configuration, or run from the start."))
    length(snapshot.spacecraft) == length(spacecraft) ||
        throw(ArgumentError("The callback checkpoint state must match the spacecraft count."))
    # Checked above: from here on the restore cannot fail part-way.
    for (sc, saved) in zip(spacecraft, snapshot.spacecraft)
        b = SimulationModel.battery(sc)
        if b !== nothing
            b.full, b.empty, b.shed_tier = saved.battery.full, saved.battery.empty, saved.battery.shed_tier
        end
        for (f, s) in zip(_record_faults(sc), saved.faults)
            f.active, f.held_level = s.active, s.held_level
        end
        for (e, s) in zip(SimulationModel.mounted_sensors(sc), saved.sensors)
            sensor = e.sensor
            sensor.rng = s.rng === nothing ? nothing : copy(s.rng)
            sensor.latest = deepcopy(s.latest)
            sensor.latest_time_s = s.latest_time_s
            s.bias === nothing || (sensor.bias = s.bias)
        end
        for ((link, h), s) in zip(_record_hinges(sc), saved.hinges)
            h.angle_rad, h.last_update_s = s.angle_rad, s.last_update_s
            SimulationModel.ActuatorMounting._set_hinge_attitude!(link, h, h.angle_rad)
        end
        for ((link, g), s) in zip(_record_gimbals(sc), saved.gimbals)
            g.angle_rad, g.last_update_s, g.previous_angle_rad = s.angle_rad, s.last_update_s, s.previous_angle_rad
            SimulationModel.ActuatorMounting._update_gimbal_rotation!(link, g.thruster)
        end
    end
    p.shared_buffers.callback_resume[] = true
    return nothing
end

# The run state the models keep themselves (`checkpoint_state`): the guidance,
# navigation and control models in the configuration's order, then each
# spacecraft's sensors in mounting order. Written next to the callbacks' snapshot
# when any of them keeps some, with each model's kind at its position; restored,
# after checking those match the run's, once the run-start initializers ran.
function _checkpoint_models(args)
    return (
        navigation=collect(Any, args.navigation_model.navigation_effectors),
        guidance=collect(Any, args.guidance_model.guidance_effectors),
        control=collect(Any, args.control_model.control_effectors),
        sensors=[Any[e.sensor for e in SimulationModel.mounted_sensors(sc)] for sc in args.dynamics_model.spacecraft],
    )
end

_each_model(groups) = Iterators.flatten((groups.navigation, groups.guidance, groups.control, Iterators.flatten(groups.sensors)))
_model_kinds(group) = [nameof(typeof(m)) for m in group]
_model_checkpoint_configuration(models) = (navigation=_model_kinds(models.navigation),
    guidance=_model_kinds(models.guidance), control=_model_kinds(models.control),
    sensors=[_model_kinds(group) for group in models.sensors])

_missing_restore(model) = ArgumentError("$(nameof(typeof(model))) returns run state from checkpoint_state but has no " *
    "restore_checkpoint_state! method for it; implement SpaceAGORA.restore_checkpoint_state!(::$(nameof(typeof(model))), data), " *
    "or return nothing from checkpoint_state.")

# A checkpointed run's models that keep state can put it back.
function _check_model_checkpoint_hooks(args)
    for model in _each_model(_checkpoint_models(args))
        data = SimulationLifecycle.checkpoint_state(model)
        data === nothing || applicable(SimulationLifecycle.restore_checkpoint_state!, model, data) ||
            throw(_missing_restore(model))
    end
    return nothing
end

function _model_checkpoint_state(args)
    models = _checkpoint_models(args)
    saved = map(group -> Any[SimulationLifecycle.checkpoint_state(m) for m in group],
        (navigation=models.navigation, guidance=models.guidance, control=models.control))
    sensors = [Any[SimulationLifecycle.checkpoint_state(m) for m in group] for group in models.sensors]
    all(data -> data === nothing, _each_model((saved..., sensors=sensors))) && return nothing
    return (version=1, configuration=_model_checkpoint_configuration(models), saved..., sensors=sensors)
end

# Checked when the checkpoint is read, before anything is restored; `nothing`
# when the checkpoint holds no model state.
function _check_model_checkpoint_state(args, checkpoint)
    runtime = get(checkpoint, :runtime_state, nothing)
    snapshot = runtime !== nothing && hasproperty(runtime, :models) ? runtime.models : nothing
    snapshot === nothing && return nothing
    hasproperty(snapshot, :version) && snapshot.version === 1 ||
        throw(ArgumentError("Model checkpoint state version 1 is required to resume this run."))
    models = _checkpoint_models(args)
    hasproperty(snapshot, :configuration) && isequal(snapshot.configuration, _model_checkpoint_configuration(models)) ||
        throw(ArgumentError("The guidance, navigation or control models or the sensors differ from the checkpoint's. " *
            "Resume with the same configuration, or run from the start."))
    for (model, data) in zip(_each_model(models), _each_model(snapshot))
        data === nothing || applicable(SimulationLifecycle.restore_checkpoint_state!, model, data) ||
            throw(_missing_restore(model))
    end
    return snapshot
end

# After the run-start initializers of a resumed run. Before anything is
# restored, a model that keeps run state now and has none in the file is
# refused: no snapshot at all, or `nothing` saved at its position (another
# model wrote the snapshot, and this one kept no state then).
function _restore_model_checkpoint_state!(args, snapshot)
    models = collect(_each_model(_checkpoint_models(args)))
    saved = snapshot === nothing ? fill(nothing, length(models)) : collect(_each_model(snapshot))
    kept = unique(nameof(typeof(m)) for (m, data) in zip(models, saved)
                  if data === nothing && SimulationLifecycle.checkpoint_state(m) !== nothing)
    isempty(kept) || throw(ArgumentError("The run state of $(join(kept, ", ")) (checkpoint_state) is not in this checkpoint " *
        "(it was written by an earlier SpaceAGORA or by another configuration, or the model kept none then). Run from the start."))
    for (model, data) in zip(models, saved)
        data === nothing || SimulationLifecycle.restore_checkpoint_state!(model, data)
    end
    return nothing
end

# A checkpointed run's segment from `t` ends one interval later, or at the run's
# end when that comes first or is the same time to the periodic tick clock
# (`_same_run_time`): accumulated decimal intervals stop short of the end by an
# ulp (ten 0.1 s intervals reach 0.9999999999999999), and a last segment that
# short is a step the solver can reject and then abort as unstable.
function _checkpoint_segment_end(t::Float64, interval::Float64, mission_end::Float64)
    t_next = t + interval
    t_next >= mission_end && return mission_end
    return SimulationModel.SimulationCallbacks._same_run_time(t_next, mission_end) ? mission_end : t_next
end

# A checkpoint written at its run's end: the run did not take the periodic
# navigation, guidance and control ticks due then, so a run resumed from it takes
# them at its start. One written at a segment boundary holds no entry: the ticks
# due there came before it. So does one written where a solve stopped early, at an
# event's `terminate!` (impact, touchdown, orbit or entry count): the solver applies
# every discrete callback of that step after the event's, so a tick due then ran
# before the checkpoint, as in a run in one piece. "Due then" is up to the clock's
# rounding tolerance (`_same_run_time`) on both sides of the checkpoint.
_periodic_tick_checkpoint_state(t::Float64, mission_end) = t >= mission_end ? (version=1, pending_s=t) : nothing

# The resumed run's start when the ticks due there are pending, NaN otherwise.
function _pending_periodic_ticks_s(checkpoint)
    runtime = get(checkpoint, :runtime_state, nothing)
    entry = runtime !== nothing && hasproperty(runtime, :periodic_ticks) ? runtime.periodic_ticks : nothing
    entry === nothing && return NaN
    hasproperty(entry, :version) && entry.version === 1 && hasproperty(entry, :pending_s) &&
        entry.pending_s == checkpoint.t ||
        throw(ArgumentError("Periodic tick checkpoint state version 1 is required to resume this run."))
    return Float64(entry.pending_s)
end

@inline function _write_checkpoint!(args, t::Float64, u_state, solver_mode::String; is_active=nothing, callback_state=nothing,
                                     model_state=nothing, periodic_ticks=nothing)
    runtime = _interlink_checkpoint_runtime_state(args, is_active)
    if callback_state !== nothing
        runtime = runtime === nothing ? (callbacks=callback_state,) : merge(runtime, (callbacks=callback_state,))
    end
    if is_active !== nothing
        activity = _activity_checkpoint_state(is_active)
        runtime = runtime === nothing ? (activity=activity,) : merge(runtime, (activity=activity,))
    end
    if model_state !== nothing
        runtime = runtime === nothing ? (models=model_state,) : merge(runtime, (models=model_state,))
    end
    if periodic_ticks !== nothing
        runtime = runtime === nothing ? (periodic_ticks=periodic_ticks,) : merge(runtime, (periodic_ticks=periodic_ticks,))
    end
    return SimulationModel.IOSerialization._write_checkpoint!(
        args,
        t,
        u_state,
        CHECKPOINT_SCHEMA_VERSION;
        solver_mode=solver_mode,
        runtime_state=runtime
    )
end

@inline _load_checkpoint(args) = SimulationModel.IOSerialization._load_checkpoint(args)
@inline _clear_checkpoint!(args) = SimulationModel.IOSerialization._clear_checkpoint!(args)
