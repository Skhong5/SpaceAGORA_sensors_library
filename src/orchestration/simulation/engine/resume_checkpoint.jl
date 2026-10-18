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

function _callback_checkpoint_configuration(args)
    return [(
        battery=(b = SimulationModel.battery(sc); b === nothing ? nothing : (b.capacity_wh, b.recovery_soc)),
        faults=[(nameof(typeof(f.actuator)), f.kind, f.start_s, f.stop_s, f.value) for f in _record_faults(sc)],
        sensors=[(nameof(typeof(e.sensor)), e.sensor.period_s, e.sensor.seed) for e in SimulationModel.mounted_sensors(sc)],
        hinges=length(_record_hinges(sc)),
        gimbals=length(_record_gimbals(sc)),
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
    any(_has_callback_state, spacecraft) || return nothing
    runtime = get(checkpoint, :runtime_state, nothing)
    snapshot = runtime !== nothing && hasproperty(runtime, :callbacks) ? runtime.callbacks : nothing
    if snapshot === nothing
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

@inline function _write_checkpoint!(args, t::Float64, u_state, solver_mode::String; is_active=nothing, callback_state=nothing)
    runtime = _interlink_checkpoint_runtime_state(args, is_active)
    if callback_state !== nothing
        runtime = runtime === nothing ? (callbacks=callback_state,) : merge(runtime, (callbacks=callback_state,))
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
