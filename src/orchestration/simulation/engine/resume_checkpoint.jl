# The run state a checkpoint holds next to the integrated state, written with every
# checkpoint (`_write_checkpoint!`) and read back when a run resumes. Its entries,
# in the order a resumed run uses them (see "Run state: owners, resets and
# restores" in docs/contributing/architecture.md):
#
# - `activity`, `interlinks` and `callbacks`: the engine's and the callbacks' run
#   state, restored by `_resume_engine_state!` (execution.jl) before the models
#   start and before the callbacks initialize;
# - `models`: the models' own run state, checked when the checkpoint is read and
#   restored by `_start_run_models!` after the models' initializers;
# - `gnc_scopes`: the spacecraft each guidance, navigation and control model acts
#   on, compared by `_start_run_models!` after the initializers and before the
#   models' run state is restored;
# - `periodic_ticks`: the periodic ticks not yet taken at the checkpoint's time,
#   which a resumed run takes at its start (`_start_run_clock!`).
#
# The definitions below follow that order, then the segment ends and the file.

@inline _checkpoint_directory(args) = SimulationModel.IOConfig._checkpoint_directory(args)
@inline _checkpoint_paths(args) = SimulationModel.IOConfig._checkpoint_paths(args)

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

# Keep feature identity and lifecycle knowledge in the simulation engine. The
# file layer stores this plain-data snapshot without importing interlink models.
# A link's identity is its kind and its fields. The kind is named by the full
# path of the module that defines it and its name, so same-named kinds of two
# packages, or of two modules of one package, differ; a kind moved to another
# module needs its old checkpoints migrated. Its concrete type parameters are
# part of it, since they can select different hooks (`ModeLink{1}` and
# `ModeLink{-1}`).
function _interlink_link_identity(link)
    kind = typeof(link)
    names = fieldnames(kind)
    return (kind=_interlink_kind_identity(kind),
        fields=NamedTuple{names}(map(name -> getfield(link, name), names)))
end

# A type as plain data that the checkpoint file serializes: the full path of
# its module, its name and its parameters. A parameter that is a type is
# described the same way, and a tuple parameter element by element. A value
# parameter (a number, symbol or other bits value, the only values a type
# parameter can hold) is recorded with its own type, since `1`, `1.0`, `true`
# and `Int32(1)` select different methods yet compare equal as numbers. A
# floating-point value also keeps its bits, so that -0.0 and 0.0, and NaNs with
# different payloads, stay apart as they do in types. A composite bits value (a
# `ComplexF64`, a struct of numbers) is recorded field by field, the same way:
# its own `isequal` can merge values that are distinct types (two NaN payloads
# in one `Complex`), and its padding bytes carry no meaning.
_interlink_kind_identity(kind::DataType) = (module_path=fullname(parentmodule(kind)), name=nameof(kind),
    parameters=map(_interlink_kind_parameter, Tuple(kind.parameters)))
_interlink_kind_parameter(parameter::DataType) = _interlink_kind_identity(parameter)
_interlink_kind_parameter(parameter::Union) = (union=map(_interlink_kind_parameter, Tuple(Base.uniontypes(parameter))),)
_interlink_kind_parameter(parameter::Type) = string(parameter)   # a UnionAll parameter: its printed form
_interlink_kind_parameter(parameter::Tuple) = map(_interlink_kind_parameter, parameter)
function _interlink_kind_parameter(parameter)
    kind = typeof(parameter)
    isbitstype(kind) && !isprimitivetype(kind) && return (type=_interlink_kind_identity(kind),
        fields=ntuple(index -> _interlink_kind_parameter(getfield(parameter, index)), fieldcount(kind)))
    return (type=_interlink_kind_identity(kind), value=parameter)
end
_interlink_kind_parameter(parameter::Base.IEEEFloat) = (type=_interlink_kind_identity(typeof(parameter)),
    value=parameter, bits=reinterpret(Base.uinttype(typeof(parameter)), parameter))

function _interlink_checkpoint_configuration(args)
    model = args.interlink_model
    graph = model.graph
    policy = args.scheduling_policy_model
    configuration = (
        vertices=copy(graph.vertices),
        candidates=[(
            endpoints=map(vertex -> graph.vertices[vertex], edge),
            link=_interlink_link_identity(graph.edge_properties[index]),
        ) for (index, edge) in enumerate(graph.edges)],
        forbidden_pairs=sort!(collect(model.forbidden_pairs)),
        policy=policy.policy,
        # The keys keep the names checkpoints were written with, so those resume.
        target_idx=policy.target_index,
        target_idxs=policy.target_indices === nothing ? nothing : copy(policy.target_indices),
        terminal_counts=[spacecraft.n_terminal for spacecraft in args.dynamics_model.spacecraft],
    )
    # Terminal numbers past `n_terminal` name mounted terminal hardware, so a run
    # with any records each spacecraft's terminals, in order; a run without keeps
    # the configuration it had before hardware terminals.
    hardware = [_mounted_terminal_identities(spacecraft) for spacecraft in args.dynamics_model.spacecraft]
    all(isempty, hardware) && return configuration
    return merge(configuration, (hardware_terminals=hardware,))
end

_mounted_terminal_identities(spacecraft::SimulationModel.SpacecraftModel) =
    Any[_interlink_terminal_identity(entry.terminal) for entry in SimulationModel.mounted_link_terminals(spacecraft)]
_mounted_terminal_identities(spacecraft) = Any[]

# A terminal's identity is its kind, named as link kinds are, and every one of
# its fields, encoded as type parameters are: a float with its bits, a composite
# bits value (an `SVector`) field by field. Its fields set what its links do (a
# receiver's aperture, efficiency and limit, a transmitter's efficiency and
# limits), and the pointing models to come will read the rest, so a rebuilt
# terminal with any other value refuses the checkpoint.
function _interlink_terminal_identity(terminal)
    kind = typeof(terminal)
    names = fieldnames(kind)
    return (kind=_interlink_kind_identity(kind),
        fields=NamedTuple{names}(map(name -> _interlink_kind_parameter(getfield(terminal, name)), names)))
end

function _interlink_checkpoint_runtime_state(args, is_active)
    args.interlink_model === nothing && return nothing
    count = length(args.dynamics_model.spacecraft)
    is_active isa AbstractVector{Bool} && length(is_active) == count ||
        throw(ArgumentError("Writing an interlink checkpoint requires one activity flag per spacecraft."))
    return (interlinks=(
        version=2,
        configuration=_interlink_checkpoint_configuration(args),
        is_active=Vector{Bool}(is_active),
    ),)
end

# A snapshot's configuration in the current form. Version 1 (before link kinds)
# held one laser link type for the run and P, B and range per candidate.
function _interlink_snapshot_configuration(snapshot)
    configuration = snapshot.configuration
    snapshot.version === 2 && return configuration
    hasproperty(configuration, :link_type) && configuration.link_type === :laser || return nothing
    candidates = [(endpoints=candidate.endpoints,
        link=_interlink_link_identity(SimulationModel.LaserMomentumLink(candidate.P, candidate.B, candidate.range)))
        for candidate in configuration.candidates]
    rest = (name => getproperty(configuration, name) for name in keys(configuration)
        if !(name in (:link_type, :vertices, :candidates)))
    return (; vertices=configuration.vertices, candidates=candidates, rest...)
end

function _restore_interlink_checkpoint_state!(args, checkpoint, is_active)
    runtime = get(checkpoint, :runtime_state, nothing)
    snapshot = runtime !== nothing && hasproperty(runtime, :interlinks) ? runtime.interlinks : nothing
    if args.interlink_model === nothing
        snapshot === nothing || throw(ArgumentError(
            "An interlink checkpoint cannot resume with interlink_model=nothing."))
        return nothing
    end
    snapshot !== nothing && hasproperty(snapshot, :version) && snapshot.version in (1, 2) ||
        throw(ArgumentError("Interlink checkpoint runtime metadata version 1 or 2 is required; " *
            "older checkpoints without this metadata cannot safely resume an interlinked run."))
    hasproperty(snapshot, :configuration) &&
        isequal(_interlink_snapshot_configuration(snapshot), _interlink_checkpoint_configuration(args)) ||
        throw(ArgumentError("Interlink configuration differs from the checkpoint. Resume with the same " *
            "candidate topology, link kinds and parameters, forbidden pairs, policy targets and terminal counts."))
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
# measurements and gyro biases, the hinges' and gimbals' angles, and the firing
# state of the mounted thrusters with firing dynamics (the ramp factor `κ` and
# the average thrust `thrust` their last control tick scheduled). A plain-data
# snapshot written with every checkpoint and restored, after checking it belongs
# to the same configuration, before a resumed run's callbacks initialize.
_record_faults(sc) = sc.root.faults.faults
_record_hinges(sc) = [(link, h) for link in sc.links for h in link.hinges]
_record_gimbals(sc) = [(link, g) for link in sc.links for g in link.gimbals]
_record_firing(sc) = [(link, t) for link in sc.links for t in link.thrusters if t.firing_dynamics]

# Runs whose resume needs the snapshot (their resume was refused before it
# existed, or its firing state is missing from it), and runs with articulated
# actuators whose angles it also carries.
_needs_callback_state(sc) = SimulationModel.battery(sc) !== nothing || !isempty(_record_faults(sc)) ||
    !isempty(SimulationModel.mounted_sensors(sc)) || !isempty(_record_firing(sc))
_has_callback_state(sc) = _needs_callback_state(sc) || !isempty(_record_hinges(sc)) || !isempty(_record_gimbals(sc))

# Version 2 added the thrusters' firing state, and the link of a faulted
# actuator. A version 1 snapshot (SpaceAGORA 0.4 and earlier) still resumes a run
# without firing dynamics; a run with them refuses it, since it cannot restore
# their firing state.
const CALLBACK_CHECKPOINT_STATE_VERSION = 2

# Each record with where it sits, so a record moved to another device of the same
# kind refuses the snapshot: a fault or a thruster with firing dynamics with its
# actuator's place (`_actuator_place`), a sensor, hinge or gimbal with its link's
# position, a gimbal also with its thruster's on that link. A thruster also with
# the parameters of its firing dynamics, which its saved state follows.
_link_position(sc, link) = findfirst(l -> l === link, sc.links)

# Where an actuator is mounted: its position in the mounting order (as
# `resolve_faults!` finds it), its link's position, and its position among the
# actuators on that link, in mounting order. The mounting order alone stays the
# same when an actuator moves to another link with nothing mounted between.
function _actuator_place(sc, actuator)
    entries = SimulationModel.mounted_actuators(sc)
    k = findfirst(e -> e.actuator === actuator, entries)
    k === nothing && return nothing
    link = entries[k].link
    return (k, _link_position(sc, link), count(e -> e.link === link, view(entries, 1:k)))
end

# The configuration a snapshot of `version` was written with. Version 1 had no
# thrusters, so a run with firing dynamics has none to compare (it is refused),
# and placed a faulted actuator by its position in the mounting order alone.
function _callback_checkpoint_configuration(args, version=CALLBACK_CHECKPOINT_STATE_VERSION)
    return [begin
        configuration = (
            battery=(b = SimulationModel.battery(sc); b === nothing ? nothing : (b.capacity_wh, b.recovery_soc)),
            faults=[(nameof(typeof(f.actuator)), (place = _actuator_place(sc, f.actuator); version === 1 ? first(place) : place),
                     f.kind, f.start_s, f.stop_s, f.value) for f in _record_faults(sc)],
            sensors=[(nameof(typeof(e.sensor)), _link_position(sc, e.link), e.sensor.period_s, e.sensor.seed)
                     for e in SimulationModel.mounted_sensors(sc)],
            hinges=[_link_position(sc, link) for (link, _) in _record_hinges(sc)],
            gimbals=[(_link_position(sc, link), findfirst(t -> t === g.thruster, link.thrusters)) for (link, g) in _record_gimbals(sc)],
        )
        version === 1 ? configuration : merge(configuration, (thrusters=[(_actuator_place(sc, t), t.max_thrust,
            t.cutoff_frequency, t.min_firing_time, t.level_on, t.level_off) for (_, t) in _record_firing(sc)],))
    end for sc in args.dynamics_model.spacecraft]
end

function _callback_snapshot_configuration(args, version)
    version === 1 && any(sc -> !isempty(_record_firing(sc)), args.dynamics_model.spacecraft) && throw(ArgumentError(
        "This run has thrusters with firing dynamics, whose firing state this checkpoint does not hold " *
        "(it was written by SpaceAGORA 0.4 or earlier). Run from the start."))
    return _callback_checkpoint_configuration(args, version)
end

function _callback_checkpoint_state(args)
    spacecraft = args.dynamics_model.spacecraft
    any(_has_callback_state, spacecraft) || return nothing
    return (
        version=CALLBACK_CHECKPOINT_STATE_VERSION,
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
            thrusters=[(κ=t.κ, thrust=t.thrust) for (_, t) in _record_firing(sc)],
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
            "This run has batteries, actuator faults, sensors or thrusters with firing dynamics, whose run state " *
            "this checkpoint does not hold (it was written by an earlier SpaceAGORA or by another configuration). " *
            "Run from the start."))
        @warn "The checkpoint holds no hinge or gimbal angles; the resumed run starts them from their current values."
        return nothing
    end
    version = hasproperty(snapshot, :version) ? snapshot.version : nothing
    version === 1 || version === CALLBACK_CHECKPOINT_STATE_VERSION ||
        throw(ArgumentError("Callback checkpoint state version 1 or $(CALLBACK_CHECKPOINT_STATE_VERSION) is required to resume this run."))
    hasproperty(snapshot, :configuration) && isequal(snapshot.configuration, _callback_snapshot_configuration(args, version)) ||
        throw(ArgumentError("The batteries, actuator faults, sensors, hinges, gimbals or thrusters with firing dynamics " *
            "differ from the checkpoint's. Resume with the same configuration, or run from the start."))
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
        # Version 1 holds no thrusters: the run has none with firing dynamics (checked).
        version === 1 && continue
        for ((_, t), s) in zip(_record_firing(sc), saved.thrusters)
            t.κ, t.thrust = s.κ, s.thrust
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

# The spacecraft each guidance, navigation and control model acts on, as its
# `spacecraft_scope` reads once the run-start initializers ran (the RPO planner
# and controller set their positions there, from the spacecraft ids). A
# checkpointed or resumed run takes this baseline after its initializers and
# writes it with every checkpoint (`gnc_scopes`). A resumed run compares its own
# baseline with the checkpoint's before any model's run state is restored, so
# saved state never goes back into a model that acts on other spacecraft, such as
# a momentum manager's held command into a manager of another spacecraft, or a
# bound copy declared on another spacecraft. Each group's scopes are compared in
# configuration order (the configuration's models, then each spacecraft's bound
# copies, as `_flatten_spacecraft_gnc` orders them). The models' types are
# compared with their run state (`models`), as before.
#
# A scope is plain data built afresh, so a selection changed in place later
# cannot change a baseline already taken:
# - `AllSpacecraft()`: `(scope=:all,)`;
# - `BoundSpacecraft`, or a `SpacecraftSelection`: `(scope=:bound, positions)`,
#   the run positions it resolves to, sorted and without repeats;
# - `CoupledSpacecraft`: `(scope=:coupled, primary, partners)`, as declared.
# Before the run, `check_spacecraft_scopes` refuses any other scope.
_gnc_scope_identity(::GNCRuntime.AllSpacecraft, spacecraft, owner) = (scope=:all,)
_gnc_scope_identity(scope::Union{GNCRuntime.BoundSpacecraft, SimulationModel.SpacecraftSelection}, spacecraft, owner) =
    (scope=:bound, positions=SimulationModel.resolve_spacecraft_scope(scope, spacecraft; owner=owner))
_gnc_scope_identity(scope::GNCRuntime.CoupledSpacecraft, spacecraft, owner) =
    (scope=:coupled, primary=scope.primary, partners=collect(Int, scope.partners))

const _GNC_SCOPE_GROUPS = (navigation="navigation", guidance="guidance", control="control")

_gnc_groups(args) = (navigation=args.navigation_model.navigation_effectors,
    guidance=args.guidance_model.guidance_effectors, control=args.control_model.control_effectors)

_gnc_scope_owner(group::Symbol, j::Int, model) = "$(_GNC_SCOPE_GROUPS[group]) model $j ($(nameof(typeof(model))))"

function _gnc_scope_baseline(args)
    spacecraft = args.dynamics_model.spacecraft
    groups = _gnc_groups(args)
    scopes(group) = Any[_gnc_scope_identity(GNCRuntime.spacecraft_scope(m), spacecraft, _gnc_scope_owner(group, j, m))
        for (j, m) in enumerate(getproperty(groups, group))]
    return (version=1, navigation=scopes(:navigation), guidance=scopes(:guidance), control=scopes(:control))
end

# The checkpoint's entry, checked for its form when the checkpoint is read, before
# anything is restored; `nothing` for a checkpoint written without one.
function _gnc_scope_checkpoint_entry(checkpoint)
    runtime = get(checkpoint, :runtime_state, nothing)
    entry = runtime !== nothing && hasproperty(runtime, :gnc_scopes) ? runtime.gnc_scopes : nothing
    entry === nothing && return nothing
    hasproperty(entry, :version) && entry.version === 1 && all(group -> hasproperty(entry, group) &&
        getproperty(entry, group) isa AbstractVector, keys(_GNC_SCOPE_GROUPS)) ||
        throw(ArgumentError("GNC scope checkpoint state version 1 is required to resume this run."))
    return entry
end

function _describe_gnc_scope(scope)
    scope.scope === :all && return "every spacecraft"
    scope.scope === :bound && return isempty(scope.positions) ? "no spacecraft" :
        "the spacecraft at run position$(length(scope.positions) == 1 ? "" : "s") $(join(scope.positions, ", "))"
    return "the spacecraft at run position $(scope.primary), coupled with run position" *
        "$(length(scope.partners) == 1 ? "" : "s") $(join(scope.partners, ", "))"
end

# A checkpoint written without the entry (by SpaceAGORA 0.5.1 or earlier) resumes
# as it did then, with no scope compared. A run with a model bound to some
# spacecraft is warned that its scopes were not checked; the checkpoints it
# writes record them.
function _check_gnc_scope_baseline(args, entry, baseline)
    if entry === nothing
        unchecked = [_gnc_scope_owner(group, j, m) for (group, models) in pairs(_gnc_groups(args))
            for (j, m) in enumerate(models) if getproperty(baseline, group)[j].scope !== :all]
        isempty(unchecked) || @warn "The checkpoint records no GNC spacecraft scopes (it was written by SpaceAGORA " *
            "0.5.1 or earlier), so the spacecraft of $(join(unchecked, ", ")) were not checked against it. Resume it " *
            "only with the configuration that wrote it."
        return nothing
    end
    for (group, models) in pairs(_gnc_groups(args))
        saved, current = getproperty(entry, group), getproperty(baseline, group)
        isequal(saved, current) && continue
        name = _GNC_SCOPE_GROUPS[group]
        length(saved) == length(current) || throw(ArgumentError(
            "The GNC models' spacecraft scopes differ from the checkpoint's: it has $(length(saved)) $name " *
            "model$(length(saved) == 1 ? "" : "s") and this run $(length(current)). Resume with the same models, " *
            "or run from the start."))
        j = findfirst(k -> !isequal(saved[k], current[k]), eachindex(current))
        owner = _gnc_scope_owner(group, j, models[j])
        throw(ArgumentError(
            "The GNC models' spacecraft scopes differ from the checkpoint's: $owner acts on " *
            "$(_describe_gnc_scope(current[j])) here and on $(_describe_gnc_scope(saved[j])) in the " *
            "checkpoint. Resume with the scopes the checkpoint was written with, or run from the start."))
    end
    return nothing
end

# Every checkpoint a run writes records, for each of the run's periodic
# navigation, guidance and control clocks (in the order of their callbacks), whether
# it had not yet taken the tick due at the checkpoint's time, up to the clock's
# rounding tolerance (`_same_run_time`, on both sides of that time). That is what
# each clock did, not a rule about the time: at a segment boundary the ticks due
# there ran before the checkpoint, and at the run's end they did not. Where a solve
# stopped early, at an event's `terminate!` (impact, touchdown, orbit or entry
# count) or a discrete callback's, a tick exactly then ran; one an ulp or so off ran
# only if its callback came after the termination (the solver applies the step's
# discrete callbacks after its events, in order), and none ran when the stop came
# just before the run's end, whose ticks the run leaves.
function _periodic_tick_checkpoint_state(p, t::Float64)
    pending = NamedTuple{(:clock, :period_s), Tuple{Int, Float64}}[]
    for (position, clock) in pairs(p.shared_buffers.periodic_clocks)
        next_tick = clock.origin[] + clock.index[] * clock.period_s
        SimulationModel.SimulationCallbacks._same_run_time(next_tick, t) &&
            push!(pending, (clock=position, period_s=clock.period_s))
    end
    return (version=2, time_s=t, pending=pending)
end

# The ticks a run resumed from `checkpoint` takes at its start, at `at_s` (the
# checkpoint's time; NaN when there are none), up to `_same_run_time`: those of the
# clocks the checkpoint lists, as (position, period) pairs, matched to the resumed
# run's clocks, or, when it lists none, every clock's tick from `from_s` on.
# Checkpoints written before the list (SpaceAGORA 0.4.0 and earlier) are read as
# that version left them: it compared tick times exactly, so it took a tick just
# before its checkpoint's time at that tick's own time and left one just after it
# (3 * 0.1 after a boundary at 0.3) to the next segment, where the solver could not
# reach it.
# - A version 1 entry (0.4.0, at the run's end): the ticks at the end and just after
#   it are pending.
# - No entry (at a segment boundary or an early stop): the ticks at that time ran
#   before the checkpoint, and those just after it are pending.
function _pending_periodic_ticks(checkpoint)
    t = Float64(checkpoint.t)
    runtime = get(checkpoint, :runtime_state, nothing)
    entry = runtime !== nothing && hasproperty(runtime, :periodic_ticks) ? runtime.periodic_ticks : nothing
    entry === nothing && return (at_s=t, from_s=nextfloat(t), clocks=nothing)
    version = hasproperty(entry, :version) ? entry.version : nothing
    if version === 2 && hasproperty(entry, :time_s) && hasproperty(entry, :pending) &&
            entry.time_s == t && entry.pending isa AbstractVector &&
            all(c -> hasproperty(c, :clock) && hasproperty(c, :period_s) && c.clock isa Int &&
                c.period_s isa Float64, entry.pending)
        return (at_s=t, from_s=-Inf, clocks=Tuple{Int, Float64}[(c.clock, c.period_s) for c in entry.pending])
    elseif version === 1 && hasproperty(entry, :pending_s) && entry.pending_s == t
        return (at_s=t, from_s=t, clocks=nothing)
    end
    throw(ArgumentError("Periodic tick checkpoint state version 1 or 2, at the checkpoint's time, is required to resume this run."))
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

@inline function _write_checkpoint!(args, t::Float64, u_state, solver_mode::String; is_active=nothing, callback_state=nothing,
                                     model_state=nothing, periodic_ticks=nothing, gnc_scopes=nothing)
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
    if gnc_scopes !== nothing
        runtime = runtime === nothing ? (gnc_scopes=gnc_scopes,) : merge(runtime, (gnc_scopes=gnc_scopes,))
    end
    effector_scopes = _effector_scope_checkpoint_state(args)
    if effector_scopes !== nothing
        runtime = runtime === nothing ? (effector_scopes=effector_scopes,) : merge(runtime, (effector_scopes=effector_scopes,))
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
