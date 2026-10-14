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

@inline function _write_checkpoint!(args, t::Float64, u_state, solver_mode::String; is_active=nothing)
    return SimulationModel.IOSerialization._write_checkpoint!(
        args,
        t,
        u_state,
        CHECKPOINT_SCHEMA_VERSION;
        solver_mode=solver_mode,
        runtime_state=_interlink_checkpoint_runtime_state(args, is_active)
    )
end

@inline _load_checkpoint(args) = SimulationModel.IOSerialization._load_checkpoint(args)
@inline _clear_checkpoint!(args) = SimulationModel.IOSerialization._clear_checkpoint!(args)
