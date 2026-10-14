module InterlinkCheckpointTests

using Test
using Serialization
using SpaceAGORA

const SE = SpaceAGORA.SimulationEngine

function checkpoint_fixture(directory)
    spacecraft = [(n_terminal=2,) for _ in 1:3]
    model = ConstellationInterLinkModel(forbidden_pairs=[(3, 2)])
    register_allowed_link!(model, spacecraft, (1, 1), (2, 1);
        parameters=InterLinkParameters(P=12.0, B=3.0, range=100.0))
    policy = SchedulingPolicyModel(:gve_sma; target_idxs=[1, 2])
    args = (interlink_model=model, scheduling_policy_model=policy,
        dynamics_model=(spacecraft=spacecraft,),
        simulation_settings=(checkpoint_directory=directory, results_directory=directory))
    state = (sc=[(pos=[Float64(i), 0.0, 0.0], laser_dv=zeros(3), laser_delta_sma=0.0) for i in 1:3],)
    return args, state
end

@testset "Interlink checkpoint identity and activity" begin
    mktempdir() do directory
        args, state = checkpoint_fixture(directory)
        flags = Bool[true, false, true]
        snapshot = SE._interlink_checkpoint_runtime_state(args, flags)
        flags[2] = true
        @test snapshot.interlinks.is_active == [true, false, true]
        @test snapshot.interlinks.version === 1
        @test snapshot.interlinks.configuration.candidates == [(
            endpoints=((1, 1), (2, 1)), P=12.0, B=3.0, range=100.0)]
        @test snapshot.interlinks.configuration.forbidden_pairs == [(2, 3)]
        @test snapshot.interlinks.configuration.terminal_counts == [2, 2, 2]
        SE._write_checkpoint!(args, 12.5, state, "tsit5"; is_active=snapshot.interlinks.is_active)
        checkpoint = SE._load_checkpoint(args)
        @test checkpoint.t == 12.5
        @test checkpoint.u == state
        @test checkpoint.runtime_state == snapshot
        restored = trues(3)
        @test isnothing(SE._restore_interlink_checkpoint_state!(args, checkpoint, restored))
        @test restored == [true, false, true]

        changes = (
            "topology" => a -> begin
                register_allowed_link!(a.interlink_model, a.dynamics_model.spacecraft, (1, 2), (3, 1))
                a
            end,
            "power" => a -> begin
                a.interlink_model.graph.edge_properties[1] = InterLinkParameters(P=24.0, B=3.0, range=100.0)
                a
            end,
            "magnification" => a -> begin
                a.interlink_model.graph.edge_properties[1] = InterLinkParameters(P=12.0, B=4.0, range=100.0)
                a
            end,
            "range" => a -> begin
                a.interlink_model.graph.edge_properties[1] = InterLinkParameters(P=12.0, B=3.0, range=101.0)
                a
            end,
            "forbidden pairs" => a -> begin
                empty!(a.interlink_model.forbidden_pairs)
                a
            end,
            "link type" => a -> begin
                a.interlink_model.constellation_link_type = :radio
                a
            end,
            "policy" => a -> merge(a, (scheduling_policy_model=SchedulingPolicyModel(:gve_sma_first_index),)),
            "targets" => a -> merge(a, (scheduling_policy_model=SchedulingPolicyModel(:gve_sma; target_idxs=[1, 3]),)),
            "terminal counts" => a -> begin
                a.dynamics_model.spacecraft[3] = (n_terminal=3,)
                a
            end,
        )
        for (name, change) in changes
            @testset "$name" begin
                changed, _ = checkpoint_fixture(directory)
                destination = trues(3)
                @test_throws ArgumentError SE._restore_interlink_checkpoint_state!(change(changed), checkpoint, destination)
                @test all(destination) # Failed admission must not partly restore state.
            end
        end

        @test_throws ArgumentError SE._write_checkpoint!(args, 12.5, state, "tsit5")
        for invalid_runtime in (nothing,
            (interlinks=merge(snapshot.interlinks, (version=2,)),),
            (interlinks=merge(snapshot.interlinks, (is_active=Bool[true, false],)),),
            (interlinks=merge(snapshot.interlinks, (is_active=[1, 0, 1],)),))
            invalid = merge(checkpoint, (runtime_state=invalid_runtime,))
            @test_throws ArgumentError SE._restore_interlink_checkpoint_state!(args, invalid, trues(3))
        end
        legacy_args = merge(args, (interlink_model=nothing,))
        @test_throws ArgumentError SE._restore_interlink_checkpoint_state!(legacy_args, checkpoint, trues(3))

        # An actual old payload has no runtime_state member. The generic reader
        # still loads it, and ordinary non-interlink restoration is unchanged.
        paths = SE._checkpoint_paths(args)
        legacy_state = (sc=[(pos=copy(spacecraft.pos),) for spacecraft in state.sc],)
        open(paths.data, "w") do io
            serialize(io, (schema_version="1", t=12.5, u=legacy_state, solver_mode="tsit5"))
        end
        legacy = SE._load_checkpoint(legacy_args)
        @test get(legacy, :runtime_state, nothing) === nothing
        @test !haskey(legacy, :runtime_state)
        @test legacy.u == legacy_state
        @test_throws ArgumentError SE._restore_interlink_checkpoint_state!(args, legacy, trues(3))
        destination = Bool[true, false, true]
        @test isnothing(SE._restore_interlink_checkpoint_state!(legacy_args, legacy, destination))
        @test destination == [true, false, true]
        # An explicitly empty runtime member also retains the ordinary reader shape.
        open(paths.data, "w") do io
            serialize(io, (schema_version="1", t=12.5, u=legacy_state,
                solver_mode="tsit5", runtime_state=nothing))
        end
        @test SE._load_checkpoint(legacy_args) == legacy
        SE._write_checkpoint!(legacy_args, 13.0, legacy_state, "tsit5")
        ordinary = SE._load_checkpoint(legacy_args)
        @test get(ordinary, :runtime_state, nothing) === nothing
        @test keys(ordinary) == keys(legacy)
        @test keys(deserialize(paths.data)) == (:schema_version, :created_utc, :t, :solver_mode, :u)
    end
end

end # module InterlinkCheckpointTests
