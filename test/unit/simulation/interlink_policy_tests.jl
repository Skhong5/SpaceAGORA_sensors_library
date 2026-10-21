module InterlinkPolicyTests

# Scheduling policies as types: the built-in policies' constructors, a policy of
# your own through `candidate_score`, the setup check, the refusal of the
# symbol forms removed in 0.6.0, and the checkpoint entry of a policy.

using Test
using LinearAlgebra
using StaticArrays
using SpaceAGORA

const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const SP = SM.SchedulingPolicies
const PLANET = make_no_gram_planet(:earth)
const RADIUS = PLANET.Rp_e + 700e3
const SPEED = sqrt(PLANET.μ / RADIUS)

# A policy of your own: prefer short links, `1 / separation` (1/m). It records
# the times it is called at, so the tests can count its calls.
struct ShortLinkPolicy <: AbstractSchedulingPolicy
    calls::Vector{Float64}
end
ShortLinkPolicy() = ShortLinkPolicy(Float64[])
function SpaceAGORA.candidate_score(policy::ShortLinkPolicy, candidate::LinkCandidate, ctx::SchedulingContext, t::Float64)
    push!(policy.calls, t)
    @assert isequal(t, ctx.link_context.time_s)
    return 1 / norm(candidate.a.pos_ii - candidate.b.pos_ii)
end

struct NoScorePolicy <: AbstractSchedulingPolicy end

struct ConstantPolicy <: AbstractSchedulingPolicy
    value::Float64
end
SpaceAGORA.candidate_score(policy::ConstantPolicy, candidate, ctx, t) = policy.value

# Same-named policies in two modules, and a policy with nested and vector fields.
module Left
import SpaceAGORA
struct Same <: SpaceAGORA.AbstractSchedulingPolicy end
SpaceAGORA.candidate_score(::Same, candidate, ctx, t) = 1.0
end
module Right
import SpaceAGORA
struct Same <: SpaceAGORA.AbstractSchedulingPolicy end
SpaceAGORA.candidate_score(::Same, candidate, ctx, t) = 1.0
end
struct NestedPolicy <: AbstractSchedulingPolicy
    inner::AbstractSchedulingPolicy
    weights::Vector{Float64}
    label::Symbol
end
SpaceAGORA.candidate_score(policy::NestedPolicy, candidate, ctx, t) = 1.0

# Policies with a function field: a closure from one factory expression, which
# captures its gain or a vector; a closure that captures a reassigned variable
# (a `Core.Box`); and a list that can contain itself.
struct CallablePolicy{F} <: AbstractSchedulingPolicy
    f::F
end
# Counts every call in the process, through the run's own copy of the policy.
const CALLABLE_SCORES = Ref(0)
SpaceAGORA.candidate_score(policy::CallablePolicy, candidate, ctx, t) = (CALLABLE_SCORES[] += 1; policy.f(1.0))
gain_policy(gain) = CallablePolicy(x -> gain * x)
vector_policy(v) = CallablePolicy(x -> v[1] * x)
function boxed_policy()
    gain = 1.0
    f = x -> gain * x
    gain = 2.0
    return CallablePolicy(f)
end
struct ListPolicy <: AbstractSchedulingPolicy
    items::Vector{Any}
end
SpaceAGORA.candidate_score(policy::ListPolicy, candidate, ctx, t) = 1.0

function sat(index, arc_m)
    bus = Link(root=true, m=100.0)
    θ = arc_m / RADIUS
    ic = CartesianInitialCondition(SVector(RADIUS * cos(θ), RADIUS * sin(θ), 0.0),
        SVector(-SPEED * sin(θ), SPEED * cos(θ), 0.0); q=SVector(0.0, 0.0, 0.0, 1.0))
    return SpacecraftModel(root=bus, links=[bus], inertia_tensor=bus.inertia, initial_condition=ic, id=index,
        n_terminal=1)
end

function run_args(policy; duration=5.0, settings=SimulationSettings(results=false, generate_plots=false))
    spacecraft = [sat(1, 0.0), sat(2, 2000.0), sat(3, 9000.0)]
    model = InterlinkModel()
    link = LaserMomentumLink(power_w=40.0, magnification=1.0, range_m=1e4)
    register_link!(model, spacecraft, (1, 1), (2, 1), link)
    register_link!(model, spacecraft, (1, 1), (3, 1), link)
    return SimulationConfiguration(
        simulation_settings=settings,
        mission_configuration=MissionConfiguration(mission_type=MissionTime, mission_time=duration,
            orientation_sim=false, data_rate=1.0),
        environment_model=make_no_gram_environment(planet=PLANET),
        dynamics_model=DynamicsModel(spacecraft, (InverseSquaredGravityModel(),)),
        guidance_model=GuidanceModel((), Float64[]), navigation_model=NavigationModel((), Float64[]),
        control_model=ControlModel((), Float64[]), initial_time=InitialTime(),
        integration_tolerances=IntegrationTolerances(reltol_orbit=1e-10, abstol_orbit=1e-8, dt_max_orbit=1.0),
        solver_config=SM.SolverConfig(solver_mode=:tsit5), interlink_model=model,
        scheduling_policy_model=policy)
end

# A state of two spacecraft 1 km apart and a third out of range, with one link to each.
function scoring_fixture()
    spacecraft = [(n_terminal=1,), (n_terminal=1,), (n_terminal=1,)]
    model = InterlinkModel()
    link = LaserMomentumLink(power_w=40.0, magnification=1.0, range_m=1e4)
    register_link!(model, spacecraft, (1, 1), (2, 1), link)
    register_link!(model, spacecraft, (1, 1), (3, 1), link)
    state = (sc=[(pos=[RADIUS, 0.0, 0.0], vel=[0.0, SPEED, 0.0], mass=100.0),
                 (pos=[RADIUS, 1000.0, 0.0], vel=[0.0, SPEED, 0.0], mass=100.0),
                 (pos=[RADIUS, 5e4, 0.0], vel=[0.0, SPEED, 0.0], mass=100.0)],)
    update_availability!(model, spacecraft, state)
    return spacecraft, model, state
end

# The removed symbol forms and the typed call each message names.
const REMOVED = [
    ((:gve_sma, (;)), "SemimajorAxisRatePolicy()"),
    ((:gve_sma, (target_index=2,)), "SemimajorAxisRatePolicy(targets=2)"),
    ((:gve_sma, (target_idx=2,)), "SemimajorAxisRatePolicy(targets=2)"),
    ((:gve_sma, (target_index=nothing,)), "SemimajorAxisRatePolicy(targets=:both)"),
    ((:gve_sma, (target_idx=nothing,)), "SemimajorAxisRatePolicy(targets=:both)"),
    ((:gve_sma, (target_indices=[31, 32],)), "SemimajorAxisRatePolicy(targets=[31, 32])"),
    ((:gve_sma, (target_idxs=[31, 32],)), "SemimajorAxisRatePolicy(targets=[31, 32])"),
    ((:gve_sma, (target_indices=31:40,)), "SemimajorAxisRatePolicy(targets=31:40)"),
    ((:gve_sma, (target_index=99, target_indices=[2, 1])), "SemimajorAxisRatePolicy(targets=[2, 1])"),
    ((:gve_sma_first_index, (;)), "SemimajorAxisRatePolicy(targets=:lower_index)"),
    ((:gve_sma_first_index, (target_index=3,)), "SemimajorAxisRatePolicy(targets=:lower_index)"),
    ((:gve_eccentricity, (;)), "EccentricityRatePolicy()"),
    ((:gve_eccentricity, (target_index=3,)), "EccentricityRatePolicy(targets=3)"),
    ((:gve_eccentricity, (target_idx=nothing,)), "EccentricityRatePolicy(targets=:both)"),
    ((:power_delivery, (;)), "PowerDeliveryPolicy()"),
    ((:power_delivery, (target_index=2,)), "PowerDeliveryPolicy()"),
    ((:data_throughput, (;)), "DataThroughputPolicy()"),
]

refusal(f) = try f(); nothing catch err err end

@testset "Interlink scheduling policies" begin
    @testset "built-in constructors" begin
        @test SemimajorAxisRatePolicy().targets == 1
        @test SemimajorAxisRatePolicy(targets=3).targets === 3
        @test SemimajorAxisRatePolicy(targets=[3, 1]).targets == [1, 3]
        @test SemimajorAxisRatePolicy(targets=31:33).targets == [31, 32, 33]
        @test SemimajorAxisRatePolicy(targets=:both).targets === :both
        @test SemimajorAxisRatePolicy(targets=:lower_index).targets === :lower_index
        for bad in (0, -1, Int[], [0], [1, 1], :all, true, [true], 1.5, "2", nothing)
            @test_throws ArgumentError SemimajorAxisRatePolicy(targets=bad)
        end
        @test EccentricityRatePolicy().targets === 1
        @test EccentricityRatePolicy(targets=:both).targets === :both
        for bad in (0, [1], :lower_index, true, nothing)
            @test_throws ArgumentError EccentricityRatePolicy(targets=bad)
        end
        @test PowerDeliveryPolicy() isa AbstractSchedulingPolicy && DataThroughputPolicy() isa AbstractSchedulingPolicy
    end

    @testset "the scheduler and the configuration" begin
        scheduler = SchedulingPolicyModel(PowerDeliveryPolicy())
        @test scheduler.objective isa PowerDeliveryPolicy && scheduler.matching === nothing
        @test fieldnames(SchedulingPolicyModel) == (:objective, :matching)
        args = run_args(DataThroughputPolicy())
        @test args.scheduling_policy_model isa SchedulingPolicyModel && args.scheduling_policy_model.objective isa DataThroughputPolicy
        args = run_args(scheduler)
        @test args.scheduling_policy_model === scheduler
        default = SimulationConfiguration(environment_model=args.environment_model, dynamics_model=args.dynamics_model,
            initial_time=args.initial_time).scheduling_policy_model
        @test default.objective isa SemimajorAxisRatePolicy && default.objective.targets === 1
    end

    @testset "a policy of your own" begin
        spacecraft, model, state = scoring_fixture()
        policy = ShortLinkPolicy()
        scheduler = SchedulingPolicyModel(policy)
        score_candidates!(model, scheduler, state, PLANET.μ; context=LinkContext(time_s=12.0, planet=PLANET))
        # Only the available candidate is scored; the other scores 0 without a call.
        @test policy.calls == [12.0]
        @test SP.matching_workspace!(model, scheduler).score == [1 / 1000.0, 0.0]
        @test select_interlinks!(model, scheduler) == [((1, 1), (2, 1))]
        score_candidates!(model, scheduler, state, PLANET.μ)
        @test isnan(last(policy.calls))
        # A nonfinite score is refused, naming the policy and the candidate.
        err = refusal(() -> score_candidates!(model, SchedulingPolicyModel(ConstantPolicy(NaN)), state, PLANET.μ))
        @test err isa ArgumentError && occursin("ConstantPolicy", err.msg) && occursin("((1, 1), (2, 1))", err.msg)
        # The positive-score rule holds for any policy.
        negative = SchedulingPolicyModel(ConstantPolicy(-1.0))
        score_candidates!(model, negative, state, PLANET.μ)
        @test isempty(select_interlinks!(model, negative))

        # In a run: selected at every accepted step, at the step's time.
        policy = ShortLinkPolicy()
        solution = run_simulation(run_args(policy); return_solution=true)
        history = solution.prob.p.args.interlink_model.history
        @test !isempty(history) && all(sample -> sample.active == [((1, 1), (2, 1))], history)
        @test solution.prob.p.args.scheduling_policy_model.objective isa ShortLinkPolicy
        # The run's policy is a copy (state isolation), whose calls cover the run's steps.
        calls = solution.prob.p.args.scheduling_policy_model.objective.calls
        @test !isempty(calls) && all(t -> 0.0 <= t <= 5.0, calls)
    end

    @testset "a policy without candidate_score is refused at setup" begin
        err = refusal(() -> run_simulation(run_args(NoScorePolicy())))
        @test err isa ArgumentError && occursin("NoScorePolicy has no candidate_score method", err.msg)
        @test_throws MethodError score_candidates!(scoring_fixture()[2], SchedulingPolicyModel(NoScorePolicy()),
            scoring_fixture()[3], PLANET.μ)
    end

    @testset "power and data policies need the spacecraft models" begin
        spacecraft, model, state = scoring_fixture()
        for policy in (PowerDeliveryPolicy(), DataThroughputPolicy())
            scheduler = SchedulingPolicyModel(policy)
            score_candidates!(model, scheduler, state, PLANET.μ)
            @test all(iszero, SP.matching_workspace!(model, scheduler).score)
        end
    end

    @testset "the symbol forms are refused with their replacement" begin
        for ((symbol, keywords), replacement) in REMOVED
            err = refusal(() -> SchedulingPolicyModel(symbol; keywords...))
            @test err isa ArgumentError
            @test occursin("was removed in SpaceAGORA 0.6.0", err.msg)
            @test occursin("Use $(replacement), for example scheduling_policy_model=$(replacement).", err.msg)
            # The named call builds the policy the symbol form stood for.
            policy = Core.eval(@__MODULE__, Meta.parse(replacement))
            expected = SP._symbol_scheduling_policy(symbol, get(keywords, :target_index, get(keywords, :target_idx, 1)),
                get(keywords, :target_indices, get(keywords, :target_idxs, nothing)))
            @test typeof(policy) === typeof(expected) && isequal(getfield.(Ref(policy), fieldnames(typeof(policy))),
                getfield.(Ref(expected), fieldnames(typeof(expected))))
        end
        err = refusal(() -> SchedulingPolicyModel(:invalid))
        @test err isa ArgumentError && occursin(":invalid is not a scheduling policy", err.msg) &&
            all(name -> occursin(name, err.msg), ("SemimajorAxisRatePolicy", "EccentricityRatePolicy",
                "PowerDeliveryPolicy", "DataThroughputPolicy", "AbstractSchedulingPolicy"))
        err = refusal(() -> SchedulingPolicyModel(:gve_sma; target_index=2, mode=:fast))
        @test err isa ArgumentError && occursin("targets=2", err.msg) && occursin("The keyword mode has no replacement.", err.msg)
        # A combination the old constructor refused still names a typed call, whose own check refuses it.
        err = refusal(() -> SchedulingPolicyModel(:gve_eccentricity; target_indices=[1]))
        @test err isa ArgumentError && occursin("EccentricityRatePolicy(targets=[1])", err.msg)
        @test_throws ArgumentError EccentricityRatePolicy(targets=[1])
        err = refusal(() -> SchedulingPolicyModel(:gve_sma; target_indices=Int[]))
        @test err isa ArgumentError && occursin("SemimajorAxisRatePolicy(targets=Int64[])", err.msg)
        # The removed properties are gone.
        scheduler = SchedulingPolicyModel(SemimajorAxisRatePolicy())
        for name in (:policy, :target_index, :target_indices, :target_idx, :target_idxs)
            @test_throws FieldError getproperty(scheduler, name)
        end
        # The same refusal under --depwarn=error: it is no deprecation.
        script = """
            using SpaceAGORA
            @assert Base.JLOptions().depwarn == 2
            err = try SchedulingPolicyModel(:gve_sma; target_idx=2); nothing catch e e end
            println("REFUSAL ", nameof(typeof(err)), " ", occursin("SemimajorAxisRatePolicy(targets=2)", err.msg))
            """
        output = read(pipeline(`$(Base.julia_cmd()) --startup-file=no --depwarn=error
            --project=$(Base.active_project()) -e $script`; stderr=devnull), String)
        @test occursin("REFUSAL ArgumentError true", output)
    end

    @testset "checkpoint entry" begin
        spacecraft = [(n_terminal=2,) for _ in 1:3]
        model = InterlinkModel()
        register_link!(model, spacecraft, (1, 1), (2, 1), LaserMomentumLink(power_w=12.0, magnification=3.0, range_m=100.0))
        args(policy) = (interlink_model=model, scheduling_policy_model=SchedulingPolicyModel(policy),
            dynamics_model=(spacecraft=spacecraft,))
        entry(policy) = (c = SE._interlink_checkpoint_configuration(args(policy)); (c.policy, c.target_idx, c.target_idxs))
        # The policies 0.5 chose by a symbol are recorded as 0.5 recorded them.
        @test entry(SemimajorAxisRatePolicy()) == (:gve_sma, 1, nothing)
        @test entry(SemimajorAxisRatePolicy(targets=2)) == (:gve_sma, 2, nothing)
        @test entry(SemimajorAxisRatePolicy(targets=:both)) == (:gve_sma, nothing, nothing)
        @test entry(SemimajorAxisRatePolicy(targets=[3, 1])) == (:gve_sma, 1, [1, 3])
        @test entry(SemimajorAxisRatePolicy(targets=:lower_index)) == (:gve_sma_first_index, 1, nothing)
        @test entry(EccentricityRatePolicy(targets=3)) == (:gve_eccentricity, 3, nothing)
        @test entry(EccentricityRatePolicy(targets=:both)) == (:gve_eccentricity, nothing, nothing)
        @test entry(PowerDeliveryPolicy()) == (:power_delivery, 1, nothing)
        @test entry(DataThroughputPolicy()) == (:data_throughput, 1, nothing)
        @test keys(SE._interlink_checkpoint_configuration(args(PowerDeliveryPolicy()))) ==
            (:vertices, :candidates, :forbidden_pairs, :policy, :target_idx, :target_idxs, :terminal_counts)
        # Another policy by its kind and every field: floats with their bits, a policy
        # field by its own identity, a vector element by element.
        nested = NestedPolicy(ConstantPolicy(2.0), [0.5, -0.0], :x)
        policy, target_idx, target_idxs = entry(nested)
        @test target_idx === nothing && target_idxs === nothing
        @test policy.kind.name === :NestedPolicy && policy.kind.module_path == fullname(@__MODULE__)
        @test policy.fields.inner.kind.name === :ConstantPolicy
        @test policy.fields.inner.fields.value.bits == reinterpret(UInt64, 2.0)
        @test [element.bits for element in policy.fields.weights.elements] == reinterpret.(UInt64, [0.5, -0.0])

        # Resume: the snapshot written with a policy, and with the targets 0.5 wrote.
        flags = Bool[true, true, true]
        state = (sc=[(pos=[Float64(i), 0.0, 0.0],) for i in 1:3],)
        checkpoint(policy; configuration=identity) = (u=state, runtime_state=(interlinks=merge(
            SE._interlink_checkpoint_runtime_state(args(policy), flags).interlinks,
            (configuration=configuration(SE._interlink_checkpoint_configuration(args(policy))),)),))
        resumes(policy, snapshot) = try
            SE._restore_interlink_checkpoint_state!(args(policy), snapshot, trues(3)); true
        catch err
            err isa ArgumentError || rethrow(); false
        end
        recorded(symbol, index, indices) = c -> merge(c, (policy=symbol, target_idx=index, target_idxs=indices))
        base = SemimajorAxisRatePolicy()
        for (snapshot_entry, policy) in (
                ((:gve_sma, 1, nothing), SemimajorAxisRatePolicy()),
                ((:gve_sma, nothing, [2, 3]), SemimajorAxisRatePolicy(targets=[2, 3])),
                ((:gve_sma, 99, [1, 2]), SemimajorAxisRatePolicy(targets=[1, 2])),
                ((:gve_sma_first_index, 3, nothing), SemimajorAxisRatePolicy(targets=:lower_index)),
                ((:gve_eccentricity, nothing, nothing), EccentricityRatePolicy(targets=:both)),
                ((:power_delivery, 7, nothing), PowerDeliveryPolicy()),
                ((:data_throughput, nothing, nothing), DataThroughputPolicy()))
            @test resumes(policy, checkpoint(base; configuration=recorded(snapshot_entry...)))
        end
        for (snapshot_entry, policy) in (
                ((:gve_sma, 1, nothing), SemimajorAxisRatePolicy(targets=2)),
                ((:gve_sma, 1, [1, 2]), SemimajorAxisRatePolicy(targets=[1, 3])),
                ((:gve_sma, 1, nothing), EccentricityRatePolicy()),
                ((:gve_sma_first_index, 1, nothing), SemimajorAxisRatePolicy(targets=:both)),
                ((:power_delivery, 1, nothing), DataThroughputPolicy()),
                ((:radio, 1, nothing), SemimajorAxisRatePolicy()),
                ((:gve_eccentricity, 1, [1, 2]), EccentricityRatePolicy()))
            @test !resumes(policy, checkpoint(base; configuration=recorded(snapshot_entry...)))
        end
        # A policy of your own resumes itself and refuses any other field, kind or module.
        @test resumes(nested, checkpoint(nested))
        @test resumes(ConstantPolicy(2.0), checkpoint(ConstantPolicy(2.0)))
        @test !resumes(ConstantPolicy(2.5), checkpoint(ConstantPolicy(2.0)))
        @test !resumes(ConstantPolicy(0.0), checkpoint(ConstantPolicy(-0.0)))
        @test !resumes(NestedPolicy(ConstantPolicy(2.0), [0.5, 0.0], :x), checkpoint(nested))
        @test !resumes(NestedPolicy(ConstantPolicy(3.0), [0.5, -0.0], :x), checkpoint(nested))
        @test !resumes(Right.Same(), checkpoint(Left.Same()))
        @test !resumes(SemimajorAxisRatePolicy(), checkpoint(ConstantPolicy(1.0)))
        @test !resumes(ConstantPolicy(1.0), checkpoint(SemimajorAxisRatePolicy()))

        # A function field: a closure is recorded with every value it captures, so
        # two closures of one factory expression differ by their captures.
        @test typeof(gain_policy(1.0)) === typeof(gain_policy(2.0))
        @test resumes(gain_policy(2.0), checkpoint(gain_policy(2.0)))
        @test !resumes(gain_policy(1.0), checkpoint(gain_policy(2.0)))
        @test resumes(vector_policy([2.0]), checkpoint(vector_policy([2.0])))
        @test !resumes(vector_policy([1.0]), checkpoint(vector_policy([2.0])))
        captured = entry(gain_policy(2.0))[1].fields.f
        @test captured.captures.gain.bits == reinterpret(UInt64, 2.0)
        # A named function is its type: another function is refused.
        @test resumes(CallablePolicy(sin), checkpoint(CallablePolicy(sin)))
        @test !resumes(CallablePolicy(cos), checkpoint(CallablePolicy(sin)))
        @test entry(CallablePolicy(sin))[1].fields.f == (callable=SE._interlink_kind_identity(typeof(sin)),)
        # A reassigned capture and a payload that contains itself cannot be recorded.
        err = refusal(() -> entry(boxed_policy()))
        @test err isa ArgumentError && occursin("reassigned after the closure was made", err.msg)
        cycle = Any[]
        push!(cycle, cycle)
        err = refusal(() -> entry(ListPolicy(cycle)))
        @test err isa ArgumentError && occursin("contain themselves", err.msg)
        shared = [1.0]
        @test entry(ListPolicy(Any[shared, shared]))[2] === nothing   # shared, not cyclic
        # A checkpointed run refuses such a policy at setup, before it starts.
        mktempdir() do directory
            settings = SimulationSettings(results=false, generate_plots=false, checkpoint_enabled=true,
                checkpoint_interval_s=1.0, checkpoint_directory=directory)
            CALLABLE_SCORES[] = 0
            err = refusal(() -> run_simulation(run_args(boxed_policy(); settings)))
            @test err isa ArgumentError && occursin("reassigned after the closure was made", err.msg)
            @test isempty(readdir(directory))
            @test CALLABLE_SCORES[] == 0   # refused before the scheduler's first call
        end
    end
end

end # module InterlinkPolicyTests
