# Deliberately standalone, like the RPO contract tests: the general contract and
# its RPO family load with Julia standard libraries alone.
module GNCAlgorithmContractTests
using Test, Random, LinearAlgebra
const SRC = joinpath(@__DIR__, "..", "..", "..", "src", "models", "gnc")
include(joinpath(SRC, "interfaces", "gnc_algorithms.jl"))
include(joinpath(SRC, "interfaces", "rpo_planner.jl"))
include(joinpath(SRC, "planning", "rpo", "direct_planner.jl"))
const G = GNCAlgorithms
const P = RPOPlannerInterfaces
const D = DirectRPOPlanning

# A minimal family and algorithm written the way a contributor would.
struct ToyFamily <: G.AbstractAlgorithmFamily end
struct ToyRequest <: G.AbstractAlgorithmRequest
    request_id::Int
end
struct ToyAlgorithm <: G.AbstractGNCAlgorithm end
G.algorithm_family(::ToyAlgorithm) = ToyFamily()
struct FamilylessAlgorithm <: G.AbstractGNCAlgorithm end
struct NullRPOPlanner <: P.AbstractRPOPlanner end

function rpo_request(; kwargs...)
    defaults = (request_id=1, chaser_id=101, target_id=201, epoch="fixture", time_s=0.,
        x_rtn=(3.,0.,0.,0.,0.,0.), target_state_ii=(7e6,0.,0.,0.,7500.,0.), goal_rtn_m=(5.,0.,0.),
        geometry=(center=[0.,0.,0.], radius=0.4), geometry_revision="sphere-v1",
        constraints=P.RPOPlanningConstraints(clearance_m=.2, max_speed_mps=.5, max_acceleration_mps2=.1),
        reference_dt_s=.1, preview_horizon_steps=2, valid_until_s=100.)
    P.RPOPlanningRequest(; merge(defaults, (;kwargs...))...)
end
function segment_clearance(a, b, g)
    x, y = collect(a), collect(b); d = y - x
    u = norm(d) == 0 ? 0. : clamp(dot(g.center - x, d) / dot(d, d), 0., 1.)
    norm(x + u * d - g.center) - g.radius
end
point_clearance(p, g) = norm(collect(p) - g.center) - g.radius
direct() = D.DirectRPOPlanner(segment_clearance_at=segment_clearance)

# A proposal from an algorithm that declares `consumes_rng = false` leaves the stream unchanged.
function stream_untouched(propose)
    rng = MersenneTwister(2026); witness = copy(rng)
    propose(rng)
    return rand(rng, UInt64) == rand(witness, UInt64)
end

@testset "General contract defaults" begin
    @test !isdefined(@__MODULE__, :SpaceAGORA)
    @test_throws ArgumentError G.algorithm_family(FamilylessAlgorithm())
    toy, request = ToyAlgorithm(), ToyRequest(7)
    @test G.algorithm_capabilities(toy) == NamedTuple() == G.default_capabilities(ToyFamily())
    @test G.require_algorithm_support(toy) === nothing
    @test G.initialize_algorithm(toy, (seed=1,)) === nothing
    @test G.consumes_rng(toy)
    result = G.propose!(nothing, toy, request, MersenneTwister(1))
    @test result isa G.AlgorithmResult && result isa G.AbstractAlgorithmResult
    @test (result.request_id, result.status, result.termination) == (7, :unsupported, :proposal_not_implemented)
    @test result.product === nothing
    revision = G.propose_revision!(nothing, toy, request, :installed, MersenneTwister(1))
    @test (revision.status, revision.termination) == (:unsupported, :revision_not_implemented)
    @test stream_untouched(rng -> G.propose!(nothing, toy, request, rng))
    @test stream_untouched(rng -> G.propose_revision!(nothing, toy, request, :installed, rng))
    accepted = G.validate_capabilities(ToyFamily(), toy, request)
    @test accepted isa G.ValidationResult && accepted.accepted && accepted.reason === :supported
    @test G.on_rejection(ToyFamily()) === :fail
    @test G.family_contract_version(ToyFamily()) === nothing
    # The four family obligations have no fallback: a family must define them.
    @test_throws MethodError G.validate_proposal(ToyFamily(), request, result, NamedTuple())
    @test_throws MethodError G.install!(Ref{Any}(nothing), ToyFamily(), :product, request, 0.0)
    @test_throws MethodError G.is_current(ToyFamily(), request, :product, 0.0)
    @test_throws MethodError G.fallback_product(ToyFamily(), request)
end

@testset "AlgorithmResult and ValidationResult" begin
    product = [1.0, 2.0]; diagnostics = (trace=[3.0],)
    result = G.AlgorithmResult(request_id=3, status=:candidate, termination=:completed,
        product=product, diagnostics=diagnostics)
    product[1] = 99.0; diagnostics.trace[1] = 99.0
    @test result.product == [1.0, 2.0] && result.diagnostics.trace == [3.0]
    @test_throws ArgumentError G.AlgorithmResult(request_id=3, status=:candidate, termination=:completed)
    @test_throws ArgumentError G.AlgorithmResult(request_id=3, status=:failed, termination=:x, product=1.0)
    @test_throws ArgumentError G.AlgorithmResult(request_id=3, status=:accepted, termination=:x)
    for status in (:infeasible, :unsupported, :failed)
        @test G.AlgorithmResult(request_id=3, status=status, termination=:x).product === nothing
    end
    checked = G.ValidationResult(false, :too_fast, (speed_mps=2.0,))
    @test !checked.accepted && checked.reason === :too_fast && checked.metrics.speed_mps == 2.0
    @test G.ValidationResult(true, :ok).metrics == NamedTuple()
end

@testset "RPO is the first family" begin
    @test P.AbstractRPOPlanner <: G.AbstractGNCAlgorithm
    @test P.RPOPlanningRequest <: G.AbstractAlgorithmRequest
    @test P.RPOPlanningResult <: G.AbstractAlgorithmResult
    @test P.RPOValidationResult <: G.AbstractValidationResult
    family = P.RPOReferenceFamily()
    planner, request = direct(), rpo_request()
    @test G.algorithm_family(planner) === family === G.algorithm_family(NullRPOPlanner())
    @test G.default_capabilities(family) == P.RPOPlannerCapabilities()
    @test G.algorithm_capabilities(planner) == P.planner_capabilities(planner)
    @test G.algorithm_capabilities(NullRPOPlanner()) == P.RPOPlannerCapabilities()
    @test G.require_algorithm_support(planner) === P.require_planner_support(planner) === nothing
    @test G.initialize_algorithm(planner, (seed=1,)) === P.initialize_planner(planner, (seed=1,))
    @test G.on_rejection(family) === :fail
    @test G.family_contract_version(family) == v"1.0.0"

    # propose! is plan_rpo!: the same candidate, the same validation.
    proposed = G.propose!(nothing, planner, deepcopy(request), MersenneTwister(1))
    planned = P.plan_rpo!(nothing, planner, deepcopy(request), MersenneTwister(1))
    @test proposed.status === planned.status === :candidate
    @test proposed.reference.t_ref_s == planned.reference.t_ref_s
    @test proposed.reference.r_ref_rtn_m == planned.reference.r_ref_rtn_m
    @test proposed.reference.v_ref_rtn_mps == planned.reference.v_ref_rtn_mps
    checked = G.validate_proposal(family, request, proposed, (clearance_at=point_clearance,))
    reference_check = P.validate_rpo_result(request, planned; clearance_at=point_clearance)
    @test checked isa P.RPOValidationResult
    @test checked.accepted && reference_check.accepted && checked.reason === reference_check.reason
    @test G.validate_proposal(family, request, proposed, NamedTuple()).reason ===
        P.validate_rpo_result(request, planned).reason      # missing trusted query fails closed
    for t in (0.0, 1.0, 50.0, 99.95)
        @test G.is_current(family, request, proposed.reference, t) ==
            P.rpo_reference_is_current(request, proposed.reference, t)
    end

    # Capability checks forward, including retiming and restart refusals.
    @test G.validate_capabilities(family, planner, request).accepted
    retime = rpo_request(reason=:retime)
    @test G.validate_capabilities(family, planner, retime).reason ===
        P.validate_rpo_capabilities(planner, retime).reason === :unsupported_retiming
    @test G.validate_capabilities(family, planner, request; checkpoint=true).reason === :unsupported_restart
    @test G.validate_capabilities(family, NullRPOPlanner(), request).reason === :unsupported_state_source

    # propose_revision! is retime_rpo!: unsupported for both built-in shapes, never a silent replan.
    revised = G.propose_revision!(nothing, NullRPOPlanner(), retime, proposed.reference, MersenneTwister(1))
    @test revised isa P.RPOPlanningResult && revised.termination === :retiming_not_implemented
    @test G.propose!(nothing, NullRPOPlanner(), request, MersenneTwister(1)).termination === :planning_not_implemented
end

@testset "Random streams of deterministic algorithms" begin
    @test !G.consumes_rng(direct())
    @test G.consumes_rng(NullRPOPlanner())              # the default for planners that do not declare
    request = rpo_request()
    @test stream_untouched(rng -> G.propose!(nothing, direct(), request, rng))
    @test stream_untouched(rng -> G.propose!(nothing, NullRPOPlanner(), request, rng))
    @test stream_untouched(rng -> G.propose_revision!(nothing, NullRPOPlanner(),
        rpo_request(reason=:retime), P.plan_rpo!(nothing, direct(), request, rng).reference, rng))
end
end # module GNCAlgorithmContractTests

# The package exposes the same contract at its root, with the lifecycle's error type.
module GNCAlgorithmContractPackageTests
using Test, SpaceAGORA
@testset "GNC algorithm contract at the package root" begin
    @test SpaceAGORA.propose! === SpaceAGORA.GNCAlgorithms.propose!
    @test SpaceAGORA.AbstractRPOPlanner <: SpaceAGORA.AbstractGNCAlgorithm
    @test SpaceAGORA.RPOPlanningError <: SpaceAGORA.GNCAlgorithmError <: Exception
    hypr = SpaceAGORA.HYPRRPOPlanner(SpaceAGORA.RPOPSOConfig())
    @test SpaceAGORA.algorithm_family(hypr) === SpaceAGORA.RPOReferenceFamily()
    @test SpaceAGORA.consumes_rng(hypr)                 # HYPR samples
    @test !SpaceAGORA.consumes_rng(SpaceAGORA.DirectRPOPlanner())
    for name in (:AbstractGNCAlgorithm, :AlgorithmResult, :ValidationResult, :RPOReferenceFamily,
                 :propose!, :propose_revision!, :consumes_rng, :validate_proposal, :install!,
                 :is_current, :on_rejection, :family_contract_version)
        @test Base.isexported(SpaceAGORA, name)
    end
end
end # module GNCAlgorithmContractPackageTests
