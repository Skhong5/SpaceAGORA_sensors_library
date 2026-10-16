# Deliberately standalone, like the RPO contract tests: the general contract and
# its RPO family load with Julia standard libraries alone.
module GNCAlgorithmContractTests
using Test, Random, LinearAlgebra, SHA
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

# The shared request driver, run by a toy binding and family written the way a
# contributor would; the policy is the family's type parameter.
struct DriverRequest <: G.AbstractAlgorithmRequest
    request_id::Int
    reason::Symbol
    value::Float64
end
struct DriverFamily{Policy} <: G.AbstractAlgorithmFamily end
G.on_rejection(::DriverFamily{Policy}) where {Policy} = Policy
G.validate_capabilities(::DriverFamily, algorithm, request::DriverRequest; checkpoint::Bool=false) =
    request.reason === :unsupported ? G.ValidationResult(false, :unsupported_reason) : G.ValidationResult(true, :supported)
G.validate_proposal(::DriverFamily, request::DriverRequest, result, trusted) =
    result.product <= trusted.limit ? G.ValidationResult(true, :within_limit) :
        G.ValidationResult(false, :above_limit, (product=result.product,))
function G.install!(link::Base.RefValue{Any}, ::DriverFamily, product, request::DriverRequest, t; validation, diagnostics)
    link[] = (product=product, t=t, validated=validation.reason)
    return nothing
end
G.fallback_product(::DriverFamily{:install_fallback}, request::DriverRequest) = 0.0
G.is_revision_request(::DriverFamily, request::DriverRequest) = request.reason === :revise
struct Doubler{Policy} <: G.AbstractGNCAlgorithm
    mode::Symbol
end
G.algorithm_family(::Doubler{Policy}) where {Policy} = DriverFamily{Policy}()
G.consumes_rng(::Doubler) = false
function G.propose!(state, algorithm::Doubler, request::DriverRequest, rng)
    algorithm.mode === :throw && error("algorithm failed")
    algorithm.mode === :foreign_failure && throw(DriverFailure(:foreign, nothing, NamedTuple()))
    algorithm.mode === :wrong_type && return :not_a_result
    return G.AlgorithmResult(request_id=request.request_id, status=:candidate, termination=:completed,
        product=2request.value)
end
G.propose_revision!(state, algorithm::Doubler, request::DriverRequest, active, rng) =
    G.AlgorithmResult(request_id=request.request_id, status=:candidate, termination=:completed,
        product=active + request.value)
struct DriverFailure <: G.GNCAlgorithmError
    reason::Symbol
    cause::Any
    details::Any
end
mutable struct DriverBinding
    algorithm::Any
    runtime::G.AlgorithmRuntime
    link::Base.RefValue{Any}
    throws::Bool
end
DriverBinding(algorithm; throws=true) =
    DriverBinding(algorithm, G.AlgorithmRuntime(G.gnc_stream_words("Toy/v1", 1, 2)), Ref{Any}(nothing), throws)
G.algorithm_runtime(b::DriverBinding) = b.runtime
G.bound_algorithm(b::DriverBinding) = b.algorithm
G.bound_link(b::DriverBinding) = b.link
G.trusted_queries(::DriverBinding) = (limit=10.0,)
G.build_algorithm_request(b::DriverBinding, u, p, t, reason, request_id) = DriverRequest(request_id, reason, u)
function G.fail_algorithm_request!(b::DriverBinding, reason; cause=nothing, backtrace=nothing, details=NamedTuple())
    push!(b.runtime.records, (event=:failure, reason=reason))
    b.throws && throw(DriverFailure(reason, cause, details))
    return nothing
end
# A family whose record hook mutates the product it is given.
struct VectorFamily <: G.AbstractAlgorithmFamily end
G.validate_proposal(::VectorFamily, request::DriverRequest, result, trusted) = G.ValidationResult(true, :accepted)
function G.install!(link::Base.RefValue{Any}, ::VectorFamily, product, request::DriverRequest, t; validation, diagnostics)
    link[] = product
    return nothing
end
function G.installed_record(::VectorFamily, request, result, validation, product, reason, t)
    product[1] = -1.0
    return (event=:installed, product=product)
end
struct VectorAlgorithm <: G.AbstractGNCAlgorithm end
G.algorithm_family(::VectorAlgorithm) = VectorFamily()
G.propose!(state, ::VectorAlgorithm, request::DriverRequest, rng) = G.AlgorithmResult(
    request_id=request.request_id, status=:candidate, termination=:completed, product=[2request.value])
run_request!(b, value, t, reason=:initial) = G.run_algorithm_request!(b, value, nothing, t, reason)
failure(f) = try f(); nothing catch err; err end
events(b) = [r.event for r in b.runtime.records]

@testset "Shared request driver" begin
    b = DriverBinding(Doubler{:fail}(:ok))
    stream = copy(b.runtime.rng)
    @test run_request!(b, 2.0, 0.5) === true
    @test b.link[] == (product=4.0, t=0.5, validated=:within_limit)
    @test b.runtime.product == 4.0 && b.runtime.installed_request == DriverRequest(1, :initial, 2.0)
    @test b.runtime.request_id == 1 && b.runtime.last_request_time == 0.5
    @test events(b) == [:request, :installed] && last(b.runtime.records).product == 4.0
    @test rand(copy(b.runtime.rng)) == rand(stream)          # a deterministic algorithm draws nothing
    # A revision request revises the installed product.
    @test run_request!(b, 1.0, 1.0, :revise) === true && b.link[].product == 5.0
    # :fail: the binding's failure carries the validation; nothing is installed.
    e = failure(() -> run_request!(b, 6.0, 2.0))
    @test e isa DriverFailure && e.reason === :above_limit
    @test e.details.validation.metrics.product == 12.0 && e.details.termination === :completed
    @test b.link[].product == 5.0 && b.runtime.product == 5.0 && b.runtime.request_id == 3
    # A capability refusal fails before any proposal, without details.
    e = failure(() -> run_request!(DriverBinding(Doubler{:fail}(:ok)), 1.0, 0.0, :unsupported))
    @test e isa DriverFailure && e.reason === :unsupported_reason && e.details == NamedTuple()
    # :keep_previous keeps the installed product and records the rejection.
    k = DriverBinding(Doubler{:keep_previous}(:ok)); run_request!(k, 2.0, 0.0)
    @test run_request!(k, 6.0, 1.0) === false
    @test k.link[].product == 4.0 && k.runtime.product == 4.0 && k.runtime.last_request_time == 0.0
    @test events(k) == [:request, :installed, :request, :rejected]
    @test last(k.runtime.records).validation.reason === :above_limit
    @test run_request!(k, 1.0, 2.0, :unsupported) === false
    @test last(k.runtime.records).termination === :not_proposed
    # :install_fallback installs the family's fallback product.
    f = DriverBinding(Doubler{:install_fallback}(:ok))
    @test run_request!(f, 6.0, 1.0) === true
    @test f.link[].product == 0.0 && f.runtime.product == 0.0
    @test events(f) == [:request, :rejected, :fallback_installed]
    # A proposal of the wrong type, and algorithm exceptions, fail the request.
    e = failure(() -> run_request!(DriverBinding(Doubler{:fail}(:wrong_type)), 1.0, 0.0))
    @test e isa DriverFailure && e.reason === :invalid_result_type
    w = DriverBinding(Doubler{:fail}(:throw))
    e = failure(() -> G.run_algorithm_request!(w, 1.0, nothing, 0.0, :initial; exception_reason=:toy_exception))
    @test e isa DriverFailure && e.reason === :toy_exception && e.cause isa ErrorException
    @test w.link[] === nothing && w.runtime.product === nothing
    # An algorithm's own GNCAlgorithmError is wrapped as this binding's failure.
    e = failure(() -> run_request!(DriverBinding(Doubler{:fail}(:foreign_failure)), 1.0, 0.0))
    @test e isa DriverFailure && e.reason === :algorithm_exception
    @test e.cause isa DriverFailure && e.cause.reason === :foreign
    # A record hook gets copies: mutating its product leaves the installed one intact.
    v = DriverBinding(VectorAlgorithm())
    @test run_request!(v, 2.0, 0.0) === true
    @test v.link[] == [4.0] && v.runtime.product == [4.0] && last(v.runtime.records).product == [-1.0]
    # A failure hook that returns instead of throwing breaks the contract.
    @test_throws ArgumentError run_request!(DriverBinding(Doubler{:fail}(:ok); throws=false), 6.0, 0.0)
end

@testset "Stream words and runtime" begin
    words = G.gnc_stream_words("RPO/v1", 741, 101, 201)
    digest = sha256("SpaceAGORA/RPO/v1/741/101/201")
    @test words == [sum(UInt32(digest[i+j]) << (8j) for j in 0:3) for i in 1:4:32]
    @test length(words) == 8 && all(w -> w <= typemax(UInt32), words)
    @test G.gnc_stream_words("RPO/v1", 741, 101, 202) != words
    @test G.gnc_stream_words("RPO/v2", 741, 101, 201) != words
    runtime = G.AlgorithmRuntime(words)
    @test rand(runtime.rng) == rand(MersenneTwister(words))
    @test runtime.stream_words == words && runtime.stream_words isa Vector{UInt32}
    # The runtime owns its seed words: reusing the caller's vector changes nothing.
    caller_words = collect(UInt32, words)
    owned = G.AlgorithmRuntime(caller_words)
    caller_words[1] = 0
    @test owned.stream_words == words && rand(owned.rng) == rand(MersenneTwister(words))
    @test runtime.request_id == 0 && runtime.product === nothing && runtime.installed_request === nothing
    @test isempty(runtime.records) && isempty(runtime.processed_tokens) && runtime.last_request_time == -Inf
end

@testset "RPO lifecycle hooks" begin
    family, request = P.RPOReferenceFamily(), rpo_request()
    proposed = G.propose!(nothing, direct(), deepcopy(request), MersenneTwister(1))
    checked = G.validate_proposal(family, request, proposed, (clearance_at=point_clearance,))
    @test G.algorithm_result_type(family) === P.RPOPlanningResult
    @test G.is_revision_request(family, rpo_request(reason=:retime))
    @test !G.is_revision_request(family, request) && !G.is_revision_request(family, rpo_request(reason=:replan))
    @test G.proposed_product(proposed) === proposed.reference
    record = G.installed_record(family, request, proposed, checked, proposed.reference, :initial, 0.0)
    @test keys(record) == (:event, :time_s, :request_id, :reason, :termination, :validation, :reference, :diagnostics)
    @test record.reference === proposed.reference && record.event === :installed
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
