# Scheduling policies: what the interlink scheduler maximizes. A policy is a value
# of a policy type; it scores each available candidate link (`candidate_score`),
# and the scheduler activates the terminal matching with the largest total
# score. Included by `SchedulingPolicies` (interlink_scheduling.jl).

"""
    AbstractSchedulingPolicy

Supertype of the interlink scheduling policies. A policy is an immutable value
whose fields say what it scores, and it holds no run state. A policy type
implements `candidate_score`. The built-in policies are
`SemimajorAxisRatePolicy`, `EccentricityRatePolicy`, `PowerDeliveryPolicy` and
`DataThroughputPolicy`. Give a run its policy with
`SimulationConfiguration(scheduling_policy_model=policy)`.
"""
abstract type AbstractSchedulingPolicy end

"""
    LinkCandidate

One available candidate link, as a scheduling policy scores it.

- `key`: its endpoints, `(spacecraft vector index, terminal number)` each, in key
  order: the transmitter first for a directed link, the lower endpoint first
  otherwise.
- `link`: its link kind value.
- `a`, `b`: the endpoints' state samples (`StateSample`), in key order, with
  their spacecraft models when the run's are known.
- `terminal_a`, `terminal_b`: the terminal hardware at the two ends, `nothing`
  for a virtual terminal or when the spacecraft models are not known.
"""
struct LinkCandidate
    key::InterLinkKey
    link::AbstractInterlink
    a::StateSample
    b::StateSample
    terminal_a::Any
    terminal_b::Any
end

"""
    SchedulingContext

What a scheduling policy may read besides the candidate:

- `mu`: the planet's gravitational parameter (m³/s²);
- `link_context`: the time and the planet (`LinkContext`), to pass to the link
  hooks;
- `spacecraft`: the run's spacecraft models, or `nothing` when a caller scores
  without them.

Its other fields are the engine's.
"""
struct SchedulingContext
    mu::Float64
    link_context::LinkContext
    spacecraft::Any
    # The run state, which the built-in orbital policies read as the engine
    # holds it, and a caller's `candidate_force(target, partner, link)`.
    _state::Any
    _candidate_force::Any
end

"""
    candidate_score(policy, candidate::LinkCandidate, ctx::SchedulingContext, t::Float64) -> Float64

The score `policy` gives `candidate`, an available candidate link, at `t` s into
the run (`ctx.link_context.time_s`; NaN outside a run). The scheduler activates
the terminal matching with the largest total score, and only links with a
positive score can be selected. The engine calls it at the start of the run and
after every accepted step, once for each available candidate; an unavailable
candidate scores 0 without a call. A nonfinite score is refused. Every policy
type implements it: a run refuses at setup a policy without a method.
"""
function candidate_score end

# The candidate of graph edge `edge`, at the context's state.
function _link_candidate(model, edge::Int, ctx::SchedulingContext)
    graph = model.graph
    first, second = map(vertex -> graph.vertices[vertex], graph.edges[edge])
    spacecraft, state = ctx.spacecraft, ctx._state
    a = link_endpoint(state.sc[first[1]], _spacecraft_at(spacecraft, first[1]), first[1])
    b = link_endpoint(state.sc[second[1]], _spacecraft_at(spacecraft, second[1]), second[1])
    terminal_a = spacecraft === nothing ? nothing : _terminal_device(spacecraft[first[1]], first[2])
    terminal_b = spacecraft === nothing ? nothing : _terminal_device(spacecraft[second[1]], second[2])
    return LinkCandidate((first, second), graph.edge_properties[edge], a, b, terminal_a, terminal_b)
end

# Checks a policy makes of the run before scoring (its targets, the gravitational
# parameter), and checks across the candidates' scores after it. Both default to
# none.
_prepare_scoring(policy, model, ctx::SchedulingContext) = nothing
_check_scores(policy, model, scores, ctx::SchedulingContext) = nothing

# The run's policy has a `candidate_score` method: checked at setup.
function _check_candidate_score(policy)
    hasmethod(candidate_score, Tuple{typeof(policy), LinkCandidate, SchedulingContext, Float64}) || throw(ArgumentError(
        "The scheduling policy $(nameof(typeof(policy))) has no candidate_score method; implement " *
        "SpaceAGORA.candidate_score(policy::$(nameof(typeof(policy))), candidate, ctx, t), the score of one candidate link."))
    return nothing
end

# ---- The orbital policies ---------------------------------------------------------

# The force the candidate puts on spacecraft `target`: from the link's
# `link_force`, or from a caller's `candidate_force(target, partner, link)`.
function _target_force(ctx::SchedulingContext, candidate::LinkCandidate, target::Int, partner::Int)
    ctx._candidate_force === nothing ||
        return ctx._candidate_force(ctx._state.sc[target], ctx._state.sc[partner], candidate.link)
    first_force, second_force = link_force(candidate.link, candidate.a, candidate.b, ctx.link_context)
    return target == candidate.key[1][1] ? first_force : second_force
end

# The rate (m/s) at which `force` changes the osculating semimajor axis of the
# spacecraft state `target`, as the engine holds it.
function _semimajor_axis_rate(target, force, mu::Real)
    semimajor_axis = inv(2 / norm(target.pos) - dot(target.vel, target.vel) / mu)
    return 2 * semimajor_axis^2 / mu * dot(target.vel, force) / target.mass
end

_check_targets(name, targets) = throw(ArgumentError(
    "$(name) targets must be a positive spacecraft vector index$(name === :SemimajorAxisRatePolicy ?
        ", a nonempty vector of distinct positive indices, :both or :lower_index" : " or :both"); got $(repr(targets))."))

"""
    SemimajorAxisRatePolicy(; targets=1)

Score each available link by the rate (m/s) at which its force raises the
osculating semimajor axis of its target spacecraft:
`da/dt = 2a² (v · F) / (μ m)` at each target end, summed. The force is the
link's `link_force`. `targets` picks the ends:

- a spacecraft vector index: that spacecraft's end only;
- a vector or range of indices: every end in the set, stored sorted;
- `:both`: both ends of every link;
- `:lower_index`: the lower spacecraft index of each pair.

A link that lowers the semimajor axis scores negative, and links that apply no
force (power beams, data links) score 0, so this policy never selects them.
"""
struct SemimajorAxisRatePolicy <: AbstractSchedulingPolicy
    targets::Union{Int, Vector{Int}, Symbol}

    function SemimajorAxisRatePolicy(targets)
        name = :SemimajorAxisRatePolicy
        if targets isa Symbol
            targets in (:both, :lower_index) || _check_targets(name, targets)
            return new(targets)
        elseif targets isa Integer && !(targets isa Bool)
            targets > 0 || _check_targets(name, targets)
            return new(Int(targets))
        elseif targets isa AbstractVector{<:Integer} && !(eltype(targets) <: Bool)
            selected = sort!(Int[targets...])
            !isempty(selected) && all(>(0), selected) && allunique(selected) || _check_targets(name, targets)
            return new(selected)
        end
        _check_targets(name, targets)
    end
end
SemimajorAxisRatePolicy(; targets=1) = SemimajorAxisRatePolicy(targets)

_is_target(targets::Int, satellite::Int) = targets == satellite
_is_target(targets::Vector{Int}, satellite::Int) = satellite in targets
_is_target(targets::Symbol, satellite::Int) = targets === :both

function _prepare_scoring(policy::SemimajorAxisRatePolicy, model, ctx::SchedulingContext)
    isfinite(ctx.mu) && ctx.mu > 0 || throw(ArgumentError("The gravitational parameter must be finite and positive."))
    count = length(ctx._state.sc)
    targets = policy.targets
    targets isa Vector{Int} && !all(index -> index <= count, targets) &&
        throw(ArgumentError("SemimajorAxisRatePolicy targets contain an index outside the spacecraft state vector."))
    targets isa Int && targets > count &&
        throw(ArgumentError("The SemimajorAxisRatePolicy target is outside the spacecraft state vector."))
    return nothing
end

function candidate_score(policy::SemimajorAxisRatePolicy, candidate::LinkCandidate, ctx::SchedulingContext, t::Float64)
    first_satellite, second_satellite = candidate.key[1][1], candidate.key[2][1]
    if policy.targets === :lower_index
        target, partner = minmax(first_satellite, second_satellite)
        return _semimajor_axis_rate(ctx._state.sc[target], _target_force(ctx, candidate, target, partner), ctx.mu)
    end
    score = 0.0
    for (target, partner) in ((first_satellite, second_satellite), (second_satellite, first_satellite))
        _is_target(policy.targets, target) || continue
        score += _semimajor_axis_rate(ctx._state.sc[target], _target_force(ctx, candidate, target, partner), ctx.mu)
    end
    return score
end

"""
    EccentricityRatePolicy(; targets=1)

Score each available link by the rate (1/s) at which its force raises the
eccentricity of its target spacecraft, `de/dt = ê · ė`, at each target end,
summed. `targets` is a spacecraft vector index, or `:both` for both ends of
every link. The force is the link's `link_force`.

At an eccentricity no greater than `sqrt(eps(Float64))` the score is the norm of
the eccentricity-vector rate, `‖ė‖`. Such norms are not additive, so a candidate
set that could put two positive-score links on one such target, through
different terminals, is refused. A link that lowers the eccentricity scores
negative, and links that apply no force score 0.
"""
struct EccentricityRatePolicy <: AbstractSchedulingPolicy
    targets::Union{Int, Symbol}

    function EccentricityRatePolicy(targets)
        name = :EccentricityRatePolicy
        targets === :both && return new(targets)
        targets isa Integer && !(targets isa Bool) && targets > 0 && return new(Int(targets))
        _check_targets(name, targets)
    end
end
EccentricityRatePolicy(; targets=1) = EccentricityRatePolicy(targets)

function _prepare_scoring(policy::EccentricityRatePolicy, model, ctx::SchedulingContext)
    isfinite(ctx.mu) && ctx.mu > 0 || throw(ArgumentError("The gravitational parameter must be finite and positive."))
    policy.targets isa Int && policy.targets > length(ctx._state.sc) &&
        throw(ArgumentError("The EccentricityRatePolicy target is outside the spacecraft state vector."))
    return nothing
end

# The eccentricity rate `force` gives the spacecraft state `target`, and whether
# its orbit is near circular (the rate is then the norm of the vector rate).
function _eccentricity_rate(target, force, mu::Real)
    position = target.pos
    velocity = target.vel
    acceleration = force / target.mass
    angular_momentum = cross(position, velocity)
    eccentricity_vector = cross(velocity, angular_momentum) / mu - position / norm(position)
    eccentricity_vector_rate = (
        cross(acceleration, angular_momentum) + cross(velocity, cross(position, acceleration))
    ) / mu
    eccentricity = norm(eccentricity_vector)
    near_circular = eccentricity <= sqrt(eps(Float64))
    rate = near_circular ? norm(eccentricity_vector_rate) :
        dot(eccentricity_vector, eccentricity_vector_rate) / eccentricity
    return rate, near_circular
end

function candidate_score(policy::EccentricityRatePolicy, candidate::LinkCandidate, ctx::SchedulingContext, t::Float64)
    first_satellite, second_satellite = candidate.key[1][1], candidate.key[2][1]
    score = 0.0
    for (target, partner) in ((first_satellite, second_satellite), (second_satellite, first_satellite))
        policy.targets === :both || policy.targets == target || continue
        rate, _ = _eccentricity_rate(ctx._state.sc[target], _target_force(ctx, candidate, target, partner), ctx.mu)
        score += rate
    end
    return score
end

# Near circularity the score is the norm of the eccentricity-vector rate. Norms
# from two simultaneous forces are not additive: opposite forces can receive
# positive individual scores even though their combined rate is zero. Admit the
# objective only when a scored near-circular spacecraft cannot receive two
# positive-score candidate links in the same terminal matching.
function _check_scores(policy::EccentricityRatePolicy, model, scores, ctx::SchedulingContext)
    graph = model.graph
    circular_candidates = Dict{Int, Vector{Int}}()
    for edge in eachindex(graph.edges)
        graph.available[edge] || continue
        candidate = nothing
        first_satellite = graph.vertices[graph.edges[edge][1]][1]
        second_satellite = graph.vertices[graph.edges[edge][2]][1]
        for (target, partner) in ((first_satellite, second_satellite), (second_satellite, first_satellite))
            policy.targets === :both || policy.targets == target || continue
            candidate === nothing && (candidate = _link_candidate(model, edge, ctx))
            rate, near_circular = _eccentricity_rate(ctx._state.sc[target], _target_force(ctx, candidate, target, partner), ctx.mu)
            near_circular && rate > 0 && push!(get!(Vector{Int}, circular_candidates, target), edge)
        end
    end
    for (satellite, edges) in circular_candidates
        candidates = Tuple{Int, Int}[]
        for edge in edges
            graph.available[edge] && scores[edge] > 0 || continue
            endpoints = graph.edges[edge]
            for previous in candidates
                if all(vertex -> vertex != endpoints[1] && vertex != endpoints[2], previous)
                    throw(ArgumentError("The EccentricityRatePolicy objective does not support simultaneous links " *
                        "on near-circular target spacecraft $satellite: per-link eccentricity-rate norms " *
                        "are not additive. Use candidate links that share one terminal on that target."))
                end
            end
            push!(candidates, endpoints)
        end
    end
    return nothing
end

# ---- Power and data ----------------------------------------------------------------

"""
    PowerDeliveryPolicy()

Score each available directed link whose receiving end is terminal hardware by
the electrical power (W) it would deliver to the receiver's bus,
`link_received_power_w` at the current state. It selects power beams, which
apply no force. Other links score 0, and so does every link when the run's
spacecraft models are not given.
"""
struct PowerDeliveryPolicy <: AbstractSchedulingPolicy end

function candidate_score(::PowerDeliveryPolicy, candidate::LinkCandidate, ctx::SchedulingContext, t::Float64)
    ctx.spacecraft === nothing && return 0.0
    link_directed(candidate.link) || return 0.0
    candidate.terminal_b === nothing && return 0.0
    return Float64(link_received_power_w(candidate.link, candidate.terminal_b, candidate.a, candidate.b, ctx.link_context))
end

"""
    DataThroughputPolicy()

Score each available directed link by the data rate (bit/s) it would carry,
`link_data_rate_bps` at the current state, with the terminal hardware at its two
ends (`nothing` for a virtual terminal). It selects data links. Undirected links
score 0, and so does every link when the run's spacecraft models are not given.
"""
struct DataThroughputPolicy <: AbstractSchedulingPolicy end

function candidate_score(::DataThroughputPolicy, candidate::LinkCandidate, ctx::SchedulingContext, t::Float64)
    ctx.spacecraft === nothing && return 0.0
    link_directed(candidate.link) || return 0.0
    return Float64(link_data_rate_bps(candidate.link, candidate.terminal_a, candidate.terminal_b,
        candidate.a, candidate.b, ctx.link_context))
end

# ---- The scheduling policies chosen by a symbol, removed in 0.6.0 ---------------

# The typed policy a removed symbol form stands for: its type and its targets
# (`nothing` for a type without targets), from the old `target_index` and
# `target_indices`. One table for the refusal message and the checkpoint reader.
function _symbol_policy_spec(policy::Symbol, index, indices)
    targets = indices !== nothing ? indices : index === nothing ? :both : index
    policy === :gve_sma && return (SemimajorAxisRatePolicy, targets)
    policy === :gve_sma_first_index && return (SemimajorAxisRatePolicy, :lower_index)
    policy === :gve_eccentricity && return (EccentricityRatePolicy, targets)
    policy === :power_delivery && return (PowerDeliveryPolicy, nothing)
    policy === :data_throughput && return (DataThroughputPolicy, nothing)
    return nothing
end

# The typed policy a checkpoint's recorded symbol and targets stand for, or
# `nothing` when they stand for none.
function _symbol_scheduling_policy(policy::Symbol, index, indices)
    spec = _symbol_policy_spec(policy, index, indices)
    spec === nothing && return nothing
    T, targets = spec
    try
        return targets === nothing ? T() : T(targets)
    catch err
        err isa ArgumentError || rethrow()
        return nothing
    end
end

# The symbol, target index and target set that a checkpoint records for a policy
# that had a symbol form, as 0.5 recorded them; `nothing` for any other policy.
_symbol_policy_fields(policy) = nothing
function _symbol_policy_fields(policy::SemimajorAxisRatePolicy)
    targets = policy.targets
    targets === :lower_index && return (:gve_sma_first_index, 1, nothing)
    targets === :both && return (:gve_sma, nothing, nothing)
    targets isa Int && return (:gve_sma, targets, nothing)
    return (:gve_sma, 1, copy(targets))
end
_symbol_policy_fields(policy::EccentricityRatePolicy) =
    (:gve_eccentricity, policy.targets === :both ? nothing : policy.targets, nothing)
_symbol_policy_fields(::PowerDeliveryPolicy) = (:power_delivery, 1, nothing)
_symbol_policy_fields(::DataThroughputPolicy) = (:data_throughput, 1, nothing)

const _TARGET_KEYWORDS = (:target_index, :target_indices, :target_idx, :target_idxs)

# The message for a removed symbol form: the call as given, and its typed replacement.
function _removed_symbol_message(policy::Symbol, kwargs)
    given(name) = haskey(kwargs, name)
    index = given(:target_index) ? kwargs[:target_index] : given(:target_idx) ? kwargs[:target_idx] : 1
    indices = given(:target_indices) ? kwargs[:target_indices] : given(:target_idxs) ? kwargs[:target_idxs] : nothing
    call = "SchedulingPolicyModel($(repr(policy))" *
        (isempty(kwargs) ? "" : "; " * join(("$(name)=$(repr(value))" for (name, value) in pairs(kwargs)), ", ")) * ")"
    spec = _symbol_policy_spec(policy, index, indices)
    if spec === nothing
        return "$(call): $(repr(policy)) is not a scheduling policy, and policies chosen by a symbol were removed in " *
            "SpaceAGORA 0.6.0. Use a policy type: SemimajorAxisRatePolicy, EccentricityRatePolicy, PowerDeliveryPolicy " *
            "or DataThroughputPolicy, or one of your own (a subtype of AbstractSchedulingPolicy with a candidate_score method)."
    end
    T, targets = spec
    shown = targets !== nothing && (policy === :gve_sma_first_index || any(given, _TARGET_KEYWORDS))
    replacement = "$(nameof(T))(" * (shown ? "targets=$(repr(targets))" : "") * ")"
    others = [name for name in keys(kwargs) if !(name in _TARGET_KEYWORDS)]
    return "$(call) was removed in SpaceAGORA 0.6.0: scheduling policies are types. Use $(replacement), for example " *
        "scheduling_policy_model=$(replacement)." *
        (isempty(others) ? "" : " The keyword$(length(others) == 1 ? "" : "s") $(join(others, ", ")) " *
            "$(length(others) == 1 ? "has" : "have") no replacement.")
end
