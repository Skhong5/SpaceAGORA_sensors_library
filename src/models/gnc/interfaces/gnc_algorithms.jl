# General GNC algorithm contract. This module loads with Julia standard libraries
# alone. The RPO planner contract (rpo_planner.jl) is its first family.
module GNCAlgorithms

"""
    AbstractGNCAlgorithm

An algorithm whose output is consumed later or can be rejected: a planner, a
scheduler or a decision service. It runs in the planning profile of the GNC
contract: its family builds an owned request, the algorithm proposes a product,
trusted code validates the proposal, and only a validated copy is installed.
Feedback control laws that act at once use the feedback profile instead (a tick
that computes and holds a command).

An algorithm declares `algorithm_family` and implements
`propose!`; the other algorithm-side functions have defaults.
`AbstractRPOPlanner` is a subtype.
"""
abstract type AbstractGNCAlgorithm end

"""
    AbstractAlgorithmFamily

The trusted definition of one problem: its request and product types, how a
proposal is validated (`validate_proposal`), where an accepted product
is installed (`install!`), when an installed product stops being usable
(`is_current`) and what happens on rejection (`on_rejection`).
Algorithms declare a family; they never validate or install their own products.
`RPOReferenceFamily` is a subtype.
"""
abstract type AbstractAlgorithmFamily end

"""
    AbstractAlgorithmRequest

An owned snapshot handed to an algorithm, in SI units, carrying a positive
`request_id`. The lifecycle keeps its own copy for validation and passes the
algorithm a separate deep copy. `RPOPlanningRequest` is a subtype.
"""
abstract type AbstractAlgorithmRequest end

"""
    AbstractAlgorithmResult

An algorithm's outcome for one request: `request_id`, `status` (`:candidate`,
`:infeasible`, `:unsupported` or `:failed`), `termination`, the proposed product
(present exactly for `:candidate`) and diagnostics. A candidate still needs
trusted validation. `AlgorithmResult` and `RPOPlanningResult` are subtypes.
"""
abstract type AbstractAlgorithmResult end

"""
    AbstractValidationResult

The outcome of trusted validation, with fields `accepted::Bool`, `reason::Symbol`
and `metrics`. `ValidationResult` and `RPOValidationResult` are subtypes.
"""
abstract type AbstractValidationResult end

"""
    GNCAlgorithmError <: Exception

Supertype of the errors an algorithm lifecycle raises when a request fails under
the `:fail` policy. Subtypes carry a copied run report, the reason, and the
original cause and backtrace. `RPOPlanningError` is a subtype.
"""
abstract type GNCAlgorithmError <: Exception end

const ALGORITHM_STATUSES = (:candidate, :infeasible, :unsupported, :failed)

"""
    AlgorithmResult(; request_id, status, termination, product=nothing, diagnostics=NamedTuple())

A generic algorithm outcome for families that need no result type of their own.
`status` is one of `:candidate`, `:infeasible`, `:unsupported` or `:failed`; only a
`:candidate` carries a `product`, and it must carry one. Construction deep-copies
the product and the diagnostics, so later edits through an algorithm-held handle
cannot change them. Diagnostics are never read by validation or installation.
"""
struct AlgorithmResult{P, D} <: AbstractAlgorithmResult
    request_id::Int
    status::Symbol
    termination::Symbol
    product::P
    diagnostics::D
    function AlgorithmResult(; request_id::Integer, status::Symbol, termination::Symbol,
            product=nothing, diagnostics=NamedTuple())
        status in ALGORITHM_STATUSES || throw(ArgumentError(
            "Unknown algorithm result status $(repr(status)); use one of $(ALGORITHM_STATUSES)."))
        (status === :candidate) == (product !== nothing) || throw(ArgumentError(
            "A :candidate result carries a product and no other status does."))
        p, d = deepcopy(product), deepcopy(diagnostics)
        new{typeof(p), typeof(d)}(Int(request_id), status, termination, p, d)
    end
end

"""
    ValidationResult(accepted, reason, metrics=NamedTuple())

A generic validation outcome: whether the proposal is accepted, a machine-readable
reason and the measured values behind it.
"""
struct ValidationResult{M} <: AbstractValidationResult
    accepted::Bool
    reason::Symbol
    metrics::M
end
ValidationResult(accepted::Bool, reason::Symbol) = ValidationResult(accepted, reason, NamedTuple())

_request_id(request) = hasproperty(request, :request_id) ? Int(getproperty(request, :request_id)) : 0

# ---- Algorithm side: what an algorithm author implements ----------------------

"""
    algorithm_family(algorithm) -> AbstractAlgorithmFamily

Required. The family whose request, validation, installation and failure rules
govern `algorithm`. The fallback throws an `ArgumentError` that names the type.
"""
function algorithm_family(algorithm)
    throw(ArgumentError("$(nameof(typeof(algorithm))) declares no algorithm family: implement SpaceAGORA.algorithm_family."))
end

"""
    default_capabilities(family)

The capabilities an algorithm of `family` has when it declares none: support for
nothing. The fallback is an empty `NamedTuple`; a family with a capability type
returns its empty instance.
"""
default_capabilities(family) = NamedTuple()

"""
    algorithm_capabilities(algorithm)

What `algorithm` supports, in its family's capability type. Defaults to
`default_capabilities(algorithm_family(algorithm))`, which supports nothing.
Declaring capabilities never loads algorithm configuration.
"""
algorithm_capabilities(algorithm) = default_capabilities(algorithm_family(algorithm))

"""
    require_algorithm_support(algorithm) -> nothing

Throw before a run starts if something `algorithm` needs is missing, such as an
optional package. The default checks nothing.
"""
require_algorithm_support(algorithm) = nothing

"""
    initialize_algorithm(algorithm, run_context) -> state

Create fresh run-owned state for `algorithm`; stateless algorithms return
`nothing` (the default). `run_context` is a copied `NamedTuple` (for example
`seed`, `stream_words`, `epoch` and spacecraft IDs).
"""
initialize_algorithm(algorithm, run_context) = nothing

"""
    propose!(state, algorithm, request, rng) -> AbstractAlgorithmResult

Required. Propose a product for `request` using run-owned `state`. `rng` is the
only random stream the algorithm may draw from; there is no default global stream.
The algorithm receives its own copy of `request` and must not install anything.
The fallback returns an `:unsupported` result and draws nothing.
"""
function propose!(state, algorithm, request, rng)
    return AlgorithmResult(request_id=_request_id(request), status=:unsupported,
        termination=:proposal_not_implemented)
end

"""
    propose_revision!(state, algorithm, request, active_product, rng) -> AbstractAlgorithmResult

Optional. Revise the installed `active_product` instead of proposing from scratch
(RPO calls this retiming). The fallback returns an `:unsupported` result and
draws nothing; an unsupported revision never falls back to a full proposal.
"""
function propose_revision!(state, algorithm, request, active_product, rng)
    return AlgorithmResult(request_id=_request_id(request), status=:unsupported,
        termination=:revision_not_implemented)
end

"""
    consumes_rng(algorithm) -> Bool

Whether `algorithm` draws from the stream passed to `propose!`. The default
is `true`. A deterministic algorithm declares `false`; its contract tests then
check that a proposal leaves the stream unchanged.
"""
consumes_rng(algorithm) = true

# ---- Family side: trusted, core-owned -----------------------------------------

"""
    validate_capabilities(family, algorithm, request; checkpoint=false) -> AbstractValidationResult

Check `algorithm`'s declared capabilities against `request` before asking for a
proposal; `checkpoint=true` also requires restart support. The default accepts,
for families that have no capability rules.
"""
validate_capabilities(family, algorithm, request; checkpoint::Bool=false) =
    ValidationResult(true, :supported)

"""
    validate_proposal(family, request, result, trusted) -> AbstractValidationResult

Required for each family. Validate `result` against the lifecycle's own copy of
`request`. `trusted` holds queries the lifecycle supplies independently of the
algorithm (for RPO, `clearance_at`). Validation never installs a product, never
advances a random stream and never reads diagnostics as geometry.
"""
function validate_proposal end

"""
    install!(link, family, product, request, t) -> nothing

Required for each family. Install a validated copy of `product` into `link`, the
object the family shares with the product's consumer, at time `t`. Installation
is atomic: no yield or user callback runs between validation and installation.
"""
function install! end

"""
    is_current(family, request, product, t) -> Bool

Required for each family. Whether an installed `product` (accepted for `request`)
may still be used at time `t`, including any preview the consumer needs.
"""
function is_current end

"""
    on_rejection(family) -> Symbol

What the lifecycle does when capability checks or validation reject a proposal:
`:fail` raises a `GNCAlgorithmError` (the default), `:keep_previous`
keeps the installed product, and `:install_fallback` installs
`fallback_product`.
"""
on_rejection(family) = :fail

"""
    fallback_product(family, request)

The product installed on rejection by a family whose `on_rejection` is
`:install_fallback`, such as a hold. Families with another policy need no method.
"""
function fallback_product end

"""
    family_contract_version(family) -> Union{Nothing, VersionNumber}

The version of `family`'s request, product and validation contract, so that a
companion package can require the one it was built for. The default `nothing`
means unversioned.
"""
family_contract_version(family) = nothing

end # module GNCAlgorithms
