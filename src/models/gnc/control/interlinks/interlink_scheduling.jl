module SchedulingPolicies

using LinearAlgebra
using ..InterLinkModels
using ..InterLinkModels: link_endpoint, _terminal_device, _spacecraft_at
using ..EffectorSampling: StateSample

export AbstractSchedulingPolicy, LinkCandidate, SchedulingContext, candidate_score
export SemimajorAxisRatePolicy, EccentricityRatePolicy, PowerDeliveryPolicy, DataThroughputPolicy
export SchedulingPolicyModel, score_candidates!
export select_interlinks!, schedule_interlinks!

# 1. Matching workspace for dynamic programming
# 1.1. Scheduler-owned scores and matching scratch (reused between scheduling calls)
struct InterLinkMatchingWorkspace
	graph::InterLinkGraph
	score::Vector{Float64}
	candidates::Vector{Int}
	exact_scores::Vector{Rational{BigInt}}
	weights::Vector{BigInt}
	parent::Vector{Int}
	order::Vector{Int}
	stack::Vector{Int}
	free_score::Vector{BigInt}
	blocked_score::Vector{BigInt}
	choice::Vector{Int}
	blocked::Vector{Bool}
	cyclic::Vector{Bool}
end

# 2. Scheduling policies (their types and scores are in scheduling_policies.jl) and the scheduler
include(joinpath(@__DIR__, "scheduling_policies.jl"))

# 2.1. The scheduler
"""
    SchedulingPolicyModel(policy::AbstractSchedulingPolicy)

The run's interlink scheduler: the scheduling policy whose total score it
maximizes (`objective`) and the matching workspace it reuses between
scheduling calls (`matching`). Pass it, or the policy alone, as
`SimulationConfiguration(scheduling_policy_model=...)`; the default is
`SchedulingPolicyModel(SemimajorAxisRatePolicy())`.

The policies chosen by a symbol, `SchedulingPolicyModel(:gve_sma)` and the
others, were removed in SpaceAGORA 0.6.0, with their `target_index` and
`target_indices` keywords and properties and the older `target_idx` and
`target_idxs`. A call with a symbol throws an `ArgumentError` that names its
typed replacement, such as `SemimajorAxisRatePolicy(targets=[31, 32])` for
`SchedulingPolicyModel(:gve_sma; target_indices=[31, 32])`.
"""
mutable struct SchedulingPolicyModel
	const objective::AbstractSchedulingPolicy
	matching::Union{Nothing, InterLinkMatchingWorkspace}

	SchedulingPolicyModel(objective::AbstractSchedulingPolicy) = new(objective, nothing)
end
SchedulingPolicyModel(policy::Symbol; kwargs...) = throw(ArgumentError(_removed_symbol_message(policy, kwargs)))

# 1.2. Create and maintain the dynamic-programming workspace.
function matching_workspace!(model::InterlinkModel, policy::SchedulingPolicyModel)
	graph = model.graph
	vertices, edges = length(graph.vertices), length(graph.edges)
	work = policy.matching
	if work !== nothing && work.graph === graph &&
		length(work.parent) == vertices && length(work.weights) == edges
		return work
	end
	scores = zeros(Float64, edges)
	if work !== nothing && work.graph === graph
		copyto!(scores, 1, work.score, 1, min(edges, length(work.score)))
	end
	policy.matching = InterLinkMatchingWorkspace(
		graph, scores, sizehint!(Int[], edges), sizehint!(Rational{BigInt}[], edges), zeros(BigInt, edges),
		zeros(Int, vertices), sizehint!(Int[], vertices), sizehint!(Int[], vertices),
		zeros(BigInt, vertices), zeros(BigInt, vertices), zeros(Int, vertices),
		fill(false, vertices), fill(false, vertices))
	return policy.matching
end

# 2.2. Score every candidate link with the scheduler's policy
"""
	score_candidates!(model::InterlinkModel, policy::SchedulingPolicyModel, state, mu::Real;
	                  candidate_force=nothing, spacecraft=nothing, context=LinkContext())

Score every candidate link of `model` with the scheduler's policy
(`policy.objective`) into its matching workspace: `candidate_score` for each
available candidate, at the run state `state`, and 0 for the others. `mu` is the
planet's gravitational parameter (m³/s²), `spacecraft` the run's spacecraft
models (the power and data policies score nothing without them) and `context`
the link context. The orbital policies take each link's force from its
`link_force`, or from a caller's `candidate_force(target, partner, link)`.
"""
function score_candidates!(model::InterlinkModel, policy::SchedulingPolicyModel, state, mu::Real;
	candidate_force=nothing, spacecraft=nothing, context=LinkContext())
	ctx = SchedulingContext(Float64(mu), context, spacecraft, state, candidate_force)
	objective = policy.objective
	_prepare_scoring(objective, model, ctx)
	work = matching_workspace!(model, policy)
	_score_candidates!(work.score, objective, model, ctx)
	_check_scores(objective, model, work.score, ctx)
	return nothing
end

# The scores of every edge: one loop compiled for each policy type.
function _score_candidates!(scores, objective::P, model::InterlinkModel, ctx::SchedulingContext) where {P}
	graph = model.graph
	t = ctx.link_context.time_s
	for edge in eachindex(graph.edges)
		score = 0.0
		if graph.available[edge]
			candidate = _link_candidate(model, edge, ctx)
			score = Float64(candidate_score(objective, candidate, ctx, t))
			isfinite(score) || throw(ArgumentError("Nonfinite $(nameof(P)) score for candidate $(candidate.key)."))
		end
		scores[edge] = score
	end
	return scores
end

# 3. Dynamic programming:  Select the best set of interlinks based on the current scores
# Helper Function: given one vertex and a related edge, this helper function of find the other vertex of the edge
other_endpoint(model, edge, vertex) = model.graph.edges[edge][1] == vertex ?
	model.graph.edges[edge][2] : model.graph.edges[edge][1]

# Helper function: select the best set of interlinks in a tree structure
function select_tree!(model, work, first_index, last_index, selected)
	for index in last_index:-1:first_index
		vertex = work.order[index]
		base = BigInt(0)
		for edge in model.graph.vertex_neighbors[vertex]
			work.weights[edge] > 0 || continue
			child = other_endpoint(model, edge, vertex)
			work.parent[child] == vertex || continue
			base += work.free_score[child]
		end
		work.blocked_score[vertex] = base
		gain, chosen = BigInt(0), 0
		for edge in model.graph.vertex_neighbors[vertex]
			work.weights[edge] > 0 || continue
			child = other_endpoint(model, edge, vertex)
			work.parent[child] == vertex || continue
			candidate = work.weights[edge] + work.blocked_score[child] - work.free_score[child]
			if candidate > gain
				gain, chosen = candidate, edge
			end
		end
		work.free_score[vertex] = base + gain
		work.choice[vertex] = chosen
	end
	for index in first_index:last_index
		vertex = work.order[index]
		edge = work.choice[vertex]
		(work.blocked[vertex] || edge == 0) && continue
		work.blocked[other_endpoint(model, edge, vertex)] = true
		push!(selected, map(v -> model.graph.vertices[v], model.graph.edges[edge]))
	end
	return nothing
end


function _select_cyclic!(model, work, selected)
	extension = Base.get_extension(parentmodule(parentmodule(@__MODULE__)), :SpaceAGORAPythonCallExt)
	extension === nothing && throw(ArgumentError(
		"Cyclic interlink matching requires the optional PythonCall backend. " *
		"Install PythonCall in the active Julia environment and load it with `using PythonCall`; " *
		"install NetworkX in the Python environment used by PythonCall. Tree matching works without Python."))
	return extension.select_cyclic!(model, work, selected)
end


"""
	select_interlinks!(model::InterlinkModel, policy::SchedulingPolicyModel)

Activate a maximum-total-score matching of available, positive-score links,
with at most one link per terminal. Of parallel links (directed links both ways
between the same two terminals), only the highest-scoring is a candidate, the
first in endpoint order on a tie. Topology belongs to `model.graph`; `policy.matching`
contains raw per-edge `score` values and reusable solver scratch arrays. Pass the same
policy used by `score_candidates!`; selection uses its stored scores, not its scoring
objective. A fresh workspace starts with zero scores. Appending links preserves existing
scores and initializes new scores to zero when the workspace is refreshed. Switching
graphs or discarding the workspace resets scores; rescore before selecting again.
Use separate policies for independent scheduling runs. Deep-copy the model and policy
together to preserve their workspace binding. Edge order is sorted locally on each
selection, not stored in either structure. Registered topology and iterative DFS
identify tree components, solved with linear-time dynamic programming. Cyclic components
use NetworkX's weighted Blossom algorithm through PythonCall, with O(N^3)
arithmetic operations for N cyclic endpoints. Both solvers use exact BigInt
weights scaled from Float64 scores; no floating-point score sums are compared.
Ties are deterministic for a given backend version, but tree ties may choose a
different optimum than Blossom alone. Register topology through `register_allowed_link!`.
For manual scores, refresh with `matching_workspace!(model, policy)` before editing
its `score` vector. Graph availability and workspace scores are reread on every call.
Tree components work without Python. Cyclic components require the optional
PythonCall package loaded with `using PythonCall`, and NetworkX installed in the
Python environment used by PythonCall. Core loading does not install either backend.
"""
function select_interlinks!(model::InterlinkModel, policy::SchedulingPolicyModel)
	# Step 1. Collect available, positive-score edges in canonical endpoint order.
	work = matching_workspace!(model, policy)
	empty!(work.candidates)
	empty!(work.exact_scores)
	edge_order = sortperm([map(v -> model.graph.vertices[v], endpoints) for endpoints in model.graph.edges])
	for edge in edge_order
		if model.graph.available[edge] && work.score[edge] > 0.0
			isfinite(work.score[edge]) || throw(ArgumentError("Interlink matching scores must be finite."))
			push!(work.candidates, edge)
			push!(work.exact_scores, Rational{BigInt}(work.score[edge]))
		end
	end
	_drop_parallel_candidates!(model, work)
	selected = InterLinkKey[]
	if !isempty(work.candidates)
		# Step 2. Scale exact Float64 rationals to integer weights without rounding score sums.
		# Their denominators are powers of two, so the largest is a common denominator.
		common_denominator = maximum(denominator, work.exact_scores)
		fill!(work.weights, 0)
		for (edge, score) in zip(work.candidates, work.exact_scores)
			work.weights[edge] = numerator(score) * div(common_denominator, denominator(score))
		end
		# Step 3. Traverse positive-weight components with depth-first search; solve trees and mark cycles if detected.
		fill!(work.parent, 0)
		fill!(work.blocked, false)
		fill!(work.cyclic, false)
		empty!(work.order)
		empty!(work.stack)
		for root in sortperm(model.graph.vertices)
			work.parent[root] == 0 || continue
			work.parent[root] = -1
			first_index = length(work.order) + 1
			degree_sum = 0
			push!(work.stack, root)
			while !isempty(work.stack)
				vertex = pop!(work.stack)
				push!(work.order, vertex)
				for edge in model.graph.vertex_neighbors[vertex]
					work.weights[edge] > 0 || continue
					degree_sum += 1
					partner = other_endpoint(model, edge, vertex)
					work.parent[partner] == 0 || continue
					work.parent[partner] = vertex
					push!(work.stack, partner)
				end
			end
			last_index = length(work.order)
			degree_sum == 0 && continue
			# Each undirected edge is counted twice; a connected tree has E = V - 1.
			if degree_sum ÷ 2 == last_index - first_index
				select_tree!(model, work, first_index, last_index, selected)
			else
				for index in first_index:last_index
					work.cyclic[work.order[index]] = true
				end
			end
		end
	end
	# Step 4. Solve cyclic components with NetworkX's maximum-weight Blossom matching.
	if !isempty(work.candidates) && any(work.cyclic)
		_select_cyclic!(model, work, selected)
	end
	# Step 5. Replace the active-link flags with the combined tree and cyclic selections.
	fill!(model.graph.active, false)
	for key in selected
		model.graph.active[model.graph.edge_index_lookup[key]] = true
	end
	return sort!(selected)
end

# Parallel candidates (directed links both ways between the same two terminals)
# share both terminals, so a matching holds at most one: keep the highest-scoring,
# the first in endpoint order on a tie. The components then stay simple graphs,
# and a two-way pair alone stays a tree.
function _drop_parallel_candidates!(model, work)
	length(work.candidates) < 2 && return nothing
	edges = model.graph.edges
	best = Dict{Tuple{Int, Int}, Int}()   # vertex pair => position in the candidates
	keep = fill(true, length(work.candidates))
	for (position, edge) in enumerate(work.candidates)
		pair = minmax(edges[edge]...)
		other = get(best, pair, 0)
		if other == 0
			best[pair] = position
		elseif work.exact_scores[position] > work.exact_scores[other]
			keep[other] = false
			best[pair] = position
		else
			keep[position] = false
		end
	end
	all(keep) && return nothing
	keepat!(work.candidates, keep)
	keepat!(work.exact_scores, keep)
	return nothing
end

# 4. Overall scheduling function to schedule interlinks based on the current state and scoring policy
function schedule_interlinks!(model::InterlinkModel, policy::SchedulingPolicyModel,
	spacecraft, state, mu::Real; candidate_force=nothing, is_active=nothing, context=LinkContext(),
	terminal_available=nothing)
	# step 1. update the availability of each interlink based on the current state
	update_availability!(model, spacecraft, state; is_active, context, terminal_available)

	# step 2. compute the score of each interlink based on the scheduling policy / scoring policy
	score_candidates!(model, policy, state, mu; candidate_force, spacecraft, context)

	# step 3. select the best set of interlinks based on the current scores
	return select_interlinks!(model, policy)
end

end # module SchedulingPolicies
