module SchedulingPolicies

using LinearAlgebra
using ..InterLinkModels
using ..InterLinkModels: link_endpoint_forces

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

# 2. Scheduling policy for interlink selection
# 2.1. Scheduling policy model
"""
	SchedulingPolicyModel(policy; target_idx=1, target_idxs=nothing)

Create a scheduling policy. For `:gve_sma` and `:gve_eccentricity`, `target_idx`
selects one target and `target_idx=nothing` scores both link endpoints.
For `:gve_sma`, `target_idxs` optionally selects a set of targets and takes
precedence over `target_idx`; all selected indices must exist in the simulation state.
"""
mutable struct SchedulingPolicyModel
	const policy::Symbol
	const target_idx::Union{Nothing, Int}
	const target_idxs::Union{Nothing, Vector{Int}}
	matching::Union{Nothing, InterLinkMatchingWorkspace}

	function SchedulingPolicyModel(policy::Symbol; target_idx::Union{Nothing, Integer}=1,
		target_idxs::Union{Nothing, AbstractVector{<:Integer}}=nothing)
		policy in (:gve_sma, :gve_eccentricity, :gve_sma_first_index) ||
			throw(ArgumentError("Supported scheduling policies: :gve_sma, :gve_eccentricity, and :gve_sma_first_index."))
		target_idxs === nothing || policy === :gve_sma ||
			throw(ArgumentError("target_idxs is supported only by the :gve_sma policy."))
		target_idx === nothing || target_idx > 0 ||
			throw(ArgumentError("target_idx must be nothing or a positive spacecraft vector index."))
		target_indices = target_idxs === nothing ? nothing : Int[target_idxs...]
		if target_indices !== nothing
			isempty(target_indices) && throw(ArgumentError("target_idxs must not be empty."))
			all(>(0), target_indices) || throw(ArgumentError("target_idxs must contain only positive indices."))
			length(unique(target_indices)) == length(target_indices) ||
				throw(ArgumentError("target_idxs must not contain duplicates."))
			sort!(target_indices)
		end
		new(policy, target_idx === nothing ? nothing : Int(target_idx), target_indices, nothing)
	end
end

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

# The force candidate link `edge` puts on spacecraft `target_idx`: from the link's
# `link_force`, or from a caller's `candidate_force(target, partner, link)`.
function _candidate_force(candidate_force, model, edge::Int, target_idx::Int, partner_idx::Int, state, spacecraft, context)
	candidate_force === nothing ||
		return candidate_force(state.sc[target_idx], state.sc[partner_idx], model.graph.edge_properties[edge])
	first_force, second_force = link_endpoint_forces(model, edge, state; spacecraft, context)
	first_satellite = model.graph.vertices[model.graph.edges[edge][1]][1]
	return target_idx == first_satellite ? first_force : second_force
end

# POLICY 1. Increase Target Satellite Semimajor Axis Rate
function score_gve_sma!(model::InterlinkModel, policy::SchedulingPolicyModel, state, mu::Real;
	candidate_force=nothing, spacecraft=nothing, context=LinkContext())
	function semimajor_axis_rate(target, force, mu::Real)
		semimajor_axis = inv(2 / norm(target.pos) - dot(target.vel, target.vel) / mu)
		return 2 * semimajor_axis^2 / mu * dot(target.vel, force) / target.mass
	end

	isfinite(mu) && mu > 0 || throw(ArgumentError("The gravitational parameter must be finite and positive."))
	policy.target_idxs === nothing || all(index -> index <= length(state.sc), policy.target_idxs) ||
		throw(ArgumentError("target_idxs contains an index outside the spacecraft state vector."))
	policy.target_idxs !== nothing || policy.target_idx === nothing || policy.target_idx <= length(state.sc) ||
		throw(ArgumentError("target_idx is outside the spacecraft state vector."))
	work = matching_workspace!(model, policy)
	for index in eachindex(model.graph.edges)
		key = map(v -> model.graph.vertices[v], model.graph.edges[index])
		score = 0.0
		first_satellite, second_satellite = key[1][1], key[2][1]
		if model.graph.available[index]
			for (target_idx, partner_idx) in ((first_satellite, second_satellite), (second_satellite, first_satellite))
				if policy.target_idxs === nothing
					policy.target_idx === nothing || policy.target_idx == target_idx || continue
				else
					target_idx in policy.target_idxs || continue
				end
				target = state.sc[target_idx]
				partner = state.sc[partner_idx]
				force = _candidate_force(candidate_force, model, index, target_idx, partner_idx, state, spacecraft, context)
				score += semimajor_axis_rate(target, force, mu)
			end
		end
		isfinite(score) || throw(ArgumentError("Nonfinite $(policy.policy) score for candidate $key."))
		work.score[index] = score
	end
	return nothing
end

# Near circularity the existing score is the norm of the eccentricity-vector
# rate. Norms from two simultaneous forces are not additive: opposite forces
# can receive positive individual scores even though their combined rate is zero.
# Admit this objective only when a scored near-circular spacecraft cannot receive
# two positive-score candidate links in the same terminal matching.
function _validate_eccentricity_matching_objective!(model, work, circular_candidates)
	for (satellite, edges) in circular_candidates
		candidates = Tuple{Int, Int}[]
		for edge in edges
			model.graph.available[edge] && work.score[edge] > 0 || continue
			endpoints = model.graph.edges[edge]
			for previous in candidates
				if all(vertex -> vertex != endpoints[1] && vertex != endpoints[2], previous)
					throw(ArgumentError("The :gve_eccentricity objective does not support simultaneous links " *
						"on near-circular target spacecraft $satellite: per-link eccentricity-rate norms " *
						"are not additive. Use candidate links that share one terminal on that target."))
				end
			end
			push!(candidates, endpoints)
		end
	end
	return nothing
end

# POLICY 2. Increase Target Satellite Eccentricity Rate
"""
Score the selected spacecraft's eccentricity rate. At eccentricity no greater
than `sqrt(eps(Float64))`, the retained per-link score is an eccentricity-vector
rate norm. Reject candidate matchings that can apply multiple positive-score
links to the same such target, because summing those norms is not the rate of
the combined force. The scalar projected rate remains additive away from this
near-circular branch. This restriction does not apply to the SMA policies.
"""
function score_gve_eccentricity!(model::InterlinkModel, policy::SchedulingPolicyModel, state, mu::Real;
	candidate_force=nothing, spacecraft=nothing, context=LinkContext())
	function eccentricity_rate(target, force, mu::Real)
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

	isfinite(mu) && mu > 0 || throw(ArgumentError("The gravitational parameter must be finite and positive."))
	policy.target_idx === nothing || policy.target_idx <= length(state.sc) ||
		throw(ArgumentError("target_idx is outside the spacecraft state vector."))
	work = matching_workspace!(model, policy)
	circular_candidates = Dict{Int, Vector{Int}}()
	for index in eachindex(model.graph.edges)
		key = map(v -> model.graph.vertices[v], model.graph.edges[index])
		score = 0.0
		first_satellite, second_satellite = key[1][1], key[2][1]
		if model.graph.available[index]
			for (target_idx, partner_idx) in ((first_satellite, second_satellite), (second_satellite, first_satellite))
				policy.target_idx === nothing || policy.target_idx == target_idx || continue
				target = state.sc[target_idx]
				partner = state.sc[partner_idx]
				force = _candidate_force(candidate_force, model, index, target_idx, partner_idx, state, spacecraft, context)
				rate, near_circular = eccentricity_rate(target, force, mu)
				score += rate
				if near_circular && rate > 0
					push!(get!(Vector{Int}, circular_candidates, target_idx), index)
				end
			end
		end
		isfinite(score) || throw(ArgumentError("Nonfinite $(policy.policy) score for candidate $key."))
		work.score[index] = score
	end
	_validate_eccentricity_matching_objective!(model, work, circular_candidates)
	return nothing
end

# POLICY 3: Increase the SMA of the lower-index satellite in each interlink pair.
"""
Score each available link by the SMA rate of its lower spacecraft vector index,
regardless of altitude or registration order. The target is chosen per pair,
so `policy.target_idx` is ignored. Nonpositive scores are excluded by the matcher.
"""
function score_gve_sma_first_index!(model::InterlinkModel, policy::SchedulingPolicyModel, state, mu::Real;
	candidate_force=nothing, spacecraft=nothing, context=LinkContext())
	isfinite(mu) && mu > 0 || throw(ArgumentError("The gravitational parameter must be finite and positive."))
	work = matching_workspace!(model, policy)
	for index in eachindex(model.graph.edges)
		key = map(v -> model.graph.vertices[v], model.graph.edges[index])
		score = 0.0
		if model.graph.available[index]
			target_idx, partner_idx = minmax(key[1][1], key[2][1])
			target = state.sc[target_idx]
			partner = state.sc[partner_idx]
			force = _candidate_force(candidate_force, model, index, target_idx, partner_idx, state, spacecraft, context)
			semimajor_axis = inv(2 / norm(target.pos) - dot(target.vel, target.vel) / mu)
			score = 2 * semimajor_axis^2 / mu * dot(target.vel, force) / target.mass
		end
		isfinite(score) || throw(ArgumentError("Nonfinite $(policy.policy) score for candidate $key."))
		work.score[index] = score
	end
	return nothing
end

# 2.2. Define the function to score candidate interlinks based on the scheduling policy
# The scores follow each link's `link_force` (`candidate_force=nothing`), or a
# caller's `candidate_force(target, partner, link)` giving the force on `target`.
function score_candidates!(model::InterlinkModel, policy::SchedulingPolicyModel, state, mu::Real;
	# compute the score of each interlink based on the scheduling policy / scoring policy
	# resulting scores will be stored in the policy's workspace
	candidate_force=nothing, spacecraft=nothing, context=LinkContext())
	if policy.policy === :gve_sma
		return score_gve_sma!(model, policy, state, mu; candidate_force, spacecraft, context)
	elseif policy.policy === :gve_eccentricity
		return score_gve_eccentricity!(model, policy, state, mu; candidate_force, spacecraft, context)
	elseif policy.policy === :gve_sma_first_index
		return score_gve_sma_first_index!(model, policy, state, mu; candidate_force, spacecraft, context)
	end
	throw(ArgumentError("Unsupported scheduling policy: $(policy.policy)."))
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
with at most one link per terminal. Topology belongs to `model.graph`; `policy.matching`
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
