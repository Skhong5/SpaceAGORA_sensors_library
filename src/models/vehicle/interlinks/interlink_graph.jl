module InterLinkModels

	using LinearAlgebra
	using StaticArrays
	using ..Components: LaserTerminal, _SPEED_OF_LIGHT_MPS
	using ..EffectorSampling: StateSample
	using ..HookDeprecation: _hook_deprecation, _NOT_PASSED

	export AbstractInterlink, LinkContext, LaserMomentumLink, link_range_m, link_available, link_force
	export InterlinkModel, InterLinkGraph, register_link!, update_availability!, interlink_force_on_spacecraft
	export vertex, InterLinkKey, InterLinkHistory
	export ConstellationInterLinkModel, InterLinkParameters, register_allowed_link!

	include(joinpath(@__DIR__, "links.jl"))

	const vertex = Tuple{Int, Int} # (spacecraft vector index, terminal number)
	const InterLinkKey = Tuple{vertex, vertex} # Canonically ordered pair of terminal endpoints

	"""
	    InterLinkParameters

	Deprecated, use `LaserMomentumLink`: the same type under its old name, which
	will be removed in SpaceAGORA 1.0. The name is an alias and cannot warn on its
	own: `InterLinkParameters()`, `InterLinkParameters(terminal)` and the positional
	`InterLinkParameters(P, B, range)` run silently. Its keywords `P`, `B` and
	`range` and the fields of those names (now `power_w`, `magnification` and
	`range_m`) warn.
	"""
	const InterLinkParameters = LaserMomentumLink

	# 1.1 Candidate graph (Vertices are satellite terminals; Edges are allowed interlinks)
	mutable struct InterLinkGraph
		# i) Core graph data structures
		vertices::Vector{vertex} # vertices[v] = (satellite, terminal)
		edges::Vector{Tuple{Int, Int}} # edges[e] = (first vertex index, second vertex index)

		# ii) The link of each edge (its kind and parameters)
		edge_properties::Vector{AbstractInterlink}

		# iii) Edge state (availability does not imply scheduler selection)
		available::Vector{Bool} # Within range, not forbidden, and both spacecraft active if flags are supplied
		active::Vector{Bool} # Links selected by the scheduler

		# iv) Auxiliary indexes for efficient graph operations
		vertex_neighbors::Vector{Vector{Int}} # Incident edge indexes for each terminal vertex
		edge_index_lookup::Dict{InterLinkKey, Int} # Canonical endpoint pair => edge index
		satellite_edges::Vector{Vector{Int}} # Incident edge indexes across all terminals of each spacecraft

		InterLinkGraph() = new(vertex[], Tuple{Int, Int}[], AbstractInterlink[],
			Bool[], Bool[], Vector{Int}[], Dict{InterLinkKey, Int}(), Vector{Int}[])
	end

	# 1.2. Interlink history
	struct InterLinkHistory
		time::Float64
		active::Vector{InterLinkKey}
	end

	# 1.3. Interlink model
	"""
	    InterlinkModel(; forbidden_pairs=())

	The interlinks of a run: the candidate links between spacecraft terminals,
	each with its own link kind (`register_link!`), spacecraft pairs that may never
	link, and the accepted-step history of the links the scheduler selected. Attach
	it with `SimulationConfiguration(interlink_model=model, ...)`. Endpoints are
	`(spacecraft vector index, terminal number)` pairs: the position of the
	spacecraft in `dynamics_model.spacecraft`, not its `id`.
	"""
	mutable struct InterlinkModel
		# i) Core model parameters
		forbidden_pairs::Set{Tuple{Int, Int}}
		graph::InterLinkGraph

		# ii) Scheduling history (independent of the selection algorithm)
		history::Vector{InterLinkHistory}

		function InterlinkModel(; forbidden_pairs=Tuple{Int, Int}[], constellation_link_type=_NOT_PASSED)
			if constellation_link_type !== _NOT_PASSED
				_interlink_deprecation((InterlinkModel, :constellation_link_type),
					"The constellation_link_type keyword is deprecated and will be removed in SpaceAGORA 1.0: " *
					"each registered link now has its own kind (register_link!).")
				constellation_link_type === :laser || throw(ArgumentError(
					"constellation_link_type=$(repr(constellation_link_type)) is not supported: only laser momentum " *
					"links (LaserMomentumLink) are implemented."))
			end
			pairs = Set{Tuple{Int, Int}}(minmax(pair...) for pair in forbidden_pairs)
			new(pairs, InterLinkGraph(), InterLinkHistory[])
		end
	end

	"""
	    ConstellationInterLinkModel

	Deprecated, use `InterlinkModel`: the same type under its old name, which will
	be removed in SpaceAGORA 1.0. The name is an alias and cannot warn on its own:
	`ConstellationInterLinkModel(; forbidden_pairs)` runs silently. Its
	`constellation_link_type` keyword and property warn: each registered link has
	its own kind.
	"""
	const ConstellationInterLinkModel = InterlinkModel

	# `model.constellation_link_type`: every link of an `InterlinkModel` is a laser
	# momentum link or a custom kind; the old property reads `:laser`.
	@inline function Base.getproperty(model::InterlinkModel, name::Symbol)
		if name === :constellation_link_type
			_interlink_deprecation((InterlinkModel, :constellation_link_type),
				"InterlinkModel.constellation_link_type is deprecated and will be removed in SpaceAGORA 1.0: " *
				"each registered link has its own kind (graph.edge_properties).")
			return :laser
		end
		return getfield(model, name)
	end

	# 2. Register a allowed pair as for vertices and edges in graph without checking its current availability
	"""
	    register_link!(model::InterlinkModel, spacecraft, first_endpoint, second_endpoint, link::AbstractInterlink) -> InterLinkKey

	Register a candidate link of kind `link` between two terminals, each a
	`(spacecraft vector index, terminal number)` pair, and return its canonical
	key. The terminals must be on different spacecraft and within each
	spacecraft's `n_terminal`; a pair registers once. The link is available only
	when `link_available` allows it, and active only when the scheduler selects it.
	"""
	function register_link!(model::InterlinkModel, spacecraft,
		first_endpoint::vertex, second_endpoint::vertex, link::AbstractInterlink)

		# 2.1. Validate the terminals before changing the graph.
		# i) Rule 1: The interlink must join different spacecraft.
		first_endpoint[1] != second_endpoint[1] || throw(ArgumentError("An interlink must join different spacecraft."))

		# ii) Rule 2: Both terminal numbers must be valid for their spacecraft.
		for (satellite, terminal) in (first_endpoint, second_endpoint)
			1 <= satellite <= length(spacecraft) || throw(ArgumentError("Invalid spacecraft vector index $satellite."))
			1 <= terminal <= spacecraft[satellite].n_terminal || throw(ArgumentError("Invalid terminal $terminal on spacecraft $satellite."))
		end

		# iii) Rule 3: Sort endpoints so reversing a pair produces the same key.
		key = minmax(first_endpoint, second_endpoint)

		# iv) Rule 4: The candidate must not already be registered.
		graph = model.graph
		haskey(graph.edge_index_lookup, key) && throw(ArgumentError("Candidate $key is already registered."))

		# 2.2. Connect terminal-pairs, and give the new link its own parameters and initial state.
		# i) Locate the terminals, put them into the graph's vertex list
		indices = map(key) do endpoint
			index = findfirst(==(endpoint), graph.vertices)
			if index === nothing
				push!(graph.vertices, endpoint)
				push!(graph.vertex_neighbors, Int[])
				index = length(graph.vertices)
			end
			index
		end

		# ii) Add the new edge (interlink) to the graph's edge list (interlink list) and initialize the properties and state of that edge
		push!(graph.edges, indices)
		push!(graph.edge_properties, link)
		push!(graph.available, false)
		push!(graph.active, false)
		edge = length(graph.edges)

		# iii) Record the lookup
		graph.edge_index_lookup[key] = edge

		# 2.3. Keep terminal adjacency sorted by endpoint key for deterministic matching.
		for index in indices
			neighbors = graph.vertex_neighbors[index]
			position = searchsortedfirst(neighbors, edge;
				by=e -> map(v -> graph.vertices[v], graph.edges[e]))
			insert!(neighbors, position, edge)
		end

		# 2.4. Add the edge to both spacecraft's adjacency lists, including empty lists for unlinked spacecraft.
		while length(graph.satellite_edges) < length(spacecraft)
			push!(graph.satellite_edges, Int[])
		end
		for (satellite, _) in key
			push!(graph.satellite_edges[satellite], edge)
		end

		return key
	end

	"""
	    register_allowed_link!(model, spacecraft, first_endpoint, second_endpoint; parameters=InterLinkParameters())

	Deprecated, use `register_link!`: registers a `LaserMomentumLink` given as
	`parameters`. The old name will be removed in SpaceAGORA 1.0.
	"""
	function register_allowed_link!(model::InterlinkModel, spacecraft,
		first_endpoint::vertex, second_endpoint::vertex; parameters::AbstractInterlink=LaserMomentumLink())
		_interlink_deprecation((InterlinkModel, :register_allowed_link!),
			"register_allowed_link! is deprecated and will be removed in SpaceAGORA 1.0; use " *
			"register_link!(model, spacecraft, first_endpoint, second_endpoint, link) instead.")
		return register_link!(model, spacecraft, first_endpoint, second_endpoint, parameters)
	end

	# The model of spacecraft `index`, when the run's models are known.
	_spacecraft_at(spacecraft, index::Int) = spacecraft === nothing ? nothing : spacecraft[index]

	# The forces the link of `edge` puts on its two endpoints, in key order.
	function link_endpoint_forces(model::InterlinkModel, edge::Int, state; spacecraft=nothing, context=LinkContext())
		graph = model.graph
		first, second = map(v -> graph.vertices[v], graph.edges[edge])
		a = link_endpoint(state.sc[first[1]], _spacecraft_at(spacecraft, first[1]), first[1])
		b = link_endpoint(state.sc[second[1]], _spacecraft_at(spacecraft, second[1]), second[1])
		return link_force(graph.edge_properties[edge], a, b, context)
	end

	# 3. Update link availability
	"""
		update_availability!(model::InterlinkModel, spacecraft, state; is_active=nothing, context=LinkContext())

	Mark each candidate link available when its spacecraft pair is not forbidden,
	both spacecraft are active (when activity flags are supplied) and its link's
	`link_available` allows it (by default, the endpoints are apart and within
	`link_range_m`). Battery and temperature do not gate availability. Return a
	fresh `Vector{Bool}` in registration order.
	"""
	function update_availability!(model::InterlinkModel, spacecraft, state; is_active=nothing, context=LinkContext())
		graph = model.graph
		for edge in eachindex(graph.edges)
			first, second = map(v -> graph.vertices[v], graph.edges[edge])

			# i) Rule 1: The spacecraft pair must not be forbidden.
			allowed = !((first[1], second[1]) in model.forbidden_pairs) &&

				# ii) Rule 2: Both spacecraft must be active when activity flags are supplied.
				(is_active === nothing || (is_active[first[1]] && is_active[second[1]]))

			# iii) Rule 3: The link itself must be able to close (by default, within its range).
			graph.available[edge] = allowed && link_available(graph.edge_properties[edge],
				link_endpoint(state.sc[first[1]], _spacecraft_at(spacecraft, first[1]), first[1]),
				link_endpoint(state.sc[second[1]], _spacecraft_at(spacecraft, second[1]), second[1]), context)
		end
		return copy(graph.available)
	end

	# 4. The force of the active links on one spacecraft
	"""
		interlink_force_on_spacecraft(model::InterlinkModel, state, satellite; is_active=nothing, spacecraft=nothing, context=LinkContext())

	The sum of the inertial forces (N) the selected links put on spacecraft
	`satellite` (its vector index), each from its link's `link_force`. Optional
	`is_active` flags mask links with an inactive endpoint.
	"""
	function interlink_force_on_spacecraft(model::InterlinkModel, state, satellite::Int; is_active=nothing,
		spacecraft=nothing, context=LinkContext())
		force = SVector{3, Float64}(0.0, 0.0, 0.0)
		1 <= satellite <= length(model.graph.satellite_edges) || return force
		for index in model.graph.satellite_edges[satellite]
			key = map(v -> model.graph.vertices[v], model.graph.edges[index])
			model.graph.active[index] || continue
			is_active === nothing || (is_active[key[1][1]] && is_active[key[2][1]]) || continue
			first_force, second_force = link_endpoint_forces(model, index, state; spacecraft, context)
			if key[1][1] == satellite
				force += first_force
			elseif key[2][1] == satellite
				force += second_force
			end
		end
		return force
	end

end
