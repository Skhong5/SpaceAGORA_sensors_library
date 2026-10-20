module InterLinkModels

	using LinearAlgebra
	using StaticArrays
	using ..Components: LaserTerminal, _SPEED_OF_LIGHT_MPS
	using ..AbstractTypes: AbstractActuator
	using ..EffectorSampling: StateSample
	using ..HookDeprecation: _hook_deprecation, _NOT_PASSED

	export AbstractInterlink, LinkContext, LaserMomentumLink, link_range_m, link_available, link_force
	export link_directed, link_terminal_levels, link_terminal, terminal_supports, terminal_count
	export register_links!, all_pairs, same_plane_pairs, ring_pairs, grid_pairs
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
	spacecraft in `dynamics_model.spacecraft`, not its `id`. Terminals
	`1:n_terminal` are virtual; mounted terminal hardware (see `link_terminal`)
	follows.
	"""
	mutable struct InterlinkModel
		# i) Core model parameters
		forbidden_pairs::Set{Tuple{Int, Int}}
		graph::InterLinkGraph

		# ii) Scheduling history (independent of the selection algorithm)
		history::Vector{InterLinkHistory}

		# iii) The run's hardware link endpoints, set at run setup: for each spacecraft,
		# `(edge, side, mounted actuator position)` of each terminal hardware endpoint.
		terminal_endpoints::Vector{Vector{NTuple{3, Int}}}

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
			new(pairs, InterLinkGraph(), InterLinkHistory[], Vector{NTuple{3, Int}}[])
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

	Register a candidate link of kind `link` between two terminals and return its
	key. Each endpoint is `(spacecraft vector index, terminal)`, where the terminal
	is a terminal number or a terminal mounted on that `SpacecraftModel`. The
	terminals must be on different spacecraft, numbered within each spacecraft's
	`terminal_count`, and able to carry the link (`terminal_supports`). A link
	registers once. Its key lists the lower `(spacecraft, terminal)` first, or,
	for a directed link (`link_directed`), the endpoints in the order given. The
	link is available only when `link_available` allows it, and active only when
	the scheduler selects it.
	"""
	function register_link!(model::InterlinkModel, spacecraft,
		first_endpoint::Tuple{Integer, Any}, second_endpoint::Tuple{Integer, Any}, link::AbstractInterlink)

		# 2.1. Validate the terminals before changing the graph.
		# i) Rule 1: The interlink must join different spacecraft.
		first_endpoint[1] != second_endpoint[1] || throw(ArgumentError("An interlink must join different spacecraft."))

		# ii) Rule 2: Both terminals must be valid for their spacecraft and able to carry the link.
		endpoints = map((first_endpoint, second_endpoint)) do (satellite, terminal)
			1 <= satellite <= length(spacecraft) || throw(ArgumentError("Invalid spacecraft vector index $satellite."))
			number = _terminal_number(spacecraft[satellite], terminal)
			1 <= number <= terminal_count(spacecraft[satellite]) ||
				throw(ArgumentError("Invalid terminal $number on spacecraft $satellite."))
			device = _terminal_device(spacecraft[satellite], number)
			device === nothing || terminal_supports(device, link) || throw(ArgumentError(
				"Terminal $number of spacecraft $satellite, a $(nameof(typeof(device))), cannot carry a " *
				"$(nameof(typeof(link))) (terminal_supports)."))
			(Int(satellite), number)
		end

		# iii) Rule 3: An undirected link is keyed lower endpoint first, so reversing a pair gives
		# the same key; a directed one keeps its transmitter first.
		key = link_directed(link) ? endpoints : minmax(endpoints...)

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

	"""
	    all_pairs(spacecraft) -> Vector{Tuple{Int, Int}}

	Every pair `(i, j)` of distinct spacecraft vector indices with `i < j`, in
	order: the candidate pairs of a fully connected interlink graph, for
	`register_links!`.
	"""
	all_pairs(spacecraft) = [(i, j) for i in eachindex(spacecraft) for j in eachindex(spacecraft) if i < j]

	# The spacecraft with a constellation slot, by plane: plane => [(slot, vector index)], by slot.
	function _slots_by_plane(spacecraft)
		planes = Dict{Int, Vector{Tuple{Int, Int}}}()
		for (index, sc) in enumerate(spacecraft)
			slot = hasproperty(sc, :constellation_slot) ? sc.constellation_slot : nothing
			slot === nothing && continue
			push!(get!(Vector{Tuple{Int, Int}}, planes, slot.plane), (slot.slot, index))
		end
		foreach(sort!, values(planes))
		return planes
	end

	"""
	    same_plane_pairs(spacecraft) -> Vector{Tuple{Int, Int}}

	Every pair `(i, j)`, `i < j`, of spacecraft in the same orbital plane, by their
	`constellation_slot` (set by `build_constellation`), for `register_links!`.
	Spacecraft without a slot take part in no pair.
	"""
	function same_plane_pairs(spacecraft)
		pairs = Tuple{Int, Int}[]
		for members in values(_slots_by_plane(spacecraft)), a in eachindex(members), b in a+1:length(members)
			push!(pairs, minmax(members[a][2], members[b][2]))
		end
		return sort!(unique!(pairs))
	end

	"""
	    ring_pairs(spacecraft) -> Vector{Tuple{Int, Int}}

	Each spacecraft with the next slot in its plane, and the last slot with the
	first: the in-plane rings of a constellation, by `constellation_slot`. A plane
	of two gives one pair, a plane of one none. Pairs are `(i, j)` with `i < j`.
	"""
	function ring_pairs(spacecraft)
		pairs = Tuple{Int, Int}[]
		for members in values(_slots_by_plane(spacecraft))
			n = length(members)
			n < 2 && continue
			for k in 1:n
				push!(pairs, minmax(members[k][2], members[mod1(k + 1, n)][2]))
			end
		end
		return sort!(unique!(pairs))
	end

	"""
	    grid_pairs(spacecraft; wrap_planes=true) -> Vector{Tuple{Int, Int}}

	The "+grid" of a constellation, by `constellation_slot`: the in-plane rings
	(`ring_pairs`) and each slot with the same slot of the next plane, planes taken
	in plane-number order. With `wrap_planes`, the last plane links to the first;
	pass `false` for a Walker star, whose first and last planes counter-rotate. A
	slot missing from the next plane links nothing across. Pairs are `(i, j)` with
	`i < j`.
	"""
	function grid_pairs(spacecraft; wrap_planes::Bool=true)
		planes = _slots_by_plane(spacecraft)
		pairs = ring_pairs(spacecraft)
		order = sort!(collect(keys(planes)))
		for (k, plane) in enumerate(order)
			k == length(order) && !wrap_planes && break
			next = order[mod1(k + 1, length(order))]
			next == plane && continue
			across = Dict(planes[next])
			for (slot, index) in planes[plane]
				haskey(across, slot) && push!(pairs, minmax(index, across[slot]))
			end
		end
		return sort!(unique!(pairs))
	end

	"""
	    register_links!(model::InterlinkModel, spacecraft, pairs, link; terminals=nothing) -> Vector{InterLinkKey}

	Register one candidate link per spacecraft pair `(i, j)` in `pairs` and return
	their keys in order. The pairs come from `all_pairs(spacecraft)`, from a
	constellation's planes and slots (`same_plane_pairs`, `ring_pairs`,
	`grid_pairs`), or from any vector of index pairs. `link` is a link kind value, or a function
	`(i, j) -> link` that builds each pair's link.

	Each spacecraft's links take the terminals that can carry them in turn,
	starting again from the first when they run out; links that share a terminal
	compete for it when the scheduler selects. Pass
	`terminals=(i, j) -> (terminal_i, terminal_j)` to choose them instead. For a
	directed link, `i` is the transmitter.
	"""
	function register_links!(model::InterlinkModel, spacecraft, pairs, link; terminals=nothing)
		turn = zeros(Int, length(spacecraft))
		keys = InterLinkKey[]
		for (i, j) in pairs
			pair_link = link isa AbstractInterlink ? link : link(i, j)
			pair_link isa AbstractInterlink || throw(ArgumentError(
				"register_links!: the link for pair ($i, $j) is a $(typeof(pair_link)), not a link kind (AbstractInterlink)."))
			ends = terminals === nothing ?
				(_next_terminal!(turn, spacecraft, i, pair_link), _next_terminal!(turn, spacecraft, j, pair_link)) :
				terminals(i, j)
			push!(keys, register_link!(model, spacecraft, (i, ends[1]), (j, ends[2]), pair_link))
		end
		return keys
	end

	# The next terminal of spacecraft `satellite` that can carry `link`, in turn.
	function _next_terminal!(turn::Vector{Int}, spacecraft, satellite::Integer, link)
		1 <= satellite <= length(spacecraft) || throw(ArgumentError("Invalid spacecraft vector index $satellite."))
		usable = filter(1:terminal_count(spacecraft[satellite])) do number
			device = _terminal_device(spacecraft[satellite], number)
			device === nothing || terminal_supports(device, link)
		end
		isempty(usable) && throw(ArgumentError(
			"Spacecraft $satellite has no terminal that can carry a $(nameof(typeof(link)))."))
		turn[satellite] += 1
		return usable[mod1(turn[satellite], length(usable))]
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
		update_availability!(model::InterlinkModel, spacecraft, state; is_active=nothing, context=LinkContext(),
		                     terminal_available=nothing)

	Mark each candidate link available when its spacecraft pair is not forbidden,
	both spacecraft are active (when activity flags are supplied), both terminals
	are available (when `terminal_available` gives a flag per graph vertex: in a
	run, terminal hardware the power budget sheds is not), and its link's
	`link_available` allows it (by default, the endpoints are apart and within
	`link_range_m`). Return a fresh `Vector{Bool}` in registration order.
	"""
	function update_availability!(model::InterlinkModel, spacecraft, state; is_active=nothing, context=LinkContext(),
		terminal_available=nothing)
		graph = model.graph
		for edge in eachindex(graph.edges)
			first, second = map(v -> graph.vertices[v], graph.edges[edge])

			# i) Rule 1: The spacecraft pair must not be forbidden.
			allowed = !(minmax(first[1], second[1]) in model.forbidden_pairs) &&

				# ii) Rule 2: Both spacecraft must be active when activity flags are supplied.
				(is_active === nothing || (is_active[first[1]] && is_active[second[1]])) &&

				# iii) Rule 3: Both terminals must be available when terminal flags are supplied.
				(terminal_available === nothing ||
					(terminal_available[graph.edges[edge][1]] && terminal_available[graph.edges[edge][2]]))

			# iv) Rule 4: The link itself must be able to close (by default, within its range).
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
