module InterlinkTerminals

# Interlink terminal hardware on a spacecraft: which mounted actuators are
# terminals and what they are numbered, a run's hardware link endpoints, the
# levels and power their links set, and the shedding that closes their links.
# The link contract and the graph are in InterLinkModels (interlink_graph.jl),
# which comes before the mounted-actuator and power models this module needs;
# its terminal methods for a SpacecraftModel are added here.

using ..Components: LaserTerminal
using ..AbstractTypes: AbstractActuator
using ..SpacecraftModels: SpacecraftModel
using ..InterLinkModels: InterlinkModel, link_terminal, link_terminal_levels, terminal_supports
import ..InterLinkModels: terminal_count, _terminal_device, _terminal_number
using ..ActuatorMounting: mounted_actuators, actuator_context
using ..Actuators: actuator_consumption, actuator_limits
using ..PowerSystems: battery

export mounted_link_terminals, terminal_number

"""
    mounted_link_terminals(spacecraft::SpacecraftModel) -> Vector

The terminal hardware mounted on `spacecraft` (actuators with `link_terminal`),
in mounting order. Each entry gives:

- `number`: its terminal number, after the `n_terminal` virtual terminals;
- `position`: its position among the mounted actuators, the index of its level;
- `link`: the link it is mounted on;
- `terminal`: the actuator.
"""
function mounted_link_terminals(spacecraft::SpacecraftModel)
    out = NamedTuple{(:number, :position, :link, :terminal), Tuple{Int, Int, Any, Any}}[]
    for (position, entry) in enumerate(mounted_actuators(spacecraft))
        link_terminal(entry.actuator) || continue
        push!(out, (number=spacecraft.n_terminal + length(out) + 1, position=position,
                    link=entry.link, terminal=entry.actuator))
    end
    return out
end

terminal_count(spacecraft::SpacecraftModel) =
    spacecraft.n_terminal + count(entry -> link_terminal(entry.actuator), mounted_actuators(spacecraft))

_terminal_device(spacecraft::SpacecraftModel, number::Integer) =
    number <= spacecraft.n_terminal ? nothing : mounted_link_terminals(spacecraft)[number - spacecraft.n_terminal].terminal

"""
    terminal_number(spacecraft::SpacecraftModel, terminal) -> Int

The terminal number of `terminal`, mounted on `spacecraft`, for a link endpoint
`(spacecraft index, terminal number)`. `register_link!` also takes the terminal
itself. Two mounted terminals with equal fields cannot be told apart, so for
those give the number.
"""
function terminal_number(spacecraft::SpacecraftModel, terminal)::Int
    matches = [entry.number for entry in mounted_link_terminals(spacecraft) if entry.terminal === terminal]
    isempty(matches) && throw(ArgumentError(
        "The $(nameof(typeof(terminal))) given as a link endpoint is not terminal hardware mounted on spacecraft " *
        "$(spacecraft.id) (see link_terminal and add_actuator!)."))
    length(matches) == 1 || throw(ArgumentError(
        "Spacecraft $(spacecraft.id) has $(length(matches)) mounted terminals equal to the one given (numbers " *
        "$(join(matches, ", "))): give the terminal number."))
    return only(matches)
end

_terminal_number(spacecraft::SpacecraftModel, terminal::AbstractActuator) = terminal_number(spacecraft, terminal)

# ---- A run's hardware link endpoints -----------------------------------------

"""
    prepare_terminal_endpoints!(model::InterlinkModel, spacecraft) -> Bool

At run setup, record on `model` the hardware endpoints of every candidate link
of the run's spacecraft. For each spacecraft, each endpoint is
`(edge, side, mounted actuator position)`. Each hardware terminal must still be
able to carry its link (`terminal_supports`), at a level within its
`actuator_limits`. Returns whether there are any.
"""
function prepare_terminal_endpoints!(model::InterlinkModel, spacecraft)::Bool
    graph = model.graph
    endpoints = [NTuple{3, Int}[] for _ in eachindex(spacecraft)]
    hardware = Dict{Int, Any}()
    for (edge, vertices) in enumerate(graph.edges), (side, vertex) in enumerate(vertices)
        satellite, number = graph.vertices[vertex]
        model_s = spacecraft[satellite]
        model_s isa SpacecraftModel && number > model_s.n_terminal || continue
        mounted = get!(() -> mounted_link_terminals(model_s), hardware, satellite)
        number - model_s.n_terminal <= length(mounted) || throw(ArgumentError(
            "Interlinks: spacecraft $satellite has no terminal $number (terminal_count $(terminal_count(model_s)))."))
        entry = mounted[number - model_s.n_terminal]
        link = graph.edge_properties[edge]
        terminal_supports(entry.terminal, link) || throw(ArgumentError(
            "Interlinks: terminal $number of spacecraft $satellite, a $(nameof(typeof(entry.terminal))), cannot " *
            "carry a $(nameof(typeof(link))) (terminal_supports)."))
        level = Float64(link_terminal_levels(link)[side])
        low, high = actuator_limits(entry.terminal)
        low <= level <= high || throw(ArgumentError(
            "Interlinks: the $(nameof(typeof(link))) on terminal $number of spacecraft $satellite sets it to level " *
            "$level, outside its limits [$low, $high] (for a LaserTerminal, max_power_w); build the link from the " *
            "terminal, LaserMomentumLink(terminal), or lower its power."))
        push!(endpoints[satellite], (edge, side, entry.position))
    end
    model.terminal_endpoints = endpoints
    return any(!isempty, endpoints)
end

# Whether links drive terminal hardware on spacecraft `satellite`.
@inline link_driven(model, satellite::Int) =
    model !== nothing && satellite <= length(model.terminal_endpoints) && !isempty(model.terminal_endpoints[satellite])

"""
    without_link_driven(levels, model, satellite) -> AbstractVector

`levels` (one per mounted actuator of spacecraft `satellite`) with its
link-driven terminals at zero. Their link drives them for the whole run, so a
controller's levels for them apply nothing: no recoil and no draw. This is a
copy, or `levels` itself when no terminal of that spacecraft is link-driven.
"""
function without_link_driven(levels, model, satellite::Int)
    (levels === nothing || !link_driven(model, satellite)) && return levels
    out = collect(Float64, levels)
    for (_, _, position) in model.terminal_endpoints[satellite]
        out[position] = 0.0
    end
    return out
end

"""
    link_driven_levels!(levels, model, satellite) -> levels

Write into `levels` (one per mounted actuator) the levels that the links set on
the link-driven terminals of spacecraft `satellite`. That is the
`link_terminal_levels` of a terminal's active link, and 0 when none of its
links is active. Other entries are left as they are.
"""
function link_driven_levels!(levels, model, satellite::Int)
    link_driven(model, satellite) || return levels
    graph = model.graph
    for (_, _, position) in model.terminal_endpoints[satellite]
        levels[position] = 0.0
    end
    for (edge, side, position) in model.terminal_endpoints[satellite]
        graph.active[edge] || continue
        levels[position] = Float64(link_terminal_levels(graph.edge_properties[edge])[side])
    end
    return levels
end

"""
    link_terminal_power(model, satellite, spacecraft::SpacecraftModel) -> (by_tier, load_w)

The power (W) the link-driven terminals of spacecraft `satellite` (model
`spacecraft`) ask for and draw.

- `by_tier` is the demand, split as `actuator_power_by_tier` does: laser
  terminals, torque rods, reaction wheels, every other kind. A terminal with an
  active link asks for its link's level. A terminal the battery's shedding has
  closed keeps asking for the highest level of its links, so the tier clears
  only when the battery can supply it again, not because its link closed.
- `load_w` is what is drawn: the active links' levels, less what the shedding
  sheds (tier 1 sheds laser terminals).
"""
function link_terminal_power(model, satellite::Int, spacecraft::SpacecraftModel)
    link_driven(model, satellite) || return ((0.0, 0.0, 0.0, 0.0), 0.0)
    graph = model.graph
    endpoints = model.terminal_endpoints[satellite]
    b = battery(spacecraft)
    tier = b === nothing ? 0 : b.shed_tier
    entries = mounted_actuators(spacecraft)
    lasers = other = load = 0.0
    for position in unique(last(endpoint) for endpoint in endpoints)
        active = false
        level = wanted = 0.0
        for (edge, side, at) in endpoints
            at == position || continue
            this = Float64(link_terminal_levels(graph.edge_properties[edge])[side])
            wanted = max(wanted, this)
            graph.active[edge] && (active = true; level = this)
        end
        entry = entries[position]
        laser = entry.actuator isa LaserTerminal
        shed = laser && tier >= 1
        demand = active ? level : (shed ? wanted : 0.0)
        demand == 0.0 && continue
        w = Float64(actuator_consumption(entry.actuator, demand, actuator_context(entry.link)).power_w)
        laser ? (lasers += w) : (other += w)
        active && !shed && (load += w)
    end
    return ((lasers, 0.0, 0.0, other), load)
end

"""
    terminal_availability(model, spacecraft) -> Union{Nothing, Vector{Bool}}

One flag per graph vertex, false for terminal hardware that its spacecraft's
power budget sheds: a laser terminal while the battery sheds tier 1 or more.
Links through such a terminal are unavailable until the tier clears. Virtual
terminals are always available. Returns `nothing` when the run has no hardware
endpoints.
"""
function terminal_availability(model, spacecraft)
    any(!isempty, model.terminal_endpoints) || return nothing
    graph = model.graph
    flags = fill(true, length(graph.vertices))
    for (vertex, (satellite, number)) in enumerate(graph.vertices)
        model_s = spacecraft[satellite]
        model_s isa SpacecraftModel && number > model_s.n_terminal || continue
        flags[vertex] = !_terminal_shed(model_s, _terminal_device(model_s, number))
    end
    return flags
end

_terminal_shed(spacecraft::SpacecraftModel, terminal) =
    terminal isa LaserTerminal && (b = battery(spacecraft); b !== nothing && b.shed_tier >= 1)

end # module InterlinkTerminals
