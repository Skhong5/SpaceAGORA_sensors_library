# Interlinks

Use this page to connect spacecraft through interlinks, with a policy that
selects which links are active.

What to read next:

- [Configuring a Simulation](simulation_configuration.md)
- [Parallel Execution](parallel_execution.md)

## Interlinks and constellations

An **interlink** is a connection between two spacecraft terminals that does
something while the geometry allows it: push the spacecraft apart, and in later
releases carry power or data. A **constellation** is a set of spacecraft and
their orbits, laid out and run together. They are different things: a
constellation needs no interlinks, and two spacecraft can be linked without
forming one.

## Links and link kinds

Each candidate link has its own kind, a value of a subtype of
`AbstractInterlink`. The kind decides when the link can close and what it does
to its two endpoints:

- `link_range_m(link)`: the maximum separation in metres. Every kind
  implements it.
- `link_available(link, a, b, ctx)`: whether the link can close. By default,
  the endpoints are apart and within `link_range_m`.
- `link_force(link, a, b, ctx)`: the inertial forces (N) on `a` and `b` while
  the link is active. By default, none.

`a` and `b` are the endpoints' state samples (`StateSample`: position,
velocity, mass, attitude and `spacecraft_index`), in the order of the link's
key, the lower `(spacecraft, terminal)` pair first. `ctx` is a `LinkContext`
holding the time (s into the run) and the run's planet.

The built-in kind is `LaserMomentumLink(; power_w, magnification, range_m)`: an
open-cavity laser link with the idealized central force
`magnification * power_w / c`, directed away from the other endpoint, equal and
opposite on the two spacecraft. `LaserMomentumLink(terminal)` takes the numbers
of a `LaserTerminal`.

## Setting up interlinks

An `InterlinkModel` holds the candidate links, spacecraft pairs that may never
link, and the history of the links the scheduler selected. Endpoints are
`(spacecraft vector index, terminal number)` pairs: the spacecraft's position
in `dynamics_model.spacecraft`, not its `id`. Each terminal can take part in at
most one selected link.

```julia
model = InterlinkModel()
for partner in 2:3
    register_link!(model, spacecraft, (1, 1), (partner, 1),
        LaserMomentumLink(power_w=10_000.0, magnification=100.0, range_m=200e3))
end
args = SimulationConfiguration(...; interlink_model=model,
    scheduling_policy_model=SchedulingPolicyModel(:gve_sma))
```

Start with `examples/gve_sma_interlinks.jl`. It builds a three-spacecraft case,
registers two candidate links, and selects the link that increases the target's
instantaneous semimajor-axis rate. `examples/targeted_40sat_sma_interlinks.jl`
uses the same interface with thirty helpers and ten targets. These are analytical
atmosphere examples; they do not require native GRAM or private telemetry.

```julia
include("examples/gve_sma_interlinks.jl")
args = gve_sma_configuration(duration=120.0)
solution = run_simulation(args; return_solution=true)
history = solution.prob.p.args.interlink_model.history
```

## Scheduling

At the start of the run and after every accepted solver step, the engine marks
a candidate available when its spacecraft pair is not forbidden, both
spacecraft are active, and its link's `link_available` allows it. The scheduling
policy then scores the available links by the forces their `link_force` gives,
and selects the terminal matching with the largest total score. Only links with
a positive score can be selected. The selected schedule is held during trial
stages, which avoids changing it during rejected steps. An inactive endpoint
contributes no force even if an earlier schedule still names its link.

The saved `laser_dv` vector integrates the interlink acceleration.
`laser_delta_sma` integrates its contribution to the instantaneous osculating
semimajor-axis rate; it is not a comparison against a separately propagated
unforced trajectory. The history belongs to the returned run configuration.
With the default state isolation, the input configuration is unchanged.

### Optional matching backend

Tree-shaped components are solved in Julia. Cyclic components use NetworkX's
maximum-weight matching through the optional PythonCall extension. Core loading
and tree matching do not initialize Python. A cyclic component without the
backend produces an installation error rather than using an approximate match.

Install PythonCall in the Julia environment used for the run, and install
NetworkX (`>=3.4,<4`) in a compatible Python environment. Set
`JULIA_PYTHONCALL_EXE` to that interpreter before starting Julia; when managing
Python yourself, also set `JULIA_CONDAPKG_BACKEND=Null`. Then explicitly load
`PythonCall` and `SpaceAGORA` in either order. For example, in a separate Julia
project containing SpaceAGORA:

```julia
using Pkg
Pkg.add("PythonCall")
using SpaceAGORA, PythonCall
```

The backend requires Python's global interpreter lock. Run cyclic matching on
its owning thread, or use independent process workers. Merely adding more Julia
threads does not make arbitrary calls into Python safe. Both matching paths use
exact integer weights derived from the floating-point scores. Equal-score ties
may choose different valid optima across backend versions.

## A custom link kind

A kind implements `link_range_m` and the effects it has. This one pulls its
endpoints together with a constant force:

```julia
struct PullLink <: SpaceAGORA.AbstractInterlink
    force_n::Float64
    range_m::Float64
end
SpaceAGORA.link_range_m(link::PullLink) = link.range_m
function SpaceAGORA.link_force(link::PullLink, a, b, ctx)
    direction = (b.pos_ii - a.pos_ii) / sqrt(sum(abs2, b.pos_ii - a.pos_ii))
    return (link.force_n * direction, -link.force_n * direction)
end

register_link!(model, spacecraft, (1, 1), (2, 1), PullLink(3.0, 2_000.0))
```

A run refuses at setup a link whose kind has no `link_range_m` method. Links of
different kinds can be registered in one model.

### Supported scope

Interlinks currently apply a translational force to rigid spacecraft. The laser
momentum link does not model terminal pointing, occultation, beam losses,
torques, battery depletion or thermal dynamics. The spacecraft's battery and
temperature index fields are metadata, not availability constraints. The cavity
magnification and force law are modeling assumptions; passing implementation
tests does not validate a physical laser system.

Use a full-state solver such as `:tsit5`. Split solvers, external propagator
coupling, articulated/compliant vehicles and independent constellation ensembles
are rejected for linked spacecraft. Near circularity, `:gve_eccentricity` also
rejects candidate sets that can simultaneously apply several positive-score
links to one target: individual eccentricity-rate norms are not additive there.
The semimajor-axis policies do not have that restriction.

Scheduling follows accepted steps, so changes of range or preferred link are
resolved at that cadence. Check timestep convergence for a new case. A checkpoint
resume must retain the interlink topology, link kinds and parameters and scoring
policy; a new run is required when those change. Checkpoints written by
SpaceAGORA 0.4 and earlier still resume. The resumed history covers the resumed
interval, while integrated diagnostic states continue from the checkpoint.

## Deprecated names

The names from when every link was a laser link keep working until SpaceAGORA
1.0. The old keywords, fields and functions warn once per process, and throw
under `julia --depwarn=error`. The two old type names are aliases of the new
types and cannot warn on their own: `InterLinkParameters()`,
`InterLinkParameters(terminal)`, the positional `InterLinkParameters(P, B, range)`
and `ConstellationInterLinkModel(; forbidden_pairs)` run silently, so search for
those names when you move to the new ones.

| Deprecated | Use instead | Warns |
| --- | --- | --- |
| `ConstellationInterLinkModel` | `InterlinkModel` | no, an alias |
| its `constellation_link_type` keyword and property | nothing: each link has its own kind | yes |
| `InterLinkParameters` | `LaserMomentumLink` | no, an alias |
| its keywords and fields `P`, `B`, `range` | `power_w`, `magnification`, `range_m` | yes |
| `register_allowed_link!(model, spacecraft, a, b; parameters)` | `register_link!(model, spacecraft, a, b, link)` | yes |
| `force_on_endpoint(current, other, link)` | `link_force(link, a, b, ctx)` | yes |
| `laser_force_on_spacecraft(model, state, satellite)` | `interlink_force_on_spacecraft(model, state, satellite)` | yes |
