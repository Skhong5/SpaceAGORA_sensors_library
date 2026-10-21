# Interlinks

Use this page to connect spacecraft through interlinks, with a policy that
selects which links are active.

What to read next:

- [Configuring a Simulation](simulation_configuration.md)
- [Parallel Execution](parallel_execution.md)

## Interlinks and constellations

An **interlink** is a connection between two spacecraft terminals that does
something while the geometry allows it: push the spacecraft apart, carry power,
or carry data. A **constellation** is a set of spacecraft and
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
key: the lower `(spacecraft, terminal)` pair first, or, for a directed link, the
transmitter first. `ctx` is a `LinkContext`
holding the time (s into the run) and the run's planet.

The built-in kinds are:

- `LaserMomentumLink(; power_w, magnification, range_m)`: an open-cavity laser
  link with the idealized central force `magnification * power_w / c`, directed
  away from the other endpoint, equal and opposite on the two spacecraft.
  `LaserMomentumLink(terminal)` takes the numbers of a `LaserTerminal`.
- `PowerBeamLink(; power_w, wavelength_m, waist_radius_m, range_m)`: a power
  beam from a `LaserTerminal` to a `PowerReceiver`. See [Power links](#Power-links).
- `DataLink(; transmit_power_w, budget, range_m)`: a one-way data link, from a
  `RadioTerminal` or a virtual terminal to another, at the rate its link budget
  sets. See [Data links](#Data-links).

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

## Terminals

A spacecraft's terminals are its `n_terminal` virtual terminals, numbered
`1:n_terminal`, followed by the terminal hardware mounted on it, in mounting
order. A `LaserTerminal` mounted with `add_actuator!` is terminal hardware; a
kind of your own opts in with `SpaceAGORA.link_terminal(::MyTerminal) = true`.
`terminal_count(spacecraft)` gives the total, and
`mounted_link_terminals(spacecraft)` lists the hardware with its numbers. A
spacecraft with only hardware sets `n_terminal=0`. Virtual terminals stay for
idealized studies, which run as before.

An endpoint's terminal is a number, or the mounted terminal itself:

```julia
bus = Link(root=true, m=100.0)
sc = SpacecraftModel(root=bus, links=[bus], inertia_tensor=bus.inertia, n_terminal=0)
terminal = LaserTerminal(boresight=[0.0, 1.0, 0.0], max_power_w=50.0, magnification=100.0, range_m=200e3)
add_actuator!(sc, bus, terminal)
# ... a second spacecraft, `other`, likewise
spacecraft = [sc, other]
register_link!(model, spacecraft, (1, terminal), (2, 1), LaserMomentumLink(terminal))
```

A terminal must be able to carry its link (`terminal_supports(terminal, link)`).
A `LaserTerminal` carries `LaserMomentumLink`, and a virtual terminal carries
any kind, except a `DataLink` whose `ShannonBudget` reads the radios at both
ends. `terminal_number(spacecraft, terminal)` gives a mounted terminal's
number. Two mounted terminals with equal fields cannot be told apart, so for
those give the number.

### Directed links

A kind with `link_directed(link) = true` runs one way: from the first endpoint
given to `register_link!` (the transmitter) to the second (the receiver). Its
key keeps that order, so its hooks receive the transmitter as `a`. The opposite
direction between the same terminals is a separate link. Laser momentum links
are undirected.

### Registering many links

`register_links!(model, spacecraft, pairs, link)` registers one link per
spacecraft pair. `all_pairs(spacecraft)` gives every pair, or you can pass any
vector of `(i, j)` index pairs. For a constellation laid out by
`build_constellation` or `walker_constellation` (see
[Constellations](constellations.md)), its planes and slots give the usual
topologies:

- `same_plane_pairs(spacecraft)`: every pair within each plane.
- `ring_pairs(spacecraft)`: each slot with the next in its plane, closing the
  ring.
- `grid_pairs(spacecraft; wrap_planes=true)`: the "+grid", the rings plus each
  slot with the same slot of the next plane. Pass `wrap_planes=false` for a
  Walker star, whose first and last planes counter-rotate. `link` is a kind value, or a function
`(i, j) -> link`. Each spacecraft's links take its terminals that can carry them
in turn, starting again from the first when they run out; links that share a
terminal compete for it when the scheduler selects. Pass
`terminals=(i, j) -> (terminal_i, terminal_j)` to choose the terminals yourself.

```julia
register_links!(model, spacecraft, all_pairs(spacecraft), LaserMomentumLink(power_w=40.0))
# or, for a constellation's +grid:
register_links!(model, spacecraft, grid_pairs(spacecraft), LaserMomentumLink(power_w=40.0))
```

### Terminal hardware and power

A hardware terminal that is a link endpoint is driven by its links for the
whole run:

- **Its level.** While one of its links is active, it runs at the level the
  link sets, `link_terminal_levels(link)`. For a `LaserMomentumLink`, that is
  the link's `power_w` at both ends. Otherwise its level is 0.
- **Its power.** It draws `actuator_consumption(terminal, level)` from its
  spacecraft's power budget: `power_w / efficiency` for a `LaserTerminal`. The
  draw shows in the power columns, and in the `actuator_level` and
  `actuator_power` columns.
- **Its force.** Only the link's force applies while the link is active. The
  terminal's own recoil applies only when a controller drives a terminal that
  is no link's endpoint.
- **Controllers.** A mounted-actuation controller's levels for a link-driven
  terminal apply nothing, and setup says so once.
- **Limits.** A link that would drive its terminal past its limits (above a
  `LaserTerminal`'s `max_power_w`) is refused at setup. Build the link from the
  terminal, `LaserMomentumLink(terminal)`, to match them.

When the battery sheds tier 1 (see [Powering a Spacecraft](power_budget.md)), it
sheds laser and radio terminals. Links through shed hardware terminals then
become unavailable, and stay so until the tier clears: a shed terminal keeps
asking for its link's power, so the tier clears when the battery can supply it
again. A terminal kind's tier is `terminal_shed_tier(terminal)`: 1 for
`LaserTerminal` and `RadioTerminal`, 0 (never shed) for `PowerReceiver` and by
default. Virtual terminals draw nothing and are never shed.

## Power links

A `PowerBeamLink` carries power, one way, from a `LaserTerminal` (the
transmitter) to a `PowerReceiver` (the receiver). It applies no force.

```julia
receiver = PowerReceiver(normal=[0.0, -1.0, 0.0], radius_m=0.25, efficiency=0.4)
add_actuator!(other, other.root, receiver)
register_link!(model, spacecraft, (1, terminal), (2, receiver),
    PowerBeamLink(power_w=80.0, wavelength_m=1.064e-6, waist_radius_m=0.05, range_m=50e3))
args = SimulationConfiguration(...; interlink_model=model,
    scheduling_policy_model=SchedulingPolicyModel(:power_delivery))
```

- **The beam.** While the link is active, the transmitter runs at `power_w`
  optical watts. It draws `power_w / efficiency` from its own battery as a
  tier-1 laser load (see [Terminal hardware and power](#Terminal-hardware-and-power)).
  The beam is Gaussian:
  - its waist, of 1/e² radius `waist_radius_m`, is at the transmitter;
  - its spot radius at range `d`, `beam_spot_radius_m(link, d)`, is
    `w0 √(1 + (d / z_R)²)`, with the Rayleigh range `z_R = π w0² / λ`.
- **The receiver.** It captures `beam_capture_fraction(link, radius_m, d)`,
  `1 − exp(−2 r² / w²)`, of the beam through its aperture. It delivers that
  times its `efficiency` to its bus, up to its `max_power_w`
  (`link_received_power_w`).
  - For example, with a 1.064 µm beam from a 5 cm waist, a 25 cm receiver
    catches nearly all of it out to about 10 km, about a quarter at 100 km, and
    under 1 % at 1000 km.
- **Generation.** The delivered power is generation in the receiver's power
  budget. It shows in `power_generated_w` and in its own `power_received_w`
  column, charges the receiver's battery, and helps an empty battery recover.
  A full receiver battery curtails it, as it does solar power; the transmitter
  keeps paying.
- **Availability.** The link can close within `range_m`, while the segment
  between the spacecraft clears the planet (a sphere of its equatorial radius).
  When the transmitter's battery sheds tier 1, the beam closes until the tier
  clears.
- **Direction.** A `PowerReceiver` can only receive, and a `LaserTerminal` can
  only transmit a power beam (`terminal_supports(terminal, link, side)`).
- **Scheduling.** Policies that score links by their forces never select a
  power beam, so use `:power_delivery`, which scores each link by the power it
  would deliver. A run uses one policy; mixing momentum and power links waits
  for a later scheduling contract.
- **Columns.** A run with a power beam adds `power_received_w` and
  `power_beamed_w` per spacecraft to the [power columns](output_columns.md#Power-budget).
- **Not yet modelled.** Pointing and jitter, the receiver's incidence angle,
  the atmosphere, beam quality, and the beam's radiation pressure are not
  modelled.

## Data links

A `DataLink` carries data, one way, from a transmitter terminal to a receiver.
It applies no force. Its `budget` sets the rate:

```julia
radio() = RadioTerminal(boresight=[0.0, 1.0, 0.0], max_power_w=5.0, gain_dbi=6.0, efficiency=0.5,
    noise_temperature_k=500.0)
sender, receiver = radio(), radio()
add_actuator!(sc, bus, sender)
add_actuator!(other, other.root, receiver)
link = DataLink(transmit_power_w=1.0, range_m=2000e3,
    budget=ShannonBudget(bandwidth_hz=1e6, frequency_hz=2.2e9, implementation_efficiency=0.5))
register_link!(model, spacecraft, (1, sender), (2, receiver), link)
args = SimulationConfiguration(...; interlink_model=model,
    scheduling_policy_model=SchedulingPolicyModel(:data_throughput))
```

- **The budget.**
  - `FixedRateBudget(; rate_bps)`: a constant rate at any range up to
    `range_m`. It works between virtual terminals too.
  - `ShannonBudget(; bandwidth_hz, frequency_hz, implementation_efficiency=0.5, max_rate_bps=Inf)`:
    a radio link budget between two `RadioTerminal`s.
    - The received power follows Friis: `P_r = P_t G_t G_r (λ / 4πd)²`, with
      the link's `transmit_power_w` as `P_t`, the two radios' `gain_dbi` and
      `λ = c / frequency_hz`.
    - The noise is `k_B T bandwidth_hz`, with the receiver's
      `noise_temperature_k` as `T`.
    - The rate is `implementation_efficiency × bandwidth_hz × log₂(1 + P_r / N)`,
      up to `max_rate_bps`.
    - For example, in the code above the rate is about 5.7 Mbit/s at 10 km,
      2.4 Mbit/s at 100 km and 0.17 Mbit/s at 1000 km.
  - A budget kind of your own subtypes `AbstractDataBudget` and implements
    `budget_rate_bps(budget, link, transmitter, receiver, distance_m)`. The
    terminals are `nothing` at a virtual end.
  - The rate is capped at each `RadioTerminal`'s `max_rate_bps`
    (`link_data_rate_bps`).
- **The radios.** A `RadioTerminal` can transmit or receive. While it transmits,
  it runs at the link's `transmit_power_w` and draws `transmit_power_w / efficiency`,
  as a tier-1 load. A receiving radio draws nothing; give its electronics a
  constant `PowerLoad`. When the transmitter's battery sheds tier 1, the link
  closes until the tier clears. A `LaserTerminal` or a `PowerReceiver` cannot
  carry a data link.
- **Bits.** In a run with a data link, every spacecraft has two more integrated
  states, `data_sent_bits` and `data_received_bits`. Their rates are the sums of
  the rates its active links carry out and in, evaluated at every
  right-hand-side call, so they are exact integrals of a varying rate. They
  continue across a checkpoint resume.
- **Availability.** As for a power beam: within `range_m`, while the segment
  between the spacecraft clears the planet.
- **Direction.** Two-way traffic is two links, one each way. A radio carries
  one active link at a time, so the scheduler selects one direction of a
  two-way pair between the same two radios: the one with the higher score.
- **Scheduling.** Use `:data_throughput`, which scores each link by the rate it
  would carry. The force and power policies never select a data link.
- **Columns.** A run with a data link adds `data_rate_tx_bps`,
  `data_rate_rx_bps`, `data_sent_bits` and `data_received_bits` per spacecraft
  (see [the data columns](output_columns.md#Data-links)).
- **Not yet modelled.** Onboard storage and queues, ground stations, pointing
  and antenna patterns, Doppler, atmospheric and polarization losses,
  interference between links and acquisition time are not modelled.

## Scheduling

At the start of the run and after every accepted solver step, the engine marks
a candidate available when its spacecraft pair is not forbidden, both
spacecraft are active, neither terminal is shed, and its link's
`link_available` allows it. The scheduling
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

### Contact windows

The history (`model.history`) has one `InterLinkHistory` sample per accepted
step: its time, the links the scheduler selected (`active`) and the links that
were available (`available`). The available links are recorded in a run with a
data link, or when you ask for them with `InterlinkModel(; record_available=true)`;
`record_available=false` leaves them out. In a large constellation over a long
run they are a list per step, so other runs leave them out by default. `contact_windows(history)` turns it into windows,
one `(key, start_s, end_s, duration_s)` for each span over which a link could
close; `contact_windows(history; active=true)` gives the spans over which it
was selected. `contact_time_s(history, key)` sums a link's windows.

```julia
solution = run_simulation(args; return_solution=true)
history = solution.prob.p.args.interlink_model.history
windows = contact_windows(history)
contact_time_s(history, ((1, 1), (2, 1)); active=true)
```

A window starts at the first accepted step that has the link and ends at the
next step that does not, or at the last step: the resolution is the
scheduler's. A history resumed from a checkpoint covers the resumed interval
only, so a window that spans the resume is split at it. The results table, unlike
the history, holds the whole run (see [Outputs](outputs.md)).

### Optional matching backend

Tree-shaped components are solved in Julia. Of parallel links (directed links
both ways between the same two terminals), only the higher-scoring one is a
candidate, so a two-way pair alone is a tree. Cyclic components use NetworkX's
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

Interlinks currently apply a translational force to rigid spacecraft, or carry
power or data. The laser momentum link does not model terminal pointing,
occultation, beam losses, torques or thermal dynamics. Links through hardware terminals draw battery
power and close when it is shed; links between virtual terminals draw none.
Actuator faults on a link-driven terminal do not yet act on its link. The
spacecraft's `battery_energy_index` and `tempurature_index` fields are
metadata, not availability constraints. The cavity
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
resume must retain the interlink topology, link kinds (each named by its module
and type, with its concrete type parameters) and parameters, scoring policy,
and each spacecraft's terminal hardware: its kinds, in mounting order, and every
field of each, such as a receiver's aperture or a transmitter's efficiency; a
new run is required when those change. Checkpoints written by SpaceAGORA 0.4
and earlier still resume. The resumed history covers the resumed interval, while
integrated diagnostic states continue from the checkpoint and the results table
holds the whole run.

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
