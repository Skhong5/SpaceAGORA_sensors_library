# Laser Interlinks

Use this page to couple spacecraft in a constellation through laser
interlinks, with a policy that selects which links are active.

What to read next:

- [Configuring a Simulation](simulation_configuration.md)
- [Parallel Execution](parallel_execution.md)

## Constellation laser interlinks

A laser interlink couples two spacecraft through an idealized central force. The
model keeps the candidate terminal connections separate from the scheduler's
selection: a connection can be available without being active. Each terminal can
participate in at most one selected link. Spacecraft endpoints use their position
in `dynamics_model.spacecraft`, rather than the vehicle's `id` field.

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

`InterLinkParameters` specifies power `P` in watts, magnification `B`, and range
in metres. The retained force law is `B * P / c`, directed away from the other
endpoint, with equal and opposite forces on the two active spacecraft. Range and
forbidden-pair rules determine availability at initialization and after every
accepted solver step. The selected schedule is held during trial stages; this
avoids changing it during rejected steps. An inactive endpoint contributes no
force even if an earlier schedule still names its link.

The saved `laser_dv` vector integrates the laser acceleration. `laser_delta_sma`
integrates its contribution to the instantaneous osculating semimajor-axis rate;
it is not a comparison against a separately propagated unforced trajectory.
The history belongs to the returned run configuration. With the default state
isolation, the input configuration is unchanged.

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

### Supported scope

This model currently applies a translational force to rigid spacecraft. It does
not model terminal pointing, occultation, beam losses, torques, battery depletion
or thermal dynamics. The spacecraft's battery and temperature index fields are
metadata, not availability constraints. The cavity magnification and force law
are modeling assumptions; passing implementation tests does not validate a
physical laser system.

Use a full-state solver such as `:tsit5`. Split solvers, external propagator
coupling, articulated/compliant vehicles and independent constellation ensembles
are rejected for linked spacecraft. Near circularity, `:gve_eccentricity` also
rejects candidate sets that can simultaneously apply several positive-score
links to one target: individual eccentricity-rate norms are not additive there.
The semimajor-axis policies do not have that restriction.

Scheduling follows accepted steps, so changes of range or preferred link are
resolved at that cadence. Check timestep convergence for a new case. A checkpoint
resume must retain the interlink topology, parameters and scoring policy; a new
run is required when those change. The resumed history covers the resumed
interval, while integrated diagnostic states continue from the checkpoint.
