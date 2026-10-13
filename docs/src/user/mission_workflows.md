# Define, run and inspect a mission

A mission starts with the same `SimulationConfiguration` used by every
SpaceAGORA run. A `MissionPlan` gives that configuration a reusable builder and a
name, and can collect descriptive phases and event callback factories. It does
not choose a different integrator or substitute different physical models.

The workflow has three steps: define the configuration, run it into a fresh
output directory, then inspect the returned data or open the saved viewer
payload. The viewer can run independently of the simulation package.

## Build the configuration inside a function

Put spacecraft, controllers and other mutable model state inside the builder.
Calling the plan again then starts from a fresh configuration. The engine also
retains its normal state isolation, so the returned configuration contains the
objects that actually ran.

This complete example uses analytic Earth gravity and no atmosphere or SPICE
assets. The numerical choices are explicit in the configuration builder; the
mission wrapper preserves them.

```julia
using SpaceAGORA

function build_coast(; duration_s=300.0)
    planet = make_no_gram_planet(:earth)
    spacecraft = make_three_body_spacecraft(
        bus_dims=(2.0, 2.0, 2.0), panel_dims=(0.02, 2.0, 1.0),
        bus_mass=500.0, panel_mass_each=10.0, panel_offset_y=2.0,
        ic=InitialCondition(ra=7.0e6, rp=7.0e6, i=45.0, ω=0.0, Ω=0.0, ν=0.0),
        prop_mass=0.0, id=1)
    return make_example_config(
        planet=planet, spacecraft=spacecraft, mission_time=Float64(duration_s),
        initial_time=InitialTime(year=2024, month=1, day=1),
        dynamic_effectors=(InverseSquaredJ2GravityModel(),),
        density_model=NoAtmosphereModel(),
        ephemerides_model=SimpleEphemeridesModel(),
        orientation_sim=false, keplerian=true, verbose=false,
        solver_config=SolverConfig(solver_mode=:tsit5, parallel=false))
end

mission = MissionPlan("Earth coast", build_coast)
run = run_mission(mission;
    parameters=(duration_s=300.0,), output_dir="output/earth-coast-001")

run.results.table                # the saved samples, already available in memory
run.results.configuration        # the isolated configuration used by the engine
run.files                        # files written by this run
run.payload_path                 # portable viewer JSON
```

`output_dir` must be absent or empty. Reuse the plan with another destination
for the next run. The workflow refuses a populated directory instead of
replacing an earlier result. It also refuses checkpoint writing and resume;
those workflows continue to use `run_simulation` directly.

The wrapper changes only output settings: it enables results, chooses the output
directory and sets the scene-output flag. It preserves the builder's dynamics,
GNC, epoch, solver, tolerances and output sampling. The `MissionConfiguration`
still determines duration or orbit-count termination. No phase or event shortens
the run unless its existing callback explicitly implements a stop.

## Inspect data or open the viewer later

By default the workflow saves the normal result files, the scene sidecar and
`simulation_results_viewer.json`. The JSON contains the renderer-ready data and
can be viewed after the simulation process has ended. The default payload omits
surface textures to keep the portable output small; a richer payload can be
exported later from the saved bundle without rerunning the simulation.

Install the independent viewer using the [visualization guide](visualization.md),
then use:

```julia
using SpaceAGORAVisualizer
SpaceAGORAVisualizer.visualize("output/earth-coast-001")
```

You can move or copy that run directory and pass its new path to the viewer.
The live `MissionRun` object retains its original paths; it is not needed to
view the moved payload. The viewer does not need the configuration builder,
SpaceAGORA, GRAM or MuJoCo to display this saved data.

With both packages loaded, `SpaceAGORAVisualizer.visualize(run)` uses the saved
payload directly. To rebuild a page with different export options, call
`export_visualization(run)` explicitly.
It reads the saved scene and result bundle; it does not propagate again. To
save only result tables and their bundle, set `save_scene=false` and
`save_payload=false` together. A payload requires scene output and enabled
result bundles. The wrapper rejects an incompatible output request before
creating its destination.

`run.results.solution` is `nothing` unless you pass `return_solution=true`.
The returned table follows the configured output sampling; it is not a promise
that every accepted solver step or event instant appears as a row.

## Name phases without changing the equations

A `MissionPhase` names an elapsed-time window for interpreting one run:

```julia
mission = MissionPlan("Earth coast", build_coast;
    phases=(
        MissionPhase(:initial_coast; window_s=(0.0, 120.0)),
        MissionPhase(:later_coast; window_s=(120.0, 300.0)),
    ))
```

These windows are observation metadata. They do not switch force models,
activate a controller, reset state, add time stops or divide the run into
separate solves. The names are available in `run.phases`, including when an
existing stop condition ends the run before a later window is reached. Windows
must be finite, ordered, nonoverlapping and within the configured mission-time
horizon; gaps are allowed.

The engine already has atmosphere-entry and exit behavior, solver regimes and
controller lifecycle rules. A user-defined phase name does not override them.
Scripted transitions that change physical models need an explicit transition
contract and are outside this initial workflow.

## Give events fresh callback state

A `MissionEvent` stores a function that builds a solver callback. SpaceAGORA
calls that factory once for each run, after epoch alignment, spacecraft GNC
composition and state isolation. Its argument is the configuration the engine
will use. Mutable callback state belongs inside this factory, so running the
same plan twice does not reuse counters or flags from the first run.

This example stops at the first accepted step at or after a requested elapsed
time, using the existing discrete callback mechanism:

```julia
using OrdinaryDiffEq: DiscreteCallback, terminate!

stop_event = MissionEvent(:coast_limit, config -> begin
    fired = Ref(false)
    limit_s = min(120.0, config.mission_configuration.mission_time)
    condition(u, t, integrator) = !fired[] && t >= limit_s
    function stop!(integrator)
        fired[] = true
        terminate!(integrator)
    end
    DiscreteCallback(condition, stop!; save_positions=(false, false))
end)

mission = MissionPlan("Earth coast with stop", build_coast; events=(stop_event,))
run = run_mission(mission; output_dir="output/earth-coast-stop-001",
    return_solution=true)
run.results.solution.t[end]
```

This is an accepted-step condition, not an exact-time event: the stop can occur
later than `limit_s`, and the final saved table row can precede the stop. See
[Stopping on a Condition](stop_conditions.md) for continuous and discrete
conditions and their timing. If an existing callback registers exact time
stops, that behavior is retained. A state anchor still replaces the specified
state; wrapping it in a `MissionEvent` does not turn it into a physical burn.

Factories must return a fresh callback or callback set. They should construct
callbacks without performing I/O or mutating configuration. Their callback
closures may refer to the isolated controller and model objects supplied in
`config`. Returning a callback shared with another run defeats this ownership
rule. Existing built-in callbacks retain their ordering; mission callbacks use
the existing extra-callback position.

`run.event_names` records the events that were configured. It is not a log of
which conditions fired. Event diagnostics remain with the callback or controller
that owns them. Invalid factories are rejected before SpaceAGORA creates run
outputs. A failure during propagation follows the existing engine behavior:
partial outputs can remain, and the exception is rethrown rather than returned
as a successful `MissionRun`.
