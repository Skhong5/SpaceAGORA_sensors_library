# Simulation Outputs

Use this page when you want to understand the files written by
`run_simulation`, the column names in the CSV, or how to load results for
post-processing.

This page is for users who have already run a simulation and now need to
interpret the output.

Shortest successful command:

```text
julia --project=. examples/AGORA_Basic_Quickstart.jl
```

What to read next:

- [Output Columns](output_columns.md)
- [The Integrated State](integrated_state.md) (which columns are integrated and which are derived)
- [Simulation Configuration](simulation_configuration.md)
- [Verification Study](verification_study.md)
- [Recipes](recipes.md)

## Output files

With the default output settings, `run_simulation` writes three files to the
`results_directory`:

```text
output/
  simulation_results.csv         ← tabular results, one row per time step
  simulation_results.feather     ← same data in Apache Arrow / Feather format
  simulation_results.manifest.toml ← metadata: row count, spacecraft count, file hashes
```

Set `simulation_settings.save_csv = false` to save the Feather file and manifest
without writing a CSV. A CSV left by an earlier run is preserved, but is not
listed in the new manifest or in `SimulationResults.files`.
Set `simulation_settings.results = false` to disable result-file output.
The manifest records
`schema_version`, `created_utc`, `mission_time_s`, `steps`,
`spacecraft_count`, and SHA-256 hashes for each data file.
`created_utc` is the wall-clock time the file was written, not the simulation
epoch. The epoch is the `InitialTime` you configured; the manifest does not
store it, so keep it with your script or read it from the returned
configuration.

The developer override `SPACEAGORA_SAVE_BUNDLE=0` disables Feather and manifest
output. With this override and `save_csv=false`, use `return_results=true` to
receive the table in memory. Explicitly enabled scene sidecars and checkpoints
can still be written.

## Getting the results in memory

Pass `return_results=true` to get a [`SimulationResults`](@ref) back instead of
reading files. Its `table` is the same `DataFrame` the CSV would contain
(same columns and values), and it is built even when
`simulation_settings.results = false`, in which case result files are not written.
Explicitly enabled checkpoints can still be written; they are listed in `files`
only when this call writes them. Resuming an already completed checkpoint does
not report the old checkpoint files as new output.
`configuration` is the configuration that actually ran: under the default
`isolate_state=true` that is the deep copy, so controller state and logs, such
as the RPO command log, are reachable there and not on the object you passed
in. `files` lists the output files the run wrote, and `solution` holds the ODE
solution when `return_solution=true` is passed as well (otherwise `nothing`).
`return_results=true` cannot be combined with `return_solver_metadata=true`.

```julia
res = run_simulation(args; return_results=true)
res.table                      # DataFrame, one row per saved step
res.files                      # e.g. ["output/simulation_results.csv", ...]
res.configuration              # the (isolated) configuration that ran

# Read back a setting that actually ran, for example spacecraft 1's inclination.
# Keplerian InitialCondition fields are stored in SI units and radians.
ic = res.configuration.dynamics_model.spacecraft[1].initial_condition
rad2deg(ic.i)                  # inclination in degrees

res = run_simulation(args; return_results=true, return_solution=true)
res.solution.t
```

## Where the examples write, and how to keep runs apart

Most repository examples build their configuration with `make_example_config`,
whose `results_directory` is `<repository root>/output` unless
`SPACEAGORA_CLI_OUTPUT_DIR` is set; the CLI's `--output-dir` sets exactly that
variable. With the default result settings, the three file names do not
change, so:

- two runs using the same results directory overwrite those files;
- `julia --project=. src/interfaces/cli/main.jl run --example=<script> --output-dir=output/<name>`
  gives each run its own directory, for the scripts that take their directory
  from `make_example_config`: the first-run scripts (`AGORA_Basic_Quickstart.jl`,
  `AGORA_Earth_NoGRAM.jl`), the controls and torque
  tests, and the mission scripts `AGORA_Basic_GRAMEarth.jl`, `AGORA_Odyssey.jl`,
  `AGORA_Vex.jl`, `AGORA_Titan.jl`, `AGORA_Magellan.jl`, `AGORA_LOFTID.jl`,
  `AGORA_Mars_NoGRAM.jl` and `Earth_Thruster_Test.jl`;
- `--output-dir` has no effect on the scripts that choose their own directory:
  `AGORA_Earth.jl`, `AGORA_Keplerian.jl` and `Earth_Navigation.jl` write to
  `output/` directly, `AGORA_Earth_Aerobraking.jl` to `output/earth_aerobraking/`,
  `AGORA_Mars_RAAN_Scenario.jl` to `output/mars_raan_scenario/`, and the RPO,
  robot-arm and compliant multibody demos to their own `output/<demo>/` directories; running
  one of them twice overwrites its previous results;
- `odyssey_surrogate.jl` runs in its own environment and writes its own files:
  the comparison writes to `odyssey_surrogate_results/` in the current
  directory unless you pass `--output=DIR`, refuses an existing directory, and
  prints the absolute path when it finishes (see the
  [Odyssey walkthrough](../tutorials/odyssey_surrogate.md));
- for your own scripts, pass `results_directory=` to `make_example_config` or
  set it on `SimulationSettings`.

The quickstart example additionally saves four PNG plots under
`<results_directory>/plots/`. The RPO examples also generate HTML plots.
`Solar_Panel_Cloth_Deployment_Demo.jl` writes four HTML files in its demo
directory instead of the three simulation result files.

`AGORA_Earth_MonteCarlo.jl` prints its successful and failed sample counts and
elapsed time to the terminal. It disables result saving, so it writes no
result files, including when `--output-dir` is supplied.

Smoke mode (`--smoke` on the CLI, or `SPACEAGORA_EXAMPLE_SMOKE=1` for a script)
uses a short mission, normally at most 120 s and one orbit. An explicit
`--output-dir` is honored through `SPACEAGORA_CLI_OUTPUT_DIR`. Without that
override, smoke mode writes to `output/` under the current working directory.
Choose a separate destination to keep earlier results: reusing the same
directory replaces its previous result files. Results are kept only when
`SPACEAGORA_EXAMPLE_SMOKE_RESULTS=1`, which the CLI sets for you.

### Visualization scene sidecar (opt-in)

Setting `simulation_settings.save_visualization_scene = true` adds one more
file and one more group of columns, both consumed by the post-hoc 3D viewer
described in [Interactive Visualization](visualization.md):

```text
output/
  simulation_results_scene.json  ← planet, epoch, spacecraft geometry, frame rotation samples
```

The sidecar carries everything the viewer needs besides the trajectory rows:
the central body's radii, spin and texture key, a table of J2000-to-body-fixed
quaternions sampled over the mission span (so the viewer needs no SPICE
kernels), and every spacecraft's links as boxes built from `Link.dims`, with
thruster, facet and joint glyphs. It is written after the results bundle by
the same run, and only when the flag is set; the default output is unchanged.

With the flag on, spacecraft that have non-root links also get the
`link_pose` save field: columns `sc{N}_link_pose_1` .. `sc{N}_link_pose_7n`
holding `[rx, ry, rz, qx, qy, qz, qw]` per non-root link, in the link order
recorded in the sidecar. These are the link poses relative to the root bus
at each saved step, so articulated panels play back as they moved. A
spacecraft that carries a thruster gimbal also gets the `thruster_direction`
field (`sc{N}_thruster_direction_1` .. `_3k`): every thruster's exhaust
direction in its link's frame at each saved step, so the viewer turns the
gimballed thruster's cone and plume as the gimbal moved. Reading the sidecar
back:

```julia
using SpaceAGORA
scene = read_visualization_scene("output/simulation_results_scene.json")
scene.planet.name, length(scene.spacecraft), scene.spacecraft[1].bounding_radius_m
```

The flag adds four more fields when the run can supply them, each listed in
on [Output Columns](output_columns.md#Visualization-fields):

- `thruster_level`: `sc{N}_thruster_level_{k}`, the firing level of thruster
  `k` from 0 to 1, for a spacecraft with thrusters whose control effector
  reports per-thruster levels. Thrusters are numbered in the sidecar's order:
  the spacecraft's links in order, each link's thrusters in order.
- `sun_dir`: `sun_dir_1` .. `sun_dir_3`, the unit vector from the planet's
  centre to the Sun in the inertial frame of the saved positions. One
  direction per row, shared by every spacecraft, written when the run's
  ephemerides can locate the Sun at the start epoch. The viewer's sun lighting
  reads it.
- `arm_pose`: `sc{N}_arm_pose_1` .. `sc{N}_arm_pose_7m` for a spacecraft with
  a compliant robot-arm plan, `[rx, ry, rz, qx, qy, qz, qw]` per arm link: the
  link centre of mass relative to the spacecraft position, in inertial metres,
  and the link's inertial attitude quaternion.
- Articulated spacecraft (any non-fixed `Joint`) add four fields to the default
  set: `joint_q` (`sc{N}_joint_q_1` ..) and `joint_qd` (`sc{N}_joint_qd_1` ..)
  list the non-fixed joints in `spacecraft.joints` order (a ball joint takes four
  and three columns); `articulated_link_pose` (`sc{N}_articulated_link_pose_1` ..
  `7n`) is `[rx, ry, rz, qx, qy, qz, qw]` per link in `spacecraft.links` order:
  the link's COM in inertial metres and its scalar-last body-to-inertial
  attitude, from the joint kinematics (links merged through `:fixed` joints
  included); `system_com` (`sc{N}_system_com_1..3`) is the inertial center of
  mass of the whole spacecraft. The `link_pose` viewer field still holds the
  configured rigid geometry.
- Spacecraft with compliant attachments add `attachment_pose`
  (`sc{N}_attachment_pose_1` .. `7N`): `[rx, ry, rz, qx, qy, qz, qw]` per attachment body in
  attachment order, the body COM in inertial metres (`pos + att_r`) and its scalar-last body-to-inertial
  attitude. `system_com` (`sc{N}_system_com_1..3`) includes the attachment bodies (rigid
  spacecraft get the column from the attachment fields; articulated ones from the articulated
  fields). Spacecraft without attachments save neither.
- `density`: `sc{N}_density`, the atmospheric density along the trajectory in
  kg/m³, for a run with an atmosphere model.

An explicit `save_fields` list receives the visualization fields it lacks,
appended once after the fields you named. The plume columns below are not
visualization fields: they belong to the default field list whenever the
plume-surface effector is in the run, with or without the sidecar, and an
explicit list that omits them does not write them.

### Interactive viewer page

With the sidecar present, `export_visualization` builds a self-contained
HTML page next to the bundle, or pass `visualization=true` to `run_simulation`
to do both in one go:

```julia
run_simulation(args; visualization=true)
# or, from an existing flagged run
export_visualization("output/simulation_results"; max_frames=2000, frame=:inertial)
```

Setting `SPACEAGORA_VISUALIZATION=1` in the environment does the same for a
script you do not want to edit, for example
`SPACEAGORA_VISUALIZATION=1 julia --project=. examples/AGORA_Earth_NoGRAM.jl`.

```text
output/
  simulation_results_viewer.html  ← three.js viewer: textured globe, markers, trails, timeline
```

The page embeds three.js, the surface texture from `data/scene/textures/`
(Earth, Mars, Venus, Titan, Moon are registered in `manifest.toml`), the
sidecar and a decimated copy of the trajectory, so it opens from disk in any
modern browser without a server. `max_frames` and `data_budget_mb` bound the
embedded rows; the info panel shows the effective cadence. `trail_orbits`
sets how much history trails behind each spacecraft (default three orbits,
selectable on the page). Zooming in on a spacecraft, or selecting it and
pressing F, swaps its marker for the link-box assembly with thruster and
facet glyphs; `stl=Dict(id => "model.stl")` draws a CAD mesh instead.
The [Interactive Visualization](visualization.md) page covers the options,
ensembles and textures; [the independent renderer guide](https://github.com/Space-FALCON-Lab/SpaceAGORAVisualizer.jl/blob/75b32bc610e3227035810a4c8e12a515106c5a62/viewer/README.md) documents the controls and how to
develop the viewer itself.

## Loading results in Julia

```julia
using CSV, DataFrames
df = CSV.read("output/simulation_results.csv", DataFrame)
```

Or from the Feather file (faster for large runs):

```julia
using Arrow, DataFrames
df = DataFrame(Arrow.Table("output/simulation_results.feather"))
```

## Column reference

The meaning, units and frame of every column in the saved table are on
[Output Columns](output_columns.md).

## Multi-spacecraft runs

When multiple spacecraft are passed to `DynamicsModel`, each spacecraft
contributes its own column set. Spacecraft 1 produces `sc1_*` columns,
spacecraft 2 produces `sc2_*`, and so on. The spacecraft position in the
configuration sets the numeric suffix; `SpacecraftModel.id` does not.

## Output rate

The `data_rate` field in `MissionConfiguration` sets the output sample cadence
in seconds of simulated time. The default is `10.0` seconds. Decreasing
`data_rate` produces more rows and larger files; increasing it reduces output
size but loses temporal resolution.

The `num_steps_to_save` field controls how many time steps are buffered in
memory before a flush. For very long runs, tuning this can reduce peak memory
use.

## Checkpointing

Enable periodic checkpointing via `SimulationSettings`:

```julia
SM.SimulationSettings(
    checkpoint_enabled      = true,
    checkpoint_interval_s   = 300.0,         # every 300 s of simulated time
    checkpoint_directory    = "output/checkpoints"
)
```

A checkpointed run is solved in segments of `checkpoint_interval_s`. Its
navigation, guidance and control effectors tick at the same times as in a run
solved in one piece, including a tick that falls on a segment boundary, which
comes before the checkpoint written there. A tick and a boundary that differ only
by floating-point rounding, within 100 units in the last place (`3 * 0.1` is
0.30000000000000004, and a 0.3 s interval ends the first segment at 0.3), count
as one time: the tick is taken at the boundary, once. Likewise, a segment that
would end that close to the run's end ends at the run's end.

A crashed or interrupted run can be resumed by setting
`resume_from_checkpoint = true` in `SimulationSettings`. The resumed run's
effectors keep the same tick times, from the run's start. A run resumed from a
checkpoint written at a segment boundary starts with the first tick after it;
one resumed from the checkpoint written at the end of a run, to extend that
run, first takes the ticks due at that time, which the ended run did not take.
Each checkpoint records which periodic models had not yet taken the tick due at
its time, also when the run stopped early there, and a run resumed from it takes
those models' ticks first, once. A checkpoint written by SpaceAGORA 0.4.0 or
earlier holds no such record; it is read as that version left it, which also
means a tick an ulp or so after its time, which 0.4.0 left to the next segment, is
taken when the resumed run starts.
The checkpoint directory defaults to `results_directory/checkpoints` when left
empty. Besides
the state, the checkpoint holds the run state the callbacks keep on the
spacecraft: the batteries' empty and full flags and shedding tier, the
actuator faults' activity and held levels, the sensors' random streams,
latest measurements and gyro biases, the hinges' and gimbals' angles, and the
ramp factor and held thrust of thrusters with firing dynamics. A run with
batteries, faults, sensors or thrusters with firing dynamics refuses a
checkpoint written without them, or for a different configuration of them; a
run with firing dynamics therefore refuses a checkpoint written by SpaceAGORA
0.4 or earlier. A guidance, navigation or
control model, or a sensor kind, that keeps run state of its own adds it to
the file through `checkpoint_state` (see
[Run state in checkpoint files](../extensibility.md#Run-state-in-checkpoint-files));
without it, a resumed run starts that state from its reset.

The checkpoint also records which spacecraft are active. A spacecraft lost to
an impact or a touchdown before the checkpoint stays inactive in the resumed
run, as it would in a run that was never interrupted: it is not propagated, its
guidance, navigation and control models treat it as inactive (`skip_inactive`),
and its impact or touchdown does not happen again. A checkpoint written by an
earlier SpaceAGORA records this only for an interlinked run; any other run
resumed from such a checkpoint starts with every spacecraft active, as before.

A run with scoped dynamic effectors (`scoped`) also records each effector's kind
and the run positions it acts on, and resumes only with the same scopes. It
refuses a checkpoint whose scopes differ, and a checkpoint written without scoped
effectors (by a run without them, or by an earlier SpaceAGORA). A run without
scoped effectors records nothing more than before.

## Campaign and multi-process runs

`run_monte_carlo` and `run_constellation_ensemble` samples each call
`run_simulation` independently (per-sample `results_directory` and
`SimulationSettings` come from whatever `SimulationConfiguration` your
per-seed closure returns), so the file layout above applies per sample
whether the outer route is serial, threaded, or process-backed. See
[Parallel Execution](parallel_execution.md) for how to select and configure
outer routing.


## In-memory constellation recorder

For fixed-cadence output without retaining the solver's complete step history,
attach the optional recorder to an existing `SimulationConfiguration` named
`args`:

```julia
using SpaceAGORA
using SpaceAGORA.SimulationModel: TrajectoryRecorder, get_trajectory_recorder_callback,
    trajectory_times, trajectory_positions, trajectory_save_data

recorder = TrajectoryRecorder(args)
run_simulation(args; return_solution=false,
    extra_callbacks=(get_trajectory_recorder_callback(recorder),))
times = trajectory_times(recorder)
positions = trajectory_positions(recorder)  # component x spacecraft x sample
```

The cadence defaults to `args.mission_configuration.data_rate`, in seconds.
The recorder preallocates arrays and grows them when needed; memory still grows
with the number of spacecraft and recorded samples. Read the returned views
after the solve, and obtain new views after resetting or reusing the recorder.
`trajectory_save_data(recorder)` materializes ordinary saved snapshots when
needed; that conversion allocates dictionaries at the output boundary.

This callback leaves configured file output unchanged. Use
`SimulationSettings(results=false)` when an in-memory result is sufficient.
Explicit `save_fields` use their supplied getters, including a custom getter
with a built-in field name. Additional default fields without a specialized
array filler also retain their getters. Timing improvements depend on the run
and should be measured before adopting this recorder for a campaign.
