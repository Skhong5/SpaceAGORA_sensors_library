# Run an RPO case with interchangeable planners

!!! note "Optional package"
    The direct baseline planner runs with SpaceAGORA alone. The HYPR planner
    requires the optional `SpaceAGORAHYPR` package; see
    [Installing optional HYPR](#Installing-optional-HYPR).

The opt-in RPO pilot connects a planner to the existing simulation and LQ-MPC
controller. It supports truth observations, a passive target on a circular Earth
orbit, and a static station point cloud aligned with the target's rotating RTN
frame. The existing research examples remain available; they still use the
deprecated `RPOGuidanceModel` (see
[Moving from RPOGuidanceModel](#Moving-from-RPOGuidanceModel)).

```julia
using SpaceAGORA
args = make_rpo_configuration(planner=DirectRPOPlanner(), seed=741)
solution = run_simulation(args; return_solution=true)
report = rpo_run_report(solution)
report.planners[1].records
report.commands[1].log
```

Load `SpaceAGORAHYPR` in your project, then use `HYPRRPOPlanner(RPOPSOConfig(...))` as the `planner` to choose HYPR. The separate
`examples/rpo_planner_env` project provides exact, deterministic settings and runs
both choices through this interface. Its README documents setup and offline
execution. The default two-second case is a partial maneuver; the planners need
not return the same trajectory. It is not a completed rendezvous benchmark.

## The bounded scenario

The target is on a circular equatorial Earth orbit at 420 km. The default chaser
starts at `(3,0,0)` m and aims for `(5,0,0)` m in target RTN, with zero initial RTN
relative velocity. The run uses inverse-square gravity, no atmosphere and simple
ephemerides. The synthetic station has one point at the origin, a 0.25 m keepout
radius and a 0.15 m largest-chaser-half-extent allowance. Its geometry is a
point-sphere approximation, not watertight CAD geometry.

Reference limits are 0.2 m clearance, 0.5 m/s speed and `0.5*0.05/5.2` m/s²
acceleration. The 5 kg chaser carries 0.2 kg propellant and six 0.05 N thrusters
with 60 s specific impulse. Control runs every 0.1 s with a 12-step preview,
`Q=diag(20,20,20,2,2,2)`, `R=0.1I` and `Qf=10Q`. The default explicit Tsit5 solver
uses orbit/quaternion absolute and relative tolerances of 1e-8 and a 0.05 s cap.
Only this default `:tsit5` route is validated for the planner lifecycle. Other
solver modes may be accepted by the configuration type, but are outside this
pilot; some modes omit guidance and control callbacks. These are pilot settings,
not a certification of tracking or actuator feasibility.

Both adapters retain their reviewed 1% planning reserve and independent reference
validation. The direct baseline uses a cubic rest-to-rest law. Neither adapter
matches an arbitrary measured initial velocity as a boundary condition; the pilot
starts at zero relative velocity. Replanning can therefore introduce a reference
velocity discontinuity. General tracking acceptance remains separate.

## Initialization, replanning and expiry

The engine copies the configuration and prepares the initial plan from the actual
initial state before propagation. The planner receives a separate copied request;
the lifecycle validates against its retained snapshot. Validation checks identities,
frame, endpoints, uniform timing, finite values, configured limits and sampled
clearance. Only then is a complete plan installed for the controller.

Navigation, control and guidance retain their existing order. At a coincident
update the controller uses the preceding reference; guidance can install the next
one afterward. Ordinary and rejected dynamics evaluations only read held control
commands and do not invoke planning. A scheduled request is delivered at the first
accepted guidance tick at or after its requested time:

```julia
events = (RPOPlanningEvent(0.5; reason=:forced, token=1),
          RPOPlanningEvent(1.0; reason=:replan, token=2))
args = make_rpo_configuration(planner=DirectRPOPlanner(), planning_events=events)
```

A token names one event. Duplicate delivery is ignored; different tokens at the
same tick are distinct requests. Optional positive `replan_interval_s` or
`tracking_error_limit_m` enable background requests. These policies operate on the
configured geometry; a geometry provider supplies geometry that changes during the
run (see [Changing geometry, holds and retiming](@ref)). Periodic and scheduled requests use the same configured
absolute time tolerance at tick boundaries. There is no guidance tick at the
mission end. Events without an eligible guidance tick before the end remain
undelivered; inspect the recorded requests rather than assuming every configured
event ran.

By default, every failed initial, forced, background or retiming request stops this
route with `RPOPlanningError`, which carries copied diagnostics, spacecraft/request
IDs, and any original exception and backtrace. There is no implicit retain-old-plan
or emergency-controller fallback; `on_rejection=:install_fallback` opts into an
explicit hold instead. An explicit `:retime` event requires the planner's retiming
capability; `HYPRRPOPlanner` advertises it and `DirectRPOPlanner` does not.

Immediately before control reads a reference, the lifecycle checks its lifetime
through the entire preview. Repeating the last reference column cannot bypass
expiry. `plan_validity_s` is a request lifetime, not the duration of the maneuver.
A short plan may repeat its final sample while still valid. A long plan must fit
inside its lifetime. Reference validity does not guarantee a stationary vehicle.

The constructor accepts a lifetime as short as `12*control_dt_s`, but the first
control update is at `control_dt_s` and needs at least `13*control_dt_s` of initial
validity. For an interval aligned with guidance ticks, periodic replanning needs
at least `replan_interval_s + 12*control_dt_s`, because control reads the old plan
before the replacement. Other intervals must cover the next eligible guidance
tick plus the preview. Without replanning, validity must reach the last control
update plus the preview. These bounds describe consumption timing; a planner may
require a longer lifetime to fit its maneuver.

## Changing geometry, holds and retiming

Three opt-in keywords of `make_rpo_configuration` extend the loop above. Each
request still carries one owned geometry snapshot, and every installed reference
still passes the same validation.

`geometry_provider` supplies the geometry when it changes during the run. The
built-in `RPOReplanningSpheres` adds spheres that appear and disappear to the
station geometry:

```julia
spheres = RPOReplanningSpheres([(center_rtn=(4.0, 0.0, 0.0), radius_m=0.25,
                                 appear_time_s=0.2, disappear_time_s=0.8)])
args = make_rpo_configuration(planner=HYPRRPOPlanner(config), geometry_provider=spheres)
```

Each geometry has a revision string; equal revisions mean equal geometry. When the
revision changes, the remaining path of the installed reference is checked against
the new geometry with the same clearance query and sampling as validation. If it
still clears, the reference is kept; otherwise a `:replan` request follows. To
write a provider, subtype `AbstractRPOGeometryProvider` and implement
`rpo_geometry_revision(provider, revision, time_s)` and
`rpo_geometry_snapshot(provider, geometry, time_s)`; both receive the configured
station geometry or its revision.

`on_rejection=:install_fallback` replaces a rejected request with a hold: a
reference at the chaser's position with zero velocity, which the controller brakes
onto. A hold is not validated and does not reach the goal. It is retried once with
a `:replan` request when the chaser settles on it (`hold_position_tolerance_m`,
`hold_speed_tolerance_mps`), renewed by a request before it expires, and replanned
when the geometry revision changes. Interval and tracking-error replans are
suspended while it holds, so a planner that keeps refusing is asked again only by
these triggers or a planning event. The default, `on_rejection=:fail`, stops the
run as before.

`retime_tracking_error_m` requests `:retime` when the tracking error exceeds it and
stays within `tracking_error_limit_m`. It needs a planner that declares retiming.
`HYPRRPOPlanner` retimes the installed reference's validated path from the point
nearest the chaser, with the same limits as a new plan; a retime cannot reach a
changed goal, so it is then refused. Validation can also refuse a retimed
reference, for example where HYPR's tangential profile turns faster than the
common acceleration limit; with `on_rejection=:install_fallback` the chaser then
holds.

The run records mark each of these requests, and each kept geometry revision, with
`event=:trigger`, and each hold with `event=:fallback_installed`.

## Ownership, reproducibility and inspection

Each controlled spacecraft owns fresh planner state, an independent controller
workspace, a reference slot and a MersenneTwister stream. Stream seed words are the
eight little-endian UInt32 words of SHA-256 over the ASCII string
`SpaceAGORA/RPO/v1/<seed>/<chaser_id>/<target_id>`. They do not depend on spacecraft
order, callbacks, threads or Julia's randomized `hash`. Reusing a configuration
starts fresh state. Deterministic parity is scoped to the recorded Julia and
package versions; wall-clock stopping and cross-version bit identity are excluded.

`chasers` accepts named tuples containing `id`, `start_rtn_m` and `goal_rtn_m`.
They share the passive target specified by `target_id` but own distinct runtime
state. The geometry checks the station only; inter-chaser collisions are not
modeled by this interface.

`rpo_run_report(solution)` returns copies of requests, validation, installed
references, planner settings, seed mapping, held-command samples and saved states.
Reading and changing this report cannot change the run. The returned solution
itself still exposes the usual advanced solver internals; do not mutate them while
running. Reference acceptance is distinct from physical tracking acceptance.

Checkpoint writing/resume and `isolate_state=false` are refused before propagation
or output creation. Faithful restart needs a future contract for planner/RNG,
controller, active-reference and trigger state. General restart remains open work.
A core-only installation can run the direct planner without HYPR; see the optional
installation instructions below.

## Add your own planner or force

Subtype `AbstractRPOPlanner`. Implement `planner_capabilities`, optionally
`initialize_planner`, and `plan_rpo!(state, planner, request, rng)`. Return an
`RPOPlanningResult` containing an `RPOReference` for candidate status. Reference
times are uniform and relative to `origin_time_s`; vectors are SI target RTN,
including the rotating-frame transport term in velocities. The contract guide
lists validation rules. Add `retime_rpo!` only when you also declare retiming.

A custom planner must support `deepcopy`, including when copied per spacecraft,
per run and into failure reports. `initialize_planner` must return fresh state
for each call. Use only the supplied `rng` for randomness; keep mutable runtime
state in that returned state, with no module-level mutable state or background
tasks. The lifecycle isolates cooperating implementations; it is not a sandbox
for arbitrary planner code.

The separate-project smoke defines a tiny test planner using only these public
methods, including an explicit failed result. Its force example uses the existing
`AbstractForceTorqueModel` and `wrench` interface, passed through
`extra_effectors=(model,)`. No internal package module or source include is needed.

## Installing optional HYPR

`using SpaceAGORA` loads the direct planner, simulation lifecycle, shared geometry,
sampling, metrics, RRT, retiming and quintic robot-arm planner. It preserves HYPR
configuration and result types so saved configurations and qualified names retain
their defining modules. It does not load the HYPR optimizer or configured execution.

From a reviewed SpaceAGORA Git clone, prepare a separate environment:

```sh
julia --startup-file=no scripts/optional_packages.jl setup hypr /path/to/my-rpo-project
```

The helper checks out `packages/robotics/HYPR.jl` at the parent repository's
recorded revision, verifies its identity and clean state, and delegates to the
existing HYPR installer with that path. The pin agrees with
`packages/SpaceAGORAHYPR/HYPRSource.toml`. It selects HYPR 0.2.0 alongside
SpaceAGORA 0.2.0 and compatibility shim 0.2.0. The shim keeps its existing location;
HYPR remains optional for the simulator core.

The original external-source route is also supported:

```sh
julia --startup-file=no scripts/setup_hypr.jl /path/to/my-rpo-project
```

Without an override, this route resolves the immutable HYPR source pin directly
from its repository, rather than using the submodule checkout. For coordinated
local development, `SPACEAGORA_HYPR_PATH` explicitly substitutes another local
HYPR checkout; that override is not published-installation evidence. Failed
resolution is an error, not a fallback to another revision. Both routes use the
existing installer's dependency-preservation policy. See [Optional package commands](installation_environment.md#Optional-package-commands)
for source checks and Git-clone requirements.

Run with `--project=/path/to/my-rpo-project` and explicitly load the companion.

```julia
using SpaceAGORA, SpaceAGORAHYPR
@assert hypr_available()
planner = HYPRRPOPlanner(RPOPSOConfig())
```

When the planning configuration carries an acceleration limit, also set
`retime_accel_limit_enable=true` in `RPOPSOConfig`; see
[Built-in planners](../reference/rpo_planner_contract.md#Built-in-planners).

Either package may be loaded first. Without the companion, HYPR selection raises
`HYPRUnavailableError` with installation instructions. Public RPO assembly and
simulation preflight perform this check before creating run outputs. A legacy
`RPOGuidanceModel` needs the companion when it must plan or replan; tracking an
already supplied reference with replanning disabled remains available in core.
Robot-arm `planner=:hypr` also requires explicit loading; `:cloth_quintic` does not.

Comparison planners that request HYPR-configured objectives, refinement, sampling
or retiming need the companion even when their search algorithm is RRT, CHOMP or
STOMP. Shared RRT with explicit policies and shared scalar APIs remain independent.
The baseline LQ-MPC controller still needs OSQP; optional HYPR does not remove that
controller dependency. Package separation changes installation, not the validated
physical limits, planning reserve or numerical policies.

The `examples/rpo_planner_env/setup.jl` command above installs both components
explicitly for the two-planner demonstration. The RPO and compliant robot-arm HYPR examples and ISS viewer demo also run from that project and
load the companion before their common helper activates the root project. See
[Examples Catalog](examples_catalog.md) for the exact commands. Baseline examples
keep `--project=.` and do not preload HYPR. Do not add the companion as a required
dependency of SpaceAGORA itself.

## Moving from RPOGuidanceModel

`RPOGuidanceModel`, the HYPR guidance model of the older RPO examples, is
deprecated. A run that configures it warns once before it starts; the run itself
is unchanged. Build new RPO runs with `make_rpo_configuration` and a planner. For
HYPR planning, pass `planner=HYPRRPOPlanner(RPOPSOConfig(...))`; the configuration
then uses a `PlannerGuidance`, which validates each reference before installing
it.

The RPO examples run on this path: the main example plans inside the run, and
the reconstruction example flies each comparison planner's plan with
`RetainedPathRPOPlanner`, which times a given path for the run's request so that
the run can validate it. The replanning study
(`examples/Earth_RPO_HyPR_FinalISpaRo_Replanning.jl`) stays on the model: it
reproduces the published disturbance families, including a goal change and the
model's brake-then-replan policy.

The planner path covers the model's replanning features:

| `RPOGuidanceModel` | `make_rpo_configuration` |
| --- | --- |
| `replanning_config` spheres | `geometry_provider=RPOReplanningSpheres(spheres)` |
| safe hold when replanning fails | `on_rejection=:install_fallback`, `hold_position_tolerance_m`, `hold_speed_tolerance_mps` |
| retiming on tracking error | `retime_tracking_error_m`, or a `:retime` planning event |
| `safe_distance_m` | the clearance in `constraints` |
| `force_replan` | `RPOPlanningEvent(time_s; reason=:forced)` |

The two differ in policy. The model brakes to a hold before every replan and
retimes when clearance falls below a margin; the planner path replans at once and
holds only when a request is rejected. A run on the planner path also draws from a
different random stream, and its validation can refuse a reference the model would
install, so its trajectories are not identical to the model's. The model will be
removed in a later release.
