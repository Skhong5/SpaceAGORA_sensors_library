# Writing a GNC Algorithm

Use this page to add a controller, an estimator or a planner of your own. Pick
the profile that matches what your algorithm's output does, implement only what
that profile needs, and test it with the engine's own checks.

Read [Your First Script](first_script.md) and the
[Extension Conventions](../reference/extension_conventions.md) first: every hook
below follows their argument order, units, frames and signs. The
[Extension Hook Reference](../generated/extension_hook_reference.md) lists each
hook with its signature and fallback.

The worked examples on this page are scripts in
`examples/gnc_algorithm_env/`, a separate project that uses only the public
`SpaceAGORA` surface. CI runs them, and a gate checks that each code block taken
from them is identical to the script. To run them yourself:

```text
julia --project=examples/gnc_algorithm_env examples/gnc_algorithm_env/setup.jl
julia --project=examples/gnc_algorithm_env examples/gnc_algorithm_env/feedback_controller.jl
```

## Which profile?

Every GNC algorithm runs one loop: take an input, produce an output, check it,
keep it until it is replaced. It runs that loop in one of two profiles.

| Your algorithm | Profile | Built-in examples |
|---|---|---|
| computes a command from the current state at each tick, applied at once | feedback | the RPO MPC controller, the momentum manager, the Apollo descent controller |
| estimates the state from measurements | feedback: a tick that holds an estimate, with no limiting step and no validation | (none built in yet) |
| produces something used later, or that can be checked before use: a reference trajectory, a burn plan, a joint path, a passage decision | planning | `DirectRPOPlanner`, `HYPRRPOPlanner`, `ApoapsisBurnPlanner` |
| plans for a controller | both, paired through a link | the RPO pilot: a planner and the MPC controller share the plan buffer |

The two profiles share one vocabulary:

| Step | Feedback profile | Planning profile |
|---|---|---|
| input | the state at the tick, or a measurement | an owned request snapshot, built by trusted code |
| output | a command or an estimate | a product proposed in a result |
| check | your own limits, in the tick | validation by the algorithm's family, which can reject |
| keep | hold until the next tick | install into a link, use while it is current |

## The feedback profile

A feedback controller has two halves:

- **The tick**, `update_control!(model, u, p, t, spacecraft_index)`, runs once per
  control period. It computes the command from the state `u`, limits it, and
  holds it in the model.
- **The output hooks**, `control_wrench(model, x, p, t)`,
  `control_propellant_rate(model, x, p, t)` and `control_wheel_torque(model, x, p, t)`,
  run at every evaluation of the equations of motion. They return the held
  command and never run the law. The force is inertial, the torque in the body
  frame, and the propellant rate positive.

A controller that drives mounted hardware declares `actuation_mode(model) = :mounted`
and returns the hardware levels from `actuator_levels(model, spacecraft_index)`
instead of the output hooks; see [Adding an Actuator](custom_actuator.md).
Estimators and guidance models tick the same way, with
`update_navigation!(model, u, p, t, spacecraft_index)` and
`update_guidance!(model, u, p, t, spacecraft_index)`, and keep their result for
the controller to read. When ticks coincide, sensors sample first, then
navigation ticks, then control, then guidance; a planning binding's request comes
before the control tick it feeds.

### Declarations

A model declares what it acts on and what it assumes. Each declaration has a
default equal to the engine's usual behaviour, so declare only what differs.
The engine reads them before the run and at each tick.

| Declaration | Default | Declare it when |
|---|---|---|
| `spacecraft_scope(model)` | `AllSpacecraft()`: ticked for every spacecraft | the model acts on some spacecraft only: `BoundSpacecraft(indices)`, or `CoupledSpacecraft(primary, partners)` for a model ticked for `primary` that also reads `partners` |
| `gnc_period_s(model)` | `nothing` | the law assumes a tick period, such as the step it was discretized with. Before the run the engine warns when the control rate differs; SpaceAGORA 0.4.0 will refuse the run |
| `skip_inactive(model)` | `false` | the model should stop ticking for a spacecraft that is no longer active. Each tick then calls `release_command!(model, t, spacecraft_index)` instead, which sets the off command. SpaceAGORA 0.4.0 will make `true` the default |
| `held_command(model, spacecraft_index)`, `held_command_time(model, spacecraft_index)` | `nothing`, `NaN` | the model holds a command: return it, as an immutable value, and the time of the tick that produced it |
| `gnc_links(model)` | none | the model shares an object with a partner: see [Pairing through a link](#Pairing-through-a-link) |

Output hooks are called for every spacecraft whatever the scope, so they check
`x.spacecraft_index` themselves.

A control model that is not ticked periodically declares
`tick_schedule(model) = :event_driven` and brings its own callbacks with
`gnc_callbacks(model, num_sats, args, period_s)`, as the burn scheduler does. A
periodic model can add solver stops after each tick with
`register_tstops!(integrator, model, spacecraft_index)`, and record when its
clock starts with `gnc_clock_started!(model, period_s, origin_s)`. Most models
need none of these.

### Worked example A: a rate damper

`RateDamper` damps a spacecraft's body rate: its law is a torque proportional to
the rate, limited on each axis. The script is
`examples/gnc_algorithm_env/feedback_controller.jl`.

```julia
Base.@kwdef mutable struct RateDamper <: SpaceAGORA.AbstractControlEffectorModel
    spacecraft_index::Int = 1           # the spacecraft it acts on, its position in the run
    gain_n_m_s::Float64 = 2.0           # torque per unit body rate, N·m per rad/s
    max_torque_n_m::Float64 = 1.0       # torque limit on each body axis, N·m
    period_s::Float64 = 1.0             # the tick period the law assumes, s
    # Run state: the held command and the time of the tick that set it.
    torque_body::SVector{3, Float64} = zero(SVector{3, Float64})
    tick_s::Float64 = NaN
end

# Declarations: what the controller acts on and what it assumes.
SpaceAGORA.spacecraft_scope(c::RateDamper) = BoundSpacecraft((c.spacecraft_index,))
SpaceAGORA.gnc_period_s(c::RateDamper) = c.period_s
SpaceAGORA.skip_inactive(::RateDamper) = true
SpaceAGORA.held_command(c::RateDamper, spacecraft_index) = c.torque_body
SpaceAGORA.held_command_time(c::RateDamper, spacecraft_index) = c.tick_s
function SpaceAGORA.release_command!(c::RateDamper, t, spacecraft_index)
    c.torque_body, c.tick_s = zero(SVector{3, Float64}), t
    return nothing
end

# The tick (shape T), once per control period: run the law, limit it, hold the command.
function SpaceAGORA.update_control!(c::RateDamper, u, p, t, spacecraft_index)
    ω_body = SVector{3, Float64}(u.sc[spacecraft_index].ω)
    c.torque_body = clamp.(-c.gain_n_m_s .* ω_body, -c.max_torque_n_m, c.max_torque_n_m)
    c.tick_s = t
    return nothing
end

# The output (shape R), at every evaluation of the equations of motion: the held
# command, never the law. Output hooks are called for every spacecraft.
function SpaceAGORA.control_wrench(c::RateDamper, x::StateSample, p, t)
    zero3 = zero(SVector{3, Float64})
    x.spacecraft_index == c.spacecraft_index || return zero3, zero3
    return zero3, c.torque_body             # force_ii (N), torque_body (N·m)
end

# Declared on a spacecraft: a copy for that spacecraft, with fresh run state.
SpaceAGORA.bind_spacecraft(c::RateDamper, spacecraft_index::Int) = RateDamper(;
    spacecraft_index, gain_n_m_s=c.gain_n_m_s, max_torque_n_m=c.max_torque_n_m, period_s=c.period_s)
```

Run it on a cube far from any planet, spinning about one axis, with nothing else
acting on it:

```julia
# A 500 kg cube far from any planet, spinning about body z; only the controller acts.
root = Link(root=true, m=500.0, dims=MVector(1.0, 1.0, 1.0))
ω0 = 0.05                                    # initial rate, rad/s
ic = CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), SVector(0.0, 0.0, 0.0);
                               q=SVector(0.0, 0.0, 0.0, 1.0), ang_vel=SVector(0.0, 0.0, ω0))
spacecraft = SpacecraftModel(links=[root], root=root, prop_mass=0.0, inertia_tensor=root.inertia,
                             initial_condition=ic, id=1)
damper = RateDamper()
mission_s = 120.0
planet = make_no_gram_planet(:earth)
config = SimulationConfiguration(
    simulation_settings=SimulationSettings(
        results=false, verbose=false, generate_plots=false, normalize=false),
    mission_configuration=MissionConfiguration(
        mission_type=MissionTime, mission_time=mission_s, number_of_orbits=1,
        keplerian=false, orientation_sim=true, num_steps_to_save=120, data_rate=1.0),
    environment_model=EnvironmentModel(
        planet=planet, EI=120.0, density_model=NoAtmosphereModel(),
        ephemerides_model=SimpleEphemeridesModel(),
        thermal_model=MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet),
        topography=false, wind=false),
    dynamics_model=DynamicsModel([spacecraft], ()),
    guidance_model=GuidanceModel(guidance_effectors=(), guidance_rates=Float64[]),
    navigation_model=NavigationModel(navigation_effectors=(), navigation_rates=Float64[]),
    control_model=ControlModel(control_effectors=(damper,), control_rates=[damper.period_s]),
    initial_time=InitialTime(year=2024, month=1, day=1, hour=0, minute=0, second=0.0),
    integration_tolerances=IntegrationTolerances())

solution = run_simulation(config; return_solution=true)
```

The result can be checked by hand. A torque held for one control period on a
body whose inertia is the same about every axis changes the rate by a fixed
fraction:

```julia
# By hand: the first tick is one period after the start, and each held torque
# -k ω(t_n) acts for one period on a cube (no gyroscopic torque), so after the
# ticks at 1, 2, ..., N s the rate is ω0 (1 - k Δt / I)^N.
inertia = root.inertia[3, 3]
ticks = floor(Int, mission_s / damper.period_s) - 1        # ticks before the end
expected = ω0 * (1 - damper.gain_n_m_s * damper.period_s / inertia)^ticks
final = solution.u[end].sc[1].ω[3]
println("body rate after $(mission_s) s: $(final) rad/s, by hand $(expected) rad/s")
@test final ≈ expected rtol=1e-6
```

The important choices:

- **The law runs in the tick only.** `update_control!` reads the rate from
  `u.sc[spacecraft_index]` (`ω` is the body rate in rad/s), limits the torque and
  stores it. `control_wrench` returns the stored torque at every evaluation, so
  the vehicle feels a zero-order hold of the command, as it would from flight
  software.
- **The command is held until the first tick.** The first tick is one control
  period after the start; until then the initial held command acts, here zero.
- **The declarations state the assumptions.** `gnc_period_s` lets the engine
  warn if the control rate in `ControlModel(control_rates=...)` is not the period
  the gain was tuned for. `held_command` and `held_command_time` make the held
  command readable outside the model.
- **Registration.** Add the model to `ControlModel(control_effectors=(...),
  control_rates=[...])`, or declare it on a spacecraft with
  `SpacecraftModel(...; control=ControlModel(...))`: the engine then calls
  `bind_spacecraft(model, spacecraft_index)`, which returns a copy for that
  spacecraft with fresh run state.

!!! note "TODO(0.3.0 PR 1): lifecycle hooks"
    When GNC PR 12 makes the lifecycle hooks public, document here
    `initialize_navigation!`, `initialize_guidance!` and `initialize_control!`
    (their order at the run's start, and that they reset run state so that two
    runs with `isolate_state=false` start equal), add an `initialize_control!`
    method to `RateDamper` and the control template, and link the conformance
    check `initialize_resets`.

### Worked example B: an estimator and a link

An estimator ticks and holds, like a controller, but its output is an estimate
for other models to read. `GyroRateEstimator` reads a gyro and holds the
measured rate; `EstimatedRateDamper` is example A's controller reading that
estimate instead of the true state. The script is
`examples/gnc_algorithm_env/estimator.jl`.

#### Pairing through a link

The two models share one object, a *link*. Each declares its part with
`gnc_links(model)`: `Produces(link)` for the model that writes it,
`Consumes(link)` for a model that reads it. The engine groups the run's models
by link, which survives the deep copy of the configuration, and before the run
checks each link's counts against `link_cardinality(link)` and calls
`check_link(link, producers, consumers, args)`, where you can refuse settings
that disagree. At the start of every run it calls
`link_initialize!(link, producers, consumers, u, p, t)`, before any model's own
initializer, so the link starts empty in every run.

```julia
# The estimate the navigation model writes and the controller reads: one shared object.
mutable struct RateEstimate
    ω_body::SVector{3, Float64}     # estimated body rate, rad/s
    time_s::Float64                 # time of the measurement behind it, s into the run
end
RateEstimate() = RateEstimate(zero(SVector{3, Float64}), NaN)

# One estimator writes it; any number of controllers read it. Each run starts empty.
SpaceAGORA.link_cardinality(::RateEstimate) = (producers = 1:1, consumers = 0:typemax(Int))
function SpaceAGORA.link_initialize!(e::RateEstimate, producers, consumers, u, p, t)
    e.ω_body, e.time_s = zero(SVector{3, Float64}), NaN
    return nothing
end
```

The estimator reads the latest measurement of the gyro mounted on its
spacecraft with `sensor_measurement` and `sensor_measurement_time`:

```julia
struct GyroRateEstimator <: SpaceAGORA.AbstractNavigationModel
    spacecraft_index::Int
    gyro::Int                       # the gyro's position among the spacecraft's sensors
    estimate::RateEstimate
end
SpaceAGORA.spacecraft_scope(n::GyroRateEstimator) = BoundSpacecraft((n.spacecraft_index,))
SpaceAGORA.gnc_links(n::GyroRateEstimator) = (Produces(n.estimate),)

# The tick: read the latest measurement and hold it as the estimate. Sensors
# sample before navigation ticks at the same time.
function SpaceAGORA.update_navigation!(n::GyroRateEstimator, u, p, t, spacecraft_index)
    spacecraft = p.args.dynamics_model.spacecraft[spacecraft_index]
    n.estimate.ω_body = SVector{3, Float64}(sensor_measurement(spacecraft, n.gyro))
    n.estimate.time_s = sensor_measurement_time(spacecraft, n.gyro)
    return nothing
end
```

The controller declares that it consumes the estimate:

```julia
# The rate damper of example A, reading the estimate instead of the true state.
Base.@kwdef mutable struct EstimatedRateDamper <: SpaceAGORA.AbstractControlEffectorModel
    spacecraft_index::Int = 1
    gain_n_m_s::Float64 = 2.0
    max_torque_n_m::Float64 = 1.0
    period_s::Float64 = 1.0
    estimate::RateEstimate
    torque_body::SVector{3, Float64} = zero(SVector{3, Float64})
    tick_s::Float64 = NaN
end
SpaceAGORA.spacecraft_scope(c::EstimatedRateDamper) = BoundSpacecraft((c.spacecraft_index,))
SpaceAGORA.gnc_period_s(c::EstimatedRateDamper) = c.period_s
SpaceAGORA.gnc_links(c::EstimatedRateDamper) = (Consumes(c.estimate),)
SpaceAGORA.held_command(c::EstimatedRateDamper, spacecraft_index) = c.torque_body
SpaceAGORA.held_command_time(c::EstimatedRateDamper, spacecraft_index) = c.tick_s

function SpaceAGORA.update_control!(c::EstimatedRateDamper, u, p, t, spacecraft_index)
    c.torque_body = clamp.(-c.gain_n_m_s .* c.estimate.ω_body, -c.max_torque_n_m, c.max_torque_n_m)
    c.tick_s = t
    return nothing
end

function SpaceAGORA.control_wrench(c::EstimatedRateDamper, x::StateSample, p, t)
    zero3 = zero(SVector{3, Float64})
    x.spacecraft_index == c.spacecraft_index || return zero3, zero3
    return zero3, c.torque_body
end
```

With an exact gyro sampled at the control ticks, the pair gives example A's
result: the gyro samples, then the estimator ticks, then the controller, all at
the same time.

```julia
root = Link(root=true, m=500.0, dims=MVector(1.0, 1.0, 1.0))
ω0 = 0.05
ic = CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), SVector(0.0, 0.0, 0.0);
                               q=SVector(0.0, 0.0, 0.0, 1.0), ang_vel=SVector(0.0, 0.0, ω0))
spacecraft = SpacecraftModel(links=[root], root=root, prop_mass=0.0, inertia_tensor=root.inertia,
                             initial_condition=ic, id=1)
add_sensor!(spacecraft, root, RateGyro(period_s=1.0))      # no noise and no bias: exact rates
estimate = RateEstimate()
estimator = GyroRateEstimator(1, 1, estimate)
damper = EstimatedRateDamper(estimate=estimate)
mission_s = 120.0
planet = make_no_gram_planet(:earth)
config = SimulationConfiguration(
    simulation_settings=SimulationSettings(
        results=false, verbose=false, generate_plots=false, normalize=false),
    mission_configuration=MissionConfiguration(
        mission_type=MissionTime, mission_time=mission_s, number_of_orbits=1,
        keplerian=false, orientation_sim=true, num_steps_to_save=120, data_rate=1.0),
    environment_model=EnvironmentModel(
        planet=planet, EI=120.0, density_model=NoAtmosphereModel(),
        ephemerides_model=SimpleEphemeridesModel(),
        thermal_model=MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet),
        topography=false, wind=false),
    dynamics_model=DynamicsModel([spacecraft], ()),
    guidance_model=GuidanceModel(guidance_effectors=(), guidance_rates=Float64[]),
    navigation_model=NavigationModel(navigation_effectors=(estimator,), navigation_rates=[1.0]),
    control_model=ControlModel(control_effectors=(damper,), control_rates=[damper.period_s]),
    initial_time=InitialTime(year=2024, month=1, day=1, hour=0, minute=0, second=0.0),
    integration_tolerances=IntegrationTolerances())

solution = run_simulation(config; return_solution=true)

# With an exact gyro sampled at the control ticks, the controller sees the true
# rate at each tick, and the run matches example A's closed form.
inertia = root.inertia[3, 3]
ticks = floor(Int, mission_s / damper.period_s) - 1
expected = ω0 * (1 - damper.gain_n_m_s * damper.period_s / inertia)^ticks
final = solution.u[end].sc[1].ω[3]
println("body rate after $(mission_s) s with the estimate: $(final) rad/s, by hand $(expected) rad/s")
@test final ≈ expected rtol=1e-6
```

A gyro with noise or a bias gives an estimate that differs from the true rate;
see the sensor kinds under [Extensibility](../extensibility.md#Add-a-sensor).

## The planning profile

A planning algorithm proposes; its *family* decides. A family is trusted,
core-owned code that defines one problem: what a request contains, what product
an algorithm proposes, how a proposal is validated, where an accepted product is
installed, when it stops being current, and what happens on rejection. Your
algorithm never validates or installs its own result.

Each request runs through the same steps, in `run_algorithm_request!`:

1. Trusted code builds an owned request snapshot of the state, the goal, the
   limits and the request's validity, and the binding records a copy.
2. The family checks the algorithm's declared capabilities against the request.
3. The algorithm proposes, from its own copy of the request: `propose!(state,
   algorithm, request, rng)`, or `propose_revision!(state, algorithm, request,
   active_product, rng)` when the request asks to revise the installed product.
4. The family validates a copy of the proposal against its retained request.
5. An accepted product is installed into the link it shares with its consumer,
   with nothing running between validation and installation.
6. The binding records the outcome.

When the capability check or the validation refuses, the family's
`on_rejection` policy applies: `:fail` stops the run with a `GNCAlgorithmError`
that carries the run report, `:keep_previous` keeps the installed product and
records the refusal, and `:install_fallback` installs the family's
`fallback_product`, such as a hold.

What you implement for an algorithm:

| Function | Required | What it does |
|---|---|---|
| `algorithm_family(algorithm)` | yes, usually inherited from the family's abstract type | the family whose rules govern the algorithm |
| `propose!(state, algorithm, request, rng)` | yes | a result for the request: a `:candidate` with a product, or `:infeasible`, `:unsupported` or `:failed` with a reason |
| `propose_revision!(state, algorithm, request, active_product, rng)` | no | a revision of the installed product; unsupported by default, never a silent full proposal |
| `algorithm_capabilities(algorithm)` | no | what it supports, in the family's capability type; by default nothing (`default_capabilities(family)`) |
| `initialize_algorithm(algorithm, run_context)` | no | fresh run state for each run; `nothing` by default |
| `require_algorithm_support(algorithm)` | no | throw before the run if something it needs, such as an optional package, is missing |
| `consumes_rng(algorithm)` | no | `false` for an algorithm that draws nothing from `rng`; the default is `true` |

The families:

| Family | Request and product | Validation | Installed into | On rejection | Algorithms | Runs in a simulation through |
|---|---|---|---|---|---|---|
| RPO reference, `RPOReferenceFamily` | `RPOPlanningRequest`, `RPOReference` | `validate_rpo_result`: identity, timing, endpoints, speed and acceleration limits, sampled clearance | the RPO plan buffer, read by the MPC controller | `:fail`, or `:install_fallback` (a hold) when configured | `DirectRPOPlanner`, `HYPRRPOPlanner` (with `using SpaceAGORAHYPR`), the seven comparison planners | `make_rpo_configuration(planner=...)` |
| Burn plan, `BurnPlanFamily` | `BurnPlanRequest`, a timed constant-thrust burn | `validate_burn_plan`: the burn scheduler's own size check, thrust, specific impulse and window | the spacecraft's slot in the burn scheduler's plan buffer, `BurnPlanSlot` | `:keep_previous` | `ApoapsisBurnPlanner` | `AlgorithmGuidance` |

!!! note "TODO(0.3.0 PR 1 and PR 2): families table"
    Add the joint-reference family (GNC PR 17, PR 1 of 0.3.0: request, product,
    validation, install target on the robot-arm controller, `:fail`) and the EDG
    passage-decision family (GNC PR 18, PR 2 of 0.3.0, served through
    `EDGServices` 2.0.0, `consumes_rng = false`), with their algorithms and how
    each runs.

### Worked example C: an RPO planner

For the RPO family the RPO names are the supported way to write a planner:
subtype `AbstractRPOPlanner` and implement `planner_capabilities` and
`plan_rpo!`, and optionally `initialize_planner` and `retime_rpo!`. They forward
to the general names (`algorithm_capabilities`, `propose!`,
`initialize_algorithm`, `propose_revision!`); the
[RPO planner contract](../reference/rpo_planner_contract.md#The-general-algorithm-contract)
maps one to the other. The script is `examples/gnc_algorithm_env/rpo_planner.jl`.

```julia
# A reference along the straight line to the goal, in the target's RTN frame: it
# accelerates, cruises and brakes within fractions of the request's limits. It
# ignores obstacles; the family's validation refuses a line that comes too close.
Base.@kwdef struct LinePlanner <: AbstractRPOPlanner
    speed_fraction::Float64 = 0.5            # of the request's speed limit
    acceleration_fraction::Float64 = 0.5     # of the request's acceleration limit
end

SpaceAGORA.planner_capabilities(::LinePlanner) =
    RPOPlannerCapabilities(state_sources=(:truth,), frames=(:target_rtn,))
SpaceAGORA.consumes_rng(::LinePlanner) = false     # it draws nothing from `rng`

function SpaceAGORA.plan_rpo!(state, planner::LinePlanner, request::RPOPlanningRequest, rng::AbstractRNG)
    refuse(why) = RPOPlanningResult(request_id=request.request_id, status=:infeasible, termination=why)
    start, goal = collect(Float64, request.x_rtn[1:3]), collect(Float64, request.goal_rtn_m)
    distance = norm(goal - start)
    v = planner.speed_fraction * something(request.constraints.max_speed_mps, Inf)
    a = planner.acceleration_fraction * something(request.constraints.max_acceleration_mps2, Inf)
    isfinite(v) && isfinite(a) && v > 0 && a > 0 || return refuse(:needs_speed_and_acceleration_limits)
    v = min(v, sqrt(distance * a))           # too short to reach cruise speed: a triangle
    ramp_s = v / a
    duration_s = distance == 0 ? 0.0 : distance / v + ramp_s
    dt = request.reference_dt_s
    steps = max(1, ceil(Int, duration_s / dt))
    request.time_s + steps * dt <= request.valid_until_s || return refuse(:outlives_the_request)
    # Distance along the line and speed at time t from the start.
    function along(t)
        t >= duration_s && return (distance, 0.0)
        t < ramp_s && return (a * t^2 / 2, a * t)
        t <= duration_s - ramp_s && return (v * (t - ramp_s / 2), v)
        left = duration_s - t
        return (distance - a * left^2 / 2, a * left)
    end
    direction = distance == 0 ? zeros(3) : (goal - start) / distance
    times = [k * dt for k in 0:steps]
    profile = along.(times)
    reference = RPOReference(t_ref_s=times,
        r_ref_rtn_m=reduce(hcat, [start + s * direction for (s, _) in profile]),
        v_ref_rtn_mps=reduce(hcat, [speed * direction for (_, speed) in profile]),
        origin_time_s=request.time_s, valid_until_s=request.valid_until_s,
        chaser_id=request.chaser_id, target_id=request.target_id,
        geometry_revision=request.geometry_revision)
    return RPOPlanningResult(request_id=request.request_id, status=:candidate,
        termination=:completed, reference=reference, diagnostics=(duration_s=duration_s,))
end
```

Run it in the RPO pilot and read the run report:

```julia
# The RPO pilot: a chaser 3 m from the station plans to 5 m, and an MPC controller
# tracks the installed reference.
configuration = make_rpo_configuration(planner=LinePlanner(), mission_time_s=2.0)
solution = run_simulation(configuration; return_solution=true)
report = rpo_run_report(solution)
installed = only(filter(record -> record.event === :installed, only(report.planners).records))
println("installed request ", installed.request_id, ": ", installed.validation.reason,
        ", reference of ", last(installed.reference.t_ref_s), " s")
@test installed.validation.accepted
```

The important choices:

- **A reason, not an exception.** A planner that cannot plan returns a result
  with a status and a termination reason. Under the RPO family's `:fail` policy
  the run then stops with an `RPOPlanningError` that names it.
- **Declared capabilities.** `LinePlanner` declares truth states in the target's
  RTN frame and no retiming. The family checks every request against them before
  asking the planner, and refuses one that needs more.
- **No randomness, declared.** `consumes_rng = false` lets the checks confirm
  that the stream is left untouched. A planner that does draw uses only `rng`,
  the stream the binding derives from the run's seed and the spacecraft IDs.

[Run an RPO case with interchangeable planners](rpo_planner_pilot.md) covers the
pilot's replanning, holds, retiming and inspection.

### Other families

A planner of another family runs through `AlgorithmGuidance`, the generic
binding, among the guidance effectors:

```julia
binding = AlgorithmGuidance(algorithm=ApoapsisBurnPlanner(delta_v_mps=5.0), spacecraft_index=1,
                            events=[AlgorithmEvent(0.0)])
guidance = GuidanceModel(guidance_effectors=(binding,), guidance_rates=[60.0])
```

`AlgorithmGuidance(; algorithm, spacecraft_index, settings, link, seed, events,
replan_interval_s)` makes a request at each `AlgorithmEvent` and, with a finite
`replan_interval_s`, at that interval, at its guidance ticks. `settings` are the
family's request settings and `link` its install target, for a family that does
not find its own in the run. The binding's random stream comes from `seed`, the
family and the spacecraft's ID. `algorithm_run_report(solution)`, on a run made
with `return_solution=true`, returns every binding's records: each request,
installation, rejection and failure, in order.

The burn-plan family installs into the burn scheduler, `BaseThrusterModel`: the
configuration needs exactly one, planning no burn of its own for that spacecraft,
and no thruster guidance model beside the binding. The scheduler takes up an
installed plan at the end of the step it was installed in and fires the engine
inside the plan's window.

!!! note "TODO(0.3.0 PR 1 and PR 2): other families"
    Add how the joint-reference family runs (its binding and the arm controller
    it installs into) and how the EDG family's decisions reach the
    energy-depletion guidance through `EDGServices` 2.0.0.

### The family side

These functions are core-owned: an algorithm never implements them. They appear
in run reports, in the [GNC Algorithm Contract API](../generated/gnc_algorithm_contract_api.md)
and in the reference table, and a new family implements them in a SpaceAGORA
pull request (the repository's contributor guide,
`docs/contributing/gnc_algorithm_families.md`, lists the steps).

- **The family:** `validate_capabilities`, `validate_proposal`, `install!`,
  `is_current`, `on_rejection`, `fallback_product`, `family_contract_version`
  (for a family a companion package serves), `algorithm_result_type`,
  `is_revision_request`, `proposed_product`, and the run records
  `installed_record` and `rejected_record`.
- **What `AlgorithmGuidance` asks of a family:** `family_request` (the owned
  request), `family_link` (the install target) and `preflight_family` (the
  configuration check before the run).
- **The binding**, the guidance model that runs an algorithm:
  `algorithm_runtime` (its run-owned `AlgorithmRuntime`), `bound_algorithm`,
  `bound_family`, `bound_link`, `build_algorithm_request`, `trusted_queries`
  and `fail_algorithm_request!`. `AlgorithmGuidance` and the RPO pilot's binding
  implement them.

## House rules for both profiles

- **Draw randomness only from the `rng` you are given**, never from Julia's
  global stream: planning functions take an explicit stream and have no default
  one, and the conformance checks rerun your model with the global stream
  reseeded.
- **Keep run state in the model** (feedback) or in the state
  `initialize_algorithm` returns (planning), never in globals.
- **Survive `deepcopy`.** `run_simulation` runs a copy of the configuration unless
  `isolate_state=false`, and a link shared by two models stays shared in the copy.
- **`bind_spacecraft` gives fresh run state** to each spacecraft's copy.
- **Keep the output hooks pure:** no law, no warnings, no I/O and no heavy
  allocation. They run at every evaluation of the equations of motion.

## Test your algorithm

`SpaceAGORA.Conformance` runs the engine's own machinery on your model or
algorithm: the pre-run hook check, short runs repeated sequentially, after a
`deepcopy`, with the global random stream reseeded and on two threads, and, for
planners, the family's request driver. Each `check_*` function returns a report;
with `using Test`, each `test_*` function records the same checks as tests. Each
example script ends with them:

```julia
# The engine's own checks on the controller: hooks, reproducibility, copies,
# output-hook purity and the copies `bind_spacecraft` makes.
@testset "RateDamper conformance" begin
    test_control_model(RateDamper())
end
```

For an estimator, `test_navigation_model` mounts the sensors it reads; a model
that needs its partner is checked on a fixture that holds both:

```julia
@testset "Estimator and controller conformance" begin
    # The estimator alone, with a gyro mounted for it.
    test_navigation_model(GyroRateEstimator(1, 1, RateEstimate()); sensors=(RateGyro(period_s=1.0),))
    # The controller needs its producer: check it on a fixture that holds the pair.
    shared = RateEstimate()
    controller = EstimatedRateDamper(estimate=shared)
    fixture = conformance_fixture(navigation=(GyroRateEstimator(1, 1, shared),), control=(controller,),
                                  sensors=(RateGyro(period_s=1.0),), navigation_rate_s=1.0)
    test_control_model(controller; fixture=fixture)
end
```

A planner is checked against its family's representative requests:

```julia
# The family's representative requests, its trusted validation queries and a short
# simulation through the RPO binding.
@testset "LinePlanner conformance" begin
    test_algorithm(LinePlanner(), rpo_conformance_requests(); trusted=rpo_trusted_queries(),
                   simulation=rpo_conformance_simulation())
end
```

For the burn-plan family, use `burn_plan_conformance_requests()` and
`burn_plan_conformance_simulation()`. Run the checks with two or more threads,
so that concurrent runs are compared with sequential ones. A pass certifies
interface behaviour only: the method, its parameters and the claims made from it
stay with you.

To keep your algorithm in a repository of your own, tested against a recorded
SpaceAGORA commit, start from a contributor starter: see
[Building Your Own Research Repository](research_repository.md).

## Templates

- `templates/control_hook_template.jl`: a controller in the feedback profile,
  with its declarations, a tick that holds the command and the output hooks.
- `templates/gnc_planner_template.jl`: an RPO planner on the public names, and
  the steps for another family.

## Deprecated paths

- **`RPOGuidanceModel`** warns before the run and is removed in a later release.
  Use `make_rpo_configuration(planner=HYPRRPOPlanner(...))`; see
  [Moving from RPOGuidanceModel](rpo_planner_pilot.md#Moving-from-RPOGuidanceModel).
- **Selecting an RPO planner by a `Symbol`**, such as `:rrt_star`, warns before
  the run and is removed in SpaceAGORA 0.5.0. Pass a planner object, such as
  `RRTStarPlanner()`.
- **The old hook names** (`calcControlEffect!`, `calcControlForceTorque`,
  `calcControlMassFlowRate`, `calcGuidanceEffect!`, `calcNavigationEffect!` and
  the rest) still run with a warning; see
  [Migrating Extension Hooks](../reference/migrating_extension_hooks.md).
