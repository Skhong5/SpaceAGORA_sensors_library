# The run-time declarations of control models (GNCRuntime), and the run-start
# initialization of navigation effectors and of a sensor's own state.
module GNCRuntimeDeclarationsTests
using Test, StaticArrays, LinearAlgebra, Logging, SpaceAGORA
using DiffEqCallbacks: PresetTimeCallback
import SpaceAGORA.TelemetryVerification: make_example_config
const SM = SpaceAGORA.SimulationModel
const L = SpaceAGORA.SimulationLifecycle
const ZERO3 = SVector(0.0, 0.0, 0.0)

# The order in which the run-start initializers ran.
const LOG = Symbol[]

struct LoggedNavigation <: AbstractNavigationModel end
SpaceAGORA.calcNavigationEffect!(::LoggedNavigation, u, p, t::Float64, i::Int) = nothing
L.initialize_navigation!(::LoggedNavigation, u, p, t) = (push!(LOG, :navigation); nothing)

struct LoggedGuidance <: AbstractGuidanceModel end
SpaceAGORA.calcGuidanceEffect!(::LoggedGuidance, u, p, t::Float64, i::Int) = nothing
L.initialize_guidance!(::LoggedGuidance, u, p, t) = (push!(LOG, :guidance); nothing)

# A periodic controller that records its clock starts, ticks and stop requests.
mutable struct ClockedControl <: SM.AbstractControlEffectorModel
    clock::Vector{Tuple{Float64, Float64}}
    ticks::Int
    stops::Int
end
ClockedControl() = ClockedControl(Tuple{Float64, Float64}[], 0, 0)
SpaceAGORA.calcControlEffect!(m::ClockedControl, u, p, t, i) = (m.ticks += 1; nothing)
SpaceAGORA.calcControlForceTorque(::ClockedControl, u, p, i, t) = (ZERO3, ZERO3)
L.initialize_control!(::ClockedControl, u, p, t) = (push!(LOG, :control); nothing)
SpaceAGORA.gnc_clock_started!(m::ClockedControl, period_s, origin_s) = (push!(m.clock, (period_s, origin_s)); nothing)
SpaceAGORA.register_tstops!(integrator, m::ClockedControl, spacecraft_index) = (m.stops += 1; nothing)

# An event-driven controller: no periodic tick, one callback of its own.
mutable struct ScheduledControl <: SM.AbstractControlEffectorModel
    fired::Vector{Float64}
    ticks::Int
    period_s::Float64
end
ScheduledControl() = ScheduledControl(Float64[], 0, NaN)
SpaceAGORA.calcControlEffect!(m::ScheduledControl, u, p, t, i) = (m.ticks += 1; nothing)
SpaceAGORA.calcControlForceTorque(::ScheduledControl, u, p, i, t) = (ZERO3, ZERO3)
SpaceAGORA.tick_schedule(::ScheduledControl) = :event_driven
function SpaceAGORA.gnc_callbacks(m::ScheduledControl, num_sats, args, period_s)
    m.period_s = period_s
    return (PresetTimeCallback([1.5], integrator -> push!(m.fired, integrator.t); save_positions=(false, false)),)
end

# A schedule the engine does not know.
struct OddControl <: SM.AbstractControlEffectorModel end
SpaceAGORA.calcControlEffect!(::OddControl, u, p, t, i) = nothing
SpaceAGORA.calcControlForceTorque(::OddControl, u, p, i, t) = (ZERO3, ZERO3)
SpaceAGORA.tick_schedule(::OddControl) = :sometimes

# A thread-safe periodic law that records, per spacecraft, the thread of its
# last tick and of its last stop registration.
struct ThreadedLaw <: SM.AbstractControlEffectorModel
    tick_thread::Vector{Int}
    stop_thread::Vector{Int}
end
ThreadedLaw(n) = ThreadedLaw(zeros(Int, n), zeros(Int, n))
SpaceAGORA.calcControlEffect!(m::ThreadedLaw, u, p, t, i) = (m.tick_thread[i] = Threads.threadid(); nothing)
SpaceAGORA.calcControlForceTorque(::ThreadedLaw, u, p, i, t) = (ZERO3, ZERO3)
SM.SimulationCallbacks.control_model_threadsafe(::ThreadedLaw) = true
SpaceAGORA.register_tstops!(integrator, m::ThreadedLaw, i) = (m.stop_thread[i] = Threads.threadid(); nothing)

function constellation_args(control; n=4, mission_time=5.0)
    planet = SM.make_no_gram_planet(:earth)
    sats = map(1:n) do id
        root = SM.Link(root=true, m=100.0, ref_area=2.0)
        ic = SM.InitialCondition(ra=planet.Rp_e + 700e3, rp=planet.Rp_e + 650e3, i=45.0, ω=0.0, Ω=30.0id, ν=10.0)
        SM.SpacecraftModel(joints=SM.Joint[], links=[root], root=root, prop_mass=0.0, inertia_tensor=root.inertia,
            initial_condition=ic, id=id)
    end
    env = SM.EnvironmentModel(planet=planet, EI=120.0, density_model=SM.NoAtmosphereModel(),
        ephemerides_model=SM.SimpleEphemeridesModel(),
        thermal_model=SM.MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet), topography=false, wind=false)
    return SM.SimulationConfiguration(
        simulation_settings=SM.SimulationSettings(results=false, verbose=false, generate_plots=false, normalize=false),
        mission_configuration=SM.MissionConfiguration(mission_type=SM.MissionTime, keplerian=true, number_of_orbits=1,
            mission_time=mission_time, orientation_sim=false, num_steps_to_save=100),
        environment_model=env,
        dynamics_model=SM.DynamicsModel(SM.SpacecraftModel[sats...], (SM.InverseSquaredJ2GravityModel(),)),
        guidance_model=SM.GuidanceModel((), Float64[]), navigation_model=SM.NavigationModel((), Float64[]),
        control_model=SM.ControlModel(control_effectors=(control,), control_rates=[1.0]),
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0))
end

# A periodic law that declares the period it assumes.
struct PeriodicLaw <: SM.AbstractControlEffectorModel
    period_s::Float64
end
SpaceAGORA.calcControlEffect!(::PeriodicLaw, u, p, t, i) = nothing
SpaceAGORA.calcControlForceTorque(::PeriodicLaw, u, p, i, t) = (ZERO3, ZERO3)
SpaceAGORA.gnc_period_s(m::PeriodicLaw) = m.period_s

# The period warnings `f()` gives. Under --depwarn=error the shared pre-run
# warning throws instead, with the same message.
function period_warnings(f)
    is_period(msg) = occursin("will refuse this run", msg)
    if Base.JLOptions().depwarn == 2
        try
            f()
            return String[]
        catch e
            msg = sprint(showerror, e)
            is_period(msg) || rethrow()
            return [msg]
        end
    end
    logger = Test.TestLogger(min_level=Logging.Warn)
    with_logger(f, logger)
    return filter(is_period, [string(l.message) for l in logger.logs])
end
period_messages(args) = period_warnings(() -> SM.SimulationCallbacks.check_control_periods(args))

# A sensor with state of its own: the number of samples since the run started.
mutable struct CountingSensor <: AbstractSensor
    period_s::Float64
    seed::Int
    rng::Any
    latest::SVector{1, Float64}
    latest_time_s::Float64
    count::Int
    initialized_at::Vector{Float64}
end
CountingSensor() = CountingSensor(1.0, 1, nothing, SVector(NaN), NaN, 0, Float64[])
SpaceAGORA.sensor_dimension(::CountingSensor) = 1
SpaceAGORA.sensor_measure(s::CountingSensor, truth::SensorTruth, ctx, rng) = SVector(Float64(s.count += 1))
function SpaceAGORA.initialize_sensor!(s::CountingSensor, truth::SensorTruth, ctx, rng)
    s.count = 0
    push!(s.initialized_at, truth.t_s)
    return nothing
end

function runtime_args(control; sensor=CountingSensor(), mission_time=4.0)
    root = SM.Link(root=true, m=50.0, dims=MVector(1.0, 1.0, 1.0))
    r = SM.Earth().Rp_e + 500e3
    ic = SM.CartesianInitialCondition(SVector(r, 0.0, 0.0), SVector(0.0, sqrt(SM.Earth().μ / r), 0.0);
        q=SVector(0.0, 0.0, 0.0, 1.0), ang_vel=ZERO3)
    sc = SM.SpacecraftModel(links=[root], root=root, inertia_tensor=root.inertia, initial_condition=ic, id=3)
    add_sensor!(sc, root, sensor)
    base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=mission_time,
        initial_time=SM.InitialTime(year=2024, month=3, day=20, hour=0, minute=0, second=0.0),
        dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
        keplerian=false, EI_km=100.0, verbose=false, results=false)
    return SM.SimConfig._with_configuration(base;
        navigation_model=SM.NavigationModel(navigation_effectors=(LoggedNavigation(),), navigation_rates=[1.0]),
        guidance_model=SM.GuidanceModel(guidance_effectors=(LoggedGuidance(),), guidance_rates=[1.0]),
        control_model=SM.ControlModel(control_effectors=control, control_rates=fill(0.5, length(control))))
end

@testset "GNC runtime declarations" begin
    @testset "defaults keep the periodic tick" begin
        @test tick_schedule(:any_model) === :periodic
        @test gnc_callbacks(:any_model, 2, nothing, 0.1) === ()
        @test register_tstops!(nothing, :any_model, 1) === nothing
        @test gnc_clock_started!(:any_model, 0.1, 0.0) === nothing
        @test L.initialize_navigation!(:any_model, nothing, nothing, 0.0) === nothing
        @test tick_schedule === SpaceAGORA.GNCRuntime.tick_schedule
        @test initialize_sensor! === SM.Sensors.initialize_sensor!
        # The sensor fallback types nothing, so a sensor's method that types only
        # some arguments is not ambiguous with it.
        s = CountingSensor(); s.count = 3
        truth = SensorTruth(ZERO3, ZERO3, SVector(0.0, 0.0, 0.0, 1.0), ZERO3, 1.0, 0.0, ZERO3)
        @test initialize_sensor!(s, truth, actuator_context(SM.Link(root=true, m=1.0)), nothing) === nothing
        @test s.count == 0 && s.initialized_at == [0.0]
    end

    @testset "built-in models declare what the engine used to test by type" begin
        thruster = SM.BaseThrusterModel(thrust=[0.0], direction=[0.0], Δv=[0.0], start_burn_time=[0.0],
            stop_burn_time=[0.0], Isp=[300.0])
        @test tick_schedule(thruster) === :event_driven
        @test !SM.SimulationCallbacks.control_requires_periodic_callback(thruster)
        @test SM.SimulationCallbacks.control_requires_periodic_callback(ClockedControl())
        # Its schedule callback and its burn-edge callback; no mounted tick in :ideal actuation.
        @test length(gnc_callbacks(thruster, 1, runtime_args((thruster,)), 0.5)) == 2
        own(f, T) = which(f, T).module !== SpaceAGORA.GNCRuntime
        @test own(register_tstops!, Tuple{Any, SM.BaseThrusterModel, Int})
        @test own(gnc_clock_started!, Tuple{SM.AerobrakingEnergyDepletionControlModel, Float64, Float64})
    end

    @testset "a run follows the declarations and initializes in order" begin
        empty!(LOG)
        sensor, clocked, scheduled = CountingSensor(), ClockedControl(), ScheduledControl()
        args = runtime_args((clocked, scheduled); sensor=sensor)
        ticks_per_run = Ref(0)
        for run in 1:2
            run_simulation(args; isolate_state=false)
            run == 1 && (ticks_per_run[] = clocked.ticks)
            @test LOG == repeat([:navigation, :guidance, :control], run)
            @test clocked.clock == fill((0.5, 0.0), run)           # one clock start per solve
            @test ticks_per_run[] > 0 && clocked.ticks == run * ticks_per_run[]
            @test clocked.stops == clocked.ticks                   # one stop request per tick
            @test scheduled.ticks == 0 && scheduled.fired == fill(1.5, run) && scheduled.period_s == 0.5
            @test sensor.initialized_at == fill(0.0, run)         # once per run, before the first sample
            @test sensor.count == 5                               # this run's samples only: 0, 1, 2, 3, 4 s
        end
    end

    @testset "stop registration runs on one thread when the ticks are threaded" begin
        law = ThreadedLaw(4)
        withenv("SPACEAGORA_CONTROL_CALLBACK_PARALLEL" => "on", "SPACEAGORA_CONTROL_CALLBACK_THREAD_THRESHOLD" => "2") do
            run_simulation(constellation_args(law); isolate_state=false)
        end
        @test all(>(0), law.tick_thread) && all(>(0), law.stop_thread)
        @test length(unique(law.stop_thread)) == 1                 # one thread registers every stop
        Threads.nthreads() > 1 && @test length(unique(law.tick_thread)) > 1   # while the ticks spread
    end

    @testset "declared periods are checked before the run" begin
        @test gnc_period_s(:any_model) === nothing
        # The MPC controllers keep the step they were discretized with.
        ctrl = SM.init_rpo_lqmpc(0.001, 0.2, Matrix(1.0I, 6, 6), Matrix(1.0I, 3, 3), Matrix(1.0I, 6, 6), 4)
        @test ctrl.dt_s == 0.2 && deepcopy(ctrl).dt_s == 0.2
        # A contributor's declared period against its callback's rate (0.5 s).
        @test isempty(period_messages(runtime_args((PeriodicLaw(0.5),))))
        warned = period_messages(runtime_args((PeriodicLaw(0.25),)))
        @test length(warned) == 1 && occursin("0.25 s period", only(warned)) && occursin("every 0.5 s", only(warned))
        # A configuration check, not a deprecation: the :preflight log group.
        if Base.JLOptions().depwarn != 2
            logger = Test.TestLogger(min_level=Logging.Warn)
            with_logger(() -> SM.SimulationCallbacks.check_control_periods(runtime_args((PeriodicLaw(0.25),))), logger)
            @test only(logger.logs).group === :preflight
        end
        # The RPO pilot builds its controller with the model's step; a different step warns.
        rpo = make_rpo_configuration(planner=DirectRPOPlanner(), mission_time_s=0.2)
        mpc = only(rpo.control_model.control_effectors)
        @test gnc_period_s(mpc) == mpc.control_dt_s == mpc.controller.dt_s
        @test isempty(period_messages(rpo))
        mpc.controller.dt_s = 2mpc.control_dt_s
        warned = period_messages(rpo)
        @test length(warned) == 1 && occursin("discretized with a", only(warned))
        # The engine runs the check before every run.
        warned = period_warnings(() -> run_simulation(runtime_args((PeriodicLaw(0.25),))))
        @test length(warned) == 1 && occursin("0.25 s period", only(warned))
    end

    @testset "an unknown schedule is refused" begin
        err = try
            run_simulation(runtime_args((OddControl(),)))
            nothing
        catch e
            e
        end
        @test err isa ArgumentError && occursin("tick_schedule", sprint(showerror, err))
    end
end
end
