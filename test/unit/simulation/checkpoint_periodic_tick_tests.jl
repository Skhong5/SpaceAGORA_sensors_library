module CheckpointPeriodicTickTests
using Test, StaticArrays, SpaceAGORA
import SpaceAGORA.TelemetryVerification: make_example_config
const SM = SpaceAGORA.SimulationModel
const SC = SM.SimulationCallbacks
const SE = SpaceAGORA.SimulationEngine
const L = SpaceAGORA.SimulationLifecycle

# A mounted controller that commands its wheel 0.001 * t N m at each tick and
# records the tick, so a lost or shifted tick changes the commanded level and
# the wheel momentum. It keeps the held level in checkpoint files, which a
# resumed run needs until its first tick.
mutable struct RampWheel <: AbstractControlEffectorModel
    levels::Vector{Float64}
    ticks::Vector{Float64}
end
RampWheel() = RampWheel([0.0], Float64[])
SpaceAGORA.actuation_mode(::RampWheel) = :mounted
SpaceAGORA.actuator_levels(c::RampWheel, i::Int) = i == 1 ? c.levels : nothing
function SpaceAGORA.update_control!(c::RampWheel, u, p, t::Float64, i::Int64)
    c.levels[1] = 0.001 * t
    push!(c.ticks, t)
    return nothing
end
L.checkpoint_state(c::RampWheel) = (levels=copy(c.levels),)
L.restore_checkpoint_state!(c::RampWheel, data) = (c.levels .= data.levels; nothing)

# Guidance, navigation and control effectors that only record their ticks (and,
# for control, the clock the engine reports), and a navigation effector that
# fails at its first tick after a given time, to interrupt a run.
struct TickRecorder
    ticks::Vector{Float64}
end
TickRecorder() = TickRecorder(Float64[])
SpaceAGORA.update_guidance!(r::TickRecorder, u, p, t::Float64, i::Int) = (push!(r.ticks, t); nothing)
SpaceAGORA.update_navigation!(r::TickRecorder, u, p, t::Float64, i::Int) = (push!(r.ticks, t); nothing)
struct ControlRecorder <: AbstractControlEffectorModel
    ticks::Vector{Float64}
    clock::Vector{Tuple{Float64, Float64}}
end
ControlRecorder() = ControlRecorder(Float64[], Tuple{Float64, Float64}[])
SpaceAGORA.update_control!(r::ControlRecorder, u, p, t::Float64, i::Int64) = (push!(r.ticks, t); nothing)
SpaceAGORA.control_wrench(::ControlRecorder, x, p, t) = (SVector(0.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0))
SpaceAGORA.gnc_clock_started!(r::ControlRecorder, period_s, origin_s) = (push!(r.clock, (period_s, origin_s)); nothing)
struct FailAfter
    t_s::Float64
end
SpaceAGORA.update_navigation!(f::FailAfter, u, p, t::Float64, i::Int) = t > f.t_s ? error("interrupted at $t s") : nothing

# A 100 kg spacecraft far from Earth with one wheel and no forces.
function wheel_config(mission_time)
    root = SM.Link(root=true, m=100.0, dims=MVector(1.0, 1.0, 1.0))
    ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), SVector(0.0, 0.0, 0.0); q=SVector(0.0, 0.0, 0.0, 1.0))
    sc = SM.SpacecraftModel(links=[root], root=root, prop_mass=1.0, inertia_tensor=root.inertia, initial_condition=ic, id=1)
    add_actuator!(sc, root, ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0), max_torque_nm=1.0, max_momentum_nms=10.0))
    return make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=mission_time,
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
        keplerian=false, EI_km=1.0, verbose=false, results=false)
end

# The ramp controller at `period`, a second control effector, guidance and
# navigation at their own periods; `fail_after` adds a navigation effector that
# interrupts the run.
function ramp_setup(; interval=nothing, period, data_rate=period, guidance_period=0.25, navigation_period=0.3,
                    second_control_period=0.3, mission_time=5.0, dir=mktempdir(), resume=false, fail_after=nothing)
    control, second, guidance, navigation = RampWheel(), ControlRecorder(), TickRecorder(), TickRecorder()
    navigation_effectors = fail_after === nothing ? (navigation,) : (navigation, FailAfter(fail_after))
    args = SM.SimConfig._with_configuration(wheel_config(mission_time);
        simulation_settings=SM.SimulationSettings(results=false, verbose=false, generate_plots=false,
            normalize=false, save_csv=false, results_directory=dir,
            checkpoint_enabled=interval !== nothing, checkpoint_interval_s=something(interval, 300.0),
            checkpoint_directory=joinpath(dir, "checkpoints"), resume_from_checkpoint=resume),
        mission_configuration=SM.MissionConfiguration(mission_type=SM.MissionTime, keplerian=false,
            mission_time=mission_time, orientation_sim=true, data_rate=data_rate),
        control_model=SM.ControlModel(control_effectors=(control, second), control_rates=[period, second_control_period]),
        guidance_model=SM.GuidanceModel(guidance_effectors=(guidance,), guidance_rates=[guidance_period]),
        navigation_model=SM.NavigationModel(navigation_effectors=navigation_effectors,
            navigation_rates=fill(navigation_period, length(navigation_effectors))))
    return (args=args, control=control, second=second, guidance=guidance, navigation=navigation)
end

# The final state of a checkpointed run is its last checkpoint.
function ramp_run(; interval=nothing, kw...)
    s = ramp_setup(; interval=interval, kw...)
    out = run_simulation(s.args; return_results=true, return_solution=interval === nothing, isolate_state=false)
    final = interval === nothing ? out.solution.u[end].sc[1] : SE._load_checkpoint(s.args).u.sc[1]
    return (table=out.table, final=final, control=s.control.ticks, second=s.second.ticks, guidance=s.guidance.ticks,
            navigation=s.navigation.ticks, clock=s.second.clock, args=s.args)
end

const TICKS = (:control, :second, :guidance, :navigation)

function same_ticks_and_state(split, unsplit)
    @test all(field -> !isempty(getfield(unsplit, field)), TICKS)
    for field in TICKS
        @test getfield(split, field) == getfield(unsplit, field)
    end
    @test split.final.h_wheels[1] ≈ unsplit.final.h_wheels[1] rtol=1e-12
    @test split.final.ω ≈ unsplit.final.ω rtol=1e-12
end

# The run state of a checkpoint (`nothing` when it has none).
runtime_state(args) = get(SE._load_checkpoint(args), :runtime_state, nothing)

@testset "checkpointed runs tick as runs in one piece" begin
    @testset "segment boundaries on the tick grid keep their ticks" begin
        # Exact binary times: every boundary and saved row lands on a tick.
        unsplit = ramp_run(period=0.125)
        split = ramp_run(period=0.125, interval=1.0)
        @test unsplit.control == 0.125:0.125:4.875
        same_ticks_and_state(split, unsplit)
        @test count(t -> t in (1.0, 2.0, 3.0, 4.0), split.control) == 4
        @test count(t -> t in (1.0, 2.0, 3.0, 4.0), split.guidance) == 4
        @test split.table.time == unsplit.table.time
        @test split.table.sc1_actuator_level_1 == unsplit.table.sc1_actuator_level_1
        # The level saved at a boundary is the one commanded there.
        @test split.table.sc1_actuator_level_1[findfirst(==(1.0), split.table.time)] == 0.001
        # Each segment reports the run's clock origin, as the run's start does.
        @test unsplit.clock == [(0.3, 0.0), (0.3, 0.0)]
        @test split.clock == fill((0.3, 0.0), 1 + 5)
    end

    @testset "the reported 0.1 s control period with 1 s segments" begin
        unsplit = ramp_run(period=0.1)
        split = ramp_run(period=0.1, interval=1.0)
        # 30 * 0.1 is 3.0000000000000004, so that tick falls just after a boundary.
        @test count(t -> t in (1.0, 2.0, 4.0), split.control) == 3
        same_ticks_and_state(split, unsplit)
    end

    @testset "segment boundaries off the tick grid do not shift it" begin
        unsplit = ramp_run(period=0.1)
        split = ramp_run(period=0.1, interval=0.75)
        same_ticks_and_state(split, unsplit)
        # Several rates, some ticking on the 1.5 s boundaries and some not.
        rates = (period=0.125, guidance_period=0.4, navigation_period=0.75, second_control_period=0.2)
        unsplit = ramp_run(; rates...)
        split = ramp_run(; rates..., interval=1.5)
        same_ticks_and_state(split, unsplit)
        @test 1.5 in split.navigation && 3.0 in split.navigation && 1.5 in split.control
        @test !any(t -> t in (1.5, 3.0, 4.5), split.guidance)
    end

    @testset "a run resumed after an interruption ticks as one in one piece" begin
        unsplit = ramp_run(period=0.125)
        dir = mktempdir()
        # Interrupted after its checkpoint at 2 s, which it wrote after that
        # time's ticks: a checkpoint at a segment boundary has no pending ticks.
        interrupted = ramp_setup(period=0.125, interval=1.0, dir=dir, fail_after=2.5)
        err = try
            run_simulation(interrupted.args; isolate_state=false)
            nothing
        catch e
            e
        end
        @test err !== nothing && occursin("interrupted at", sprint(showerror, err))
        @test 2.0 in interrupted.control.ticks && SE._load_checkpoint(interrupted.args).t == 2.0
        @test !hasproperty(runtime_state(interrupted.args), :periodic_ticks)
        resumed = ramp_run(period=0.125, interval=1.0, dir=dir, resume=true, fail_after=Inf)
        @test resumed.control[1] == 2.125
        for field in TICKS
            @test getfield(resumed, field) == filter(>(2.0), getfield(unsplit, field))
        end
        @test all(c -> c[2] == 0.0, resumed.clock)
        @test resumed.final.h_wheels[1] ≈ unsplit.final.h_wheels[1] rtol=1e-12
        @test resumed.table.sc1_actuator_level_1[end] == unsplit.table.sc1_actuator_level_1[end]
    end

    @testset "a run extended from the end of a finished run takes the ticks due there" begin
        unsplit = ramp_run(period=0.125)
        dir, again_dir = mktempdir(), mktempdir()
        first_leg = ramp_run(period=0.125, interval=1.0, dir=dir, mission_time=2.0)
        @test last(first_leg.control) == 1.875                               # no tick at the run's end
        @test runtime_state(first_leg.args).periodic_ticks == (version=1, pending_s=2.0)
        cp(joinpath(dir, "checkpoints"), joinpath(again_dir, "checkpoints"))
        resumed = ramp_run(period=0.125, interval=1.0, dir=dir, resume=true)
        # Only the rates whose grid meets 2 s tick there.
        @test resumed.control[1] == 2.0 && resumed.guidance[1] == 2.0
        @test resumed.second[1] > 2.0 && resumed.navigation[1] > 2.0
        for field in TICKS
            @test vcat(getfield(first_leg, field), getfield(resumed, field)) == getfield(unsplit, field)
        end
        @test resumed.final.h_wheels[1] ≈ unsplit.final.h_wheels[1] rtol=1e-12
        # The row saved at the resume time holds the level commanded there.
        @test resumed.table.time[1] == 2.0 && resumed.table.sc1_actuator_level_1[1] == 0.002
        # Resumed without checkpoints: the same ticks.
        again = ramp_run(period=0.125, dir=again_dir, resume=true)
        @test again.control == resumed.control && again.guidance == resumed.guidance
        @test again.final.h_wheels[1] ≈ unsplit.final.h_wheels[1] rtol=1e-12
        # A pending entry that does not belong to the checkpoint's time is refused.
        bad = merge(SE._load_checkpoint(first_leg.args), (runtime_state=(periodic_ticks=(version=1, pending_s=1.0),),))
        @test_throws ArgumentError SE._pending_periodic_ticks_s(bad)
        @test isnan(SE._pending_periodic_ticks_s((t=2.0, u=nothing)))
    end
end

@testset "periodic GNC callback outside a checkpointed run is PeriodicCallback" begin
    function direct_ticks(make_callback, tspan; origin=NaN, horizon=NaN, pending=NaN)
        p = SM.ODEParams(n_sats=1, args=wheel_config(5.0))
        p.shared_buffers.periodic_clock_origin_s[] = origin
        p.shared_buffers.periodic_clock_horizon_s[] = horizon
        p.shared_buffers.periodic_clock_pending_s[] = pending
        ticks = Float64[]
        cb = make_callback(integrator -> push!(ticks, integrator.t))
        prob = SC.ODEProblem((du, u, p, t) -> fill!(du, 0.0), [1.0], tspan, p)
        sol = SC.solve(prob, SC.Tsit5(); callback=cb)
        return ticks, sol.t
    end
    for tspan in ((0.0, 1.0), (12.345, 13.0), (0.0, 0.95))
        reference = direct_ticks(f -> SC.PeriodicCallback(f, 0.1), tspan)
        @test direct_ticks(f -> SC._run_clock_periodic_callback(f, 0.1), tspan) == reference
        @test !isempty(first(reference)) && last(tspan) ∉ first(reference)
    end
    # A later segment of a checkpointed run: the run's grid, and its tick at
    # the segment's end, which is before the run's end.
    origins = Float64[]
    recorded(f) = SC._run_clock_periodic_callback(f, 0.125; on_clock=(origin, integrator) -> push!(origins, origin))
    ticks, _ = direct_ticks(recorded, (1.0, 2.0); origin=0.0, horizon=5.0)
    @test ticks == 1.125:0.125:2.0
    @test origins == [0.0]
    ticks, _ = direct_ticks(recorded, (4.0, 5.0); origin=0.0, horizon=5.0)
    @test ticks == 4.125:0.125:4.875
    _ = direct_ticks(recorded, (12.345, 13.0))
    @test origins == [0.0, 0.0, 12.345]
    # The ticks pending at the start of a resumed run's first solve: only on the
    # grid, and only at the pending time.
    ticks, _ = direct_ticks(recorded, (2.0, 3.0); origin=0.0, horizon=5.0, pending=2.0)
    @test ticks == 2.0:0.125:3.0
    ticks, _ = direct_ticks(recorded, (2.0, 3.0); origin=0.0, horizon=5.0, pending=1.0)
    @test ticks == 2.125:0.125:3.0
    ticks, _ = direct_ticks(f -> SC._run_clock_periodic_callback(f, 0.3), (2.0, 3.0); origin=0.0, horizon=5.0, pending=2.0)
    @test ticks == [k * 0.3 for k in 7:10]
end

end
