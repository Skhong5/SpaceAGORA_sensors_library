module InterlinkLaserPhysicsTests

# The terminal-aware link contract: the six-argument `link_available`,
# `interlink_wrench` and its torques, `link_applies_torque` and `terminal_pose`.

using Test
using LinearAlgebra
using StaticArrays
using SpaceAGORA

const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const IM = SM.InterLinkModels
const PLANET = make_no_gram_planet(:earth)
const RADIUS = PLANET.Rp_e + 700e3
const SPEED = sqrt(PLANET.μ / RADIUS)
const ZERO3 = SVector(0.0, 0.0, 0.0)

bits(x) = reinterpret(UInt64, collect(Float64, x))
sample(pos; index=1) = IM.StateSample(SVector{3, Float64}(pos), SVector(0.0, SPEED, 0.0), 100.0, nothing, nothing,
    nothing, index)

# A 100 kg bus on a circular orbit, `y` metres along track, with `terminals`
# laser terminals at `location` and `n_terminal` virtual terminals.
function bus_sat(index; y=0.0, terminals=0, n_terminal=1, location=ZERO3, ang_vel=ZERO3)
    bus = Link(root=true, m=100.0)
    ic = CartesianInitialCondition(SVector(RADIUS, y, 0.0), SVector(0.0, SPEED, 0.0); q=SVector(0.0, 0.0, 0.0, 1.0),
        ang_vel)
    sc = SpacecraftModel(root=bus, links=[bus], inertia_tensor=bus.inertia, initial_condition=ic, id=index,
        n_terminal=n_terminal)
    for _ in 1:terminals
        add_actuator!(sc, bus, LaserTerminal(boresight=[0.0, 1.0, 0.0], location=location, max_power_w=50.0))
    end
    return sc
end

function run_args(spacecraft, model; policy=SemimajorAxisRatePolicy(), duration=60.0, orientation=false)
    return SimulationConfiguration(
        simulation_settings=SimulationSettings(results=false, generate_plots=false),
        mission_configuration=MissionConfiguration(mission_type=MissionTime, mission_time=duration,
            orientation_sim=orientation, data_rate=1.0),
        environment_model=make_no_gram_environment(planet=PLANET),
        dynamics_model=DynamicsModel(spacecraft, (InverseSquaredGravityModel(),)),
        guidance_model=GuidanceModel((), Float64[]), navigation_model=NavigationModel((), Float64[]),
        control_model=ControlModel((), Float64[]), initial_time=InitialTime(),
        integration_tolerances=IntegrationTolerances(reltol_orbit=1e-10, abstol_orbit=1e-8, dt_max_orbit=1.0),
        solver_config=SM.SolverConfig(solver_mode=:tsit5), interlink_model=model,
        scheduling_policy_model=SchedulingPolicyModel(policy))
end

# A kind written with the four-argument hooks only, as kinds were before the
# terminal-aware forms; and the same force given through `interlink_wrench` only.
struct FourArgumentLink <: AbstractInterlink
    range_m::Float64
end
SpaceAGORA.link_range_m(link::FourArgumentLink) = link.range_m
SpaceAGORA.link_available(link::FourArgumentLink, a, b, ctx) = norm(a.pos_ii - b.pos_ii) < link.range_m / 2
SpaceAGORA.link_force(link::FourArgumentLink, a, b, ctx) = (0.3 * (a.pos_ii - b.pos_ii) / 7.0, 0.3 * (b.pos_ii - a.pos_ii) / 7.0)

struct WrenchOnlyLink <: AbstractInterlink
    range_m::Float64
end
SpaceAGORA.link_range_m(link::WrenchOnlyLink) = link.range_m
SpaceAGORA.interlink_wrench(link::WrenchOnlyLink, terminal_a, terminal_b, a, b, ctx) =
    (0.3 * (a.pos_ii - b.pos_ii) / 7.0, ZERO3, 0.3 * (b.pos_ii - a.pos_ii) / 7.0, ZERO3)

# A kind that records the terminals its six-argument hooks receive and turns its
# two endpoints the opposite ways about body z, with no force.
const RECEIVED = Any[]
struct TurningLink <: AbstractInterlink
    torque_n_m::Float64
    declared::Bool
end
SpaceAGORA.link_range_m(::TurningLink) = 1e9
SpaceAGORA.link_applies_torque(link::TurningLink) = link.declared
SpaceAGORA.terminal_supports(::LaserTerminal, ::TurningLink) = true
SpaceAGORA.link_available(::TurningLink, terminal_a, terminal_b, a, b, ctx) = (push!(RECEIVED, (terminal_a, terminal_b)); true)
SpaceAGORA.interlink_wrench(link::TurningLink, terminal_a, terminal_b, a, b, ctx) =
    (ZERO3, SVector(0.0, 0.0, link.torque_n_m), ZERO3, SVector(0.0, 0.0, -link.torque_n_m))

# Selects every available link.
struct EveryLinkPolicy <: AbstractSchedulingPolicy end
SpaceAGORA.candidate_score(::EveryLinkPolicy, candidate, ctx, t) = 1.0

function turning_pair(link; ang_vel=ZERO3)
    spacecraft = [bus_sat(1; terminals=1, n_terminal=0, ang_vel), bus_sat(2; y=-1000.0, ang_vel)]
    model = InterlinkModel()
    register_link!(model, spacecraft, (1, 1), (2, 1), link)
    return spacecraft, model
end

@testset "Interlink laser physics" begin
    @testset "the terminal-aware forms call the four-argument hooks" begin
        a, b = sample([RADIUS, 0.0, 0.0]), sample([RADIUS, -1500.0, 0.0]; index=2)
        far = sample([RADIUS, -9000.0, 0.0]; index=2)
        ctx = LinkContext(time_s=3.0, planet=PLANET)
        link = FourArgumentLink(10_000.0)
        for other in (b, far)
            @test link_available(link, nothing, nothing, a, other, ctx) === link_available(link, a, other, ctx)
        end
        @test link_available(link, nothing, nothing, a, b, ctx) && !link_available(link, nothing, nothing, a, far, ctx)
        force_a, torque_a, force_b, torque_b = interlink_wrench(link, nothing, nothing, a, b, ctx)
        expected = link_force(link, a, b, ctx)
        @test bits(force_a) == bits(expected[1]) && bits(force_b) == bits(expected[2])
        @test torque_a === ZERO3 && torque_b === ZERO3
        @test !link_applies_torque(link)
        # The built-in kinds, likewise.
        terminal = LaserTerminal(boresight=[0.0, 1.0, 0.0], max_power_w=50.0)
        for builtin in (LaserMomentumLink(power_w=40.0, magnification=100.0, range_m=200e3),
                        PowerBeamLink(power_w=40.0, range_m=200e3),
                        DataLink(transmit_power_w=1.0, budget=FixedRateBudget(1e6), range_m=200e3))
            for ends in ((nothing, nothing), (terminal, nothing))
                wrench = interlink_wrench(builtin, ends..., a, b, ctx)
                forces = link_force(builtin, a, b, ctx)
                @test bits(wrench[1]) == bits(forces[1]) && bits(wrench[3]) == bits(forces[2])
                @test iszero(wrench[2]) && iszero(wrench[4])
                @test link_available(builtin, ends..., a, b, ctx) === link_available(builtin, a, b, ctx)
            end
            @test !link_applies_torque(builtin)
        end
        # The power beam's occultation, through the six-argument form.
        behind = sample([-RADIUS, 0.0, 0.0]; index=2)
        @test !link_available(PowerBeamLink(power_w=40.0, range_m=3e7), nothing, nothing, a, behind, ctx)
    end

    @testset "the engine and the policies read interlink_wrench" begin
        spacecraft = [bus_sat(1), bus_sat(2; y=-1000.0)]
        models = map((FourArgumentLink(10_000.0), WrenchOnlyLink(10_000.0))) do link
            model = InterlinkModel()
            register_link!(model, spacecraft, (2, 1), (1, 1), link)
            model
        end
        args = run_args(spacecraft, first(models))
        state = SE.build_initial_conditions(args)
        mu = PLANET.μ
        scores = map(models) do model
            scheduler = SchedulingPolicyModel(SemimajorAxisRatePolicy())
            update_availability!(model, spacecraft, state)
            score_candidates!(model, scheduler, state, mu; spacecraft)
            copy(scheduler.matching.score)
        end
        @test bits(scores[1]) == bits(scores[2]) && all(>(0), scores[1])
        for model in models
            model.graph.active .= true
            force = interlink_force_on_spacecraft(model, state, 1; spacecraft)
            @test bits(force) == bits(0.3 * (state.sc[1].pos - state.sc[2].pos) / 7.0)
            @test !last(IM.interlink_wrench_on_spacecraft(model, state, 1; spacecraft))
        end
        # A run of each ends on the same bits, with the link selected.
        finals = map(models) do model
            solution = run_simulation(run_args(spacecraft, model); return_solution=true)
            @test any(sample -> !isempty(sample.active), solution.prob.p.args.interlink_model.history)
            solution.u[end]
        end
        @test bits(finals[1]) == bits(finals[2])
    end

    @testset "the hooks receive the terminal hardware" begin
        spacecraft, model = turning_pair(TurningLink(0.0, false))
        terminal = only(mounted_link_terminals(spacecraft[1])).terminal
        # Outside a run, from the spacecraft models; without them, none.
        empty!(RECEIVED)
        state = SE.build_initial_conditions(run_args(spacecraft, model))
        update_availability!(model, spacecraft, state)
        update_availability!(model, nothing, state)
        @test RECEIVED[1][1] === terminal && RECEIVED[1][2] === nothing
        @test RECEIVED[2] === (nothing, nothing)
        # In a run, as its setup recorded them.
        empty!(RECEIVED)
        solution = run_simulation(run_args(spacecraft, model; policy=EveryLinkPolicy(), duration=5.0); return_solution=true)
        @test !isempty(RECEIVED) && all(ends -> ends[1] === terminal && ends[2] === nothing, RECEIVED)
        @test solution.prob.p.args.interlink_model.edge_terminals == [(terminal, nothing)]
    end

    @testset "torques turn the endpoints by I⁻¹ τ" begin
        torque = 2.0e-3
        spacecraft, model = turning_pair(TurningLink(torque, true))
        duration = 60.0
        solution = run_simulation(run_args(spacecraft, model; policy=EveryLinkPolicy(), duration, orientation=true);
            return_solution=true)
        final = solution.u[end]
        # About a principal axis from rest, ω grows as τ t / I.
        for (index, sign) in ((1, 1.0), (2, -1.0))
            inertia = spacecraft[index].inertia_tensor
            @test final.sc[index].ω[3] ≈ sign * torque * duration / inertia[3, 3] rtol = 1e-9
            @test abs(final.sc[index].ω[1]) < 1e-15 && abs(final.sc[index].ω[2]) < 1e-15
        end
        # No force: the orbits are those of a run without the link (to the
        # integration's tolerance, since the attitude states differ).
        unlinked = run_simulation(run_args(spacecraft, InterlinkModel(); policy=EveryLinkPolicy(), duration,
            orientation=true); return_solution=true).u[end]
        @test final.sc[1].pos ≈ unlinked.sc[1].pos rtol = 1e-12
        # A link whose torques are zero leaves a tumbling spacecraft's attitude
        # equation as it is, bit for bit.
        tumble = SVector(0.01, -0.02, 0.03)
        still, still_model = turning_pair(TurningLink(0.0, true); ang_vel=tumble)
        resting = run_simulation(run_args(still, still_model; policy=EveryLinkPolicy(), duration, orientation=true);
            return_solution=true).u[end]
        free_pair, _ = turning_pair(TurningLink(0.0, true); ang_vel=tumble)
        free = run_simulation(run_args(free_pair, InterlinkModel(); policy=EveryLinkPolicy(), duration, orientation=true);
            return_solution=true).u[end]
        @test bits(resting) == bits(free)
        @test resting.sc[1].ω != tumble
    end

    @testset "a torque needs attitude" begin
        # Declared: refused at setup, before any solve.
        spacecraft, model = turning_pair(TurningLink(1e-3, true))
        declared = run_args(spacecraft, model; policy=EveryLinkPolicy(), duration=5.0)
        err = try
            SE._validate_interlinks!(declared, :tsit5)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError && occursin("applies torques to its endpoints (link_applies_torque)", sprint(showerror, err))
        @test_throws ArgumentError run_simulation(declared)
        # Not declared: setup lets it through, and the run stops at its first
        # nonzero torque.
        spacecraft, model = turning_pair(TurningLink(1e-3, false))
        undeclared = run_args(spacecraft, model; policy=EveryLinkPolicy(), duration=5.0)
        @test SE._validate_interlinks!(undeclared, :tsit5) === nothing
        err = try
            run_simulation(undeclared)
            nothing
        catch e
            e
        end
        @test err !== nothing && occursin("applied a torque to spacecraft 1 in a run without attitude", sprint(showerror, err))
        # Not declared, with attitude: applied.
        spacecraft, model = turning_pair(TurningLink(1e-3, false))
        final = run_simulation(run_args(spacecraft, model; policy=EveryLinkPolicy(), duration=5.0, orientation=true);
            return_solution=true).u[end]
        @test final.sc[1].ω[3] > 0
    end

    @testset "terminal_pose" begin
        bus = Link(root=true, m=100.0)
        turn = SVector(0.0, 0.0, sin(pi / 4), cos(pi / 4))   # 90° about z
        boom = Link(root=false, m=5.0, r=MVector(0.0, 1.5, 0.2), q=MVector(turn...))
        sc = SpacecraftModel(root=bus, links=[bus, boom], inertia_tensor=bus.inertia, id=1, n_terminal=0,
            initial_condition=CartesianInitialCondition(SVector(RADIUS, 0.0, 0.0), SVector(0.0, SPEED, 0.0)))
        laser = LaserTerminal(boresight=[1.0, 0.0, 0.0], location=[0.25, 0.0, -0.1], max_power_w=10.0)
        add_actuator!(sc, boom, laser)
        receiver = PowerReceiver(normal=[0.0, 0.0, 1.0], location=[0.0, 0.3, 0.0], radius_m=0.5, efficiency=0.4)
        add_actuator!(sc, bus, receiver)
        ctx = SM.ActuatorMounting.actuator_context(boom)
        boresight, location = terminal_pose(sc, laser)
        @test boresight == ctx.link_to_body * laser.boresight
        @test location == ctx.link_to_body * laser.location + ctx.link_r_body
        @test boresight ≈ SVector(0.0, 1.0, 0.0) atol = 1e-15
        # A force at the terminal has the torque of the terminal's own recoil.
        recoil, recoil_torque = actuator_wrench(laser, 7.0, ctx)
        @test bits(cross(location, recoil)) == bits(recoil_torque)
        # A receiver points along its normal, from the root link.
        @test terminal_pose(sc, receiver) == (receiver.normal, receiver.location)
        # Equal terminals at different poses cannot be told apart; at one pose they can.
        add_actuator!(sc, bus, LaserTerminal(boresight=[1.0, 0.0, 0.0], location=[0.25, 0.0, -0.1], max_power_w=10.0))
        @test_throws ArgumentError terminal_pose(sc, laser)
        @test_throws ArgumentError terminal_pose(sc, LaserTerminal(boresight=[0.0, 0.0, 1.0], max_power_w=1.0))
        twin = bus_sat(2; terminals=2)
        @test terminal_pose(twin, mounted_link_terminals(twin)[1].terminal) == (SVector(0.0, 1.0, 0.0), ZERO3)
    end
end

end # module InterlinkLaserPhysicsTests
