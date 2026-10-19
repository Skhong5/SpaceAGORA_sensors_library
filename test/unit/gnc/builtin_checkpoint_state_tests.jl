# The built-in guidance and control models' run state in checkpoint files
# (checkpoint_state / restore_checkpoint_state!): a run resumed from a checkpoint
# ends as an uninterrupted run with the same checkpoints, bit for bit, in
# configurations where each model's state changes (the kit's
# `checkpoint_round_trip`, on runs it cannot make with its own fixture); the RPO
# planner binding resumes without planning again and refuses a planner whose own
# state a checkpoint cannot hold; the RPO MPC solver continues from its saved
# iterate and step size exactly.
module BuiltinCheckpointStateTests

using Test, Random, Serialization, StaticArrays, LinearAlgebra, SpaceAGORA
import SpaceAGORA.TelemetryVerification: make_example_config
const SM = SpaceAGORA.SimulationModel
const C = SpaceAGORA.Conformance
const L = SpaceAGORA.SimulationLifecycle

round_trip(model, kind, configuration) = C._gnc_checkpoint_round_trip(model, kind, configuration)
# The RPO MPC controller's run state but its solver workspace and last result, which
# a copy rebuilds (and whose result holds the solve's wall-clock times): the held
# command with its write record, the warm start, the mounted levels and the command
# log, which records every solve's command and status.
mpc_state(c) = (held=c.held, run_written=c.run_written, U_prev=c.controller.U_prev, mounted_levels=c.mounted_levels,
    command_log=c.command_log)
function mpc_round_trip(configuration)
    run = (cfg, factories) -> C._model_solve(cfg, :control_model, 1; factories)
    difference = function (a, b)
        gap = C._run_difference(a, b)
        gap === nothing || return gap
        return C._same_state(mpc_state(a.solution.model), mpc_state(b.solution.model)) ? nothing :
            "the controller's held command, warm start, mounted levels or command log differ"
    end
    return C._checkpoint_round_trip_check(run, difference, only(configuration.control_model.control_effectors), configuration)
end
# The data as a resumed run reads it from the file.
from_file(data) = (io = IOBuffer(); serialize(io, data); seekstart(io); deserialize(io))
const Z3 = SVector(0.0, 0.0, 0.0)

# A two-spacecraft RPO run through the planner binding and the MPC controller,
# with a replan after the halfway checkpoint.
rpo_configuration(planner=DirectRPOPlanner(segment_clearance_at=SpaceAGORA.RPOPlannerLifecycle.station_segment_clearance);
                  kwargs...) =
    make_rpo_configuration(; planner=planner, mission_time_s=4.0, planning_events=(RPOPlanningEvent(3.0; reason=:replan),),
        kwargs...)

# User planners holding the chaser at its goal: one that counts its proposals in
# its own state, with or without declaring restart, and one that fails an initial
# request made after the run's start (a resumed run must not plan again).
function hold_result(r::SpaceAGORA.RPOPlanningRequest, diagnostics)
    reference = SpaceAGORA.RPOReference(t_ref_s=[0.0, r.reference_dt_s],
        r_ref_rtn_m=hcat(collect(r.x_rtn[1:3]), collect(r.goal_rtn_m)), v_ref_rtn_mps=zeros(3, 2),
        origin_time_s=r.time_s, valid_until_s=r.valid_until_s, chaser_id=r.chaser_id, target_id=r.target_id,
        geometry_revision=r.geometry_revision)
    return SpaceAGORA.RPOPlanningResult(request_id=r.request_id, status=:candidate, termination=:completed,
        reference=reference, diagnostics=diagnostics)
end
struct CountingPlanner{R} <: SpaceAGORA.AbstractRPOPlanner end
SpaceAGORA.planner_capabilities(::CountingPlanner{R}) where {R} =
    SpaceAGORA.RPOPlannerCapabilities(state_sources=(:truth,), frames=(:target_rtn,), restart=R)
SpaceAGORA.initialize_planner(::CountingPlanner, context) = Ref(0)
SpaceAGORA.plan_rpo!(state, ::CountingPlanner, r::SpaceAGORA.RPOPlanningRequest, rng::Random.AbstractRNG) =
    (state[] += 1; hold_result(r, (calls=state[], draw=rand(rng))))
struct StartOnlyPlanner <: SpaceAGORA.AbstractRPOPlanner end
SpaceAGORA.planner_capabilities(::StartOnlyPlanner) =
    SpaceAGORA.RPOPlannerCapabilities(state_sources=(:truth,), frames=(:target_rtn,))
function SpaceAGORA.plan_rpo!(::Nothing, ::StartOnlyPlanner, r::SpaceAGORA.RPOPlanningRequest, rng::Random.AbstractRNG)
    r.reason === :initial && r.time_s > 0 && error("an initial request after the run's start")
    return hold_result(r, (draw=rand(rng),))
end

# Guidance models for the round trip's other outcomes: one that refuses checkpointed
# runs at setup, one whose run state is not in a checkpoint, and one that returns
# run state it cannot restore.
mutable struct RefusesCheckpoints <: SpaceAGORA.AbstractGuidanceModel
    ticks::Int
end
SpaceAGORA.update_guidance!(m::RefusesCheckpoints, u, p, t::Float64, i::Int) = (m.ticks += 1; nothing)
L.initialize_guidance!(m::RefusesCheckpoints, u, p, t) = (m.ticks = 0; nothing)
function L.preflight_guidance(m::RefusesCheckpoints, args; isolate_state)
    args.simulation_settings.checkpoint_enabled && throw(ArgumentError("RefusesCheckpoints does not run with checkpoints."))
    return nothing
end
mutable struct NoCheckpoint <: SpaceAGORA.AbstractGuidanceModel
    ticks::Int
end
SpaceAGORA.update_guidance!(m::NoCheckpoint, u, p, t::Float64, i::Int) = (m.ticks += 1; nothing)
L.initialize_guidance!(m::NoCheckpoint, u, p, t) = (m.ticks = 0; nothing)
mutable struct CannotRestore <: SpaceAGORA.AbstractGuidanceModel
    ticks::Int
end
SpaceAGORA.update_guidance!(m::CannotRestore, u, p, t::Float64, i::Int) = (m.ticks += 1; nothing)
L.initialize_guidance!(m::CannotRestore, u, p, t) = (m.ticks = 0; nothing)
L.checkpoint_state(m::CannotRestore) = (ticks=m.ticks,)

function with_settings(configuration; kwargs...)
    st = configuration.simulation_settings
    fields = NamedTuple{fieldnames(typeof(st))}(map(n -> getfield(st, n), fieldnames(typeof(st))))
    return SM.SimConfig._with_configuration(configuration; simulation_settings=SM.SimulationSettings(; merge(fields, (; kwargs...))...))
end

# The ideal-actuation lunar descent of the Apollo unit tests, for 20 s.
function apollo_configuration()
    planet = SM.Moon()
    ephem = SM.SimpleEphemeridesModel()
    initial_time = SM.InitialTime(year=1969, month=7, day=20, hour=20, minute=10, second=0.0)
    lat, lon, h0 = 0.674, 23.473, 60.0
    et = SM.ephemerides_epoch_seconds(ephem, initial_time)
    l_pi = SM.ephemerides_frame_lpi(ephem, planet, et)
    upv = SVector(cosd(lat) * cosd(lon), cosd(lat) * sind(lon), sind(lat))
    r_p = (planet.Rp_e + h0) * upv
    r_i = SVector{3, Float64}(l_pi' * r_p)
    v_i = SVector{3, Float64}(l_pi' * cross(SVector{3, Float64}(planet.ω), r_p))
    up_i = normalize(r_i)
    tilt = SVector(0.0, sin(deg2rad(8.0) / 2), 0.0)
    fwd_i = normalize(SVector{3, Float64}(l_pi' * SVector(sind(lon), -cosd(lon), 0.0)))
    q0 = SM.quat_mult(descent_attitude_command(up_i, up_i, fwd_i), SVector(tilt[1], tilt[2], tilt[3], cos(deg2rad(8.0) / 2)))
    q0 = q0 / norm(q0)
    ic = SM.CartesianInitialCondition(r_i, v_i; q=SVector{4, Float64}(q0))
    root = SM.Link(root=true, m=6_900.0, dims=MVector{3, Float64}(4.2, 4.2, 4.0), ref_area=1.0, q=MVector{4, Float64}(q0...))
    sc = SM.SpacecraftModel(links=[root], root=root, prop_mass=1_000.0,
        inertia_tensor=SMatrix{3, 3, Float64}(2.2e4, 0, 0, 0, 2.4e4, 0, 0, 0, 2.0e4), initial_condition=ic, id=97)
    ph = DescentPhaseTargets(name=:braking, r_T=Z3, v_T=Z3, a_T=Z3, t_go_initial_s=10.0, t_go_min_s=1.0, altitude_switch_m=Inf)
    gcfg = ApolloDescentConfig(reference_radius_m=planet.Rp_e, site_lat_deg=lat, site_lon_deg=lon, site_height_m=0.0,
        approach_azimuth_deg=270.0, braking=ph, approach=ph, ignition_trim_s=0.0, engine_cutoff_altitude_m=0.0)
    state = ApolloDescentState(1)
    guidance = ApolloDescentGuidanceModel(gcfg, state, NoTerrainModel())
    control = ApolloDescentControlModel(ApolloDescentControlConfig(touchdown_height_m=0.5), gcfg, state, NoTerrainModel())
    base = make_example_config(planet=planet, spacecraft=sc, mission_time=20.0, initial_time=initial_time,
        dynamic_effectors=(SM.InverseSquaredGravityModel(),), density_model=SM.NoAtmosphereModel(), ephemerides_model=ephem,
        orientation_sim=true, keplerian=false, EI_km=1.0, verbose=false, results=false)
    args = SM.SimConfig._with_configuration(base;
        guidance_model=SM.GuidanceModel(guidance_effectors=(guidance,), guidance_rates=[1.0]),
        control_model=SM.ControlModel(control_effectors=(control,), control_rates=[0.05]),
        integration_tolerances=SM.IntegrationTolerances(reltol_orbit=1e-9, abstol_orbit=1e-9,
            dt_max_orbit=0.25, reltol_atmosphere=1e-9, abstol_atmosphere=1e-9, dt_max_atmosphere=0.25),
        solver_config=SM.SolverConfig(solver_mode=:tsit5))
    return args, guidance, control
end

@testset "Built-in models' run state survives a checkpoint" begin
    @testset "a resumed run ends as an uninterrupted one" begin
        # The momentum manager with its command and field callables: it accumulates
        # wheel momentum and holds a rod torque that turns the spacecraft.
        manager = SM.ControlHooks.MagneticMomentumManagerModel(h_wheels_0=SVector(0.05, -0.02, 0.01),
            commanded_torque=(t, r, v, q, w) -> SVector(1.0e-5, -2.0e-5, 5.0e-6), b_field_ii=(t, r) -> SVector(2.0e-5, 1.0e-5, -3.0e-5))
        cases = Any[(:control_model, manager, C.conformance_fixture(control=(manager,)).configuration)]
        # A burn that is on at the halfway checkpoint.
        burn = SM.ThrusterModels.BaseThrusterModel(thrust=[0.5], direction=[0.0], Δv=[0.1], start_burn_time=[25.0],
            stop_burn_time=[40.0], Isp=[300.0])
        push!(cases, (:control_model, burn, C.conformance_fixture(control=(burn,)).configuration))
        # The RPO planner, which replans after the checkpoint (its controller below).
        rpo = rpo_configuration()
        push!(cases, (:guidance_model, only(rpo.guidance_model.guidance_effectors), rpo))
        # The Apollo descent guidance and control.
        apollo, guidance, control = apollo_configuration()
        push!(cases, (:guidance_model, guidance, apollo), (:control_model, control, apollo))
        for (kind, model, configuration) in cases
            check = round_trip(model, kind, configuration)
            @testset let model = nameof(typeof(model)), status = check.status, detail = check.detail
                @test status === :pass
            end
        end
        check = mpc_round_trip(rpo)
        @testset let model = :RPOMPCControlModel, status = check.status, detail = check.detail
            @test status === :pass
        end
        # A run's random stream compares by its state.
        @test C._same_state(MersenneTwister(3), from_file(MersenneTwister(3)))
        @test !C._same_state(MersenneTwister(3), MersenneTwister(4))
    end

    @testset "the round trip skips a refusal and fails a model whose state does not survive" begin
        # Added in 0.4.0: advisory in 0.4, required from 0.5.0.
        @test C.CHECKPOINT_ROUND_TRIP_SINCE == v"0.4.0"
        failing = C._failure_status(v"0.4.0")
        @test C._failure_status(v"0.4.0", v"0.4.3") === :advisory_fail && C._failure_status(v"0.4.0", v"0.5.0") === :fail
        refusing = RefusesCheckpoints(0)
        check = round_trip(refusing, :guidance_model, C.conformance_fixture(guidance=(refusing,)).configuration)
        @test check.status === :skip && occursin("refuses to run with checkpoints", check.detail)
        forgetful = NoCheckpoint(0)
        check = round_trip(forgetful, :guidance_model, C.conformance_fixture(guidance=(forgetful,)).configuration)
        @test check.status === failing && check.since == v"0.4.0" &&
            occursin("and an uninterrupted run differ after it: the model's own state", check.detail)
        unrestorable = CannotRestore(0)
        check = round_trip(unrestorable, :guidance_model, C.conformance_fixture(guidance=(unrestorable,)).configuration)
        @test check.status === failing && occursin("has no restore_checkpoint_state! method", check.detail)
    end

    @testset "the RPO planner binding" begin
        # A resumed run takes the binding's state from the file: its initializer makes
        # no initial request when the engine says a restore follows.
        args = rpo_configuration()
        g = deepcopy(only(args.guidance_model.guidance_effectors))
        L.initialize_guidance!(g, nothing, (args=args, shared_buffers=(model_resume=Ref(true),)), 0.0)
        @test g.runtime.request_id == 0 && isempty(g.runtime.records) && g.runtime.product === nothing
        @test g.chaser_index == 1 && g.target_index == 2 && !g.plan_buffer.valid
        # In a run: the resumed run's initializer would fail an initial request.
        start_only = rpo_configuration(StartOnlyPlanner(); goal_rtn_m=(3.0, 0.0, 0.0))
        check = round_trip(only(start_only.guidance_model.guidance_effectors), :guidance_model, start_only)
        @test check.status === :pass
        # A planner with state of its own runs with checkpoints when it declares
        # restart, and its state then resumes with the run.
        restartable = rpo_configuration(CountingPlanner{true}(); goal_rtn_m=(3.0, 0.0, 0.0))
        check = round_trip(only(restartable.guidance_model.guidance_effectors), :guidance_model, restartable)
        @test check.status === :pass
        refused = with_settings(rpo_configuration(CountingPlanner{false}(); goal_rtn_m=(3.0, 0.0, 0.0));
            checkpoint_enabled=true, checkpoint_directory=mktempdir())
        err = try
            run_simulation(refused)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError && occursin("does not declare restart", sprint(showerror, err))
        # Without checkpoints it runs.
        @test string(run_simulation(rpo_configuration(CountingPlanner{false}(); goal_rtn_m=(3.0, 0.0, 0.0));
            return_solution=true).retcode) == "Success"
    end

    @testset "the RPO MPC solver continues from its saved iterate and step size" begin
        controller = only(rpo_configuration().control_model.control_effectors).controller
        reference = zeros(6, controller.horizon + 1)
        reference[1, :] .= 5.0
        state(k) = SVector(3.0 + 0.1k, 0.01k, 0.0, 0.002, 0.0, 0.0)
        for k in 1:30
            SM.ControlHooks.rpo_lqmpc_control(controller, state(k), reference)
        end
        saved = from_file((U_prev=copy(controller.U_prev),
            osqp=SM.ControlHooks._rpo_lqmpc_solver_state(controller.qp_model)))
        fresh = deepcopy(controller)                     # a fresh workspace: rho and the iterate start over
        copyto!(fresh.U_prev, saved.U_prev)
        unrestored = deepcopy(fresh)
        SM.ControlHooks._restore_rpo_lqmpc_solver_state!(fresh.qp_model, saved.osqp)
        @test SM.ControlHooks._rpo_lqmpc_solver_state(fresh.qp_model) == saved.osqp
        commands(c) = [SM.ControlHooks.rpo_lqmpc_control(c, state(k), reference) for k in 31:40]
        expected = commands(controller)
        @test commands(fresh) == expected                 # bit for bit
        @test commands(unrestored) != expected            # the fresh workspace's iterate and step size differ
        @test_throws ArgumentError SM.ControlHooks._restore_rpo_lqmpc_solver_state!(fresh.qp_model, (y=zeros(2), rho=0.1))
    end

    @testset "each built-in model's data restores into a fresh copy" begin
        # The deprecated RPO guidance and the robot-arm controller: state a run
        # wrote, saved, read back and put into a fresh copy after its reset.
        legacy = SM.GuidanceModels.RPOGuidanceModel()
        legacy.replan_count, legacy.replanning_phase = 3, :holding
        push!(legacy.replanning_events, (t_s=1.0, reason=:replan))
        legacy.plan_buffer.valid, legacy.plan_buffer.updated_at_s = true, 2.0
        SM.GuidanceHooks._note_rpo_guidance_writes!(legacy)
        fresh = SM.GuidanceModels.RPOGuidanceModel()
        L.restore_checkpoint_state!(fresh, from_file(L.checkpoint_state(legacy)))
        @test C._same_state(fresh, legacy)
        arm = SM.default_robot_arm_model()
        model = SM.RobotArmControlEffector(controller=SM.init_robot_arm_joint_mpc(arm; dt_s=0.1, horizon=4))
        model.held = SM.RobotArmHeldActuation(joint_torque_nm=[0.5, -0.25, 0.0])
        model.run_written = deepcopy(model.held)
        model.controller.U_prev .= 1.5
        restored_arm = SM.RobotArmControlEffector(controller=SM.init_robot_arm_joint_mpc(arm; dt_s=0.1, horizon=4))
        L.restore_checkpoint_state!(restored_arm, from_file(L.checkpoint_state(model)))
        @test C._same_state(restored_arm, model)
        # A checkpoint from another controller is refused.
        plain = SM.RobotArmControlEffector()
        @test_throws ArgumentError L.restore_checkpoint_state!(plain, from_file(L.checkpoint_state(model)))
        # The periapsis-raise guidance and the burn scheduler.
        raise = SpaceAGORA.ApoapsisTargetPeriapsisRaiseGuidanceModel(target_apoapsis_radius_m=7.0e6, command_state=[2, 0])
        other = SpaceAGORA.ApoapsisTargetPeriapsisRaiseGuidanceModel(target_apoapsis_radius_m=7.0e6)
        L.restore_checkpoint_state!(other, from_file(L.checkpoint_state(raise)))
        @test other.command_state == [2, 0]
        two(Δv) = SM.ThrusterModels.BaseThrusterModel(thrust=[50.0, 50.0], direction=[0.0, 0.0], Δv=Δv,
            start_burn_time=[5.0, 6.0], stop_burn_time=[20.0, 21.0], Isp=[300.0, 300.0])
        scheduler = two([0.1, 0.2])
        scheduler.start_burn_time[2], scheduler.burning[2] = 7.5, true
        SM.ControlHooks._note_schedule!(scheduler, 2)
        restored = two([0.3, 0.4])
        L.restore_checkpoint_state!(restored, from_file(L.checkpoint_state(scheduler)))
        @test restored.Δv == [0.1, 0.2] && restored.start_burn_time == [5.0, 7.5] && restored.burning == [false, true]
        @test restored.written_schedule == scheduler.written_schedule
        @test_throws ArgumentError L.restore_checkpoint_state!(SM.ThrusterModels.BaseThrusterModel(thrust=[50.0],
            direction=[0.0], Δv=[0.1], start_burn_time=[5.0], stop_burn_time=[20.0], Isp=[300.0]), from_file(L.checkpoint_state(scheduler)))
    end
end

end
