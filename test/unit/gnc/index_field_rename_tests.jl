# The index fields renamed to `spacecraft_index`, `chaser_index` and
# `target_index` in 0.3.0 (0.3.1 for `RobotArmReactionEffector`): the old keywords
# and properties were removed in 0.5.0, and the engine's index-field checks read
# only the new names. The older index names deprecated in 0.5.0 forward, with one
# warning per owner and name, until 1.0.
module IndexFieldRenameTests
using Test, Logging, StaticArrays, SpaceAGORA
const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine

# A contributor model whose index field keeps the older name.
struct ContributorIndexEffector
    spacecraft_idx::Int
end

# Each renamed type, its old and new names, and a value for the field.
const RENAMES = [
    (SM.MagneticMomentumManagerModel, :sat_idx, :spacecraft_index, 3),
    (SM.RobotArmControlEffector, :spacecraft_idx, :spacecraft_index, 3),
    (SM.RobotArmReactionEffector, :spacecraft_idx, :spacecraft_index, 3),
    (SM.RPOMPCControlModel, :chaser_idx, :chaser_index, 3),
    (SM.RPOMPCControlModel, :target_idx, :target_index, 4),
]

@testset "GNC index field names" begin
    @testset "$(nameof(T)) has $(new) and no longer $(old)" for (T, old, new, value) in RENAMES
        @test new in fieldnames(T) && !(old in fieldnames(T))
        m = T(; (new => value,)...)
        @test getfield(m, new) == getproperty(m, new) == value
        # The old keyword and property are gone: no forwarding, no warning.
        @test_throws MethodError T(; (old => value,)...)
        @test !hasproperty(m, old)
        @test_throws FieldError getproperty(m, old)
        @test_throws FieldError setproperty!(m, old, value + 1)
        setproperty!(m, new, value + 1)
        @test getfield(m, new) == value + 1
    end

    @testset "an explicit nothing is refused" begin
        for (T, old, new, value) in RENAMES
            @test_throws MethodError T(; (new => nothing,)...)        # the Int field refuses it
        end
    end

    @testset "the defaults are unchanged" begin
        mm = SM.MagneticMomentumManagerModel()
        @test (mm.spacecraft_index, mm.mu_gain, mm.m_max_am2, mm.h_wheels_0, mm.commanded_torque, mm.b_field_ii) ==
            (1, 3.0e-4, Inf, SVector(0.0, 0.0, 0.0), nothing, nothing)
        @test (mm.h_wheels, mm.held_torque_body, mm.held_dipole_am2, mm.initialized, mm.ticks, mm.actuation) ==
            (SVector(0.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0), false, 0, :ideal)
        @test isnan(mm.last_update_s) && mm.mounted_levels == Float64[]
        @test SM.MagneticMomentumManagerModel().mounted_levels !== mm.mounted_levels   # a fresh buffer each time
        arm = SM.RobotArmControlEffector()
        @test (arm.spacecraft_index, arm.plan, arm.controller, arm.updated_at_s, arm.joint_kp, arm.joint_kd,
            arm.control_dt_s, arm.k_translation_n_m, arm.c_translation_n_s_m, arm.k_rotation_n_m_rad,
            arm.c_rotation_n_m_s_rad) == (1, nothing, nothing, 0.0, 0.0, 0.0, 0.1, 5.0e3, 30.0, 15.0, 0.5)
        @test arm.held.joint_torque_nm == Float64[]
        reaction = SM.RobotArmReactionEffector()
        @test (reaction.spacecraft_index, reaction.plan, reaction.updated_at_s, reaction.force_scale,
            reaction.torque_scale, reaction.k_translation_n_m, reaction.c_translation_n_s_m, reaction.k_rotation_n_m_rad,
            reaction.c_rotation_n_m_s_rad) == (1, nothing, 0.0, 0.0, 0.0, 5.0e3, 30.0, 15.0, 0.5)
        @test isempty(reaction.joint_actuators)
        @test SM.RobotArmReactionEffector().joint_actuators !== reaction.joint_actuators  # a fresh vector each time
        mpc = SM.RPOMPCControlModel()
        @test (mpc.chaser_index, mpc.target_index, mpc.minimum_throttle_fraction, mpc.controller, mpc.control_dt_s,
            mpc.attitude_kp, mpc.rate_kd, mpc.max_rw_torque_nm, mpc.command_log, mpc.actuation) ==
            (1, 2, 0.0, nothing, 1.0, 0.0, 0.0, Inf, nothing, :ideal)
        @test mpc.thrusters == SM.SixAxisThrusterModel() && !mpc.plan_buffer.valid && mpc.mounted_levels == Float64[]
    end

    @testset "the planner binding" begin
        rpo = make_rpo_configuration(planner=DirectRPOPlanner(), mission_time_s=0.2)
        planner = only(rpo.guidance_model.guidance_effectors)
        @test (planner.chaser_index, planner.target_index) == (1, 2)
        fields = Dict(name => getfield(planner, name) for name in fieldnames(typeof(planner)))
        kwargs = Dict(k => v for (k, v) in fields if !(k in (:chaser_index, :target_index)))
        T = typeof(planner)
        built = T(; kwargs..., chaser_index=1, target_index=2)
        @test (built.chaser_index, built.target_index) == (1, 2)
        @test !(:chaser_idx in fieldnames(T)) && !(:target_idx in fieldnames(T))
        # An old keyword is unsupported, and no longer stands in for the new one.
        @test_throws MethodError T(; kwargs..., chaser_index=1, target_index=2, chaser_idx=1)
        @test_throws MethodError T(; kwargs..., chaser_index=1, target_index=2, target_idx=2)
        @test_throws UndefKeywordError T(; kwargs..., chaser_idx=1, target_index=2)
        @test_throws UndefKeywordError T(; kwargs..., target_index=2)
        @test_throws MethodError T(; kwargs..., chaser_index=nothing, target_index=2)
        @test !hasproperty(planner, :chaser_idx)
        @test_throws FieldError planner.chaser_idx
    end

    @testset "the engine reads the new names" begin
        @test SE._effector_couples_satellites(SM.RPOMPCControlModel(chaser_index=1, target_index=2))
        @test !SE._effector_couples_satellites(SM.RPOMPCControlModel(chaser_index=2, target_index=2))
        @test SE._control_effector_targets_owned(SM.MagneticMomentumManagerModel(spacecraft_index=2), [2])
        @test !SE._control_effector_targets_owned(SM.MagneticMomentumManagerModel(spacecraft_index=2), [1])
        @test !SE._control_effector_targets_owned(SM.RPOMPCControlModel(chaser_index=1, target_index=2), [2])
        @test SE._effector_spacecraft_index(SM.RobotArmControlEffector(spacecraft_index=2)) == 2
        reaction = SM.RobotArmReactionEffector(spacecraft_index=2)
        @test SE._effector_spacecraft_index(reaction) == 2
        @test SE._control_effector_targets_owned(reaction, [2]) && !SE._control_effector_targets_owned(reaction, [1])
        # A contributor model with only the older field name is not read: it is bound
        # to no spacecraft, so the external-propagation check counts it as acting on
        # every spacecraft.
        legacy = ContributorIndexEffector(2)
        @test SE._effector_spacecraft_index(legacy) === nothing
        @test SE._control_effector_targets_owned(legacy, [1]) && SE._control_effector_targets_owned(legacy, [2])
        @test SE._effector_spacecraft_index(SM.BaseThrusterModel(thrust=[0.0], direction=[0.0], Δv=[0.0],
            start_burn_time=[0.0], stop_burn_time=[0.0], Isp=[300.0])) === nothing
    end
end

# Deprecation messages that `f` logs, or the one it throws under --depwarn=error.
function rename_warnings(f)
    is_rename(msg) = occursin("is deprecated and will be removed in SpaceAGORA 1.0", msg)
    if Base.JLOptions().depwarn == 2
        try
            f()
            return String[]
        catch e
            msg = sprint(showerror, e)
            is_rename(msg) || rethrow()
            return [msg]
        end
    end
    logger = Test.TestLogger(min_level=Logging.Warn)
    with_logger(f, logger)
    @test all(l -> l.group === :depwarn, logger.logs)
    return filter(is_rename, [string(l.message) for l in logger.logs])
end
quietly(f) = Base.JLOptions().depwarn == 2 ? nothing : with_logger(f, Test.TestLogger())

const PULSE = SM.SimulationCallbacks.get_periapsis_pulse_callback
const ANCHOR_STATE = [7.0e6, 0.0, 0.0, 0.0, 7500.0, 0.0]

@testset "Older index names deprecated in 0.5.0" begin
    @testset "SchedulingPolicyModel target_index and target_indices" begin
        T = SchedulingPolicyModel
        @test :target_index in fieldnames(T) && :target_indices in fieldnames(T)
        @test !(:target_idx in fieldnames(T)) && !(:target_idxs in fieldnames(T))
        default = T(:gve_sma)
        @test default.target_index == 1 && default.target_indices === nothing
        @test isempty(rename_warnings(() -> (T(:gve_sma; target_index=2), T(:gve_sma; target_indices=[2, 1]))))
        @test T(:gve_sma; target_index=nothing).target_index === nothing
        @test T(:gve_sma; target_indices=[3, 1]).target_indices == [1, 3]
        for (old, new, value, expected) in ((:target_idx, :target_index, 2, 2), (:target_idxs, :target_indices, [2, 1], [1, 2]))
            warned = rename_warnings(() -> T(:gve_sma; (old => value,)...))
            @test length(warned) == 1 && occursin("SchedulingPolicyModel.$(old)", only(warned)) &&
                occursin("use $(new)", only(warned))
            @test_throws ArgumentError T(:gve_sma; (old => value, new => value)...)
            if Base.JLOptions().depwarn != 2
                logger = Test.TestLogger(min_level=Logging.Warn)
                built = with_logger(() -> (T(:gve_sma; (old => value,)...), T(:gve_sma; (old => value,)...)), logger)
                @test all(b -> getfield(b, new) == expected, built)
                @test length(logger.logs) == 1                     # once per type and name
                # The old property reads the new field.
                @test quietly(() -> getproperty(built[1], old)) == expected
            else
                @test_throws ErrorException getproperty(T(:gve_sma; (new => value,)...), old)
            end
        end
        # An explicit nothing through the old keyword still means "both endpoints".
        Base.JLOptions().depwarn != 2 && @test quietly(() -> T(:gve_sma; target_idx=nothing)).target_index === nothing
        @test_throws TypeError T(:gve_sma; target_index="2")
        @test_throws TypeError T(:gve_sma; target_indices=1)
        # An interlink checkpoint keeps its stored keys, so older checkpoints resume.
        stub = (interlink_model=(graph=(vertices=Any[], edges=Any[], edge_properties=Any[]),
                forbidden_pairs=Set{Any}(), constellation_link_type=:laser),
            scheduling_policy_model=T(:gve_sma; target_indices=[2, 1]), dynamics_model=(spacecraft=Any[],))
        snapshot = SE._interlink_checkpoint_configuration(stub)
        @test snapshot.target_idx == 1 && snapshot.target_idxs == [1, 2]
    end

    @testset "StateAnchor spacecraft_index" begin
        a = StateAnchor(10.0, 2, ANCHOR_STATE)
        @test a.spacecraft_index == 2 && :spacecraft_index in fieldnames(StateAnchor)
        @test !(:sat_idx in fieldnames(StateAnchor))
        @test_throws ArgumentError StateAnchor(10.0, 0, ANCHOR_STATE)
        warned = rename_warnings(() -> a.sat_idx)
        @test length(warned) == 1 && occursin("StateAnchor.sat_idx", only(warned)) && occursin("use spacecraft_index", only(warned))
        Base.JLOptions().depwarn != 2 && @test quietly(() -> a.sat_idx) == 2
    end

    @testset "get_periapsis_pulse_callback spacecraft_index" begin
        @test isempty(rename_warnings(() -> PULSE(0.04, 1:3; spacecraft_index=2)))
        @test getfield(PULSE(0.04, 1:3; spacecraft_index=2).condition, :sat) == 2
        @test getfield(PULSE(0.04, 1:3).condition, :sat) == 1
        warned = rename_warnings(() -> PULSE(0.04, 1:3; sat_idx=2))
        @test length(warned) == 1 && occursin("The sat_idx keyword of get_periapsis_pulse_callback", only(warned)) &&
            occursin("use spacecraft_index", only(warned))
        Base.JLOptions().depwarn != 2 && @test getfield(quietly(() -> PULSE(0.04, 1:3; sat_idx=2)).condition, :sat) == 2
        @test_throws ArgumentError PULSE(0.04, 1:3; sat_idx=2, spacecraft_index=2)
        @test_throws TypeError PULSE(0.04, 1:3; spacecraft_index=nothing)
    end

    @testset "under --depwarn=error the older names are refused, the new ones are not" begin
        script = """
            using SpaceAGORA
            const SM = SpaceAGORA.SimulationModel
            const PULSE = SM.SimulationCallbacks.get_periapsis_pulse_callback
            @assert Base.JLOptions().depwarn == 2
            outcome(f) = try f(); :ok catch e
                e isa ErrorException && occursin("deprecated and will be removed in SpaceAGORA 1.0", e.msg) ? :refused :
                    (showerror(stderr, e); :other) end
            anchor = StateAnchor(10.0, 2, [7.0e6, 0.0, 0.0, 0.0, 7500.0, 0.0])
            policy = SchedulingPolicyModel(:gve_sma; target_index=2)
            println("STRICT ", join(map(outcome, (
                () -> SchedulingPolicyModel(:gve_sma; target_idx=2),
                () -> SchedulingPolicyModel(:gve_sma; target_idxs=[2]),
                () -> policy.target_idx,
                () -> anchor.sat_idx,
                () -> PULSE(0.04, 1:3; sat_idx=2),
                () -> SchedulingPolicyModel(:gve_sma; target_index=2, target_indices=[2]),
                () -> anchor.spacecraft_index,
                () -> PULSE(0.04, 1:3; spacecraft_index=2))), ","))
            """
        output = read(pipeline(`$(Base.julia_cmd()) --startup-file=no --depwarn=error
            --project=$(Base.active_project()) -e $script`; stderr=devnull), String)
        @test occursin("STRICT refused,refused,refused,refused,refused,ok,ok,ok", output)
    end
end
end
