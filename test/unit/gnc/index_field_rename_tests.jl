# The index fields renamed to `spacecraft_index`, `chaser_index` and
# `target_index` in 0.3.0 (0.3.1 for `RobotArmReactionEffector`): the old keywords
# and properties were removed in 0.5.0, and the engine's index-field checks read
# the new names.
module IndexFieldRenameTests
using Test, StaticArrays, SpaceAGORA
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
        # A contributor model that still has the older field is still read.
        legacy = ContributorIndexEffector(2)
        @test SE._effector_spacecraft_index(legacy) == 2
        @test SE._control_effector_targets_owned(legacy, [2]) && !SE._control_effector_targets_owned(legacy, [1])
        @test SE._effector_spacecraft_index(SM.BaseThrusterModel(thrust=[0.0], direction=[0.0], Δv=[0.0],
            start_burn_time=[0.0], stop_burn_time=[0.0], Isp=[300.0])) === nothing
    end
end
end
