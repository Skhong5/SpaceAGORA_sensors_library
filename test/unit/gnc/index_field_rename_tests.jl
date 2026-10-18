# The index fields renamed to `spacecraft_index`, `chaser_index` and
# `target_index`: the old keywords and properties forward with one deprecation
# warning per type and name, and the engine's index-field checks read the new
# names.
module IndexFieldRenameTests
using Test, Logging, StaticArrays, SpaceAGORA
const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine

# Deprecation messages that `f` logs, or the one it throws under --depwarn=error.
function rename_warnings(f)
    is_rename(msg) = occursin("is deprecated and will be removed in SpaceAGORA 0.5.0", msg)
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

@testset "GNC index field renames" begin
    @testset "$(nameof(T)).$(old) forwards to $(new)" for (T, old, new, value) in RENAMES
        @test new in fieldnames(T) && !(old in fieldnames(T))
        @test !hasproperty(T(), old)                          # engine checks never see the old name
        m = T(; (new => value,)...)
        @test getfield(m, new) == value
        @test isempty(rename_warnings(() -> T(; (new => value,)...)))
        # The old keyword: one warning, the same value.
        warned = rename_warnings(() -> T(; (old => value,)...))
        @test length(warned) == 1 && occursin("$(nameof(T)).$(old)", only(warned)) && occursin("use $(new)", only(warned))
        if Base.JLOptions().depwarn != 2
            logger = Test.TestLogger(min_level=Logging.Warn)
            built = with_logger(() -> (T(; (old => value,)...), T(; (old => value,)...)), logger)
            @test all(b -> getfield(b, new) == value, built)
            @test length(logger.logs) == 1                     # once per type and name
            # The old property reads and writes the new field.
            logger = Test.TestLogger(min_level=Logging.Warn)
            with_logger(logger) do
                @test getproperty(m, old) == value
                setproperty!(m, old, value + 1)
            end
            @test getfield(m, new) == value + 1
            @test length(logger.logs) == 1 && logger.logs[1].group === :depwarn
        else
            # Under --depwarn=error the old property refuses, for reads and writes.
            @test_throws ErrorException getproperty(m, old)
            @test_throws ErrorException setproperty!(m, old, value + 1)
            @test getfield(m, new) == value
        end
        @test_throws ArgumentError T(; (old => value, new => value)...)
    end

    @testset "an explicit nothing is refused, as before the rename" begin
        for (T, old, new, value) in RENAMES
            @test_throws MethodError T(; (new => nothing,)...)        # the Int field refuses it
            if Base.JLOptions().depwarn == 2
                @test_throws ErrorException T(; (old => nothing,)...)  # the old name's warning refuses first
            else
                with_logger(Test.TestLogger()) do
                    @test_throws MethodError T(; (old => nothing,)...)
                end
            end
            # Both names are refused whichever of them is nothing.
            @test_throws ArgumentError T(; (old => value, new => nothing)...)
            @test_throws ArgumentError T(; (old => nothing, new => value)...)
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
        @test isempty(rename_warnings(() -> make_rpo_configuration(planner=DirectRPOPlanner(), mission_time_s=0.2)))
        fields = Dict(name => getfield(planner, name) for name in fieldnames(typeof(planner)))
        kwargs = Dict(k => v for (k, v) in fields if !(k in (:chaser_index, :target_index)))
        T = typeof(planner)
        @test length(rename_warnings(() -> T(; kwargs..., chaser_idx=1, target_index=2))) == 1
        if Base.JLOptions().depwarn != 2
            old = with_logger(() -> T(; kwargs..., chaser_idx=1, target_idx=2), Test.TestLogger())
            @test (old.chaser_index, old.target_index) == (1, 2)
        end
        @test_throws UndefKeywordError T(; kwargs..., target_index=2)
        @test_throws MethodError T(; kwargs..., chaser_index=nothing, target_index=2)
        @test_throws ArgumentError T(; kwargs..., chaser_idx=1, chaser_index=nothing, target_index=2)
        @test_throws ArgumentError T(; kwargs..., chaser_idx=nothing, chaser_index=1, target_index=2)
        if Base.JLOptions().depwarn != 2
            logger = Test.TestLogger(min_level=Logging.Warn)
            with_logger(() -> (@test planner.chaser_idx == 1), logger)
            @test length(logger.logs) == 1
        end
    end

    @testset "the engine reads the new names" begin
        @test SE._effector_couples_satellites(SM.RPOMPCControlModel(chaser_index=1, target_index=2))
        @test !SE._effector_couples_satellites(SM.RPOMPCControlModel(chaser_index=2, target_index=2))
        @test SE._effector_couples_satellites(SM.RPOGuidanceModel(chaser_idx=1, target_idx=2))   # not renamed
        @test SE._control_effector_targets_owned(SM.MagneticMomentumManagerModel(spacecraft_index=2), [2])
        @test !SE._control_effector_targets_owned(SM.MagneticMomentumManagerModel(spacecraft_index=2), [1])
        @test !SE._control_effector_targets_owned(SM.RPOMPCControlModel(chaser_index=1, target_index=2), [2])
        @test SE._effector_spacecraft_index(SM.RobotArmControlEffector(spacecraft_index=2)) == 2
        reaction = SM.RobotArmReactionEffector(spacecraft_index=2)
        @test SE._effector_spacecraft_index(reaction) == 2
        @test SE._control_effector_targets_owned(reaction, [2]) && !SE._control_effector_targets_owned(reaction, [1])
        @test isempty(rename_warnings(() -> (SE._effector_spacecraft_index(reaction),
            SE._control_effector_targets_owned(reaction, [2]))))
        # A contributor model that still has the older field is still read.
        legacy = ContributorIndexEffector(2)
        @test SE._effector_spacecraft_index(legacy) == 2
        @test SE._control_effector_targets_owned(legacy, [2]) && !SE._control_effector_targets_owned(legacy, [1])
        @test SE._effector_spacecraft_index(SM.BaseThrusterModel(thrust=[0.0], direction=[0.0], Δv=[0.0],
            start_burn_time=[0.0], stop_burn_time=[0.0], Isp=[300.0])) === nothing
    end
end
end
