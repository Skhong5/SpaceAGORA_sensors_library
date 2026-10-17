"""
    Wrapper module for all dynamic effector models (all forces/torques)
"""
module DynamicEffectors
    """
        calcForceTorque(model, x, p, i) -> (force_n, torque_n_m)

    Deprecated: implement `wrench(model, x::StateSample, env::EnvironmentSample,
    t::Float64)` instead. `calcForceTorque` will be removed in SpaceAGORA 0.5.0.

    The older force and torque hook for an [`AbstractForceTorqueModel`](@ref).
    `x` is the spacecraft's state view (`x.pos`, `x.vel`, `x.mass`, and with
    attitude propagation `x.q` and `x.ω`), `p` the run's parameters and `i` the
    spacecraft's position in the run (from 1). It returns the force in the
    inertial frame in N and the torque in the bus frame in N·m. The dynamics call
    a model's own `calcForceTorque` method only when the model has no `wrench`
    method, and the run warns about it before integration.

    Called on a model with `wrench`, as every built-in model is, it warns once
    per model type and evaluates that `wrench` at the run's current time with
    the state sample and environment built from `x`, writing spacecraft `i`'s
    drag diagnostics.
    """
    function calcForceTorque end
    using ..EffectorSampling: wrench, wrench_caching!, environment_requirements, solver_partition

    include(joinpath(@__DIR__, "force_torque_models", "gravity_effectors.jl"))
    include(joinpath(@__DIR__, "force_torque_models", "aerodynamic_effectors.jl"))
    include(joinpath(@__DIR__, "force_torque_models", "perturbation_effectors.jl"))
    include(joinpath(@__DIR__, "force_torque_models", "guidance_models.jl"))
    include(joinpath(@__DIR__, "force_torque_models", "robot_arm_reaction_effector.jl"))
    include(joinpath(@__DIR__, "force_torque_models", "reaction_wheel_momentum.jl"))
    include(joinpath(@__DIR__, "force_torque_models", "laser_force_effectors.jl"))

    include(joinpath(@__DIR__, "force_torque_models", "plume_surface_interaction.jl"))

    # Gravity helpers still rely on perturbation calculations for legacy aerobraking paths.
    @eval GravityEffectors using ..PerturbationEffectors

    using .GravityEffectors: ConstantGravityModel, InverseSquaredGravityModel, InverseSquaredJ2GravityModel
    using .GravityEffectors: GravityGradientTorqueModel
    using .GravityEffectors: aerobraking_gravity_force_ii
    using .AerodynamicEffectors: AerodynamicCoefficientConstant, AerodynamicCoefficientfM, AerodynamicCoefficientNoBallisticFlight
    using .AerodynamicEffectors: _parse_bool_env, _multibody_outer_parallel_hint, collect_and_reset_link_wrenches!
    using .AerodynamicEffectors: _multibody_parallel_mode, _multibody_thread_threshold, _multibody_max_threads, refresh_multibody_parallel_mode!
    using .AerodynamicEffectors: _threadid_capacity, _multibody_use_threads, _multibody_thread_decision
    using .AerodynamicEffectors: _make_aero_scratch_workspace, _ensure_aero_workspace_capacity!, _aero_workspace_for_sat!
    using .AerodynamicEffectors: MeshAeroPanels, MeshAeroSurrogate, AerodynamicCoefficientMeshSurrogate, MESH_AERO_MAX_DEGREE
    using .AerodynamicEffectors: mesh_aero_panels, panel_aero_coefficients, panel_aero_coefficients_split, panel_shadow_mask, panel_projected_area
    using .AerodynamicEffectors: fit_mesh_aero_surrogate, mesh_aero_coefficients, write_mesh_aero_surrogate, read_mesh_aero_surrogate
    using .PerturbationEffectors: NBodyGravityModel, GravitationalHarmonicsModel, SolarRadiationPressureModel, FacetSolarRadiationPressureModel
    using .PerturbationEffectors: MagneticTorqueRodModel, get_magnetic_field_dipole, calculate_magnetic_torque, rod_field_body
    using .PerturbationEffectors: EddyCurrentDampingModel, eddy_damping_torque
    using .PerturbationEffectors: LVLHCascadeAttitudeControlModel
    using .PerturbationEffectors: srp, srp_cannonball_accel, _spice_query_name
    using .PerturbationEffectors: planetary_albedo_accel, planetary_ir_accel
    using .PerturbationEffectors: _make_nbody_scratch_workspace, _ensure_nbody_workspace_capacity!, _nbody_workspace_for_sat!
    using .PerturbationEffectors: _make_harmonics_scratch_workspace, _harmonics_workspace_for_sat!
    using .PerturbationEffectors: _nbody_body_position_from_cache_j2000_m, _srp_sun_position_from_cache_j2000_m
    using .PerturbationEffectors: eclipse_area_calc
    using .GuidanceModels: AerobrakingCampaignPropulsiveManeuverGuidanceModel
    using .RobotArmReactionEffectors: RobotArmReactionEffector
    using .ReactionWheelMomentum: WheelSpeedSpline, ReactionWheelMomentumModel, ReactionWheelMomentumState
    using .ReactionWheelMomentum: wheel_speed_spline, wheel_spline_value, wheel_spline_derivative
    using .ReactionWheelMomentum: wheel_momentum_body, wheel_momentum_rate_body, wheel_reaction_torque, wheel_speeds_rad_s
    using .LaserForceEffectors: force_on_endpoint, laser_force_on_spacecraft

    using .PlumeSurfaceInteraction: PlumeSurfaceConfig, PlumeSurfaceInteractionModel, PlumeSurfaceState, plume_surface_footprint, plume_erosion_onset_height, plume_ground_effect_force, plume_quantities
    export PlumeSurfaceConfig, PlumeSurfaceInteractionModel, PlumeSurfaceState, plume_surface_footprint, plume_erosion_onset_height, plume_ground_effect_force, plume_quantities

    export ConstantGravityModel, InverseSquaredGravityModel, InverseSquaredJ2GravityModel, GravityGradientTorqueModel
    export NBodyGravityModel, GravitationalHarmonicsModel, SolarRadiationPressureModel, FacetSolarRadiationPressureModel
    export aerobraking_gravity_force_ii, srp, srp_cannonball_accel, planetary_albedo_accel, planetary_ir_accel
    export MagneticTorqueRodModel, get_magnetic_field_dipole, calculate_magnetic_torque, rod_field_body
    export EddyCurrentDampingModel, eddy_damping_torque
    export LVLHCascadeAttitudeControlModel
    export AerodynamicCoefficientConstant, AerodynamicCoefficientfM, AerodynamicCoefficientNoBallisticFlight
    export MeshAeroPanels, MeshAeroSurrogate, AerodynamicCoefficientMeshSurrogate, MESH_AERO_MAX_DEGREE
    export mesh_aero_panels, panel_aero_coefficients, panel_aero_coefficients_split, panel_shadow_mask, panel_projected_area
    export fit_mesh_aero_surrogate, mesh_aero_coefficients, write_mesh_aero_surrogate, read_mesh_aero_surrogate
    export calcForceTorque
    export wrench, wrench_caching!, environment_requirements, solver_partition
    export AerobrakingCampaignPropulsiveManeuverGuidanceModel
    export RobotArmReactionEffector
    export WheelSpeedSpline, ReactionWheelMomentumModel, ReactionWheelMomentumState
    export wheel_speed_spline, wheel_spline_value, wheel_spline_derivative
    export wheel_momentum_body, wheel_momentum_rate_body, wheel_reaction_torque, wheel_speeds_rad_s
    export force_on_endpoint, laser_force_on_spacecraft, is_gravitational
    # Which effectors model gravity (after the gravity models are in scope).
    include(joinpath(@__DIR__, "force_torque_models", "gravitational.jl"))
end
