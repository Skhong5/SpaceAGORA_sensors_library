module ControlHooks
    import ...SimulationLifecycle
    using ..Structure

    using ..ConfigTypes: ODEParams
    using ..AbstractTypes: AbstractPlanet, AbstractControlEffectorModel, AbstractThrusterModel
    using ..RobotArmPlanning: RobotArmPlan, robot_arm_plan_sample
    using ..ThrusterModels: BaseThrusterModel, SixAxisThrusterModel
    import ..Actuators: actuation_mode, actuator_levels
    using ..Actuators: propellant_mass_rate, allocate_actuators
    using ..ActuatorMounting: mounted_actuator_count, mounted_thruster_count, mounted_wheel_count, mounted_magnetorquer_count, actuator_context, move_hinge!
    using ..Components: thrust_direction, PanelHinge, ThrusterGimbal
    using ..ActuatorMounting: mounted_actuators
    using ..CommandTypes: PropulsiveManeuverCommand, PropulsiveBurnPlan
    using ..GuidanceModels: RPOPlan, RPOPlanBuffer
    using ..GravityEffectors: aerobraking_gravity_force_ii
    using ..AerodynamicEffectors: aerodynamic_coefficient_fM
    using ..GuidanceHooks: AerobrakingEnergyDepletionConfig, AerobrakingEnergyDepletionState, AerobrakingEnergyDepletionGuidanceModel
    using ..GuidanceHooks: ApolloDescentConfig, ApolloDescentState, _validate_descent_config, _descent_indices
    using ..AbstractTypes: AbstractTerrainModel
    using ..TerrainModels: NoTerrainModel, DEMTerrainModel
    using ..EnvironmentModels: getDensity, _environment_wind
    using ..EphemeridesModels: ephemerides_requires_spice, planet_frame_lpi
    using ..ReferenceSystems
    using ..LinearAlgebra
    using ..StaticArrays
    using ..Kinematics
    using SparseArrays
    using AstroTime
    using SPICE
    using OSQP

    const config = Structure

    export calcControlForceTorque, calcControlEffect!, calcControlMassFlowRate, calcReactionWheelTorque
    export control_thruster_levels, touchdown_spec, refresh_burn!, mounted_burn_tick!, burn_edge
    export AerobrakingEnergyDepletionControlModel, SolarPanelAngleOfAttackControlModel
    export ApolloDescentControlConfig, ApolloDescentControlModel, ApolloDescentControlState, attitude_error_vector
    export RpoLQMPCController, init_rpo_lqmpc, rpo_lqmpc_control
    export RPOControlCommandLog, RPOHeldActuation, RPOMPCControlModel
    export MagneticMomentumManagerModel
    export RobotArmHeldActuation, RobotArmJointMPCController, RobotArmControlEffector
    export init_robot_arm_joint_mpc, robot_arm_joint_mpc_reference_preview
    export robot_arm_joint_mpc_control, robot_arm_measured_joint_state
    export rpo_allocate_six_axis_thrusters, rpo_thruster_wrench_body

    """
        control_thruster_levels(effector, spacecraft_index::Int) -> Union{Nothing, AbstractVector{<:Real}}

    Optional hook: the firing level (0 to 1) of every thruster of the spacecraft
    at `spacecraft_index` (its position in the run, from 1), in the order the
    visualization scene lists them (the spacecraft's links in
    order, each link's `thrusters` in order). Effectors that drive no named
    thruster return `nothing`, the default for any effector type that does not
    override this method; an effector that drives only some of them may return a
    shorter vector, and the remaining thrusters are idle.

    The engine reads this at save time, so an effector must compute the levels
    in `calcControlEffect!` and keep them in its own actuator state rather than
    recomputing them here.
    """
    control_thruster_levels(effector, spacecraft_index::Int) = nothing

    """
        touchdown_spec(effector, spacecraft_index::Int) -> Union{Nothing, NamedTuple}

    Optional terrain-contact event for a selected spacecraft's index in the run
    (not its user-defined ID). Return `nothing` for unselected spacecraft, or
    `(terrain, reference_radius_m, height_m, on_touchdown)`: an AbstractTerrainModel,
    its explicit positive reference-sphere radius in metres, a finite nonnegative
    clearance in metres, and a callable accepting `(t, r_p, v_p, spacecraft_index)`.
    The callback receives planet-fixed position and ground-relative velocity in
    metres and metres per second. It may record contact and turn off actuators.

    The first downward crossing deactivates only this spacecraft; the solve ends
    once all spacecraft are inactive. Initial contact/below-ground states and
    multiple specifications for one index are rejected. The usual 50 km impact
    stop remains active for every spacecraft with no touchdown specification.
    """
    touchdown_spec(effector, spacecraft_index::Int) = nothing

    """
        calcControlForceTorque(model, u, p, spacecraft_index, t) -> (force, torque)

    The force and torque a control effector applies to the spacecraft at
    `spacecraft_index`, which the dynamics evaluate at every right-hand-side
    evaluation in `:ideal` actuation, the default `actuation_mode`. Every
    control effector whose actuation is not `:mounted` needs a method for its
    own type. There is no fallback: `run_simulation` checks for the method
    before integration starts, as it does for `calcControlEffect!`.

    Return a tuple of two 3-vectors:

    - `force`: in the inertial frame, in N;
    - `torque`: in the bus frame, about the root body's centre of mass, in N·m.
      The dynamics apply it only when the run propagates attitude.

    An effector that applies neither returns two zero vectors. The arguments:

    - `model`: the control effector.
    - `u`: the state of that spacecraft alone (`u.pos`, `u.vel`, `u.mass`, and
      with attitude propagation `u.q` and `u.ω`), not the full state the tick
      receives.
    - `p`: the run's parameters.
    - `spacecraft_index`: the spacecraft's position in the run (`Int`, from 1), not its ID.
    - `t`: elapsed time since the run's start epoch, in seconds (`Float64`).

    The spacecraft index comes before the time here, as in
    `calcControlMassFlowRate` and `calcReactionWheelTorque`, and after it in
    `calcControlEffect!`. Return the command held since the last
    `calcControlEffect!` tick rather than recomputing the control law here.
    """
    function calcControlForceTorque end

    """
        calcControlEffect!(model, u, p, t, spacecraft_index) -> nothing

    The control tick, which every effector in a `ControlModel` implements for its
    own type. The effector computes its command (a thrust, a torque, actuator
    levels, panel angles) and keeps it in its own state; the engine ignores the
    return value and applies the held command until the next tick:

    - in `:ideal` actuation, the default `actuation_mode`, through the effector's
      `calcControlForceTorque`, `calcControlMassFlowRate` and
      `calcReactionWheelTorque`, which the dynamics evaluate at every step;
    - in `:mounted` actuation, through the levels `actuator_levels` returns for
      the actuators mounted on the spacecraft; `calcControlForceTorque` and
      `calcControlMassFlowRate` are not called.

    The arguments:

    - `model`: the control effector.
    - `u`: the full integrated state. `u.sc[spacecraft_index]` is the spacecraft, with
      `pos` (m) and `vel` (m/s) relative to the planet's centre in the inertial
      frame and `mass` (kg); with attitude propagation also `q`, the scalar-last
      quaternion from inertial to bus axes, and `ω`, the bus angular velocity in
      rad/s.
    - `p`: the run's parameters; `p.args` is the `SimulationConfiguration`.
    - `t`: elapsed time since the run's start epoch, in seconds (`Float64`).
    - `spacecraft_index`: the spacecraft's position in the run (`Int`, from 1), not its ID.

    Effector `k` runs every `control_rates[k]` seconds of simulated time, once for
    each spacecraft, starting one period after the solve starts. The calls for
    different spacecraft run one after another unless the effector's type is
    declared thread-safe (`SimulationModel.SimulationCallbacks.control_model_threadsafe(::MyEffector) = true`)
    or the run sets `SPACEAGORA_CONTROL_CALLBACK_ASSUME_THREADSAFE=1`; then they
    may run at the same time on different threads, and state that several
    spacecraft share needs a lock. `BaseThrusterModel` is the exception: it runs
    at the start of the solve and when a spacecraft leaves the atmosphere, to
    schedule that spacecraft's burns.

    Before integration starts, `run_simulation` checks that every control
    effector has a method of this function and, unless its actuation is
    `:mounted`, of `calcControlForceTorque`, and stops with an `ArgumentError`
    naming the effector otherwise.
    """
    function calcControlEffect! end

    using ..QuaternionMath
    include(joinpath(@__DIR__, "propulsive_maneuvers.jl"))
    include(joinpath(@__DIR__, "heat_rate_control.jl"))
    include(joinpath(@__DIR__, "heat_load_control.jl"))
    include(joinpath(@__DIR__, "struct_load_control.jl"))
    include(joinpath(@__DIR__, "targeting_control.jl"))
    include(joinpath(@__DIR__, "edg_heat_accounting.jl"))
    include(joinpath(@__DIR__, "energy_depletion_lifecycle.jl"))
    include(joinpath(@__DIR__, "rpo_mpc", "lqmpc.jl"))
    include(joinpath(@__DIR__, "rpo_mpc", "rpo_control_types.jl"))
    include(joinpath(@__DIR__, "..", "..", "vehicle", "actuators", "thruster_allocation.jl"))
    include(joinpath(@__DIR__, "rpo_mpc", "reaction_wheel_allocator.jl"))
    include(joinpath(@__DIR__, "rpo_mpc", "rpo_mpc_control_model.jl"))
    include(joinpath(@__DIR__, "robot_arm_control.jl"))
    include(joinpath(@__DIR__, "momentum_manager.jl"))
    include(joinpath(@__DIR__, "landing", "apollo_descent_control.jl"))

    # Per-spacecraft GNC binding (see SimulationLifecycle.bind_spacecraft).
    function SimulationLifecycle.bind_spacecraft(m::MagneticMomentumManagerModel, sat_idx::Int)
        m.sat_idx == sat_idx && return m
        b = SimulationLifecycle._shallow_copy(m)
        b.sat_idx = sat_idx
        b.mounted_levels = Float64[]          # its own buffer, not the template's
        return b
    end
    function SimulationLifecycle.bind_spacecraft(m::RobotArmControlEffector, sat_idx::Int)
        m.spacecraft_idx == sat_idx && return m
        b = SimulationLifecycle._shallow_copy(m)
        b.spacecraft_idx = sat_idx
        b.controller = deepcopy(m.controller)  # its own MPC warm start, not the template's
        b.held = deepcopy(m.held)              # its own held command, starting from the template's
        return b
    end
    function SimulationLifecycle.bind_spacecraft(m::ApolloDescentControlModel, sat_idx::Int)
        ids = _descent_indices(m.state, (sat_idx,))
        m.spacecraft_indices == ids && return m
        return ApolloDescentControlModel(m.config, m.guidance, m.state, m.terrain, m.actuators, ids, m.actuation)
    end
end
