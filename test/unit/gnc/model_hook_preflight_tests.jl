using SpaceAGORA, Test
using StaticArrays
using ComponentArrays: ComponentVector

const HP_SM = SpaceAGORA.SimulationModel
const HP_SE = SpaceAGORA.SimulationEngine
const HP_ZERO3 = SVector{3, Float64}(0.0, 0.0, 0.0)

# One model per accepted or refused case of the pre-run hook check.
struct HpGuidanceNoHook <: SpaceAGORA.AbstractGuidanceModel end
struct HpGuidanceWrongTime <: SpaceAGORA.AbstractGuidanceModel end
SpaceAGORA.update_guidance!(::HpGuidanceWrongTime, u, p, t::Int, spacecraft_index::Int) = nothing
mutable struct HpGuidanceCounter <: SpaceAGORA.AbstractGuidanceModel
    calls::Int
    first_t::Float64
end
HpGuidanceCounter() = HpGuidanceCounter(0, NaN)
function SpaceAGORA.update_guidance!(m::HpGuidanceCounter, u, p, t::Float64, spacecraft_index::Int)
    m.calls == 0 && (m.first_t = t)
    m.calls += 1
    return nothing
end

# A state argument typed narrower than the ComponentVector the engine passes is
# refused; one typed as that ComponentVector is accepted.
struct HpGuidanceVectorState <: SpaceAGORA.AbstractGuidanceModel end
SpaceAGORA.update_guidance!(::HpGuidanceVectorState, u::Vector{Float64}, p, t::Float64, spacecraft_index::Int) = nothing
struct HpGuidanceComponentState <: SpaceAGORA.AbstractGuidanceModel end
SpaceAGORA.update_guidance!(::HpGuidanceComponentState, u::ComponentVector, p::HP_SM.ODEParams, t::Float64, spacecraft_index::Int) = nothing

struct HpNavigationNoHook <: SpaceAGORA.AbstractNavigationModel end
mutable struct HpNavigationCounter <: SpaceAGORA.AbstractNavigationModel
    calls::Int
    first_t::Float64
end
HpNavigationCounter() = HpNavigationCounter(0, NaN)
function SpaceAGORA.update_navigation!(m::HpNavigationCounter, u, p, t::Float64, spacecraft_index::Int)
    m.calls == 0 && (m.first_t = t)
    m.calls += 1
    return nothing
end

# A control tick but no wrench hook: refused in :ideal actuation, accepted in :mounted.
struct HpControlTickOnly <: SpaceAGORA.AbstractControlEffectorModel end
SpaceAGORA.update_control!(::HpControlTickOnly, u, p, t::Float64, i::Int) = nothing
struct HpMountedControl <: SpaceAGORA.AbstractControlEffectorModel end
SpaceAGORA.update_control!(::HpMountedControl, u, p, t::Float64, i::Int) = nothing
SpaceAGORA.actuation_mode(::HpMountedControl) = :mounted

struct HpForceNoHook <: SpaceAGORA.AbstractForceTorqueModel end
# Only the deprecated calcForceTorque hook; counts the dynamics evaluations it serves.
mutable struct HpLegacyForceCounter <: SpaceAGORA.AbstractForceTorqueModel
    calls::Int
end
function SpaceAGORA.calcForceTorque(m::HpLegacyForceCounter, x::AbstractVector, p, i::Int)
    m.calls += 1
    return HP_ZERO3, HP_ZERO3
end
# For the deprecation warnings: only calcForceTorque, and both hooks.
struct HpDeprecatedForce <: SpaceAGORA.AbstractForceTorqueModel end
SpaceAGORA.calcForceTorque(::HpDeprecatedForce, x, p, i) = (HP_ZERO3, HP_ZERO3)
struct HpBothForce <: SpaceAGORA.AbstractForceTorqueModel end
SpaceAGORA.calcForceTorque(::HpBothForce, x, p, i) = (HP_ZERO3, HP_ZERO3)
SpaceAGORA.wrench(::HpBothForce, x::SpaceAGORA.StateSample, env::SpaceAGORA.EnvironmentSample, t::Float64) = (HP_ZERO3, HP_ZERO3)

# Backbone declarations: a gravity core with its acceleration hook, one without,
# and a kick without its kick hook; none has wrench or calcForceTorque. Then a
# model that opts into the per-link kernel without the base-body hooks.
struct HpBackboneCore <: SpaceAGORA.AbstractForceTorqueModel end
SpaceAGORA.gravity_backbone_structure(::HpBackboneCore) = :position_only_static_gravity
SpaceAGORA.gravity_backbone_acceleration_ii(::HpBackboneCore, x::SpaceAGORA.StateSample, env::SpaceAGORA.EnvironmentSample, t::Float64) = HP_ZERO3
struct HpBackboneOnly <: SpaceAGORA.AbstractForceTorqueModel end
SpaceAGORA.gravity_backbone_structure(::HpBackboneOnly) = :position_only_static_gravity
struct HpKickNoAccel <: SpaceAGORA.AbstractForceTorqueModel end
SpaceAGORA.gravity_backbone_kick_structure(::HpKickNoAccel) = :velocity_kick_explicit
struct HpLinkOnly <: SpaceAGORA.AbstractForceTorqueModel end
HP_SM.link_wrench_capable(::HpLinkOnly) = true

struct HpDensitySixArgs <: SpaceAGORA.AbstractDensityModel end
SpaceAGORA.getDensity(::HpDensitySixArgs, h::Float64, lat::Float64, lon::Float64, t::Float64, wind::Bool) =
    (0.0, 200.0, HP_ZERO3)
struct HpDensitySevenArgs <: SpaceAGORA.AbstractDensityModel end
SpaceAGORA.getDensity(::HpDensitySevenArgs, h::Float64, lat::Float64, lon::Float64, t::Float64, wind::Bool, p) =
    (0.0, 200.0, HP_ZERO3)

include(joinpath(@__DIR__, "..", "..", "..", "templates", "force_torque_model_template.jl"))

# The parts of a configuration the check reads.
hp_args(; guidance=(), navigation=(), control=(), effectors=(), density=SpaceAGORA.NoAtmosphereModel()) = (
    guidance_model=HP_SM.GuidanceModel(guidance, fill(1.0, length(guidance))),
    navigation_model=HP_SM.NavigationModel(navigation, fill(1.0, length(navigation))),
    control_model=HP_SM.ControlModel(control, fill(1.0, length(control))),
    dynamics_model=(dynamic_effectors=effectors,),
    environment_model=(density_model=density, ephemerides_model=SpaceAGORA.SimpleEphemeridesModel(),
        planet=HP_SM.make_no_gram_planet(:earth)),
    initial_time=HP_SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
)

# Every concrete type defined in SpaceAGORA's modules that subtypes `T`.
const hp_package_types = SpaceAGORA.Conformance.package_types

function hp_run_config(; guidance=(), navigation=(), effectors=(), solver_mode=:tsit5)
    planet = HP_SM.make_no_gram_planet(:earth)
    root = HP_SM.Link(root=true, m=100.0, ref_area=2.0)
    ic = HP_SM.InitialCondition(ra=planet.Rp_e + 700e3, rp=planet.Rp_e + 650e3, i=45.0, ω=0.0, Ω=0.0, ν=10.0)
    sc = HP_SM.SpacecraftModel(joints=HP_SM.Joint[], links=[root], root=root, prop_mass=0.0,
        inertia_tensor=root.inertia, initial_condition=ic, id=1)
    env = HP_SM.EnvironmentModel(planet=planet, EI=120.0, density_model=SpaceAGORA.NoAtmosphereModel(),
        ephemerides_model=SpaceAGORA.SimpleEphemeridesModel(),
        thermal_model=HP_SM.MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet),
        topography=false, wind=false)
    return HP_SM.SimulationConfiguration(
        simulation_settings=HP_SM.SimulationSettings(results=false, verbose=false, generate_plots=false, normalize=false),
        mission_configuration=HP_SM.MissionConfiguration(mission_type=HP_SM.MissionTime, keplerian=true,
            number_of_orbits=1, mission_time=60.0, orientation_sim=false, num_steps_to_save=100),
        environment_model=env,
        dynamics_model=HP_SM.DynamicsModel(HP_SM.SpacecraftModel[sc], (HP_SM.InverseSquaredJ2GravityModel(), effectors...)),
        solver_config=HP_SM.SolverConfig(; solver_mode),
        guidance_model=HP_SM.GuidanceModel(guidance, fill(10.0, length(guidance))),
        navigation_model=HP_SM.NavigationModel(navigation, fill(15.0, length(navigation))),
        control_model=HP_SM.ControlModel((), Float64[]),
        initial_time=HP_SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0))
end

# The concrete state, spacecraft-view and parameter types of a real run, which the
# check uses.
const HP_FIXTURE = hp_run_config()
const HP_U0 = HP_SE.build_initial_conditions(HP_FIXTURE)
const HP_PARAMS = HP_SM.ODEParams(n_sats=1, args=HP_FIXTURE)
const HP_CALL = HP_SE._hook_call_types(HP_U0, HP_PARAMS)
const HP_P = HP_CALL.params
const HP_X = only(HP_CALL.spacecraft)
hp_sampled(f, T) = hasmethod(f, Tuple{T, SpaceAGORA.StateSample, SpaceAGORA.EnvironmentSample, Float64})
# Whether `T` has its own method of a backbone declaration hook, not the default.
hp_declares(f, T) = which(f, Tuple{T}).sig !== Tuple{typeof(f), Any}

@testset "Model hook declarations and pre-run check" begin
    @testset "hooks are public with generic docstrings" begin
        for name in (:AbstractNavigationModel, :update_guidance!, :update_navigation!, :update_control!,
                     :calcGuidanceEffect!, :calcNavigationEffect!, :calcControlEffect!,
                     :calcReactionWheelTorque, :getHeatRate, :touchdown_spec,
                     :control_wrench, :control_propellant_rate, :control_wheel_torque)
            @test name in names(SpaceAGORA)
        end
        @test SpaceAGORA.control_wrench === HP_SM.ControlHooks.control_wrench
        @test SpaceAGORA.control_propellant_rate === HP_SM.ControlHooks.control_propellant_rate
        @test SpaceAGORA.control_wheel_torque === HP_SM.ControlHooks.control_wheel_torque
        @test isabstracttype(SpaceAGORA.AbstractNavigationModel)
        @test SpaceAGORA.AbstractNavigationModel === HP_SM.AbstractTypes.AbstractNavigationModel
        @test SpaceAGORA.update_guidance! === HP_SM.GuidanceHooks.update_guidance!
        @test SpaceAGORA.update_navigation! === HP_SM.NavigationHooks.update_navigation!
        @test SpaceAGORA.update_control! === HP_SM.ControlHooks.update_control!
        @test SpaceAGORA.calcGuidanceEffect! === HP_SM.GuidanceHooks.calcGuidanceEffect!
        @test SpaceAGORA.calcNavigationEffect! === HP_SM.NavigationHooks.calcNavigationEffect!
        @test SpaceAGORA.calcControlEffect! === HP_SM.ControlHooks.calcControlEffect!
        @test SpaceAGORA.calcReactionWheelTorque === HP_SM.ControlHooks.calcReactionWheelTorque
        @test SpaceAGORA.touchdown_spec === HP_SM.ControlHooks.touchdown_spec
        @test SpaceAGORA.getHeatRate === HP_SM.VehicleThermalModels.getHeatRate

        # The docstring each public hook shows is the generic one, not a method's.
        root_doc(name) = string(Docs.meta(SpaceAGORA)[Docs.Binding(SpaceAGORA, name)].docs[Union{}].text...)
        @test startswith(root_doc(:update_guidance!), "    update_guidance!(model, u, p, t, spacecraft_index)")
        @test startswith(root_doc(:update_navigation!), "    update_navigation!(model, u, p, t, spacecraft_index)")
        @test startswith(root_doc(:update_control!), "    update_control!(model, u, p, t, spacecraft_index)")
        @test !occursin("update_control!(model::BaseThrusterModel", root_doc(:update_control!))
        @test startswith(root_doc(:calcReactionWheelTorque), "    calcReactionWheelTorque(model, u, p, spacecraft_index, t)")
        for name in (:control_wrench, :control_propellant_rate, :control_wheel_torque)
            @test startswith(root_doc(name), "    $(name)(model, x::StateSample, p, t::Float64)")
            @test !occursin("::BaseThrusterModel", root_doc(name))
        end
        for name in (:calcControlForceTorque, :calcControlMassFlowRate, :calcReactionWheelTorque)
            @test occursin("\n\nDeprecated, use [`control_", root_doc(name))
        end
        for (old, new) in ((:calcGuidanceEffect!, :update_guidance!), (:calcNavigationEffect!, :update_navigation!),
                           (:calcControlEffect!, :update_control!))
            @test startswith(root_doc(old), "    $(old)(model, u, p, t, spacecraft_index)")
            @test occursin("\n\nDeprecated, use [`$(new)`]", root_doc(old))
        end
        # No catch-all method of a new hook during the deprecation window, so
        # `hasmethod` tells which types implement them.
        for f in (SpaceAGORA.control_wrench, SpaceAGORA.control_propellant_rate, SpaceAGORA.control_wheel_torque)
            @test !hasmethod(f, Tuple{Any, Any, Any, Any})
        end
        for f in (SpaceAGORA.update_guidance!, SpaceAGORA.update_navigation!, SpaceAGORA.update_control!)
            @test !hasmethod(f, Tuple{Any, Any, Any, Any, Any})
        end
        @test startswith(root_doc(:getHeatRate), "    getHeatRate(model, S, T, ρ, v, α)")
        @test startswith(root_doc(:touchdown_spec), "    touchdown_spec(effector, spacecraft_index::Int)")
        # The ephemerides hooks, model first, and the model-last names they replace.
        em = HP_SM.EphemeridesModels
        for (name, arity) in ((:ephemerides_frame_lpi, 3), (:ephemerides_sun_ii, 3), (:ephemerides_epoch_seconds, 2),
                              (:ephemerides_requires_spice, 1), (:ephemerides_cache_key, 1))
            @test name in names(SpaceAGORA) && getfield(SpaceAGORA, name) === getfield(em, name)
            @test startswith(root_doc(name), "    $(name)(model")
            @test !hasmethod(getfield(SpaceAGORA, name), Tuple{ntuple(_ -> Any, arity)...})
        end
        for (old, new, form) in ((:planet_frame_lpi, :ephemerides_frame_lpi, "(planet, et, model)"),
                                 (:ephemerides_sun_direction_ii, :ephemerides_sun_ii, "(planet, et, model)"),
                                 (:ephemerides_time_seconds, :ephemerides_epoch_seconds, "(initial_time, model)"))
            @test old in names(SpaceAGORA) && getfield(SpaceAGORA, old) === getfield(em, old)
            @test startswith(root_doc(old), "    $(old)$(form)")
            @test occursin("\n\nDeprecated, use [`$(new)`]", root_doc(old))
        end
        # No catch-all navigation method: an effector without one is still a
        # MethodError when called, under either name.
        @test_throws MethodError SpaceAGORA.update_navigation!(HpNavigationNoHook(), Float64[], nothing, 0.0, 1)
        @test_throws MethodError SpaceAGORA.calcNavigationEffect!(HpNavigationNoHook(), Float64[], nothing, 0.0, 1)
    end

    @testset "the new hook names collide with no package often loaded beside SpaceAGORA" begin
        # A script that loads these packages with SpaceAGORA must see the new
        # names unqualified: a name two of them export would be ambiguous.
        new_names = (:control_wrench, :control_propellant_rate, :control_wheel_torque,
            :update_control!, :update_guidance!, :update_navigation!,
            :ephemerides_frame_lpi, :ephemerides_sun_ii, :ephemerides_epoch_seconds,
            :ephemerides_requires_spice, :ephemerides_cache_key,
            :planet_frame_lpi, :ephemerides_sun_direction_ii, :ephemerides_time_seconds)
        scope = Module(:HpNameCollisions)
        Core.eval(scope, :(using SpaceAGORA, Plots, OrdinaryDiffEq, ComponentArrays, StaticArrays, DataFrames))
        for pkg in (:Plots, :OrdinaryDiffEq, :ComponentArrays, :StaticArrays, :DataFrames)
            exported = names(getfield(scope, pkg))
            @test isempty(filter(in(exported), new_names))
        end
        for name in new_names
            @test isdefined(scope, name) && getfield(scope, name) === getfield(SpaceAGORA, name)
        end
    end

    @testset "the check uses the run's concrete argument types" begin
        @test HP_CALL.state === typeof(HP_U0) && HP_CALL.state <: ComponentVector
        @test HP_X === typeof(HP_U0.sc[1]) && HP_X <: ComponentVector
        @test HP_P === typeof(HP_PARAMS)
    end

    @testset "built-in models pass the check" begin
        # Guidance, control and force/torque models: the conformance tests
        # (test/unit/conformance/builtin_conformance_tests.jl) run this check, the
        # deprecated-hook list and the sample signatures on an instance of each.
        # The conformance guidance report has no deprecated-hook check, so the
        # deprecated guidance tick is checked here: no built-in guidance
        # effector has a method of its own of it.
        for T in hp_package_types(SpaceAGORA.AbstractGuidanceModel)
            @test !HP_SE._own_method(SpaceAGORA.calcGuidanceEffect!, Tuple{T, HP_CALL.state, HP_P, Float64, Int})
        end
        forces = hp_package_types(SpaceAGORA.AbstractForceTorqueModel)
        # Every built-in backbone declaration has its acceleration hook.
        cores = filter(T -> hp_declares(SpaceAGORA.gravity_backbone_structure, T), forces)
        kicks = filter(T -> hp_declares(SpaceAGORA.gravity_backbone_kick_structure, T), forces)
        @test !isempty(cores) && !isempty(kicks)
        for T in cores
            @test hp_sampled(SpaceAGORA.gravity_backbone_acceleration_ii, T)
        end
        for T in kicks
            @test hp_sampled(SpaceAGORA.gravity_backbone_kick_acceleration_ii, T)
        end
        # The GRAM-backed density models get getDensity from the GRAMSuite extension.
        em = HP_SM.EnvironmentModels
        gram = (em.GRAMAtmosphereModel, em.GRAMAtmosphereModelSurrogate, em.GRAMGridAtmosphereModel, em.GRAMNearSurfaceAtmosphereModel)
        gram_loaded = Base.get_extension(SpaceAGORA, :SpaceAGORAGRAMSuiteExt) !== nothing
        densities = hp_package_types(SpaceAGORA.AbstractDensityModel)
        @test length(densities) >= 10
        for T in densities
            ok = hasmethod(SpaceAGORA.getDensity, Tuple{T, Float64, Float64, Float64, Float64, Bool, HP_P})
            @test ok == (gram_loaded || !any(G -> Base.unwrap_unionall(T) <: G, gram))
        end
    end

    @testset "missing hooks are named" begin
        args = hp_args(
            guidance=(HpGuidanceNoHook(), HpGuidanceWrongTime(), HpGuidanceCounter(), HpGuidanceVectorState(), HpGuidanceComponentState()),
            navigation=(HpNavigationNoHook(), HpNavigationCounter()),
            control=(HpControlTickOnly(), HpMountedControl()),
            effectors=(HpForceNoHook(), HpLegacyForceCounter(0), HP_SM.InverseSquaredJ2GravityModel()),
            density=HpDensitySixArgs(),
        )
        lines = HP_SE._missing_model_hooks(args, HP_CALL)
        has(parts...) = any(line -> all(part -> occursin(part, line), parts), lines)
        @test length(lines) == 7
        @test has("guidance_model.guidance_effectors[1] (HpGuidanceNoHook)", "update_guidance!(model, u, p, t::Float64, spacecraft_index::Int)")
        @test has("guidance_model.guidance_effectors[2] (HpGuidanceWrongTime)", "update_guidance!", "none accepts the arguments this run passes")
        @test has("guidance_model.guidance_effectors[4] (HpGuidanceVectorState)", "update_guidance!", "none accepts the arguments this run passes")
        @test !has("(HpGuidanceNoHook)", "none accepts")
        @test has("navigation_model.navigation_effectors[1] (HpNavigationNoHook)", "update_navigation!")
        @test has("control_model.control_effectors[1] (HpControlTickOnly)",
            "has no method control_wrench(model, x::StateSample, p, t::Float64)", "actuation_mode :ideal")
        @test !has("(HpControlTickOnly)", "calcControlForceTorque")
        @test has("dynamics_model.dynamic_effectors[1] (HpForceNoHook)", "has no method wrench(model, x::StateSample")
        @test !has("(HpForceNoHook)", "calcForceTorque")
        @test has("environment_model.density_model (HpDensitySixArgs)", "getDensity(", "wind::Bool, p)")
        # A :mounted effector needs no control_wrench, and calcForceTorque alone serves a force model.
        for accepted in ("HpGuidanceCounter", "HpGuidanceComponentState", "HpNavigationCounter", "HpMountedControl", "HpLegacyForceCounter", "InverseSquaredJ2GravityModel")
            @test !has(accepted)
        end

        err = try
            HP_SE._validate_model_hooks!(args, HP_U0, HP_PARAMS)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError
        @test all(line -> occursin(line, err.msg), lines)

        @test HP_SE._validate_model_hooks!(hp_args(
            guidance=(HpGuidanceCounter(),), navigation=(HpNavigationCounter(),), control=(HpMountedControl(),),
            effectors=(HpLegacyForceCounter(0),), density=HpDensitySevenArgs(),
        ), HP_U0, HP_PARAMS) === nothing

        # Outside gravity_backbone_split the backbone hooks do not count; a per-link
        # model passes in any mode.
        special = hp_args(effectors=(HpBackboneCore(), HpBackboneOnly(), HpKickNoAccel(), HpLinkOnly()))
        plain = HP_SE._missing_model_hooks(special, HP_CALL)
        @test length(plain) == 3
        @test all(k -> any(l -> occursin("dynamic_effectors[$k] (", l) && occursin("has no method wrench(", l), plain), 1:3)
        # In gravity_backbone_split a declaration needs its acceleration hook.
        split = HP_SE._missing_model_hooks(special, HP_CALL, :gravity_backbone_split)
        @test length(split) == 2
        @test any(l -> occursin("dynamic_effectors[2] (HpBackboneOnly) has no method gravity_backbone_acceleration_ii(", l), split)
        @test any(l -> occursin("dynamic_effectors[3] (HpKickNoAccel) has no method gravity_backbone_kick_acceleration_ii(", l), split)
        @test !any(l -> occursin("HpBackboneCore", l) || occursin("HpLinkOnly", l), split)

        # A model in the wrong container is pointed to its own.
        @test occursin("Its type belongs in control_model.", only(HP_SE._missing_model_hooks(hp_args(guidance=(HpControlTickOnly(),)), HP_CALL)))
        # The energy depletion controller's panel actuator cannot run on its own.
        aoa_line = only(HP_SE._missing_model_hooks(hp_args(control=(SpaceAGORA.SolarPanelAngleOfAttackControlModel(),)), HP_CALL))
        @test occursin("(SolarPanelAngleOfAttackControlModel) has no method update_control!", aoa_line)
    end

    @testset "a run with a missing hook stops before integration" begin
        force = HpLegacyForceCounter(0)
        navigation = HpNavigationCounter()
        args = hp_run_config(guidance=(HpGuidanceNoHook(),), navigation=(navigation,), effectors=(force,))
        err = try
            run_simulation(args; isolate_state=false)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError
        @test occursin("guidance_model.guidance_effectors[1] (HpGuidanceNoHook) has no method update_guidance!", err.msg)
        # Nothing was integrated: no dynamics evaluation and no navigation tick.
        @test force.calls == 0
        @test navigation.calls == 0

        # A guidance method typed for a Vector state is refused before the run too:
        # the run passes a ComponentVector.
        err = try
            run_simulation(hp_run_config(guidance=(HpGuidanceVectorState(),), effectors=(force,)); isolate_state=false)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError
        @test occursin("(HpGuidanceVectorState) has no method update_guidance!", err.msg)
        @test force.calls == 0

        # A backbone gravity core without its acceleration hook, in gravity_backbone_split.
        err = try
            run_simulation(hp_run_config(effectors=(HpBackboneOnly(),), solver_mode=:gravity_backbone_split); isolate_state=false)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError
        @test occursin("(HpBackboneOnly) has no method gravity_backbone_acceleration_ii(", err.msg)

        # With the hook present the same run integrates, through the deprecated
        # calcForceTorque path too, which it warns about.
        guidance = HpGuidanceCounter()
        @test_logs (:warn, r"HpLegacyForceCounter implements calcForceTorque") match_mode=:any run_simulation(
            hp_run_config(guidance=(guidance,), navigation=(navigation,), effectors=(force,)); isolate_state=false)
        @test force.calls > 0
        @test guidance.calls > 0 && navigation.calls > 0
        # The first tick comes one period after the start, as the docstrings say.
        @test guidance.first_t == 10.0
        @test navigation.first_t == 15.0
    end

    @testset "deprecated calcForceTorque: warning before the run, direct calls forwarded" begin
        # One warning per model type: a model with only calcForceTorque, and one
        # with both hooks, whose calcForceTorque the dynamics never call. Built-in
        # models and models without the hook give none.
        args = hp_args(effectors=(HpDeprecatedForce(), HpBothForce(), HpDeprecatedForce(),
            HP_SM.InverseSquaredJ2GravityModel()))
        found = HP_SE._deprecated_model_hooks(args, HP_CALL)
        @test first.(found) == [(HpDeprecatedForce, :calcForceTorque), (HpBothForce, :calcForceTorque)]
        @test occursin("HpDeprecatedForce implements calcForceTorque(model, x, p, i), which is deprecated", last(found[1]))
        @test occursin("x.spacecraft_index", last(found[1]))
        @test occursin("HpBothForce implements both wrench and calcForceTorque", last(found[2]))
        @test occursin("never used", last(found[2]))
        @test isempty(HP_SE._deprecated_model_hooks(hp_args(effectors=(HP_SM.InverseSquaredJ2GravityModel(), HpBackboneCore())), HP_CALL))
        # The force template implements wrench and passes without a warning.
        template = ExampleForceTorqueModelTemplate.ExampleForceTorqueModel(HP_ZERO3, HP_ZERO3)
        @test isempty(HP_SE._missing_model_hooks(hp_args(effectors=(template,)), HP_CALL))
        @test isempty(HP_SE._deprecated_model_hooks(hp_args(effectors=(template,)), HP_CALL))
        # The check warns once per type and hook, however often it runs.
        @test_logs (:warn, r"^HpDeprecatedForce implements") (:warn, r"^HpBothForce implements") begin
            HP_SE._validate_model_hooks!(args, HP_U0, HP_PARAMS)
            HP_SE._validate_model_hooks!(args, HP_U0, HP_PARAMS)
        end
        # Under --depwarn=error the warning is an error.
        helper = joinpath(pkgdir(SpaceAGORA), "src", "foundation", "core", "interfaces", "hook_deprecation.jl")
        script = "include($(repr(helper))); try HookDeprecation._hook_deprecation((Int, :hook), \"msg\"); " *
                 "print(\"no error\") catch err; print(err isa ErrorException && err.msg == \"msg\") end"
        @test read(`$(Base.julia_cmd()) --startup-file=no --depwarn=error -e $script`, String) == "true"
        @test read(`$(Base.julia_cmd()) --startup-file=no --depwarn=no -e $script`, String) == "no error"

        # A direct call on a built-in model evaluates it as the dynamics do.
        p = HP_SM.ODEParams(n_sats=1, args=HP_FIXTURE)
        p.shared_buffers.current_time[] = 30.0
        for model in (HP_SM.InverseSquaredJ2GravityModel(), HP_SM.InverseSquaredGravityModel(), HP_SM.GravityGradientTorqueModel())
            sample = HP_SE.build_state_sample(HP_U0.sc[1], HP_FIXTURE.dynamics_model.spacecraft[1], false, 1)
            expected = HP_SE._evaluate_dynamic_effector(model, HP_U0.sc[1], sample, p, 1, 30.0)
            @test SpaceAGORA.calcForceTorque(model, HP_U0.sc[1], p, 1) === expected
        end
        # A direct call builds the environment from its own `x`, not from what a
        # right-hand side prefilled for another state: the buffers below hold
        # state A's planet frame and a buffered density, and the calls pass
        # state B. A direct call must equal the evaluation at B with nothing
        # prefilled. This run has no atmosphere, so the buffered density alone
        # would give the aerodynamic model a drag, and its prefilled planet-frame
        # position is A's; the dynamics' path reads both. J2 reads only the
        # frame rotation, which is the same for A and B at one time.
        HP_SE._initialize_save_cache_buffers!(p)
        spacecraft = HP_FIXTURE.dynamics_model.spacecraft[1]
        state_a = HP_U0.sc
        x_b = copy(HP_U0.sc[1])
        x_b.pos .= 1.01 .* x_b.pos
        x_b.vel .= 0.99 .* x_b.vel
        sample_b = HP_SE.build_state_sample(x_b, spacecraft, false, 1)
        models = (HP_SM.InverseSquaredJ2GravityModel(), HP_SM.AerodynamicCoefficientfM())
        fresh_b = map(model -> HP_SE._evaluate_dynamic_effector(model, x_b, sample_b, p, 1, 30.0), models)
        @test iszero(last(fresh_b)[1])
        sb = p.shared_buffers
        HP_SE._prefill_environment_samples!(p, 30.0, state_a; atmosphere=false)
        sb.densities[1] = 1.0e-6
        sb.temperatures[1] = 220.0
        sb.density_sample_t[1] = 30.0
        sb.rhs_planet_frame_prefilled[] = true
        sb.rhs_atmosphere_prefilled[] = true
        try
            for (model, expected) in zip(models, fresh_b)
                @test SpaceAGORA.calcForceTorque(model, x_b, p, 1) === expected
            end
            @test HP_SE._evaluate_dynamic_effector(last(models), x_b, sample_b, p, 1, 30.0) !== last(fresh_b)
        finally
            sb.rhs_planet_frame_prefilled[] = false
            sb.rhs_atmosphere_prefilled[] = false
        end
        # A direct call warns that the hook is deprecated, once per model type and
        # independently of the pre-run check, which never ran for these types.
        warning(T) = (:warn, Regex("^calcForceTorque\\(model, x, p, i\\) is deprecated .* called directly on a $(nameof(T))\\."))
        @test_logs warning(HP_SM.InverseSquaredGravityModel) warning(HP_SM.ConstantGravityModel) begin
            SpaceAGORA.calcForceTorque(HP_SM.InverseSquaredGravityModel(), HP_U0.sc[1], p, 1)
            SpaceAGORA.calcForceTorque(HP_SM.InverseSquaredGravityModel(), HP_U0.sc[1], p, 1)
            SpaceAGORA.calcForceTorque(HP_SM.ConstantGravityModel(), HP_U0.sc[1], p, 1)
        end
        # Under --depwarn=error a direct call throws before evaluating anything.
        strict = "using SpaceAGORA; try SpaceAGORA.calcForceTorque(SpaceAGORA.InverseSquaredGravityModel(), nothing, nothing, 1); " *
                 "print(\"no error\") catch err; print(err isa ErrorException && occursin(\"is deprecated\", err.msg)) end"
        @test read(`$(Base.julia_cmd()) --startup-file=no --depwarn=error --project=$(pkgdir(SpaceAGORA)) -e $strict`, String) == "true"
        # A model with neither hook raises a MethodError.
        @test_throws MethodError SpaceAGORA.calcForceTorque(HpForceNoHook(), HP_U0.sc[1], p, 1)
    end
end
