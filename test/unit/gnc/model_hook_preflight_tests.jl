using SpaceAGORA, Test
using StaticArrays

const HP_SM = SpaceAGORA.SimulationModel
const HP_SE = SpaceAGORA.SimulationEngine
const HP_ZERO3 = SVector{3, Float64}(0.0, 0.0, 0.0)

# One model per accepted or refused case of the pre-run hook check.
struct HpGuidanceNoHook <: SpaceAGORA.AbstractGuidanceModel end
struct HpGuidanceWrongTime <: SpaceAGORA.AbstractGuidanceModel end
SpaceAGORA.calcGuidanceEffect!(::HpGuidanceWrongTime, u, p, t::Int, spacecraft_index::Int) = nothing
mutable struct HpGuidanceCounter <: SpaceAGORA.AbstractGuidanceModel
    calls::Int
    first_t::Float64
end
HpGuidanceCounter() = HpGuidanceCounter(0, NaN)
function SpaceAGORA.calcGuidanceEffect!(m::HpGuidanceCounter, u, p, t::Float64, spacecraft_index::Int)
    m.calls == 0 && (m.first_t = t)
    m.calls += 1
    return nothing
end

struct HpNavigationNoHook <: SpaceAGORA.AbstractNavigationModel end
mutable struct HpNavigationCounter <: SpaceAGORA.AbstractNavigationModel
    calls::Int
    first_t::Float64
end
HpNavigationCounter() = HpNavigationCounter(0, NaN)
function SpaceAGORA.calcNavigationEffect!(m::HpNavigationCounter, u, p, t::Float64, spacecraft_index::Int)
    m.calls == 0 && (m.first_t = t)
    m.calls += 1
    return nothing
end

# A control tick but no wrench hook: refused in :ideal actuation, accepted in :mounted.
struct HpControlTickOnly <: SpaceAGORA.AbstractControlEffectorModel end
SpaceAGORA.calcControlEffect!(::HpControlTickOnly, u, p, t::Float64, i::Int) = nothing
struct HpMountedControl <: SpaceAGORA.AbstractControlEffectorModel end
SpaceAGORA.calcControlEffect!(::HpMountedControl, u, p, t::Float64, i::Int) = nothing
SpaceAGORA.actuation_mode(::HpMountedControl) = :mounted

struct HpForceNoHook <: SpaceAGORA.AbstractForceTorqueModel end
# Only the older calcForceTorque hook; counts the dynamics evaluations it serves.
mutable struct HpLegacyForceCounter <: SpaceAGORA.AbstractForceTorqueModel
    calls::Int
end
function SpaceAGORA.calcForceTorque(m::HpLegacyForceCounter, x::AbstractVector, p, i::Int)
    m.calls += 1
    return HP_ZERO3, HP_ZERO3
end

# A gravity core with only the gravity_backbone_split hooks, and a model that
# opts into the per-link kernel without the base-body hooks.
struct HpBackboneOnly <: SpaceAGORA.AbstractForceTorqueModel end
SpaceAGORA.gravity_backbone_structure(::HpBackboneOnly) = :position_only_static_gravity
struct HpLinkOnly <: SpaceAGORA.AbstractForceTorqueModel end
HP_SM.link_wrench_capable(::HpLinkOnly) = true

struct HpDensitySixArgs <: SpaceAGORA.AbstractDensityModel end
SpaceAGORA.getDensity(::HpDensitySixArgs, h::Float64, lat::Float64, lon::Float64, t::Float64, wind::Bool) =
    (0.0, 200.0, HP_ZERO3)
struct HpDensitySevenArgs <: SpaceAGORA.AbstractDensityModel end
SpaceAGORA.getDensity(::HpDensitySevenArgs, h::Float64, lat::Float64, lon::Float64, t::Float64, wind::Bool, p) =
    (0.0, 200.0, HP_ZERO3)

# The parts of a configuration the check reads.
hp_args(; guidance=(), navigation=(), control=(), effectors=(), density=SpaceAGORA.NoAtmosphereModel()) = (
    guidance_model=HP_SM.GuidanceModel(guidance, fill(1.0, length(guidance))),
    navigation_model=HP_SM.NavigationModel(navigation, fill(1.0, length(navigation))),
    control_model=HP_SM.ControlModel(control, fill(1.0, length(control))),
    dynamics_model=(dynamic_effectors=effectors,),
    environment_model=(density_model=density,),
)

# Every concrete type defined in SpaceAGORA's modules that subtypes `T`.
function hp_package_types(T::Type)
    found = Any[]
    seen = Set{Module}()
    function walk(m::Module)
        m in seen && return
        push!(seen, m)
        for n in names(m; all=true)
            isdefined(m, n) || continue
            v = getfield(m, n)
            if v isa Module
                parentmodule(v) === m && v !== m && walk(v)
            elseif v isa Union{DataType, UnionAll} && parentmodule(v) === m
                body = Base.unwrap_unionall(v)
                body isa DataType && !isabstracttype(body) && body <: T && push!(found, v)
            end
        end
    end
    walk(SpaceAGORA)
    return unique(found)
end

const HP_P = HP_SM.ODEParams
hp_tick_ok(f, T) = HP_SE._hook_method_may_apply(f, Tuple{T, AbstractVector, HP_P, Float64, Int})

function hp_run_config(; guidance=(), navigation=(), effectors=())
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
        guidance_model=HP_SM.GuidanceModel(guidance, fill(10.0, length(guidance))),
        navigation_model=HP_SM.NavigationModel(navigation, fill(15.0, length(navigation))),
        control_model=HP_SM.ControlModel((), Float64[]),
        initial_time=HP_SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0))
end

@testset "Model hook declarations and pre-run check" begin
    @testset "hooks are public with generic docstrings" begin
        for name in (:AbstractNavigationModel, :calcGuidanceEffect!, :calcNavigationEffect!,
                     :calcReactionWheelTorque, :getHeatRate, :touchdown_spec)
            @test name in names(SpaceAGORA)
        end
        @test isabstracttype(SpaceAGORA.AbstractNavigationModel)
        @test SpaceAGORA.AbstractNavigationModel === HP_SM.AbstractTypes.AbstractNavigationModel
        @test SpaceAGORA.calcGuidanceEffect! === HP_SM.GuidanceHooks.calcGuidanceEffect!
        @test SpaceAGORA.calcNavigationEffect! === HP_SM.NavigationHooks.calcNavigationEffect!
        @test SpaceAGORA.calcReactionWheelTorque === HP_SM.ControlHooks.calcReactionWheelTorque
        @test SpaceAGORA.touchdown_spec === HP_SM.ControlHooks.touchdown_spec
        @test SpaceAGORA.getHeatRate === HP_SM.VehicleThermalModels.getHeatRate

        # The docstring each public hook shows is the generic one, not a method's.
        root_doc(name) = string(Docs.meta(SpaceAGORA)[Docs.Binding(SpaceAGORA, name)].docs[Union{}].text...)
        @test startswith(root_doc(:calcGuidanceEffect!), "    calcGuidanceEffect!(model, u, p, t, spacecraft_index)")
        @test !occursin("RPOGuidanceModel", root_doc(:calcGuidanceEffect!))
        @test startswith(root_doc(:calcNavigationEffect!), "    calcNavigationEffect!(model, u, p, t, spacecraft_index)")
        @test startswith(root_doc(:calcControlEffect!), "    calcControlEffect!(model, u, p, t, spacecraft_index)")
        @test !occursin("calcControlEffect!(model::BaseThrusterModel", root_doc(:calcControlEffect!))
        @test startswith(root_doc(:calcReactionWheelTorque), "    calcReactionWheelTorque(model, u, p, spacecraft_index, t)")
        @test startswith(root_doc(:getHeatRate), "    getHeatRate(model, S, T, ρ, v, α)")
        @test startswith(root_doc(:touchdown_spec), "    touchdown_spec(effector, spacecraft_index::Int)")
        # The RPO text stays on the RPO method.
        gh = HP_SM.GuidanceHooks
        rpo_sig = Tuple{HP_SM.GuidanceModels.RPOGuidanceModel, Any, Any, Float64, Int}
        rpo_doc = Docs.meta(gh)[Docs.Binding(gh, :calcGuidanceEffect!)].docs[rpo_sig]
        @test occursin("model.chaser_idx", string(rpo_doc.text...))

        # No catch-all navigation method: an effector without one is still a MethodError when called.
        @test_throws MethodError SpaceAGORA.calcNavigationEffect!(HpNavigationNoHook(), Float64[], nothing, 0.0, 1)
    end

    @testset "built-in models pass the check" begin
        guidance = hp_package_types(SpaceAGORA.AbstractGuidanceModel)
        @test length(guidance) >= 5
        for T in guidance
            @test hp_tick_ok(SpaceAGORA.calcGuidanceEffect!, T)
        end
        # SolarPanelAngleOfAttackControlModel is the panel actuator inside the
        # energy depletion control model and has no control tick of its own.
        aoa = SpaceAGORA.SolarPanelAngleOfAttackControlModel
        control = [hp_package_types(SpaceAGORA.AbstractControlEffectorModel); HP_SM.ThrusterModels.BaseThrusterModel]
        @test aoa in control
        @test length(control) >= 6
        for T in control
            @test hp_tick_ok(SpaceAGORA.calcControlEffect!, T) == (T !== aoa)
            @test HP_SE._hook_method_may_apply(SpaceAGORA.calcControlForceTorque, Tuple{T, AbstractVector, HP_P, Int, Float64})
        end
        forces = hp_package_types(SpaceAGORA.AbstractForceTorqueModel)
        @test length(forces) >= 15
        for T in forces
            wrench_ok = hasmethod(SpaceAGORA.wrench, Tuple{T, SpaceAGORA.StateSample, SpaceAGORA.EnvironmentSample, Float64})
            legacy_ok = HP_SE._hook_method_may_apply(SpaceAGORA.calcForceTorque, Tuple{T, AbstractVector, HP_P, Int})
            @test wrench_ok || legacy_ok
        end
        # The GRAM-backed density models get getDensity from the GRAMSuite extension.
        em = HP_SM.EnvironmentModels
        gram = (em.GRAMAtmosphereModel, em.GRAMAtmosphereModelSurrogate, em.GRAMGridAtmosphereModel, em.GRAMNearSurfaceAtmosphereModel)
        gram_loaded = Base.get_extension(SpaceAGORA, :SpaceAGORAGRAMSuiteExt) !== nothing
        densities = hp_package_types(SpaceAGORA.AbstractDensityModel)
        @test length(densities) >= 10
        for T in densities
            ok = HP_SE._hook_method_may_apply(SpaceAGORA.getDensity, Tuple{T, Float64, Float64, Float64, Float64, Bool, HP_P})
            @test ok == (gram_loaded || !any(G -> Base.unwrap_unionall(T) <: G, gram))
        end
    end

    @testset "missing hooks are named" begin
        args = hp_args(
            guidance=(HpGuidanceNoHook(), HpGuidanceWrongTime(), HpGuidanceCounter()),
            navigation=(HpNavigationNoHook(), HpNavigationCounter()),
            control=(HpControlTickOnly(), HpMountedControl()),
            effectors=(HpForceNoHook(), HpLegacyForceCounter(0), HP_SM.InverseSquaredJ2GravityModel()),
            density=HpDensitySixArgs(),
        )
        lines = HP_SE._missing_model_hooks(args)
        has(parts...) = any(line -> all(part -> occursin(part, line), parts), lines)
        @test length(lines) == 6
        @test has("guidance_model.guidance_effectors[1] (HpGuidanceNoHook)", "calcGuidanceEffect!(model, u, p, t::Float64, spacecraft_index::Int)")
        @test has("guidance_model.guidance_effectors[2] (HpGuidanceWrongTime)", "calcGuidanceEffect!")
        @test has("navigation_model.navigation_effectors[1] (HpNavigationNoHook)", "calcNavigationEffect!")
        @test has("control_model.control_effectors[1] (HpControlTickOnly)", "calcControlForceTorque", "actuation_mode :ideal")
        @test has("dynamics_model.dynamic_effectors[1] (HpForceNoHook)", "wrench(", "calcForceTorque(")
        @test has("environment_model.density_model (HpDensitySixArgs)", "getDensity(", "wind::Bool, p)")
        # A :mounted effector needs no calcControlForceTorque, and calcForceTorque alone serves a force model.
        for accepted in ("HpGuidanceCounter", "HpNavigationCounter", "HpMountedControl", "HpLegacyForceCounter", "InverseSquaredJ2GravityModel")
            @test !has(accepted)
        end

        err = try
            HP_SE._validate_model_hooks!(args)
            nothing
        catch e
            e
        end
        @test err isa ArgumentError
        @test all(line -> occursin(line, err.msg), lines)

        @test HP_SE._validate_model_hooks!(hp_args(
            guidance=(HpGuidanceCounter(),), navigation=(HpNavigationCounter(),), control=(HpMountedControl(),),
            effectors=(HpLegacyForceCounter(0),), density=HpDensitySevenArgs(),
        )) === nothing

        # Backbone-only gravity passes only where gravity_backbone_split evaluates it;
        # a per-link model passes in any mode.
        special = hp_args(effectors=(HpBackboneOnly(), HpLinkOnly()))
        @test occursin("dynamic_effectors[1] (HpBackboneOnly) has no method wrench(", only(HP_SE._missing_model_hooks(special)))
        @test isempty(HP_SE._missing_model_hooks(special, :gravity_backbone_split))

        # A model in the wrong container is pointed to its own.
        @test occursin("Its type belongs in control_model.", only(HP_SE._missing_model_hooks(hp_args(guidance=(HpControlTickOnly(),)))))
        # The energy depletion controller's panel actuator cannot run on its own.
        aoa_line = only(HP_SE._missing_model_hooks(hp_args(control=(SpaceAGORA.SolarPanelAngleOfAttackControlModel(),))))
        @test occursin("(SolarPanelAngleOfAttackControlModel) has no method calcControlEffect!", aoa_line)
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
        @test occursin("guidance_model.guidance_effectors[1] (HpGuidanceNoHook) has no method calcGuidanceEffect!", err.msg)
        # Nothing was integrated: no dynamics evaluation and no navigation tick.
        @test force.calls == 0
        @test navigation.calls == 0

        # With the hook present the same run integrates, through the older calcForceTorque path too.
        guidance = HpGuidanceCounter()
        run_simulation(hp_run_config(guidance=(guidance,), navigation=(navigation,), effectors=(force,)); isolate_state=false)
        @test force.calls > 0
        @test guidance.calls > 0 && navigation.calls > 0
        # The first tick comes one period after the start, as the docstrings say.
        @test guidance.first_t == 10.0
        @test navigation.first_t == 15.0
    end
end
