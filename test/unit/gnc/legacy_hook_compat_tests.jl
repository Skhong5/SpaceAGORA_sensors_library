# The deprecated control output hooks (calcControlForceTorque,
# calcControlMassFlowRate, calcReactionWheelTorque), ticks
# (calcControlEffect!, calcGuidanceEffect!, calcNavigationEffect!) and
# model-last ephemerides hooks (planet_frame_lpi, ephemerides_sun_direction_ii,
# ephemerides_time_seconds), which SpaceAGORA 0.5.0 removes. The package, its
# templates, its examples and the other tests use control_wrench,
# control_propellant_rate, control_wheel_torque, update_control!,
# update_guidance!, update_navigation!, ephemerides_frame_lpi,
# ephemerides_sun_ii and ephemerides_epoch_seconds; the old-style probes stay
# here, with their warnings, until the removal.
module LegacyHookCompatTests
using Test, Logging, LinearAlgebra, StaticArrays, ComponentArrays, SpaceAGORA
const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const CH = SM.ControlHooks
const ZERO3 = SVector(0.0, 0.0, 0.0)

# A thruster along the velocity, a body-rate damper and a propellant use that
# grows with time, written once with the deprecated hooks, once with the new
# ones, and once mixing both (the new wrench, the old mass rate).
abstract type TwinThruster <: SM.AbstractControlEffectorModel end
for T in (:OldStyleThruster, :NewStyleThruster, :MixedStyleThruster)
    @eval struct $T <: TwinThruster
        thrust_n::Float64
        gain::Float64
        propellant_kg_s::Float64
    end
end
SM.update_control!(::TwinThruster, u, p, t::Float64, i::Int) = nothing
_rate(m, t) = m.propellant_kg_s * (1.0 + 1.0e-3 * t)

function CH.calcControlForceTorque(m::OldStyleThruster, u::AbstractVector, p::SM.ODEParams, i::Int64, t::Float64)
    v = SVector{3, Float64}(u.vel)
    return m.thrust_n * v / norm(v), -m.gain * SVector{3, Float64}(u.ω)
end
# The mass derivative, negative while propellant is used.
CH.calcControlMassFlowRate(m::Union{OldStyleThruster, MixedStyleThruster}, u::AbstractVector, p::SM.ODEParams, i::Int64, t::Float64) =
    -_rate(m, t)

function CH.control_wrench(m::Union{NewStyleThruster, MixedStyleThruster}, x::SM.StateSample, p::SM.ODEParams, t::Float64)
    v = x.vel_ii
    return m.thrust_n * v / norm(v), -m.gain * x.ω_body
end
# The propellant used, positive.
CH.control_propellant_rate(m::NewStyleThruster, x::SM.StateSample, p::SM.ODEParams, t::Float64) = _rate(m, t)

# Both forms of the wrench: the dynamics call the new one.
struct BothStyleThruster <: SM.AbstractControlEffectorModel end
SM.update_control!(::BothStyleThruster, u, p, t::Float64, i::Int) = nothing
CH.calcControlForceTorque(::BothStyleThruster, u, p, i, t) = (SVector(1.0, 0.0, 0.0), ZERO3)
CH.control_wrench(::BothStyleThruster, x, p, t) = (ZERO3, ZERO3)
# A control tick and nothing else, in ideal and in mounted actuation.
struct NoOutputHooks <: SM.AbstractControlEffectorModel end
SM.update_control!(::NoOutputHooks, u, p, t::Float64, i::Int) = nothing
struct MountedNoOutputHooks <: SM.AbstractControlEffectorModel end
SM.update_control!(::MountedNoOutputHooks, u, p, t::Float64, i::Int) = nothing
SM.actuation_mode(::MountedNoOutputHooks) = :mounted

# All three old hooks, untyped but for the effector, recording the calls.
struct OldStyleProbe{ID} <: SM.AbstractControlEffectorModel
    rate::Float64
    calls::Vector{Tuple{Symbol, Symbol, Int}}
end
CH.calcControlForceTorque(m::OldStyleProbe{ID}, u, p, i, t) where {ID} =
    (push!(m.calls, (ID, :force, i)); (SVector(1.0, 0.0, 0.0), ZERO3))
CH.calcControlMassFlowRate(m::OldStyleProbe{ID}, u, p, i, t) where {ID} =
    (push!(m.calls, (ID, :mass, i)); m.rate)
CH.calcReactionWheelTorque(m::OldStyleProbe{ID}, u, p, i, t) where {ID} =
    (push!(m.calls, (ID, :wheel, i)); SVector(0.0, 0.0, 1.0))

# GNC ticks written with the deprecated names (`OLD = true`) and with the new
# ones. Navigation records what it sees, guidance sets a target rate from the
# position, and control holds a torque toward that rate, so the ticks steer the
# solve.
struct TickNav{OLD} <: SpaceAGORA.AbstractNavigationModel
    seen::Vector{Float64}
end
struct TickGuide{OLD} <: SpaceAGORA.AbstractGuidanceModel
    target::Vector{SVector{3, Float64}}
end
struct TickControl{OLD} <: SM.AbstractControlEffectorModel
    guide::TickGuide{OLD}
    torque::Vector{SVector{3, Float64}}
end
_nav_tick!(m, u, t, i) = (push!(m.seen, t + u.sc[i].pos[1]); nothing)
_guide_tick!(m, u, i) = (m.target[i] = 1.0e-3 * normalize(SVector{3, Float64}(u.sc[i].pos)); nothing)
_control_tick!(m, u, i) = (m.torque[i] = -0.2 * (SVector{3, Float64}(u.sc[i].ω) - m.guide.target[i]); nothing)
SM.calcNavigationEffect!(m::TickNav{true}, u, p, t::Float64, i::Int) = _nav_tick!(m, u, t, i)
SM.update_navigation!(m::TickNav{false}, u, p, t::Float64, i::Int) = _nav_tick!(m, u, t, i)
SM.calcGuidanceEffect!(m::TickGuide{true}, u, p, t::Float64, i::Int) = _guide_tick!(m, u, i)
SM.update_guidance!(m::TickGuide{false}, u, p, t::Float64, i::Int) = _guide_tick!(m, u, i)
SM.calcControlEffect!(m::TickControl{true}, u, p, t::Float64, i::Int) = _control_tick!(m, u, i)
SM.update_control!(m::TickControl{false}, u, p, t::Float64, i::Int) = _control_tick!(m, u, i)
SM.control_wrench(m::TickControl, x::SM.StateSample, p, t) = (ZERO3, m.torque[x.spacecraft_index])
function ticks(old::Bool)
    guide = TickGuide{old}([ZERO3])
    return (TickNav{old}(Float64[]), guide, TickControl{old}(guide, [ZERO3]))
end
# Guidance ticks for the pre-run check: neither name, the old one only (and one
# typed for a state the run does not pass), the new one only, and both.
struct GuideNeither <: SpaceAGORA.AbstractGuidanceModel end
struct GuideOld <: SpaceAGORA.AbstractGuidanceModel end
SM.calcGuidanceEffect!(::GuideOld, u, p, t::Float64, i::Int) = nothing
struct GuideOldNarrow <: SpaceAGORA.AbstractGuidanceModel end
SM.calcGuidanceEffect!(::GuideOldNarrow, u::Vector{Float64}, p, t::Float64, i::Int) = nothing
struct GuideNew <: SpaceAGORA.AbstractGuidanceModel end
SM.update_guidance!(::GuideNew, u, p, t::Float64, i::Int) = nothing
struct GuideBoth <: SpaceAGORA.AbstractGuidanceModel end
SM.calcGuidanceEffect!(::GuideBoth, u, p, t::Float64, i::Int) = error("the old tick must not run")
SM.update_guidance!(::GuideBoth, u, p, t::Float64, i::Int) = nothing

# An ephemerides model written with the model-last names (`OLD = true`) and with
# the model-first ones, both answering as the simple model and counting the
# frames they give. The keys differ, so neither run reuses the other's caches.
const EPH_INNER = SpaceAGORA.SimpleEphemeridesModel()
struct TwinEphemerides{OLD} <: SpaceAGORA.AbstractEphemeridesModel
    frames::Base.Threads.Atomic{Int}
end
TwinEphemerides{OLD}() where {OLD} = TwinEphemerides{OLD}(Base.Threads.Atomic{Int}(0))
SM.ephemerides_requires_spice(::TwinEphemerides) = false
SM.ephemerides_cache_key(::TwinEphemerides{OLD}) where {OLD} = (:twin, OLD ? 1 : 2, 0)
_frame(m, planet, et) = (Base.Threads.atomic_add!(m.frames, 1); SM.ephemerides_frame_lpi(EPH_INNER, planet, et))
SM.ephemerides_time_seconds(it, ::TwinEphemerides{true}) = SM.ephemerides_epoch_seconds(EPH_INNER, it)
SM.planet_frame_lpi(planet, et::Float64, m::TwinEphemerides{true}) = _frame(m, planet, et)
SM.ephemerides_sun_direction_ii(planet, et::Float64, ::TwinEphemerides{true}) = SM.ephemerides_sun_ii(EPH_INNER, planet, et)
SM.ephemerides_epoch_seconds(::TwinEphemerides{false}, it) = SM.ephemerides_epoch_seconds(EPH_INNER, it)
SM.ephemerides_frame_lpi(m::TwinEphemerides{false}, planet, et::Float64) = _frame(m, planet, et)
SM.ephemerides_sun_ii(::TwinEphemerides{false}, planet, et::Float64) = SM.ephemerides_sun_ii(EPH_INNER, planet, et)
# A model with both frame names, and one with neither hook.
struct BothFrames <: SpaceAGORA.AbstractEphemeridesModel end
SM.ephemerides_frame_lpi(::BothFrames, planet, et::Float64) = SM.ephemerides_frame_lpi(EPH_INNER, planet, et)
SM.planet_frame_lpi(planet, et::Float64, ::BothFrames) = error("the old frame hook must not run")
struct NoEphemerides <: SpaceAGORA.AbstractEphemeridesModel end
# A small force along the planet-fixed x-axis, so the solution follows the frame.
struct FrameProbeForce <: SM.AbstractForceTorqueModel end
SpaceAGORA.environment_requirements(::FrameProbeForce) = SpaceAGORA.EffectorEnvironmentRequirements(planet_frame=true)
SpaceAGORA.wrench(::FrameProbeForce, x::SpaceAGORA.StateSample, env::SpaceAGORA.EnvironmentSample, t::Float64) =
    (1.0e-3 * (transpose(env.planet_frame.l_pi) * SVector(1.0, 0.0, 0.0)), ZERO3)

function twin_args(control::Tuple; n_spacecraft::Int=1, mission_time=120.0, guidance::Tuple=(), navigation::Tuple=(),
        effectors::Tuple=(SM.InverseSquaredJ2GravityModel(),), ephemerides=SpaceAGORA.SimpleEphemeridesModel())
    planet = SM.make_no_gram_planet(:earth)
    spacecraft = map(1:n_spacecraft) do k
        root = SM.Link(root=true, m=100.0, dims=MVector(1.0, 1.2, 0.8))
        ic = SM.InitialCondition(ra=planet.Rp_e + 700e3, rp=planet.Rp_e + 650e3, i=45.0, ω=0.0, Ω=0.0,
            ν=10.0 * k, q=normalize(SVector(0.1, -0.2, 0.3, 0.9)), ang_vel=SVector(0.01, -0.02, 0.015))
        SM.SpacecraftModel(joints=SM.Joint[], links=[root], root=root, prop_mass=10.0,
            inertia_tensor=root.inertia, initial_condition=ic, id=k)
    end
    env = SM.EnvironmentModel(planet=planet, EI=120.0, density_model=SpaceAGORA.NoAtmosphereModel(),
        ephemerides_model=ephemerides,
        thermal_model=SM.MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet),
        topography=false, wind=false)
    return SM.SimulationConfiguration(
        simulation_settings=SM.SimulationSettings(results=false, verbose=false, generate_plots=false, normalize=false),
        mission_configuration=SM.MissionConfiguration(mission_type=SM.MissionTime, keplerian=true,
            number_of_orbits=1, mission_time=mission_time, orientation_sim=true, num_steps_to_save=50),
        environment_model=env,
        dynamics_model=SM.DynamicsModel(SM.SpacecraftModel[spacecraft...], effectors),
        guidance_model=SM.GuidanceModel(guidance, fill(10.0, length(guidance))),
        navigation_model=SM.NavigationModel(navigation, fill(5.0, length(navigation))),
        control_model=SM.ControlModel(control, fill(10.0, length(control))),
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0))
end

# The configuration parts the pre-run check reads.
check_args(control::Tuple; guidance::Tuple=(), ephemerides=SpaceAGORA.SimpleEphemeridesModel()) = (
    guidance_model=SM.GuidanceModel(guidance, fill(1.0, length(guidance))),
    navigation_model=SM.NavigationModel((), Float64[]),
    control_model=SM.ControlModel(control, fill(1.0, length(control))),
    dynamics_model=(dynamic_effectors=(),),
    environment_model=(density_model=SpaceAGORA.NoAtmosphereModel(), ephemerides_model=ephemerides,
        planet=SM.make_no_gram_planet(:earth)),
    initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
)

solution_bits(sol) = (reinterpret(UInt64, collect(sol.t)), [reinterpret(UInt64, Vector{Float64}(u)) for u in sol.u])
const OLD_HOOKS = (:calcControlForceTorque, :calcControlMassFlowRate, :calcReactionWheelTorque,
    :calcControlEffect!, :calcGuidanceEffect!, :calcNavigationEffect!,
    :planet_frame_lpi, :ephemerides_sun_direction_ii, :ephemerides_time_seconds)
depwarns(logs) = filter(r -> r.group === :depwarn && r.id isa Tuple && length(r.id) == 2 && r.id[2] in OLD_HOOKS, logs)

# Solve under a logger that records every log call, without the once-per-key
# limit, so a warning from the right-hand side would show once per evaluation.
function logged_solve(args)
    logger = Test.TestLogger(min_level=Logging.Info, respect_maxlog=false)
    sol = with_logger(() -> run_simulation(args; return_solution=true), logger)
    return sol, logger.logs
end

const TWIN = (thrust_n=0.5, gain=0.2, propellant_kg_s=1.0e-3)
twin(T) = T(TWIN.thrust_n, TWIN.gain, TWIN.propellant_kg_s)

@testset "Deprecated control output hooks, ticks and ephemerides hooks" begin
    @testset "a legacy effector and its new-style twin give identical solutions" begin
        sol_new, logs_new = logged_solve(twin_args((twin(NewStyleThruster),)))
        sol_old, logs_old = logged_solve(twin_args((twin(OldStyleThruster),)))
        sol_mixed, logs_mixed = logged_solve(twin_args((twin(MixedStyleThruster),)))
        @test solution_bits(sol_old) == solution_bits(sol_new)
        @test solution_bits(sol_mixed) == solution_bits(sol_new)
        # The runs used propellant and turned, so the mass and attitude paths ran.
        @test sol_new.u[end].sc[1].mass < sol_new.u[1].sc[1].mass
        @test sol_new.u[end].sc[1].ω != sol_new.u[1].sc[1].ω
        # One warning per type and deprecated hook, from the check before
        # integration; none from the solve, which evaluated the hooks many times.
        @test isempty(depwarns(logs_new))
        @test [r.id for r in depwarns(logs_old)] ==
            [(OldStyleThruster, :calcControlForceTorque), (OldStyleThruster, :calcControlMassFlowRate)]
        @test [r.id for r in depwarns(logs_mixed)] == [(MixedStyleThruster, :calcControlMassFlowRate)]
        mass_warning = string(only(filter(r -> r.id == (OldStyleThruster, :calcControlMassFlowRate), depwarns(logs_old))).message)
        @test occursin("control_propellant_rate returns the propellant used, positive", mass_warning)
    end

    @testset "the pre-run check: neither, old only, new only, both" begin
        args = twin_args((twin(NewStyleThruster),))
        u0 = SE.build_initial_conditions(args)
        call = SE._hook_call_types(u0, SM.ODEParams(n_sats=1, args=args))
        control = (NoOutputHooks(), twin(OldStyleThruster), twin(NewStyleThruster), BothStyleThruster(),
            twin(OldStyleThruster))
        lines = SE._missing_model_hooks(check_args(control), call)
        @test length(lines) == 1
        @test occursin("control_model.control_effectors[1] (NoOutputHooks) has no method control_wrench(model, x::StateSample, p, t::Float64)", only(lines))
        found = SE._deprecated_model_hooks(check_args(control), call)
        @test first.(found) == [(OldStyleThruster, :calcControlForceTorque), (OldStyleThruster, :calcControlMassFlowRate),
            (BothStyleThruster, :calcControlForceTorque)]
        @test occursin("OldStyleThruster implements calcControlForceTorque(model, u, p, i, t), which is deprecated", last(found[1]))
        @test occursin("control_wrench(model, x::StateSample, p, t::Float64) instead", last(found[1]))
        @test occursin("x.spacecraft_index", last(found[1]))
        @test occursin("BothStyleThruster implements both control_wrench and calcControlForceTorque", last(found[3]))
        @test occursin("never used", last(found[3]))
        # A :mounted effector needs no wrench hook.
        @test isempty(SE._missing_model_hooks(check_args((MountedNoOutputHooks(),)), call))
        # The built-in effectors give no warning.
        manager = SM.MagneticMomentumManagerModel(spacecraft_index=1)
        @test isempty(SE._deprecated_model_hooks(check_args((manager, SM.RPOMPCControlModel())), call))
    end

    @testset "old-style hooks keep their sign and order through the dynamics" begin
        calls = Tuple{Symbol, Symbol, Int}[]
        ids = (:large, :cancel, :unit, :nan, :positive_inf, :negative_inf)
        rates = (1.0e16, -1.0e16, 1.0, NaN, Inf, -Inf)   # mass derivatives, as the old hook returns them
        control = ntuple(k -> OldStyleProbe{ids[k]}(rates[k], calls), length(ids))
        p = SM.ODEParams(n_sats=1, args=twin_args(control))
        state = SE.build_initial_conditions(p.args).sc[1]
        f, tq, rw = MVector{3, Float64}(ZERO3), MVector{3, Float64}(ZERO3), MVector{3, Float64}(ZERO3)
        rate = SE._accumulate_control_effectors!(f, tq, rw, state, p, 1, 2.0, false)
        # The old accumulation, ((0 + 1e16) + (-1e16)) + 1, bit for bit.
        @test rate === 1.0
        @test SVector{3, Float64}(f) == SVector(6.0, 0.0, 0.0)
        @test SVector{3, Float64}(rw) == SVector(0.0, 0.0, 6.0)
        @test calls == [(id, hook, 1) for id in ids for hook in (:force, :mass, :wheel)]
        # An old-style method typed on the effector alone is not ambiguous with
        # the untyped fallbacks, and a direct call reaches it without a warning.
        @test !any(Base.isambiguous(a, b) for f in (CH.calcControlForceTorque, CH.calcControlMassFlowRate,
            CH.calcReactionWheelTorque) for a in methods(f), b in methods(f) if a !== b)
        @test (@test_logs CH.calcControlMassFlowRate(control[1], state, p, 1, 0.0)) === 1.0e16
    end

    @testset "old-style ticks and their new-style twins give identical runs" begin
        old_nav, old_guide, old_control = ticks(true)
        new_nav, new_guide, new_control = ticks(false)
        sol_old, logs_old = logged_solve(twin_args((old_control,); guidance=(old_guide,), navigation=(old_nav,)))
        sol_new, logs_new = logged_solve(twin_args((new_control,); guidance=(new_guide,), navigation=(new_nav,)))
        @test solution_bits(sol_old) == solution_bits(sol_new)
        # Each tick ran, at the same times on the same states.
        nav_old = only(sol_old.prob.p.args.navigation_model.navigation_effectors)
        nav_new = only(sol_new.prob.p.args.navigation_model.navigation_effectors)
        @test !isempty(nav_new.seen) && reinterpret(UInt64, nav_old.seen) == reinterpret(UInt64, nav_new.seen)
        @test sol_new.u[end].sc[1].ω != sol_new.u[1].sc[1].ω
        # One warning per type and old tick, from the check before integration.
        @test isempty(depwarns(logs_new))
        @test Set(r.id for r in depwarns(logs_old)) == Set([(TickGuide{true}, :calcGuidanceEffect!),
            (TickNav{true}, :calcNavigationEffect!), (TickControl{true}, :calcControlEffect!)])
        @test length(depwarns(logs_old)) == 3
        guide_warning = string(only(filter(r -> r.id == (TickGuide{true}, :calcGuidanceEffect!), depwarns(logs_old))).message)
        @test occursin("Rename the method update_guidance!: only the name changes.", guide_warning)
    end

    @testset "the pre-run check for ticks: neither, old only, new only, both" begin
        args = twin_args((twin(NewStyleThruster),))
        u0 = SE.build_initial_conditions(args)
        call = SE._hook_call_types(u0, SM.ODEParams(n_sats=1, args=args))
        guidance = (GuideNeither(), GuideOld(), GuideOldNarrow(), GuideNew(), GuideBoth())
        lines = SE._missing_model_hooks(check_args((); guidance), call)
        @test length(lines) == 2
        @test occursin("guidance_model.guidance_effectors[1] (GuideNeither) has no method update_guidance!(model, u, p, t::Float64, spacecraft_index::Int).", lines[1])
        @test !occursin("none accepts", lines[1])
        # A method of the old name for another state type is named as such.
        @test occursin("guidance_model.guidance_effectors[3] (GuideOldNarrow) has no method update_guidance!(", lines[2])
        @test occursin("It has methods of that function, but none accepts the arguments this run passes.", lines[2])
        found = SE._deprecated_model_hooks(check_args((); guidance), call)
        @test first.(found) == [(GuideOld, :calcGuidanceEffect!), (GuideBoth, :calcGuidanceEffect!)]
        @test occursin("GuideOld implements calcGuidanceEffect!(model, u, p, t, spacecraft_index), which is deprecated", last(found[1]))
        @test occursin("GuideBoth implements both update_guidance! and calcGuidanceEffect!", last(found[2]))
        @test occursin("never used", last(found[2]))
        # The built-in GNC effectors give no tick warning.
        manager = SM.MagneticMomentumManagerModel(spacecraft_index=1)
        @test isempty(SE._deprecated_model_hooks(check_args((manager, SM.RPOMPCControlModel())), call))
    end

    @testset "a direct old tick call runs the new tick and warns" begin
        manager_args = twin_args((); n_spacecraft=1)
        p = SM.ODEParams(n_sats=1, args=manager_args)
        u = SE.build_initial_conditions(manager_args)
        make() = SM.MagneticMomentumManagerModel(spacecraft_index=1, mu_gain=0.01, h_wheels_0=SVector(0.02, 0.0, 0.0),
            commanded_torque=(t, r, v, q, w) -> SVector(1.0e-6, 0.0, 0.0), b_field_ii=(t, r) -> SVector(0.0, 0.0, 3.0e-5))
        by_old, by_new = make(), make()
        warning = (:warn, r"^calcControlEffect!\(model, u, p, t, spacecraft_index\) is deprecated .* called directly on a MagneticMomentumManagerModel\.")
        @test_logs warning begin
            for t in (0.0, 1.0, 2.0)
                @test SpaceAGORA.calcControlEffect!(by_old, u, p, t, 1) === SpaceAGORA.update_control!(by_new, u, p, t, 1)
            end
        end
        @test by_old.held_torque_body === by_new.held_torque_body && by_old.h_wheels === by_new.h_wheels
        # A type with neither tick raises a MethodError naming the new tick.
        err = try
            SpaceAGORA.calcGuidanceEffect!(GuideNeither(), u, p, 0.0, 1)
        catch e
            e
        end
        @test err isa MethodError && err.f === SpaceAGORA.update_guidance!
    end

    @testset "a direct old-name call on a built-in returns the new hook's value and warns" begin
        six = SM.SixAxisThrusterModel(max_thrust_n=SVector{6}(fill(1.0, 6)), isp_s=SVector{6}(fill(65.0, 6)))
        rpo = SM.RPOMPCControlModel(chaser_index=1, target_index=2, thrusters=six)
        rpo.held = CH.RPOHeldActuation(force_ii=SVector(0.3, -0.1, 0.2), torque_body=SVector(1e-3, 0.0, -2e-3),
            thruster_forces_n=SVector{6}(0.4, 0.0, 0.0, 0.25, 0.1, 0.0), rw_torque_body=SVector(1e-4, -2e-4, 3e-4))
        manager = SM.MagneticMomentumManagerModel(spacecraft_index=2, held_torque_body=SVector(1e-5, 2e-5, -3e-5))
        args = twin_args((rpo, manager); n_spacecraft=2)
        p = SM.ODEParams(n_sats=2, args=args)
        u = SE.build_initial_conditions(args)
        x(i) = SE._control_state_sample(u.sc[i], p, i)
        warning(old, T) = (:warn, Regex("^$(old)\\(model, u, p, i, t\\) is deprecated .* called directly on a $(nameof(T))\\."))
        @test_logs warning(:calcControlForceTorque, SM.RPOMPCControlModel) warning(:calcControlMassFlowRate, SM.RPOMPCControlModel) warning(:calcReactionWheelTorque, SM.RPOMPCControlModel) warning(:calcControlForceTorque, SM.MagneticMomentumManagerModel) begin
            for i in 1:2, t in (0.0, 5.0)
                @test SpaceAGORA.calcControlForceTorque(rpo, u.sc[i], p, i, t) === SpaceAGORA.control_wrench(rpo, x(i), p, t)
                @test SpaceAGORA.calcControlMassFlowRate(rpo, u.sc[i], p, i, t) === -SpaceAGORA.control_propellant_rate(rpo, x(i), p, t)
                @test SpaceAGORA.calcReactionWheelTorque(rpo, u.sc[i], p, i, t) === SpaceAGORA.control_wheel_torque(rpo, x(i), p, t)
                @test SpaceAGORA.calcControlForceTorque(manager, u.sc[i], p, i, t) === SpaceAGORA.control_wrench(manager, x(i), p, t)
            end
            @test SpaceAGORA.calcControlMassFlowRate(rpo, u.sc[1], p, 1, 0.0) < 0.0
            @test SpaceAGORA.calcControlForceTorque(manager, u.sc[2], p, 2, 0.0) == (ZERO3, manager.held_torque_body)
            # Without the new hook the defaults remain, with no warning.
            @test SpaceAGORA.calcControlMassFlowRate(manager, u.sc[2], p, 2, 0.0) === 0.0
            @test SpaceAGORA.calcReactionWheelTorque(manager, u.sc[2], p, 2, 0.0) === nothing
        end
        # A type with neither hook raises a MethodError naming the new hook.
        err = try
            SpaceAGORA.calcControlForceTorque(NoOutputHooks(), u.sc[1], p, 1, 0.0)
        catch e
            e
        end
        @test err isa MethodError && err.f === SpaceAGORA.control_wrench
        # Under --depwarn=error each direct call, of an output hook or a tick,
        # throws before evaluating anything.
        strict = """
            using SpaceAGORA
            struct NewOnly <: SpaceAGORA.AbstractControlEffectorModel end
            SpaceAGORA.control_wrench(::NewOnly, x, p, t) = error("evaluated")
            SpaceAGORA.control_propellant_rate(::NewOnly, x, p, t) = error("evaluated")
            SpaceAGORA.control_wheel_torque(::NewOnly, x, p, t) = error("evaluated")
            thrown = map((SpaceAGORA.calcControlForceTorque, SpaceAGORA.calcControlMassFlowRate,
                    SpaceAGORA.calcReactionWheelTorque)) do f
                try
                    f(NewOnly(), nothing, nothing, 1, 0.0)
                    false
                catch err
                    err isa ErrorException && occursin("is deprecated", err.msg)
                end
            end
            struct NewTicks end
            SpaceAGORA.update_control!(::NewTicks, u, p, t, i) = error("ticked")
            SpaceAGORA.update_guidance!(::NewTicks, u, p, t, i) = error("ticked")
            SpaceAGORA.update_navigation!(::NewTicks, u, p, t, i) = error("ticked")
            ticks_thrown = map((SpaceAGORA.calcControlEffect!, SpaceAGORA.calcGuidanceEffect!,
                    SpaceAGORA.calcNavigationEffect!)) do f
                try
                    f(NewTicks(), nothing, nothing, 0.0, 1)
                    false
                catch err
                    err isa ErrorException && occursin("is deprecated", err.msg)
                end
            end
            struct NewEphemerides <: SpaceAGORA.AbstractEphemeridesModel end
            SpaceAGORA.ephemerides_frame_lpi(::NewEphemerides, planet, et) = error("evaluated")
            SpaceAGORA.ephemerides_sun_ii(::NewEphemerides, planet, et) = error("evaluated")
            SpaceAGORA.ephemerides_epoch_seconds(::NewEphemerides, initial_time) = error("evaluated")
            ephemerides_thrown = map(((SpaceAGORA.planet_frame_lpi, (nothing, 0.0)),
                    (SpaceAGORA.ephemerides_sun_direction_ii, (nothing, 0.0)), (SpaceAGORA.ephemerides_time_seconds, (nothing,)))) do (f, leading)
                try
                    f(leading..., NewEphemerides())
                    false
                catch err
                    err isa ErrorException && occursin("is deprecated", err.msg)
                end
            end
            print(all(thrown) && all(ticks_thrown) && all(ephemerides_thrown))
            """
        @test read(`$(Base.julia_cmd()) --startup-file=no --depwarn=error --project=$(pkgdir(SpaceAGORA)) -e $strict`, String) == "true"
    end

    @testset "old-style ephemerides models and their new-style twins give identical runs" begin
        effectors = (SM.InverseSquaredJ2GravityModel(), FrameProbeForce())
        sol_old, logs_old = logged_solve(twin_args((); effectors, ephemerides=TwinEphemerides{true}()))
        sol_new, logs_new = logged_solve(twin_args((); effectors, ephemerides=TwinEphemerides{false}()))
        @test solution_bits(sol_old) == solution_bits(sol_new)
        # Each run took its frames from its own model, as often as the other.
        frames(sol) = sol.prob.p.args.environment_model.ephemerides_model.frames[]
        @test frames(sol_new) > 0 && frames(sol_old) == frames(sol_new)
        # One warning per old name, from the check before integration.
        @test isempty(depwarns(logs_new))
        T = TwinEphemerides{true}
        @test Set(r.id for r in depwarns(logs_old)) ==
            Set([(T, :planet_frame_lpi), (T, :ephemerides_sun_direction_ii), (T, :ephemerides_time_seconds)])
        @test length(depwarns(logs_old)) == 3
        frame_warning = string(only(filter(r -> r.id == (T, :planet_frame_lpi), depwarns(logs_old))).message)
        @test occursin("Rename the method ephemerides_frame_lpi and put the model first: ephemerides_frame_lpi(model, planet, et).", frame_warning)
    end

    @testset "the pre-run check for ephemerides: old only, new only, both, built-in" begin
        args = twin_args((twin(NewStyleThruster),))
        u0 = SE.build_initial_conditions(args)
        call = SE._hook_call_types(u0, SM.ODEParams(n_sats=1, args=args))
        found(model) = SE._deprecated_model_hooks(check_args((); ephemerides=model), call)
        T = TwinEphemerides{true}
        old = found(T())
        @test first.(old) == [(T, :planet_frame_lpi), (T, :ephemerides_sun_direction_ii), (T, :ephemerides_time_seconds)]
        @test occursin("TwinEphemerides implements planet_frame_lpi(planet, et, model), which is deprecated", last(old[1]))
        @test occursin("ephemerides_epoch_seconds(model, initial_time).", last(old[3]))
        @test isempty(found(TwinEphemerides{false}()))
        both = only(found(BothFrames()))
        @test first(both) == (BothFrames, :planet_frame_lpi)
        @test occursin("BothFrames implements both ephemerides_frame_lpi(model, planet, et) and planet_frame_lpi(planet, et, model).", last(both))
        @test occursin("never used", last(both))
        for builtin in (SpaceAGORA.SimpleEphemeridesModel(), SM.SpiceEphemeridesModel())
            @test isempty(found(builtin))
        end
    end

    @testset "a direct old-name ephemerides call returns the new hook's value and warns" begin
        planet = SM.make_no_gram_planet(:earth)
        it = SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0)
        simple = SpaceAGORA.SimpleEphemeridesModel()
        et = SpaceAGORA.ephemerides_epoch_seconds(simple, it)
        frame = (:warn, r"^planet_frame_lpi\(planet, et, model\) is deprecated .* called directly on a SimpleEphemeridesModel\.")
        sun = (:warn, r"^ephemerides_sun_direction_ii\(planet, et, model\) is deprecated .* called directly on a SimpleEphemeridesModel\.")
        epoch = (:warn, r"^ephemerides_time_seconds\(initial_time, model\) is deprecated .* called directly on a SimpleEphemeridesModel\.")
        @test_logs frame sun epoch begin
            for t in (0.0, 60.0)
                @test SpaceAGORA.planet_frame_lpi(planet, et + t, simple) === SpaceAGORA.ephemerides_frame_lpi(simple, planet, et + t)
                @test SpaceAGORA.ephemerides_sun_direction_ii(planet, et + t, simple) === SpaceAGORA.ephemerides_sun_ii(simple, planet, et + t)
            end
            @test SpaceAGORA.ephemerides_time_seconds(it, simple) === et
        end
        # A model's own old method answers a direct call without a warning, and
        # the engine's epoch lookup takes it.
        @test (@test_logs SpaceAGORA.ephemerides_time_seconds(it, TwinEphemerides{true}())) === et
        @test SE._ephemerides_time_seconds_flexible(it, TwinEphemerides{true}()) === et
        # A model with neither name raises a MethodError naming the new hook.
        caught(f) = try
            f()
        catch e
            e
        end
        err = caught(() -> SpaceAGORA.planet_frame_lpi(planet, et, NoEphemerides()))
        @test err isa MethodError && err.f === SpaceAGORA.ephemerides_frame_lpi
        err = caught(() -> SE._ephemerides_time_seconds_flexible(it, NoEphemerides()))
        @test err isa MethodError && err.f === SpaceAGORA.ephemerides_epoch_seconds
    end
end
end # module LegacyHookCompatTests
