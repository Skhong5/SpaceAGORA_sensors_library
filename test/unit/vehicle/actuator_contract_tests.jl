module ActuatorContractTests
using Test, StaticArrays, LinearAlgebra, Random, ComponentArrays, SpaceAGORA
import SpaceAGORA.TelemetryVerification: make_example_config
const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const CH = SM.ControlHooks
const G0 = 9.80665
const ZERO3 = SVector(0.0, 0.0, 0.0)

# A user-defined actuator kind, written the way a user would: a cold-gas puck
# that pushes along a fixed axis of its link.
struct ColdGasPuck <: AbstractActuator
    axis::SVector{3, Float64}
    location::SVector{3, Float64}
    max_force_n::Float64
    isp_s::Float64
end
SpaceAGORA.actuator_limits(a::ColdGasPuck) = (0.0, a.max_force_n)
function SpaceAGORA.actuator_wrench(a::ColdGasPuck, level, ctx::ActuatorContext)
    force = ctx.link_to_body * (level * a.axis)
    arm = ctx.link_to_body * a.location + ctx.link_r_body
    return force, cross(arm, force)
end
SpaceAGORA.actuator_consumption(a::ColdGasPuck, level, ctx) = (mass_rate_kg_s = level / (a.isp_s * G0), power_w = 0.0)

# A control effector that commands mounted actuators and writes no force,
# mass-flow or firing-level hook of its own.
mutable struct HoldLevels <: AbstractControlEffectorModel
    sat_idx::Int
    levels::Vector{Float64}
end
SpaceAGORA.actuation_mode(::HoldLevels) = :mounted
SpaceAGORA.actuator_levels(c::HoldLevels, i::Int) = i == c.sat_idx ? c.levels : nothing
SpaceAGORA.calcControlEffect!(::HoldLevels, u, p, t::Float64, i::Int64) = nothing

function readout_params(spacecraft::Vector{SM.SpacecraftModel}, effectors::Tuple)
    planet = SM.Earth()
    args = SM.SimulationConfiguration(
        simulation_settings=SM.SimulationSettings(results=false, verbose=false,
            generate_plots=false, normalize=false, save_csv=false),
        mission_configuration=SM.MissionConfiguration(mission_type=SM.MissionTime,
            keplerian=true, number_of_orbits=1, mission_time=1.0,
            orientation_sim=true, num_steps_to_save=1),
        environment_model=SM.EnvironmentModel(planet=planet, EI=120.0,
            density_model=SM.NoAtmosphereModel(), ephemerides_model=SM.SimpleEphemeridesModel(),
            thermal_model=SM.MaxwellianHeat(thermal_accomodation_factor=1.0, planet=planet),
            topography=false, wind=false),
        dynamics_model=SM.DynamicsModel(spacecraft, (SM.InverseSquaredGravityModel(),)),
        guidance_model=SM.GuidanceModel(guidance_effectors=(), guidance_rates=Float64[]),
        navigation_model=SM.NavigationModel(navigation_effectors=(), navigation_rates=Float64[]),
        control_model=SM.ControlModel(control_effectors=effectors, control_rates=fill(1.0, length(effectors))),
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0))
    return SM.ODEParams(n_sats=length(spacecraft), args=args)
end

function readout(p, i, state)
    f, tq, rw = MVector{3, Float64}(ZERO3), MVector{3, Float64}(ZERO3), MVector{3, Float64}(ZERO3)
    rate = SE._accumulate_control_effectors!(f, tq, rw, state, p, i, 2.0, false)
    return SVector{3, Float64}(f), SVector{3, Float64}(tq), SVector{3, Float64}(rw), rate
end

body_state(q; mass=500.0, ω=ZERO3) = ComponentVector(mass=mass, q=collect(q), ω=collect(ω))
spacecraft_with(root; id=1, links=[root]) = SM.SpacecraftModel(links=links, root=root, prop_mass=10.0, inertia_tensor=root.inertia, id=id)

include(joinpath(@__DIR__, "..", "..", "..", "templates", "actuator_template.jl"))

function wheel_spin_up(max_momentum)
    inertia = SMatrix{3, 3, Float64}(2.0, 0, 0, 0, 3.0, 0, 0, 0, 4.0)
    root = SM.Link(root=true, m=100.0, dims=MVector(1.0, 1.0, 1.0))
    ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), ZERO3; q=SVector(0.0, 0.0, 0.0, 1.0))
    sc = SM.SpacecraftModel(links=[root], root=root, prop_mass=1.0, inertia_tensor=inertia, initial_condition=ic, id=4)
    for axis in (SVector(1.0, 0.0, 0.0), SVector(0.0, 1.0, 0.0), SVector(0.0, 0.0, 1.0))
        add_actuator!(sc, root, ReactionWheel(spin_axis=axis, max_torque_nm=0.02, max_momentum_nms=max_momentum))
    end
    control = HoldLevels(1, [0.01, 0.0, 0.0])                # 0.01 N m on the body about +x
    base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=5.0,
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
        keplerian=false, EI_km=1.0, verbose=false, results=false)
    args = SM.SimConfig._with_configuration(base;
        control_model=SM.ControlModel(control_effectors=(control,), control_rates=[1.0]))
    return run_simulation(args; return_solution=true).u[end].sc[1], inertia
end


function pyramid_run(levels)
    s2 = 1 / sqrt(2)
    axes = (SVector(s2, 0.0, s2), SVector(-s2, 0.0, s2), SVector(0.0, s2, s2), SVector(0.0, -s2, s2))
    inertia = SMatrix{3, 3, Float64}(2.0, 0, 0, 0, 3.0, 0, 0, 0, 4.0)
    root = SM.Link(root=true, m=100.0, dims=MVector(1.0, 1.0, 1.0))
    ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), ZERO3; q=SVector(0.0, 0.0, 0.0, 1.0))
    sc = SM.SpacecraftModel(links=[root], root=root, prop_mass=1.0, inertia_tensor=inertia, initial_condition=ic, id=7)
    foreach(a -> add_actuator!(sc, root, ReactionWheel(spin_axis=a)), axes)
    base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=5.0,
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
        keplerian=false, EI_km=1.0, verbose=false, results=false)
    args = SM.SimConfig._with_configuration(base;
        control_model=SM.ControlModel(control_effectors=(HoldLevels(1, collect(levels)),), control_rates=[1.0]))
    return run_simulation(args; return_solution=true).u[end].sc[1], inertia, hcat(collect.(axes)...)
end

@testset "Actuator contract" begin
    @testset "a thruster's wrench, limits and propellant follow from its record" begin
        root = SM.Link(root=true, m=10.0)
        jet = Thruster(max_thrust=2.0, location=MVector(1.0, 0.0, 0.0), direction=MVector(0.0, 0.0, 2.0), Isp=200.0)
        ctx = actuator_context(root)
        force, torque = actuator_wrench(jet, 1.5, ctx)
        @test force == SVector(0.0, 0.0, 1.5)                 # along the (normalized) thrust direction
        @test torque == SVector(0.0, -1.5, 0.0)               # (1, 0, 0) x (0, 0, 1.5)
        @test actuator_consumption(jet, 1.5, ctx).mass_rate_kg_s == 1.5 / (200.0 * G0)
        @test actuator_consumption(jet, 1.5, ctx).power_w == 0.0
        @test actuator_limits(jet) == (0.0, 2.0)
        @test actuator_saturate(jet, 5.0) == 2.0
        @test actuator_saturate(jet, -1.0) == 0.0
        @test actuator_saturate(jet, NaN) == 0.0
        @test actuator_fraction(jet, 1.0) == 0.5
        @test actuator_unit_wrench(jet, ctx) == actuator_wrench(jet, 1.0, ctx)
        @test actuator_consumption(Thruster(Isp=0.0), 1.0, ctx).mass_rate_kg_s == 0.0
        @test actuator_wrench(Thruster(direction=MVector(0.0, 0.0, 0.0)), 1.0, ctx) == (ZERO3, ZERO3)

        # On a rotated, offset link the torque is the column of the existing
        # thruster torque Jacobian (ThrusterHooks), so both use one link-to-body map.
        panel = SM.Link(r=MVector(0.0, 1.0, 0.5), q=MVector(0.0, 0.0, sin(pi / 8), cos(pi / 8)))
        side = Thruster(max_thrust=1.0, location=MVector(0.3, -0.2, 0.1), direction=MVector(0.0, 1.0, 1.0))
        R = Matrix(SM.rotate_to_body(panel))
        d = normalize(Vector(side.direction))
        _, panel_torque = actuator_wrench(side, 1.0, actuator_context(panel))
        @test panel_torque ≈ cross(R * Vector(side.location) + Vector(panel.r), R * d) atol=1e-14
    end

    @testset "add_actuator! mounts thrusters and user kinds in mounting order" begin
        root = SM.Link(root=true, m=10.0)
        pod = SM.Link(m=1.0)
        sc = spacecraft_with(root; links=[root, pod])
        puck = ColdGasPuck(SVector(1.0, 0.0, 0.0), ZERO3, 0.5, 50.0)
        @test add_actuator!(sc, pod, puck) === puck
        a = add_actuator!(sc, root, Thruster(max_thrust=1.0, location=MVector(0.0, 0.0, 1.0), direction=MVector(3.0, 0.0, 0.0)))
        b = add_actuator!(sc, pod, Thruster(max_thrust=2.0, direction=MVector(0.0, 1.0, 0.0)))
        c = add_actuator!(sc, root, Thruster(max_thrust=3.0, location=MVector(1.0, 0.0, 0.0), direction=MVector(0.0, 0.0, 1.0)))
        @test a.direction == [1.0, 0.0, 0.0]                  # normalized on mounting
        @test size(root.J_thruster) == (3, 2)                 # add_thruster! now builds the Jacobian
        @test root.J_thruster[:, 1] == cross(Vector(a.location), Vector(a.direction))
        @test sc.n_thrusters == 3
        @test SM.mounted_thruster_count(sc) == 3 && SM.mounted_actuator_count(sc) == 4
        order = [entry.actuator for entry in mounted_actuators(sc)]
        @test order[1] === a && order[2] === c && order[3] === b && order[4] === puck   # links in order, thrusters first
        @test pod.custom_actuators == [puck]
        B = allocation_matrix(sc)
        @test size(B) == (6, 4)
        @test B[:, 1] == vcat(SVector(1.0, 0.0, 0.0), cross(SVector(0.0, 0.0, 1.0), SVector(1.0, 0.0, 0.0)))
        @test B[1:3, 4] == SVector(1.0, 0.0, 0.0)
        # Links carry an empty list of user kinds by default.
        @test isempty(SM.Link().custom_actuators)
    end

    @testset "six_axis_thrusters reproduce the six-axis model" begin
        six = SM.SixAxisThrusterModel(locations_body=SMatrix{3, 6, Float64}(0.1 .* randn(MersenneTwister(1), 3, 6)),
            max_thrust_n=SVector{6}(0.5, 0.6, 0.7, 0.8, 0.9, 1.0), isp_s=SVector{6}(fill(70.0, 6)), min_firing_time_s=0.02)
        jets = six_axis_thrusters(six)
        @test length(jets) == 6
        @test all(j -> Vector(jets[j].direction) == six.directions_body[:, j] && Vector(jets[j].location) == six.locations_body[:, j] &&
            jets[j].max_thrust == six.max_thrust_n[j] && jets[j].Isp == 70.0 && jets[j].min_firing_time == 0.02, 1:6)
        sc = spacecraft_with(SM.Link(root=true))
        foreach(j -> add_actuator!(sc, sc.root, j), jets)
        forces = SVector{6}(0.3, 0.0, 0.2, 0.0, 0.0, 0.7)
        f_rpo, tq_rpo = SM.rpo_thruster_wrench_body(forces, six)
        f, tq, mdot = mounted_wrench(sc, forces)
        @test f ≈ f_rpo atol=1e-15
        @test tq ≈ tq_rpo atol=1e-15
        @test mdot ≈ sum(forces) / (70.0 * G0)
    end

    @testset "allocation reproduces the existing allocators exactly" begin
        rng = MersenneTwister(20261015)
        six = SM.SixAxisThrusterModel(max_thrust_n=SVector{6}(0.5, 0.6, 0.7, 0.8, 0.9, 1.0))
        sc = spacecraft_with(SM.Link(root=true))
        foreach(j -> add_actuator!(sc, sc.root, j), six_axis_thrusters(six))
        B = allocation_matrix(sc)
        mismatches = 0
        for _ in 1:2000
            command = SVector{3}(randn(rng, 3))
            m = 0.5 * rand(rng)
            a = allocate_actuators(B, command, 0.0, collect(six.max_thrust_n); method=:axis_clamp, minimum_fraction=m)
            mismatches += a == collect(SM.rpo_allocate_six_axis_thrusters(command, six; minimum_throttle_fraction=m)) ? 0 : 1
        end
        @test mismatches == 0

        # The descent controller's jet levels: the least-norm allocation of a
        # torque over the jets' full-thrust torque arms, clipped to 0..1.
        jet(r, thrust_dir, rating) = Thruster(location=MVector{3, Float64}(r), direction=MVector{3, Float64}(thrust_dir), max_thrust=rating)
        root = SM.Link(root=true, m=500.0, thrusters=[jet((0.0, 1.0, 0.0), (0.0, 0.0, 2.0), 10.0), jet((0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 100.0)])
        pod = SM.Link(thrusters=[jet((0.0, 0.0, 1.0), (3.0, 0.0, 0.0), 20.0), jet((1.0, 0.0, 0.0), (0.0, 4.0, 0.0), 30.0),
                                 jet((0.5, 0.5, 0.0), (1.0, -1.0, 1.0), 15.0)])
        craft = SM.SpacecraftModel(links=[root, pod], root=root, id=97)
        layout = CH.descent_thruster_layout(craft)
        levels = zeros(5)
        mismatches = 0
        for _ in 1:2000
            torque = SVector{3}(30.0 .* randn(rng, 3))
            CH.descent_thruster_levels!(levels, layout, 0.0, torque)
            mismatches += allocate_actuators(layout.torque_arms_nm, torque, 0.0, 1.0) == levels[layout.jets] ? 0 : 1
        end
        @test mismatches == 0

        # Jets on a rotated, offset link: the descent layout's torque arms are
        # the mounted jets' torques at full thrust, so its levels act as allocated.
        tilted = SM.Link(r=MVector(0.4, -0.3, 1.2), q=MVector(normalize([0.1, 0.3, -0.2, 0.9])...),
            thrusters=[jet((0.5, 0.0, 0.0), (0.0, 1.0, 0.0), 12.0), jet((0.0, 0.5, 0.0), (0.0, 0.0, 1.0), 14.0),
                       jet((0.0, 0.0, 0.5), (1.0, 0.0, 0.0), 16.0)])
        engine_link = SM.Link(root=true, thrusters=[jet((0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 100.0)])
        posed = SM.SpacecraftModel(links=[engine_link, tilted], root=engine_link, id=98)
        posed_layout = CH.descent_thruster_layout(posed)
        B_posed = allocation_matrix(posed)
        ratings = [12.0, 14.0, 16.0]
        @test posed_layout.jets == [2, 3, 4]
        @test posed_layout.torque_arms_nm ≈ B_posed[4:6, 2:4] .* ratings' atol=1e-12

        # A feasible wrench is reproduced by the least-norm levels.
        full = allocation_matrix(spacecraft_with(SM.Link(root=true; thrusters=six_axis_thrusters(SM.SixAxisThrusterModel(max_thrust_n=SVector{6}(fill(5.0, 6)))))))
        target = vcat(SVector(0.2, -0.1, 0.3), ZERO3)
        @test full * allocate_actuators(full, target, -Inf, Inf) ≈ target
        floor_levels = allocate_actuators(full, target, 0.05, 5.0; method=:axis_clamp)
        @test all(>=(0.05), floor_levels) && all(<=(5.0), floor_levels)       # both bounds hold, idle jets included
        @test_throws ArgumentError allocate_actuators(full, target, 0.0, 1.0; method=:nearest)
        @test_throws ArgumentError allocate_actuators(full[:, [2, 1, 3, 4, 5, 6]], target, 0.0, 1.0; method=:axis_clamp)
        @test_throws DimensionMismatch allocate_actuators(full, target, zeros(3), 1.0)
    end

    @testset "a controller in mounted mode gets its wrench from the mounted actuators" begin
        root = SM.Link(root=true, m=500.0)
        sc = spacecraft_with(root)
        add_actuator!(sc, root, Thruster(max_thrust=2.0, location=MVector(0.0, 1.0, 0.0), direction=MVector(0.0, 0.0, 1.0), Isp=100.0))
        add_actuator!(sc, root, ColdGasPuck(SVector(1.0, 0.0, 0.0), SVector(0.0, 0.0, 0.5), 0.5, 50.0))
        control = HoldLevels(1, [1.5, 9.0])                    # the puck's 9 N saturates at 0.5 N
        p = readout_params(SM.SpacecraftModel[sc], (control,))
        q = normalize(SVector(0.1, -0.2, 0.3, 0.9))
        force, torque, wheel, rate = readout(p, 1, body_state(q))
        f_body, tq_body, mdot = mounted_wrench(sc, [1.5, 0.5])
        @test f_body == SVector(0.5, 0.0, 1.5)
        @test tq_body == cross(SVector(0.0, 1.0, 0.0), SVector(0.0, 0.0, 1.5)) + cross(SVector(0.0, 0.0, 0.5), SVector(0.5, 0.0, 0.0))
        @test force ≈ SM.rot(q)' * f_body                      # inertial force from the current attitude
        @test torque == tq_body
        @test wheel == ZERO3
        @test rate ≈ -(1.5 / (100.0 * G0) + 0.5 / (50.0 * G0))
        @test SpaceAGORA.actuation_mode(control) === :mounted
        @test SpaceAGORA.actuation_mode(SM.MagneticMomentumManagerModel()) === :ideal

        control.sat_idx = 2                                    # drives no spacecraft of this run
        @test readout(p, 1, body_state(q)) == (ZERO3, ZERO3, ZERO3, 0.0)
        control.sat_idx = 1
        control.levels = [1.0]
        @test_throws ArgumentError readout(p, 1, body_state(q))   # one level for two actuators
        control.levels = [1.0, 0.0]
        @test_throws ArgumentError readout(p, 1, ComponentVector(mass=500.0))   # no attitude state
    end

    @testset "mounted RPO actuation drives the chaser's mounted jets" begin
        six = SM.SixAxisThrusterModel(max_thrust_n=SVector{6}(fill(1.0, 6)), isp_s=SVector{6}(fill(65.0, 6)))
        root = SM.Link(root=true, m=12.0)
        chaser = spacecraft_with(root)
        foreach(j -> add_actuator!(chaser, root, j), six_axis_thrusters(six))
        forces = SVector{6}(0.4, 0.0, 0.0, 0.25, 0.1, 0.0)
        rw = SVector(1e-4, -2e-4, 3e-4)
        function held_model(actuation)
            model = SM.RPOMPCControlModel(chaser_idx=1, target_idx=2, thrusters=six, actuation=actuation)
            f_body, tq_body = SM.rpo_thruster_wrench_body(forces, six)
            model.held = SM.ControlHooks.RPOHeldActuation(force_ii=f_body, torque_body=tq_body + rw,
                thruster_forces_n=forces, rw_torque_body=rw)          # held at an identity attitude
            actuation === :mounted && CH._rpo_fill_mounted_levels!(model, chaser, forces)   # what a control tick does
            return model
        end
        ideal, mounted = held_model(:ideal), held_model(:mounted)
        p_ideal = readout_params(SM.SpacecraftModel[chaser], (ideal,))
        p_mounted = readout_params(SM.SpacecraftModel[chaser], (mounted,))
        identity = SVector(0.0, 0.0, 0.0, 1.0)
        fi, ti, wi, ri = readout(p_ideal, 1, body_state(identity; mass=12.0))
        fm, tm, wm, rm = readout(p_mounted, 1, body_state(identity; mass=12.0))
        @test fm ≈ fi atol=1e-15
        @test tm ≈ ti atol=1e-15                               # jets' torque plus the wheels'
        @test wm == wi == rw
        @test rm ≈ ri
        # Between ticks the vehicle turns: the ideal force stays where the tick
        # put it, the mounted jets turn with the body.
        q = normalize(SVector(0.0, 0.0, sin(0.2), cos(0.2)))
        @test readout(p_ideal, 1, body_state(q; mass=12.0))[1] == fi
        @test readout(p_mounted, 1, body_state(q; mass=12.0))[1] ≈ SM.rot(q)' * fm atol=1e-15
        @test SpaceAGORA.actuator_levels(mounted, 2) === nothing
        @test SpaceAGORA.actuator_levels(SM.RPOMPCControlModel(actuation=:mounted), 1) === nothing   # before the first tick
        @test_throws ArgumentError SpaceAGORA.actuation_mode(SM.RPOMPCControlModel(actuation=:bogus))
        # Other hardware on the chaser stays idle rather than failing the evaluation.
        add_actuator!(chaser, root, ColdGasPuck(SVector(0.0, 1.0, 0.0), ZERO3, 0.5, 50.0))
        CH._rpo_fill_mounted_levels!(mounted, chaser, forces)
        @test SpaceAGORA.actuator_levels(mounted, 1) == vcat(collect(forces), 0.0)
        @test readout(p_mounted, 1, body_state(identity; mass=12.0))[1] ≈ fm atol=1e-15
        @test_throws ArgumentError CH._rpo_fill_mounted_levels!(mounted, spacecraft_with(SM.Link(root=true)), forces)
    end

    @testset "mounted controllers fill the thruster_level columns" begin
        root = SM.Link(root=true, m=50.0)
        sc = spacecraft_with(root)
        add_actuator!(sc, root, Thruster(max_thrust=4.0, direction=MVector(1.0, 0.0, 0.0)))
        add_actuator!(sc, root, Thruster(max_thrust=2.0, direction=MVector(-1.0, 0.0, 0.0)))
        control = HoldLevels(1, [1.0, 3.0])
        p = readout_params(SM.SpacecraftModel[sc], (control,))
        @test SM.SimulationCallbacks.thruster_level_counts(p.args) == [2]
        levels = SM.SimulationCallbacks._save_thruster_levels(1, [2], nothing, 0.0, (p=p,))
        @test levels == [[0.25, 1.0]]                          # 1 of 4 N, and 3 N saturated at 2 N
        @test SM.mounted_thruster_fractions(sc, [1.0]) == [0.25, 0.0]
    end

    @testset "the actuator template mounts and drives through the engine" begin
        T = ExampleActuatorTemplate
        root = SM.Link(root=true, m=200.0)
        sc = spacecraft_with(root)
        nozzle = add_actuator!(sc, root, T.ExampleColdGasNozzle(SVector(1.0, 0.0, 0.0), SVector(0.0, 0.2, 0.0), 0.5, 60.0))
        @test root.custom_actuators == [nozzle]
        control = T.ExampleNozzleController(1, [0.25])
        p = readout_params(SM.SpacecraftModel[sc], (control,))
        force, torque, _, rate = readout(p, 1, body_state(SVector(0.0, 0.0, 0.0, 1.0); mass=210.0))
        @test force == SVector(0.25, 0.0, 0.0)
        @test torque == cross(SVector(0.0, 0.2, 0.0), SVector(0.25, 0.0, 0.0))
        @test rate == -0.25 / (60.0 * 9.80665)
    end

    @testset "a reaction wheel mounts on the root link and its torque and momentum are limited" begin
        root = SM.Link(root=true, m=50.0)
        sc = spacecraft_with(root)
        wheel = ReactionWheel(spin_axis=SVector(0.0, 0.0, 2.0), max_torque_nm=0.01, max_momentum_nms=0.05)
        @test wheel.spin_axis == SVector(0.0, 0.0, 1.0)
        @test add_actuator!(sc, root, wheel) === wheel
        @test root.rw_assembly.n_wheels == 1 && root.rw_assembly.J_rw == reshape([0.0, 0.0, 1.0], 3, 1)
        @test root.rw_assembly.h_wheels == [0.0] && sc.n_reaction_wheels == 1 && SM.mounted_wheel_count(sc) == 1
        @test_throws ArgumentError add_actuator!(sc, SM.Link(m=1.0), ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0)))
        @test actuator_limits(wheel) == (-0.01, 0.01)
        @test actuator_saturate(wheel, 1.0) == 0.01 && actuator_saturate(wheel, -1.0) == -0.01
        @test actuator_wrench(wheel, 0.005, actuator_context(root)) == (ZERO3, SVector(0.0, 0.0, 0.005))
        @test SM.wheel_momentum_saturate(wheel, 0.005, -0.05) == 0.0       # at the limit, storing more (dh/dt = -level)
        @test SM.wheel_momentum_saturate(wheel, -0.005, -0.05) == -0.005   # unloading is allowed
        @test SM.wheel_momentum_saturate(wheel, 0.005, -0.04) == 0.005
        @test_throws ArgumentError ReactionWheel(spin_axis=ZERO3)
        @test_throws ArgumentError ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0), max_torque_nm=0.0)
        @test isinf(ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0)).max_momentum_nms)   # unlimited by default
        # Mounting order: thrusters, then wheels, then user-defined kinds.
        puck = add_actuator!(sc, root, ColdGasPuck(SVector(1.0, 0.0, 0.0), ZERO3, 0.5, 50.0))
        jet = add_actuator!(sc, root, Thruster(max_thrust=1.0, direction=MVector(0.0, 1.0, 0.0)))
        order = [entry.actuator for entry in mounted_actuators(sc)]
        @test order[1] === jet && order[2] === wheel && order[3] === puck
        force, torque, _, wheel_torque = mounted_wrench(sc, [0.0, 0.004, 0.0]; wheel_momentum=[0.0])
        @test force == ZERO3 && wheel_torque == SVector(0.0, 0.0, 0.004) && torque == wheel_torque
        @test mounted_wrench(sc, [0.0, 0.004, 0.0]; wheel_momentum=[-0.05])[4] == ZERO3      # saturated
        @test allocation_matrix(sc)[:, 2] == [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    end

    @testset "mounted wheels spin the body up and store the opposite momentum" begin
        final, inertia = wheel_spin_up(Inf)
        @test final.ω[1] ≈ 0.01 * 5.0 / 2.0 rtol=1e-8              # τ T / I_xx
        @test final.h_wheels ≈ [-0.05, 0.0, 0.0] atol=1e-12
        @test inertia * collect(final.ω) + collect(final.h_wheels) ≈ zeros(3) atol=1e-12   # angular momentum conserved
        limited, _ = wheel_spin_up(0.03)
        @test limited.h_wheels[1] ≈ -0.03 atol=2e-4                 # stops storing at the limit
        @test limited.ω[1] ≈ 0.03 / 2.0 rtol=1e-2
    end

    @testset "the momentum manager reads the integrated wheel momentum when the vehicle carries wheels" begin
        root = SM.Link(root=true, m=50.0)
        sc = spacecraft_with(root)
        for axis in (SVector(1.0, 0.0, 0.0), SVector(0.0, 1.0, 0.0), SVector(0.0, 0.0, 1.0))
            add_actuator!(sc, root, ReactionWheel(spin_axis=axis))
        end
        manager = SM.MagneticMomentumManagerModel(sat_idx=1, mu_gain=0.01, h_wheels_0=SVector(9.0, 9.0, 9.0),
            commanded_torque=(t, r, v, q, w) -> SVector(0.0, 0.0, 0.0), b_field_ii=(t, r) -> SVector(0.0, 0.0, 3.0e-5))
        p = readout_params(SM.SpacecraftModel[sc], (manager,))
        u = (sc=[(pos=[7.0e6, 0.0, 0.0], vel=[0.0, 7.5e3, 0.0], q=[0.0, 0.0, 0.0, 1.0], ω=zeros(3), h_wheels=[0.02, -0.01, 0.0])],)
        CH.calcControlEffect!(manager, u, p, 0.0, 1)
        @test manager.h_wheels == SVector(0.02, -0.01, 0.0)       # not the configured h_wheels_0
        bare = SM.MagneticMomentumManagerModel(sat_idx=1, h_wheels_0=SVector(9.0, 9.0, 9.0),
            commanded_torque=(t, r, v, q, w) -> SVector(0.0, 0.0, 0.0), b_field_ii=(t, r) -> SVector(0.0, 0.0, 3.0e-5))
        CH.calcControlEffect!(bare, (sc=[(pos=[7.0e6, 0.0, 0.0], vel=[0.0, 7.5e3, 0.0], q=[0.0, 0.0, 0.0, 1.0], ω=zeros(3))],),
            readout_params(SM.SpacecraftModel[spacecraft_with(SM.Link(root=true, m=50.0))], (bare,)), 0.0, 1)
        @test bare.h_wheels == SVector(9.0, 9.0, 9.0)               # no wheels: its own accumulator
    end

    @testset "a torque rod's torque is its dipole crossed with the field" begin
        root = SM.Link(root=true, m=20.0)
        rod = Magnetorquer(axis=SVector(0.0, 0.0, 2.0), max_dipole_am2=1.5)
        @test rod.axis == SVector(0.0, 0.0, 1.0)
        @test actuator_limits(rod) == (-1.5, 1.5) && actuator_saturate(rod, -4.0) == -1.5
        field = SVector(2.0e-5, -1.0e-5, 3.0e-5)
        @test actuator_wrench(rod, 1.0, actuator_context(root; magnetic_field_body=field)) == (ZERO3, cross(SVector(0.0, 0.0, 1.0), field))
        @test actuator_wrench(rod, 1.0, actuator_context(root)) == (ZERO3, ZERO3)      # no field, no torque
        @test_throws ArgumentError Magnetorquer(axis=ZERO3)
        @test_throws ArgumentError Magnetorquer(axis=SVector(1.0, 0.0, 0.0), max_dipole_am2=-1.0)
        sc = spacecraft_with(root)
        @test add_actuator!(sc, root, rod) === rod && root.magnetorquers == [rod]
        wheel = add_actuator!(sc, root, ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0)))
        puck = add_actuator!(sc, root, ColdGasPuck(SVector(1.0, 0.0, 0.0), ZERO3, 0.5, 50.0))
        jet = add_actuator!(sc, root, Thruster(max_thrust=1.0, direction=MVector(0.0, 1.0, 0.0)))
        @test [e.actuator for e in mounted_actuators(sc)] == AbstractActuator[jet, wheel, rod, puck]   # thrusters, wheels, rods, user kinds
        @test SM.mounted_magnetorquer_count(sc) == 1 && SM.mounted_actuator_count(sc) == 4
        @test mounted_wrench(sc, [0.0, 0.0, 1.0, 0.0]; magnetic_field_body=field)[2] == cross(SVector(0.0, 0.0, 1.0), field)
        @test isempty(SM.Link().magnetorquers)
    end

    @testset "a mounted rod acts in the run's field, as an equal permanent dipole does" begin
        q = normalize(SVector(0.1, 0.2, 0.3, 0.9))
        x = ComponentVector(pos=[7.0e6, 1.0e6, 2.0e5], vel=[0.0, 7.5e3, 0.0], mass=50.0, q=collect(q), ω=zeros(3))
        passive_root = SM.Link(root=true, m=50.0)
        push!(passive_root.magnets, SM.Magnet(m=MVector(0.3, -0.2, 0.8)))
        _, torque_passive = SM.calcForceTorque(SM.MagneticTorqueRodModel(), x,
            readout_params(SM.SpacecraftModel[spacecraft_with(passive_root)], ()), 1)
        rod_root = SM.Link(root=true, m=50.0)
        rods = spacecraft_with(rod_root)
        for axis in (SVector(1.0, 0.0, 0.0), SVector(0.0, 1.0, 0.0), SVector(0.0, 0.0, 1.0))
            add_actuator!(rods, rod_root, Magnetorquer(axis=axis))
        end
        _, torque_rods, _, _ = readout(readout_params(SM.SpacecraftModel[rods], (HoldLevels(1, [0.3, -0.2, 0.8]),)), 1, x)
        @test torque_rods ≈ torque_passive rtol=1e-12
        @test norm(torque_rods) > 0.0
    end

    @testset "the momentum manager drives mounted torque rods with its dipole command" begin
        root = SM.Link(root=true, m=50.0)
        sc = spacecraft_with(root)
        for axis in (SVector(1.0, 0.0, 0.0), SVector(0.0, 1.0, 0.0), SVector(0.0, 0.0, 1.0))
            add_actuator!(sc, root, Magnetorquer(axis=axis))
        end
        estimate(t, r) = SVector(1.0e-5, -2.0e-5, 3.0e-5)
        manager = SM.MagneticMomentumManagerModel(sat_idx=1, mu_gain=0.01, h_wheels_0=SVector(0.02, 0.0, 0.01),
            commanded_torque=(t, r, v, q, w) -> ZERO3, b_field_ii=estimate, actuation=:mounted)
        p = readout_params(SM.SpacecraftModel[sc], (manager,))
        u = (sc=[(pos=[7.0e6, 0.0, 0.0], vel=[0.0, 7.5e3, 0.0], q=[0.0, 0.0, 0.0, 1.0], ω=zeros(3))],)
        @test SpaceAGORA.actuator_levels(manager, 1) === nothing                  # before the first tick
        CH.calcControlEffect!(manager, u, p, 0.0, 1)
        @test SpaceAGORA.actuation_mode(manager) === :mounted
        @test SpaceAGORA.actuator_levels(manager, 1) ≈ collect(manager.held_dipole_am2) atol=1e-15   # orthogonal rods carry its components
        x = ComponentVector(pos=[7.0e6, 0.0, 0.0], vel=[0.0, 7.5e3, 0.0], mass=50.0, q=[0.0, 0.0, 0.0, 1.0], ω=zeros(3))
        _, torque, _, _ = readout(p, 1, x)
        truth = SM.rod_field_body(SM.MagneticTorqueRodModel(), SVector(7.0e6, 0.0, 0.0), SVector(0.0, 0.0, 0.0, 1.0), p)
        @test torque ≈ cross(manager.held_dipole_am2, truth) rtol=1e-12          # the run's field, not the estimate
        bound = SpaceAGORA.bind_spacecraft(manager, 2)
        @test bound.mounted_levels !== manager.mounted_levels && isempty(bound.mounted_levels)
        rodless = SM.MagneticMomentumManagerModel(sat_idx=1, h_wheels_0=SVector(0.02, 0.0, 0.0),
            commanded_torque=(t, r, v, q, w) -> ZERO3, b_field_ii=estimate, actuation=:mounted)
        @test_throws ArgumentError CH.calcControlEffect!(rodless, u,
            readout_params(SM.SpacecraftModel[spacecraft_with(SM.Link(root=true, m=50.0))], (rodless,)), 0.0, 1)
        @test_throws ArgumentError SpaceAGORA.actuation_mode(SM.MagneticMomentumManagerModel(actuation=:bogus))
    end

    @testset "a laser terminal recoils with its transmitted power over c" begin
        c = 299_792_458.0
        root = SM.Link(root=true, m=30.0)
        laser = LaserTerminal(boresight=SVector(2.0, 0.0, 0.0), location=SVector(0.0, 0.5, 0.0), max_power_w=1.0e4,
            magnification=50.0, range_m=1.0e5, efficiency=0.4)
        @test laser.boresight == SVector(1.0, 0.0, 0.0)
        force, torque = actuator_wrench(laser, 3.0e3, actuator_context(root))
        @test force == SVector(-3.0e3 / c, 0.0, 0.0)                         # opposite the beam
        @test torque == cross(SVector(0.0, 0.5, 0.0), force)
        @test actuator_consumption(laser, 3.0e3, actuator_context(root)) == (mass_rate_kg_s = 0.0, power_w = 3.0e3 / 0.4)
        @test actuator_limits(laser) == (0.0, 1.0e4) && actuator_fraction(laser, 2.5e3) == 0.25
        @test_throws ArgumentError LaserTerminal(boresight=ZERO3, max_power_w=1.0)
        @test_throws ArgumentError LaserTerminal(boresight=SVector(1.0, 0.0, 0.0), max_power_w=1.0, efficiency=1.5)
        link_parameters = SM.InterLinkParameters(laser)
        @test (link_parameters.P, link_parameters.B, link_parameters.range) == (1.0e4, 50.0, 1.0e5)
        mixed = spacecraft_with(SM.Link(root=true, m=30.0))
        rod = add_actuator!(mixed, mixed.root, Magnetorquer(axis=SVector(0.0, 0.0, 1.0)))
        puck = add_actuator!(mixed, mixed.root, ColdGasPuck(SVector(1.0, 0.0, 0.0), ZERO3, 0.5, 50.0))
        beam = add_actuator!(mixed, mixed.root, LaserTerminal(boresight=SVector(1.0, 0.0, 0.0), max_power_w=10.0))
        @test mixed.root.laser_terminals == [beam] && SM.mounted_laser_count(mixed) == 1
        @test [e.actuator for e in mounted_actuators(mixed)] == AbstractActuator[rod, beam, puck]   # rods, lasers, user kinds
        sc = spacecraft_with(root)
        add_actuator!(sc, root, laser)
        p = readout_params(SM.SpacecraftModel[sc], (HoldLevels(1, [2.0e4]),))   # saturates at 1e4 W
        q = normalize(SVector(0.0, 0.0, sin(0.3), cos(0.3)))
        f_ii, tq_body, _, rate = readout(p, 1, body_state(q))
        @test f_ii ≈ SM.rot(q)' * SVector(-1.0e4 / c, 0.0, 0.0) rtol=1e-14
        @test tq_body == cross(SVector(0.0, 0.5, 0.0), SVector(-1.0e4 / c, 0.0, 0.0))
        @test rate == 0.0                                                  # no propellant
    end

    @testset "a hinge turns its link within its angle and rate limits at control ticks" begin
        root = SM.Link(root=true, m=50.0)
        panel = SM.Link(m=5.0, r=MVector(0.0, 1.5, 0.0))
        sc = SM.SpacecraftModel(links=[root, panel], root=root, prop_mass=1.0, inertia_tensor=root.inertia, id=5)
        hinge = PanelHinge(axis=SVector(0.0, 2.0, 0.0), min_angle_rad=-1.0, max_angle_rad=1.0, max_rate_rad_s=0.1, angle_rad=0.2)
        @test hinge.axis == SVector(0.0, 1.0, 0.0)
        @test add_actuator!(sc, panel, hinge) === hinge
        @test panel.q ≈ [0.0, sin(0.1), 0.0, cos(0.1)]                       # the link takes the hinge's angle
        @test_throws ArgumentError add_actuator!(sc, root, PanelHinge(axis=SVector(1.0, 0.0, 0.0)))   # not the root
        @test_throws ArgumentError add_actuator!(sc, panel, PanelHinge(axis=SVector(1.0, 0.0, 0.0)))  # one per link
        @test_throws ArgumentError PanelHinge(axis=SVector(1.0, 0.0, 0.0), min_angle_rad=1.0, max_angle_rad=0.0)
        @test actuator_wrench(hinge, 0.5, actuator_context(panel)) == (ZERO3, ZERO3)
        @test actuator_limits(hinge) == (-1.0, 1.0)
        @test SM.hinge_step(hinge, 5.0, 0.0) == 0.2                         # first tick: holds, rate-limited
        SM.apply_hinge_levels!(sc, [5.0], 0.0)
        @test hinge.angle_rad == 0.2 && hinge.last_update_s == 0.0
        SM.apply_hinge_levels!(sc, [5.0], 2.0)
        @test hinge.angle_rad ≈ 0.4                                          # 0.1 rad/s for 2 s toward the limit
        SM.apply_hinge_levels!(sc, [5.0], 100.0)
        @test hinge.angle_rad == 1.0                                          # the angle limit
        @test panel.q ≈ [0.0, sin(0.5), 0.0, cos(0.5)]
        @test SM.hinge_step(PanelHinge(axis=SVector(1.0, 0.0, 0.0)), 0.7, 0.0) == 0.7   # unlimited: at once
        @test [e.actuator for e in mounted_actuators(sc)] == AbstractActuator[hinge]
        # After the commanding effector's tick the engine applies its hinge levels.
        control = HoldLevels(1, [0.3])
        p = readout_params(SM.SpacecraftModel[sc], (control,))
        SM.SimulationCallbacks._apply_mounted_hinges!(control, p, 1, 110.0)
        @test hinge.angle_rad ≈ 0.3
        @test SM.SimulationCallbacks._apply_mounted_hinges!(SM.MagneticMomentumManagerModel(), p, 1, 120.0) === nothing
        @test hinge.angle_rad ≈ 0.3                                          # ideal effectors leave hinges alone
    end

    @testset "a mounted run moves the hinge at the controller's ticks" begin
        root = SM.Link(root=true, m=100.0, dims=MVector(1.0, 1.0, 1.0))
        panel = SM.Link(m=5.0, r=MVector(0.0, 1.5, 0.0), dims=MVector(0.02, 2.0, 1.0))
        ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), ZERO3; q=SVector(0.0, 0.0, 0.0, 1.0))
        sc = SM.SpacecraftModel(links=[root, panel], root=root, prop_mass=1.0, inertia_tensor=root.inertia, initial_condition=ic, id=6)
        hinge = add_actuator!(sc, panel, PanelHinge(axis=SVector(0.0, 1.0, 0.0), max_rate_rad_s=0.1))
        base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=3.0,
            initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
            dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
            keplerian=false, EI_km=1.0, verbose=false, results=false)
        args = SM.SimConfig._with_configuration(base;
            control_model=SM.ControlModel(control_effectors=(HoldLevels(1, [0.5]),), control_rates=[1.0]))
        run_simulation(args; isolate_state=false)
        @test 0.1 - 1e-12 <= hinge.angle_rad <= 0.3 + 1e-12                  # 0.1 rad/s from the first tick
        @test hinge.angle_rad < 0.5                                          # still on its way
        @test panel.q ≈ [0.0, sin(hinge.angle_rad / 2), 0.0, cos(hinge.angle_rad / 2)]
    end

    @testset "redundant mounted wheels each integrate their own command" begin
        # A pure null-space command: the four torques cancel on the body, yet
        # every wheel spins up at minus its own level.
        null_cmd = [0.01, 0.01, -0.01, -0.01]
        final, inertia, J = pyramid_run(null_cmd)
        @test collect(final.h_wheels) ≈ -5.0 .* null_cmd atol=1e-12
        @test collect(final.ω) ≈ zeros(3) atol=1e-14
        # A mixed command: a body torque about x plus a null-space part.
        mixed = [0.02, 0.0, -0.01, -0.01]
        final, inertia, J = pyramid_run(mixed)
        @test collect(final.h_wheels) ≈ -5.0 .* mixed atol=1e-12
        @test collect(final.ω) ≈ inertia \ (J * mixed .* 5.0) rtol=1e-8
        @test inertia * collect(final.ω) + J * collect(final.h_wheels) ≈ zeros(3) atol=1e-12   # angular momentum conserved
    end

    @testset "actuators mount only on the spacecraft's own links" begin
        root = SM.Link(root=true, m=10.0)
        sc = spacecraft_with(root)
        stray = SM.Link(m=1.0)                                       # not in sc.links
        for actuator in (Thruster(direction=MVector(1.0, 0.0, 0.0)), Magnetorquer(axis=SVector(1.0, 0.0, 0.0)),
                         LaserTerminal(boresight=SVector(1.0, 0.0, 0.0), max_power_w=1.0), PanelHinge(axis=SVector(1.0, 0.0, 0.0)),
                         ColdGasPuck(SVector(1.0, 0.0, 0.0), ZERO3, 0.5, 50.0))
            @test_throws ArgumentError add_actuator!(sc, stray, actuator)
        end
        @test isempty(stray.thrusters) && isempty(stray.magnetorquers) && isempty(stray.laser_terminals) &&
              isempty(stray.hinges) && isempty(stray.custom_actuators)
        @test stray.q == [0.0, 0.0, 0.0, 1.0]                           # a refused hinge leaves the link alone
        @test SM.mounted_actuator_count(sc) == 0
    end

    @testset "a mounted run moves the vehicle and uses propellant as its jets say" begin
        root = SM.Link(root=true, m=500.0, dims=MVector(1.0, 1.0, 1.0))
        ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), SVector(0.0, 0.0, 0.0); q=SVector(0.0, 0.0, 0.0, 1.0))
        sc = SM.SpacecraftModel(links=[root], root=root, prop_mass=10.0, inertia_tensor=root.inertia, initial_condition=ic, id=3)
        add_actuator!(sc, root, ColdGasPuck(SVector(1.0, 0.0, 0.0), ZERO3, 0.5, 50.0))   # through the centre: no torque
        thrust_n, duration_s = 0.1, 100.0
        control = HoldLevels(1, [thrust_n])
        base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=duration_s,
            initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
            dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
            keplerian=false, EI_km=1.0, verbose=false, results=false)
        args = SM.SimConfig._with_configuration(base;
            control_model=SM.ControlModel(control_effectors=(control,), control_rates=[10.0]))
        result = run_simulation(args; return_solution=true)
        m0 = sc.dry_mass + sc.prop_mass
        mdot = thrust_n / (50.0 * G0)
        final = result.u[end].sc[1]
        @test result.t[end] ≈ duration_s
        @test final.mass ≈ m0 - mdot * duration_s rtol=1e-9
        @test final.vel[1] ≈ (thrust_n / mdot) * log(m0 / (m0 - mdot * duration_s)) rtol=1e-6   # rocket equation
        @test abs(final.vel[2]) < 1e-12 && abs(final.vel[3]) < 1e-12
    end
end

end # module ActuatorContractTests
