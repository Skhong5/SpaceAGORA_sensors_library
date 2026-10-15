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

        # A feasible wrench is reproduced by the least-norm levels.
        full = allocation_matrix(spacecraft_with(SM.Link(root=true; thrusters=six_axis_thrusters(SM.SixAxisThrusterModel(max_thrust_n=SVector{6}(fill(5.0, 6)))))))
        target = vcat(SVector(0.2, -0.1, 0.3), ZERO3)
        @test full * allocate_actuators(full, target, -Inf, Inf) ≈ target
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
        @test_throws ArgumentError SpaceAGORA.actuation_mode(SM.RPOMPCControlModel(actuation=:bogus))
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
