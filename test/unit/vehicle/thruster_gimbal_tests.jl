module ThrusterGimbalTests
using Test, StaticArrays, LinearAlgebra, SpaceAGORA
import SpaceAGORA.TelemetryVerification: make_example_config
const SM = SpaceAGORA.SimulationModel

mutable struct HoldLevels <: AbstractControlEffectorModel
    levels::Vector{Float64}
end
SpaceAGORA.actuation_mode(::HoldLevels) = :mounted
SpaceAGORA.actuator_levels(c::HoldLevels, i::Int) = i == 1 ? c.levels : nothing
SpaceAGORA.update_control!(::HoldLevels, u, p, t::Float64, i::Int64) = nothing

function far_spacecraft(; mass=500.0)
    root = SM.Link(root=true, m=mass, dims=MVector(1.0, 1.0, 1.0))
    ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), SVector(0.0, 0.0, 0.0); q=SVector(0.0, 0.0, 0.0, 1.0))
    return SM.SpacecraftModel(links=[root], root=root, prop_mass=0.0, inertia_tensor=root.inertia, initial_condition=ic, id=1), root
end

function held_run(sc, levels; mission_time=5.0, rate=1.0)
    base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=mission_time,
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
        keplerian=false, EI_km=1.0, verbose=false, results=false)
    args = SM.SimConfig._with_configuration(base;
        control_model=SM.ControlModel(control_effectors=(HoldLevels(collect(Float64, levels)),), control_rates=[rate]))
    return run_simulation(args; isolate_state=false, return_results=true, return_solution=true)
end

engine(; location=(0.0, 0.0, -1.0)) = Thruster(max_thrust=2.0, direction=MVector(1.0, 0.0, 0.0), location=MVector(location...))

@testset "Thruster gimbals" begin
    @testset "the record and mounting" begin
        sc, root = far_spacecraft()
        jet = add_actuator!(sc, root, engine())
        @test_throws ArgumentError ThrusterGimbal(jet; axis=SVector(0.0, 0.0, 0.0))
        @test_throws ArgumentError ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0), min_angle_rad=0.2, max_angle_rad=0.1)
        @test_throws ArgumentError ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0), max_angle_rad=0.1, angle_rad=0.2)
        loose = Thruster(max_thrust=1.0, direction=MVector(1.0, 0.0, 0.0))
        @test_throws ArgumentError add_actuator!(sc, root, ThrusterGimbal(loose; axis=SVector(0.0, 1.0, 0.0)))   # its thruster is not on the link
        g = add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 2.0, 0.0)))
        @test_throws ArgumentError add_actuator!(sc, root, g)                                                     # mounted twice
        @test g.axis == SVector(0.0, 1.0, 0.0) && jet.gimballed
        @test [e.actuator for e in mounted_actuators(sc)] == [jet, g] && SM.mounted_gimbal_count(sc) == 1
        @test actuator_limits(g) == (-Inf, Inf)
        plain = Thruster(max_thrust=1.0, direction=MVector(0.0, 1.0, 0.0))
        @test thrust_direction(plain) === SVector(0.0, 1.0, 0.0)                                                 # no gimbal: the direction itself
    end

    @testset "a gimbal turns the thrust about its axis, not the thrust point" begin
        θ = 0.1
        sc, root = far_spacecraft()
        jet = add_actuator!(sc, root, engine())
        g = add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0), angle_rad=θ))
        @test thrust_direction(jet) ≈ SVector(cos(θ), 0.0, -sin(θ)) atol=1e-15
        force, torque = actuator_wrench(jet, 2.0, actuator_context(root))
        @test force ≈ 2.0 * SVector(cos(θ), 0.0, -sin(θ)) atol=1e-15
        @test torque ≈ cross(SVector(0.0, 0.0, -1.0), force) atol=1e-15                                     # about the fixed nozzle
        B = allocation_matrix(sc)
        @test B[1:3, 1] ≈ SVector(cos(θ), 0.0, -sin(θ)) && all(iszero, B[:, 2])                             # the gimbal has no column of its own
        # A second axis, about z, turns after the first.
        φ = 0.3
        add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 0.0, 1.0), angle_rad=φ))
        @test thrust_direction(jet) ≈ SVector(cos(φ) * cos(θ), sin(φ) * cos(θ), -sin(θ)) atol=1e-15
        @test applied_actuator_levels(sc, [2.0, 0.0, 0.0]) == [2.0, θ, φ]
    end

    @testset "a gimbal moves at control ticks within its angle and rate limits" begin
        sc, root = far_spacecraft()
        jet = add_actuator!(sc, root, engine())
        slow = add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0), max_angle_rad=0.3, max_rate_rad_s=0.05))
        SM.apply_gimbal_levels!(sc, [1.0, 1.0], 0.0)
        @test slow.angle_rad == 0.0                                                                          # a rate-limited first tick holds
        SM.apply_gimbal_levels!(sc, [1.0, 1.0], 2.0)
        @test slow.angle_rad ≈ 0.1 && thrust_direction(jet) ≈ SVector(cos(0.1), 0.0, -sin(0.1))
        SM.apply_gimbal_levels!(sc, [1.0, 1.0], 100.0)
        @test slow.angle_rad == 0.3                                                                          # within its angle limit
        @test SM.gimbal_step(slow, -1.0, 101.0) ≈ 0.25
    end

    @testset "a run vectors the thrust from the gimbal's first tick" begin
        # 1 N along x from the centre of mass, the gimbal commanded to 0.2 rad: the
        # thrust is along x until the first control tick at 1 s, then turned.
        θ = 0.2
        sc, root = far_spacecraft(mass=500.0)
        jet = add_actuator!(sc, root, engine(location=(0.0, 0.0, 0.0)))
        g = add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0)))
        res = held_run(sc, [1.0, θ])
        v = res.solution.u[end].sc[1].vel
        @test v[1] ≈ (1.0 + 4.0 * cos(θ)) / 500.0 rtol=1e-9
        @test v[3] ≈ -4.0 * sin(θ) / 500.0 rtol=1e-9
        @test norm(res.solution.u[end].sc[1].ω) < 1e-15                                                      # no torque about the centre of mass
        @test res.table.sc1_actuator_level_2[end] ≈ θ                                                        # the gimbal's column is its angle
    end

    @testset "the visualization saves a gimballed thruster's exhaust direction" begin
        θ = 0.2
        sc, root = far_spacecraft()
        jet = add_actuator!(sc, root, engine(location=(0.0, 0.0, 0.0)))
        add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0)))
        base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=3.0,
            initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
            dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
            keplerian=false, EI_km=1.0, verbose=false, results=false)
        settings = SM.SimulationSettings(results=false, verbose=false, generate_plots=false, normalize=false,
            save_visualization_scene=true)
        args = SM.SimConfig._with_configuration(base; simulation_settings=settings,
            control_model=SM.ControlModel(control_effectors=(HoldLevels([1.0, θ]),), control_rates=[1.0]))
        @test :thruster_direction in [f.name for f in SM.SimulationCallbacks.visualization_save_fields(args)]
        table = run_simulation(args; isolate_state=false, return_results=true).table
        exhaust(t) = [table[t, "sc1_thruster_direction_$(c)"] for c in 1:3]
        @test exhaust(1) ≈ [-1.0, 0.0, 0.0]                                                                  # nominal before the first tick
        @test exhaust(size(table, 1)) ≈ -collect(thrust_direction(jet)) && abs(exhaust(size(table, 1))[3]) > 0.1
        plain, proot = far_spacecraft()
        add_actuator!(plain, proot, engine())
        @test !SM.SimulationCallbacks._thruster_direction_field_enabled(SM.SimConfig._with_configuration(args;
            dynamics_model=SM.DynamicsModel([plain], ())))                                                    # no gimbal, no field
    end

    @testset "a failed gimbal holds its angle" begin
        sc, root = far_spacecraft()
        jet = add_actuator!(sc, root, engine())
        g = add_actuator!(sc, root, ThrusterGimbal(jet; axis=SVector(0.0, 1.0, 0.0), angle_rad=0.05))
        add_fault!(sc, ActuatorFault(g; kind=:failed, start_s=0.0))
        held_run(sc, [1.0, 0.4]; mission_time=3.0)
        @test g.angle_rad == 0.05 && thrust_direction(jet) ≈ SVector(cos(0.05), 0.0, -sin(0.05))
    end
end

end # module ThrusterGimbalTests
