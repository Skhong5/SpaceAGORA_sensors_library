module ActuatorFaultTests
using Test, StaticArrays, LinearAlgebra, SpaceAGORA
import SpaceAGORA.TelemetryVerification: make_example_config
const SM = SpaceAGORA.SimulationModel
const G0 = 9.80665

mutable struct HoldLevels <: AbstractControlEffectorModel
    levels::Vector{Float64}
end
SpaceAGORA.actuation_mode(::HoldLevels) = :mounted
SpaceAGORA.actuator_levels(c::HoldLevels, i::Int) = i == 1 ? c.levels : nothing
SpaceAGORA.calcControlEffect!(::HoldLevels, u, p, t::Float64, i::Int64) = nothing

function far_spacecraft(; mass=100.0, prop_mass=10.0)
    root = SM.Link(root=true, m=mass, dims=MVector(1.0, 1.0, 1.0))
    ic = SM.CartesianInitialCondition(SVector(4.0e9, 0.0, 0.0), SVector(0.0, 0.0, 0.0); q=SVector(0.0, 0.0, 0.0, 1.0))
    return SM.SpacecraftModel(links=[root], root=root, prop_mass=prop_mass, inertia_tensor=root.inertia, initial_condition=ic, id=1), root
end

function held_run(sc, levels; mission_time=5.0, rate=1.0, kw...)
    base = make_example_config(planet=SM.Earth(), spacecraft=sc, mission_time=mission_time,
        initial_time=SM.InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(), ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=true,
        keplerian=false, EI_km=1.0, verbose=false, results=false)
    args = SM.SimConfig._with_configuration(base;
        control_model=SM.ControlModel(control_effectors=(HoldLevels(collect(Float64, levels)),), control_rates=[rate]))
    return run_simulation(args; isolate_state=false, return_results=true, return_solution=true, kw...)
end

@testset "Actuator faults" begin
    @testset "the record and mounting" begin
        jet = Thruster(max_thrust=1.0, direction=MVector(1.0, 0.0, 0.0))
        @test_throws ArgumentError ActuatorFault(jet; kind=:melted)
        @test_throws ArgumentError ActuatorFault(jet; kind=:failed, start_s=5.0, stop_s=5.0)
        @test_throws ArgumentError ActuatorFault(jet; kind=:degraded, value=1.5)
        @test_throws ArgumentError ActuatorFault(jet; kind=:biased)                 # needs a value
        sc, root = far_spacecraft()
        @test_throws ArgumentError add_fault!(sc, ActuatorFault(jet; kind=:failed))  # not mounted
        add_actuator!(sc, root, jet)
        f = add_fault!(sc, ActuatorFault(jet; kind=:failed, start_s=1.0))
        @test SM.has_faults(sc) && sc.root.faults.faults == [f]
    end

    @testset "each kind changes the level as documented" begin
        sc, root = far_spacecraft()
        panel = SM.Link(m=1.0, r=MVector(0.0, 1.0, 0.0))
        push!(sc.links, panel)
        jet = add_actuator!(sc, root, Thruster(max_thrust=2.0, direction=MVector(1.0, 0.0, 0.0)))
        wheel = add_actuator!(sc, root, ReactionWheel(spin_axis=SVector(0.0, 0.0, 1.0), max_torque_nm=0.1))
        rod = add_actuator!(sc, root, Magnetorquer(axis=SVector(1.0, 0.0, 0.0), max_dipole_am2=1.0))
        hinge = add_actuator!(sc, panel, PanelHinge(axis=SVector(0.0, 1.0, 0.0), angle_rad=0.3))
        faults = [add_fault!(sc, ActuatorFault(jet; kind=:degraded, value=0.5)),
                  add_fault!(sc, ActuatorFault(wheel; kind=:failed)),
                  add_fault!(sc, ActuatorFault(rod; kind=:biased, value=0.25)),
                  add_fault!(sc, ActuatorFault(hinge; kind=:failed))]
        SM.resolve_faults!(sc)
        @test [f.index for f in faults] == [1, 2, 3, 4]
        levels = [3.0, 0.05, 0.5, 1.0]
        @test SM.faulted_levels(sc, levels) === levels                               # none active yet
        @test SM.refresh_faults!(sc, levels, 0.0)
        out = SM.faulted_levels(sc, levels)
        @test out[1] == 1.0                    # 3 N on a 2 N jet at half: within the halved limit of 1 N
        @test out[2] == 0.0                    # failed wheel
        @test out[3] == 0.75                   # biased rod
        @test out[4] == 0.3                    # a failed hinge holds its angle
        @test levels == [3.0, 0.05, 0.5, 1.0]  # the command is untouched
        stuck_sc, stuck_root = far_spacecraft()
        w = add_actuator!(stuck_sc, stuck_root, ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0)))
        add_fault!(stuck_sc, ActuatorFault(w; kind=:stuck, start_s=2.0, stop_s=4.0))
        SM.resolve_faults!(stuck_sc)
        @test !SM.refresh_faults!(stuck_sc, [0.01], 1.0)
        @test SM.refresh_faults!(stuck_sc, [0.02], 2.0) && SM.faulted_levels(stuck_sc, [0.07]) == [0.02]   # held at onset
        @test SM.refresh_faults!(stuck_sc, [0.07], 4.0) && SM.faulted_levels(stuck_sc, [0.07]) == [0.07]   # over at stop
        @test SM.fault_edge(stuck_sc, 2.0) && SM.fault_edge(stuck_sc, 4.0) && !SM.fault_edge(stuck_sc, 3.0)
    end

    @testset "a wheel that fails between ticks stops at the fault time exactly" begin
        function wheel_run(fail)
            sc, root = far_spacecraft()
            wheel = add_actuator!(sc, root, ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0)))
            fail && add_fault!(sc, ActuatorFault(wheel; kind=:failed, start_s=2.5))
            return held_run(sc, [0.01]), sc.inertia_tensor
        end
        (failed, inertia), (healthy, _) = wheel_run(true), wheel_run(false)
        x = failed.solution.u[end].sc[1]
        @test x.ω[1] ≈ 0.01 * 2.5 / inertia[1, 1] rtol=1e-9                           # torque for 2.5 s only
        @test x.h_wheels[1] ≈ -0.01 * 2.5 rtol=1e-9
        @test healthy.solution.u[end].sc[1].h_wheels[1] ≈ -0.01 * 5.0 rtol=1e-9
        col = failed.table.sc1_fault_active_1
        @test all(i -> col[i] == (failed.table.time[i] >= 2.5 ? 1.0 : 0.0), eachindex(col))
        @test all(i -> failed.table.sc1_actuator_level_1[i] == (failed.table.time[i] >= 2.5 ? 0.0 : 0.01), eachindex(col))
    end

    @testset "a degraded thruster burns half its propellant in the window; a stuck hinge stays" begin
        sc, root = far_spacecraft(mass=500.0)
        jet = add_actuator!(sc, root, Thruster(max_thrust=1.0, direction=MVector(1.0, 0.0, 0.0), Isp=200.0))
        add_fault!(sc, ActuatorFault(jet; kind=:degraded, value=0.5, start_s=1.5, stop_s=3.5))
        res = held_run(sc, [1.0])
        used = 510.0 - res.solution.u[end].sc[1].mass
        @test used ≈ (3.0 * 1.0 + 2.0 * 0.5) / (200.0 * G0) rtol=1e-9                  # 3 s full, 2 s at half
        hsc, hroot = far_spacecraft()
        panel = SM.Link(m=2.0, r=MVector(0.0, 1.5, 0.0), dims=MVector(0.02, 2.0, 1.0))
        push!(hsc.links, panel)
        hinge = add_actuator!(hsc, panel, PanelHinge(axis=SVector(0.0, 1.0, 0.0), max_rate_rad_s=0.1))
        add_fault!(hsc, ActuatorFault(hinge; kind=:stuck, start_s=2.5))
        held_run(hsc, [1.0])
        @test 0.0 < hinge.angle_rad < 0.25 + 1e-12                                     # moved until the fault, then held
        stuck_at = hinge.angle_rad
        @test panel.q ≈ [0.0, sin(stuck_at / 2), 0.0, cos(stuck_at / 2)]
    end

    @testset "faults are opt-in" begin
        sc, root = far_spacecraft()
        add_actuator!(sc, root, ReactionWheel(spin_axis=SVector(1.0, 0.0, 0.0)))
        @test !any(n -> occursin("fault", n), names(held_run(sc, [0.01]).table))
    end
end

end # module ActuatorFaultTests
