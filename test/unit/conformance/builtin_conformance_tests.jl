module BuiltinConformanceTests

using Test, SpaceAGORA
using SpaceAGORA.Conformance: check_force_torque_model, package_types, test_report

include(joinpath(@__DIR__, "..", "..", "fixtures", "conformance_instances.jl"))
using .ConformanceInstances: force_torque_instances, force_torque_exemptions, sensor_instances, sensor_exemptions,
    gnc_instances, gnc_exemptions
using SpaceAGORA.Conformance: check_sensor, check_guidance_model, check_control_model

const SM = SpaceAGORA.SimulationModel
const C = SpaceAGORA.Conformance

@testset "Built-in force/torque models pass the conformance checks" begin
    types = package_types(SpaceAGORA.AbstractForceTorqueModel)
    instances = force_torque_instances()
    exemptions = force_torque_exemptions()
    @testset "every concrete built-in type has an instance or a listed exemption" begin
        @test length(types) >= 15
        for T in types
            covered = count(entry -> typeof(entry.model) <: T, instances)
            exempt = haskey(exemptions, T)
            @testset let type = T, instance_count = covered, exempt = exempt
                @test (instance_count == 1) != exempt
            end
        end
        # Every instance is of a walked type, and every exemption names one, with a reason.
        @test all(entry -> any(T -> typeof(entry.model) <: T, types), instances)
        for (T, reason) in exemptions
            @test T in types
            @test !isempty(strip(reason))
        end
    end
    for entry in instances
        entry.runs || @test !isempty(strip(entry.reason))
        report = entry.runs ? check_force_torque_model(entry.model; fixture=entry.fixture) :
            SpaceAGORA.Conformance._static_force_report(entry.model)
        test_report(report)
    end
end

@testset "Built-in sensors pass the conformance checks" begin
    types = package_types(SpaceAGORA.AbstractSensor)
    instances = sensor_instances()
    exemptions = sensor_exemptions()
    @testset "every concrete built-in sensor kind has an instance or a listed exemption" begin
        @test length(types) >= 3
        for T in types
            covered = count(sensor -> typeof(sensor) <: T, instances)
            @testset let type = T, instance_count = covered, exempt = haskey(exemptions, T)
                @test (instance_count == 1) != exempt
            end
        end
        @test all(sensor -> any(T -> typeof(sensor) <: T, types), instances)
        for (T, reason) in exemptions
            @test T in types
            @test !isempty(strip(reason))
        end
    end
    for sensor in instances
        test_report(check_sensor(sensor))
    end
end

@testset "Built-in guidance and control models pass the conformance checks" begin
    abstract_types = (guidance_model=SpaceAGORA.AbstractGuidanceModel, navigation_model=SpaceAGORA.AbstractNavigationModel,
        control_model=SpaceAGORA.AbstractControlEffectorModel)
    instances = gnc_instances()
    exemptions = gnc_exemptions()
    @testset "every concrete built-in GNC type has an instance or a listed exemption" begin
        types = Dict(kind => C.package_types(T) for (kind, T) in pairs(abstract_types))
        @test length(types[:guidance_model]) >= 5 && length(types[:control_model]) >= 6
        for (kind, kind_types) in types, T in kind_types
            covered = count(entry -> entry.kind === kind && typeof(entry.model) <: T, instances)
            @testset let kind = kind, type = T, instance_count = covered, exempt = haskey(exemptions, T)
                @test (instance_count == 1) != exempt
            end
        end
        # The thrusters run in the control model without subtyping the control effector type.
        @test count(entry -> entry.model isa SM.ThrusterModels.BaseThrusterModel, instances) == 1
        for (T, reason) in exemptions
            @test any(kind_types -> T in kind_types, values(types))
            @test !isempty(strip(reason))
        end
        for entry in instances
            entry.runs || @test !isempty(strip(entry.reason))
        end
    end
    @testset "the exempt RPO planner binding has its guidance tick" begin
        fixture = C.conformance_fixture()
        planner_guidance = SpaceAGORA.RPOPlannerLifecycle.PlannerGuidance
        @test hasmethod(SpaceAGORA.calcGuidanceEffect!,
            Tuple{planner_guidance, fixture.call.state, fixture.call.params, Float64, Int})
    end
    @testset "the energy depletion panel actuator has outputs but no tick of its own" begin
        aoa = SpaceAGORA.SolarPanelAngleOfAttackControlModel
        fixture = C.conformance_fixture(control=(aoa(),))
        @test !hasmethod(SpaceAGORA.calcControlEffect!, Tuple{aoa, fixture.call.state, fixture.call.params, Float64, Int})
        @test hasmethod(SpaceAGORA.control_wrench, Tuple{aoa, SpaceAGORA.StateSample, fixture.call.params, Float64})
        X = only(fixture.call.spacecraft)
        for old in (SpaceAGORA.calcControlForceTorque, SpaceAGORA.calcControlMassFlowRate, SpaceAGORA.calcReactionWheelTorque)
            @test !SpaceAGORA.SimulationEngine._own_method(old, Tuple{aoa, X, fixture.call.params, Int, Float64})
        end
    end
    for entry in instances
        if entry.runs
            check = entry.kind === :guidance_model ? check_guidance_model : check_control_model
            test_report(check(entry.model))
        else
            test_report(C._static_gnc_report(entry.model, entry.kind))
        end
    end
end

end # module BuiltinConformanceTests
