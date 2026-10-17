module BuiltinConformanceTests

using Test, SpaceAGORA
using SpaceAGORA.Conformance: check_force_torque_model, package_types, test_report

include(joinpath(@__DIR__, "..", "..", "fixtures", "conformance_instances.jl"))
using .ConformanceInstances: force_torque_instances, force_torque_exemptions, sensor_instances, sensor_exemptions
using SpaceAGORA.Conformance: check_sensor

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
        test_report(check_force_torque_model(entry.model; fixture=entry.fixture))
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

end # module BuiltinConformanceTests
