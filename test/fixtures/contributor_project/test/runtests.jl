using Test, SpaceAGORA, ContributorProject

@testset "Contributor project fixture" begin
    @testset "Short run" begin
        solution = ContributorProject.run_case(mission_time_s=10.0)
        @test last(solution.t) ≈ 10.0
        @test all(isfinite, solution.u[end])
    end
    # Placeholder until SpaceAGORA.Conformance exists: the generated starters replace
    # this fixture and call its checks on the project's own models.
    @testset "Conformance checks" begin
        @test_skip isdefined(SpaceAGORA, :Conformance)
    end
end
