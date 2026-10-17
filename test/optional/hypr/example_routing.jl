using Test, JSON
const smoke=joinpath(@__DIR__,"..","..","smoke","ci_examples_suite_smoke.jl")
const routing=Module(:ExampleRoutingWitness)
Base.include(ex -> ex == :(examples = list_examples()) ? :(examples = String[]) : ex,
    routing,smoke)
@testset "Example commands isolate the optional companion" begin
    for name in ("Earth_RPO_CubeSat_MPC.jl", "Earth_RPO_CubeSat_MPC_Batch.jl",
            "Earth_RPO_CubeSat_MPC_PlannerComparison.jl", "Earth_RPO_CubeSat_MPC_Replanning.jl",
            "Robot_Arm_Planner_Cloth_Demo.jl",
            "Earth_RPO_HyPR_FinalISpaRo_Reconstruction.jl", "Earth_RPO_HyPR_FinalISpaRo_Replanning.jl")
        cmd=routing.example_command(joinpath(routing.EXAMPLES_DIR,name))
        @test "--project=$(joinpath(routing.EXAMPLES_DIR,"rpo_planner_env"))" in cmd.exec
        @test all(!startswith(arg,"--load") && arg!="-L" for arg in cmd.exec)
    end
    @test routing.example_project("Earth_Torque_Free_Test.jl")==routing.REPO_ROOT
    mktempdir() do dir
        # Use the actual smoke child process to prove an ordinary example cannot
        # see the companion, even after the parent has loaded it elsewhere.
        probe=joinpath(dir,"baseline_core_witness.jl")
        write(probe,"using SpaceAGORA; @assert !hypr_available(); @assert Base.find_package(\"SpaceAGORAHYPR\") === nothing; println(\"baseline_core_only_ok\")")
        ok,has_nan,output=routing.run_example(probe)
        @test ok
        @test !has_nan
        @test occursin("baseline_core_only_ok",output)
    end
end

@testset "Paper entry points use the isolated companion environment" begin
    mktempdir() do output_root
        withenv("HYPR_OUTPUT_ROOT"=>output_root) do
            for (name, mode) in (("Earth_RPO_HyPR_FinalISpaRo_Reconstruction.jl", "reconstruction"),
                    ("Earth_RPO_HyPR_FinalISpaRo_Replanning.jl", "replanning"))
                path=joinpath(routing.EXAMPLES_DIR,name)
                @test path in routing.list_examples()
                ok,has_nan,output=routing.run_example(path)
                @test ok
                @test !has_nan
                @test occursin("hypr_paper_startup_smoke_ok "*mode,output)
                @test !occursin("CASE_START",output)
                @test !occursin("FULL_DYNAMIC_START",output)
                # The replanning study stays on the deprecated RPOGuidanceModel until
                # its removal; its smoke mode never configures that model.
                @test !occursin("RPOGuidanceModel is deprecated",output)
                @test isempty(readdir(output_root))
            end
            # A readable but structurally incomplete baseline must also fail.
            malformed=JSON.parsefile(joinpath(routing.EXAMPLES_DIR,"support","hypr_final_isparo_case4.json"))
            delete!(malformed,"reference_position")
            badfile=joinpath(output_root,"malformed.json")
            open(io->JSON.print(io,malformed),badfile,"w")
            withenv("HYPR_BASELINE_RECORD"=>badfile) do
                ok,_,output=routing.run_example(joinpath(routing.EXAMPLES_DIR,"Earth_RPO_HyPR_FinalISpaRo_Replanning.jl"))
                @test !ok
                @test !occursin("hypr_paper_startup_smoke_ok",output)
            end
            # An invalid retained input must still fail the real child process.
            withenv("HYPR_BASELINE_RECORD"=>joinpath(output_root,"missing.json")) do
                ok,_,output=routing.run_example(joinpath(routing.EXAMPLES_DIR,"Earth_RPO_HyPR_FinalISpaRo_Replanning.jl"))
                @test !ok
                @test !occursin("hypr_paper_startup_smoke_ok",output)
            end
        end
    end
end
