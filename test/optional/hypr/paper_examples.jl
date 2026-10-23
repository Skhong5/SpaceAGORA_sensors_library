module PaperExampleChecks
using Test, Random, StaticArrays, SpaceAGORA, HYPR, JSON, SHA
const G=SpaceAGORA.SimulationModel.GuidanceHooks
include(joinpath(@__DIR__, "..", "..", "..", "examples", "support", "hypr_paper_comparison.jl"))
@testset "Paper example inputs and retained baseline" begin
    @test paper_case_ids(Dict()) == [1]
    @test paper_case_ids(Dict("HYPR_CASE_IDS"=>"4,7")) == [4,7]
    for value in ("0", "-2", "4,4")
        @test_throws ArgumentError paper_case_ids(Dict("HYPR_CASE_IDS"=>value))
    end
    @test_throws ArgumentError paper_case_ids(Dict("HYPR_CASES"=>"0"))
    @test paper_planners(Dict()) == [:hypr]
    @test paper_planners(Dict("HYPR_PLANNERS"=>"hypr,rrt_connect,rrt_connect_bezier")) == [:hypr,:rrt_connect,:rrt_connect_bezier]
    for value in ("chomp", "hypr,unknown", "hypr,hypr", "hypr,rrt_connect_bezier", "hypr,rrt_connect_bezier,rrt_connect")
        @test_throws ArgumentError paper_planners(Dict("HYPR_PLANNERS"=>value))
    end
    # Each name stands for the planner object the comparison configuration builds by default.
    cc = G.RPOPlannerComparisonConfig(show_progress=false)
    for (name, planner) in zip((:hypr, :pso_unrefined, :rrt_connect, :rrt_connect_bezier, :rrt_star, :chomp, :stomp), cc.planners)
        object = paper_planner_object(name, cc)
        @test typeof(object) == typeof(planner) &&
            all(f -> isequal(getfield(object, f), getfield(planner, f)), fieldnames(typeof(object)))
    end
    @test_throws ArgumentError paper_planner_object(:unknown, cc)
    @test paper_dynamic_budget(84.4, Dict()) == 600.
    @test paper_dynamic_budget(200., Dict()) == 800.
    @test paper_dynamic_budget(84.4, Dict("HYPR_DYNAMIC_MAX_TIME_S"=>"1200")) == 1200.
    for value in ("0", "-1", "Inf", "NaN")
        @test_throws ArgumentError paper_dynamic_budget(84.4, Dict("HYPR_DYNAMIC_MAX_TIME_S"=>value))
    end
    fixture=joinpath(@__DIR__,"..","..","..","examples","support","hypr_final_isparo_case4.json")
    @test bytes2hex(sha256(read(fixture))) == "889c2df68edf946d7c097a7b95068f40809afb90aa003fe6af984393293843f5"
    d=JSON.parsefile(fixture)
    @test d["case"]["case_id"] == 4
    @test first(d["reference_time"]) == 0.
    @test all(iszero, first(d["reference_velocity"]))
    @test all(iszero, last(d["reference_velocity"]))
end
@testset "Identical RRT base and failed-search evidence" begin
    geometry=G.RPOReferenceGeometry(G.RPOStationGeometry(zeros(3,1);keepout_radius_m=.25))
    cfg=G.RPOPSOConfig(n_waypoints=3,refinement_enable=false)
    settings=G.RPORRTConnectSettings(n_iters=1000)
    start=SVector(-3.,2.,0.);goal=SVector(3.,2.,0.)
    base=G.rpo_rrt_connect_plan_path(start,goal,geometry,cfg;settings=settings,rng=MersenneTwister(741))
    @test base.path_found
    @test !paper_planning_failed(base)
    paired=paired_bezier_shape(base,geometry,cfg)
    standard=G.rpo_rrt_connect_bezier_plan_path(start,goal,geometry,cfg;settings=settings,rng=MersenneTwister(741))
    @test paired.path == standard.path
    @test paired.raw_path == base.raw_path == standard.raw_path
    @test paired.cost == standard.cost
    @test paired.config == standard.config
    @test paired.iterations == base.iterations == standard.iterations
    impossible=G.rpo_rrt_connect_plan_path(start,SVector(0.,0.,0.),geometry,cfg;settings=G.RPORRTConnectSettings(n_iters=1),rng=MersenneTwister(12))
    @test impossible.path_found === false
    @test paper_planning_failed(impossible)
end
end
