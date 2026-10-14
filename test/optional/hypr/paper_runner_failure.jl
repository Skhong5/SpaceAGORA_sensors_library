# Fault injection into the actual example main(), without expensive numerical runs.
module PaperRunnerFailureChecks
using Test, Random
module SpaceAGORA
load_rpo_station_cad_pointcloud(args...;kwargs...)=nothing
end
module JSON
json(x)=repr(x)
end
module G
const calls=Any[]
const mode=Ref(:planning)
RPOPlannerComparisonConfig(;kwargs...)=(;kwargs...)
RPORRTConnectSettings(;kwargs...)=nothing
RPORRTStarSettings(;kwargs...)=nothing
RPOTrajectoryOptimizerSettings(;kwargs...)=nothing
function rpo_plan_comparison_path(planner,start,goal,geometry,cfg;runtime_limit_s,rng,warmstart_path=nothing)
    start[1] == -2. && return nothing
    push!(calls,(planner=planner,allowance=runtime_limit_s,warmstart=warmstart_path))
    planner != :hypr && error("Injected stop after recording dependent invocation")
    mode[] == :planning && error("Injected HYPR planning exception")
    return (planner_plan_time=mode[]==:invalid ? Inf : .25,warmstart=(path=zeros(3,2),))
end
end
paper_config()=(n=1,)
tracking_config(args...;kwargs...)=nothing
paper_case_ids()=[1]
paper_planners()=[:hypr,:chomp]
paper_planning_failed(plan)=false
RPOReferenceGeometry(args...;kwargs...)=nothing
RPOStationGeometry(args...;kwargs...)=nothing
RPOCubeSatGeometry(args...;kwargs...)=nothing
SVector(args...)=args
full_evaluate(plan,case,args...)=case.seed==100 ? nothing : error("Injected HYPR tracking exception")
const saved=Dict{String,Any}()
savejson(name,x)=(saved[name]=deepcopy(x);nothing)
generate_rpo_seeded_batch_cases(;kwargs...)=[(start_rtn=(-1.,0.,0.),goal_rtn=(1.,0.,0.))]
const USE_FULL_SIM=true
source=joinpath(@__DIR__,"..","..","examples","Earth_RPO_HyPR_FinalISpaRo_Reconstruction.jl")
for expression in Meta.parseall(read(source,String)).args
    if expression isa Expr && expression.head==:function && expression.args[1] == :(main())
        eval(expression)
    end
end
@testset "Failed baseline cannot launch unmatched comparisons" begin
    for mode in (:planning,:invalid)
        empty!(G.calls);empty!(saved);G.mode[]=mode
        redirect_stdout(devnull) do
            main()
        end
        @test length(G.calls)==1
        @test only(G.calls).planner==:hypr
        rows=saved["pilot_results.json"]
        @test length(rows)==2
        @test rows[1].status=="execution_error" && !rows[1].success
        @test rows[2].status=="blocked_baseline" && !rows[2].success
    end
end
@testset "Tracking failure preserves a valid planning allowance" begin
    empty!(G.calls);empty!(saved);G.mode[]=:tracking
    redirect_stdout(devnull) do
        main()
    end
    @test length(G.calls)==2
    @test G.calls[2].planner==:chomp
    @test G.calls[2].allowance==.25
    @test G.calls[2].warmstart==zeros(3,2)
    @test all(row->row.status=="execution_error" && !row.success,saved["pilot_results.json"])
end
end
