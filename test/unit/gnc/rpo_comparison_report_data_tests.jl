module RPOComparisonReportDataTests
using Test, Logging, SpaceAGORA
const G = SpaceAGORA.SimulationModel.GuidanceHooks
const Reports = G.RPOComparisonReports
const PLANNING = normpath(joinpath(@__DIR__, "..", "..", "..", "src", "interfaces", "analysis", "planning"))

# Every symbol in a file's code. Docstrings are strings and comments are not
# parsed, so neither counts.
function _code_symbols(path)
    symbols = Set{Symbol}()
    walk(x::Symbol) = push!(symbols, x)
    walk(x::Expr) = foreach(walk, x.args)
    walk(x) = nothing
    walk(Meta.parseall(read(path, String); filename=path))
    return symbols
end

@testset "Comparison report data are prepared without PlotlyJS" begin
    # The report data, the CSV and the writer never name PlotlyJS; the figures
    # file is the module's only PlotlyJS user.
    @test !(:PlotlyJS in _code_symbols(joinpath(PLANNING, "rpo_comparison_reports.jl")))
    @test :PlotlyJS in _code_symbols(joinpath(PLANNING, "rpo_comparison_plots.jl"))
    importers = [f for f in readdir(PLANNING) if endswith(f, ".jl") &&
                 occursin(r"^\s*(import|using)\s+PlotlyJS"m, read(joinpath(PLANNING, f), String))]
    @test importers == ["rpo_comparison_plots.jl"]
end

ok(planner, label, fuel) = (planner=planner, case_label=label, success=true, planner_compute_time=0.5,
    fuel_used_pct=fuel, control_effort_total=2.0, thrust_saturation_fraction=0.0, min_clearance=3.0,
    final_pos_error=0.1, actual_travel_duration=4.0, keepout_violations=0)
failed(planner, label) = merge(ok(planner, label, 99.0), (success=false, keepout_violations=2))
path(label; raw=nothing, costs=nothing) = begin
    ref = Float64[0 1 2; 0 0 1; 0 0 0]
    nt = (case=(label=label, start_rtn=[0.0, 0.0, 0.0], goal_rtn=[2.0, 1.0, 0.0]),
          tracking=(r_ref_rtn=ref, x_hist=vcat(ref .+ 0.5, zeros(3, 3))), planner_iteration_count=6)
    raw === nothing || (nt = merge(nt, (raw_path=raw,)))
    costs === nothing || (nt = merge(nt, (cost_history=costs,)))
    nt
end
batch = (planner_types=[:rrt_connect, :chomp],
    results_by_planner=Dict(:rrt_connect => [ok(:rrt_connect, "a", 1.0), failed(:rrt_connect, "b")],
                            :chomp => [ok(:chomp, "a", 3.0)]),
    plans_by_planner=Dict(:rrt_connect => [path("a"; costs=[5.0, NaN, 3.0]), path("b"; raw=[0 2; 0 1; 0 0])],
                          :chomp => [path("a"; costs=[4.0])]),
    geometry=(station=(points_body=Float64[1 4 7 10; 2 5 8 11; 3 6 9 12],),))

@testset "Each figure shows the data its preparation returns" begin
    series = Reports._rpo_comparison_metric_series(batch)
    @test [s.metric for s in series] == first.(G.rpo_comparison_metric_specs())
    fuel = only(filter(s -> s.metric === :fuel_used_pct, series))
    @test fuel.planners == ["RRT-Connect", "CHOMP"] # the failed run is left out of the fuel aggregate
    @test fuel.values == [1.0, 3.0]
    @test fuel.case_labels == ["case a", "case a"]
    success = only(filter(s -> s.metric === :success_pct, series))
    @test success.values == [100.0, 0.0, 100.0]
    boxes = filter(t -> get(t.fields, :type, "") == "box", G.rpo_comparison_metric_summary_plot(batch).data)
    @test [b.fields[:y] for b in boxes] == [s.values for s in series]
    @test [b.fields[:x] for b in boxes] == [s.planners for s in series]

    @test Reports._rpo_comparison_failed_indices(batch.results_by_planner[:rrt_connect]) == [2]
    @test Reports._rpo_comparison_failed_indices(batch.results_by_planner[:chomp]) == Int[]

    costs = Reports._rpo_comparison_cost_series(batch, :rrt_connect)
    @test [(c.case_label, c.iterations, c.costs) for c in costs] == [("a", [1, 2], [5.0, 3.0])]
    single = Reports._rpo_comparison_cost_series(batch, :chomp)
    @test [(c.case_label, c.iterations, c.costs) for c in single] == [("a", [6], [4.0])]
    plot = G.rpo_comparison_cost_iteration_plot(batch; planner=:rrt_connect)
    @test [(t.fields[:x], t.fields[:y]) for t in plot.data] == [([1, 2], [5.0, 3.0])]

    with_raw = Reports._rpo_comparison_case_path(batch.plans_by_planner[:rrt_connect][2])
    @test with_raw.has_raw && with_raw.raw == [0 2; 0 1; 0 0]
    @test with_raw.tracked == batch.plans_by_planner[:rrt_connect][2].tracking.x_hist[1:3, :]
    without_raw = Reports._rpo_comparison_case_path(batch.plans_by_planner[:chomp][1])
    @test !without_raw.has_raw && without_raw.raw === nothing
    failed_plot = G.rpo_comparison_failed_paths_plot(batch; planner=:rrt_connect, failed_indices=[2])
    names = [t.fields[:name] for t in failed_plot.data]
    @test names == ["station point cloud", "case b desired", "case b tracked", "case b raw planner path",
                    "case b start", "case b goal"]

    @test Reports._rpo_comparison_station_point_indices(batch.geometry.station.points_body, 2) == 1:2:4
    @test !Reports._rpo_comparison_has_station_mesh(batch)
    @test !Reports._rpo_comparison_has_station_mesh(merge(batch, (station_triangles=nothing,)))
    mesh = Reports._rpo_comparison_station_mesh(Float64[0 1 0; 0 0 1; 0 0 0])
    @test (mesh.i, mesh.j, mesh.k) == ([0], [1], [2])
    @test Reports._rpo_comparison_has_station_mesh(merge(batch, (station_triangles=mesh.vertices,)))
end

@testset "The 0.6.0 PlotlyJS notice is told once and never throws" begin
    notice_id = :spaceagora_plotlyjs_extension_notice
    notices(logs) = filter(log -> log.id === notice_id, logs)
    # One notice for a session's plots, whichever function makes them.
    logs, _ = Test.collect_test_logs() do
        SpaceAGORA.rpo_tracking_plot([0.0, 1.0], [1.0, 0.5])
        SpaceAGORA.rpo_path_plot(zeros(3, 2))
        G.rpo_comparison_metric_summary_plot(batch)
        G.rpo_comparison_cost_iteration_plot(batch; planner=:chomp)
    end
    told = notices(logs)
    @test length(told) == 1
    @test only(told).level == Logging.Info
    @test occursin("SpaceAGORA 0.6.0", only(told).message)
    # Both steps: a project that depends on SpaceAGORA has PlotlyJS only as
    # SpaceAGORA's dependency, so it must add PlotlyJS before loading it.
    @test occursin("`Pkg.add(\"PlotlyJS\")`", only(told).message)
    @test occursin("`using PlotlyJS`", only(told).message)
    # CSV-only reporting makes no plot and tells nothing.
    logs, _ = Test.collect_test_logs() do
        mktempdir() do dir
            G.rpo_write_planner_comparison_outputs(batch; output_dir=dir,
                write_plotly_outputs=false, write_failed_path_outputs=false)
        end
    end
    @test isempty(notices(logs))
    # A script's first plot under --depwarn=error still runs, and tells once.
    root = normpath(joinpath(@__DIR__, "..", "..", ".."))
    code = """
        using SpaceAGORA
        SpaceAGORA.rpo_tracking_plot([0.0, 1.0], [1.0, 0.5])
        SpaceAGORA.rpo_path_plot(zeros(3, 2))
        println("plots_made")
        """
    output = IOBuffer()
    process = run(pipeline(ignorestatus(`$(Base.julia_cmd()) --startup-file=no --depwarn=error --project=$(root) -e $code`);
        stdout=output, stderr=output))
    text = String(take!(output))
    @test success(process)
    @test occursin("plots_made", text)
    @test count("`using PlotlyJS`", text) == 1
    @test count("`Pkg.add(\"PlotlyJS\")`", text) == 1
end

end # module RPOComparisonReportDataTests
