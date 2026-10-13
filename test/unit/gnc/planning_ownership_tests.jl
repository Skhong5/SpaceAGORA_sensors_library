module PlanningOwnershipTests
using Test, Serialization, SpaceAGORA
const SM = SpaceAGORA.SimulationModel
const G = SM.GuidanceHooks
const Reports = G.RPOComparisonReports

# Independent inventory of historical qualified report entry points. Keeping the
# defining function objects lets existing imports and serialized references work.
const REPORT_FUNCTIONS = (
    :rpo_flatten_planner_results, :rpo_write_namedtuple_csv, :rpo_group_metric_mean,
    :rpo_comparison_metric_specs, :rpo_comparison_metric_value,
    :rpo_comparison_metric_includes_result, :rpo_comparison_axis_name,
    :rpo_comparison_trace_axis_name, :rpo_comparison_metric_summary_plot,
    :rpo_comparison_station_mesh_trace, :rpo_comparison_station_trace,
    :rpo_comparison_path_family_plot, :rpo_comparison_single_path_plot,
    :rpo_comparison_failed_paths_plot, :rpo_comparison_cost_iteration_xvalues,
    :rpo_comparison_cost_iteration_plot, :rpo_comparison_artifact_slug,
    :rpo_write_failed_path_outputs, :rpo_write_planner_comparison_outputs,
)

@testset "Historical report identities, analysis-owned methods" begin
    for name in REPORT_FUNCTIONS
        f = getproperty(G, name)
        @test getproperty(Reports, name) === f
        @test parentmodule(f) === G
        @test all(m -> m.module === Reports, methods(f))
        io = IOBuffer()
        serialize(io, f)
        seekstart(io)
        @test deserialize(io) === f
    end
    for name in (:rpo_flatten_planner_results, :rpo_comparison_cost_iteration_plot,
                 :rpo_write_planner_comparison_outputs)
        @test getproperty(SM, name) === getproperty(G, name)
    end
    @test G.PlotlyJS === Reports.PlotlyJS
    @test !isdefined(Reports, :RPOPSOConfig)
    @test !isdefined(Reports, :HYPRSupport)
    @test SpaceAGORA.HYPRServices.RPO.RPOPSOConfig === G.RPOPSOConfig
    @test SpaceAGORA.HYPRServices.RPO.rpo_retime_samples === G.rpo_retime_samples
    @test SpaceAGORA.HYPRServices.RobotArm.RobotArmPlan === SM.RobotArmPlanning.RobotArmPlan
    @test parentmodule(SM.RPOReferenceGeometry) === SM.NavigationHooks
end

@testset "Reporting preserves result selection and file content" begin
    success = (planner=:rrt_connect, case_label="one,two", success=true,
        planner_compute_time=0.5, fuel_used_pct=1.0, control_effort_total=2.0,
        thrust_saturation_fraction=0.0, min_clearance=3.0, final_pos_error=0.1,
        actual_travel_duration=4.0, keepout_violations=0)
    failure = merge(success, (case_label="failed", success=false, fuel_used_pct=99.0))
    batch = (planner_types=[:rrt_connect],
        results_by_planner=Dict(:rrt_connect=>[success, failure]),
        plans_by_planner=Dict(:rrt_connect=>[(case=(label="one,two",),
            cost_history=[5.0, NaN, 3.0], planner_iteration_count=4)]),)
    @test G.rpo_flatten_planner_results(batch) == [success, failure]
    @test G.rpo_group_metric_mean([(v=1.0,), (v=NaN,), (v=3.0,)], :v) == 2.0
    @test isnan(G.rpo_group_metric_mean([(v=NaN,)], :v))
    @test G.rpo_comparison_cost_iteration_xvalues((iterations=7,), [3.0]) == [7]
    @test G.rpo_comparison_cost_iteration_xvalues((iterations=7,), [5.0, 3.0]) == [1, 2]
    @test G.rpo_comparison_cost_iteration_xvalues((;), Float64[]) == Int[]
    plot = G.rpo_comparison_metric_summary_plot(batch)
    boxes = filter(t -> get(t.fields, :type, "") == "box", plot.data)
    @test length(boxes) == 9
    @test boxes[1].fields[:y] == [100.0, 0.0]
    @test boxes[3].fields[:y] == [1.0] # failed runs do not enter the fuel aggregate
    costs = G.rpo_comparison_cost_iteration_plot(batch; planner=:rrtconnect)
    @test length(costs.data) == 1
    @test costs.data[1].fields[:x] == [1, 2]
    @test costs.data[1].fields[:y] == [5.0, 3.0]
    mktempdir() do dir
        output = G.rpo_write_planner_comparison_outputs(batch; output_dir=dir,
            write_plotly_outputs=false, write_failed_path_outputs=false)
        @test basename(output.csv) == "rpo_planner_comparison_results.csv"
        lines = readlines(output.csv)
        @test lines[1] == join(string.(keys(success)), ",")
        @test lines[2] == "rrt_connect,one;two,true,0.5,1.0,2.0,0.0,3.0,0.1,4.0,0"
        @test lines[3] == "rrt_connect,failed,false,0.5,99.0,2.0,0.0,3.0,0.1,4.0,0"
        @test readdir(dir) == [basename(output.csv)]
        @test output.metrics_plot === output.path_plot === nothing
        @test isempty(output.method_path_plots) && isempty(output.cost_iteration_plots)
        @test isempty(output.failed_path_plots)
        @test !output.plotly_outputs && !output.failed_path_outputs
    end
end
end # module PlanningOwnershipTests
