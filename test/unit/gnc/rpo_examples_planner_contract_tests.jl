# The RPO examples on the planner contract: the main example plans inside the run,
# and a saved paper baseline replays through RetainedPathRPOPlanner.
module RPOExamplesPlannerContractTests
using Test
using LinearAlgebra
using Random
using StaticArrays
using JSON
using SpaceAGORA
const SM = SpaceAGORA.SimulationModel
const P = SpaceAGORA.RPOPlannerInterfaces

# The example scripts carry their own includes and constants; load them into a
# dedicated module so nothing leaks into Main or other test files. The replanning
# study includes the reconstruction example, which includes the main example.
module RPOExamplesOnContract
using SpaceAGORA
end
const RPO_EXAMPLES_DIR = normpath(joinpath(@__DIR__, "..", "..", "..", "examples"))
const RPO_EXAMPLES_SPICE = joinpath(RPO_EXAMPLES_DIR, "..", "data/GRAMSuite.jl/GRAM Suite 2.0", "SPICE")

@testset "Examples build the planner contract, the held study the deprecated model" begin
    source(name) = read(joinpath(RPO_EXAMPLES_DIR, name), String)
    for name in ("Earth_RPO_CubeSat_MPC.jl", "Earth_RPO_CubeSat_MPC_Batch.jl", "Earth_RPO_HyPR_FinalISpaRo_Reconstruction.jl")
        @test !occursin("RPOGuidanceModel", source(name))
    end
    @test occursin("PlannerGuidance", source("Earth_RPO_CubeSat_MPC.jl"))
    @test occursin("RetainedPathRPOPlanner", source("Earth_RPO_CubeSat_MPC.jl"))
    # The replanning study reproduces the published disturbance families on the
    # deprecated model until its goal-change and brake-then-replan scenarios have a
    # planner-contract counterpart; it assembles that legacy guidance itself.
    @test occursin("legacy_guidance_demo", source("Earth_RPO_HyPR_FinalISpaRo_Replanning.jl"))
end

if !isdir(RPO_EXAMPLES_SPICE)
    @info "SPICE kernels absent; skipping the RPO example planner-contract runs" RPO_EXAMPLES_SPICE
else
    Base.include(RPOExamplesOnContract, joinpath(RPO_EXAMPLES_DIR, "Earth_RPO_HyPR_FinalISpaRo_Replanning.jl"))
    const X = RPOExamplesOnContract

    @testset "The published case-4 baseline replays to its recorded reference" begin
        d = JSON.parsefile(joinpath(RPO_EXAMPLES_DIR, "support", "hypr_final_isparo_case4.json"))
        cfg = Base.invokelatest(X.config_from_json, d["config"]); case = d["case"]
        cloud = SpaceAGORA.load_rpo_station_cad_pointcloud(:gateway; n_points=10000, rng=MersenneTwister(740))
        geometry = SM.RPOReferenceGeometry(SM.RPOStationGeometry(cloud; keepout_radius_m=0.25, name="gateway_core");
            chaser=SM.RPOCubeSatGeometry(dims_m=(0.1, 0.1, 0.3)))
        request = RPOPlanningRequest(request_id=1, chaser_id=101, target_id=201, epoch="case4", time_s=0.0,
            x_rtn=(Float64.(case["start_rtn"])..., 0.0, 0.0, 0.0), target_state_ii=(7e6, 0.0, 0.0, 0.0, 7500.0, 0.0),
            goal_rtn_m=Tuple(Float64.(case["goal_rtn"])), geometry=geometry, geometry_revision="gateway_core",
            constraints=RPOPlanningConstraints(clearance_m=0.5), reference_dt_s=0.1, preview_horizon_steps=60,
            valid_until_s=200.0)
        planner = RetainedPathRPOPlanner(reduce(hcat, d["path"]), cfg)
        out = plan_rpo!(nothing, planner, request, MersenneTwister(1))
        @test out.status === :candidate
        # Exactly the published reference: the retimer is deterministic and the
        # request asks for no limit beyond the clearance.
        @test out.reference.t_ref_s == Float64.(d["reference_time"])
        @test out.reference.r_ref_rtn_m == reduce(hcat, d["reference_position"])
        @test out.reference.v_ref_rtn_mps == reduce(hcat, d["reference_velocity"])
        checked = validate_rpo_result(request, out; clearance_at=(p, g) -> SM.rpo_clearance_distance_to_station(p, g))
        @test checked.accepted && checked.metrics.min_clearance_m >= 0.5
    end

    @testset "The main example plans, validates, flies and stops on the planner path" begin
        small = (pso_n_particles=8, pso_n_iters=2, n_station_points=256, verbose=false, results_directory=mktempdir(),
            mission_time=10.0, data_rate_s=1.0, start_rtn=SVector(-8.0, -4.0, 2.0), goal_rtn=SVector(-5.5, -2.5, 1.0),
            record_control_commands=true)
        demo = Base.invokelatest(X.build_rpo_cubesat_mpc_demo; small...)
        @test demo.guidance isa SpaceAGORA.RPOPlannerLifecycle.PlannerGuidance
        @test demo.planner isa HYPRRPOPlanner && demo.guidance.planner === demo.planner
        @test demo.guidance.constraints.clearance_m == 0.1
        @test demo.guidance.constraints.max_speed_mps === nothing
        @test demo.guidance.constraints.max_acceleration_mps2 === nothing   # the default retimer is not acceleration-limited
        @test demo.args.mission_configuration.mission_time == demo.mission.max_time_s == 1800.0
        @test !haskey(demo, :initial_plan)
        sol = Base.invokelatest(X.run_rpo_case, demo)
        plan = Base.invokelatest(X.rpo_installed_plan, sol)
        report = only(rpo_run_report(sol).planners)
        @test report.request_id == 1 && plan.validation.accepted && plan.planner === :hypr
        @test report.stream_words == gnc_stream_words("RPO/v1", 741, 101, 201)
        @test plan.r_ref_rtn[:, 1] ≈ [-8.0, -4.0, 2.0] atol=1e-9
        @test plan.r_ref_rtn[:, end] == [-5.5, -2.5, 1.0]
        # The run ends post_reference_hold_s after the installed reference, as the
        # builder's simulated time did before.
        @test last(sol.t) ≈ max(10.0, plan.t_ref_s[end] + 20.0) atol=0.06
        @test !isempty(only(rpo_run_report(sol).commands).log.t_s)
        # Its path and configuration replay through RetainedPathRPOPlanner to the same
        # reference and the same flight.
        replay = Base.invokelatest(X.build_rpo_cubesat_mpc_demo; small..., pso_config=plan.config,
            precomputed_plan=(path=plan.path_rtn, config=plan.config))
        @test replay.planner isa RetainedPathRPOPlanner
        sol2 = Base.invokelatest(X.run_rpo_case, replay)
        plan2 = Base.invokelatest(X.rpo_installed_plan, sol2)
        @test plan2.planner === :retained_path
        @test (plan2.t_ref_s, plan2.r_ref_rtn, plan2.v_ref_rtn) == (plan.t_ref_s, plan.r_ref_rtn, plan.v_ref_rtn)
        @test sol2.t == sol.t && sol2.u == sol.u
        # Planning-time hooks of the old builder are refused, not ignored.
        @test_throws ArgumentError Base.invokelatest(X.build_rpo_cubesat_mpc_demo; small..., pso_iteration_callback=identity)
        @test_throws ArgumentError Base.invokelatest(X.build_rpo_cubesat_mpc_demo; small..., max_mission_time_s=5.0)
    end
end
end # module RPOExamplesPlannerContractTests
