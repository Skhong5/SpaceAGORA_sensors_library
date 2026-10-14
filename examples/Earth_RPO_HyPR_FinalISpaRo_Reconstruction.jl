# Final iSpaRo paper reconstruction, not an original-run replay.
# Requires both HYPR and SpaceAGORA reconstruction candidates in examples/rpo_planner_env.
# Default: five cases, all seven methods, full nonlinear SpaceAGORA execution.
# HYPR_CASES=500 selects the Table II campaign size; no claim is made until it finishes.
# HYPR_OUTPUT_ROOT must be outside cloud-synced research folders.
using SpaceAGORA, SpaceAGORAHYPR, HYPR, JSON, SHA, Random, LinearAlgebra, StaticArrays, Statistics, Dates, Test
const OUT = get(ENV,"HYPR_OUTPUT_ROOT",joinpath(homedir(),"AgentScratch","hypr_final_isparo"))
mkpath(OUT)
const ROOT = normpath(joinpath(@__DIR__,".."))
include(joinpath(ROOT,"examples","Earth_RPO_CubeSat_MPC_Batch.jl"))
const G = SM.GuidanceHooks
const C = SM.ControlHooks
BLAS.set_num_threads(1)
function savejson(name,x)
    open(joinpath(OUT,name),"w") do io; JSON.print(io,x,2);end
end
function paper_config()
    cfg = RPOPSOConfig(hypr_mode=:manuscript, n_waypoints=5,n_particles=100,n_iters=60,
        mass_kg=5.,isp_s=60.,w_obs=1e6,w_fuel=1.,safe_distance_m=.5,
        obstacle_sigmoid_k=1e5,obstacle_sigmoid_tol_m=.001,
        adaptive_allow_downscale=true,adaptive_n_particles_min=60,adaptive_n_particles_max=160,
        adaptive_n_iters_min=10,adaptive_n_iters_max=60,
        adaptive_n_waypoints_min=3,adaptive_n_waypoints_max=8,
        retime_dt_s=.1,fuel_proxy_dt_s=.1,retime_reaction_time_s=.25,
        retime_a_max_mps2=.00625,retime_speed_scale=.5,retime_max_speed_mps=.25,
        retime_accel_limit_enable=false,rrt_warmstart_enable=true,
        rrt_warmstart_iters=1000,rrt_warmstart_step_size_m=.75,
        rrt_warmstart_shortcut_iters=80,cull_start_iter=10,cull_fraction_max=.25,
        stagnation_learning_threshold=10,schedule_transition_fraction=.5,
        refinement_start_iter=60,refinement_period=10,refinement_sample_ds_m=.025,
        early_stopping_enable=false,iteration_runtime_limit_s=Inf)
    n=sqrt(SM.Earth().μ/(SM.Earth().Rp_e+420000.0)^3)
    return G.rpo_pso_config(cfg;mean_motion_radps=n, reexplore_max_waypoints=8)
end

const USE_FULL_SIM=true
"""Authoritative evaluation in the current full SpaceAGORA simulator, at fixed inertial attitude."""
function full_evaluate(plan,case,geometry,tracking)
    demo=build_rpo_cubesat_mpc_demo(mission_time=1.,seed=case.seed,start_rtn=case.start_rtn,goal_rtn=case.goal_rtn,
        pso_config=plan.config,precomputed_plan=plan,chaser_dry_mass_kg=tracking.mass_kg-tracking.propellant_mass_kg,
        minimum_throttle_fraction=tracking.minimum_throttle_fraction,station_points=geometry.station.points_body,
        station_keepout_radius_m=geometry.station.keepout_radius_m,safe_distance_m=.5,mpc_horizon=60,mpc_terminal_weight_scale=20.,
        post_reference_hold_s=tracking.settle_time_s,retime_accel_mps2=.00625,data_rate_s=.05,record_control_commands=true,verbose=false)
    a=demo.args
    args=SimulationConfiguration(;(k=>getfield(a,k) for k in fieldnames(typeof(a)) if !(k in (:solver_config,:simulation_settings)))...,
        solver_config=SolverConfig(solver_mode=:tsit5),simulation_settings=SimulationSettings(results=false,verbose=false,generate_plots=false))
    sol=run_simulation(args;return_solution=true,isolate_state=false)
    log=demo.control.command_log
    relative(u)=G.inertial_to_rtn_relative_state(SVector{3}(u.sc[1].pos),SVector{3}(u.sc[1].vel),SVector{3}(u.sc[2].pos),SVector{3}(u.sc[2].vel))
    terminal=relative(last(sol.u));dt=diff(vcat(log.t_s,last(sol.t)))
    fuel=sum(sum(f)*h for (f,h) in zip(log.thruster_forces_n,dt))/(60*9.80665)
    ts=collect(0.:.05:last(sol.t));last(ts)<last(sol.t)&&push!(ts,last(sol.t))
    minimum_clearance=Inf;violations=0;before=relative(sol(first(ts)))
    for t in ts[2:end]
        next=relative(sol(t));q=G.rpo_comparison_segment_clearance(before[1:3],next[1:3],geometry)
        minimum_clearance=min(minimum_clearance,q);violations+=q<.5;before=next
    end
    times=collect(0.:.1:last(sol.t));positions=reduce(hcat,[relative(sol(t)) for t in times])
    statuses=Dict(string(s)=>count(==(s),log.qp_status) for s in unique(log.qp_status))
    error=norm(terminal[1:3]-case.goal_rtn)
    success=string(sol.retcode)=="Success"&&error<=.25&&violations==0&&all(==("Solved"),keys(statuses))&&fuel<=.2
    return (success=success,fuel_used=fuel,fuel_used_pct=100fuel/.2,min_clearance=minimum_clearance,keepout_violations=violations,
        final_pos_error=error,actual_travel_duration=last(sol.t),planned_travel_duration=last(demo.initial_plan.t_ref_s),
        qp_status_counts=statuses,thruster_forces_n=reduce(hcat,log.thruster_forces_n),mass_history_kg=[sol(t).sc[1].mass for t in times],
        x_hist=positions,t_ref=demo.initial_plan.t_ref_s,r_ref_rtn=demo.initial_plan.r_ref_rtn,v_ref_rtn=demo.initial_plan.v_ref_rtn,
        plant=:full_spaceagora,retcode=string(sol.retcode))
end

function tracking_config(cfg;deadband=.2,mass=5.)
    G.RPOLQMPCTrackingSettings(dt_s=.1,mean_motion_radps=cfg.mean_motion_radps,horizon=60,mass_kg=mass,propellant_mass_kg=.2,
        isp_s=60.,u_max_mps2=SVector{3}(fill(.05/mass,3)),q_pos=20.,q_vel=2.,r_accel=.1,qf_pos=400.,qf_vel=40.,
        settle_time_s=30.,final_position_tol_m=.25,plant=:two_body,minimum_throttle_fraction=deadband,max_thrust_n=.05)
end
function summarize(track)
    return (success=track.success,fuel_g=1000track.fuel_used,fuel_percent=track.fuel_used_pct,
        min_clearance_m=track.min_clearance,clearance_violations=track.keepout_violations,final_error_m=track.final_pos_error,
        duration_s=track.actual_travel_duration,reference_duration_s=track.planned_travel_duration,
        qp_status_counts=track.qp_status_counts,subthreshold_commands=count(x->0<x<=.01,track.thruster_forces_n),
        mass_balance_error_kg=abs(track.fuel_used-(first(track.mass_history_kg)-last(track.mass_history_kg))))
end
function main()
    cfg=paper_config(); trackcfg=tracking_config(cfg)
    n=parse(Int,get(ENV,"HYPR_CASES","5"));prefix=get(ENV,"HYPR_RUN_NAME","pilot")
    cloud=SpaceAGORA.load_rpo_station_cad_pointcloud(:gateway;n_points=10000,rng=MersenneTwister(740))
    geometry=RPOReferenceGeometry(RPOStationGeometry(cloud;keepout_radius_m=.25,name="gateway_core");chaser=RPOCubeSatGeometry(dims_m=(.1,.1,.3)))
    cases=Any[]
    for i in 1:n
        c=only(generate_rpo_seeded_batch_cases(n_cases=1,seed=740+i,geometry_seed=740,n_station_points=10000,endpoint_min_clearance_m=.8,endpoint_max_clearance_m=1.5))
        push!(cases,merge(c,(case_id=i,seed=740+i)))
    end
    savejson(prefix*"_cases.json",cases)
    cc=G.RPOPlannerComparisonConfig(pso_config=cfg,tracking=trackcfg,show_progress=false,write_plotly_outputs=false,write_failed_path_outputs=false,
        rrt_connect=G.RPORRTConnectSettings(n_iters=1000,step_size_m=.75,goal_sample_rate=.05,shortcut_iters=80),
        rrt_star=G.RPORRTStarSettings(n_iters=1000,step_size_m=.75,goal_sample_rate=.05,shortcut_iters=80),
        optimizer=G.RPOTrajectoryOptimizerSettings(match_hypr_runtime=true,runtime_max_iters=100000,force_full_iters=true))
    # Compile each planner and evaluator before any measured per-case allowance.
    warmstart=SVector(-2.,30.,0.);warmgoal=SVector(2.,30.,0.)
    planners=Symbol.(split(get(ENV,"HYPR_PLANNERS","hypr,pso_unrefined,rrt_connect,rrt_connect_bezier,rrt_star,chomp,stomp"),","))
    for planner in planners
        println("WARMUP ",planner);flush(stdout)
        p=G.rpo_plan_comparison_path(planner,warmstart,warmgoal,geometry,cc;rng=MersenneTwister(100),runtime_limit_s=.1)
        if planner==:hypr
            USE_FULL_SIM ? full_evaluate(p,(seed=100,start_rtn=warmstart,goal_rtn=warmgoal),geometry,trackcfg) : G.rpo_track_retimed_path_lqmpc(p.path,warmgoal,geometry,p.config,trackcfg;safe_distance_m=.5)
        end
    end
    rows=Any[]
    for case in cases
        allowance=Inf;warm=nothing
        for planner in planners
            println("CASE_START ",case.case_id," ",planner);flush(stdout)
            begun=time_ns()
            try
                pcfg=planner in (:rrt_connect,:rrt_connect_bezier,:rrt_star) ?
                    G.RPOPlannerComparisonConfig(; (k=>getfield(cc,k) for k in fieldnames(typeof(cc)) if k!=:pso_config)...,
                        pso_config=G.rpo_pso_config(cfg;refinement_enable=false)) : cc
                plan=G.rpo_plan_comparison_path(planner,case.start_rtn,case.goal_rtn,geometry,pcfg;rng=MersenneTwister(case.seed),
                    runtime_limit_s=allowance,warmstart_path=warm)
                if planner==:hypr
                    allowance=plan.planner_plan_time;warm=plan.warmstart.path
                end
                track=USE_FULL_SIM ? full_evaluate(plan,case,geometry,trackcfg) : G.rpo_track_retimed_path_lqmpc(plan.path,case.goal_rtn,geometry,plan.config,trackcfg;safe_distance_m=.5)
                row=merge((case_id=case.case_id,seed=case.seed,planner=string(planner),plant=USE_FULL_SIM ? "full_spaceagora" : "reduced_two_body",planning_s=plan.planner_plan_time,
                    iterations=plan.planner_iteration_count,whole_wall_s=(time_ns()-begun)/1e9,cost=plan.cost),summarize(track))
                savejson(prefix*"_case_$(case.case_id)_$(planner)_plan.json",(case=case,path=plan.path,config=Dict(string(k)=>getfield(plan.config,k) for k in fieldnames(typeof(cfg))),reference_time=track.t_ref,reference_position=track.r_ref_rtn,reference_velocity=track.v_ref_rtn,adaptive=plan.adaptive,cost_history=plan.cost_history))
                if planner==:hypr
                    continuous=USE_FULL_SIM ? full_evaluate(plan,case,geometry,tracking_config(cfg;deadband=0.)) : G.rpo_track_retimed_path_lqmpc(plan.path,case.goal_rtn,geometry,plan.config,tracking_config(cfg;deadband=0.);safe_distance_m=.5)
                    row=merge(row,(continuous=summarize(continuous),))
                    savejson(prefix*"_case_$(case.case_id)_plan.json",(case=case,path=plan.path,config=Dict(string(k)=>getfield(plan.config,k) for k in fieldnames(typeof(cfg))),
                        reference_time=track.t_ref,reference_position=track.r_ref_rtn,reference_velocity=track.v_ref_rtn,warmstart=plan.warmstart,
                        adaptive=plan.adaptive,cost_history=plan.cost_history))
                    CSV.write(joinpath(OUT,prefix*"_case_$(case.case_id)_trace.csv"),DataFrame(t_s=collect(0:.1:track.actual_travel_duration),
                        x_m=track.x_hist[1,:],y_m=track.x_hist[2,:],z_m=track.x_hist[3,:],mass_kg=track.mass_history_kg))
                end
                push!(rows,row);println("CASE_RESULT ",JSON.json(row));flush(stdout)
            catch err
                row=(case_id=case.case_id,seed=case.seed,planner=string(planner),success=false,error=sprint(showerror,err),whole_wall_s=(time_ns()-begun)/1e9)
                push!(rows,row);println("CASE_ERROR ",JSON.json(row));showerror(stdout,err,catch_backtrace());println();flush(stdout)
            end
            savejson(prefix*"_results.json",rows)
        end
    end
    println("CAMPAIGN_COMPLETE ",length(rows));flush(stdout)
end

if abspath(PROGRAM_FILE)==@__FILE__
    main()
end
