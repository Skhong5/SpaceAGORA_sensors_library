# Reconstruct the final paper's disturbance families from one saved baseline.
# Original obstacle draws are unavailable; placements below are explicit choices.
# The obstructed-goal case is an extra negative control, outside Table III.
using DiffEqCallbacks, SciMLBase
include(joinpath(@__DIR__,"Earth_RPO_HyPR_FinalISpaRo_Reconstruction.jl"))
function config_from_json(d)
    defaults=paper_config()
    vals=Pair{Symbol,Any}[]
    for k in fieldnames(RPOPSOConfig)
        v=d[string(k)]; def=getfield(defaults,k)
        v === nothing && def isa AbstractFloat && !isfinite(def) && (v=def)
        value=def isa Symbol ? Symbol(v) : def isa Tuple ? Tuple(v) : def isa AbstractVector ? typeof(def)(v) : convert(typeof(def),v)
        push!(vals,k=>value)
    end
    return RPOPSOConfig(;vals...)
end

function full_replanning_pilot()
    d=JSON.parsefile(get(ENV,"HYPR_BASELINE_RECORD",joinpath(OUT,"pilot_case_4_plan.json")));cfg=config_from_json(d["config"]);c=d["case"]
    cloud=SpaceAGORA.load_rpo_station_cad_pointcloud(:gateway;n_points=10000,rng=MersenneTwister(740))
    geometry=RPOReferenceGeometry(RPOStationGeometry(cloud;keepout_radius_m=.25,name="gateway_core");chaser=RPOCubeSatGeometry(dims_m=(.1,.1,.3)))
    rt=Float64.(d["reference_time"]);r=reduce(hcat,d["reference_position"]);v=reduce(hcat,d["reference_velocity"])
    initial=RPOPlan(valid=true,t_ref_s=rt,r_ref_rtn=r,v_ref_rtn=v,path_rtn=reduce(hcat,d["path"]),diagnostics=(planned_at_s=0.,))
    initialT=last(rt);trigger=round(.25initialT;digits=1);idx=searchsortedfirst(rt,trigger)
    anchor=clamp(searchsortedfirst(rt,.6initialT),idx+1,length(rt));tangent=r[:,min(anchor+1,end)]-r[:,max(anchor-1,1)]
    normal=cross(tangent,[0.,0.,1.]);norm(normal)<1e-8&&(normal=cross(tangent,[0.,1.,0.]));normal/=norm(normal)
    function pathdistance(point)
        minimum(begin
            a=r[:,j];b=r[:,j+1];e=b-a;u=clamp(dot(point-a,e)/max(dot(e,e),eps()),0.,1.);norm(point-(a+u*e))
        end for j in idx:length(rt)-1)
    end
    function offsetcenter(clearance)
        desired=.25+.4+clearance;lo=0.;hi=20.
        for _ in 1:50
            mid=(lo+hi)/2
            pathdistance(r[:,anchor]+mid*normal)<desired ? (lo=mid) : (hi=mid)
        end
        r[:,anchor]+hi*normal
    end
    # The geometric placement rules are explicit reconstruction choices; original draws are missing.
    specs=[:baseline,:far_obstacle,:near_path_feasible,:partial_blockage,:directly_ahead,:obstructed_goal_negative_control,:goal_change,:displacement_retime,:displacement_replan,:sensor_false_positive]
    if haskey(ENV,"HYPR_DYNAMIC_SCENARIOS")
        specs=Symbol.(split(ENV["HYPR_DYNAMIC_SCENARIOS"],","))
    end
    prefix=get(ENV,"HYPR_DYNAMIC_PREFIX","full_dynamic")
    # Compile the real replacement planner before measuring any hold latency.
    warm=SM.RPOGuidanceModel(goal_rtn=SVector{3}(c["goal_rtn"]),geometry=geometry,pso_config=paper_config(),safe_distance_m=.5)
    G.build_rpo_plan_from_start(warm,r[:,1],geometry,0.;safe_distance_override=.5,force_rrt_warmstart=true)
    rows=Any[]
    for scenario in specs
        println("FULL_DYNAMIC_START ",scenario);flush(stdout)
        spheres=G.RPOReplanningSphere[]
        if scenario in (:far_obstacle,:near_path_feasible,:partial_blockage,:sensor_false_positive)
            clearance=scenario==:far_obstacle ? 6. : scenario==:partial_blockage ? .3 : .55
            push!(spheres,G.RPOReplanningSphere(offsetcenter(clearance),.25;appear_time_s=trigger,
                disappear_time_s=scenario==:sensor_false_positive ? trigger+.1 : Inf,label=string(scenario)))
        elseif scenario in (:directly_ahead,:obstructed_goal_negative_control)
            ahead=findfirst(j->norm(r[:,j]-r[:,idx])>=1.5,idx:length(rt))
            j=ahead===nothing ? length(rt) : idx+ahead-1
            if scenario==:directly_ahead
                feasible=findfirst(k->norm(r[:,k]-r[:,idx])>=1.16 && norm(r[:,k]-r[:,end])>=1.16,idx:length(rt))
                feasible===nothing && error("No feasible ahead-obstacle placement for this baseline")
                j=idx+feasible-1
            end
            push!(spheres,G.RPOReplanningSphere(r[:,j],.25;appear_time_s=trigger,label=string(scenario)))
        end
        pc=G.RPOReplanningConfig(safe_distance_m=.5,retime_clearance_m=.6,tracking_error_retime_m=.75,tracking_error_replan_m=1.1,
            observation_window_s=.02initialT,spheres=spheres,rng_seed=1744,remaining_sample_ds_m=.1)
        model=SM.RPOGuidanceModel(goal_rtn=SVector{3}(c["goal_rtn"]),geometry=geometry,pso_config=paper_config(),safe_distance_m=.5,replanning_config=pc)

        path=reduce(hcat,d["path"])
        plan=(path=path,config=cfg,cost=last(d["cost_history"]),components=NamedTuple(),adaptive=d["adaptive"],refinement_improved=false,cost_history=d["cost_history"])
        demo=build_rpo_cubesat_mpc_demo(mission_time=max(600.,4initialT),seed=744,start_rtn=c["start_rtn"],goal_rtn=c["goal_rtn"],
            pso_config=cfg,precomputed_plan=plan,chaser_dry_mass_kg=4.8,minimum_throttle_fraction=.2,
            station_points=cloud,station_keepout_radius_m=.25,safe_distance_m=.5,mpc_horizon=60,mpc_terminal_weight_scale=20.,
            post_reference_hold_s=30.,retime_accel_mps2=.00625,data_rate_s=.05,record_control_commands=true,verbose=false)
        model=demo.guidance;model.replanning_config=pc
        a=demo.args
        args=SimulationConfiguration(;(k=>getfield(a,k) for k in fieldnames(typeof(a)) if !(k in (:solver_config,:simulation_settings)))...,
            solver_config=SolverConfig(solver_mode=:tsit5),simulation_settings=SimulationSettings(results=false,verbose=false,generate_plots=false))
        relative(u)=G.inertial_to_rtn_relative_state(SVector{3}(u.sc[1].pos),SVector{3}(u.sc[1].vel),SVector{3}(u.sc[2].pos),SVector{3}(u.sc[2].vel))
        eventdone=Ref(false);eventrecord=Ref{Any}(nothing);phasehistory=Any[];lastphase=Ref(:tracking);done=Ref(false)
        history=Any[];wall=time_ns()
        function disturbance!(integrator)
            t=Float64(integrator.t);x=relative(integrator.u)
            if !eventdone[] && t+1e-9>=trigger
                eventdone[]=true
                if scenario in (:displacement_retime,:displacement_replan)
                    dist=scenario==:displacement_retime ? .9 : 1.35
                    directions=[normal,-normal,[1.,0.,0.],[-1.,0.,0.],[0.,1.,0.],[0.,-1.,0.],[0.,0.,1.],[0.,0.,-1.]]
                    candidates=[x[1:3]+dist*z for z in directions]
                    chosen=findfirst(p->G.rpo_clearance_distance_to_station(p,geometry)>=.5 &&
                        (scenario==:displacement_retime ? .75<G.rpo_reference_tracking_error(model.plan_buffer.plan,p;t=t)<1.1 : G.rpo_reference_tracking_error(model.plan_buffer.plan,p;t=t)>1.1),candidates)
                    chosen===nothing && error("No admissible displacement for scenario $scenario")
                    x=vcat(candidates[chosen],x[4:6])
                    target=integrator.u.sc[2]
                    cr,cv=G.rtn_to_inertial_relative_state(x[1:3],x[4:6],SVector{3}(target.pos),SVector{3}(target.vel))
                    integrator.u.sc[1].pos .= cr
                    integrator.u.sc[1].vel .= cv
                    SciMLBase.u_modified!(integrator,true)
                    eventrecord[]=(kind=:state_displacement,position=copy(x[1:3]),magnitude_m=dist)
                elseif scenario==:goal_change
                    candidates=[model.goal_rtn+z for z in ([1.,0.,0.],[-1.,0.,0.],[0.,1.,0.],[0.,0.,1.])]
                    chosen=findfirst(p->G.rpo_clearance_distance_to_station(p,geometry)>=.8,candidates)
                    chosen===nothing&&error("No admissible changed goal")
                    pc=G.RPOReplanningConfig(safe_distance_m=.5,retime_clearance_m=.6,tracking_error_retime_m=.75,tracking_error_replan_m=1.1,
                        observation_window_s=.02initialT,spheres=spheres,rng_seed=1744,desired_goal_rtn=candidates[chosen])
                    model.replanning_config=pc;eventrecord[]=(kind=:goal_change,goal=collect(candidates[chosen]))
                end
            end

            return nothing
        end
        function observe!(integrator)
            t=Float64(integrator.t);x=relative(integrator.u)
            if model.replanning_phase!=lastphase[]
                push!(phasehistory,(t_s=t,phase=model.replanning_phase));lastphase[]=model.replanning_phase
                savejson(prefix*"_$(scenario)_phase_checkpoint.json",(scenario=scenario,t_s=t,x=x,
                    phase=model.replanning_phase,phases=phasehistory,events=model.replanning_events))
            end
            push!(history,(t_s=t,x=x[1],y=x[2],z=x[3],mass_kg=integrator.u.sc[1].mass,phase=string(model.replanning_phase)))
            finish=model.plan_buffer.updated_at_s+last(model.plan_buffer.plan.t_ref_s)+30.
            if model.replanning_phase==:tracking && t+1e-8>=max(initialT,finish) && eventdone[]
                done[]=true;SciMLBase.terminate!(integrator)
            elseif model.replanning_phase==:hold_failed
                SciMLBase.terminate!(integrator)
            end
            return nothing
        end
        sol=run_simulation(args;return_solution=true,isolate_state=false,
            priority_callback_factories=(args->PeriodicCallback(disturbance!,.1;initial_affect=false),),
            extra_callbacks=(PeriodicCallback(observe!,.1;initial_affect=false),))
        log=demo.control.command_log;dt=diff(vcat(log.t_s,last(sol.t)))
        fuel=sum(sum(f)*h for (f,h) in zip(log.thruster_forces_n,dt))/(60*9.80665)
        statuses=Dict(string(q)=>count(==(q),log.qp_status) for q in unique(log.qp_status))
        ts=collect(0.:.05:last(sol.t));last(ts)<last(sol.t)&&push!(ts,last(sol.t))
        clearance=Inf;violations=0;before=relative(sol(0.))
        for t in ts[2:end]
            next=relative(sol(t));q=G.rpo_comparison_segment_clearance(before[1:3],next[1:3],geometry)
            # An imposed displacement is instantaneous: do not invent a traveled chord through the station.
            if scenario in (:displacement_retime,:displacement_replan) && t-.05 < trigger-1e-9 && t+1e-9>=trigger
                q=min(G.rpo_clearance_distance_to_station(before[1:3],geometry),G.rpo_clearance_distance_to_station(next[1:3],geometry))
            end
            if scenario!=:sensor_false_positive
                for sphere in spheres
                    if sphere.appear_time_s<=t<=sphere.disappear_time_s
                        aa=collect(before[1:3]);e=next[1:3]-aa;center=G.rpo_replanning_sphere_center(sphere,t)
                        fraction=clamp(dot(center-aa,e)/max(dot(e,e),eps()),0.,1.)
                        q=min(q,norm(aa+fraction*e-center)-sphere.radius_m-.4)
                    end
                end
            end
            clearance=min(clearance,q);violations+=q+1e-9<.5;before=next
        end
        error=norm(relative(last(sol.u))[1:3]-model.goal_rtn)
        row=(scenario=scenario,negative_control=scenario==:obstructed_goal_negative_control,plant=:full_spaceagora,case_id=4,success=done[]&&violations==0&&error<=.25&&all(==("Solved"),keys(statuses))&&fuel<=.2,
            fuel_g=1000fuel,final_error_m=error,min_clearance_m=clearance,violations=violations,duration_s=last(sol.t),
            retime_count=model.retime_count,replan_count=model.replan_count,safe_hold_count=model.safe_hold_count,
            phase=model.replanning_phase,phases=phasehistory,events=model.replanning_events,event_input=eventrecord[],
            spheres=spheres,trigger_time_s=trigger,observation_window_s=.02initialT,qp_status_counts=statuses,retcode=string(sol.retcode),
            mass_balance_error_kg=abs(fuel-(first(sol.u).sc[1].mass-last(sol.u).sc[1].mass)),wall_s=(time_ns()-wall)/1e9)
        push!(rows,row);savejson(prefix*"_results.json",rows)
        CSV.write(joinpath(OUT,prefix*"_$(scenario)_trace.csv"),DataFrame(history))
        println("FULL_DYNAMIC_RESULT ",JSON.json((scenario=scenario,success=row.success,fuel_g=row.fuel_g,phase=row.phase,replans=row.replan_count)));flush(stdout)
    end
end
if abspath(PROGRAM_FILE)==@__FILE__
    full_replanning_pilot()
end
