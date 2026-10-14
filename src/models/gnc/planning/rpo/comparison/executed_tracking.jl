"""Exact segment-to-point-cloud clearance, with the existing station/chaser buffer."""
function rpo_comparison_segment_clearance(a, b, geometry)
    a = SVector{3,Float64}(a); b = SVector{3,Float64}(b)
    d = b-a; d2 = dot(d,d); best = Inf
    points = geometry.station.points_body
    @inbounds for j in axes(points,2)
        q = SVector(points[1,j],points[2,j],points[3,j]) - a
        alpha = d2 > 0 ? clamp(dot(q,d)/d2,0.0,1.0) : 0.0
        best = min(best, dot(q-alpha*d,q-alpha*d))
    end
    return sqrt(best) - geometry.station.keepout_radius_m - maximum(geometry.chaser.half_extents_body)
end

"""
Execute a retimed path using recovered nonlinear Earth two-body propagation.
The default six-axis actuator holds fixed inertial body axes and starts at the
first 0.1 s control tick, matching the full demo. Above-deadband commands use
interval-average throttle. `actuator_frame=:rtn` retains the rotating-axis
idealization for explicitly labelled diagnostics. This is a translational evaluator, not a resolved
valve pulse or attitude/wheel qualification. Fuel uses delivered force and mass
is depleted at every update. Every 0.05 s plant substep checks swept clearance.
"""
function rpo_track_two_body_path(path_rtn, goal_rtn, geometry, pso_cfg, tracking; safe_distance_m=0.0)
    tracking.dt_s > 0 && tracking.mass_kg > tracking.propellant_mass_kg >= 0 ||
        throw(ArgumentError("Invalid tracking step or masses."))
    cfg = rpo_pso_config(pso_cfg; retime_dt_s=tracking.dt_s)
    t_ref,r_ref,v_ref = rpo_reference_from_path(path_rtn,geometry,cfg;safe_distance_m=safe_distance_m)
    return rpo_execute_two_body_reference(t_ref,r_ref,v_ref,goal_rtn,geometry,cfg,tracking;safe_distance_m=safe_distance_m)
end

function rpo_execute_two_body_reference(t_ref,r_ref,v_ref,goal_rtn,geometry,cfg,tracking;safe_distance_m=0.0,initial_state=nothing,initial_mass_kg=tracking.mass_kg)
    tracking.actuator_frame in (:inertial,:rtn) || throw(ArgumentError("Unknown actuator frame."))
    Q = Diagonal(vcat(fill(tracking.q_pos,3),fill(tracking.q_vel,3)))
    R = Diagonal(fill(tracking.r_accel,3)); Qf = Diagonal(vcat(fill(tracking.qf_pos,3),fill(tracking.qf_vel,3)))
    C = _control_module()
    u_max = min.(tracking.u_max_mps2,tracking.max_thrust_n/tracking.mass_kg)
    ctrl = C.init_rpo_lqmpc(tracking.mean_motion_radps,tracking.dt_s,Q,R,Qf,tracking.horizon;u_min=-u_max,u_max=u_max)
    thrusters = C.SixAxisThrusterModel(max_thrust_n=fill(tracking.max_thrust_n,6),isp_s=fill(tracking.isp_s,6))
    x = initial_state === nothing ? vcat(r_ref[:,1],zeros(3)) : collect(initial_state)
    plant = rpo_init_two_body_plant(x,tracking.mean_motion_radps)
    steps = max(length(t_ref)-1,0) + ceil(Int,tracking.settle_time_s/tracking.dt_s)
    x_hist=zeros(6,steps+1);x_hist[:,1]=x;u_hist=zeros(3,steps);force_hist=zeros(6,steps)
    mass=Float64(initial_mass_kg);fuel=0.0; effort=0.0; saturated=0; violations=0
    min_clearance=rpo_clearance_distance_to_station(x[1:3],geometry)
    statuses=Dict{String,Int}(); mass_hist=[mass]
    for k in 1:steps
        preview=rpo_lqmpc_reference_preview(r_ref,v_ref,min(k,length(t_ref)),tracking.horizon)
        # The production callback first runs at dt, with zero held actuation at t=0.
        u=k==1 ? zeros(3) : C.rpo_lqmpc_control(ctrl,x,preview)
        if k>1
            status=string(ctrl.qp_results.info.status);statuses[status]=get(statuses,status,0)+1
        end
        target_r=plant.state[1:3];target_v=plant.state[4:6]
        desired_force=tracking.actuator_frame===:inertial ? mass*rtn_dcm_from_inertial(target_r,target_v)*u : mass*u
        forces=C.rpo_allocate_six_axis_thrusters(desired_force,thrusters;minimum_throttle_fraction=tracking.minimum_throttle_fraction)
        force,_=C.rpo_thruster_wrench_body(forces,thrusters)
        dm=sum(forces)*tracking.dt_s/(tracking.isp_s*tracking.g0_mps2)
        fuel+=dm;effort+=norm(force/mass)*tracking.dt_s
        saturated+=any(forces .>= 0.999*tracking.max_thrust_n)
        # Subdivision also bounds the chord approximation used for clearance.
        nsub=max(1,ceil(Int,tracking.dt_s/0.05));dt=tracking.dt_s/nsub
        for j in 1:nsub
            previous=copy(x)
            x=rpo_step_two_body!(plant,force/(mass-dm*(j-0.5)/nsub),dt;max_step_s=dt,acceleration_frame=tracking.actuator_frame)
            clearance=rpo_comparison_segment_clearance(previous[1:3],x[1:3],geometry)
            min_clearance=min(min_clearance,clearance)
            violations+=clearance + 1e-9 < safe_distance_m
        end
        mass-=dm;push!(mass_hist,mass)
        x_hist[:,k+1]=x;u_hist[:,k]=u;force_hist[:,k]=forces
    end
    error=norm(x[1:3]-goal_rtn)
    qp_ok=all(s=="Solved" for s in keys(statuses))
    success=error <= tracking.final_position_tol_m && min_clearance+1e-9>=safe_distance_m && qp_ok && fuel<=tracking.propellant_mass_kg
    return (success=success,fuel_used=fuel,fuel_used_pct=100*fuel/tracking.propellant_mass_kg,
        control_effort_total=effort,translational_control_effort=effort,thrust_saturation_fraction=steps>0 ? saturated/steps : 0.,
        min_clearance=min_clearance,keepout_violations=violations,final_pos_error=error,
        planned_travel_duration=last(t_ref),actual_travel_duration=steps*tracking.dt_s,t_ref=t_ref,r_ref_rtn=r_ref,v_ref_rtn=v_ref,
        x_hist=x_hist,u_hist=u_hist,thruster_forces_n=force_hist,mass_history_kg=mass_hist,qp_status_counts=statuses,
        plant=:two_body,actuation=:held_average_throttle,actuator_frame=tracking.actuator_frame,minimum_throttle_fraction=tracking.minimum_throttle_fraction)
end
