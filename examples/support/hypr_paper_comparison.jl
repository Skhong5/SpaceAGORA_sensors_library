# Same fitted-curve processing as pinned HYPR rrt_adapters.jl, accepting the exact saved RRT-Connect plan.
function paired_bezier_shape(base_plan,geometry,cfg;safe_distance_m=.5)
    bezier_cfg = G.rpo_pso_config(cfg; curve_type=:bezier)
    samples = G.rpo_sample_path(
        base_plan.path,
        bezier_cfg,
        geometry;
        safe_distance_m=safe_distance_m,
        base_ds_m=bezier_cfg.sample_ds_m,
        curve_type=:polyline,
    )
    base_controls = max(2, bezier_cfg.n_waypoints + 2)
    max_controls = max(base_controls, min(size(samples, 2), max(base_controls + 6, size(base_plan.path, 2))))

    best_path = SpaceAGORAHYPR.RPO.rpo_fit_bezier_fixed_endpoints(samples, base_controls, bezier_cfg)
    best_components = G.rpo_normalized_path_cost_components(best_path, geometry, bezier_cfg; safe_distance_m=safe_distance_m)
    for n_control in (base_controls + 1):max_controls
        candidate = SpaceAGORAHYPR.RPO.rpo_fit_bezier_fixed_endpoints(samples, n_control, bezier_cfg)
        comps = G.rpo_normalized_path_cost_components(candidate, geometry, bezier_cfg; safe_distance_m=safe_distance_m)
        if comps.J_obs < best_components.J_obs - 1.0e-9 ||
                (comps.J_obs <= best_components.J_obs + 1.0e-9 && comps.total < best_components.total)
            best_path = candidate
            best_components = comps
        end
        best_components.violation_count == 0 && break
    end

    refined, refined_cost, improved = G.rpo_post_refine_path(best_path, geometry, bezier_cfg; safe_distance_m=safe_distance_m)
    refined_components = G.rpo_normalized_path_cost_components(refined, geometry, bezier_cfg; safe_distance_m=safe_distance_m)
    return (
        path=refined,
        raw_path=base_plan.raw_path,
        smoothed_path=best_path,
        cost=refined_cost,
        raw_cost=base_plan.raw_cost,
        components=refined_components,
        raw_components=base_plan.raw_components,
        config=bezier_cfg,
        adaptive=(enabled=false,),
        refinement_improved=improved,
        cost_history=[refined_components.total],
        history=[base_plan.raw_path],
        iterations=base_plan.iterations,
        objective=refined_components.total,
        path_found=base_plan.path_found,
    )
end

# These are example input and evidence rules, not planner policy.
paper_planning_failed(plan) = hasproperty(plan, :path_found) && plan.path_found === false
function paper_case_ids(env=ENV)
    ids = haskey(env, "HYPR_CASE_IDS") ? parse.(Int, split(env["HYPR_CASE_IDS"], ",")) : collect(1:parse(Int, get(env, "HYPR_CASES", "1")))
    !isempty(ids) && all(>(0), ids) && length(unique(ids)) == length(ids) ||
        throw(ArgumentError("Provide unique positive HYPR_CASE_IDS, or a positive HYPR_CASES."))
    return ids
end
function paper_planners(env=ENV)
    methods = Symbol.(split(get(env, "HYPR_PLANNERS", "hypr"), ","))
    allowed = (:hypr, :pso_unrefined, :rrt_connect, :rrt_connect_bezier, :rrt_star, :chomp, :stomp)
    !isempty(methods) && first(methods) == :hypr && all(in(allowed), methods) && length(unique(methods)) == length(methods) ||
        throw(ArgumentError("HYPR_PLANNERS must start with hypr and contain unique supported planners."))
    if :rrt_connect_bezier in methods
        idx = findfirst(==(:rrt_connect), methods)
        idx !== nothing && idx < findfirst(==(:rrt_connect_bezier), methods) ||
            throw(ArgumentError("Run rrt_connect before rrt_connect_bezier to share the exact base path."))
    end
    return methods
end
# The comparison planner a paper planner name stands for, built from the comparison
# configuration's settings fields, as RPOPlannerComparisonConfig builds its defaults.
function paper_planner_object(name::Symbol, cfg)
    name === :hypr && return HYPRComparisonPlanner(cfg.pso_config)
    name === :pso_unrefined && return UnrefinedPSOPlanner(cfg.pso_config)
    name === :rrt_connect && return RRTConnectPlanner(cfg.rrt_connect; config=cfg.pso_config, optimizer=cfg.optimizer)
    name === :rrt_connect_bezier &&
        return RRTConnectBezierPlanner(cfg.rrt_connect; config=cfg.pso_config, optimizer=cfg.optimizer)
    name === :rrt_star && return RRTStarPlanner(cfg.rrt_star; config=cfg.pso_config, optimizer=cfg.optimizer)
    name === :chomp && return CHOMPPlanner(cfg.chomp; config=cfg.pso_config, optimizer=cfg.optimizer, seed=cfg.rrt_connect)
    name === :stomp && return STOMPPlanner(cfg.stomp; config=cfg.pso_config, optimizer=cfg.optimizer, seed=cfg.rrt_connect)
    throw(ArgumentError("Unsupported paper planner $(name)."))
end
function paper_dynamic_budget(initial_duration, env=ENV)
    seconds = parse(Float64, get(env, "HYPR_DYNAMIC_MAX_TIME_S", string(max(600., 4initial_duration))))
    isfinite(seconds) && seconds > 0 || throw(ArgumentError("HYPR_DYNAMIC_MAX_TIME_S must be finite and positive."))
    return seconds
end
