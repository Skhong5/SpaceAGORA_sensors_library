function _edg_pass_heat_load_for_links(sc, links::Tuple{Vararg{Int}}, state, i::Int)::Float64
    hasproperty(sc, :heat_loads) || return 0.0
    baseline = state.heat_load_entry_j_cm2[i]
    loads = isempty(state.heat_load_exit_j_cm2[i]) ? sc.heat_loads : state.heat_load_exit_j_cm2[i]
    value = 0.0
    for idx in links
        if 1 <= idx <= length(loads)
            entry = idx <= length(baseline) ? baseline[idx] : 0.0
            load = Float64(loads[idx]) - entry
            isfinite(load) && (value = max(value, max(0.0, load)))
        end
    end
    return value
end
