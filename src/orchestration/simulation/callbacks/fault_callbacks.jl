# Actuator faults during a run (see ActuatorFaults): the faults switch exactly
# at their window ends, which are registered as solver stops, and a stuck
# actuator captures its level at onset; the fault columns.

_fault_spacecraft(args) = [i for i in eachindex(args.dynamics_model.spacecraft) if has_faults(args.dynamics_model.spacecraft[i])]

# The command the first `:mounted` effector holds for spacecraft `i`, or nothing.
function _fault_command_levels(p, i::Int)
    sc = p.args.dynamics_model.spacecraft[i]
    n = mounted_actuator_count(sc)
    n == 0 && return nothing
    return _mounted_effector_levels(p.args.control_model.control_effectors, i, n)
end

"""
    get_fault_callbacks(num_sats, args) -> Tuple

For runs with actuator faults: one callback at the faults' window ends, where
the solver stops, that switches them and declares a derivative discontinuity
when one switched;
its initialization resolves each fault's mounting position, registers the
window ends as stops and sets the faults active at the start. Empty without
faults.
"""
function get_fault_callbacks(num_sats::Int, args::SimulationConfiguration)
    sats = _fault_spacecraft(args)
    isempty(sats) && return ()
    function switch!(integrator)
        changed = false
        for i in sats
            sc = integrator.p.args.dynamics_model.spacecraft[i]
            changed |= refresh_faults!(sc, _fault_command_levels(integrator.p, i), integrator.t)
        end
        _mark_discontinuity!(integrator, changed)   # declared only when a fault switched
        return nothing
    end
    condition(u, t, integrator) = any(i -> fault_edge(integrator.p.args.dynamics_model.spacecraft[i], t), sats)
    function initialize!(cb, u, t, integrator)
        for i in sats
            sc = integrator.p.args.dynamics_model.spacecraft[i]
            resolve_faults!(sc)
            for f in sc.root.faults.faults
                f.active = false
                _maybe_add_control_tstop!(integrator, f.start_s)
                _maybe_add_control_tstop!(integrator, f.stop_s)
            end
        end
        switch!(integrator)
        _mark_discontinuity!(integrator, true)
        return nothing
    end
    return (DiscreteCallback(condition, switch!; initialize=initialize!, save_positions=(false, false)),)
end

"""
    fault_save_fields(args) -> Vector{SaveField}

The `fault_active` field when a spacecraft carries faults: per spacecraft one
column per fault, in the order they were added, 1 while it is active and 0
otherwise (`sc{i}_fault_active_{j}`).
"""
function fault_save_fields(args::SimulationConfiguration)
    any(has_faults, args.dynamics_model.spacecraft) || return SaveField[]
    num_sats = length(args.dynamics_model.spacecraft)
    getter = (u, t, integrator) -> [Float64[f.active ? 1.0 : 0.0 for f in integrator.p.args.dynamics_model.spacecraft[i].root.faults.faults]
                                    for i in 1:num_sats]
    return SaveField[SaveField(:fault_active, getter; per_satellite=true, column_prefix="fault_active")]
end
