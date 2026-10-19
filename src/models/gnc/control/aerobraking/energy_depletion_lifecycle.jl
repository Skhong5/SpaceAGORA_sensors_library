import ..GuidanceHooks

# The shared state is initialized once per run as a link (GNCRuntime), whether the
# controller is paired with guidance or a maximum-depletion controller works alone.
SimulationLifecycle.preflight_control(c::AerobrakingEnergyDepletionControlModel, args; isolate_state) =
    GuidanceHooks._edg_preflight(c, args)
function SimulationLifecycle.atmosphere_transition!(c::AerobrakingEnergyDepletionControlModel, u, p, t, i, inside)
    # The dedicated geodetic boundary callback owns EDG passage transitions.
    return nothing
end

SimulationLifecycle.requires_atmosphere_events(::AerobrakingEnergyDepletionControlModel) = true
