# Accepted-step scheduling and history recording belong to simulation orchestration.
using ..InterLinkModels: InterLinkHistory, LinkContext
using ..SchedulingPolicies: schedule_interlinks!

export interlink_scheduler_callback

# Update the matching and its recorded history at initialization and accepted steps.
# The links score by their own `link_force`; `candidate_force` overrides it for
# callers that pass one (`candidate_force(target, partner, link)`).
function interlink_scheduler_callback(; candidate_force=nothing)
	function affect!(integrator)
		args = integrator.p.args
		model = args.interlink_model
		planet = args.environment_model.planet
		selected = schedule_interlinks!(model, args.scheduling_policy_model,
			args.dynamics_model.spacecraft, integrator.u, planet.μ;
			candidate_force, is_active=integrator.p.is_active,
			context=LinkContext(time_s=Float64(integrator.t), planet=planet))
		sample = InterLinkHistory(Float64(integrator.t), selected)
		if !isempty(model.history) && model.history[end].time == sample.time
			model.history[end] = sample
		else
			push!(model.history, sample)
		end
		DiffEqBase.derivative_discontinuity!(integrator, true)
		return nothing
	end
	initialize = (callback, state, time, integrator) -> affect!(integrator) # update link graph when the simulation is initialized
	return DiffEqBase.DiscreteCallback((state, time, integrator) -> true, affect!; # update link graph at each accepted simulation step
		initialize=initialize, save_positions=(false, false))
end
