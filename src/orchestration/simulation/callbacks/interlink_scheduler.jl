# Accepted-step scheduling and history recording belong to simulation orchestration.
using ..InterLinkModels: InterLinkHistory
using ..SchedulingPolicies: schedule_interlinks!

export interlink_scheduler_callback

# Update the matching and its recorded history at initialization and accepted steps.
function interlink_scheduler_callback(; candidate_force)
	function affect!(integrator)
		args = integrator.p.args
		model = args.interlink_model
		selected = schedule_interlinks!(model, args.scheduling_policy_model,
			args.dynamics_model.spacecraft, integrator.u, args.environment_model.planet.μ;
			candidate_force, is_active=integrator.p.is_active)
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
