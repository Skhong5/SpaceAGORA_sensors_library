# Declarations a control model makes about how the engine runs it. Each has a
# default that keeps the engine's usual behaviour, so a model declares only what
# differs from a periodic tick.
module GNCRuntime

export tick_schedule, gnc_callbacks, register_tstops!, gnc_clock_started!, gnc_period_s

"""
    tick_schedule(model) -> Symbol

How the engine runs control `model`.

- `:periodic`, the default: a periodic callback at the model's control rate calls
  its tick (`calcControlEffect!`) for every spacecraft.
- `:event_driven`: no periodic tick. The model brings its own callbacks
  (`gnc_callbacks`). `BaseThrusterModel`, the burn scheduler, is event-driven.
"""
tick_schedule(model) = :periodic

"""
    gnc_callbacks(model, num_sats, args, period_s) -> Tuple

The callbacks an `:event_driven` control `model` adds to a run of `num_sats`
spacecraft with configuration `args`. `period_s` is the model's configured control
rate, for a model that also ticks periodically. The engine reads it only for
`:event_driven` models. Default: none.
"""
gnc_callbacks(model, num_sats, args, period_s) = ()

"""
    register_tstops!(integrator, model, spacecraft_index) -> nothing

Called after each periodic tick of control `model`, once for every spacecraft
index. Use it to add the solver stops that the new command needs; the burn
scheduler adds the ends of its burn window. The ticks of several spacecraft may
run on different threads, but these calls do not: they run one spacecraft at a
time, after all of that tick's spacecraft have ticked, because the stop list
belongs to the integrator they share. Default: none.
"""
register_tstops!(integrator, model, spacecraft_index) = nothing

"""
    gnc_clock_started!(model, period_s, origin_s) -> nothing

Called when the periodic callback of control `model` starts, with its period and
the time it starts from (s into the run). It is called at every solve's start, so a
checkpointed run calls it again at each segment. A model that keeps a clock of its
own ticks records the origin here. Default: nothing.
"""
gnc_clock_started!(model, period_s, origin_s) = nothing

"""
    gnc_period_s(model) -> Union{Nothing, Float64}

The tick period (s) that control `model`'s law assumes, such as the step its
controller was discretized with. Before the run the engine compares it with the
rate of the model's periodic callback and warns when they differ; SpaceAGORA
0.4.0 will refuse such a run. Default: `nothing`, no declared period, as for a
law that measures the elapsed time itself.
"""
gnc_period_s(model) = nothing

end # module GNCRuntime
