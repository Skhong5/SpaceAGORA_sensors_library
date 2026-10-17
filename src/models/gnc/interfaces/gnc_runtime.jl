# Declarations a control model makes about how the engine runs it. Each has a
# default that keeps the engine's usual behaviour, so a model declares only what
# differs from a periodic tick.
module GNCRuntime

export tick_schedule, gnc_callbacks, register_tstops!, gnc_clock_started!, gnc_period_s
export Produces, Consumes, gnc_links, link_cardinality, check_link, link_initialize!

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

# Links: the shared objects through which guidance and control models are paired,
# such as the RPO plan buffer or the energy-depletion state. A model declares the
# links it writes (`Produces`) and reads (`Consumes`). The engine groups the run's
# models by link identity, which survives the run's deep copy, and checks and
# initializes each link once.

"""
    Produces(link)

Wraps a shared object that a model writes for others to read, in `gnc_links`.
"""
struct Produces{L}
    link::L
end

"""
    Consumes(link)

Wraps a shared object that a model reads, in `gnc_links`. Before a periodic control
model's tick, the engine calls `before_reference_control!` for each model that
produces a link it consumes.
"""
struct Consumes{L}
    link::L
end

"""
    gnc_links(model) -> Tuple

The links a guidance, navigation or control `model` takes part in, as `Produces`
and `Consumes` wrappers around the shared objects. Default: none.
"""
gnc_links(model) = ()

"""
    link_cardinality(link) -> NamedTuple

How many models may produce and consume `link`, as ranges:
`(producers = 0:1, consumers = 0:1)`. Before the run the engine refuses a link
whose counts fall outside them. Default: any number of each.
"""
link_cardinality(link) = (producers = 0:typemax(Int), consumers = 0:typemax(Int))

"""
    check_link(link, producers, consumers, args) -> nothing

Before the run, once per link: check that the models sharing `link` agree, such
as matching settings. Throw an `ArgumentError` to refuse the run. `producers` and
`consumers` are the configured models in configuration order. Default: nothing.
"""
check_link(link, producers, consumers, args) = nothing

"""
    link_initialize!(link, producers, consumers, u, p, t) -> nothing

At the start of the run, once per link and before any model's own initializer:
reset the run state the link owns. Default: nothing.
"""
link_initialize!(link, producers, consumers, u, p, t) = nothing

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
