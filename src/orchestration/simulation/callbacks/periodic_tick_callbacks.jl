# Periodic navigation, guidance and control ticks.
#
# DiffEqCallbacks' `PeriodicCallback(f, Δt)` ticks at t0 + k*Δt from the start
# t0 of each solve and never at the solve's final time. A checkpointed run
# solves consecutive segments, so under it the tick at a segment boundary on the
# grid was lost, and a boundary off the grid shifted every later tick. The
# callback below keeps one grid for the whole run instead. The engine sets the
# run clock in the shared buffers for a checkpointed or resumed run: the grid's
# origin is the run's start, and a tick fires at any grid time before the run's
# end, including a segment's final time. There it fires after the step to that
# time and before the samples saved there, as in one solve, and before the
# checkpoint written there; the next segment, or a run resumed from that
# checkpoint, starts with the following tick. A run does not tick at its own end,
# so a checkpoint written there marks the ticks due then as pending, and a run
# resumed from it takes them when its first solve starts, before the samples
# saved there (`initial_affect` of PeriodicCallback).
#
# Without the run clock (a run in one solve, or a solve outside the engine) it
# is `PeriodicCallback(f, Δt)`: the origin is the solve's start and the tick
# times, condition and stops are the same. Under the run clock the run's end
# replaces the solve's final time in the condition: a cached integrator that
# `reinit!` moves to the next segment keeps the first segment's
# `integrator.sol.prob.tspan`.

const _PeriodicTickClock = NamedTuple{(:origin, :index, :horizon),
    Tuple{Base.RefValue{Float64}, Base.RefValue{Int}, Base.RefValue{Float64}}}

# Each Task running the callback keeps its own clock, as PeriodicCallback keeps
# its cache, so concurrent solves of one callback do not share a phase.
@inline _periodic_tick_clock(key::Symbol) = task_local_storage(key)::_PeriodicTickClock

@inline _periodic_tick_time(clock::_PeriodicTickClock, Δt::Float64, k::Int) = clock.origin[] + k * Δt

@inline function _periodic_run_clock(p)
    hasproperty(p, :shared_buffers) || return (NaN, NaN, NaN)
    buffers = p.shared_buffers
    return (buffers.periodic_clock_origin_s[], buffers.periodic_clock_horizon_s[],
            buffers.periodic_clock_pending_s[])
end

# Stop at the next tick unless the solve ends first, as PeriodicCallback does.
@inline function _schedule_next_periodic_tick!(integrator, clock::_PeriodicTickClock, Δt::Float64)
    clock.index[] += 1
    tnew = _periodic_tick_time(clock, Δt, clock.index[])
    if integrator.tdir * tnew < DiffEqBase.get_tstops_max(integrator)
        DiffEqBase.add_tstop!(integrator, tnew)
    end
    return nothing
end

"""
    _run_clock_periodic_callback(f, Δt; on_clock=(origin, integrator) -> nothing)

A `DiscreteCallback` applying `f(integrator)` every `Δt` seconds on the run's
tick grid (see the comment at the top of this file). `on_clock` receives the
grid's origin when the callback initializes, before the first tick is scheduled.
"""
function _run_clock_periodic_callback(f, Δt::Float64; on_clock=(origin, integrator) -> nothing)
    key = gensym(:run_clock_periodic_callback)

    condition = function (u, t, integrator)
        clock = _periodic_tick_clock(key)
        t == _periodic_tick_time(clock, Δt, clock.index[]) || return false
        hasfield(typeof(integrator), :iter) && integrator.iter == integrator.opts.maxiters && return false
        horizon = clock.horizon[]
        return isnan(horizon) ? t != last(integrator.sol.prob.tspan) : t < horizon
    end

    affect! = function (integrator)
        _schedule_next_periodic_tick!(integrator, _periodic_tick_clock(key), Δt)
        return f(integrator)
    end

    initialize = function (cb, u, t, integrator)
        @assert integrator.tdir == sign(Δt)
        DiffEqBase.derivative_discontinuity!(integrator, false)
        storage = task_local_storage()
        clock = get(storage, key, nothing)
        if clock === nothing
            clock = (origin=Ref(NaN), index=Ref(0), horizon=Ref(NaN))
            storage[key] = clock
        end
        clock = clock::_PeriodicTickClock
        run_origin, run_horizon, pending = _periodic_run_clock(integrator.p)
        clock.origin[] = isnan(run_origin) ? Float64(t) : run_origin
        clock.horizon[] = run_horizon
        # The first tick after the solve's start. A tick at the start itself
        # fired at the end of the previous segment, unless it is pending.
        k = max(1, floor(Int, (t - clock.origin[]) / Δt))
        while _periodic_tick_time(clock, Δt, k) <= t
            k += 1
        end
        while k > 1 && _periodic_tick_time(clock, Δt, k - 1) > t
            k -= 1
        end
        clock.index[] = k - 1
        on_clock(clock.origin[], integrator)
        if k > 1 && t == pending && _periodic_tick_time(clock, Δt, k - 1) == t
            # The tick at the start, which the run that wrote the checkpoint
            # ended before taking. It changes the derivative.
            DiffEqBase.derivative_discontinuity!(integrator, true)
            affect!(integrator)
            return nothing
        end
        _schedule_next_periodic_tick!(integrator, clock, Δt)
        return nothing
    end

    return DiscreteCallback(condition, affect!; initialize=initialize)
end
