# Force/torque models: the pre-run hook check, the supported sample signature,
# run-level reproducibility and the purity of `wrench`.

# The sampled force/torque hooks, whose sample arguments must accept every
# `StateSample` and `EnvironmentSample` (outside review, decision 1).
const _SAMPLED_FORCE_HOOKS = (SimulationModel.wrench, SimulationModel.wrench_caching!,
    SimulationModel.gravity_backbone_acceleration_ii, SimulationModel.gravity_backbone_relative_acceleration_ii,
    SimulationModel.gravity_backbone_kick_acceleration_ii)

# Capture times of the samples `output_hook_pure` and `environment_declared` use, as
# fractions of the mission time; each is taken at the first step that reaches it.
const _SAMPLE_FRACTIONS = (0.0, 0.25, 0.5, 0.75, 1.0)

# A concurrent run waits at most this long for the other one at each step.
const _RENDEZVOUS_TIMEOUT_S = 60.0

"""
    check_force_torque_model(model; fixture=conformance_fixture(effectors=(model,)), samples=nothing)
        -> ConformanceReport

Check a force/torque model against the engine's contract for it. `fixture` must
contain `model` itself (found by identity) among its `dynamic_effectors`. The checks,
in report order:

- `hooks_present`: the engine's own pre-run hook check, on the fixture's concrete
  argument types, reports nothing for the fixture.
- `no_deprecated_hooks`: the pre-run check has no deprecated hook to warn about (a
  `calcForceTorque` method of the model's own).
- `supported_sample_signature`: no method of `wrench`, `wrench_caching!` or a gravity
  backbone hook that applies to the model types a sample argument narrower than
  `StateSample` or `EnvironmentSample` (such as `x::StateSample{MySpacecraft}`); the
  supported forms are `x::StateSample` and an untyped `x`.
- `repeatable_run`: two runs with `isolate_state=true` save equal times and states,
  bit for bit.
- `deepcopy_equivalent`: a run of the model itself (`isolate_state=false`) equals the
  run of its deep copy (`isolate_state=true`).
- `global_rng_independent`: runs with the global random number generator seeded two
  ways are equal (it is restored afterwards); a model draws only from the streams the
  engine supplies.
- `concurrent_equals_sequential`: two runs on separate threads, kept in step after
  every solver step so that they overlap, equal the two sequential runs. It fails when
  a run waits more than 60 s for the other at a step, since the runs then did not
  overlap. Skipped when Julia runs with one thread.
- `output_hook_pure`: two calls of `wrench` with the same arguments return equal values,
  leave a deep copy of the model unchanged, and log nothing at `Info` level or above.
- `environment_declared`: `wrench` returns a finite force and torque (two
  three-vectors) when only the `EnvironmentSample` fields its `environment_requirements`
  declares are set and the rest are `nothing`. A model that declares third bodies
  needs a fixture with SPICE ephemerides, which the built-in one does not load.

The last two call `wrench` on samples the engine builds (`build_state_sample` and
`sample_environment`) at the start, quarters and end of a fixture run, or on
`samples`: a collection of `(x, env, t)` tuples. A model with no `wrench` method skips
them.

Every run deep-copies the fixture except the `deepcopy_equivalent` run, which runs
`model` itself after all the others, so a model with run state holds that run's
state afterwards. A check that cannot run its comparison because a run failed fails
with the run's error. A pass certifies interface behaviour, not the model's physics.
"""
function check_force_torque_model(model; fixture::ConformanceFixture=conformance_fixture(effectors=(model,)),
        samples=nothing)
    effectors = fixture.configuration.dynamics_model.dynamic_effectors
    index = findfirst(effector -> effector === model, effectors)
    index === nothing && throw(ArgumentError(
        "check_force_torque_model: the fixture's dynamics_model.dynamic_effectors does not contain this model " *
        "(it is found by identity); use conformance_fixture(effectors=(model,)) or a configuration that holds it."))
    since = FIRST_RELEASE
    configuration = fixture.configuration
    hooks = _hooks_present_check(fixture, since)
    deprecated = _no_deprecated_hooks_check(fixture, model, since)
    signature = _sample_signature_check(model, since)
    base = _solve(configuration)
    again = _solve(configuration)
    seeded = map(seed -> _with_global_seed(() -> _solve(configuration), seed), (1, 2))
    concurrency = _concurrency_check(configuration, base, again, since)
    has_wrench = _has_wrench(model)
    captured = !has_wrench ? _Samples(Any[], nothing) :
        samples === nothing ? _capture_samples(configuration, index) : _Samples(collect(Any, samples), nothing)
    purity = _output_hook_pure_check(model, captured, has_wrench, since)
    declared = _environment_declared_check(model, captured, has_wrench, since)
    # Last: this run uses the caller's objects.
    original = _solve(configuration; isolate_state=false)

    repeat_gap = _run_difference(base, again)
    copy_gap = _run_difference(original, base)
    seed_gap = _run_difference(seeded...)
    checks = [
        hooks,
        deprecated,
        signature,
        _check(:repeatable_run, since, repeat_gap === nothing,
            "two runs of the fixture with isolate_state=true differ: $(repeat_gap)"),
        _check(:deepcopy_equivalent, since, copy_gap === nothing,
            "a run of the model itself (isolate_state=false) and a run of its deep copy (isolate_state=true) " *
            "differ: $(copy_gap)"),
        _check(:global_rng_independent, since, seed_gap === nothing,
            "runs with the global random number generator seeded 1 and 2 differ: $(seed_gap); draw random " *
            "numbers only from a stream the engine supplies, never from rand() or Random.default_rng()"),
        concurrency,
        purity,
        declared,
    ]
    return ConformanceReport(string(nameof(typeof(model))), :force_torque_model, checks)
end

"""
    test_force_torque_model(model; kwargs...)

Run [`check_force_torque_model`](@ref) and record its report with
[`test_report`](@ref). Needs the `Test` standard library: `using Test` loads the
method.
"""
function test_force_torque_model end

function _hooks_present_check(fixture::ConformanceFixture, since::VersionNumber)
    lines = SimulationEngine._missing_model_hooks(fixture.arguments, fixture.call, fixture.solver_mode)
    return _check(:hooks_present, since, isempty(lines),
        "the engine's pre-run hook check reports:\n" * join(("- " * line for line in lines), "\n"))
end

# The checks that need no run: the pre-run check, the deprecated hooks and the sample
# signature. Core's tests use them on built-in models that cannot run in the fixture.
function _static_force_report(model; fixture::ConformanceFixture=conformance_fixture(effectors=(model,)))
    since = FIRST_RELEASE
    checks = [_hooks_present_check(fixture, since), _no_deprecated_hooks_check(fixture, model, since),
        _sample_signature_check(model, since)]
    return ConformanceReport(string(nameof(typeof(model))), :force_torque_model, checks)
end

# The deprecated hooks the engine's pre-run check would warn about for the model's own
# type (`_deprecated_model_hooks`, the list behind its depwarn-group warnings, keyed by
# type and hook). Other models of a caller's fixture are not the model's to answer for.
function _no_deprecated_hooks_check(fixture::ConformanceFixture, model, since::VersionNumber)
    found = [entry for entry in SimulationEngine._deprecated_model_hooks(fixture.arguments, fixture.call)
        if first(first(entry)) === typeof(model)]
    return _check(:no_deprecated_hooks, since, isempty(found),
        "the engine's pre-run check warns:\n" * join(("- " * last(entry) for entry in found), "\n"))
end

_has_wrench(model) = hasmethod(SimulationModel.wrench,
    Tuple{typeof(model), SimulationModel.StateSample, SimulationModel.EnvironmentSample, Float64})

# A method's argument type at position `i` of its signature (1 is the function),
# with the method's type parameters applied.
function _signature_slot(sig, i::Int)
    parameters = Base.unwrap_unionall(sig).parameters
    slot = parameters[min(i, length(parameters))]
    slot isa Core.TypeofVararg && (slot = Base.unwrapva(slot))
    return Base.rewrap_unionall(slot, sig)
end

function _sample_signature_check(model, since::VersionNumber; hooks::Tuple=_SAMPLED_FORCE_HOOKS)
    T = typeof(model)
    problems = String[]
    for hook in hooks, method in methods(hook)
        n = length(Base.unwrap_unionall(method.sig).parameters)
        n >= 2 && T <: _signature_slot(method.sig, 2) || continue
        for i in 3:n
            slot = _signature_slot(method.sig, i)
            for (sample, name) in ((SimulationModel.StateSample, "StateSample"), (SimulationModel.EnvironmentSample, "EnvironmentSample"))
                slot !== Union{} && slot <: sample && !(sample <: slot) || continue
                push!(problems, "$(method) types argument $(i - 1) as $(slot), narrower than $(name); " *
                    "the engine passes $(name){...} with the run's own type parameters, so type it `::$(name)` or leave it untyped")
            end
        end
    end
    return _check(:supported_sample_signature, since, isempty(problems), join(problems, "\n"))
end

function _with_global_seed(f, seed)
    rng = Random.default_rng()
    saved = copy(rng)
    try
        Random.seed!(seed)
        return f()
    finally
        copy!(rng, saved)
    end
end

# Two concurrent runs advance in step: after each solver step a run waits until the
# other has taken as many steps or has finished, so that the runs overlap.
struct _Rendezvous
    steps::Vector{Threads.Atomic{Int}}
    finished::Vector{Threads.Atomic{Bool}}
    timeouts::Threads.Atomic{Int}
end

_Rendezvous() = _Rendezvous([Threads.Atomic{Int}(0) for _ in 1:2], [Threads.Atomic{Bool}(false) for _ in 1:2],
    Threads.Atomic{Int}(0))

function _lockstep_factory(rendezvous::_Rendezvous, me::Int, timeout_s::Float64=_RENDEZVOUS_TIMEOUT_S)
    other = 3 - me
    function wait_for_other!(integrator)
        n = Threads.atomic_add!(rendezvous.steps[me], 1) + 1
        deadline = time() + timeout_s
        while rendezvous.steps[other][] < n && !rendezvous.finished[other][]
            if time() > deadline
                Threads.atomic_add!(rendezvous.timeouts, 1)
                break
            end
            yield()
        end
        # Waiting changes neither the state nor any force input.
        SciMLBase.derivative_discontinuity!(integrator, false)
        return nothing
    end
    return configuration -> SciMLBase.DiscreteCallback((u, t, integrator) -> true, wait_for_other!;
        initialize=(callback, u, t, integrator) -> wait_for_other!(integrator), save_positions=(false, false))
end

_concurrency_check(configuration::SimulationModel.SimulationConfiguration, base::_RunOutcome, again::_RunOutcome,
    since::VersionNumber; timeout_s::Float64=_RENDEZVOUS_TIMEOUT_S) =
    _concurrency_check(factories -> _solve(configuration; extra_callback_factories=factories), base, again, since;
        timeout_s)

# `run(factories)` runs the fixture with the extra callback factories given and
# returns its outcome; `difference(a, b)` compares two outcomes (`noun` names what
# the default comparison compares).
function _concurrency_check(run::Function, base::_RunOutcome, again::_RunOutcome, since::VersionNumber;
        timeout_s::Float64=_RENDEZVOUS_TIMEOUT_S, noun::AbstractString="state",
        difference::Function=(a, b) -> _run_difference(a, b; noun))
    Threads.nthreads() >= 2 || return _skip(:concurrent_equals_sequential, since,
        "needs Julia started with at least two threads; this session has $(Threads.nthreads())")
    rendezvous = _Rendezvous()
    tasks = map(1:2) do me
        Threads.@spawn try
            run((_lockstep_factory(rendezvous, me, timeout_s),))
        finally
            rendezvous.finished[me][] = true
        end
    end
    runs = fetch.(tasks)
    gaps = (difference(base, runs[1]), difference(again, runs[2]))
    problems = String["concurrent run $k and sequential run $k differ: $(gap)" for (k, gap) in enumerate(gaps) if gap !== nothing]
    # Equal results from runs that did not overlap certify nothing about concurrency.
    timeouts = rendezvous.timeouts[]
    timeouts > 0 && pushfirst!(problems, "the concurrent runs fell out of step: a run waited more than " *
        "$(timeout_s) s for the other $(timeouts) time(s), so the runs did not overlap and the comparison does not apply")
    return _check(:concurrent_equals_sequential, since, isempty(problems), join(problems, "\n"))
end

# The `(x, env, t)` arguments `wrench` is called with, or the error that stopped
# the sampling.
struct _Samples
    arguments::Vector{Any}
    error::Any
end

function _capture_samples(configuration, index::Int)
    targets = [fraction * configuration.mission_configuration.mission_time for fraction in _SAMPLE_FRACTIONS]
    arguments = Any[]
    failure = Ref{Any}(nothing)
    next = Ref(1)
    function capture!(integrator)
        t = Float64(integrator.t)
        if failure[] === nothing && next[] <= length(targets) && t >= targets[next[]]
            while next[] <= length(targets) && t >= targets[next[]]
                next[] += 1
            end
            try
                p, u = integrator.p, integrator.u
                args = p.args
                model = args.dynamics_model.dynamic_effectors[index]
                requirements = SimulationModel.environment_requirements(model)
                orientation = args.mission_configuration.orientation_sim
                for (i, spacecraft) in enumerate(args.dynamics_model.spacecraft)
                    x = SimulationEngine.build_state_sample(u.sc[i], spacecraft, orientation, i)
                    env = SimulationEngine.sample_environment(requirements, model, u.sc[i], p, i, t; write_buffers=false)
                    push!(arguments, (x, env, t))
                end
            catch err
                failure[] = err
            end
        end
        # Sampling reads the state and changes no force input.
        SciMLBase.derivative_discontinuity!(integrator, false)
        return nothing
    end
    factory = configuration -> SciMLBase.DiscreteCallback((u, t, integrator) -> true, capture!;
        initialize=(callback, u, t, integrator) -> capture!(integrator), save_positions=(false, false))
    run = _solve(configuration; extra_callback_factories=(factory,))
    failure[] === nothing && _failed(run) && isempty(arguments) && (failure[] = run.error)
    return _Samples(arguments, failure[])
end

# Collects the messages a call logs at Info level or above.
struct _MessageLog <: Logging.AbstractLogger
    messages::Vector{String}
end

Logging.min_enabled_level(::_MessageLog) = Logging.Info
Logging.shouldlog(::_MessageLog, level, _module, group, id) = true
Logging.catch_exceptions(::_MessageLog) = false
function Logging.handle_message(log::_MessageLog, level, message, _module, group, id, file, line; kwargs...)
    push!(log.messages, string(level, ": ", message))
    return nothing
end

_sampling_failure(samples::_Samples) = "the fixture run could not be sampled: " * _summary(samples.error)

const _NO_WRENCH = "the model has no wrench(model, x::StateSample, env::EnvironmentSample, t::Float64) method to call"

function _output_hook_pure_check(model, samples::_Samples, has_wrench::Bool, since::VersionNumber)
    has_wrench || return _skip(:output_hook_pure, since, _NO_WRENCH)
    samples.error === nothing || return _check(:output_hook_pure, since, false, _sampling_failure(samples))
    isempty(samples.arguments) && return _skip(:output_hook_pure, since, "no samples to call wrench with")
    subject = deepcopy(model)
    snapshot = deepcopy(subject)
    log = _MessageLog(String[])
    problems = String[]
    Logging.with_logger(log) do
        for (x, env, t) in samples.arguments
            outputs = try
                (SimulationModel.wrench(subject, x, env, t), SimulationModel.wrench(subject, x, env, t))
            catch err
                push!(problems, "wrench threw at t = $(t) s: " * _summary(err))
                break
            end
            isequal(outputs...) && continue
            push!(problems, "two calls with the same arguments at t = $(t) s returned $(outputs[1]) and then $(outputs[2])")
            break
        end
    end
    _same_state(subject, snapshot) ||
        push!(problems, "the calls changed the model: its state differs from a deep copy taken before them")
    isempty(log.messages) ||
        push!(problems, "the calls logged $(length(log.messages)) message(s), the first: $(first(log.messages))")
    return _check(:output_hook_pure, since, isempty(problems), join(problems, "\n"))
end

_set_fields(env) = join((String(name) * (getfield(env, name) === nothing ? " = nothing" : " set")
    for name in (:planet_frame, :atmosphere, :solar, :third_bodies)), ", ")

function _is_wrench_output(value)
    value isa Tuple && length(value) == 2 || return false
    return all(v -> v isa AbstractVector{<:Real} && length(v) == 3 && all(isfinite, v), value)
end

function _environment_declared_check(model, samples::_Samples, has_wrench::Bool, since::VersionNumber)
    has_wrench || return _skip(:environment_declared, since, _NO_WRENCH)
    samples.error === nothing || return _check(:environment_declared, since, false, _sampling_failure(samples))
    isempty(samples.arguments) && return _skip(:environment_declared, since, "no samples to call wrench with")
    subject = deepcopy(model)
    for (x, env, t) in samples.arguments
        output = try
            SimulationModel.wrench(subject, x, env, t)
        catch err
            return _check(:environment_declared, since, false,
                "wrench threw at t = $(t) s with only the declared environment fields ($(_set_fields(env))): " *
                _summary(err) * "\nDeclare every field it reads in environment_requirements(model).")
        end
        _is_wrench_output(output) || return _check(:environment_declared, since, false,
            "wrench returned $(repr(output)) at t = $(t) s with only the declared environment fields " *
            "($(_set_fields(env))); it must return a finite force and torque, two three-vectors")
    end
    return _check(:environment_declared, since, true)
end
