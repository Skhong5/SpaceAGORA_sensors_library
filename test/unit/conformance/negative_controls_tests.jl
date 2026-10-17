module ConformanceNegativeControlsTests

# Each check of SpaceAGORA.Conformance gets at least one broken input that it must
# fail, so that a check passing a defective model is noticed. The file fails when a
# check id has no control.

using Test, SpaceAGORA, Logging
using StaticArrays: SVector
using SpaceAGORA.Conformance: CHECK_IDS, check_force_torque_model, check_service_facade, check_sensor, conformance_fixture,
    passed, check_guidance_model, check_navigation_model, check_control_model
using Random: Random

const SM = SpaceAGORA.SimulationModel
const C = SpaceAGORA.Conformance
const Z3 = SVector(0.0, 0.0, 0.0)
const ForceModel = SpaceAGORA.AbstractForceTorqueModel
const Sample = SpaceAGORA.StateSample
const Env = SpaceAGORA.EnvironmentSample

along_velocity(x, newtons) = (newtons * x.vel_ii / sqrt(sum(abs2, x.vel_ii)), Z3)

# A clean model: a small constant drag-like force along the velocity.
struct NcClean <: ForceModel end
SpaceAGORA.wrench(::NcClean, x::Sample, env::Env, t::Float64) = along_velocity(x, -1.0e-3)

# hooks_present: neither wrench nor the deprecated calcForceTorque.
struct NcNoHook <: ForceModel end

# supported_sample_signature: a method on a concrete StateSample{Nothing}, which
# only samples built outside a run reach.
struct NcConcreteSample <: ForceModel end
SpaceAGORA.wrench(::NcConcreteSample, x::Sample, env::Env, t::Float64) = along_velocity(x, -1.0e-3)
SpaceAGORA.wrench(::NcConcreteSample, x::Sample{Nothing}, env::Env, t::Float64) = (Z3, Z3)

# repeatable_run: state kept outside the model, advanced at every run start.
const NC_RUN_EPOCH = Ref(0)
struct NcRunEpoch <: ForceModel end
function SpaceAGORA.wrench(::NcRunEpoch, x::Sample, env::Env, t::Float64)
    t == 0.0 && (NC_RUN_EPOCH[] += 1; return (Z3, Z3))
    return along_velocity(x, 1.0e-3 * NC_RUN_EPOCH[])
end

# deepcopy_equivalent: a gain registered under the identity of the model's own
# field, which a deep copy does not carry.
const NC_GAINS = IdDict{Any, Float64}()
struct NcIdentityGain <: ForceModel
    key::Base.RefValue{Int}
end
NcIdentityGain() = (model = NcIdentityGain(Ref(0)); NC_GAINS[model.key] = 1.0e-3; model)
SpaceAGORA.wrench(m::NcIdentityGain, x::Sample, env::Env, t::Float64) = along_velocity(x, get(NC_GAINS, m.key, 0.0))

# global_rng_independent: a draw from the global generator.
struct NcGlobalRandom <: ForceModel end
SpaceAGORA.wrench(::NcGlobalRandom, x::Sample, env::Env, t::Float64) = along_velocity(x, 1.0e-3 * rand())

# concurrent_equals_sequential: scratch shared by every instance, reset at the
# start of a run, so that a run alone is repeatable but two overlapping runs mix.
const NC_SCRATCH = Ref(0.0)
struct NcSharedScratch <: ForceModel end
function SpaceAGORA.wrench(::NcSharedScratch, x::Sample, env::Env, t::Float64)
    t == 0.0 && (NC_SCRATCH[] = 0.0)
    NC_SCRATCH[] += 1.0
    return along_velocity(x, 1.0e-6 * NC_SCRATCH[])
end

# output_hook_pure: a call counter in the model, and a log message.
mutable struct NcCountsCalls <: ForceModel
    calls::Int
end
SpaceAGORA.wrench(m::NcCountsCalls, x::Sample, env::Env, t::Float64) = (m.calls += 1; along_velocity(x, -1.0e-3))
struct NcLogs <: ForceModel end
SpaceAGORA.wrench(::NcLogs, x::Sample, env::Env, t::Float64) = (@info "NcLogs wrench" maxlog = 1; along_velocity(x, -1.0e-3))

# output_hook_pure: two fields share one vector, and each call replaces one of them
# with an equal copy, so the values stay the same but the aliasing does not.
mutable struct NcBreaksAliasing <: ForceModel
    a::Vector{Float64}
    b::Vector{Float64}
end
NcBreaksAliasing() = (v = [1.0e-3]; NcBreaksAliasing(v, v))
SpaceAGORA.wrench(m::NcBreaksAliasing, x::Sample, env::Env, t::Float64) = (m.b = copy(m.b); along_velocity(x, -m.a[1]))

# concurrent_equals_sequential: the first call after the token is set sleeps, so one
# of two concurrent runs stalls while the other waits for it at the rendezvous.
const NC_STALL = Threads.Atomic{Bool}(false)
struct NcStallsOnce <: ForceModel end
function SpaceAGORA.wrench(::NcStallsOnce, x::Sample, env::Env, t::Float64)
    t > 0.0 && Threads.atomic_cas!(NC_STALL, true, false) && sleep(4.0)
    return along_velocity(x, -1.0e-3)
end

# environment_declared: reads the atmosphere without declaring it, and returns a
# non-finite force when the undeclared solar sample is missing.
struct NcUndeclaredAtmosphere <: ForceModel end
SpaceAGORA.wrench(::NcUndeclaredAtmosphere, x::Sample, env::Env, t::Float64) =
    along_velocity(x, -env.atmosphere.rho_kg_m3)
struct NcUndeclaredSolar <: ForceModel end
SpaceAGORA.wrench(::NcUndeclaredSolar, x::Sample, env::Env, t::Float64) =
    env.solar === nothing ? (SVector(NaN, NaN, NaN), Z3) : along_velocity(x, 1.0e-6)

# environment_declared, third bodies: declares the Moon and reads a second body.
# Needs SPICE ephemerides, so it runs when the kernels are present.
struct NcUndeclaredThirdBody <: ForceModel end
SpaceAGORA.environment_requirements(::NcUndeclaredThirdBody) =
    SpaceAGORA.EffectorEnvironmentRequirements(third_body_names=("Moon",))
SpaceAGORA.wrench(::NcUndeclaredThirdBody, x::Sample, env::Env, t::Float64) =
    along_velocity(x, 1.0e-20 * sum(abs2, env.third_bodies.positions_ii[2]))

include(joinpath(@__DIR__, "..", "..", "fixtures", "conformance_instances.jl"))
using .ConformanceInstances: on_spice, spice_available

# Sensors: one record shape, a 3-component measurement every second, and a counter
# some controls keep. The clean one adds noise from its own stream to the body rate.
for name in (:NcCleanSensor, :NcShortSensor, :NcNaNSensor, :NcRunEpochSensor, :NcIdentitySensor, :NcGlobalRandSensor,
        :NcSharedSensor, :NcFixedSeedSensor, :NcCountingSensor, :NcAtmosphereSensor)
    @eval begin
        mutable struct $name <: SpaceAGORA.AbstractSensor
            period_s::Float64
            seed::Int
            rng::Any
            latest::Any
            latest_time_s::Float64
            count::Int
        end
        $name() = $name(1.0, 1, nothing, SVector(NaN, NaN, NaN), NaN, 0)
        SpaceAGORA.sensor_dimension(::$name) = 3
    end
end
noisy(truth, rng) = truth.ω_body + 1.0e-4 * SVector(randn(rng), randn(rng), randn(rng))
SpaceAGORA.sensor_measure(::NcCleanSensor, truth, ctx, rng) = noisy(truth, rng)
# measurement_shape: two components where it declares three; a NaN component.
SpaceAGORA.sensor_measure(::NcShortSensor, truth, ctx, rng) = SVector(truth.ω_body[1], truth.ω_body[2])
SpaceAGORA.sensor_measure(::NcNaNSensor, truth, ctx, rng) = SVector(NaN, truth.ω_body[2], truth.ω_body[3])
# repeatable_run: state outside the sensor, advanced at every run start.
const NC_SENSOR_EPOCH = Ref(0)
SpaceAGORA.initialize_sensor!(::NcRunEpochSensor, truth, ctx, rng) = (NC_SENSOR_EPOCH[] += 1; nothing)
SpaceAGORA.sensor_measure(::NcRunEpochSensor, truth, ctx, rng) = noisy(truth, rng) .+ 1.0e-3 * NC_SENSOR_EPOCH[]
# deepcopy_equivalent: an offset registered under the identity of the sensor.
const NC_SENSOR_OFFSETS = IdDict{Any, Float64}()
NcRegisteredSensor() = (s = NcIdentitySensor(); NC_SENSOR_OFFSETS[s] = 1.0e-3; s)
SpaceAGORA.sensor_measure(s::NcIdentitySensor, truth, ctx, rng) = noisy(truth, rng) .+ get(NC_SENSOR_OFFSETS, s, 0.0)
# global_rng_independent: noise from the global generator.
SpaceAGORA.sensor_measure(::NcGlobalRandSensor, truth, ctx, rng) = truth.ω_body .+ 1.0e-4 * rand()
# concurrent_equals_sequential: scratch shared by every instance, reset at a run's start.
const NC_SENSOR_SCRATCH = Ref(0.0)
function SpaceAGORA.sensor_measure(::NcSharedSensor, truth, ctx, rng)
    truth.t_s == 0.0 && (NC_SENSOR_SCRATCH[] = 0.0)
    NC_SENSOR_SCRATCH[] += 1.0
    return noisy(truth, rng) .+ 1.0e-6 * NC_SENSOR_SCRATCH[]
end
# seed_repeatable: noise from its stream, but the stream is reset to a fixed seed.
SpaceAGORA.initialize_sensor!(::NcFixedSeedSensor, truth, ctx, rng) = (Random.seed!(rng, 1234); nothing)
SpaceAGORA.sensor_measure(::NcFixedSeedSensor, truth, ctx, rng) = noisy(truth, rng)
# custom_state_reset: a sample counter that initialize_sensor! does not reset.
SpaceAGORA.sensor_measure(s::NcCountingSensor, truth, ctx, rng) = (s.count += 1; noisy(truth, rng) .+ 1.0e-6 * s.count)
# requirements_supported: asks for the atmosphere.
SpaceAGORA.sensor_requirements(::NcAtmosphereSensor) = SpaceAGORA.ActuatorRequirements(atmosphere=true)
SpaceAGORA.sensor_measure(::NcAtmosphereSensor, truth, ctx, rng) = noisy(truth, rng)
# run_state_fields: not mutable; without latest_time_s.
struct NcImmutableSensor <: SpaceAGORA.AbstractSensor
    period_s::Float64
    seed::Int
    rng::Any
    latest::Any
    latest_time_s::Float64
end
NcImmutableSensor() = NcImmutableSensor(1.0, 1, nothing, SVector(NaN, NaN, NaN), NaN)
SpaceAGORA.sensor_dimension(::NcImmutableSensor) = 3
SpaceAGORA.sensor_measure(::NcImmutableSensor, truth, ctx, rng) = noisy(truth, rng)
mutable struct NcNoTimeSensor <: SpaceAGORA.AbstractSensor
    period_s::Float64
    seed::Int
    rng::Any
    latest::Any
end
NcNoTimeSensor() = NcNoTimeSensor(1.0, 1, nothing, SVector(NaN, NaN, NaN))
SpaceAGORA.sensor_dimension(::NcNoTimeSensor) = 3
SpaceAGORA.sensor_measure(::NcNoTimeSensor, truth, ctx, rng) = noisy(truth, rng)

const SENSOR_RUN_LEVEL = (:repeatable_run, :deepcopy_equivalent, :global_rng_independent, :concurrent_equals_sequential,
    :seed_repeatable, :custom_state_reset)
sensor_except(ids...) = filter(id -> !(id in ids), CHECK_IDS.sensor)

# no_deprecated_hooks (force/torque): a calcForceTorque method of its own beside wrench.
struct NcDeprecatedForce <: ForceModel end
SpaceAGORA.wrench(::NcDeprecatedForce, x::Sample, env::Env, t::Float64) = along_velocity(x, -1.0e-3)
SpaceAGORA.calcForceTorque(::NcDeprecatedForce, x, p, i) = (Z3, Z3)

# Guidance and navigation models: one record shape and one behaviour per control.
# `acc` is the model's own state, which the run-level checks compare at the end of a
# run; `ticked` marks a run's first tick; `buffer` is what bind_spacecraft shares.
const NC_GNC_EPOCH = Ref(0)
const NC_GNC_SCRATCH = Ref(0.0)
const NC_GNC_GAINS = IdDict{Any, Float64}()
gnc_step(::Val{:Clean}, m, u, i) = 1.0e-9 * u.sc[i].pos[1]
gnc_step(::Val{:RunEpoch}, m, u, i) = (m.ticked || (m.ticked = true; NC_GNC_EPOCH[] += 1); Float64(NC_GNC_EPOCH[]))
gnc_step(::Val{:Identity}, m, u, i) = get(NC_GNC_GAINS, m, 0.0) + 1.0e-9 * u.sc[i].pos[1]
gnc_step(::Val{:Rand}, m, u, i) = rand()
gnc_step(::Val{:Shared}, m, u, i) = (m.ticked || (m.ticked = true; NC_GNC_SCRATCH[] = 0.0); NC_GNC_SCRATCH[] += 1.0)
gnc_step(::Val{:SharedBind}, m, u, i) = (m.buffer[1] += 1.0; m.buffer[1])
gnc_step(::Val{:NoReset}, m, u, i) = 1.0
for (prefix, Abstract, tick) in ((:NcG, SpaceAGORA.AbstractGuidanceModel, :calcGuidanceEffect!),
        (:NcN, SpaceAGORA.AbstractNavigationModel, :calcNavigationEffect!))
    for variant in (:Clean, :NoTick, :RunEpoch, :Identity, :Rand, :Shared, :SharedBind, :NoReset)
        name = Symbol(prefix, variant)
        @eval begin
            mutable struct $name <: $Abstract
                acc::Float64
                ticked::Bool
                buffer::Vector{Float64}
            end
            $name() = $name(0.0, false, [0.0])
        end
        variant === :NoTick && continue
        @eval SpaceAGORA.$tick(m::$name, u, p, t::Float64, i::Int) = (m.acc += gnc_step(Val($(QuoteNode(variant))), m, u, i); nothing)
        # Navigation models reset their own state at a run's start, except NoReset.
        prefix === :NcN && variant !== :NoReset &&
            @eval SpaceAGORA.SimulationLifecycle.initialize_navigation!(m::$name, u, p, t) = (m.acc = 0.0; m.ticked = false; nothing)
        variant === :SharedBind && @eval SpaceAGORA.bind_spacecraft(m::$name, i::Int) = $name(m.acc, m.ticked, m.buffer)
    end
end
registered(model) = (NC_GNC_GAINS[model] = 1.0e-3; model)

# Control models: the tick sets `cmd`, and the output hooks read it.
mutable struct NcCtl{V} <: SpaceAGORA.AbstractControlEffectorModel
    acc::Float64
    ticked::Bool
    buffer::Vector{Float64}
    calls::Int
end
NcCtl{V}() where {V} = NcCtl{V}(0.0, false, [0.0], 0)
const NC_CTL_STEPS = (:Clean, :RunEpoch, :Identity, :Rand, :Shared, :SharedBind)
for variant in (:Clean, :RunEpoch, :Identity, :Rand, :Shared, :SharedBind, :NoWrench, :Deprecated, :Narrow, :Impure, :Negative)
    step = variant in NC_CTL_STEPS ? variant : :Clean
    @eval SpaceAGORA.calcControlEffect!(m::NcCtl{$(QuoteNode(variant))}, u, p, t::Float64, i::Int) =
        (m.acc += gnc_step(Val($(QuoteNode(step))), m, u, i); nothing)
    variant === :NoWrench && continue
    @eval SpaceAGORA.control_wrench(m::NcCtl{$(QuoteNode(variant))}, x::Sample, p, t::Float64) =
        (SVector(1.0e-4 * m.acc / (1 + abs(m.acc)), 0.0, 0.0), Z3)
    variant === :Negative || @eval SpaceAGORA.control_propellant_rate(m::NcCtl{$(QuoteNode(variant))}, x::Sample, p, t::Float64) =
        1.0e-6 * abs(m.acc) / (1 + abs(m.acc))
end
SpaceAGORA.bind_spacecraft(m::NcCtl{:SharedBind}, i::Int) = NcCtl{:SharedBind}(m.acc, m.ticked, m.buffer, m.calls)
SpaceAGORA.calcControlForceTorque(::NcCtl{:Deprecated}, u, p, i, t) = (Z3, Z3)
SpaceAGORA.control_wrench(m::NcCtl{:Narrow}, x::Sample{Nothing}, p, t::Float64) = (Z3, Z3)
SpaceAGORA.control_wrench(m::NcCtl{:Impure}, x::Sample, p, t::Float64) = (m.calls += 1; (Z3, Z3))
SpaceAGORA.control_propellant_rate(m::NcCtl{:Negative}, x::Sample, p, t::Float64) = -1.0e-5

gnc_except(kind, ids...) = filter(id -> !(id in ids), CHECK_IDS[kind])
const GNC_RUN_LEVEL = (:repeatable_run, :deepcopy_equivalent, :global_rng_independent, :concurrent_equals_sequential,
    :initialize_resets)
function gnc_controls()
    controls = Any[]
    for (kind, prefix, check) in ((:guidance_model, :NcG, check_guidance_model), (:navigation_model, :NcN, check_navigation_model))
        model(variant) = getfield(@__MODULE__, Symbol(prefix, variant))()
        resets = kind === :navigation_model ? (:initialize_resets,) : ()
        push!(controls,
            (kind=kind, id=:hooks_present, broken="no tick method", expect="has no method",
                report=() -> check(model(:NoTick)), passing=(:bind_fresh_state,)),
            (kind=kind, id=:repeatable_run, broken="state outside the model advanced at every run start",
                expect="two runs of the fixture with isolate_state=true differ: ", report=() -> check(model(:RunEpoch)),
                passing=(:hooks_present, :bind_fresh_state)),
            (kind=kind, id=:deepcopy_equivalent, broken="a gain keyed by the model's identity",
                expect="and a run of its deep copy (isolate_state=true) differ: ",
                report=() -> check(registered(model(:Identity))), passing=gnc_except(kind, :deepcopy_equivalent)),
            (kind=kind, id=:global_rng_independent, broken="rand() in the tick", expect="seeded 1 and 2 differ: ",
                report=() -> check(model(:Rand)), passing=(:hooks_present, :bind_fresh_state)),
            (kind=kind, id=:concurrent_equals_sequential, broken="scratch shared by every instance",
                expect="and sequential run ", report=() -> check(model(:Shared)),
                passing=gnc_except(kind, :concurrent_equals_sequential)),
            (kind=kind, id=:bind_fresh_state, broken="bind_spacecraft copies share a buffer the tick changes",
                expect="changed during the run", report=() -> check(model(:SharedBind)),
                passing=gnc_except(kind, :bind_fresh_state, resets...)))
        kind === :navigation_model && push!(controls,
            (kind=kind, id=:initialize_resets, broken="its own state never reset in initialize_navigation!",
                expect="two runs with isolate_state=false on one configuration differ: ",
                report=() -> check(model(:NoReset)), passing=gnc_except(kind, :initialize_resets)))
    end
    ctl(variant) = NcCtl{variant}()
    kind = :control_model
    push!(controls,
        (kind=kind, id=:hooks_present, broken="a tick but no control_wrench", expect="has no method",
            report=() -> check_control_model(ctl(:NoWrench)), passing=(:no_deprecated_hooks, :supported_sample_signature)),
        (kind=kind, id=:no_deprecated_hooks, broken="a calcControlForceTorque method beside control_wrench",
            expect="implements both control_wrench and calcControlForceTorque",
            report=() -> check_control_model(ctl(:Deprecated)), passing=(:hooks_present, :supported_sample_signature)),
        (kind=kind, id=:supported_sample_signature, broken="a control_wrench method on StateSample{Nothing}",
            expect="StateSample{Nothing}, narrower than StateSample",
            report=() -> check_control_model(ctl(:Narrow)), passing=gnc_except(kind, :supported_sample_signature)),
        (kind=kind, id=:repeatable_run, broken="state outside the model advanced at every run start",
            expect="two runs of the fixture with isolate_state=true differ: ",
            report=() -> check_control_model(ctl(:RunEpoch)), passing=(:hooks_present, :no_deprecated_hooks)),
        (kind=kind, id=:deepcopy_equivalent, broken="a gain keyed by the model's identity",
            expect="and a run of its deep copy (isolate_state=true) differ: ",
            report=() -> check_control_model(registered(ctl(:Identity))), passing=gnc_except(kind, :deepcopy_equivalent)),
        (kind=kind, id=:global_rng_independent, broken="rand() in the tick", expect="seeded 1 and 2 differ: ",
            report=() -> check_control_model(ctl(:Rand)), passing=(:hooks_present, :no_deprecated_hooks)),
        (kind=kind, id=:concurrent_equals_sequential, broken="scratch shared by every instance", expect="and sequential run ",
            report=() -> check_control_model(ctl(:Shared)), passing=gnc_except(kind, :concurrent_equals_sequential)),
        (kind=kind, id=:output_hook_pure, broken="control_wrench counts its calls in the model",
            expect="calls of control_wrench at t = ", report=() -> check_control_model(ctl(:Impure)),
            passing=gnc_except(kind, :output_hook_pure)),
        (kind=kind, id=:positive_consumption, broken="a negative propellant rate",
            expect="control_propellant_rate returned -1.0e-5",
            report=() -> check_control_model(ctl(:Negative)), passing=gnc_except(kind, :positive_consumption)),
        (kind=kind, id=:bind_fresh_state, broken="bind_spacecraft copies share a buffer the tick changes",
            expect="changed during the run", report=() -> check_control_model(ctl(:SharedBind)),
            passing=gnc_except(kind, :bind_fresh_state)),
        (kind=:force_torque_model, id=:no_deprecated_hooks, broken="a calcForceTorque method beside wrench",
            expect="implements both wrench and calcForceTorque",
            report=() -> check_force_torque_model(NcDeprecatedForce()), passing=(:hooks_present, :supported_sample_signature)))
    return controls
end

# A facade with the shape the adapter standard requires; each control breaks one rule.
function control_facade(; version=v"1.0.0", require=:exact, implemented=(:serve,), consumed=(:Value,),
        omit::Union{Nothing, Symbol}=nothing)
    facade = Module(:NcFacade)
    check = require === :exact ? :(v == CONTRACT_VERSION ? nothing : throw(CompatibilityError(string(v)))) :
        require === :major ? :(v.major == CONTRACT_VERSION.major ? nothing : throw(CompatibilityError(string(v)))) :
        :(v == CONTRACT_VERSION ? nothing : throw(ArgumentError(string(v))))
    Core.eval(facade, quote
        struct CompatibilityError <: Exception
            msg::String
        end
        const CONTRACT_VERSION = $version
        require_version(v::VersionNumber) = $check
        serve() = 1
        const Value = 2.0
        const Empty = nothing
        const Nested = (; serve)
    end)
    omit === :IMPLEMENTED_FUNCTIONS || Core.eval(facade, :(const IMPLEMENTED_FUNCTIONS = $(QuoteNode(implemented))))
    omit === :CONSUMED_SERVICES || Core.eval(facade, :(const CONSUMED_SERVICES = $(QuoteNode(consumed))))
    return facade
end

# kind, check id, what is broken, a fragment of the failure detail it must give, the
# report, and the checks that must still pass.
const ALL_FORCE = CHECK_IDS.force_torque_model
except(ids...) = filter(id -> !(id in ids), ALL_FORCE)
const RUN_LEVEL = (:repeatable_run, :deepcopy_equivalent, :global_rng_independent, :concurrent_equals_sequential)

controls() = [
    (kind=:force_torque_model, id=:hooks_present, broken="no wrench and no calcForceTorque", expect="has no method wrench(",
        report=() -> check_force_torque_model(NcNoHook()), passing=(:supported_sample_signature,)),
    (kind=:force_torque_model, id=:supported_sample_signature, broken="a method on StateSample{Nothing}", expect="StateSample{Nothing}, narrower than StateSample",
        report=() -> check_force_torque_model(NcConcreteSample()), passing=except(:supported_sample_signature)),
    (kind=:force_torque_model, id=:repeatable_run, broken="state outside the model advanced at each run start", expect="two runs of the fixture with isolate_state=true differ: ",
        report=() -> check_force_torque_model(NcRunEpoch()),
        passing=(:hooks_present, :supported_sample_signature, :output_hook_pure, :environment_declared)),
    (kind=:force_torque_model, id=:deepcopy_equivalent, broken="a gain keyed by the identity of a field", expect="and a run of its deep copy (isolate_state=true) differ: ",
        report=() -> check_force_torque_model(NcIdentityGain()), passing=except(:deepcopy_equivalent)),
    (kind=:force_torque_model, id=:global_rng_independent, broken="rand() from the global generator", expect="seeded 1 and 2 differ: ",
        report=() -> check_force_torque_model(NcGlobalRandom()),
        passing=(:hooks_present, :supported_sample_signature, :environment_declared)),
    (kind=:force_torque_model, id=:concurrent_equals_sequential, broken="scratch shared by every instance", expect="and sequential run ",
        report=() -> check_force_torque_model(NcSharedScratch()),
        passing=(:hooks_present, :supported_sample_signature, :repeatable_run, :deepcopy_equivalent,
            :global_rng_independent, :environment_declared)),
    (kind=:force_torque_model, id=:output_hook_pure, broken="a call counter in the model", expect="the calls changed the model",
        report=() -> check_force_torque_model(NcCountsCalls(0)), passing=except(:output_hook_pure)),
    (kind=:force_torque_model, id=:output_hook_pure, broken="a log message", expect="the first: Info: NcLogs wrench",
        report=() -> check_force_torque_model(NcLogs()), passing=except(:output_hook_pure)),
    (kind=:force_torque_model, id=:output_hook_pure, broken="replaces one of two aliased fields with an equal copy",
        expect="the calls changed the model",
        report=() -> check_force_torque_model(NcBreaksAliasing()), passing=except(:output_hook_pure)),
    (kind=:force_torque_model, id=:environment_declared, broken="reads the atmosphere without declaring it", expect="wrench threw at t = 0.0 s with only the declared environment fields (planet_frame = nothing, atmosphere = nothing",
        report=() -> check_force_torque_model(NcUndeclaredAtmosphere()),
        passing=(:hooks_present, :supported_sample_signature)),
    (kind=:force_torque_model, id=:environment_declared, broken="NaN without the undeclared solar sample", expect="solar = nothing, third_bodies = nothing); it must return a finite force and torque",
        report=() -> check_force_torque_model(NcUndeclaredSolar()),
        passing=(:hooks_present, :supported_sample_signature, :output_hook_pure)),
    (spice_available() ? [(kind=:force_torque_model, id=:environment_declared,
        broken="reads a third body it did not declare", expect="BoundsError",
        report=() -> (entry = on_spice(planet -> NcUndeclaredThirdBody());
            check_force_torque_model(entry.model; fixture=entry.fixture)),
        passing=(:hooks_present, :supported_sample_signature))] : [])...,
    (kind=:sensor, id=:run_state_fields, broken="an immutable record", expect="is not a mutable struct",
        report=() -> check_sensor(NcImmutableSensor()), passing=()),
    (kind=:sensor, id=:run_state_fields, broken="no latest_time_s field", expect="lacks the field(s) latest_time_s",
        report=() -> check_sensor(NcNoTimeSensor()), passing=()),
    (kind=:sensor, id=:requirements_supported, broken="asks for the atmosphere", expect="add_sensor! refuses the sensor",
        report=() -> check_sensor(NcAtmosphereSensor()), passing=(:run_state_fields,)),
    (kind=:sensor, id=:measurement_shape, broken="two components where it declares three",
        expect="not 3 finite component(s)",
        report=() -> check_sensor(NcShortSensor()), passing=sensor_except(:measurement_shape)),
    (kind=:sensor, id=:measurement_shape, broken="a NaN component", expect="not 3 finite component(s)",
        report=() -> check_sensor(NcNaNSensor()), passing=sensor_except(:measurement_shape)),
    (kind=:sensor, id=:repeatable_run, broken="state outside the sensor advanced at every run start",
        expect="two runs of the fixture with isolate_state=true differ: ",
        report=() -> check_sensor(NcRunEpochSensor()), passing=(:run_state_fields, :requirements_supported, :measurement_shape)),
    (kind=:sensor, id=:deepcopy_equivalent, broken="an offset keyed by the sensor's identity",
        expect="and a run of its deep copy (isolate_state=true) differ: ",
        report=() -> check_sensor(NcRegisteredSensor()), passing=sensor_except(:deepcopy_equivalent)),
    (kind=:sensor, id=:global_rng_independent, broken="noise from the global generator", expect="seeded 1 and 2 differ: ",
        report=() -> check_sensor(NcGlobalRandSensor()), passing=(:run_state_fields, :requirements_supported, :measurement_shape)),
    (kind=:sensor, id=:concurrent_equals_sequential, broken="scratch shared by every instance", expect="and sequential run ",
        report=() -> check_sensor(NcSharedSensor()), passing=sensor_except(:concurrent_equals_sequential)),
    (kind=:sensor, id=:seed_repeatable, broken="its stream reset to a fixed seed",
        expect="yet runs with those seeds give the same measurements",
        report=() -> check_sensor(NcFixedSeedSensor()), passing=sensor_except(:seed_repeatable)),
    (kind=:sensor, id=:custom_state_reset, broken="a sample counter it never resets",
        expect="two runs with isolate_state=false on one configuration differ: ",
        report=() -> check_sensor(NcCountingSensor()), passing=sensor_except(:custom_state_reset)),
    gnc_controls()...,
    (kind=:service_facade, id=:facade_shape, broken="CONTRACT_VERSION other than the reviewed one", expect="CONTRACT_VERSION is v\"1.0.1\", not the reviewed 1.0.0",
        report=() -> facade_report(control_facade(version=v"1.0.1"); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="require_version accepts any minor or patch", expect="accepted a version other than 1.0.0",
        report=() -> facade_report(control_facade(require=:major); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="require_version throws another error", expect="threw ArgumentError, not CompatibilityError",
        report=() -> facade_report(control_facade(require=:argument); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="an implemented name listed twice", expect="implemented entries listed more than once: serve",
        report=() -> facade_report(control_facade(implemented=(:serve, :serve)); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="a name both implemented and consumed", expect="listed as both implemented and consumed: serve",
        report=() -> facade_report(control_facade(consumed=(:Value, :serve)); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="an implemented name that is not a function", expect="implemented Value is not a function",
        report=() -> facade_report(control_facade(implemented=(:Value,), consumed=()); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="an implemented path that does not resolve", expect="implemented Nested.absent does not resolve",
        report=() -> facade_report(control_facade(implemented=((:Nested, :absent),)); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="a consumed name bound to nothing", expect="consumed Empty is nothing",
        report=() -> facade_report(control_facade(consumed=(:Empty,)); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="IMPLEMENTED_FUNCTIONS a bare name, not a collection",
        expect="IMPLEMENTED_FUNCTIONS is :serve, not a collection of names or name paths",
        report=() -> facade_report(control_facade(implemented=:serve); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="CONSUMED_SERVICES nothing",
        expect="CONSUMED_SERVICES is nothing, not a collection of names or name paths",
        report=() -> facade_report(control_facade(consumed=nothing); version=v"1.0.0"), passing=()),
    (kind=:service_facade, id=:facade_shape, broken="no CONSUMED_SERVICES", expect="does not define CONSUMED_SERVICES",
        report=() -> facade_report(control_facade(omit=:CONSUMED_SERVICES); version=v"1.0.0"), passing=()),
]

# The facades are defined at run time; the check runs in the world that has them.
facade_report(facade; version) = Base.invokelatest(check_service_facade, facade; version)

status_of(report, id) = only(check.status for check in report.checks if check.id === id)

@testset "Conformance negative controls" begin
    all_controls = controls()
    @testset "every check id has a negative control" begin
        for (kind, ids) in pairs(CHECK_IDS), id in ids
            @testset let kind = kind, id = id
                @test any(control -> control.kind === kind && control.id === id, all_controls)
            end
        end
    end
    @testset "clean inputs pass" begin
        report = check_force_torque_model(NcClean())
        @test [check.id for check in report.checks] == collect(CHECK_IDS.force_torque_model)
        @test all(check -> check.status === :pass, report.checks) || Threads.nthreads() < 2
        @test passed(report)
        facade = facade_report(control_facade(); version=v"1.0.0")
        @test [check.id for check in facade.checks] == collect(CHECK_IDS.service_facade)
        @test passed(facade)
        nested = facade_report(control_facade(implemented=(:serve, (:Nested, :serve))); version=v"1.0.0")
        @test passed(nested)
        for (report, kind) in ((check_guidance_model(NcGClean()), :guidance_model),
                (check_navigation_model(NcNClean()), :navigation_model), (check_control_model(NcCtl{:Clean}()), :control_model))
            @test [check.id for check in report.checks] == collect(CHECK_IDS[kind])
            @test all(check -> check.status in (:pass, :skip), report.checks)
            @test passed(report)
        end
        sensor = check_sensor(NcCleanSensor())
        @test [check.id for check in sensor.checks] == collect(CHECK_IDS.sensor)
        @test all(check -> check.status === :pass, sensor.checks) || Threads.nthreads() < 2
        @test passed(sensor)
    end
    for control in all_controls
        # A check reports a broken input as a failing check; it never throws.
        report = try
            with_logger(NullLogger()) do
                control.report()
            end
        catch err
            err
        end
        expected = control.id === :concurrent_equals_sequential && Threads.nthreads() < 2 ? :skip : :fail
        @testset "$(control.id): $(control.broken)" begin
            @testset let id = control.id, thrown = report isa Exception ? sprint(showerror, report) : ""
                @test !(report isa Exception)
            end
            if !(report isa Exception)
                detail = only(check.detail for check in report.checks if check.id === control.id)
                @testset let id = control.id, status = status_of(report, control.id), detail = detail
                    @test status === expected
                    expected === :fail && @test occursin(control.expect, detail)
                    # A run-level control fails by its behaviour, not by a run that stopped.
                    control.id in (control.kind === :sensor ? SENSOR_RUN_LEVEL :
                        control.kind in (:guidance_model, :navigation_model, :control_model) ? GNC_RUN_LEVEL : RUN_LEVEL) &&
                        @test !occursin("run failed", detail)
                end
                expected === :fail && @test !passed(report)
                for id in control.passing
                    @testset let id = id, status = status_of(report, id)
                        # Skipped is not failed: concurrency needs two threads, and a model
                        # without bind_spacecraft has no bound copies to check.
                        @test status === :pass || (id in (:concurrent_equals_sequential, :bind_fresh_state) && status === :skip)
                    end
                end
            end
        end
    end
    @testset "concurrent_equals_sequential: a run stalls past the rendezvous limit" begin
        model = NcStallsOnce()
        fixture = conformance_fixture(effectors=(model,))
        base = C._solve(fixture.configuration)
        again = C._solve(fixture.configuration)
        NC_STALL[] = true
        check = C._concurrency_check(fixture.configuration, base, again, v"0.3.0"; timeout_s=1.0)
        @test !NC_STALL[]   # one concurrent run took the token and stalled
        if Threads.nthreads() >= 2
            @testset let status = check.status, detail = check.detail
                @test status === :fail
                @test occursin("the concurrent runs fell out of step", detail)
                @test !occursin("differ", detail)   # equal results do not rescue it
            end
        else
            @test check.status === :skip
        end
    end
    @testset "the state comparison matches mutable objects one to one" begin
        shared = [1.0, 2.0]
        aliased = NcBreaksAliasing(shared, shared)
        separate = NcBreaksAliasing([1.0, 2.0], [1.0, 2.0])
        @test C._same_state(aliased, deepcopy(aliased))
        @test C._same_state(separate, deepcopy(separate))
        @test !C._same_state(aliased, separate)
        @test !C._same_state(separate, aliased)
    end
    # The checks run copies; only the deepcopy_equivalent run mutates the model given.
    counter = NcCountsCalls(0)
    with_logger(NullLogger()) do
        check_force_torque_model(counter)
    end
    @test counter.calls > 0
end

end # module ConformanceNegativeControlsTests
