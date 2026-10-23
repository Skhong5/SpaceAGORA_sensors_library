module InterlinkStorageTests

# Onboard data storage: the store and the sources, the setup checks, the flow
# control at a store (an empty store passes what comes in on, a full one takes
# in no more than it sends), relays, production lost into a full store, a source
# of your own, the columns, the checkpoint state, and a negative control.

using Test
using LinearAlgebra
using StaticArrays
using SpaceAGORA

const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const PLANET = make_no_gram_planet(:earth)
const RADIUS = PLANET.Rp_e + 700e3

# A source of your own: a rate that rises and falls with time.
struct WaveSource <: AbstractDataSource
    rate_bps::Float64
    period_s::Float64
end
SpaceAGORA.data_production_bps(source::WaveSource, x, ctx, t) = source.rate_bps * (1 + sin(2π * t / source.period_s)) / 2
# A source kind without a production rate, and one with a negative rate.
struct SilentSource <: AbstractDataSource end
struct DrainSource <: AbstractDataSource end
SpaceAGORA.data_production_bps(::DrainSource, x, ctx, t) = -1.0

# A 100 kg bus `arc_m` along a circular orbit, with `n_terminal` virtual
# terminals, and a data store and sources when given.
function data_sat(index, arc_m; n_terminal=1, store=nothing, sources=())
    bus = Link(root=true, m=100.0)
    θ = arc_m / RADIUS
    speed = sqrt(PLANET.μ / RADIUS)
    ic = CartesianInitialCondition(SVector(RADIUS * cos(θ), RADIUS * sin(θ), 0.0),
        SVector(-speed * sin(θ), speed * cos(θ), 0.0); q=SVector(0.0, 0.0, 0.0, 1.0))
    sc = SpacecraftModel(root=bus, links=[bus], inertia_tensor=bus.inertia, initial_condition=ic, id=index,
        n_terminal=n_terminal)
    store === nothing || add_data!(sc, bus, store)
    foreach(source -> add_data!(sc, bus, source), sources)
    return sc
end

fixed(rate) = DataLink(transmit_power_w=0.0, budget=FixedRateBudget(rate_bps=rate), range_m=1e7)

# Links `(from, to, rate)` between terminals: each spacecraft's first terminal,
# or `(spacecraft, terminal)` pairs.
function data_model(spacecraft, links)
    model = InterlinkModel()
    endpoint(e) = e isa Integer ? (e, 1) : e
    for (from, to, rate) in links
        register_link!(model, spacecraft, endpoint(from), endpoint(to), fixed(rate))
    end
    return model
end

function run_args(spacecraft, model; duration=60.0, solver_mode=:tsit5,
                  settings=SimulationSettings(results=false, generate_plots=false))
    return SimulationConfiguration(
        simulation_settings=settings,
        mission_configuration=MissionConfiguration(mission_type=MissionTime, mission_time=duration,
            orientation_sim=false, data_rate=1.0),
        environment_model=make_no_gram_environment(planet=PLANET),
        dynamics_model=DynamicsModel(spacecraft, (InverseSquaredGravityModel(),)),
        guidance_model=GuidanceModel((), Float64[]), navigation_model=NavigationModel((), Float64[]),
        control_model=ControlModel((), Float64[]), initial_time=InitialTime(),
        integration_tolerances=IntegrationTolerances(reltol_orbit=1e-10, abstol_orbit=1e-8, dt_max_orbit=1.0),
        solver_config=SM.SolverConfig(solver_mode=solver_mode), interlink_model=model,
        scheduling_policy_model=DataThroughputPolicy())
end

run(spacecraft, model; kwargs...) = run_simulation(run_args(spacecraft, model; kwargs...);
    return_results=true, return_solution=true)
refusal(f) = try f(); nothing catch err err end
run_store(solution, i) = SM.data_store(solution.prob.p.args.dynamics_model.spacecraft[i])

# The store's balance at every saved row: stored = initial + produced + received − sent − lost.
function balanced(table, i, initial; atol)
    column(name) = getproperty(table, Symbol("sc$(i)_", name))
    received = hasproperty(table, Symbol("sc$(i)_data_received_bits")) ? column(:data_received_bits) : zero(table.time)
    sent = hasproperty(table, Symbol("sc$(i)_data_sent_bits")) ? column(:data_sent_bits) : zero(table.time)
    residual = column(:data_stored_bits) .- (initial .+ column(:data_produced_bits) .+ received .- sent .- column(:data_lost_bits))
    return maximum(abs, residual) <= atol
end

# One sender holding data and producing, one receiver.
function one_link(; c=2e6, production=1e5, initial=1e7)
    spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e8, initial_bits=initial),
                           sources=(DataSource(rate_bps=production),)),
                  data_sat(2, 100e3; store=DataStore(capacity_bits=5e7))]
    return spacecraft, data_model(spacecraft, [(1, 2, c)])
end

@testset "Data storage" begin
    @testset "parts" begin
        @test_throws ArgumentError DataStore(capacity_bits=0.0)
        @test_throws ArgumentError DataStore(capacity_bits=Inf)
        @test_throws ArgumentError DataStore(capacity_bits=1e6, initial_bits=2e6)
        @test_throws ArgumentError DataStore(capacity_bits=1e6, initial_bits=-1.0)
        @test_throws ArgumentError DataSource(rate_bps=-1.0)
        @test_throws ArgumentError DataSource(rate_bps=NaN)
        sc = data_sat(1, 0.0)
        @test !SM.has_data_store(sc) && SM.data_store(sc) === nothing
        store = add_data!(sc, sc.root, DataStore(capacity_bits=1e6, initial_bits=1e5))
        @test SM.has_data_store(sc) && SM.data_store(sc) === store && !store.full && !store.empty
        @test_throws ArgumentError add_data!(sc, sc.root, DataStore(capacity_bits=1e6))   # at most one
        @test_throws ArgumentError add_data!(sc, Link(root=true, m=1.0), DataSource(rate_bps=1.0))   # not its link
        source = add_data!(sc, sc.root, DataSource(rate_bps=5.0, name="camera"))
        @test only(sc.root.data_sources) === source && source.name == "camera"
        x = SM.InterLinkModels.link_endpoint((pos=[RADIUS, 0.0, 0.0],), nothing, 1)
        @test data_production_bps(source, x, LinkContext(), 0.0) == 5.0
        @test data_production_bps(WaveSource(4.0, 8.0), x, LinkContext(), 2.0) ≈ 4.0
        # The spacecraft with data parts are claimed by the constellation builder's sharing check.
        @test SM.DataStorage._has_data_sources(sc)
    end

    @testset "setup refusals" begin
        no_store = [data_sat(1, 0.0; sources=(DataSource(rate_bps=1.0),)), data_sat(2, 100e3)]
        err = refusal(() -> run_simulation(run_args(no_store, data_model(no_store, [(1, 2, 1e6)]))))
        @test err isa ArgumentError && occursin("Spacecraft 1 has a data source but no data store", err.msg)
        silent = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e6), sources=(SilentSource(),))]
        err = refusal(() -> run_simulation(run_args(silent, nothing)))
        @test err isa ArgumentError && occursin("SilentSource has no data_production_bps method", err.msg)
        split = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e6), sources=(DataSource(rate_bps=1.0),))]
        err = refusal(() -> run_simulation(run_args(split, nothing; solver_mode=:split_imex)))
        @test err isa ArgumentError && occursin("data store needs a solver mode that integrates the full right-hand side", err.msg)
        drain = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e6), sources=(DrainSource(),))]
        err = refusal(() -> run_simulation(run_args(drain, nothing)))
        @test err isa ArgumentError && occursin("finite and nonnegative", err.msg)
    end

    @testset "one link" begin
        c, production, initial = 2e6, 1e5, 1e7
        result = run(one_link(; c, production, initial)...)
        table = result.table
        empty_at = initial / (c - production)
        # A sender holding data sends at the link's rate; once empty, it passes its production on.
        for (t, rate) in zip(table.time, table.sc1_data_rate_tx_bps)
            t < empty_at - 1.5 && @test rate == c
            t > empty_at + 1.5 && @test rate ≈ production rtol=1e-12
        end
        # The stored data follows the closed form through the event (the solution;
        # a saved row inside the step an event cuts short can hold another value).
        @test all(t -> isapprox(result.solution(t).sc[1].data_stored_bits, max(initial - (c - production) * t, 0.0);
            atol=1e-6 * initial), 0.0:0.25:60.0)
        @test run_store(result.solution, 1).empty && !run_store(result.solution, 2).empty
        # The receiver holds what was sent, and nothing is lost.
        @test table.sc2_data_received_bits[end] == table.sc1_data_sent_bits[end]
        @test table.sc2_data_stored_bits[end] ≈ initial + production * 60.0 rtol=1e-9
        @test table.sc1_data_lost_bits[end] == 0.0 && table.sc2_data_lost_bits[end] == 0.0
        @test balanced(table, 1, initial; atol=1e-6) && balanced(table, 2, 0.0; atol=1e-6)
        @test table.sc2_data_store_fill ≈ table.sc2_data_stored_bits ./ 5e7
    end

    @testset "a full receiver" begin
        c, production = 2e6, 1e5
        spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e9, initial_bits=1e8)),
                      data_sat(2, 100e3; store=DataStore(capacity_bits=1e7), sources=(DataSource(rate_bps=production),))]
        result = run(spacecraft, data_model(spacecraft, [(1, 2, c)]))
        table = result.table
        full_at = 1e7 / (c + production)
        # The receiver fills, then takes in nothing more: the link stops, the sender
        # keeps its data, and the receiver's own production is lost.
        @test run_store(result.solution, 2).full
        @test table.sc1_data_rate_tx_bps[end] == 0.0
        @test table.sc1_data_sent_bits[end] ≈ c * full_at rtol=1e-6
        @test table.sc1_data_stored_bits[end] ≈ 1e8 - table.sc1_data_sent_bits[end] rtol=1e-12
        @test table.sc1_data_lost_bits[end] == 0.0
        @test table.sc2_data_stored_bits[end] ≈ 1e7 rtol=1e-9
        @test table.sc2_data_store_fill[end] ≈ 1.0 atol=1e-9
        @test table.sc2_data_lost_bits[end] ≈ production * (60.0 - full_at) rtol=1e-6
        @test balanced(table, 1, 1e8; atol=1e-3) && balanced(table, 2, 0.0; atol=1e-3)
    end

    @testset "a full store that drains takes data again" begin
        # Full at the start, sending on: its own production is lost only until the
        # step-end update finds room again.
        rate = 1e4
        spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e7, initial_bits=1e7), sources=(DataSource(rate_bps=rate),)),
                      data_sat(2, 100e3; store=DataStore(capacity_bits=1e9))]
        result = run(spacecraft, data_model(spacecraft, [(1, 2, 1e5)]))
        table = result.table
        @test !run_store(result.solution, 1).full
        @test table.sc1_data_lost_bits[end] <= rate * 1.0
        @test table.sc1_data_stored_bits[end] ≈ 1e7 - (1e5 - rate) * 60.0 rtol=1e-3
        @test balanced(table, 1, 1e7; atol=1e-3)
    end

    @testset "a chain of relays" begin
        # A holds data; B, empty, relays it on to C on a faster link: B stays empty
        # and C receives what A sends.
        c1, c2 = 1e6, 3e6
        spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e9, initial_bits=1e8)),
                      data_sat(2, 100e3; n_terminal=2, store=DataStore(capacity_bits=1e8)),
                      data_sat(3, 200e3; store=DataStore(capacity_bits=1e9))]
        result = run(spacecraft, data_model(spacecraft, [(1, (2, 1), c1), ((2, 2), 3, c2)]))
        table = result.table
        @test maximum(table.sc2_data_stored_bits) <= 1e-6 * c1
        @test table.sc2_data_rate_tx_bps[end] ≈ c1 rtol=1e-12
        @test table.sc3_data_received_bits[end] ≈ c1 * 60.0 rtol=1e-9
        @test table.sc3_data_received_bits[end] == table.sc2_data_sent_bits[end]
        @test all(i -> balanced(table, i, (1e8, 0.0, 0.0)[i]; atol=1e-3), 1:3)
    end

    @testset "production into a full store is lost" begin
        rate = 4e4
        spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e6, initial_bits=2e5), sources=(DataSource(rate_bps=rate),))]
        table = run(spacecraft, nothing).table
        full_at = 8e5 / rate
        @test table.sc1_data_stored_bits[end] ≈ 1e6 rtol=1e-9
        @test table.sc1_data_produced_bits[end] ≈ rate * 60.0 rtol=1e-9
        @test table.sc1_data_lost_bits[end] ≈ rate * (60.0 - full_at) rtol=1e-6
        @test table.sc1_data_lost_bits ≈ table.sc1_data_produced_bits .- (table.sc1_data_stored_bits .- 2e5) atol=1e-3
        @test table.sc1_data_store_fill[end] ≈ 1.0 atol=1e-9
        # No data link: no counters.
        @test !hasproperty(table, :sc1_data_sent_bits)
    end

    @testset "a source of your own" begin
        source = WaveSource(1e5, 40.0)
        spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e9), sources=(source, DataSource(rate_bps=1e3)))]
        table = run(spacecraft, nothing).table
        T = 60.0
        wave = source.rate_bps * T / 2 + source.rate_bps * source.period_s / (4π) * (1 - cos(2π * T / source.period_s))
        @test table.sc1_data_produced_bits[end] ≈ wave + 1e3 * T rtol=1e-6
        @test table.sc1_data_stored_bits[end] ≈ table.sc1_data_produced_bits[end] rtol=1e-12
    end

    @testset "columns" begin
        spacecraft = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e8, initial_bits=1e7)), data_sat(2, 100e3)]
        table = run(spacecraft, data_model(spacecraft, [(1, 2, 2e5)]); duration=10.0).table
        for name in (:data_stored_bits, :data_store_fill, :data_produced_bits, :data_lost_bits)
            @test hasproperty(table, Symbol("sc1_", name)) && all(isnan, getproperty(table, Symbol("sc2_", name)))
        end
        # The state has one shape: the spacecraft without a store carries the store states, at zero.
        @test run(spacecraft, data_model(spacecraft, [(1, 2, 2e5)]); duration=10.0).solution.u[end].sc[2].data_stored_bits == 0.0
        # A receiver without a store takes everything the link carries.
        @test table.sc2_data_rate_rx_bps[end] == 2e5
        @test table.sc2_data_received_bits[end] ≈ 2e6 rtol=1e-9
    end

    @testset "checkpoint state" begin
        spacecraft, model = one_link()
        args = run_args(spacecraft, model)
        state = SE._callback_checkpoint_state(args)
        @test state.configuration[1].data_store == (1e8, 1e7) && state.configuration[2].data_store == (5e7, 0.0)
        # The flags are written and restored.
        SM.data_store(spacecraft[1]).empty = true
        SM.data_store(spacecraft[2]).full = true
        saved = (runtime_state=(callbacks=SE._callback_checkpoint_state(args),),)
        @test saved.runtime_state.callbacks.spacecraft[1].data_store == (full=false, empty=true)
        fresh, fresh_model = one_link()
        parameters = (shared_buffers=(callback_resume=Ref(false),),)
        SE._restore_callback_checkpoint_state!(run_args(fresh, fresh_model), saved, parameters)
        @test SM.data_store(fresh[1]).empty && !SM.data_store(fresh[1]).full
        @test SM.data_store(fresh[2]).full && !SM.data_store(fresh[2]).empty
        @test parameters.shared_buffers.callback_resume[]
        # Another capacity or initial content refuses the checkpoint.
        for store in (DataStore(capacity_bits=2e8, initial_bits=1e7), DataStore(capacity_bits=1e8, initial_bits=2e7))
            other = [data_sat(1, 0.0; store, sources=(DataSource(rate_bps=1e5),)), data_sat(2, 100e3; store=DataStore(capacity_bits=5e7))]
            err = refusal(() -> SE._restore_callback_checkpoint_state!(run_args(other, data_model(other, [(1, 2, 2e6)])), saved,
                (shared_buffers=(callback_resume=Ref(false),),)))
            @test err isa ArgumentError && occursin("data stores differ from the checkpoint's", err.msg)
        end
        # A run without a store writes the snapshot it wrote before data stores.
        plain = [data_sat(1, 0.0), data_sat(2, 100e3)]
        add_power!(plain[1], plain[1].root, Battery(capacity_wh=10.0, initial_soc=0.5))
        plain_state = SE._callback_checkpoint_state(run_args(plain, data_model(plain, [(1, 2, 2e6)])))
        @test keys(plain_state.configuration[1]) == (:battery, :faults, :sensors, :hinges, :gimbals, :thrusters)
        @test keys(plain_state.spacecraft[1]) == (:battery, :faults, :sensors, :hinges, :gimbals, :thrusters)
    end

    @testset "a resumed run" begin
        straight = run(one_link()...).table
        mktempdir() do directory
            segments(resume) = SimulationSettings(results=resume, results_directory=directory, generate_plots=false,
                checkpoint_enabled=true, checkpoint_interval_s=10.0, checkpoint_directory=directory,
                resume_from_checkpoint=resume)
            run(one_link()...; duration=30.0, settings=segments(false))
            resumed = run(one_link()...; settings=segments(true))
            # The sender emptied in the first leg; the resumed run keeps it empty and passes its production on.
            @test run_store(resumed.solution, 1).empty
            @test resumed.table.sc1_data_rate_tx_bps[end] ≈ 1e5 rtol=1e-12
            for name in (:sc1_data_stored_bits, :sc1_data_produced_bits, :sc2_data_stored_bits, :sc2_data_received_bits)
                @test getproperty(resumed.table, name)[end] ≈ getproperty(straight, name)[end] rtol=1e-9 atol=1e-6
            end
        end
    end

    @testset "without a store, as before" begin
        # Negative control: without stores a link carries its rate whatever either
        # end holds; an empty store with nothing coming in sends nothing.
        plain = [data_sat(1, 0.0), data_sat(2, 100e3)]
        table = run(plain, data_model(plain, [(1, 2, 2e6)]); duration=10.0).table
        @test table.sc1_data_sent_bits[end] ≈ 2e7 rtol=1e-9
        @test !hasproperty(table, :sc1_data_stored_bits)
        empty_sender = [data_sat(1, 0.0; store=DataStore(capacity_bits=1e8)), data_sat(2, 100e3)]
        table = run(empty_sender, data_model(empty_sender, [(1, 2, 2e6)]); duration=10.0).table
        @test table.sc1_data_sent_bits[end] == 0.0 && table.sc1_data_rate_tx_bps[end] == 0.0
    end
end

end # module InterlinkStorageTests
