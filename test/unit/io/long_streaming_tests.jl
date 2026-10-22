module LongStreamingTests
# A long-layout run with checkpoints streams its results: each checkpoint
# appends its rows to the results streams and the CSV, and the run keeps only
# its last row in memory. The streamed files hold the rows an unstreamed run
# writes, also when thinned, after a crash and its resume, and across layouts.
using Test
using SpaceAGORA
using SpaceAGORA.SimulationModel
using DataFrames
using Arrow
using CSV
using TOML
using Logging
using OrdinaryDiffEq: DiscreteCallback
import SpaceAGORA.TelemetryVerification: make_example_config, make_three_body_spacecraft

const SM = SpaceAGORA.SimulationModel
const SE = SpaceAGORA.SimulationEngine
const PLANET = make_no_gram_planet(:earth)

quiet(f) = with_logger(f, ConsoleLogger(stderr, Logging.Error))

# Three spacecraft, 500 s, a checkpoint every 100 s, written to `dir` in `layout`.
function config(dir; layout=:long, checkpoints=true, resume=false, mission_time=500.0)
    spacecraft(index) = make_three_body_spacecraft(bus_dims=(1.0, 1.0, 1.0), panel_dims=(0.01, 1.0, 0.5),
        bus_mass=200.0, panel_mass_each=5.0, panel_offset_y=1.0,
        ic=InitialCondition(ra=7.0e6, rp=7.0e6, i=53.0, ω=0.0, Ω=0.0, ν=30.0 * index), prop_mass=0.0, id=index)
    base = make_example_config(planet=PLANET, spacecraft=spacecraft(1), mission_time=mission_time,
        initial_time=InitialTime(year=2020, month=1, day=1, hour=0, minute=0, second=0.0),
        dynamic_effectors=(SM.InverseSquaredJ2GravityModel(),), density_model=SM.NoAtmosphereModel(),
        ephemerides_model=SM.SimpleEphemeridesModel(), orientation_sim=false, keplerian=true, EI_km=120.0,
        verbose=false, results=true, results_directory=dir, solver_config=SM.SolverConfig(solver_mode=:tsit5))
    settings = SM.SimConfig._with_fields(base.simulation_settings; results_layout=layout,
        checkpoint_enabled=checkpoints, checkpoint_interval_s=100.0, resume_from_checkpoint=resume)
    return SM.SimConfig._with_configuration(base; simulation_settings=settings,
        dynamics_model=SM.DynamicsModel([spacecraft(k) for k in 1:3], base.dynamics_model.dynamic_effectors))
end

# The default fields plus a run-wide field, so that both tables are streamed.
fields(args) = vcat(SM.default_save_fields(args), [SM.SaveField(:clock, (u, t, integrator) -> t^2)])

run(args; kwargs...) = quiet(() -> run_simulation(args; return_results=true, save_fields=fields(args), kwargs...))

csvs(dir) = (read(joinpath(dir, "simulation_results_spacecraft.csv")), read(joinpath(dir, "simulation_results_run.csv")))
feather(dir, part) = DataFrame(Arrow.Table(joinpath(dir, "simulation_results_$(part).feather")))
entry(args) = SE._load_checkpoint(args).runtime_state.results_rows

# The same run unstreamed: written without the bundle, which a streamed run needs.
unstreamed(args) = withenv(() -> run(args), "SPACEAGORA_SAVE_BUNDLE" => "0")

# Rows of the spacecraft stream beside the checkpoint while the run goes on.
struct Watch
    at::Vector{Float64}
    rows::Vector{Int}
end
function watcher(watch::Watch, path)
    return DiscreteCallback((u, t, integrator) -> any(s -> t > s && !(s in watch.at), (150.0, 250.0, 350.0)),
        integrator -> begin
            push!(watch.at, maximum(s for s in (150.0, 250.0, 350.0) if integrator.t > s))
            push!(watch.rows, isfile(path) ? length(Arrow.Table(path).time) : 0)
        end; save_positions=(false, false))
end

@testset "streamed long results" begin
    @testset "the streamed files hold the unstreamed run's rows" begin
        dir = mktempdir()
        args = config(dir)
        ck = SM.IOConfig._checkpoint_paths(args)
        watch = Watch(Float64[], Int[])
        streamed = run(args; extra_callbacks=(watcher(watch, ck.stream_spacecraft),))
        plain_dir = mktempdir()
        plain = unstreamed(config(plain_dir))
        # The same tables, in memory and on disk, and the same CSV bytes.
        @test isequal(streamed.table, plain.table) && isequal(run_table(streamed), run_table(plain))
        @test isequal(feather(dir, "spacecraft"), plain.table) && isequal(feather(dir, "run"), run_table(plain))
        @test csvs(dir) == csvs(plain_dir)
        # The values are those of the wide layout, rearranged.
        wide = run(config(mktempdir(); layout=:wide))
        @test isequal(streamed.table, long_table(wide))
        @test isequal(run_table(streamed).clock, wide.table.clock)
        # The rows were written as the run went: before the checkpoint at 200 s,
        # those up to 90 s; before the one at 300 s, those up to 190 s.
        @test watch.at == [150.0, 250.0, 350.0] && watch.rows == 3 .* [10, 20, 30]
        # The checkpoint keeps only the last row beside it; the streams hold the rest.
        last_entry = entry(args)
        @test last_entry.version == 2 && last_entry.rows == 1
        @test last_entry.stream.written == nrow(run_table(plain)) - 1 && last_entry.stream.released == nrow(run_table(plain)) - 1
        kept = SM.IOSerialization._read_checkpoint_rows(ck.rows, last_entry.bytes)
        @test kept.times == [500.0]
        # The returned tables read the feathers in place.
        @test !(streamed.table.pos_1 isa Vector) && streamed.table.pos_1 == plain.table.pos_1
        @test ck.stream_spacecraft in streamed.files && ck.stream_run in streamed.files
        # The manifest is the long layout's, written when the run ended.
        manifest = TOML.parsefile(joinpath(dir, "simulation_results.manifest.toml"))
        @test manifest["layout"] == "long" && manifest["steps"] == nrow(run_table(plain)) && !haskey(manifest, "streaming")
        @test manifest["files"]["spacecraft_feather"]["sha256"] ==
            SM.IOSerialization._sha256_hex(joinpath(dir, "simulation_results_spacecraft.feather"))
    end

    @testset "thinned rows" begin
        for (name, value) in (("SPACEAGORA_RESULTS_THIN_STRIDE", "3"), ("SPACEAGORA_RESULTS_FINAL_ONLY", "1"))
            withenv(name => value) do
                dir, plain_dir = mktempdir(), mktempdir()
                streamed = run(config(dir))
                plain = unstreamed(config(plain_dir))
                @test isequal(streamed.table, plain.table) && isequal(run_table(streamed), run_table(plain))
                @test csvs(dir) == csvs(plain_dir)
                @test last(run_table(streamed).time) == 500.0
                name == "SPACEAGORA_RESULTS_FINAL_ONLY" && @test run_table(streamed).time == [500.0]
                name == "SPACEAGORA_RESULTS_THIN_STRIDE" && @test run_table(streamed).time == [0.0:30.0:480.0; 500.0]
            end
        end
    end

    @testset "a crash after the third checkpoint, and the resume" begin
        straight_dir = mktempdir()
        straight = run(config(straight_dir))
        dir = mktempdir()
        crash = DiscreteCallback((u, t, integrator) -> t > 350.0, integrator -> error("crashed"); save_positions=(false, false))
        @test_throws Exception run(config(dir); extra_callbacks=(crash,))
        # The failed run wrote its rows so far, past the checkpoint at 300 s.
        @test last(feather(dir, "run").time) == 300.0
        @test entry(config(dir)).stream.written == 30                     # the checkpoint's rows end before 300 s
        resumed = run(config(dir; resume=true))
        @test isequal(resumed.table, straight.table) && isequal(run_table(resumed), run_table(straight))
        @test isequal(feather(dir, "spacecraft"), straight.table)
        @test csvs(dir) == csvs(straight_dir)
        manifest = TOML.parsefile(joinpath(dir, "simulation_results.manifest.toml"))
        @test manifest["resumed_from_s"] == 300.0 && manifest["first_time_s"] == 0.0
        # A resume of the finished run solves nothing, returns the same tables and
        # leaves the files as they were.
        again = run(config(dir; resume=true))
        @test isequal(again.table, straight.table) && isequal(run_table(again), run_table(straight))
        @test csvs(dir) == csvs(straight_dir) && isequal(feather(dir, "spacecraft"), straight.table)
    end

    @testset "resumes across layouts" begin
        # A wide checkpoint, resumed in the long layout: the rows it kept are
        # streamed with the rest.
        straight = run(config(mktempdir()))
        dir = mktempdir()
        first_leg = run(config(dir; layout=:wide, mission_time=300.0))
        resumed = run(config(dir; resume=true))
        @test isequal(resumed.table, straight.table)
        # A streamed checkpoint, resumed in the wide layout: its rows are in the
        # streams, so the resume warns and writes beside them.
        dir = mktempdir()
        run(config(dir; mission_time=300.0))
        @test_logs (:warn, r"does not stream its results") match_mode=:any run_simulation(
            config(dir; layout=:wide, resume=true); save_fields=fields(config(dir)))
        @test isdir(joinpath(dir, "resumed_at_300.0s"))
        # A stream another run wrote is not taken.
        dir = mktempdir()
        run(config(dir; mission_time=300.0))
        other = mktempdir()
        run(config(other; mission_time=300.0))
        ck, other_ck = SM.IOConfig._checkpoint_paths(config(dir)), SM.IOConfig._checkpoint_paths(config(other))
        cp(other_ck.stream_spacecraft, ck.stream_spacecraft; force=true)
        @test_logs (:warn, r"written by another run") match_mode=:any run_simulation(
            config(dir; resume=true); save_fields=fields(config(dir)))
    end

    @testset "what does not stream" begin
        # Without checkpoints, or in the wide layout, nothing streams.
        dir = mktempdir()
        run(config(dir; checkpoints=false))
        @test !isfile(SM.IOConfig._checkpoint_paths(config(dir)).stream_spacecraft)
        dir = mktempdir()
        run(config(dir; layout=:wide))
        @test !isfile(SM.IOConfig._checkpoint_paths(config(dir)).stream_spacecraft)
        @test entry(config(dir; layout=:wide)).version == 1
    end
end

end # module LongStreamingTests
