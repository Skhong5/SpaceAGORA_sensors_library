module InterlinkScheduleTests

using Test
using Arrow
using CSV
using DataFrames
using DiffEqBase
using SHA
using TOML
using SpaceAGORA
include(joinpath(@__DIR__, "..", "..", "..", "examples", "gve_sma_interlinks.jl"))

const SE = SpaceAGORA.SimulationEngine
const SP = SM.SchedulingPolicies

function with_settings(args; overrides...)
    settings = args.simulation_settings
    names = fieldnames(typeof(settings))
    fields = NamedTuple{names}(map(name -> getfield(settings, name), names))
    return SM.SimConfig._with_configuration(args;
        simulation_settings=SimulationSettings(; merge(fields, (; overrides...))...))
end
with_policy(args, policy; model=args.interlink_model) =
    SM.SimConfig._with_configuration(args; scheduling_policy_model=SchedulingPolicyModel(policy), interlink_model=model)

# The gve example's two candidate laser links, on a model of one's own.
function laser_model(spacecraft; kwargs...)
    model = InterlinkModel(; kwargs...)
    for partner in 2:3
        register_link!(model, spacecraft, (1, 1), (partner, 1),
            LaserMomentumLink(power_w=10_000.0, magnification=100.0, range_m=200e3))
    end
    return model
end

# Weights and scales that are not powers of two, so that any change in the order of
# the mix's arithmetic changes its bits.
const MIX = WeightedPolicy(PolicyTerm(SemimajorAxisRatePolicy(targets=1); scale=0.3, weight=0.7),
    PolicyTerm(SemimajorAxisRatePolicy(targets=:both); scale=1.7, weight=-0.35))

bits(x) = reinterpret(UInt64, collect(Float64, x))

@testset "Interlink link schedule" begin
    @testset "history samples" begin
        key = ((1, 1), (2, 1))
        @test InterLinkHistory(1.0, [key]).scores == Float64[]
        @test size(InterLinkHistory(1.0, [key]).terms) == (0, 1)
        @test size(InterLinkHistory(1.0, [key], InterLinkKey[], [2.0]).terms) == (0, 1)
        @test InterLinkHistory(1.0, [key], InterLinkKey[], [2.0], [3.0 ; 4.0 ;;]).terms == [3.0; 4.0;;]
        @test_throws ArgumentError InterLinkHistory(1.0, [key], InterLinkKey[], [1.0, 2.0])
        @test_throws ArgumentError InterLinkHistory(1.0, [key], InterLinkKey[], [1.0], zeros(2, 2))
    end

    @testset "each sample records the score that selected each link" begin
        args = gve_sma_configuration(duration=600.0)
        checked = Ref(0)
        mismatches = Ref(0)
        probe = DiffEqBase.DiscreteCallback((u, t, integrator) -> true, function (integrator)
            run_args = integrator.p.args
            model = run_args.interlink_model
            sample = model.history[end]
            work = run_args.scheduling_policy_model.matching
            for (key, score) in zip(sample.active, sample.scores)
                checked[] += 1
                score === work.score[model.graph.edge_index_lookup[key]] || (mismatches[] += 1)
            end
            length(sample.scores) == length(sample.active) || (mismatches[] += 1)
        end; save_positions=(false, false))
        run_simulation(args; extra_callbacks=(probe,))
        @test checked[] > 0
        @test mismatches[] == 0
    end

    @testset "a weighted policy's terms" begin
        args = gve_sma_configuration(duration=20.0)
        spacecraft = args.dynamics_model.spacecraft
        state = SE.build_initial_conditions(args)
        mu = args.environment_model.planet.μ
        scored = map((false, true)) do record_terms
            model = laser_model(spacecraft; record_terms)
            scheduler = SchedulingPolicyModel(MIX)
            update_availability!(model, spacecraft, state)
            score_candidates!(model, scheduler, state, mu; spacecraft)
            (model=model, work=scheduler.matching)
        end
        plain, recorded = scored
        @test plain.work.terms === nothing
        @test size(recorded.work.terms) == (2, 2)
        # Recording the terms leaves the mix's scores unchanged, bit for bit.
        @test bits(plain.work.score) == bits(recorded.work.score)
        @test any(!iszero, recorded.work.score)
        # Each row is its term's own score.
        for (k, term) in enumerate(MIX.terms)
            alone = SchedulingPolicyModel(term.policy)
            score_candidates!(recorded.model, alone, state, mu; spacecraft)
            @test bits(recorded.work.terms[k, :]) == bits(alone.matching.score)
        end
        # A nested mix is one term, its own mixed score.
        nested = WeightedPolicy(PolicyTerm(MIX; scale=1.0), PolicyTerm(DataThroughputPolicy(); scale=1e6))
        model = laser_model(spacecraft; record_terms=true)
        scheduler = SchedulingPolicyModel(nested)
        update_availability!(model, spacecraft, state)
        score_candidates!(model, scheduler, state, mu; spacecraft)
        @test size(scheduler.matching.terms) == (2, 2)
        @test bits(scheduler.matching.terms[1, :]) == bits(recorded.work.score)
        @test all(iszero, scheduler.matching.terms[2, :])

        # In a run: the history and the table carry the terms, and the run is the
        # same with and without recording them.
        runs = map((false, true)) do record_terms
            run_args = with_policy(args, MIX; model=laser_model(spacecraft; record_terms))
            run_simulation(run_args; return_results=true, return_solution=true)
        end
        @test bits(runs[1].solution.u[end]) == bits(runs[2].solution.u[end])
        history = runs[2].configuration.interlink_model.history
        @test all(sample -> size(sample.terms) == (2, length(sample.active)), history)
        @test all(sample -> size(sample.terms, 1) == 0, runs[1].configuration.interlink_model.history)
        table = link_schedule(runs[2])
        @test "term1_start" in names(table) && "term2_start" in names(table) && !("term3_start" in names(table))
        @test !("term1_start" in names(link_schedule(runs[1])))
        @test all(==("active"), table.window)   # no contact rows without recorded availability
        sample = history[findfirst(sample -> sample.time == table.start_s[1], history)]
        index = findfirst(==(((table.a_spacecraft[1], table.a_terminal[1]), (table.b_spacecraft[1], table.b_terminal[1]))),
            sample.active)
        @test table.term1_start[1] === sample.terms[1, index]
        @test table.term2_start[1] === sample.terms[2, index]
        @test DataFrames.metadata(table) == Dict("policy" => "WeightedPolicy", "score_unit" => "1")

        # Only a weighted policy has terms to record.
        refused = with_policy(args, SemimajorAxisRatePolicy(); model=laser_model(spacecraft; record_terms=true))
        @test_throws ArgumentError run_simulation(refused)
    end

    @testset "the table from a hand-built history" begin
        spacecraft = [(n_terminal=2,) for _ in 1:3]
        model = InterlinkModel()
        laser = register_link!(model, spacecraft, (1, 1), (2, 1),
            LaserMomentumLink(power_w=12.0, magnification=3.0, range_m=100.0))
        data = register_link!(model, spacecraft, (2, 2), (3, 1),
            DataLink(transmit_power_w=1.0, budget=FixedRateBudget(1e6), range_m=1e6))
        @test laser < data
        push!(model.history,
            InterLinkHistory(0.0, [laser], [laser, data], [1.0]),
            InterLinkHistory(10.0, [laser, data], [laser, data], [2.0, 5.0]),
            InterLinkHistory(20.0, [data], [data], [6.0]),
            InterLinkHistory(30.0, InterLinkKey[], [data], Float64[]),
            InterLinkHistory(40.0, [laser], InterLinkKey[], [3.0]))
        table = link_schedule(model; policy=SchedulingPolicyModel(PowerDeliveryPolicy()))
        @test names(table) == ["window", "link", "kind", "directed", "a_spacecraft", "a_terminal", "b_spacecraft",
            "b_terminal", "start_s", "end_s", "duration_s", "steps", "score_start", "score_end", "score_min", "score_max"]
        @test eltype.(eachcol(table)) == [String, Int, String, Bool, Int, Int, Int, Int, Float64, Float64, Float64,
            Int, Float64, Float64, Float64, Float64]
        @test table.window == ["active", "contact", "contact", "active", "active"]
        @test table.link == [1, 1, 2, 2, 1]
        @test table.kind == ["LaserMomentumLink", "LaserMomentumLink", "DataLink{FixedRateBudget}", "DataLink{FixedRateBudget}",
            "LaserMomentumLink"]
        @test table.directed == [false, false, true, true, false]
        @test [(row.a_spacecraft, row.a_terminal, row.b_spacecraft, row.b_terminal) for row in eachrow(table)] ==
            [(1, 1, 2, 1), (1, 1, 2, 1), (2, 2, 3, 1), (2, 2, 3, 1), (1, 1, 2, 1)]
        @test table.start_s == [0.0, 0.0, 0.0, 10.0, 40.0]
        @test table.end_s == [20.0, 20.0, 40.0, 30.0, 40.0]
        @test table.duration_s == [20.0, 20.0, 40.0, 20.0, 0.0]
        @test table.steps == [2, 2, 4, 2, 1]
        active = table.window .== "active"
        @test table.score_start[active] == [1.0, 5.0, 3.0]
        @test table.score_end[active] == [2.0, 6.0, 3.0]
        @test table.score_min[active] == [1.0, 5.0, 3.0]
        @test table.score_max[active] == [2.0, 6.0, 3.0]
        @test all(isnan, Matrix(table[.!active, [:score_start, :score_end, :score_min, :score_max]]))
        @test DataFrames.metadata(table) == Dict("policy" => "PowerDeliveryPolicy", "score_unit" => "W")
        # The same spans as contact_windows.
        for (rows, windows) in ((table[active, :], contact_windows(model.history; active=true)),
                                (table[.!active, :], contact_windows(model.history)))
            @test [(row.start_s, row.end_s, row.duration_s) for row in eachrow(rows)] ==
                [(window.start_s, window.end_s, window.duration_s) for window in windows]
        end
        # Samples without scores give NaN; terms give their columns.
        bare = InterlinkModel()
        register_link!(bare, spacecraft, (1, 1), (2, 1), LaserMomentumLink(power_w=1.0, magnification=1.0, range_m=1.0))
        push!(bare.history, InterLinkHistory(0.0, [laser]), InterLinkHistory(5.0, InterLinkKey[]))
        bare_table = link_schedule(bare)
        @test nrow(bare_table) == 1 && isnan(bare_table.score_start[1]) && bare_table.end_s[1] == 5.0
        @test DataFrames.metadata(bare_table) == Dict("policy" => "", "score_unit" => "")
        push!(bare.history, InterLinkHistory(6.0, [laser], InterLinkKey[], [4.0], [7.0; 8.0;;]))
        termed = link_schedule(bare; policy=MIX)
        @test isnan(termed.term1_start[1]) && termed.term1_start[2] == 7.0
        @test isnan(termed.term2_start[1]) && termed.term2_start[2] == 8.0
        @test DataFrames.metadata(termed)["score_unit"] == "1"
        # An empty history gives the columns and no rows.
        @test size(link_schedule(InterlinkModel())) == (0, 16)
    end

    @testset "files, manifest and returned results" begin
        mktempdir() do directory
            args = with_settings(gve_sma_configuration(duration=600.0); results=true, results_directory=directory,
                save_csv=true)
            result = run_simulation(args; return_results=true)
            table = link_schedule(result)
            @test nrow(table) > 0
            csv_path = joinpath(directory, "simulation_results_link_schedule.csv")
            feather_path = joinpath(directory, "simulation_results_link_schedule.feather")
            @test csv_path in result.files && feather_path in result.files
            from_csv = CSV.read(csv_path, DataFrame)
            from_feather = DataFrame(Arrow.Table(feather_path))
            for frame in (from_csv, from_feather)
                @test names(frame) == names(table)
                @test frame.window == table.window && frame.link == table.link && frame.steps == table.steps
                @test frame.start_s == table.start_s && frame.end_s == table.end_s
                @test isequal(frame.score_start, table.score_start) && isequal(frame.score_max, table.score_max)
            end
            manifest = TOML.parsefile(joinpath(directory, "simulation_results.manifest.toml"))
            @test manifest["files"]["link_schedule_csv"]["sha256"] == bytes2hex(sha256(read(csv_path)))
            @test manifest["files"]["link_schedule_feather"]["path"] == feather_path
            @test manifest["interlinks"] == Dict("policy" => "SemimajorAxisRatePolicy", "score_unit" => "m/s",
                "rows" => nrow(table), "record_available" => false, "record_terms" => false)
            # Without a CSV, the feather file alone.
            rm(csv_path)
            run_simulation(with_settings(args; save_csv=false))
            @test !isfile(csv_path) && isfile(feather_path)
            manifest = TOML.parsefile(joinpath(directory, "simulation_results.manifest.toml"))
            @test !haskey(manifest["files"], "link_schedule_csv") && haskey(manifest["files"], "link_schedule_feather")
        end
        # In memory only: no files, and the table still comes back.
        in_memory = run_simulation(gve_sma_configuration(duration=60.0); return_results=true)
        @test isempty(in_memory.files) && nrow(link_schedule(in_memory)) > 0
        # A run without interlinks writes no schedule and returns the empty table.
        mktempdir() do directory
            plain_args = with_settings(gve_sma_configuration(duration=60.0, with_interlinks=false); results=true,
                results_directory=directory, save_csv=true)
            plain = run_simulation(plain_args; return_results=true)
            @test !isfile(joinpath(directory, "simulation_results_link_schedule.csv"))
            @test !isfile(joinpath(directory, "simulation_results_link_schedule.feather"))
            @test !haskey(TOML.parsefile(joinpath(directory, "simulation_results.manifest.toml")), "interlinks")
            @test size(link_schedule(plain)) == (0, 16)
        end
        # The schedule is not thinned with the results rows.
        thinned = withenv("SPACEAGORA_RESULTS_FINAL_ONLY" => "1") do
            run_simulation(gve_sma_configuration(duration=600.0); return_results=true)
        end
        @test nrow(thinned.table) == 1
        @test link_schedule(thinned).steps == link_schedule(run_simulation(gve_sma_configuration(duration=600.0);
            return_results=true)).steps
    end

    @testset "a resumed run's schedule starts at the resume" begin
        mktempdir() do directory
            args = with_settings(gve_sma_configuration(duration=20.0); results=false, checkpoint_enabled=true,
                checkpoint_interval_s=10.0, checkpoint_directory=directory)
            run_simulation(args)
            longer = SM.SimConfig._with_configuration(args;
                mission_configuration=gve_sma_configuration(duration=30.0).mission_configuration)
            resumed = run_simulation(with_settings(longer; resume_from_checkpoint=true); return_results=true)
            table = link_schedule(resumed)
            @test minimum(table.start_s) == 20.0
            @test maximum(table.end_s) == 30.0
        end
    end
end

end # module InterlinkScheduleTests
