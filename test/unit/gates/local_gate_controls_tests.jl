using Test
using TOML

# Negative controls for the four contract gates that the full local runner
# (test/contracts/runtests.jl) adds on top of test/contracts/pr_runtests.jl.
# Each gate runs against an isolated copy of the source tree, so a forbidden
# construct never enters the working checkout. The copy is the whole src/ tree
# plus the small inputs the gates read, and injection targets are located by
# file name, so the controls follow the gates across source moves.

const _GATE_ROOT_LINE = "const REPO_ROOT = normpath(joinpath(@__DIR__, \"..\", \"..\"))"

function _copy_into!(fixture::String, repo::String, rel::String)
    source = joinpath(repo, rel)
    target = joinpath(fixture, rel)
    mkpath(dirname(target))
    cp(source, target)
    return target
end

function _unique_source(fixture::String, name::String)
    hits = String[]
    for (root, _, files) in walkdir(joinpath(fixture, "src"))
        name in files && push!(hits, joinpath(root, name))
    end
    length(hits) == 1 || error("Expected exactly one $(name) under the fixture source tree; found $(length(hits)).")
    return only(hits)
end

function _gate_runner(repo::String, fixture::String, gate::String)
    path = joinpath(repo, "test", "gates", gate)
    source = read(path, String)
    occursin(_GATE_ROOT_LINE, source) || error("$(gate) does not define the expected REPO_ROOT line.")
    isolated_source = replace(source, _GATE_ROOT_LINE => "const REPO_ROOT = $(repr(fixture))"; count=1)
    return function ()
        isolated = Module(gensym(Symbol(gate)))
        # Module(name) has no include binding. Gate helpers must load into the
        # same isolated module, with paths resolved from the gate file.
        Core.eval(isolated, :(include(path) = Base.include(@__MODULE__, path)))
        failure = try
            redirect_stdout(devnull) do
                Base.include_string(isolated, isolated_source, path)
            end
            nothing
        catch err
            err
        end
        violations = Core.eval(isolated,
            :(isdefined(@__MODULE__, :violations) ? copy(violations) : String[]))
        return (; failure, violations)
    end
end

_failed_with(result, message::String) =
    result.failure isa LoadError &&
    result.failure.error isa ErrorException &&
    result.failure.error.msg == message

_failed_starting(result, prefix::String) =
    result.failure isa LoadError &&
    result.failure.error isa ErrorException &&
    startswith(result.failure.error.msg, prefix)

_reports(result, probe::String, fixture::String, fragment::String) =
    any(v -> startswith(v, relpath(probe, fixture) * ":") && occursin(fragment, v), result.violations)

function _with_text(f, path::String, text::String)
    original = read(path, String)
    write(path, text)
    try
        f()
    finally
        write(path, original)
    end
end

function _with_probe(f, path::String, text::String)
    mkpath(dirname(path))
    write(path, text)
    try
        f()
    finally
        rm(path)
    end
end

@testset "Local contract gates reject deliberate violations" begin
    repo = normpath(joinpath(@__DIR__, "..", "..", ".."))
    mktempdir() do fixture
        _copy_into!(fixture, repo, "src")
        _copy_into!(fixture, repo, "Project.toml")
        _copy_into!(fixture, repo, joinpath("packages", "SpaceAGORAHYPR", "src"))

        @testset "Project compat gate" begin
            run_gate = _gate_runner(repo, fixture, "ci_project_compat_gate.jl")
            @test isnothing(run_gate().failure)

            project_path = joinpath(fixture, "Project.toml")
            project = TOML.parsefile(project_path)
            compat = project["compat"]
            # A real package dependency whose compat entry the gate must require.
            @test haskey(compat, "StaticArrays")
            # A standard library with a compat entry; the gate exempts standard libraries.
            @test haskey(compat, "Base64")
            original = read(project_path, String)
            # Remove the entry from the [compat] table only; [deps] lists the same names first.
            compat_start = first(findfirst("[compat]", original))
            function without(name)
                head = original[1:compat_start - 1]
                tail = original[compat_start:end]
                return head * replace(tail, Regex("^" * name * " = .*\\n", "m") => ""; count=1)
            end
            @test without("StaticArrays") != original
            @test without("Base64") != original

            _with_text(project_path, without("StaticArrays")) do
                @test _failed_with(run_gate(), "Missing compat entries in Project.toml: StaticArrays")
            end
            _with_text(project_path, without("Base64")) do
                @test isnothing(run_gate().failure)
            end
            @test isnothing(run_gate().failure)
        end

        @testset "Translational ownership gate" begin
            run_gate = _gate_runner(repo, fixture, "ci_translational_ownership_gate.jl")
            @test isnothing(run_gate().failure)

            owner_dir = dirname(_unique_source(fixture, "translational_models.jl"))
            outsider_dir = dirname(_unique_source(fixture, "guidance_hooks.jl"))
            forbidden = "du_view.pos .= sc_view.vel\n"
            outsider_probe = joinpath(outsider_dir, "translational_probe.jl")

            _with_probe(outsider_probe, forbidden) do
                rejected = run_gate()
                @test _failed_with(rejected, "Translational ownership gate failed")
                @test _reports(rejected, outsider_probe, fixture, "forbidden inline translational dynamics pattern")
            end
            # Comments are not active source.
            _with_probe(outsider_probe, "# " * forbidden) do
                @test isnothing(run_gate().failure)
            end
            # The owner directory may contain the equations.
            _with_probe(joinpath(owner_dir, "translational_probe.jl"), forbidden) do
                @test isnothing(run_gate().failure)
            end
            @test isnothing(run_gate().failure)
        end

        @testset "Runtime legacy surface gate" begin
            run_gate = _gate_runner(repo, fixture, "ci_runtime_legacy_surface_gate.jl")
            @test isnothing(run_gate().failure)

            # A shared forbidden token in a listed file.
            listed = _unique_source(fixture, "io_paths.jl")
            _with_text(listed, read(listed, String) * "\ngravity_const(1.0)\n") do
                rejected = run_gate()
                @test _failed_with(rejected, "Runtime legacy surface gate failed")
                @test _reports(rejected, listed, fixture, "gravity_const")
            end
            # A file-specific forbidden token.
            setup = _unique_source(fixture, "setup.jl")
            _with_text(setup, read(setup, String) * "\n_compat_probe = nothing\n") do
                rejected = run_gate()
                @test _failed_with(rejected, "Runtime legacy surface gate failed")
                @test _reports(rejected, setup, fixture, "_compat_")
            end
            # Directory targets are scanned recursively.
            guidance_dir = dirname(_unique_source(fixture, "adapter_queries.jl"))
            nested_probe = joinpath(guidance_dir, "nested", "legacy_probe.jl")
            _with_probe(nested_probe, "gravity_GRAM(x)\n") do
                rejected = run_gate()
                @test _failed_with(rejected, "Runtime legacy surface gate failed")
                @test _reports(rejected, nested_probe, fixture, "gravity_GRAM")
            end
            rm(dirname(nested_probe))
            # A missing listed target is an explicit failure, not a silent skip.
            persistence = _unique_source(fixture, "persistence.jl")
            persistence_source = read(persistence, String)
            rm(persistence)
            try
                @test _failed_with(run_gate(),
                    "Missing runtime legacy-surface target: " * relpath(persistence, fixture))
            finally
                write(persistence, persistence_source)
            end
            @test isnothing(run_gate().failure)
        end

        @testset "Ownership gates contain by path, not by name prefix" begin
            for (gate, owner_file, forbidden, failure, message) in (
                ("ci_translational_ownership_gate.jl", "translational_models.jl",
                    "du_view.pos .= sc_view.vel\n", "Translational ownership gate failed",
                    "forbidden inline translational dynamics pattern"),
                ("ci_rotational_ownership_gate.jl", "rotational_models.jl",
                    "dq = 0.5 * quat_mult(q, w)\n", "Rotational ownership gate failed",
                    "forbidden inline rotational dynamics pattern"),
            )
                run_gate = _gate_runner(repo, fixture, gate)
                @test isnothing(run_gate().failure)
                owner_dir = dirname(_unique_source(fixture, owner_file))
                # A sibling whose name merely starts with the owner's name is not the owner.
                sibling = joinpath(dirname(owner_dir), basename(owner_dir) * "_outside", "probe.jl")
                _with_probe(sibling, forbidden) do
                    rejected = run_gate()
                    @test _failed_with(rejected, failure)
                    @test _reports(rejected, sibling, fixture, message)
                end
                rm(dirname(sibling))
                # Nested directories inside the owner remain exempt.
                nested = joinpath(owner_dir, "nested", "probe.jl")
                _with_probe(nested, forbidden) do
                    @test isnothing(run_gate().failure)
                end
                rm(dirname(nested))
                # A missing owner root is an explicit failure, not an empty scan.
                parked = owner_dir * "_parked"
                mv(owner_dir, parked)
                try
                    @test _failed_with(run_gate(), "Missing " * split(basename(owner_dir), "_")[1] * " ownership root")
                finally
                    mv(parked, owner_dir)
                end
                @test isnothing(run_gate().failure)
            end
        end

        @testset "Benchmark mode parity gate" begin
            # The gate loads SpaceAGORA through the fixture root on LOAD_PATH; keep
            # the process load path unchanged afterwards.
            modes_rel = joinpath("benchmarks", "studies", "parallelization_performance", "modes.jl")
            _copy_into!(fixture, repo, modes_rel)
            modes = joinpath(fixture, modes_rel)
            original = read(modes, String)
            gate = _gate_runner(repo, fixture, "ci_ppc_mode_profile_parity_gate.jl")
            run_gate = () -> (result = gate(); filter!(!=(fixture), LOAD_PATH); result)
            @test isnothing(run_gate().failure)
            disagree = "Benchmark mode table disagrees with the shipped profiles"
            # An inline mode block that drifts from its profile.
            inline = replace(original, "\"outer_threads\" => PPCModeSpec(\n            name=\"outer_threads\",\n            profile=\"R1a\",\n            backend=\"threads\",\n            outer_active=true,\n            policy_adaptive=false,\n            rhs_batch=\"auto\",\n            density=\"off\"" =>
                "\"outer_threads\" => PPCModeSpec(\n            name=\"outer_threads\",\n            profile=\"R1a\",\n            backend=\"threads\",\n            outer_active=true,\n            policy_adaptive=false,\n            rhs_batch=\"auto\",\n            density=\"bogus\""; count=1)
            @test inline != original
            _with_text(modes, inline) do
                @test _failed_starting(run_gate(), disagree)
            end
            # The serial mode is bound to a local name before it enters the table;
            # the gate must follow that binding rather than skip the mode.
            serial = replace(original, "    serial = PPCModeSpec(\n        name=\"serial\",\n        profile=\"R0\",\n        backend=\"none\",\n        outer_active=false,\n        policy_adaptive=false,\n        rhs_batch=\"off\",\n        density=\"off\"" =>
                "    serial = PPCModeSpec(\n        name=\"serial\",\n        profile=\"R0\",\n        backend=\"none\",\n        outer_active=false,\n        policy_adaptive=false,\n        rhs_batch=\"off\",\n        density=\"bogus\""; count=1)
            @test serial != original
            _with_text(modes, serial) do
                @test _failed_starting(run_gate(), disagree)
            end
            # A checked mode that disappears from the table is an explicit failure.
            _with_text(modes, replace(original, "\"serial\" => serial," => ""; count=1)) do
                @test _failed_starting(run_gate(), "Benchmark mode table has no PPCModeSpec for checked mode \"serial\"")
            end
            # A missing table is an explicit failure.
            rm(modes)
            try
                @test _failed_starting(run_gate(), "modes.jl not found")
            finally
                write(modes, original)
            end
            @test isnothing(run_gate().failure)
        end

        @testset "Typed command boundary gate" begin
            run_gate = _gate_runner(repo, fixture, "ci_gnc_typed_command_boundary_gate.jl")
            @test isnothing(run_gate().failure)

            # The control owner must import the typed command from CommandTypes.
            hooks = _unique_source(fixture, "control_hooks.jl")
            hooks_source = read(hooks, String)
            needle = "using ..CommandTypes: PropulsiveManeuverCommand"
            @test occursin(needle, hooks_source)
            _with_text(hooks, replace(hooks_source, needle => "using ..CommandTypes: AerobrakingControlCommand"; count=1)) do
                @test _failed_with(run_gate(),
                    "Typed-command boundary violation: ControlHooks is not importing PropulsiveManeuverCommand from CommandTypes.")
            end
            # maneuver_commands may only be touched by the typed writer and consumer.
            guidance_dir = dirname(_unique_source(fixture, "guidance_hooks.jl"))
            probe = joinpath(guidance_dir, "ownership_probe.jl")
            _with_probe(probe, "x = p.shared_buffers.maneuver_commands\n") do
                rejected = run_gate()
                @test _failed_with(rejected, "GNC typed-command boundary gate failed")
                @test _reports(rejected, probe, fixture, "maneuver_commands ownership is restricted")
            end
            @test isnothing(run_gate().failure)
        end

        @testset "Deprecated hook methods gate" begin
            run_gate = _gate_runner(repo, fixture, "ci_deprecated_hook_methods_gate.jl")
            @test isnothing(run_gate().failure)
            message = "Deprecated hook methods found in src/, templates/ or examples/."
            # A method of the deprecated hook is refused in every spelling.
            probe = joinpath(dirname(_unique_source(fixture, "gravity_gradient_effector.jl")), "deprecated_hook_probe.jl")
            for text in ("calcForceTorque(model::Probe, x, p, i) = (0.0, 0.0)\n",
                         "function SimulationModel.calcForceTorque(model::Probe, x::AbstractVector, p, i::Int)\n    return 0.0, 0.0\nend\n",
                         "@inline SpaceAGORA.calcForceTorque(::Probe, x, p, i)::Tuple = (0.0, 0.0)\n",
                         "(f::typeof(calcForceTorque))(model::Probe, x, p, i) = (0.0, 0.0)\n",
                         "(::typeof(SimulationModel.calcForceTorque))(model::Probe, x, p, i) = (0.0, 0.0)\n")
                _with_probe(probe, text) do
                    rejected = run_gate()
                    @test _failed_with(rejected, message)
                    @test _reports(rejected, probe, fixture, "deprecated hook calcForceTorque")
                end
            end
            # The forwarder for direct calls stays, with four untyped positional arguments.
            forwarder = joinpath(fixture, "src", "orchestration", "simulation", "engine", "effector_sampling.jl")
            original = read(forwarder, String)
            needle = "function SimulationModel.calcForceTorque(model, x, p, i)"
            @test occursin(needle, original)
            for changed in ("function SimulationModel.calcForceTorque(model, x, p, i::Int)",
                            "function SimulationModel.calcForceTorque(model, x, p, i, extra)")
                _with_text(forwarder, replace(original, needle => changed; count=1)) do
                    rejected = run_gate()
                    @test _failed_with(rejected, message)
                    @test any(v -> occursin("forwarder for direct calls (four untyped positional arguments) is missing", v), rejected.violations)
                end
            end
            @test isnothing(run_gate().failure)
        end
    end
end

@testset "Random-stream gate distinguishes in-place destinations" begin
    repo = normpath(joinpath(@__DIR__, "..", "..", ".."))
    mktempdir() do fixture
        _copy_into!(fixture, repo, joinpath("src", "models", "gnc"))
        _copy_into!(fixture, repo, joinpath("packages", "SpaceAGORAHYPR", "src"))
        run_gate = _gate_runner(repo, fixture, "ci_gnc_random_stream_gate.jl")
        @test isnothing(run_gate().failure)
        probe = joinpath(fixture, "src", "models", "gnc", "random_stream_probe.jl")
        message = "draw without an explicit stream as its first argument"
        failure = "GNC random-stream gate failed: take an explicit rng argument and pass it to every draw"

        for draw in ("rand!", "randn!", "randexp!", "shuffle!", "randperm!", "randcycle!")
            @testset "$draw" begin
                for source in (
                    "rng_buffer = zeros(3); $draw(rng_buffer)\n",
                    "Random.$draw(\n    rng_buffer,\n)\n",
                    "$draw(view(rng_buffer, :, 1))\n",
                    "$draw((rng_buffer,))\n",
                    "$draw(rng_buffer; keyword=rng)\n",
                    "$draw(rng_buffer, keyword=rng)\n",
                    "$draw(rng_buffer, rest...)\n",
                    "$draw(rng..., buffer)\n",
                )
                    _with_probe(probe, source) do
                        rejected = run_gate()
                        @test _failed_with(rejected, failure)
                        @test _reports(rejected, probe, fixture, message)
                    end
                end
                for source in (
                    "$draw(rng, buffer)\n",
                    "Random.$draw(\n    rng, # caller's stream\n    view(buffer, :, 1),\n)\n",
                    "$draw(state.master_rng, buffer)\n",
                )
                    _with_probe(probe, source) do
                        @test isnothing(run_gate().failure)
                    end
                end
            end
        end
        # The permutation APIs have exactly two explicit-stream arguments.
        for draw in ("randperm!", "randcycle!"), source in (
            "$draw(rng, buffer, extra)\n",
            "$draw(rng, buffer, rest...)\n",
        )
            _with_probe(probe, source) do
                rejected = run_gate()
                @test _failed_with(rejected, failure)
                @test _reports(rejected, probe, fixture, message)
            end
        end
        @testset "randsubseq!" begin
            # A destination named rng_output is not the first of four arguments.
            for source in (
                "randsubseq!(rng_output, values, p)\n",
                "Random.randsubseq!(\n    rng_output,\n    view(values, :, 1),\n    p,\n)\n",
                "randsubseq!(rng, output)\n",
                "randsubseq!(rng_output, values, p; keyword=rng)\n",
                "randsubseq!(rng, output, values, p=p)\n",
                "randsubseq!(rng..., output, values, p)\n",
                "randsubseq!(rng, outputs..., values, p)\n",
                "randsubseq!(rng, output, values..., p)\n",
                "randsubseq!(rng, output, values, probabilities...)\n",
                "randsubseq!(rng, output, values, p, extra)\n",
            )
                _with_probe(probe, source) do
                    rejected = run_gate()
                    @test _failed_with(rejected, failure)
                    @test _reports(rejected, probe, fixture, message)
                end
            end
            for source in (
                "randsubseq!(rng, output, values, p)\n",
                "Random.randsubseq!(\n    rng, # caller's stream\n    output,\n    view(values, :, 1),\n    p,\n)\n",
                "randsubseq!(state.master_rng, output, values, p)\n",
            )
                _with_probe(probe, source) do
                    @test isnothing(run_gate().failure)
                end
            end
        end
        # Existing variable-arity forms keep optional arguments after rng and A.
        for draw in ("rand!", "randn!", "randexp!", "shuffle!"), source in (
            "$draw(rng, buffer, extra)\n",
            "$draw(rng, buffer, rest...)\n",
        )
            _with_probe(probe, source) do
                @test isnothing(run_gate().failure)
            end
        end
        # Non-mutating calls still require the named first stream argument.
        for source in ("rand()\n", "randn(3)\n", "randexp(Float64)\n", "rand!(buffer, rng)\n")
            _with_probe(probe, source) do
                rejected = run_gate()
                @test _failed_with(rejected, failure)
                @test _reports(rejected, probe, fixture, message)
            end
        end
        _with_probe(probe, "rand(rng)\nrandn(master_rng, 3)\nrandexp(state.rng)\n") do
            @test isnothing(run_gate().failure)
        end
        # Comments and string examples are not draw call sites.
        _with_probe(probe, "# rand!(rng_buffer)\nexample = \"rand!(rng_buffer)\"\n") do
            @test isnothing(run_gate().failure)
        end
        # Both roots remain covered, with the draw's source line in the report.
        hypr_probe = joinpath(fixture, "packages", "SpaceAGORAHYPR", "src", "random_stream_probe.jl")
        _with_probe(hypr_probe, "rng_buffer = zeros(3)\nrand!(\n    rng_buffer\n)\n") do
            rejected = run_gate()
            @test _failed_with(rejected, failure)
            @test _reports(rejected, hypr_probe, fixture, "random_stream_probe.jl:2: " * message)
        end
        @test isnothing(run_gate().failure)
    end
end
