# Bounded tests of scripts/contributor_environment.jl: local Git fixtures only; no Pkg
# operation, network access or core import. The contributor workflow runs them in core CI.
using Test, TOML
include(joinpath(@__DIR__, "..", "..", "scripts", "contributor_environment.jl"))
using .ContributorEnvironment: plan, check_repository, scan, scan_source, summary, options, companion_names,
    CORE_URL, PIN_FILE, RECORD

const CORE_UUID = "afbfb69f-5c0b-4832-b760-43725dff8540"
const TEST_UUID = "8dfed614-e22c-5e08-85e1-65c5234f0b40"
const MUJOCO = (name="SpaceAGORAMuJoCo", path="packages/robotics/SpaceAGORAMuJoCo.jl",
                uuid="a46e04e1-adb7-43c3-a03b-26575bf4fb67")

function fixture_git(args...)
    addenv(Cmd(["git", "-c", "init.defaultBranch=main", "-c", "user.name=Contributor fixture",
                "-c", "user.email=fixture@example.invalid", "-c", "core.autocrlf=false", String.(args)...]),
           "GIT_DIR" => nothing, "GIT_WORK_TREE" => nothing, "GIT_INDEX_FILE" => nothing,
           "GIT_COMMON_DIR" => nothing, "GIT_OBJECT_DIRECTORY" => nothing,
           "GIT_ALTERNATE_OBJECT_DIRECTORIES" => nothing, "GIT_CONFIG_NOSYSTEM" => "1",
           "GIT_CONFIG_GLOBAL" => (Sys.iswindows() ? "NUL" : "/dev/null"))
end
fixture_run(args...) = run(pipeline(fixture_git(args...); stdout=devnull, stderr=devnull))
fixture_read(args...) = readchomp(fixture_git(args...))

function write_toml(path, data)
    mkpath(dirname(path))
    open(io -> TOML.print(io, data; sorted=true), path, "w")
end

# A core checkout with one companion gitlink, and a contributor repository beside it.
function contributor_fixture(f)
    mktempdir(; prefix="spaceagora-contributor-fixture-") do temporary
        temporary = realpath(temporary)
        core = joinpath(temporary, "core")
        fixture_run("init", "--quiet", core)
        write_toml(joinpath(core, "Project.toml"), Dict("name" => "SpaceAGORA", "uuid" => CORE_UUID, "version" => "0.2.0"))
        write(joinpath(core, "Manifest.toml"), "# Core manifest sentinel\n")
        registry = Dict(
            "hypr" => Dict("name" => "HYPR", "path" => "packages/robotics/HYPR.jl", "url" => "https://example.invalid/HYPR.jl.git",
                           "uuid" => "342c8629-b435-4bda-9d24-004c735fb297", "version" => "0.1.1"),
            "mujoco" => Dict("name" => MUJOCO.name, "path" => MUJOCO.path, "url" => "https://example.invalid/SpaceAGORAMuJoCo.jl.git",
                             "uuid" => MUJOCO.uuid, "version" => "0.1.0"),
            "edg" => Dict("name" => "SpaceAGORAEDG", "path" => "packages/gnc/SpaceAGORAEDG.jl", "url" => "https://example.invalid/SpaceAGORAEDG.jl.git",
                          "uuid" => "a782e8ee-96d7-4dd6-8231-bb64649e6ad4", "version" => "0.1.0"))
        write_toml(joinpath(core, "packages", "OptionalPackages.toml"), registry)
        write(joinpath(core, ".gitmodules"), "[submodule \"$(MUJOCO.name)\"]\n\tpath = $(MUJOCO.path)\n\turl = $(registry["mujoco"]["url"])\n")
        companion_rev = repeat("c", 40)
        fixture_run("-C", core, "update-index", "--add", "--cacheinfo", "160000,$companion_rev,$(MUJOCO.path)")
        fixture_run("-C", core, "add", "Project.toml", "Manifest.toml", ".gitmodules", "packages/OptionalPackages.toml")
        fixture_run("-C", core, "commit", "--quiet", "-m", "Fixture core")
        rev = fixture_read("-C", core, "rev-parse", "HEAD")
        repo = joinpath(temporary, "my-study")
        project = Dict{String,Any}(
            "name" => "MyStudy", "uuid" => "2c0a3d4e-1111-4222-8333-444455556666",
            "deps" => Dict("SpaceAGORA" => CORE_UUID, "Test" => TEST_UUID),
            "compat" => Dict("julia" => "~1.12", "SpaceAGORA" => "0.2", "Unused" => "1"))
        pin = Dict{String,Any}("url" => CORE_URL, "rev" => rev, "uuid" => CORE_UUID)
        write_toml(joinpath(repo, "Project.toml"), project)
        write_toml(joinpath(repo, PIN_FILE), pin)
        mkpath(joinpath(repo, "src"))
        write(joinpath(repo, "src", "MyStudy.jl"), "module MyStudy\nusing SpaceAGORA\nend\n")
        f((; temporary, core, rev, repo, project, pin, companion_rev))
    end
end

# Edit a copy of the project or pin file for one block (an edit returning :remove
# deletes the file), then restore it.
function with_toml(f, path, original, edit)
    data = deepcopy(original)
    edit(data) === :remove ? rm(path) : write_toml(path, data)
    try f() finally write_toml(path, original) end
end
with_project(f, fixture, edit) = with_toml(f, joinpath(fixture.repo, "Project.toml"), fixture.project, edit)
with_pin(f, fixture, edit) = with_toml(f, joinpath(fixture.repo, PIN_FILE), fixture.pin, edit)
refused(f, pattern) = @test_throws pattern f()
# Each refusal names its own reason, so one check cannot mask another.
function refused_with(f, fragment)
    err = try
        f()
        nothing
    catch e
        e
    end
    @test err isa ArgumentError && occursin(fragment, err.msg)
end
env(fixture, name="env") = joinpath(fixture.temporary, name)

@testset "Contributor environment plan" begin
    contributor_fixture() do fixture
        @testset "Accepted pins and environment contents" begin
            p = plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev)
            @test p.pin == "rev $(fixture.rev)"
            @test p.companions == String[]
            @test p.package == (; name="MyStudy", uuid=fixture.project["uuid"])
            @test p.project["deps"] == Dict("SpaceAGORA" => CORE_UUID, "Test" => TEST_UUID, "MyStudy" => fixture.project["uuid"])
            @test p.project["sources"]["SpaceAGORA"] == Dict("path" => fixture.core)
            @test p.project["sources"]["MyStudy"] == Dict("path" => fixture.repo)
            @test p.project["compat"] == Dict("julia" => "~1.12", "SpaceAGORA" => "0.2")
            @test p.developed == [fixture.core, fixture.repo]
            @test p.seed == joinpath(fixture.core, "Manifest.toml")
            @test plan("project", fixture.repo, env(fixture); root=fixture.core).core.rev == fixture.rev
            @test plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev, manifest="resolve").seed === nothing
            @test check_repository("package", fixture.repo; root=fixture.core).pin == "rev $(fixture.rev)"
            with_pin(fixture, d -> d["url"] = chop(CORE_URL; tail=4)) do
                @test plan("project", fixture.repo, env(fixture); root=fixture.core).pin == "rev $(fixture.rev)"
            end
            with_pin(fixture, d -> d["version"] = "0.2.0") do
                @test plan("project", fixture.repo, env(fixture); root=fixture.core).pin == "rev $(fixture.rev)"
            end
            # Core CI's own fixture names the checkout under test by path.
            with_pin(fixture, d -> (delete!(d, "url"); delete!(d, "rev"); d["path"] = "../core")) do
                @test plan("project", fixture.repo, env(fixture); root=fixture.core, core_from_caller=true).pin == "path ../core"
                refused_with("a path pin is for core CI only") do
                    plan("project", fixture.repo, env(fixture); root=fixture.core)
                end
            end
            # A package's test target reaches the environment with its bounds.
            with_project(fixture, d -> begin
                d["deps"] = Dict("SpaceAGORA" => CORE_UUID)
                d["extras"] = Dict("Test" => TEST_UUID, "Aqua" => "4c88cf16-eb10-579e-8560-4a9242c79595")
                d["targets"] = Dict("test" => ["Test", "Aqua"])
                d["compat"] = Dict("julia" => "~1.12", "SpaceAGORA" => "0.2", "Aqua" => "0.8", "Unused" => "1")
            end) do
                p = plan("package", fixture.repo, env(fixture); root=fixture.core)
                @test p.project["deps"]["Test"] == TEST_UUID
                @test p.project["compat"] == Dict("julia" => "~1.12", "SpaceAGORA" => "0.2", "Aqua" => "0.8")
            end
            # A project without a module is a plain environment.
            mv(joinpath(fixture.repo, "src"), joinpath(fixture.temporary, "src-aside"))
            p = plan("project", fixture.repo, env(fixture); root=fixture.core)
            @test p.package === nothing && p.developed == [fixture.core]
            refused_with("module file is missing") do
                plan("package", fixture.repo, env(fixture); root=fixture.core)
            end
            mv(joinpath(fixture.temporary, "src-aside"), joinpath(fixture.repo, "src"))
        end

        @testset "A test/Project.toml reaches the environment with its bounds and sources" begin
            test_project = joinpath(fixture.repo, "test", "Project.toml")
            random = "9a3f8284-a2c9-5f02-9a11-845980a1fd5c"
            helper = "6d3c0f1e-2222-4333-8444-555566667777"
            write_toml(test_project, Dict(
                "deps" => Dict("Random" => random, "TestHelper" => helper, "MyStudy" => fixture.project["uuid"]),
                "compat" => Dict("Random" => "1.11", "TestHelper" => "0.3", "Unrelated" => "2"),
                "sources" => Dict("TestHelper" => Dict("path" => "../vendor/TestHelper"), "MyStudy" => Dict("path" => ".."))))
            p = plan("package", fixture.repo, env(fixture); root=fixture.core)
            @test p.project["deps"]["TestHelper"] == helper && !haskey(p.project["compat"], "Unrelated")
            @test p.project["compat"]["Random"] == "1.11" && p.project["compat"]["TestHelper"] == "0.3"
            @test p.project["sources"]["TestHelper"] == Dict("path" => joinpath(fixture.repo, "vendor", "TestHelper"))
            @test p.project["sources"]["MyStudy"] == Dict("path" => fixture.repo)
            # A bound in both projects becomes their intersection; disjoint bounds are refused.
            with_toml(test_project, TOML.parsefile(test_project), d -> d["compat"]["SpaceAGORA"] = "0.2.0 - 0.2.5, 0.3") do
                @test plan("project", fixture.repo, env(fixture); root=fixture.core).project["compat"]["SpaceAGORA"] == "0.2.0 - 0.2.5"
            end
            with_toml(test_project, TOML.parsefile(test_project), d -> d["compat"]["SpaceAGORA"] = "0.3") do
                refused_with("does not overlap") do
                    plan("project", fixture.repo, env(fixture); root=fixture.core)
                end
            end
            with_toml(test_project, TOML.parsefile(test_project), d -> d["sources"]["SpaceAGORA"] = Dict("path" => "../../core")) do
                refused_with("test/Project.toml: remove the SpaceAGORA entry from [sources]") do
                    plan("package", fixture.repo, env(fixture); root=fixture.core)
                end
            end
            with_toml(test_project, TOML.parsefile(test_project), d -> d["deps"][MUJOCO.name] = MUJOCO.uuid) do
                refused_with("list \"mujoco\" in SpaceAGORASource.toml companions") do
                    plan("package", fixture.repo, env(fixture); root=fixture.core)
                end
            end
            rm(joinpath(fixture.repo, "test"); recursive=true)
        end

        @testset "Every direct dependency keeps its source" begin
            # The environment lists the repository's dependencies directly, so a source the
            # package declares does not reach them through the developed package: carry it.
            shared = "7e3c1a2b-3333-4444-8555-666677778888"
            test_project = joinpath(fixture.repo, "test", "Project.toml")
            own = d -> begin
                d["deps"]["Shared"] = shared
                d["sources"] = Dict("Shared" => Dict("path" => "deps/Shared"))
            end
            with_project(fixture, own) do
                for kind in ("package", "project")
                    @test plan(kind, fixture.repo, env(fixture); root=fixture.core).project["sources"]["Shared"] ==
                          Dict("path" => joinpath(fixture.repo, "deps", "Shared"))
                end
            end
            # A test-project source for a dependency of the package itself is carried too.
            write_toml(test_project, Dict("deps" => Dict("Shared" => shared),
                                          "sources" => Dict("Shared" => Dict("path" => "../vendor/Shared"))))
            with_project(fixture, d -> d["deps"]["Shared"] = shared) do
                @test plan("package", fixture.repo, env(fixture); root=fixture.core).project["sources"]["Shared"] ==
                      Dict("path" => joinpath(fixture.repo, "vendor", "Shared"))
            end
            # Both declare one: the test project's wins for a package, as under Pkg.test;
            # the project's own wins for a project, which is the environment its tests run in.
            with_project(fixture, own) do
                @test plan("package", fixture.repo, env(fixture); root=fixture.core).project["sources"]["Shared"] ==
                      Dict("path" => joinpath(fixture.repo, "vendor", "Shared"))
                @test plan("project", fixture.repo, env(fixture); root=fixture.core).project["sources"]["Shared"] ==
                      Dict("path" => joinpath(fixture.repo, "deps", "Shared"))
            end
            rm(joinpath(fixture.repo, "test"); recursive=true)
        end

        @testset "The pin file must name the tested commit" begin
            other = repeat("a", 40)
            for (label, edit, reason) in (
                    ("different commit", d -> d["rev"] = other, "pins SpaceAGORA $other, but the tested revision is $(fixture.rev)"),
                    ("short commit", d -> d["rev"] = fixture.rev[1:12], "is not a full 40-character"),
                    ("uppercase commit", d -> d["rev"] = uppercase(fixture.rev), "is not a full 40-character"),
                    ("branch", d -> d["rev"] = "main", "is not a full 40-character"),
                    ("no rev", d -> delete!(d, "rev"), "rev is missing"),
                    ("other repository", d -> d["url"] = "https://github.com/someone/SpaceAGORA.jl.git", "url must be"),
                    ("no url", d -> delete!(d, "url"), "url must be"),
                    ("no uuid", d -> delete!(d, "uuid"), "uuid must be SpaceAGORA's"),
                    ("another uuid", d -> d["uuid"] = TEST_UUID, "uuid must be SpaceAGORA's"),
                    ("another version", d -> d["version"] = "0.3.0", "is not 0.2.0, the SpaceAGORA version"),
                    ("unknown key", d -> d["subdir"] = "x", "unknown key(s) subdir"),
                    ("companions as a string", d -> d["companions"] = "mujoco", "companions must be a list"),
                    ("unknown companion", d -> d["companions"] = ["visualizer"], "Unknown companion 'visualizer'"),
                    ("missing pin file", d -> :remove, "SpaceAGORASource.toml is missing"))
                with_pin(fixture, edit) do
                    @testset "$label" begin
                        refused_with(reason) do
                            plan("package", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev)
                        end
                    end
                end
            end
            for (label, edit, reason) in (
                    ("path elsewhere", d -> (delete!(d, "url"); delete!(d, "rev"); d["path"] = "."), "is not the core under test"),
                    ("path and rev", d -> d["path"] = "../core", "either path or url and rev"))
                with_pin(fixture, edit) do
                    @testset "$label" begin
                        refused_with(reason) do
                            plan("project", fixture.repo, env(fixture); root=fixture.core, core_from_caller=true)
                        end
                    end
                end
            end
        end

        @testset "Project.toml carries no SpaceAGORA pin" begin
            for (label, edit, reason) in (
                    ("[sources] url and rev", d -> d["sources"] = Dict("SpaceAGORA" => Dict("url" => CORE_URL, "rev" => fixture.rev)),
                     "remove the SpaceAGORA entry from [sources]"),
                    ("[sources] path", d -> d["sources"] = Dict("SpaceAGORA" => Dict("path" => "../core")),
                     "remove the SpaceAGORA entry from [sources]"),
                    ("no SpaceAGORA dependency", d -> delete!(d["deps"], "SpaceAGORA"), "[deps] must list SpaceAGORA"),
                    ("another SpaceAGORA UUID", d -> d["deps"]["SpaceAGORA"] = TEST_UUID, "[deps] must list SpaceAGORA"))
                with_project(fixture, edit) do
                    @testset "$label" begin
                        refused_with(reason) do
                            plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev, core_from_caller=true)
                        end
                    end
                end
            end
        end

        @testset "The core checkout must be the requested commit" begin
            # The pin file names the requested commit, so only the checkout check can refuse.
            for (rev, reason) in ((repeat("b", 40), "is at $(fixture.rev), not the requested revision"),
                                  (fixture.rev[1:7], "must be a full 40-character lowercase commit SHA"))
                with_pin(fixture, d -> d["rev"] = rev) do
                    refused_with(reason) do
                        plan("project", fixture.repo, env(fixture); root=fixture.core, rev)
                    end
                end
            end
            write(joinpath(fixture.core, "Manifest.toml"), "# changed\n")
            refused_with("has tracked changes") do
                plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev)
            end
            fixture_run("-C", fixture.core, "checkout", "--quiet", "--", "Manifest.toml")
            write(joinpath(fixture.core, "untracked.txt"), "untracked files do not change the tested commit\n")
            @test plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev).core.rev == fixture.rev
            refused(ArgumentError) do
                plan("project", fixture.repo, env(fixture); root=fixture.repo)
            end
        end

        @testset "Inputs and destinations" begin
            for (kind, manifest, destination, reason) in (
                    ("library", "core", env(fixture), "Unknown kind"), ("project", "latest", env(fixture), "Unknown manifest mode"),
                    ("project", "core", joinpath(fixture.core, "env"), "outside the SpaceAGORA checkout"),
                    ("project", "core", joinpath(fixture.repo, "env"), "outside the repository"),
                    ("project", "committed", env(fixture), "no committed Manifest.toml"))
                refused_with(reason) do
                    plan(kind, fixture.repo, destination; manifest, root=fixture.core)
                end
            end
            mkpath(env(fixture, "used")); write(joinpath(env(fixture, "used"), "x"), "")
            refused_with("new or empty environment directory") do
                plan("project", fixture.repo, env(fixture, "used"); root=fixture.core)
            end
            write(joinpath(fixture.repo, "Manifest.toml"), "")
            @test plan("project", fixture.repo, env(fixture); manifest="committed", root=fixture.core).seed == joinpath(fixture.repo, "Manifest.toml")
            versioned = joinpath(fixture.repo, "Manifest-v$(VERSION.major).$(VERSION.minor).toml")
            write(versioned, "")
            @test plan("project", fixture.repo, env(fixture); manifest="committed", root=fixture.core).seed == versioned
            rm(versioned); rm(joinpath(fixture.repo, "Manifest.toml"))
            @test companion_names(["mujoco", "edg", "mujoco", "hypr"]) == ["mujoco", "edg", "hypr"]
            @test companion_names(String[]) == String[]
        end

        @testset "Companions come from the pin file, at the core's pins" begin
            depend = d -> (d["deps"][MUJOCO.name] = MUJOCO.uuid)
            with_project(fixture, depend) do
                refused_with("list \"mujoco\" in SpaceAGORASource.toml companions") do
                    plan("project", fixture.repo, env(fixture); root=fixture.core)
                end
                with_pin(fixture, d -> d["companions"] = ["mujoco"]) do
                    @test plan("project", fixture.repo, env(fixture); root=fixture.core).companions == ["mujoco"]
                end
            end
            for (rev, accepted) in ((fixture.companion_rev, true), (repeat("d", 40), false))
                with_project(fixture, d -> (depend(d); d["sources"] = Dict(MUJOCO.name => Dict("url" => "https://example.invalid/m.git", "rev" => rev)))) do
                    with_pin(fixture, d -> d["companions"] = ["mujoco"]) do
                        run_plan = () -> plan("project", fixture.repo, env(fixture); root=fixture.core)
                        accepted ? (@test run_plan().companions == ["mujoco"]) :
                            refused_with(run_plan, "pins $(MUJOCO.name) $rev, but this core pins $(fixture.companion_rev)")
                    end
                end
            end
        end
    end
end

@testset "Internal-name scan" begin
    internal = [
        ("SpaceAGORA.SimulationModel.Planets", 1, "SpaceAGORA.SimulationModel"),
        ("x = SpaceAGORA.SimulationEngine", 1, "SpaceAGORA.SimulationEngine"),
        ("using SpaceAGORA.SimulationModel", 1, "SpaceAGORA.SimulationModel"),
        ("import SpaceAGORA: run_simulation, _missing_model_hooks", 1, "SpaceAGORA._missing_model_hooks"),
        ("using SpaceAGORA: SimulationEngine as E", 1, "SpaceAGORA.SimulationEngine"),
        ("SpaceAGORA._hook_call_types(1)", 1, "SpaceAGORA._hook_call_types"),
        ("SpaceAGORA.Conformance._fixture", 1, "SpaceAGORA.Conformance._fixture"),
        ("getproperty(SpaceAGORA, :_x)", 1, "SpaceAGORA._x"),
        ("import SpaceAGORA as S\nf() = S._y", 2, "S._y"),
        ("const S = SpaceAGORA\nS.SimulationModel", 2, "S.SimulationModel"),
        ("function f()\n    g(SpaceAGORA._z)\nend", 2, "SpaceAGORA._z"),
    ]
    for (source, line, name) in internal
        @test scan_source(source, "case.jl") == [(line, "$name is internal to SpaceAGORA")]
    end
    # Aliases follow lexical scope: a local or argument named like an alias is not SpaceAGORA.
    scoped = [
        ("f() = (S = SpaceAGORA; S.run_simulation)\ng(S) = S._cache", Tuple{Int,String}[]),
        ("function h()\n    S = SpaceAGORA\n    S._x\nend", [(3, "S._x is internal to SpaceAGORA")]),
        ("x -> (S = SpaceAGORA; S._t)", [(1, "S._t is internal to SpaceAGORA")]),
        ("const S = SpaceAGORA\nk(S) = S._x\nm() = S._y", [(3, "S._y is internal to SpaceAGORA")]),
        ("const S = SpaceAGORA\nfunction n(; S=Dict())\n    S._k\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\nfunction q()\n    S = Dict()\n    S._l\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\nfunction p()\n    t = (S = 1,)\n    S._n\nend", [(4, "S._n is internal to SpaceAGORA")]),
        ("const S = SpaceAGORA\nfor S in (1, 2)\n    S._w\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\nlet S = Dict()\n    S._z\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\nmap(xs) do S\n    S._v\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\ntry\n    f()\ncatch S\n    S._e\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\n[S._g for S in xs]", Tuple{Int,String}[]),
        ("module Inner\nf() = S._u\nend\nconst S = SpaceAGORA", Tuple{Int,String}[]),
        ("SpaceAGORA._hook(x) = 1", [(1, "SpaceAGORA._hook is internal to SpaceAGORA")]),
        # Return annotations and `where` bounds belong to the signature, scanned in the enclosing scope.
        ("const S = SpaceAGORA\nf()::S._Private = 1", [(2, "S._Private is internal to SpaceAGORA")]),
        ("function g(x)::SpaceAGORA._Ret\n    x\nend", [(1, "SpaceAGORA._Ret is internal to SpaceAGORA")]),
        ("h(x::T) where {T<:SpaceAGORA._Abstract} = x", [(1, "SpaceAGORA._Abstract is internal to SpaceAGORA")]),
        ("const S = SpaceAGORA\nk(x::T) where T<:S._B = x", [(2, "S._B is internal to SpaceAGORA")]),
        ("m(x::T) where {SpaceAGORA._Lo<:T<:Real} = x", [(1, "SpaceAGORA._Lo is internal to SpaceAGORA")]),
        ("const T = SpaceAGORA\nn(x::T) where {T} = T._z", Tuple{Int,String}[]),
        # A later `for` clause sees the earlier loop variables; a comma comprehension's
        # iterables are evaluated outside it, where S is still the alias.
        ("const S = SpaceAGORA\nfor S in xs, x in S._items\n    x\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\nfor (S, y) in zs, x in S._v\n    x\nend", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\nfor x in S._items, S in xs\n    S\nend", [(2, "S._items is internal to SpaceAGORA")]),
        ("const S = SpaceAGORA\nfor S in S._items\n    S\nend", [(2, "S._items is internal to SpaceAGORA")]),
        ("const S = SpaceAGORA\n[x for S in xs for x in S._items]", Tuple{Int,String}[]),
        ("const S = SpaceAGORA\n[x for S in xs, x in S._items]", [(2, "S._items is internal to SpaceAGORA")]),
        ("function r(x::SpaceAGORA._Internal)\n    x\nend", [(1, "SpaceAGORA._Internal is internal to SpaceAGORA")]),
    ]
    for (source, expected) in scoped
        @test scan_source(source, "case.jl") == expected
    end
    for source in ("using SpaceAGORA\nrun_simulation(args)", "SpaceAGORA.wrench(m, x, env, t) = 0",
                   "SpaceAGORA.Conformance.check_sensor(s)", "# SpaceAGORA._x in a comment", "s = \"SpaceAGORA.SimulationModel\"",
                   "Base._private()", "x._y", "using .SimulationModel")
        @test isempty(scan_source(source, "case.jl"))
    end
    @test scan_source("f(x = ", "case.jl") == [(1, "the file does not parse")]
    mktempdir() do repo
        write(joinpath(mkpath(joinpath(repo, "src")), "A.jl"), "module A\nusing SpaceAGORA\nend\n")
        write(joinpath(mkpath(joinpath(repo, "test")), "runtests.jl"), "using Test\n\nSpaceAGORA.SimulationModel.Foo\n")
        write(joinpath(mkpath(joinpath(repo, ".spaceagora", "src")), "Core.jl"), "SpaceAGORA._inside_the_core = 1\n")
        @test scan("project", repo) == [(joinpath("test", "runtests.jl"), 3, "SpaceAGORA.SimulationModel is internal to SpaceAGORA")]
        @test scan("package", repo) == scan("project", repo)
        @test isempty(scan("companion", repo))
        @test_throws ArgumentError scan("study", repo)
    end
end

@testset "Summary and command line" begin
    mktempdir() do dir
        reports, environment = mkpath(joinpath(dir, "reports")), mkpath(joinpath(dir, "env"))
        text = summary(joinpath(dir, "absent"))
        @test occursin("scientific validation is the author's", text)
        @test occursin("No conformance reports were written", text)
        write_toml(joinpath(environment, RECORD), Dict("kind" => "package", "package" => "MyGNC", "manifest" => "core",
            "spaceagora_rev" => repeat("e", 40), "spaceagora_pin" => "rev $(repeat("e", 40))",
            "companions" => Dict("mujoco" => Dict("package" => MUJOCO.name, "revision" => repeat("c", 40)))))
        write_toml(joinpath(reports, "controller.toml"), Dict("target" => "MyController", "kind" => "control_model",
            "checks" => [Dict("id" => "hooks_present", "status" => "pass", "since" => "0.3.0", "detail" => ""),
                         Dict("id" => "repeatable_run", "status" => "fail", "since" => "0.3.0", "detail" => "u differs | at 3")]))
        text = summary(reports, environment)
        @test occursin("- Kind: package (`MyGNC`); manifest: core", text)
        @test occursin("`$(repeat("e", 40))`, pinned in SpaceAGORASource.toml by rev $(repeat("e", 40))", text)
        @test occursin("- Companion mujoco: $(MUJOCO.name) `$(repeat("c", 40))`", text)
        @test occursin("| MyController (control_model) | hooks_present | pass | 0.3.0 |  |", text)
        @test occursin("| MyController (control_model) | repeatable_run | fail | 0.3.0 | u differs \\| at 3 |", text)
        @test occursin("1 fail, 1 pass", text)
    end
    @test options(["project", "r", "e", "--manifest=core", "--rev", "x"]) ==
          (["project", "r", "e"], Dict("manifest" => "core", "rev" => "x"))
    @test options(["check", "project", "r", "--core-from-caller"]) == (["check", "project", "r"], Dict("core-from-caller" => "true"))
    @test_throws ArgumentError options(["project", "r", "e", "--frozen", "x"])
    @test_throws ArgumentError options(["project", "r", "e", "--companions", "mujoco"])
    @test_throws ArgumentError options(["project", "--manifest"])
end
