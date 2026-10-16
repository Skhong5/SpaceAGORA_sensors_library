# Bounded tests of scripts/contributor_environment.jl: local Git fixtures only; no Pkg
# operation, network access or core import. The contributor workflow runs them in core CI.
using Test, TOML
include(joinpath(@__DIR__, "..", "..", "scripts", "contributor_environment.jl"))
using .ContributorEnvironment: plan, scan, scan_source, summary, options, companion_names, CORE_URL, RECORD

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
            "sources" => Dict("SpaceAGORA" => Dict("url" => CORE_URL, "rev" => rev)),
            "compat" => Dict("julia" => "~1.12", "SpaceAGORA" => "0.2", "Unused" => "1"))
        write_toml(joinpath(repo, "Project.toml"), project)
        mkpath(joinpath(repo, "src"))
        write(joinpath(repo, "src", "MyStudy.jl"), "module MyStudy\nusing SpaceAGORA\nend\n")
        f((; temporary, core, rev, repo, project, companion_rev))
    end
end

reproject(fixture, data) = write_toml(joinpath(fixture.repo, "Project.toml"), data)
with_project(f, fixture, edit) = begin
    data = deepcopy(fixture.project)
    edit(data)
    reproject(fixture, data)
    try f() finally reproject(fixture, fixture.project) end
end
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
            @test p.pin == "url + rev"
            @test p.package == (; name="MyStudy", uuid=fixture.project["uuid"])
            @test p.project["deps"] == Dict("SpaceAGORA" => CORE_UUID, "Test" => TEST_UUID, "MyStudy" => fixture.project["uuid"])
            @test p.project["sources"]["SpaceAGORA"] == Dict("path" => fixture.core)
            @test p.project["sources"]["MyStudy"] == Dict("path" => fixture.repo)
            @test p.project["compat"] == Dict("julia" => "~1.12", "SpaceAGORA" => "0.2")
            @test p.developed == [fixture.core, fixture.repo]
            @test p.seed == joinpath(fixture.core, "Manifest.toml")
            @test plan("project", fixture.repo, env(fixture); root=fixture.core).core.rev == fixture.rev
            @test plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev, manifest="resolve").seed === nothing
            with_project(fixture, d -> d["sources"]["SpaceAGORA"] = Dict("path" => "../core")) do
                @test plan("project", fixture.repo, env(fixture); root=fixture.core).pin == "path ../core"
            end
            with_project(fixture, d -> d["sources"]["SpaceAGORA"]["url"] = chop(CORE_URL; tail=4)) do
                @test plan("project", fixture.repo, env(fixture); root=fixture.core).pin == "url + rev"
            end
            # A package's test target and a test/Project.toml both reach the environment.
            with_project(fixture, d -> begin
                d["deps"] = Dict("SpaceAGORA" => CORE_UUID)
                d["extras"] = Dict("Test" => TEST_UUID)
                d["targets"] = Dict("test" => ["Test"])
            end) do
                p = plan("package", fixture.repo, env(fixture); root=fixture.core)
                @test p.project["deps"]["Test"] == TEST_UUID
                @test isempty(p.project["compat"])
            end
            write_toml(joinpath(fixture.repo, "test", "Project.toml"), Dict("deps" => Dict("Random" => "9a3f8284-a2c9-5f02-9a11-845980a1fd5c", "MyStudy" => fixture.project["uuid"])))
            @test haskey(plan("package", fixture.repo, env(fixture); root=fixture.core).project["deps"], "Random")
            rm(joinpath(fixture.repo, "test"); recursive=true)
            # A project without a module is a plain environment.
            mv(joinpath(fixture.repo, "src"), joinpath(fixture.temporary, "src-aside"))
            p = plan("project", fixture.repo, env(fixture); root=fixture.core)
            @test p.package === nothing && p.developed == [fixture.core]
            refused(ArgumentError) do
                plan("package", fixture.repo, env(fixture); root=fixture.core)
            end
            mv(joinpath(fixture.temporary, "src-aside"), joinpath(fixture.repo, "src"))
        end

        @testset "The [sources] pin must name the tested commit" begin
            other = repeat("a", 40)
            for (label, edit, reason) in (
                    ("different commit", d -> d["sources"]["SpaceAGORA"]["rev"] = other, "but the tested revision is"),
                    ("short commit", d -> d["sources"]["SpaceAGORA"]["rev"] = fixture.rev[1:12], "is not a full 40-character"),
                    ("uppercase commit", d -> d["sources"]["SpaceAGORA"]["rev"] = uppercase(fixture.rev), "is not a full 40-character"),
                    ("branch", d -> d["sources"]["SpaceAGORA"]["rev"] = "main", "is not a full 40-character"),
                    ("no rev", d -> delete!(d["sources"]["SpaceAGORA"], "rev"), "has no rev"),
                    ("other repository", d -> d["sources"]["SpaceAGORA"]["url"] = "https://github.com/someone/SpaceAGORA.jl.git", "url must be"),
                    ("subdir", d -> d["sources"]["SpaceAGORA"]["subdir"] = "x", "must not set subdir"),
                    ("path elsewhere", d -> d["sources"]["SpaceAGORA"] = Dict("path" => "."), "is not the core under test"),
                    ("path and rev", d -> d["sources"]["SpaceAGORA"]["path"] = "../core", "mixes path"),
                    ("no pin", d -> delete!(d["sources"], "SpaceAGORA"), "[sources] must pin SpaceAGORA"),
                    ("no SpaceAGORA dependency", d -> delete!(d["deps"], "SpaceAGORA"), "[deps] must list SpaceAGORA"),
                    ("another SpaceAGORA UUID", d -> d["deps"]["SpaceAGORA"] = TEST_UUID, "[deps] must list SpaceAGORA"))
                with_project(fixture, edit) do
                    @testset "$label" begin
                        refused_with(reason) do
                            plan("project", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev)
                        end
                    end
                end
            end
            err = try
                with_project(fixture, d -> d["sources"]["SpaceAGORA"]["rev"] = other) do
                    plan("package", fixture.repo, env(fixture); root=fixture.core, rev=fixture.rev)
                end
            catch e
                e
            end
            @test err isa ArgumentError && occursin("pins SpaceAGORA $other, but the tested revision is $(fixture.rev)", err.msg)
        end

        @testset "The core checkout must be the requested commit" begin
            # The [sources] pin names the requested commit, so only the checkout check can refuse.
            for (rev, reason) in ((repeat("b", 40), "is at $(fixture.rev), not the requested revision"),
                                  (fixture.rev[1:7], "must be a full 40-character lowercase commit SHA"))
                with_project(fixture, d -> d["sources"]["SpaceAGORA"]["rev"] = rev) do
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
            for (kind, manifest, destination) in (("library", "core", env(fixture)), ("project", "latest", env(fixture)),
                    ("project", "core", joinpath(fixture.core, "env")), ("project", "core", joinpath(fixture.repo, "env")),
                    ("project", "committed", env(fixture)))
                refused(ArgumentError) do
                    plan(kind, fixture.repo, destination; manifest, root=fixture.core)
                end
            end
            mkpath(env(fixture, "used")); write(joinpath(env(fixture, "used"), "x"), "")
            refused(ArgumentError) do
                plan("project", fixture.repo, env(fixture, "used"); root=fixture.core)
            end
            write(joinpath(fixture.repo, "Manifest.toml"), "")
            @test plan("project", fixture.repo, env(fixture); manifest="committed", root=fixture.core).seed == joinpath(fixture.repo, "Manifest.toml")
            versioned = joinpath(fixture.repo, "Manifest-v$(VERSION.major).$(VERSION.minor).toml")
            write(versioned, "")
            @test plan("project", fixture.repo, env(fixture); manifest="committed", root=fixture.core).seed == versioned
            rm(versioned); rm(joinpath(fixture.repo, "Manifest.toml"))
            @test companion_names("mujoco, edg mujoco\nhypr") == ["mujoco", "edg", "hypr"]
            @test companion_names("") == String[]
            refused(ArgumentError) do
                companion_names("mujoco visualizer")
            end
        end

        @testset "Companion dependencies follow the core's pins" begin
            depend = d -> (d["deps"][MUJOCO.name] = MUJOCO.uuid)
            with_project(fixture, depend) do
                refused(ArgumentError) do
                    plan("project", fixture.repo, env(fixture); root=fixture.core)
                end
                @test plan("project", fixture.repo, env(fixture); root=fixture.core, companions="mujoco").companions == ["mujoco"]
            end
            for (rev, accepted) in ((fixture.companion_rev, true), (repeat("d", 40), false))
                with_project(fixture, d -> (depend(d); d["sources"][MUJOCO.name] = Dict("url" => "https://example.invalid/m.git", "rev" => rev))) do
                    run_plan = () -> plan("project", fixture.repo, env(fixture); root=fixture.core, companions=["mujoco"])
                    accepted ? (@test run_plan().companions == ["mujoco"]) : refused(run_plan, ArgumentError)
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
            "spaceagora_rev" => repeat("e", 40), "spaceagora_pin" => "url + rev",
            "companions" => Dict("mujoco" => Dict("package" => MUJOCO.name, "revision" => repeat("c", 40)))))
        write_toml(joinpath(reports, "controller.toml"), Dict("target" => "MyController", "kind" => "control_model",
            "checks" => [Dict("id" => "hooks_present", "status" => "pass", "since" => "0.3.0", "detail" => ""),
                         Dict("id" => "repeatable_run", "status" => "fail", "since" => "0.3.0", "detail" => "u differs | at 3")]))
        text = summary(reports, environment)
        @test occursin("- Kind: package (`MyGNC`); manifest: core", text)
        @test occursin("`$(repeat("e", 40))`, pinned in [sources] by url + rev", text)
        @test occursin("- Companion mujoco: $(MUJOCO.name) `$(repeat("c", 40))`", text)
        @test occursin("| MyController (control_model) | hooks_present | pass | 0.3.0 |  |", text)
        @test occursin("| MyController (control_model) | repeatable_run | fail | 0.3.0 | u differs \\| at 3 |", text)
        @test occursin("1 fail, 1 pass", text)
    end
    @test options(["project", "r", "e", "--manifest=core", "--rev", "x"]) ==
          (["project", "r", "e"], Dict("manifest" => "core", "rev" => "x"))
    @test_throws ArgumentError options(["project", "r", "e", "--frozen", "x"])
    @test_throws ArgumentError options(["project", "--manifest"])
end
