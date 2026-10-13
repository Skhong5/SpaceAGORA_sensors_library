using Test
using TOML

# The asset check must not call a Git LFS pointer, an empty directory or an
# empty payload "usable". The asset source is loaded into a sandbox module so
# this fixture runs without compiling the package; the file is located by name
# so the fixture follows it across source moves.

function _asset_source(repo::String)
    hits = String[]
    for (root, _, files) in walkdir(joinpath(repo, "src"))
        "assets.jl" in files && push!(hits, joinpath(root, "assets.jl"))
    end
    length(hits) == 1 || error("Expected exactly one assets.jl under src; found $(length(hits)).")
    return only(hits)
end

const _LFS_POINTER = "version https://git-lfs.github.com/spec/v1\noid sha256:" * "0"^64 * "\nsize 149545985\n"

function _write(path::String, text::String)
    mkpath(dirname(path))
    write(path, text)
    return path
end

# The sandbox is built at file top level so the methods it defines are visible
# to the test set below (a later top-level expression) without invokelatest.
const _ASSET_FIXTURE = mktempdir()
const _ASSET_SANDBOX = Module(gensym(:AssetCheck))
Core.eval(_ASSET_SANDBOX, :(const REPO_ROOT = $_ASSET_FIXTURE))
Base.include(_ASSET_SANDBOX, _asset_source(normpath(joinpath(@__DIR__, "..", "..", ".."))))

@testset "Asset readiness states" begin
    fixture = _ASSET_FIXTURE
    sandbox = _ASSET_SANDBOX
    try
        # Directory entries.
        mkpath(joinpath(fixture, "empty_dir"))
        _write(joinpath(fixture, "dotfile_dir", ".DS_Store"), "x")
        _write(joinpath(fixture, "pointer_dir", "a.jls"), _LFS_POINTER)
        _write(joinpath(fixture, "pointer_dir", "b.jls"), _LFS_POINTER)
        _write(joinpath(fixture, "mixed_dir", "a.jls"), _LFS_POINTER)
        _write(joinpath(fixture, "mixed_dir", "nested", "b.jls"), "payload bytes")
        _write(joinpath(fixture, "zero_dir", "a.csv"), "")
        _write(joinpath(fixture, "zero_dir", "b.csv"), "")
        _write(joinpath(fixture, "usable_dir", "nested", "truth.csv"), "t,x\n0,1\n")
        _write(joinpath(fixture, "usable_dir", ".hidden"), "ignored")
        # File entries.
        _write(joinpath(fixture, "empty.feather"), "")
        _write(joinpath(fixture, "pointer.jls"), _LFS_POINTER)
        _write(joinpath(fixture, "usable.stl"), "solid cube\nendsolid cube\n")
        # A short real payload must not be mistaken for a pointer.
        _write(joinpath(fixture, "short.toml"), "v = 1\n")

        entry(name, kind, path; required=false) = Dict(
            "name" => name, "scope" => "test", "kind" => kind, "relative_path" => path,
            "required" => required, "licensing" => "open")
        manifest = joinpath(fixture, "assets_manifest.toml")
        open(manifest, "w") do io
            TOML.print(io, Dict("asset" => [
                entry("builtin_entry", "builtin", "."; required=true),
                entry("missing_dir", "directory", "missing_dir"; required=true),
                entry("empty_dir", "directory", "empty_dir"),
                entry("dotfile_dir", "directory", "dotfile_dir"),
                entry("pointer_dir", "directory", "pointer_dir"),
                entry("mixed_dir", "directory", "mixed_dir"),
                entry("zero_dir", "directory", "zero_dir"),
                entry("usable_dir", "directory", "usable_dir"; required=true),
                entry("missing_file", "file", "missing.feather"),
                entry("empty_file", "file", "empty.feather"),
                entry("pointer_file", "file", "pointer.jls"),
                entry("usable_file", "file", "usable.stl"; required=true),
                entry("short_file", "file", "short.toml"),
            ]))
        end

        report = sandbox.check_assets(; repo_root=fixture, manifest_path=manifest)
        states = Dict(item.name => item.state for item in report.items)
        reasons = Dict(item.name => item.reason for item in report.items)
        @test states["builtin_entry"] === :usable
        @test states["missing_dir"] === :missing
        @test states["empty_dir"] === :empty
        @test states["dotfile_dir"] === :empty
        @test states["pointer_dir"] === :pointer_only
        @test states["mixed_dir"] === :pointer_only
        @test states["zero_dir"] === :invalid
        @test states["usable_dir"] === :usable
        @test states["missing_file"] === :missing
        @test states["empty_file"] === :invalid
        @test states["pointer_file"] === :pointer_only
        @test states["usable_file"] === :usable
        @test states["short_file"] === :usable

        # `available` agrees with the state, and every non-usable entry explains itself.
        for item in report.items
            @test item.available == (item.state === :usable)
            @test isempty(item.reason) == (item.state === :usable)
        end
        @test occursin("2 of 2 files are undownloaded Git LFS pointers", reasons["pointer_dir"])
        @test occursin("1 of 2 files are undownloaded Git LFS pointers alongside 1 downloaded payload file", reasons["mixed_dir"])
        @test occursin("git lfs pull --include=\"pointer.jls\"", reasons["pointer_file"])
        @test occursin("git lfs pull --include=\"pointer_dir/*\"", reasons["pointer_dir"])
        @test occursin("all 2 files are empty", reasons["zero_dir"])

        # Rendering names the state, adds the reason and ends with a summary.
        text = sprint(io -> sandbox.render_asset_report(report; io=io))
        @test occursin("- missing_dir: missing-required", text)
        @test occursin("- pointer_dir: pointer-only-optional", text)
        @test occursin("- zero_dir: invalid-optional", text)
        @test occursin("- usable_dir: usable", text)
        @test !occursin(": available", text)
        @test occursin("  reason: the directory contains no files", text)
        @test occursin("summary: 4 of 13 entries usable; 1 required entries not usable", text)

        # Exit status follows required entries only.
        @test sandbox.asset_check_exit_status(report) == 1
        open(manifest, "w") do io
            TOML.print(io, Dict("asset" => [
                entry("builtin_entry", "builtin", "."; required=true),
                entry("usable_dir", "directory", "usable_dir"; required=true),
                entry("pointer_dir", "directory", "pointer_dir"),
            ]))
        end
        clean = sandbox.check_assets(; repo_root=fixture, manifest_path=manifest)
        @test sandbox.asset_check_exit_status(clean) == 0
        @test any(item -> item.name == "pointer_dir" && item.state === :pointer_only, clean.items)

        # Unknown kinds and unknown states are rejected, not guessed.
        open(manifest, "w") do io
            TOML.print(io, Dict("asset" => [entry("odd", "archive", "usable.stl")]))
        end
        @test_throws ArgumentError sandbox.check_assets(; repo_root=fixture, manifest_path=manifest)
        @test_throws ArgumentError sandbox._asset_item("x", "test", fixture; state=:present, required=false)

        # Keyword construction without a state still works for existing callers.
        legacy = sandbox.AssetCheckItem(name="x", scope="test", path=fixture, available=false, required=true)
        @test legacy.state === :missing
        @test isempty(legacy.reason)
    finally
        rm(fixture; recursive=true, force=true)
    end
end
