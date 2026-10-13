module ViewerOutputPreservationTests

using Test
Base.include(@__MODULE__, joinpath(@__DIR__, "..", "..", "helpers", "load_visualizer.jl"))
include(joinpath(@__DIR__, "..", "..", "optional_visualizer", "fixtures.jl"))

# Use tiny saved tables, not a simulation. Exercise public exporters so these
# checks also cover sidecar field lookup and the atomic-write call boundary.
function aliases(path, directory, label)
    symbolic = joinpath(directory, label * "-symbolic")
    linked = joinpath(directory, label * "-hard")
    symlink(path, symbolic)
    hardlink(path, linked)
    return (path, symbolic, linked)
end

function preserves_inputs(writer, target, originals)
    @test_throws ArgumentError writer(target)
    for (path, bytes) in originals
        @test isfile(path)
        @test read(path) == bytes
    end
end

@testset "Single-run exporters preserve saved inputs and directories" begin
    mktempdir() do directory
        saved = saved_visualizer_fixture(joinpath(directory, "saved"))
        payload_path = export_viewer_payload(saved.prefix; textures=false)
        inputs = (saved.prefix * "_scene.json", saved.prefix * ".feather")
        originals = Dict(path => read(path) for path in (inputs..., payload_path))
        sentinel = joinpath(directory, "keep.txt")
        write(sentinel, "keep exactly")
        directory_alias = joinpath(directory, "saved-directory-alias")
        symlink(dirname(saved.prefix), directory_alias)
        writers = (
            payload = out -> export_viewer_payload(saved.prefix; out, textures=false),
            html = out -> export_visualization(saved.prefix; out, textures=false),
            development = out -> write_viewer_dev_payload(saved.prefix, out; textures=false),
        )
        targets = [aliases(path, directory, "input-$i") for (i,path) in enumerate(inputs)]
        for writer in values(writers)
            for target in (directory, directory_alias)
                preserves_inputs(writer, target, originals)
                @test isdir(target)
                @test read(sentinel, String) == "keep exactly"
            end
            for group in targets, target in group
                preserves_inputs(writer, target, originals)
            end
        end
        for target in aliases(payload_path, directory, "portable")
            preserves_inputs(writers.html, target, originals)
            preserves_inputs(writers.development, target, originals)
        end
        @test islink(directory_alias)
        @test all(islink(group[2]) && samefile(group[1], group[3]) for group in targets)

        # Explicit replacement of separate output files remains useful.
        json = joinpath(directory, "separate.json")
        html = joinpath(directory, "separate.html")
        javascript = joinpath(directory, "dev_data.js")
        for target in (json, html, javascript)
            write(target, "replace this output")
        end
        @test writers.payload(json) == json
        @test JSON.parsefile(json) == JSON.parsefile(payload_path)
        @test writers.html(html) == html
        @test embedded_visualizer_payload(html) == JSON.parsefile(payload_path)
        @test writers.development(javascript) == javascript
        @test startswith(read(javascript, String), "window.SPACEAGORA_VIEWER = ")
        @test read(sentinel, String) == "keep exactly"
    end
end

@testset "Ensemble HTML protects every sample and its manifest" begin
    mktempdir() do directory
        campaign = joinpath(directory, "campaign")
        fixtures = [saved_visualizer_fixture(joinpath(campaign, "sample_$(lpad(i,4,'0'))")) for i in 1:2]
        samples = [EnsembleSample(i, string(i), true, Float64(i), "sample $i", dirname(f.prefix)) for (i,f) in enumerate(fixtures)]
        manifest = write_ensemble_manifest(campaign, samples; scalar_name="synthetic fixture")
        inputs = [manifest]
        for fixture in fixtures
            push!(inputs, fixture.prefix * "_scene.json", fixture.prefix * ".feather",
                export_viewer_payload(fixture.prefix; textures=false))
        end
        originals = Dict(path => read(path) for path in inputs)
        sentinel = joinpath(campaign, "keep.txt"); write(sentinel, "keep exactly")
        directory_alias = joinpath(directory, "campaign-alias"); symlink(campaign, directory_alias)
        writer = out -> export_ensemble_visualization(campaign; out, textures=false, max_frames=3)
        for target in (campaign, directory_alias)
            preserves_inputs(writer, target, originals)
            @test isdir(target)
            @test read(sentinel, String) == "keep exactly"
        end
        for (i,path) in enumerate(inputs), target in aliases(path, directory, "ensemble-$i")
            preserves_inputs(writer, target, originals)
        end
        @test islink(directory_alias)
        output = joinpath(directory, "ensemble.html"); write(output, "replace this output")
        @test writer(output) == output
        @test embedded_visualizer_payload(output)["ensemble"]["count"] == 2
        @test read(sentinel, String) == "keep exactly"
    end
end

end # module ViewerOutputPreservationTests
