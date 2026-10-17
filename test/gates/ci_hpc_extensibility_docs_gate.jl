const REPO_ROOT = dirname(dirname(dirname(@__FILE__)))

required_paths = [
    joinpath(REPO_ROOT, "docs", "src", "public_api_policy.md"),
    joinpath(REPO_ROOT, "docs", "src", "distributed_hpc.md"),
    joinpath(REPO_ROOT, "docs", "src", "extensibility.md"),
    joinpath(REPO_ROOT, "templates", "force_torque_model_template.jl"),
    joinpath(REPO_ROOT, "templates", "density_model_template.jl"),
    joinpath(REPO_ROOT, "templates", "control_hook_template.jl"),
    joinpath(REPO_ROOT, "templates", "actuator_template.jl"),
    joinpath(REPO_ROOT, "templates", "gnc_planner_template.jl"),
    joinpath(REPO_ROOT, "docs", "src", "user", "gnc_algorithm.md"),
    [joinpath(REPO_ROOT, "examples", "gnc_algorithm_env", name) for name in
        ("README.md", "Project.template.toml", "setup.jl", "smoke.jl")]...,
    joinpath(REPO_ROOT, "templates", "starters", "README.md"),
    [joinpath(REPO_ROOT, "templates", "starters", kind, rel) for kind in ("project", "package", "companion")
        for rel in ("README.md", "Project.toml", "SpaceAGORASource.toml", joinpath("src", "StarterName.jl"),
            joinpath("test", "runtests.jl"), joinpath(".github", "workflows", "ci.yml"))]...,
    joinpath(REPO_ROOT, "templates", "starters", "companion", "test", "incompatible_core.jl"),
    joinpath(REPO_ROOT, "templates", "starters", "companion", "test", "load_order.jl"),
    joinpath(REPO_ROOT, "scripts", "new_contributor_repository.jl"),
    joinpath(REPO_ROOT, "scripts", "hpc", "local_process_runtime_analysis.sh"),
    joinpath(REPO_ROOT, "scripts", "hpc", "slurm_process_runtime_analysis.sh"),
]

missing = filter(path -> !isfile(path), required_paths)
isempty(missing) || error("Missing Wave 3/Wave 5 documentation assets: $(missing)")

hpc_doc = read(joinpath(REPO_ROOT, "docs", "src", "distributed_hpc.md"), String)
ext_doc = read(joinpath(REPO_ROOT, "docs", "src", "extensibility.md"), String)
api_doc = read(joinpath(REPO_ROOT, "docs", "src", "public_api_policy.md"), String)

for marker in (
    "SPACEAGORA_PERF_PROCS",
    "SPACEAGORA_PERF_WORKER_PROJECT",
    "SPACEAGORA_PARALLEL_POLICY_STATE_PATH",
    "shared depot",
    "scripts/hpc/slurm_process_runtime_analysis.sh",
)
    occursin(marker, hpc_doc) || error("distributed_hpc.md missing marker: $(marker)")
end

for marker in (
    "wrench",
    "getDensity",
    "update_control!",
    "control_wrench",
    "reference/extension_conventions.md",
    "templates/force_torque_model_template.jl",
    "templates/control_hook_template.jl",
    "templates/gnc_planner_template.jl",
    "user/gnc_algorithm.md",
)
    occursin(marker, ext_doc) || error("extensibility.md missing marker: $(marker)")
end

for marker in (
    "root `SpaceAGORA` module",
    "Public API",
    "internal",
    "generated public API",
)
    occursin(marker, api_doc) || error("public_api_policy.md missing marker: $(marker)")
end

# The worked examples of "Writing a GNC Algorithm" run in CI from a separate
# project. Each region of its scripts, between a `# snippet: <name>` line and
# `# end snippet`, must appear on the page as one Julia code block, character for
# character, so the page shows the code CI runs; and the runner CI calls must run
# every script.
function snippet_regions(path)
    regions = Pair{String, String}[]
    name, lines = nothing, String[]
    for (number, line) in enumerate(eachline(path))
        start = match(r"^# snippet: (\S+)$", line)
        if start !== nothing
            name === nothing || error("$(path):$(number): snippet $(start[1]) starts inside snippet $(name)")
            name, lines = String(start[1]), String[]
        elseif line == "# end snippet"
            name === nothing && error("$(path):$(number): `# end snippet` without a start")
            push!(regions, name => join(lines, "\n"))
            name = nothing
        elseif name !== nothing
            push!(lines, line)
        end
    end
    name === nothing || error("$(path): snippet $(name) has no `# end snippet`")
    return regions
end

function julia_code_blocks(path)
    blocks, lines, inside = String[], String[], false
    for line in eachline(path)
        if !inside && line == "```julia"
            inside, lines = true, String[]
        elseif inside && line == "```"
            push!(blocks, join(lines, "\n"))
            inside = false
        elseif inside
            push!(lines, line)
        end
    end
    return blocks
end

gnc_examples = joinpath(REPO_ROOT, "examples", "gnc_algorithm_env")
gnc_page_blocks = Set(julia_code_blocks(joinpath(REPO_ROOT, "docs", "src", "user", "gnc_algorithm.md")))
gnc_runner = read(joinpath(gnc_examples, "smoke.jl"), String)
gnc_scripts = sort!(filter(name -> endswith(name, ".jl") && !(name in ("setup.jl", "smoke.jl")), readdir(gnc_examples)))
isempty(gnc_scripts) && error("examples/gnc_algorithm_env/ has no example script")
for script in gnc_scripts
    occursin("\"$(script)\"", gnc_runner) ||
        error("examples/gnc_algorithm_env/smoke.jl does not run $(script)")
    regions = snippet_regions(joinpath(gnc_examples, script))
    isempty(regions) && error("examples/gnc_algorithm_env/$(script) has no snippet shown on the GNC page")
    for (name, text) in regions
        text in gnc_page_blocks || error("docs/src/user/gnc_algorithm.md has no Julia code block equal to " *
            "snippet `$(name)` of examples/gnc_algorithm_env/$(script); change the page and the script together.")
    end
end

println("hpc_extensibility_docs_gate_ok")
