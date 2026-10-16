const REPO_ROOT = normpath(joinpath(@__DIR__, "..", ".."))
# Every random draw in GNC code comes from a stream its caller supplies. A hidden
# global stream (`default_rng()`, `seed!`, an unseeded `rand()`) would make planner
# and controller results depend on unrelated code that drew before them, and on
# thread scheduling, so it is not allowed under these roots.
const GNC_RANDOM_STREAM_ROOTS = (
    joinpath(REPO_ROOT, "src", "models", "gnc"),
    joinpath(REPO_ROOT, "packages", "SpaceAGORAHYPR", "src"),
)
const GNC_HIDDEN_STREAM_PATTERNS = (
    r"\bdefault_rng\s*\(",
    r"\bseed!\s*\(",
    r"\bGLOBAL_RNG\b",
    r"\bTaskLocalRNG\s*\(",
)
# A draw names its stream as the first argument (`rand(rng)`, `randn(master_rng, n)`),
# including the in-place forms (`rand!(rng, buffer)`), which otherwise fill the
# buffer from the global stream.
const GNC_RANDOM_DRAW = r"\b(rand|randn|randexp|randperm|randcycle|randsubseq|shuffle)!?\(\s*([^,()\s]*)"

# Source text without docstrings and comments, keeping line numbers.
function _gnc_random_active_lines(path::String)
    src = read(path, String)
    src = replace(src, r"\"\"\".*?\"\"\""s => m -> "\n"^count(==('\n'), m))
    return [strip(first(split(line, '#', limit=2))) for line in split(src, '\n')]
end

violations = String[]
for root in GNC_RANDOM_STREAM_ROOTS
    isdir(root) || error("Missing GNC random-stream scan root: $(relpath(root, REPO_ROOT))")
end
for scan_root in GNC_RANDOM_STREAM_ROOTS, (root, _, files) in walkdir(scan_root)
    for file in files
        endswith(file, ".jl") || continue
        path = joinpath(root, file)
        rel = relpath(path, REPO_ROOT)
        for (lineno, line) in enumerate(_gnc_random_active_lines(path))
            isempty(line) && continue
            for pattern in GNC_HIDDEN_STREAM_PATTERNS
                occursin(pattern, line) && push!(violations, "$rel:$lineno: hidden global stream: $line")
            end
            for m in eachmatch(GNC_RANDOM_DRAW, line)
                occursin(r"rng"i, m.captures[2]) ||
                    push!(violations, "$rel:$lineno: draw without an explicit stream as its first argument: $line")
            end
        end
    end
end

if !isempty(violations)
    println("gnc_random_stream_violations:")
    for v in sort(unique(violations))
        println("  - $v")
    end
    error("GNC random-stream gate failed: take an explicit rng argument and pass it to every draw")
end

println("gnc_random_stream_gate_ok")
