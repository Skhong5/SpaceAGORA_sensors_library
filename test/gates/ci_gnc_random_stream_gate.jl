const REPO_ROOT = normpath(joinpath(@__DIR__, "..", ".."))
# Static check of the stream argument at direct random call sites, not proof of the
# stream's type or provenance. Draw calls are parsed, and their first argument
# must be an identifier or field whose name contains `rng` (case-insensitive).
# This naming convention cannot follow aliases, macro expansion or runtime
# dispatch; behavioral tests remain responsible for stream ownership. A hidden
# global stream makes results depend on unrelated draws and thread scheduling,
# so explicit global-stream spellings are also forbidden under these roots.
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
# The in-place forms need a separate stream and destination: a one-argument
# `rand!(rng_buffer)` still draws from the global stream. Keywords and splats
# cannot stand in for either of these two explicit positional arguments.
const GNC_RANDOM_DRAW_NAMES = Tuple(name * suffix for name in
    ("rand", "randn", "randexp", "randperm", "randcycle", "randsubseq", "shuffle")
    for suffix in ("", "!"))
const GNC_INPLACE_RANDOM_DRAWS = ("rand!", "randn!", "randexp!", "shuffle!")
const _GNC_RANDOM_SYNTAX = Base.JuliaSyntax

function _gnc_random_name(node)
    kind = _GNC_RANDOM_SYNTAX.kind(node)
    kind == _GNC_RANDOM_SYNTAX.K"Identifier" && return String(_GNC_RANDOM_SYNTAX.sourcetext(node))
    if kind == _GNC_RANDOM_SYNTAX.K"."
        return _gnc_random_name(last(_GNC_RANDOM_SYNTAX.children(node)))
    end
    return nothing
end

function _gnc_random_draws!(violations, node, rel)
    kids = _GNC_RANDOM_SYNTAX.children(node)
    kids === nothing && return
    if _GNC_RANDOM_SYNTAX.kind(node) == _GNC_RANDOM_SYNTAX.K"call"
        name = _gnc_random_name(first(kids))
        if name in GNC_RANDOM_DRAW_NAMES
            args = filter(kids[2:end]) do arg
                !(_GNC_RANDOM_SYNTAX.kind(arg) in
                    (_GNC_RANDOM_SYNTAX.K"parameters", _GNC_RANDOM_SYNTAX.K"="))
            end
            stream_name = isempty(args) ? nothing : _gnc_random_name(first(args))
            explicit_stream = stream_name !== nothing && occursin(r"rng"i, stream_name)
            separate_destination = !(name in GNC_INPLACE_RANDOM_DRAWS) ||
                (length(args) >= 2 && _GNC_RANDOM_SYNTAX.kind(args[2]) != _GNC_RANDOM_SYNTAX.K"...")
            if !explicit_stream || !separate_destination
                lineno = _GNC_RANDOM_SYNTAX.source_line(node.source, _GNC_RANDOM_SYNTAX.first_byte(node))
                call = replace(String(_GNC_RANDOM_SYNTAX.sourcetext(node)), '\n' => ' ')
                push!(violations, "$rel:$lineno: draw without an explicit stream as its first argument: $call")
            end
        end
    end
    for kid in kids
        _gnc_random_draws!(violations, kid, rel)
    end
end

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
        tree = _GNC_RANDOM_SYNTAX.parseall(_GNC_RANDOM_SYNTAX.SyntaxNode, read(path, String);
            filename=path, ignore_errors=false, ignore_warnings=true)
        _gnc_random_draws!(violations, tree, rel)
        for (lineno, line) in enumerate(_gnc_random_active_lines(path))
            isempty(line) && continue
            for pattern in GNC_HIDDEN_STREAM_PATTERNS
                occursin(pattern, line) && push!(violations, "$rel:$lineno: hidden global stream: $line")
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
