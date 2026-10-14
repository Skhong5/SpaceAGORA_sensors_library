# Every repository-relative file path named in maintained Markdown must exist.
# Scope: tracked *.md files except review records (test/ai_reviews/), the dated
# session records kept loose under docs/, generated docs pages and the topology
# contract pages that describe retired paths on purpose. A named path that is
# gitignored, lives inside a submodule or sits under an output/ directory is
# expected to be absent from a clean checkout and is not reported.
const REPO_ROOT = normpath(joinpath(@__DIR__, "..", ".."))

const PATH_ROOTS = ("src", "test", "docs", "scripts", "benchmarks", "examples", "data",
                    "ext", "packages", "templates", "bin", "build", ".github")
const PATH_EXTS = ("jl", "py", "sh", "md", "toml", "csv", "yml", "yaml", "json", "tex", "txt",
                   "svg", "png", "pdf", "jls", "tle", "feather", "arrow", "cff", "html", "js")
const PATH_RE = Regex("(?<![A-Za-z0-9_./-])((?:" * join(PATH_ROOTS, "|") *
                      ")/[A-Za-z0-9_./-]*[A-Za-z0-9_]\\.(?:" * join(PATH_EXTS, "|") * "))(?![A-Za-z0-9_])")

# Pages that name retired or historical paths deliberately.
const EXEMPT_FILES = Set([
    joinpath("docs", "architecture", "canonical_topology_contract.md"),
    joinpath("docs", "architecture", "src_completeness_contract.md"),
    joinpath("docs", "architecture", "src_canonical_owner_audit.md"),
])

_git(args...) = readchomp(setenv(Cmd(["git", args...]); dir=REPO_ROOT))

function _scanned_files()
    files = String[]
    for rel in split(_git("ls-files", "--", "*.md"), '\n'; keepempty=false)
        startswith(rel, joinpath("test", "ai_reviews") * "/") && continue
        startswith(rel, joinpath("docs", "src", "generated") * "/") && continue
        rel in EXEMPT_FILES && continue
        # Loose dated records directly under docs/ are history, not maintained pages.
        dirname(rel) == "docs" && continue
        push!(files, rel)
    end
    return files
end

function _submodule_paths()
    paths = String[]
    for line in split(_git("ls-files", "--stage"), '\n'; keepempty=false)
        startswith(line, "160000") || continue
        push!(paths, split(line, '\t'; limit=2)[2])
    end
    return paths
end

_ignored(rel) = success(setenv(`git check-ignore -q -- $rel`; dir=REPO_ROOT))

const SUBMODULES = _submodule_paths()
_in_submodule(rel) = any(startswith(rel, sub * "/") for sub in SUBMODULES)

function _expected_absent(rel)
    occursin("/output/", "/" * rel) && return true
    _in_submodule(rel) && return true
    return _ignored(rel)
end

violations = String[]
for rel in _scanned_files()
    text = read(joinpath(REPO_ROOT, rel), String)
    seen = Set{String}()
    for m in eachmatch(PATH_RE, text)
        target = normpath(m.captures[1])
        target in seen && continue
        push!(seen, target)
        ispath(joinpath(REPO_ROOT, target)) && continue
        _expected_absent(target) && continue
        push!(violations, "$rel: names missing path $target")
    end
end

if !isempty(violations)
    println("markdown_path_reference_violations:")
    for v in sort(violations)
        println("  - $v")
    end
    error("Markdown path reference gate failed")
end

println("markdown_path_reference_gate_ok")
