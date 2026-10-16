"""Build the separate environment in which a contributor repository is tested against this core.

The environment develops this checkout by path, so the tested core is exactly the
checked-out commit, and the repository's own `[sources]` pin is checked against it
instead of being installed. The reusable workflow `.github/workflows/contributor-ci.yml`
calls this script; it also works from a local clone of the pinned commit.
"""
module ContributorEnvironment

using Pkg, TOML
include(joinpath(@__DIR__, "optional_packages.jl"))
using .OptionalPackages: OptionalPackages, gitread, fail, same_origin

const ROOT = normpath(joinpath(@__DIR__, ".."))
const CORE_URL = "https://github.com/Space-FALCON-Lab/SpaceAGORA.jl.git"
const KINDS = ("project", "package", "companion")
const MANIFEST_MODES = ("core", "committed", "resolve")
const SCANNED_KINDS = ("project", "package")
const INTERNAL_MODULES = (:SimulationModel, :SimulationEngine)
const RECORD = "ContributorEnvironment.toml"
const HELP = """Usage: julia --startup-file=no scripts/contributor_environment.jl <project|package|companion> <repository> <environment> --manifest <core|committed|resolve> [--rev <sha>] [--companions <hypr,mujoco,edg>]
       julia --startup-file=no scripts/contributor_environment.jl scan <project|package|companion> <repository>
       julia --startup-file=no scripts/contributor_environment.jl summary <report-directory> [<environment>]
The environment directory must be new and outside both the repository and this checkout.
--rev defaults to this checkout's HEAD; the repository's [sources] must pin SpaceAGORA to it."""

full_sha(rev) = rev isa AbstractString && occursin(r"^[0-9a-f]{40}$", rev)
inside(path, root) = first(splitpath(relpath(path, root))) != ".."

function project_file(directory)
    for name in Base.project_names
        path = joinpath(directory, name)
        isfile(path) && return path
    end
    return nothing
end

function manifest_file(directory)
    versioned = joinpath(directory, "Manifest-v$(VERSION.major).$(VERSION.minor).toml")
    isfile(versioned) && return versioned
    plain = joinpath(directory, "Manifest.toml")
    return isfile(plain) ? plain : nothing
end

"""The core checkout at `rev`, with no tracked changes: the commit that is tested."""
function verified_core(root, rev)
    path = OptionalPackages.source_root(root)
    head = gitread(path, "rev-parse", "HEAD")
    rev = rev === nothing ? head : String(rev)
    full_sha(rev) || fail("The SpaceAGORA revision must be a full 40-character lowercase commit SHA, not '$rev'.")
    head == rev || fail("The SpaceAGORA checkout at $path is at $head, not the requested revision $rev.")
    # Submodules are not part of the tested core; optional_packages.jl verifies companions.
    changes = gitread(path, "status", "--porcelain=v1", "--untracked-files=no", "--ignore-submodules=all")
    isempty(changes) || fail("The SpaceAGORA checkout at $path has tracked changes, so it is not revision $rev:\n$changes")
    project = TOML.parsefile(joinpath(path, "Project.toml"))
    get(project, "name", "") == "SpaceAGORA" || fail("$path is not the SpaceAGORA package.")
    return (; path, rev, uuid=project["uuid"])
end

"""Refuse a `[sources]` entry for SpaceAGORA that does not name the tested commit."""
function core_pin(project, file, repository, core)
    sources = get(project, "sources", Dict{String,Any}())
    entry = get(sources, "SpaceAGORA", nothing)
    entry isa AbstractDict || fail("$file: [sources] must pin SpaceAGORA to the tested revision, e.g. " *
        "SpaceAGORA = {url = \"$CORE_URL\", rev = \"$(core.rev)\"}.")
    if haskey(entry, "path")
        length(entry) == 1 || fail("$file: the SpaceAGORA [sources] entry mixes path with $(join(sort!(collect(keys(entry))), ", ")).")
        target = normpath(joinpath(repository, entry["path"]))
        isdir(target) && samefile(target, core.path) ||
            fail("$file: [sources] SpaceAGORA path $(entry["path"]) is not the core under test ($(core.path)). " *
                 "Pin the commit with url and rev instead.")
        return "path $(entry["path"])"
    end
    url = get(entry, "url", nothing)
    rev = get(entry, "rev", nothing)
    url isa AbstractString && same_origin(url, CORE_URL) ||
        fail("$file: [sources] SpaceAGORA url must be $CORE_URL, not $(repr(url)).")
    rev isa AbstractString || fail("$file: [sources] SpaceAGORA has no rev; a branch is not a pin. Use rev = \"$(core.rev)\".")
    full_sha(rev) || fail("$file: [sources] SpaceAGORA rev \"$rev\" is not a full 40-character lowercase commit SHA.")
    rev == core.rev || fail("$file: [sources] pins SpaceAGORA $rev, but the tested revision is $(core.rev). " *
        "Use the same commit in [sources] and in the workflow call, so the recorded pair is the tested pair.")
    haskey(entry, "subdir") && fail("$file: [sources] SpaceAGORA must not set subdir.")
    return "url + rev"
end

"""Name and UUID of the repository's own package, or `nothing` for a project without a module."""
function package_identity(kind, project, file, repository)
    name, uuid = get(project, "name", nothing), get(project, "uuid", nothing)
    has_module = name isa AbstractString && isfile(joinpath(repository, "src", name * ".jl"))
    if kind != "project"
        name isa AbstractString && uuid isa AbstractString || fail("$file: a $kind needs a name and a uuid.")
        has_module || fail("$(joinpath(repository, "src", string(name, ".jl"))): the $kind's module file is missing.")
    end
    name == "SpaceAGORA" && fail("$file: this is the SpaceAGORA package itself, not a contributor repository.")
    return has_module && uuid isa AbstractString ? (; name=String(name), uuid=String(uuid)) : nothing
end

"""Dependencies that `test/runtests.jl` imports: `[targets] test`, or a `test/Project.toml`."""
function test_dependencies(project, file, repository)
    deps = Dict{String,Any}()
    for name in get(get(project, "targets", Dict{String,Any}()), "test", String[])
        uuid = get(get(project, "extras", Dict{String,Any}()), name,
                   get(get(project, "weakdeps", Dict{String,Any}()), name, nothing))
        uuid === nothing && fail("$file: test target $name is not in [extras] or [weakdeps].")
        deps[name] = uuid
    end
    test_project = project_file(joinpath(repository, "test"))
    if test_project !== nothing
        merge!(deps, get(TOML.parsefile(test_project), "deps", Dict{String,Any}()))
        delete!(deps, get(project, "name", ""))
    end
    return deps
end

companion_names(list::AbstractVector) = companion_names(join(list, ' '))
function companion_names(list::AbstractString)
    names = unique(filter!(!isempty, split(list, r"[\s,]+")))
    for name in names
        name in OptionalPackages.NAMES ||
            fail("Unknown companion '$name'. Choose from $(join(OptionalPackages.NAMES, ", ")).")
    end
    return String.(names)
end

"""Package names that each companion installs: its registry name, and SpaceAGORAHYPR for HYPR."""
function companion_package_names(name; root)
    registry = TOML.parsefile(joinpath(root, "packages", "OptionalPackages.toml"))
    return name == "hypr" ? [registry[name]["name"], "SpaceAGORAHYPR"] : [registry[name]["name"]]
end

"""A dependency on a companion needs the companion selected, at the core's recorded pin."""
function check_companion_dependencies(project, file, companions; root)
    used = merge(get(project, "deps", Dict{String,Any}()), get(project, "extras", Dict{String,Any}()))
    sources = get(project, "sources", Dict{String,Any}())
    for name in OptionalPackages.NAMES, dependency in companion_package_names(name; root)
        haskey(used, dependency) || continue
        name in companions || fail("$file: $dependency is a dependency; select the $name companion.")
        rev = get(get(sources, dependency, Dict{String,Any}()), "rev", nothing)
        rev === nothing && continue
        pinned = OptionalPackages.spec(name; root).revision
        rev == pinned || fail("$file: [sources] pins $dependency $rev, but this core pins $pinned.")
    end
end

"""Check every input and decide the environment's contents; nothing is written or fetched."""
function plan(kind, repository, environment; manifest="core", rev=nothing, companions="", root=ROOT)
    kind in KINDS || fail("Unknown kind '$kind'. Choose from $(join(KINDS, ", ")).")
    manifest in MANIFEST_MODES || fail("Unknown manifest mode '$manifest'. Choose from $(join(MANIFEST_MODES, ", ")).")
    core = verified_core(root, rev)
    isdir(repository) || fail("Repository directory not found: $repository")
    repository = realpath(repository)
    file = project_file(repository)
    file === nothing && fail("$repository has no Project.toml.")
    project = TOML.parsefile(file)
    package = package_identity(kind, project, file, repository)
    pin = core_pin(project, file, repository, core)
    names = companion_names(companions)
    check_companion_dependencies(project, file, names; root=core.path)
    destination = OptionalPackages.canonical_destination(environment)
    inside(destination, core.path) && fail("Choose an environment directory outside the SpaceAGORA checkout.")
    inside(destination, repository) && fail("Choose an environment directory outside the repository.")
    isdir(destination) && !isempty(readdir(destination)) && fail("Choose a new or empty environment directory: $destination")
    seed = if manifest == "core"
        joinpath(core.path, "Manifest.toml")
    elseif manifest == "committed"
        committed = manifest_file(repository)
        committed === nothing && fail("$repository has no committed Manifest.toml for manifest mode 'committed'.")
        committed
    else
        nothing
    end
    get(get(project, "deps", Dict{String,Any}()), "SpaceAGORA", nothing) == core.uuid ||
        fail("$file: [deps] must list SpaceAGORA = \"$(core.uuid)\".")
    deps = merge(get(project, "deps", Dict{String,Any}()), test_dependencies(project, file, repository))
    sources = Dict{String,Any}("SpaceAGORA" => Dict("path" => core.path))
    developed = [core.path]
    if package !== nothing
        deps[package.name] = package.uuid
        sources[package.name] = Dict("path" => repository)
        push!(developed, repository)
    end
    compat = Dict{String,Any}()
    if kind == "project"
        # The project is the environment's template: its other sources and its bounds apply.
        for (name, entry) in get(project, "sources", Dict{String,Any}())
            haskey(sources, name) && continue
            sources[name] = haskey(entry, "path") ?
                merge(entry, Dict("path" => normpath(joinpath(repository, entry["path"])))) : entry
        end
        for (name, bound) in get(project, "compat", Dict{String,Any}())
            (name == "julia" || haskey(deps, name)) && (compat[name] = bound)
        end
    end
    return (; kind, repository, file, package, pin, core, destination, manifest, seed, companions=names,
              project=Dict{String,Any}("deps" => deps, "sources" => sources, "compat" => compat), developed)
end

"""Fetch the selected companions at the core's gitlinks and add them to the plan."""
function add_companions(p)
    linked = Dict{String,Any}()
    for name in p.companions
        package = OptionalPackages.spec(name; root=p.core.path)
        paths = [OptionalPackages.checkout(name; root=p.core.path)]
        name == "hypr" && push!(paths, joinpath(p.core.path, "packages", "SpaceAGORAHYPR"))
        for path in paths
            identity = TOML.parsefile(joinpath(path, "Project.toml"))
            p.project["deps"][identity["name"]] = identity["uuid"]
            p.project["sources"][identity["name"]] = Dict("path" => path)
            push!(p.developed, path)
        end
        linked[name] = Dict("package" => package.name, "revision" => package.revision, "path" => first(paths))
    end
    return linked
end

function write_toml(path, data)
    open(path, "w") do io
        TOML.print(io, data; sorted=true)
    end
end

"""Write, seed and instantiate the environment, then record the tested pair."""
function build(p)
    linked = add_companions(p)
    mkpath(p.destination)
    write_toml(joinpath(p.destination, "Project.toml"), p.project)
    p.seed === nothing || cp(p.seed, joinpath(p.destination, "Manifest.toml"))
    preserve = p.seed === nothing ? Pkg.PRESERVE_NONE : Pkg.PRESERVE_ALL
    previous = Base.active_project()
    try
        withenv("JULIA_PKG_PRECOMPILE_AUTO" => "0") do
            Pkg.activate(p.destination)
            try
                Pkg.develop([PackageSpec(path=path) for path in p.developed]; preserve)
            catch error
                error isa Pkg.Resolve.ResolverError && p.seed !== nothing &&
                    @error "The repository needs versions that the $(p.manifest) manifest does not pin. Manifest mode 'resolve' shows the newest compatible set."
                rethrow()
            end
            Pkg.instantiate()
        end
    finally
        previous === nothing ? Pkg.activate() : Pkg.activate(dirname(previous))
    end
    write_toml(joinpath(p.destination, RECORD), Dict(
        "kind" => p.kind, "repository" => p.repository, "spaceagora_rev" => p.core.rev,
        "spaceagora_pin" => p.pin, "core" => p.core.path, "manifest" => p.manifest,
        "package" => p.package === nothing ? "" : p.package.name, "companions" => linked))
    return p.destination
end

setup(kind, repository, environment; kw...) = build(plan(kind, repository, environment; kw...))

# --- internal-name scan --------------------------------------------------------------------------

internal_name(name::Symbol) = name in INTERNAL_MODULES || startswith(String(name), "_")

"""`a.b.c` as `[:a, :b, :c]`, from a property access or an import path; `nothing` otherwise."""
function dotted(ex)
    ex isa Symbol && return Symbol[ex]
    ex isa Expr && ex.head === :. || return nothing
    if length(ex.args) == 2 && ex.args[2] isa QuoteNode && ex.args[2].value isa Symbol
        left = dotted(ex.args[1])
        return left === nothing ? nothing : push!(left, ex.args[2].value)
    end
    return all(arg -> arg isa Symbol, ex.args) ? collect(Symbol, ex.args) : nothing
end

"""The first internal segment of a path rooted at SpaceAGORA (or an alias of it)."""
function internal_path(chain, roots)
    (chain === nothing || length(chain) < 2 || !(first(chain) in roots)) && return nothing
    index = findfirst(internal_name, @view chain[2:end])
    return index === nothing ? nothing : join(chain[1:index+1], ".")
end

# Names bound to the SpaceAGORA module: `import SpaceAGORA as S` and `const S = SpaceAGORA`.
function aliases!(roots, ex)
    ex isa Expr || return roots
    if ex.head === :as && dotted(ex.args[1]) == [:SpaceAGORA] && ex.args[2] isa Symbol
        push!(roots, ex.args[2])
    elseif ex.head === :(=) && length(ex.args) == 2 && ex.args[1] isa Symbol && ex.args[2] === :SpaceAGORA
        push!(roots, ex.args[1])
    end
    foreach(arg -> aliases!(roots, arg), ex.args)
    return roots
end

internal!(found, line, name) = name === nothing || push!(found, (line, "$name is internal to SpaceAGORA"))

function scan_expression!(found, ex, line, roots)
    ex isa LineNumberNode && return ex.line
    ex isa Expr || return line
    if ex.head in (:error, :incomplete)
        push!(found, (line, "the file does not parse"))
        return line
    elseif ex.head === :.
        name = internal_path(dotted(ex), roots)
        if name !== nothing
            internal!(found, line, name)
            return line
        end
    elseif ex.head in (:using, :import)
        for arg in ex.args
            arg = arg isa Expr && arg.head === :as ? arg.args[1] : arg
            if arg isa Expr && arg.head === :(:)
                base = dotted(arg.args[1])
                for item in arg.args[2:end]
                    item = item isa Expr && item.head === :as ? item.args[1] : item
                    chain = dotted(item)
                    (base === nothing || chain === nothing) && continue
                    internal!(found, line, internal_path(vcat(base, chain), roots))
                end
            else
                internal!(found, line, internal_path(dotted(arg), roots))
            end
        end
        return line
    elseif ex.head === :call && length(ex.args) == 3 && ex.args[1] in (:getproperty, :getfield) &&
            ex.args[3] isa QuoteNode && ex.args[3].value isa Symbol
        chain = dotted(ex.args[2])
        chain === nothing || internal!(found, line, internal_path(push!(chain, ex.args[3].value), roots))
    end
    for arg in ex.args
        line = scan_expression!(found, arg, line, roots)
    end
    return line
end

"""References to `SpaceAGORA.SimulationModel`, `SpaceAGORA.SimulationEngine` and underscore names."""
function scan_source(text::AbstractString, filename::AbstractString)
    ex = Meta.parseall(text; filename)
    found = Tuple{Int,String}[]
    scan_expression!(found, ex, 0, aliases!(Set([:SpaceAGORA]), ex))
    return found
end

function scan(kind, repository)
    kind in KINDS || fail("Unknown kind '$kind'. Choose from $(join(KINDS, ", ")).")
    kind in SCANNED_KINDS || return Tuple{String,Int,String}[]
    repository = realpath(repository)
    findings = Tuple{String,Int,String}[]
    for (directory, folders, files) in walkdir(repository)
        filter!(folder -> !startswith(folder, "."), folders)
        for file in sort!(filter(endswith(".jl"), files))
            path = joinpath(directory, file)
            for (line, message) in scan_source(read(path, String), path)
                push!(findings, (relpath(path, repository), line, message))
            end
        end
    end
    return findings
end

# --- CI summary ----------------------------------------------------------------------------------

cell(value) = replace(string(value), "|" => "\\|", "\n" => " ")

"""Markdown with one line per check id from the reports in `directory`."""
function summary(directory, environment=nothing)
    io = IOBuffer()
    println(io, "## Contributor conformance\n")
    println(io, "Interface conformance only; scientific validation is the author's.\n")
    record = environment === nothing ? nothing : joinpath(environment, RECORD)
    if record !== nothing && isfile(record)
        r = TOML.parsefile(record)
        package = isempty(r["package"]) ? "" : " (`$(r["package"])`)"
        println(io, "- Kind: $(r["kind"])$package; manifest: $(r["manifest"])")
        println(io, "- SpaceAGORA: `$(r["spaceagora_rev"])`, pinned in [sources] by $(r["spaceagora_pin"])")
        for (name, companion) in sort!(collect(get(r, "companions", Dict{String,Any}())); by=first)
            println(io, "- Companion $name: $(companion["package"]) `$(companion["revision"])`")
        end
        println(io)
    end
    files = isdir(directory) ? sort!(filter(endswith(".toml"), readdir(directory; join=true))) : String[]
    if isempty(files)
        println(io, "No conformance reports were written: the tests did not call the conformance checks.")
        return String(take!(io))
    end
    counts = Dict{String,Int}()
    println(io, "| Target | Check | Status | Since | Detail |")
    println(io, "|---|---|---|---|---|")
    for file in files
        report = TOML.parsefile(file)
        target = string(get(report, "target", basename(file)), " (", get(report, "kind", "?"), ")")
        for check in get(report, "checks", Any[])
            status = string(get(check, "status", "?"))
            counts[status] = get(counts, status, 0) + 1
            println(io, "| ", cell(target), " | ", cell(get(check, "id", "?")), " | ", cell(status), " | ",
                    cell(get(check, "since", "")), " | ", cell(get(check, "detail", "")), " |")
        end
    end
    println(io, "\n", join(("$(n) $(status)" for (status, n) in sort!(collect(counts); by=first)), ", "))
    return String(take!(io))
end

# --- command line --------------------------------------------------------------------------------

function options(args)
    positional, named = String[], Dict{String,String}()
    i = 1
    while i <= length(args)
        arg = args[i]
        if startswith(arg, "--")
            key, value = occursin('=', arg) ? split(arg[3:end], '='; limit=2) :
                (arg[3:end], i < length(args) ? args[i += 1] : fail("Option $arg needs a value.\n$HELP"))
            key in ("manifest", "rev", "companions") || fail("Unknown option --$key.\n$HELP")
            named[key] = value
        else
            push!(positional, arg)
        end
        i += 1
    end
    return positional, named
end

function main(args=ARGS)
    positional, named = options(args)
    if length(positional) == 3 && positional[1] == "scan" && isempty(named)
        findings = scan(positional[2], positional[3])
        for (file, line, message) in findings
            println(stderr, "$file:$line: $message")
        end
        isempty(findings) || exit(1)
        println(positional[2] in SCANNED_KINDS ?
            "Internal-name scan: no SimulationModel, SimulationEngine or underscore names in $(positional[3])" :
            "Internal-name scan: a $(positional[2]) is not scanned; it uses its facade's bindings")
    elseif length(positional) in (2, 3) && positional[1] == "summary" && isempty(named)
        print(summary(positional[2], get(positional, 3, nothing)))
    elseif length(positional) == 3 && haskey(named, "manifest")
        rev = get(named, "rev", nothing)
        environment = setup(positional...; manifest=named["manifest"], rev,
                            companions=get(named, "companions", ""))
        println("Environment: ", environment)
    else
        fail(HELP)
    end
end

end

abspath(PROGRAM_FILE) == abspath(@__FILE__) && ContributorEnvironment.main()
