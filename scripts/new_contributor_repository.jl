"""Create a contributor repository from a starter in `templates/starters/`.

    julia --startup-file=no scripts/new_contributor_repository.jl <project|package|companion> <directory> --name <Name>

- `project`: a research study on the public API (committed Manifest, scripts, results kept out).
- `package`: an algorithm package for an existing family (an RPO reference planner).
- `companion`: a companion that serves a core service facade (the external-propagation boundary).

The new repository pins this checkout's commit: its `SpaceAGORASource.toml` and its
workflow call name `HEAD`, its `[compat]` admits this core's release series, and a
companion requires the facade's current contract version. It gets a fresh UUID; a
project also gets this checkout's Manifest as its committed Manifest. The command
refuses a checkout with tracked changes (the pinned commit would not be the code the
starter was made from), an existing non-empty directory, a directory inside this
checkout, and a name that is not a package name.
"""
module NewContributorRepository

using TOML, UUIDs

const ROOT = normpath(joinpath(@__DIR__, ".."))
const KINDS = ("project", "package", "companion")
const STARTERS = joinpath("templates", "starters")
# The placeholders of the starter templates. Each keeps its file valid Julia, TOML
# or YAML, so the templates parse and the repository's gates can scan them.
const NAME_TOKEN = "StarterName"
const UUID_TOKEN = "00000000-0000-0000-0000-000000000000"
const REV_TOKEN = "0"^40
const COMPAT_TOKEN = "SpaceAGORA = \"0.0\""
const PIN_VERSION_TOKEN = "version = \"0.0.0\""
const CONTRACT_TOKEN = "const CORE_CONTRACT_VERSION = v\"0.0.0\""
const FACADE_SOURCE = joinpath("src", "orchestration", "simulation", "external_propagation_services.jl")
const HELP = "Usage: julia --startup-file=no scripts/new_contributor_repository.jl <$(join(KINDS, '|'))> <directory> --name <Name>"

fail(message) = throw(ArgumentError(message))

function git(root, args...)
    command = `git -c maintenance.auto=false -c gc.auto=0 -C $root $args`
    return readchomp(command)
end

"""The checkout's commit, version and facade contract version; refuses tracked changes unless `require_clean=false`."""
function core_identity(root=ROOT; require_clean::Bool=true)
    rev = git(root, "rev-parse", "HEAD")
    occursin(r"^[0-9a-f]{40}$", rev) || fail("$root: HEAD is not a commit ($rev).")
    if require_clean
        changes = git(root, "status", "--porcelain=v1", "--untracked-files=no", "--ignore-submodules=all")
        isempty(changes) || fail("$root has tracked changes, so its HEAD $rev is not the code a starter would be " *
            "made from. Commit or set them aside first:\n$changes")
    end
    project = TOML.parsefile(joinpath(root, "Project.toml"))
    get(project, "name", "") == "SpaceAGORA" || fail("$root is not the SpaceAGORA package.")
    version = VersionNumber(project["version"])
    source = read(joinpath(root, FACADE_SOURCE), String)
    contract = match(r"const CONTRACT_VERSION = v\"([^\"]+)\"", source)
    contract === nothing && fail("$(FACADE_SOURCE): no CONTRACT_VERSION found.")
    return (; root, rev, uuid=project["uuid"], version, contract=VersionNumber(contract.captures[1]))
end

"""The release series a `[compat]` entry admits: `0.minor` before 1.0, the major version after it."""
compat_series(version::VersionNumber) = version.major == 0 ? "0.$(version.minor)" : "$(version.major)"

"""Refuse a name that is not a package name, or one the starters reserve."""
function check_name(name::AbstractString)
    occursin(r"^[A-Z][A-Za-z0-9]*$", name) ||
        fail("--name must be a package name: a capital letter, then letters and digits (got \"$name\").")
    name in ("SpaceAGORA", NAME_TOKEN) && fail("--name $name is reserved; choose the repository's own name.")
    endswith(name, "jl") && fail("--name is the package name without .jl (got \"$name\").")
    return String(name)
end

inside(path, root) = first(splitpath(relpath(path, root))) != ".."

function check_target(directory, root)
    target = abspath(directory)
    inside(target, realpath(root)) && fail("$target is inside the SpaceAGORA checkout; create the repository elsewhere.")
    isdir(target) && !isempty(readdir(target)) && fail("$target exists and is not empty.")
    ispath(target) && !isdir(target) && fail("$target exists and is not a directory.")
    return target
end

"""The text of a starter file with every placeholder filled in."""
function fill_in(text::AbstractString, values)
    for (token, value) in values
        text = replace(text, token => value)
    end
    return text
end

"""
    generate(kind, directory; name, root=ROOT, require_clean=true) -> directory

Write the `kind` starter, filled in for `name` and this checkout, into `directory`.
"""
function generate(kind::AbstractString, directory::AbstractString; name::AbstractString, root=ROOT,
        require_clean::Bool=true)
    kind in KINDS || fail("Unknown kind \"$kind\". Choose from $(join(KINDS, ", ")).")
    name = check_name(name)
    core = core_identity(root; require_clean)
    target = check_target(directory, root)
    template = joinpath(root, STARTERS, kind)
    isdir(template) || fail("$template is missing.")
    values = (
        NAME_TOKEN => name,
        UUID_TOKEN => string(uuid4()),
        REV_TOKEN => core.rev,
        COMPAT_TOKEN => "SpaceAGORA = \"$(compat_series(core.version))\"",
        PIN_VERSION_TOKEN => "version = \"$(core.version)\"",
        CONTRACT_TOKEN => "const CORE_CONTRACT_VERSION = v\"$(core.contract)\"",
    )
    mkpath(target)
    for (folder, _, files) in walkdir(template), file in files
        source = joinpath(folder, file)
        destination = joinpath(target, fill_in(relpath(source, template), (NAME_TOKEN => name,)))
        mkpath(dirname(destination))
        write(destination, fill_in(read(source, String), values))
    end
    # A study commits the exact versions behind its results; it starts from the core's.
    kind == "project" && cp(joinpath(root, "Manifest.toml"), joinpath(target, "Manifest.toml"))
    for (folder, _, files) in walkdir(target), file in files
        text = read(joinpath(folder, file), String)
        for token in (NAME_TOKEN, UUID_TOKEN, REV_TOKEN)
            occursin(token, text) && error("$(joinpath(folder, file)) still contains the placeholder $token.")
        end
    end
    return target
end

function main(args=ARGS)
    positional, name = String[], nothing
    k = 1
    while k <= length(args)
        if args[k] == "--name" && k < length(args)
            name = args[k+1]
            k += 2
        elseif startswith(args[k], "--")
            fail("Unknown option $(args[k]).\n$HELP")
        else
            push!(positional, args[k])
            k += 1
        end
    end
    length(positional) == 2 && name !== nothing || fail(HELP)
    kind, directory = positional
    target = generate(kind, directory; name)
    core = core_identity(ROOT; require_clean=false)
    println("Created the $kind starter $name in $target, pinned to SpaceAGORA $(core.rev).")
    println("Next: read its README.md, then `git init` there and push it to its own repository.")
    return target
end

end # module NewContributorRepository

if abspath(PROGRAM_FILE) == @__FILE__
    try
        NewContributorRepository.main()
    catch error
        error isa ArgumentError || rethrow()
        println(stderr, "ERROR: ", error.msg)
        exit(1)
    end
end
