using TOML

const ASSET_STATES = (:usable, :missing, :empty, :pointer_only, :invalid)

"""
    AssetCheckItem

Single asset-root status entry produced by `check_assets()`.

`state` is one of `:usable`, `:missing`, `:empty`, `:pointer_only` or
`:invalid`. `available` is `true` exactly when `state === :usable`; it is kept
so existing readers keep working. `reason` explains any other state in one
sentence, including what to do about it.

The original six-argument positional constructor and keyword construction
without `state` derive the state from `available`. An explicit state must be
one of the supported values and agree with `available`.
"""
Base.@kwdef struct AssetCheckItem
    name::String
    scope::String
    path::String
    available::Bool
    required::Bool
    detail::String = ""
    state::Symbol = convert(Bool, available) ? :usable : :missing
    reason::String = ""

    function AssetCheckItem(name, scope, path, available, required, detail, state, reason)
        ready = convert(Bool, available)
        status = convert(Symbol, state)
        status in ASSET_STATES || throw(ArgumentError("Unknown asset readiness state $(repr(status)) for $(name)."))
        ready == (status === :usable) || throw(ArgumentError("Asset availability must agree with readiness state $(repr(status)) for $(name)."))
        return new(name, scope, path, ready, required, detail, status, reason)
    end
end

AssetCheckItem(name, scope, path, available, required, detail) =
    AssetCheckItem(name, scope, path, available, required, detail,
                   convert(Bool, available) ? :usable : :missing, "")

"""
    AssetCheckReport

Repository-local asset availability report returned by `check_assets()`.
"""
Base.@kwdef struct AssetCheckReport
    repo_root::String
    items::Vector{AssetCheckItem}
end

Base.@kwdef struct AssetManifestEntry
    name::String
    scope::String
    relative_path::String
    kind::String
    required::Bool
    licensing::String
    detail::String = ""
end

@inline function _asset_item(name::String, scope::String, path::String; state::Symbol, required::Bool, detail::String="", reason::String="")
    return AssetCheckItem(
        name=name,
        scope=scope,
        path=path,
        available=state === :usable,
        required=required,
        detail=detail,
        state=state,
        reason=reason,
    )
end

function load_asset_manifest(; repo_root::String=REPO_ROOT, manifest_path::String=joinpath(repo_root, "data", "assets_manifest.toml"))
    raw = TOML.parsefile(manifest_path)
    entries = AssetManifestEntry[]
    for entry in get(raw, "asset", Any[])
        push!(entries, AssetManifestEntry(
            name=String(entry["name"]),
            scope=String(entry["scope"]),
            relative_path=String(get(entry, "relative_path", ".")),
            kind=String(get(entry, "kind", "directory")),
            required=Bool(get(entry, "required", false)),
            licensing=String(get(entry, "licensing", "unspecified")),
            detail=String(get(entry, "detail", "")),
        ))
    end
    return entries
end

@inline function _manifest_entry_path(repo_root::String, entry::AssetManifestEntry)
    entry.relative_path in ("", ".") && return repo_root
    return joinpath(repo_root, splitpath(entry.relative_path)...)
end

# A Git LFS pointer is a short text stub left in place of a payload that was not
# downloaded (for example after a clone with GIT_LFS_SKIP_SMUDGE=1). The preset
# verifier in the environment models recognises the same header for its own
# payloads; this check only classifies, it never fetches.
const _LFS_POINTER_HEADER = "version https://git-lfs.github.com/spec/v1"
const _LFS_POINTER_MAX_BYTES = 1024

# Git worktrees and initialized submodules use a .git file instead of a
# directory. Finding that marker needs no Git process or network access.
function _asset_git_root(path::String)
    candidate = isdir(path) ? path : dirname(path)
    while true
        marker = joinpath(candidate, ".git")
        (isdir(marker) || isfile(marker)) && return candidate
        parent = dirname(candidate)
        parent == candidate && return nothing
        candidate = parent
    end
end

_asset_shell_quote(value::String) = "'" * replace(value, "'" => "'\"'\"'") * "'"

function _lfs_fetch_hint(path::String; asset_directory::Union{Nothing,String}=nothing)
    payload = realpath(path)
    owner = _asset_git_root(payload)
    isnothing(owner) && return "retrieve the payload through the documented asset route (no Git checkout found for this path)"
    include_pattern = relpath(payload, owner)
    if !isnothing(asset_directory) && _asset_git_root(asset_directory) == owner
        # Collapse ordinary directory pointers into one hint, but preserve the
        # actual owner and exact path for file symlinks outside this asset root.
        relative_parts = splitpath(relpath(payload, asset_directory))
        if first(relative_parts) != ".."
            directory_pattern = relpath(asset_directory, owner)
            include_pattern = directory_pattern == "." ? "*" : joinpath(directory_pattern, "*")
        end
    end
    command = "git -C $(_asset_shell_quote(owner)) lfs pull --include=$(_asset_shell_quote(include_pattern))"
    return "fetch the payload with `$(command)` or the documented asset route"
end

function _is_lfs_pointer(path::String)::Bool
    size = filesize(path)
    0 < size <= _LFS_POINTER_MAX_BYTES || return false
    header = open(io -> read(io, min(ncodeunits(_LFS_POINTER_HEADER), size)), path)
    return String(header) == _LFS_POINTER_HEADER
end

function _file_readiness(path::String)
    isfile(path) || return (:missing, "no file at this path")
    filesize(path) == 0 && return (:invalid, "the file is empty (0 bytes)")
    _is_lfs_pointer(path) && return (:pointer_only, "the file is an undownloaded Git LFS pointer; " * _lfs_fetch_hint(path))
    return (:usable, "")
end

# Hidden files and directories never count as payload. Symlinks to files
# count as whatever they point at; the walk does not follow symlinked
# directories.
function _directory_readiness(path::String)
    isdir(path) || return (:missing, "no directory at this path")
    payloads = 0
    pointers = 0
    empties = 0
    fetch_hints = String[]
    canonical_directory = realpath(path)
    for (root, directories, files) in walkdir(path)
        filter!(directory -> !startswith(directory, "."), directories)
        for file in files
            startswith(file, ".") && continue
            full = joinpath(root, file)
            isfile(full) || continue
            if filesize(full) == 0
                empties += 1
            elseif _is_lfs_pointer(full)
                pointers += 1
                hint = _lfs_fetch_hint(full; asset_directory=canonical_directory)
                hint in fetch_hints || push!(fetch_hints, hint)
            else
                payloads += 1
            end
        end
    end
    total = payloads + pointers + empties
    total == 0 && return (:empty, "the directory contains no files")
    if pointers > 0
        detail = payloads > 0 ? " alongside $(payloads) downloaded payload file$(payloads == 1 ? "" : "s")" : ""
        return (:pointer_only, "$(pointers) of $(total) files are undownloaded Git LFS pointers$(detail); " * join(fetch_hints, "; "))
    end
    payloads == 0 && return (:invalid, "all $(total) files are empty (0 bytes)")
    return (:usable, "")
end

"""
    _manifest_entry_readiness(repo_root, entry) -> (state, reason)

Classify a manifest entry without fetching anything. Directory and file
entries distinguish a missing path, an empty directory, undownloaded Git LFS
pointers, empty payloads and usable payloads. Built-in entries are always
usable. The check confirms that payload bytes exist; it does not validate their
contents, which the named-preset `assets check --preset` route does for grids.
"""
function _manifest_entry_readiness(repo_root::String, entry::AssetManifestEntry)
    path = _manifest_entry_path(repo_root, entry)
    entry.kind == "builtin" && return (:usable, "")
    entry.kind == "directory" && return _directory_readiness(path)
    entry.kind == "file" && return _file_readiness(path)
    throw(ArgumentError("Unsupported asset manifest kind '$(entry.kind)' for $(entry.name)."))
end

@inline _manifest_entry_available(repo_root::String, entry::AssetManifestEntry) =
    first(_manifest_entry_readiness(repo_root, entry)) === :usable

"""
    check_assets(; repo_root=REPO_ROOT, manifest_path=joinpath(repo_root, "data", "assets_manifest.toml")) -> AssetCheckReport

Inspect the standard SpaceAGORA asset roots and return a typed readiness
report for baseline, optional, and high-fidelity assets. Each item carries a
`state` (`:usable`, `:missing`, `:empty`, `:pointer_only` or `:invalid`) and a
`reason` for any state other than `:usable`. The check reads directory
listings and file headers only; it never downloads. By default, inspect the
package repository root, not the current working directory.
"""
function check_assets(; repo_root::String=REPO_ROOT, manifest_path::String=joinpath(repo_root, "data", "assets_manifest.toml"))::AssetCheckReport
    entries = load_asset_manifest(; repo_root=repo_root, manifest_path=manifest_path)
    items = AssetCheckItem[]
    for entry in entries
        detail = isempty(entry.detail) ? "licensing=$(entry.licensing)" : "$(entry.detail) [licensing=$(entry.licensing)]"
        state, reason = _manifest_entry_readiness(repo_root, entry)
        push!(items, _asset_item(
            entry.name,
            entry.scope,
            _manifest_entry_path(repo_root, entry);
            state=state,
            required=entry.required,
            detail=detail,
            reason=reason,
        ))
    end

    return AssetCheckReport(repo_root=repo_root, items=items)
end

_asset_status_label(item::AssetCheckItem) =
    item.state === :usable ? "usable" :
    replace(String(item.state), '_' => '-') * (item.required ? "-required" : "-optional")

"""
    asset_check_exit_status(report) -> Int

`0` when every required entry is usable, `1` otherwise. Optional entries never
change the exit status; their state is only reported.
"""
asset_check_exit_status(report::AssetCheckReport)::Int =
    any(item -> item.required && item.state !== :usable, report.items) ? 1 : 0

"""
    render_asset_report(report; io=stdout)

Render a human-readable asset readiness report. Each entry shows its status
(`usable`, or its state suffixed with `-required` or `-optional`), its scope,
path and licensing detail, and a `reason` line for any entry that is not
usable. The report ends with a one-line summary.
"""
function render_asset_report(report::AssetCheckReport; io::IO=stdout)
    println(io, "SpaceAGORA asset check")
    println(io, "repo_root=$(report.repo_root)")
    for item in report.items
        println(io, "- $(item.name): $(_asset_status_label(item))")
        println(io, "  scope: $(item.scope)")
        println(io, "  path: $(item.path)")
        println(io, "  detail: $(item.detail)")
        isempty(item.reason) || println(io, "  reason: $(item.reason)")
    end
    usable = count(item -> item.state === :usable, report.items)
    blocked = count(item -> item.required && item.state !== :usable, report.items)
    println(io, "summary: $(usable) of $(length(report.items)) entries usable; $(blocked) required entries not usable")
    return nothing
end

function render_asset_manifest(entries::Vector{AssetManifestEntry}; io::IO=stdout)
    println(io, "SpaceAGORA asset manifest")
    println(io, "entries=$(length(entries))")
    for entry in entries
        println(io, "- $(entry.name)")
        println(io, "  scope: $(entry.scope)")
        println(io, "  kind: $(entry.kind)")
        println(io, "  relative_path: $(entry.relative_path)")
        println(io, "  required: $(entry.required)")
        println(io, "  licensing: $(entry.licensing)")
        !isempty(entry.detail) && println(io, "  detail: $(entry.detail)")
    end
    return nothing
end

function setup_open_assets(; repo_root::String=REPO_ROOT, io::IO=stdout)
    entries = load_asset_manifest(; repo_root=repo_root)
    report = check_assets(; repo_root=repo_root)
    manifest_path = joinpath(repo_root, "data", "assets_manifest.toml")
    println(io, "SpaceAGORA open-asset setup")
    println(io, "No downloads are required for baseline no-GRAM mode.")
    println(io, "Manifest path: $manifest_path")
    println(io)
    println(io, "Baseline/open entries:")
    for entry in entries
        entry.licensing == "licensed-external" && continue
        println(io, "- $(entry.name): $(entry.relative_path)")
    end
    println(io)
    println(io, "Licensed external entries remain user-provided:")
    for entry in entries
        entry.licensing == "licensed-external" || continue
        println(io, "- $(entry.name): $(entry.relative_path)")
    end
    println(io)
    render_asset_report(report; io=io)
    return report
end

function _run_preset_assets(command::String, args::Vector{String}; io::IO=stdout)::Int
    if command == "list"
        isempty(args) || throw(ArgumentError("assets list takes no options"))
        for entry in available_surrogate_presets()
            status = entry["release_enabled"] ? "available" : "unpublished"
            println(io, entry["id"], "@", entry["version"], " (", entry["planet"], ", ", status, ")")
            println(io, "  ", entry["description"])
        end
        return 0
    end
    options = Dict{String,String}()
    offline = command == "check"
    i = 1
    while i <= length(args)
        arg = args[i]
        if arg == "--offline"
            offline = true
        elseif startswith(arg, "--")
            key_value = split(arg[3:end], '='; limit=2)
            key = first(key_value)
            key in ("preset", "version", "file") || throw(ArgumentError("Unknown preset option $arg"))
            haskey(options, key) && throw(ArgumentError("Duplicate --$key option"))
            if length(key_value) == 2
                options[key] = key_value[2]
            else
                i += 1
                i <= length(args) && !startswith(args[i], "--") || throw(ArgumentError("--$key needs a value"))
                options[key] = args[i]
            end
            isempty(options[key]) && throw(ArgumentError("--$key needs a nonempty value"))
        else
            throw(ArgumentError("Unexpected preset argument $arg"))
        end
        i += 1
    end
    all(key -> haskey(options, key), ("preset", "version")) || throw(ArgumentError(
        "assets $command requires --preset=<id> and --version=<version>; use assets list"))
    preset = options["preset"]
    version = options["version"]
    file = get(options, "file", "")
    # Fetch verifies the downloaded bytes. Check additionally loads and validates
    # the schema, axes and metadata; neither command initializes native GRAM.
    if command == "fetch"
        resolved = resolve_surrogate_preset(preset; version, file, offline)
        println(io, "Verified ", preset, "@", version, ": ", resolved.file)
        println(io, "sha256=", resolved.expected_sha256)
    else
        model = surrogate_preset_model(preset; version, file, offline)
        provenance = atmosphere_provenance(model)
        println(io, "Validated ", preset, "@", version)
        TOML.print(io, provenance)
    end
    return 0
end
