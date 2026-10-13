"""
Repository asset manifest and availability operations. This internal service
returns plain field records so command front ends retain their own public types.
It performs no downloads and does not interpret atmosphere or geometry schemas.
"""
module AssetCatalog

using TOML

const REPO_ROOT = normpath(joinpath(@__DIR__, "..", "..", ".."))

function load_asset_manifest(; repo_root::String=REPO_ROOT,
    manifest_path::String=joinpath(repo_root, "data", "assets_manifest.toml"))
    raw = TOML.parsefile(manifest_path)
    entries = NamedTuple[]
    for entry in get(raw, "asset", Any[])
        push!(entries, (
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

@inline function manifest_entry_path(repo_root::String, entry)
    entry.relative_path in ("", ".") && return repo_root
    return joinpath(repo_root, splitpath(entry.relative_path)...)
end

@inline function manifest_entry_available(repo_root::String, entry)
    path = manifest_entry_path(repo_root, entry)
    entry.kind == "builtin" && return true
    entry.kind == "directory" && return isdir(path)
    entry.kind == "file" && return isfile(path)
    throw(ArgumentError("Unsupported asset manifest kind '$(entry.kind)' for $(entry.name)."))
end

function check_assets(; repo_root::String=REPO_ROOT,
    manifest_path::String=joinpath(repo_root, "data", "assets_manifest.toml"))
    items = NamedTuple[]
    for entry in load_asset_manifest(; repo_root, manifest_path)
        detail = isempty(entry.detail) ? "licensing=$(entry.licensing)" : "$(entry.detail) [licensing=$(entry.licensing)]"
        push!(items, (
            name=entry.name,
            scope=entry.scope,
            path=manifest_entry_path(repo_root, entry),
            available=manifest_entry_available(repo_root, entry),
            required=entry.required,
            detail=detail,
        ))
    end
    return (; repo_root, items)
end

end # module AssetCatalog
