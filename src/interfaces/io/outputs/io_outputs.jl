module IOOutputs

using Arrow
using CSV
using DataFrames
using Dates
using TOML

using ..IOConfig
using ..IOSerialization
using ..EnvironmentModels: atmosphere_provenance

function _append_saved_segment!(times_acc::Vector{Float64}, data_acc::Vector, saved_values)
    seg_len = length(saved_values.t)
    seg_len == 0 && return nothing
    start_idx = 1
    if !isempty(times_acc) && isapprox(times_acc[end], saved_values.t[1]; atol=0.0, rtol=0.0)
        start_idx = 2
    end
    if start_idx <= seg_len
        append!(times_acc, @view saved_values.t[start_idx:seg_len])
        append!(data_acc, @view saved_values.saveval[start_idx:seg_len])
    end
    return nothing
end

@inline function _is_flat_scalar(value)::Bool
    return value === missing || value === nothing || value isa Number || value isa AbstractString || value isa Symbol || value isa Bool
end

function _find_sample_value(series)
    for value in series
        if value !== nothing
            return value
        end
    end
    return nothing
end

function _append_series_columns!(results_df::DataFrame, prefix::String, series)
    sample = _find_sample_value(series)
    if sample === nothing || _is_flat_scalar(sample)
        results_df[!, prefix] = collect(series)
        return nothing
    end

    if sample isa NamedTuple
        for key in keys(sample)
            child_series = [value === nothing ? nothing : getproperty(value, key) for value in series]
            _append_series_columns!(results_df, string(prefix, "_", key), child_series)
        end
        return nothing
    end

    if sample isa AbstractDict
        for key in sort!(collect(keys(sample)); by=string)
            child_series = [value === nothing ? nothing : value[key] for value in series]
            _append_series_columns!(results_df, string(prefix, "_", key), child_series)
        end
        return nothing
    end

    if sample isa Tuple || sample isa AbstractArray
        for idx in eachindex(sample)
            child_series = [value === nothing ? nothing : value[idx] for value in series]
            _append_series_columns!(results_df, string(prefix, "_", idx), child_series)
        end
        return nothing
    end

    results_df[!, prefix] = collect(series)
    return nothing
end

# Direct assembly for the shape every built-in per-satellite save field has:
# each snapshot holds one `Vector` of `num_sats` entries, and each entry is
# either a real or a fixed-width vector of reals.
#
# The generic path reaches those columns through three layers of throwaway
# intermediates -- one `sat_series` per satellite, then one `child_series` per
# component, and `collect` copies that last one again. For a 3-vector field that
# is 72 bytes of allocation per (satellite, row) to produce 24 bytes of column.
# Here each final column is allocated once and filled in one pass.
#
# The reason this is split across a function barrier rather than written as one
# loop: `saved_data` is a `Vector{SaveData}` and `SaveData` is
# `Dict{Symbol, Any}`, so a snapshot lookup is `Any`. Doing the fills directly
# off that makes every element access a dynamic dispatch, which measured *three
# times worse* than the generic path it was meant to replace -- the generic
# comprehensions are fast precisely because `[snapshot[name] for ...]` narrows
# to a concrete element type. So the narrowing comprehension is kept, and the
# concrete `values` is handed to a method that dispatches on its element type.
#
# Column order and column element types must match the generic path exactly,
# because the feather schema is part of the output. On types, note that the
# generic component path builds `value === nothing ? nothing : value[idx]` but
# still yields a `Vector{T}`, not a `Vector{Union{Nothing, T}}` -- the
# comprehension narrows to the element type actually produced.
function _direct_assembly_columns!(results_df::DataFrame, field, saved_data::Vector, num_sats::Int)::Bool
    (isempty(saved_data) || num_sats <= 0) && return false
    values = [snapshot[field.name] for snapshot in saved_data]
    return _fill_per_satellite_columns!(results_df, field.column_prefix, values, num_sats)
end

# Both concrete methods below require the same two things of their input, and
# both learned it the hard way. `T` must be concrete: `T <: Real` admits `Real`
# and `Union{Int, Float64}`, and a `Vector{T}` built from those carries a
# different element type than the generic path's narrowing comprehension
# produces -- which for a union is a different Arrow schema, not just a
# different `eltype`. And every row must be indexed `1:num_sats`: `length` alone
# proves the count, not the index range, so a row indexed `0:num_sats-1` passed
# the old guard and the `@inbounds` write then read outside it.
@inline _rows_are_one_based(values, num_sats::Int)::Bool =
    all(row -> axes(row) == (Base.OneTo(num_sats),), values)

# Anything the two concrete methods below do not claim stays with the generic path.
_fill_per_satellite_columns!(::DataFrame, ::String, values, ::Int)::Bool = false

# One scalar column per satellite.
function _fill_per_satellite_columns!(
    results_df::DataFrame, prefix::String, values::Vector{V}, num_sats::Int
)::Bool where {T <: Real, V <: AbstractVector{T}}
    isconcretetype(T) || return false
    n_rows = length(values)
    _rows_are_one_based(values, num_sats) || return false
    for sat_idx in 1:num_sats
        column = Vector{T}(undef, n_rows)
        @inbounds for row in 1:n_rows
            column[row] = values[row][sat_idx]
        end
        results_df[!, "sc$(sat_idx)_$(prefix)"] = column
    end
    return true
end

# `n_comp` columns per satellite, named `_1`.._n` to match the generic path's
# `eachindex` walk.
#
# The column set has to be a property of the field, so every entry must be
# indexed `1:n_comp` for one fixed `n_comp`. That is checked here rather than
# inferred from the element type. `isbitstype(S)` was used for it
# once and is not proof of anything of the kind: it says the element type has a
# fixed bit layout, not a fixed length, and `UnitRange{Int}` -- two `Int`s, so a
# bitstype -- carries its length as runtime data. Both ways of getting it wrong
# are silent. A ragged field whose first entry is short drops a real column
# (`[[1:2, 10:12]]` wrote four columns where the generic path writes five,
# losing `sc2_x_3`); one whose first entry is long fabricates a value, because
# the write loop is `@inbounds` and reading past the end of a `UnitRange`
# computes an element that was never there.
function _fill_per_satellite_columns!(
    results_df::DataFrame, prefix::String, values::Vector{V}, num_sats::Int
)::Bool where {T <: Real, S <: AbstractVector{T}, V <: AbstractVector{S}}
    isconcretetype(T) || return false
    n_rows = length(values)
    _rows_are_one_based(values, num_sats) || return false
    n_comp = length(first(first(values)))
    n_comp > 0 || return false
    for row in values, entry in row
        axes(entry) == (Base.OneTo(n_comp),) || return false
    end
    for sat_idx in 1:num_sats
        columns = [Vector{T}(undef, n_rows) for _ in 1:n_comp]
        @inbounds for row in 1:n_rows
            entry = values[row][sat_idx]
            for comp in 1:n_comp
                columns[comp][row] = entry[comp]
            end
        end
        for comp in 1:n_comp
            results_df[!, "sc$(sat_idx)_$(prefix)_$(comp)"] = columns[comp]
        end
    end
    return true
end

function _append_save_field_columns!(results_df::DataFrame, field, saved_data::Vector, num_sats::Int)
    if field.per_satellite && _direct_assembly_columns!(results_df, field, saved_data, num_sats)
        return nothing
    end
    field_series = [snapshot[field.name] for snapshot in saved_data]
    if field.per_satellite
        for sat_idx in 1:num_sats
            sat_series = [value[sat_idx] for value in field_series]
            _append_series_columns!(results_df, "sc$(sat_idx)_$(field.column_prefix)", sat_series)
        end
        return nothing
    end
    _append_series_columns!(results_df, field.column_prefix, field_series)
    return nothing
end

function _build_results_dataframe(times::Vector{Float64}, saved_data::Vector, save_fields, args)::DataFrame
    results_df = DataFrame(time=times)
    num_sats = length(args.dynamics_model.spacecraft)
    for field in save_fields
        _append_save_field_columns!(results_df, field, saved_data, num_sats)
    end
    return results_df
end

# The long layout (`results_layout=:long`). The spacecraft table has one row per
# spacecraft and saved time, time-major (every spacecraft at the first time, then
# at the next): `spacecraft` (its position in the configuration), `spacecraft_id`,
# `plane` and `slot` (its constellation slot, `missing` without one), `time`,
# then each per-spacecraft field, named as in the wide table without its
# `sc<i>_` prefix. The run table has one row per saved time: `time` and the
# run-wide fields. Both flatten values as the wide table does, and which table a
# field goes to follows its `per_satellite` flag, not its name.
function _build_long_results_tables(times::Vector{Float64}, saved_data::Vector, save_fields, args)
    spacecraft = args.dynamics_model.spacecraft
    num_sats = length(spacecraft)
    n_times = length(times)
    slot_part(sc, part) = sc.constellation_slot === nothing ? missing : Int16(getproperty(sc.constellation_slot, part))
    spacecraft_df = DataFrame(
        spacecraft=repeat(Int32.(1:num_sats), n_times),
        spacecraft_id=repeat(Int64[sc.id for sc in spacecraft], n_times),
        plane=repeat(Union{Missing, Int16}[slot_part(sc, :plane) for sc in spacecraft], n_times),
        slot=repeat(Union{Missing, Int16}[slot_part(sc, :slot) for sc in spacecraft], n_times),
        time=repeat(times; inner=num_sats),
    )
    run_df = DataFrame(time=times)
    for field in save_fields
        if field.per_satellite
            _append_long_field_columns!(spacecraft_df, field, saved_data, num_sats)
        else
            _append_series_columns!(run_df, field.column_prefix, [snapshot[field.name] for snapshot in saved_data])
        end
    end
    return (spacecraft=spacecraft_df, run=ncol(run_df) > 1 ? run_df : nothing)
end

function _append_long_field_columns!(results_df::DataFrame, field, saved_data::Vector, num_sats::Int)
    values = [snapshot[field.name] for snapshot in saved_data]
    !isempty(values) && num_sats > 0 && _fill_long_columns!(results_df, field.column_prefix, values, num_sats) &&
        return nothing
    _append_series_columns!(results_df, field.column_prefix, [value[sat] for value in values for sat in 1:num_sats])
    return nothing
end

# The long counterparts of `_fill_per_satellite_columns!`, under the same
# conditions, so a column has the element type the wide table's columns have.
_fill_long_columns!(::DataFrame, ::String, values, ::Int)::Bool = false

function _fill_long_columns!(
    results_df::DataFrame, prefix::String, values::Vector{V}, num_sats::Int
)::Bool where {T <: Real, V <: AbstractVector{T}}
    isconcretetype(T) || return false
    _rows_are_one_based(values, num_sats) || return false
    column = Vector{T}(undef, length(values) * num_sats)
    @inbounds for (row, entries) in enumerate(values), sat in 1:num_sats
        column[(row - 1) * num_sats + sat] = entries[sat]
    end
    results_df[!, prefix] = column
    return true
end

function _fill_long_columns!(
    results_df::DataFrame, prefix::String, values::Vector{V}, num_sats::Int
)::Bool where {T <: Real, S <: AbstractVector{T}, V <: AbstractVector{S}}
    isconcretetype(T) || return false
    _rows_are_one_based(values, num_sats) || return false
    n_comp = length(first(first(values)))
    n_comp > 0 || return false
    for row in values, entry in row
        axes(entry) == (Base.OneTo(n_comp),) || return false
    end
    columns = [Vector{T}(undef, length(values) * num_sats) for _ in 1:n_comp]
    @inbounds for (row, entries) in enumerate(values), sat in 1:num_sats
        entry = entries[sat]
        for comp in 1:n_comp
            columns[comp][(row - 1) * num_sats + sat] = entry[comp]
        end
    end
    for comp in 1:n_comp
        results_df[!, "$(prefix)_$(comp)"] = columns[comp]
    end
    return true
end

# A saved mission record belongs to one completed result write. Invalidate only
# its manifest association before replacing any outputs, including CSV-only and
# partial-failure saves. Keep the history itself for inspection or recovery.
function _invalidate_mission_history!(args)
    manifest_path = IOConfig._results_bundle_prefix(args) * ".manifest.toml"
    isfile(manifest_path) || return nothing
    manifest = try
        TOML.parsefile(manifest_path)
    catch err
        err isa TOML.ParserError || rethrow()
        return nothing # an unrelated/unreadable manifest carries no owned association
    end
    association = get(manifest, "mission_history", nothing)
    association isa AbstractDict && get(association, "schema", nothing) === 1 || return nothing
    delete!(manifest, "mission_history")
    IOSerialization._atomic_write_file(manifest_path, tmp -> open(tmp, "w") do io
        TOML.print(io, manifest)
    end)
    return nothing
end

function _write_results_csv!(results_df::DataFrame, args)::String
    _invalidate_mission_history!(args)
    primary_path = IOConfig._results_csv_path(args)
    started_s = time()
    existed_before = isfile(primary_path)
    try
        return IOSerialization._atomic_write_file(primary_path, tmp -> CSV.write(tmp, results_df); force=false)
    catch err
        if err isa ArgumentError && isfile(primary_path)
            mtime_s = try
                stat(primary_path).mtime
            catch
                0.0
            end
            concurrent_writer = (!existed_before) || (mtime_s >= started_s)
            if concurrent_writer
                collision_path = IOConfig._collision_results_csv_path(args)
                IOSerialization._atomic_write_file(collision_path, tmp -> CSV.write(tmp, results_df); force=false)
            end
            return IOSerialization._atomic_write_file(primary_path, tmp -> CSV.write(tmp, results_df); force=true)
        end
        rethrow(err)
    end
end

function _write_results_bundle!(
    results_df::DataFrame,
    times::Vector{Float64},
    args,
    results_bundle_schema_version;
    csv_path::Union{Nothing, String}=nothing,
    resumed_from_s::Union{Nothing, Float64}=nothing
)
    _invalidate_mission_history!(args)
    prefix = IOConfig._results_bundle_prefix(args)
    feather_path = prefix * ".feather"
    manifest_path = prefix * ".manifest.toml"

    IOSerialization._atomic_write_file(feather_path, tmp -> Arrow.write(tmp, results_df))

    files = Dict{String, Any}()
    files["feather"] = Dict(
        "path" => feather_path,
        "size_bytes" => filesize(feather_path),
        "sha256" => IOSerialization._sha256_hex(feather_path)
    )

    if args.simulation_settings.save_csv
        csv_file_path = csv_path === nothing ? (prefix * ".csv") : csv_path
        if csv_path === nothing
            IOSerialization._atomic_write_file(csv_file_path, tmp -> CSV.write(tmp, results_df))
        end
        files["csv"] = Dict(
            "path" => csv_file_path,
            "size_bytes" => filesize(csv_file_path),
            "sha256" => IOSerialization._sha256_hex(csv_file_path)
        )
    end

    manifest = Dict{String, Any}(
        "schema_version" => results_bundle_schema_version,
        "created_utc" => string(now(UTC)),
        "mission_time_s" => args.mission_configuration.mission_time,
        "steps" => length(times),
        "spacecraft_count" => length(args.dynamics_model.spacecraft),
        "orientation_sim" => args.mission_configuration.orientation_sim,
        "atmosphere" => atmosphere_provenance(args.environment_model.density_model),
        "files" => files
    )
    # When the rows start, and for a resumed run where it resumed: a run resumed
    # from a checkpoint that kept no rows starts at its resume.
    isempty(times) || (manifest["first_time_s"] = times[1])
    resumed_from_s === nothing || (manifest["resumed_from_s"] = resumed_from_s)

    IOSerialization._atomic_write_file(manifest_path, tmp -> begin
        open(tmp, "w") do io
            TOML.print(io, manifest)
        end
    end)

    return nothing
end

# The long layout's files, beside the bundle prefix: `<prefix>_spacecraft.*` and,
# when the run has run-wide fields, `<prefix>_run.*`, as `.csv` with `save_csv`
# and as `.feather` with the bundle, whose manifest (`schema_version` "2",
# `layout` "long") lists them. A run without run-wide fields removes the run
# table files an earlier long run left under the same prefix. Returns the CSV
# paths written.
function _write_long_results!(
    tables,
    times::Vector{Float64},
    args,
    results_bundle_schema_version;
    write_bundle::Bool=true,
    resumed_from_s::Union{Nothing, Float64}=nothing
)
    _invalidate_mission_history!(args)
    prefix = IOConfig._results_bundle_prefix(args)
    parts = Pair{String, DataFrame}["spacecraft" => tables.spacecraft]
    tables.run === nothing ? foreach(ext -> rm(prefix * "_run" * ext; force=true), (".csv", ".feather")) :
        push!(parts, "run" => tables.run)
    files = Dict{String, Any}()
    entry(path) = Dict("path" => path, "size_bytes" => filesize(path), "sha256" => IOSerialization._sha256_hex(path))
    csv_paths = (spacecraft=nothing, run=nothing)
    for (part, df) in parts
        if args.simulation_settings.save_csv
            csv_path = IOSerialization._atomic_write_file(prefix * "_" * part * ".csv", tmp -> CSV.write(tmp, df))
            files[part * "_csv"] = entry(csv_path)
            csv_paths = merge(csv_paths, NamedTuple{(Symbol(part),)}((csv_path,)))
        end
        if write_bundle
            feather_path = IOSerialization._atomic_write_file(prefix * "_" * part * ".feather", tmp -> Arrow.write(tmp, df))
            files[part * "_feather"] = entry(feather_path)
        end
    end
    write_bundle || return csv_paths
    _write_long_manifest!(args, results_bundle_schema_version, files, length(times),
        isempty(times) ? nothing : times[1], resumed_from_s)
    return csv_paths
end

# The long layout's manifest: `schema_version` "2", `layout` "long", the `files`
# given, `steps` saved times from `first_time_s`. `extra` adds entries (a
# streaming run's manifest before it ends).
function _write_long_manifest!(args, results_bundle_schema_version, files, steps::Integer,
                               first_time_s, resumed_from_s; extra=())
    prefix = IOConfig._results_bundle_prefix(args)
    manifest = Dict{String, Any}(
        "schema_version" => results_bundle_schema_version,
        "layout" => "long",
        "created_utc" => string(now(UTC)),
        "mission_time_s" => args.mission_configuration.mission_time,
        "steps" => steps,
        "spacecraft_count" => length(args.dynamics_model.spacecraft),
        "orientation_sim" => args.mission_configuration.orientation_sim,
        "atmosphere" => atmosphere_provenance(args.environment_model.density_model),
        "files" => files
    )
    first_time_s === nothing || (manifest["first_time_s"] = first_time_s)
    resumed_from_s === nothing || (manifest["resumed_from_s"] = resumed_from_s)
    for (key, value) in extra
        manifest[key] = value
    end
    IOSerialization._atomic_write_file(prefix * ".manifest.toml", tmp -> open(io -> TOML.print(io, manifest), tmp, "w"))
    return nothing
end

# Streaming, for a long-layout run with checkpoints that writes results: each
# checkpoint appends its rows to an Arrow IPC stream beside the checkpoint, which
# Arrow can append to, and to the CSV. When the run ends, or a solve fails, the
# results feather is written from the stream, one record batch per append, so it
# is an Arrow file as the unstreamed run's is. The stream stays beside the
# checkpoint for a later resume, which cuts it back to the checkpoint's length.

# Append `df` to the stream at `path`, which a first append starts and tags with
# `token`, and to `csv_path` (with the header when it is empty). Returns their
# lengths in bytes.
function _append_long_stream!(path::String, csv_path::Union{Nothing, String}, df::DataFrame, token::String)
    mkpath(dirname(path))
    started = isfile(path) && filesize(path) > 0
    Arrow.append(path, df; metadata=started ? nothing : Dict("spaceagora_results_token" => token), ntasks=1)
    csv_bytes = 0
    if csv_path !== nothing
        mkpath(dirname(csv_path))
        CSV.write(csv_path, df; append=isfile(csv_path) && filesize(csv_path) > 0)
        csv_bytes = filesize(csv_path)
    end
    return filesize(path), csv_bytes
end

# The token a stream was started with, or `nothing` (also for a file that is not
# a readable stream).
function _long_stream_token(path::String)
    isfile(path) && filesize(path) > 0 || return nothing
    metadata = try
        Arrow.getmetadata(Arrow.Table(path))
    catch err
        err isa InterruptException && rethrow()
        nothing
    end
    return metadata === nothing ? nothing : get(metadata, "spaceagora_results_token", nothing)
end

# The long layout's files from the streams (`streams.spacecraft`, and
# `streams.run` or `nothing` without run-wide fields): the feathers written from
# the streams and the CSVs as streamed, listed in the manifest. A run without a
# run table removes the run table files an earlier run left. Returns the CSV
# paths and the feather paths.
function _finish_long_stream!(streams, args, results_bundle_schema_version; steps::Integer, first_time_s, resumed_from_s)
    _invalidate_mission_history!(args)
    prefix = IOConfig._results_bundle_prefix(args)
    files = Dict{String, Any}()
    entry(path) = Dict("path" => path, "size_bytes" => filesize(path), "sha256" => IOSerialization._sha256_hex(path))
    csv = Dict{Symbol, Union{Nothing, String}}(:spacecraft => nothing, :run => nothing)
    feather = Dict{Symbol, Union{Nothing, String}}(:spacecraft => nothing, :run => nothing)
    streams.run === nothing && foreach(ext -> rm(prefix * "_run" * ext; force=true), (".csv", ".feather"))
    for part in (:spacecraft, :run)
        stream = getproperty(streams, part)
        stream === nothing && continue
        csv_path = prefix * "_" * String(part) * ".csv"
        if args.simulation_settings.save_csv && isfile(csv_path)
            files[String(part) * "_csv"] = entry(csv_path)
            csv[part] = csv_path
        end
        feather[part] = IOSerialization._atomic_write_file(prefix * "_" * String(part) * ".feather",
            tmp -> Arrow.write(tmp, Arrow.Stream(stream); metadata=nothing, ntasks=1))
        files[String(part) * "_feather"] = entry(feather[part])
    end
    _write_long_manifest!(args, results_bundle_schema_version, files, steps, first_time_s, resumed_from_s)
    return (csv=(spacecraft=csv[:spacecraft], run=csv[:run]), feather=(spacecraft=feather[:spacecraft], run=feather[:run]))
end

export _append_saved_segment!
export _append_series_columns!
export _direct_assembly_columns!
export _build_results_dataframe
export _build_long_results_tables
export _write_results_csv!
export _write_results_bundle!
export _write_long_results!
export _write_long_manifest!
export _append_long_stream!
export _long_stream_token
export _finish_long_stream!

end # module IOOutputs
