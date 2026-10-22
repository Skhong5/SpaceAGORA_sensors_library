using ..SimulationModel.SimConfig: _with_fields
import ..SimulationModel
import ..SimulationEngine
using Arrow: Arrow
using CSV: CSV
using DataFrames: DataFrame, nrow, ncol

"""
    ConstellationEnsembleResult

What [`run_constellation_ensemble`](@ref)`(args; merge_results=true)` returns.

- `campaign::MonteCarloResult`: what the call returns without `merge_results`.
  Sample `k` is member `k`, and its `value` is what `run_simulation` returned for
  the keywords passed: `nothing` by default, the solution with
  `return_solution=true`, the `SimulationResults` with `return_results=true`.
- `members`: one row per member and saved time, a Tables.jl table (a
  `DataFrame`). It has the long layout's columns (see [`long_table`](@ref)):
  `spacecraft` (the member's position in `args.dynamics_model.spacecraft`, which
  is also its sample index and folder number), `spacecraft_id`, `plane` and
  `slot` (from its constellation slot, `missing` without one) and `time`, then
  the member's per-spacecraft fields without a `sc<i>_` prefix and its run-wide
  fields. Members with different fields give the union of the columns, in the
  order first seen, with `missing` where a member has none. Rows are in position
  order, then saved time. A failed member has no rows. When the ensemble writes
  `constellation_members.feather`, it writes it member by member and `members`
  reads it in place, with read-only columns; otherwise `members` is built in
  memory.
- `constellation::DataFrame`: one row per member, in position order:
  `spacecraft`, `spacecraft_id`, `plane`, `slot`, `member_tag` (the member's
  folder name), `success`, `error` (the error text of a failed member, `missing`
  otherwise), `elapsed_s`, `rows` (its rows in `members`) and the member's last
  row in `members` under the same names (`missing` for a member without rows).
- `files::Vector{String}`: the merged files written. With `results=true`,
  `constellation_members.feather` (with the results bundle, as the members
  write theirs), `constellation_members.csv` (with `save_csv`) and
  `constellation_summary.csv`, in `results_directory` beside the member
  folders. Empty otherwise.

Each member's rows are in memory until the merge. For large campaigns,
`SPACEAGORA_RESULTS_THIN_STRIDE` and `SPACEAGORA_RESULTS_FINAL_ONLY` thin each
member's rows, and [`read_constellation_results`](@ref) merges member folders
already written, one member at a time.
"""
struct ConstellationEnsembleResult{M}
    campaign::MonteCarloResult
    members::M
    constellation::DataFrame
    files::Vector{String}
end

const _CONSTELLATION_IDENTITY = ("spacecraft", "spacecraft_id", "plane", "slot")

# A run-wide save field and a per-spacecraft one with the same column prefix
# would give two columns of one name in the members table, where a member's
# run-wide fields sit beside its per-spacecraft ones. Refused before any member
# runs; names that only meet after flattening are refused when the tables merge.
function _validate_merged_save_fields(run_kwargs)
    fields = get(run_kwargs, :save_fields, nothing)
    fields === nothing && return nothing
    per_spacecraft = Set{String}(field.column_prefix for field in fields if field.per_satellite)
    for field in fields
        field.per_satellite && continue
        field.column_prefix in per_spacecraft && throw(ArgumentError(
            "run_constellation_ensemble with merge_results=true puts each member's run-wide fields beside its " *
            "per-spacecraft fields, so they need different names, but the run-wide save field :$(field.name) " *
            "has the column prefix \"$(field.column_prefix)\" of a per-spacecraft field. Give it another column_prefix."))
    end
    return nothing
end

# One member's run with merge_results=true. It runs with `return_results=true`,
# so its saved rows are kept, and its final results save puts them, in the long
# layout from the long builder, in the sink. It returns what `run_simulation`
# returns for the caller's own keywords (`value`) and those tables. The
# configuration that came back with the rows is dropped here unless the caller
# asked for the results, so it never travels from a process worker.
function _constellation_member_outcome(member::SimulationConfiguration, run_kwargs)
    wants_results = get(run_kwargs, :return_results, false) === true
    wants_solution = get(run_kwargs, :return_solution, false) === true
    kwargs = Dict{Symbol, Any}(pairs(run_kwargs))
    kwargs[:return_results] = true
    sink = Ref{Any}(nothing)
    result = Base.ScopedValues.with(SimulationEngine._LONG_TABLES_SINK => sink) do
        SimulationEngine.run_simulation(member; kwargs...)
    end
    tables = sink[]
    value = wants_results ? result : wants_solution ? result.solution : nothing
    return (value=value, spacecraft=tables.spacecraft, run=tables.run)
end

# The member run of an ensemble: `run_simulation`, or with merge_results=true the
# outcome above.
_constellation_member_run(member::SimulationConfiguration, run_kwargs, merge_results::Bool) =
    merge_results ? _constellation_member_outcome(member, run_kwargs) :
        SimulationEngine.run_simulation(member; run_kwargs...)

# Member `position`'s rows in the members table: its spacecraft table, with
# `spacecraft` set to its position in the constellation, and its run-wide
# columns beside them. A member runs one spacecraft, so its run table has one
# row per row of its spacecraft table.
function _constellation_member_rows(position::Integer, outcome)::DataFrame
    rows = outcome.spacecraft
    rows[!, "spacecraft"] = fill(Int32(position), nrow(rows))
    run = outcome.run
    run === nothing && return rows
    isequal(run[!, "time"], rows[!, "time"]) ||
        error("Member $(position)'s run table and spacecraft table have different saved times.")
    for name in names(run)
        name == "time" && continue
        name in names(rows) && throw(ArgumentError(
            "run_constellation_ensemble with merge_results=true cannot put member $(position)'s run-wide column " *
            "\"$(name)\" beside its per-spacecraft column of the same name. Give the run-wide save field another " *
            "column_prefix."))
        rows[!, name] = run[!, name]
    end
    return rows
end

# The folder name of member `position` (see `_ensemble_member_configuration`):
# the unpadded name when the member resumed from a checkpoint written there.
function _constellation_member_tag(member::SimulationConfiguration, position::Integer, count::Integer, id::Integer)
    unpadded = "sat_$(position)_id_$(id)"
    basename(normpath(member.simulation_settings.results_directory)) == unpadded && return unpadded
    return _ensemble_member_tag(position, count, id)
end

# The merged result of an ensemble run with merge_results=true, from the
# campaign whose samples hold the members' outcomes. Members are merged in
# position order, whatever order they finished in, and each member's tables are
# released once appended.
function _constellation_ensemble_result(args::SimulationConfiguration, member_configs, campaign::MonteCarloResult)
    spacecraft = args.dynamics_model.spacecraft
    count = length(spacecraft)
    slot_part(sc, part) = sc.constellation_slot === nothing ? missing : Int16(getproperty(sc.constellation_slot, part))
    member_rows = DataFrame[]
    last_rows = Union{Nothing, DataFrame}[]
    row_counts = Int[]
    samples = MonteCarloSampleResult[]
    for sample in campaign.samples
        position = sample.index
        if sample.success
            outcome = sample.value
            rows = _constellation_member_rows(position, outcome)
            push!(row_counts, nrow(rows))
            push!(last_rows, nrow(rows) == 0 ? nothing : rows[nrow(rows):nrow(rows), :])
            push!(member_rows, rows)
            push!(samples, _with_fields(sample; value=outcome.value))
        else
            push!(row_counts, 0)
            push!(last_rows, nothing)
            push!(samples, sample)
        end
    end

    positions = [sample.index for sample in campaign.samples]
    identity = DataFrame(
        spacecraft=Int32.(positions),
        spacecraft_id=Int64[spacecraft[k].id for k in positions],
        plane=Union{Missing, Int16}[slot_part(spacecraft[k], :plane) for k in positions],
        slot=Union{Missing, Int16}[slot_part(spacecraft[k], :slot) for k in positions],
        member_tag=String[_constellation_member_tag(member_configs[k], k, count, spacecraft[k].id) for k in positions],
    )
    constellation = _constellation_summary(identity, Bool[sample.success for sample in campaign.samples],
        Union{Missing, String}[sample.success ? missing : sprint(showerror, sample.error) for sample in campaign.samples],
        Float64[sample.elapsed_s for sample in campaign.samples], row_counts, last_rows)

    result_campaign = MonteCarloResult(samples, campaign.elapsed_s, campaign.threads;
                                       route=campaign.route, local_slots=campaign.local_slots)
    members, files = _write_constellation_results!(args, member_rows, constellation)
    return ConstellationEnsembleResult(result_campaign, members, constellation, files)
end

# The constellation table: the members' identity and folder (`identity`), their
# status, their rows in the members table and their last row there, under the
# same names (`missing` for a member without rows).
function _constellation_summary(identity::DataFrame, success, error, elapsed_s, row_counts, last_rows)
    constellation = copy(identity)
    constellation[!, "success"] = success
    constellation[!, "error"] = error
    constellation[!, "elapsed_s"] = elapsed_s
    constellation[!, "rows"] = row_counts
    last_names = String[]
    for last in last_rows
        last === nothing && continue
        for name in names(last)
            name in _CONSTELLATION_IDENTITY || name in last_names || push!(last_names, name)
        end
    end
    for name in last_names
        constellation[!, name] = [last === nothing || !(name in names(last)) ? missing : last[1, name]
                                  for last in last_rows]
    end
    return constellation
end

# The members table of an ensemble without rows: its identity columns.
_empty_members() = DataFrame(spacecraft=Int32[], spacecraft_id=Int64[], plane=Union{Missing, Int16}[],
    slot=Union{Missing, Int16}[], time=Float64[])

# The members table's columns from the members' tables, in the order first seen,
# each with a type that holds every member's values and `missing` where a member
# lacks the column, as `append!(...; cols=:union, promote=true)` gives them.
function _members_columns(tables)
    order = String[]
    types = Dict{String, Type}()
    present = Dict{String, Int}()
    seen = 0
    for table in tables
        seen += 1
        for name in names(table)
            T = eltype(table[!, name])
            if haskey(types, name)
                types[name] = promote_type(types[name], T)
                present[name] += 1
            else
                push!(order, name)
                types[name] = T
                present[name] = 1
            end
        end
    end
    if seen == 0
        empty = _empty_members()
        return Pair{String, Type}[name => eltype(empty[!, name]) for name in names(empty)]
    end
    return Pair{String, Type}[name => (present[name] < seen ? Union{Missing, types[name]} : types[name]) for name in order]
end

# A member's rows in the members table's `columns`.
_members_batch(rows::DataFrame, columns) = DataFrame([name => (name in names(rows) ?
    convert(Vector{T}, rows[!, name]) : Vector{T}(fill(missing, nrow(rows)))) for (name, T) in columns]; copycols=false)

# The members table as an Arrow file at `path`, written member by member, one
# record batch each, so that a member's rows are in memory only while they are
# written. `tables` gives the members' tables in position order, one at a time.
function _write_members_feather!(path::String, columns, tables)
    return SimulationModel.IOSerialization._atomic_write_file(path, tmp -> begin
        writer = open(Arrow.Writer, tmp; ntasks=1)
        try
            written = 0
            for rows in tables
                Arrow.write(writer, _members_batch(rows, columns))
                written += 1
            end
            written == 0 && Arrow.write(writer, _members_batch(_empty_members(), columns))
        finally
            close(writer)
        end
    end)
end

# The merged files, in `results_directory` beside the member folders, when the
# ensemble saves results: the members table as the members write theirs (the
# feather with the results bundle, written member by member, and then read in
# place; the CSV with `save_csv`), and the constellation table as
# `constellation_summary.csv`. Without the feather the members table is built in
# memory. Returns the members table and the files written.
function _write_constellation_results!(args::SimulationConfiguration, member_rows::Vector{DataFrame}, constellation::DataFrame)
    settings = args.simulation_settings
    directory = settings.results_directory
    write(path, writer) = SimulationModel.IOSerialization._atomic_write_file(path, writer)
    files = String[]
    if settings.results && SimulationEngine._typed_save_bundle_enabled()
        mkpath(directory)
        path = joinpath(directory, "constellation_members.feather")
        columns = _members_columns(member_rows)
        # Each member's rows are released once written.
        pending = Iterators.map(_ -> popfirst!(member_rows), 1:length(member_rows))
        push!(files, _write_members_feather!(path, columns, pending))
        members = DataFrame(Arrow.Table(path); copycols=false)
    else
        members = isempty(member_rows) ? _empty_members() : popfirst!(member_rows)
        while !isempty(member_rows)
            append!(members, popfirst!(member_rows); cols=:union, promote=true)
        end
    end
    settings.results || return members, files
    mkpath(directory)
    settings.save_csv &&
        push!(files, write(joinpath(directory, "constellation_members.csv"), tmp -> CSV.write(tmp, members)))
    push!(files, write(joinpath(directory, "constellation_summary.csv"), tmp -> CSV.write(tmp, constellation)))
    return members, files
end

const _MEMBER_FOLDER = r"^sat_(\d+)_id_(-?\d+)$"

"""
    read_constellation_results(results_directory) -> (members, constellation, files)

Merge the member folders a constellation ensemble wrote under
`results_directory` (`sat_<i>_id_<id>`, each with its results bundle) into the
tables of [`ConstellationEnsembleResult`](@ref), reading one member at a time.
Nothing is run again.

- `members` has the columns and rows `run_constellation_ensemble(args;
  merge_results=true)` gives: `spacecraft` (the folder's `<i>`),
  `spacecraft_id`, `plane`, `slot` and `time`, then each member's
  per-spacecraft fields without a prefix and its run-wide fields. It is written
  to `constellation_members.feather`, one record batch per member, and read in
  place: only one member's rows are in memory at a time.
- `constellation` has one row per member folder, as the ensemble's
  `constellation` table: when `constellation_summary.csv` is there, the members'
  `success`, `error` and `elapsed_s` come from it; otherwise `success` says
  whether the member wrote results, and the other two are `missing`. It is
  written to `constellation_summary.csv`.
- `files` lists the two files written.

A member written in the long layout (`results_layout=:long`) gives its tables as
they are, with its position set. A member written in the wide layout has one
spacecraft, so its per-spacecraft columns are its `sc1_` ones, renamed without
the prefix; its plane and slot come from `constellation_summary.csv`, or are
`missing` without it. A member without a results bundle has no rows, and so has
one that `constellation_summary.csv` records as failed, whose files are partial.
"""
function read_constellation_results(results_directory::AbstractString)
    directory = String(results_directory)
    isdir(directory) || throw(ArgumentError("No results directory at $(directory)."))
    folders = Tuple{Int, Int, String}[]
    for name in readdir(directory)
        m = match(_MEMBER_FOLDER, name)
        (m === nothing || !isdir(joinpath(directory, name))) && continue
        push!(folders, (parse(Int, m[1]), parse(Int, m[2]), name))
    end
    isempty(folders) && throw(ArgumentError("No constellation member folders (sat_<i>_id_<id>) under $(directory)."))
    sort!(folders)
    summary_path = joinpath(directory, "constellation_summary.csv")
    earlier = isfile(summary_path) ? CSV.read(summary_path, DataFrame) : nothing
    earlier_row(tag) = earlier === nothing ? nothing :
        (k = findfirst(==(tag), string.(earlier.member_tag)); k === nothing ? nothing : earlier[k, :])
    as_int16(v) = v === nothing || ismissing(v) ? missing : Int16(v)
    rows_of(folder) = begin
        position, id, tag = folder
        row = earlier_row(tag)
        # A member the summary records as failed has no rows, as in the ensemble's merge.
        row === nothing || row.success || return nothing
        located = (plane=as_int16(row === nothing ? nothing : row.plane), slot=as_int16(row === nothing ? nothing : row.slot))
        _disk_member_rows(joinpath(directory, tag), position, id, located)
    end
    # A first pass for the columns, the row counts and the last rows; the second
    # writes the members, reading each again.
    row_counts = Int[]
    last_rows = Union{Nothing, DataFrame}[]
    planes = Union{Missing, Int16}[]
    slots = Union{Missing, Int16}[]
    schema_tables = DataFrame[]
    for folder in folders
        rows = rows_of(folder)
        if rows === nothing
            push!(row_counts, 0); push!(last_rows, nothing)
            row = earlier_row(folder[3])
            push!(planes, as_int16(row === nothing ? nothing : row.plane))
            push!(slots, as_int16(row === nothing ? nothing : row.slot))
            continue
        end
        push!(row_counts, nrow(rows))
        push!(last_rows, nrow(rows) == 0 ? nothing : DataFrame(rows[nrow(rows):nrow(rows), :]))
        first_row = nrow(rows) == 0 ? nothing : rows[1, :]
        row = earlier_row(folder[3])
        push!(planes, first_row === nothing ? as_int16(row === nothing ? nothing : row.plane) : first_row.plane)
        push!(slots, first_row === nothing ? as_int16(row === nothing ? nothing : row.slot) : first_row.slot)
        # Only the names and types are kept.
        push!(schema_tables, rows[1:0, :])
    end
    columns = _members_columns(schema_tables)
    members_path = _write_members_feather!(joinpath(directory, "constellation_members.feather"), columns,
        (rows for rows in Iterators.map(rows_of, folders) if rows !== nothing))
    members = DataFrame(Arrow.Table(members_path); copycols=false)

    identity = DataFrame(
        spacecraft=Int32[folder[1] for folder in folders],
        spacecraft_id=Int64[folder[2] for folder in folders],
        plane=planes,
        slot=slots,
        member_tag=String[folder[3] for folder in folders],
    )
    status(tag, name, otherwise) = (row = earlier_row(tag); row === nothing ? otherwise : row[name])
    constellation = _constellation_summary(identity,
        Bool[status(folder[3], "success", row_counts[k] > 0) for (k, folder) in enumerate(folders)],
        Union{Missing, String}[status(folder[3], "error", missing) for folder in folders],
        Union{Missing, Float64}[status(folder[3], "elapsed_s", missing) for folder in folders],
        row_counts, last_rows)
    summary = SimulationModel.IOSerialization._atomic_write_file(summary_path, tmp -> CSV.write(tmp, constellation))
    return (members=members, constellation=constellation, files=[members_path, summary])
end

# A member folder's rows in the members table, from its results bundle, or
# `nothing` when it has none. `located` gives the plane and slot of a member
# written in the wide layout.
function _disk_member_rows(folder::String, position::Int, id::Int, located)
    manifest_path = joinpath(folder, "simulation_results.manifest.toml")
    long_path = joinpath(folder, "simulation_results_spacecraft.feather")
    wide_path = joinpath(folder, "simulation_results.feather")
    layout = isfile(manifest_path) ? get(TOML.parsefile(manifest_path), "layout", "wide") :
        isfile(long_path) ? "long" : "wide"
    if layout == "long"
        isfile(long_path) || return nothing
        run_path = joinpath(folder, "simulation_results_run.feather")
        spacecraft = DataFrame(Arrow.Table(long_path); copycols=false)
        run = isfile(run_path) ? DataFrame(Arrow.Table(run_path); copycols=false) : nothing
        return _constellation_member_rows(position, (spacecraft=spacecraft, run=run))
    end
    isfile(wide_path) || return nothing
    wide = DataFrame(Arrow.Table(wide_path); copycols=false)
    n = nrow(wide)
    spacecraft = DataFrame(spacecraft=fill(Int32(position), n), spacecraft_id=fill(Int64(id), n),
        plane=Vector{Union{Missing, Int16}}(fill(located.plane, n)),
        slot=Vector{Union{Missing, Int16}}(fill(located.slot, n)), time=wide[!, "time"]; copycols=false)
    run = DataFrame(time=wide[!, "time"]; copycols=false)
    for name in names(wide)
        name == "time" && continue
        startswith(name, "sc1_") ? (spacecraft[!, name[5:end]] = wide[!, name]) : (run[!, name] = wide[!, name])
    end
    return _constellation_member_rows(position, (spacecraft=spacecraft, run=ncol(run) > 1 ? run : nothing))
end
