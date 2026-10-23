using ..SimulationModel.SimConfig: _with_fields
import ..SimulationModel
import ..SimulationEngine
using Arrow: Arrow
using CSV: CSV
using DataFrames: DataFrame, nrow

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
  order, then saved time. A failed member has no rows.
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

The members table holds every saved row of every member in memory. Each member's
rows come back from it whole and are released as they are merged, so while the
merge runs the members' rows are held about once. For large campaigns,
`SPACEAGORA_RESULTS_THIN_STRIDE` and `SPACEAGORA_RESULTS_FINAL_ONLY` thin each
member's rows.
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
# row per row of its spacecraft table. When the member's returned value holds
# these tables (the long layout with `return_results=true`), they are copied
# first, so that the value stays what `run_simulation` returned.
function _constellation_member_rows(position::Integer, outcome)::DataFrame
    value = hasproperty(outcome, :value) ? outcome.value : nothing
    shared = value isa SimulationEngine.SimulationResults &&
        (value.table === outcome.spacecraft || (outcome.run !== nothing && value.run_table === outcome.run))
    rows = shared ? copy(outcome.spacecraft) : outcome.spacecraft
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
        rows[!, name] = shared ? copy(run[!, name]) : run[!, name]
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
# position order, whatever order they finished in. As each member is merged, its
# sample in `campaign` is replaced by one holding only the caller's value, so its
# tables are released once appended: the peak is the merged table, the member
# being appended and the members not yet merged.
function _constellation_ensemble_result(args::SimulationConfiguration, member_configs, campaign::MonteCarloResult)
    spacecraft = args.dynamics_model.spacecraft
    count = length(spacecraft)
    slot_part(sc, part) = sc.constellation_slot === nothing ? missing : Int16(getproperty(sc.constellation_slot, part))
    members = nothing
    last_rows = Union{Nothing, DataFrame}[]
    row_counts = Int[]
    for (k, sample) in enumerate(campaign.samples)
        position = sample.index
        if sample.success
            outcome = sample.value
            rows = _constellation_member_rows(position, outcome)
            push!(row_counts, nrow(rows))
            push!(last_rows, nrow(rows) == 0 ? nothing : rows[nrow(rows):nrow(rows), :])
            if members === nothing
                members = rows
            else
                append!(members, rows; cols=:union, promote=true)
            end
            # The sample now holds only the caller's value, here and in
            # `successful`, so nothing keeps this member's tables.
            released = _with_fields(sample; value=outcome.value)
            campaign.samples[k] = released
            j = findfirst(s -> s.index == position, campaign.successful)
            j === nothing || (campaign.successful[j] = released)
        else
            push!(row_counts, 0)
            push!(last_rows, nothing)
        end
    end
    members === nothing && (members = DataFrame(spacecraft=Int32[], spacecraft_id=Int64[],
        plane=Union{Missing, Int16}[], slot=Union{Missing, Int16}[], time=Float64[]))

    positions = [sample.index for sample in campaign.samples]
    constellation = DataFrame(
        spacecraft=Int32.(positions),
        spacecraft_id=Int64[spacecraft[k].id for k in positions],
        plane=Union{Missing, Int16}[slot_part(spacecraft[k], :plane) for k in positions],
        slot=Union{Missing, Int16}[slot_part(spacecraft[k], :slot) for k in positions],
        member_tag=String[_constellation_member_tag(member_configs[k], k, count, spacecraft[k].id) for k in positions],
        success=Bool[sample.success for sample in campaign.samples],
        error=Union{Missing, String}[sample.success ? missing : sprint(showerror, sample.error)
                                     for sample in campaign.samples],
        elapsed_s=Float64[sample.elapsed_s for sample in campaign.samples],
        rows=row_counts,
    )
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

    result_campaign = MonteCarloResult(campaign.samples, campaign.elapsed_s, campaign.threads;
                                       route=campaign.route, local_slots=campaign.local_slots)
    files = _write_constellation_results!(args, members, constellation)
    return ConstellationEnsembleResult(result_campaign, members, constellation, files)
end

# The merged files, in `results_directory` beside the member folders, when the
# ensemble saves results: the members table as the members write theirs (the
# feather with the results bundle, the CSV with `save_csv`), and the
# constellation table as `constellation_summary.csv`.
function _write_constellation_results!(args::SimulationConfiguration, members::DataFrame, constellation::DataFrame)
    settings = args.simulation_settings
    settings.results || return String[]
    directory = settings.results_directory
    mkpath(directory)
    write(path, writer) = SimulationModel.IOSerialization._atomic_write_file(path, writer)
    files = String[]
    SimulationEngine._typed_save_bundle_enabled() &&
        push!(files, write(joinpath(directory, "constellation_members.feather"), tmp -> Arrow.write(tmp, members)))
    settings.save_csv &&
        push!(files, write(joinpath(directory, "constellation_members.csv"), tmp -> CSV.write(tmp, members)))
    push!(files, write(joinpath(directory, "constellation_summary.csv"), tmp -> CSV.write(tmp, constellation)))
    return files
end
