"""
    SimulationResults

What `run_simulation(args; return_results=true)` returns.

- `table::DataFrame`: the results table, same columns and values as the results
  CSV, built in memory even when `simulation_settings.results=false`. A resumed
  run's table holds the whole run, from the rows kept beside its checkpoint.
  With `results_layout=:long` it is the spacecraft table: one row per spacecraft
  and saved time (see [`long_table`](@ref)). A long-layout run that streams its
  results at each checkpoint reads it from its Feather file in place, with
  read-only columns; `DataFrame(results.table)` copies it.
- `configuration`: the configuration that ran. Under the default
  `isolate_state=true` this is the deep copy, so mutated controller state and
  logs (for example the RPO command log) are reachable here and not on the
  caller's object. A resumed run whose checkpoint holds no usable rows writes
  to `resumed_at_<t>s`, and its configuration's `results_directory` names that
  folder.
- `files::Vector{String}`: paths of existing files the run wrote (results CSV,
  bundle, scene, checkpoint); empty when none.
- `solution`: the ODE solution when `return_solution=true` was also passed,
  otherwise `nothing`.
- `run_table`: with `results_layout=:long`, the run table (one row per saved
  time, `time` and the run-wide fields), or `nothing` when the run has none;
  `nothing` in the wide layout. Read it with [`run_table`](@ref).
"""
struct SimulationResults{C, S}
    table::DataFrame
    configuration::C
    files::Vector{String}
    solution::S
    run_table::Union{Nothing, DataFrame}
end

SimulationResults(table::DataFrame, configuration, files::Vector{String}, solution) =
    SimulationResults(table, configuration, files, solution, nothing)

# Set by `run_constellation_ensemble(merge_results=true)` around each member's
# run. The run's final results save puts its tables in the long layout there,
# built by the long builder from the saved rows, whichever layout the run writes
# and returns.
const _LONG_TABLES_SINK = Base.ScopedValues.ScopedValue{Union{Nothing, Base.RefValue{Any}}}(nothing)

"""
    run_table(results::SimulationResults) -> Union{Nothing, DataFrame}

The run table of a run with `results_layout=:long`: one row per saved time,
`time` and the run-wide fields (such as `sun_dir_1..3`). `nothing` for a wide
run, whose table holds those columns, and for a long run without run-wide
fields.
"""
run_table(results::SimulationResults) = results.run_table

"""
    long_table(results::SimulationResults) -> DataFrame

The spacecraft table in the long layout: one row per spacecraft and saved time,
with `spacecraft` (its position in the configuration), `spacecraft_id`, `plane`
and `slot` (from its constellation slot, `missing` without one) and `time` first,
then each per-spacecraft column without its `sc<i>_` prefix. Rows are
time-major. A run with `results_layout=:long` returns its table as it is. For a
wide run the table is rearranged from the `sc<i>_` columns of its spacecraft; a
column that some spacecraft lack (thruster levels of a spacecraft without
thrusters) is `missing` for them. Its run-wide columns stay in `results.table`.
"""
function long_table(results::SimulationResults)::DataFrame
    results.configuration.simulation_settings.results_layout === :long && return results.table
    wide = results.table
    spacecraft = results.configuration.dynamics_model.spacecraft
    num_sats = length(spacecraft)
    n_times = nrow(wide)
    prefix(i) = "sc$(i)_"
    # Every spacecraft's columns, in the order first seen: spacecraft can differ.
    fields = String[]
    for name in names(wide)
        m = match(r"^sc(\d+)_(.*)$", name)
        m === nothing || !(1 <= parse(Int, m[1]) <= num_sats) || m[2] in fields || push!(fields, m[2])
    end
    slot_part(sc, part) = sc.constellation_slot === nothing ? missing : Int16(getproperty(sc.constellation_slot, part))
    long = DataFrame(
        spacecraft=repeat(Int32.(1:num_sats), n_times),
        spacecraft_id=repeat(Int64[sc.id for sc in spacecraft], n_times),
        plane=repeat(Union{Missing, Int16}[slot_part(sc, :plane) for sc in spacecraft], n_times),
        slot=repeat(Union{Missing, Int16}[slot_part(sc, :slot) for sc in spacecraft], n_times),
        time=repeat(Vector{Float64}(wide.time); inner=num_sats),
    )
    for field in fields
        columns = [prefix(i) * field in names(wide) ? wide[!, prefix(i) * field] : nothing for i in 1:num_sats]
        long[!, field] = [columns[sat] === nothing ? missing : columns[sat][row] for row in 1:n_times for sat in 1:num_sats]
    end
    return long
end
