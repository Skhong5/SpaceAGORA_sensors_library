# Studies and Benchmarks

Use this page when you need a repeatable study or benchmark, not just one
example run.

This page is for users generating verification reports, runtime measurements,
GRAM comparison outputs, or paper-study artifacts.

Inspect the benchmark command without starting a workload:

```text
julia --project=. src/interfaces/cli/main.jl benchmark runtime-analysis smoke --output-dir=output/perf_smoke --print-only
```

What to read next:

- [CLI](../cli.md)
- [Parallel Execution](parallel_execution.md)
- [Distributed and HPC](../distributed_hpc.md)
- [Simulation Outputs](outputs.md)

## Start here

After checking the prerequisites below, run a reduced benchmark workload:

```text
julia --project=. src/interfaces/cli/main.jl benchmark runtime-analysis smoke --output-dir=output/perf_smoke
```

Benchmark `smoke` profiles execute workloads, including simulations. Executing
the `runtime-analysis` or `smart-parallel-ladder` launcher needs the
`data/GRAMSuite.jl` submodule ([GRAMSuite Setup](gramsuite_setup.md)) because
both import the vendored package before parsing study options. GRAM-backed
cases also need the native GRAM library built. The CLI runs its child under the
repository project; instantiate that project's dependencies before execution.

Other direct studies have their own prerequisites. For example, the aerobraking
perturbation study's `--help` command loads SpaceAGORA and exits before GRAM
setup, worker creation or simulation.

If you need telemetry verification outputs, run:

```text
julia --project=. src/interfaces/cli/main.jl telemetry quick --output-dir=output/telemetry_cli --enforce=1
```

If you need parallel-policy comparison outputs, run:

```text
julia --project=. src/interfaces/cli/main.jl benchmark smart-parallel-ladder smoke --output-dir=output/smart_ladder_smoke
```

Use `--print-only` on any CLI-backed study to inspect the resolved Julia command
before running it.

## Choose by output

### Telemetry verification report

Use this when the goal is orbit-event or time-aligned telemetry validation.

```text
julia --project=. src/interfaces/cli/main.jl telemetry quick --output-dir=output/telemetry_cli --enforce=1
```

The command writes:

```text
telemetry_orbit_accuracy_summary.csv
telemetry_orbit_accuracy_errors.csv
```

Add `--plots=1` when plot artifacts are required.

### Runtime analysis report

Use this when the goal is timing, scaling, hardware metadata, or performance
regression artifacts.

```text
julia --project=. src/interfaces/cli/main.jl benchmark runtime-analysis quick --output-dir=output/performance
```

Use `smoke` first on a new machine. Use `full` only when you need full study
artifacts.

### Smart parallel ladder

Use this when the goal is comparing serial, threaded, process, and adaptive
parallel-policy rungs.

```text
julia --project=. src/interfaces/cli/main.jl benchmark smart-parallel-ladder smoke --output-dir=output/smart_ladder_cli
```

### Aerobraking perturbation Monte Carlo

Use this when the goal is the aerobraking perturbation study rather than one
mission example.

```text
julia --project=. benchmarks/studies/aerobraking_perturbation_mc/main.jl --smoke --output-dir output/aerobraking_perturbation_smoke
```

For the full option list:

```text
julia --project=. benchmarks/studies/aerobraking_perturbation_mc/main.jl --help
```

## Profiles

| Profile | Use for |
|---|---|
| `smoke` | Reduced workload after its prerequisites are ready |
| `quick` | Development and regression artifacts |
| `full` | Full study or paper-grade artifacts |

## Direct study index

Use the CLI when a workflow is exposed there. Use a direct study script when the
study has its own option set, resume behavior, or output layout.

Run [GRAMSuite Setup](gramsuite_setup.md) before using GRAM-backed studies.

| Area | Scripts |
|---|---|
| GRAM fidelity | `gram_full_vs_trajectory_density_compare.jl`, `gram_interpolation_vs_point_to_point_analysis.jl`, `gram_single_call_vs_point_to_point_analysis.jl`, `gram_offline_db_accuracy_study.jl`, `gram_planet_rho_altitude_sweep.jl`, `gram_real_sim_runtime_compare.jl`, `gram_real_sim_surrogate_matrix.jl`, `gram_real_sim_surrogate_decision_table.jl`, `gram_trajectory_lookahead_fidelity/main.jl` |
| GRAM Mars fix and constellation scaling | `gram_mars_fix_and_constellation_scaling/{mars_150km_gram_scaling.jl, earth_standard_gram_scaling.jl, earth_surrogate_gram_scaling.jl, leo_constellation_size_scaling.jl, leo_ensemble_outer_backend_scaling.jl, leo_thread_scaling_by_mode.jl, mc_multisat_process_backend.jl, mc_multisat_thread_allocation.jl}` — see the directory's `README.md` for the Mars ephemeris-bypass fix background and the run procedures and interpretation limits |
| Performance support | `performance_effector_reduction_microbench.jl`, `performance_split_imex_compare.jl`, `performance_static_vs_parallel.jl`, `performance_thread_scaling_1024.jl`, `performance_thread_scaling_64_aero_gram.jl`, `performance_hybrid_scaling_64_aero_gram.jl`, `performance_mc_thread_scaling.jl`, `performance_smart_parallel_ladder_cross_machine.jl` |
| Parallelization paper benchmarks | `paper_parallelization_benchmarks.jl`, `parallelization_performance.jl` — each has a same-named subdirectory with its own `protocol.sh`, `cli.jl`, and `CASES.md` case catalog |
| Tuning and validation | `telemetry_hybrid_tuner.jl`, `telemetry_odyssey_tuner.jl` |

The former `SpaceAGORACalibration.jl/` campaign package and its `bin/` drivers
have been retired. Use the tuning scripts listed above for telemetry parameter
fitting, or [Verification Study](verification_study.md) to compare simulations
with observations. Automatic performance tuning for parallel execution remains
available.

For long studies, set an explicit output directory and keep the terminal log with
the artifacts.

## Research records and generated results

The public repository keeps runnable studies, their inputs, measurement methods
and interpretation limits. Session notes, timing tables, profiler captures and
generated plots belong with the research record rather than the source tree.
Keep the source revision, configuration, host, logs and raw outputs together
when recording a new result; a historical result does not validate current code.

The originals removed during the October 14, 2026 cleanup are retained in the
private research archive under their original repository paths. Authorized
collaborators can obtain the archive location and verification manifest from
the maintainers. The record identifies its source revision and any unavailable
underlying inputs; a copied report is not a replacement for missing raw data.
The public procedures remain usable without access to those historical records.

## Study tools and shared maintenance

The paper-routing study keeps its table builders, plots, raw-run policy checks and
run comparison beside its runner in
`benchmarks/studies/paper_parallelization_benchmarks/`. Use
`check_policy_criterion.py` with the raw per-repeat CSV when assessing failures.
`make_paper_routing_tables.py` and `make_paper_routing_plots.py` read aggregated
successful-run timings; the regret table and adaptive comparison use raw runs.
Their different inputs serve different questions, so they are not interchangeable.
The study's `CASES.md` lists the measurement protocol and analysis commands.

`benchmarks/scripts/archive_paper_run.py` is shared by the paper harnesses. It
copies run data to an external archive and maintains that archive's manifest,
index and provenance record. It does not publish results into this repository.
Select the destination with `--archive` or `SPACEAGORA_PAPER_ARCHIVE`.

Shared installation, calibration and verification commands remain in `scripts/`.
Development probes live in `scripts/dev/`, including the native-lock thread
ladder and its contention helpers. The flat benchmark launchers used by the CLI
remain stable entry points; their implementations stay in the corresponding
study directories.
