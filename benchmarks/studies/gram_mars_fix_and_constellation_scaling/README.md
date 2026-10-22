# GRAM Mars Fix and Constellation Size Scaling

These scripts exercise native-GRAM ephemeris setup, constellation-size scaling
and thread/process allocation. Build the native library and provide the relevant
SPICE kernels before running GRAM cases; see
[GRAMSuite setup](../../../docs/src/user/gramsuite_setup.md). Historical
investigations, timing tables and plots follow the
[research-record policy](../../../docs/src/user/studies_benchmarks.md#research-records-and-generated-results).

## Mars ephemeris setup

The SpaceAGORA GRAMSuite extension supplies `_gram_spice_ephemeris_state`
through GRAMSuite's `_GRAM_EPHEMERIS_STATE_FN` hook. It computes the required
solar geometry with SpaceAGORA's SPICE bindings and supplies it through
`set_ephemeris_state!`. The hook defaults to `nothing` when the extension is
not loaded. This avoids relying on GRAM's own kernel set for that calculation;
it does not remove the need for valid kernels in the calling application.

Use the Mars case to exercise that path. A native error concerning longitude
of the Sun is a reason to inspect extension loading, kernel coverage and the
ephemeris hook, not evidence that the whole scaling run is valid.

```bash
julia --project=. --threads=4 benchmarks/studies/gram_mars_fix_and_constellation_scaling/mars_150km_gram_scaling.jl parallel
```

## Constellation size scaling

`leo_constellation_size_scaling.jl` sweeps LEO constellation size (1, 2, 4,
..., 1024 satellites -- powers of 2) **and** thread count (default 1, 2, 4,
8, 16, 32, 64 -- capped at 64) at a fixed 600s mission, comparing:

- **standard**: real/native GRAM, no vacuum-predicted lookahead cache
  (`SPACEAGORA_DENSITY_FREEZE_PER_STEP=1`, density calls serialized)
- **surrogate**: the GRAM offline surrogate (`GRAMAtmosphereModelSurrogate`)
  -- precomputed interpolation grid, no native calls, no lock
- **no_gram**: `NoAtmosphereModel()` with the aerodynamic effector dropped
  entirely -- baseline gravity-only dynamics cost, isolating
  constellation-size scaling with zero atmosphere-model overhead

All 33 `(N_SATS, mode)` points for one thread count run back-to-back in ONE
process (`leo_constellation_size_scaling_point.jl`'s `run_scaling_point`) --
`ODEParams`/`SharedBuffers`'s `n_sats` is a runtime field
(`src/orchestration/simulation/state/runtime_types.jl`), so every `N_sats` value for a given mode
shares one compiled specialization and only the 3 modes still differ in
type. Thread count, however, is fixed at Julia process startup, so each
thread count in the ladder still runs as its own subprocess (this script
re-invoking itself with `--threads=<N>`); a crashed thread-count subprocess
only drops that thread count from the merged results rather than aborting
the rest of the ladder.

Each point also records its own resource footprint -- peak RSS and mean/peak
CPU%, sampled from that process only via `ps -p <pid>` polling
(`resource_monitor.jl`). This keeps other processes out of the recorded
resource totals; contention can still affect utilization and elapsed time.

```bash
julia --project=examples benchmarks/studies/gram_mars_fix_and_constellation_scaling/leo_constellation_size_scaling.jl
```

Override the thread ladder (comma-separated, values must be <= 64):

```bash
SPACEAGORA_SCALING_THREADS=1,8,64 julia --project=examples benchmarks/studies/gram_mars_fix_and_constellation_scaling/leo_constellation_size_scaling.jl
```

Produces `leo_constellation_size_scaling_with_gram.png` (standard vs.
surrogate, both GRAM-derived, one small-multiple panel per thread count) and
`leo_constellation_size_scaling_without_gram.png` (the no-GRAM baseline, one
line per thread count), each with a dashed O(N) linear-scaling reference line
anchored at N=1 for visually judging super-/sub-linear scaling, plus
`leo_constellation_size_scaling_resource_ram.png` and
`leo_constellation_size_scaling_resource_cpu.png` (peak RSS / mean CPU%, one
panel per mode, one line per thread count).

Look-ahead caching adds trajectory-generation and interpolation setup. Include
that setup in a comparison and vary mission duration before concluding that a
cache is beneficial. `earth_standard_gram_scaling.jl` and
`earth_surrogate_gram_scaling.jl` provide single-point comparisons with
`LEO_SCALING_N_SATS` selecting the constellation size.

## Thread allocation and process routes

`leo_ensemble_outer_backend_scaling.jl` compares `:threads` and `:process`
for a fixed 64-spacecraft native-GRAM ensemble:

```bash
julia --project=examples benchmarks/studies/gram_mars_fix_and_constellation_scaling/leo_ensemble_outer_backend_scaling.jl
```

Its outputs include `leo_ensemble_outer_backend_scaling_summary.csv` and the
corresponding plot. Native-GRAM construction and calls require the repository's
locking/isolation rules. Process workers own separate native state, at a memory
and startup cost that must be included in the comparison.

`mc_multisat_thread_allocation.jl` varies outer workers, inner threads and the
outer backend for a campaign. `mc_multisat_process_backend.jl` isolates the
process backend. Read the status of every row: timeouts, failed worker startup
and incomplete points are not runtime measurements. Warm the actual sample
closure before timing; generic package loading alone does not warm its solver
specialization.

Nested outer/inner runs must respect the runtime's outer-active guard and
resource budget. Do not disable those guards to reproduce an old timing.
See [parallel execution](../../../docs/src/user/parallel_execution.md) for
current routing controls. Keep generated CSVs, plots and logs with the run
record rather than committing them beside the scripts.
