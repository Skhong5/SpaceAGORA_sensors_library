# paper_scenarios — JAIS Parallelization Paper Benchmark Suite (S1–S5)

The S1–S5 harness compares constellation size, atmosphere routes, Monte Carlo
worker counts, hybrid allocation and adaptive routing. All scenarios use Earth
with Earth GRAM or the documented atmosphere controls and GGM05C L20/L50
harmonics. These are questions to measure on a recorded revision and host, not
performance guarantees.

| Scenario | Script | Comparison |
|---|---|---|
| S1 | `s1_constellation_scaling.jl` | constellation size and thread budget |
| S2 | `s2_gram_atmosphere_modes.jl` | direct native GRAM, look-ahead caching and process routes |
| S3 | `s3_montecarlo_process_scaling.jl` | process-worker count, campaign throughput and memory |
| S4 | `s4_hybrid_mc_constellation.jl` | outer sample and inner constellation allocation |
| S5 | `s5_routing_profile_ladder.jl` | adaptive profiles and fixed-route baselines |

Historical results and review notes follow the
[research-record policy](../../../docs/src/user/studies_benchmarks.md#research-records-and-generated-results).

## Running

Each point is a separate `julia --threads=N` subprocess (thread count is fixed at
Julia startup), spawned by the controller and measured by `scenario_worker.jl`.

```bash
# Individual scenarios
julia --project=. benchmarks/studies/paper_scenarios/s1_constellation_scaling.jl
julia --project=. benchmarks/studies/paper_scenarios/s2_gram_atmosphere_modes.jl
julia --project=. benchmarks/studies/paper_scenarios/s3_montecarlo_process_scaling.jl
julia --project=. benchmarks/studies/paper_scenarios/s4_hybrid_mc_constellation.jl
julia --project=. benchmarks/studies/paper_scenarios/s5_routing_profile_ladder.jl

# Everything (paper protocol order; S2/S3 are the long ones)
bash benchmarks/studies/paper_scenarios/run_all.sh
```

Results land in `results/<hostname>/sN_*.csv` plus one `.log` per point. Run the
suite on each paper machine (M3 Pro MacBook Pro, Zen 5 Ryzen 9 / 64 GB,
Threadripper 64-thread / 256 GB); the hostname keys the per-machine tables.

### Running on a remote box

`scripts/remote/spaceagora-remote` mirrors this repo to a remote host and runs
a job there detached (config: `scripts/remote/remotes.conf`, gitignored — copy
`scripts/remote/remotes.conf.example`). It syncs the full local working tree
(gitignore-aware, so uncommitted edits and untracked scripts ship too — not
just `git ls-files`) and runs `Pkg.instantiate()` before every job, so a stale
shared package depot fails fast with one clear log instead of every scenario
point silently erroring on a missing package. GRAM/SPICE data (~20G) is opt-in
via `spaceagora-remote seed-data --remote <name>` (or `push --sync-data`) — run
it once per remote before the first job that needs it. Its auto-pull only
captures a release's top-level `output/`/`results/` dirs, not the nested
`benchmarks/studies/paper_scenarios/results/<hostname>/` path this suite
writes to, so pull that manually once the job finishes:

```bash
rsync -az <ssh-alias>:<remote-base>/releases/<job-id>/benchmarks/studies/paper_scenarios/results/<hostname>/ \
  benchmarks/studies/paper_scenarios/results/<hostname>/
```

## Knobs

| Env var | Meaning | Default |
|---|---|---|
| `PS_THREADS` | Parallel thread budget per point — **set to physical core count for paper numbers**. S1 also accepts a comma-separated ladder (e.g. `1,2,4,8,12`): serial runs once per size regardless of ladder length (thread-count-independent), and each ladder value adds one more `parallel` row for that size, distinguished by `julia_threads` — a full thread-count sweep is one invocation/CSV write, not one per thread value | `Sys.CPU_THREADS` |
| `PS_REPEATS` / `PS_WARMUP` | Timed repeats / warmup solves per point | 3 / 1 |
| `PS_MISSION_S` | Mission length override | 1800 s (S1), 600 s (S2–S5) |
| `PS_SIZES` | Constellation size ladder (S1, S2) | `1,4,16,64,256,1024,2048,4096` / `4,16,64` |
| `PS_WORKERS` | S3 process-worker ladder | `1,2,4,8` (extend to `16,32` on the Threadripper) |
| `PS_MC_SAMPLES` | Samples per campaign (S3, S4) | 32 / 8 |
| `PS_PROC_WORKERS` | S2 process-mode worker count | `min(8, PS_THREADS)` |
| `PS_TIMEOUT_S` | Per-point kill deadline | 3600 s |
| `PS_GRAVITY` / `PS_DENSITY` | S1-only: force-model override (`invsq\|l20\|l50` / `none\|gram_standard\|gram_lookahead\|gram_surrogate`), isolates per-satellite cost as a variable | `l20` / `none` |
| `PS_NO_SPICE` | Build the workload's Earth from the built-in constants and `SimpleEphemeridesModel` instead of the SPICE-backed constructor. Set automatically when the SPICE bundle is absent, so a baseline no-GRAM checkout runs S1 unchanged; set it explicitly on a machine that does have kernels to reproduce that path. Treat this as a different ephemeris configuration when comparing results. The `spice` column in the CSV records which path produced each row. A `PS_DENSITY=gram*` mode still requires the real bundle and errors without it | auto |
| `PS_RESULTS_SUFFIX` | Appended to `results/<hostname>` — use for exploratory runs (e.g. a thread-count sweep, or a one-off thread-budget override) so they land in their own pseudo-host directory instead of overwriting the canonical per-host baseline CSV | (empty) |

For a reduced smoke workload, set
`PS_SIZES=1,4 PS_MC_SAMPLES=4 PS_WORKERS=1,2 PS_MISSION_S=60 PS_REPEATS=1`.
Use the recorded mission length when comparing full runs, and inspect whether
the measured work is long enough to separate it from startup and dispatch costs.

## Interpreting the CSVs

- `median_s`/`min_s`/`max_s`/`times_s` — wall time over `PS_REPEATS` (median is the
  paper metric). Warmups are excluded; inspect repeats for residual compilation
  or first-use work instead of assuming one warmup removes all of it.
- `maxrss_mb` — measured process peak RSS (`Sys.maxrss`), the memory column.
- `workers_rss_mb` — summed peak RSS of Distributed pool workers (process modes
  only): the RAM cost of process-level parallelism that S2's shared-instance
  thread route avoids.
- Speedup for S1/S2 = serial median / mode median at the same size. S3's headline
  is per-sample time (`median_s / samples`) versus worker count.
- `PS_GRAVITY`/`PS_DENSITY` overrides in S1 write a suffixed CSV
  (`s1_constellation_scaling_<gravity>_<density>.csv`) instead of overwriting the
  (`l20`, vacuum) baseline; `plot_results.jl` picks up every such file automatically
  and tags rows with a `variant` column. S1 selects `rhs_mode=serial` for
  the baseline and `flat` for the parallel run. The resulting ratio therefore
  includes an execution-path change as well as a thread-count change. Report
  that baseline convention; the ratio alone does not establish superlinear
  hardware scaling or a cache explanation.

For a thread-count sweep, `plot_results.jl` can join several parallel rows at
one constellation size into a single line. Treat those plots as diagnostics
unless the thread axis is explicitly represented. Use `PS_RESULTS_SUFFIX` to
keep exploratory configurations separate from a recorded baseline.

## GRAM safety conventions (inherited from prior studies)

- `gram_standard` (direct native GRAM): density frozen per accepted step
  (`SPACEAGORA_DENSITY_FREEZE_PER_STEP=1`; per-RK-stage perturbation noise
  otherwise collapses adaptive dt) and density callback kept serial.
- `gram_lookahead`: the study sets the vacuum-predicted cache horizon and
  deviation limits beyond the mission reach to avoid rebuilds during the planned
  run. The existing study comments attribute these settings to reported native
  rebuild hangs with multiple satellites and cite
  `project_gram_multisat_rebuild_bug`; that cited source is not included in this
  checkout. See [the study configuration](common.jl) for the retained settings;
  this note does not report a new native validation.
- Process modes: each pool worker is `--threads=1` with its own native GRAM; the
  coordinator only dispatches.
