# Precompile workload for the paper benchmark harness

Every point of the paper harness is a fresh Julia process
(`ppc_worker_cmd` in `../../parallelization_performance/execution.jl`), and so is
every process-pool worker a Monte Carlo point starts. Each one loads the packages
and then compiles the solver and right-hand-side specializations of the case it
is about to run before it can time anything. This directory builds a package
image that already holds those specializations, and the harness loads it into
every worker when asked (`SPACEAGORA_PPB_WORKLOAD=auto` or `1`).
It is off by default. Loading a package image changes code layout as well as
startup work, so validate each build and keep rows made with and without the
workload in separate comparisons. Historical timings and build investigations
follow the [research-record policy](../../../../docs/src/user/studies_benchmarks.md#research-records-and-generated-results).

## What it covers

`SpaceAGORAPaperWorkload/src/points.jl` derives the point list from the harness's
own definitions: every case of every P phase (`PAPER_BENCHMARK_PHASES` in
`../cli.jl`, P1 through P7, P5f included) under every mode the phase runs it with, plus the
phase's parity points. Each case is built by `ppc_single_config` at the size its
name says (ComponentArrays puts the satellite count into the state vector's type,
so each size is its own specialization), under the mode's route environment
(`ppc_mode_env_pairs`), through the same worker entry points a harness worker
calls. Campaign cases run two samples, which reaches the Monte Carlo runner
(`run_monte_carlo` with `threads=:auto`, including the policy_v2 and predictive
planners) and the pool worker's sample task. A case added to a P phase is picked
up without an edit; `test/gates/ci_ppb_workload_coverage_gate.jl` fails if a
P-phase (case, mode) is ever missing.

The workload runs each point on the harness's `test` profile, whose mission is
10 s for every P-phase case (ASSUMED: long enough to take several steps at every
case's step cap, short enough that the 4096-spacecraft cases stay cheap; only the
types matter). At the time of writing that is 90 points: 72 for P1-P6p and 18
for P7's three cases under its six modes. P5f adds none: its two cases and five
modes are all P5's, and a point is a (case, mode) pair, so the full-budget
predictive run is the same specialization as P5's predictive points (the thread
and worker counts are not in any type).

What it leaves out, and why:

- **Native-GRAM cases** (P6 trace 5, `aero_4096sat_l50_gram_lookahead_100s`, and
  P6p, `aero_4096sat_l50_gram_process_100s`; in general every case matched by
  `PPC_GRAM_LIVE_CASES` in `cases.jl`). Warming them would need GRAMSuite as a
  dependency of the workload package, which makes the image unloadable on a
  machine without the GRAM build, and it would run native GRAM inside the
  precompile process: GRAMSuite includes its native wrapper module on first
  construction, an evaluation into a closed module during incremental
  compilation, and any model reachable from the workload module's globals would
  be serialized into the image with its native handles. This is the same class
  of hazard `src/precompile_workload.jl` describes for SPICE furnish state. The
  harness launches these points without the workload.
- **The harness's own functions and closures.** A worker runs the harness code in
  its `Main`; the image holds the specializations of SpaceAGORA and its
  dependencies that the harness calls into (the solve on each configuration
  type, the ODE integrator on each state type, the campaign planner, CSV writing
  of the row type), not `Main`'s functions. Those functions still compile in each worker process.
- **Threaded code paths as such.** Julia precompiles single-threaded, so the
  bodies of the tasks a threaded route spawns are compiled at run time; the
  per-satellite kernels they call are in the image.

What the workload keeps out of the image: it runs with its working directory and
every runtime state path (`SPACEAGORA_OUTER_ROUTE_STATE_PATH`,
`SPACEAGORA_PARALLEL_POLICY_STATE_PATH`, `SPACEAGORA_COST_CONSTANTS_PATH`,
`SPACEAGORA_RHS_CALIBRATION_PATH`, `SPACEAGORA_CAMPAIGN_CORRECTIONS_PATH`) in a
scratch directory, so building never reads or writes this machine's learned
routing state; it caps every mode's process pool at one worker so nothing is
spawned during precompilation, and fails the point if a worker appears; and it
resets its own copies of the harness caches and SpaceAGORA's furnish set before
the image is written.

## Building

```bash
bash benchmarks/studies/paper_parallelization_benchmarks/workload/build_workload.sh
```

Three stages (`setup`, `precompile`, `check`; `all` is the default):

1. `setup_workload_env.jl` creates `output/paper_workload/env`: the repository's
   exact `Manifest.toml` plus path entries for SpaceAGORA and this package, added
   offline with `PRESERVE_ALL`; it fails if any package version moved or anything
   beyond the workload package was added. The repository's own `Project.toml`
   and `Manifest.toml` are never written.
2. Precompiles `SpaceAGORAPaperWorkload` (which runs the workload) in a
   `systemd-run --user --scope -p MemoryMax=16G -p MemorySwapMax=0` scope when
   systemd provides one, under `/usr/bin/time -v`. It rebuilds SpaceAGORA's own
   image first if that is stale, which is the image the harness loads anyway.
3. Asks, the same way the harness does, whether the image is current.

Environment: `SPACEAGORA_PPB_WORKLOAD_ENV` (environment directory),
`SPACEAGORA_PPB_WORKLOAD_MEMORY_MAX` (scope cap), `SPACEAGORA_PPB_WORKLOAD_SCOPE=0`
(no scope), `SPACEAGORA_PPB_WORKLOAD_LOG` (log directory).

The image is invalidated by anything that invalidates SpaceAGORA's image and by
any edit to the harness files it includes (`cli.jl`, `modes.jl`, `cases.jl`,
`trajectory_parity.jl`, `reporting.jl`, `execution.jl`, the paper `cli.jl`), so
rebuild after changing any of them. A stale image is never used; see below.

## How the harness uses it

The environment is stacked behind the repository project rather than replacing
it. A worker keeps `--project=<repo>`, so it loads the same SpaceAGORA image,
resolves every other package the same way and has the same active project
(which the runtime's process pool and state paths key on) as a worker launched
without the workload; the environment only adds the workload package, second on
`JULIA_LOAD_PATH`.

- **Controller** (`ppc_run_controller`): once per run, `ppc_workload_env()`
  starts a probe process exactly the way a worker is started and asks
  `Base.isprecompiled` about the workload package. That fails whenever SpaceAGORA
  or any other dependency changed since the image was built. When the image is
  current, `ppc_worker_cmd` adds `JULIA_LOAD_PATH="@:<env>:@v#.#:@stdlib"`,
  `SPACEAGORA_PPC_WORKLOAD=1` and `SPACEAGORA_PROCESS_WORKER_PRELOAD` to every
  worker it launches, except for the native-GRAM cases.
- **Per-point worker**: `parallelization_performance/cli.jl` loads the package
  when `SPACEAGORA_PPC_WORKLOAD=1`, after checking again that the image is
  current (a stale image would otherwise be rebuilt inside the point).
- **Harness process pool** (`ppc_ensure_process_workers!`): pool workers inherit
  the worker's environment, and load the package through the same `cli.jl`
  include. They load GRAMSuite only when the coordinator has it, i.e. only for a
  native-GRAM case: the GRAM extension's methods invalidate SpaceAGORA's
  compiled solve path.
- **Runtime process pool** (`SpaceAGORA.ParallelProcess.ensure_process_workers!`):
  new workers load every package named in `SPACEAGORA_PROCESS_WORKER_PRELOAD`
  right after SpaceAGORA, when it is precompiled for their environment. Their
  load path already carries the workload environment (`_process_worker_load_path`
  copies the coordinator's).

Turning it on: `SPACEAGORA_PPB_WORKLOAD=auto` uses the image when it is current
and otherwise runs without it; `SPACEAGORA_PPB_WORKLOAD=1` (or `on`) makes a
missing or stale image an error. The default, `0`, never loads it.
The controller prints `precompile_workload=<env>` or `precompile_workload=off`
at the start of every run.

Remote jobs: `scripts/remote/spaceagora-remote push` builds the image once per
job, after `Pkg.instantiate()` and before the command, when asked
(`--workload auto|on|off`, default `off`; `auto` builds it when the command runs the paper harness; the log is `workload_build.log` in the
job directory and the exit code `WORKLOAD_BUILD_EXIT` in `job.meta`). The
package name starts with `SpaceAGORA`, so the runner's per-job purge of
`shared_depot/compiled/v*/SpaceAGORA*` removes it together with SpaceAGORA's
image, the environment lives in the release's own `output/`, and the harness's
freshness probe refuses any image built against a different SpaceAGORA. A failed
build costs startup time, never correctness.

## Validating

```bash
julia --project=. --startup-file=no \
  benchmarks/studies/paper_parallelization_benchmarks/workload/validate_workload.jl <out_dir> --pairs=3
```

launches five representative harness workers (a P1 single-spacecraft point, a
P2 4096-spacecraft point, P3 on the process route and on policy_v2, and a P5
grid point) through `ppc_worker_cmd`, with and without the workload, and writes
`workload_validation.csv`. It uses two opt-in hooks in the harness:
`SPACEAGORA_PPC_STARTUP_TRACE=1` prints a time stamp when a worker has loaded,
when its first solve returns and when its first timed repeat starts, and
`SPACEAGORA_PPC_DUMP_STATE_DIR=<dir>` writes every sample's step times and final
state as raw Float64 for a byte comparison. Run it on a quiet machine, inside a
memory-capped scope, with nothing else timing.

## Interpretation and assumptions

Compare startup, timed solves, allocations and full saved states separately.
Faster startup does not establish unchanged timed throughput. Repeat the
stock/workload comparison after rebuilding an image, preserve the build and
host metadata, and retain all launch pairs rather than selecting a favorable
build. No previous build's timing or state comparison qualifies a new one.

The workload assumes that the `test` profile's 10-second mission and two
samples per campaign reach the needed specializations. The build's default
16 GiB scope is a resource limit, not a forecast of peak memory. The workload
remains opt-in, and native-GRAM points remain excluded.
