# Paper parallelization benchmark cases

This harness groups the case and mode definitions in
`benchmarks/studies/parallelization_performance/` into B and P phases. Use
`cli.jl` and the underlying `cases.jl`/`modes.jl` as the executable definitions.
The catalog below describes the axes, settings and reporting conventions;
it does not promise a scaling result on a new revision or machine. Historical
tables, phase-sizing investigations and figure runs follow the
[research-record policy](../../../docs/src/user/studies_benchmarks.md#research-records-and-generated-results).

## Measurement conventions

Each `(case, mode, threads, mc_samples)` point runs in a fresh Julia subprocess.
Its repeats stay inside that worker, with the requested warm-up before each
repeat. Process-pool provisioning and warm-up precede the timed region. Resume
granularity is a whole point; worker CSV names are
`perf_<case>_<mode>_t<N>_mc<M>.csv`. Record both startup and timed work when the
research question includes end-to-end throughput.

The router ladder is capped at 32 threads by
`PPB_ROUTER_LADDER_MAX_THREADS`; scaling phases use their full machine ladder.
Thread ladders use physical cores (`SPACEAGORA_PPC_PHYSICAL_CORES`), and B13
uses a 32-core declared budget grid. Record host load and the resolved budget;
physical-core selection does not eliminate contention.

`SPACEAGORA_RHS_CALIBRATE` follows each mode's adaptive flag: static profiles
use `off`, adaptive profiles use `auto`. The requested `rhs_execution_mode` is
not evidence of the selected route; read the telemetry columns below.
Trajectory output is controlled by `simulation_settings.results` and the saving
callback's `data_rate`. Varying `num_steps_to_save` does not create a cadence
comparison in this harness.

## Phases B1–B14

### B1 — Constellation Scaling, Gravity-Only Baseline

Cases: `single_inverse_square_vacuum` through `gravity_1024sat_inverse_square_vacuum`
(1, 4, 16, 64, 256, 1024 spacecraft, inverse-square gravity, no atmosphere).
Modes: `serial`, `outer_threads`, `full_smart`. Thread ladder: full
machine-scaled ladder.

### B2 — Constellation Scaling, L20 Harmonics

Cases: `single_harmonics_l20_vacuum` through `gravity_1024sat_l20_vacuum`
(same constellation sizes as B1, but gravity is a degree/order-20 spherical
harmonics model instead of inverse-square). Modes: `serial`, `outer_threads`,
`outer_inner_adaptive`, `full_smart`.

### B3 — Constellation Scaling, Atmosphere and GRAMTrackCache Surrogate

Cases: `multi_16_aero_surrogate_cached` (16 sat, aerodynamic drag with a
cached/surrogate exponential density model) and `multi_64_high_fidelity` (64
sat, harmonics + SRP + aero + density). Modes: `serial`, `outer_threads`,
`full_smart`.

### B4 — Monte Carlo Throughput Scaling

Cases: `montecarlo_mars_aerobraking`, `montecarlo_high_accuracy` (single
spacecraft, MC-jittered initial conditions). Modes: `serial`,
`outer_process`, `full_smart`. MC sample ladder: 1, 4, 16, 64, 256, 1024.
Thread mode: `:single` (1 thread per Julia process — all parallelism comes
from process count). Worker ladder: 1, 2, 4, 8, 16, 32 process workers.

### B5 — Routing Profile Comparison, R0–R5

Cases: `gravity_16sat_l20_vacuum`, `multi_16_aero_surrogate_cached`,
`articulated_1sat_fullstack` (three representative workloads: pure gravity,
gravity+aero, and a fully articulated single spacecraft with harmonics, aero,
thermal, and attitude). Modes: all six routing profiles — `serial`,
`outer_threads`, `inner_only`, `outer_inner_static`, `outer_inner_adaptive`,
`full_smart` (= R0–R5). Thread mode: `:max_only` (fixed at the machine's
maximum thread count for every mode, so all profiles are compared at the
same total thread budget).

### B6 — Small-Workload Control

Cases: the `many_sat_high_fidelity` spacecraft-count ladder (`multi_4`,
`multi_16`, `multi_64`, `multi_256`), `gravity_16sat_l20_vacuum`,
`montecarlo_mars_gram_live`, `montecarlo_multi_sat`,
`articulated_1sat_fullstack`, `multi_8sat_magnetorquer_attitude`, and
`gravity_16sat_l20_vacuum_longmission`. Modes: `serial`, `outer_threads`,
`outer_process`, `outer_inner_adaptive`, `full_smart`. Thread mode:
`:low_high` (1 and the machine maximum). MC samples: 1 and 8.

### B9 — Router Evaluation: Spacecraft Count

Cases: `gravity_{64,256,1024,4096}sat_l50_vacuum_1hr`. All six modes.
Thread mode: `:router_ladder` (`[1, 8, 32]`).

### B10 — Router Evaluation: Atmosphere and GRAM Usage

Cases: `atmo256_vacuum_10min`, `atmo256_exponential_10min`,
`atmo256_gram_surrogate_10min`, `atmo256_gram_live_10min`,
`atmo256_gram_live_nbody_10min`. All six modes. Thread mode: `:router_ladder` (`[1, 8, 32]`).

### B11 — Router Evaluation: Force and Actuator Model Count

Cases: the `stack<N>_e1..e6` ladder. All six modes. Thread mode:
`:router_ladder`.

### B12 — Router Evaluation: Interacting vs. Independent Propagation

Cases: `interact_{16,64,256}sat_1hr` and `independent_1sat_1hr`. Modes: the
static ladder plus `outer_process`, minus `inner_only`. MC samples: 16/64/256.
Thread mode: `:router_ladder` (`[1, 8, 32]`).

### B13 — Router Evaluation: Thread vs. Process Budget Split

Case: `montecarlo_heavy_aerobraking` at 64 samples. Modes: `outer_process`,
`outer_threads`, `outer_inner_adaptive`, `full_smart`. Budget grid:
`(workers, threads)` ∈ {(1,32), (2,16), (4,8), (8,4), (16,2), (32,1)}.

### B14 — Router Evaluation: Mission Duration and Output Cadence

Cases: `gravity_1024sat_l50_vacuum_15min`, `gravity_1024sat_l50_vacuum_1hr`,
`heavy_1024sat_l50_6hr` (duration), and `cadence_1024sat_{none,60s,10s,1s}`
(cadence). All six modes. Thread mode: `:router_ladder` (`[1, 8, 32]`).

### B7 — Heavy Constellation Scaling

Cases: `heavy_1024sat_l50_6hr`, `heavy_4096sat_l50_1hr`,
`heavy_1024sat_fullstack_1hr`, `heavy_256sat_coupled6dof_2hr`. Modes:
`serial`, `outer_threads`, `outer_inner_adaptive`, `full_smart`. Thread ladder:
full machine-scaled ladder (physical cores).

### B8 — Heavy Monte Carlo Process Throughput

Case: `montecarlo_heavy_aerobraking`. Modes: `serial`, `outer_process`,
`full_smart`. MC sample ladder: 16, 64. Thread mode: `:single`. Worker
ladder: 1, 2, 4, 8, 16, 32, 64.

B6 is a small-workload control; rows below the serial-baseline floor are
excluded from router-performance summaries. B9–B14 isolate spacecraft count,
atmosphere, effector/control count, interacting versus independent propagation,
worker/thread split, and mission duration/output cadence. B11 separates attitude
propagation from actuators; its workload is defined by the catalog, not by a
claim that callback cost is linear. `independent_1sat_1hr` has no initial-state
jitter so that the comparison does not add a sample-dependent step count.

`--phases` preserves the requested order. For example,
`--phases=B9,B10,B12,B11,B13,B14` runs those phases in that sequence; use
`--resume` to continue completed points. Wall time depends on the host and
configuration, so an old session estimate is not a run budget.

## Paper routing phases P1–P5

| Phase | Axis | Cases | Modes |
|---|---|---|---|
| P1 | constellation size at a fixed budget | the iso-work `gravity_{N}sat_l50_vacuum_{S}s` ladder, N = 1…4096 | serial, outer_threads, inner_only, outer_inner_static, policy_v2 |
| P2 | thread budget at a fixed size | that ladder's 4096 rung | same as P1 |
| P3 | Monte Carlo resource ladder, 1 spacecraft/sample | `independent_1sat_1hr`, 256 samples | serial, outer_threads, outer_process, policy_v2 |
| P4 | the same ladder, compute-bound samples | `montecarlo_heavy_aerobraking`, 32 samples | same as P3 |
| P5 | worker/thread split of one fixed budget | `mcgrid_16sat_8mc`, `mcgrid_8sat_16mc` | serial, outer_threads, outer_process, outer_inner_static, policy_v2 |

Every ladder and grid is derived from the host's physical core count (capped at
`PPB_ROUTER_LADDER_MAX_THREADS`, override with `SPACEAGORA_PPB_PAPER_BUDGET`),
because the same five phases run on each paper machine: 12 cores gives a
`[1, 2, 4, 8, 12]` budget ladder and the six splits of 12, while 64 cores gives
`[1, 2, 4, 8, 16, 32]` and the six splits of 32.

P1 uses `PPC_L50_ISO_MISSION_S` to vary duration with constellation size.
The duration is part of the case name and must remain a table column; these are
not the same mission repeated at each N. The configured durations are:

| Spacecraft | Mission duration (s) |
|---:|---:|
| 1 | 4,150,000 |
| 16 | 514,000 |
| 64 | 415,000 |
| 256 | 124,000 |
| 1024 | 24,600 |
| 4096 | 5,800 |

Use `bash benchmarks/studies/paper_parallelization_benchmarks/calibrate_iso_ladder.sh`
to inspect timing on a new host. Do not silently replace configured durations
inside a recorded comparison.

P3/P4 vary budget; P5 varies its split. A P3 grid entry `(b, b)` offers b
process workers and b threads, allowing each route to use its own resource.
P5's fixed-total splits explore outer and inner parallelism together.
`budget_grid_fixed` preserves already host-sized grids instead of rescaling
them as a grid declared against a reference host.

Every point needs its matching serial row. Aggregation joins the baseline on
`(phase_id, case, mc_samples, process_workers)`; without it a ratio is missing,
not zero-cost or infinite-speedup evidence.

```bash
julia --project=examples benchmarks/studies/paper_parallelization_benchmarks.jl \
    --phases=P1,P2,P3,P4,P5 --threads=1,2,4,8,12 --process-workers=12
python3 benchmarks/studies/paper_parallelization_benchmarks/make_paper_routing_tables.py \
    output/performance/paper_benchmarks/<stamp> --out output/paper_routing_tables
```

## Figure F2 (P6, P6p) — thread scaling across force models and density paths

One constellation size, six traces, the thread ladder `1, 2, 4, 8, 16, 32`: what
happens to constellation thread scaling as the force model and the density path
get heavier. Trace 2 is P1/P2's own 4096 rung, reused rather than duplicated, so
the figure and those tables share a serial baseline.

| # | Trace | Case | Mission | `dt_max` | Phase |
|---|---|---|---|---:|---|
| 1 | degree 20 harmonics, vacuum | `gravity_4096sat_l20_vacuum_19700s` | 19 700 s | 20 s | P6 |
| 2 | degree 50 harmonics, vacuum | `gravity_4096sat_l50_vacuum_5800s` | 5 800 s | 20 s | P6 |
| 3 | + SRP + Sun/Moon third body | `gravity_4096sat_l50_srp_nbody_vacuum_5800s` | 5 800 s | 20 s | P6 |
| 4 | + analytic exponential atmosphere | `aero_4096sat_l50_expatm_100s` | 100 s | 5 s | P6 |
| 5 | + native GRAM, look-ahead cache | `aero_4096sat_l50_gram_lookahead_100s` | 100 s | 5 s | P6 |
| 6 | + native GRAM, process route | `aero_4096sat_l50_gram_process_100s` | 100 s | 5 s | P6p |

Modes: P6 runs `serial` (the speedup denominator, at the bottom rung only),
`inner_only` (R2 — one simulation, the split inside the RHS across satellites,
which is what this figure is about) and `predictive` (R7, the adaptive arm).
P6p runs `serial`, `outer_process` and `predictive` over a worker ladder with
threads pinned at 1. Three repeats, one warm-up, cold store.

**Two phases, one figure.** A phase declares one mode ladder for all of its
cases and the six traces do not share one. Traces 1–5 are single simulations of
4096 spacecraft, and `outer_process` is a no-op on them: the harness only ever
spreads *samples* across processes
(`uses_process_pool = sample_count > 1 && mode.backend == "process"`,
`execution.jl`), so a one-sample constellation case under the process route
degenerates to one in-process solve and contributes six identical rows per case.
Trace 6 is the same 4096 spacecraft-missions arranged as 4096 one-spacecraft
samples — the only arrangement the process route can spread, and the same
arrangement `paper_scenarios` S2 uses for its `process_members` mode — and
running *that* under `inner_only` would be 4096 sequential solves with the pool
idle. The satellites in this catalog exert no force on one another, so the two
arrangements do the same total physics; what differs is where the parallelism
can go and how many native GRAM images the machine must hold at once, which is
the trade the figure exists to show. Always run and archive P6 and P6p together.

The native-GRAM density path is selected by `_ppc_p6_gram_density_env!` in
`cases.jl`. Inspect that configuration before running or comparing archived
rows: cache, freeze and perturbation settings are part of the experiment.
They are not interchangeable implementations of an assumed identical density
history. Keep mission duration and step cap with each row.

Recalibrate timing on the target host with:

```bash
bash benchmarks/studies/paper_parallelization_benchmarks/paper_figure_runs.sh calibrate-p6 --execute
```

## P7 — one spacecraft, short missions

P1's one-spacecraft rung is a 4 150 000 s pure-gravity mission, sized so its
serial baseline is intended to clear the 3 s floor. P7 measures the opposite
end: one spacecraft over about one orbit, at the full thread budget, where a route's fixed setup (planning, the
campaign machinery, calibration probes) is a large share of the run. Its serial
baselines are under the floor by design; what the phase reports is each route's
wall time against serial in seconds, not a speedup.

| Row | Physics | Case | Mission | `dt_max` |
|---|---|---|---:|---:|
| 1 | degree 50, vacuum (P1's physics and spacecraft) | `gravity_1sat_l50_vacuum_5702s` | 5 702 s | 20 s |
| 2 | + SRP + Sun/Moon third body (P6 trace 3) | `gravity_1sat_l50_srp_nbody_vacuum_5702s` | 5 702 s | 20 s |
| 3 | degree 50 + exponential-atmosphere aero (P6 trace 4) | `aero_1sat_l50_expatm_5702s` | 5 702 s | 5 s |

**The mission is derived, not calibrated.** It is the two-body period of the
initial osculating orbit of P1's spacecraft (member 1 of `ppc_constellation`,
500 km periapsis and 540 km apoapsis altitude):
a = 6 378 136.6 m + 520 000 m = 6 898 136.6 m, and with the built-in Earth's
μ = 3.98600436233e14 m³/s², T = 2π √(a³/μ) = 5 701.76 s, rounded to 5 702 s
because the case name carries an integer. `PPC_P7_MISSION_S` in `cases.jl`
computes it from the same altitudes and constants at load time, so the case
names follow the orbit if it changes; the workload coverage gate pins that. The
degree-50 field moves the real revolution time off the two-body value by a
relative amount of order J2 (Rp_e/a)², about 1e-3.

**The aero row flies a different orbit and keeps the same duration.** P1's
spacecraft at 500–540 km with the default 120 km entry interface would be outside
the atmosphere, so row 3 flies member 1 of the P6 density constellation (300 km
periapsis, 400 km apoapsis, 600 km interface), inside the atmosphere from the
first step as trace 4 is. It keeps 5 702 s rather than trace 4's 100 s: 100 s
was sized to lift a 4096-spacecraft constellation above the floor and means
nothing at one spacecraft, and a shared duration makes the three rows
iso-mission. 5 702 s is slightly more than one revolution of that lower orbit,
whose own two-body period is 5 492 s by the same formula.

Modes and thread axis are P1's: `serial`, `outer_threads`, `inner_only`,
`outer_inner_static`, `policy_v2`, `predictive`, one sample, at the maximum of
the thread ladder (`thread_mode = :max_only`). Eleven repeats are declared in
the phase itself rather than left to `SPACEAGORA_PPB_MIN_REPEATS` (which only
raises a count), one warm-up. No parity case: P1 carries the P-series parity
check. `make_paper_routing_tables.py` and `make_paper_routing_plots.py` put the
three rows in one table and one figure with the force model on the axis.

## P5f — Monte Carlo over constellations, full machine

P5 runs every mode once per split of the budget, `predictive` and `policy_v2`
included, so its adaptive arms were always told the split and chose only a route
inside it. P5f asks the question a user of the whole machine has: handed every
core and no split, does R7 (`predictive`) land within the 10% criterion of the
best static allocation, that is, the fastest static route at the fastest split?

| | P5 | P5f |
|---|---|---|
| Cases | `mcgrid_16sat_8mc`, `mcgrid_8sat_16mc` | the same |
| Static modes (`serial`, `outer_threads`, `outer_process`, `outer_inner_static`) | every split of the budget | every split of the budget, exactly as P5 |
| `predictive` (R7) | every split | once per case, full budget, no split |
| `policy_v2` (R6) | every split | not run |
| Repeats / warm-up | 5 declared (11 by `SPACEAGORA_PPB_MIN_REPEATS`) / 1 | 11 declared / 1 |

The phase lists `predictive` in `modes` (so the precompile workload and
`--lean-modes` see it) and in `full_budget_modes`; the per-split runs take the
other modes, and the full-budget modes run once more after the splits, in
`P5f/full_w<B>_t<B>/`. `B` is the grid's budget, `PPB_PAPER_BUDGET` (12 on the
workstation, 32 on TRX50), and the run is launched with `--threads=B
--process-workers=B`: a Julia process with every core as threads and a
process-pool cap (`SPACEAGORA_PERF_PROCS`) equal to the same count, which on the
V2 profiles is also the cap an unset environment gets (`usable_core_budget`,
memory permitting). The pool is still capped by the run's `--process-workers`,
as the splits are. From there the campaign runner and the R7 planner choose the
route, pool width, local slots and each sample's inner thread budget
themselves.

**What the adaptive row records.** Every row now carries
`adaptive_allocation`, empty for the pinned modes and
`<route>:w<W>+l<L>:b<B>` for an adaptive campaign: the route it ran, `W`
consumers apart from local slots (thread tasks, or pool worker processes), `L`
coordinator local slots beside a pool, and the largest inner thread budget any
sample ran under, read inside the sample (`ppc_adaptive_allocation` in
`parallelization_performance/execution.jl`). It is read from what ran, not from
the plan, so a guard that closed consumers mid-campaign shows what remained. The
full-budget run also switches `SPACEAGORA_CAMPAIGN_DISPATCH_TRACE=1` on unless
the launcher set it, so the job log carries each campaign's candidates, the
inner-speedup curve the planner read and the reason for its choice.

**Can R7 give a sample more than one thread?** Only when the planner has an
inner-speedup curve for the workload (`_predictive_inner_candidates!` in
`src/orchestration/simulation/campaigns/predictive_planner.jl`); without one every concurrent
plan runs its samples at budget 1. The curve (`rhs_inner_speedup_curve`,
`src/orchestration/simulation/engine/rhs_calibration.jl`) is assembled from RHS calibration
store rows that match the workload's signature stem: machine, spacecraft
bucket, effector set, density model and the code token
(`_RHS_CALIB_CODE_TOKEN`, currently `2026-09-23`), whatever budget or
outer-split flag they were written under, and that carry per-candidate
timings (store schema 2) including a width-1 timing and at least one wider
one. Those timings are written only by a calibration sweep, which runs only in
a solve whose inner budget is above 1 under an adaptive mode
(`SPACEAGORA_RHS_CALIBRATE=auto`; the static modes set it off), so static rows
never contribute. In this harness the adaptive point's own warm-up solve runs
on the coordinator with no inner budget declared, i.e. at the whole pool, and
sweeps; so even from a cold store the timed campaigns see a curve, and a
converged store adds the rows of earlier adaptive runs of the same stem.
`SPACEAGORA_PREDICTIVE_INNER_CURVE=0` disables it. With a curve, the plan space
gains `threads@W` at `b = fld(T, W)`, one sample on the whole pool, and mixed
local slots at `b` threads, each subject to the existing 15% decision margin.
Inspect the dispatch trace and recorded allocation instead of assuming the
planner used the widest candidate.

**The criterion rule.** `benchmarks/studies/paper_parallelization_benchmarks/check_policy_criterion.py` scores a P5f
adaptive row against every static route at every split whose total equals its
budget, with the bias correction every other point gets: the mean of those
(route, split) medians when all of them lie within `--equiv-band` of the
fastest, otherwise the fastest. Tolerance and the failed-campaign rule are
unchanged. `make_paper_routing_tables.py` writes one row per workload with the
winning static route and its split; the plot script draws the two workloads on
one categorical axis.

Run it on its own, recording whether the calibration store is cold or converged:

```bash
julia --project=examples benchmarks/studies/paper_parallelization_benchmarks.jl \
    --phases=P5f --threads=1,2,4,8,12 --process-workers=12
```

## Underlying case families (`parallelization_performance/cases.jl`)

The phases above draw from a shared case catalog, grouped into families:

- **`gravity_only`** — `single_inverse_square_vacuum`,
  `gravity_{4,16,64,256,1024,2048}sat_inverse_square_vacuum`,
  `single_harmonics_l20_vacuum`, `gravity_{n}sat_l20_vacuum`. Pure orbital
  dynamics (inverse-square or L20 harmonics), no atmosphere, no attitude.
  Isolates outer-loop and gravity-RHS parallelism from every other subsystem.
- **`few_sat_high_fidelity`** — `single_harmonics_l50_vacuum` (1 sat, L50
  harmonics), `srp_heavy_high_area` (1 articulated sat, stacked SRP,
  attitude), `articulated_1sat_fullstack` (1 articulated sat: harmonics,
  aero, thermal, attitude). Stresses per-satellite (inner-loop/callback)
  cost with a single spacecraft, so outer-loop parallelism has nothing to
  exploit — these cases isolate inner-loop/effector/callback parallelism.
- **`many_sat_high_fidelity`** — `multi_16_aero_surrogate_cached`,
  `multi_{64,128}_high_fidelity`, `callback_128_aero_thermal`. Constellations
  with harmonics, SRP, aero, and density models together, exercising outer-
  and inner-loop parallelism simultaneously plus callback-heavy aero/thermal
  paths at scale.
- **`monte_carlo`** — `montecarlo_high_accuracy` (1 sat, jittered orbit,
  L50 harmonics), `montecarlo_multi_sat` (4 sat, jittered orbits, L20
  harmonics), `montecarlo_mars_aerobraking` (Mars aerobraking, jittered
  periapsis/apoapsis). Independent-trial workloads that isolate
  process-level throughput scaling from thread-level RHS parallelism.
- **`heavy_scaling`** — `heavy_1024sat_l50_6hr`, `heavy_4096sat_l50_1hr`,
  `heavy_1024sat_fullstack_1hr`, `heavy_256sat_coupled6dof_2hr`,
  `montecarlo_heavy_aerobraking`. Sized so the serial baseline is several
  seconds of integration rather than a fraction of one, which is the
  precondition for any of the other families' routing questions to have a
  measurable answer. See B7/B8 above.
- **`atmosphere_ladder`** — `atmo256_{vacuum,exponential,gram_surrogate,
  gram_live,gram_live_nbody}_10min`. N=256, 600 s, L50 harmonics held fixed;
  only the density path varies. B10.
- **`effector_ladder`** — `stack<N>_e1..e6` for N ∈ {32, 64, 256}. Fixed N and
  duration; one model added per rung, with separate attitude and actuator
  workloads (see B11). A phase picks one N per sub-ladder.
- **`coupling_mode`** — `interact_{16,64,256}sat_1hr` and
  `independent_1sat_1hr`. Matched satellite-hours, arranged as one coupled
  simulation or as N independent trials. B12.
- **`duration_cadence`** — `gravity_16sat_l20_vacuum_longmission`,
  `gravity_1024sat_l50_vacuum_15min`, `cadence_1024sat_{none,10s,1s}`. Mission
  length and trajectory-output volume. B14.
- **`p6_force_ladder`** — `gravity_{N}sat_l20_vacuum_19700s`,
  `gravity_{N}sat_l50_srp_nbody_vacuum_5800s`. Traces 1 and 3 of figure F2:
  the vacuum force model made lighter (degree 20) and heavier (SRP plus
  Sun/Moon third body, which puts CSPICE under `SPICE_LOCK` in the middle of an
  otherwise perfectly parallel RHS). P6.
- **`p6_density_ladder`** — `aero_{N}sat_l50_{expatm,gram_lookahead,gram_process}_100s`.
  Traces 4–6 of the same figure: the `atmo256_*` ladder's design (fixed
  spacecraft count, mission and harmonic degree; only the density model varies)
  at the figure's constellation size. `gram_process` is a Monte Carlo case with
  `default_samples` equal to the constellation size — one spacecraft per sample —
  because the process route spreads samples and never constellation members.
  P6 and P6p. Registered at N = 4096 (the figure) and N = 16; **the 16-spacecraft
  entries exist for the `--profile=test` smoke path only** — that profile pins the
  mission at 10 s and so ignores the duration in the name, and those rows are not
  sized rungs and must never be quoted as measurements.
- **`p7_short_1sat`** — `gravity_1sat_l50_vacuum_{S}s`,
  `gravity_1sat_l50_srp_nbody_vacuum_{S}s`, `aero_1sat_l50_expatm_{S}s` with
  S = `PPC_P7_MISSION_S` (5 702 s). The P1 and P6 patterns at one spacecraft,
  built by the same `ppc_single_config` branches. P7.

## Router-selection telemetry

The raw CSV includes both requested settings and selected-route telemetry.
Read the latter when attributing an observed route choice.

| Column | Meaning |
|---|---|
| `rhs_plan_source` | `none` (calibration never ran), `cache`, or `sweep` |
| `rhs_plan_mode` | the RHS plan installed, e.g. `satellite_batch` or `flat` |
| `rhs_plan_allotment` | that plan's worker allotment |
| `policy_last_mode` / `policy_last_allotment` | last inner-policy decision |
| `policy_last_outer_active` | whether an enclosing outer split was claimed |
| `policy_decisions_total` / `policy_adaptive_decisions_total` | decision counts |
| `policy_threads_enabled_total` | decisions that actually threaded |
| `policy_discarded_by_route_total` | policy decisions the route then overrode |

The selected plan lives in per-solve `shared_buffers.rhs_plan_override` and is
gone once the solve returns, so `record_rhs_plan_selection!`
(`src/orchestration/parallel/policy/policy_telemetry.jl`) copies it into the process-level
policy telemetry, which `ppc_policy_snapshot` reads back after the timed batch.
Telemetry is reset immediately before the timed region so warm-up decisions are
not mixed in.

**Process-route caveat.** These counters are per-process. Under `outer_process`
the solves run on Distributed workers, so the controller's snapshot is empty and
the columns come back as `none`/0. That is accurate rather than missing — the
controller genuinely made no routing decision — but it means the selected-route
columns are only informative for the in-process (serial/threads) routes.

## Outer-backend resolution for the adaptive profiles

R4/R5 declare `backend="auto"`. `ppc_resolve_outer_backend` constructs
`OuterRouteFeatures` from the configuration, selects a route before warm-up,
and uses that choice for the recorded environment and timed dispatch. R3
retains its explicit thread pin as the fixed hybrid baseline. A single-sample
batch resolves to threads because there is no outer campaign split.

## Effective core budget in regret

Regret is only comparable between points given the same machine, and
`thread_count` is the wrong proxy for that on the process route:
`ppc_run_sample_batch` spreads it across `min(mc_samples, process_workers)`
single-threaded worker processes regardless of `thread_count`. An
`outer_process` row with `thread_count=1` can therefore use more than one core;
its actual worker count determines the effective budget.

`_ppb_add_effective_cores!` records `1` for serial,
`min(process_workers, mc_samples)` for a process-backed point, and
`thread_count` otherwise. Regret comparisons use this effective budget.

## Reported regret

Two definitions, both in the aggregated CSV and the report:

- `regret_vs_best_static` — best static route at the **same** thread/worker/MC
  point.
- `regret_vs_best_static_oracle` — best static route at **any** budget tested
  for that case. R4/R5 pick the parallel width as well as the route, so the
  matched-budget figure credits them for a budget someone else chose; the oracle
  charges an adaptive route for running wide on a workload that saturates
  narrow.

`best_static_mode` names which static route set the bar. `below_noise_floor`
marks points whose serial baseline is under 3 s; those are excluded from the
per-axis summary and never quoted as router performance. The summary itself
(worst, median, and fraction of points within 10%) is written to
`router_regret_by_axis_<stamp>.csv`.

## Routing profiles / modes (`parallelization_performance/modes.jl`)

Each mode corresponds to a `ParallelProfile` (R0–R5) and toggles a specific
combination of outer-loop backend and inner-loop/callback parallelism:

| Mode | Profile | Outer backend | Inner/callback parallelism | Adaptive routing | Exercises |
|---|---|---|---|---|---|
| `serial` | R0 | none | off | no | True serial baseline, no parallelism anywhere |
| `outer_threads` | R1_a | threads | off | no | Outer-loop (across-spacecraft) thread parallelism only |
| `outer_process` | R1_b | process | off | no | Outer-loop (across-spacecraft/trial) process parallelism only |
| `inner_only` | R2 | none | auto (density/control/thermal/multibody/effector) | no | Inner-loop/callback parallelism only, no outer parallelism |
| `outer_inner_static` | R3 | threads | auto, statically scheduled | no | Combined outer+inner parallelism with a fixed (static) schedule |
| `outer_inner_adaptive` | R4 | auto | auto, dynamically scheduled | yes | Combined outer+inner parallelism with adaptive/dynamic routing |
| `full_smart` | R5 | auto | auto + thermal always on, persistent hints | yes | The full adaptive routing policy SpaceAGORA ships with, including persistent scheduling hints and control-tail guarding |

## Preview mode

`--preview` (or `SPACEAGORA_PPB_PREVIEW=1`) caps constellation size at 64
spacecraft, MC samples at 16, process workers at 4, and repeats at 2, so the
same phase structure can be smoke-tested on a laptop before committing to a
full run on the benchmark machine.

One exception: a host-sized split grid (`budget_grid_fixed` with every entry one
split of the same budget, which is P5 and P5f) is kept whole under `--preview`,
and its runs are not held to the 4-worker cap. The cap and the `w * t <= 4`
filter exist to fit a grid declared against a 32-core box onto a laptop; this
grid is already the host's own budget, and on any host above 4 cores the filter
emptied it, so the phase ran as one unsplit run that measured none of its axis.
`--process-workers` still caps it, as it does outside preview.
