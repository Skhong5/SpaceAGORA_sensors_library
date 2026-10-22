# Telemetry validation: Odyssey and Venus Express aerobraking reconstructions

This study was developed alongside the internal technical record
`docs/spaceagora_aerobraking_reconstruction_record.md` and its PDF counterpart.
That record is not distributed in this checkout; references to its sections and
reported measurements in the archived original preserve their historical attribution. They cannot
be independently checked against the complete record from this repository alone.

The runnable definitions are the checked-in [manifests](manifests/), one per run.
They configure Mars Odyssey and Venus Express (VEX) aerobraking reconstructions
through `SpaceAGORA.TelemetryVerification`; commands and input requirements are
given below. For the general telemetry workflow, see the
[verification guide](../../../docs/src/user/verification_study.md).

The manifests are copies-of-record: `odyssey_tolson` and `vex_venusgram`
mirror the `odyssey`/`vex` scenarios in `test/telemetry_benchmark_manifest.toml`
(the nightly/PR sentinel path); the MarsGRAM comparator and the ±1σ envelope
exist only here. If the record configuration ever changes, both places must
change together.

## Runs

| Run | Atmosphere | Native GRAM | Purpose |
|---|---|---|---|
| `odyssey_tolson` | Tolson accelerometer-derived tabulated density | no | calibrated Odyssey reconstruction |
| `odyssey_marsgram` | MarsGRAM TES Mapping Year 2 (MY25), deterministic | yes | climatology comparator |
| `odyssey_tolson_sigma_minus` | Tolson density at −1σ | no | informational density-uncertainty envelope |
| `odyssey_tolson_sigma_plus` | Tolson density at +1σ | no | informational density-uncertainty envelope |
| `vex_venusgram` | VenusGRAM, deterministic | yes | VEX 2014 reconstruction |

Measured runtimes and error tables belong in the
[research record](../../../docs/src/user/studies_benchmarks.md#research-records-and-generated-results),
with the revision, input versions and calibration choices. Do not treat a
previous platform's values as a current acceptance result.

## Commands

```bash
# Record scenarios (Tolson needs no GRAM; VEX needs the native GRAM build):
julia --project=. benchmarks/studies/telemetry_validation/run_validation.jl full --runs=core

# Everything in the record (adds the MarsGRAM comparator and the ±1σ envelope):
julia --project=. benchmarks/studies/telemetry_validation/run_validation.jl full --runs=all --enforce=true

# Fast pipeline check (first orbits only; does NOT reproduce record values):
julia --project=. benchmarks/studies/telemetry_validation/run_validation.jl quick --runs=odyssey_tolson

# Venus Express against ESA's reconstructed orbit (ORVM_T19). Download the kernel
# ORVM_T19___________00001.BSP (SHA-256 6b422c69...ab512dc9) from
# https://naif.jpl.nasa.gov/pub/naif/pds/data/vex-e_v-spice-6-v2.0/vexsp_2000/DATA/SPK/,
# build the reference files (not tracked), then run the five vex_orvm_* manifests.
# SPACEAGORA_TELEMETRY_STATE_OUT=<csv> also writes the final run's state history.
julia --project=. benchmarks/studies/telemetry_validation/vex_orvm/build_reference.jl \
    data/spice_external/vex/ORVM_T19___________00001.BSP data/telemetry/VEx/orvm
julia --project=. benchmarks/studies/telemetry_validation/run_validation.jl full --runs=vex_orvm

# Figure 1 input: matched accelerometer-vs-MarsGRAM density comparison
# (needs native GRAM + the m01_ab_v2.bsp kernel under the GRAM Suite SPICE tree):
julia --project=. benchmarks/studies/telemetry_validation/density_compare.jl

# Figures 1-5 of the record's visual section, from whatever outputs exist:
julia --project=examples benchmarks/studies/telemetry_validation/plot_results.jl
```

`--enforce=true` applies the manifest gate tolerances to `odyssey_tolson`,
`odyssey_marsgram`, and `vex_venusgram` only; the ±1σ envelope runs are never
enforced. Check trajectory completion before comparing full-span metrics; an
early impact or terminated run is not a full-span reconstruction.

Outputs land in `results/<hostname>/` (gitignored):

```
results/<hostname>/
  <run>/telemetry_orbit_accuracy_summary.csv   # per-event metrics, calibration, gates
  <run>/telemetry_orbit_accuracy_errors.csv    # per-orbit truth/sim/error rows
  density_compare/odyssey_accel_vs_marsgram_density.csv
  figures/figure{1..5}_*.png
```

## What each run does (and does not) show

These are **calibrated mission reconstructions**, not blind predictions and
not component-by-component validation (record §1, §7, §8):

- No pass or time interval is withheld. The constant per-event altitude
  alignments (`b = -median(e_1..e_10)`, ±500 km cap — a candidate at the cap
  is a model-mismatch alarm and is reported unapplied) are estimated from the
  same event series that is scored.
- `odyssey_tolson` largely removes atmospheric-climatology error, so its
  error metrics evaluate the aggregate propagation + free-molecular aero +
  maneuver replay + apsis-detection chain. The Tolson density itself embeds a
  flight-side aerodynamic model, so the run is only weakly independent of
  aerodynamic assumptions. Cr is pinned at 1.3 and Cd scale at 1.0 by
  the manifest configuration.
- `odyssey_marsgram` shows what restoring the selected climatology does to
  the same chain. It is not blind MarsGRAM
  validation: the TES family was selected using Odyssey density data from
  this same campaign.
- `vex_venusgram` exercises Venus-specific geometry, element-frame handling,
  maneuver replay, and geodesy against plot-digitized truth. It fits Cr on
  the scored campaign and uses the legacy comparison axis (no
  `epoch_orbit_offset`), so orbit registration is part of the error budget.
  A drag-corridor discrepancy is aggregate; VEX alone cannot split atmosphere
  from aerodynamics.

## Relationship to the nightly sentinel

`benchmarks/studies/telemetry_orbit_accuracy_study.jl` +
`test/telemetry_benchmark_manifest.toml` remain the CI/nightly path (the
Tolson digital-twin regression sentinel). This directory exists to reproduce
the *complete* record — including the MarsGRAM comparator and the ±1σ
envelope, which are deliberately not part of the nightly — without touching
the sentinel configuration.

## Data prerequisites

- `data/telemetry/Odyssey/*.feather`, `data/telemetry/VEx/*.feather` — in-tree.
- `data/planets/gravity/{Mars50c,MGNP180U}.csv` — in-tree.
- Native GRAM build under `data/GRAMSuite.jl/GRAM Suite 2.0` (see
  `scripts/ensure_gram_native.jl`) — required by `odyssey_marsgram`,
  `vex_venusgram`, and `density_compare.jl` only.
- `data/GRAMSuite.jl/GRAM Suite 2.0/SPICE/spk/missions/m01_ab_v2.bsp` —
  required by `density_compare.jl`.
