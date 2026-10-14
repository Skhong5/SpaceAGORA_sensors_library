# CYGNSS reconstruction study

This study owns the CYGNSS input pipeline, orbit reconstruction and slew
diagnostics. These are research tools, separate from the local mission examples
in [viewer_demos](../../../scripts/dev/viewer_demos/README.md). The study reuses
that family's `common.jl` for mission setup and saved-result export; HTML
rendering remains in the optional SpaceAGORAVisualizer package.

## Inputs and scope

`fetch_cygnss_l1_states.py` retrieves state variables from NASA's CYGNSS Level 1
v3.2 product for the seven spacecraft present in the June 6–9, 2025 window. The
product is available through Earthdata, with a bearer token supplied in
`~/.edl_token` or the file named by `EARTHDATA_TOKEN_FILE`. The fetcher subsets
the state variables rather than downloading complete granules.

`build_cygnss_ics.jl` converts those Earth-fixed states to J2000, produces initial
conditions and reference tracks, and compares them with the retained public
`cygnss_historical.tle` input. `cygnss_constellation.jl` simulates the resulting
constellation. Its optional fit uses the same reference window; agreement after
fitting is an in-sample result, not an independent prediction.

The slew tools are a separate path. `cygnss_slew.jl` and
`cygnss_slew_checks.jl` require private FM01 attitude, position/velocity and
configuration inputs; command replay also requires command telemetry. Those
inputs, fitted constants and generated flight pages remain private. The
wheels-only physical-adequacy check is currently failing even when the run
completes. Do not report execution success as physical acceptance.

The private reconstruction record was originally
`docs/spaceagora_cygnss_reconstruction_record.md`. Authorized collaborators can
obtain the October 14, 2026 archive snapshot and private-input instructions from
the maintainers. Helper tests cover synthetic inputs and the retained public
TLEs; optional checks of locally generated records skip explicitly when those
records are absent.

## Run from the repository root

Instantiate the repository and `scripts/dev` Julia environments. The latter
supplies the pinned TLE/SGP4 tools through the existing development launcher.
Install the Python requirements in a separate environment using
`scripts/dev/requirements-cygnss.txt`. The state conversion and simulation need
the starter SPICE assets; set `SPACEAGORA_SPICE_PATH` when they are elsewhere.
The constellation uses NRLMSISE-00 and space-weather indices, not native GRAM.

```sh
python3 benchmarks/studies/cygnss_reconstruction/fetch_cygnss_l1_states.py
julia --project=. scripts/dev/run.jl ../../benchmarks/studies/cygnss_reconstruction/build_cygnss_ics.jl
```

Generated inputs stay under the ignored `data/telemetry/CYGNSS/` directory. For
rendering, prepare the separate visualizer environment using the viewer guide,
then expose it to the launcher:

```sh
SPACEAGORA_VISUALIZER_ENV=/path/to/visualizer-env julia --project=. scripts/dev/run.jl ../../benchmarks/studies/cygnss_reconstruction/cygnss_constellation.jl
```

The existing `SPACEAGORA_CYGNSS_DATA` and `SPACEAGORA_DEMO_CYGNSS_*` overrides
remain available in the simulation drivers. The default generated pages still
use `output/viewer_demos/`; moving the tools does not move or overwrite earlier
results. Review each driver's inputs before running a private-data diagnostic.

## Offline checks

These checks need the Python requirements but no token, network or flight data:

```sh
python3 test/scripts/test_cygnss_fetch.py
python3 benchmarks/studies/cygnss_reconstruction/fetch_cygnss_l1_states.py --self-test
```

The core unit suite also tests `cygnss_ics.jl`, `cygnss_tracks.jl`,
`cygnss_slew_telemetry.jl` and `cygnss_command_replay.jl`. Keep these helpers and
the TLE input with the study: the tests consume them directly, so the private
research archive is not a replacement for their public implementation.
