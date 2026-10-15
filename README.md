# SpaceAGORA.jl

SpaceAGORA.jl is a Julia toolkit for spacecraft orbit and atmospheric-flight
simulation, including aerobraking, entry, and aerocapture workflows. The main
user-facing surfaces are:

- the root `SpaceAGORA` Julia API
- the repository CLI wrapper at `./bin/spaceagora` on Linux/macOS or
  `bin\spaceagora.bat` on Windows
- runnable example scripts under `examples/`

Cross-platform command conventions used below:

- `julia --project=. src/interfaces/cli/main.jl ...` works on Windows, Linux, and macOS
- `julia --project=. scripts/ensure_gram_native.jl ...` works on Windows,
  Linux, and macOS
- `./bin/spaceagora` and `./scripts/ensure_gram_native.sh` remain available as
  convenience wrappers on Linux/macOS
- command blocks say where they are typed: **Terminal** means a shell prompt
  (PowerShell or Command Prompt on Windows, a terminal on Linux/macOS) and
  **Julia REPL** means the `julia>` prompt of a running Julia session. Unless
  a block says otherwise, terminal commands run from the repository root, the
  directory that contains `Project.toml`

## Installation

**SpaceAGORA requires Julia 1.12** (any 1.12.x release). Other releases,
including Julia 1.13, are not supported and the package manager refuses them;
check with `julia --version` before anything else. See
[Julia version](docs/src/user/installation_environment.md#julia-version) for
what an unsupported release reports.

Use the repository root environment as the canonical committed execution environment for examples, tests, and normal local runs.

Terminal, Linux/macOS. The first line creates the checkout; the other lines
run inside it, which is the repository root for every later command:

```text
GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none https://github.com/Space-FALCON-Lab/SpaceAGORA.jl
cd SpaceAGORA.jl
julia --project=. -e "using Pkg; Pkg.instantiate(julia_version_strict=true)"
```

`julia_version_strict=true` makes the package manager stop with one clear
message when the running Julia is not a 1.12 release, instead of continuing
into unrelated errors.

On Windows the variable is set in a separate statement, and the `cd` and
`julia` lines above then run unchanged. Terminal, Windows PowerShell:

```text
$env:GIT_LFS_SKIP_SMUDGE = "1"
git clone --filter=blob:none https://github.com/Space-FALCON-Lab/SpaceAGORA.jl
```

Terminal, Windows Command Prompt:

```text
set GIT_LFS_SKIP_SMUDGE=1
git clone --filter=blob:none https://github.com/Space-FALCON-Lab/SpaceAGORA.jl
```

`GIT_LFS_SKIP_SMUDGE=1` leaves the seven GRAM surrogate grids under `data/planets/gram_surrogate/` as small Git LFS pointers instead of downloading about 2.5 GB that nothing on the quickstart path reads (a benchmark study is their only consumer; `git lfs pull --include "data/planets/gram_surrogate/*"` fetches them later if you need that study). The `--filter=blob:none` keeps the clone to the current tree: the repository history carries large data blobs that no longer exist in `main`, and a plain clone downloads all of them.

That gives you the baseline open-data onboarding path immediately. You do not
need GRAM or SPICE to run the first quickstart example, and there is no bootstrap step beyond instantiating the committed root environment.

### Check the installation

A successful setup looks like this. Terminal, from the repository root:

```text
julia --version
julia --project=. -e "using SpaceAGORA; println(pkgdir(SpaceAGORA))"
```

The first command prints `julia version 1.12.` followed by the patch number.
The second prints the absolute path of your checkout; the first time it runs
it also compiles the package, which takes several minutes and shows
precompilation progress, and nothing else should be printed. The same check
from a Julia REPL started from the repository root with `julia --project=.`:

```julia
julia> Base.active_project()
"/absolute/path/to/SpaceAGORA.jl/Project.toml"

julia> using SpaceAGORA

julia>
```

If the path does not end in `SpaceAGORA.jl/Project.toml`, Julia was started
from another directory or without `--project=.`; see
[Troubleshooting](docs/src/user/troubleshooting.md#wrong-working-directory).

Helpful first checks. Terminal, from the repository root:

```text
julia --project=. src/interfaces/cli/main.jl assets check
julia --project=. src/interfaces/cli/main.jl assets setup-open
```

`assets check` reports whether each local asset root has payload bytes, is missing or empty, or still contains Git LFS pointers. It exits with status 1 when a required entry is not usable; this inventory does not validate payload contents.
`assets setup-open` summarizes the baseline repo-only path and which licensed
external assets remain user-provided.

## Optional MuJoCo scenes

MuJoCo proximity scenes are provided by the independent
[SpaceAGORAMuJoCo.jl companion](https://github.com/Space-FALCON-Lab/SpaceAGORAMuJoCo.jl).
It owns the native binding, scene dynamics, adapter tests and Basilisk comparison
harness. Native scenes support Linux x86_64 with MuJoCo 3.11.0.

Install the companion in a separate environment and explicitly load it when
needed; see [installation and environment](docs/src/user/installation_environment.md#optional-mujoco-companion).
Its connection to SpaceAGORA follows the same
[versioned adapter standard](docs/src/reference/adapter_contract.md) as other
optional integrations. Loading SpaceAGORA alone does not load the companion or
download MuJoCo.

## Guidance and control with a surrogate atmosphere

The bounded Mars Odyssey P20 preset supplies a reproducible frozen atmosphere
without native GRAM. From a normal clone, set up the example's own environment,
which pins the public GRAMSuite wrapper and resolves the other packages, and
run the active solar-panel control comparison. Terminal, from the repository
root (the directory that contains `Project.toml`); the paths below are
relative to it:

```sh
julia --project=examples/odyssey_surrogate_env examples/odyssey_surrogate_env/setup.jl
julia --project=examples/odyssey_surrogate_env examples/odyssey_surrogate.jl
```

The setup clones the EDG guidance companion from GitHub and adds it to the
example environment, which edits the tracked
`examples/odyssey_surrogate_env/Project.toml`; the walkthrough linked below
describes both. On the first example run, only the selected grid and the
public scenario assets download. Their
checksums are verified before simulation. The example compares 90-degree and
30-degree panel-angle limits and saves the actual commands, trajectory, heating
and atmosphere provenance in `odyssey_surrogate_results/`, or the directory given
with `--output=DIR`; it prints that location when it finishes. See the [Odyssey walkthrough](docs/src/tutorials/odyssey_surrogate.md)
for the supported domain, frozen epoch, offline use and changing the controller.
Native GRAM remains an optional advanced backend for atmosphere studies.

## Asset model

SpaceAGORA supports two common setup modes. A third, native-free route supplies
a named frozen atmosphere through its own example environment; see "Guidance and
control with a surrogate atmosphere" above.

### 1. Baseline no-GRAM mode

This mode is fully repository-local and is the recommended first run.

- uses built-in planets such as `Earth()`, `Mars()`, and `Venus()`
- uses fallback atmospheres such as `NoAtmosphereModel()` or
  `ExponentialAtmosphereModel(planet)`
- uses `SimpleEphemeridesModel()` instead of SPICE-backed ephemerides

### 2. GRAM/SPICE-backed mode

This mode is for higher-fidelity atmosphere and ephemeris workflows.

- requires a usable GRAM installation under
  `data/GRAMSuite.jl/GRAM Suite 2.0`
- requires local SPICE kernels under
  `data/GRAMSuite.jl/GRAM Suite 2.0/SPICE`
- loads the `GRAMSuite` package extension automatically when `GRAMSuite` is
  importable

Need the exact process for pulling the `GRAMSuite.jl` submodule and copying the
official NASA GRAM Suite folders into the expected repo path after access is
approved? See
[docs/src/user/gramsuite_setup.md](docs/src/user/gramsuite_setup.md). It is a
step-by-step local setup guide for licensed GRAM assets. The full GRAM submodule
checkout uses Git LFS and downloads several GB of binary data, so it is separate
from the default baseline installation path.

If the native GRAM shared library for your operating system is missing, build
it with:

```text
julia --project=. scripts/ensure_gram_native.jl
```

The command checks whether the library file exists; it does not check its
architecture or test whether it can load. If you copied an existing build from
another machine, force a clean rebuild with:

```text
julia --project=. scripts/ensure_gram_native.jl --clean
```

## Quick Start

The fastest first run. Terminal, from the repository root (the directory that
contains `Project.toml`):

```text
julia --project=. examples/AGORA_Basic_Quickstart.jl
```

Or through the CLI, from the same directory:

```text
julia --project=. src/interfaces/cli/main.jl run --example=AGORA_Basic_Quickstart.jl
```

That example is intentionally simple:

- Earth orbit
- no atmosphere drag
- no SPICE dependency
- no GRAM dependency
- results written under `output/`

If you only want to verify the end-to-end path with a short run, use:

```text
julia --project=. src/interfaces/cli/main.jl run --example=AGORA_Basic_Quickstart.jl --smoke
```

## Basic Example

The quickstart script lives at
`examples/AGORA_Basic_Quickstart.jl` and is the recommended first code sample
to read. It demonstrates the minimum pieces needed to run a simulation:

1. activate the shared example helpers
2. choose a no-GRAM planet model
3. build a simple spacecraft
4. assemble a `SimulationConfiguration`
5. run the simulation and save a CSV report

Key ideas in that example:

- `make_no_gram_planet(:earth)` avoids SPICE and uses built-in Earth constants
- `NoAtmosphereModel()` gives a vacuum baseline so installation is easier to
  validate
- `SimpleEphemeridesModel()` keeps the frame/ephemeris path open-data and
  lightweight
- `make_three_body_spacecraft(...)` builds a simple bus-plus-panels geometry
- `make_example_config(...)` packages common example defaults into a single
  configuration object

## GRAM Access

Once GRAM assets are available locally, the smallest GRAM-backed example is:

```text
julia --project=. examples/AGORA_Basic_GRAMEarth.jl
```

That example adds the minimum GRAM-specific pieces on top of the baseline path:

1. `setup_gram_example!()` loads the vendored `GRAMSuite` project if needed
2. `Earth("", SPICE_PATH)` uses the SPICE-backed planet constructor
3. `GRAMAtmosphereModel(planet_name="earth")` switches the atmosphere source
   from the built-in fallback model to GRAM

Before running it, verify:

- `data/GRAMSuite.jl/GRAM Suite 2.0` exists
- `data/GRAMSuite.jl/GRAM Suite 2.0/SPICE` exists
- the platform-native GRAM shared library exists, or can be built with
  `julia --project=. scripts/ensure_gram_native.jl`

Useful commands:

```text
julia --project=. src/interfaces/cli/main.jl assets check
julia --project=. scripts/ensure_gram_native.jl
julia --project=. examples/AGORA_Basic_GRAMEarth.jl
```

If you want fuller mission examples after that, start with:

- `examples/AGORA_Vex.jl`
- `examples/AGORA_Odyssey.jl`

Both of those use the same GRAM access pattern as the basic GRAM example, but
add mission-specific guidance, maneuvers, and higher-fidelity dynamics.

## Reusable missions and viewing results

A `MissionPlan` builds a fresh configuration for each run. `run_mission` saves
results into a new directory and returns the table, configuration and portable
viewer payload together. Named phases describe observation windows; named events
construct fresh callbacks against the configuration that actually runs.
See the [mission workflow guide](docs/src/user/mission_workflows.md) and
`examples/AGORA_Mission_Workflow.jl` for a complete example.

The browser viewer is the independent
[SpaceAGORAVisualizer.jl package](https://github.com/Space-FALCON-Lab/SpaceAGORAVisualizer.jl).
Install the tested pair with `julia --project=. scripts/setup_visualizer.jl .visualizer-env`,
then load both packages in that environment. A saved viewer JSON can also be
opened with the companion alone, after moving the output directory or closing
the simulation process. See the [visualization guide](docs/src/user/visualization.md).

## CLI and docs

- Docs site: [space-falcon-lab.github.io/SpaceAGORA.jl](https://space-falcon-lab.github.io/SpaceAGORA.jl)
- Cross-platform CLI entrypoint: `julia --project=. src/interfaces/cli/main.jl`
- Linux/macOS wrapper: `./bin/spaceagora`
- Windows wrapper: `bin\spaceagora.bat`
- Run an example: `julia --project=. src/interfaces/cli/main.jl run --example=AGORA_Basic_Quickstart.jl`
- Asset check: `julia --project=. src/interfaces/cli/main.jl assets check`
- Verification study: `julia --project=. src/interfaces/cli/main.jl telemetry quick --output-dir=output/telemetry_cli --enforce=1`

## For contributors

Choose a location by what the code is responsible for. The repository has these
main entry points:

| Location | What belongs here |
| --- | --- |
| [src/](src/README.md) | Installed implementation, grouped into interfaces, orchestration, models and foundation. |
| [examples/](docs/src/user/examples_catalog.md) | Runnable scenarios that demonstrate supported usage, with their shared support code and any separate environments. |
| [templates/](docs/src/extensibility.md) | Starting points to copy and complete when implementing an extension. |
| [benchmarks/](docs/src/user/studies_benchmarks.md) | Evaluation procedures, comparison studies and their analysis tools. |
| [scripts/](scripts/) | Installation, maintenance, diagnostics and operational tools. |
| [test/](test/README.md) | Automated verification, test helpers and required fixtures. |
| [docs/](docs/) | User guides, reference documentation and maintained design contracts. |
| [data/](docs/src/assets.md) | Declared input assets and external-data setup, including the GRAMSuite checkout. |
| [packages/](packages/README.md) | Optional companion checkouts, installation metadata and the HYPR compatibility loader. |
| [ext/](ext/) | Julia package extensions owned by SpaceAGORA. |
| [bin/](bin/) | Thin command launchers for users. |
| [build/](build/) | The separate environment and driver for building a system image. |
| [.github/](.github/) | Repository automation and contribution templates. |

`experimental/` remains a separate research area. Browser rendering is maintained in
[SpaceAGORAVisualizer.jl](https://github.com/Space-FALCON-Lab/SpaceAGORAVisualizer.jl),
with installation and usage linked above.

Keep related files together and reuse an existing home before creating another
folder. The [topology contract](docs/architecture/canonical_topology_contract.md)
describes the placement and retirement rules. The
[source directory guide](src/README.md) explains the four source groups and the
composition loader; the groups do not introduce new Julia namespaces.

- The [user documentation](docs/src/index.md) covers installation, simulations,
  results and supported extension interfaces.
- Start with [How a Simulation Fits Together](docs/src/user/architecture.md)
  for the simulation flow and spacecraft composition.
- Repository changes, documentation builds, ownership rules and release checks
  are covered separately in the [contributor guide](docs/contributing/index.md).

Generated reports, docs builds, and other local outputs stay in ignored paths
such as `output/`, `docs/build/`, `docs/site/`, and `docs/src/generated/`.
