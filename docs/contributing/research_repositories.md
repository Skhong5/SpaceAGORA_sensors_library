# Research repository ownership

Paper and mission studies are independent consumers of SpaceAGORA. Small teaching
examples, regression fixtures and the reusable benchmark and verification tools
stay in core. The [user guide](../src/user/studies_benchmarks.md) selects the right
entry point.

## Source transfer

The public study snapshot comes from core
`bfa32f6a44c5f242714b8cf182a451e863119a05`. Each destination records original Git
blobs, the license and intentional path/environment adaptations. The initial
import commits identify the transferred source. Later commits on each import adapt
it (study folder names, and the parallelization studies' core pin, now the v0.6.0
release); core's links point at the current heads below. Neither is a floating
dependency pin for core runtime.

| Repository | Owns | Imported source | Current head |
|---|---|---|---|
| [SpaceAGORAAtmosphereStudies](https://github.com/Space-FALCON-Lab/SpaceAGORAAtmosphereStudies/tree/32ca5be9d6b6fe4e865365a470d03a2a56b3a1fe) | Atmosphere and entry-density research | `32ca5be9d6b6fe4e865365a470d03a2a56b3a1fe` | `32ca5be9d6b6fe4e865365a470d03a2a56b3a1fe` |
| [SpaceAGORAParallelizationStudies](https://github.com/Space-FALCON-Lab/SpaceAGORAParallelizationStudies/tree/7821388870571474550533a15d0ecb2b60d01650) | Paper campaigns and scaling investigations | `0071dd9b6e0f94828fb4da8cfcddc8425c403abd` | `7821388870571474550533a15d0ecb2b60d01650` |
| [SpaceAGORAMissionReconstructions](https://github.com/Space-FALCON-Lab/SpaceAGORAMissionReconstructions/tree/ff67a512face495afa6dec10171806209a62033d) | Mission reconstruction and telemetry fitting | `ff67a512face495afa6dec10171806209a62033d` | `ff67a512face495afa6dec10171806209a62033d` |
| [SpaceAGORARPOStudies](https://github.com/Space-FALCON-Lab/SpaceAGORARPOStudies/tree/76d12285689b339807c648fabe2f2cdddeae6828) | RPO paper reconstruction | `9356eacc6de660947133c8574fc652bae3b8e58b` | `76d12285689b339807c648fabe2f2cdddeae6828` |

## Tests and callers

- Core retains generic simulation, planner, actuator, telemetry-parser and
  parallel-execution assertions. Minimal attributed fixtures preserve the generic
  contracts formerly exercised by importing a mission driver.
- Paper-only gates and tests move with the complete study. The parallelization
  project keeps the paper workload coverage gate and its controls. The RPO
  project keeps saved-paper-input, swarm-policy and refused-plan evidence.
- The mission project tests the actual study producers. Core's smaller fixtures
  do not claim that a later edit to an external study still produces the same data.
- The automatic native example job checks the transferred Mars RAAN study at an
  exact public revision, using its pinned core and public GRAM wrapper. It reuses
  provisioned assets; missing licensed inputs fail rather than count as coverage.
- The core CLI's runtime-analysis, smart-parallel-ladder and telemetry commands
  continue to use core-owned tools. Paper workload preparation and paper remote
  recipes belong to the parallelization project.

The study environments intentionally retain some private core dependencies.
Their source and environment manifests document that boundary. This extraction
does not promote those names to public APIs or make arbitrary newer core commits
compatible. A pin update needs the affected original-versus-new scientific
checks, with unavailable native/data cases stated explicitly.

## Merge and maintenance order

Make the replacement repository source reachable before deleting a core caller.
Keep the documented source commits reachable from each research repository's
main history. Use merge commits for these imports, or update all recorded pins
and requalify them before choosing a history rewrite.

The core relocation is composed with the examples environment, optional plotting,
HYPR-loader retirement and public documentation work. Those changes must be
integrated before this relocation is merged into main. Hosted checks and the
native research smoke remain required; local deterministic checks do not waive
those jobs.
