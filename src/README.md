# Source directory

SpaceAGORA organizes its implementation into four responsibility groups. The
child directories keep their domain ownership; the parent folders make the
command surfaces, execution machinery, models and shared utilities easier to
find.

```text
src/
├── SpaceAGORA.jl
├── precompile_workload.jl
├── interfaces/
│   ├── cli/
│   ├── io/
│   └── analysis/
├── orchestration/
│   ├── simulation/
│   ├── mission/
│   └── parallel/
├── models/
│   ├── vehicle/
│   ├── environment/
│   ├── dynamics/
│   └── gnc/
└── foundation/
    ├── core/
    └── assets/
```

**Interfaces** contains the CLI, configuration and output I/O, and analysis and
visualization support. It includes the operations used to enter a run or inspect
its results; it is not a registry of every abstract interface in the package.
Model-specific hooks stay with their model owners.

**Orchestration** prepares and executes simulations, dispatches callbacks and
campaigns, selects mission policy and routes parallel work. **Models** holds
vehicle construction, environmental models, dynamical equations and the GNC
contracts and algorithms used by those runs. **Foundation** holds shared state
and types, frame and numerical utilities, model composition and asset lookup.

These are folders, not new Julia namespace modules or strict dependency layers.
`SpaceAGORA.jl` remains the package entry point, with its package-level
`precompile_workload.jl` helper alongside it. The composition loader
`foundation/core/simulation_model.jl` still builds `SimulationModel` by including
definitions from several groups. The grouping preserves module identities,
public names and simulation behavior; it does not change which module owns a
type or function. Prefer the supported package API over including source files
from a script.

Julia package extensions remain in the repository-root [`ext/`](../ext/), where
Julia discovers them. Optional companion repositories such as HYPR and
SpaceAGORAMuJoCo keep their own source trees outside this directory. The existing
[`packages/SpaceAGORAHYPR/`](../packages/SpaceAGORAHYPR/) package is a compatibility
loader for external HYPR. Runnable scenarios remain in [`examples/`](../examples/),
operational tools in [`scripts/`](../scripts/) and browser code in
[`viewer/`](../viewer/).

See [Architecture and Responsibilities](../docs/src/user/architecture.md) for
the runtime flow, detailed source owners and guidance on where new work belongs.
