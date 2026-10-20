# GNC and planning ownership

Guidance decides what a vehicle should do. Planning produces a reference that
can be checked and tracked. Control turns that reference into actuator commands,
while navigation provides the hook for a sensor or estimator update. These
responsibilities share state through explicit contracts; their source folders do
not create new Julia modules.

## Planning and its adapters

`src/models/gnc/planning/` collects the planning sources in three groups. `rpo/`
contains the RPO planner contract (`contract/`), target-relative geometry, the
direct baseline, RRT kernels, sampling, retiming, metrics and the planner-neutral
lifecycle. `robot_arm/` contains quintic joint references and arm path geometry.
`common/` contains path and tree operations shared by those applications.
`src/models/gnc/companions/hypr/` contains the core-owned configuration and
compatibility surface used by the optional HYPR implementation, and
`companions/edg/` the `EDGServices` facade of the optional EDG package.

HYPR's search implementation remains in its companion repository. A
`HYPRServices` name in core may identify an extension point or a service supplied
to that implementation; it does not imply another copy of the search algorithm.
Existing module and type identities are retained, including historical
`GuidanceHooks`, `RobotArmPlanning`, `HYPRUtils` and `NavigationHooks` bindings.
The [HYPR contract](../src/reference/hypr_services.md) governs the external boundary.

RPO has one integration path. The public planner interface connects a planner
to `RPOPlannerLifecycle`, which validates returned references before installing
them in the buffer shared with one controller. HYPR plans through it as
`HYPRRPOPlanner`; the HYPR-specific `RPOGuidanceModel`, which used its HYPR
configuration directly, was removed in 0.5.0. See the
[public planner guide](../src/user/rpo_planner_pilot.md).

The `navigation/` directory now contains the navigation extension hook. RPO
reference and collision geometry lives in `planning/rpo/geometry/`; its defining
`NavigationHooks` bindings remain unchanged. That geometry describes the planner's
world and clearance queries. It is not, by itself, a navigation estimator.

## Reporting is an analysis responsibility

Planner comparison execution and tracking stay with the planning sources.
Reporting completed results belongs to
`src/interfaces/analysis/planning/rpo_comparison_reports.jl`. Its internal
`RPOComparisonReports` module owns CSV generation, metric presentation and Plotly
output. It imports only the historical report function identities and the planner
label helpers from `GuidanceHooks`; it does not choose a planner or change a
reference, controller or acceptance result.

The `reporting_compat.jl` file declares the old function objects before loading
the analysis module. The analysis module adds their sole implementations, so
existing `GuidanceHooks` and `SimulationModel` imports, qualified helper access,
and serialized function references continue to resolve to the same defining
names. The namespace is an implementation boundary, not a new public API. This
change does not make plotting an optional package dependency.

## EDG

The independent experimental `SpaceAGORAEDG.jl` companion owns the EDG numerical
kernels. Core retains public model types, run-owned state and passage resets,
physical integration, queries, outputs and panel actuation.
`guidance/aerobraking/typed_edg/` contains these core models and services; its
`algorithms.jl` forwards to the optional provider through `SpaceAGORA.EDGServices`.
The short files in `control/aerobraking/` that import EDG calculations are
compatibility bindings, not additional algorithms.

SpaceAGORA no longer carries the older E-EDG and T-EDG strategy routes: the
`guidance/aerobraking/e_edg/` and `t_edg/` solvers, their execution paths (then in a
`control/aerobraking/` folder), the `internal/bridge_helpers.jl` runtime context conversions and
the mission-owned `operations/aerobraking_policy/` selector. They were
SpaceAGORA's port of the original energy depletion guidance (Falcone and Putnam,
JGCD 45(4), 2022) and of the earlier targeting route, and no simulation path had
reached them since 2026-03-07. The typed path and the companion implement the
generalized formulation (Simha, Yu and Falcone, SciTech 2026) and make no claim of
exact reproduction of the 2022 paper. Every main commit up to and including
`bc823fecc`, the base this removal was prepared on, still contains the routes. To
reproduce them, use tag `parallel-paper-freeze-2026-03-05`, the last tag with the
routes wired into the simulation path, or tag `v1.0.0`, whose older example
scripts configure them.
The [EDG adapter contract](../src/reference/edg_adapter_boundary.md)
defines the supported service boundary; the [EDG guide](../src/user/edg_guidance.md)
describes installation and the bounded experimental scope. Extraction and
interface conformance do not establish broader scientific qualification.

Robot hardware remains under vehicle models, and compliant multibody equations
remain under dynamics. MuJoCo uses the engine's
[external-propagation contract](../src/reference/external_propagation_services.md), which governs
propagation ownership rather than planning. A Robotics grouping for companion
checkouts does not transfer these core responsibilities to either companion.

## Companion boundaries

[Companion boundaries](companion_boundaries.md) lists, for HYPR, SpaceAGORAEDG and
SpaceAGORAMuJoCo, the private SpaceAGORA paths that their tests and research
scripts still use, the rules for moving them to supported calls, and the checks to
run when you change a boundary.
