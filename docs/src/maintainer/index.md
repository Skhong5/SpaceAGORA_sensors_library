# Maintainer Overview

Use this section when you are changing documentation structure, public API
shape, architecture rules, or CI-backed repository policy.

This page is for maintainers and contributors working on the repository itself,
not for first-time users trying to run a simulation.

Quickest maintainer build:

```text
julia --project=. docs/make.jl
```

What to read next:

- [Optional Adapter Standard](adapter_contract.md)
- [EDG Package Boundary](edg_adapter_boundary.md)
- [Documentation Policy](../documentation_policy.md)
- [API Policy](../public_api_policy.md)
- [Contracts](../contracts.md)

## How this section is organized

### Documentation Policy

Use [Documentation Policy](../documentation_policy.md) when you are changing the
docs process, generated API handling, or release-gate expectations.

### API Policy

Use [API Policy](../public_api_policy.md) when you need to know which package
surface is stable and which repository namespaces are still free to move.

### Contracts

Use [Contracts](../contracts.md) when you are changing structure, ownership,
quality gates, or runtime-analysis policy and need the authoritative contract
pages.

## Contributor checklist

Use this checklist for a new contribution or an incoming research branch before
review. Existing adapter-specific requirements remain in the
[optional adapter standard](adapter_contract.md).

1. **Identify the change.** Record the base and candidate revisions, affected
   source owners, intended behavior changes and existing behavior to preserve.
   Use the [architecture map](../user/architecture.md#Map-of-the-repository) to
   distinguish shared core mechanisms, optional implementations and study code.
2. **Name the boundary.** List the documented root API or versioned facade
   used by the contribution. Call out any internal API access explicitly;
   an importable name is not automatically supported. Follow
   [Public API Policy](../public_api_policy.md) and
   [Extensibility](../extensibility.md).
3. **Preserve configuration and numerical contracts.** Use the existing
   configuration-copy helpers so unspecified settings survive overrides; see
   [configuration ownership](../user/simulation_configuration.md#Source-ownership-for-contributors).
   Reuse the shared calculation owner when its semantics match. Record units,
   frames, epoch and quaternion conventions, including normalization and rotation
   direction. Follow the [integrated-state conventions](../user/integrated_state.md)
   and [solver precedence](../user/solver_configuration.md#Solver-policy-and-partitioning);
   do not infer that every quantity uses the same units or change solver policy
   as an incidental cleanup.
4. **Identify runtime ownership.** State who owns mutable state, random-number
   streams and native resources, how they reset between runs, and how workers
   obtain independent instances. Declare unsupported modes and failure behavior.
   For adapters, apply the existing lifecycle and conformance requirements.
5. **Retain evidence with its source.** Record input and asset identifiers,
   configuration, seed policy, platform and dependency revisions alongside the
   [saved outputs](../user/outputs.md). Identify the affected tests and focused
   comparisons, their executed revision, and any failures or unexecuted checks.
   Preserve scientific tolerances and historical results; distinguish new
   measurements from old evidence and PR-head checks from integrated-main checks.
   Follow [Contracts](../contracts.md) and the
   [documentation review checklist](../documentation_policy.md#PR-review-checklist).

## Generated and mirrored content

The generated Public API, Terrain API, Mesh Aerodynamics API and Visualization API pages share one
`docs/public_api_symbols.jl` registry.

The contract pages shown in the docs site are mirrored from the hand-written
sources under:

- `docs/architecture/*`
- `docs/quality/*`

That split is intentional: user-facing docs stay task-oriented, while maintainer
rules remain authoritative and reviewable in their source locations.

## Planner interface development

The [RPO planner contract](rpo_planner_contract.md) describes the internal
request/reference validation layer, its limits and the remaining integration
work. Existing runtime planner selection is unchanged.
