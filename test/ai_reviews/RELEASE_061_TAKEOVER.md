# Review record: SpaceAGORA 0.6.1 candidate

## Scope

Combine the existing release preparation at `558d3e36472d5a19cd7647c44e484e3ee47a50dd`
with merged core #500 and the follow-ups in #508, #509, #513 and #514. Pin EDG
exactly to its published v0.5.1 merge commit
`51cd88b5a54cbcdadbadae8f916004bdf2ccdcf1`.
This is a draft candidate and author integration verification, not an independent
review or a completed release qualification. Merge the prerequisite fixes first.

## Changed Files

Source changes relative to main are inherited from the named fixes:

- `src/orchestration/simulation/engine/resume_checkpoint.jl`: preserve prior rows
  until replacement and persist maneuver, orbit and atmosphere-entry state.
- `src/orchestration/simulation/engine/execution.jl`: write and restore that state.
- `src/orchestration/simulation/state/runtime_types.jl`: own atmosphere-entry counts.
- `src/orchestration/simulation/callbacks/event_callbacks.jl`: use the run-owned count.
- `src/orchestration/simulation/campaigns/predictive_planner.jl`: clear cached generation.
- `src/precompile_workload.jl`: clear process-local ephemeris and multibody caches.
- `src/orchestration/simulation/config/run_settings.jl`: correct a field comment.

Project and root/study manifests carry version 0.6.1. The EDG gitlink uses its
final tag. Documentation, contributor examples, existing review records and
regression tests accompany the inherited changes. No additional runtime patch
was introduced while combining the branches.

## Findings

The merges applied without conflicts. EDG #13 and #16 were independently inspected
and merged after passing hosted CI; #500 was inspected and merged with all 61
hosted checks passing. The final EDG tag declares version 0.5.1 and contains the
checked test migration. Its core service contract remains 3.0.0.

## P1 Assessment

No new P1 identified in the conflict-free integration. This does not clear pending
checks or the independent-review requirement of the constituent runtime changes.

## Tests Added/Updated

The constituent fixes have focused passing regressions and their recorded
contract checks. Their full local unit runs are still in progress; several found
the pre-existing precompile-state failure reproduced on unchanged main. #514
repairs that failure and passes its 13 focused assertions and 33 contract gates.
The combined candidate's checkpoint/output/cache tests and contract gates are
running against the final EDG tag. Hosted CI, strict docs, coverage, contributor
demonstration and release/nightly requirements remain pending. No whole-suite
pass is claimed.

## Residual Risk

Do not merge or tag this candidate from focused evidence alone. The current docs
retain the unreleased marker until final release qualification. The separate
atmosphere-crossing (#516) and event-output (#515) corrections require their own
review and integration decision; the current candidate has not incorporated them.
C-6 and the other 0.7.0 work remain outstanding. Scientific baseline reconciliation
and student acceptance were explicitly deferred until after restructuring.

Unresolved P1: No
