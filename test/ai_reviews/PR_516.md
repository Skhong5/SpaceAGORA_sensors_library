# Review record: atmosphere-entry and drag-phase crossings

## Scope

Prepared on 2026-10-23 as a follow-up stacked on PR #509, including its
entry-counter checkpoint repair at `3ed4b179a`.
This record is author verification, not an independent review.

A three-entry mission with constant-coefficient drag and a zero-density
atmosphere could stop making progress at an atmosphere exit. The entry-limit
and drag-phase callbacks located the same root separately. The regression
reaches `MaxIters` with the counter repair alone. Sharing the crossing callback
allows it to finish, including after loading an older checkpoint without an
entry counter.

## Changed Files

- `src/orchestration/simulation/callbacks/event_callbacks.jl`: share entry counting and combine coincident atmosphere roots.
- `src/orchestration/simulation/callbacks/density_callbacks/assembly.jl`: choose combined or standalone root handling.
- `test/unit/simulation/entry_limit_crossing_tests.jl` and `test/unit/runtests.jl`: regression coverage and inclusion.

## Findings

- When both entry limits and drag-phase changes need the atmosphere interface,
  one callback handles that root and counts downward crossings before applying
  phase settings. Each active spacecraft is counted once per event mask.
- Entry limits without phase changes retain their standalone callback.
  Runs without an entry limit retain the existing drag callback behavior.
- The new unit file is included in the complete unit-test runner. It covers
  distinct and simultaneous roots, equal and different phase tolerances,
  inactive spacecraft, exact resumed state and old-checkpoint counter restart.

This intentionally changes the root-handling path of entry-limited runs with
phase changes. It does not claim bitwise agreement with the old stalled run.
The parent counter repair remains in PR #509; this follow-up changes root handling.

## P1 Assessment

No P1 finding identified in the changed paths. The previous zero-step failure is
reproduced by the negative control and removed by the shared crossing.

## Tests Added/Updated

Julia 1.12.1, local isolated checkout:

- `/tmp/spaceagora-entry-events-focused.log`: all 14 assertions passed.
- `/tmp/spaceagora-entry-events-negative-control.log`: the same new test on
  the counter-only checkout fails with `Checkpointed solve failed with
  retcode=MaxIters` before its first assertion. This is the intended negative
  control, not a passing test.
- Final contract rerun: all 33 gates pass, exit 0, in
  `/tmp/spaceagora-entry-events-gates-final.log`.
- Broader corrected invocation passes builtin checkpoint (62 assertions), vector
  callbacks (21 assertions) and callback-container tests, with no failed items.
  The first invocation failed before tests because its EDG environment was absent.
- The complete unit tree is still running. Hosted CI is pending.

## Residual Risk

Verify the complete unit result and the required PR checks on the eventual
published head. Inspect simultaneous-event termination and old-checkpoint
behavior independently before merging. Do not interpret the focused checks as
completion of the full 0.6.1 or 0.7.0 release qualification.

Unresolved P1: No
