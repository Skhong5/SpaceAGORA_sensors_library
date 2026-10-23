# Review record: output inside an event-shortened step

Prepared 2026-10-23 on main `0f03bbfcef0aa694fc01cd3209fed5aa07edafd7`.
This is author verification, not an independent review.

## Scope

Correct fixed-cadence output sampled inside a step shortened by a continuous event.

## Changed Files

- `src/orchestration/simulation/callbacks/event_callbacks.jl`: drain pending pre-event samples before event effects and preserve callback lifecycle hooks.
- `src/orchestration/simulation/callbacks/density_callbacks/assembly.jl`: collect savers and wrap continuous effects after EDG observer assembly.
- `test/unit/simulation/event_output_tests.jl`: analytic and callback-lifecycle regression checks.
- `test/unit/runtests.jl`: include the new regression file.

## Findings

Claude's battery reproducer empties a 0.5 Wh initial charge at 6.1 seconds.
On unchanged main, the row at 6.0 seconds contains 0.012642884545096104 Wh;
the analytic value and saved solution give about 0.008196721311475556 Wh.
See `/tmp/spaceagora-event-output-baseline.log`.

The regular SavingCallback runs after continuous event effects. Those effects
can change the live integrator's state or RHS before it interpolates an output
time earlier in the shortened step. The repair drains pending output times
strictly before the event immediately before its effect, while the incoming
interpolant is still available. A sample exactly at the event retains its
existing post-effect saving position.

The callback assembly captures the SavingAffect objects before the EDG observer
wraps callbacks, then wraps continuous effects outside that observer. It
preserves callback options and passes original callbacks to their lifecycle
hooks. The same mechanism handles standard saved rows and the preallocated
TrajectoryRecorder, including an extra recorder that is the only saver. It
does not require retained dense solution history or insert additional solver
stops. It currently relies on the pinned DiffEqCallbacks SavingAffect fields;
that dependency needs review when upgrading the callback library.

## P1 Assessment

No P1 findings identified in this change. Full integration and independent review remain outstanding.

## Tests Added/Updated

- Original battery probe with the repair: the affected row and solution agree;
  `/tmp/spaceagora-event-output-repaired.log`, exit 0.
- New `simulation/event_output_tests.jl`: 16 assertions pass, exit 0, in
  `/tmp/spaceagora-event-output-focused.log`, Julia 1.12.1. Checks include
  analytic battery values with and without returned solution history, the
  preallocated recorder, scalar/vector events, post-event samples, and original
  callback identity in initialization/finalization.
- All 33 PR contract gates pass. The new test is included in the full unit runner,
  which is still running; no full-tree pass is claimed.
- On the separate combined I-9b worktree, 32 analytic checks also pass for store
  fill, store drain, station upload, row conservation and station counters, with
  and without returned solution history. That integration test requires the
  unmerged storage/station features and is not part of this main-based PR.

## Residual Risk

Complete combined-branch regression checks, checkpoint continuation,
solver-cache reuse, EDG observer interactions and existing recorder/result tests.
The initial tests do not prove those combinations. The old precompile-state
failure is being repaired separately and must be accounted for in final unit
results. This repair has not been independently reviewed.

Unresolved P1: No
