# Local takeover review: ground stations with data stores

Prepared 2026-10-23 on Claude's local commit
`308bba1b53448f37493c610d8738aaa82f77dbbb`, after its combined parent
`8fdd50e9b`. The implementation was unpublished when inspected. This record accompanies
its draft publication with the verified test corrections and resume regression.

## Scope

Reviewed the station flow-node mapping, unbounded station supply/capacity,
integrated station counters, saved columns, and preservation of those counters
through checkpoint state renaming. The implementation retains the existing
backlog policy; counting ongoing production still awaits a decision.

Applied the same two inherited test corrections prepared separately for PRs
#510 and #512: the refusal-message assertions permit the added data-store item,
and the old index-name fixture uses a real empty InterlinkModel.

Added an upload/downlink checkpoint regression with a data store. It interrupts
the same 450-second mission after the 200-second checkpoint, resumes it, and
checks exact saved solution times/states, conservation and station output
counters. The whole reference uses the same checkpoint boundaries.

## Findings

The inherited source change maps ground stations to unbounded flow nodes,
accounts for their sent/received data in the integrated state, and preserves
those counters when renaming checkpoint components. No new source correction
was needed for the exercised upload, downlink and same-mission resume cases.

## P1 Assessment

No P1 identified in this review. Full verification and review of the new tests
remain pending; the draft is not release-ready.

## Tests Added/Updated

- `/tmp/spaceagora-i9b-gates.log`: all 33 contract gates, exit 0.
- `/tmp/spaceagora-i9b-interruption.log`: all 195 ground-station assertions
  pass, including the new 20 interruption assertions; exit 0.
- `/tmp/spaceagora-i9b-focused-prepared.log`: the inherited index, storage and
  thruster checks ran; the ground-station file failed its initial new exactness
  assertion. That initial test changed the configured mission duration from
  200 to 450 seconds as well as resuming. The diagnostic logs
  `/tmp/spaceagora-i9b-resume-diagnostic.log` and
  `/tmp/spaceagora-i9b-resume-early.log` retain a roughly 7e-12-bit discrepancy
  in the empty store's residual. Other state components and saved times were
  identical in that probe. The numerical cause has not been established.
  Exact continuation is demonstrated for an interruption of the same mission;
  bitwise equality after changing its configured duration is not claimed.
- The first focused invocation failed before tests because the optional EDG
  environment was not supplied; both pinned provider environments were prepared
  before the corrected invocation.
- The complete unit tree is running in `/tmp/spaceagora-i9b-full-unit.log`.
  It is not yet a passing result. The pre-existing precompile reset failure is
  being repaired in a separate checkout.

This review does not cover final post-0.6.1 integration, viewer station drawing,
the pending backlog-policy decision, or independent human acceptance. The new
test changes were authored here; they still need independent review.

## Residual Risk

The shared event-shortened output bug remains in this branch. The separate
#515 correction plus this integration passed 32 analytic data/station output
assertions and six related unit files in the station-output-integration
worktree, but that combination is not included in this PR.

This branch also inherits #510 and #511 while targeting #512. Retarget and
revalidate after those parents and 0.6.1 land. Do not merge this feature chain
into the 0.6.1 release.

Unresolved P1: No
