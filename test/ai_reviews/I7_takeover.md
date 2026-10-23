# Laser physics: takeover verification and geometry repair

## Scope

I-7a/I-7b for 0.7.0, based on #502 (`0ca1afa5de5f49aabd66d8297c763f6b4fdcb17e`).
Preserves Claude's contract commit `93765cdc6cfeafc2ac7efc3903c4bb22ce008f5b`
and laser implementation `97e9b04cadd0c0ba8424e71424b0f9b47727aab3`.
Codex inspected these changes independently, reproduced a mounted-ray defect,
and authored the follow-up repair and regression. That repair still needs an
independent reviewer; this record does not constitute that review.

The terminal-aware availability/wrench hooks retain the four-argument defaults.
LaserBeamLink offers Gaussian capture, occultation, ideal pointing and mounted
recoil, while its all-default path reproduces LaserMomentumLink. The geometry
repair makes pointing/mounted configurations use the actual terminal ray for
force, range, capture, occultation and pointing. Configurations without either
option retain body-reference geometry, documented explicitly. No changes to the
fields of the old laser, power or data kinds are introduced.

## Changed Files

- `src/SpaceAGORA.jl`: public exports.
- `src/models/gnc/control/interlinks/interlink_scheduling.jl`: policy forces from the terminal-aware wrench.
- `src/models/vehicle/interlinks/interlink_graph.jl`: terminal-aware availability and force/torque aggregation.
- `src/models/vehicle/interlinks/links.jl`: hook defaults, terminal pose and requirement contracts.
- `src/models/vehicle/interlinks/terminals.jl`: mounted pose lookup and endpoint hardware preparation.
- `src/models/vehicle/interlinks/laser_beam_links.jl`: opt-in physics and corrected physical ray.
- `src/models/vehicle/interlinks/power_links.jl`: beam helper documentation; existing power physics unchanged.
- `src/models/vehicle/interlinks/data_links.jl`: requirement-message dispatch; existing data physics unchanged.
- `src/orchestration/simulation/engine/dynamics_rhs.jl`: rigid-body I^-1 tau contribution.
- `src/orchestration/simulation/engine/setup.jl`: attitude requirements.
- Registry/public API docs, interlinks/frames docs, the gve example, and unit runner/laser tests.

## Findings

**F1 (P2), fixed: mounting torque used a different ray from the force.** On
`97e9b04c`, two terminals offset by +1 m and -1 m laterally, on spacecraft 1 km
apart, received forces parallel to the body-reference separation and mount
lever-arm torques. At 40 W and magnification 100 the net mechanical angular
momentum rate was 2.6685127615852164e-5 N m despite equal/opposite forces. The
beam direction was wrong by 0.0019999973333397333 rad. Rotating a spacecraft
also exposes the defect even with equal body-frame offsets.

The repair transforms mount offsets from body to inertial axes and uses one
terminal ray consistently. Separation is evaluated as the body-position
difference plus the offset difference, avoiding cancellation of small offsets
against large inertial coordinates. Torque remains the mounted lever arm crossed
with that force in body axes. All-default force/availability arithmetic is kept.

Six new assertions fail on the original implementation: force direction and net
angular moment for two attitudes, pointing-cone rejection, and range rejection.
The first negative run also hit a keyword typo in the new Gaussian fixture; that
typo was corrected without changing tolerances. Both failed and successful logs
are retained. Existing laser checks passed before the repair.

## P1 Assessment

No P1 finding identified in this bounded review. The mounted-ray P2 finding is
covered by the repaired regression. This does not qualify the final combined
0.7.0 release or waive independent review of the Codex-authored follow-up.

## Tests Added/Updated

Executed locally with Julia 1.12.3, `--compiled-modules=existing`, the supported
CI shard entrypoint and isolated pinned EDG/HYPR provider environments:

- `interlink_laser_physics_tests.jl`: 152/152 assertions pass. The 14 new
  assertions cover unequal offsets, rotated attitude, force direction, total
  angular moment, each lever-arm torque, range, pointing, Gaussian capture and
  an occultation case where terminals clear a sphere that blocks the centers.
- Updated the existing rotated-mount assertion to use its actual beam force;
  the new analytic direction test independently checks that force.
- All 33 PR contract gates pass.
- Six related files pass all 508 assertions: scoring 24, checkpoint 135,
  terminal 69, power 95, data 136, and contract 49. Both shard reports list
  no failed files.

Logs: `/tmp/spaceagora-i7-before.log`, `/tmp/spaceagora-i7-after.log`,
`/tmp/spaceagora-i7-gates.log`, `/tmp/spaceagora-i7-related.log`; copied to the
October 23 takeover evidence archive.

## Residual Risk

- Independent review of the new geometry repair and hosted CI remain pending.
- This is stacked on #502 and follows the 0.6.1 release; no release tag is implied.
- The all-default bitwise regression is rerun in the laser file. The full
  historical fingerprint campaign and author's mutation campaign were not
  independently rerun here; old logs are not substituted for execution.
- The beam is an engineering equal/opposite-force model, not an optical cavity
  or escaped-photon model. Power/data pointing, jitter, slew and attenuation
  remain outside I-7a/I-7b.
- With `recoil=:central`, torque is deliberately omitted. Full angular-momentum
  conservation for offset endpoints is asserted for `recoil=:mounted`.
- Identical immutable terminals on different mount poses remain explicitly
  rejected by terminal_pose; no claim of resolving that existing ambiguity.
