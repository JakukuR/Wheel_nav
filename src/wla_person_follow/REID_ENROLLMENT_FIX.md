# ReID enrollment gap fix — 2026-10-20

The supplied continuous-follow log accepted one descriptor, stayed `enrolling`
with `initial_identity_not_confirmed`, then exceeded the 8 s search window.
Planner times were 33–47 ms with no output freshness blockers. It generated no
forward candidates because the target identity was not established. The old
`following` / `motion_permitted` status described the healthy solver/output
channel and misleadingly remained true while enrollment had failed.

Confirmed reproducible defect: after the first accepted descriptor at t=1.0,
a second correct, same-ID/same-position descriptor at t=1.7 was never accepted.
Even perfect subsequent frames at 1.8/1.9/2.0 remained stuck at one reference:
the ordinary direct-match path required age <= 0.6 s, while the recovery path
refused enrollment with fewer than two confirmations. The pasted log cannot
establish what caused the original missing/rejected second frame.

Fix:

- A late same-ID sample that still passes the existing appearance and position
  gates starts a fresh provisional pair. It does not complete enrollment.
  Another distinct matching frame within 0.6 s is still required.
- The enrollment deadline is fixed at identity initialization + 8 s; restarting
  a pair never extends it. No-feature enrollment also expires. Expiration is
  terminal for that identity instance and stops requesting target features.
- Initial enrollment cannot adopt another detector ID, weaken appearance
  similarity, skip position validation, or bypass confirmation.
- Incomplete and expired identity forces zero output through the existing
  publish function and reports `waiting` / `motion_permitted=false`. The
  already-enrolled target's normal lost-person search behavior is retained.
- Target-only crop diagnostics distinguish absence, overlap, clipping, crop
  size, and unavailable geometry. Resolver status reports the latest candidate
  similarity/position error, frame age, sample count, and pair restarts.
  The accepted-match `similarity` field is not evidence of a new successful
  identity match; inspect `last_candidate_similarity` for rejected candidates.

34 relevant tests passed: 18 identity cases; 10 isolated ROS cases including a
0.9 s initial feature gap, ambiguous rebind, mux/expiry/estop, and explicit
nonzero-command rejection during enrollment/timeout; 6 sidecar/crop/model tests.
The failing five-frame reproduction now reaches `tracking` after the fresh
pair. No physical following trial was initiated.

Deployment: `releases/sim-follow-20261020-reidfix/install`, loaded after the
latency overlay. The camera, detector, mux, and web viewer are not restarted.
Rollback on an idle robot: remove only the reidfix overlay line from
`r680_person_follow.sh` and restart the follower.
