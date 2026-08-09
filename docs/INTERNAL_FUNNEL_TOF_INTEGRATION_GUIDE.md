# Future internal funnel ToF integration guide

## Boundary

The current matrix pickup flow ends with an assumed funnel handoff and an
unconfirmed forward feed. It must not publish payload confirmation, material
identity, or increment a payload count. A future internal ToF is a new evidence
source at the capture mechanism; it is not a replacement for front collision
sensing and must not weaken the final motor safety gate.

## Required observation contract

Add a centrally configured mount, bus/address/XSHUT identity, valid range and
status semantics, freshness limit, sequence number, acquisition timestamp,
signal rate, and ambient rate. Publish immutable observations. Confirmation
counters may advance only once per newer sequence.

## Proposed payload states

Use typed states such as `EMPTY_CONFIRMED`, `CAPTURE_ENTRY_SEEN`,
`PAYLOAD_PRESENT_UNCLASSIFIED`, `PAYLOAD_PRESENT_CLASSIFIED`, and `UNKNOWN`.
The current `WEIGHT_FUNNEL_HANDOFF_ASSUMED` event may transition only to entry
seen or unknown. It must never directly become payload present.

Require a calibrated temporal/range signature across the funnel entry and
settled payload position, with contradictory, stale, invalid, saturated, or
single-sample evidence resolving to `UNKNOWN`. Material classification, if it
is later shown to be separable, must be an independent confidence result and
must not be a motion-safety input.

## Route and failure behavior

Payload count may increment only after a fresh confirmation observation tied
to the active track and capture attempt. A timeout or safety-vetoed feed leaves
payload state unknown and does not increment. Duplicate observations from the
same capture attempt must be idempotent. Unload confirmation needs a separate
empty transition before another collection can be counted.

## Characterization gate

Before implementation thresholds are enabled, save stationary and controlled
capture traces for empty funnel, valid weights, wall/contact interference,
partial capture, bounce, double object, ambient extremes, cross-talk, and
unload. Hardware access and all movement remain separately authorized under
the repository safety rules.
