# Official route progress design

## Source and integrity gate

The manager reads the immutable repository file
`2026_molit_comp_global_path (3).txt`; it never rewrites or normalizes it. Startup
requires all of the following:

- 4,430 parsed points;
- SHA-256 `50658991e607d9339d76e4cd6cb169dfc733ea53b93de2c3e222460bb497cc05`;
- XY length within 0.1 m of 2,184.612 m;
- first and last XY positions within 0.05 m.

The source contains 38 consecutive duplicate XY points. They remain in the
published global path and retain their original indices. Only the private
projection geometry removes them, preventing zero-length division and undefined
yaw. A non-zero neighbouring segment supplies orientation for duplicate points.

## Progress matching

The first accepted pose is projected against the full route. After initialization,
only a bounded window around the preceding segment is searched. Position distance
and heading disagreement form the candidate score, and candidates with more than
100 degrees of heading error are rejected. Accepted scalar progress is clamped to
be nondecreasing, so localization jitter cannot move checkpoint or link context
backward.

The 6 m lateral acceptance radius accommodates an adjacent K-City lane. It is not
a lane-safety threshold and must never be used as one; `path_planning_pkg` owns the
vehicle-footprint-to-boundary clearance checks. A pose outside this radius, a
parent frame other than `map`, a child frame other than `base_link`, invalid
quaternion, a non-finite position/twist/covariance value, any future timestamp, or
odometry older than 0.20 s produces `RouteContext.valid=false`. Validation occurs
before the monotonic accepted-stamp watermark is updated, so a rejected future or
wrong-frame sample cannot block a later valid sample. Invalid observations do not
change the last accepted matcher state and never trigger an unrestricted mid-run
global reacquisition. Out-of-order odometry is discarded. After an intentional
simulation-clock reset, restart this node to begin a new route run.

A valid `RouteContext.header.stamp` is the accepted odometry estimate time. Every
invalid context, including missing, rejected, stale, heading-mismatched, and
off-route input, uses the context publication time instead. This makes invalid
output observability unambiguous without presenting a rejected sample as accepted.

## HD-map link context

The ordered link spans in `config/competition_route.yaml` are the result of
projecting the official reference path onto canonical KATRI MGeo centerlines.
They use rounded tenths of a metre, so sub-metre gaps are resolved to the nearest
span edge. This preserves the audited order and does not infer a new topology.

The regulation speed-limit exception is true from the start of
`A2256W000411` through the end of `A2256W000153`, inclusive. If route context is
invalid, the flag is always false and both current/horizon link ID fields are
cleared. A planner therefore cannot authorize high-speed behavior or a lane
change from stale localization.

## Runtime validation still required

Unit tests cover parsing, source-index preservation, duplicate removal,
monotonic projection, heading/off-route rejection, time/frame rejection, link
selection, the exact 40-span competition configuration and inclusive high-speed
bounds. When the pinned source map is present, every configured route Link ID is
also checked against its `link_set.json`. MORAI closed-loop validation must still
measure localization time alignment, adjacent-lane projection, lap-end behavior,
and the correspondence between the runtime scene and the supplied KATRI map
before competition use.
