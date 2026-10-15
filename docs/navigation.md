# Navigation

`zms_navigation` accepts a single active waypoint route and drives a differential drive base. The control loop runs at the configured rate. Requests, odometry, optional scans, and optional maps enter bounded queues; the control loop owns the active command. Grid search runs on a separate cancellable worker. It publishes zero velocity on cancel, odometry timeout, stale scan after scans have begun, obstacle stop, success, replacement, and process shutdown.

The current follower turns toward a waypoint, then drives toward it under speed and acceleration limits. It aligns with the waypoint yaw at the final target. Once it is turning at a target, it keeps turning while it stays within twice `position_tolerance`, so drift during the turn does not send it back to driving; it drives again only if it drifts further. A command fails with `no_progress` when neither the distance nor the heading error to the current waypoint improves for `progress_timeout_s`, including while stopped for an obstacle. When a map is available when a goal is accepted, the worker searches an inflated occupancy grid with eight-neighbor A* and prevents diagonal corner cutting. Unknown cells are blocked. A failed or out-of-bounds path publishes `planning_failed`. When scans are available, a return in the forward half-plane closer than the configured footprint radius plus margin stops motion until clearance returns. If a previously available scan goes stale, the active command fails. Without avoidance this is only a forward stop check; with `avoidance.enabled` the stack detours around obstacles, remembers them, and recovers, as described below. Use it only in a clear test area with an independent robot watchdog.

The `features` field of `navigation/status` lists exactly `waypoint_route`, `differential_drive`, `scan_obstacle_stop`, and `grid_astar`. OMPL planning and additional drive models are not advertised.

## Obstacle avoidance

With `avoidance.enabled` (the default) and a `scan` stream, the stack drives around obstacles instead of stopping in front of them. Each control cycle it converts the latest scan to points, assuming the scan frame is the base frame, and:

1. Picks a target: the current waypoint, or for a planned map route the route point `lookahead_distance` ahead.
2. Plans a detour to that target with A* on a rolling grid of `avoidance.local_window_m` around the robot, inflated by `footprint_radius + obstacle_margin`. If no detour is found, the robot stops and enters blocked recovery instead of following the obstructed line.
3. Chooses a velocity with a dynamic window search: candidate speed and turn-rate pairs within the acceleration limits are simulated for `avoidance.horizon_s`, discarded if they bring the circular footprint closer than `footprint_radius + obstacle_margin` to a scan point or cannot stop in time, and scored on progress toward a point on the detour, heading, clearance, and speed.

When the straight line to the current target keeps that clearance, the stack uses the waypoint follower instead, so open stretches and the final approach behave as before. Turning to a waypoint heading also uses the follower. When no candidate moves the robot for `avoidance.recovery_after_s`, the stack re-plans the remaining route through the map with the scan points marked as occupied (when a map is available), turns in place toward the side with fewer obstacles for 2 seconds, then backs up up to 0.2 m when the rear is clear. Feedback reports `blocked` and `recovery` (`none`, `rotate`, or `backup`). `progress_timeout_s` still ends a command that makes no progress. With `avoidance.enabled` false the stack keeps the earlier behavior and stops for close forward returns, reporting `obstacle_stopped`.

The `features` field then lists `waypoint_route`, `differential_drive`, `grid_astar`, `local_avoidance`, `obstacle_replanning`, and `recovery_behaviors`.

`scan_offset` gives the scanner pose in the base frame (`x`, `y` in meters, `yaw` in radians). Scan points are moved into the base frame before obstacle checks, avoidance, and localization.
`scan_ignore_radius` optionally filters returns closer than that distance to the base centre. It defaults to 0 m, which keeps all valid returns, including obstacles touching the robot. Set it only when measured self hits require filtering. Avoidance retains a bounded set of scan obstacles in the world frame for 3 seconds after their last return, so a nearby obstacle remains considered when its range drops below the scanner minimum. A return inside the inflated footprint blocks local detour planning and stops forward motion.

## Localization

With `localization.enabled`, the stack runs a particle filter against the `map` stream. Particles move with the odometry change using the standard odometry motion model (all four noise terms 0.2), are weighted against a likelihood field (distance from each scan point to the nearest occupied cell, 0.2 m spread, 5 % random returns, 60 beams), and are resampled when the effective particle count falls below half. The filter updates after 0.05 m or 0.05 rad of motion. Control, goals, avoidance, and map re-planning then use the map-frame pose, so drifting odometry is corrected. The start pose comes from `localization.initial_pose` or a put on `localization/initial_pose` (`{"contract":1,"x":…,"y":…,"yaw":…}`). The estimate is published on `localization/pose` at 5 Hz with a `spread` field (meters), status reports `localized`, and `features` adds `particle_filter_localization`. Global localization without an initial pose is not implemented.

## Visualization outputs

With `visualization.enabled` (the default) the stack publishes its internal plans for clients, described in `contract.md`: `navigation/path` (global route, re-published on each plan and cleared when a command ends), `navigation/local_path` (the local detour while avoiding or recovering), `navigation/costmap` (the inflated grid the planner uses, or a local rolling window when no map is available), and `navigation/obstacles` (remembered scan points). Rates come from `visualization.path_rate_hz` and `visualization.costmap_rate_hz`. The costmap is published in the same shape as the `map` input and downsampled above 40000 cells.

Not yet implemented: persistent costmap layers, omnidirectional and car-like drive models, and behavior trees.
