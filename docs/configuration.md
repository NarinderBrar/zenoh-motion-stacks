# Navigation configuration version 1

Run `zms_navigation --check-config <file>` before starting a stack. Unknown fields and missing required fields are errors. `--version` prints stack, contract, and configuration versions as JSON.

The top level requires integer `version: 1` and string `robot_prefix`, which may be empty for unprefixed keys. Optional fields are `control_rate_hz` (1 to 200, default 20), `odom_timeout_s` (positive seconds, default 1), `scan_timeout_s` (positive seconds, default 1), `scan_ignore_radius` (nonnegative meters, default 0; returns closer to the base centre are ignored), `footprint_radius` (positive meters, default 0.25), `obstacle_margin` (positive meters, default 0.1), `progress_timeout_s` (positive seconds, default 15; a command fails with `no_progress` when the robot gets no closer to its current waypoint, by distance or heading, for this long), `avoidance` (an object with `enabled` (boolean, default true), `horizon_s` (positive seconds, default 1.5), `recovery_after_s` (positive seconds, default 2), and `local_window_m` (positive meters, default 6)), `scan_offset` (object with `x`, `y`, `yaw`, default all 0), `localization` (object with `enabled` (default false), `particles` (50 to 5000, default 400), and `initial_pose` with `x`, `y`, `yaw`), `visualization` (object with `enabled` (boolean, default true), `costmap_rate_hz` (positive seconds^-1, default 1), and `path_rate_hz` (positive seconds^-1, default 5); controls the `navigation/path`, `navigation/local_path`, `navigation/costmap`, and `navigation/obstacles` outputs), and `limits`. Limit fields are positive finite numbers:

| Field | Default | Unit |
|---|---:|---|
| `max_linear_speed` | 0.5 | m/s |
| `max_angular_speed` | 1.0 | rad/s |
| `max_linear_acceleration` | 0.5 | m/s² |
| `max_angular_acceleration` | 1.0 | rad/s² |
| `position_tolerance` | 0.05 | m |
| `yaw_tolerance` | 0.1 | rad |
| `lookahead_distance` | 0.4 | m |

See [`config/examples/navigation.json`](../config/examples/navigation.json) for a complete file.
