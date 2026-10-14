# Navigation configuration version 1

Run `zms_navigation --check-config <file>` before starting a stack. Unknown fields and missing required fields are errors. `--version` prints stack, contract, and configuration versions as JSON.

The top level requires integer `version: 1` and string `robot_prefix`. Optional fields are `control_rate_hz` (1 to 200, default 20), `odom_timeout_s` (positive seconds, default 1), `scan_timeout_s` (positive seconds, default 1), `footprint_radius` (positive meters, default 0.25), `obstacle_margin` (positive meters, default 0.1), and `limits`. Limit fields are positive finite numbers:

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
