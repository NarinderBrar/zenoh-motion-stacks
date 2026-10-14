# Manipulation baseline

`zms_manipulation` currently executes direct joint targets for a URDF chain. It reads the chain from `base_frame` to `eef_frame`, accepts revolute and prismatic joints with finite position and velocity limits, and rejects unsupported or mimicked joints. A target must name every controllable joint exactly once. Targets outside URDF limits are rejected before execution.

Start the bundled joint mock and the stack in separate terminals:

```bash
build/system/zms_mock_robot demo/robot1 --joints joint_1,joint_2
build/system/zms_manipulation --config config/examples/manipulation.json
```

Start a result watcher in a third terminal, then send a joint target from a fourth:

```bash
build/system/zms_cli demo/robot1 operator-1 watch manipulation move-1 5
build/system/zms_cli demo/robot1 operator-1 joint-target move-1 joint_1,joint_2 0.5,-0.4
```

The stack streams velocity- and acceleration-limited position commands at `control_rate_hz`. Each joint brakes in time to reach its target at zero speed without exceeding `max_acceleration`. It publishes progress and tracking error, fails when joint state goes stale or tracking error exceeds the configured bound, and publishes a terminal result. A goal from another client is rejected while the stack is busy. The owner can replace or cancel a command. Cancel and shutdown command the latest measured positions.

This baseline has no pose IK, collision checking, path planning, TOPP-RA timing, or jerk-limited stop. Its `features` status field currently lists `urdf_joint_limits` and `joint_target`. Use it with the bundled mock while those capabilities are under development.

Configuration version 1 requires `robot_prefix` (a string, which may be empty for unprefixed keys), `urdf_path`, `base_frame`, and `eef_frame`. Relative URDF paths resolve beside the JSON file. Optional fields are `control_rate_hz` (1 to 200, default 20), `state_timeout_s` (positive seconds, default 1), `tracking_error` (positive radians or meters, default 0.4), `position_tolerance` (positive radians or meters, default 0.02), and `max_acceleration` (positive radians or meters per second squared, default 2). Unknown fields are errors. Validate without starting Zenoh with `zms_manipulation --check-config config/examples/manipulation.json`; use `--version` to inspect supported contract and config versions.
