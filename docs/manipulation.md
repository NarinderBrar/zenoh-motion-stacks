# Manipulation

`zms_manipulation` moves a URDF chain to joint or end-effector pose targets along collision-free paths. It reads the chain from `base_frame` to `eef_frame`, accepts revolute and prismatic joints with finite position and velocity limits, and rejects unsupported or mimicked joints. A target must name every controllable joint exactly once. Targets outside URDF limits are rejected before execution.

Start the bundled joint mock and the stack in separate terminals:

```bash
build/system/zms_mock_robot demo/robot1 --joints joint_1,joint_2
build/system/zms_manipulation --config config/examples/manipulation.json
```

Start a result watcher in a third terminal, then send a joint target from a fourth:

```bash
build/system/zms_cli demo/robot1 operator-1 watch manipulation move-1 5
build/system/zms_cli demo/robot1 operator-1 joint-target move-1 joint_1,joinA pose target is solved with damped least-squares inverse kinematics on the chain, using joint limits and up to 32 random starts; the collision-free solution closest to the measured joints is used. A target that no collision-free solution reaches is rejected with `unreachable`. A joint target in collision is rejected with `invalid_target`.

The stack then plans a joint-space path from the measured joints with RRT-Connect, checking every state along each edge at 0.02 rad resolution, and shortens it. Planning runs on a worker thread while the command reports `pending`. It fails with `start_in_collision` or `planning_failed` (including `planning_timeout` after `planning.max_time_s`).

Collision checking uses FCL with the URDF collision boxes, cylinders, and spheres of every link, inflated by `planning.collision_padding`. Mesh geometry is skipped and counted by `--check-config`. Link pairs that are directly connected, or that touch in the all-zero reference pose, are not checked against each other. Static obstacles come from `planning.environment` in the base frame.

The path is timed so all joints start and stop together, within each joint's URDF velocity limit and `max_acceleration`. The stack streams the timed samples on `cmd_joints` at `control_rate_hz`, publishes progress and tracking error, and succeeds once the joints are within `position_tolerance` after the trajectory ends. It fails with `goal_not_reached` if they do not settle within `planning.settle_timeout_s`, and with `stale_joint_states` or `tracking_error` as before. A goal from another client is rejected while the stack is busy. The owner can replace or cancel a command. Cancel and shutdown command the latest measured positions.

The `features` status field lists `urdf_joint_limits`, `joint_target`, `pose_target`, `self_collision`, `environment_collision`, `rrt_connect`, and `synchronized_trajectory`. Not yet implemented: Cartesian straight-line moves, gripper commands, obstacles updated at runtime, mesh collision, and a jerk-limited stop on cancel.

der development.

Configuration version 1 requires `robot_prefix` (a string, which may be empty for unprefixed keys), `urdf_path`, `base_frame`, and `eef_frame`. Relative URDF paths resolve beside the JSON file. Optional fields are `control_rate_hz` (1 to 200, default 20), `state_timeout_s` (positive seconds, default 1), `tracking_error` (positive radians or meters, default 0.4), `position_tolerance` (positive radians or meters, default 0.02), and `max_acceleration` (positive radians or meters per second squared, default 2). Unknown fields are errors. Validate without starting Zenoh with `zms_manipulation --check-config config/examples/manipulation.json`; use `--version` to inspect supported contract and config versions.

The optional `planning` object accepts `max_time_s` (default 2), `collision_padding` (meters, default 0.01), `ik_position_tolerance` (meters, default 0.003), `ik_orientation_tolerance` (radians, default 0.02), `settle_timeout_s` (default 2), and `environment`, an array of up to 256 obstacles. Each obstacle has `name`, `type` (`box` with `size` [x, y, z], `cylinder` with `size` [radius, length], or `sphere` with `size` [radius]), `position` [x, y, z], and optional `orientation` [x, y, z, w]. See `config/examples/mobile_manipulator.json`.
