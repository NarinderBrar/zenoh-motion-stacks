# Zenoh motion contract

The current implemented contract major version is **1**. Every payload this repository sends is UTF-8 JSON with Zenoh encoding `application/json`. The `contract` field is the integer `1`. Clients must reject an unknown major version. Command identifiers and client identifiers are nonempty strings of at most 128 characters.

The stacks and the mock robot enable the Zenoh admin space, so monitoring tools can list their sessions, publishers, subscribers, and queryables.

All keys below are relative to the configured robot prefix. A prefix has no leading slash. For example, prefix `demo/robot1` and suffix `odom` form `demo/robot1/odom`. An empty prefix uses the bare suffixes, such as `odom` and `cmd_vel`.

| Key suffix | Zenoh operation | Current implementation |
|---|---|---|
| `navigation/goal` | queryable | Submit waypoint route; accept or reject reply |
| `navigation/cancel` | queryable | Cancel matching command owned by caller |
| `navigation/status` | queryable | Current state, version, features |
| `navigation/feedback` | publish | Current waypoint index and pose |
| `navigation/result` | publish | Terminal command state |
| `navigation/alive` | liveliness token | Presence |
| `navigation/path` | publish | Planned global path and empty path on completion |
| `navigation/local_path` | publish | Local detour while avoiding |
| `navigation/costmap` | publish | Inflated occupancy grid the planner uses |
| `navigation/obstacles` | publish | Remembered obstacle points |
| `cmd_vel` | publish | Linear and angular velocity |
| `odom` | subscribe | Base pose |
| `scan` | subscribe | Optional 2D range scan; forward obstacle stop |
| `map` | subscribe | Optional occupancy grid for A* planning |
| `manipulation/target` | queryable | Submit a joint or end-effector pose target |
| `manipulation/cancel` | queryable | Cancel an owned manipulation command |
| `manipulation/status` | queryable | Current state, version, and supported features |
| `manipulation/feedback` | publish | Joint execution progress and tracking error |
| `manipulation/result` | publish | Terminal command state |
| `manipulation/alive` | liveliness token | Presence |
| `manipulation/trajectory` | publish | Planned joint trajectory and end-effector path |
| `cmd_joints` | publish | Joint command samples |
| `manipulation/gripper` | queryable | Open, close, or position the configured gripper |
| `localization/pose` | publish | Map-frame pose estimate when localization is enabled |
| `localization/initial_pose` | subscribe | Map-frame start pose for localization |
| `joint_states` | subscribe | Joint positions and velocities |

## Navigation requests

Submit one or more waypoints, in order:

```json
{"contract":1,"command_id":"goal-1","client_id":"operator-1","waypoints":[{"x":1.0,"y":0.0,"yaw":0.0}]}
```

The queryable replies with `{"contract":1,"accepted":true,"command_id":"goal-1","client_id":"operator-1"}`. A rejected request has `accepted:false`, `reason`, and `message` fields. Defined reasons are `invalid_request`, `busy`, and `not_owner`. A goal, target, or cancel request that the stack cannot answer within 2 seconds is answered with `invalid_request` and is discarded; it never takes effect later.

Cancel uses `{"contract":1,"command_id":"goal-1","client_id":"operator-1"}`. Cancelling while idle returns `{"contract":1,"state":"idle"}`. A different client cannot cancel the active command. A new goal from the same owner replaces it and publishes a cancelled result for the old command.

`navigation/status` takes no payload. It reports `contract`, `state`, `stack_version`, `features`, and `map_available`. While a command is pending or active it also reports `command_id` and `client_id`. States are `idle`, `pending`, `active`, `succeeded`, `cancelled`, and `failed`; only `pending`, `active`, and `idle` appear in status replies in this release.

Feedback contains `contract`, `command_id`, `client_id`, `state`, `waypoint_index`, `x`, `y`, and `yaw`. `waypoint_index` is the zero-based index of the submitted waypoint the robot is currently driving toward, also when the stack plans its own path through a map. With obstacle avoidance, feedback also carries `blocked` (boolean) and `recovery` (`none`, `rotate`, or `backup`). Result contains `contract`, `command_id`, `client_id`, and a terminal `state`. Failures also contain `reason` and `message`. Defined failure reasons are `stale_odom`, `stale_scan`, `planning_failed`, `no_progress`, and `shutdown`. A replaced command reports reason `replaced`.

The robot publishes odometry as `{"x":0.0,"y":0.0,"yaw":0.0}` with optional velocity fields. The stack publishes `{"contract":1,"linear":0.2,"angular":0.0}` to `cmd_vel`. Linear speed is meters per second, angular speed and yaw are radians per second and radians, and position is meters in a consistent world frame. The robot must stop if velocity commands cease; the bundled mock stops after 0.5 seconds.

## Manipulation requests

`manipulation/target` accepts exactly one of `pose` or `joints`. Pose targets have finite `x`, `y`, `z` in meters and a unit quaternion `qx`, `qy`, `qz`, `qw`. The optional `base_frame` and `eef_frame` strings override the configured frames for this command:

```json
{"contract":1,"command_id":"pose-1","client_id":"operator-1","pose":{"x":0.2,"y":0.1,"z":0.4,"qx":0,"qy":0,"qz":0,"qw":1},"base_frame":"base","eef_frame":"tool"}
```

Joint targets carry matching arrays of 1 to 64 unique names and finite positions in radians. The stack rejects names outside its configured joint set and positions outside model limits:

```json
{"contract":1,"command_id":"joints-1","client_id":"operator-1","joints":{"name":["joint_1","joint_2"],"position":[0.3,-0.2]}}
```

The queryable reply, cancel request, command ownership rules, states, and terminal results have the same fields and meanings as navigation. `manipulation/feedback` additionally carries `progress` from 0 to 1 and `tracking_error` in radians. The stack publishes `cmd_joints` as `{"contract":1,"name":["joint_1"],"position":[0.3]}` and accepts `joint_states` as `{"name":["joint_1"],"position":[0.3],"velocity":[0.0]}`. All arrays must have equal length and names must be unique. A `joint_states` message may list up to 1024 joints, including joints outside the configured chain; the stack reads the chain joints by name and ignores the others. Measured positions are not checked against the URDF limits, so a small overshoot does not discard the message. A message that omits a chain joint is ignored.

## Visualization outputs

These optional outputs publish what the stacks plan internally so clients can draw it. They are always in the contract major version 1 and are controlled by the `visualization` config object. A client may ignore them.

`navigation/path` publishes the global path the stack is following, once each time a route is planned or re-planned and again when the command ends (an empty `poses` array). The path is the map A* route when a map is available, otherwise the submitted waypoints. `navigation/local_path` publishes the local detour from obstacle avoidance at up to `path_rate_hz` while the robot is blocked or recovering, and an empty path otherwise. Both share the payload shape:

```json
{"contract":1,"command_id":"goal-1","frame":"map","poses":[{"x":1.0,"y":0.0,"yaw":0.0}]}
```

`navigation/costmap` publishes the inflated occupancy grid the planner uses in exactly the same shape as the `map` input: `contract`, `width`, `height`, `resolution`, `origin` with `x`, `y`, and `yaw`, and a row-major `data` array. With a map it is the map inflated by the footprint radius plus obstacle margin and marked with remembered scan obstacles; without a map it is a local rolling window around the robot built from current and remembered scan obstacles. It publishes at most `costmap_rate_hz` and only when the grid changed. Above 40000 cells the grid is downsampled (larger cells, adjusted resolution and origin) before publishing.

`navigation/obstacles` publishes the remembered obstacle points in the odom/map frame at up to 5 Hz, capped at 10000 points:

```json
{"contract":1,"frame":"map","points":[{"x":1.0,"y":0.5}]}
```

`manipulation/trajectory` publishes the planned joint trajectory when a target is accepted and planned. `points` carries time in seconds and joint positions in radians and is downsampled to at most 200 points. For pose and Cartesian targets an `eef_path` array of `{x,y,z}` end-effector positions in the base frame is included:

```json
{"contract":1,"command_id":"pose-1","name":["joint_1","joint_2"],"points":[{"t":0.0,"position":[0.0,0.0]}],"eef_path":[{"x":0.3,"y":0.0,"z":0.2}]}
```

## Optional navigation sensors

`scan` is a JSON object with `contract`, finite `angle_min`, positive `angle_increment`, nonnegative `range_min`, positive `range_max`, and a `ranges` array in meters. A JSON `null` range means no return. The server considers valid returns in the forward half-plane; a reading inside the configured clearance stops motion. `map` is a JSON object with `contract`, positive integer `width` and `height`, positive `resolution` in meters per cell, `origin` with finite `x`, `y`, and `yaw`, and a row-major `data` array of length `width * height`. Cells are `-1` for unknown or 0 to 100 for occupancy probability. The server plans through cells below 50 and inflates blocked cells by the configured footprint radius plus margin. A map received after a goal starts applies to the next goal.

The manipulation server accepts targets when `joint_states` is fresh and contains every joint of the configured URDF chain. Pose targets are in the configured base frame; frame overrides that differ from the configuration are rejected with `unsupported`. An accepted reply also carries `joints`, the planned goal configuration as `name` and `position` arrays. Rejection reasons are `invalid_target` (outside limits or in collision), `unreachable` (no collision-free inverse kinematics solution), `state_unavailable`, `busy`, and `not_owner`. While the path is planned, status reports `pending`. Failure reasons are `start_in_collision`, `planning_failed`, `goal_not_reached`, `stale_joint_states`, `tracking_error`, and `shutdown`. Cancel commands the measured joint positions immediately.
