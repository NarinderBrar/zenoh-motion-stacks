# Motion stacks for Eclipse zenoh

Standalone native Zenoh motion processes and a versioned JSON contract. This repository currently ships a **navigation baseline**, a command line client, and a headless base and joint-arm mock. Manipulation planning and advanced navigation remain planned work.

## Build on Ubuntu 24.04

Install CMake 3.22+, Ninja, a C++20 compiler, Eigen, urdfdom, OMPL, nlohmann/json 3.11.3+, GoogleTest, zenoh-c 1.5.0+, and zenoh-cpp 1.5.0+ with its zenoh-c backend. The baseline installer checks Ubuntu 24.04 packages and installs missing ones. When needed, it adds the [official Eclipse Zenoh apt source](https://zenoh.io/docs/getting-started/installation/). It supports `--yes` and `--dry-run`:

```bash
./scripts/install_dependencies.sh --dry-run
./scripts/install_dependencies.sh
```

Zenoh-c must be installed before configuring this repository. If its development package is unavailable from your configured apt sources, use the [upstream Zenoh C++ guide](https://github.com/eclipse-zenoh/zenoh-cpp#how-to-build-and-install-it). The `system` preset uses installed packages:

```bash
cmake --preset system
cmake --build --preset system
ctest --preset system
```

`cmake --preset release` can fetch missing nlohmann/json and zenoh-cpp from checksum-pinned archives. It still requires an installed zenoh-c. The installer does not yet cover the advanced planning libraries. Install and packaging targets are available only when all dependencies were found on the system.

Install the current binaries and libraries with `cmake --install build/system --prefix <directory>`, or create a tarball with `cpack --config build/system/CPackConfig.cmake`.
The exported CMake package is `motion_stacks`, with
`motion_stacks::contract`, `motion_stacks::contract_transport`, and
`motion_stacks::navigation_core` targets.

`docker build -t motion-stacks .` runs the same baseline installer, build, and live tests in an Ubuntu 24.04 image. The container path has not yet been verified locally. The tarball uses installed system libraries; it does not bundle dependency binaries.

## Run a local demonstration

Open three terminals from this repository. The mock robot publishes odometry and stops if commands cease:

```bash
build/system/zms_mock_robot demo/robot1
```

```bash
build/system/zms_navigation --config config/examples/navigation.json
```

```bash
build/system/zms_cli demo/robot1 operator-1 goal goal-1 1 0 0
build/system/zms_cli demo/robot1 operator-1 status
```

Press Ctrl+C in the first two terminals to stop them. The stack can also be addressed by any client that follows [`docs/contract.md`](docs/contract.md). The executable reports its supported versions with `--version` and validates configuration with `--check-config <file>`.

To exercise the joint-arm mock, start it with `--joints joint_1,joint_2` and publish a target with
`build/system/zms_cli demo/robot1 operator-1 mock-joints joint_1,joint_2 0.3,-0.2`.
The CLI also understands manipulation target and cancel queries, which will become usable when
the manipulation server lands.

## Status and scope

The repository is at an early implementation stage. Navigation currently follows waypoints, plans through occupancy grids with A*, and stops on close forward scan returns. It does not use OMPL. Manipulation, advanced planning, full dependency automation, clean-container verification, and desktop client integration remain open. The default test suite covers codecs and a fake stack. The optional native Zenoh integration test starts the stack and mock robot, then checks joint commands, goals, routes, mapped paths, busy rejection, cancel, stale odometry, obstacle stop, and shutdown:

```bash
cmake --preset system -DZMS_ENABLE_LIVE_TESTS=ON
cmake --build --preset system
ctest --preset system
```

The conformance executable can target a separately running navigation stack with `ZMS_CONFORMANCE_PREFIX=<robot-prefix> build/system/zms_conformance`. See [`docs/navigation.md`](docs/navigation.md) and [`docs/manipulation.md`](docs/manipulation.md).

The source is licensed under Apache-2.0. See [`LICENSE`](LICENSE).
Eclipse zenoh is a trademark of the Eclipse Foundation. This independent
project is not affiliated with or endorsed by the Eclipse Foundation.
