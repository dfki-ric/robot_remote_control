# Robot Remote Control web interface

A browser interface for robots that use robot_remote_control with the web transports:
commands via `TransportHttp`, telemetry via `TransportWebSocket` in `SERVER_TEXT` (json) mode.

* 3D view of the robot model (downloaded from the robot) at the current pose with the current joint values,
  driven trail, planned path, laser scans, point clouds and maps; set goals by clicking/dragging on the ground
* driving with on screen joysticks, keyboard (W/S, A/D, Q/E, space: stop) or a gamepad
* joint sliders, simple actions (incl. named values and dependencies), complex actions with points picked in the 3D view
* image region above the 3D view with any number of views side by side, each showing an image channel or a
  layer of image layers (e.g. rgb and depth), log region below the 3D view; both are resizable and can be
  hidden, the layout is remembered by the browser
* simple sensors, IMU and wrenches
* robot state, log level, permission requests, telemetry inspector, file downloads
* heartbeats, so the robot detects a lost connection

## Setup

The javascript libraries ([three.js](https://threejs.org), MIT and
[urdf-loader](https://github.com/gkjohnson/urdf-loaders), Apache-2.0) are not part of the repository:

    ./download_libraries.sh

downloads them into `lib/`. The folder is installed to `share/robot_remote_control/web`
(including `lib/` if it was downloaded before).

## Usage

The interface has to be served by the `TransportHttp` of the robot, the browser sends the commands to the
server it was loaded from:

```c++
auto commands = std::make_shared<TransportHttp>("http://0.0.0.0:7001");
auto telemetry = std::make_shared<TransportWebSocket>(TransportWebSocket::SERVER_TEXT, 7002);
commands->serveFolder("<prefix>/share/robot_remote_control/web", "html");
commands->setDefaultPage("/html/index.html?telemetryport=7002");
ControlledRobot robot(commands, telemetry);
```

Open http://robot:7001/ (or http://robot:7001/html/index.html?telemetryport=7002).

| url parameter | default | |
| --- | --- | --- |
| `telemetryport` | 7002 | port of the telemetry websocket |
| `commandport` | port of the page | port of the http commands (cross origin requests are not supported by the robot) |

For robots that only use zmq transports, `robot_remote_control-web_proxy` connects to the robot and serves
the interface locally (see [src/TransportProxy](../TransportProxy)):

    robot_remote_control-web_proxy <robot host>

then open http://localhost:8001/.

The robot model is loaded from the http server if it hosts one at `/model/<robot name>/model.json` (the web proxy
caches the model of the robot there), otherwise it is downloaded from the robot.

To test the interface without a robot, use the fake robots of `rrc_test_robots`:

    rrc_test_robots --html <this folder> [config.yml]
