# robot_remote_control web interface (src/Web)

Browser interface for robots using robot_remote_control: plain ES modules, no framework, no build step,
no npm. Usage is in README.md.

## Setup and testing

- `./download_libraries.sh` downloads three.js and urdf-loader into `lib/` (git ignored). Versions, sha512
  checksums of the npm archives and the list of extracted files are pinned in the script. When updating a
  version: update the checksum (`sha512sum` of the `.tgz`) and check the file list, it must contain every
  module the used modules import (e.g. ColladaLoader imports `collada/*.js` and `TGALoader.js`).
- Serve it with the fake robots: `rrc_test_robots --html <this folder> [config.yml]` (in
  `interaction/executables/rrc_test_robots`), then open http://localhost:7001/. Robot r1 of its default
  config uses ports 7001 (http) and 7002 (websocket).
- Robots using zmq are reached through `robot_remote_control-web_proxy` (src/TransportProxy), which converts
  the binary messages to json: the interface must not depend on anything the robot's TransportHttp does
  besides `/api/command`, the `/api/*` shortcuts, and serving the files.
- There is no javascript engine or browser in the development container. Verify protocol changes by sending
  the same requests with Python (`http.client`, plus a minimal websocket client for telemetry) against the
  fake robot, and check that every import resolves through the import map and every `$('id')` exists in
  index.html. Say what could not be checked in a browser.

## Structure

- `index.html`: layout, the import map (`three` and `three/examples/jsm/` point into `lib/three/`).
- `js/rrc-client.js`: protocol client. `command()` POSTs a ControlMessage to `/api/command`,
  `requestTelemetry()`, telemetry websocket with reconnect, heartbeats, `LatestValueSender` for continuous
  commands (one request at a time, in order, newer values replace pending ones).
- `js/app.js`: all panels and their wiring to telemetry and commands. The 3D view has a `Region` above
  (image views) and below (log), each with a splitter and a toolbar toggle; the sidebar holds the other
  panels as collapsible cards. The layout (open cards, image views, region heights and visibility) is kept in
  `localStorage` through `storageGet()`/`storageSet()`, which tolerate unavailable storage: the page must
  work without it.
- `js/viewer.js`: three.js scene. Everything robot related is in `world`, a group rotated by -90° around x,
  because robot_remote_control uses z up and three.js y up: use robot coordinates inside it.
- `js/model-loader.js`: loads the urdf model from the http server if it hosts one at
  `/model/<modelFolderName(robot name)>/model.json` (`{path, modelfilename, files: [paths relative to the folder]}`,
  written by robot_remote_control-web_proxy, which uses the same folder name rule), otherwise downloads it like
  `RobotController::requestRobotModel()` (ROBOT_MODEL_INFORMATION, then FILE_REQUEST). Mesh urls (package://,
  file://, relative) are mapped to the model files by the longest matching path suffix.
- `js/input.js`: on screen joysticks, keyboard and gamepad input.

## Protocol facts the code relies on

- Messages use the protobuf json format: lowerCamelCase field names (`valueNames`, `childFrameId`), enums as
  names, and **fields with default values are omitted** (a pose at the origin is `"position": {}`, channel 0
  is missing): always read numbers with `|| 0` and arrays with `|| []`.
- Websocket messages are TelemetryMessages: `{type, channel, json}`, `json` is the stringified telemetry.
- `TELEMETRY_REQUEST` replies with the telemetry json, or an empty body if the robot never set it.
  Command replies are ControlMessageReplies (`{"type": ...}`), version requests reply with plain text.
- The page must come from the robot's TransportHttp: the robot does not allow cross origin requests.
- The robot's frame names in telemetry headers may be prefixed (e.g. `r1_laser_link` for the urdf link
  `laser_link`), `Viewer.frameObject()` matches them by suffix.
- Twist commands may not time out on the robot: always send a zero twist when the input stops, heartbeats
  let the robot stop when the page is closed.

## Style

4 space indentation, single quotes, semicolons, ES2020 modules, doxygen style `/** */` comments on classes and
non-obvious functions. DOM is created with the `element()` helper in app.js, no innerHTML with robot data.
