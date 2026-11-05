# robot_remote_control transport proxy (src/TransportProxy)

`GenericProxy` (library `robot_remote_control-transport_proxy`) forwards one robot_remote_control connection
between two transports; `robot_remote_control-web_proxy` (WebProxyMain.cpp) uses it to connect a zmq robot to
the web interface in `../Web`. Usage is in README.md.

## Build and test

- Part of the library build (`src/CMakeLists.txt` adds this folder). The web proxy is only built if the http
  and websocket transports exist (cpprestsdk, websocketpp); it links Boost program_options and `stdc++fs`.
- Tests: `test/test_Proxy.cpp`, executable `test_proxy` (CI step `test_proxy`). Ports 7211-7232, run from the
  repository root (it transfers `./test/testfiles`).
- The workspace `env.sh` puts the installed libraries on `LD_LIBRARY_PATH`, which wins over the build tree.
  After changing `Transports/Transport.hpp` (or other ABI), run tests and binaries from a build folder with
  `env -u LD_LIBRARY_PATH`, otherwise the old installed libraries are loaded and fail in strange ways.
- End to end: a fake robot in zmq mode (`interaction/executables/rrc_test_robots` with a config on free
  ports), the web proxy in front of it, then the requests the browser sends (http.client in Python, plus a
  minimal websocket client for telemetry). No browser or javascript engine is available in the container.

## How GenericProxy works

- One proxy per connection: `COMMANDS` (requests from the controller transport go to the robot transport, the
  reply goes back) or `TELEMETRY` (from the robot transport to the controller transport). The constructor checks
  the transport support flags (e.g. zmq REQ/SUB on the robot side, zmq REP/PUB, http or websocket servers on
  the controller side).
- Same serialization on both sides: messages are forwarded without parsing. Different serialization (binary
  zmq vs json http/websocket): messages are re-serialized, which needs the message type:
  - commands: the type per `ControlMessageType` (`registerCommandType`), version requests have no payload
  - telemetry: the type per `TelemetryMessageType` (`registerTelemetryType`), must match the types the
    `ControlledRobot` registers; extended libraries register their own types
  - replies depend on the request: `TELEMETRY_REQUEST` -> the requested telemetry type, `FILE_REQUEST` ->
    `FolderTransfer`, version requests -> plain text (unchanged), everything else -> `ControlMessageReply`

## Constraints that are not obvious

- The controller transports expect exactly one reply per request (zmq REP, and `TransportHttp::send()` answers
  the request returned by the last `receive()`): always send a reply, an empty one on errors and timeouts.
- The robot zmq REQ socket is relaxed/correlated, so a new request can be sent after a timeout; late replies
  are dropped. File and map requests get the long timeout.
- Empty replies are valid (telemetry that was never set; in binary also any all-default message). They are
  forwarded as empty, never converted (json would turn them into `{}` = data). `Transport::receive()` returns
  0 for them like for "nothing received": use `receiveMessage()`, which the queue based transports implement.
  `RobotController::sendRequest()` relies on it as well.
- `Serialization::serialize()` in json mode appends to the output string: always serialize into an empty string.
- `TransportWebSocket` in text mode rejects payloads that are not valid utf-8 (it shows as "A payload contained
  invalid data"), that is usually a binary message that was not converted.
- `reportError()` prints each distinct error on its 1st, 10th, 100th ... occurrence: keep messages constant
  (no changing numbers in the text except type ids), or they are printed every time.

## Web proxy

- Serves the web interface at `/html/`, the http api (commands) and the telemetry websocket; the models
  folder is served at `/model/`. `TransportHttp::serveFolder()` must be called before requests arrive (the
  folder map of the RestServer is not locked), so the models root is served at startup and the downloaded
  model just appears in it.
- The model cache runs in a detached thread: it polls `requestRobotName()` until `isConnected()` (also true for
  an empty name). Do not switch to `waitForConnection()`, it needs heartbeats and a telemetry connection (decided
  with the user).
- `models/<modelFolderName(robot name)>/`: the folder name rule must stay identical to `modelFolderName()` in
  `../Web/js/model-loader.js` (A-Z a-z 0-9 _ . - kept, one `_` per other character, utf-8 aware).
  The folder is only downloaded if it does not exist, `--force-model-download` deletes it first. Downloads go to
  `<folder>.download` and are renamed when complete.
- `model.json` in the model folder (`{path, modelfilename, files}`, files relative to the folder) is the
  interface to the web interface, change both sides together.

## Style

Like the rest of the library: 4 space indentation, camelCase, `printf` for output, doxygen `@brief` comments.
