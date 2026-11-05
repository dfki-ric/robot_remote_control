# Transport proxy

`GenericProxy` forwards a robot_remote_control connection between two transports, so controllers can use
other transports (or serializations) than the robot offers. One proxy forwards one connection:

```c++
// robot side: connects to the robot like a RobotController, controller side: offered like a ControlledRobot
GenericProxy commands(GenericProxy::COMMANDS, robotCommandTransport, controllerCommandTransport);
GenericProxy telemetry(GenericProxy::TELEMETRY, robotTelemetryTransport, controllerTelemetryTransport);
commands.startUpdateThread(5);
telemetry.startUpdateThread(5);
```

When both transports use the same serialization, messages are forwarded unchanged. Otherwise (e.g. binary
zmq to json http/websocket) they are converted, which needs the message types: all robot_remote_control
types are registered, libraries that add their own types register them with `registerCommandType<Type>(id)`
and `registerTelemetryType<Type>(id)`. Command replies that do not arrive within `setTimeout()` are answered
with an empty reply.

## robot_remote_control-web_proxy

Connects to a robot via zmq and offers it to browsers: the web interface (`src/Web`) and the http api for
commands, and a websocket for telemetry. This way the web interface can be used with robots that only use zmq:

    robot_remote_control-web_proxy <robot host> [--commandport 7001] [--telemetryport 7002] [--httpport 8001] [--wsport 8002]

then open http://localhost:8001/. `--help` shows all options.

When the robot is reachable, the proxy downloads its model (`RobotController::requestRobotModel()`) into
`models/<robot name>/` (`--models` sets the folder) and serves it at `/model/<robot name>/`, with a
`model.json` index the web interface uses to load the model from there, instead of transferring it from the
robot on each page load. A model that was downloaded before is used again, `--force-model-download`
deletes it and downloads it again.
