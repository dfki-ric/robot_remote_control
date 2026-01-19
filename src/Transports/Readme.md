# Transports

RRC lets you select a transport of your choice: 

# ZeroMQ

This is the default for robot_remote_control. For commands, the request/reply pattern is used, for telemetery publish/subcribe.

It allows multiple connections and supports auto-reconnect. Also a Proxy implementaion is available that runs only a single connection to the robot, but allows multiple local connections

# UDT

UDT is a protocol that is based on UDP, but with reliable delivery.
It is used in higk latency situations, but only allows a single connection.

# WebSocket

The Websocket transport works similar to the other transports, but has the advantage that is usable from brosers.

The robot side has to be running before connecting.
When using a Browser as client, the SERVER_TEXT mode should be used, which switches the serialization to json (see examples/html). 

# Http

http transport can be used for commands, the transport on the RobotController (SERVER) launches an REST server with a generic POST entpoint.
The POST endpoint is at /api/command and takes generic json commands of the protocol, the RovotController may use the http trasnport, but there are better options.

The http transport is meant to be used for browser-based access in combination with the WebSocket trasnports (http for commands and websocket for telemetry)

The http transport also offers additional data shortcuts with GET calls. You can see available calls when opening the api page in a browser (http://localhost:7001/api) you'll get an overview of available calls (and you can test them in the browser).

curl -X POST http://localhost:7001/api/command -H "Content-Type: application/json" -d ' {"type":"HEARTBEAT","json":"{\"heartBeatDuration\":1}"}' && echo


# Webbrowser Functionality

If http is used for commands and websocket for telemetry (in JSON serialization mode, SERVER_JSON), you can provide full web functionality for browser use.
