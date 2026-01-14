# Transports

RRC lets you select a transport of your choice, 

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


