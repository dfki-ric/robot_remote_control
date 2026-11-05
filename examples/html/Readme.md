# html

A minimal example, the complete web interface (with a 3D view of the robot) is in [src/Web](../../src/Web).

When rrc ist set up zo use websocket transport AND uses JSON serialization, bowsers are able to receive rrc data though the websocket.

Run a webserver (e.g. `python3 -m http.server`) in this folder to open the example in a browser.

Alternatively, the robot can serve this folder itself on the port of a `TransportWebSocket` or `TransportHttp` server:

```c++
std::dynamic_pointer_cast<TransportWebSocket>(telemetry)->serveFolder("examples/html");
```

and the example is available at http://robot:7002/html/index.html


