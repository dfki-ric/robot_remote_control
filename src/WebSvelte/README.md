# Robot Remote Control web interface (Svelte)

Browser interface for robots using robot_remote_control, the SvelteKit counterpart of `../Web`.
Commands go to the robot's `TransportHttp` (`/api/command`), telemetry comes from its
`TransportWebSocket` in `SERVER_TEXT` mode.

* 3D view ([urdf-viewer](https://codeberg.org/brean/urdf-viewer), threlte): robot model at the current pose
  and joint values, driven trail, planned path, laser scans, point clouds and maps; set goals and task points
  by clicking (and dragging for the orientation) on the ground
* driving with two joysticks of [svelte-gamepad-virtual-joystick](https://codeberg.org/brean/svelte-gamepad-virtual-joystick):
  on screen, keyboard (default: W/S forward, A/D sideways, Q/E turn, Space stops, H shows the controls) or a
  gamepad (left stick drives, right stick or LEFT/RIGHT button (X/B) turns, HOME stops, VIEW shows the controls)
* speed sliders of noph-gamepad-components: L1/R1 (F/G) lower/raise the linear speed, L2/R2 (R/T) the angular
  speed
* the settings button in the top bar opens the settings, tabs switched with L1/R1:
  * Controls: gamepad, dead zone, stick axes (with detection), keys and gamepad buttons per action (captured by
    pressing them); conflicts block saving
  * Theme: color scheme (system, light, dark) and a palette generated from a source color
    (`@materialx/material-color-utilities`, the roles of noph-ui's default theme), previewed while editing
  Both are remembered by the browser. While the settings are open, gamepad and keyboard input goes to the
  `settings` input context, the robot controls ignore it and the drive card shows that driving is paused. The
  gamepad navigates the settings (`settings-navigation.ts`): d-pad or left stick up/down move the focus,
  left/right change sliders and selects, A selects, B closes, Menu saves
* joint sliders, simple actions (named values, dependencies), complex actions
* image views above and the log below the 3D view, resizable, hideable, remembered by the browser
* sensors, IMU, wrenches, robot state, permission requests, telemetry inspector, file downloads, heartbeats

UI components: [noph-ui](https://noph.dev) (Material 3). noph-gamepad-components is not used: its components
add global `.hint` / `.hint-container` styles that break other layouts, and they read their input mapping only
once. `components/input/GamepadActionButton.svelte` is a noph-ui button with the same gamepad/keyboard
behaviour that always shows its key and button.

## AI usage
This web interface was generated with the help of Claude Opus 5.5

## Usage
To write the self-contained `build/index.html` run
```bash
npm run build
```
so it works from any folder of the robot's http server.

You can then run it with a custom robot simulation (that implements a Robot Remote Control Robot) like this:
```bash
./rrc_test_robots --web --html <path_to_robot_remote_control>/src/WebSvelte/build/
```

## Development
```bash
npm i -D
RRC_ROBOT=127.0.0.1 npm run dev
```

The development server proxies `/api` and `/model` to `http://$RRC_ROBOT:$RRC_COMMAND_PORT` (default 7001)
and `/telemetry` to `ws://$RRC_ROBOT:$RRC_TELEMETRY_PORT` (default 7002), the robot does not allow cross
origin requests. Test robots: `rrc_test_robots`. `npm run check` type checks.

## Deployment

```c++
commands->serveFolder("<path>/WebSvelte/build", "html");
commands->setDefaultPage("/html/index.html?telemetryport=7002");
```

| url parameter   | default          |                                                    |
| --------------- | ---------------- | -------------------------------------------------- |
| `telemetryport` | 7002             | port of the telemetry websocket                    |
| `commandport`   | port of the page | port of the http commands (no cross origin support) |

Invalid parameters stop with an error page instead of connecting elsewhere.

## Structure

* `src/lib/rrc/`: framework independent protocol code: `client.ts` (commands, telemetry websocket,
  heartbeats, `LatestValueSender`), `protocol.ts` (message types), `model-loader.ts` (model hosted by the
  server at `/model/<robot>/model.json` or transferred with `FILE_REQUEST`, parsed with urdf-viewer's
  `UrdfParser`), `kinematics.ts` (forward kinematics for data in link frames, joint values for the viewer)
* `src/lib/session/`: reactive state of one robot (`RobotSession`: interface description, model, commands;
  `TelemetryState`: frequent telemetry applied once per animation frame; maps; image sources)
* `drive.svelte.ts`, `layout.svelte.ts`, `viewer/`: driving, remembered layout, 3D view state and helpers
* `theme.svelte.ts`: noph-ui theme, applied as inline `--np-color-*` variables and `data-theme` on `<html>`
* `input-config.svelte.ts`: keyboard/gamepad bindings and their svelte-gamepad-virtual-joystick mappings.
  The library's components keep the mapping object they were created with, so the mapping objects are updated
  in place; registering components again on changes loses registrations in the library
* `src/lib/components/`: `RobotConsole.svelte` creates the state and provides it as contexts;
  `viewer/` (threlte scene), `layout/` (top bar, regions, images, log, dialog, toasts), `sidebar/` (cards)

## urdf-viewer notes

The robot model is shown with urdf-viewer's `UrdfThree` through its global `urdf_viewer_state`. Its parser
rounds joint limits to integers and leaves missing visual origins undefined, so limits are read from the xml
in `kinematics.ts` and origins are filled in by the model loader. It only rotates joints: prismatic joints are
shown by moving their origin. Mesh paths are mapped to the model files before rendering, visuals with missing
files are dropped (a message lists them). Joint and link names ("Frames") use troika text, which loads its
default font from the internet.
