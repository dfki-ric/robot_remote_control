# Command Line Interfaces

These interafces are "work in progress", if you are missing a feature please request it in an issue on github.

## robot_controller

The robot_controller CLI establishes a connection to the robot and provides basic requests and controls for the robot.
It can also be used to access the statistics module to check which telemetry at which rates are actually send by the robot.

```bash
    $> robot_controller [ROBOT_IP] [COMMAND_PORT TELEMETRY_PORT]
```

It supports tab completion to show available commands.

The "stats" command is only working in case the library was build with this feature enabled.


## console_joystick

Work in Progress
