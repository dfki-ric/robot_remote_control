# rrc tools

These libs are not used in the rrc library itself, but may be useful to integrate the library into your application.

## SendTimers

Data rates and bandwidth are often a limiting factor for remote control, e.g. the controller does not need the same data rated on the joint state than the robot itself. Also, some robots need a command in a specific rate, so the application might have to repeat the command faster than it is send via rrc (be careful when doing this, you'll still need to stop repeating on connection losses).



