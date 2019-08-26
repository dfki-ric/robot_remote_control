# interaction-library-controlled_robot


This is a library for Framework independent control of semi-autonomous robots.


## Motivation

Robots or semi-autonomous vehicles often use their own Framework, with it's own proprietary communication.
While these are often well suited for in-system communication, they often cause trouble when those systems sould be controlled by an external connection.

Those Connections may also have additional requirements, like low bandwidth, high latency, etc., or custom communication hardware, which does not have an ethernet stack.
This library defines an interface to those robots and supports externally programmed Transports that can handle the requirements above.


## License

BSD Clause 3
Copyright DFKI GmbH


## Installation

The easiest way to build and install this package is to use Rock's build system.
See [this page](http://rock-robotics.org/stable/documentation/installation.html)
on how to install Rock.

You can also build the Library without rock, by using the rock CMake marcos.

As a reference you can also have a look into the gitlab-ci.yml file.

## Rock CMake Macros

The ./install_dependencies.sh script will install the rock cmake macros, if you provide a path to that script, the macros will be installed there.
In this case an env.sh script is generated, which sets up the environment to find the macros, please source it before using cmake to compile this library.

To compile it manually:

    ./install_dependencies.sh ./
    source env.sh
    mkdir build
    cd build
    cmake ..
    make

Documentations on the rock cmake macros is available on [this page](http://rock-robotics.org/stable/documentation/packages/cmake_macros.html).


### Rock Standard Layout

This directory structure follows some simple rules, to allow for generic build
processes and simplify reuse of this project. Following these rules ensures that
the Rock CMake macros automatically handle the project's build process and
install setup properly.

### Folder Structure

| directory         |       purpose                                                        |
| ----------------- | ------------------------------------------------------               |
| src/              | Contains all header (*.h/*.hpp) and source files                     |
| build/ *          | The target directory for the build process, temporary content        |
| bindings/         | Language bindings for this package, e.g. put into subfolders such as |
| ruby/             | Ruby language bindings                                               |
| viz/              | Source files for a vizkit plugin / widget related to this library    |
| resources/        | General resources such as images that are needed by the program      |
| configuration/    | Configuration files for running the program                          |
| external/         | When including software that needs a non standard installation process, or one that can be easily embedded include the external software directly here |
| doc/              | should contain the existing doxygen file: doxygen.conf               |




## Gettings started

have a look into the provided Main examples

* src/ControlledRobotMain.cpp (Robot that takes the position command and sets its position to it)
* src/RobotControllerMain.cpp (controller that sets the desired robot position)

You can build the Doxygen documentation using `$> doxygen Doxyfile`, it will be written to the build/doc folder.


There are two main libraries generated when compiling this repository:

### RobotController

The RobotController class is a framework independent cpp class which is able to control any Robot/Vehicle that has a ControlledRobot library wrapped into its control framework.

### ControlledRobot

This library is to be used on the robot, you can map the commands received from the RobotController Library to commands of your Robot to listen to the commands.
Also you can also add Telemery to this library, which are then send to the RobotController.


### Transports

Transports are seperated into two directions: commands and telemetry.
While commands from RobotController to ControlledRobot are getting acknowledged on retrieval, the retrieval of telemetry is not confirmed.

These Transports are given to the RobotController and ControlledRobot in their contructor and only implement an send() and receive() function.
So it is easy to implement other means of connections between those.

