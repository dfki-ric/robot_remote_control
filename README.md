[![CircleCI](https://circleci.com/gh/dfki-ric/robot_remote_control.svg?style=svg)](https://circleci.com/gh/dfki-ric/robot_remote_control) CircleCI build and test

# Robot Remote Control

This is a library for framework independent remote control of semi-autonomous robots.

This library was initiated and is currently developed at the
[Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the
[German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.



## Motivation

Robots or semi-autonomous vehicles often use their own Framework, with it's own proprietary communication.
While these are often well suited for in-system communication, they often cause trouble when those systems should be controlled by an external connection.

Those connections may also have additional requirements, like low bandwidth, high latency, etc., or custom communication hardware, which does not have an ethernet stack.
This library defines an interface to those robots and supports externally programmed Transports that can handle the requirements above.


## License

[BSD Clause 3](https://opensource.org/licenses/BSD-3-Clause)<br> - Copyright DFKI GmbH

## Installation

The easiest way to build and install this package is to use Rock's build system.
See [this page](http://rock-robotics.org/documentation/installation.html)
on how to install Rock.

You can also build the Library without rock, by using the rock CMake marcos.

As a reference you can also have a look into the gitlab-ci.yml file.

### Dependencies

Additionally you need to manually install the following OS dependencies: protobuf and zeromq

    sudo apt install libprotobuf-dev protobuf-compiler libzmqpp-dev

### Building

The ./install_source_dependencies.sh script will install the rock cmake macros, if you provide a path to that script, the macros will be installed there.
In this case an env.sh script is generated, which sets up the environment to find the macros, please source it before using cmake to compile this library.

To compile it manually:

    ./install_source_dependencies.sh ./
    source env.sh
    mkdir build
    cd build
    cmake ..
    make

Documentations on the rock cmake macros is available on [this page](http://rock-robotics.org/documentation/packages/cmake_macros.html).

#### Builddin on systems without protobuf3

You need to install protobuf3 from source, to do so, install its build dependencies:

    apt-get install autoconf automake libtool curl make g++ unzip

And edit the install_source_dependencies.sh script to uncomment the protobuf install line

    build_protobuf "$ABS_PREFIX/install"

This will install protobuf 3 in the selected install folder (parameter of the install_source_dependencies.sh script)







### Rock Standard Layout

This directory structure follows some simple rules, to allow for generic build
processes and simplify reuse of this project. Following these rules ensures that
the Rock CMake macros automatically handle the project's build process and
install setup properly.

### Folder Structure

| directory         |       purpose                                                        |
| ----------------- | ------------------------------------------------------               |
| src/              | Contains all header (*.hpp) and source (*.cpp) files                     |
| build/ *          | The target directory for the build process, temporary content        |
| test/             | contains the boost_test based unit tests                       |


## Gettings started

Please have a look into the provided Main examples

* src/ControlledRobotMain.cpp (Robot that takes the position command and sets its position to it)
* src/RobotControllerMain.cpp (controller that sets the desired robot position)

You can build the Doxygen documentation using `$> doxygen Doxyfile`, it will be written to the build/doc folder.


There are two main libraries generated when compiling this repository:

### RobotController

The RobotController class is a framework independent cpp class which is able to control any Robot/Vehicle that has a ControlledRobot library wrapped into its control framework.

### ControlledRobot

This library is to be used on the robot, you can map the commands received from the RobotController Library to commands of your Robot to listen to the commands.
Also you can add Telemery to this library, which are then send to the RobotController.


### Transports

Transports are seperated into two directions: commands and telemetry.
While commands from RobotController to ControlledRobot are getting acknowledged on retrieval, the retrieval of telemetry is not confirmed.

These Transports are given to the RobotController and ControlledRobot in their contructor and only implement an send() and receive() function.
So it is easy to implement other means of connections between those by implementing the Transport.hpp interface.


## Testing

The repository includes a test directory with several boost test cases. You can build this directory and the test_suite executable as follows:

    mkdir build
    cd build
    cmake -D ROCK_TEST_ENABLED=ON ..
    make

If you always want to build the test directory as well add the following lines to your CMakeLists.txt

    ENABLE_TESTING()
    ADD_SUBDIRECTORY(test)

In order to run the tests you can either run all by executing (in /build/test) ```./test_suite``` or choose a specific test with the -t flag ```./test_suite -t checking_current_pose```

## Bug Reports

To search for bugs or report them, please use GitHubs [Issue-Tracker](https://github.com/dfki-ric/robot_remote_control/issues)
