rock_library(interaction-library-controlled_robot
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(interaction-library-controlled_robot_bin Main.cpp
    DEPS interaction-library-controlled_robot)

