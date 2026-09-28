# cmake_minimum_required(VERSION 3.7)

# project(plotjuggler_rock_plugin)

find_package(plotjuggler)

if (plotjuggler_FOUND)

    # set(CMAKE_CXX_STANDARD 17)
    # set(CMAKE_CXX_STANDARD_REQUIRED ON)

    #-------  Include Qt dependencies -------
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOMOC ON)
    SET(CMAKE_AUTOUIC ON)

    find_package(Qt5 REQUIRED COMPONENTS
        Core
        Widgets
        Concurrent
        Xml
        Svg
        OpenGL)


    include_directories(
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Xml_INCLUDE_DIRS}
        ${Qt5Svg_INCLUDE_DIRS}
        ${OROCOS_CPP_INCLUDE_DIRS} )

    set(QT_LIBRARIES
        Qt5::Core
        Qt5::Widgets
        Qt5::Xml
        Qt5::Svg )

    add_definitions( ${QT_DEFINITIONS} -DQT_PLUGIN)
    set( PJ_LIBRARIES ${QT_LIBRARIES} plotjuggler_base)

    #--------------------------------------------------------
    #-------------- Build with CATKIN (ROS1) ----------------
    if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)

        set(COMPILING_WITH_CATKIN 1)
        message(STATUS "COMPILING_WITH_CATKIN")
        add_definitions(-DCOMPILED_WITH_CATKIN)

        find_package(catkin REQUIRED COMPONENTS plotjuggler )
        catkin_package(
            CATKIN_DEPENDS plotjuggler
            )
        include_directories(${catkin_INCLUDE_DIRS})
        list(APPEND ${PJ_LIBRARIES} ${catkin_LIBRARIES} ${OROCOS_CPP_LIBRARIES})
        set(PJ_PLUGIN_INSTALL_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION} )

        #--------------------------------------------------------
        #-------------- Build with AMENT (ROS2) -----------------
    elseif( DEFINED ENV{AMENT_PREFIX_PATH})

        set(COMPILING_WITH_AMENT 1)
        message(STATUS "COMPILING_WITH_CATKIN")
        add_definitions(-DCOMPILED_WITH_AMENT)

        find_package(ament_cmake REQUIRED)
        set(PJ_PLUGIN_INSTALL_DIRECTORY lib/${PROJECT_NAME} )

        #--------------------------------------------------------
        #------------- Build without any ROS support ------------
    else()
        message(STATUS "PlotJuggler FOUND")
        message(STATUS "plotjuggler_INCLUDE_DIR: ${plotjuggler_INCLUDE_DIR}")
        message(STATUS "plotjuggler_LIBRARIES: ${plotjuggler_LIBRARIES}")

        include_directories(${plotjuggler_INCLUDE_DIR})
        list(APPEND ${PJ_LIBRARIES} ${plotjuggler_LIBRARIES})
        set(PJ_PLUGIN_INSTALL_DIRECTORY bin )

    endif()

    #------- Create the libraries -------
    add_library(DataStreamRRC SHARED
        DataStreamRRC.hpp
        DataStreamRRC.cpp)


    # target_include_directories(DataStreamRRC
    #         PUBLIC
    #                 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    #                 $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>/src/
    #                 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    # )


    target_link_libraries(DataStreamRRC
        robot_remote_control-robot_controller
        robot_remote_control-transport_zmq
        ${PJ_LIBRARIES})


    # add_library(PlotJugglerDataSample SHARED
    #    DataLoadSampleCSV/dataload_simple_csv.h
    #    DataLoadSampleCSV/dataload_simple_csv.cpp )

    # target_link_libraries(PlotJugglerDataSample
    #     ${PJ_LIBRARIES})

    if (COMPILING_WITH_AMENT)
        ament_target_dependencies(DataStreamRRC plotjuggler)
        # ament_target_dependencies(PlotJugglerDataSample plotjuggler)
    endif()
    #------- Install the libraries -------

    install(
        TARGETS
        DataStreamRRC
            # PlotJugglerDataSample
        DESTINATION
            ${PJ_PLUGIN_INSTALL_DIRECTORY}  )

endif (plotjuggler_FOUND)
