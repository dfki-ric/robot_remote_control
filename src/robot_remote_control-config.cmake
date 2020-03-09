

find_package (Threads)
find_package(Protobuf REQUIRED)

find_package(PkgConfig)
pkg_check_modules(ZMQ REQUIRED libzmq)


include("${CMAKE_CURRENT_LIST_DIR}/robot_remote_control-targets.cmake")

