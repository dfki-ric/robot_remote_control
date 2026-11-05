#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include <chrono>

#include "../src/TransportProxy/GenericProxy.hpp"
#include "../src/ControlledRobot/ControlledRobot.hpp"
#include "../src/RobotController/RobotController.hpp"
#include "../src/Transports/ZeroMQ/TransportZmq.hpp"
#include "../src/Transports/Http/TransportHttp.hpp"
#include "../src/Transports/WebSocket/TransportWebSocket.hpp"
#include "../src/TypeGenerator.hpp"

using namespace robot_remote_control;

#define COMPARE_PROTOBUF(VAR1, VAR2) BOOST_TEST(VAR1.SerializeAsString() == VAR2.SerializeAsString())

/**
 * the proxy tests use their own ports, the robot uses zmq
 */
void checkProxiedConnection(ControlledRobot *robot, RobotController *controller) {
    robot->startUpdateThread(10);
    controller->startUpdateThread(10);

    robot->initRobotName("proxied robot");
    RobotName name;
    BOOST_REQUIRE(controller->requestRobotName(&name));
    BOOST_CHECK_EQUAL(name.value(), "proxied robot");

    // plain text replies
    BOOST_CHECK_EQUAL(controller->requestLibraryVersion(), controller->libraryVersion());

    // empty replies (telemetry that was never set) do not wait for timeouts
    const auto start = std::chrono::steady_clock::now();
    VideoStreams streams;
    BOOST_CHECK_EQUAL(controller->requestVideoStreams(&streams), false);
    BOOST_CHECK(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500));
    BOOST_CHECK(controller->isConnected());

    // commands
    Twist twist = TypeGenerator::genTwist();
    controller->setTwistCommand(twist);
    Twist receivedTwist;
    for (int i = 0; i < 100 && !robot->getTwistCommand(&receivedTwist); ++i) {
        usleep(10000);
    }
    COMPARE_PROTOBUF(receivedTwist, twist);

    // telemetry, repeated because subscribers miss messages until they are connected
    Pose pose = TypeGenerator::genPose();
    Pose receivedPose;
    bool poseReceived = false;
    for (int i = 0; i < 200 && !poseReceived; ++i) {
        robot->setCurrentPose(pose);
        usleep(10000);
        poseReceived = controller->getCurrentPose(&receivedPose);
    }
    BOOST_REQUIRE(poseReceived);
    COMPARE_PROTOBUF(receivedPose, pose);

    // maps contain a protobuf Any
    PointCloud cloud;
    for (int i = 0; i < 100; ++i) {
        *cloud.add_points() = TypeGenerator::genPosition();
    }
    robot->setMap(cloud);
    Map map;
    BOOST_REQUIRE(controller->requestMap(&map));
    PointCloud receivedCloud;
    BOOST_REQUIRE(map.map().UnpackTo(&receivedCloud));
    COMPARE_PROTOBUF(receivedCloud, cloud);

    // file transfer
    FileDefinition files;
    File *file = files.add_file();
    files.add_isfolder(false);
    file->set_identifier("file");
    file->set_path("./test/testfiles/topfolderfile");
    robot->initFiles(files);
    BOOST_REQUIRE(controller->requestFile("file", false, "./proxy_download"));
    BOOST_CHECK(boost::filesystem::exists("./proxy_download/test/testfiles/topfolderfile"));
    boost::filesystem::remove_all("./proxy_download");
}

BOOST_AUTO_TEST_CASE(proxy_zmq_to_zmq) {
    // same serialization, the messages are forwarded unchanged
    ControlledRobot robot(std::make_shared<TransportZmq>("tcp://*:7211", TransportZmq::REP),
                          std::make_shared<TransportZmq>("tcp://*:7212", TransportZmq::PUB));
    GenericProxy commands(GenericProxy::COMMANDS,
                          std::make_shared<TransportZmq>("tcp://127.0.0.1:7211", TransportZmq::REQ),
                          std::make_shared<TransportZmq>("tcp://*:7213", TransportZmq::REP));
    GenericProxy telemetry(GenericProxy::TELEMETRY,
                           std::make_shared<TransportZmq>("tcp://127.0.0.1:7212", TransportZmq::SUB),
                           std::make_shared<TransportZmq>("tcp://*:7214", TransportZmq::PUB));
    commands.startUpdateThread(1);
    telemetry.startUpdateThread(1);
    RobotController controller(std::make_shared<TransportZmq>("tcp://127.0.0.1:7213", TransportZmq::REQ),
                               std::make_shared<TransportZmq>("tcp://127.0.0.1:7214", TransportZmq::SUB));

    checkProxiedConnection(&robot, &controller);
    BOOST_CHECK_EQUAL(commands.getErrorCount(), 0);
    BOOST_CHECK_EQUAL(telemetry.getErrorCount(), 0);
}

BOOST_AUTO_TEST_CASE(proxy_zmq_to_web) {
    // binary zmq to json http/websocket, the messages are converted
    ControlledRobot robot(std::make_shared<TransportZmq>("tcp://*:7221", TransportZmq::REP),
                          std::make_shared<TransportZmq>("tcp://*:7222", TransportZmq::PUB));
    GenericProxy commands(GenericProxy::COMMANDS,
                          std::make_shared<TransportZmq>("tcp://127.0.0.1:7221", TransportZmq::REQ),
                          std::make_shared<TransportHttp>("http://0.0.0.0:7223"));
    GenericProxy telemetry(GenericProxy::TELEMETRY,
                           std::make_shared<TransportZmq>("tcp://127.0.0.1:7222", TransportZmq::SUB),
                           std::make_shared<TransportWebSocket>(TransportWebSocket::SERVER_TEXT, 7224));
    commands.startUpdateThread(1);
    telemetry.startUpdateThread(1);
    // the servers start listening in their threads
    usleep(200000);
    RobotController controller(std::make_shared<TransportHttp>("http://127.0.0.1:7223", TransportHttp::CLIENT),
                               std::make_shared<TransportWebSocket>(TransportWebSocket::CLIENT_TEXT, 7224, "127.0.0.1"));

    checkProxiedConnection(&robot, &controller);
    BOOST_CHECK_EQUAL(commands.getErrorCount(), 0);
    BOOST_CHECK_EQUAL(telemetry.getErrorCount(), 0);
}

BOOST_AUTO_TEST_CASE(proxy_checks_transports) {
    auto pub = std::make_shared<TransportZmq>("tcp://*:7231", TransportZmq::PUB);
    auto rep = std::make_shared<TransportZmq>("tcp://*:7232", TransportZmq::REP);
    // the robot side of a command proxy must send commands, the controller side receive them
    BOOST_CHECK_THROW(GenericProxy(GenericProxy::COMMANDS, rep, pub), std::runtime_error);
    BOOST_CHECK_THROW(GenericProxy(GenericProxy::TELEMETRY, rep, pub), std::runtime_error);
}
