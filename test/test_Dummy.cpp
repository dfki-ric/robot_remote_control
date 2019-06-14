#include <boost/test/unit_test.hpp>

#include "../src/ControlledRobot.hpp"
#include "../src/RobotController.hpp"
#include "../src/Transports/TransportZmq.hpp"


using namespace interaction;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{

   
   interaction::TransportSharedPtr commands = interaction::TransportSharedPtr(new interaction::TransportZmq("tcp://127.0.0.1:7001",interaction::TransportZmq::REQ));
   interaction::TransportSharedPtr telemetry = interaction::TransportSharedPtr(new interaction::TransportZmq("tcp://127.0.0.1:7002",interaction::TransportZmq::SUB));
   interaction::RobotController test();
   interaction::RobotController dummy(commands, telemetry);
   interaction::Pose pose;
   
 
   
   BOOST_TEST( true /* test assertion */ );


}

