#include <boost/test/unit_test.hpp>

#include "../src/Tools/SendTimers.hpp"
#include "../src/MessageTypes.hpp"

using namespace robot_remote_control;




BOOST_AUTO_TEST_CASE(send_timers)
{
    SendTimers telemetrytimers(robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER);
    SendTimers commandtimers(robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER, false);

    // they should return defaults
    BOOST_CHECK_EQUAL(telemetrytimers.isSendRequired(CURRENT_POSE), true);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);

    // test enable/disable settings
    telemetrytimers.disable(CURRENT_POSE);
    BOOST_CHECK_EQUAL(telemetrytimers.isSendRequired(CURRENT_POSE), false);
    commandtimers.enable(TARGET_POSE_COMMAND);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), true);

    // set to 1 ms
    commandtimers.setSendSpeed(TARGET_POSE_COMMAND, 10);
    // "send once"
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), true);
    // no send after first
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);
    // wait 10 ms
    usleep(10000);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), true);

    commandtimers.disable(TARGET_POSE_COMMAND);
    // is false also before tiem is over
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);
    // wait 10 ms
    usleep(10000);
    // still false
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);

    commandtimers.enable(TARGET_POSE_COMMAND);
    // directly true after enable
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), true);


    // enabled and speed < 0
    commandtimers.setSendSpeed(TARGET_POSE_COMMAND, -1);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), true);
    // speed 0 == false
    commandtimers.setSendSpeed(TARGET_POSE_COMMAND, 0);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);
    
    // disabled independent to speed setting (always off)
    commandtimers.disable(TARGET_POSE_COMMAND);
    commandtimers.setSendSpeed(TARGET_POSE_COMMAND, -1);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);
    // speed 0 == false
    commandtimers.setSendSpeed(TARGET_POSE_COMMAND, 0);
    BOOST_CHECK_EQUAL(commandtimers.isSendRequired(TARGET_POSE_COMMAND), false);


}

