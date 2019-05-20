#include <boost/test/unit_test.hpp>
#include <interaction-library-controlled_robot/ControlledRobot.hpp>

using namespace interaction;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    interaction::ControlledRobot dummy;
    dummy.welcome();
}
