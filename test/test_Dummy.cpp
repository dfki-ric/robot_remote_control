#include <boost/test/unit_test.hpp>
#include <interaction-library-controlled_robot/Dummy.hpp>

using namespace interaction-library-controlled_robot;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    interaction-library-controlled_robot::DummyClass dummy;
    dummy.welcome();
}
