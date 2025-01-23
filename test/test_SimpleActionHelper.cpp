#include <boost/test/unit_test.hpp>

#include "../src/Tools/SimpleActionHelper.hpp"
#include "../src/MessageTypes.hpp"
#include "../src/Types/RobotRemoteControl.pb.h"

using namespace robot_remote_control;


BOOST_AUTO_TEST_CASE(simple_action_helper) {
   
SimpleActionsHelper helper;

std::shared_ptr<SimpleActionWrapper> action1 = helper.addSimpleAction("action1", robot_remote_control::VALUE_INT);
std::shared_ptr<SimpleActionWrapper> action2 = helper.addSimpleAction("action2", robot_remote_control::VALUE_FLOAT);
std::shared_ptr<SimpleActionWrapper> action3 = helper.addSimpleAction("action3", robot_remote_control::TRIGGER);


BOOST_CHECK_EQUAL(action1->getType(), robot_remote_control::VALUE_INT);
BOOST_CHECK_EQUAL(action2->getType(), robot_remote_control::VALUE_FLOAT);
BOOST_CHECK_EQUAL(action3->getType(), robot_remote_control::TRIGGER);

action1->setLimits(-1,3,1);
BOOST_CHECK_EQUAL(action1->getAction().type().min_state(), -1);
BOOST_CHECK_EQUAL(action1->getAction().type().max_state(), 3);
BOOST_CHECK_EQUAL(action1->getAction().type().step_size(), 1);

action1->addNamedValues({ {"VAL1", 1}, {"VAL2", 2},{"VAL3", 3} });
BOOST_CHECK_EQUAL(action1->getAction().type().value_names(0).value(), 1);
BOOST_CHECK_EQUAL(action1->getAction().type().value_names(1).value(), 2);
BOOST_CHECK_EQUAL(action1->getAction().type().value_names(2).value(), 3);

BOOST_CHECK_EQUAL(action1->getNamedValue("VAL1"), 1);
BOOST_CHECK_EQUAL(action1->getNamedValue("VAL2"), 2);
BOOST_CHECK_EQUAL(action1->getNamedValue("VAL3"), 3);

BOOST_CHECK_EQUAL(action1->getValueName(1), "VAL1");
BOOST_CHECK_EQUAL(action1->getValueName(2), "VAL2");
BOOST_CHECK_EQUAL(action1->getValueName(3), "VAL3");


action1->setState("VAL2");
BOOST_CHECK_EQUAL(action1->getState(), 2);


// helper.getSimpleActionsDefinition().PrintDebugString();

action1->addActionDependency("action2", 3);
action1->addActionDependency("action2", 4);// or relation

BOOST_CHECK_EQUAL(helper.canExecute(action1), false);
action2->setState(3);
BOOST_CHECK_EQUAL(helper.canExecute(action1), true);
action2->setState(4);
BOOST_CHECK_EQUAL(helper.canExecute(action1), true);
action2->setState(5);
BOOST_CHECK_EQUAL(helper.canExecute(action1), false);

// add a and dependency on another action
action1->addActionDependency("action3", 1);
BOOST_CHECK_EQUAL(helper.canExecute(action1), false);
action2->setState(3);
action3->setState(1);
BOOST_CHECK_EQUAL(helper.canExecute(action1), true);
action3->setState(0);
BOOST_CHECK_EQUAL(helper.canExecute(action1), false);

// actions without deps can always be executed
BOOST_CHECK_EQUAL(helper.canExecute(action2), true);



// check copy/restore
SimpleActionsHelper helper2(helper.getSimpleActionsDefinition());

BOOST_TEST(helper.getSimpleActionsDefinition().SerializeAsString() == helper2.getSimpleActionsDefinition().SerializeAsString());
BOOST_TEST(helper.getSimpleAction("action1")->getAction().SerializeAsString() == helper2.getSimpleAction("action1")->getAction().SerializeAsString());
BOOST_CHECK_EQUAL(helper.getSimpleAction("action1")->getNamedValue("VAL2"), helper2.getSimpleAction("action1")->getNamedValue("VAL2"));

BOOST_CHECK_EQUAL(helper2.getSimpleAction("action1")->getType(), robot_remote_control::VALUE_INT);
BOOST_CHECK_EQUAL(helper2.getSimpleAction("action2")->getType(), robot_remote_control::VALUE_FLOAT);
BOOST_CHECK_EQUAL(helper2.getSimpleAction("action3")->getType(), robot_remote_control::TRIGGER);

BOOST_CHECK_EQUAL(helper2.canExecute(action1), false);
helper2.setActionState("action3", 1);
BOOST_CHECK_EQUAL(helper2.canExecute(action1), true);

// now we set a state on the copy, they are different
BOOST_TEST(helper.getSimpleActionsDefinition().SerializeAsString() != helper2.getSimpleActionsDefinition().SerializeAsString());



}


BOOST_AUTO_TEST_CASE(simple_action_helper_update) {
    SimpleActionsHelper helper;
    helper.addSimpleAction("action", robot_remote_control::VALUE_INT);

    //simulate a freshly received action update
    SimpleActionsHelper remote_helper;
    remote_helper.addSimpleAction("action", robot_remote_control::VALUE_INT);

    // get an action
    std::shared_ptr<SimpleActionWrapper> renewedaction = helper.getSimpleAction("action");

    const robot_remote_control::SimpleAction* oldptr = &renewedaction->getAction();

    // renew helper2 info

    helper.update(remote_helper.getSimpleActionsDefinition());

    const robot_remote_control::SimpleAction* newptr = &renewedaction->getAction();

    BOOST_CHECK(&renewedaction->getAction() != nullptr);
}