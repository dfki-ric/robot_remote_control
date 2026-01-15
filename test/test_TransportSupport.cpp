#include <boost/test/unit_test.hpp>
#include <unistd.h>

#include <iostream>
#include "Transports.hpp"

#include "../src/RobotController/RobotController.hpp"
#include "../src/ControlledRobot/ControlledRobot.hpp"



using namespace robot_remote_control;

class TransportDummy : public Transport {
 public:
    TransportDummy(){};

    virtual int send(const std::string& buf, Flags flags = NONE){return false;};
    virtual int receive(std::string* buf, Flags flags = NONE){return false;};

    void setSupport(uint8_t s) {
        this->setTransportSupport(s);
    }

};



BOOST_AUTO_TEST_CASE(transport_implemented_correctly) {
    const Transports& t = Transports::instance();

    BOOST_CHECK_NE(t.robotControllerCommands->getTransportSupportBitmask(), Transport::UNSET);
    BOOST_CHECK_NE(t.robotControllerTelemetry->getTransportSupportBitmask(), Transport::UNSET);
    BOOST_CHECK_NE(t.controlledRobotCommands->getTransportSupportBitmask(), Transport::UNSET);
    BOOST_CHECK_NE(t.controlledRobotTelemetry->getTransportSupportBitmask(), Transport::UNSET);

    BOOST_CHECK_EQUAL(t.robotControllerCommands->supportsRobotControllerCommands(), true);
    BOOST_CHECK_EQUAL(t.robotControllerTelemetry->supportsRobotControllerTelemetry(), true);
    BOOST_CHECK_EQUAL(t.controlledRobotCommands->supportsControlledRobotCommands(), true);
    BOOST_CHECK_EQUAL(t.controlledRobotTelemetry->supportsControlledRobotTelemetry(), true);
    
    BOOST_CHECK_NO_THROW(RobotController(t.robotControllerCommands, t.robotControllerTelemetry));
    BOOST_CHECK_NO_THROW(ControlledRobot(t.controlledRobotCommands, t.controlledRobotTelemetry));
}

BOOST_AUTO_TEST_CASE(transport_compatibility_reports_correctly) {
    // check generic 
    
    TransportSharedPtr dummy = TransportSharedPtr(new TransportDummy());
    std::shared_ptr<TransportDummy> tdummy = std::dynamic_pointer_cast<TransportDummy>(dummy);
    
    tdummy->setSupport(Transport::ROBOTCOMMANDS);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotCommands(), true);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotTelemetry(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerTelemetry(), false);

    tdummy->setSupport(Transport::ROBOTTELEMETRY);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotTelemetry(), true);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerTelemetry(), false);

    tdummy->setSupport(Transport::CONTOLLERCOMMANDS);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotTelemetry(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerCommands(), true);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerTelemetry(), false);

    tdummy->setSupport(Transport::CONTROLLERTELEMETRY);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotTelemetry(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerTelemetry(), true);

    tdummy->setSupport(Transport::ROBOTCOMMANDS | Transport::ROBOTTELEMETRY);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotCommands(), true);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotTelemetry(), true);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerTelemetry(), false);

    tdummy->setSupport(Transport::CONTROLLERTELEMETRY | Transport::CONTOLLERCOMMANDS);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotCommands(), false);
    BOOST_CHECK_EQUAL(dummy->supportsControlledRobotTelemetry(), false);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerCommands(), true);
    BOOST_CHECK_EQUAL(dummy->supportsRobotControllerTelemetry(), true);
}

BOOST_AUTO_TEST_CASE(throws_on_incorrect_transport_use) {
    const Transports& t = Transports::instance();
    // check generic
    TransportSharedPtr dummyCommands = TransportSharedPtr(new TransportDummy());
    TransportSharedPtr dummyTelemetry = TransportSharedPtr(new TransportDummy());

    std::shared_ptr<TransportDummy> tdummyCommands = std::dynamic_pointer_cast<TransportDummy>(dummyCommands);
    std::shared_ptr<TransportDummy> tdummyTelemetry = std::dynamic_pointer_cast<TransportDummy>(dummyTelemetry);

    // throw on false telemetry
    tdummyCommands->setSupport(Transport::CONTOLLERCOMMANDS);
    tdummyTelemetry->setSupport(Transport::ROBOTCOMMANDS);
    BOOST_CHECK_THROW(ControlledRobot(dummyCommands, dummyTelemetry), std::runtime_error);

    tdummyCommands->setSupport(Transport::ROBOTCOMMANDS);
    tdummyTelemetry->setSupport(Transport::ROBOTCOMMANDS);
    BOOST_CHECK_THROW(RobotController(dummyCommands, dummyTelemetry), std::runtime_error);
}
