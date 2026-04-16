#include "SendTimerSimpleActions.hpp"


using namespace robot_remote_control;

void SendTimerSimpleActions::generateTelemetrySimpleActions() {
    for (int i = 1; i < timers.config.size(); ++i) {
        addTelemetrySimpleAction((robot_remote_control::TelemetryMessageType)i);
    }
}

void SendTimerSimpleActions::addTelemetrySimpleAction(const robot_remote_control::TelemetryMessageType& type) {
        std::string actioname = "set_" + TelemetryMessageType_Name(type) + "_interval";
        simpleActionNameToIndex[actioname] = type;
        auto action = actionhelper.addSimpleAction(actioname, VALUE_FLOAT, "set time between sends of " + TelemetryMessageType_Name(type) + " messages");
        action->setLimits(-1, 60, 0.01);
}

bool SendTimerSimpleActions::evaluateSimpleAction(const SimpleAction& simpleaction) {
    if (simpleActionNameToIndex.find(simpleaction.name()) != simpleActionNameToIndex.end()) {
        printf("%s:%i %s %.2f\n", __PRETTY_FUNCTION__, __LINE__,simpleaction.name().c_str(), simpleaction.state());
        size_t interval_ms = simpleaction.state()*1000;
        timers.setSendSpeed(simpleActionNameToIndex[simpleaction.name()], interval_ms);
        actionhelper.setActionState(simpleaction.name(), simpleaction.state());
        printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        return true;
    }
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    return false;
}

