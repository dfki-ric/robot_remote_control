#pragma once

#include "SimpleActionHelper.hpp"
#include "SendTimers.hpp"


namespace robot_remote_control {

class SendTimerSimpleActions {
 public:
    /**
     * @brief Construct a new Send Timer Simple Actions object
     * 
     * @param timers 
     * @param actionhelper needs to be a referecne to be able to use the same instatnce in other code
     */
    SendTimerSimpleActions(SendTimers& timers, SimpleActionsHelper& actionhelper) : timers(timers), actionhelper(actionhelper) {}

    void addTelemetrySimpleAction(const robot_remote_control::TelemetryMessageType& type);

    void generateTelemetrySimpleActions();

    /**
     * @brief 
     * 
     * 
     * @param simpleaction 
     * @return true the action was a time setting, value was updated, if you provide the state, you should run robot->initSimpleActions(simpelactionhelper->getSimpleActionsDefinition())
     * @return false 
     */
    bool evaluateSimpleAction(const SimpleAction& simpleaction);
 
 private:
    
    SendTimers& timers;
    SimpleActionsHelper& actionhelper;
    std::map<std::string, robot_remote_control::TelemetryMessageType> simpleActionNameToIndex;
};

}  // namespace robot_remote_control
