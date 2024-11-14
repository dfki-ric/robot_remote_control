#pragma once

#include <string>
#include <map>
#include <memory>
#include "../Types/RobotRemoteControl.pb.h"


class SimpleActionWrapper;


/**
 * @brief helper definitions to handle Simpleactions and dependencies more easily
 * Usage: create this class, use the function of this classes to configure simpleactions, and finally use 
 * 
 * SimpleActionsHelper simpleactions;
 * simpleactions.addAction() 
 * [...]
 * controlledrobot.initSimpleActions(simpleactions.getSimpleActionsDefinition());
 * 
 * see tests/test_SimpleActionHelper.cpp for more examples
 */
class SimpleActionsHelper {
 public:

    SimpleActionsHelper(){}
    SimpleActionsHelper(const robot_remote_control::SimpleActions& initfrom);
    virtual ~SimpleActionsHelper(){}

    /**
     * @brief add a action to 
     * 
     * @param actionname 
     * @param type 
     * @return std::shared_ptr<SimpleActionWrapper> 
     */
    std::shared_ptr<SimpleActionWrapper> addSimpleAction(const std::string &actionname, const robot_remote_control::SimpleActionType& type);

    /**
     * @brief Get the Simple Actions Definition object to be passed to ControlledRobot::initSimpleActions()
     * 
     * @return const robot_remote_control::SimpleActions& 
     */
    const robot_remote_control::SimpleActions& getSimpleActionsDefinition();

    /**
     * @brief Get the Simple Action object in case the return value of addSimpleAction was not stored
     * 
     * @param actionname 
     * @return std::shared_ptr<SimpleActionWrapper> 
     */
    std::shared_ptr<SimpleActionWrapper> getSimpleAction(const std::string &actionname);

    /**
     * @brief Set the current state in the action decription, needed to use canExecute()
     * can also be set on the SimpleActionWrapper directly
     * 
     * @param actionname 
     * @param value 
     */
    void setActionState(const std::string &actionname, const float &value);

    /**
     * @brief Get the state of a action
     * can also be set on the SimpleActionWrapper directly
     * 
     * @param actionname 
     * @return float 
     */
    float getActionState(const std::string &actionname);

    /**
     * @brief check if the dependencies of the simpleaction are met
     * 
     * @warning You'll need to keep track of the current states using setState()
     * @param action the action to check if dependencies are met
     * @return true if all dependencies are ok
     * @return false if on of the dependencies is not met
     */
    bool canExecute(const std::shared_ptr<SimpleActionWrapper> &action);

    /**
     * @brief wrapper for canExecute in case you have not saved the action object std::shared_ptr<SimpleActionWrapper>
     */
    bool canExecute(const std::string &actionname) {
        return canExecute(actionbyname[actionname]);
    }

 private:    
    void initFrom(const robot_remote_control::SimpleActions& initfrom);
    robot_remote_control::SimpleActions actions;
    std::map<std::string, std::shared_ptr<SimpleActionWrapper>> actionbyname;

};

/**
 * @brief wrapper around a single action
 * 
 */
class SimpleActionWrapper {
 public:

    SimpleActionWrapper(robot_remote_control::SimpleAction* action);
    virtual ~SimpleActionWrapper(){}

    void setLimits(const float &min_state, const float &max_state, const float &step_size);

    void addNamedValues(const std::map<std::string, float> & valuenames);

    void addNamedValue(const std::string &name, const float &value);

    float getNamedValue(const std::string &valuename);

    std::string getValueName(const float &valuename);

    void addActionDependency(const std::string &name, const float &value);

    void setState(const float &value) {
        action->set_state(value);
    }
    void setState(const std::string &valuename) {
        setState(getNamedValue(valuename));
    }

    float getState() {
        return action->state();
    }

    robot_remote_control::SimpleActionType getType() {
        return action->type().type();
    }

    
    const robot_remote_control::SimpleAction& getAction() {
        return *action;
    }

 private:
    robot_remote_control::SimpleAction* action;

    friend class SimpleActionsHelper;
    std::map<std::string, float> valueByName;
    std::map<float, std::string> nameByValue;

    std::map<std::string, std::vector<float> > actionDependencyStates;

};

