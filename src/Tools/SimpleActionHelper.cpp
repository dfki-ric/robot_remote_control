#include "SimpleActionHelper.hpp"
#include <cmath>



SimpleActionsHelper::SimpleActionsHelper(const robot_remote_control::SimpleActions& initfrom) {
    update(initfrom);
}

void SimpleActionsHelper::update(const robot_remote_control::SimpleActions& initfrom) {
    actions.Clear();
    // actionbyname.clear();

    actions.CopyFrom(initfrom);

    //restore access buffers
    for (auto& action : *actions.mutable_actions()) {
        std::shared_ptr<SimpleActionWrapper> wrapper;
        if (actionbyname.find(action.name()) == actionbyname.end()) {
            //create new wrapper
            wrapper = std::make_shared<SimpleActionWrapper>(&action);
            actionbyname[action.name()] = wrapper;
        } else {
            //update action pointer
            printf("update pointer on existing wrapper\n");
            wrapper = actionbyname[action.name()];
            wrapper->updateActionPointer(&action);
        }

        // restore valueByName access buffer
        for (auto& namedValue : action.type().value_names()) {
            wrapper->valueByName[namedValue.name()] = namedValue.value();
            wrapper->nameByValue[namedValue.value()] = namedValue.name();
        }
        // restore actionDependencyStates access buffer
        for (const auto & dep : action.type().action_dependency()) {
            wrapper->actionDependencyStates[dep.depends_on_action()].push_back(dep.depends_on_action_in_state());
        }
    }
}

const robot_remote_control::SimpleActions& SimpleActionsHelper::getSimpleActionsDefinition() {
    return actions;
}

std::shared_ptr<SimpleActionWrapper> SimpleActionsHelper::getSimpleAction(const std::string &actionname) {
    return actionbyname[actionname];
}

float SimpleActionsHelper::getActionState(const std::string &actionname) {
    return getSimpleAction(actionname)->getState();
}

void SimpleActionsHelper::setActionState(const std::string &actionname, const float &value) {
    getSimpleAction(actionname)->setState(value);
}

std::shared_ptr<SimpleActionWrapper> SimpleActionsHelper::addSimpleAction(const std::string &actionname, const robot_remote_control::SimpleActionType& type) {
    robot_remote_control::SimpleAction* action = actions.add_actions();
    action->set_name(actionname);
    action->mutable_type()->set_type(type);
    std::shared_ptr<SimpleActionWrapper> wrapper = std::make_shared<SimpleActionWrapper>(action);
    actionbyname[actionname] = wrapper;
    return wrapper;
}

bool SimpleActionsHelper::canExecute(const std::shared_ptr<SimpleActionWrapper> &action) {    
    //check dependencies (multiple entries on single actions: OR, between actions: AND)
    for (const auto& actionstate : action->actionDependencyStates) {
        bool has_match = false;
        float curr_state = getActionState(actionstate.first);
        //check or relation, if one is matching, the action has the correct state
        for (const auto& state : actionstate.second) {
            if (state ==  curr_state) {
                has_match = true;
            }
        }
        // if no state this action is not matching, action has unmet deps
        if (!has_match) {
            return false;
        }
    }
    return true;
}







SimpleActionWrapper::SimpleActionWrapper(robot_remote_control::SimpleAction* action): action(action) {

}

void SimpleActionWrapper::setLimits(const float &min_state, const float &max_state, const float &step_size) {
    action->mutable_type()->set_min_state(min_state);
    action->mutable_type()->set_max_state(max_state);
    action->mutable_type()->set_step_size(step_size);
}

/**
 * @brief add named values to action as a 
 * 
 * @param valuenames 
 */
void SimpleActionWrapper::addNamedValues(const std::map<std::string, float> & valuenames) {
    for (const auto& entry : valuenames) {
        addNamedValue(entry.first, entry.second);
    }
}

void SimpleActionWrapper::addNamedValue(const std::string &name, const float &value) {
    robot_remote_control::NamedValue *nv = action->mutable_type()->add_value_names();
    nv->set_name(name);
    nv->set_value(value);
    valueByName[name] = value;
    nameByValue[value] = name;
}

float SimpleActionWrapper::getNamedValue(const std::string &valuename) {
    return valueByName[valuename];
}

std::string SimpleActionWrapper::getValueName(const float &value) {
    return nameByValue[value];
}

void SimpleActionWrapper::addActionDependency(const std::string &name, const float &value) {
    robot_remote_control::ActionDependency* dep = action->mutable_type()->add_action_dependency();
    dep->set_depends_on_action(name);
    dep->set_depends_on_action_in_state(value);
    actionDependencyStates[name].push_back(value);
}


bool SimpleActionWrapper::isValidState(const float &value, const float& epsilon) {
    // it value exitst in named values, it is valid
    for (const auto& nv : action->type().value_names()) {
        if (value == nv.value()) {
            return true;
        }
    }
    // if it is not within min/max it is invalid
    if (value < action->type().min_state() || value > action->type().max_state()) {
        return false;
    }

    switch (action->type().type()) {
        case robot_remote_control::UNDEFINED:
             return false;
            //  break;
        case robot_remote_control::VALUE_INT: {
            if (nearbyintf(value) != value) { // rounded to int is not the same as numer vlaue not an int
                return false;
            }
            bool isInStep = false;
            if (action->type().step_size() == 0) {
                isInStep = true;
            } else {
                for (int i = action->type().min_state(); i<=action->type().max_state(); i += action->type().step_size()) {
                    if (i == value) {
                        isInStep=true;
                        break;
                    }
                }
            }
            if (!isInStep) {
                return false;
            }
            return true;
        }
        case robot_remote_control::VALUE_FLOAT:{
            bool isInStep = false;
            double values = (action->type().max_state() - action->type().min_state()) / action->type().step_size();
            if (action->type().step_size() == 0) {
                isInStep = true;
            } else {
                for (int i = 0; i< values; ++i) {
                    // to avoid imcremental rounding errors, calc offset in each step (min + i * stepzize) to see if it is matching
                    // there will be still an error so check against std::numeric_limits<float>::epsilon()
                    if (fabs(action->type().min_state() + (i * action->type().step_size()) - value) < epsilon) {
                        isInStep=true;
                        break;
                    }
                }
            }
            if (!isInStep) {
                return false;
            }
            return true;
        }
        case robot_remote_control::TRIGGER:
            return true;
    }
    return false;
}

