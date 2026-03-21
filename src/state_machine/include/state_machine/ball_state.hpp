#pragma once

#include "state_machine/robot_state.hpp"

class BallState : public RobotState
{
public:
    std::string getName() const override;
    uint8_t getStateEnum() const override;

    void onEnter(RobotStateMachineNode *context) override;
    void onExit(RobotStateMachineNode *context) override;
    void handleButton(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;
    void update(RobotStateMachineNode *context) override;

private:
    bool a_pressed_latched_ = false;
};
