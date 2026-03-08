#pragma once

#include <array>

#include "state_machine/robot_state.hpp"

class PoleState : public RobotState
{
public:
    std::string getName() const override;
    uint8_t getStateEnum() const override;
    uint8_t getSubState() const override;
    std::vector<std::string> getAvailableModes() const override;

    void onEnter(RobotStateMachineNode *context) override;
    void onExit(RobotStateMachineNode *context) override;

    void handleButton(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;

    void handleJoystick(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;

    void handleTrigger(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::TriggerIntent::SharedPtr msg) override;

    void update(RobotStateMachineNode *context) override;

private:
    std::array<bool, 2> pole_locked_ = {false, false};
    bool rt_pressed_ = false;
    uint8_t selected_id_ = 0;
    double applyDeadzone(double value, double deadzone) const;
    void publishSelectedId(RobotStateMachineNode *context) const;
    void publishLockMask(RobotStateMachineNode *context) const;
};
