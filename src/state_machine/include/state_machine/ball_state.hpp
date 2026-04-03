#pragma once

#include <vector>

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
    void handleJoystick(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;
    void update(RobotStateMachineNode *context) override;

    uint8_t getSubState() const override;
    std::vector<std::string> getAvailableModes() const override;

private:
    using CommandHandler = void (BallState::*)(RobotStateMachineNode *context);

    struct CommandItem {
        const char *label;
        CommandHandler handler;
    };

    bool a_pressed_latched_ = false;
    bool submenu_active_ = false;
    int submenu_selection_ = 0;
    std::vector<CommandItem> submenu_items_;

    void updateSubmenuUi(RobotStateMachineNode *context);
    int angleToOctant(float x, float y) const;
    void publishCollectorCommand(RobotStateMachineNode *context, const std::string &command);
    void publishNextCollectorCommand(RobotStateMachineNode *context);
    void publishCollectorMiddleCommand(RobotStateMachineNode *context);
    void publishCollectorDownCommand(RobotStateMachineNode *context);
    void publishCollectorUpCommand(RobotStateMachineNode *context);
    void publishCollectorOpenCommand(RobotStateMachineNode *context);
    void publishCollectorCloseCommand(RobotStateMachineNode *context);
    void publishNoOpCommand(RobotStateMachineNode *context);
};
