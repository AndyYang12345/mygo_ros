#pragma once

#include "state_machine/robot_state.hpp"
#include <vector>

class ChassisState : public RobotState
{
public:
    ChassisState();
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
    using CommandHandler = void (ChassisState::*)(RobotStateMachineNode *context);

    struct CommandItem {
        const char *label;
        CommandHandler handler;
    };

    bool submenu_active_ = false;
    int submenu_selection_ = 0;
    std::vector<CommandItem> submenu_items_;

    // 辅助函数
    double applyDeadzone(double value, double deadzone) const;
    void publishCollectorCommand(RobotStateMachineNode *context, const std::string &command);
    void publishArmUnderBridgeCommand(RobotStateMachineNode *context);
    void publishArmFoldedCommand(RobotStateMachineNode *context);
    void publishCollectorMiddleCommand(RobotStateMachineNode *context);
    void publishCollectorDownCommand(RobotStateMachineNode *context);
    void publishNoOpCommand(RobotStateMachineNode *context);

    // 统一底盘控制逻辑。
    void processChassisControl(RobotStateMachineNode *context);
    void updateSubmenuUi(RobotStateMachineNode *context);
    int angleToOctant(float x, float y) const;
};