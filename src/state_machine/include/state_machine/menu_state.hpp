#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "state_machine/robot_state.hpp"

class MenuState : public RobotState {
public:
    std::string getName() const override { return "MENU"; }
    uint8_t getStateEnum() const override { return 4; }
    uint8_t getSubState() const override { return static_cast<uint8_t>(selection_index_); }
    std::vector<std::string> getAvailableModes() const override;

    void onEnter(RobotStateMachineNode* context) override;
    void onExit(RobotStateMachineNode* context) override;

    void handleButton(RobotStateMachineNode* context,
                      const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;

    void handleJoystick(RobotStateMachineNode* context,
                        const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;

    void handleTrigger(RobotStateMachineNode* context,
                       const custom_interfaces::msg::TriggerIntent::SharedPtr msg) override;

    void update(RobotStateMachineNode* context) override;

private:
    const std::vector<std::pair<std::string, uint8_t>> menu_entries_ = {
        {"CHASSIS", 2},
        {"ARM", 3},
        {"BALL", 6},
        {"VISION_TASK", 7},
        {"IDLE", 1},
        {"POLE", 5},
    };

    int selection_index_ = 0;
    bool trigger_pressed_ = false;
    bool selection_touched_ = false;
    rclcpp::Time last_activity_;

    void refreshMenuState(RobotStateMachineNode* context);
    int angleToOctant(float x, float y) const;
    int octantToMenuIndex(int octant) const;
};
