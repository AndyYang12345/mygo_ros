#pragma once

#include <string>
#include <vector>

#include "state_machine/robot_state.hpp"

class ArmState : public RobotState {
public:
  std::string getName() const override;
  uint8_t getStateEnum() const override;

  void onEnter(RobotStateMachineNode * context) override;
  void onExit(RobotStateMachineNode * context) override;
  void handleButton(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;
  void handleJoystick(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;
  void handleTrigger(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg) override;
  void handleCombo(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::ComboIntent::SharedPtr msg) override;
  void update(RobotStateMachineNode * context) override;

  uint8_t getSubState() const override;
  std::vector<std::string> getAvailableModes() const override;

private:
  void updateSubmenuUi(RobotStateMachineNode * context);
  int angleToOctant(float x, float y) const;

  // Index order follows octants: RIGHT, UP_RIGHT, UP, UP_LEFT, LEFT, DOWN_LEFT, DOWN, DOWN_RIGHT.
  // Left/right variants are mirrored around the vertical axis for intuitive selection.
  std::vector<std::string> presets_ = {
    "pickup_right",
    // "cylinder_right",
    "pose_1",
    "home",
    // "cylinder_left",
    "pose_2",
    "pickup_left",
    "box_left",
    "normal_detection",
    "box_right"
  };

  bool submenu_active_ = false;
  int submenu_selection_ = 0;
};
