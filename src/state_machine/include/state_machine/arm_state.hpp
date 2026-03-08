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

  std::vector<std::string> presets_ = {
    "home",
    "normal_detection",
    "pickup_left",
    "pickup_right",
    "cylinder_left",
    "cylinder_right",
    "box_left",
    "box_right"
  };

  // Map 8-direction submenu selections to valid MoveIt named targets.
  // mygo.srdf currently provides: home, pose_1, pose_2.
  std::vector<std::string> named_target_map_ = {
    "home",
    "pose_1",
    "pose_2",
    "pose_1",
    "home",
    "pose_2",
    "pose_1",
    "home"
  };

  bool submenu_active_ = false;
  int submenu_selection_ = 0;
};
