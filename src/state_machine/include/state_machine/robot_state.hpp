#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/msg/button_intent.hpp"
#include "custom_interfaces/msg/combo_intent.hpp"
#include "custom_interfaces/msg/joystick_intent.hpp"
#include "custom_interfaces/msg/trigger_intent.hpp"

class RobotStateMachineNode;

/*
    @brief 机器人状态基类，定义了所有状态的接口
    @param context 状态机上下文，提供对状态机节点的访问
    @param msg 各种输入消息的智能指针，包含按钮、摇杆
*/ 
class RobotState {
public:
  virtual ~RobotState() = default;

  virtual std::string getName() const = 0;
  virtual uint8_t getStateEnum() const = 0;
  virtual uint8_t getSubState() const { return 0; }
  virtual std::vector<std::string> getAvailableModes() const { return {}; }

  virtual void onEnter(RobotStateMachineNode * context) {}
  virtual void onExit(RobotStateMachineNode * context) {}

  virtual void handleButton(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg) {}

  virtual void handleJoystick(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg) {}

  virtual void handleTrigger(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg) {}

  virtual void handleCombo(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::ComboIntent::SharedPtr msg) {}

  virtual void update(RobotStateMachineNode * context) {}
};
