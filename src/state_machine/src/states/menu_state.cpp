#include "state_machine/menu_state.hpp"

#include <cmath>

#include "state_machine/robot_state_machine_node.hpp"
#include "std_msgs/msg/int32.hpp"

std::string MenuState::getName() const
{
  return "MENU";
}

uint8_t MenuState::getStateEnum() const
{
  return 4;
}

void MenuState::onEnter(RobotStateMachineNode * context)
{
  RCLCPP_INFO(context->get_logger(), "Entered MENU mode");
  menu_items_ = context->getMenuItems();
  selected_index_ = context->getMenuSelection();
}

void MenuState::handleButton(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
  if (msg->event_type != 0 || menu_items_.empty()) {
    return;
  }

  switch (msg->button_id) {
    case 0:
    {
      std::string selected = menu_items_[selected_index_];
      RCLCPP_INFO(context->get_logger(), "MENU: Confirmed %s", selected.c_str());

      if (selected == "ARM Control") {
        context->changeState(3);
      } else if (selected == "Preset 1") {
        auto preset_msg = std_msgs::msg::Int32();
        preset_msg.data = 1;
        context->getPresetPub()->publish(preset_msg);
      } else if (selected == "Preset 2") {
        auto preset_msg = std_msgs::msg::Int32();
        preset_msg.data = 2;
        context->getPresetPub()->publish(preset_msg);
      }
      break;
    }
    case 1:
      RCLCPP_INFO(context->get_logger(), "MENU: Back to CHASSIS");
      context->changeState(2);
      break;
    case 4:
    case 5:
    default:
      break;
  }
}

void MenuState::handleJoystick(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
  if (msg->joystick_id != 0 || menu_items_.empty()) {
    return;
  }

  static bool was_centered = true;
  bool is_centered = (std::abs(msg->x) < 0.2 && std::abs(msg->y) < 0.2);

  if (was_centered && !is_centered) {
    if (std::abs(msg->x) > std::abs(msg->y)) {
      if (msg->x > 0) {
        selected_index_ = (selected_index_ + 1) % static_cast<int>(menu_items_.size());
      } else {
        selected_index_ =
          (selected_index_ - 1 + static_cast<int>(menu_items_.size())) % static_cast<int>(menu_items_.size());
      }
      context->setMenuSelection(selected_index_);
      RCLCPP_INFO(context->get_logger(), "MENU: Selected %s", menu_items_[selected_index_].c_str());
    }
  }

  was_centered = is_centered;
}

void MenuState::handleTrigger(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
  if (msg->event_type != 1 || menu_items_.empty()) {
    return;
  }

  if (msg->trigger_id == 0) {
    selected_index_ =
      (selected_index_ - 1 + static_cast<int>(menu_items_.size())) % static_cast<int>(menu_items_.size());
  } else if (msg->trigger_id == 1) {
    selected_index_ = (selected_index_ + 1) % static_cast<int>(menu_items_.size());
  }

  context->setMenuSelection(selected_index_);
  RCLCPP_INFO(context->get_logger(), "MENU: Selected %s", menu_items_[selected_index_].c_str());
}

void MenuState::handleCombo(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ComboIntent::SharedPtr msg)
{
  if (msg->combo_name == "LT_RT") {
    auto synthetic_msg = std::make_shared<custom_interfaces::msg::ButtonIntent>();
    synthetic_msg->button_id = 0;
    synthetic_msg->event_type = 0;
    handleButton(context, synthetic_msg);
  }
}
