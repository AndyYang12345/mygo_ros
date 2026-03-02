#include "state_machine/robot_state_machine_node.hpp"

#include <chrono>

#include "state_machine/arm_state.hpp"
#include "state_machine/chassis_state.hpp"
#include "state_machine/emergency_state.hpp"
#include "state_machine/idle_state.hpp"
#include "state_machine/menu_state.hpp"

using namespace std::chrono_literals;

RobotStateMachineNode::RobotStateMachineNode()
: Node("robot_state_machine")
{
  RCLCPP_INFO(this->get_logger(), "Robot State Machine Node starting...");

  states_[1] = std::make_shared<IdleState>();
  states_[2] = std::make_shared<ChassisState>();
  states_[3] = std::make_shared<ArmState>();
  states_[4] = std::make_shared<MenuState>();
  states_[6] = std::make_shared<EmergencyState>();

  current_state_ = states_[1].get();

  declareParameters();
  setupPublishers();
  setupSubscribers();
  setupTimers();

  if (current_state_) {
    current_state_->onEnter(this);
  }

  RCLCPP_INFO(
    this->get_logger(), "Robot State Machine Node started. Initial state: %s",
    current_state_->getName().c_str());
}

void RobotStateMachineNode::changeState(uint8_t state_enum)
{
  if (states_.find(state_enum) == states_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid state enum: %d", state_enum);
    return;
  }

  auto new_state = states_[state_enum].get();
  if (current_state_ == new_state) {
    return;
  }

  if (!isTransitionAllowed(current_state_->getStateEnum(), state_enum)) {
    RCLCPP_WARN(
      this->get_logger(), "State transition from %s to %s not allowed",
      current_state_->getName().c_str(), new_state->getName().c_str());
    return;
  }

  if (current_state_) {
    current_state_->onExit(this);
  }

  auto old_state_name = current_state_->getName();
  current_state_ = new_state;
  current_state_->onEnter(this);

  RCLCPP_INFO(
    this->get_logger(), "State changed: %s -> %s",
    old_state_name.c_str(), current_state_->getName().c_str());

  publishState();
}

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr RobotStateMachineNode::getChassisCmdPub()
{
  return chassis_cmd_pub_;
}

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr RobotStateMachineNode::getArmCmdPub()
{
  return arm_cmd_pub_;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RobotStateMachineNode::getGripperCmdPub()
{
  return gripper_cmd_pub_;
}

rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr RobotStateMachineNode::getPresetPub()
{
  return preset_pub_;
}

rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
RobotStateMachineNode::getJointTrajectoryPub()
{
  return joint_trajectory_pub_;
}

double RobotStateMachineNode::getChassisMaxLinearSpeed() const
{
  return params_.chassis_max_linear_speed;
}

double RobotStateMachineNode::getChassisMaxAngularSpeed() const
{
  return params_.chassis_max_angular_speed;
}

double RobotStateMachineNode::getArmSpeedScale() const
{
  return params_.arm_speed_scale;
}

double RobotStateMachineNode::getJoystickDeadzone() const
{
  return params_.joystick_deadzone;
}

const RobotStateMachineNode::JoystickData & RobotStateMachineNode::getLeftJoystick() const
{
  return left_joystick_;
}

const RobotStateMachineNode::JoystickData & RobotStateMachineNode::getRightJoystick() const
{
  return right_joystick_;
}

void RobotStateMachineNode::setMenuItems(const std::vector<std::string> & items)
{
  menu_items_ = items;
}

const std::vector<std::string> & RobotStateMachineNode::getMenuItems() const
{
  return menu_items_;
}

void RobotStateMachineNode::setMenuSelection(int index)
{
  menu_selection_ = index;
}

int RobotStateMachineNode::getMenuSelection() const
{
  return menu_selection_;
}

void RobotStateMachineNode::declareParameters()
{
  this->declare_parameter<double>("chassis.max_linear_speed", 0.5);
  this->declare_parameter<double>("chassis.max_angular_speed", 1.0);
  this->declare_parameter<double>("arm.speed_scale", 0.05);
  this->declare_parameter<double>("joystick.deadzone", 0.1);
  this->declare_parameter<int>("control.frequency", 50);

  params_.chassis_max_linear_speed = this->get_parameter("chassis.max_linear_speed").as_double();
  params_.chassis_max_angular_speed = this->get_parameter("chassis.max_angular_speed").as_double();
  params_.arm_speed_scale = this->get_parameter("arm.speed_scale").as_double();
  params_.joystick_deadzone = this->get_parameter("joystick.deadzone").as_double();
  params_.control_frequency = this->get_parameter("control.frequency").as_int();
}

void RobotStateMachineNode::setupPublishers()
{
  state_pub_ = this->create_publisher<custom_interfaces::msg::RobotState>("/robot/state", 10);
  chassis_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/chassis/velocity", 10);
  arm_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/arm/velocity", 10);
  gripper_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/gripper/action", 10);
  preset_pub_ = this->create_publisher<std_msgs::msg::Int32>("/cmd/preset/trigger", 10);
  joint_trajectory_pub_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/cmd/arm/joint_trajectory", 10);
}

void RobotStateMachineNode::setupSubscribers()
{
  button_sub_ = this->create_subscription<custom_interfaces::msg::ButtonIntent>(
    "button_intent", 10,
    [this](const custom_interfaces::msg::ButtonIntent::SharedPtr msg) {
      if (current_state_) {
        current_state_->handleButton(this, msg);
      }
    });

  joystick_sub_ = this->create_subscription<custom_interfaces::msg::JoystickIntent>(
    "joystick_intent", 10,
    [this](const custom_interfaces::msg::JoystickIntent::SharedPtr msg) {
      if (msg->joystick_id == 0) {
        left_joystick_.x = msg->x;
        left_joystick_.y = msg->y;
        left_joystick_.timestamp = rclcpp::Time(msg->timestamp);
      } else if (msg->joystick_id == 1) {
        right_joystick_.x = msg->x;
        right_joystick_.y = msg->y;
        right_joystick_.timestamp = rclcpp::Time(msg->timestamp);
      }

      if (current_state_) {
        current_state_->handleJoystick(this, msg);
      }
    });

  trigger_sub_ = this->create_subscription<custom_interfaces::msg::TriggerIntent>(
    "trigger_intent", 10,
    [this](const custom_interfaces::msg::TriggerIntent::SharedPtr msg) {
      if (current_state_) {
        current_state_->handleTrigger(this, msg);
      }
    });

  combo_sub_ = this->create_subscription<custom_interfaces::msg::ComboIntent>(
    "combo_intent", 10,
    [this](const custom_interfaces::msg::ComboIntent::SharedPtr msg) {
      if (current_state_) {
        current_state_->handleCombo(this, msg);
      }
    });
}

void RobotStateMachineNode::setupTimers()
{
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / params_.control_frequency),
    [this]() {
      if (current_state_) {
        current_state_->update(this);
      }
    });

  state_timer_ = this->create_wall_timer(100ms, std::bind(&RobotStateMachineNode::publishState, this));
}

bool RobotStateMachineNode::isTransitionAllowed(uint8_t from, uint8_t to)
{
  if (from == 6) {
    return to == 1;
  }
  if (to == 6) {
    return true;
  }
  return true;
}

void RobotStateMachineNode::publishState()
{
  auto msg = custom_interfaces::msg::RobotState();
  msg.main_state = current_state_->getStateEnum();
  msg.state_name = current_state_->getName();
  msg.menu_items = menu_items_;
  msg.menu_selection = static_cast<uint8_t>(menu_selection_);
  msg.timestamp = this->now();
  state_pub_->publish(msg);
}

void RobotStateMachineNode::sendStopCommands()
{
  chassis_cmd_pub_->publish(geometry_msgs::msg::Twist());
  arm_cmd_pub_->publish(geometry_msgs::msg::Twist());

  auto gripper_stop = std_msgs::msg::String();
  gripper_stop.data = "STOP";
  gripper_cmd_pub_->publish(gripper_stop);
}
