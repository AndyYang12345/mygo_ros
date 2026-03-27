#include "state_machine/robot_state_machine_node.hpp"

#include <chrono>

#include "state_machine/arm_state.hpp"
#include "state_machine/ball_state.hpp"
#include "state_machine/chassis_state.hpp"
#include "state_machine/idle_state.hpp"
#include "state_machine/menu_state.hpp"
#include "state_machine/pole_state.hpp"
#include "state_machine/vision_task_state.hpp"

using namespace std::chrono_literals;

RobotStateMachineNode::RobotStateMachineNode()
    : Node("robot_state_machine")
{
    RCLCPP_INFO(this->get_logger(), "Robot State Machine Node starting...");
    // 在对应的状态中重载需要实现的函数即可
    states_[1] = std::make_shared<IdleState>();
    states_[2] = std::make_shared<ChassisState>();
    states_[3] = std::make_shared<ArmState>();
    states_[4] = std::make_shared<MenuState>();
    states_[5] = std::make_shared<PoleState>();
    states_[6] = std::make_shared<BallState>();
    states_[7] = std::make_shared<VisionTaskState>();

    current_state_ = states_[1].get();

    declareParameters();
    setupPublishers();
    setupSubscribers();
    setupTimers();

    if (current_state_)
    {
        current_state_->onEnter(this);
    }

    RCLCPP_INFO(
        this->get_logger(), "Robot State Machine Node started. Initial state: %s",
        current_state_->getName().c_str());
}

void RobotStateMachineNode::changeState(uint8_t state_enum)
{
    if (states_.find(state_enum) == states_.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid state enum: %d", state_enum);
        return;
    }

    auto new_state = states_[state_enum].get();
    if (current_state_ == new_state)
    {
        return;
    }

    if (state_enum == 4 && current_state_ != nullptr && current_state_->getStateEnum() != 4)
    {
        state_before_menu_ = current_state_->getStateEnum();
    }

    if (state_enum == 6 && current_state_ != nullptr && current_state_->getStateEnum() != 6)
    {
        state_before_ball_ = current_state_->getStateEnum();
    }

    if (!isTransitionAllowed(current_state_->getStateEnum(), state_enum))
    {
        RCLCPP_WARN(
            this->get_logger(), "State transition from %s to %s not allowed",
            current_state_->getName().c_str(), new_state->getName().c_str());
        return;
    }

    if (current_state_)
    {
        current_state_->onExit(this);
    }

    const uint8_t old_state_enum = current_state_->getStateEnum();
    auto old_state_name = current_state_->getName();
    current_state_ = new_state;
    current_state_->onEnter(this);

    if (old_state_enum == 4 && state_enum != 4)
    {
        joystick_block_until_ = this->now() + rclcpp::Duration::from_seconds(
            static_cast<double>(params_.menu_exit_joystick_delay_ms) / 1000.0);
        RCLCPP_INFO(
            this->get_logger(),
            "Joystick input blocked for %d ms after MENU selection",
            params_.menu_exit_joystick_delay_ms);
    }

    RCLCPP_INFO(
        this->get_logger(), "State changed: %s -> %s",
        old_state_name.c_str(), current_state_->getName().c_str());

    publishState();
}

uint8_t RobotStateMachineNode::getStateBeforeMenu() const
{
    return state_before_menu_;
}

uint8_t RobotStateMachineNode::getStateBeforeBall() const
{
    return state_before_ball_;
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

rclcpp::Publisher<custom_interfaces::msg::PoleCommand>::SharedPtr
RobotStateMachineNode::getPoleRotatePub()
{
    return pole_rotate_pub_;
}

rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr RobotStateMachineNode::getPoleSelectedIdPub()
{
    return pole_selected_id_pub_;
}

rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr RobotStateMachineNode::getPoleLockMaskPub()
{
    return pole_lock_mask_pub_;
}

rclcpp::Publisher<custom_interfaces::msg::ArmNamedTarget>::SharedPtr
RobotStateMachineNode::getArmNamedTargetPub()
{
    return arm_named_target_pub_;
}

rclcpp::Publisher<custom_interfaces::msg::ArmPoseTarget>::SharedPtr
RobotStateMachineNode::getArmPoseTargetPub()
{
    return arm_pose_target_pub_;
}

rclcpp::Publisher<custom_interfaces::msg::ArmJointTarget>::SharedPtr
RobotStateMachineNode::getArmJointTargetPub()
{
    return arm_joint_target_pub_;
}

rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr
RobotStateMachineNode::getArmJointCommandPub()
{
    return arm_joint_command_pub_;
}

rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr
RobotStateMachineNode::getArmDirectPwmPub()
{
    return arm_direct_pwm_pub_;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
RobotStateMachineNode::getArmQueryCurrentPub()
{
    return arm_query_current_pub_;
}

rclcpp::Publisher<custom_interfaces::msg::GripperCommand>::SharedPtr
RobotStateMachineNode::getArmGripperCmdPub()
{
    return arm_gripper_cmd_pub_;
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

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RobotStateMachineNode::getCameraStartAppPub()
{
    return camera_start_app_pub_;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RobotStateMachineNode::getCameraExitAppPub()
{
    return camera_exit_app_pub_;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RobotStateMachineNode::getCameraVisionStartPub()
{
    return camera_vision_start_pub_;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RobotStateMachineNode::getCameraVisionStopPub()
{
    return camera_vision_stop_pub_;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr RobotStateMachineNode::getCollectorCmdPub()
{
    return collector_cmd_pub_;
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

const RobotStateMachineNode::JoystickData &RobotStateMachineNode::getLeftJoystick() const
{
    return left_joystick_;
}

const RobotStateMachineNode::JoystickData &RobotStateMachineNode::getRightJoystick() const
{
    return right_joystick_;
}

void RobotStateMachineNode::setMenuItems(const std::vector<std::string> &items)
{
    menu_items_ = items;
}

const std::vector<std::string> &RobotStateMachineNode::getMenuItems() const
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

bool RobotStateMachineNode::consumeVisionTaskDone()
{
    if (!vision_task_done_)
    {
        return false;
    }
    vision_task_done_ = false;
    return true;
}

bool RobotStateMachineNode::isVisionTaskDone() const
{
    return vision_task_done_;
}

void RobotStateMachineNode::declareParameters()
{
    this->declare_parameter<double>("chassis.max_linear_speed", 0.5);
    this->declare_parameter<double>("chassis.max_angular_speed", 1.0);
    this->declare_parameter<double>("arm.speed_scale", 0.05);
    this->declare_parameter<double>("joystick.deadzone", 0.1);
    this->declare_parameter<int>("control.frequency", 50);
    this->declare_parameter<int>("menu.exit_joystick_delay_ms", 350);

    params_.chassis_max_linear_speed = this->get_parameter("chassis.max_linear_speed").as_double();
    params_.chassis_max_angular_speed = this->get_parameter("chassis.max_angular_speed").as_double();
    params_.arm_speed_scale = this->get_parameter("arm.speed_scale").as_double();
    params_.joystick_deadzone = this->get_parameter("joystick.deadzone").as_double();
    params_.control_frequency = this->get_parameter("control.frequency").as_int();
    params_.menu_exit_joystick_delay_ms = this->get_parameter("menu.exit_joystick_delay_ms").as_int();
}

void RobotStateMachineNode::setupPublishers()
{
    state_pub_ = this->create_publisher<custom_interfaces::msg::RobotState>("/robot/state", 10);
    chassis_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/chassis/velocity", 10);
    arm_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd/arm/velocity", 10);
    gripper_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/gripper/action", 10);
    pole_rotate_pub_ =
        this->create_publisher<custom_interfaces::msg::PoleCommand>("/cmd/pole/rotate", 10);
    pole_selected_id_pub_ =
        this->create_publisher<std_msgs::msg::UInt8>("/cmd/pole/selected_id", 10);
    pole_lock_mask_pub_ =
        this->create_publisher<std_msgs::msg::UInt8>("/cmd/pole/lock_mask", 10);
    arm_named_target_pub_ =
        this->create_publisher<custom_interfaces::msg::ArmNamedTarget>("/cmd/arm/named_target", 10);
    arm_pose_target_pub_ =
        this->create_publisher<custom_interfaces::msg::ArmPoseTarget>("/cmd/arm/pose_target", 10);
    arm_joint_target_pub_ =
        this->create_publisher<custom_interfaces::msg::ArmJointTarget>("/cmd/arm/joint_target", 10);
    arm_joint_command_pub_ =
        this->create_publisher<example_interfaces::msg::Float64MultiArray>("/cmd/arm/joint_command", 10);
    arm_direct_pwm_pub_ =
        this->create_publisher<example_interfaces::msg::Float64MultiArray>("/cmd/arm/direct_pwm_command", 10);
    arm_query_current_pub_ =
        this->create_publisher<std_msgs::msg::String>("/cmd/arm/query_current", 10);
    arm_gripper_cmd_pub_ =
        this->create_publisher<custom_interfaces::msg::GripperCommand>("/cmd/arm/gripper", 10);
    preset_pub_ = this->create_publisher<std_msgs::msg::Int32>("/cmd/preset/trigger", 10);
    joint_trajectory_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/cmd/arm/joint_trajectory", 10);
    camera_start_app_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/camera/start_app", 10);
    camera_exit_app_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/camera/exit_app", 10);
    camera_vision_start_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/camera/vision/start", 10);
    camera_vision_stop_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/camera/vision/stop", 10);
    collector_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd/collector/ball_submission", 10);
}

void RobotStateMachineNode::setupSubscribers()
{
    button_sub_ = this->create_subscription<custom_interfaces::msg::ButtonIntent>(
        "button_intent", 10,
        [this](const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
        {
            const bool home_pressed = (msg->button_id == 8) && (msg->event_type == 0);
            if (home_pressed && current_state_ && current_state_->getStateEnum() != 6)
            {
                this->changeState(6);
                return;
            }

            if (current_state_)
            {
                current_state_->handleButton(this, msg);
            }
        });

    joystick_sub_ = this->create_subscription<custom_interfaces::msg::JoystickIntent>(
        "joystick_intent", 10,
        [this](const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
        {
            if (msg->joystick_id == 0)
            {
                left_joystick_.x = msg->x;
                left_joystick_.y = msg->y;
                left_joystick_.timestamp = rclcpp::Time(msg->timestamp);
            }
            else if (msg->joystick_id == 1)
            {
                right_joystick_.x = msg->x;
                right_joystick_.y = msg->y;
                right_joystick_.timestamp = rclcpp::Time(msg->timestamp);
            }

            if (current_state_)
            {
                if (this->now() < joystick_block_until_)
                {
                    return;
                }
                current_state_->handleJoystick(this, msg);
            }
        });

    trigger_sub_ = this->create_subscription<custom_interfaces::msg::TriggerIntent>(
        "trigger_intent", 10,
        [this](const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
        {
            const bool lt_pressed =
                (msg->trigger_id == 0) && (msg->event_type == 1 || msg->value < -0.1F);

            if (lt_pressed && current_state_ && current_state_->getStateEnum() != 4)
            {
                RCLCPP_INFO(this->get_logger(), "LT pressed -> switching to MENU");
                this->changeState(4);
                return;
            }

            if (current_state_)
            {
                current_state_->handleTrigger(this, msg);
            }
        });

    combo_sub_ = this->create_subscription<custom_interfaces::msg::ComboIntent>(
        "combo_intent", 10,
        [this](const custom_interfaces::msg::ComboIntent::SharedPtr msg)
        {
            if (current_state_)
            {
                current_state_->handleCombo(this, msg);
            }
        });

    vision_task_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/vision/task_done", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (msg->data)
            {
                vision_task_done_ = true;
                RCLCPP_INFO(this->get_logger(), "Received /vision/task_done=true");
            }
        });

    set_mode_service_ = this->create_service<custom_interfaces::srv::SetMode>(
        "/robot/set_mode",
        [this](
            const custom_interfaces::srv::SetMode::Request::SharedPtr request,
            custom_interfaces::srv::SetMode::Response::SharedPtr response)
        {
            const auto target_state = mapSetModeToState(request->target_mode);
            if (states_.find(target_state) == states_.end())
            {
                response->success = false;
                response->message = "Unsupported target_mode";
                response->timestamp = this->now();
                return;
            }

            this->changeState(target_state);
            response->success = (current_state_ && current_state_->getStateEnum() == target_state);
            response->message = response->success ? "mode changed" : "transition rejected";
            response->timestamp = this->now();
        });
}

void RobotStateMachineNode::setupTimers()
{
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / params_.control_frequency),
        [this]()
        {
            if (current_state_)
            {
                current_state_->update(this);
            }
        });

    state_timer_ = this->create_wall_timer(100ms, std::bind(&RobotStateMachineNode::publishState, this));
}

bool RobotStateMachineNode::isTransitionAllowed(uint8_t from, uint8_t to)
{
    if (from == to)
    {
        return true;
    }

    if (from == 6)
    {
        return to == state_before_ball_ || to == 1;
    }

    if (to == 6)
    {
        return true;
    }

    if (from == 1)
    {
        return to == 4;
    }

    if (from == 4)
    {
        return to == 1 || to == 2 || to == 3 || to == 5 || to == 6 || to == 7;
    }

    if (from == 2 || from == 3 || from == 5 || from == 7)
    {
        return to == 4;
    }

    return false;
}

void RobotStateMachineNode::publishState()
{
    auto msg = custom_interfaces::msg::RobotState();
    msg.main_state = current_state_->getStateEnum();
    msg.sub_state = current_state_->getSubState();
    msg.state_name = current_state_->getName();
    msg.available_modes = current_state_->getAvailableModes();
    msg.menu_items = menu_items_;
    msg.menu_selection = static_cast<uint8_t>(menu_selection_);
    msg.health_status = 0;
    msg.error_message = "";
    msg.flags = 0;
    msg.timestamp = this->now();
    state_pub_->publish(msg);
}

uint8_t RobotStateMachineNode::mapSetModeToState(uint8_t target_mode) const
{
    switch (target_mode)
    {
        case 0:
            return 1;  // IDLE
        case 1:
            return 2;  // CHASSIS
        case 2:
            return 3;  // ARM
        case 3:
            return 4;  // MENU
        case 4:
            return 5;  // POLE
        case 5:
            return 7;  // VISION TASK
        case 6:
            return 6;  // BALL
        default:
            return 0;
    }
}

void RobotStateMachineNode::sendStopCommands()
{
    chassis_cmd_pub_->publish(geometry_msgs::msg::Twist());
    arm_cmd_pub_->publish(geometry_msgs::msg::Twist());

    auto arm_gripper_stop = custom_interfaces::msg::GripperCommand();
    arm_gripper_stop.open = true;
    arm_gripper_cmd_pub_->publish(arm_gripper_stop);

    auto pole_stop = custom_interfaces::msg::PoleCommand();
    pole_stop.id = false;
    pole_stop.delta = 0.0F;
    pole_rotate_pub_->publish(pole_stop);
}
