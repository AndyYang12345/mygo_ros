#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/msg/button_intent.hpp"
#include "custom_interfaces/msg/combo_intent.hpp"
#include "custom_interfaces/msg/joystick_intent.hpp"
#include "custom_interfaces/msg/robot_state.hpp"
#include "custom_interfaces/msg/trigger_intent.hpp"
#include "custom_interfaces/msg/arm_joint_target.hpp"
#include "custom_interfaces/msg/arm_named_target.hpp"
#include "custom_interfaces/msg/arm_pose_target.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "custom_interfaces/msg/pole_command.hpp"
#include "custom_interfaces/srv/set_mode.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine/robot_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class RobotStateMachineNode : public rclcpp::Node
{
public:
    RobotStateMachineNode();

    void changeState(uint8_t state_enum);
    uint8_t getStateBeforeMenu() const;
    uint8_t getStateBeforeEmergency() const;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr getChassisCmdPub();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr getArmCmdPub();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr getGripperCmdPub();
    rclcpp::Publisher<custom_interfaces::msg::PoleCommand>::SharedPtr getPoleRotatePub();
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr getPoleSelectedIdPub();
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr getPoleLockMaskPub();
    rclcpp::Publisher<custom_interfaces::msg::ArmNamedTarget>::SharedPtr getArmNamedTargetPub();
    rclcpp::Publisher<custom_interfaces::msg::ArmPoseTarget>::SharedPtr getArmPoseTargetPub();
    rclcpp::Publisher<custom_interfaces::msg::ArmJointTarget>::SharedPtr getArmJointTargetPub();
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr getArmJointCommandPub();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr getArmQueryCurrentPub();
    rclcpp::Publisher<custom_interfaces::msg::GripperCommand>::SharedPtr getArmGripperCmdPub();
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr getPresetPub();
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr getJointTrajectoryPub();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr getCameraStartAppPub();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr getCameraExitAppPub();

    double getChassisMaxLinearSpeed() const;
    double getChassisMaxAngularSpeed() const;
    double getArmSpeedScale() const;
    double getJoystickDeadzone() const;

    struct JoystickData
    {
        float x = 0.0;
        float y = 0.0;
        rclcpp::Time timestamp;
        bool is_active(const rclcpp::Node* node, double timeout_seconds = 0.2) const {
            auto now = node->now();
            return (now - timestamp).seconds() < timeout_seconds;
        }
    };

    const JoystickData &getLeftJoystick() const;
    const JoystickData &getRightJoystick() const;

    void setMenuItems(const std::vector<std::string> &items);
    const std::vector<std::string> &getMenuItems() const;
    void setMenuSelection(int index);
    int getMenuSelection() const;

    bool consumeVisionTaskDone();
    bool isVisionTaskDone() const;

    void sendStopCommands();

private:
    void declareParameters();
    void setupPublishers();
    void setupSubscribers();
    void setupTimers();

    bool isTransitionAllowed(uint8_t from, uint8_t to);
    void publishState();
    uint8_t mapSetModeToState(uint8_t target_mode) const;

    std::map<uint8_t, std::shared_ptr<RobotState>> states_;
    RobotState *current_state_ = nullptr;
    uint8_t state_before_menu_ = 1;
    uint8_t state_before_emergency_ = 1;

    struct Parameters
    {
        double chassis_max_linear_speed = 1.0;
        double chassis_max_angular_speed = 1.0;
        double arm_speed_scale = 0.05;
        double joystick_deadzone = 0.1;
        int control_frequency = 50;
        int menu_exit_joystick_delay_ms = 350;
    } params_;

    std::vector<std::string> menu_items_;
    int menu_selection_ = 0;

    JoystickData left_joystick_;
    JoystickData right_joystick_;
    rclcpp::Time joystick_block_until_{0, 0, RCL_ROS_TIME};
    bool vision_task_done_ = false;

    rclcpp::Publisher<custom_interfaces::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_cmd_pub_;
    rclcpp::Publisher<custom_interfaces::msg::PoleCommand>::SharedPtr pole_rotate_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pole_selected_id_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pole_lock_mask_pub_;
    rclcpp::Publisher<custom_interfaces::msg::ArmNamedTarget>::SharedPtr arm_named_target_pub_;
    rclcpp::Publisher<custom_interfaces::msg::ArmPoseTarget>::SharedPtr arm_pose_target_pub_;
    rclcpp::Publisher<custom_interfaces::msg::ArmJointTarget>::SharedPtr arm_joint_target_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr arm_joint_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_query_current_pub_;
    rclcpp::Publisher<custom_interfaces::msg::GripperCommand>::SharedPtr arm_gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr preset_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_start_app_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_exit_app_pub_;

    rclcpp::Subscription<custom_interfaces::msg::ButtonIntent>::SharedPtr button_sub_;
    rclcpp::Subscription<custom_interfaces::msg::JoystickIntent>::SharedPtr joystick_sub_;
    rclcpp::Subscription<custom_interfaces::msg::TriggerIntent>::SharedPtr trigger_sub_;
    rclcpp::Subscription<custom_interfaces::msg::ComboIntent>::SharedPtr combo_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_task_done_sub_;

    rclcpp::Service<custom_interfaces::srv::SetMode>::SharedPtr set_mode_service_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
};
