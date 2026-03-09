#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <custom_interfaces/msg/arm_named_target.hpp>
#include <custom_interfaces/msg/arm_pose_target.hpp>
#include <custom_interfaces/msg/arm_joint_target.hpp>
#include <custom_interfaces/msg/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <thread>
#include <unordered_map>

using Float64MultiArray = example_interfaces::msg::Float64MultiArray;
using ArmNamedTarget = custom_interfaces::msg::ArmNamedTarget;
using ArmPoseTarget = custom_interfaces::msg::ArmPoseTarget;
using ArmJointTarget = custom_interfaces::msg::ArmJointTarget;
using GripperCommand = custom_interfaces::msg::GripperCommand;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class Commander {
public:
    explicit Commander(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {
        RCLCPP_INFO(node_->get_logger(), "Commander node has been started.");

        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        open_gripper_sub_ = node_->create_subscription<GripperCommand>(
            "/cmd/arm/gripper",
            10,
            std::bind(&Commander::openGripperCallback, this, std::placeholders::_1));
        joint_cmd_sub_ = node_->create_subscription<Float64MultiArray>(
            "/cmd/arm/joint_command",
            10,
            std::bind(&Commander::jointCmdCallback, this, std::placeholders::_1));
        named_target_sub_ = node_->create_subscription<ArmNamedTarget>(
            "/cmd/arm/named_target",
            10,
            std::bind(&Commander::namedTargetCallback, this, std::placeholders::_1));
        pose_cmd_sub_ = node_->create_subscription<ArmPoseTarget>(
            "/cmd/arm/pose_target",
            10,
            std::bind(&Commander::poseCmdCallback, this, std::placeholders::_1));

        arm_joint_target_pub_ = node_->create_publisher<ArmJointTarget>(
            "/cmd/arm/joint_target",
            10);
    }

    void goToNamedTarget(const std::string &target_name) {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(target_name);
        planAndExecute(arm_);
        last_named_target_ = target_name;
    }

    void goToJointTarget(const std::vector<double> &joints) {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        bool cartesian_path = false) {
        const auto current_pose = arm_->getCurrentPose().pose;
        geometry_msgs::msg::Pose pose = current_pose;

        pose.position.x += x;
        pose.position.y += y;
        pose.position.z += z;

        tf2::Quaternion q_current(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);
        tf2::Quaternion q_delta;
        q_delta.setRPY(roll, pitch, yaw);
        tf2::Quaternion q_target = q_current * q_delta;
        q_target.normalize();
        pose.orientation.x = q_target.x();
        pose.orientation.y = q_target.y();
        pose.orientation.z = q_target.z();
        pose.orientation.w = q_target.w();

        arm_->setStartStateToCurrentState();
        if (!cartesian_path) {
            arm_->clearPoseTargets();
            const bool target_ok = arm_->setPoseTarget(pose);
            if (!target_ok) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Failed to set pose delta target: dx=%.3f dy=%.3f dz=%.3f dr=%.3f dp=%.3f dy=%.3f",
                    x,
                    y,
                    z,
                    roll,
                    pitch,
                    yaw);
                return;
            }
            planAndExecute(arm_);
        } else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            if (fraction == 1.0) {
                if (!publishArmTrajectory(trajectory.joint_trajectory)) {
                    RCLCPP_WARN(node_->get_logger(), "Failed to publish cartesian trajectory to /cmd/arm/joint_target.");
                }
                arm_->execute(trajectory);
            }
        }
    }

    void openGripper() {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper() {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }

private:
    bool publishArmTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory) {
        if (trajectory.points.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Planned arm trajectory has no points.");
            return false;
        }

        static const std::vector<std::string> kArmJointNames = {
            "joint1", "joint2", "joint3", "joint4", "joint5"};

        std::unordered_map<std::string, size_t> name_to_index;
        name_to_index.reserve(trajectory.joint_names.size());
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            name_to_index[trajectory.joint_names[i]] = i;
        }

        std::vector<int> indices;
        indices.reserve(kArmJointNames.size());
        for (const auto &joint_name : kArmJointNames) {
            const auto it = name_to_index.find(joint_name);
            if (it == name_to_index.end()) {
                RCLCPP_ERROR(node_->get_logger(), "Trajectory does not contain required joint: %s", joint_name.c_str());
                return false;
            }
            indices.push_back(static_cast<int>(it->second));
        }

        rclcpp::Duration previous_from_start(0, 0);
        for (const auto &point : trajectory.points) {
            ArmJointTarget msg;
            msg.joints.reserve(kArmJointNames.size());
            for (const int idx : indices) {
                if (idx < 0 || static_cast<size_t>(idx) >= point.positions.size()) {
                    RCLCPP_ERROR(node_->get_logger(), "Trajectory point index out of range when publishing arm joint target.");
                    return false;
                }
                msg.joints.push_back(point.positions[static_cast<size_t>(idx)]);
            }

            arm_joint_target_pub_->publish(msg);

            const rclcpp::Duration current_from_start(point.time_from_start);
            const auto delta = current_from_start - previous_from_start;
            previous_from_start = current_from_start;

            if (delta.nanoseconds() > 0) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(delta.nanoseconds()));
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Published %zu trajectory points to /cmd/arm/joint_target", trajectory.points.size());
        return true;
    }

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Planning was successful.");
            if (interface.get() == arm_.get()) {
                if (!publishArmTrajectory(plan.trajectory.joint_trajectory)) {
                    RCLCPP_WARN(node_->get_logger(), "Failed to publish planned arm trajectory to /cmd/arm/joint_target.");
                }
            }
            interface->execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
        }
    }

    void namedTargetCallback(const ArmNamedTarget::SharedPtr msg) {
        if (!msg) {
            return;
        }
        goToNamedTarget(msg->target_name);
    }

    void openGripperCallback(const GripperCommand::SharedPtr msg) {
        if (!msg) {
            return;
        }
        if (msg->open) {
            openGripper();
        } else {
            closeGripper();
        }
    }

    void jointCmdCallback(const Float64MultiArray::SharedPtr msg) {
        if (!msg) {
            return;
        }
        auto joints = msg->data;
        if (joints.size() == 5) {
            goToJointTarget(joints);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Received joint command with incorrect size: %zu (expected 5)", joints.size());
        }
    }

    void poseCmdCallback(const ArmPoseTarget::SharedPtr msg) {
        if (!msg) {
            return;
        }
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "Received pose_target: x=%.3f y=%.3f z=%.3f r=%.3f p=%.3f y=%.3f cartesian=%s",
            msg->x,
            msg->y,
            msg->z,
            msg->roll,
            msg->pitch,
            msg->yaw,
            msg->cartesian_path ? "true" : "false");
        goToPoseTarget(msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw, msg->cartesian_path);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<GripperCommand>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<ArmNamedTarget>::SharedPtr named_target_sub_;
    rclcpp::Subscription<ArmPoseTarget>::SharedPtr pose_cmd_sub_;
    rclcpp::Publisher<ArmJointTarget>::SharedPtr arm_joint_target_pub_;

    std::string last_named_target_ = "home";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
