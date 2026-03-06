#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <custom_interfaces/msg/arm_pose_target.hpp>
#include <custom_interfaces/msg/gripper_command.hpp>

using Float64MultiArray = example_interfaces::msg::Float64MultiArray;
using ArmPoseTarget = custom_interfaces::msg::ArmPoseTarget;
using GripperCommand = custom_interfaces::msg::GripperCommand;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class Commander{
public:
    Commander(std::shared_ptr<rclcpp::Node> node) : node_(node){
        RCLCPP_INFO(node_->get_logger(), "Commander node has been started.");
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        open_gripper_sub_ = node_->create_subscription<GripperCommand>(
            "open_gripper",
            10,
            std::bind(&Commander::openGripperCallback, this, std::placeholders::_1)
        );
        joint_cmd_sub_ = node_->create_subscription<Float64MultiArray>(
            "joint_command",
            10,
            std::bind(&Commander::jointCmdCallback, this, std::placeholders::_1)
        );
        pose_cmd_sub_ = node_->create_subscription<ArmPoseTarget>(
            "arm_pose_target",
            10,
            std::bind(&Commander::poseCmdCallback, this, std::placeholders::_1)
        );
    }

    void goToNamedTarget(const std::string &target_name){
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(target_name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints){
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double pitch, double roll, double yaw, bool cartesian_path = false){
        geometry_msgs::msg::Pose pose;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        arm_->setStartStateToCurrentState();
        if (!cartesian_path) {
            arm_->setPoseTarget(pose);
            planAndExecute(arm_);
        }else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
            if(fraction == 1.0){
                arm_->execute(trajectory);
            }
        }
    }
    void openGripper(){
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }
    void closeGripper(){
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }
private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface){
        // Here you can add your MoveIt! related code to test the functionality.
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Planning was successful.");
            interface->execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
        }

    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<GripperCommand>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<ArmPoseTarget>::SharedPtr pose_cmd_sub_;

    void openGripperCallback(const GripperCommand &msg){
        if (msg.open) {
            openGripper();
        } else {
            closeGripper();
        }
    }
    void jointCmdCallback(const Float64MultiArray &msg){
        auto joints = msg.data;
        if (joints.size() == 5) {
            goToJointTarget(joints);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Received joint command with incorrect size: %zu (expected 5)", joints.size());
        }
    }   
    void poseCmdCallback(const ArmPoseTarget &msg){
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }
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