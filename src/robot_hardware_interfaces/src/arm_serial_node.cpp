#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <custom_interfaces/msg/arm_joint_target.hpp>
#include <custom_interfaces/msg/arm_named_target.hpp>
#include <custom_interfaces/msg/arm_pose_target.hpp>
#include <custom_interfaces/msg/gripper_command.hpp>

#include "robot_hardware_interfaces/send_command.hpp"

using moveit::planning_interface::MoveGroupInterface;
using custom_interfaces::msg::ArmJointTarget;
using custom_interfaces::msg::ArmNamedTarget;
using custom_interfaces::msg::ArmPoseTarget;
using custom_interfaces::msg::GripperCommand;

class ArmSerialNode : public rclcpp::Node
{
public:
	ArmSerialNode()
	: Node("arm_serial_node")
	{
		const auto mode_name = this->declare_parameter<std::string>("mode_name", "ARM");
		arm_group_name_ = this->declare_parameter<std::string>("arm_group", "arm");
		gripper_group_name_ = this->declare_parameter<std::string>("gripper_group", "gripper");
		use_moveit_mode_ = this->declare_parameter<bool>("use_moveit_mode", true);
		min_segment_time_ms_ = this->declare_parameter<int>("min_segment_time_ms", 20);
		gripper_motion_time_ms_ = this->declare_parameter<int>("gripper_motion_time_ms", 1000);
		direct_joint_time_ms_ = this->declare_parameter<int>("direct_joint_time_ms", 1000);
		arm_joint_names_ = this->declare_parameter<std::vector<std::string>>(
			"arm_joint_names",
			std::vector<std::string>{});

		sender_ = std::make_unique<SendCommand>(mode_name);
		if (!sender_->initialize()) {
			const auto &cfg = sender_->config();
			RCLCPP_ERROR(
				this->get_logger(),
				"Failed to open serial for mode=%s, device=%s, baud=%d",
				cfg.mode_name.c_str(),
				cfg.device.c_str(),
				cfg.baudrate);
		} else {
			const auto &cfg = sender_->config();
			RCLCPP_INFO(
				this->get_logger(),
				"Serial ready for mode=%s, device=%s, baud=%d",
				cfg.mode_name.c_str(),
				cfg.device.c_str(),
				cfg.baudrate);
		}

		if (use_moveit_mode_) {
			moveit_init_timer_ = this->create_wall_timer(
				std::chrono::milliseconds(50),
				std::bind(&ArmSerialNode::initializeMoveGroups, this));
			RCLCPP_INFO(this->get_logger(), "Control mode: MoveIt planning.");
		} else {
			RCLCPP_WARN(this->get_logger(), "Control mode: Direct joint command (MoveIt disabled).");
		}

		named_target_sub_ = this->create_subscription<ArmNamedTarget>(
			"/cmd/arm/named_target",
			10,
			std::bind(&ArmSerialNode::namedTargetCallback, this, std::placeholders::_1));

		joint_target_sub_ = this->create_subscription<ArmJointTarget>(
			"/cmd/arm/joint_target",
			10,
			std::bind(&ArmSerialNode::jointTargetCallback, this, std::placeholders::_1));

		pose_target_sub_ = this->create_subscription<ArmPoseTarget>(
			"/cmd/arm/pose_target",
			10,
			std::bind(&ArmSerialNode::poseTargetCallback, this, std::placeholders::_1));

		gripper_sub_ = this->create_subscription<GripperCommand>(
			"/cmd/arm/gripper",
			10,
			std::bind(&ArmSerialNode::gripperCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "arm_serial_node started.");
	}

private:
	static constexpr int kArmServoCount = 5;
	static constexpr int kGripperServoId = 5;

	static double clamp(double value, double min_value, double max_value)
	{
		return std::max(min_value, std::min(value, max_value));
	}

	static int angleDegreeToPwm(double degree)
	{
		const double bounded_degree = clamp(degree, 0.0, 270.0);
		const double pwm = 500.0 + (bounded_degree / 270.0) * 2000.0;
		return static_cast<int>(std::lround(pwm));
	}

	std::string formatServoCommand(int servo_id, int pwm, int duration_ms) const
	{
		std::ostringstream ss;
		ss << '#'
			 << std::setw(3) << std::setfill('0') << servo_id
			 << 'P'
			 << std::setw(4) << std::setfill('0') << pwm
			 << 'T'
			 << std::setw(4) << std::setfill('0') << std::max(duration_ms, min_segment_time_ms_)
			 << '!';
        RCLCPP_INFO(this->get_logger(), "Formatted servo command: %s", ss.str().c_str());
		return ss.str();
	}

	bool sendDirectJointCommand(const std::vector<double> &joints, int duration_ms)
	{
		if (!sender_ || !sender_->is_ready()) {
			RCLCPP_WARN(this->get_logger(), "Serial sender not ready, direct joint command skipped.");
			return false;
		}

		if (joints.size() < kArmServoCount) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Direct joint command requires at least %d joints, got %zu",
				kArmServoCount,
				joints.size());
			return false;
		}

		std::string payload;
		for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
			const double degree = joints[servo_id] * 180.0 / M_PI;
			const int pwm = angleDegreeToPwm(degree);
			payload += formatServoCommand(servo_id, pwm, duration_ms);
		}

		if (!sender_->send(payload)) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send direct joint payload: %s", payload.c_str());
			return false;
		}

		return true;
	}

	static int durationForPoint(
		const trajectory_msgs::msg::JointTrajectoryPoint &current,
		const trajectory_msgs::msg::JointTrajectoryPoint *previous,
		int min_duration_ms)
	{
		const auto current_seconds =
			static_cast<double>(current.time_from_start.sec) +
			static_cast<double>(current.time_from_start.nanosec) * 1e-9;

		double segment_seconds = current_seconds;
		if (previous != nullptr) {
			const auto previous_seconds =
				static_cast<double>(previous->time_from_start.sec) +
				static_cast<double>(previous->time_from_start.nanosec) * 1e-9;
			segment_seconds = current_seconds - previous_seconds;
		}

		const auto ms = static_cast<int>(std::lround(segment_seconds * 1000.0));
		return std::max(ms, min_duration_ms);
	}

	std::vector<int> resolveArmJointIndices(const trajectory_msgs::msg::JointTrajectory &trajectory) const
	{
		std::vector<int> indices(kArmServoCount, -1);
		if (!arm_joint_names_.empty()) {
			std::unordered_map<std::string, int> name_to_index;
			for (size_t idx = 0; idx < trajectory.joint_names.size(); ++idx) {
				name_to_index[trajectory.joint_names[idx]] = static_cast<int>(idx);
			}
			for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
				if (servo_id < static_cast<int>(arm_joint_names_.size())) {
					const auto it = name_to_index.find(arm_joint_names_[servo_id]);
					if (it != name_to_index.end()) {
						indices[servo_id] = it->second;
					}
				}
			}
			return indices;
		}

		const int max_joint = std::min(kArmServoCount, static_cast<int>(trajectory.joint_names.size()));
		for (int servo_id = 0; servo_id < max_joint; ++servo_id) {
			indices[servo_id] = servo_id;
		}
		return indices;
	}

	void initializeMoveGroups()
	{
		if (!use_moveit_mode_) {
			if (moveit_init_timer_) {
				moveit_init_timer_->cancel();
			}
			return;
		}

		if (moveit_ready_) {
			if (moveit_init_timer_) {
				moveit_init_timer_->cancel();
			}
			return;
		}

		try {
			auto node_shared = this->shared_from_this();
			arm_ = std::make_shared<MoveGroupInterface>(node_shared, arm_group_name_);
			arm_->setMaxVelocityScalingFactor(1.0);
			arm_->setMaxAccelerationScalingFactor(1.0);
			gripper_ = std::make_shared<MoveGroupInterface>(node_shared, gripper_group_name_);
			moveit_ready_ = true;
			if (moveit_init_timer_) {
				moveit_init_timer_->cancel();
			}
			RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized.");
		} catch (const std::exception &e) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				2000,
				"MoveIt interface init retry: %s",
				e.what());
		}
	}

	bool ensureMoveItReady() const
	{
		if (!moveit_ready_ || !arm_ || !gripper_) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				2000,
				"MoveIt interfaces are not ready yet, command skipped.");
			return false;
		}
		return true;
	}

	bool sendArmTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory)
	{
		if (!sender_ || !sender_->is_ready()) {
			RCLCPP_WARN(this->get_logger(), "Serial sender not ready, arm command skipped.");
			return false;
		}

		if (trajectory.points.empty()) {
			RCLCPP_WARN(this->get_logger(), "Trajectory has no points.");
			return false;
		}

		const auto joint_indices = resolveArmJointIndices(trajectory);
		const auto *previous = static_cast<const trajectory_msgs::msg::JointTrajectoryPoint *>(nullptr);

		for (const auto &point : trajectory.points) {
			const int duration_ms = durationForPoint(point, previous, min_segment_time_ms_);
			std::string payload;

			for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
				const int joint_index = joint_indices[servo_id];
				if (joint_index < 0 || joint_index >= static_cast<int>(point.positions.size())) {
					continue;
				}

				const double degree = point.positions[joint_index] * 180.0 / M_PI;
				const int pwm = angleDegreeToPwm(degree);
				payload += formatServoCommand(servo_id, pwm, duration_ms);
			}

			if (payload.empty()) {
				RCLCPP_ERROR(this->get_logger(), "No valid servo command generated for trajectory point.");
				return false;
			}

			if (!sender_->send(payload)) {
				RCLCPP_ERROR(this->get_logger(), "Failed to send arm payload: %s", payload.c_str());
				return false;
			}

			previous = &point;
		}

		return true;
	}

	bool sendGripperTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory)
	{
		if (!sender_ || !sender_->is_ready()) {
			RCLCPP_WARN(this->get_logger(), "Serial sender not ready, gripper command skipped.");
			return false;
		}
		if (trajectory.points.empty() || trajectory.points.back().positions.empty()) {
			RCLCPP_ERROR(this->get_logger(), "Invalid gripper trajectory.");
			return false;
		}

		const auto final_position_rad = trajectory.points.back().positions.front();
		const double degree = final_position_rad * 180.0 / M_PI;
		const int pwm = angleDegreeToPwm(degree);
		const auto payload = formatServoCommand(kGripperServoId, pwm, gripper_motion_time_ms_);

		if (!sender_->send(payload)) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send gripper payload: %s", payload.c_str());
			return false;
		}

		return true;
	}

	bool planAndSendArm()
	{
		MoveGroupInterface::Plan plan;
		const bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (!success) {
			RCLCPP_ERROR(this->get_logger(), "Arm planning failed.");
			return false;
		}
		return sendArmTrajectory(plan.trajectory.joint_trajectory);
	}

	bool computeCartesianAndSend(const geometry_msgs::msg::Pose &pose)
	{
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.push_back(pose);

		moveit_msgs::msg::RobotTrajectory trajectory;
		const double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
		if (fraction < 0.999) {
			RCLCPP_ERROR(this->get_logger(), "Cartesian planning fraction too low: %.3f", fraction);
			return false;
		}
		return sendArmTrajectory(trajectory.joint_trajectory);
	}

	bool planAndSendGripper(const std::string &target)
	{
		gripper_->setStartStateToCurrentState();
		gripper_->setNamedTarget(target);

		MoveGroupInterface::Plan plan;
		const bool success = (gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (!success) {
			RCLCPP_ERROR(this->get_logger(), "Gripper planning failed for target: %s", target.c_str());
			return false;
		}

		return sendGripperTrajectory(plan.trajectory.joint_trajectory);
	}

	void namedTargetCallback(const ArmNamedTarget::SharedPtr msg)
	{
		if (!use_moveit_mode_) {
			RCLCPP_WARN(this->get_logger(), "named_target is disabled in direct mode.");
			return;
		}
		if (!ensureMoveItReady()) {
			return;
		}
		arm_->setStartStateToCurrentState();
		arm_->setNamedTarget(msg->target_name);
		planAndSendArm();
	}

	void jointTargetCallback(const ArmJointTarget::SharedPtr msg)
	{
		if (!use_moveit_mode_) {
			sendDirectJointCommand(msg->joints, direct_joint_time_ms_);
			return;
		}

		if (!ensureMoveItReady()) {
			return;
		}
		arm_->setStartStateToCurrentState();
		arm_->setJointValueTarget(msg->joints);
		planAndSendArm();
	}

	void poseTargetCallback(const ArmPoseTarget::SharedPtr msg)
	{
		if (!use_moveit_mode_) {
			RCLCPP_WARN(this->get_logger(), "pose_target is disabled in direct mode.");
			return;
		}
		if (!ensureMoveItReady()) {
			return;
		}
		geometry_msgs::msg::Pose pose;
		tf2::Quaternion q;
		q.setRPY(msg->roll, msg->pitch, msg->yaw);
		q.normalize();

		pose.position.x = msg->x;
		pose.position.y = msg->y;
		pose.position.z = msg->z;
		pose.orientation.x = q.x();
		pose.orientation.y = q.y();
		pose.orientation.z = q.z();
		pose.orientation.w = q.w();

		arm_->setStartStateToCurrentState();
		if (msg->cartesian_path) {
			computeCartesianAndSend(pose);
			return;
		}

		arm_->setPoseTarget(pose);
		planAndSendArm();
	}

	void gripperCallback(const GripperCommand::SharedPtr msg)
	{
		if (!use_moveit_mode_) {
			RCLCPP_WARN(this->get_logger(), "gripper command via MoveIt is disabled in direct mode.");
			return;
		}
		if (!ensureMoveItReady()) {
			return;
		}
		if (msg->open) {
			planAndSendGripper("open");
		} else {
			planAndSendGripper("close");
		}
	}

	std::string arm_group_name_;
	std::string gripper_group_name_;
	bool use_moveit_mode_;
	int min_segment_time_ms_;
	int gripper_motion_time_ms_;
	int direct_joint_time_ms_;
	std::vector<std::string> arm_joint_names_;

	std::unique_ptr<SendCommand> sender_;
	bool moveit_ready_ = false;
	rclcpp::TimerBase::SharedPtr moveit_init_timer_;
	std::shared_ptr<MoveGroupInterface> arm_;
	std::shared_ptr<MoveGroupInterface> gripper_;

	rclcpp::Subscription<ArmNamedTarget>::SharedPtr named_target_sub_;
	rclcpp::Subscription<ArmJointTarget>::SharedPtr joint_target_sub_;
	rclcpp::Subscription<ArmPoseTarget>::SharedPtr pose_target_sub_;
	rclcpp::Subscription<GripperCommand>::SharedPtr gripper_sub_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArmSerialNode>());
	rclcpp::shutdown();
	return 0;
}
