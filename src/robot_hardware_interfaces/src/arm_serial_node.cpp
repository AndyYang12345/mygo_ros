#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
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
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_hardware_interfaces/send_command.hpp"

using moveit::planning_interface::MoveGroupInterface;
using custom_interfaces::msg::ArmJointTarget;
using custom_interfaces::msg::ArmNamedTarget;
using custom_interfaces::msg::ArmPoseTarget;
using custom_interfaces::msg::GripperCommand;
using example_interfaces::msg::Float64MultiArray;
using std_msgs::msg::String;

class ArmSerialNode : public rclcpp::Node
{
public:
	ArmSerialNode()
	: Node("arm_serial_node")
	{
		const auto mode_name = this->declare_parameter<std::string>("mode_name", "ARM");
		arm_group_name_ = this->declare_parameter<std::string>("arm_group", "arm");
		gripper_group_name_ = this->declare_parameter<std::string>("gripper_group", "gripper");
		use_moveit_mode_ = false;
		min_segment_time_ms_ = this->declare_parameter<int>("min_segment_time_ms", 20);
		gripper_motion_time_ms_ = this->declare_parameter<int>("gripper_motion_time_ms", 1000);
		direct_joint_time_ms_ = this->declare_parameter<int>("direct_joint_time_ms", 1000);
		gripper_open_pwm_ = static_cast<int>(std::lround(clamp(
			this->declare_parameter<double>("gripper_open_pwm", 1400.0),
			500.0,
			2500.0)));
		gripper_close_pwm_ = static_cast<int>(std::lround(clamp(
			this->declare_parameter<double>("gripper_close_pwm", 1650.0),
			500.0,
			2500.0)));
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

		RCLCPP_INFO(this->get_logger(), "Control mode: Direct joint command (MoveIt planned upstream).");

		joint_target_sub_ = this->create_subscription<ArmJointTarget>(
			"/cmd/arm/joint_target",
			10,
			std::bind(&ArmSerialNode::jointTargetCallback, this, std::placeholders::_1));

		joint_command_sub_ = this->create_subscription<Float64MultiArray>(
			"/cmd/arm/joint_command",
			10,
			std::bind(&ArmSerialNode::directJointCommandCallback, this, std::placeholders::_1));

		direct_pwm_sub_ = this->create_subscription<Float64MultiArray>(
			"/cmd/arm/direct_pwm_command",
			10,
			std::bind(&ArmSerialNode::directPwmCallback, this, std::placeholders::_1));

		gripper_sub_ = this->create_subscription<GripperCommand>(
			"/cmd/arm/gripper",
			10,
			std::bind(&ArmSerialNode::gripperCallback, this, std::placeholders::_1));

		query_current_sub_ = this->create_subscription<String>(
			"/cmd/arm/query_current",
			10,
			std::bind(&ArmSerialNode::queryCurrentCallback, this, std::placeholders::_1));

		current_pwm_pub_ = this->create_publisher<Float64MultiArray>("/arm/current_pwm", 10);
		current_joint_rad_pub_ = this->create_publisher<Float64MultiArray>("/arm/current_joint_radians", 10);

		RCLCPP_INFO(this->get_logger(), "arm_serial_node started.");
	}

private:
	static constexpr int kArmServoCount = 5;
	static constexpr int kGripperServoId = 5;
	static constexpr int kTotalServoCount = 6;
	static constexpr size_t kDurationIndex = 6;

	static double clamp(double value, double min_value, double max_value)
	{
		return std::max(min_value, std::min(value, max_value));
	}

	static int angleDegreeToPwm(double degree)
	{
		// Centered mapping: 0 deg -> 1500, -135..135 deg -> 500..2500.
		const double bounded_degree = clamp(degree, -135.0, 135.0);
		const double pwm = 1500.0 + (bounded_degree / 135.0) * 1000.0;
		return static_cast<int>(std::lround(pwm));
	}

	static double pwmToAngleDegree(double pwm)
	{
		return (pwm - 1500.0) / 1000.0 * 135.0;
	}

	std::optional<std::array<int, kTotalServoCount>> parsePwmFrame(const std::string &frame) const
	{
		std::array<int, kTotalServoCount> pwm{};
		size_t count = 0;
		size_t i = 0;
		while (i < frame.size() && count < pwm.size())
		{
			if (frame[i] != 'P')
			{
				++i;
				continue;
			}

			++i;
			std::string digits;
			while (i < frame.size() && std::isdigit(static_cast<unsigned char>(frame[i])))
			{
				digits.push_back(frame[i]);
				++i;
			}

			if (!digits.empty())
			{
				pwm[count++] = std::stoi(digits);
			}
		}

		if (count != pwm.size())
		{
			return std::nullopt;
		}

		return pwm;
	}

	void queryCurrentCallback(const String::SharedPtr msg)
	{
		if (!msg || msg->data != "kg")
		{
			return;
		}

		if (!sender_ || !sender_->is_ready())
		{
			RCLCPP_WARN(this->get_logger(), "Serial sender not ready, kg query skipped.");
			return;
		}

		std::string response;
		if (!sender_->request("kg", response, 500))
		{
			RCLCPP_WARN(this->get_logger(), "kg query timeout or read failure.");
			return;
		}

		auto parsed = parsePwmFrame(response);
		if (!parsed.has_value())
		{
			RCLCPP_WARN(this->get_logger(), "Invalid kg response frame: %s", response.c_str());
			return;
		}

		Float64MultiArray pwm_msg;
		pwm_msg.data.reserve(kTotalServoCount);
		for (const int value : parsed.value())
		{
			pwm_msg.data.push_back(static_cast<double>(value));
		}
		current_pwm_pub_->publish(pwm_msg);

		Float64MultiArray joint_msg;
		joint_msg.data.reserve(kArmServoCount);
		for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id)
		{
			const double degree = pwmToAngleDegree(static_cast<double>(parsed.value()[servo_id]));
			joint_msg.data.push_back(degree * M_PI / 180.0);
		}
		current_joint_rad_pub_->publish(joint_msg);

		RCLCPP_INFO(this->get_logger(), "kg current state updated: frame=%s", response.c_str());
	}

	std::string formatPwmField(int pwm, int duration_ms) const
	{
		const int bounded_pwm = static_cast<int>(clamp(static_cast<double>(pwm), 500.0, 2500.0));
		const int bounded_duration = std::max(duration_ms, min_segment_time_ms_);
		std::ostringstream ss;
		ss << 'P' << std::setw(4) << std::setfill('0') << bounded_pwm
		   << 'T' << std::setw(4) << std::setfill('0') << bounded_duration;
		return ss.str();
	}

	std::string formatArmFrame(const std::array<int, kTotalServoCount> &pwms, int duration_ms) const
	{
		std::string frame = "{";
		for (const auto pwm : pwms) {
			frame += formatPwmField(pwm, duration_ms);
		}
		frame += "}";
		return frame;
	}

	std::string formatGripperCommand(int pwm, int duration_ms) const
	{
		const int bounded_pwm = static_cast<int>(clamp(static_cast<double>(pwm), 500.0, 2500.0));
		const int bounded_duration = std::max(duration_ms, min_segment_time_ms_);
		std::ostringstream ss;
		ss << "#005P"
		   << std::setw(4) << std::setfill('0') << bounded_pwm
		   << 'T'
		   << std::setw(4) << std::setfill('0') << bounded_duration;
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

		std::array<int, kTotalServoCount> frame_pwms = {
			last_arm_pwms_[0],
			last_arm_pwms_[1],
			last_arm_pwms_[2],
			last_arm_pwms_[3],
			last_arm_pwms_[4],
			last_gripper_pwm_};

		for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
			const double degree = joints[servo_id] * 180.0 / M_PI;
			const int pwm = angleDegreeToPwm(degree);
			frame_pwms[static_cast<size_t>(servo_id)] = pwm;
			last_arm_pwms_[static_cast<size_t>(servo_id)] = pwm;
		}

		const std::string payload = formatArmFrame(frame_pwms, duration_ms);
        RCLCPP_INFO(this->get_logger(), "Formatted arm frame: %s", payload.c_str());

		if (!sender_->send(payload)) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send direct joint payload: %s", payload.c_str());
			return false;
		}

		last_direct_joints_.assign(joints.begin(), joints.begin() + kArmServoCount);
		last_direct_joints_valid_ = true;

		int selected_servo_id = 0;
		double max_delta = 0.0;
		for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
			const double delta = std::abs(joints[servo_id] - last_reported_joints_[servo_id]);
			if (delta > max_delta) {
				max_delta = delta;
				selected_servo_id = servo_id;
			}
			last_reported_joints_[servo_id] = joints[servo_id];
		}

		RCLCPP_INFO(
			this->get_logger(),
			"Direct mode selected servo id: %d, sent_count=%d",
			selected_servo_id,
			kArmServoCount);

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
			std::array<int, kTotalServoCount> frame_pwms = {
				last_arm_pwms_[0],
				last_arm_pwms_[1],
				last_arm_pwms_[2],
				last_arm_pwms_[3],
				last_arm_pwms_[4],
				last_gripper_pwm_};

			for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
				const int joint_index = joint_indices[servo_id];
				if (joint_index < 0 || joint_index >= static_cast<int>(point.positions.size())) {
					RCLCPP_ERROR(this->get_logger(), "Invalid joint index for servo_id=%d in trajectory point.", servo_id);
					return false;
				}

				const double degree = point.positions[joint_index] * 180.0 / M_PI;
				const int pwm = angleDegreeToPwm(degree);
				frame_pwms[static_cast<size_t>(servo_id)] = pwm;
				last_arm_pwms_[static_cast<size_t>(servo_id)] = pwm;
			}

			const std::string payload = formatArmFrame(frame_pwms, duration_ms);
            RCLCPP_INFO(this->get_logger(), "Formatted arm frame: %s", payload.c_str());

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
		last_gripper_pwm_ = pwm;
		const auto payload = formatGripperCommand(last_gripper_pwm_, gripper_motion_time_ms_);
        RCLCPP_INFO(this->get_logger(), "Formatted gripper frame: %s", payload.c_str());

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

	void directJointCommandCallback(const Float64MultiArray::SharedPtr msg)
	{
		if (!msg) {
			return;
		}

		if (msg->data.size() < kArmServoCount) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Direct joint command requires at least %d joints, got %zu",
				kArmServoCount,
				msg->data.size());
			return;
		}

		sendDirectJointCommand(msg->data, direct_joint_time_ms_);
	}

	void directPwmCallback(const Float64MultiArray::SharedPtr msg)
	{
		if (!msg) {
			return;
		}

		if (!sender_ || !sender_->is_ready()) {
			RCLCPP_WARN(this->get_logger(), "Serial sender not ready, direct pwm command skipped.");
			return;
		}

		if (msg->data.size() < kArmServoCount) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Direct PWM command requires at least %d values, got %zu",
				kArmServoCount,
				msg->data.size());
			return;
		}

		int duration_ms = direct_joint_time_ms_;
		if (msg->data.size() > kDurationIndex) {
			const double duration_value = msg->data[kDurationIndex];
			if (!std::isfinite(duration_value)) {
				RCLCPP_ERROR(this->get_logger(), "Direct PWM command contains non-finite duration");
				return;
			}
			duration_ms = std::max(
				min_segment_time_ms_,
				static_cast<int>(std::lround(duration_value)));
		}

		std::array<int, kTotalServoCount> frame_pwms = {
			last_arm_pwms_[0],
			last_arm_pwms_[1],
			last_arm_pwms_[2],
			last_arm_pwms_[3],
			last_arm_pwms_[4],
			last_gripper_pwm_};

		for (int servo_id = 0; servo_id < kArmServoCount; ++servo_id) {
			const double value = msg->data[static_cast<size_t>(servo_id)];
			if (!std::isfinite(value)) {
				RCLCPP_ERROR(this->get_logger(), "Direct PWM command contains non-finite value at servo_id=%d", servo_id);
				return;
			}

			const int pwm = static_cast<int>(std::lround(clamp(value, 500.0, 2500.0)));
			frame_pwms[static_cast<size_t>(servo_id)] = pwm;
			last_arm_pwms_[static_cast<size_t>(servo_id)] = pwm;

			const double joint_rad = pwmToAngleDegree(static_cast<double>(pwm)) * M_PI / 180.0;
			if (servo_id < static_cast<int>(last_reported_joints_.size())) {
				last_reported_joints_[static_cast<size_t>(servo_id)] = joint_rad;
			}
		}

		if (msg->data.size() > static_cast<size_t>(kGripperServoId)) {
			const double gripper_value = msg->data[static_cast<size_t>(kGripperServoId)];
			if (!std::isfinite(gripper_value)) {
				RCLCPP_ERROR(this->get_logger(), "Direct PWM command contains non-finite gripper value");
				return;
			}

			last_gripper_pwm_ = static_cast<int>(std::lround(clamp(gripper_value, 500.0, 2500.0)));
			frame_pwms[static_cast<size_t>(kGripperServoId)] = last_gripper_pwm_;
		}

		const auto payload = formatArmFrame(frame_pwms, duration_ms);
		RCLCPP_INFO(this->get_logger(), "Formatted direct pwm frame: %s", payload.c_str());
		if (!sender_->send(payload)) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send direct pwm payload: %s", payload.c_str());
		}
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
		if (!sender_ || !sender_->is_ready()) {
			RCLCPP_WARN(this->get_logger(), "Serial sender not ready, gripper command skipped.");
			return;
		}

		last_gripper_pwm_ = msg->open ? gripper_open_pwm_ : gripper_close_pwm_;
		const auto payload = formatGripperCommand(last_gripper_pwm_, gripper_motion_time_ms_);
        RCLCPP_INFO(this->get_logger(), "Formatted gripper frame: %s", payload.c_str());
		if (!sender_->send(payload)) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send direct gripper payload: %s", payload.c_str());
		}
	}

	std::string arm_group_name_;
	std::string gripper_group_name_;
	bool use_moveit_mode_;
	int min_segment_time_ms_;
	int gripper_motion_time_ms_;
	int direct_joint_time_ms_;
	int gripper_open_pwm_;
	int gripper_close_pwm_;
	std::vector<std::string> arm_joint_names_;

	std::unique_ptr<SendCommand> sender_;
	std::vector<double> last_direct_joints_;
	std::vector<double> last_reported_joints_ = std::vector<double>(kArmServoCount, 0.0);
	std::array<int, kArmServoCount> last_arm_pwms_ = {1500, 1500, 1500, 1500, 1500};
	int last_gripper_pwm_ = 1500;
	bool last_direct_joints_valid_ = false;
	bool moveit_ready_ = false;
	rclcpp::TimerBase::SharedPtr moveit_init_timer_;
	std::shared_ptr<MoveGroupInterface> arm_;
	std::shared_ptr<MoveGroupInterface> gripper_;

	rclcpp::Subscription<ArmJointTarget>::SharedPtr joint_target_sub_;
	rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_command_sub_;
	rclcpp::Subscription<Float64MultiArray>::SharedPtr direct_pwm_sub_;
	rclcpp::Subscription<GripperCommand>::SharedPtr gripper_sub_;
	rclcpp::Subscription<String>::SharedPtr query_current_sub_;
	rclcpp::Publisher<Float64MultiArray>::SharedPtr current_pwm_pub_;
	rclcpp::Publisher<Float64MultiArray>::SharedPtr current_joint_rad_pub_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArmSerialNode>());
	rclcpp::shutdown();
	return 0;
}
