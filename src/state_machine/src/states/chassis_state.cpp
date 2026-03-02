#include "state_machine/chassis_state.hpp"
#include "state_machine/robot_state_machine_node.hpp"

std::string ChassisState::getName() const
{
	return "CHASSIS";
}

uint8_t ChassisState::getStateEnum() const
{
	return 2;
}

void ChassisState::onEnter(RobotStateMachineNode *context)
{
	RCLCPP_INFO(context->get_logger(), "Entered CHASSIS state");
}

void ChassisState::onExit(RobotStateMachineNode *context)
{
	RCLCPP_INFO(context->get_logger(), "Exiting CHASSIS state");
}

void ChassisState::handleButton(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ChassisState::handleJoystick(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ChassisState::handleTrigger(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ChassisState::update(RobotStateMachineNode* context) {
    // 1. 获取摇杆数据（来自意图层）
    const auto& joystick = context->getLeftJoystick();
    
    // 2. 检查数据有效性
    if (!joystick.is_active(context)) {
        // 超时，发送停止命令
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        return;
    }
    
    // 3. 应用死区
    double x = (std::abs(joystick.x) < context->getJoystickDeadzone()) ? 0.0 : joystick.x;
    double y = (std::abs(joystick.y) < context->getJoystickDeadzone()) ? 0.0 : joystick.y;
    
    // 4. 生成Twist消息（这就是你的核心工作！）
    auto twist = geometry_msgs::msg::Twist();
    
    // 左摇杆Y轴 → 前进/后退速度 (m/s)
    twist.linear.x = y * context->getChassisMaxLinearSpeed();
    
    // 左摇杆X轴 → 旋转速度 (rad/s)  
    twist.angular.z = -x * context->getChassisMaxAngularSpeed();
    
    // 5. 发布Twist命令
    context->getChassisCmdPub()->publish(twist);
    
    // 可选：打印调试信息
    RCLCPP_DEBUG(context->get_logger(), 
                "Chassis cmd: linear=%.2f m/s, angular=%.2f rad/s",
                twist.linear.x, twist.angular.z);
}
