#include "state_machine/chassis_state.hpp"
#include "state_machine/robot_state_machine_node.hpp"
// @todo #include "custom_interfaces/msg/arm_command.hpp"  // 假设你需要自定义的机械臂命令

// ==================== 初始化函数指针表 ====================

void ChassisState::initHandlerTables()
{
    // Home模式
    handler_tables_[0] = {
        .joystick = &ChassisState::handleHomeJoystick,
        .button = &ChassisState::handleHomeButton,
        .trigger = &ChassisState::handleHomeTrigger,
        .update = &ChassisState::updateHome
    };
    
    // NormalDetection模式
    handler_tables_[1] = {
        .joystick = &ChassisState::handleNormalDetectionJoystick,
        .button = &ChassisState::handleNormalDetectionButton,
        .trigger = &ChassisState::handleNormalDetectionTrigger,
        .update = &ChassisState::updateNormalDetection
    };
    
    // CylinderSubmission模式
    handler_tables_[2] = {
        .joystick = &ChassisState::handleCylinderSubmissionJoystick,
        .button = &ChassisState::handleCylinderSubmissionButton,
        .trigger = &ChassisState::handleCylinderSubmissionTrigger,
        .update = &ChassisState::updateCylinderSubmission
    };
    
    // CubeSubmission模式
    handler_tables_[3] = {
        .joystick = &ChassisState::handleCubeSubmissionJoystick,
        .button = &ChassisState::handleCubeSubmissionButton,
        .trigger = &ChassisState::handleCubeSubmissionTrigger,
        .update = &ChassisState::updateCubeSubmission
    };
    
    // CylinderCollection模式
    handler_tables_[4] = {
        .joystick = &ChassisState::handleCylinderCollectionJoystick,
        .button = &ChassisState::handleCylinderCollectionButton,
        .trigger = &ChassisState::handleCylinderCollectionTrigger,
        .update = &ChassisState::updateCylinderCollection
    };
    
    // CubeCollection模式
    handler_tables_[5] = {
        .joystick = &ChassisState::handleCubeCollectionJoystick,
        .button = &ChassisState::handleCubeCollectionButton,
        .trigger = &ChassisState::handleCubeCollectionTrigger,
        .update = &ChassisState::updateCubeCollection
    };
    
    // UnderBridge模式
    handler_tables_[6] = {
        .joystick = &ChassisState::handleUnderBridgeJoystick,
        .button = &ChassisState::handleUnderBridgeButton,
        .trigger = &ChassisState::handleUnderBridgeTrigger,
        .update = &ChassisState::updateUnderBridge
    };
}

// ==================== 构造函数 ====================

ChassisState::ChassisState()
{
    initHandlerTables();
}

// ==================== 基本接口实现 ====================

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
    speed_multiplier_ = 1.0;
    arm_mode_ = ArmMode::Home;  // 默认进入Home模式
}

void ChassisState::onExit(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "Exiting CHASSIS state");
    // 退出时停止所有运动
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
    context->getArmCmdPub()->publish(geometry_msgs::msg::Twist());
}

void ChassisState::setArmMode(ArmMode mode)
{
    if (arm_mode_ != mode) {
        RCLCPP_INFO(rclcpp::get_logger("ChassisState"), 
                   "Switching arm mode from %d to %d", 
                   static_cast<int>(arm_mode_), 
                   static_cast<int>(mode));
        arm_mode_ = mode;
    }
}

// ==================== 统一的事件分发接口 ====================

void ChassisState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    size_t index = static_cast<size_t>(arm_mode_);
    auto handler = handler_tables_[index].joystick;
    (this->*handler)(context, msg);
}

void ChassisState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    size_t index = static_cast<size_t>(arm_mode_);
    auto handler = handler_tables_[index].button;
    (this->*handler)(context, msg);
}

void ChassisState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    size_t index = static_cast<size_t>(arm_mode_);
    auto handler = handler_tables_[index].trigger;
    (this->*handler)(context, msg);
}

void ChassisState::update(RobotStateMachineNode *context)
{
    size_t index = static_cast<size_t>(arm_mode_);
    auto handler = handler_tables_[index].update;
    (this->*handler)(context);
}

// ==================== 辅助函数 ====================

double ChassisState::applyDeadzone(double value, double deadzone) const
{
    return std::abs(value) < deadzone ? 0.0 : value;
}

void ChassisState::publishChassisCommand(
    RobotStateMachineNode *context, 
    double x, double y, 
    double speed_multiplier)
{
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = y * context->getChassisMaxLinearSpeed() * speed_multiplier;
    twist.angular.z = -x * context->getChassisMaxAngularSpeed() * speed_multiplier;
    context->getChassisCmdPub()->publish(twist);
}

// ==================== Home模式实现 ====================

void ChassisState::handleHomeJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    // Home模式下，摇杆控制底盘移动
    if (msg->joystick_id == 0) {
        const auto& joystick = context->getLeftJoystick();
        if (!joystick.is_active(context)) {
            context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
            return;
        }
        
        double x = applyDeadzone(joystick.x, context->getJoystickDeadzone());
        double y = applyDeadzone(joystick.y, context->getJoystickDeadzone());
        
        publishChassisCommand(context, x, y, speed_multiplier_);
    }
}

void ChassisState::handleHomeButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    // 按下X键切换到NormalDetection模式
    if (msg->button_id == 2 && msg->event_type == 0) {
        RCLCPP_INFO(context->get_logger(), "Home: X pressed -> switching to NormalDetection");
        setArmMode(ArmMode::NormalDetection);
    }
}

void ChassisState::handleHomeTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    // LT/RT控制速度倍率
    if (msg->trigger_id == 0) {  // LT
        speed_multiplier_ = 1.0 - msg->value * 0.5;
    } else if (msg->trigger_id == 1) {  // RT
        speed_multiplier_ = 1.0 + msg->value * 1.0;
    }
}

void ChassisState::updateHome(RobotStateMachineNode *context)
{
    // Home模式下不需要额外的更新逻辑
}

// ==================== NormalDetection模式实现 ====================

void ChassisState::handleNormalDetectionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 0) {  // 左摇杆控制底盘
        const auto& joystick = context->getLeftJoystick();
        if (!joystick.is_active(context)) {
            context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
            return;
        }
        
        double x = applyDeadzone(joystick.x, context->getJoystickDeadzone());
        double y = applyDeadzone(joystick.y, context->getJoystickDeadzone());
        
        publishChassisCommand(context, x, y, speed_multiplier_);
    }
    else if (msg->joystick_id == 1) {  // 右摇杆控制机械臂yaw
        double x = applyDeadzone(msg->x, context->getJoystickDeadzone());
        
        // 控制机械臂yaw轴旋转
        auto arm_twist = geometry_msgs::msg::Twist();
        arm_twist.angular.z = x * 0.5;  // yaw轴速度
        
        // 假设你有机械臂命令发布者
        // context->getArmCmdPub()->publish(arm_twist);
        
        RCLCPP_DEBUG(context->get_logger(), 
                    "NormalDetection: Moving arm yaw: %.2f", x);
    }
}

void ChassisState::handleNormalDetectionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0) return;
    
    switch(msg->button_id) {
        case 0:  // A键：切换到CubeCollection
            setArmMode(ArmMode::CubeCollection);
            RCLCPP_INFO(context->get_logger(), "NormalDetection: Switching to CubeCollection");
            break;
        case 1:  // B键：切换到CylinderCollection
            setArmMode(ArmMode::CylinderCollection);
            RCLCPP_INFO(context->get_logger(), "NormalDetection: Switching to CylinderCollection");
            break;
        case 2:  // X键：返回Home
            setArmMode(ArmMode::Home);
            RCLCPP_INFO(context->get_logger(), "NormalDetection: Returning to Home");
            break;
    }
}

void ChassisState::handleNormalDetectionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    // LT/RT控制速度倍率（与Home模式相同）
    if (msg->trigger_id == 0) {
        speed_multiplier_ = 1.0 - msg->value * 0.5;
    } else if (msg->trigger_id == 1) {
        speed_multiplier_ = 1.0 + msg->value * 1.0;
    }
}

void ChassisState::updateNormalDetection(RobotStateMachineNode *context)
{
    // 可以添加自动检测逻辑
}

// ==================== CubeCollection模式实现 ====================

void ChassisState::handleCubeCollectionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 0) {  // 左摇杆控制底盘（减速）
        const auto& joystick = context->getLeftJoystick();
        if (!joystick.is_active(context)) {
            context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
            return;
        }
        
        double x = applyDeadzone(joystick.x, context->getJoystickDeadzone());
        double y = applyDeadzone(joystick.y, context->getJoystickDeadzone());
        
        // 抓取模式下速度减慢
        publishChassisCommand(context, x, y, speed_multiplier_ * 0.5);
    }
    else if (msg->joystick_id == 1) {  // 右摇杆微调夹爪
        double y = applyDeadzone(msg->y, context->getJoystickDeadzone());
        
        if (y > 0) {
            RCLCPP_INFO(context->get_logger(), "CubeCollection: Adjusting gripper closer");
            // 发送夹爪微调命令
        } else if (y < 0) {
            RCLCPP_INFO(context->get_logger(), "CubeCollection: Adjusting gripper opener");
        }
    }
}

void ChassisState::handleCubeCollectionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0) return;
    
    switch(msg->button_id) {
        case 0:  // A键：执行自动抓取
            RCLCPP_INFO(context->get_logger(), "CubeCollection: Executing auto grab");
            // 触发自动抓取序列
            break;
        case 1:  // B键：取消，返回NormalDetection
            setArmMode(ArmMode::NormalDetection);
            RCLCPP_INFO(context->get_logger(), "CubeCollection: Cancelled, returning to NormalDetection");
            break;
    }
}

void ChassisState::handleCubeCollectionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    // 扳机控制夹爪开合
    if (msg->trigger_id == 0) {  // LT：张开
        double position = msg->value;  // 0-1
        // 发送夹爪位置命令
        RCLCPP_DEBUG(context->get_logger(), "CubeCollection: Gripper open: %.2f", position);
    } else if (msg->trigger_id == 1) {  // RT：闭合
        double position = 1.0 - msg->value;
        RCLCPP_DEBUG(context->get_logger(), "CubeCollection: Gripper close: %.2f", position);
    }
}

void ChassisState::updateCubeCollection(RobotStateMachineNode *context)
{
    // 自动抓取逻辑的状态机
    static enum { SEARCHING, APPROACHING, GRASPING, RETREATING } auto_state = SEARCHING;
    
    // 实现自动抓取的状态转换逻辑...
}

// ==================== 其他模式的简化实现 ====================

// CylinderCollection模式（与CubeCollection类似，但针对能量单元）
void ChassisState::handleCylinderCollectionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    // 可以复用CubeCollection的逻辑，或者根据能量单元的特性调整
    handleCubeCollectionJoystick(context, msg);
}

void ChassisState::handleCylinderCollectionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    handleCubeCollectionButton(context, msg);
}

void ChassisState::handleCylinderCollectionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    handleCubeCollectionTrigger(context, msg);
}

void ChassisState::updateCylinderCollection(RobotStateMachineNode *context)
{
    // 能量单元特定的自动抓取逻辑
}

// CylinderSubmission模式
void ChassisState::handleCylinderSubmissionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    // 自动提交模式下，摇杆可能被禁用或只有部分功能
    (void)context;
    (void)msg;
}

void ChassisState::handleCylinderSubmissionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->button_id == 1 && msg->event_type == 0) {  // B键取消
        setArmMode(ArmMode::NormalDetection);
    }
}

void ChassisState::handleCylinderSubmissionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    (void)context;
    (void)msg;
}

void ChassisState::updateCylinderSubmission(RobotStateMachineNode *context)
{
    // 自动提交序列
}

// CubeSubmission模式
void ChassisState::handleCubeSubmissionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    handleCylinderSubmissionJoystick(context, msg);
}

void ChassisState::handleCubeSubmissionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    handleCylinderSubmissionButton(context, msg);
}

void ChassisState::handleCubeSubmissionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    (void)context;
    (void)msg;
}

void ChassisState::updateCubeSubmission(RobotStateMachineNode *context)
{
    // 方块自动提交序列
}

// UnderBridge模式
void ChassisState::handleUnderBridgeJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 0) {  // 左摇杆控制底盘
        const auto& joystick = context->getLeftJoystick();
        if (!joystick.is_active(context)) {
            context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
            return;
        }
        
        double x = applyDeadzone(joystick.x, context->getJoystickDeadzone());
        double y = applyDeadzone(joystick.y, context->getJoystickDeadzone());
        
        // 过桥时可能限制速度
        publishChassisCommand(context, x, y, speed_multiplier_ * 0.7);
    }
    // 右摇杆在过桥模式下可能被禁用，或者控制其他功能
}

void ChassisState::handleUnderBridgeButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->button_id == 2 && msg->event_type == 0) {  // X键退出过桥模式
        setArmMode(ArmMode::NormalDetection);
    }
}

void ChassisState::handleUnderBridgeTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    // 可以复用Home模式的扳机逻辑
    if (msg->trigger_id == 0) {
        speed_multiplier_ = 1.0 - msg->value * 0.5;
    } else if (msg->trigger_id == 1) {
        speed_multiplier_ = 1.0 + msg->value * 1.0;
    }
}

void ChassisState::updateUnderBridge(RobotStateMachineNode *context)
{
    // 确保机械臂保持在过桥高度
    static bool arm_position_set = false;
    if (!arm_position_set) {
        RCLCPP_INFO(context->get_logger(), "UnderBridge: Setting arm to bridge height");
        // 发送机械臂过桥高度命令
        arm_position_set = true;
    }
}