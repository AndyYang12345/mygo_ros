#include "state_machine/chassis_state.hpp"
#include "state_machine/robot_state_machine_node.hpp"

#include <algorithm>
#include "custom_interfaces/msg/arm_named_target.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr bool kChassisDebugEnabled = true;
constexpr float kPi = 3.14159265358979323846F;
}

// ==================== 初始化函数指针表 ====================

void ChassisState::initHandlerTables()
{
    // Home模式
    handler_tables_[0] = {&ChassisState::handleHomeJoystick, &ChassisState::handleHomeButton, &ChassisState::handleHomeTrigger, &ChassisState::updateHome};
    
    // NormalDetection模式
    handler_tables_[1] = {&ChassisState::handleNormalDetectionJoystick, &ChassisState::handleNormalDetectionButton, &ChassisState::handleNormalDetectionTrigger, &ChassisState::updateNormalDetection};
    
    // CylinderSubmission模式
    handler_tables_[2] = {&ChassisState::handleCylinderSubmissionJoystick, &ChassisState::handleCylinderSubmissionButton, &ChassisState::handleCylinderSubmissionTrigger, &ChassisState::updateCylinderSubmission};
    
    // CubeSubmission模式
    handler_tables_[3] = {&ChassisState::handleCubeSubmissionJoystick, &ChassisState::handleCubeSubmissionButton, &ChassisState::handleCubeSubmissionTrigger, &ChassisState::updateCubeSubmission};
    
    // CylinderCollection模式
    handler_tables_[4] = {&ChassisState::handleCylinderCollectionJoystick, &ChassisState::handleCylinderCollectionButton, &ChassisState::handleCylinderCollectionTrigger, &ChassisState::updateCylinderCollection};
    
    // CubeCollection模式
    handler_tables_[5] = {&ChassisState::handleCubeCollectionJoystick, &ChassisState::handleCubeCollectionButton, &ChassisState::handleCubeCollectionTrigger, &ChassisState::updateCubeCollection};
    
    // UnderBridge模式
    handler_tables_[6] = {&ChassisState::handleUnderBridgeJoystick, &ChassisState::handleUnderBridgeButton, &ChassisState::handleUnderBridgeTrigger, &ChassisState::updateUnderBridge};

    // BallSubmission模式
    handler_tables_[7] = {&ChassisState::handleBallSubmissionJoystick, &ChassisState::handleBallSubmissionButton, &ChassisState::handleBallSubmissionTrigger, &ChassisState::updateBallSubmission};
}

// ==================== 构造函数 ====================

ChassisState::ChassisState()
{
    initHandlerTables();
    submenu_items_ = {
        {"MD", &ChassisState::publishCollectorMiddleCommand},
        {"DN", &ChassisState::publishCollectorDownCommand},
        {"UP", &ChassisState::publishCollectorUpCommand},
        {"OP", &ChassisState::publishCollectorOpenCommand},
        {"CL", &ChassisState::publishCollectorCloseCommand},
        {"HOME", &ChassisState::publishArmHomeCommand},
        {"-", &ChassisState::publishNoOpCommand},
        {"-", &ChassisState::publishNoOpCommand},
    };
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

uint8_t ChassisState::getSubState() const
{
    return static_cast<uint8_t>(submenu_selection_);
}

std::vector<std::string> ChassisState::getAvailableModes() const
{
    std::vector<std::string> modes;
    modes.reserve(submenu_items_.size());
    for (const auto &item : submenu_items_)
    {
        modes.push_back(item.label);
    }
    return modes;
}

void ChassisState::onEnter(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "Entered CHASSIS state");
    speed_multiplier_ = 1.0;
    arm_mode_ = ArmMode::Home;  // 默认进入Home模式
    submenu_active_ = false;
    submenu_selection_ = 0;
    updateSubmenuUi(context);
    onArmModeEnter(context, arm_mode_);
}

void ChassisState::onExit(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "Exiting CHASSIS state");
    // 退出时停止所有运动
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
    context->getArmCmdPub()->publish(geometry_msgs::msg::Twist());
}

// ==================== 统一的事件分发接口 ====================

void ChassisState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (submenu_active_ && msg->joystick_id == 1)
    {
        const float x = msg->x;
        const float y = msg->y;
        const float deadzone = 0.25F;
        const float radius = std::sqrt(x * x + y * y);

        if (radius < deadzone)
        {
            RCLCPP_INFO_THROTTLE(
                context->get_logger(), *context->get_clock(), 200,
                "[CHASSIS_SUBMENU] joystick centered (x=%.2f, y=%.2f), keep index=%d, item=%s",
                x, y, submenu_selection_, submenu_items_[static_cast<size_t>(submenu_selection_)].label);
            return;
        }

        const int octant = angleToOctant(x, y);
        if (octant != submenu_selection_)
        {
            submenu_selection_ = octant;
            updateSubmenuUi(context);
            RCLCPP_INFO(
                context->get_logger(),
                "[CHASSIS_SUBMENU] octant=%d -> index=%d, item=%s",
                octant,
                submenu_selection_,
                submenu_items_[static_cast<size_t>(submenu_selection_)].label);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                context->get_logger(), *context->get_clock(), 150,
                "[CHASSIS_SUBMENU] octant=%d, index=%d, item=%s",
                octant,
                submenu_selection_,
                submenu_items_[static_cast<size_t>(submenu_selection_)].label);
        }
        return;
    }

    size_t index = static_cast<size_t>(arm_mode_);
    auto handler = handler_tables_[index].joystick;
    (this->*handler)(context, msg);
}

void ChassisState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->button_id == 4)
    {
        if (msg->event_type == 0)
        {
            submenu_active_ = true;
            submenu_selection_ = 0;
            updateSubmenuUi(context);
            return;
        }

        if (msg->event_type == 1 && submenu_active_)
        {
            submenu_active_ = false;
            const auto index = static_cast<size_t>(std::clamp(submenu_selection_, 0, static_cast<int>(submenu_items_.size() - 1)));
            auto handler = submenu_items_[index].handler;
            (this->*handler)(context);
            updateSubmenuUi(context);
            return;
        }
    }

    if (submenu_active_)
    {
        if (msg->button_id == 1 && msg->event_type == 0)
        {
            submenu_active_ = false;
            updateSubmenuUi(context);
        }
        return;
    }

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
    twist.angular.z = x * context->getChassisMaxAngularSpeed() * speed_multiplier;
    context->getChassisCmdPub()->publish(twist);
}

void ChassisState::publishCollectorCommand(RobotStateMachineNode *context, const std::string &command)
{
    auto msg = std_msgs::msg::String();
    msg.data = command;
    context->getCollectorCmdPub()->publish(msg);
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> collector command: %s", command.c_str());
}

void ChassisState::publishArmHomeCommand(RobotStateMachineNode *context)
{
    auto target = custom_interfaces::msg::ArmNamedTarget();
    target.target_name = "home";
    context->getArmNamedTargetPub()->publish(target);
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> arm named target: home");
}

void ChassisState::publishCollectorMiddleCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "MD");
}

void ChassisState::publishCollectorDownCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "DN");
}

void ChassisState::publishCollectorUpCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "UP");
}

void ChassisState::publishCollectorOpenCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "OP");
}

void ChassisState::publishCollectorCloseCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "CL");
}

void ChassisState::publishNoOpCommand(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> no-op item selected");
}

// ==================== Home模式实现 ====================

void ChassisState::handleHomeJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 0)
    {
        processChassisControl(context, speed_multiplier_);
    }
}

void ChassisState::handleHomeButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0)
    {
        return;
    }

    if (msg->button_id == 2)
    {
        RCLCPP_INFO(context->get_logger(), "Home: X pressed -> NormalDetection");
        setArmMode(context, ArmMode::NormalDetection);
    }
    else if (msg->button_id == 0)
    {
        RCLCPP_INFO(context->get_logger(), "Home: A pressed -> CylinderSubmission");
        setArmMode(context, ArmMode::CylinderSubmission);
    }
    else if (msg->button_id == 3)
    {
        RCLCPP_INFO(context->get_logger(), "Home: Y pressed -> CubeSubmission");
        setArmMode(context, ArmMode::CubeSubmission);
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
        
        // @todo 发布机械臂yaw控制命令（例如调用 context->getArmCmdPub()->publish(arm_twist)）。
        // context->getArmCmdPub()->publish(arm_twist);
        
        if (kChassisDebugEnabled) {
            RCLCPP_DEBUG(context->get_logger(),
                        "NormalDetection: Moving arm yaw: %.2f", x);
        }
    }
}

void ChassisState::handleNormalDetectionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0) return;
    
    switch(msg->button_id) {
        case 0:  // A键：切换到CubeCollection
            setArmMode(context, ArmMode::CubeCollection);
            RCLCPP_INFO(context->get_logger(), "NormalDetection: Switching to CubeCollection");
            break;
        case 1:  // B键：切换到CylinderCollection
            setArmMode(context, ArmMode::CylinderCollection);
            RCLCPP_INFO(context->get_logger(), "NormalDetection: Switching to CylinderCollection");
            break;
        case 2:  // X键：返回Home
            setArmMode(context, ArmMode::Home);
            RCLCPP_INFO(context->get_logger(), "NormalDetection: Returning to Home");
            break;
    }
}

void ChassisState::handleNormalDetectionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    (void)context;
    (void)msg;
    // @todo NormalDetection模式扳机逻辑：例如速度倍率、目标锁定或识别灵敏度调节。
}

void ChassisState::updateNormalDetection(RobotStateMachineNode *context)
{
    processChassisControl(context);
    // @todo 接入自动检测逻辑（如目标识别结果订阅、状态判定与任务触发）。
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
            // @todo 发送夹爪微调命令（闭合方向）。
        } else if (y < 0) {
            RCLCPP_INFO(context->get_logger(), "CubeCollection: Adjusting gripper opener");
            // @todo 发送夹爪微调命令（张开方向）。
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
            // @todo 触发自动抓取序列（路径规划、夹爪控制、完成判定）。
            break;
        case 1:  // B键：取消，返回NormalDetection
            setArmMode(context, ArmMode::Home);
            RCLCPP_INFO(context->get_logger(), "CubeCollection: Cancelled, returning to HOME");
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
        // @todo 发送夹爪开合位置命令（张开）。
        if (kChassisDebugEnabled) {
            RCLCPP_DEBUG(context->get_logger(), "CubeCollection: Gripper open: %.2f", position);
        }
    } else if (msg->trigger_id == 1) {  // RT：闭合
        double position = 1.0 - msg->value;
        // @todo 发送夹爪开合位置命令（闭合）。
        if (kChassisDebugEnabled) {
            RCLCPP_DEBUG(context->get_logger(), "CubeCollection: Gripper close: %.2f", position);
        }
    }
}

void ChassisState::updateCubeCollection(RobotStateMachineNode *context)
{
    // 自动抓取逻辑的状态机
    static enum { SEARCHING, APPROACHING, GRASPING, RETREATING } auto_state = SEARCHING;
    
    // @todo 实现自动抓取状态机转换逻辑（SEARCHING/APPROACHING/GRASPING/RETREATING）。
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
    processChassisControl(context, speed_multiplier_ * 0.5); // 抓取时速度减慢
    // @todo 实现能量单元特定自动抓取逻辑。
}

// CylinderSubmission模式
void ChassisState::handleCylinderSubmissionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    // @todo 自动提交模式摇杆逻辑：根据策略禁用或保留部分控制。
    (void)context;
    (void)msg;
}

void ChassisState::handleCylinderSubmissionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->button_id == 1 && msg->event_type == 0) {  // B键取消
        setArmMode(context, ArmMode::Home);
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
    processChassisControl(context, speed_multiplier_ * 0.5); // 提交时速度减慢
    // @todo 实现能量单元自动提交序列。
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
    processChassisControl(context, speed_multiplier_ * 0.5); // 可按基础速度的50%对齐提交位置
    // @todo 实现方块自动提交序列。
}

// UnderBridge模式
void ChassisState::handleUnderBridgeJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    processChassisControl(context, speed_multiplier_ * 0.7); // 过桥时速度减慢
    // @todo 右摇杆过桥模式逻辑：明确禁用策略或定义功能映射。
}

void ChassisState::handleUnderBridgeButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->button_id == 1 && msg->event_type == 0) {  // B键返回Home
        setArmMode(context, ArmMode::Home);
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
    processChassisControl(context, speed_multiplier_ * 0.7);// 过桥速度为基础速度的70%
    // 确保机械臂保持在过桥高度
    static bool arm_position_set = false;
    if (!arm_position_set) {
        RCLCPP_INFO(context->get_logger(), "UnderBridge: Setting arm to bridge height");
        // @todo 发送机械臂过桥高度命令并做执行结果校验。
        arm_position_set = true;
    }
}

void ChassisState::handleBallSubmissionJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 0)
    {
        processChassisControl(context, speed_multiplier_ * 0.5);
    }
}

void ChassisState::handleBallSubmissionButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0)
    {
        return;
    }

    if (msg->button_id == 0)
    {
        auto preset = std_msgs::msg::Int32();
        preset.data = 80;
        context->getPresetPub()->publish(preset);
    }
    else if (msg->button_id == 1)
    {
        setArmMode(context, ArmMode::Home);
    }
}

void ChassisState::handleBallSubmissionTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    if (msg->trigger_id == 0 && msg->value < -0.8F)
    {
        auto preset = std_msgs::msg::Int32();
        preset.data = 81;
        context->getPresetPub()->publish(preset);
    }
}

void ChassisState::updateBallSubmission(RobotStateMachineNode *context)
{
    processChassisControl(context, speed_multiplier_ * 0.5);
}

void ChassisState::processChassisControl(RobotStateMachineNode *context, double speed_scale)
{
    const auto& joystick = context->getLeftJoystick();
    
    // 检查数据有效性
    if (!joystick.is_active(context)) {
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        return;
    }
    
    // 应用死区
    double x = applyDeadzone(joystick.x, context->getJoystickDeadzone());
    double y = applyDeadzone(joystick.y, context->getJoystickDeadzone());
    
    // 如果摇杆在中心，停止
    if (x == 0.0 && y == 0.0) {
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        return;
    }
    
    // 生成Twist消息
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = y * context->getChassisMaxLinearSpeed() * speed_scale;
    twist.angular.z = x * context->getChassisMaxAngularSpeed() * speed_scale;
    
    // 发布命令
    context->getChassisCmdPub()->publish(twist);
    
    if (kChassisDebugEnabled) {
        RCLCPP_DEBUG(context->get_logger(),
                    "Chassis cmd: linear=%.2f m/s, angular=%.2f rad/s (scale=%.2f)",
                    twist.linear.x, twist.angular.z, speed_scale);
    }
}

void ChassisState::setArmMode(RobotStateMachineNode *context, ArmMode new_mode)
{
    if (arm_mode_ == new_mode) return;
    
    ArmMode old_mode = arm_mode_;
    
    // // 1. 调用旧模式的退出回调
    // onArmModeExit(context, old_mode);
    
    // 2. 更新模式
    arm_mode_ = new_mode;
    
    // 3. 调用新模式的进入回调
    onArmModeEnter(context, new_mode);
}

// 子模式生命周期管理
void ChassisState::onArmModeEnter(RobotStateMachineNode *context, ArmMode new_mode)
{
    switch(new_mode) {
        case ArmMode::Home:
            onEnterHomeMode(context);
            break;
        case ArmMode::NormalDetection:
            onEnterNormalDetectionMode(context);
            break;
        case ArmMode::CylinderSubmission:
            onEnterCylinderSubmissionMode(context);
            break;
        case ArmMode::CubeSubmission:
            onEnterCubeSubmissionMode(context);
            break;
        case ArmMode::CylinderCollection:
            onEnterCylinderCollectionMode(context);
            break;
        case ArmMode::CubeCollection:
            onEnterCubeCollectionMode(context);
            break;
        case ArmMode::UnderBridge:
            onEnterUnderBridgeMode(context);
            break;
        case ArmMode::BallSubmission:
            onEnterBallSubmissionMode(context);
            break;
    }
}

void ChassisState::onEnterHomeMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入Home模式");
    }
    // @todo 调整机械臂到Home折叠位姿并等待到位反馈。
}

void ChassisState::onEnterNormalDetectionMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入NormalDetection模式");
    }
    // @todo 调整机械臂到正常检测位姿（启用yaw轴手动控制）。
}

void ChassisState::onEnterCylinderSubmissionMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入CylinderSubmission模式");
    }
    // @todo 调整机械臂到能量单元提交预备位姿并初始化自动提交流程。
}

void ChassisState::onEnterCubeSubmissionMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入CubeSubmission模式");
    }
    // @todo 调整机械臂到方块提交预备位姿并初始化自动提交流程。
}

void ChassisState::onEnterCylinderCollectionMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入CylinderCollection模式");
    }
    // @todo 调整机械臂到能量单元抓取预备位姿并初始化抓取状态机。
}

void ChassisState::onEnterCubeCollectionMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入CubeCollection模式");
    }
    // @todo 调整机械臂到方块抓取预备位姿并初始化抓取状态机。
}

void ChassisState::onEnterUnderBridgeMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入UnderBridge模式");
    }
    // @todo 调整机械臂到过桥安全高度并锁定过桥期间的机械臂自由度。
}

void ChassisState::onEnterBallSubmissionMode(RobotStateMachineNode *context)
{
    if (kChassisDebugEnabled) {
        RCLCPP_INFO(context->get_logger(), "[DEBUG] 进入BallSubmission模式");
    }
}

std::string ChassisState::armModeName(ArmMode mode) const
{
    switch (mode)
    {
        case ArmMode::Home:
            return "HOME";
        case ArmMode::NormalDetection:
            return "NORMAL_DETECTION";
        case ArmMode::CylinderSubmission:
            return "CYLINDER_SUBMISSION";
        case ArmMode::CubeSubmission:
            return "CUBE_SUBMISSION";
        case ArmMode::CylinderCollection:
            return "CYLINDER_COLLECTION";
        case ArmMode::CubeCollection:
            return "CUBE_COLLECTION";
        case ArmMode::UnderBridge:
            return "UNDER_BRIDGE";
        case ArmMode::BallSubmission:
            return "BALL_SUBMISSION";
    }
    return "UNKNOWN";
}

void ChassisState::updateSubmenuUi(RobotStateMachineNode *context)
{
    std::vector<std::string> names;
    names.reserve(submenu_items_.size());
    for (const auto &item : submenu_items_)
    {
        names.push_back(item.label);
    }

    context->setMenuItems(names);

    if (submenu_active_)
    {
        context->setMenuSelection(submenu_selection_);
        return;
    }

    for (size_t i = 0; i < submenu_items_.size(); ++i)
    {
        if (static_cast<int>(i) == submenu_selection_)
        {
            context->setMenuSelection(static_cast<int>(i));
            return;
        }
    }
    context->setMenuSelection(0);
}

int ChassisState::angleToOctant(float x, float y) const
{
    const float norm_x = -x;
    const float norm_y = y;

    float angle = std::atan2(norm_y, norm_x);
    if (angle < 0.0F)
    {
        angle += 2.0F * kPi;
    }

    const float sector = (2.0F * kPi) / 8.0F;
    int octant = static_cast<int>(std::floor((angle + sector * 0.5F) / sector));
    octant %= 8;
    return octant;
}
