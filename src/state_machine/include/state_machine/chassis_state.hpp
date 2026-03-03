#pragma once

#include "state_machine/robot_state.hpp"
#include <array>
#include <functional>

class ChassisState : public RobotState
{
public:
    std::string getName() const override;
    uint8_t getStateEnum() const override;

    void onEnter(RobotStateMachineNode *context) override;
    void onExit(RobotStateMachineNode *context) override;

    void handleButton(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;

    void handleJoystick(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;

    void handleTrigger(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::TriggerIntent::SharedPtr msg) override;

    void update(RobotStateMachineNode *context) override;

    enum class ArmMode {
        Home,               // 起始模式，机械臂折叠位于发车区
        NormalDetection,    // 正常检测模式，机械臂yaw用右摇杆控制，底盘行进用左摇杆控制
        CylinderSubmission, // 自动提交能量单元
        CubeSubmission,     // 自动提交方块
        CylinderCollection, // 自动用机械臂夹取能量单元
        CubeCollection,     // 自动用机械臂夹取方块
        UnderBridge         // 过桥模式，底盘行进用左摇杆控制，机械臂保持在过桥高度 
    };
    // 获取和设置当前机械臂模式
    ArmMode getArmMode() const { return arm_mode_; }
    void setArmMode(ArmMode mode);

private:
    // ==================== 机械臂模式枚举 ====================
     ArmMode arm_mode_ = ArmMode::Home;

    // ==================== 函数指针类型定义 ====================
    using JoystickHandler = void (ChassisState::*)(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    
    using ButtonHandler = void (ChassisState::*)(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    
    using TriggerHandler = void (ChassisState::*)(
        RobotStateMachineNode *context,
        const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    
    using UpdateHandler = void (ChassisState::*)(RobotStateMachineNode *context);

    // ==================== 函数指针表结构 ====================
    struct HandlerTable {
        JoystickHandler joystick;
        ButtonHandler button;
        TriggerHandler trigger;
        UpdateHandler update;
    };

    // 初始化函数指针表
    void initHandlerTables();

    // 函数指针表数组（大小等于ArmMode枚举的数量）
    static constexpr size_t ARM_MODE_COUNT = 7;  // Home(0)到UnderBridge(6)共7个
    std::array<HandlerTable, ARM_MODE_COUNT> handler_tables_;

    // ==================== 各模式的实现函数声明 ====================
    
    // Home模式
    void handleHomeJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleHomeButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleHomeTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateHome(RobotStateMachineNode *context);

    // NormalDetection模式
    void handleNormalDetectionJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleNormalDetectionButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleNormalDetectionTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateNormalDetection(RobotStateMachineNode *context);

    // CylinderSubmission模式
    void handleCylinderSubmissionJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleCylinderSubmissionButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleCylinderSubmissionTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateCylinderSubmission(RobotStateMachineNode *context);

    // CubeSubmission模式
    void handleCubeSubmissionJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleCubeSubmissionButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleCubeSubmissionTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateCubeSubmission(RobotStateMachineNode *context);

    // CylinderCollection模式
    void handleCylinderCollectionJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleCylinderCollectionButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleCylinderCollectionTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateCylinderCollection(RobotStateMachineNode *context);

    // CubeCollection模式
    void handleCubeCollectionJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleCubeCollectionButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleCubeCollectionTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateCubeCollection(RobotStateMachineNode *context);

    // UnderBridge模式
    void handleUnderBridgeJoystick(RobotStateMachineNode *context, const custom_interfaces::msg::JoystickIntent::SharedPtr msg);
    void handleUnderBridgeButton(RobotStateMachineNode *context, const custom_interfaces::msg::ButtonIntent::SharedPtr msg);
    void handleUnderBridgeTrigger(RobotStateMachineNode *context, const custom_interfaces::msg::TriggerIntent::SharedPtr msg);
    void updateUnderBridge(RobotStateMachineNode *context);

    // 辅助函数
    double applyDeadzone(double value, double deadzone) const;
    void publishChassisCommand(RobotStateMachineNode *context, double x, double y, double speed_multiplier = 1.0);
    
    double speed_multiplier_ = 1.0;
};