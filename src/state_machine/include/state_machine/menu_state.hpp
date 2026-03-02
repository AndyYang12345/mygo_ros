#pragma once

#include <vector>
#include "state_machine/robot_state_machine_node.hpp"
#include "state_machine/robot_state.hpp"

class MenuState : public RobotState {
public:
    std::string getName() const override { return "MENU"; }
    uint8_t getStateEnum() const override { return 4; }
    
    void onEnter(RobotStateMachineNode* context) override;
    void onExit(RobotStateMachineNode* context) override;
    
    void handleButton(RobotStateMachineNode* context,
                      const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;
    
    void handleJoystick(RobotStateMachineNode* context,
                        const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;
    
    void handleTrigger(RobotStateMachineNode* context,
                       const custom_interfaces::msg::TriggerIntent::SharedPtr msg) override;
    
    void update(RobotStateMachineNode* context) override;

private:
    // 菜单方向枚举（8方向）
    enum class MenuDirection {
        NONE = -1,
        RIGHT = 0,      // 右
        UP_RIGHT = 1,   // 右上
        UP = 2,         // 上
        UP_LEFT = 3,    // 左上
        LEFT = 4,       // 左
        DOWN_LEFT = 5,  // 左下
        DOWN = 6,       // 下
        DOWN_RIGHT = 7  // 右下
    };
    
    // 方向对应的图标/文本（可以根据需要自定义）
    const std::vector<std::string> direction_names_ = {
        "→ RIGHT",      // 右
        "↗ UP-RIGHT",   // 右上
        "↑ UP",         // 上
        "↖ UP-LEFT",    // 左上
        "← LEFT",       // 左
        "↙ DOWN-LEFT",  // 左下
        "↓ DOWN",       // 下
        "↘ DOWN-RIGHT"  // 右下
    };
    
    // 方向对应的功能（可以根据实际需求配置）
    const std::vector<std::string> direction_functions_ = {
        "ARM Mode",      // 右 → 机械臂模式
        "Gripper",       // 右上 → 夹爪控制
        "Preset 1",      // 上 → 预设动作1
        "Preset 2",      // 左上 → 预设动作2
        "Settings",      // 左 → 设置
        "Calibration",   // 左下 → 校准
        "Home",          // 下 → 归位
        "Power Off"      // 右下 → 关机
    };
    
    // 当前选中的方向
    MenuDirection current_selection_ = MenuDirection::NONE;
    
    // 摇杆状态跟踪
    struct JoystickState {
        float x = 0.0;
        float y = 0.0;
        MenuDirection last_direction = MenuDirection::NONE;
        bool was_centered = true;
    } joystick_state_;
    
    // 扳机状态跟踪
    struct TriggerState {
        bool is_pressed = false;
        bool selection_made = false;  // 是否已经完成选择
    } trigger_state_;

    rclcpp::Time last_activity_;
    
    // 将摇杆值转换为8方向
    MenuDirection joystickToDirection(float x, float y) {
        const float DEADZONE = 0.3;  // 死区，避免微小抖动
        
        // 检查是否在中心死区
        if (std::abs(x) < DEADZONE && std::abs(y) < DEADZONE) {
            return MenuDirection::NONE;
        }
        
        // 计算角度（弧度转角度）
        double angle = atan2(y, x) * 180 / 3.14;
        
        // 将角度转换为8方向
        // 角度范围：-180 到 180
        // 右：0° (±22.5°)
        // 右上：45° (22.5° - 67.5°)
        // 上：90° (67.5° - 112.5°)
        // 左上：135° (112.5° - 157.5°)
        // 左：180° or -180° (157.5° - 180° 或 -180° - -157.5°)
        // 左下：-135° (-157.5° - -112.5°)
        // 下：-90° (-112.5° - -67.5°)
        // 右下：-45° (-67.5° - -22.5°)
        
        if (angle > -22.5 && angle <= 22.5) {
            return MenuDirection::RIGHT;
        } else if (angle > 22.5 && angle <= 67.5) {
            return MenuDirection::UP_RIGHT;
        } else if (angle > 67.5 && angle <= 112.5) {
            return MenuDirection::UP;
        } else if (angle > 112.5 && angle <= 157.5) {
            return MenuDirection::UP_LEFT;
        } else if (angle > 157.5 || angle <= -157.5) {
            return MenuDirection::LEFT;
        } else if (angle > -157.5 && angle <= -112.5) {
            return MenuDirection::DOWN_LEFT;
        } else if (angle > -112.5 && angle <= -67.5) {
            return MenuDirection::DOWN;
        } else if (angle > -67.5 && angle <= -22.5) {
            return MenuDirection::DOWN_RIGHT;
        }
        
        return MenuDirection::NONE;
    }
    
    // 获取方向对应的功能
    std::string getFunctionForDirection(MenuDirection dir) {
        if (dir == MenuDirection::NONE) return "None";
        int index = static_cast<int>(dir);
        if (index >= 0 && index < (int)direction_functions_.size()) {
            return direction_functions_[index];
        }
        return "Unknown";
    }
    
    // 执行选中的功能
    void executeSelectedFunction(RobotStateMachineNode* context, MenuDirection dir) {
        if (dir == MenuDirection::NONE) return;
        
        int index = static_cast<int>(dir);
        std::string function = direction_functions_[index];
        
        // RCLCPP_INFO(context->get_logger(), "Menu selected: %s -> %s", 
        //            direction_names_[index].c_str(), function.c_str());
        
        // 根据选中的功能执行对应动作
        switch(dir) {
            case MenuDirection::RIGHT:  // 机械臂模式
                RCLCPP_INFO(context->get_logger(), "RIGHT");
                break;
                
            case MenuDirection::UP_RIGHT:  // 夹爪控制
                RCLCPP_INFO(context->get_logger(), "UP_RIGHT");
                break;
                
            case MenuDirection::UP:  // 预设动作1
                RCLCPP_INFO(context->get_logger(), "UP");
                break;
                
            case MenuDirection::UP_LEFT:  // 预设动作2
                RCLCPP_INFO(context->get_logger(), "UP_LEFT");
                break;
                
            case MenuDirection::LEFT:  // 设置
                RCLCPP_INFO(context->get_logger(), "LEFT");
                // context->changeState(9);  // 如果有设置状态
                break;
                
            case MenuDirection::DOWN_LEFT:  // 校准
                RCLCPP_INFO(context->get_logger(), "DOWN_LEFT");
                // context->changeState(7);  // CALIBRATION
                break;
                
            case MenuDirection::DOWN:  // 归位
                // context->executeHomePosition();
                RCLCPP_INFO(context->get_logger(), "DOWN");
                break;
                
            case MenuDirection::DOWN_RIGHT:  // 关机
                RCLCPP_INFO(context->get_logger(), "DOWN_RIGHT");
                // RCLCPP_WARN(context->get_logger(), "Power off sequence initiated");
                // 执行关机前的安全操作
                // context->sendStopCommands();
                // 可以调用系统关机命令
                // system("sudo shutdown now");
                break;
        }
    }
};