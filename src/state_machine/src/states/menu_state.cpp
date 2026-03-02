#include "state_machine/menu_state.hpp"

#include <cmath>


#include "std_msgs/msg/int32.hpp"

void MenuState::onEnter(RobotStateMachineNode *context) {
    RCLCPP_INFO(context->get_logger(), "Entered MENU mode - Press LT to show menu, release to select");
    
    // 重置状态
    current_selection_ = MenuDirection::NONE;
    trigger_state_.is_pressed = false;
    trigger_state_.selection_made = false;
    joystick_state_.was_centered = true;
    last_activity_ = context->now();
    
    // 通知GUI显示圆盘菜单（通过状态广播）
    // GUI订阅/robot/state，当state=4且sub_state=0时显示圆盘
}

void MenuState::onExit(RobotStateMachineNode* context) {
    RCLCPP_INFO(context->get_logger(), "Exited MENU mode");
    // 通知GUI隐藏圆盘菜单
}

void MenuState::handleTrigger(RobotStateMachineNode* context,
                              const custom_interfaces::msg::TriggerIntent::SharedPtr msg) {
    // 只处理左扳机（trigger_id = 0）
    if (msg->trigger_id != 0) return;
    
    bool current_pressed = (msg->value < -0.1);  // 按压阈值（1=松开，-1=按下）
    
    // 检测扳机按下事件
    if (current_pressed && !trigger_state_.is_pressed) {
        // 扳机按下：显示菜单，重置选择状态
        trigger_state_.is_pressed = true;
        trigger_state_.selection_made = false;
        current_selection_ = MenuDirection::NONE;
        
        RCLCPP_INFO(context->get_logger(), "LT pressed - Menu activated, move joystick to select");
        
        // 通知GUI显示圆盘（可以通过状态广播的子状态实现）
        // 可以在RobotState中添加sub_state字段
    }
    
    // 检测扳机释放事件
    if (!current_pressed && trigger_state_.is_pressed) {
        // 扳机释放：确认当前选中的选项
        trigger_state_.is_pressed = false;
        
        if (!trigger_state_.selection_made && current_selection_ != MenuDirection::NONE) {
            // 有选中项，执行对应功能
            executeSelectedFunction(context, current_selection_);
            trigger_state_.selection_made = true;
        } else {
            RCLCPP_INFO(context->get_logger(), "LT released - No selection, returning to previous mode");
        }
        
        // 无论是否选中，释放扳机后退出菜单模式并返回进入菜单前的模式
        context->changeState(context->getStateBeforeMenu());
    }
}

void MenuState::handleJoystick(RobotStateMachineNode* context,
                               const custom_interfaces::msg::JoystickIntent::SharedPtr msg) {
    // 只处理左摇杆
    if (msg->joystick_id != 0) return;
    
    // 只有扳机按下时才处理摇杆选择
    if (!trigger_state_.is_pressed) return;
    
    float x = msg->x;
    float y = msg->y;
    
    // 转换为方向
    MenuDirection new_direction = joystickToDirection(x, y);
    
    // 检测是否刚从中心移出
    bool is_centered = (new_direction == MenuDirection::NONE);
    
    if (joystick_state_.was_centered && !is_centered) {
        // 摇杆刚离开中心，选中新方向
        current_selection_ = new_direction;
        last_activity_ = context->now();
        RCLCPP_INFO(context->get_logger(), "Selected: %s -> %s", 
                   direction_names_[static_cast<int>(current_selection_)].c_str(),
                   getFunctionForDirection(current_selection_).c_str());
        
        // 通知GUI更新高亮显示
    }
    else if (!joystick_state_.was_centered && !is_centered) {
        // 摇杆持续在某个方向，检测是否改变了方向
        if (new_direction != current_selection_ && new_direction != MenuDirection::NONE) {
            current_selection_ = new_direction;
            last_activity_ = context->now();
            RCLCPP_INFO(context->get_logger(), "Changed to: %s -> %s", 
                       direction_names_[static_cast<int>(current_selection_)].c_str(),
                       getFunctionForDirection(current_selection_).c_str());
            
            // 通知GUI更新高亮显示
        }
    }
    else if (!joystick_state_.was_centered && is_centered) {
        // 摇杆回中，清除选中（但不退出，等待扳机释放）
        if (current_selection_ != MenuDirection::NONE) {
            RCLCPP_INFO(context->get_logger(), "Joystick centered - selection cleared");
            current_selection_ = MenuDirection::NONE;
            last_activity_ = context->now();
            
            // 通知GUI取消高亮
        }
    }
    
    joystick_state_.was_centered = is_centered;
}

void MenuState::handleButton(RobotStateMachineNode* context,
                             const custom_interfaces::msg::ButtonIntent::SharedPtr msg) {
    // 可以添加B键作为取消/返回的备选方案
    if (msg->button_id == 1 && msg->event_type == 0) {  // B键按下
        RCLCPP_INFO(context->get_logger(), "B pressed - cancelling menu");
        context->changeState(context->getStateBeforeMenu());
    }
}

void MenuState::update(RobotStateMachineNode* context) {
    // 可以在这里添加超时机制：如果扳机按下后长时间无操作，自动退出
    if (trigger_state_.is_pressed) {
        auto now = context->now();
        if ((now - last_activity_).seconds() > 5.0) {  // 5秒超时
            RCLCPP_WARN(context->get_logger(), "Menu timeout - returning to previous mode");
            context->changeState(context->getStateBeforeMenu());
        }
    } else {
        last_activity_ = context->now();
    }
}