/*
    @brief 订阅joy话题，解析成高级抽象意图并发布按钮和轴的状态
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_interfaces/msg/combo_intent.hpp>
#include <custom_interfaces/msg/button_intent.hpp>
#include <custom_interfaces/msg/joystick_intent.hpp>
#include <custom_interfaces/msg/trigger_intent.hpp>
#include <array>
#include <vector>

using std::placeholders::_1;

class JoystickParser : public rclcpp::Node
{
public:
  JoystickParser() : Node("joystick_parser")
  {
    create_subscriptions(); // 批量创建订阅器
    create_publishers();    // 批量创建发布器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JoystickParser::timer_callback, this)); // 定时器回调函数用于定期发布轴状态和扳机状态
  }
private:
    void create_subscriptions() {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickParser::joy_callback, this, _1));
    }
    void create_publishers() {
        combo_intent_pub_ = this->create_publisher<custom_interfaces::msg::ComboIntent>("combo_intent", 10);
        button_intent_pub_ = this->create_publisher<custom_interfaces::msg::ButtonIntent>("button_intent", 10);
        joystick_intent_pub_ = this->create_publisher<custom_interfaces::msg::JoystickIntent>("joystick_intent", 10);
        trigger_intent_pub_ = this->create_publisher<custom_interfaces::msg::TriggerIntent>("trigger_intent", 10);
    }
    // joy回调函数，解析按钮状态并发布ButtonIntent消息
    void joy_callback(const sensor_msgs::msg::Joy &msg)
    {
        last_axes_ = msg.axes; // 轴状态需要在这里更新，以便定时器回调函数能够获取最新的轴状态
        publish_button_intent(msg); //在joy回调函数中调用publish_button_intent来处理按钮状态的变化
        publish_combo_intent(); // 在joy回调函数中调用publish_combo_intent来检测组合按键
    }

    void timer_callback()
    {
        publish_joystick(0, get_axis(0), get_axis(1));  // 发布左摇杆状态
        publish_joystick(1, get_axis(3), get_axis(4));  // 发布右摇杆状态
        publish_trigger(0);  // 发布LT状态
        publish_trigger(1);  // 发布RT状态
    }
    
    void publish_button_intent(const sensor_msgs::msg::Joy &msg) {
        custom_interfaces::msg::ButtonIntent intent_msg;
        if (last_buttons_.size() != msg.buttons.size()) {
            last_buttons_.assign(msg.buttons.size(), 0);
        }
        for (size_t i = 0; i < msg.buttons.size(); ++i) {
            const bool current_pressed = (msg.buttons[i] != 0);
            const bool last_pressed = (last_buttons_[i] != 0);

            if (current_pressed == last_pressed) {
                continue;
            }
            custom_interfaces::msg::ButtonIntent intent_msg;
            intent_msg.button_id = static_cast<uint8_t>(i);
            intent_msg.event_type = current_pressed ? 0U : 1U;
            intent_msg.timestamp = this->now();
            intent_msg.source = "joystick";
            intent_msg.pressure = current_pressed ? 1.0F : 0.0F;
            button_intent_pub_->publish(intent_msg);

            last_buttons_[i] = msg.buttons[i];
        }
    }

    //发布摇杆状态，id区分左右摇杆，x和y分别对应轴0/3和轴1/4
    void publish_joystick(uint8_t id, float x, float y) {
        custom_interfaces::msg::JoystickIntent intent_msg;
        intent_msg.joystick_id = id;
        intent_msg.x = x;
        intent_msg.y = y;
        intent_msg.timestamp = this->now();
        intent_msg.source = "joystick";
        
        // 检测摇杆状态
        if (abs(x) < DEADZONE && abs(y) < DEADZONE) {
            intent_msg.state = 1;  // CENTERED
        } else if (abs(x) > 0.95 || abs(y) > 0.95) {
            intent_msg.state = 2;  // EDGE
        } else {
            intent_msg.state = 0;  // NORMAL
        }
        
        joystick_intent_pub_->publish(intent_msg);
    }

    //发布扳机状态，LT使用轴2，RT使用轴5
    void publish_trigger(uint8_t id) {
        custom_interfaces::msg::TriggerIntent intent_msg;
        intent_msg.trigger_id = id;
        intent_msg.value = get_axis(id == 0 ? 2 : 5);  // LT使用轴2，RT使用轴5
        intent_msg.timestamp = this->now();
        intent_msg.source = "joystick";
        trigger_intent_pub_->publish(intent_msg);
    }

    void publish_combo_intent() {
        custom_interfaces::msg::ComboIntent combo_msg;
        combo_msg.timestamp = this->now();
        combo_msg.source = "joystick";
        // 例如，如果同时按下A和B按钮，可以发布一个特定的ComboIntent消息
        if (get_axis(2) < -0.5F && get_axis(5) < -0.5F) { // LT和RT同时按下
            combo_msg.combo_name = "LT_RT_CONFIRM";
            combo_intent_pub_->publish(combo_msg);
        }
    }

    // 辅助函数用于获取轴状态
    float get_axis(size_t index){
        if (index < last_axes_.size()) {
            return last_axes_[index];
        }
        return 0.0F;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::ComboIntent>::SharedPtr combo_intent_pub_;
    rclcpp::Publisher<custom_interfaces::msg::ButtonIntent>::SharedPtr button_intent_pub_;
    rclcpp::Publisher<custom_interfaces::msg::JoystickIntent>::SharedPtr joystick_intent_pub_;
    rclcpp::Publisher<custom_interfaces::msg::TriggerIntent>::SharedPtr trigger_intent_pub_;
    std::vector<int32_t> last_buttons_;
    std::vector<float> last_axes_;
    rclcpp::TimerBase::SharedPtr timer_;
    float DEADZONE = 0.1F;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickParser>());
  rclcpp::shutdown();
  return 0;
}
/*
{btn:(0,"A"),(1,"B"),(2,"X"),(3,"Y"),(4,"LB"),(5,"RB"),(6,"-"),(7,"+")(8,"home")(9,"LS")(10,"RS")}
{axes:
    (0：1,"LS(左)")(0：-1,"LS(左)") 
    (1：1,"LS(上)")(1:-1,"LS(下)")
    (2：1,"LT")(2:-1,"LT(按下)")
    (3：1,"RS(左)")(3:-1,"RS(右)")
    (4：1,"RS(上)")(4:-1,"RS(下)")
    (5：1,"RT")(5:-1,"RT(按下)")
    (6：1,"左箭头")(6:-1,"右箭头")
    (7：1,"上箭头")(7:-1,"下箭头")
    }
*/