/*
    @brief 订阅joy话题，解析成高级抽象意图并发布按钮和轴的状态
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_interfaces/msg/combo_intent.hpp>
#include <custom_interfaces/msg/button_intent.hpp>
#include <custom_interfaces/msg/joystick_intent.hpp>

using std::placeholders::_1;

class JoystickParser : public rclcpp::Node
{
public:
  JoystickParser() : Node("joystick_parser")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoystickParser::joy_callback, this, _1));
  }
private:
  void joy_callback(const sensor_msgs::msg::Joy &msg) const
  {
    auto get_axis = [&msg](size_t index) -> float {
      if (index < msg.axes.size()) {
        return msg.axes[index];
      }
      return 0.0F;
    };

    auto get_button = [&msg](size_t index) -> int {
      if (index < msg.buttons.size()) {
        return msg.buttons[index];
      }
      return 0;
    };

    auto axis_dir = [](float value, const char *positive, const char *negative) -> const char * {
      if (value > 0.5F) {
        return positive;
      }
      if (value < -0.5F) {
        return negative;
      }
      return "中位";
    };

    RCLCPP_INFO(
      this->get_logger(),
      "Joy msg: stamp=%u.%u, frame_id='%s', axes_count=%zu, buttons_count=%zu",
      msg.header.stamp.sec,
      msg.header.stamp.nanosec,
      msg.header.frame_id.c_str(),
      msg.axes.size(),
      msg.buttons.size());

    RCLCPP_INFO(
      this->get_logger(),
      "Buttons | A:%d B:%d X:%d Y:%d LB:%d RB:%d -:%d +:%d HOME:%d LS:%d RS:%d",
      get_button(0), get_button(1), get_button(2), get_button(3), get_button(4),
      get_button(5), get_button(6), get_button(7), get_button(8), get_button(9),
      get_button(10));

    const float ls_x = get_axis(0);
    const float ls_y = get_axis(1);
    const float lt = get_axis(2);
    const float rs_x = get_axis(3);
    const float rs_y = get_axis(4);
    const float rt = get_axis(5);
    const float dpad_x = get_axis(6);
    const float dpad_y = get_axis(7);

    RCLCPP_INFO(
      this->get_logger(),
      "Axes[0] LS_x=%.3f -> %s | Axes[1] LS_y=%.3f -> %s",
      ls_x,
      axis_dir(ls_x, "LS左", "LS右"),
      ls_y,
      axis_dir(ls_y, "LS上", "LS下"));

    RCLCPP_INFO(
      this->get_logger(),
      "Axes[2] LT=%.3f -> %s",
      lt,
      axis_dir(lt, "LT释放", "LT按下"));

    RCLCPP_INFO(
      this->get_logger(),
      "Axes[3] RS_x=%.3f -> %s | Axes[4] RS_y=%.3f -> %s | Axes[5] RT=%.3f -> %s",
      rs_x,
      axis_dir(rs_x, "RS左", "RS右"),
      rs_y,
      axis_dir(rs_y, "RS上", "RS下"),
      rt,
      axis_dir(rt, "RT释放", "RT按下"));

    RCLCPP_INFO(
      this->get_logger(),
      "Axes[6] Dpad_x=%.3f -> %s | Axes[7] Dpad_y=%.3f -> %s",
      dpad_x,
      axis_dir(dpad_x, "左箭头", "右箭头"),
      dpad_y,
      axis_dir(dpad_y, "上箭头", "下箭头"));
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::ComboIntent>::SharedPtr combo_intent_pub_;
  rclcpp::Publisher<custom_interfaces::msg::ButtonIntent>::SharedPtr button_intent_pub_;
  rclcpp::Publisher<custom_interfaces::msg::JoystickIntent>::SharedPtr joystick_intent_pub_;

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
    (4：1,"RS(上)")(5:-1,"RS(下)")
    (5：1,"RT")(6:-1,"RT(按下)")
    (6：1,"左箭头")(6:-1,"右箭头")
    (7：1,"上箭头")(7:-1,"下箭头")
    }
*/