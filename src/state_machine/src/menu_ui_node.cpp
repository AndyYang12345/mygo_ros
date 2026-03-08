#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QFont>
#include <QKeyEvent>
#include <QMetaObject>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QTimer>
#include <QWidget>

#include "custom_interfaces/msg/button_intent.hpp"
#include "custom_interfaces/msg/joystick_intent.hpp"
#include "custom_interfaces/msg/robot_state.hpp"
#include "custom_interfaces/msg/trigger_intent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kMainMenuDeadzone = 0.25F;
constexpr int kOctantCount = 8;

constexpr std::array<int, kOctantCount> kMenuOctantToIndex = {
  1,  // RIGHT -> ARM
  3,  // UP_RIGHT -> VISION_TASK
  0,  // UP -> CHASSIS
  5,  // UP_LEFT -> POLE
  4,  // LEFT -> IDLE
  2,  // DOWN_LEFT -> EMERGENCY
  0,  // DOWN -> CHASSIS
  1   // DOWN_RIGHT -> ARM
};

int menuIndexToOctant(int menu_index)
{
  static constexpr std::array<int, 6> kMenuIndexToOctant = {
    2,  // CHASSIS
    0,  // ARM
    5,  // EMERGENCY
    1,  // VISION_TASK
    4,  // IDLE
    3   // POLE
  };

  if (menu_index < 0 || menu_index >= static_cast<int>(kMenuIndexToOctant.size())) {
    return 0;
  }
  return kMenuIndexToOctant[static_cast<size_t>(menu_index)];
}

std::vector<std::string> subModesForMainMode(const std::string & main_mode)
{
  if (main_mode == "ARM") {
    return {
      "home", "normal_detection", "pickup_left", "pickup_right",
      "cylinder_left", "cylinder_right", "box_left", "box_right"};
  }

  if (main_mode == "CHASSIS") {
    return {
      "Home", "NormalDetection", "CylinderSubmission", "CubeSubmission",
      "CylinderCollection", "CubeCollection", "UnderBridge", "BallSubmission"};
  }

  if (main_mode == "VISION_TASK") {
    return {"A_START", "B_CANCEL", "-", "-", "-", "-", "-", "-"};
  }

  if (main_mode == "POLE") {
    return {"EXIT_POLE_MODE", "-", "-", "-", "-", "-", "-", "-"};
  }

  if (main_mode == "EMERGENCY") {
    return {"E_STOP", "-", "-", "-", "-", "-", "-", "-"};
  }

  if (main_mode == "IDLE") {
    return {"WAIT", "-", "-", "-", "-", "-", "-", "-"};
  }

  return {"-", "-", "-", "-", "-", "-", "-", "-"};
}

int angleToOctant(float x, float y)
{
  const float norm_x = -x;
  const float norm_y = y;

  float angle = std::atan2(norm_y, norm_x);
  if (angle < 0.0F) {
    angle += 2.0F * kPi;
  }

  const float sector = (2.0F * kPi) / static_cast<float>(kOctantCount);
  int octant = static_cast<int>(std::floor((angle + sector * 0.5F) / sector));
  octant %= kOctantCount;
  return octant;
}

std::vector<std::string> expandToEightMain(const std::vector<std::string> & menu_items)
{
  std::vector<std::string> result(kOctantCount, "-");
  for (int i = 0; i < kOctantCount; ++i) {
    const int mapped = kMenuOctantToIndex[static_cast<size_t>(i)];
    if (mapped >= 0 && mapped < static_cast<int>(menu_items.size())) {
      result[static_cast<size_t>(i)] = menu_items[static_cast<size_t>(mapped)];
    }
  }
  return result;
}

std::vector<std::string> expandToEightSub(const std::vector<std::string> & available_modes)
{
  std::vector<std::string> result(kOctantCount, "-");
  const size_t count = std::min(available_modes.size(), static_cast<size_t>(kOctantCount));
  for (size_t i = 0; i < count; ++i) {
    result[i] = available_modes[i];
  }
  return result;
}
}  // namespace

class MenuUiWidget : public QWidget
{
public:
  MenuUiWidget()
  {
    setWindowTitle("MyGo Menu UI");
    resize(1280, 720);
    setMinimumSize(960, 540);
  }

  void updateRobotState(const custom_interfaces::msg::RobotState & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_name_ = msg.state_name;
    sub_state_ = static_cast<int>(msg.sub_state);
    main_selection_index_ = static_cast<int>(msg.menu_selection);

    main_labels_ = expandToEightMain(msg.menu_items);

    if (state_name_ == "MENU") {
      if (main_selection_index_ >= 0 && main_selection_index_ < static_cast<int>(msg.menu_items.size())) {
        selected_main_mode_ = msg.menu_items[static_cast<size_t>(main_selection_index_)];
      } else {
        selected_main_mode_.clear();
      }
      sub_labels_ = subModesForMainMode(selected_main_mode_);
      main_octant_ = menuIndexToOctant(main_selection_index_);
    } else {
      selected_main_mode_ = state_name_;
      sub_labels_ = expandToEightSub(msg.available_modes);
    }

    if (sub_state_ >= 0 && sub_state_ < static_cast<int>(msg.available_modes.size())) {
      submode_name_ = msg.available_modes[static_cast<size_t>(sub_state_)];
    } else {
      submode_name_.clear();
    }

    if (state_name_ == "POLE") {
      const uint8_t sub = static_cast<uint8_t>(sub_state_ & 0xFF);
      selected_pole_id_ = static_cast<int>(sub & 0x01U);
    }

    if (lt_held_) {
      main_octant_ = angleToOctant(joy_x_, joy_y_);
    }

    if (submenu_active_) {
      sub_octant_ = angleToOctant(joy_x_, joy_y_);
    } else if (sub_state_ >= 0 && sub_state_ < kOctantCount) {
      sub_octant_ = sub_state_;
    }
  }

  void updateLtState(bool pressed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lt_held_ = pressed;
    if (lt_held_) {
      main_octant_ = angleToOctant(joy_x_, joy_y_);
    }
  }

  void updateSubmenuState(bool active)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    submenu_active_ = active;
    if (submenu_active_) {
      sub_octant_ = angleToOctant(joy_x_, joy_y_);
    }
  }

  void updateRightJoystick(float x, float y)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joy_x_ = x;
    joy_y_ = y;

    const float radius = std::sqrt(x * x + y * y);
    if (radius < kMainMenuDeadzone) {
      return;
    }

    const int octant = angleToOctant(x, y);
    if (lt_held_ && !submenu_active_) {
      main_octant_ = octant;
    }
    if (submenu_active_) {
      sub_octant_ = octant;
    }
  }

  void updatePoleSelectedId(uint8_t selected_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    selected_pole_id_ = (selected_id == 0U) ? 0 : 1;
  }

  void updatePoleLockMask(uint8_t lock_mask)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pole_locked_state_[0] = (lock_mask & 0x01U) != 0U;
    pole_locked_state_[1] = (lock_mask & 0x02U) != 0U;
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    (void)event;

    std::string state_name;
    std::string submode_name;
    std::vector<std::string> left_labels;
    std::vector<std::string> right_labels;
    bool lt_held = false;
    bool submenu_active = false;
    int main_octant = 0;
    int sub_octant = 0;
    int selected_pole_id = 0;
    std::array<bool, 2> pole_locked_state = {false, false};

    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_name = state_name_;
      submode_name = submode_name_;
      left_labels = main_labels_;
      right_labels = sub_labels_;
      lt_held = lt_held_;
      submenu_active = submenu_active_;
      main_octant = main_octant_;
      sub_octant = sub_octant_;
      selected_pole_id = selected_pole_id_;
      pole_locked_state = pole_locked_state_;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QLinearGradient bg(0, 0, width(), height());
    bg.setColorAt(0.0, QColor(245, 248, 255));
    bg.setColorAt(0.5, QColor(232, 240, 249));
    bg.setColorAt(1.0, QColor(220, 231, 242));
    painter.fillRect(rect(), bg);

    QFont title_font("Noto Sans CJK SC", 24, QFont::DemiBold);
    QFont sub_title_font("Noto Sans CJK SC", 20, QFont::Medium);

    const std::string mode = state_name.empty() ? std::string("IDLE") : state_name;
    const std::string submode = submode_name.empty() ? std::string("NONE") : submode_name;
    const QString mode_text =
      ">" + QString::fromStdString(mode) + ">>" + QString::fromStdString(submode);

    painter.setPen(QColor(32, 53, 71));
    painter.setFont(title_font);
    painter.drawText(QRect(28, 22, width() - 56, 42), Qt::AlignLeft | Qt::AlignVCenter, "POWER ON");

    painter.setPen(QColor(18, 79, 122));
    painter.setFont(sub_title_font);
    painter.drawText(QRect(28, 64, width() - 56, 42), Qt::AlignLeft | Qt::AlignVCenter, mode_text);

    const QPointF left_center(width() * 0.30, height() * 0.58);
    const QPointF right_center(width() * 0.74, height() * 0.58);

    double left_radius = 140.0;
    double right_radius = 115.0;

    if (lt_held && !submenu_active) {
      left_radius = 185.0;
      right_radius = 100.0;
    } else if (submenu_active) {
      left_radius = 130.0;
      right_radius = 165.0;
    }

    drawWheel(
      painter, left_center, left_radius, left_labels, main_octant,
      lt_held && !submenu_active, QColor(31, 74, 112), QColor(17, 41, 65), QColor(245, 250, 255));

    drawWheel(
      painter, right_center, right_radius, right_labels, sub_octant,
      submenu_active, QColor(90, 122, 53), QColor(46, 79, 26), QColor(248, 252, 242));

    drawPoleStatusCards(painter, selected_pole_id, pole_locked_state);
  }

  void keyPressEvent(QKeyEvent * event) override
  {
    if (event && event->key() == Qt::Key_Escape) {
      QApplication::quit();
      return;
    }
    QWidget::keyPressEvent(event);
  }

private:
  void drawPoleStatusCards(
    QPainter & painter,
    int selected_pole_id,
    const std::array<bool, 2> & pole_locked_state)
  {
    const int card_width = 230;
    const int card_height = 70;
    const int gap = 18;
    const int total_width = card_width * 2 + gap;
    const int origin_x = (width() - total_width) / 2;
    const int origin_y = static_cast<int>(height() * 0.84);

    const std::array<QString, 2> names = {"LEFT", "RIGHT"};

    QFont title_font("Noto Sans CJK SC", 12, QFont::Bold);
    QFont status_font("Noto Sans CJK SC", 11, QFont::DemiBold);

    for (int i = 0; i < 2; ++i) {
      const QRect card_rect(origin_x + i * (card_width + gap), origin_y, card_width, card_height);
      const bool selected = (i == selected_pole_id);
      const bool locked = pole_locked_state[static_cast<size_t>(i)];

      const QColor bg = selected ? QColor(226, 238, 255) : QColor(240, 245, 250);
      const QColor border = selected ? QColor(35, 89, 144) : QColor(155, 172, 188);
      const QColor status = locked ? QColor(188, 47, 47) : QColor(33, 128, 66);

      painter.setPen(QPen(border, selected ? 2.8 : 1.6));
      painter.setBrush(bg);
      painter.drawRoundedRect(card_rect, 12.0, 12.0);

      painter.setFont(title_font);
      painter.setPen(QColor(30, 48, 66));
      painter.drawText(
        QRect(card_rect.x() + 12, card_rect.y() + 8, card_rect.width() - 24, 24),
        Qt::AlignLeft | Qt::AlignVCenter,
        names[static_cast<size_t>(i)]);

      painter.setFont(status_font);
      painter.setPen(status);
      painter.drawText(
        QRect(card_rect.x() + 12, card_rect.y() + 32, card_rect.width() - 24, 24),
        Qt::AlignLeft | Qt::AlignVCenter,
        locked ? "Locked" : "Unlocked");
    }
  }

  void drawWheel(
    QPainter & painter,
    const QPointF & center,
    double radius,
    const std::vector<std::string> & labels,
    int selected_octant,
    bool active,
    const QColor & base_color,
    const QColor & active_color,
    const QColor & fill_color)
  {
    painter.save();

    QColor ring = base_color;
    ring.setAlpha(190);
    painter.setPen(QPen(ring, active ? 4.0 : 2.5));
    painter.setBrush(fill_color);
    painter.drawEllipse(center, radius, radius);

    QFont label_font("Noto Sans CJK SC", active ? 14 : 11, active ? QFont::Bold : QFont::DemiBold);
    painter.setFont(label_font);

    const double item_radius = radius * 0.18;
    const double orbit = radius * 0.76;

    for (int i = 0; i < kOctantCount; ++i) {
      const double angle_deg = static_cast<double>(i) * 45.0;
      const double angle_rad = angle_deg * static_cast<double>(kPi) / 180.0;
      const QPointF p(
        center.x() + orbit * std::cos(angle_rad),
        center.y() - orbit * std::sin(angle_rad));

      const bool is_selected = active && (i == selected_octant);
      QColor item_color = is_selected ? active_color : base_color;
      item_color.setAlpha(is_selected ? 220 : 120);
      painter.setPen(Qt::NoPen);
      painter.setBrush(item_color);
      painter.drawEllipse(p, item_radius, item_radius);

      painter.setPen(is_selected ? QColor(255, 255, 255) : QColor(14, 28, 44));
      const QRectF text_rect(
        p.x() - item_radius * 0.95,
        p.y() - item_radius * 0.65,
        item_radius * 1.9,
        item_radius * 1.3);

      std::string text = "-";
      if (i >= 0 && i < static_cast<int>(labels.size())) {
        text = labels[static_cast<size_t>(i)];
      }
      painter.drawText(text_rect, Qt::AlignCenter, QString::fromStdString(text));
    }

    painter.restore();
  }

  std::mutex mutex_;
  std::string state_name_ = "IDLE";
  std::string submode_name_;
  int sub_state_ = 0;

  bool lt_held_ = false;
  bool submenu_active_ = false;
  int main_selection_index_ = 0;
  float joy_x_ = 0.0F;
  float joy_y_ = 0.0F;
  std::string selected_main_mode_;

  int main_octant_ = 0;
  int sub_octant_ = 0;
  int selected_pole_id_ = 0;
  std::array<bool, 2> pole_locked_state_ = {false, false};

  std::vector<std::string> main_labels_ = {
    "ARM", "VISION_TASK", "CHASSIS", "POLE", "IDLE", "EMERGENCY", "CHASSIS", "ARM"};
  std::vector<std::string> sub_labels_ = {
    "-", "-", "-", "-", "-", "-", "-", "-"};
};

class MenuUiNode : public rclcpp::Node
{
public:
  explicit MenuUiNode(MenuUiWidget * widget)
  : Node("menu_ui_node"), widget_(widget)
  {
    state_sub_ = create_subscription<custom_interfaces::msg::RobotState>(
      "/robot/state", 10,
      [this](const custom_interfaces::msg::RobotState::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateRobotState(*msg);
      });

    trigger_sub_ = create_subscription<custom_interfaces::msg::TriggerIntent>(
      "trigger_intent", 10,
      [this](const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }

        if (msg->trigger_id != 0) {
          return;
        }

        const bool pressed = (msg->event_type != 0) || (msg->value < 0.95F);
        widget_->updateLtState(pressed);
      });

    button_sub_ = create_subscription<custom_interfaces::msg::ButtonIntent>(
      "button_intent", 10,
      [this](const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }

        if (msg->button_id != 4) {
          return;
        }

        if (msg->event_type == 0) {
          widget_->updateSubmenuState(true);
        } else if (msg->event_type == 1) {
          widget_->updateSubmenuState(false);
        }
      });

    joystick_sub_ = create_subscription<custom_interfaces::msg::JoystickIntent>(
      "joystick_intent", 10,
      [this](const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }

        if (msg->joystick_id != 1) {
          return;
        }

        widget_->updateRightJoystick(msg->x, msg->y);
      });

    pole_selected_id_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/cmd/pole/selected_id", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updatePoleSelectedId(msg->data);
      });

    pole_lock_mask_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/cmd/pole/lock_mask", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updatePoleLockMask(msg->data);
      });

    RCLCPP_INFO(get_logger(), "menu_ui_node started.");
  }

private:
  MenuUiWidget * widget_;
  rclcpp::Subscription<custom_interfaces::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<custom_interfaces::msg::TriggerIntent>::SharedPtr trigger_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ButtonIntent>::SharedPtr button_sub_;
  rclcpp::Subscription<custom_interfaces::msg::JoystickIntent>::SharedPtr joystick_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr pole_selected_id_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr pole_lock_mask_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  MenuUiWidget widget;
  widget.show();

  auto node = std::make_shared<MenuUiNode>(&widget);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // Ctrl+C triggers ROS shutdown; mirror that into Qt loop exit.
  rclcpp::on_shutdown([&app]() {
    QMetaObject::invokeMethod(&app, "quit", Qt::QueuedConnection);
  });

  std::thread ros_spin_thread([&exec]() { exec.spin(); });

  QTimer repaint_timer;
  QObject::connect(&repaint_timer, &QTimer::timeout, [&widget]() { widget.update(); });
  repaint_timer.start(33);

  const int ret = app.exec();

  exec.cancel();
  if (ros_spin_thread.joinable()) {
    ros_spin_thread.join();
  }
  rclcpp::shutdown();

  return ret;
}
