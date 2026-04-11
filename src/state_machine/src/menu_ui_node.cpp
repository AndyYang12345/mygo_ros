#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QFont>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QMetaObject>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QTimer>
#include <QWidget>
#include <QString>

#include "custom_interfaces/msg/button_intent.hpp"
#include "custom_interfaces/msg/joystick_intent.hpp"
#include "custom_interfaces/msg/robot_state.hpp"
#include "custom_interfaces/msg/trigger_intent.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
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
  2,  // DOWN_LEFT -> BALL
  0,  // DOWN -> CHASSIS
  1   // DOWN_RIGHT -> ARM
};

int menuIndexToOctant(int menu_index)
{
  static constexpr std::array<int, 6> kMenuIndexToOctant = {
    2,  // CHASSIS
    0,  // ARM
    5,  // BALL
    1,  // VISION_TASK
    4,  // IDLE
    3   // POLE
  };

  if (menu_index < 0 || menu_index >= static_cast<int>(kMenuIndexToOctant.size())) {
    return 0;
  }
  return kMenuIndexToOctant[static_cast<size_t>(menu_index)];
}

struct ModeMenuConfig
{
  const char * mode;
  std::array<const char *, kOctantCount> labels;
  bool enable_lb_submenu;
};

const std::array<ModeMenuConfig, 6> kModeMenuConfigs = {{
  // Keep this in sync with the per-state submenu layout.
  {"ARM", {"Right Energy Unit", "under_bridge", "home", "folded", "Left Energy Unit", "-", "Side Energy Unit", "-"}, true},
  {"CHASSIS", {"MD", "-", "DN", "-", "under_bridge", "-", "folded", "-"}, true},
  {"VISION_TASK", {"A_START", "B_CANCEL", "-", "-", "-", "-", "-", "-"}, false},
  {"POLE", {"EXIT_POLE_MODE", "-", "-", "-", "-", "-", "-", "-"}, false},
  {"BALL", {"NEXT", "MD", "DN", "UP", "OP", "CL", "-", "-"}, true},
  {"IDLE", {"WAIT", "-", "-", "-", "-", "-", "-", "-"}, false}
}};

const ModeMenuConfig * findModeMenuConfig(const std::string & mode)
{
  for (const auto & cfg : kModeMenuConfigs) {
    if (mode == cfg.mode) {
      return &cfg;
    }
  }
  return nullptr;
}

std::vector<std::string> subModesForMainMode(const std::string & main_mode)
{
  std::vector<std::string> out(kOctantCount, "-");
  const auto * cfg = findModeMenuConfig(main_mode);
  if (!cfg) {
    return out;
  }
  for (size_t i = 0; i < cfg->labels.size(); ++i) {
    out[i] = cfg->labels[i];
  }
  return out;
}

bool isLbSubmenuEnabledForMode(const std::string & mode)
{
  const auto * cfg = findModeMenuConfig(mode);
  return cfg != nullptr && cfg->enable_lb_submenu;
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

std::string normalizeLabelForWrap(const std::string & raw)
{
  if (raw == "-") {
    return raw;
  }

  std::string out;
  out.reserve(raw.size() + 8);
  for (size_t i = 0; i < raw.size(); ++i) {
    const char c = raw[i];
    if (c == '_') {
      out.push_back(' ');
      continue;
    }

    if (i > 0 && std::isupper(static_cast<unsigned char>(c)) &&
      std::islower(static_cast<unsigned char>(raw[i - 1])))
    {
      out.push_back(' ');
    }
    out.push_back(c);
  }
  return out;
}

double smoothApproach(double current, double target, double alpha)
{
  return current + (target - current) * alpha;
}

double clampUnit(double value)
{
  return std::clamp(value, -1.0, 1.0);
}

QColor blendColors(const QColor & a, const QColor & b, double t)
{
  const double clamped = std::clamp(t, 0.0, 1.0);
  return QColor(
    static_cast<int>(std::lround(a.red() + (b.red() - a.red()) * clamped)),
    static_cast<int>(std::lround(a.green() + (b.green() - a.green()) * clamped)),
    static_cast<int>(std::lround(a.blue() + (b.blue() - a.blue()) * clamped)),
    static_cast<int>(std::lround(a.alpha() + (b.alpha() - a.alpha()) * clamped)));
}

QColor trackPowerColor(double normalized_value)
{
  const double magnitude = std::abs(clampUnit(normalized_value));
  const QColor low(82, 198, 110);
  const QColor high(239, 78, 54);
  return blendColors(low, high, std::pow(magnitude, 0.82));
}

std::string cameraStatusText(const std::string & raw, const std::string & fallback = "UNKNOWN")
{
  if (raw.empty()) {
    return fallback;
  }
  return normalizeLabelForWrap(raw);
}

QColor cameraConnectionColor(const std::string & state)
{
  if (state == "CONNECTED") {
    return QColor(34, 160, 86);
  }
  if (state == "CONNECTING") {
    return QColor(214, 142, 28);
  }
  return QColor(210, 72, 64);
}

QColor cameraAppStateColor(const std::string & state)
{
  if (state == "RUNNING") {
    return QColor(34, 160, 86);
  }
  if (state == "IDLE") {
    return QColor(38, 116, 188);
  }
  if (state == "STOPPED") {
    return QColor(214, 142, 28);
  }
  return QColor(210, 72, 64);
}

QColor cameraTrackStateColor(const std::string & state)
{
  if (state == "Tracking") {
    return QColor(34, 160, 86);
  }
  if (state == "Locked") {
    return QColor(214, 142, 28);
  }
  if (state == "Searching") {
    return QColor(38, 116, 188);
  }
  if (state == "Waiting" || state == "Stopped") {
    return QColor(112, 128, 148);
  }
  return QColor(210, 72, 64);
}

QString cameraSummaryText(
  const std::string & connection_state,
  const std::string & app_state,
  const std::string & track_state)
{
  if (connection_state != "CONNECTED") {
    return "Camera Offline";
  }
  if (app_state == "RUNNING" && track_state == "Tracking") {
    return "Target Tracking";
  }
  if (app_state == "RUNNING") {
    return "Vision Active";
  }
  if (app_state == "STOPPED") {
    return "Task Stopped";
  }
  if (app_state == "IDLE") {
    return "Ready For Start";
  }
  return "Camera Online";
}

QColor cameraSummaryColor(
  const std::string & connection_state,
  const std::string & app_state,
  const std::string & track_state)
{
  if (connection_state != "CONNECTED") {
    return QColor(210, 72, 64);
  }
  if (app_state == "RUNNING" && track_state == "Tracking") {
    return QColor(34, 160, 86);
  }
  if (app_state == "RUNNING") {
    return QColor(38, 116, 188);
  }
  if (app_state == "STOPPED") {
    return QColor(214, 142, 28);
  }
  return QColor(38, 116, 188);
}

double easeOutBack(double t)
{
  const double clamped = std::clamp(t, 0.0, 1.0);
  constexpr double c1 = 1.70158;
  constexpr double c3 = c1 + 1.0;
  const double x = clamped - 1.0;
  return 1.0 + c3 * x * x * x + c1 * x * x;
}
}  // namespace

class MenuUiWidget : public QWidget
{
public:
  MenuUiWidget()
  {
    setWindowTitle("MyGo Menu UI");
    resize(1480, 860);
    setMinimumSize(1200, 700);
  }

  void setChassisSpeedLimits(double max_linear_speed, double max_angular_speed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    max_linear_speed_ = std::max(0.001, max_linear_speed);
    max_angular_speed_ = std::max(0.001, max_angular_speed);
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

    if (!isLbSubmenuEnabledForMode(state_name_)) {
      submenu_active_ = false;
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
    if (!isLbSubmenuEnabledForMode(state_name_)) {
      submenu_active_ = false;
      return;
    }
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
    if (state_name_ == "MENU") {
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

  void updateChassisVelocity(double linear_x, double angular_z)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double linear_norm = max_linear_speed_ > 1e-6 ?
      linear_x / max_linear_speed_ : 0.0;
    const double angular_norm = max_angular_speed_ > 1e-6 ?
      angular_z / max_angular_speed_ : 0.0;

    chassis_linear_norm_target_ = clampUnit(linear_norm);
    chassis_turn_norm_target_ = clampUnit(angular_norm);
    chassis_left_track_target_ = clampUnit(chassis_linear_norm_target_ - chassis_turn_norm_target_);
    chassis_right_track_target_ = clampUnit(chassis_linear_norm_target_ + chassis_turn_norm_target_);
  }

  void updateCameraConnectionState(const std::string & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_connection_state_ = state.empty() ? "DISCONNECTED" : state;
  }

  void updateCameraAppState(const std::string & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_app_state_ = state.empty() ? "UNKNOWN" : state;
  }

  void updateCameraTrackState(const std::string & state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_track_state_ = state.empty() ? "UNKNOWN" : state;
  }

  void updateCameraProtocolStatus(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_protocol_status_ = text.empty() ? "waiting for camera node" : text;
  }

  void updateCameraTargetFound(bool found)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_target_found_ = found;
  }

  void updateCameraCanScan(bool can_scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_can_scan_ = can_scan;
  }

  void updateCameraTargetPixel(const std::string & pixel)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_target_pixel_ = pixel.empty() ? "-1,-1" : pixel;
  }

  bool isLbSubmenuAllowed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return isLbSubmenuEnabledForMode(state_name_);
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    (void)event;

    std::string state_name;
    std::string submode_name;
    std::vector<std::string> left_labels;
    std::vector<std::string> right_labels;
    bool submenu_active = false;
    int main_octant = 0;
    int sub_octant = 0;
    int selected_pole_id = 0;
    std::array<bool, 2> pole_locked_state = {false, false};
    double main_menu_radius = 220.0;
    double sub_menu_radius = 158.0;
    double main_menu_visibility = 0.0;
    double sub_menu_visibility = 0.0;
    double chassis_linear_norm = 0.0;
    double chassis_turn_norm = 0.0;
    double chassis_left_track = 0.0;
    double chassis_right_track = 0.0;
    std::string camera_connection_state;
    std::string camera_app_state;
    std::string camera_track_state;
    std::string camera_protocol_status;
    bool camera_target_found = false;
    bool camera_can_scan = false;
    std::string camera_target_pixel;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_name = state_name_;
      submode_name = submode_name_;
      left_labels = main_labels_;
      right_labels = sub_labels_;
      submenu_active = submenu_active_;
      main_octant = main_octant_;
      sub_octant = sub_octant_;
      selected_pole_id = selected_pole_id_;
      pole_locked_state = pole_locked_state_;
      main_menu_radius = main_menu_radius_;
      sub_menu_radius = sub_menu_radius_;
      main_menu_visibility = main_menu_visibility_;
      sub_menu_visibility = sub_menu_visibility_;

      chassis_linear_norm_ = smoothApproach(chassis_linear_norm_, chassis_linear_norm_target_, 0.18);
      chassis_turn_norm_ = smoothApproach(chassis_turn_norm_, chassis_turn_norm_target_, 0.18);
      chassis_left_track_ = smoothApproach(chassis_left_track_, chassis_left_track_target_, 0.18);
      chassis_right_track_ = smoothApproach(chassis_right_track_, chassis_right_track_target_, 0.18);

      chassis_linear_norm = chassis_linear_norm_;
      chassis_turn_norm = chassis_turn_norm_;
      chassis_left_track = chassis_left_track_;
      chassis_right_track = chassis_right_track_;
      camera_connection_state = camera_connection_state_;
      camera_app_state = camera_app_state_;
      camera_track_state = camera_track_state_;
      camera_protocol_status = camera_protocol_status_;
      camera_target_found = camera_target_found_;
      camera_can_scan = camera_can_scan_;
      camera_target_pixel = camera_target_pixel_;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QLinearGradient bg(0, 0, width(), height());
    bg.setColorAt(0.0, QColor(245, 249, 255));
    bg.setColorAt(0.45, QColor(227, 237, 246));
    bg.setColorAt(1.0, QColor(207, 221, 236));
    painter.fillRect(rect(), bg);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(255, 255, 255, 68));
    painter.drawEllipse(QPointF(width() * 0.16, height() * 0.18), 180.0, 140.0);
    painter.setBrush(QColor(120, 158, 191, 34));
    painter.drawEllipse(QPointF(width() * 0.84, height() * 0.82), 250.0, 180.0);

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

    const double camera_panel_width = std::clamp(width() * 0.27, 320.0, 390.0);
    drawCameraStatusPanel(
      painter,
      QRectF(width() - camera_panel_width - 28.0, 26.0, camera_panel_width, 214.0),
      camera_connection_state,
      camera_app_state,
      camera_track_state,
      camera_protocol_status,
      camera_target_found,
      camera_can_scan,
      camera_target_pixel);

    const bool show_pole_cards = (state_name == "POLE");
    const bool show_main_menu = (state_name == "MENU");
    const bool show_sub_menu = (!show_main_menu && submenu_active);
    const bool show_chassis_dashboard = (state_name == "CHASSIS");

    const double target_main_visibility = show_main_menu ? 1.0 : 0.0;
    const double target_sub_visibility = show_sub_menu ? 1.0 : 0.0;
    const double target_main_radius = show_main_menu ? 228.0 : 205.0;
    const double target_sub_radius = show_sub_menu ? 162.0 : 142.0;

    main_menu_visibility = smoothApproach(main_menu_visibility, target_main_visibility, 0.18);
    sub_menu_visibility = smoothApproach(sub_menu_visibility, target_sub_visibility, 0.18);
    main_menu_radius = smoothApproach(main_menu_radius, target_main_radius, 0.20);
    sub_menu_radius = smoothApproach(sub_menu_radius, target_sub_radius, 0.20);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      main_menu_radius_ = main_menu_radius;
      sub_menu_radius_ = sub_menu_radius;
      main_menu_visibility_ = main_menu_visibility;
      sub_menu_visibility_ = sub_menu_visibility;
    }

    const QPointF overlay_center(width() * 0.50, height() * 0.54);
    const double menu_overlay_visibility = std::max(main_menu_visibility, sub_menu_visibility);
    const double main_menu_anim = easeOutBack(main_menu_visibility);
    const double sub_menu_anim = easeOutBack(sub_menu_visibility);
    const double main_scale = 0.82 + 0.18 * main_menu_anim;
    const double sub_scale = 0.82 + 0.18 * sub_menu_anim;

    if (show_chassis_dashboard) {
      drawChassisDashboard(
        painter,
        QRectF(width() * 0.16, height() * 0.18, width() * 0.68, height() * 0.64),
        chassis_left_track,
        chassis_right_track,
        chassis_linear_norm,
        chassis_turn_norm,
        submode_name);
    }

    if (menu_overlay_visibility > 0.04) {
      drawBackdropBlur(painter, menu_overlay_visibility);
    }

    if (main_menu_visibility > 0.04) {
      drawWheel(
        painter, overlay_center, main_menu_radius, left_labels, main_octant,
        show_main_menu, QColor(32, 90, 154), QColor(11, 46, 86), QColor(239, 247, 255), main_menu_visibility,
        "主菜单", main_scale);
    }

    if (sub_menu_visibility > 0.04) {
      drawWheel(
        painter, overlay_center, sub_menu_radius, right_labels, sub_octant,
        show_sub_menu, QColor(103, 130, 60), QColor(52, 85, 30), QColor(246, 252, 238), sub_menu_visibility,
        "子菜单", sub_scale);
    }

    if (show_pole_cards) {
      drawPoleStatusCards(painter, selected_pole_id, pole_locked_state);
    }
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
  void drawBackdropBlur(QPainter & painter, double visibility)
  {
    painter.save();

    const double intensity = std::clamp(visibility, 0.0, 1.0);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(224, 232, 240, static_cast<int>(90 + 90 * intensity)));
    painter.drawRect(rect());

    const std::array<QPointF, 3> glow_centers = {
      QPointF(width() * 0.22, height() * 0.28),
      QPointF(width() * 0.72, height() * 0.24),
      QPointF(width() * 0.56, height() * 0.76)};
    const std::array<QColor, 3> glow_colors = {
      QColor(255, 255, 255, 74),
      QColor(120, 158, 191, 52),
      QColor(83, 133, 188, 38)};

    for (size_t i = 0; i < glow_centers.size(); ++i) {
      QRadialGradient glow(glow_centers[i], std::max(width(), height()) * 0.26);
      QColor inner = glow_colors[i];
      inner.setAlpha(static_cast<int>(inner.alpha() * intensity));
      QColor outer = glow_colors[i];
      outer.setAlpha(0);
      glow.setColorAt(0.0, inner);
      glow.setColorAt(1.0, outer);
      painter.setBrush(glow);
      painter.drawEllipse(glow_centers[i], width() * 0.22, height() * 0.18);
    }

    const QRectF focus_rect(width() * 0.16, height() * 0.18, width() * 0.68, height() * 0.68);
    painter.setBrush(QColor(255, 255, 255, static_cast<int>(24 + 30 * intensity)));
    painter.drawRoundedRect(focus_rect, 36.0, 36.0);
    painter.restore();
  }

  void drawChassisDashboard(
    QPainter & painter,
    const QRectF & panel_rect,
    double left_track,
    double right_track,
    double linear_norm,
    double turn_norm,
    const std::string & submode_name)
  {
    painter.save();

    QPainterPath panel_path;
    panel_path.addRoundedRect(panel_rect, 34.0, 34.0);

    QLinearGradient panel_grad(panel_rect.topLeft(), panel_rect.bottomRight());
    panel_grad.setColorAt(0.0, QColor(252, 254, 255, 236));
    panel_grad.setColorAt(1.0, QColor(223, 232, 241, 224));
    painter.fillPath(panel_path, panel_grad);
    painter.setPen(QPen(QColor(255, 255, 255, 180), 1.2));
    painter.drawPath(panel_path);
    painter.setPen(QPen(QColor(77, 105, 132, 92), 2.2));
    painter.drawRoundedRect(panel_rect.adjusted(1.0, 1.0, -1.0, -1.0), 34.0, 34.0);

    const QRectF title_rect(panel_rect.left() + 28.0, panel_rect.top() + 18.0, panel_rect.width() - 56.0, 40.0);
    QFont panel_title_font("Noto Sans CJK SC", 18, QFont::DemiBold);
    QFont panel_info_font("Noto Sans CJK SC", 12, QFont::Medium);
    painter.setPen(QColor(32, 54, 78));
    painter.setFont(panel_title_font);
    painter.drawText(title_rect, Qt::AlignLeft | Qt::AlignVCenter, "底盘姿态");

    const QString submode_text = QString::fromStdString(submode_name.empty() ? "Home" : normalizeLabelForWrap(submode_name));
    painter.setPen(QColor(82, 109, 132));
    painter.setFont(panel_info_font);
    painter.drawText(
      QRectF(panel_rect.right() - 280.0, panel_rect.top() + 22.0, 240.0, 28.0),
      Qt::AlignRight | Qt::AlignVCenter,
      "当前子模式  " + submode_text);

    const QRectF vehicle_rect(
      panel_rect.center().x() - panel_rect.width() * 0.16,
      panel_rect.center().y() - panel_rect.height() * 0.24,
      panel_rect.width() * 0.32,
      panel_rect.height() * 0.48);

    const qreal bar_width = std::clamp(panel_rect.width() * 0.12, 72.0, 110.0);
    const qreal bar_height = vehicle_rect.height() * 1.02;
    const qreal bar_gap = panel_rect.width() * 0.055;
    const QRectF left_bar_rect(
      vehicle_rect.left() - bar_gap - bar_width,
      vehicle_rect.center().y() - bar_height / 2.0,
      bar_width,
      bar_height);
    const QRectF right_bar_rect(
      vehicle_rect.right() + bar_gap,
      vehicle_rect.center().y() - bar_height / 2.0,
      bar_width,
      bar_height);

    drawTrackPowerBar(painter, left_bar_rect, left_track, "LEFT");
    drawTrackPowerBar(painter, right_bar_rect, right_track, "RIGHT");
    drawTrackedVehicle(painter, vehicle_rect, left_track, right_track, linear_norm, turn_norm);

    painter.restore();
  }

  void drawTrackPowerBar(
    QPainter & painter,
    const QRectF & rect,
    double value,
    const QString & label)
  {
    painter.save();

    const double clamped = clampUnit(value);
    const QColor accent = trackPowerColor(clamped);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(236, 242, 247, 228));
    painter.drawRoundedRect(rect, 20.0, 20.0);

    const QRectF channel = rect.adjusted(rect.width() * 0.28, 18.0, -rect.width() * 0.28, -18.0);
    painter.setBrush(QColor(210, 220, 229, 210));
    painter.drawRoundedRect(channel, 10.0, 10.0);

    const double center_y = channel.center().y();
    painter.setPen(QPen(QColor(94, 114, 136, 220), 2.0));
    painter.drawLine(QPointF(channel.left() - 7.0, center_y), QPointF(channel.right() + 7.0, center_y));

    const double half_height = channel.height() * 0.5;
    const double fill_height = half_height * std::abs(clamped);
    QRectF fill_rect(
      channel.left() + 2.0,
      clamped >= 0.0 ? center_y - fill_height : center_y,
      channel.width() - 4.0,
      fill_height);

    if (fill_rect.height() > 1.0) {
      QLinearGradient fill_grad(fill_rect.topLeft(), fill_rect.bottomLeft());
      if (clamped >= 0.0) {
        fill_grad.setColorAt(0.0, QColor(255, 255, 255, 210));
        fill_grad.setColorAt(1.0, accent);
      } else {
        fill_grad.setColorAt(0.0, accent);
        fill_grad.setColorAt(1.0, QColor(255, 255, 255, 210));
      }
      painter.setPen(Qt::NoPen);
      painter.setBrush(fill_grad);
      painter.drawRoundedRect(fill_rect, 8.0, 8.0);

      painter.setBrush(QColor(255, 255, 255, static_cast<int>(80 + std::abs(clamped) * 70.0)));
      painter.drawRoundedRect(fill_rect.adjusted(2.0, 2.0, -2.0, -2.0), 6.0, 6.0);
    }

    QFont label_font("Noto Sans CJK SC", 11, QFont::DemiBold);
    QFont value_font("JetBrains Mono", 12, QFont::Bold);
    painter.setPen(QColor(61, 80, 100));
    painter.setFont(label_font);
    painter.drawText(
      QRectF(rect.left(), rect.top() - 2.0, rect.width(), 24.0),
      Qt::AlignCenter | Qt::AlignVCenter,
      label);

    painter.setFont(value_font);
    painter.drawText(
      QRectF(rect.left(), rect.bottom() - 28.0, rect.width(), 22.0),
      Qt::AlignCenter | Qt::AlignVCenter,
      QString::number(clamped, 'f', 2));

    painter.restore();
  }

  void drawTrackedVehicle(
    QPainter & painter,
    const QRectF & rect,
    double left_track,
    double right_track,
    double linear_norm,
    double turn_norm)
  {
    painter.save();

    const QColor left_color = trackPowerColor(left_track);
    const QColor right_color = trackPowerColor(right_track);

    const QRectF shadow_rect = rect.adjusted(-8.0, -2.0, 8.0, 14.0);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(60, 89, 117, 34));
    painter.drawRoundedRect(shadow_rect, 54.0, 54.0);

    const qreal track_width = rect.width() * 0.18;
    const qreal body_gap = rect.width() * 0.06;
    const QRectF left_track_rect(rect.left(), rect.top(), track_width, rect.height());
    const QRectF right_track_rect(rect.right() - track_width, rect.top(), track_width, rect.height());
    const QRectF body_rect(
      left_track_rect.right() + body_gap,
      rect.top() + rect.height() * 0.09,
      rect.width() - 2.0 * (track_width + body_gap),
      rect.height() * 0.82);

    auto drawTrack = [&](const QRectF & track_rect, const QColor & color, bool left_side) {
      QLinearGradient grad(track_rect.topLeft(), track_rect.bottomRight());
      grad.setColorAt(0.0, blendColors(color, QColor(255, 255, 255), 0.55));
      grad.setColorAt(1.0, blendColors(color, QColor(24, 36, 48), 0.35));
      painter.setBrush(grad);
      painter.setPen(QPen(QColor(43, 58, 74, 170), 2.0));
      painter.drawRoundedRect(track_rect, 24.0, 24.0);

      painter.setPen(QPen(QColor(255, 255, 255, 80), 1.0));
      const double segment_step = track_rect.height() / 7.0;
      for (int i = 1; i < 7; ++i) {
        const double y = track_rect.top() + segment_step * i;
        painter.drawLine(
          QPointF(track_rect.left() + 7.0, y),
          QPointF(track_rect.right() - 7.0, y));
      }

      const QRectF drive_glow = left_side ?
        QRectF(track_rect.left() - 6.0, track_rect.center().y() - 20.0, 18.0, 40.0) :
        QRectF(track_rect.right() - 12.0, track_rect.center().y() - 20.0, 18.0, 40.0);
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor(color.red(), color.green(), color.blue(), 90));
      painter.drawRoundedRect(drive_glow, 8.0, 8.0);
    };

    drawTrack(left_track_rect, left_color, true);
    drawTrack(right_track_rect, right_color, false);

    QLinearGradient body_grad(body_rect.topLeft(), body_rect.bottomLeft());
    body_grad.setColorAt(0.0, QColor(246, 249, 252));
    body_grad.setColorAt(0.48, QColor(211, 220, 229));
    body_grad.setColorAt(1.0, QColor(170, 184, 198));
    painter.setBrush(body_grad);
    painter.setPen(QPen(QColor(83, 101, 120, 180), 2.2));
    painter.drawRoundedRect(body_rect, 34.0, 34.0);

    const QRectF cockpit_rect = body_rect.adjusted(
      body_rect.width() * 0.18, body_rect.height() * 0.14,
      -body_rect.width() * 0.18, -body_rect.height() * 0.50);
    QLinearGradient cockpit_grad(cockpit_rect.topLeft(), cockpit_rect.bottomLeft());
    cockpit_grad.setColorAt(0.0, QColor(108, 133, 157));
    cockpit_grad.setColorAt(1.0, QColor(56, 79, 100));
    painter.setBrush(cockpit_grad);
    painter.setPen(QPen(QColor(224, 236, 247, 150), 1.6));
    painter.drawRoundedRect(cockpit_rect, 22.0, 22.0);

    const QRectF deck_rect = body_rect.adjusted(
      body_rect.width() * 0.22, body_rect.height() * 0.44,
      -body_rect.width() * 0.22, -body_rect.height() * 0.14);
    painter.setBrush(QColor(232, 238, 244, 185));
    painter.setPen(QPen(QColor(121, 139, 157, 120), 1.4));
    painter.drawRoundedRect(deck_rect, 18.0, 18.0);

    const QPointF nose_center(body_rect.center().x(), body_rect.top() + body_rect.height() * 0.15);
    QPainterPath arrow_path;
    arrow_path.moveTo(nose_center.x(), nose_center.y() - 26.0);
    arrow_path.lineTo(nose_center.x() - 14.0, nose_center.y() + 6.0);
    arrow_path.lineTo(nose_center.x() - 5.0, nose_center.y() + 6.0);
    arrow_path.lineTo(nose_center.x() - 5.0, nose_center.y() + 26.0);
    arrow_path.lineTo(nose_center.x() + 5.0, nose_center.y() + 26.0);
    arrow_path.lineTo(nose_center.x() + 5.0, nose_center.y() + 6.0);
    arrow_path.lineTo(nose_center.x() + 14.0, nose_center.y() + 6.0);
    arrow_path.closeSubpath();
    painter.setBrush(QColor(35, 58, 82, 220));
    painter.setPen(Qt::NoPen);
    painter.drawPath(arrow_path);

    QFont metric_label_font("Noto Sans CJK SC", 11, QFont::Medium);
    QFont metric_value_font("JetBrains Mono", 16, QFont::Bold);
    painter.setFont(metric_label_font);
    painter.setPen(QColor(92, 111, 129));
    painter.drawText(
      QRectF(rect.left(), rect.bottom() + 18.0, rect.width(), 22.0),
      Qt::AlignHCenter | Qt::AlignVCenter,
      "LINEAR / TURN");

    painter.setFont(metric_value_font);
    painter.setPen(QColor(32, 52, 74));
    painter.drawText(
      QRectF(rect.left(), rect.bottom() + 38.0, rect.width(), 26.0),
      Qt::AlignHCenter | Qt::AlignVCenter,
      QString("%1 / %2")
      .arg(linear_norm, 0, 'f', 2)
      .arg(turn_norm, 0, 'f', 2));

    const qreal indicator_width = rect.width() * 0.34;
    const QRectF indicator_rect(
      rect.center().x() - indicator_width / 2.0,
      rect.bottom() + 78.0,
      indicator_width,
      14.0);
    painter.setBrush(QColor(220, 229, 237, 210));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(indicator_rect, 7.0, 7.0);

    const double average_drive = clampUnit((left_track + right_track) * 0.5);
    const QRectF indicator_fill(
      indicator_rect.left(),
      indicator_rect.top(),
      indicator_rect.width() * (0.5 + average_drive * 0.5),
      indicator_rect.height());
    QLinearGradient indicator_grad(indicator_fill.topLeft(), indicator_fill.topRight());
    indicator_grad.setColorAt(0.0, QColor(90, 196, 120, 140));
    indicator_grad.setColorAt(1.0, trackPowerColor(average_drive));
    painter.setBrush(indicator_grad);
    painter.drawRoundedRect(indicator_fill, 7.0, 7.0);

    painter.restore();
  }

  void drawPoleStatusCards(
    QPainter & painter,
    int selected_pole_id,
    const std::array<bool, 2> & pole_locked_state)
  {
    const int card_width = 280;
    const int card_height = 86;
    const int gap = 24;
    const int total_width = card_width * 2 + gap;
    const int origin_x = (width() - total_width) / 2;
    const int origin_y = static_cast<int>(height() * 0.54) - card_height / 2;

    const std::array<QString, 2> names = {"LEFT", "RIGHT"};

    QFont title_font("Noto Sans CJK SC", 14, QFont::Bold);
    QFont status_font("Noto Sans CJK SC", 13, QFont::DemiBold);

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
        QRect(card_rect.x() + 16, card_rect.y() + 10, card_rect.width() - 32, 30),
        Qt::AlignLeft | Qt::AlignVCenter,
        names[static_cast<size_t>(i)]);

      painter.setFont(status_font);
      painter.setPen(status);
      painter.drawText(
        QRect(card_rect.x() + 16, card_rect.y() + 44, card_rect.width() - 32, 30),
        Qt::AlignLeft | Qt::AlignVCenter,
        locked ? "Locked" : "Unlocked");
    }
  }

  void drawCameraStatusPanel(
    QPainter & painter,
    const QRectF & panel_rect,
    const std::string & connection_state,
    const std::string & app_state,
    const std::string & track_state,
    const std::string & protocol_status,
    bool target_found,
    bool can_scan,
    const std::string & target_pixel)
  {
    painter.save();

    QPainterPath panel_path;
    panel_path.addRoundedRect(panel_rect, 28.0, 28.0);

    QLinearGradient panel_grad(panel_rect.topLeft(), panel_rect.bottomRight());
    panel_grad.setColorAt(0.0, QColor(21, 45, 66, 242));
    panel_grad.setColorAt(0.5, QColor(20, 66, 97, 232));
    panel_grad.setColorAt(1.0, QColor(11, 27, 43, 242));
    painter.fillPath(panel_path, panel_grad);
    painter.setPen(QPen(QColor(220, 241, 255, 110), 1.4));
    painter.drawPath(panel_path);

    const QColor summary_color = cameraSummaryColor(connection_state, app_state, track_state);
    const QColor connection_color = cameraConnectionColor(connection_state);
    const QColor app_color = cameraAppStateColor(app_state);
    const QColor track_color = cameraTrackStateColor(track_state);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(summary_color.red(), summary_color.green(), summary_color.blue(), 46));
    painter.drawEllipse(panel_rect.topRight() - QPointF(56.0, -34.0), 44.0, 44.0);

    QFont title_font("Noto Sans CJK SC", 13, QFont::DemiBold);
    QFont headline_font("Noto Sans CJK SC", 20, QFont::Bold);
    QFont label_font("Noto Sans CJK SC", 10, QFont::Medium);
    QFont value_font("JetBrains Mono", 11, QFont::Bold);
    QFont foot_font("Noto Sans CJK SC", 10, QFont::Medium);

    painter.setPen(QColor(208, 230, 247));
    painter.setFont(title_font);
    painter.drawText(
      QRectF(panel_rect.left() + 22.0, panel_rect.top() + 18.0, panel_rect.width() - 44.0, 22.0),
      Qt::AlignLeft | Qt::AlignVCenter,
      "CAMERA LINK");

    painter.setPen(summary_color);
    painter.setFont(headline_font);
    painter.drawText(
      QRectF(panel_rect.left() + 22.0, panel_rect.top() + 42.0, panel_rect.width() - 96.0, 34.0),
      Qt::AlignLeft | Qt::AlignVCenter,
      cameraSummaryText(connection_state, app_state, track_state));

    painter.setBrush(summary_color);
    painter.drawEllipse(QPointF(panel_rect.right() - 36.0, panel_rect.top() + 34.0), 7.0, 7.0);

    const auto draw_status_row =
      [&](double y, const QString & label, const std::string & value, const QColor & color) {
        const QRectF row_rect(panel_rect.left() + 18.0, y, panel_rect.width() - 36.0, 30.0);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 255, 255, 20));
        painter.drawRoundedRect(row_rect, 12.0, 12.0);

        painter.setPen(QColor(188, 212, 229));
        painter.setFont(label_font);
        painter.drawText(
          QRectF(row_rect.left() + 14.0, row_rect.top(), 66.0, row_rect.height()),
          Qt::AlignLeft | Qt::AlignVCenter,
          label);

        const QString value_text = QString::fromStdString(cameraStatusText(value));
        const QFontMetrics value_fm(value_font);
        const QString elided_value =
          value_fm.elidedText(value_text, Qt::ElideRight, static_cast<int>(row_rect.width() - 118.0));
        painter.setPen(color);
        painter.setFont(value_font);
        painter.drawText(
          QRectF(row_rect.left() + 92.0, row_rect.top(), row_rect.width() - 106.0, row_rect.height()),
          Qt::AlignLeft | Qt::AlignVCenter,
          elided_value);
      };

    draw_status_row(panel_rect.top() + 92.0, "LINK", connection_state, connection_color);
    draw_status_row(panel_rect.top() + 126.0, "APP", app_state, app_color);
    draw_status_row(panel_rect.top() + 160.0, "TRACK", track_state, track_color);

    const QRectF foot_rect(panel_rect.left() + 18.0, panel_rect.bottom() - 44.0, panel_rect.width() - 36.0, 26.0);
    painter.setPen(QColor(176, 202, 220));
    painter.setFont(foot_font);
    std::string foot_status = protocol_status.empty() ? "waiting for camera node" : protocol_status;
    foot_status += " | TF:" + std::string(target_found ? "1" : "0");
    foot_status += " CS:" + std::string(can_scan ? "1" : "0");
    foot_status += " TP:" + target_pixel;
    const QString foot_text = QString::fromStdString(foot_status);
    const QFontMetrics foot_fm(foot_font);
    painter.drawText(
      foot_rect,
      Qt::AlignLeft | Qt::AlignVCenter,
      foot_fm.elidedText(foot_text, Qt::ElideRight, static_cast<int>(foot_rect.width())));

    painter.restore();
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
    const QColor & fill_color,
    double visibility,
    const QString & title,
    double scale)
  {
    painter.save();
    painter.setOpacity(std::clamp(visibility, 0.0, 1.0));
    painter.translate(center);
    painter.scale(std::clamp(scale, 0.7, 1.05), std::clamp(scale, 0.7, 1.05));
    painter.translate(-center);

    QColor ring = base_color;
    ring.setAlpha(190);
    painter.setPen(QPen(ring, active ? 4.0 : 2.5));
    painter.setBrush(fill_color);
    painter.drawEllipse(center, radius, radius);

    QFont title_font("Noto Sans CJK SC", active ? 15 : 13, QFont::DemiBold);
    painter.setFont(title_font);
    painter.setPen(active ? QColor(12, 37, 56) : QColor(52, 75, 96));
    painter.drawText(
      QRect(
        static_cast<int>(center.x() - radius),
        static_cast<int>(center.y() - radius - 34),
        static_cast<int>(radius * 2.0),
        28),
      Qt::AlignCenter,
      title);

    QFont label_font("Noto Sans CJK SC", active ? 13 : 11, active ? QFont::Bold : QFont::DemiBold);
    painter.setFont(label_font);
    const QFontMetrics fm(label_font);

    const double item_radius = radius * 0.12;
    const double orbit = radius * 0.78;

    for (int i = 0; i < kOctantCount; ++i) {
      const double angle_deg = static_cast<double>(i) * 45.0;
      const double angle_rad = angle_deg * static_cast<double>(kPi) / 180.0;
      const QPointF p(
        center.x() + orbit * std::cos(angle_rad),
        center.y() - orbit * std::sin(angle_rad));

      const bool is_selected = active && (i == selected_octant);
      QColor item_color = is_selected ? active_color : base_color;
      item_color.setAlpha(is_selected ? 228 : 128);
      painter.setPen(Qt::NoPen);
      painter.setBrush(item_color);
      painter.drawEllipse(p, item_radius, item_radius);

      std::string text = "-";
      if (i >= 0 && i < static_cast<int>(labels.size())) {
        text = normalizeLabelForWrap(labels[static_cast<size_t>(i)]);
      }

      const QString text_q = QString::fromStdString(text);
      const int min_w = static_cast<int>(radius * 0.36);
      const int text_w = std::max(min_w, fm.horizontalAdvance(text_q) + 20);
      const int text_h = static_cast<int>(radius * 0.19);
      const QRect text_rect(
        static_cast<int>(p.x() - text_w / 2.0),
        static_cast<int>(p.y() - text_h / 2.0),
        text_w,
        text_h);

      QColor text_bg = is_selected ? QColor(18, 41, 64, 235) : QColor(245, 250, 255, 216);
      QColor text_border = is_selected ? QColor(205, 226, 250, 230) : QColor(143, 165, 186, 180);
      painter.setBrush(text_bg);
      painter.setPen(QPen(text_border, is_selected ? 1.8 : 1.1));
      painter.drawRoundedRect(text_rect, 8.0, 8.0);

      painter.setPen(is_selected ? QColor(255, 255, 255) : QColor(12, 28, 45));
      painter.drawText(text_rect.adjusted(6, 2, -6, -2), Qt::AlignCenter | Qt::TextWordWrap, text_q);
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
  double main_menu_radius_ = 220.0;
  double sub_menu_radius_ = 158.0;
  double main_menu_visibility_ = 0.0;
  double sub_menu_visibility_ = 0.0;
  double max_linear_speed_ = 1.0;
  double max_angular_speed_ = 1.0;
  double chassis_linear_norm_target_ = 0.0;
  double chassis_turn_norm_target_ = 0.0;
  double chassis_left_track_target_ = 0.0;
  double chassis_right_track_target_ = 0.0;
  double chassis_linear_norm_ = 0.0;
  double chassis_turn_norm_ = 0.0;
  double chassis_left_track_ = 0.0;
  double chassis_right_track_ = 0.0;
  std::string camera_connection_state_ = "DISCONNECTED";
  std::string camera_app_state_ = "DISCONNECTED";
  std::string camera_track_state_ = "DISCONNECTED";
  std::string camera_protocol_status_ = "waiting for camera node";
  bool camera_target_found_ = false;
  bool camera_can_scan_ = false;
  std::string camera_target_pixel_ = "-1,-1";

  std::vector<std::string> main_labels_ = {
    "ARM", "VISION_TASK", "CHASSIS", "POLE", "IDLE", "BALL", "CHASSIS", "ARM"};
  std::vector<std::string> sub_labels_ = {
    "-", "-", "-", "-", "-", "-", "-", "-"};
};

class MenuUiNode : public rclcpp::Node
{
public:
  explicit MenuUiNode(MenuUiWidget * widget)
  : Node("menu_ui_node"), widget_(widget)
  {
    const double max_linear_speed = declare_parameter<double>("chassis.max_linear_speed", 0.5);
    const double max_angular_speed = declare_parameter<double>("chassis.max_angular_speed", 1.0);
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    if (widget_) {
      widget_->setChassisSpeedLimits(max_linear_speed, max_angular_speed);
    }

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

        if (!widget_->isLbSubmenuAllowed()) {
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

    chassis_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd/chassis/velocity", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateChassisVelocity(msg->linear.x, msg->angular.z);
      });

    camera_connection_sub_ = create_subscription<std_msgs::msg::String>(
      "/status/camera/connection", status_qos,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraConnectionState(msg->data);
      });

    camera_app_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/status/camera/vision_app_state", status_qos,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraAppState(msg->data);
      });

    camera_track_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/status/camera/vision_track_state", status_qos,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraTrackState(msg->data);
      });

    camera_protocol_sub_ = create_subscription<std_msgs::msg::String>(
      "/status/camera/protocol", status_qos,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraProtocolStatus(msg->data);
      });

    camera_target_found_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/status/camera/target_found", status_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraTargetFound(msg->data);
      });

    camera_can_scan_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/status/camera/can_scan", status_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraCanScan(msg->data);
      });

    camera_target_pixel_sub_ = create_subscription<std_msgs::msg::String>(
      "/status/camera/target_pixel", status_qos,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!msg || !widget_) {
          return;
        }
        widget_->updateCameraTargetPixel(msg->data);
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_connection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_app_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_track_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_protocol_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_target_found_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_can_scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_target_pixel_sub_;
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
