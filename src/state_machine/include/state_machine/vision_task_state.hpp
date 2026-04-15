#pragma once

#include <array>

#include "state_machine/robot_state.hpp"

class VisionTaskState : public RobotState
{
public:
    enum class EnergyMode : uint8_t {
        Small = 0,
        Big = 1,
    };

    std::string getName() const override;
    uint8_t getStateEnum() const override;
    uint8_t getSubState() const override;

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

private:
    void refreshMainUi(RobotStateMachineNode *context);
    void refreshSubmenuUi(RobotStateMachineNode *context);
    static std::string energyModeToToken(EnergyMode mode);
    bool applyEnergyModeSelection(RobotStateMachineNode *context);

    std::array<double, 5> target_pwms_{};
    std::array<double, 5> tracking_start_pwms_{};
    bool tracking_started_{false};
    bool has_tracking_start_pwms_{false};
    bool precision_mode_{false};
    bool submenu_active_{false};
    int submenu_selection_{0};
    EnergyMode energy_mode_{EnergyMode::Big};
    int right_y_selected_servo_{2};
    bool dpad_switch_latched_{false};
};
