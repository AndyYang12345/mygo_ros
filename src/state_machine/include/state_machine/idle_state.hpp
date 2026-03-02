#pragma once

#include "state_machine/robot_state.hpp"

class IdleState : public RobotState
{
public:
    std::string getName() const override;
    uint8_t getStateEnum() const override;

    void onEnter(RobotStateMachineNode *context) override;
};
