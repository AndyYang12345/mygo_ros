#pragma once

#include "state_machine/robot_state.hpp"
/*
    @brief 机器人空闲状态，默认状态，等待用户输入
    @param context 状态机上下文，提供对状态机节点的访问
    @param msg 各种输入消息的智能指针，包含按钮、摇杆等
*/
class IdleState : public RobotState
{
public:
    std::string getName() const override;
    uint8_t getStateEnum() const override;

    void onEnter(RobotStateMachineNode *context) override;
};
