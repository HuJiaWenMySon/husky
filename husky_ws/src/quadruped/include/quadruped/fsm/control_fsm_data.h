#ifndef QUADRUPED_FSM_CONTROL_FSM_DATA_H_
#define QUADRUPED_FSM_CONTROL_FSM_DATA_H_

#include "controller/desired_state_command.h"
#include "estimator/state_estimator.h"
#include "gait/wave_generator.h"
#include "robot/robot.h"
#include <ros/ros.h>


namespace quadruped
{

struct ControlFsmData
{
    WaveGenerator* wave_generator;                 //波形发生器
    DesiredStateCommand* desired_state_command;     //外部指令
    Robot* robot;                                   //机器人硬件相关数据
};





}//namespace quadruped


#endif//QUADRUPED_FSM_CONTROL_FSM_DATA_H_
