#include "fsm/fsm_state_fixed_stand.h"
#include <ros/ros.h>


namespace quadruped
{


FsmStateFixedStand::FsmStateFixedStand(ControlFsmData* data)
    :FsmState(data)
{
    state_name_ = FsmStateName::FIXED_STAND;

    fixed_stand_q_ = data->robot->param_.fixed_stand_q;
    duration_ = data->robot->param_.fixed_stand_duration;
    fixed_stand_kp_ = {180,180,300,180,180,300,180,180,300,180,180,300};
    fixed_stand_kd_ = {8,8,15,8,8,15,8,8,15,8,8,15};
    start_q_.resize(12,0.0);
}



void FsmStateFixedStand::Enter()
{
    LOG_INFO("FSM: Enter Fixed Stand State.");
    transition_data_.Zero();
    data_->robot->wave_status_ = WaveStatus::STANCE_ALL;
    for(int i = 0;i < 12;i++)
    {
        start_q_[i] = data_->robot->joint_state_angles_[i];
        // LOG_INFO("Motor %d start angle %f.",i,start_q_[i]);
    }
    percent_ = 0;
}



void FsmStateFixedStand::Run()
{
    //LOG_INFO("FSM: Run in Fix Stand State.");
    percent_ += (float)1/duration_;//计算本次更新的占比
    percent_ = percent_ > 1 ? 1:percent_;//如果大于一不再增加，保证最终达到指定的点
    for(int i = 0; i < 12; i++)
    {
        data_->robot->joint_command_angles_[i] = (1-percent_)*start_q_[i]+percent_*fixed_stand_q_[i];
        data_->robot->joint_command_velocities_[i] = 0.0f;
        data_->robot->joint_command_kp_[i] = fixed_stand_kp_[i];
        data_->robot->joint_command_kd_[i] = fixed_stand_kd_[i];
        data_->robot->joint_command_torque_[i] = 0.0f;
    }
}



void FsmStateFixedStand::Exit()
{
    percent_ = 0;
}



FsmStateName FsmStateFixedStand::CheckTransition(FsmStateName desired_state_name)
{
    FsmStateName next_state_name = state_name_;
  
    switch (desired_state_name)
    {
    case FsmStateName::PASSIVE:
    case FsmStateName::LOCOMOTION:
    case FsmStateName::FREE_STAND:
    case FsmStateName::FIXED_STAND:
        next_state_name = desired_state_name;
        break;
    default:
        LOG_WARN("Bad Request: Cannot transition from %d to %d",state_name_,desired_state_name);
        break;
    }

    return next_state_name;
}


TransitionData FsmStateFixedStand::Transition(FsmStateName next_state_name)
{
    transition_data_.done = true;
    return transition_data_;
}



}//namespace quadruped