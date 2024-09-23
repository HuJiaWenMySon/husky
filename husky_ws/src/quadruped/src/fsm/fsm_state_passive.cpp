#include "fsm/fsm_state_passive.h"


namespace quadruped
{



FsmStatePassive::FsmStatePassive(ControlFsmData* data)
    :FsmState(data)
{
    state_name_ = FsmStateName::PASSIVE;
}



void FsmStatePassive::Enter()
{
    LOG_INFO("FSM: Enter Passive State.");
    data_->robot->wave_status_ = WaveStatus::SWING_ALL;//passive状态腿没有支撑
    transition_data_.Zero();
}


/**
 * @brief 阻尼模式下，设置电机参数
 * 
 */
void FsmStatePassive::Run()
{
    // LOG_INFO("FSM: Run in Passive State.");
    for(int i = 0; i < 12; i++)
    {
        data_->robot->joint_command_angles_[i] = 0.0f;
        data_->robot->joint_command_velocities_[i] = 0.0f;
        data_->robot->joint_command_kp_[i] = 0.0f;
        data_->robot->joint_command_kd_[i] = 8.0f;
        data_->robot->joint_command_torque_[i] = 0.0f;
    }
}



void FsmStatePassive::Exit()
{
    //一般不通过软件退出阻尼模式，直接下电退出，因此这里什么都不用做
}


/**
 * @brief 根据传入的状态名称，检查状态转换是否能够进行
 * 
 * @param desired_state_name 期望的状态名称
 * @return FsmStateName 下一个可运行的状态，如果能够转换，就是期望状态，否则保持自身状态
 */
FsmStateName FsmStatePassive::CheckTransition(FsmStateName desired_state_name)
{
    FsmStateName next_state_name = state_name_;
  
    switch (desired_state_name)
    {
    case FsmStateName::FIXED_STAND://阻尼模式允许切换到固定站立模式
    case FsmStateName::PASSIVE:
        next_state_name = desired_state_name;
        break;
    default:
        LOG_WARN("Bad Request: Cannot transition from %d to %d",state_name_,desired_state_name);
        break;
    }

    return next_state_name;
}




TransitionData FsmStatePassive::Transition(FsmStateName next_state_name)
{
    transition_data_.done = true;
    return transition_data_;
}



}//namespace quadruped