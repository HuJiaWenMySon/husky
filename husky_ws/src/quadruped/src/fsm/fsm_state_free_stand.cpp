#include "fsm/fsm_state_free_stand.h"

#define MAX_ROLL_ANGLE 1.57f
#define MAX_PITCH_ANGLE 1.57f
#define MAX_YAW_ANGLE 1.57f

namespace quadruped
{


FsmStateFreeStand::FsmStateFreeStand(ControlFsmData* data)
    :FsmState(data)    
{
    state_name_ = FsmStateName::FREE_STAND;

    robot_ = data->robot;
    gait_ = new GaitGenerator(robot_,data->wave_generator);
    stance_controller_ = new StanceLegController(robot_,gait_);

    swing_kp_ = Vec3(300, 300, 300).asDiagonal();
    swing_kd_ = Vec3(10, 10, 10).asDiagonal();

    desired_state_command_ = data->desired_state_command;
}



void FsmStateFreeStand::Enter()
{
    LOG_INFO("FSM: Enter Free Stand State.");
    data_->robot->wave_status_ = WaveStatus::STANCE_ALL;
    gait_->Restart();

    base_position_desired_in_world_frame_ = robot_->base_position_;     //期望的位置初始化为当前位置
}



void FsmStateFreeStand::Run()
{
    if(gait_->IsSwitching()) return;

    SetupCommand();                

    gait_->SetGait(Vec2(base_velocity_desired_in_world_frame_[0],base_velocity_desired_in_world_frame_[1]),yaw_turn_rate_,robot_->param_.gait_height);
    gait_->Update(foot_positions_desired_in_world_frame_,foot_velocities_desired_in_world_frame_);

    stance_controller_->Update();       //计算支撑腿的足端反力
    foot_force_in_base_frame_ = stance_controller_->GetFootForceInBase(); //获取MPC计算的反力
    tau_ = robot_->quadruped_->GetTau(robot_->joint_state_angles_,foot_force_in_base_frame_);   //计算关节的扭矩

    //下发到关节，纯力控
    for(int i = 0; i < 12; i++)
    {
        data_->robot->joint_command_torque_[i] = tau_[i];
    }
}



void FsmStateFreeStand::Exit()
{
    
}



FsmStateName FsmStateFreeStand::CheckTransition(FsmStateName desired_state_name)
{
    FsmStateName next_state_name = state_name_;
  
    switch (desired_state_name)
    {
    case FsmStateName::FIXED_STAND:
    case FsmStateName::LOCOMOTION:
    case FsmStateName::FREE_STAND:
        next_state_name = desired_state_name;
        break;
    default:
        LOG_WARN("Bad Request: Cannot transition from %d to %d",state_name_,desired_state_name);
        break;
    }

    return next_state_name;
}


TransitionData FsmStateFreeStand::Transition(FsmStateName next_state_name)
{
    transition_data_.done = true;
    return transition_data_;
}



void FsmStateFreeStand::SetupCommand()
{
    float x_vel_cmd, y_vel_cmd;         //x、y方向的线速度
    float filter = 0.1;                 //滤波器系数
    float x_vel_desired = 0.f;          //期望速度x
    float y_vel_desired = 0.f;          //期望速度y，这个中间变量拿来对速度滤波


    x_vel_desired = 0.0f;                            //低通滤波
    y_vel_desired = 0.0f;                            
    Vec3 base_velocity_desired_in_base_frame(x_vel_desired, y_vel_desired, 0);              //基坐标系下的机身期望速度

    Mat3 Rwb = QuatToRotMat(robot_->base_orientation_);                                     //基座到世界坐标系的旋转矩阵
    
    base_velocity_desired_in_world_frame_ = Rwb * base_velocity_desired_in_base_frame;      //世界坐标系下的机身期望速度
    base_velocity_desired_in_world_frame_[2] = 0.0f;

    base_position_desired_in_world_frame_[0] = Saturation(base_position_desired_in_world_frame_[0]+robot_->dt_*base_velocity_desired_in_world_frame_[0],
                    Vec2(robot_->base_position_[0] - 0.05, robot_->base_position_[0] + 0.05));
    base_position_desired_in_world_frame_[1] = Saturation(base_position_desired_in_world_frame_[1]+robot_->dt_*base_velocity_desired_in_world_frame_[1],
                Vec2(robot_->base_position_[1] - 0.05, robot_->base_position_[1] + 0.05));


    yaw_desired_ = MAX_YAW_ANGLE*desired_state_command_->right_stick_[0];                          
    roll_desired_ = MAX_ROLL_ANGLE*desired_state_command_->left_stick_[0];
    pitch_desired_ = MAX_PITCH_ANGLE*desired_state_command_->left_stick_[1];

    robot_->motion_command_.base_position_desired_in_world_frame = base_position_desired_in_world_frame_;
    robot_->motion_command_.base_velocity_desired_in_world_frame = base_velocity_desired_in_world_frame_;
    robot_->motion_command_.yaw_desired = yaw_desired_;
    robot_->motion_command_.yaw_turn_rate = yaw_turn_rate_;
    robot_->motion_command_.roll_desired = roll_desired_;
    robot_->motion_command_.pitch_desired = pitch_desired_;
}



}//namespace quadruped