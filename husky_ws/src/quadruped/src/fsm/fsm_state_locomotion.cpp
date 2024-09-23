#include "fsm/fsm_state_locomotion.h"
#include "utils/logger.h"

//TODO: add this to ros param
#define MAX_VELOCITY_X     1.0
#define MAX_VELOCITY_Y     1.0
#define MAX_YAW_TURN_RATE  1.0

namespace quadruped
{


FsmStateLocomotion::FsmStateLocomotion(ControlFsmData* data)
    :FsmState(data)
{
    state_name_ = FsmStateName::LOCOMOTION;

    robot_ = data->robot;
    gait_ = new GaitGenerator(robot_,data->wave_generator);
    stance_controller_ = new StanceLegController(robot_,gait_);

    swing_kp_ = Vec3(300, 300, 300).asDiagonal();
    swing_kd_ = Vec3(10, 10, 10).asDiagonal();

    desired_state_command_ = data->desired_state_command;
}


FsmStateLocomotion::~FsmStateLocomotion()
{
    delete gait_;
}


void FsmStateLocomotion::Enter()
{
    LOG_INFO("FSM: Enter Locomotion State.");
    robot_->wave_status_ = WaveStatus::WAVE_ALL;
    gait_->Restart();

    base_position_desired_in_world_frame_ = robot_->base_position_;     //期望的位置初始化为当前位置
}


void FsmStateLocomotion::Run()
{
    if(gait_->IsSwitching()) return;

    SetupCommand();                

    gait_->SetGait(Vec2(base_velocity_desired_in_world_frame_[0],base_velocity_desired_in_world_frame_[1]),yaw_turn_rate_,robot_->param_.gait_height);
    gait_->Update(foot_positions_desired_in_world_frame_,foot_velocities_desired_in_world_frame_);
    
    // LOG_INFO("%d,%d,%d,%d",robot_->leg_state_[0],robot_->leg_state_[1],robot_->leg_state_[2],robot_->leg_state_[3]);
    // LOG_INFO("leg 1 pos desired: %f %f %f.",foot_positions_desired_in_world_frame_(0,1),foot_positions_desired_in_world_frame_(1,1),foot_positions_desired_in_world_frame_(2,1));

    stance_controller_->Update();       //计算支撑腿的足端反力
    foot_force_in_base_frame_ = stance_controller_->GetFootForceInBase(); //获取MPC计算的反力

    Mat3 Rbw = QuatToRotMat(robot_->base_orientation_).transpose();     //获取世界到基座的转换  

    //摆动腿使用PD控制
    for(int i = 0; i < 4; i++)
    {
        if(robot_->leg_state_[i] == LegState::SWING)
        {
            foot_force_in_base_frame_.col(i) = Rbw*(swing_kp_*(foot_positions_desired_in_world_frame_.col(i) - robot_->foot_positions_in_world_frame_.col(i))
                                                 + swing_kd_*(foot_velocities_desired_in_world_frame_.col(i) - robot_->foot_velocities_in_world_frame_.col(i)));
        }
    }

    tau_ = robot_->quadruped_->GetTau(robot_->joint_state_angles_,foot_force_in_base_frame_);   //计算关节的扭矩

    //计算关节位置和速度
    Mat34 foot_to_base_positions_in_base_frame_ = robot_->foot_to_base_positions_in_base_frame_;

    for(int i = 0; i < 4; i++)
    {
        foot_to_base_positions_desired_in_base_frame_.col(i) = Rbw * (foot_positions_desired_in_world_frame_.col(i) - robot_->base_position_);
        foot_to_base_velocities_desired_in_base_frame_.col(i) = Rbw * (foot_velocities_desired_in_world_frame_.col(i) - robot_->base_velocity_in_world_frame_); 
    } 

    q_ = robot_->quadruped_->GetQ(foot_to_base_positions_desired_in_base_frame_);
    qd_ = robot_->quadruped_->GetQd(foot_to_base_positions_in_base_frame_,foot_to_base_velocities_desired_in_base_frame_);

    //下发到关节
    for(int i = 0; i < 12; i++)
    {
        int leg_id = i/3;
        if(robot_->leg_state_[leg_id] == LegState::STANCE)  
        {
            data_->robot->joint_command_kp_[i] = 3.0;
            data_->robot->joint_command_kd_[i] = 2.0;
        }
        else
        {
            data_->robot->joint_command_kp_[i] = 10.0;
            data_->robot->joint_command_kd_[i] = 1.0;
        }
        data_->robot->joint_command_angles_[i] = q_[i];
        data_->robot->joint_command_velocities_[i] = qd_[i];
        data_->robot->joint_command_torque_[i] = tau_[i];
    }

}


void FsmStateLocomotion::Exit()
{
    
}


FsmStateName FsmStateLocomotion::CheckTransition(FsmStateName desired_state_name)
{
    FsmStateName next_state_name = state_name_;
  
    switch (desired_state_name)
    {
    case FsmStateName::FIXED_STAND:
    case FsmStateName::LOCOMOTION:
        next_state_name = desired_state_name;
        break;
    default:
        LOG_WARN("Bad Request: Cannot transition from %d to %d",state_name_,desired_state_name);
        break;
    }

    return next_state_name;
}


TransitionData FsmStateLocomotion::Transition(FsmStateName next_state_name)
{
    transition_data_.done = true;
    return transition_data_;
}


void FsmStateLocomotion::SetupCommand()
{
    float x_vel_cmd, y_vel_cmd;         //x、y方向的线速度
    float filter = 0.1;                 //滤波器系数
    float x_vel_desired = 0.f;          //期望速度x
    float y_vel_desired = 0.f;          //期望速度y，这个中间变量拿来对速度滤波

    x_vel_cmd = MAX_VELOCITY_X*desired_state_command_->left_stick_[1];             //左摇杆的纵轴
    y_vel_cmd = MAX_VELOCITY_Y*desired_state_command_->left_stick_[0];             //左摇杆的横轴
    yaw_turn_rate_ = MAX_YAW_TURN_RATE*desired_state_command_->right_stick_[0];       //右摇杆的横轴

    // x_vel_cmd = -0.4;                                                   //测试用数据
    // y_vel_cmd = -0.4;
    // yaw_turn_rate_ = 0.0;

    x_vel_desired = x_vel_desired*(1-filter) + x_vel_cmd*filter;                            //低通滤波
    y_vel_desired = y_vel_desired*(1-filter) + y_vel_cmd*filter;                            
    Vec3 base_velocity_desired_in_base_frame(x_vel_desired, y_vel_desired, 0);              //基坐标系下的机身期望速度

    Mat3 Rwb = QuatToRotMat(robot_->base_orientation_);                                     //基座到世界坐标系的旋转矩阵
    
    base_velocity_desired_in_world_frame_ = Rwb * base_velocity_desired_in_base_frame;      //世界坐标系下的机身期望速度
    base_velocity_desired_in_world_frame_[2] = 0.0f;

    base_position_desired_in_world_frame_[0] = Saturation(base_position_desired_in_world_frame_[0]+robot_->dt_*base_velocity_desired_in_world_frame_[0],
                    Vec2(robot_->base_position_[0] - 0.05, robot_->base_position_[0] + 0.05));
    base_position_desired_in_world_frame_[1] = Saturation(base_position_desired_in_world_frame_[1]+robot_->dt_*base_velocity_desired_in_world_frame_[1],
                Vec2(robot_->base_position_[1] - 0.05, robot_->base_position_[1] + 0.05));


    yaw_desired_ = yaw_desired_ + robot_->dt_*yaw_turn_rate_;                               //期望的Yaw角度，限幅在[-pi,pi]内
    if (yaw_desired_ >= M_PI) 
    {
        yaw_desired_ -= 2*M_PI;
    } 
    else if (yaw_desired_ <= -M_PI) 
    {
        yaw_desired_ += 2*M_PI;
    }

    robot_->motion_command_.base_position_desired_in_world_frame = base_position_desired_in_world_frame_;
    robot_->motion_command_.base_velocity_desired_in_world_frame = base_velocity_desired_in_world_frame_;
    robot_->motion_command_.yaw_desired = yaw_desired_;
    robot_->motion_command_.roll_desired = 0.0f;
    robot_->motion_command_.pitch_desired = 0.0f;
    robot_->motion_command_.yaw_turn_rate = yaw_turn_rate_;
}


}//namespace quadruped