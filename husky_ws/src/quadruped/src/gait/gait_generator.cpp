#include "gait/gait_generator.h"
#include "utils/logger.h"


namespace quadruped
{


GaitGenerator::GaitGenerator(Robot* robot, WaveGenerator* wave_generator)
{
    foot_end_cal_ = new FootEndCal(robot);
    wave_generator_ = wave_generator;
    first_run_ = true;
    robot_ = robot;

    t_stance_ = robot_->param_.period * robot_->param_.stance_phase_ratio;
    t_swing_ = robot_->param_.period * (1 - robot_->param_.stance_phase_ratio);
}


GaitGenerator::~GaitGenerator()
{
    delete foot_end_cal_;
}


void GaitGenerator::SetGait(Vec2 v_xy_goal, float v_yaw_goal, float gait_height)
{
    v_xy_goal_ = v_xy_goal;
    v_yaw_goal_ = v_yaw_goal;
    gait_height_ = gait_height;
}


void GaitGenerator::Restart()
{
    first_run_ = true;
    v_xy_goal_.setZero();//初始处于静止状态
}


void GaitGenerator::Update(Mat34& foot_positions, Mat34& foot_velocities)
{
    if(first_run_)
    {
        start_position_ = robot_->foot_positions_in_world_frame_;
        first_run_ = false;
    }

    for(int i = 0; i < 4; i++)
    {
        if(robot_->leg_state_[i] == LegState::STANCE)//支撑腿
        {
            if(robot_->phase_normalized_[i] < 0.5)
            {
                start_position_.col(i) = robot_->foot_positions_in_world_frame_.col(i);//初始位置
            }
            foot_positions.col(i) = start_position_.col(i);//支撑腿的位置不动，保持初始位置
            foot_velocities.col(i) << 0.0,0.0,0.0;//支撑腿的速度当然为0
        }
        else//摆动腿
        {
            end_position_.col(i) = foot_end_cal_->CalFootPosition(i,v_xy_goal_,v_yaw_goal_,robot_->phase_normalized_[i]);//根据期望的速度和当前相位，计算落脚点
            foot_positions.col(i) = GetFootPosition(i);
            foot_velocities.col(i) = GetFootVelocity(i);
        }
    }
}


Vec3 GaitGenerator::GetFootPosition(int leg_id)
{
    Vec3 foot_position;
    foot_position[0] = CycloidXyPosition(start_position_.col(leg_id)(0),end_position_.col(leg_id)(0),robot_->phase_normalized_(leg_id));
    foot_position[1] = CycloidXyPosition(start_position_.col(leg_id)(1),end_position_.col(leg_id)(1),robot_->phase_normalized_(leg_id));
    foot_position[2] = CycloidZPosition(start_position_.col(leg_id)(2),gait_height_,robot_->phase_normalized_(leg_id));//计算Z坐标

    return foot_position;
}


Vec3 GaitGenerator::GetFootVelocity(int leg_id)
{
    Vec3 foot_velocity;
    foot_velocity[0] = CycloidXyVelocity(start_position_.col(leg_id)(0),end_position_.col(leg_id)(0),robot_->phase_normalized_(leg_id));
    foot_velocity[1] = CycloidXyVelocity(start_position_.col(leg_id)(1),end_position_.col(leg_id)(1),robot_->phase_normalized_(leg_id));
    foot_velocity[2] = CycloidZVelocity(gait_height_,robot_->phase_normalized_(leg_id));//计算Z坐标
    return foot_velocity;
}


float GaitGenerator::CycloidXyPosition(float start, float end, float phase)
{
    float phase_pi = 2 * M_PI * phase;
    return (end - start)*(phase_pi - sin(phase_pi))/(2*M_PI) + start;
}


float GaitGenerator::CycloidXyVelocity(float start, float end, float phase)
{
    float phase_pi = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phase_pi)) / t_swing_;
}


 
float GaitGenerator::CycloidZPosition(float start, float height, float phase)
{
    float phase_pi = 2 * M_PI * phase;
    return height*(1 - cos(phase_pi))/2 + start;
}


float GaitGenerator::CycloidZVelocity(float height, float phase)
{
    float phase_pi = 2 * M_PI * phase;
    return height * M_PI * sin(phase_pi) / t_swing_;
}


}//namespace quadruped