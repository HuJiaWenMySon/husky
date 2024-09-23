#include "gait/foot_end_cal.h"
#include "utils/logger.h"


namespace quadruped
{


FootEndCal::FootEndCal(Robot* robot)
{
    kx_ = 0.005;
    ky_ = 0.005;
    kyaw_ = 0.005;

    robot_ = robot;

    t_stance_ = robot_->param_.period * robot_->param_.stance_phase_ratio;
    t_swing_ = robot_->param_.period * (1 - robot_->param_.stance_phase_ratio);

    for(int i = 0; i < 4; i++)
    {
        float x = robot->param_.foot_positions_normal(0,i);
        float y = robot->param_.foot_positions_normal(1,i);
        foot_radius_[i] = std::sqrt(x*x+y*y);
        foot_init_angle_[i] = std::atan2(y,x);
        // LOG_INFO("foot radius[%d]: %f",i,foot_radius_[i]);
        // LOG_INFO("foot init angle[%d]: %f",i,foot_init_angle_[i]);
    }
}



Vec3 FootEndCal::CalFootPosition(int leg_id, Vec2 v_xy_goal, float v_yaw_goal, float phase)
{
    Vec3 base_velocity_in_world_frame = robot_->base_velocity_in_world_frame_;//世界坐标系下的机身速度
    Vec3 base_angular_velocity_in_world_frame = QuatToRotMat(robot_->base_orientation_)*robot_->base_rpy_rate_;//世界坐标系下的角速度
    Vec3 base_rpy = robot_->base_rpy_;

    Vec3 next_step;    //下一个落脚点与质心之间的差

    next_step[0] = base_velocity_in_world_frame[0]*(1-phase)*t_swing_ + base_velocity_in_world_frame[0]*t_stance_/2 + kx_*(base_velocity_in_world_frame[0] - v_xy_goal[0]);
    next_step[1] = base_velocity_in_world_frame[1]*(1-phase)*t_swing_ + base_velocity_in_world_frame[1]*t_stance_/2 + ky_*(base_velocity_in_world_frame[1] - v_xy_goal[1]);

    // LOG_INFO("ns1: %f, %f",next_step[0],next_step[1]);

    float yaw = base_rpy[2];
    float v_yaw = base_angular_velocity_in_world_frame[2];
    float next_yaw = v_yaw*(1 - phase)*t_swing_ + v_yaw*t_stance_/2 + kyaw_*(v_yaw_goal - v_yaw);

    // LOG_INFO("%f,%f,%f,%f,%f,%f,%f",next_yaw,v_yaw,phase,t_swing_,t_stance_,kyaw_,v_yaw_goal);
    
    next_step[0] += foot_radius_[leg_id] * std::cos(yaw + foot_init_angle_[leg_id] + next_yaw);
    next_step[1] += foot_radius_[leg_id] * std::sin(yaw + foot_init_angle_[leg_id] + next_yaw);
    // LOG_INFO("ns2: %f, %f",next_step[0],next_step[1]);

    foot_position_ = robot_->base_position_ + next_step;
    // LOG_INFO("leg %d, phase: %f.",leg_id ,phase);

    return foot_position_;
}





}