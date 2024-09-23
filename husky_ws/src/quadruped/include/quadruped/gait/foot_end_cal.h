#ifndef QUADRUPED_GAIT_FOOT_END_CAL_H_
#define QUADRUPED_GAIT_FOOT_END_CAL_H_


#include "robot/robot.h"


namespace quadruped
{

class FootEndCal
{
public:
    FootEndCal(Robot* robot);
    ~FootEndCal() = default;
    Vec3 CalFootPosition(int leg_id, Vec2 v_xy_goal, float v_yaw_goal, float phase);

private:
    Robot* robot_;      //机器人指针

    float kx_;          //Raibert启发落脚点参数
    float ky_;
    float kyaw_;

    float t_stance_;    //支撑相的持续时间
    float t_swing_;     //摆动相的持续时间

    Vec3 foot_position_;     //落脚点

    Vec4 foot_radius_;
    Vec4 foot_init_angle_;
};


}//namespace quadruped



#endif//QUADRUPED_PLANNER_FOOT_END_CAL_H_