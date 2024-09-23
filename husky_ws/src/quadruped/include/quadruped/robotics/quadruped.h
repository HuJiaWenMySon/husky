#ifndef QUADRUPED_ROBOTICS_QUADRUPED_H_
#define QUADRUPED_ROBOTICS_QUADRUPED_H_


#include "robotics/single_leg.h"
#include <vector>


namespace quadruped
{


/**
 * @brief 机器人结构参数
 * 
 */
struct QuadrupedParam
{
    Mat34 hip_offset;
    float abad_link_length;
    float hip_link_length;
    float knee_link_length;
};



class Quadruped
{
private:
    std::vector<SingleLeg*> legs_;
    std::pair<float,float> vel_limit_x_;
    std::pair<float,float> vel_limit_y_;
    std::pair<float,float> vel_limit_yaw_;

    Mat34 hip_offset_;
    float abad_link_length_;
    float hip_link_length_;
    float knee_link_length_;

public:
    Quadruped(QuadrupedParam param);
    ~Quadruped();
    Vec12 GetQ(const Mat34 & p_foot_to_base);
    Vec12 GetQd(const Mat34 & p_foot_to_base, const Mat34 & v_foot_to_base);
    Vec12 GetTau(const Vec12& q, const Mat34 feet_force);
    Vec3 GetFootToBasePosition(const Mat34 & q, int id);//获取单腿的足端位置
    Vec3 GetFootToBaseVelocity(const Mat34 & q, const Mat34 & qd, int id);
    Mat34 GetFootToBasePositions(const Mat34 & q);
    Mat34 GetFootToBaseVelocities(const Mat34 & q, const Mat34 & qd);
};



}//namespace quadruped




#endif//QUADRUPED_ROBOTICS_QUADRUPED_H_