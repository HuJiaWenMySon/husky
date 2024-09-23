#include "robotics/quadruped.h"
#include "utils/logger.h"

namespace quadruped
{

Quadruped::Quadruped(QuadrupedParam param)
{
    abad_link_length_ = param.abad_link_length;
    hip_link_length_ = param.hip_link_length;
    knee_link_length_ = param.knee_link_length;

    hip_offset_ = param.hip_offset;

    for(int i = 0; i < 4; i++)
    {
        legs_.push_back(new SingleLeg(i,abad_link_length_,hip_link_length_,knee_link_length_,hip_offset_.col(i)));
    }
}


Quadruped::~Quadruped()
{
    for(int i = 0; i < 4; i++)
    {
        delete legs_[i];
    }
}


Vec12 Quadruped::GetQ(const Mat34 & p_foot_to_base)
{
    Vec12 q;
    for(int i = 0; i < 4; i++)
    {
        q.segment(3*i,3) = legs_[i]->CalQ(p_foot_to_base.col(i)); //对每条腿计算关节角
    }
    return q;
}



Vec12 Quadruped::GetQd(const Mat34 & p_foot_to_base, const Mat34 & v_foot_to_base)
{
    Vec12 qd;
    Vec12 q = GetQ(p_foot_to_base);
    for(int i = 0; i < 4; i++)
    {
        qd.segment(3*i,3) = legs_[i]->CalQd(q.segment(i*3,3),v_foot_to_base.col(i));
    }
    return qd;
}



Vec12 Quadruped::GetTau(const Vec12& q, const Mat34 feet_force)
{
    Vec12 tau;
    for(int i = 0; i < 4; i++)
    {
        tau.segment(3*i,3) = legs_[i]->CalTau(q.segment(3*i,3),feet_force.col(i));
    }
    return tau;
}



Vec3 Quadruped::GetFootToBasePosition(const Mat34 & q, int id)
{

    return legs_[id]->CalFootToBasePosition(q.col(id));
}


Vec3 Quadruped::GetFootToBaseVelocity(const Mat34 & q, const Mat34 & qd, int id)
{
    return legs_[id]->CalFootToBaseVelocity(q.col(id),qd.col(id));
}


Mat34 Quadruped::GetFootToBasePositions(const Mat34 & q)
{
    Mat34 p_foot_to_base;
    for(int i = 0; i < 4 ; i++)
    {
        p_foot_to_base.col(i) = GetFootToBasePosition(q,i);
    }
    return p_foot_to_base;
}


Mat34 Quadruped::GetFootToBaseVelocities(const Mat34 & q, const Mat34 & qd)
{
    Mat34 v_foot_to_base;
    for(int i = 0; i < 4 ; i++)
    {
        v_foot_to_base.col(i) = GetFootToBaseVelocity(q,qd,i);
    }
    return v_foot_to_base;
}



}//namespace quadruped

