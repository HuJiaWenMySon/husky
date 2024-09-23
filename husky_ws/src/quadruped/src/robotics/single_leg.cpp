#include "robotics/single_leg.h"
#include "utils/logger.h"


namespace quadruped
{


Mat3 SingleLeg::CalJacobian(const Vec3& q)
{
    Mat3 jaco;

    //连杆的长度
    float l1 = side_sign_ * abad_link_length_;
    float l2 = -hip_link_length_;
    float l3 = -knee_link_length_;

    //列出关节角的sin和cos值，方便后续的计算
    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;//cos(2+3)
    float s23 = s2 * c3 + c2 * s3;//sin(2+3)

    //构造Jacobian，参见宇数教程72页
    jaco(0, 0) = 0;
    jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jaco(0, 1) = l3 * c23 + l2 * c2;
    jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    jaco(0, 2) = l3 * c23;
    jaco(1, 2) = l3 * s1 * s23;
    jaco(2, 2) = -l3 * c1 * s23;

    return jaco;
}


SingleLeg::SingleLeg(int leg_id, float abad_link_length, float hip_link_length, float knee_link_length, Vec3 p_hip_to_base)
{
    leg_id_ = leg_id;

    abad_link_length_ = abad_link_length;
    hip_link_length_ = hip_link_length;
    knee_link_length_ = knee_link_length;

    p_hip_to_base_ = p_hip_to_base;

    if(leg_id_ == FOWARD_LEFT || leg_id_ == REAR_LEFT)
    {
        side_sign_ = 1;
    }
    else
    {
        side_sign_ = -1;
    }

    LOG_INFO("Leg: %d initialize success. abad length %f, hip link length %f, knee link length %f, hip vector:(%f,%f,%f)",leg_id_,abad_link_length_,hip_link_length_,knee_link_length_,
                                p_hip_to_base_[0],p_hip_to_base_[1],p_hip_to_base_[2]);
}


/**
 * @brief 正运动学
 * 
 * @param q 关节角
 * @return Vec3 足端的向量，相对于HIP点
 */
Vec3 SingleLeg::CalFootToHipPosition(const Vec3& q)
{
    Vec3 p_foot_to_hip;

    float l1 = side_sign_ * abad_link_length_;
    float l2 = -hip_link_length_;
    float l3 = -knee_link_length_;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    p_foot_to_hip(0) = l3 * s23 + l2 * s2;
    p_foot_to_hip(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    p_foot_to_hip(2) =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return p_foot_to_hip;
}


/**
 * @brief 正运动学
 * 
 * @param q 关节角
 * @return Vec3 足端的向量，相对于BASE点
 */
Vec3 SingleLeg::CalFootToBasePosition(const Vec3& q)
{
    return p_hip_to_base_ + CalFootToHipPosition(q);
}


Vec3 SingleLeg::CalFootToBaseVelocity(const Vec3& q, const Vec3& qd)
{
    return CalJacobian(q)*qd; //Jacobian乘转速可以得到足端速度
}



Vec3 SingleLeg::CalQ(const Vec3& p_foot_to_base)
{
    Vec3 q_result;         //返回结果
    Vec3 p_foot_to_hip;    //足端到髋关节的位置矢量，用于逆运动学计算

    p_foot_to_hip = p_foot_to_base - p_hip_to_base_;  //足端相对于机身的位置 减去 髋关节想对于机身的位置

    float px = p_foot_to_hip [0];
    float py = p_foot_to_hip [1];
    float pz = p_foot_to_hip [2];

    float l1 = side_sign_ * abad_link_length_;
    float l2 = -hip_link_length_;
    float l3 = -knee_link_length_;

    float q1,q2,q3;

    //计算Q1的逆解
    float L = std::sqrt(py*py+pz*pz-l1*l1);            //L是大腿、小腿连杆在ZOY平面上的投影，
    q1 = std::atan2(pz*l1+py*L,py*l1-pz*L);    
    
    //计算Q3的逆解
    float ap = std::sqrt(px*px+py*py+pz*pz-l1*l1);     //AP是大腿小腿构成的平面内大腿关节到足端的长度
    float temp = (l2*l2+l3*l3-ap*ap)/(2*std::fabs(l2*l3)); 
    if(temp > 1) temp = 1;
    if(temp < -1) temp = -1;

    q3 = -M_PI + std::acos(temp);

    //计算Q2的逆解
    float a1 = py*std::sin(q1)-pz*cos(q1);
    float a2 = px;
    float m1 = l3*std::sin(q3);
    float m2 = l3*std::cos(q3)+l2;

    q2 = atan2(a1*m1+a2*m2,a2*m1-a1*m2);

    q_result << q1,q2,q3;
    return q_result;
}


Vec3 SingleLeg::CalQd(const Vec3& q, const Vec3& v_foot_to_base)
{
    return CalJacobian(q).inverse()*v_foot_to_base;//四足机器人关节限位下雅可比是可逆的
}


Vec3 SingleLeg::CalTau(const Vec3& q, const Vec3& force)
{
    return CalJacobian(q).transpose() * force;
}



}//namespace quadruped