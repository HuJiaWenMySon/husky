#ifndef QUADRUPED_ROBOTICS_SINGLE_LEG_H_
#define QUADRUPED_ROBOTICS_SINGLE_LEG_H_


#include "utils/math.h"



namespace quadruped
{

/**
 * @brief 腿的编号
 * 
 */
#define FOWARD_RIGHT 0
#define FOWARD_LEFT 1
#define REAR_RIGHT 2
#define REAR_LEFT 3


/**
 * @brief 单腿运动学解算的工具
 * 
 */
class SingleLeg
{
private:
    int leg_id_;
    
    float abad_link_length_;   
    float hip_link_length_;    
    float knee_link_length_;   

    int side_sign_;
    Vec3 p_hip_to_base_;

    Mat3 CalJacobian(const Vec3& q);

public:
    SingleLeg(int leg_id, float abad_link_length, float hip_link_length, float knee_link_length,Vec3 p_hip_to_base);
    Vec3 CalFootToHipPosition(const Vec3& q);  
    Vec3 CalFootToBasePosition(const Vec3& q);
    Vec3 CalFootToBaseVelocity(const Vec3& q, const Vec3& qd);
    Vec3 CalQ(const Vec3& p_foot_to_base);
    Vec3 CalQd(const Vec3& q,const Vec3& v_foot_to_base);
    Vec3 CalTau(const Vec3& q,const Vec3& force);
};


}//namespace quadruped




#endif//QUADRUPED_ROBOTICS_SINGLE_LEG_H_