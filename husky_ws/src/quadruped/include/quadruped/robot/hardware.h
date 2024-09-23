#ifndef QUADRUPED_ROBOT_HARWARE_H_
#define QUADRUPED_ROBOT_HARDWARE_H_


#include <cassert>
#include <stdint.h>

namespace quadruped
{


//电机相关硬件
#define MOTOR_ID_FR_ABAD 0
#define MOTOR_ID_FR_HIP 1
#define MOTOR_ID_FR_KNEE 2
#define MOTOR_ID_FL_ABAD 3    
#define MOTOR_ID_FL_HIP 4       
#define MOTOR_ID_FL_KNEE 5
#define MOTOR_ID_RR_ABAD 6
#define MOTOR_ID_RR_HIP 7
#define MOTOR_ID_RR_KNEE 8
#define MOTOR_ID_RL_ABAD 9
#define MOTOR_ID_RL_HIP 10
#define MOTOR_ID_RL_KNEE 11

#define ASSERT_MOTOR_ID_VALID(motor_id)  assert((motor_id>=0)&&(motor_id<12))


/**
 * @brief 电机命令
 * 
 */
struct MotorCommand
{
    float q;
    float dq;
    float kp;
    float kd;
    float tau;
};


/**
 * @brief 电机状态
 * 
 */
struct MotorState
{
    float q;
    float dq;
    float ddq;
    float tau;
};


/**
 * @brief IMU数据
 * 
 */
struct ImuData
{
    float linear_acceleration[3];      //线性加速度
    float angular_velocity[3];         //角速度
    float quaternion[4];               //四元数  
    float rpy[3];                      //ZYX欧拉角                   
};


/**
 * @brief 三维力传感器
 * 
 */
struct ForceSensorData
{
    float force_x;
    float force_y;
    float force_z;
};


}

#endif//QUADRUPED_ROBOT_HARDWARE_H_