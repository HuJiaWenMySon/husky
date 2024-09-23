#ifndef QUADRUPED_ROBOT_PARAM_H_
#define QUADRUPED_ROBOT_PARAM_H_

#include <vector>
#include <string>

namespace quadruped
{


/**
 * @brief 机器人的通用参数
 * 
 */
struct RobotParam
{
    std::string name;                               //机器人名字
    bool is_sim;                                    //是否仿真

    //结构数据
    float abad_link_length;
    float hip_link_length;
    float knee_link_length;
    Mat34 hip_offset;                       //髋关节的偏移

    //状态估计
    float accelerometer_variance;           //加速度计的协方差，用作过程误差Q
    float sensor_variance;                  //观测变量的协方差R
    float initial_variance;                 //初始的协方差P

    Mat34 foot_positions_normal;

    //电机参数
    std::vector<float> fixed_stand_q;       //固定关节角度
    int fixed_stand_duration;               //经过多少个周期站起来

    //步态
    float period;                           //步态周期
    float stance_phase_ratio;               //处于站立相位的时间
    Vec4 bias;                              //初始的偏置
    float gait_height;                      //抬腿高度

    Vec2 v_x_limit;                         //质心速度限幅(x方向)
    Vec2 v_y_limit;                         //质心速度限幅(y方向)
    Vec2 v_yaw_limit;                       //质心角速度限幅(yaw方向)

    //MPC
    std::vector<float> mpc_q;               //MPC Q矩阵的trace 
    float total_mass;                       //总质量
    std::vector<float> total_inertia;       //总惯性张量
};



}//namespace quadruped





#endif//QUADRUPED_ROBOT_PARAM_H_