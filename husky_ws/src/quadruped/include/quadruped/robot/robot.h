#ifndef QUADRUPED_ROBOT_ROBOT_H_
#define QUADRUPED_ROBOT_ROBOT_H_


#include "robotics/quadruped.h"
#include "robot/hardware.h"
#include "robot/param.h"
#include "utils/moving_average.h"
#include "gait/wave_generator.h"

#include <ros/ros.h>
#include <vector>


namespace quadruped
{

struct RobotMotionCommand
{
    Vec3 base_velocity_desired_in_world_frame;
    Vec3 base_position_desired_in_world_frame;
    float yaw_turn_rate;
    float yaw_desired;
    float roll_desired;
    float pitch_desired;
};


class Robot
{
public: //方法
    Robot(ros::NodeHandle& nh);                     
    virtual ~Robot();                               

    void RecvTransState();                              //外部接口，接收数据
    void SendTransCommand();                            //外部接口，发送数据

public: 
    RobotParam param_;                                  //机器人参数
    Quadruped *quadruped_;                              //四足机器人学运算库
    bool is_sim_;                                       //是否是仿真

    //机械参数
    float total_mass_;                                  //总质量
    Mat3 total_inertia_;                                //总的惯性张量

    //时间参数
    float dt_ = 0.001f;                                 //当前控制周期和上一个控制周期的相差时间，单位是s
    float period_ = 0.001f;                             //当前控制周期和上一个控制周期的实际相差时间，单位是s
    long long last_time_ = 0;                           //上一时刻，单位us，初始为0,无效
    long long start_time_ = 0;                          //从start_time开始的时间，单位us
    long long tick_ = 0;                                //上一时刻，单位us，初始为0,无效
    bool first_time_run_ = false;

    //和硬件通信的数据
    ImuData imu_data_;                                  //IMU数据的缓存，包含四元数，线性加速度和角速度
    std::vector<MotorState> motor_states_;              //电机状态的缓存，将在下一个RecvState中填充
    std::vector<MotorCommand> motor_commands_;          //电机指令的缓存，将在下一个SendCommand函数中下发
    std::vector<ForceSensorData> force_sensor_data_;    //力传感器的数据

    //步态数据
    Vec4 phase_normalized_;                             //归一化的相位
    std::vector<LegState> leg_state_;                   //根据步态生成器，规划出来的腿状态
    WaveStatus wave_status_;                            //波形状态

    //数据滤波相关的参数
    float yaw_offset_ = 0.0f;                           //初始的偏行角度
    MovingAverageFilter<3> rpy_filter_;                 //欧拉角均值滤波
    MovingAverageFilter<3> gyro_filter_;                //角度度均值滤波

    //下发到电机的数据(关节空间)
    Vec12 joint_command_angles_;                        //joint_space下的关节角命令，rad
    Vec12 joint_command_velocities_;                    //joint_space下关节角速度命令，rad/s
    Vec12 joint_command_torque_;                        //扭矩，NM
    Vec12 joint_command_kp_;                            //位置刚度
    Vec12 joint_command_kd_;                            //速度刚度

    //从编码器中读取的数据(关节空间)
    Vec12 joint_state_angles_;                          //joint_space下的关节角，需要从执行器空间转换到关节空间，单位是rad
    Vec12 joint_state_velocities_;                      //joint_space下关节角速度，rad/s
    Vec12 joint_state_ddq_;                             //joint_space下的关节角加速度，rad/s2
    Vec12 joint_state_torque_;                          //关节的扭矩
    Mat34 leg_state_angles_;                            //将joint_state_angles_数据按腿排列
    Mat34 leg_state_velocities_;                        //将joint_state_velocities_数据按腿排列

    //从IMU中读取的数据
    Vec3 base_rpy_;                                     //基坐标系到世界坐标系的ZYX欧拉角
    Vec3 base_rpy_rate_;                                //基座欧拉角微分
    Quat base_orientation_;                             //基座的四元数
    Vec3 base_acc_in_base_frame_;                       //基座绝对线性加速度（基坐标系）

    //从足底力传感器读取的数据
    Vec4 foot_force_;                                   //足端作用力
    bool foot_contact_[4];                              //是否接触的矩阵

    //根据传感器读数，直接通过机器人学计算的数据
    Mat34 foot_to_base_positions_in_base_frame_;        //足端对基座位置（基座坐标系）
    Mat34 foot_to_base_velocities_in_base_frame_;       //足端对基座速度（基座坐标系）

    //速度估计器的输出
    Vec3 base_velocity_in_base_frame_;                  //基座绝对移动速度（基坐标系）
    Vec3 base_velocity_in_world_frame_;                 //基座绝对移动速度（世界坐标系）
    Mat34 foot_velocities_in_world_frame_;              //足端绝对速度（世界坐标系）   

    //位置估计器的输出
    Vec3 base_position_;                                //基座在世界坐标系下的位置
    Mat34 foot_positions_in_world_frame_;               //足端绝对位置（世界坐标系） 

    //质心离支撑平面的高度
    float height_;                                      //机身相对于它所在平面的高度

    RobotMotionCommand motion_command_;                 //机器人运动命令，给控制器的参考输入
private:
    virtual void TransCommand() = 0;                    //将机器人学参数转换为硬件参数
    virtual void SendCommand() = 0;                     //将所有控制信号输出到仿真或者下位机
    virtual void RecvState() = 0;                       //将所有传感器信号从仿真或者下位机采集上来
    virtual void TransState() = 0;                      //将硬件参数转换为机器人学基本参数
};

}

#endif//QUADRUPED_ROBOT_ROBOT_H_