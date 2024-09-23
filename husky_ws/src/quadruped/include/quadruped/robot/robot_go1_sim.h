#ifndef QUADRUPED_ROBOT_ROBOT_GO1_SIM_H_
#define QUADRUPED_ROBOT_ROBOT_GO1_SIM_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include "robot/robot.h"

//Unitree
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"


namespace quadruped
{

class RobotGo1Sim: public Robot
{
public:
    RobotGo1Sim(ros::NodeHandle& nh);
    virtual ~RobotGo1Sim();  
    virtual void TransCommand();
    virtual void SendCommand();                 
    virtual void RecvState();          
    virtual void TransState();

public:
    unitree_legged_msgs::LowCmd low_cmd_;           //宇树机器狗专有的命令结构体
    unitree_legged_msgs::LowState low_state_;       //宇树机器狗专有的状态结构体
    ros::Subscriber joint_state_sub_[12];           //关节状态接收器，和ros-control通信
    ros::Publisher joint_state_pub_[12];            //关节状态发布器，和ros-control通信
    ros::Subscriber imu_sub_;                       //IMU数据的接收器
    ros::Subscriber foot_force_sub_[4];             //足底力传感器的接收器

    void FlHipCallback(const unitree_legged_msgs::MotorState &msg);
    void FlThighCallback(const unitree_legged_msgs::MotorState &msg);
    void FlCalfCallback(const unitree_legged_msgs::MotorState &msg);
    void FrHipCallback(const unitree_legged_msgs::MotorState &msg);
    void FrThighCallback(const unitree_legged_msgs::MotorState &msg);
    void FrCalfCallback(const unitree_legged_msgs::MotorState &msg);
    void RrHipCallback(const unitree_legged_msgs::MotorState &msg);
    void RrThighCallback(const unitree_legged_msgs::MotorState &msg);
    void RrCalfCallback(const unitree_legged_msgs::MotorState &msg);
    void RlHipCallback(const unitree_legged_msgs::MotorState &msg);
    void RlThighCallback(const unitree_legged_msgs::MotorState &msg);
    void RlCalfCallback(const unitree_legged_msgs::MotorState &msg);
    void FrFootCallback(const geometry_msgs::WrenchStamped &msg);
    void FlFootCallback(const geometry_msgs::WrenchStamped &msg);
    void RrFootCallback(const geometry_msgs::WrenchStamped &msg);
    void RlFootCallback(const geometry_msgs::WrenchStamped &msg);
    void ImuCallback(const sensor_msgs::Imu &msg);
};

}//namespace quadruped


#endif//QUADRUPED_ROBOT_ROBOT_GO1_SIM_H_