#ifndef QUADRUPED_CONTROLLER_MPC_MPC_STANCE_LEG_CONTROLLER_H_
#define QUADRUPED_CONTROLLER_MPC_MPC_STANCE_LEG_CONTROLLER_H_

#include "robot/robot.h"
#include "gait/gait_generator.h"
#include "controller/mpc/mpc_interface.h"
#include "controller/desired_state_command.h" 


namespace quadruped
{


class MpcStanceLegController
{
public:
    MpcStanceLegController(Robot* robot, GaitGenerator* gait);
    ~MpcStanceLegController();

    void Run();
    Mat34 GetFootForceInBase() {return f_ff_;}

private:
    Robot* robot_;                                  //机器人结构体
    GaitGenerator* gait_;                           //步态信息

    bool use_wbc_;                                  //是否使用WBC求关节扭矩
    float dt_;                                      //基础控制周期
    float dt_mpc_;                                  //mpc周期
    int iterations_in_mpc_;                         //mpc一次计算的时间间隔  
    int horizon_length_;                            //预测区间

    Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> mpc_table_; //预测区间内的接触状态

    Mat34 f_ff_;                                    //腿对地面的作用力（基座坐标系）
    Mat34 f_;                                       //地面的反作用力（世界坐标系）
    float Q_[12];                                   //mpc的状态权重

    Vec3 rpy_int_;                                  //欧拉角偏移的微分
    Vec3 rpy_comp_;                                 //修正后的欧拉角

    float yaw_turn_rate_;                           //期望的yaw方向速度
    float yaw_desired_;                             //期望的yaw
    float roll_desired_;                            //期望的roll
    float pitch_desired_;                           //期望的pitch

    Vec3 base_position_desired_in_world_frame_;     //世界坐标系下期望的位置
    Vec3 base_velocity_desired_in_world_frame_;     //世界坐标系下期望的速度

    Vec3 base_velocity_in_world_frame_;             //世界坐标系下的当前速度

    unsigned long long iteration_counter_;          //一次MPC包含的控制周期数目
    float height_;                                  //机身高度
    float traj_all_[12*36];                         //所有的轨迹

    void Reset();                                   //重置MPC相关参数
    void UpdateMpc();                               //更新MPC期望轨迹
    void SolveDenseMpc();                           //MPC求解
};


}//namespace quadruped


#endif//QUADRUPED_CONTROLLER_MPC_MPC_STANCE_LEG_CONTROLLER_H_