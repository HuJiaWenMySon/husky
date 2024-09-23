#ifndef QUADRUPED_CONTROLLER_MPC_MPC_INTERFACE_H_
#define QUADRUPED_CONTROLLER_MPC_MPC_INTERFACE_H_


#include "robot/robot.h"
#include <qpOASES.hpp>



namespace quadruped
{

#define HORIZON_MAX 30


/**
 * @brief 使用MPC控制器所需的机器人状态
 * 
 */
struct MpcRobotState
{
    Vec3 p;                             //世界坐标系下的质心位置向量
    Vec3 v;                             //世界坐标系下的质心线速度
    Vec3 w;                             //世界坐标系下的基座角速度
    Vec3 rpy;                           //欧拉角                        
    Mat34 foot_to_base_position_in_world_frame;          //足端到基座的位置向量（基座坐标系）
    Mat3 Rwb;                           //基座坐标系到世界坐标系的旋转矩阵           
    Mat3 Ryaw;                          //只考虑yaw方向变化的旋转矩阵
    Eigen::Quaternionf quat;            //基座的四元数

    Mat3 body_inertia;                  //基座坐标系下的惯性张量  
    float mass = 12;                    //质量

    float traj[12*HORIZON_MAX];                      //参考轨迹
    float gait[4*HORIZON_MAX];                      //步态相位
};


/**
 * @brief MPC问题的参数
 * 
 */
struct ProblemConfig
{
    float dt;                                           //MPC执行一次的时间
    float friction_coeff;                               //摩擦系数
    float f_max;                                        //最大摩擦力
    int horizon;                                        //MPC的预测周期
    float total_mass;                                   //总质量
    float weight[12];                                   //12个状态变量的权重
    float alpha;                                        //足端力的权重
};


void SetUpProblem(float dt, int horizon, float friction_coeff, float f_max, float total_mass, float* inertia, float* weight, float alpha);

void ResizeQpMats(int horizon);

void ConvertToDiscreteQp(Eigen::Matrix<float,13,13> A_continuous, Eigen::Matrix<float,13,12> B_continuous, float dt, int horizon);

void ComputeContinuousMatrices(Mat3 inertia_world, float mass, Mat34 foot_to_base_position_in_world_frame, Mat3 Ryaw, 
                                Eigen::Matrix<float, 13, 13>& A, Eigen::Matrix<float, 13, 12>& B);

void SolveMpcKernel(Vec3& p, Vec3& v, Vec3& rpy, Vec3& w, Mat34& foot_to_base_position_in_world_frame, 
                    Quat base_orientation, float* state_trajectory, float* gait);

void SolveMpc(ProblemConfig* setup);

double GetMpcSolution(int index);

};//namespace quadruped

#endif//QUADRUPED_CONTROLLER_MPC_MPC_INTERFACE_H_