#include "estimator/pose_estimator.h"
#include "utils/logger.h"


namespace quadruped
{


PoseEstimator::PoseEstimator(Robot* robot,VelocityEstimator* velocity_estimator)
{
    robot_ = robot; 
    velocity_estimator_ = velocity_estimator;
    estimated_pose_ << robot_->base_position_, robot_->base_rpy_;
}


float PoseEstimator::EstimateHeight()
{
    Mat3 Rwb = QuatToRotMat(robot_->base_orientation_);//获取基坐标系到世界坐标系的旋转矩阵
    Mat34 foot_positions_in_base_frame = robot_->foot_to_base_positions_in_base_frame_;
    Vec4 useful_heights_;
    Eigen::Matrix<int, 1, 4> contacts;

    for(int i = 0; i < 4; i++)
    {
        if(robot_->foot_contact_[i]) //只有处于支撑状态，才可以使用逆运动学计算高度
        {
            contacts[i] = true;
        }
        else
        {
            contacts[i] = false;
        }
    }
    if (contacts.sum() == 0) 
    {
        return robot_->base_position_[2];
    } 
    else 
    {
        Mat34 foot_positions_in_world_frame = Rwb*foot_positions_in_base_frame;
        useful_heights_ = -foot_positions_in_world_frame.block<1, 4>(2, 0).cwiseProduct(contacts.cast<float>());
        // printf("%f,%d\r\n",useful_heights_.sum() / contacts.sum(),contacts.sum());
        return useful_heights_.sum() / contacts.sum();
    }
}


void PoseEstimator::Update()
{
    float measured_period = robot_->period_;    //时间间隔
    Mat3 Rwb = QuatToRotMat(robot_->base_orientation_);//获取基坐标系到世界坐标系的旋转矩阵

    //2D里程计
    Vec3 base_velocity_in_world_frame_ = robot_->base_velocity_in_world_frame_;

    //读取速度估计值和其他状态
    float v_theta = velocity_estimator_->estimated_angular_velocity_[2];
    float v_x = velocity_estimator_->estimated_velocity_[0];
    float v_y = velocity_estimator_->estimated_velocity_[1];
    float v_z = velocity_estimator_->estimated_velocity_[2];
    float x = estimated_pose_[0];
    float y = estimated_pose_[1];
    float theta = estimated_pose_[5];
    
    //计算位姿变化的微分
    float delta_x = (v_x * cos(theta) - v_y * sin(theta)) * measured_period;
    float delta_y = (v_x * sin(theta) + v_y * cos(theta)) * measured_period;
    float delta_theta = v_theta * measured_period;

    //积分得到位移
    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    //更新估计量
    estimated_pose_[0] = x;
    estimated_pose_[1] = y;
    estimated_pose_[5] = theta;

    //更新robot结构体中位置相关的变量
    robot_->base_position_[0] = estimated_pose_[0];         //保存世界坐标系下的x位置
    robot_->base_position_[1] = estimated_pose_[1];         //保存世界坐标系下的y位置

    float height = EstimateHeight();

    // robot_->base_position_[2] += v_z * measured_period;     //保存世界坐标系下的z位置，考虑RP较小，vx和vy对垂直方向的位移忽略不计
    robot_->base_position_[2] = height;
    // LOG_INFO("x:%f y:%f z:%f",robot_->base_position_[0],robot_->base_position_[1],robot_->base_position_[2]);

    for(int i = 0; i < 4; i++)
    {
        //更新足端在世界坐标系下的位置
        robot_->foot_positions_in_world_frame_.col(i) = robot_->base_position_ + Rwb*robot_->foot_to_base_positions_in_base_frame_.col(i);
    }
}



}//namespace quadruped