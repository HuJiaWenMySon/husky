#include "estimator/velocity_estimator.h"
#include "utils/logger.h"


namespace quadruped
{

VelocityEstimator::VelocityEstimator(Robot* robot)
{
    robot_ = robot;

    accelerometer_variance_ = robot_->param_.accelerometer_variance;        
    sensor_variance_ = robot_->param_.sensor_variance;                       
    initial_variance_ = robot_->param_.initial_variance;                       
    
    estimated_velocity_.setZero();
    estimated_angular_velocity_.setZero();

    KalmanFilterInit();//初始化卡尔曼滤波器
}


void VelocityEstimator::Update()
{
    float measured_period = robot_->period_;

    Mat3 Rwb = QuatToRotMat(robot_->base_orientation_);//获取基坐标系到世界坐标系的旋转矩阵
    Vec3 base_rpy_rate = robot_->base_rpy_rate_;
    Mat34 foot_velocities_in_base_frame = robot_->foot_to_base_velocities_in_base_frame_;
    Mat34 foot_positions_in_base_frame = robot_->foot_to_base_positions_in_base_frame_;
    Vec3 base_acc_in_world_frame = Rwb*robot_->base_acc_in_base_frame_;//世界坐标系下的加速度
    std::vector<Vec3> velocities_observed;
    base_acc_in_world_frame[2] -= 9.81;
    BU_ = base_acc_in_world_frame*measured_period;

    //计算机器人足端相对于机身的位置坐标
    for(int i = 0; i < 4; i++)
    {
        if(robot_->foot_contact_[i]) //只有处于支撑状态，才可以用公式求速度
        {
            //这个公式，参见宇树书79页
            Vec3 base_velocity_in_world_frame = -Rwb*(foot_velocities_in_base_frame.col(i) + Skew(base_rpy_rate)*foot_positions_in_base_frame.col(i));
            velocities_observed.push_back(base_velocity_in_world_frame);
        }
    }

    int nums = velocities_observed.size();  //接触的腿数目

    if(nums > 0)    //如果有接触的腿，用接触腿的观测均值来作为新的观测
    {
        Vec3 velocities_observed_mean = Vec3::Zero();
        for(auto& v:velocities_observed)
        {
            velocities_observed_mean += v;
        }
        velocities_observed_mean /= nums;
        Z_ = velocities_observed_mean;//用接触腿观测到的均值作为质心观测速度
    }
    else            //如果没有接触的腿，在当前状态及中只有一种情况，那就是Passive状态时完全趴在地上，此时观测速度应当是0
    {
        Z_ << 0.0f, 0.0f, 0.0f;
    }

    KalmanFilterPredict();//预测
    KalmanFilterUpdate();//更新

    estimated_velocity_ << X_[0] ,X_[1] ,X_[2];                     //从状态估计器中读取数据
    robot_->base_velocity_in_world_frame_ = estimated_velocity_;    //更新基座在世界坐标系下的数据

    estimated_velocity_ = Rwb.transpose()*estimated_velocity_;      //速度变换到基座坐标系
    robot_->base_velocity_in_base_frame_ = estimated_velocity_;     //更新基座在基座坐标系下的数据

    estimated_angular_velocity_ = robot_->base_rpy_rate_;           //读取角速度并保存
    
    //计算足端在世界坐标系下的速度，TODO
    for(int i = 0; i < 4; i++)
    {
        robot_->foot_velocities_in_world_frame_.col(i) = robot_->base_velocity_in_world_frame_ + Rwb*robot_->foot_to_base_velocities_in_base_frame_.col(i);
    }
}


/**
 * @brief 卡尔曼滤波器的初始化，初始化PQR三个矩阵以及初始估计
 * 
 */
void VelocityEstimator::KalmanFilterInit()
{
    X_ = {0.0f,0.0f,0.0f};              //初始的速度估计为0
    P_ << initial_variance_,0.0f,0.0f   //初始的估计协方差矩阵P
        ,0.0f,initial_variance_,0.0f
        ,0.0f,0.0f,initial_variance_;
    Q_ << accelerometer_variance_,0.0f,0.0f //过程误差Q
        ,0.0f,accelerometer_variance_,0.0f
        ,0.0f,0.0f,accelerometer_variance_;
    R_ << sensor_variance_,0.0f,0.0f        //观测误差R
        ,0.0f,sensor_variance_,0.0
        ,0.0f,0.0f,sensor_variance_;
}


/**
 * @brief 卡尔曼滤波器的预测步骤
 * 
 */
void VelocityEstimator::KalmanFilterPredict()
{
    //根据状态转移方程进行先验估计
    X_ = X_ + BU_;  //Vn+1 = Vn + aT;
    //更新先验状态估计的协方差矩阵，A是单位阵
    P_ = P_ + Q_;
}


/**
 * @brief 卡尔曼滤波器的更新步骤
 * 
 */
void VelocityEstimator::KalmanFilterUpdate()
{
    //计算kalman增益
    K_ = P_*(P_+R_).inverse();

    //i计算后验状态估计
    X_ = X_ + K_*(Z_-X_);

    //更新后验状态估计的协方差矩阵
    P_ = (Mat3::Identity() - K_)*P_;
}

}//namespace quadruped