#ifndef QUADRUPED_ESTIMATOR_VELOCITY_ESTIMATOR_H_
#define QUADRUPED_ESTIMATOR_VELOCITY_ESTIMATOR_H_

#include "robotics/quadruped.h"
#include "robot/robot.h"


namespace quadruped
{


class VelocityEstimator
{
public:
    VelocityEstimator(Robot* robot);
    void Update();

    Vec3 estimated_velocity_;               //线速度（估计值）
    Vec3 estimated_angular_velocity_;       //角速度（估计值）

private:
    Robot* robot_;

    //kalman filter
    void KalmanFilterInit();
    void KalmanFilterPredict();
    void KalmanFilterUpdate();

    float accelerometer_variance_;                  //加速度计的协方差，用作过程误差Q
    float sensor_variance_;                         //观测变量的协方差R
    float initial_variance_;                        //初始的协方差P

    Vec3 Z_;    //输出变量，一个三维的向量，在本例中就是观测到的速度
    Vec3 X_;    //状态变量，同样也是三维的向量，在本例中也是速度
    Mat3 P_;    //协方差矩阵
    Mat3 Q_;    //过程误差
    Mat3 R_;    //观测误差
    Vec3 BU_;   //状态方程Xn+1 = AXn + BU中的BU，在本例中是速度的差值
    Mat3 K_;    //卡尔曼增益的矩阵

};

}



#endif//QUADRUPED_ESTIMATOR_VELOCITY_ESTIMATOR_H_