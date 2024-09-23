#include "estimator/state_estimator.h"
#include "utils/logger.h"


namespace quadruped
{



StateEstimator::StateEstimator(Robot* robot)
{
    robot_ = robot;
    quadruped_ = robot->quadruped_;

    velocity_estimator_ = new VelocityEstimator(robot);//速度估计器
    pose_estimator_ = new PoseEstimator(robot,velocity_estimator_);
}




void StateEstimator::Update()
{
    velocity_estimator_->Update();
    pose_estimator_->Update();//位姿估计需要使用速度估计的结果，所以需要在后面
}




}//namespace quadruped