#ifndef QUADRUPED_ESTIMATOR_STATE_ESTIMATOR_H_
#define QUADRUPED_ESTIMATOR_STATE_ESTIMATOR_H_

#include "robot/robot.h"
#include "estimator/pose_estimator.h"
#include "estimator/velocity_estimator.h"


namespace quadruped
{

class StateEstimator
{
public:
    StateEstimator(Robot* robot);
    void Update();

private:
    Robot* robot_;                              //指向机器人的指针
    Quadruped* quadruped_;                      //四足机器人学库

    PoseEstimator* pose_estimator_;             //位姿估计器
    VelocityEstimator* velocity_estimator_;     //速度估计器
};

}//namespace quadruped



#endif//QUADRUPED_ESTIMATOR_STATE_ESTIMATOR_H_