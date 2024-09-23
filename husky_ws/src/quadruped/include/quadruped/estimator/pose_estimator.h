#ifndef QUADRUPED_ESTIMATOR_POSE_ESTIMATOR_H_
#define QUADRUPED_ESTIMATOR_POSE_ESTIMATOR_H_

#include "robot/robot.h"
#include "estimator/velocity_estimator.h"

namespace quadruped
{

class PoseEstimator
{
public:
    PoseEstimator(Robot* robot,VelocityEstimator* velocity_estimator);
    void Update();
    float EstimateHeight();

private:
    Robot* robot_;
    VelocityEstimator* velocity_estimator_;
    Vec6 estimated_pose_;
};
    
}



#endif//QUADRUPED_ESTIMATOR_POSE_ESTIMATOR_H_