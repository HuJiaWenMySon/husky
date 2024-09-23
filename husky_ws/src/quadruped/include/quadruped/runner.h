#ifndef QUADRUPED_RUNNER_H_
#define QUADRUPED_RUNNER_H_

#include <ros/ros.h>

#include "robot/robot_go1_sim.h"
#include "robot/joystick.h"
#include "fsm/control_fsm.h"
#include "estimator/state_estimator.h"
#include "controller/desired_state_command.h"


namespace quadruped
{

class Runner
{
public:
    Runner(ros::NodeHandle& nh);
    ~Runner();
    void Update();

private:
    Robot* robot_;
    ControlFsm* control_fsm_;
    StateEstimator* state_estimator_;
    DesiredStateCommand* desired_state_command_;
    Joystick* joystick_;
};


}//namespace quadruped



#endif//QUADRUPED_RUNNER_H_