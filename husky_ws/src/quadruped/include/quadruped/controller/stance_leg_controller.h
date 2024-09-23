#ifndef QUADRUPED_CONTROLLER_STANCE_LEG_CONTROLLER_H_
#define QUADRUPED_CONTROLLER_STANCE_LEG_CONTROLLER_H_

#include "robot/robot.h"
#include "gait/gait_generator.h"
#include "controller/mpc/mpc_stance_leg_controller.h"
#include "controller/desired_state_command.h"


namespace quadruped
{


class StanceLegController
{
public:
    StanceLegController(Robot* robot, GaitGenerator* gait);
    ~StanceLegController();

    void Update();
    Mat34 GetFootForceInBase() {return mpc_controller_->GetFootForceInBase();}    //返回世界坐标系下的反力

private:
    Robot* robot_;
    GaitGenerator* gait_;
    MpcStanceLegController* mpc_controller_;
    DesiredStateCommand* desired_state_command_;
};


}//namespace quadruped


#endif//QUADRUPED_CONTROLLER_STANCE_LEG_CONTROLLER_H_