#include "controller/stance_leg_controller.h"



namespace quadruped
{



StanceLegController::StanceLegController(Robot* robot, GaitGenerator* gait)
{
    robot_ = robot;
    gait_ = gait;
    mpc_controller_ = new MpcStanceLegController(robot_,gait_);
}


StanceLegController::~StanceLegController()
{
    delete mpc_controller_;
}


/**
 * @brief 接口函数，决定调用何种类型的站立腿控制器
 * @details 这里使用的是MPC控制器
 * 
 */
void StanceLegController::Update()
{
    mpc_controller_->Run();
}






}//namespace quadruped