#include "runner.h"



namespace quadruped
{

Runner::Runner(ros::NodeHandle& nh)
{
    robot_ = new RobotGo1Sim(nh);
    desired_state_command_ = new DesiredStateCommand();
    state_estimator_ = new StateEstimator(robot_);
    control_fsm_ = new ControlFsm(robot_,desired_state_command_);
    joystick_ = new Joystick(nh,desired_state_command_);
}


Runner::~Runner()
{
    delete joystick_;
    delete control_fsm_;
    delete state_estimator_;
    delete desired_state_command_;
    delete robot_;
}



void Runner::Update()
{
    // static int times = 0;
    // if(times == 3000)
    // {
    //     desired_state_command_->SetModeChangeRequest();
    //     desired_state_command_->SetMode(RcMode::TO_FIXEDSTAND);
    // }

    // if(times == 6000)
    // {
    //     desired_state_command_->SetModeChangeRequest();
    //     desired_state_command_->SetMode(RcMode::TO_LOCOMOTION);
    // }

    // times++;    

    robot_->RecvTransState();           //接收传感器数据并转换到关节空间

    if(robot_->first_time_run_) return; //第一次，由于时间间隔没计算出来，不进行控制

    state_estimator_->Update();         //根据观测数据进行状态估计
    desired_state_command_->Update();   //更新用户期望的命令
    control_fsm_->Run();                //根据当前状态和用户期望，执行控制逻辑，产生本次循环的控制信号
    robot_->SendTransCommand();         
}


}//namespace quadruped
