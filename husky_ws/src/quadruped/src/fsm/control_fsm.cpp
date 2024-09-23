#include "fsm/control_fsm.h"
#include "utils/logger.h"


namespace quadruped
{



ControlFsm::ControlFsm(Robot* robot, DesiredStateCommand* desired_state_command)
{
    wave_generator_ = new WaveGenerator(robot->param_.period,robot->param_.stance_phase_ratio,robot->param_.bias,robot->tick_);

    robot_ = robot;
    data_.robot = robot;
    data_.desired_state_command = desired_state_command;  
    data_.wave_generator = wave_generator_;

    state_list_.invalid  = nullptr;
    state_list_.passive = new FsmStatePassive(&data_);
    state_list_.fixed_stand = new FsmStateFixedStand(&data_);
    state_list_.free_stand = new FsmStateFreeStand(&data_);
    state_list_.locomotion = new FsmStateLocomotion(&data_);

    safety_checker_ = new SafetyChecker();      //创建安全检测器
    
    current_state_ = state_list_.passive;       //开始是阻尼模式
    next_state_ = current_state_;               //默认不进行状态转换
    desired_state_ = current_state_;            //默认不进行状态转换

    current_state_->Enter();                    //进入阻尼模式        

    operating_mode_ = FsmOperatingMode::NORMAL; //状态机正常运行
}   



void ControlFsm::Run()
{
    FsmStateName desired_state_name,current_state_name,next_state_name;

    wave_generator_->CalLegState(robot_->phase_normalized_, robot_->leg_state_, robot_->wave_status_, robot_->tick_);//生成当前时刻的波形
    // LOG_INFO("Robot Leg state:%d,%d,%d,%d.",robot_->leg_state_[0],robot_->leg_state_[1],robot_->leg_state_[2],robot_->leg_state_[3]);

    SafetyPreCheck();
    
    if(data_.desired_state_command->GetModeChangeRequest())     //如果在外部指令中发生了状态切换
    {
        RcMode mode = data_.desired_state_command->GetMode();   //根据外部指令，设置期望状态
        switch (mode)
        {
        case RcMode::TO_PASSIVE:
            desired_state_ = state_list_.passive;
            break;
        case RcMode::TO_FIXEDSTAND:      
            desired_state_ = state_list_.fixed_stand;
            break;
        case RcMode::TO_FREESTAND:
            desired_state_ = state_list_.free_stand;
            break;
        case RcMode::TO_LOCOMOTION:
            desired_state_ = state_list_.locomotion;  
            break;                   
        default:
            break;
        }
    }

    if(operating_mode_ != FsmOperatingMode::ESTOP) //状态机没有停止运行
    {       
        if(operating_mode_ == FsmOperatingMode::NORMAL) //状态机正常工作
        {
            desired_state_name = desired_state_->GetStateName();   //获得期望的状态名称
            current_state_name = current_state_->GetStateName();   //获取当前的状态名称
            next_state_name = current_state_->CheckTransition(desired_state_name);     //检查期望状态能否转换，得到下一次循环的状态
            if(next_state_name != current_state_name)    //下一个状态不等于当前状态（说明期望状态与当前状态不同，并且转换是有效的），进行转换
            {
                operating_mode_ = FsmOperatingMode::TRANSITIONING;   //将标志位设置为转换中
                next_state_ = desired_state_;         //下一次状态更新为期望状态     
            }
            else
            {
                current_state_->Run();      //直接运行当前状态的RUN函数
            }
        }

        if(operating_mode_ == FsmOperatingMode::TRANSITIONING) //状态机执行状态切换
        {
            next_state_name = next_state_->GetStateName();
            transition_data_ = current_state_->Transition(next_state_name);    //进行状态转换
            SafetyPostCheck();

            if(transition_data_.done)
            {
                current_state_->Exit();             //退出当前状态
                current_state_ = next_state_;       //标志位切换
                current_state_->Enter();            //进入新的状态
                operating_mode_ = FsmOperatingMode::NORMAL;
            }    
        }
        else
        {
            SafetyPostCheck();
        }
    }
}



FsmOperatingMode ControlFsm::SafetyPreCheck()
{
    return operating_mode_;
}



FsmOperatingMode ControlFsm::SafetyPostCheck()
{
    return operating_mode_;
}



}//namespace quadruped


