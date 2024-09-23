#ifndef QUADRUPED_FSM_CONTROL_FSM_H_
#define QUADRUPED_FSM_CONTROL_FSM_H_


#include "fsm/control_fsm_data.h"
#include "fsm/safety_checker.h"
#include "fsm/fsm_state.h"
#include "fsm/fsm_state_locomotion.h"
#include "fsm/fsm_state_passive.h"
#include "fsm/fsm_state_fixed_stand.h" 
#include "fsm/fsm_state_free_stand.h" 

#include "gait/wave_generator.h"


namespace quadruped
{

/**
 * @brief 状态机运行模式
 * 
 */
enum class FsmOperatingMode
{
    NORMAL,                         //正常
    TRANSITIONING,                  //转换中
    ESTOP,                          //紧急停止
    EDAMP                           //紧急减震
};



struct FsmStateList
{
    FsmState* invalid;
    FsmStatePassive* passive;
    FsmStateFixedStand* fixed_stand;
    FsmStateFreeStand* free_stand;
    FsmStateLocomotion* locomotion;
};



/**
 * @brief 用于执行控制算法的有限状态机
 * 
 */
class ControlFsm
{
public:
    ControlFsm(Robot* robot, DesiredStateCommand* desired_state_command);

    ~ControlFsm() = default;
    
    void Run();

private:    
    FsmOperatingMode SafetyPreCheck();  //状态转换前安全检查
    FsmOperatingMode SafetyPostCheck(); //状态转换后安全检查

    FsmOperatingMode operating_mode_;   //当前的工作模式
    SafetyChecker* safety_checker_;     //安全检查器
    ControlFsmData data_;               //保存所有控制相关的数据
    TransitionData transition_data_;    //保存转换过程的状态信息

    FsmState* current_state_;           //当前时刻的状态
    FsmState* next_state_;              //下一时刻的状态
    FsmState* desired_state_;           //外部输入的期望状态
    FsmStateList state_list_;           //状态名称和实际状态的映射表

    Robot* robot_;
    WaveGenerator* wave_generator_;     //波形发生器
};

}//namespace quadruped


#endif//QUADRUPED_FSM_CONTROL_FSM_H_