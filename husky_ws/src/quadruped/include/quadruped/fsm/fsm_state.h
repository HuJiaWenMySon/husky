#ifndef QUADRUPED_FSM_FSM_STATE_H_
#define QUADRUPED_FSM_FSM_STATE_H_

#include "fsm/control_fsm_data.h"
#include "fsm/transition_data.h"
#include "utils/logger.h"


namespace quadruped
{


/**
 * @brief 状态机所支持的所有状态列表
 * 
 */
enum class FsmStateName 
{
    INVALID,                  //无效状态
    PASSIVE,                  //阻尼模式
    FIXED_STAND,              //固定站立模式
    FREE_STAND,               //平衡站立模式
    LOCOMOTION,               //移动模式
};


/**
 * @brief 状态的基类
 * 
 */
class FsmState
{
public:
    FsmState(ControlFsmData* data);
    virtual void Enter() = 0;
    virtual void Run() = 0;
    virtual void Exit() = 0;
    virtual FsmStateName CheckTransition(FsmStateName ) {return FsmStateName::INVALID;}
    virtual TransitionData Transition(FsmStateName ) {return transition_data_;}

    inline FsmStateName GetStateName() {return state_name_;}

protected:
    FsmStateName state_name_;             //状态名称
    TransitionData transition_data_;      //状态转换的过程数据
    ControlFsmData* data_;                //指向状态机数据的指针
};




}//namespace quadruped


#endif//QUADRUPED_FSM_FSM_STATE_H_
