#ifndef QUADRUPED_FSM_FSM_STATE_FIXED_STAND_H_
#define QUADRUPED_FSM_FSM_STATE_FIXED_STAND_H_


#include "fsm/fsm_state.h"


namespace quadruped
{




class FsmStateFixedStand: public FsmState
{
public:
    FsmStateFixedStand(ControlFsmData* data);
    virtual void Enter();
    virtual void Run();
    virtual void Exit();
    virtual FsmStateName CheckTransition(FsmStateName desired_state_name);
    virtual TransitionData Transition(FsmStateName next_state_name);

private:
    std::vector<float> fixed_stand_q_;       //固定关节角度
    std::vector<float> fixed_stand_kp_;      //固定位置刚度
    std::vector<float> fixed_stand_kd_;      //固定速度刚度
    std::vector<float> start_q_;       //初始角度
    int duration_;                      //经过多少个周期站起来
    float percent_;                    //站起来的百分比
};




}//namespace quadruped


#endif//QUADRUPED_FSM_FSM_FIXED_STAND_H_