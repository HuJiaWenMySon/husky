#ifndef QUADRUPED_FSM_FSM_STATE_PASSIVE_H_
#define QUADRUPED_FSM_FSM_STATE_PASSIVE_H_

#include "fsm/fsm_state.h"


namespace quadruped
{


class FsmStatePassive: public FsmState
{
public:
    FsmStatePassive(ControlFsmData* data);
    virtual void Enter();
    virtual void Run();
    virtual void Exit();
    virtual FsmStateName CheckTransition(FsmStateName desired_state_name);
    virtual TransitionData Transition(FsmStateName next_state_name);
};



}//namespace quadruped


#endif//QUADRUPED_FSM_FSM_STATE_PASSIVE_H_