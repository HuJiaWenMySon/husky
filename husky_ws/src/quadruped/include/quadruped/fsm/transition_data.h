#ifndef QUADRUPED_FSM_TRANSITION_DATA_H_
#define QUADRUPED_FSM_TRANSITION_DATA_H_

namespace quadruped
{


struct TransitionData
{
    TransitionData() {Zero();}

    void Zero() 
    {
        done = false;
    }

    bool done = false;  //转换完成的标志位

    float t_0;         //转换开始的时间
    float t_current;   //当前时间（从转换开始算起）
    float t_duration;  //总的转换市场
};

}//namespace quadruped



#endif//QUADRUPED_FSM_TRANSITION_DATA_H_