#ifndef QUADRUPED_FSM_FSM_STATE_FREE_STAND_H_
#define QUADRUPED_FSM_FSM_STATE_FREE_STAND_H_


#include "fsm/fsm_state.h"
#include "controller/stance_leg_controller.h"

namespace quadruped
{



class FsmStateFreeStand: public FsmState
{
public:
    FsmStateFreeStand(ControlFsmData* data);
    virtual void Enter();
    virtual void Run();
    virtual void Exit();
    virtual FsmStateName CheckTransition(FsmStateName desired_state_name);
    virtual TransitionData Transition(FsmStateName next_state_name);

private:
    Robot* robot_;
    GaitGenerator* gait_;                                       //步态规划器               

    StanceLegController* stance_controller_;                    //支撑腿控制器

    //期望的质心轨迹
    float yaw_turn_rate_;                                       //期望的yaw rate
    float yaw_desired_;                                         //期望的yaw
    float roll_desired_;                                        //期望的roll
    float pitch_desired_;                                       //期望的pitch

    Vec3 base_velocity_desired_in_world_frame_;                 //世界坐标系下期望的基座速度
    Vec3 base_position_desired_in_world_frame_;                 //世界坐标系下期望的的基座位置

    Mat34 foot_positions_desired_in_world_frame_;               //步态生成器根据当前命令，规划的足端位置
    Mat34 foot_velocities_desired_in_world_frame_;              //步态生成器根据当前命令，规划的足端速度

    Mat34 foot_force_in_base_frame_;                            //计算出来的足端反力(基座系)

    Mat3 swing_kp_;                                             //摆动腿的kp
    Mat3 swing_kd_;                                             //摆动腿的kd

    Mat34 foot_to_base_positions_desired_in_base_frame_;        //足端相对基座的位置向量（基座系）
    Mat34 foot_to_base_velocities_desired_in_base_frame_;       //足端相对于基座的速度向量（基座系）

    Vec12 tau_;                                                 //将要发送下去的关节力矩
    Vec12 q_,qd_;                                               //将要发送下去的位置、速度;

    DesiredStateCommand* desired_state_command_;                 //期望的命令

private:
    void SetupCommand();                                        //用于将外部命令（手柄）转化为控制器的输入
};




}//namespace quadruped


#endif//QUADRUPED_FSM_FSM_FREE_STAND_H_