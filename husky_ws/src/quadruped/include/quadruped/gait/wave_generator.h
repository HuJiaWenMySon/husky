#ifndef QUADRUPED_GAIT_WAVE_GENERATOR_H_
#define QUADRUPED_GAIT_WAVE_GENERATOR_H_

#include "utils/math.h"
#include <vector>


namespace quadruped
{

/**
 * @brief 腿的状态
 * 
 */
enum class LegState
{   
    SWING = 0,       //摆动相
    STANCE,         //支撑相
};


/**
 * @brief 波形状态
 * 
 */
enum class WaveStatus
{
    SWING_ALL = 0,          //只摆动，这种情况下持续输出低电平
    STANCE_ALL,         //只支撑，这种情况下持续输出高电平
    WAVE_ALL            //支撑摆动交替，行走模式使用这种
};
    

/**
 * @brief 波形生成器
 * 
 */
class WaveGenerator
{
public:
    WaveGenerator(float period, float stance_phase_ratio, Vec4 bias, unsigned int start_time);

    //这个函数根据当前和上一时刻腿所处的状态，判断是否应当切换状态
    void CalLegState(Vec4& phaphase_in_full_cycle_se_normalized_result, std::vector<LegState>& leg_state_result, WaveStatus status, unsigned int current_time);   

public:
    unsigned int start_time_;                  //步态开始时间
    unsigned int current_time_;                //当前更新时间
    float period_;                      //步态周期
    float stance_phase_ratio_;          //支撑相占比
    Vec4 bias_;                         //偏置

    WaveStatus status_last_;            //上一时刻的状态

    Vec4 phase_nomalized_;                //在当前时刻中的相位
    Vec4 phase_nomalized_last_;           //在上一时刻的相位

    std::vector<LegState> leg_state_;       //腿的状态
    std::vector<LegState> leg_state_last_;  //上一时刻腿的状态
    VecInt4 switch_status_;                 //是否需要进行状态转换

    Vec4 phase_in_full_cycle_;              //在整个步态周期中的位置 

    void CalWave(Vec4& phase_normalized, std::vector<LegState>& leg_state, WaveStatus status);             //这个函数仅仅根据当前系统时间，计算波形
};




} // namespace quadruped




#endif//QUADRUPED_PLANNER_WAVE_GENERATOR_H_