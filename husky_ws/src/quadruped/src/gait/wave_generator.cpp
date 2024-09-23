#include "gait/wave_generator.h"
#include "utils/logger.h"

namespace quadruped
{


WaveGenerator::WaveGenerator(float period, float stance_phase_ratio, Vec4 bias, unsigned int start_time)
{
    period_ = period;                
    stance_phase_ratio_ = stance_phase_ratio;
    bias_ = bias;  

    if(stance_phase_ratio_ >= 1 || stance_phase_ratio_ <= 0)
    {
        LOG_ERROR("Stance Ratio Error.");
        exit(-1);
    }

    for(int i = 0; i < 4; i++)
    {
        if(bias_[i] > 1 || bias_[i] < 0)
        {
            LOG_ERROR("Bias Error.");
            exit(-1);
        }
    }

    start_time_ = start_time;//启动时间

    LOG_INFO("Wave Generator start time: %d.",start_time_);

    leg_state_.resize(4);
    leg_state_last_.resize(4);

    for(int i = 0; i < 4; i++)
    {
        leg_state_last_[i] = LegState::SWING;//初始没有支撑，统一设置为悬空
        phase_nomalized_last_[i] = 0.5; //统一设置到中间的一个相位
    }

    switch_status_ << 0,0,0,0;
    status_last_ = WaveStatus::SWING_ALL;
}


/**
 * @brief 这个函数不仅计算波形，还会结合上一时刻的腿状态判断是佛可以转换腿状态，返回检测后的规划腿状态
 * 
 * @param phase_normalized_result 在当前状态（支撑相/摆动相）中的归一化相位
 * @param leg_state_result 腿的状态
 * @param status 处于生成波形态、完全摆动态还是完全支撑态
 */
void WaveGenerator::CalLegState(Vec4& phase_normalized_result, std::vector<LegState>& leg_state_result, WaveStatus status, unsigned int current_time)
{
    current_time_ = current_time;

    // LOG_INFO("current time: %d.",current_time_);
    CalWave(phase_nomalized_,leg_state_,status);//根据要求的status和当前的时间，计算所处的腿状态和归一化相位

    if(status != status_last_) //需要切换到新的状态了
    {
        // LOG_INFO("state trans: %d,%d",status,status_last_);

        if(switch_status_.sum() == 0)//如果全为0说明这是第一次开始进行转换,否则是状态转换中
        {
            switch_status_.setOnes();   //全部设置为1,标志着要开始进行四条腿的状态转换了
        }
        CalWave(phase_nomalized_last_,leg_state_last_,status_last_);//计算上一时刻的状态和相位

        //这是两种特殊情况，仍然按照上一个状态下的时刻处理，这样后续就能够正常转换
        if(status == WaveStatus::SWING_ALL && status_last_ == WaveStatus::STANCE_ALL)
        {
            for(auto& s : leg_state_last_) s = LegState::SWING;
        }
        else if(status == WaveStatus::STANCE_ALL && status_last_ == WaveStatus::SWING_ALL)
        {
            for(auto& s: leg_state_last_) s = LegState::STANCE;
        }

        // if(status == WaveStatus::WAVE_ALL && status_last_ == WaveStatus::STANCE_ALL)
        // {
        //     LOG_INFO("status:%d,%d,%d,%d",leg_state_[0],leg_state_[1],leg_state_[2],leg_state_[3]);
        //     LOG_INFO("status last:%d,%d,%d,%d",leg_state_last_[0],leg_state_last_[1],leg_state_last_[2],leg_state_last_[3]);
        //     exit(0);
        // }
    }


    if(switch_status_.sum() != 0)//还存在腿需要进行状态转换
    {
        for(int i = 0; i < 4; i++)
        {
            if(leg_state_[i] == leg_state_last_[i])
            {
                switch_status_[i] = 0;//只有新的状态下腿状态和上一个状态结束时的腿状态相同时，才能过度到新的状态，这样就能够实现腿的状态平滑过度
            }
            else
            {
                leg_state_[i] = leg_state_last_[i]; //仍然保持转换之前的状态
                phase_nomalized_[i] = phase_nomalized_last_[i];//仍然保持转换之前的相位
                //如果处于Wave状态摆动相的腿，进入Stand的支撑相，会基于Wave状态继续摆动，直到进入支撑相。
                //如果处于Wave状态支撑相的腿，就可以直接进入Stand。
                //摆动相的腿不会直接进入Wave状态（只有Stand能进入trot）
                //处于支撑相的腿不会进入Wave状态的摆动相，也就是说会忽略初始的摆动，从下一个支撑周期开始运动，这样状态过渡就是平滑的
            }
        }
        if(switch_status_.sum() == 0)
        {
            status_last_ = status;  //完成了状态转换，因此可以把当前状态设置为last状态，准备下一次转换
        }
    }

    phase_normalized_result = phase_nomalized_; //返回经过状态转换检测后，腿应当处于的相位
    leg_state_result = leg_state_;       //腿部状态
}


/**
 * @brief 计算波形的函数，这个函数只根据当前的时刻计算波形，而不考虑上一时刻的状态
 * 
 * @param phase_normalized 在当前状态（支撑相/摆动相）中的归一化相位
 * @param leg_state 腿的状态
 * @param status 处于生成波形态、完全摆动态还是完全支撑态
 */
void WaveGenerator::CalWave(Vec4& phase_normalized, std::vector<LegState>& leg_state, WaveStatus status)
{
    if(status == WaveStatus::WAVE_ALL)
    {
        float pass_time = (current_time_ - start_time_)*1e-6; //单位s
        for(int i = 0; i < 4; i++)
        {
            phase_in_full_cycle_[i] = fmod(pass_time + period_ - period_*bias_[i],period_)/period_;   
            if(phase_in_full_cycle_[i] < stance_phase_ratio_)//处于支撑相
            {
                leg_state[i] = LegState::STANCE;
                phase_normalized[i] = phase_in_full_cycle_[i]/stance_phase_ratio_;
            }
            else
            {
                leg_state[i] = LegState::SWING;
                phase_normalized[i] = (phase_in_full_cycle_[i] - stance_phase_ratio_)/(1 - stance_phase_ratio_);
            }
        }   
    }
    else if(status == WaveStatus::SWING_ALL)
    {
        for(int i = 0; i < 4; i++)
        {
            leg_state[i] = LegState::SWING;
        }
        phase_normalized << 0.5,0.5,0.5,0.5;
    }
    else if(status == WaveStatus::STANCE_ALL)   //所有腿处于支撑模式，这种情况下，直接设置腿状态为支撑相
    {
        for(int i = 0; i < 4; i++)
        {
            leg_state[i] = LegState::STANCE;
        }
        phase_normalized << 0.5,0.5,0.5,0.5;
    }
}


   



}//namespace quadruped
