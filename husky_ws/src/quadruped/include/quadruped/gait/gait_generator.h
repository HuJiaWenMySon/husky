#ifndef QUADRUPED_GAIT_GAIT_GENERATOR_H_
#define QUADRUPED_GAIT_GAIT_GENERATOR_H_


#include "gait/wave_generator.h"
#include "gait/foot_end_cal.h"
#include "utils/logger.h"

namespace quadruped
{

class GaitGenerator
{
public:
    GaitGenerator(Robot* robot, WaveGenerator* wave_generator);
    ~GaitGenerator();
    void SetGait(Vec2 v_xy_goal, float v_yaw_goal, float gait_height);
    void Restart();
    void Update(Mat34& foot_positions, Mat34& foot_velocities);
    Vec3 GetFootPosition(int leg_id);
    Vec3 GetFootVelocity(int leg_id);
    Vec4 GetPhaseInFullCycle(){return wave_generator_->phase_in_full_cycle_;}
    float GetGaitPeriod(){return wave_generator_->period_;}
    float GetStancePhaseRatio(){return wave_generator_->stance_phase_ratio_;}
    bool IsSwitching() {return (wave_generator_->switch_status_.sum()!=0);}

private:
    Robot* robot_;
    WaveGenerator* wave_generator_;
    FootEndCal* foot_end_cal_;
    Vec2 v_xy_goal_;
    float v_yaw_goal_;
    float gait_height_;
    bool first_run_;
    Mat34 start_position_;
    Mat34 end_position_;

    float t_stance_;    //支撑相的持续时间
    float t_swing_;     //摆动相的持续时间

    float CycloidXyPosition(float start, float end, float phase);
    float CycloidXyVelocity(float start, float end, float phase);
    float CycloidZPosition(float start, float height, float phase);
    float CycloidZVelocity(float height, float phase);
};


}//namespace quadruped



#endif//QUADRUPED_PLANNER_WAVE_GENERATOR_H_