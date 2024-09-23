#include "controller/mpc/mpc_stance_leg_controller.h"
#include "utils/logger.h"


namespace quadruped
{


MpcStanceLegController::MpcStanceLegController(Robot* robot, GaitGenerator* gait)
{
    robot_ = robot;         //机器人结构体
    gait_ = gait;           //步态生成器
    use_wbc_ = false;       //默认不使用WBC
    dt_ = robot_->dt_;      //控制频率
    dt_mpc_ = 0.03;         //40ms执行一次MPC
    iterations_in_mpc_ = round(dt_mpc_/dt_);    //运行一次MPC的间隔周期
    horizon_length_ = 6;   //预测区间为10

    rpy_int_ << 0.0f,0.0f,0.0f;     //RPY积分初始化
    rpy_comp_ << 0.0f,0.0f,0.0f;    //RPY校正初始化
    
    //MPC的Q参数初始化
    for(int i = 0; i < 12; i++)
    {
        Q_[i] = robot_->param_.mpc_q[i];
    }

    Reset();    //重置MPC参数
    LOG_INFO("MPC Init Success: dtMPC:%f, iteration: %d, horizon: %d.",dt_mpc_,iterations_in_mpc_,horizon_length_);
}


MpcStanceLegController::~MpcStanceLegController()
{
    
}


void MpcStanceLegController::Reset()
{
    rpy_comp_.setZero();

    f_ff_.setZero();
    f_.setZero();

    mpc_table_.resize(horizon_length_, 4);
    mpc_table_.setOnes();

    height_ = robot_->height_;                                          //机器人当前的高度
    yaw_desired_ = robot_->base_rpy_[2];                                //机器人当前的欧拉角
    yaw_turn_rate_ = 0;                                                 //不旋转
    
    double max_force = robot_->total_mass_ * 9.81;                      //不能超过重力，否则就飞起来了
    double friction_coeff = 0.45;                                       //摩擦系数                    
    float* weight = Q_;                                                 //权重系数
    float alpha = 4e-6;                                                 //一个很小的值，在满足约束的情况下，并不care能耗

    Vec3 inertia;                                                       
    inertia << robot_->total_inertia_(0,0),robot_->total_inertia_(1,1),robot_->total_inertia_(2,2);
    SetUpProblem(dt_mpc_,horizon_length_,friction_coeff,max_force,robot_->total_mass_,inertia.data(),weight,alpha); 

    iteration_counter_ = 0; //迭代计数器初始化为0
}




void MpcStanceLegController::Run()
{
    base_velocity_desired_in_world_frame_ = robot_->motion_command_.base_velocity_desired_in_world_frame;  //世界坐标系下期望的机身速度
    base_position_desired_in_world_frame_ = robot_->motion_command_.base_position_desired_in_world_frame;  //世界坐标系下期望的机身位置
    yaw_turn_rate_ = robot_->motion_command_.yaw_turn_rate;
    yaw_desired_ = robot_->motion_command_.yaw_desired;
    roll_desired_ = robot_->motion_command_.roll_desired;
    pitch_desired_ = robot_->motion_command_.pitch_desired;

    base_velocity_in_world_frame_ = robot_->base_velocity_in_world_frame_;  //当前速度

    // //使用一个积分反馈对roll-pitch进行补偿
    // if(fabs(base_velocity_in_world_frame_[0]) > 0.2)     //如果x轴方向的速度比较大，考虑对pitch进行补偿
    // {
    //     rpy_int_[1] += dt_*(pitch_desired_ - robot_->base_rpy_[1])/base_velocity_in_world_frame_[0]; 
    // }
    // if(fabs(base_velocity_in_world_frame_[1]) > 0.1)     //如果y轴方向的速度比较大，考虑对roll进行补偿
    // {
    //     rpy_int_[0] += dt_*(roll_desired_ - robot_->base_rpy_[0])/base_velocity_in_world_frame_[1]; 
    // }
    // rpy_int_[0] = fminf(fmaxf(rpy_int_[0], -.25), .25); //限幅防止积分饱和
    // rpy_int_[1] = fminf(fmaxf(rpy_int_[1], -.25), .25); 
    // rpy_comp_[1] = base_velocity_in_world_frame_[0] * rpy_int_[1];   //计算补偿之后的roll-pitch
    // rpy_comp_[0] = base_velocity_in_world_frame_[1] * rpy_int_[0];   //如果比期望小，这里就是一个正值，使得欧拉角往正方向运动

    //MPC Table
    Vec4 progress = gait_->GetPhaseInFullCycle();
    float gait_period = gait_->GetGaitPeriod();
    float d_phase = dt_mpc_/gait_period;     

    for(int i = 0; i < horizon_length_; i++)        
    {
        for(int j = 0; j < 4; j++)
        {
            float mpc_phase_i = progress[j] + i * d_phase;      //计算每个预测区间内的步态

            while (mpc_phase_i > 1.0)                           //归一化到(0,1)
            {
                mpc_phase_i  -= 1.0;
            }
            if (mpc_phase_i < gait_->GetStancePhaseRatio())     //处于支撑相
            {
                mpc_table_(i, j) = 1;
            } 
            else                                                //处于摆动相，不考虑，后面会在约束里使用到
            {
                mpc_table_(i, j) = 0;
            }
        }
    }

    UpdateMpc();    

    iteration_counter_++;   //MPC计数器增长
}


/**
 * @brief 调用MPC求解器，进行解算
 * 
 */
void MpcStanceLegController::UpdateMpc()
{
    if((iteration_counter_ % iterations_in_mpc_) == 0)
    {
        const float max_pos_error = 0.1;
        float x_start = base_position_desired_in_world_frame_[0];       //初始的位置
        float y_start = base_position_desired_in_world_frame_[1];

        //期望的位置和实际的位置不能相差太大
        Vec3 base_position_in_world_frame = robot_->base_position_;
        if(x_start - base_position_in_world_frame[0] > max_pos_error) x_start = base_position_in_world_frame[0] + max_pos_error;
        if(base_position_in_world_frame[0] - x_start > max_pos_error) x_start = base_position_in_world_frame[0] - max_pos_error;

        if(y_start - base_position_in_world_frame[1] > max_pos_error) y_start = base_position_in_world_frame[1] + max_pos_error;
        if(base_position_in_world_frame[1] - y_start > max_pos_error) y_start = base_position_in_world_frame[1] - max_pos_error;
    
        base_position_desired_in_world_frame_[0] = x_start;
        base_position_desired_in_world_frame_[1] = y_start;

        // LOG_INFO("roll %f,pitch %f.",roll_desired_,pitch_desired_);
        //初始的轨迹
        float traj_initial[12] = {
            // (float)rpy_comp_[0],                            //修正后的roll
            // (float)rpy_comp_[1],                            //修正后的pitch
            roll_desired_,
            pitch_desired_,
            yaw_desired_,                                   //期望的yaw    
            x_start,                                        //初始的x                               
            y_start,                                        //初始的y          
            (float)height_,                                 //高度    
            0,                                              //roll的速率       
            0,                                              //pitch的速率
            yaw_turn_rate_,                                 //yaw的速率
            base_velocity_desired_in_world_frame_[0],       //x方向的速率 
            base_velocity_desired_in_world_frame_[1],       //y方向的速率    
            0                                               //不希望z方向发生变化
        };    

        //为预测区间内的每个时刻生成参考轨迹
        for(int i = 0; i < horizon_length_; i++)
        {
            for(int j = 0; j < 12; j++)
            {
                traj_all_[12 * i + j] = traj_initial[j];
            }
            if (i == 0) 
            {
                traj_all_[2] = yaw_desired_;
            } 
            else 
            {
                traj_all_[12 * i + 2] = traj_all_[12 * (i - 1) + 2] + dt_mpc_ * yaw_turn_rate_;
                traj_all_[12 * i + 3] = traj_all_[12 * (i - 1) + 3] + dt_mpc_ * base_velocity_desired_in_world_frame_[0];
                traj_all_[12 * i + 4] = traj_all_[12 * (i - 1) + 4] + dt_mpc_ * base_velocity_desired_in_world_frame_[1];
            }
        }   
          
        SolveDenseMpc();                      
    }
}


/**
 * @brief 调用MPC求解器求解MPC
 * 
 */
void MpcStanceLegController::SolveDenseMpc()
{
    Vec3 rpy = robot_->base_rpy_;                       //欧拉角
    Vec3 p = robot_->base_position_;                    //当前的位置
    Quat quat = robot_->base_orientation_;              //当前的姿态（四元数表示）

    Mat3 Rwb = QuatToRotMat(robot_->base_orientation_); //基座到世界坐标系的旋转矩阵

    Vec3 v = robot_->base_velocity_in_world_frame_;     //当前的速度
    Vec3 w = Rwb*robot_->base_rpy_rate_;                //角速度旋转到世界坐标系下

    Mat34 foot_to_base_position_in_world_frame;         //足端到基座的位置向量在世界坐标系下的表示

    foot_to_base_position_in_world_frame = Rwb*robot_->foot_to_base_positions_in_base_frame_;   //基座坐标系转换到世界坐标系

    SolveMpcKernel(p,v,rpy,w,foot_to_base_position_in_world_frame,quat,traj_all_,mpc_table_.data());    //求解MPC

    for (int leg = 0; leg < 4; leg++)                       //腿的编号
    {
        for (int axis = 0; axis < 3; axis++)                //力的维数
        {
            f_(axis, leg) = GetMpcSolution(leg * 3 + axis); //[fx1,fy1,fz1,fx2,fy2,...]
        }
        f_ff_.col(leg) = -Rwb.transpose() * f_.col(leg);    //从世界坐标系转移到机身坐标系
    }
}   


}//namespace quadruped