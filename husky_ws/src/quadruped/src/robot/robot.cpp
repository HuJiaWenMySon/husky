#include "robot/robot.h"
#include "utils/logger.h"


namespace quadruped
{


Robot::Robot(ros::NodeHandle& nh)
{
    nh.getParam("is_sim",param_.is_sim);
    is_sim_ = param_.is_sim;

    //从ROS Parameter中获取参数
    nh.getParam("fixed_stand_q",param_.fixed_stand_q);                  //关节角
    nh.getParam("fixed_stand_duration",param_.fixed_stand_duration);    //站起来的周期数
    nh.getParam("abad_link_length",param_.abad_link_length);
    nh.getParam("hip_link_length",param_.hip_link_length);
    nh.getParam("knee_link_length",param_.knee_link_length);
    nh.getParam("accelerometer_variance",param_.accelerometer_variance);
    nh.getParam("sensor_variance",param_.sensor_variance);
    nh.getParam("initial_variance",param_.initial_variance);
    nh.getParam("period",param_.period);
    nh.getParam("stance_phase_ratio",param_.stance_phase_ratio);
    nh.getParam("gait_height",param_.gait_height);

    //步态的偏置
    std::vector<float> bias;
    nh.getParam("bias",bias);
    for (int i = 0; i < 4; i++) 
    {
        param_.bias[i] = bias[i];
    }

    //髋部想对于质心的偏移
    std::vector<std::vector<float>> hip_offset(4);
    for (int i = 0; i < 4; i++) 
    {
        nh.getParam("hip_offset_"+std::to_string(i),hip_offset[i]);
        for(int j = 0; j< 3; j++)
        {
            param_.hip_offset(j,i) = hip_offset[i][j];
        }
    }

    std::vector<std::vector<float>> foot_positions_normal(4);
    for(int i = 0; i < 4; i++)
    {
        nh.getParam("foot_positions_normal_" + std::to_string(i),foot_positions_normal[i]);
        for(int j = 0; j < 3; j++)
        {
            param_.foot_positions_normal(j,i) = foot_positions_normal[i][j];
        }
    }

    std::vector<float> v_x_limit(2);
    std::vector<float> v_y_limit(2);
    std::vector<float> v_yaw_limit(2);
    nh.getParam("v_x_limit",v_x_limit);
    nh.getParam("v_y_limit",v_y_limit);
    nh.getParam("v_yaw_limit",v_yaw_limit);
    param_.v_x_limit << v_x_limit[0],v_x_limit[1];
    param_.v_y_limit << v_y_limit[0],v_y_limit[1];
    param_.v_yaw_limit << v_yaw_limit[0],v_yaw_limit[1];

    param_.mpc_q.resize(12);
    nh.getParam("Q",param_.mpc_q);

    nh.getParam("total_mass",param_.total_mass);
    nh.getParam("total_inertia",param_.total_inertia);

    total_mass_ = param_.total_mass;

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            total_inertia_(i,j) = param_.total_inertia[i*3+j];
        }
    }

    //初始化机器人学参数
    QuadrupedParam quad_param;
    quad_param.abad_link_length = param_.abad_link_length;
    quad_param.hip_link_length = param_.hip_link_length;
    quad_param.knee_link_length = param_.knee_link_length;
    quad_param.hip_offset = param_.hip_offset;
    quadruped_  = new Quadruped(quad_param);

    //初始化相关数据
    motor_commands_.resize(12);
    motor_states_.resize(12);
    force_sensor_data_.resize(4);

    leg_state_.resize(4);

    rpy_filter_ = MovingAverageFilter<3>(5);
    gyro_filter_ = MovingAverageFilter<3>(5);

    base_position_ << 0.0f,0.0f,0.0f;//直接将启动位置设置为世界坐标系原点
}


Robot::~Robot()
{
    delete quadruped_;
}


void Robot::RecvTransState()
{
    if(start_time_ == 0)    //第一次运行
    {
        first_time_run_ = true;
        start_time_ = ros::Time::now().toNSec()/1000;//us
        last_time_ = start_time_;
    }
    else
    {
        first_time_run_ = false;
        long long current_time = ros::Time::now().toNSec()/1000;
        tick_ = current_time - start_time_;
        period_ = float(current_time - last_time_)/1000000.0;
        last_time_ = current_time;
    }
    // ROS_INFO("period: %f, tick: %lld",period_,tick_);

    RecvState();
    TransState();
}


void Robot::SendTransCommand()
{
    TransCommand();
    SendCommand();
}




}//namespace quadruped
