#include "robot/robot_go1_sim.h"
#include "utils/logger.h"


namespace quadruped
{


RobotGo1Sim::RobotGo1Sim(ros::NodeHandle& nh)
    :Robot(nh)
{
    for(int i = 0; i < 12; i++)
    {
        low_cmd_.motorCmd[i].q = 0.0f;
        low_cmd_.motorCmd[i].dq = 0.0f;
        low_cmd_.motorCmd[i].Kp = 0.0f;
        low_cmd_.motorCmd[i].Kd = 0.0f;
        low_cmd_.motorCmd[i].tau = 0.0f;
    }

    joint_state_pub_[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/FR_hip_controller/command", 1);//FL_ABAD
    joint_state_pub_[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/FR_thigh_controller/command", 1);//FL_HIP
    joint_state_pub_[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/FR_calf_controller/command", 1);//FL_KNEE
    joint_state_pub_[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/FL_hip_controller/command", 1);//FR_ABAD
    joint_state_pub_[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/FL_thigh_controller/command", 1);//FR_HIP
    joint_state_pub_[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/FL_calf_controller/command", 1);//FR_KNEE
    joint_state_pub_[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/RR_hip_controller/command", 1);//RR_ABAD
    joint_state_pub_[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/RR_thigh_controller/command", 1);//RR_HIP
    joint_state_pub_[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/RR_calf_controller/command", 1);//RR_KNEE
    joint_state_pub_[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/RL_hip_controller/command", 1);//FL_ABAD
    joint_state_pub_[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/RL_thigh_controller/command", 1);//FL_HIP
    joint_state_pub_[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("go1_gazebo/RL_calf_controller/command", 1);//FL_KNEE

    joint_state_sub_[0] = nh.subscribe("go1_gazebo/FR_hip_controller/state", 1, &RobotGo1Sim::FrHipCallback, this);
    joint_state_sub_[1] = nh.subscribe("go1_gazebo/FR_thigh_controller/state", 1, &RobotGo1Sim::FrThighCallback, this);
    joint_state_sub_[2] = nh.subscribe("go1_gazebo/FR_calf_controller/state", 1, &RobotGo1Sim::FrCalfCallback, this);
    joint_state_sub_[3] = nh.subscribe("go1_gazebo/FL_hip_controller/state", 1, &RobotGo1Sim::FlHipCallback, this);
    joint_state_sub_[4] = nh.subscribe("go1_gazebo/FL_thigh_controller/state", 1, &RobotGo1Sim::FlThighCallback, this);
    joint_state_sub_[5] = nh.subscribe("go1_gazebo/FL_calf_controller/state", 1, &RobotGo1Sim::FlCalfCallback, this);
    joint_state_sub_[6] = nh.subscribe("go1_gazebo/RR_hip_controller/state", 1, &RobotGo1Sim::RrHipCallback, this);
    joint_state_sub_[7] = nh.subscribe("go1_gazebo/RR_thigh_controller/state", 1, &RobotGo1Sim::RrThighCallback, this);
    joint_state_sub_[8] = nh.subscribe("go1_gazebo/RR_calf_controller/state", 1, &RobotGo1Sim::RrCalfCallback, this);
    joint_state_sub_[9] = nh.subscribe("go1_gazebo/RL_hip_controller/state", 1, &RobotGo1Sim::RlHipCallback, this);
    joint_state_sub_[10] = nh.subscribe("go1_gazebo/RL_thigh_controller/state", 1, &RobotGo1Sim::RlThighCallback, this);
    joint_state_sub_[11] = nh.subscribe("go1_gazebo/RL_calf_controller/state", 1, &RobotGo1Sim::RlCalfCallback, this);

    imu_sub_ = nh.subscribe("trunk_imu",1,&RobotGo1Sim::ImuCallback,this);
    foot_force_sub_[0] = nh.subscribe("/visual/FR_foot_contact/the_force", 1, &RobotGo1Sim::FrFootCallback, this);
    foot_force_sub_[1] = nh.subscribe("/visual/FL_foot_contact/the_force", 1, &RobotGo1Sim::FlFootCallback, this);
    foot_force_sub_[2] = nh.subscribe("/visual/RR_foot_contact/the_force", 1, &RobotGo1Sim::RrFootCallback, this);
    foot_force_sub_[3] = nh.subscribe("/visual/RL_foot_contact/the_force", 1, &RobotGo1Sim::RlFootCallback, this);

    height_ = 0.28;
}



RobotGo1Sim::~RobotGo1Sim()
{

}


void RobotGo1Sim::TransCommand()
{
    //关节空间到执行器空间
    for(int i = 0; i< 12; i++)
    {
        motor_commands_[i].q = joint_command_angles_[i];
        motor_commands_[i].dq = joint_command_velocities_[i];
        motor_commands_[i].kp = joint_command_kp_[i];
        motor_commands_[i].kd = joint_command_kd_[i];
        motor_commands_[i].tau = joint_command_torque_[i];
    }
}



void RobotGo1Sim::SendCommand()
{
    //将数据设置到low_cmd
    for(int i = 0; i < 12; i++)
    {
        low_cmd_.motorCmd[i].mode = 0x0A;    
        low_cmd_.motorCmd[i].q = motor_commands_[i].q;
        low_cmd_.motorCmd[i].dq = motor_commands_[i].dq;
        low_cmd_.motorCmd[i].Kp = motor_commands_[i].kp;
        low_cmd_.motorCmd[i].Kd = motor_commands_[i].kd;
        low_cmd_.motorCmd[i].tau = motor_commands_[i].tau;
    }

    //将数据发布到gazebo
    for(int i = 0; i < 12; i++)
    {
        joint_state_pub_[i].publish(low_cmd_.motorCmd[i]);
    }
}



void RobotGo1Sim::RecvState()
{
    //数据拷贝到motor_state
    for(int i = 0; i < 12; i++)
    {
        motor_states_[i].q = low_state_.motorState[i].q;
        motor_states_[i].dq = low_state_.motorState[i].dq;
        motor_states_[i].ddq = low_state_.motorState[i].ddq;
        motor_states_[i].tau = low_state_.motorState[i].tauEst;
        // LOG_INFO("motor:%d, %.2f, %.2f, %.2f, %.2f.",i,motor_states_[i].q,
        //          motor_states_[i].dq,motor_states_[i].ddq,motor_states_[i].tau);
    }

    imu_data_.angular_velocity[0] = low_state_.imu.gyroscope[0];
    imu_data_.angular_velocity[1] = low_state_.imu.gyroscope[1];
    imu_data_.angular_velocity[2] = low_state_.imu.gyroscope[2];

    imu_data_.linear_acceleration[0] = low_state_.imu.accelerometer[0];
    imu_data_.linear_acceleration[1] = low_state_.imu.accelerometer[1];
    imu_data_.linear_acceleration[2] = low_state_.imu.accelerometer[2];

    imu_data_.quaternion[0] = low_state_.imu.quaternion[0];
    imu_data_.quaternion[1] = low_state_.imu.quaternion[1];
    imu_data_.quaternion[2] = low_state_.imu.quaternion[2]; 
    imu_data_.quaternion[3] = low_state_.imu.quaternion[3]; 

    imu_data_.rpy[0] = low_state_.imu.rpy[0];
    imu_data_.rpy[1] = low_state_.imu.rpy[1];
    imu_data_.rpy[2] = low_state_.imu.rpy[2];

    for(int i = 0;i <4;i++)
    {
        force_sensor_data_[i].force_x = low_state_.eeForce[i].x;
        force_sensor_data_[i].force_y = low_state_.eeForce[i].y;
        force_sensor_data_[i].force_z = low_state_.eeForce[i].z;
    }

    // LOG_INFO("Angular_velocity:%f, %f, %f.",imu_.angular_velocity[0],imu_.angular_velocity[1],imu_.angular_velocity[2]);
    // LOG_INFO("Linear_acceleration:%f, %f, %f.",imu_.linear_acceleration[0],imu_.linear_acceleration[1],imu_.linear_acceleration[2]);
    // LOG_INFO("Quaternion:%f, %f, %f, %f.",imu_.quaternion[0],imu_.quaternion[1],imu_.quaternion[2],imu_.quaternion[3]);
    // LOG_INFO("RPY: %f,%f,%f.",imu_.rpy[0],imu_.rpy[1],imu_.rpy[2]);
} 


void RobotGo1Sim::TransState()
{
   //基坐标系下的线性加速度，直接从IMU中读取即可
    base_acc_in_base_frame_ << imu_data_.linear_acceleration[0] 
                            ,imu_data_.linear_acceleration[1]
                            ,imu_data_.linear_acceleration[2];

    //欧拉角，对于偏航角，把它归一化到初始的方向上，范围为[-pi,pi]
    float rpy[3] = {imu_data_.rpy[0],imu_data_.rpy[1],imu_data_.rpy[2]};
    float calibrated_yaw = imu_data_.rpy[2] - yaw_offset_;
    if(calibrated_yaw >= M_PI) 
        calibrated_yaw -= 2*M_PI;
    else if(calibrated_yaw <= -M_PI) 
        calibrated_yaw += 2*M_PI;

    Vec3 rpy_vec(rpy[0], rpy[1], calibrated_yaw);
    base_rpy_ = rpy_filter_.CalAverage(rpy_vec);
    if(base_rpy_ [2] >= M_PI) 
        base_rpy_ [2] -= 2*M_PI;
    else if(base_rpy_ [2] <= -M_PI) 
        base_rpy_ [2] += 2*M_PI;

    //四元数，直接通过滤波后的欧拉角进行转换
    base_orientation_ = RpyToQuat(base_rpy_[0],base_rpy_[1],base_rpy_[2]);

    //角速率
    Vec3 gyro(imu_data_.angular_velocity[0],imu_data_.angular_velocity[1],imu_data_.angular_velocity[2]);
    base_rpy_rate_ = gyro_filter_.CalAverage(gyro);//滑动平均滤波

    //关节参数从执行器空间转换到关节空间，因为采用了宇树的机器人学模型，因此这里直接复制即可
    for(int i = 0; i <12; i++)
    {
        joint_state_angles_[i] = motor_states_[i].q;
        joint_state_velocities_[i] = motor_states_[i].dq;
        joint_state_ddq_[i] = motor_states_[i].ddq;
        joint_state_torque_[i] = motor_states_[i].tau;
        // LOG_INFO("Joint %d Angle: %f, Velocity: %f, Ddq: %f, Toque: %f.",i,joint_state_angles_[i],joint_state_velocities_[i],
                                                                        // joint_state_ddq_[i],joint_state_torque_[i]);
    }

    for(int i = 0; i < 4; i++)
    {
        foot_force_[i] = force_sensor_data_[i].force_z; //保存足底力
        if(force_sensor_data_[i].force_z >= 5)//判断足底是否接触，阈值参数待调整
        {
            foot_contact_[i] = true;    
        }
        else
        {
            foot_contact_[i] = false;
        }
    }

    //打印出来
    // LOG_INFO("Base Acc: %f,%f,%f.",base_acc_in_base_frame_[0],base_acc_in_base_frame_[1],base_acc_in_base_frame_[2]);
    // LOG_INFO("Base rpy rate: %f,%f,%f.",base_rpy_rate_[0],base_rpy_rate_[1],base_rpy_rate_[2]);
    // LOG_INFO("Base RPY: %f,%f,%f.",base_rpy_[0],base_rpy_[1],base_rpy_[2]);
    // LOG_INFO("Base Orientation: %f,%f,%f,%f.",base_orientation_[0],base_orientation_[1],base_orientation_[2],base_orientation_[3]);
    // LOG_INFO("Foot force: %f, %f, %f, %f",foot_force_[0],foot_force_[1],foot_force_[2],foot_force_[3]);
    // LOG_INFO("Foot contact: %d, %d, %d, %d",foot_contact_[0],foot_contact_[1],foot_contact_[2],foot_contact_[3]);
    
    //将12×1向量转化为3×4向量
    // for(int i = 0; i < 4 ; i++)
    // {
    //     leg_state_angles_.col(i) = joint_state_angles_.block<3,1>(i*3,0);
    //     leg_state_velocities_.col(i) = joint_state_velocities_.block<3,1>(i*3,0);
    // }
    leg_state_angles_ = Vec12ToMat34(joint_state_angles_);
    leg_state_velocities_ = Vec12ToMat34(joint_state_velocities_);

    foot_to_base_positions_in_base_frame_ = quadruped_->GetFootToBasePositions(leg_state_angles_);//正解获得基座坐标系下的足端位置
    foot_to_base_velocities_in_base_frame_ = quadruped_->GetFootToBaseVelocities(leg_state_angles_,leg_state_velocities_);//通过Jacobian求足端相对于基坐标系的速度

    // for(int i = 0;i<4;i++)
    // {
    //     LOG_INFO("leg %d, pos:(%f,%f,%f), vel:(%f,%f,%f).",i,foot_to_base_positions_in_base_frame_.col(i)[0],foot_to_base_positions_in_base_frame_.col(i)[1],foot_to_base_positions_in_base_frame_.col(i)[2],
    //                                                         foot_to_base_velocities_in_base_frame_.col(i)[0],foot_to_base_velocities_in_base_frame_.col(i)[1],foot_to_base_velocities_in_base_frame_.col(i)[2]);
    // }

    // //这几行代码可以测试下正逆解是否正确
    // Vec12 dq_inv = quadruped_->GetQd(foot_to_base_positions_in_base_frame_,foot_to_base_velocities_in_base_frame_);
    // Vec12 q_inv = quadruped_->GetQ(foot_to_base_positions_in_base_frame_);

    // for(int i = 0; i < 12; i++)
    // {
    //     LOG_INFO("q_inv: %f",q_inv[i]);
    // }

    // for(int i = 0; i < 12; i++)
    // {
    //     LOG_INFO("dq_inv: %f",dq_inv[i]);
    // }

    // 足端位置打印出来
    // LOG_INFO("Foot Pos in Base: FR(%f,%f,%f)", foot_to_base_positions_in_base_frame_(0,0),foot_to_base_positions_in_base_frame_(1,0),foot_to_base_positions_in_base_frame_(2,0));
    // LOG_INFO("Foot Pos in Base: FL(%f,%f,%f)", foot_to_base_positions_in_base_frame_(0,1),foot_to_base_positions_in_base_frame_(1,1),foot_to_base_positions_in_base_frame_(2,1));
    // LOG_INFO("Foot Pos in Base: RR(%f,%f,%f)", foot_to_base_positions_in_base_frame_(0,2),foot_to_base_positions_in_base_frame_(1,2),foot_to_base_positions_in_base_frame_(2,2));
    // LOG_INFO("Foot Pos in Base: RL(%f,%f,%f)", foot_to_base_positions_in_base_frame_(0,3),foot_to_base_positions_in_base_frame_(1,3),foot_to_base_positions_in_base_frame_(2,3));
    // LOG_INFO("Foot Vel in Base: FR(%f,%f,%f)", foot_to_base_velocities_in_base_frame_(0,0),foot_to_base_velocities_in_base_frame_(1,0),foot_to_base_velocities_in_base_frame_(2,0));
    // LOG_INFO("Foot Vel in Base: FL(%f,%f,%f)", foot_to_base_velocities_in_base_frame_(0,1),foot_to_base_velocities_in_base_frame_(1,1),foot_to_base_velocities_in_base_frame_(2,1));
    // LOG_INFO("Foot Vel in Base: RR(%f,%f,%f)", foot_to_base_velocities_in_base_frame_(0,2),foot_to_base_velocities_in_base_frame_(1,2),foot_to_base_velocities_in_base_frame_(2,2));
    // LOG_INFO("Foot Vel in Base: RL(%f,%f,%f)", foot_to_base_velocities_in_base_frame_(0,3),foot_to_base_velocities_in_base_frame_(1,3),foot_to_base_velocities_in_base_frame_(2,3));
}


void RobotGo1Sim::FrHipCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[0].mode = msg.mode;
    low_state_.motorState[0].q = msg.q;
    low_state_.motorState[0].dq = msg.dq;
    low_state_.motorState[0].ddq = msg.ddq;
    low_state_.motorState[0].tauEst = msg.tauEst;
}

void RobotGo1Sim::FrThighCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[1].mode = msg.mode;
    low_state_.motorState[1].q = msg.q;
    low_state_.motorState[1].dq = msg.dq;
    low_state_.motorState[1].ddq = msg.ddq;
    low_state_.motorState[1].tauEst = msg.tauEst;
}

void RobotGo1Sim::FrCalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[2].mode = msg.mode;
    low_state_.motorState[2].q = msg.q;
    low_state_.motorState[2].dq = msg.dq;
    low_state_.motorState[2].ddq = msg.ddq;
    low_state_.motorState[2].tauEst = msg.tauEst;
}

void RobotGo1Sim::FlHipCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[3].mode = msg.mode;
    low_state_.motorState[3].q = msg.q;
    low_state_.motorState[3].dq = msg.dq;
    low_state_.motorState[3].ddq = msg.ddq;
    low_state_.motorState[3].tauEst = msg.tauEst;
}

void RobotGo1Sim::FlThighCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[4].mode = msg.mode;
    low_state_.motorState[4].q = msg.q;
    low_state_.motorState[4].dq = msg.dq;
    low_state_.motorState[4].ddq = msg.ddq;
    low_state_.motorState[4].tauEst = msg.tauEst;
}

void RobotGo1Sim::FlCalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[5].mode = msg.mode;
    low_state_.motorState[5].q = msg.q;
    low_state_.motorState[5].dq = msg.dq;
    low_state_.motorState[5].ddq = msg.ddq;
    low_state_.motorState[5].tauEst = msg.tauEst;
}


void RobotGo1Sim::RrHipCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[6].mode = msg.mode;
    low_state_.motorState[6].q = msg.q;
    low_state_.motorState[6].dq = msg.dq;
    low_state_.motorState[6].ddq = msg.ddq;
    low_state_.motorState[6].tauEst = msg.tauEst;
}

void RobotGo1Sim::RrThighCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[7].mode = msg.mode;
    low_state_.motorState[7].q = msg.q;
    low_state_.motorState[7].dq = msg.dq;
    low_state_.motorState[7].ddq = msg.ddq;
    low_state_.motorState[7].tauEst = msg.tauEst;
}

void RobotGo1Sim::RrCalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[8].mode = msg.mode;
    low_state_.motorState[8].q = msg.q;
    low_state_.motorState[8].dq = msg.dq;
    low_state_.motorState[8].ddq = msg.ddq;
    low_state_.motorState[8].tauEst = msg.tauEst;
}

void RobotGo1Sim::RlHipCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[9].mode = msg.mode;
    low_state_.motorState[9].q = msg.q;
    low_state_.motorState[9].dq = msg.dq;
    low_state_.motorState[9].ddq = msg.ddq;
    low_state_.motorState[9].tauEst = msg.tauEst;
}

void RobotGo1Sim::RlThighCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[10].mode = msg.mode;
    low_state_.motorState[10].q = msg.q;
    low_state_.motorState[10].dq = msg.dq;
    low_state_.motorState[10].ddq = msg.ddq;
    low_state_.motorState[10].tauEst = msg.tauEst;
}


void RobotGo1Sim::RlCalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    low_state_.motorState[11].mode = msg.mode;
    low_state_.motorState[11].q = msg.q;
    low_state_.motorState[11].dq = msg.dq;
    low_state_.motorState[11].ddq = msg.ddq;
    low_state_.motorState[11].tauEst = msg.tauEst;
}



void RobotGo1Sim::ImuCallback(const sensor_msgs::Imu &msg)
{
    low_state_.imu.accelerometer[0] = msg.linear_acceleration.x;
    low_state_.imu.accelerometer[1] = msg.linear_acceleration.y;
    low_state_.imu.accelerometer[2] = msg.linear_acceleration.z;

    low_state_.imu.gyroscope[0] = msg.angular_velocity.x;
    low_state_.imu.gyroscope[1] = msg.angular_velocity.y;
    low_state_.imu.gyroscope[2] = msg.angular_velocity.z;

    low_state_.imu.quaternion[0] = msg.orientation.w;
    low_state_.imu.quaternion[1] = msg.orientation.x;
    low_state_.imu.quaternion[2] = msg.orientation.y;
    low_state_.imu.quaternion[3] = msg.orientation.z;

    Quat quaternion = {low_state_.imu.quaternion[0],
                        low_state_.imu.quaternion[1],
                        low_state_.imu.quaternion[2],
                        low_state_.imu.quaternion[3]};
    Vec3 rpy = QuatToRpy(quaternion);//四元数转换到欧拉角

    low_state_.imu.rpy[0] = rpy[0];
    low_state_.imu.rpy[1] = rpy[1];
    low_state_.imu.rpy[2] = rpy[2];
}


void RobotGo1Sim::FrFootCallback(const geometry_msgs::WrenchStamped &msg)
{
    low_state_.eeForce[0].x = msg.wrench.force.x;//足端是一个一维力，但在Cartesion坐标系下进行了分解
    low_state_.eeForce[0].y = msg.wrench.force.y;
    low_state_.eeForce[0].z = msg.wrench.force.z;
    low_state_.footForce[0] = msg.wrench.force.z;//footForce 变量只保存z方向上的力
}


void RobotGo1Sim::FlFootCallback(const geometry_msgs::WrenchStamped &msg)
{
    low_state_.eeForce[1].x = msg.wrench.force.x;//足端是一个一维力，但在Cartesion坐标系下进行了分解
    low_state_.eeForce[1].y = msg.wrench.force.y;
    low_state_.eeForce[1].z = msg.wrench.force.z;
    low_state_.footForce[1] = msg.wrench.force.z;//footForce 变量只保存z方向上的力
}


void RobotGo1Sim::RrFootCallback(const geometry_msgs::WrenchStamped &msg)
{
    low_state_.eeForce[2].x = msg.wrench.force.x;//足端是一个一维力，但在Cartesion坐标系下进行了分解
    low_state_.eeForce[2].y = msg.wrench.force.y;
    low_state_.eeForce[2].z = msg.wrench.force.z;
    low_state_.footForce[2] = msg.wrench.force.z;//footForce 变量只保存z方向上的力
}


void RobotGo1Sim::RlFootCallback(const geometry_msgs::WrenchStamped &msg)
{
    low_state_.eeForce[3].x = msg.wrench.force.x;//足端是一个一维力，但在Cartesion坐标系下进行了分解
    low_state_.eeForce[3].y = msg.wrench.force.y;
    low_state_.eeForce[3].z = msg.wrench.force.z;
    low_state_.footForce[3] = msg.wrench.force.z;//footForce 变量只保存z方向上的力
}



};