#include "controller/mpc/mpc_interface.h"
#include "utils/logger.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace quadruped
{

#define BIG_NUMBER 5e10

//由于求解MPC问题的全局唯一性，这里相关变量统一使用静态变量
static ProblemConfig problem_config;                                //参数                                                      
static MpcRobotState robot_state;                                   //状态
static int has_solved = 0;                                          //是否已经求解

//状态空间模型参数的连续和离散形式
static Eigen::Matrix<float, 13, 13> A_c;                            //连续时间下的单刚体动力学状态矩阵
static Eigen::Matrix<float, 13, 12> B_c;                            //连续时间下的单刚体动力学输入矩阵
static Eigen::Matrix<float, 13, 13> A_d;                            //离散化之后的A矩阵
static Eigen::Matrix<float, 13, 12> B_d;                            //离散化之后的B矩阵

//QP问题
static Eigen::Matrix<float, Eigen::Dynamic, 13> Phi;                //QP的Phi矩阵
static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Gamma;  //QP的Gamma矩阵

static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Omega;  //大的状态权重矩阵 
static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Psi;    //大的输入权重矩阵

static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H;      //H矩阵
static Eigen::Matrix<float, Eigen::Dynamic, 1> G;                   //G矩阵，《控制之美》里面是F*X0，由于这里进行tracking，5.3.10中的式子进行一个简单变换可得

static Eigen::Matrix<float, Eigen::Dynamic, 1> U_b;                 //足底力的上界
static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> fmat;   //摩擦系数矩阵

static Eigen::Matrix<float, 13, 1> x0;                              //状态空间方程中的状态变量
static Eigen::Matrix<float, Eigen::Dynamic, 1> X_d;                 //期望的轨迹，调用者来提供，这里包括多个horizon，所以X大写

static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> I_horizon_12;   //单位矩阵，12×horizon，用于生成Psi
static Eigen::Matrix<float, 25, 25> AB_c, expmm;
static Eigen::MatrixXf temp;

// qpOASES定义的矩阵
static qpOASES::real_t *H_qpoases = nullptr;    //H矩阵
static qpOASES::real_t *G_qpoases = nullptr;    //G矩阵
static qpOASES::real_t *A_qpoases = nullptr;    //系数矩阵
static qpOASES::real_t *lb_qpoases = nullptr;   //下界
static qpOASES::real_t *ub_qpoases = nullptr;   //上界
static qpOASES::real_t *q_soln = nullptr;       //QP问题的解


/**
 * @brief Eigen矩阵转化为qpOASES向量
 * 
 * @param dst 目标，qpOASES数组
 * @param src 源，Eigen矩阵
 * @param rows 行
 * @param cols 列
 */
static void EigenToOASES(qpOASES::real_t *dst, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> src, int rows, int cols)
{
    int a = 0;
    for (int r = 0; r < rows; r++) 
    {
        for (int c = 0; c < cols; c++) {
            dst[a] = src(r, c);
            a++;
        }
    }
}


/**
 * @brief 设置MPC问题的参数，主要是一些不变的参数
 * 
 * @param dt MPC的执行周期，一般是30ms
 * @param horizon 预测区间
 * @param friction_coeff 摩擦系数
 * @param f_max 最大摩擦力
 * @param total_mass 总的质量
 * @param inertia 基坐标系下的惯性张量
 * @param weight 状态变量的权重参数
 * @param alpha 输入的权重参数
 */
void SetUpProblem(float dt, int horizon, float friction_coeff, float f_max, float total_mass, float* inertia, float* weight, float alpha)
{
    LOG_INFO("MPC Parameters: dtMPC: %f, horizon: %d, friction_coeff: %f, f_max: %f, total_mass: %f inertia: (%f,%f,%f), weight:(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f), alpha: %f.",
                dt,horizon,friction_coeff,f_max,total_mass,inertia[0],inertia[1],inertia[2],
                weight[0],weight[1],weight[2],weight[3],weight[4],weight[5],weight[6],weight[7],weight[8],weight[9],weight[10],weight[11],alpha);

    problem_config.dt = dt;
    problem_config.horizon = horizon;
    problem_config.friction_coeff = friction_coeff;
    problem_config.f_max = f_max;
    problem_config.total_mass = total_mass;
    problem_config.alpha = alpha;
    memcpy(problem_config.weight,weight,sizeof(float)*12);

    robot_state.body_inertia.diagonal() << inertia[0],inertia[1],inertia[2]; 
    robot_state.mass = total_mass;

    ResizeQpMats(horizon);  //根据步长重新生成MPC计算过程所需的矩阵
}


/**
 * @brief 重新生成QP矩阵的大小
 * 
 * @param horizon 预测区间
 */
void ResizeQpMats(int horizon)
{
    //预设好空间
    Phi.resize(13*horizon,Eigen::NoChange); //(13*Np,13)
    Omega.resize(13*horizon, 13*horizon);   //(13*Np,13*Np)
    Gamma.resize(13*horizon,12*horizon);    //(13*Np,12*Np)
    X_d.resize(13*horizon,Eigen::NoChange); //13*horizon长度的向量
    U_b.resize(20*horizon,Eigen::NoChange); //4条腿，一条腿5个最大值，一共20个最大值约束，因此这是一个很长的列向量
    fmat.resize(20*horizon,12*horizon);     //摩擦系数矩阵，是输入大的系数矩阵，对于一个horizon，为（5×4）×（3×4）=20×12大小，再乘以horizon
    H.resize(12*horizon, 12*horizon);       //H矩阵，p*Np = 12*Np
    G.resize(12 * horizon, Eigen::NoChange);//G矩阵，(p*Np)*1 = (12*Np)*1
    I_horizon_12.resize(12*horizon, 12*horizon);    //用于生成Psi矩阵
    temp.resize(12 * horizon, 13 * horizon);

    //初始化
    Phi.setZero();
    Omega.setZero();
    Gamma.setZero();
    X_d.setZero();
    U_b.setZero();
    fmat.setZero();
    H.setZero();
    G.setZero();
    I_horizon_12.setIdentity();

    //摩擦力矩阵不随机器人状态变化而变化，因此在这里初始化好
    float mu = 1.f/problem_config.friction_coeff;   
    Eigen::Matrix<float,5,3> f_block;
    f_block <<  mu, 0,  1.f,
                -mu, 0,  1.f,
                0,  mu, 1.f,
                0, -mu, 1.f,
                0,   0, 1.f;

    for(int i = 0; i < problem_config.horizon*4; i++)
    {
        fmat.block(i*5,i*3,5,3) = f_block;
    }

    H_qpoases = (qpOASES::real_t *)realloc(H_qpoases, 12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
    G_qpoases = (qpOASES::real_t *)realloc(G_qpoases, 12 * 1 * horizon * sizeof(qpOASES::real_t));
    A_qpoases = (qpOASES::real_t *)realloc(A_qpoases, 12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
    lb_qpoases = (qpOASES::real_t *)realloc(lb_qpoases, 20 * 1 * horizon * sizeof(qpOASES::real_t));
    ub_qpoases = (qpOASES::real_t *)realloc(ub_qpoases, 20 * 1 * horizon * sizeof(qpOASES::real_t));
    q_soln = (qpOASES::real_t *)realloc(q_soln, 12 * horizon * sizeof(qpOASES::real_t));
}


/**
 * @brief 构造连续时间状态空间方程，这是四足的简化单刚体动力学方程
 * 
 * @param inertia_world 世界坐标系下的惯性张量
 * @param mass 质量
 * @param foot_to_base_position_in_world_frame 足端相对于质心的位置（基座坐标系）
 * @param Ryaw Yaw的旋转矩阵
 * @param A 连续时间系统的状态矩阵，返回值
 * @param B 连续时间系统的输入矩阵，返回值
 */
void ComputeContinuousMatrices(Mat3 inertia_world, float mass, Mat34 foot_to_base_position_in_world_frame, Mat3 Ryaw,
                                            Eigen::Matrix<float, 13, 13>& A, Eigen::Matrix<float, 13, 12>& B)
{
    //首先填写A矩阵
    A.setZero();                                //初始化为0矩阵
    A.block(0, 6, 3, 3) = Ryaw.transpose();     //设置R^T_z矩阵

    A(3, 9) = 1.f;                              //设置一个小的单位阵，用于转换  状态dot(p)             
    A(4, 10) = 1.f;                             
    A(5, 11) = 1.f;

    A(11, 12) = 1.0f;                           //这一位主要是在求ddot(p)的时候考虑重力项


    //下面填写B矩阵
    B.setZero();                                //初始化为0矩阵

    Mat3 inertia_world_inv = inertia_world.inverse();   //惯性张量的逆，先算出来
    Mat3 mass_inv = Mat3::Identity()/mass;              //质量的逆，也先算出来
    for (int b = 0; b < 4; b++) 
    {
        B.block(6, b * 3, 3, 3) = inertia_world_inv * Skew(foot_to_base_position_in_world_frame.col(b));     //处理第三行的分块矩阵
        B.block(9, b * 3, 3, 3) = mass_inv;                                                 //处理第四行的分块矩阵
    }
}


/**
 * @brief 转化成离散形式的状态空间方程，然后整理成QP问题
 * 
 * @param A_c 连续时间状态空间方程的A矩阵
 * @param B_c 连续时间状态空间方程的B矩阵
 * @param dt 采样时间
 * @param horizon 预测区间，Np
 */
void ConvertToDiscreteQp(Eigen::Matrix<float,13,13> A_c, Eigen::Matrix<float,13,12> B_c, float dt, int horizon)
{
    //离散化
    AB_c.setZero();
    AB_c.block(0, 0, 13, 13) = A_c;
    AB_c.block(0, 13, 13, 12) = B_c;
    AB_c = dt * AB_c;
    expmm = AB_c.exp();
    A_d = expmm.block(0, 0, 13, 13);
    B_d = expmm.block(0, 13, 13, 12); 


    //QP需要计算A矩阵的幂，存储在power_mats中
    Eigen::Matrix<float, 13, 13> power_mats[20];
    power_mats[0].setIdentity();
    for (int i = 1; i < horizon + 1; i++)   //1~Np
    {
        power_mats[i] = A_d * power_mats[i - 1];
    }

    //计算Phi矩阵和Gamma矩阵
    //Phi = [A A^2 A^3 ... A^Np]，唯独是(nNp)*n，构造时，从1到Np逐个构造N×N矩阵
    //Gamma参见5.3.5c式
    for(int r = 0; r < horizon; r++)
    {
        Phi.block(13*r, 0, 13, 13) = power_mats[r+1];   
        for(int c = 0; c < horizon; c++)
        {
            if(r >= c)//行数大于列数，这里的行和列指的是分块矩阵，所以都是Np
            {
                Gamma.block(13*r,12*c,13,12) = power_mats[r-c]*B_d;  //Gamma中A的幂取决于行和列的差值
            }
        }
    }
}


/**
 * @brief 准备MPC数据，然后构造并求解QP问题，这里主要是每次迭代所需新的参数
 * 
 * @param p 世界坐标系下的位置
 * @param v 世界坐标系下的速度
 * @param rpy 欧拉角
 * @param w 世界坐标系下的角速度
 * @param foot_to_base_position_in_world_frame 足端到质心的位置向量
 * @param base_orientation 姿态，四元数
 * @param state_trajectory 状态轨迹
 * @param gait 预测区间内的步态相位，用于生成约束
 */
void SolveMpcKernel(Vec3& p, Vec3& v, Vec3& rpy, Vec3& w, Mat34& foot_to_base_position_in_world_frame, 
                    Quat base_orientation, float* state_trajectory, float* gait)
{
    Mat34 f2bp = foot_to_base_position_in_world_frame;

    static int times = 0;
    // LOG_INFO("-------------------------------------------------------");
    // LOG_INFO("mpc times: %d.",times++);

    // LOG_INFO("p: %f,%f,%f.",p[0],p[1],p[2]);
    // LOG_INFO("v: %f,%f,%f.",v[0],v[1],v[2]);
    // LOG_INFO("rpy: %f,%f,%f.",rpy[0],rpy[1],rpy[2]);
    // LOG_INFO("w: %f,%f,%f.",w[0],w[1],w[2]);
    // LOG_INFO("foot to base in world: %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f.",
    //         f2bp(0,0),f2bp(1,0),f2bp(2,0),f2bp(0,1),f2bp(1,1),f2bp(2,1),f2bp(0,2),f2bp(1,2),f2bp(2,2),f2bp(0,3),f2bp(1,3),f2bp(2,3));
    // LOG_INFO("quat: %f,%f,%f,%f", base_orientation[0],base_orientation[1],base_orientation[2],base_orientation[3]);

    
    // LOG_INFO("gait:");
    // for(int i = 0; i < problem_config.horizon; i++)
    // {
    //     LOG_INFO("%f %f %f %f",gait[i*4],gait[i*4+1],gait[i*4+2],gait[i*4+3]);
    // }
    // LOG_INFO("traj:");
    // for(int i = 0; i < problem_config.horizon; i++)
    // {
    //     LOG_INFO("%f %f %f %f %f %f %f %f %f %f %f %f",
    //                         state_trajectory[i*12],state_trajectory[i*12+1],state_trajectory[i*12+2],state_trajectory[i*12+3],
    //                         state_trajectory[i*12+4],state_trajectory[i*12+5],state_trajectory[i*12+6],state_trajectory[i*12+7],
    //                         state_trajectory[i*12+8],state_trajectory[i*12+9],state_trajectory[i*12+10],state_trajectory[i*12+11]);
    // }

    memcpy(robot_state.gait, gait, sizeof(float)*4*problem_config.horizon);                     //步态信息填进robot state中
    memcpy(robot_state.traj, state_trajectory, sizeof(float)*12*problem_config.horizon);        //期望轨迹填进robot state中

    robot_state.p = p;                                                                          //位置信息
    robot_state.v = v;                                                                          //速度信息
    robot_state.rpy = rpy;                                                                      //姿态信息
    robot_state.w = w;                                                                          //角速度信息
    robot_state.foot_to_base_position_in_world_frame = foot_to_base_position_in_world_frame;    //足端位置
    robot_state.quat.x() = base_orientation[0];                                                 //四元数x
    robot_state.quat.y() = base_orientation[1];                                                 //四元数y
    robot_state.quat.z() = base_orientation[2];                                                 //四元数z
    robot_state.quat.y() = base_orientation[3];                                                 //四元数w

    robot_state.Rwb = robot_state.quat.toRotationMatrix();                                      //从基座坐标系到世界坐标系的旋转矩阵

    float yaw = rpy[2];                                                                         //获取Yaw
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);

    robot_state.Ryaw << cy, -sy, 0,                                                             //只考虑yaw的旋转矩阵
                        sy, cy,  0,
                        0,  0,   1;

    SolveMpc(&problem_config);                                                                  //构造QP问题并求解
    has_solved = 1;                                                                             //求解完成
}


/**
 * @brief 求解MPC的核心代码
 * 
 * @param setup MPC参数
 */
void SolveMpc(ProblemConfig* setup)
{
    //MPC求解，可以参考https://www.robotsfan.com/posts/311d50f4.html#%E7%AE%80%E5%8C%96%E5%8D%95%E5%88%9A%E4%BD%93%E5%8A%A8%E5%8A%9B%E5%AD%A6

    //初始状态包括机器人的姿态角，位置，姿态角的变化率，速度，以及g，一共13维
    x0 << robot_state.rpy, robot_state.p, robot_state.w, robot_state.v, -9.8f;

    //世界坐标系下的惯性张量，变换公式 I_world = R*I_base*R，其中R是base到world的旋转矩阵，这里作简化处理只考虑yaw方向上的变化
    Mat3 inertia_world = robot_state.Ryaw * robot_state.body_inertia * robot_state.Ryaw.transpose();        

    //构建连续状态空间方程
    ComputeContinuousMatrices(inertia_world, robot_state.mass, robot_state.foot_to_base_position_in_world_frame, robot_state.Ryaw, A_c, B_c); 

    //将MPC转化为QP问题，得到X = Phi*x_k + Gamma*U
    ConvertToDiscreteQp(A_c, B_c, setup->dt, setup->horizon);

    //保存权重参数
    Eigen::Matrix<float, 13, 1> full_weight;
    for (int i = 0; i < 12; i++)
        full_weight(i) = setup->weight[i];
    full_weight(12) = 0.f;//重力项，不作考虑
    Omega.diagonal() = full_weight.replicate(setup->horizon,1);     //生成大的Omega矩阵，参见P102

    Psi = setup->alpha*I_horizon_12;    //生成大的Psi矩阵

    // 保存期望的轨迹
    for(int i = 0; i < setup->horizon; i++)
    {
        for(int j = 0; j < 12; j++)
        {
            X_d(13*i+j,0) = robot_state.traj[12*i+j];
        }
    }

    //输入的上界
    int k = 0;

    for(int i = 0; i < setup->horizon; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            U_b(5*k + 0) = BIG_NUMBER;
            U_b(5*k + 1) = BIG_NUMBER;
            U_b(5*k + 2) = BIG_NUMBER;
            U_b(5*k + 3) = BIG_NUMBER;
            U_b(5*k + 4) = robot_state.gait[i*4 + j] * setup->f_max;    //如果处于摆动相，f最大值就是0
            k++;
        }
    }
 
    H = 2*(Gamma.transpose()*Omega*Gamma + Psi);        //计算H矩阵
    G = 2*Gamma.transpose()*Omega*(Phi*x0 - X_d);       //计算G矩阵

    int num_constraints = 20 * setup->horizon;  //每条腿5个不等式约束
    int num_variables = 12 * setup->horizon;    //12维的力

    //转换成qpOASES向量
    EigenToOASES(H_qpoases, H, num_variables, num_variables);
    EigenToOASES(G_qpoases, G, num_variables, 1);
    EigenToOASES(A_qpoases, fmat, num_constraints, num_variables);
    EigenToOASES(ub_qpoases, U_b, num_constraints, 1);

    for (int i = 0; i < num_constraints; ++i)
    {
        lb_qpoases[i] = 0.0f;
    }

    qpOASES::int_t n_wsr = 100;
    qpOASES::QProblem problem(num_variables, num_constraints);  //变量个数和约束个数
    qpOASES::Options option;      

    option.setToMPC();
    option.printLevel = qpOASES::PL_NONE;
    problem.setOptions(option);   

    //求解QP问题
    int rval = problem.init(H_qpoases, G_qpoases, A_qpoases, NULL, NULL, lb_qpoases, ub_qpoases, n_wsr);  
    int rval2 = problem.getPrimalSolution(q_soln);  

    // LOG_INFO("q soln: %f %f %f %f %f %f %f %f %f %f %f %f.",
    //         q_soln[0],q_soln[1],q_soln[2],
    //         q_soln[3],q_soln[4],q_soln[5],
    //         q_soln[6],q_soln[7],q_soln[8],
    //         q_soln[9],q_soln[10],q_soln[11]);

    if (rval2 != qpOASES::SUCCESSFUL_RETURN) 
    {
        printf("failed to solve!\n");
    }                
}


/**
 * @brief MPC会得到一个输入序列，通过这个函数取出序列中对应index的解
 * 
 * @param index 某个输入的下标
 * @return double 算出来的反力
 */
double GetMpcSolution(int index)
{
    if (!has_solved) 
    {
        return 0.f;
    }
    double *qs = q_soln;
    return qs[index];
}


}//namespace quadruped