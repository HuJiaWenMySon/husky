#include <sched.h>
#include <ros/ros.h>
#include "runner.h" 


int main(int argc, char* argv[])
{
    //初始化ROS节点
    ros::init(argc,argv,"quadruped");
    ros::NodeHandle nh;

    //设置进程的优先级
    struct sched_param param;
    param.sched_priority = 99;  //设置最高的进程优先级

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) // 将当前进程设置为SCHED_FIFO实时调度策略
    {
        ROS_WARN("Cannot set FIFO schedule.");
    }

    //定义机器人控制算法相关变量
    quadruped::Runner runner(nh);  //机器人启动器

    ros::Rate rate(1000);           //控制频率
    //执行控制循环
    while(ros::ok())
    {
        runner.Update();//us
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}