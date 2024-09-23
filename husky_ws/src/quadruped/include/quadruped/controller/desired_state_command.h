#ifndef QUADRUPED_CONTROLLER_DESIRED_STATE_COMMAND_H_
#define QUADRUPED_CONTROLLER_DESIRED_STATE_COMMAND_H_

#include "utils/math.h"


namespace quadruped
{

/**
 * @brief 命令中设置模式的部分
 * 
 */
enum class RcMode
{
    TO_PASSIVE = 0, //进入阻尼模式
    TO_FIXEDSTAND,  //进入固定站立状态
    TO_FREESTAND,   //进入自由站立状态
    TO_LOCOMOTION,  //进入移动模式
};


class DesiredStateCommand
{
    
public:
    DesiredStateCommand();
    ~DesiredStateCommand();

    void Update();

    bool GetModeChangeRequest() 
    {
        bool ret = mode_change_request_;
        mode_change_request_ = false;
        return ret;   //是否需要进行模式切换
    }
    RcMode GetMode() {return mode_;}                             //返回运行模式
    void SetModeChangeRequest() {mode_change_request_ = true;}   
    void SetMode(RcMode mode) {mode_ = mode;}

private:
    RcMode mode_;                   //控制模式
    bool mode_change_request_;      //是否需要进行模式切换

public:
    Vec2 left_stick_;               //左边的摇杆
    Vec2 right_stick_;              //右边的摇杆
};

}//namespace quadrupped



#endif//QUADRUPED_CONTROLLER_DESIRED_STATE_COMMAND_H_