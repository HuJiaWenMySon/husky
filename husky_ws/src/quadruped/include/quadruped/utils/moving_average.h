#ifndef QUADRUPED_UTILS_MOVING_AVERAGE_H_
#define QUADRUPED_UTILS_MOVING_AVERAGE_H_

#include "utils/math.h"

#include <deque>
#include <numeric>


namespace quadruped
{

#define DEFAULT_WINDOW_SIZE 10


template<int N = 1>
class MovingAverageFilter
{
public:
    MovingAverageFilter(unsigned int window_size = DEFAULT_WINDOW_SIZE);
    void Reset();
    void NeumaierSum(const Eigen::Matrix<float,N,1>& value);
    Eigen::Matrix<float,N,1> CalAverage(const Eigen::Matrix<float,N,1>& new_value);
private:   
    unsigned int window_size_;
    Eigen::Matrix<float,N,1> sum_;
    Eigen::Matrix<float, N, 1> correction_;
    std::deque<Eigen::Matrix<float,N,1>> value_deque_;
};


/**
 * @brief 构造函数
 * 
 * @tparam N 维度
 * @param window_size 窗口长度 
 */
template<int N>
MovingAverageFilter<N>::MovingAverageFilter(unsigned int window_size)
{
    window_size_ = window_size;
    for(int i = 0; i < N; i++)
    {
        sum_[i] = 0.0f;
        correction_[i] = 0.0f;
    }
}


/**
 * @brief 重置滤波器
 * 
 * @tparam N 维度
 */
template<int N>
void MovingAverageFilter<N>::Reset()
{
    for(int i = 0; i < N; i++)
    {
        sum_[i] = 0.0f;
        correction_[i] = 0.0f;
    }
    value_deque_.clear();
}


/**
 * @brief NeumaierSum求和算法降低精度损失
 * 
 * @tparam N 维度
 * @param value 新添加一个数据
 */
template<int N>
void MovingAverageFilter<N>::NeumaierSum(const Eigen::Matrix<float,N,1>& value)
{
    Eigen::Matrix<float,N,1> new_sum = sum_ + value;
    for(int i = 0; i < N; i++)
    {
        if(std::abs(sum_[i]) >= std::abs(value[i]))
        {
            correction_[i] += (sum_[i] - new_sum[i]) + value[i];
        }
        else
        {
            correction_[i] += (value[i] - new_sum[i]) + sum_[i];
        }
    }
    sum_= new_sum;
}


/**
 * @brief 计算平均值
 * 
 * @tparam N 维度
 * @param new_value 新的值 
 * @return Eigen::Matrix<float,N,1> 滤波器输出，均值 
 */
template<int N>
Eigen::Matrix<float,N,1> MovingAverageFilter<N>::CalAverage(const Eigen::Matrix<float,N,1>& new_value)
{
    int deque_len = value_deque_.size();
    if(deque_len >= window_size_)
    {
        NeumaierSum(-value_deque_[0]);
        value_deque_.pop_front();
        deque_len--;
    }

    NeumaierSum(new_value); //将新的值添加到队列，并求和
    value_deque_.push_back(new_value); 
    return (sum_+correction_)/(deque_len+1);//计算新的均值并返回
}


}//namespace quadruped


#endif//QUADRUPED_UTILS_MOVING_AVERAGE_H_