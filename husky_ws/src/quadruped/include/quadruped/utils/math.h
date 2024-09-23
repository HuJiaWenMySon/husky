#ifndef QUADRUPED_UTILS_MATH_H_
#define QUADRUPED_UTILS_MATH_H_

#include <eigen3/Eigen/Dense>



namespace quadruped
{


using Mat3 = Eigen::Matrix<float,3,3>;
using Mat4 = Eigen::Matrix<float,4,4>;
using Mat34 = Eigen::Matrix<float,3,4>;

using Vec2 = Eigen::Matrix<float,2,1>;
using Vec3 = Eigen::Matrix<float,3,1>;
using Vec4 = Eigen::Matrix<float,4,1>;
using Vec6 = Eigen::Matrix<float,6,1>;
using Vec12 = Eigen::Matrix<float,12,1>;
using Quat = Eigen::Matrix<float,4,1>;

using VecInt4 = Eigen::Matrix<int,4,1>;



/**
 * @brief 反归一化函数，将一个归一化后的数据转换成归一化之前的数据
 * 
 * @param value 归一化的值
 * @param min 区间下限
 * @param max 区间上限
 * @param min_limit 归一化区间下限，一般是-1
 * @param max_limit 归一化区间上限，一般是1
 * @return float 
 */
inline float InvNormalize(float value, float min, float max, float min_limit = -1, float max_limit = 1)
{
    return (value - min_limit)*(max - min)/(max_limit - min_limit) + min;
}


/**
 * @brief 区间限幅函数
 * 
 * @param a 一个数
 * @param limits 区间
 * @return float 限幅结果
 */
inline float Saturation(float a, Vec2 limits)
{
    float low_lim, high_lim;    //根据limits找到区间的上下界
    if(limits[0] > limits[1])
    {
        low_lim = limits[1];
        high_lim = limits[0];
    }
    else
    {
        low_lim = limits[0];
        high_lim = limits[1];
    }

    if(a < low_lim)             //如果小于下界，返回下界
    {
        return low_lim;
    }
    else if(a > high_lim)       //如果大于上界，返回上界
    {
        return high_lim;
    }
    else                        //在区间内，返回原值
    {
        return a;
    }
}


/**
 * @brief 12×1的列向量转化为3×4的矩阵
 * 
 * @param vec12 12维向量
 * @return Mat34 3×4矩阵
 */
inline Mat34 Vec12ToMat34(Vec12 vec12){
    Mat34 mat34;
    for(int i = 0; i < 4; i++){
        mat34.col(i) = vec12.segment(3*i, 3);   //使用Eigen的segment分段函数
    }
    return mat34;
}


/**
 * @brief 3×4的矩阵转为12维向量
 * 
 * @param mat34 3×4矩阵
 * @return Vec12 12维向量
 */
inline Vec12 Mat34ToVec12(Mat34 mat34)
{
    Vec12 vec12;
    for(int i = 0; i < 4; i++)
    {
        vec12.segment(3*i, 3) = mat34.col(i);   //使用Eigen的segment分段函数
    }
    return vec12;
}


Mat3 RotX(float theta);                                     //计算绕x轴旋转theta的旋转矩阵
Mat3 RotY(float theta);                                     //计算绕y轴旋转theta的旋转矩阵
Mat3 RotZ(float theta);                                     //计算绕z轴旋转theta的旋转矩阵

Mat3 Skew(const Vec3& vec);                                 //将向量转化为斜对称矩阵

Mat3 RpyToRotMat(float roll,float pitch,float yaw);         //ZYX欧拉角转化为旋转矩阵
Vec3 RotMatToRpy(const Mat3& rotation_matrix);              //旋转矩阵转化为ZYX欧拉角

Mat3 QuatToRotMat(const Quat& quat);                        //将四元数转换为对应的旋转矩阵
Quat RotMatToQuat(const Mat3& rotatation_matrix);           //旋转矩阵转化为四元数

Vec3 QuatToRpy(const Quat& quat);                           //将四元数转换到欧拉角
Quat RpyToQuat(float roll,float pitch,float yaw);           //欧拉角转换到四元数


}//namespace quadruped




#endif//QUADRUPED_UTILS_MATH_H_