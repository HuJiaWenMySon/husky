#include "utils/math.h"
#include <math.h>


namespace quadruped
{


/**
 * @brief 计算绕x轴旋转theta的旋转矩阵
 * 
 * @param theta 旋转的角度
 * @return Mat3 旋转矩阵
 */
Mat3 RotX(float theta)
{
    Mat3 rot_x;
    rot_x << 1,0,0,
            0,std::cos(theta),-std::sin(theta),
            0,std::sin(theta),std::cos(theta);
    return rot_x;
}


/**
 * @brief 计算绕y轴旋转theta的旋转矩阵
 * 
 * @param theta 旋转的角度
 * @return Mat3 旋转矩阵
 */
Mat3 RotY(float theta)
{
    Mat3 rot_y;
    rot_y << std::cos(theta),0,std::sin(theta),
            0,1,0,
            -std::sin(theta),0,std::cos(theta);
    return rot_y;
}


/**
 * @brief 计算绕z轴旋转theta的旋转矩阵
 * 
 * @param theta 旋转的角度
 * @return Mat3 旋转矩阵
 */
Mat3 RotZ(float theta)
{
    Mat3 rot_z;
    rot_z << std::cos(theta),-std::sin(theta),0,
            std::sin(theta),std::cos(theta),0,
            0,0,1;
    return rot_z;
}


/**
 * @brief 将向量转化为斜对称矩阵
 * 
 * @param vec 向量
 * @return Mat3 叉乘矩阵
 */
Mat3 Skew(const Vec3& vec)
{
    Mat3 skew_matrix;
    skew_matrix << 0,-vec(2),vec(1),
                vec(2),0,-vec(0),
                -vec(1),vec(0),0;
    return skew_matrix;
}


/**
 * @brief ZYX欧拉角转化为旋转矩阵
 * 
 * @param roll 绕x轴旋转的角度
 * @param pitch 绕y轴旋转的角度
 * @param yaw 绕z轴旋转的角度
 * @return Mat3 旋转矩阵
 */
Mat3 RpyToRotMat(float roll,float pitch,float yaw)
{
    Mat3 rotation_matrix = RotZ(yaw)*RotY(pitch)*RotX(roll);    //ZYX欧拉角，相当于XYZ固定角，见《机器人学导论》中关于固定角和欧拉角的讨论
    return rotation_matrix;
}


/**
 * @brief 旋转矩阵转化为ZYX欧拉角
 * 
 * @param rotation_matrix 旋转矩阵
 * @return Vec3 roll、pitch、yaw
 */
Vec3 RotMatToRpy(const Mat3& rotation_matrix)
{
    Vec3 rpy;
    rpy(0) = std::atan2(rotation_matrix(2,1),rotation_matrix(2,2)); //roll = atan2(r_32,r_33)
    rpy(1) = std::asin(-rotation_matrix(2,0));                      //pitch = asin(-r_31)
    rpy(2) = std::atan2(rotation_matrix(1,0),rotation_matrix(0,0)); //yaw = atan2(r_21,r_11)

    return rpy;
}


/**
 * @brief 将四元数转换为对应的旋转矩阵
 * 
 * @param quat 四元数
 * @return Mat3 旋转矩阵
 */
Mat3 QuatToRotMat(const Quat& quat)
{
    Mat3 rotation_matrix;

    float w = quat(0);
    float x = quat(1);
    float y = quat(2);
    float z = quat(3);

    rotation_matrix << 1 - 2 * y * y - 2 * z * z , 2 * x * y-2 * z * w , 2 * x * z + 2 * y * w,
                        2 * x * y + 2 * z * w , 1 - 2 * x * x - 2 * z * z , 2 * y * z - 2 * x * w,
                        2 * x * z - 2 * y * w , 2 * y * z + 2 * x * w , 1 - 2 * x * x - 2 * y * y;
    
    return rotation_matrix;
}


/**
 * @brief 旋转矩阵转换到四元数的代码，chatgpt写的，不知道对不对
 * 
 * @param rotatation_matrix 旋转矩阵
 * @return Quat 四元数
 */
Quat RotMatToQuat(const Mat3& rotatation_matrix)
{
    Quat quaternion;

    double trace = rotatation_matrix(0,0) + rotatation_matrix(1,1) + rotatation_matrix(2,2);  // 计算矩阵的trace

    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        quaternion(0) = 0.25 / s;
        quaternion(1) = (rotatation_matrix(2,1) - rotatation_matrix(1,2)) * s;
        quaternion(2) = (rotatation_matrix(0,2) - rotatation_matrix(2,0)) * s;
        quaternion(3) = (rotatation_matrix(1,0) - rotatation_matrix(0,1)) * s;
    } else {
        if (rotatation_matrix(0,0) > rotatation_matrix(1,1) && rotatation_matrix(0,0) > rotatation_matrix(2,2)) {
            double s = 2.0 * sqrt(1.0 + rotatation_matrix(0,0) - rotatation_matrix(1,1) - rotatation_matrix(2,2));
            quaternion(0) = (rotatation_matrix(2,1) - rotatation_matrix(1,2)) / s;
            quaternion(1) = 0.25 * s;
            quaternion(2) = (rotatation_matrix(0,1) + rotatation_matrix(1,0)) / s;
            quaternion(3) = (rotatation_matrix(0,2) + rotatation_matrix(2,0)) / s;
        } else if (rotatation_matrix(1,1) > rotatation_matrix(2,2)) {
            double s = 2.0 * sqrt(1.0 + rotatation_matrix(1,1) - rotatation_matrix(0,0) - rotatation_matrix(2,2));
            quaternion(0) = (rotatation_matrix(0,2) - rotatation_matrix(2,0)) / s;
            quaternion(1) = (rotatation_matrix(0,1) + rotatation_matrix(1,0)) / s;
            quaternion(2) = 0.25 * s;
            quaternion(3) = (rotatation_matrix(1,2) + rotatation_matrix(2,1)) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + rotatation_matrix(2,2) - rotatation_matrix(0,0) - rotatation_matrix(1,1));
            quaternion(0) = (rotatation_matrix(1,0) - rotatation_matrix(0,1)) / s;
            quaternion(1) = (rotatation_matrix(0,2) + rotatation_matrix(2,0)) / s;
            quaternion(2) = (rotatation_matrix(1,2) + rotatation_matrix(2,1)) / s;
            quaternion(3) = 0.25 * s;
        }
    }

    return quaternion;
}


/**
 * @brief 将四元数转换到欧拉角
 * 
 * @param quat 四元数
 * @return Vec3 ZYX欧拉角 
 */
Vec3 QuatToRpy(const Quat& quat)
{
    Mat3 rotation_matrix = QuatToRotMat(quat);//先转换到旋转矩阵
    Vec3 rpy = RotMatToRpy(rotation_matrix);  //再转换到RPY
    return rpy;
}


/**
 * @brief 欧拉角转换为四元数
 * 
 * @param roll 翻滚
 * @param pitch 俯仰
 * @param yaw 偏移
 * @return Quat 四元数
 */
Quat RpyToQuat(float roll,float pitch,float yaw)
{
    Quat quaternion;
    Mat3 rotation_matrix = RpyToRotMat(roll,pitch,yaw);
    quaternion = RotMatToQuat(rotation_matrix);
    return quaternion;
}


}