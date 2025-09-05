/**
 * @file alg_quaternion.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 四元数计算相关支持库
 * @version 0.1
 * @date 2025-08-22 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#include "alg_quaternion.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**brief 获取零四元数
 *
 */
Class_Quaternion_f32 Namespace_Quaternion::Zero()
{
    Class_Quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位实四元数
 *
 */
Class_Quaternion_f32 Namespace_Quaternion::Unit_Real()
{
    Class_Quaternion_f32 result;
    result[0] = 1.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚四元数X
 *
 */
Class_Quaternion_f32 Namespace_Quaternion::Unit_Imaginary_X()
{
    Class_Quaternion_f32 result;
    result[0] = 1.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚四元数Y
 *
 */
Class_Quaternion_f32 Namespace_Quaternion::Unit_Imaginary_Y()
{
    Class_Quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 1.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚四元数Z
 *
 */
Class_Quaternion_f32 Namespace_Quaternion::Unit_Imaginary_Z()
{
    Class_Quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 1.0f;
    return (result);
}

/**
 * @brief 根据旋转轴与旋转角度获取四元数
 *
 * @param Axis 旋转轴, 单位向量
 * @param Angle 旋转角度, 单位弧度
 * @return Class_Quaternion_f32 四元数
 */
Class_Quaternion_f32 Namespace_Quaternion::Axis_Angle_Unit(const Class_Matrix_f32<3, 1> &Axis, const float &Angle)
{
    Class_Quaternion_f32 result;
    float half_angle = Angle * 0.5f;
    float cos_half_angle = arm_cos_f32(half_angle);
    float sin_half_angle = arm_sin_f32(half_angle);
    result[0] = cos_half_angle;
    result[1] = Axis[0][0] * sin_half_angle;
    result[2] = Axis[1][0] * sin_half_angle;
    result[3] = Axis[2][0] * sin_half_angle;
    return (result);
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
