/**
 * @file drv_basic.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些极其简易的数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRV_BASIC_H
#define DRV_BASIC_H

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include <float.h>

/* Exported macros -----------------------------------------------------------*/

extern const float BASIC_MATH_RPM_TO_RADPS;
extern const float BASIC_MATH_DEG_TO_RAD;
extern const float BASIC_MATH_CELSIUS_TO_KELVIN;

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Math_Boolean_Logical_Not(bool *Value);

void Math_Endian_Reverse_16(void *Address);

uint16_t Math_Endian_Reverse_16(void *Source, void *Destination);

void Math_Endian_Reverse_32(void *Address);

uint32_t Math_Endian_Reverse_32(void *Source, void *Destination);

uint8_t Math_Sum_8(const uint8_t *Address, uint32_t Length);

uint16_t Math_Sum_16(const uint16_t *Address, uint32_t Length);

uint32_t Math_Sum_32(const uint32_t *Address, uint32_t Length);

float Math_Sinc(float x);

int32_t Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2);

float Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2);

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 * @return 输出值
 */
template<typename Type>
Type Math_Constrain(Type x, Type Min, Type Max)
{
    if (x < Min)
    {
        x = Min;
    }
    else if (x > Max)
    {
        x = Max;
    }
    return (x);
}

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 * @return 输出值
 */
template<typename Type>
Type Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 求取模归化
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param modulus 模数
 * @return Type 返回的归化数, 介于 ±modulus / 2 之间
 */
template<typename Type>
Type Math_Modulus_Normalization(Type x, Type modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
