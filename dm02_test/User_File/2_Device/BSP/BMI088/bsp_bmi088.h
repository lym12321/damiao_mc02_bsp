/**
 * @file bsp_bmi088.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-26 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel/bsp_bmi088_accel.h"
#include "bsp_bmi088_gyro/bsp_bmi088_gyro.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 板载AHRS
 *
 */
class Class_BMI088
{
public:

    Class_BMI088_Accel BMI088_Accel;
    Class_BMI088_Gyro BMI088_Gyro;

    void Init();

    inline bool Get_Init_Finished() const;

    inline bool Get_Accel_Data_Ready_Flag() const;

    inline bool Get_Gyro_Data_Ready_Flag() const;

    inline bool Get_Accel_Transfering_Flag() const;

    inline bool Get_Gyro_Transfering_Flag() const;

    inline bool Get_Accel_Data_Update_Flag() const;

    inline bool Get_Gyro_Data_Update_Flag() const;

    inline void Set_Accel_Data_Ready_Flag(const bool &__Accel_Data_Ready_Flag);

    inline void Set_Gyro_Data_Ready_Flag(const bool &__Gyro_Data_Ready_Flag);

    inline void Set_Accel_Transfering_Flag(const bool &__Accel_Transfering_Flag);

    inline void Set_Gyro_Transfering_Flag(const bool &__Gyro_Transfering_Flag);

    inline void Set_Accel_Data_Update_Flag(const bool &__Accel_Data_Update_Flag);

    inline void Set_Gyro_Data_Update_Flag(const bool &__Gyro_Data_Update_Flag);

    void SPI_RxCpltCallback();

    void EXTI_Flag_Callback(uint16_t GPIO_Pin);

protected:
    // 初始化相关常量

    // 绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 初始化完成标志
    bool Init_Finished = false;
    // 数据准备完成标志
    bool Accel_Data_Ready_Flag = false;
    bool Gyro_Data_Ready_Flag = false;
    // 数据传输中标志
    bool Accel_Transfering_Flag = false;
    bool Gyro_Transfering_Flag = false;
    // 数据获取完成标志
    bool Accel_Data_Update_Flag = false;
    bool Gyro_Data_Update_Flag = false;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取初始化完成标志位
 *
 * @return 初始化完成标志位
 */
inline bool Class_BMI088::Get_Init_Finished() const
{
    return (Init_Finished);
}

/**
 * @brief 获取加速度计数据准备完成标志位
 *
 * @return 加速度计数据准备完成标志位
 */
inline bool Class_BMI088::Get_Accel_Data_Ready_Flag() const
{
    return (Accel_Data_Ready_Flag);
}

/**
 * @brief 获取陀螺仪数据准备完成标志位
 *
 * @return 陀螺仪数据准备完成标志位
 */
inline bool Class_BMI088::Get_Gyro_Data_Ready_Flag() const
{
    return (Gyro_Data_Ready_Flag);
}

/**
 * @brief 获取加速度计数据传输中标志位
 *
 * @return 加速度计数据传输中标志位
 */
inline bool Class_BMI088::Get_Accel_Transfering_Flag() const
{
    return (Accel_Transfering_Flag);
}

/**
 * @brief 获取陀螺仪数据传输中标志位
 *
 * @return 陀螺仪数据传输中标志位
 */
inline bool Class_BMI088::Get_Gyro_Transfering_Flag() const
{
    return (Gyro_Transfering_Flag);
}

/**
 * @brief 获取加速度计数据获取完成标志位
 *
 * @return 加速度计数据获取完成标志位
 */
inline bool Class_BMI088::Get_Accel_Data_Update_Flag() const
{
    return (Accel_Data_Update_Flag);
}

/**
 * @brief 获取陀螺仪数据获取完成标志位
 *
 * @return 陀螺仪数据获取完成标志位
 */
inline bool Class_BMI088::Get_Gyro_Data_Update_Flag() const
{
    return (Gyro_Data_Update_Flag);
}

/**
 * @brief 设置加速度计数据准备完成标志位
 *
 * @param __Accel_Data_Ready_Flag 加速度计数据准备完成标志位
 */
inline void Class_BMI088::Set_Accel_Data_Ready_Flag(const bool &__Accel_Data_Ready_Flag)
{
    Accel_Data_Ready_Flag = __Accel_Data_Ready_Flag;
}

/**
 * @brief 设置陀螺仪数据准备完成标志位
 *
 * @param __Gyro_Data_Ready_Flag 陀螺仪数据准备完成标志位
 */
inline void Class_BMI088::Set_Gyro_Data_Ready_Flag(const bool &__Gyro_Data_Ready_Flag)
{
    Gyro_Data_Ready_Flag = __Gyro_Data_Ready_Flag;
}

/**
 * @brief 设置加速度计数据传输中标志位
 *
 * @param __Accel_Transfering_Flag 加速度计数据传输中标志位
 */
inline void Class_BMI088::Set_Accel_Transfering_Flag(const bool &__Accel_Transfering_Flag)
{
    Accel_Transfering_Flag = __Accel_Transfering_Flag;
}

/**
 * @brief 设置陀螺仪数据传输中标志位
 *
 * @param __Gyro_Transfering_Flag 陀螺仪数据传输中标志位
 */
inline void Class_BMI088::Set_Gyro_Transfering_Flag(const bool &__Gyro_Transfering_Flag)
{
    Gyro_Transfering_Flag = __Gyro_Transfering_Flag;
}

/**
 * @brief 设置加速度计数据获取完成标志位
 *
 * @param __Accel_Data_Update_Flag 加速度计数据获取完成标志位
 */
inline void Class_BMI088::Set_Accel_Data_Update_Flag(const bool &__Accel_Data_Update_Flag)
{
    Accel_Data_Update_Flag = __Accel_Data_Update_Flag;
}

/**
 * @brief 设置陀螺仪数据获取完成标志位
 *
 * @param __Gyro_Data_Update_Flag 陀螺仪数据获取完成标志位
 */
inline void Class_BMI088::Set_Gyro_Data_Update_Flag(const bool &__Gyro_Data_Update_Flag)
{
    Gyro_Data_Update_Flag = __Gyro_Data_Update_Flag;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
