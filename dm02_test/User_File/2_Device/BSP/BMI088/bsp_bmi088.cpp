/**
 * @file bsp_bmi088.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-26 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化BMI088
 *
 */
void Class_BMI088::Init()
{
    SPI_Manage_Object = &SPI2_Manage_Object;

    BMI088_Accel.Init();
    BMI088_Gyro.Init();

    Init_Finished = true;
}

/**
 * @brief SPI接收完成回调函数
 *
 */
void Class_BMI088::SPI_RxCpltCallback()
{
    if (SPI_Manage_Object->Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin)
    {
        BMI088_Accel.SPI_RxCpltCallback();
        if (Init_Finished)
        {
            Accel_Transfering_Flag = false;
            Accel_Data_Update_Flag = true;
        }
    }
    else if (SPI_Manage_Object->Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {

        BMI088_Gyro.SPI_RxCallback();

        if (Init_Finished)
        {
            Gyro_Transfering_Flag = false;
            Gyro_Data_Update_Flag = true;
        }
    }
}

/**
 * @brief EXTI中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void Class_BMI088::EXTI_Flag_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin)
    {
        Accel_Data_Ready_Flag = true;
    }
    else if (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        Gyro_Data_Ready_Flag = true;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
