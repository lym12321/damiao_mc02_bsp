/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/Serialplot/dvc_serialplot.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 串口绘图
char Serialplot_Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {"flag",};

// LED灯
uint8_t red = 0;
uint8_t green = 170;
uint8_t blue = 170;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

// 陀螺仪加速度计
Class_BMI088 bmi088;

// 全局初始化完成标志位
bool init_finished = false;
// 时间
float time_diff = 0.0f;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief USB虚拟串口接收回调函数
 *
 * @param Buffer 接收缓冲区
 * @param Length 接收数据长度
 */
void Serial_USB_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    Serialplot_USB.USB_RxCallback(Buffer, Length);
    int32_t index = Serialplot_USB.Get_Variable_Index();
    switch (index)
    {
    case (0):
    {
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief SPI2任务回调函数
 *
 */
void BMI088_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    bmi088.SPI_RxCpltCallback();
}

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{

}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    uint64_t now_time = SYS_Timestamp.Get_Now_Microsecond();

    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red == 255)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green == 255)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue == 255)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        BSP_WS2812.Set_RGB(red, green, blue);
        // BSP_WS2812.Set_RGB(0, 0, 0);

        // 发送实例
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
    }

    BSP_Buzzer.Set_Sound(0.0f, 0.0f);

    float battery_power = BSP_Power.Get_Power_Voltage();

    BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
    }

    time_diff = (float) (SYS_Timestamp.Get_Now_Microsecond() - now_time);

    float accel_x = bmi088.BMI088_Accel.Get_Raw_Accel_X();
    float accel_y = bmi088.BMI088_Accel.Get_Raw_Accel_Y();
    float accel_z = bmi088.BMI088_Accel.Get_Raw_Accel_Z();
    float gyro_x = bmi088.BMI088_Gyro.Get_Raw_Gyro_X();
    float gyro_y = bmi088.BMI088_Gyro.Get_Raw_Gyro_Y();
    float gyro_z = bmi088.BMI088_Gyro.Get_Raw_Gyro_Z();
    Serialplot_USB.Set_Data(6, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    Serialplot_USB.TIM_1ms_Write_PeriodElapsedCallback();

    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{

}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    if (bmi088.Get_Init_Finished() & bmi088.Get_Accel_Data_Ready_Flag() & !bmi088.Get_Accel_Transfering_Flag() & !bmi088.Get_Gyro_Transfering_Flag())
    {
        bmi088.Set_Accel_Transfering_Flag(true);

        bmi088.BMI088_Accel.SPI_Request_Accel();

        bmi088.Set_Accel_Data_Ready_Flag(false);
    }

    if (bmi088.Get_Init_Finished() & bmi088.Get_Gyro_Data_Ready_Flag() & !bmi088.Get_Gyro_Transfering_Flag() & !bmi088.Get_Accel_Transfering_Flag())
    {
        bmi088.Set_Gyro_Transfering_Flag(true);

        bmi088.BMI088_Gyro.SPI_Request_Gyro();

        bmi088.Set_Gyro_Data_Ready_Flag(false);
    }
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);
    // 串口绘图的USB
    USB_Init(Serial_USB_Call_Back);
    // 陀螺仪的SPI
    SPI_Init(&hspi2, BMI088_Callback);
    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);
    ADC_Init(&hadc1, 1);

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    Serialplot_USB.Init(Serialplot_Checksum_8_DISABLE, 1, reinterpret_cast<const char **>(Serialplot_Variable_Assignment_List));

    BSP_WS2812.Init(0, 0, 0);

    BSP_Buzzer.Init();

    BSP_Power.Init();

    BSP_Key.Init();

    bmi088.Init();

    Namespace_Timestamp::Delay_Millisecond(500);

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    Namespace_Timestamp::Delay_Millisecond(1);
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }

    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        bmi088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
