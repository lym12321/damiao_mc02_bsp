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

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"
#include "2_Device/Vofa/dvc_vofa.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "1_Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.h"
#include "1_Middleware/Algorithm/Matrix/alg_matrix.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 串口绘图
char Vofa_Variable_Assignment_List[][VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "q00", "q11", "r00", "r11",
};

// LED灯
uint8_t red = 0;
uint8_t green = 170;
uint8_t blue = 170;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

// 大疆电机3508
Class_Motor_DJI_C620 motor;
// Kalman滤波器
Class_Filter_Kalman filter_kalman;
// 相关矩阵
Class_Matrix_f32<2, 2> A;
Class_Matrix_f32<2, 1> B;
Class_Matrix_f32<2, 2> H;
Class_Matrix_f32<2, 2> Q;
Class_Matrix_f32<2, 2> R;

// 全局初始化完成标志位
bool init_finished = false;

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
    Vofa_USB.USB_RxCallback(Buffer, Length);
    int32_t index = Vofa_USB.Get_Variable_Index();
    switch (index)
    {
    case (0):
    {
        filter_kalman.Matrix_Q[0][0] = Vofa_USB.Get_Variable_Value();
        break;
    }
    case (1):
    {
        filter_kalman.Matrix_Q[1][1] = Vofa_USB.Get_Variable_Value();
        break;
    }
    case (2):
    {
        filter_kalman.Matrix_R[0][0] = Vofa_USB.Get_Variable_Value();
        break;
    }
    case (3):
    {
        filter_kalman.Matrix_R[1][1] = Vofa_USB.Get_Variable_Value();
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
void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    if (SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin || SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

/**
 * @brief CAN1回调函数
 *
 *
 */
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (0x201):
    {
        motor.CAN_RxCpltCallback();

        break;
    }
    }
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

    BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
    }

    static int mod100 = 0;
    mod100++;
    if (mod100 == 100)
    {
        mod100 = 0;

        motor.TIM_100ms_Alive_PeriodElapsedCallback();
    }
    motor.Set_Target_Current(0.5f);
    motor.TIM_Calculate_PeriodElapsedCallback();

    static int mod128 = 0;
    mod128++;
    if (mod128 == 128)
    {
        mod128 = 0;

        BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    }

    filter_kalman.Vector_Z[0][0] = motor.Get_Now_Angle();
    filter_kalman.Vector_Z[1][0] = motor.Get_Now_Omega();
    filter_kalman.TIM_Predict_PeriodElapsedCallback();
    filter_kalman.TIM_Update_PeriodElapsedCallback();

    float accel_x = BSP_BMI088.BMI088_Accel.Get_Raw_Accel_X();
    float accel_y = BSP_BMI088.BMI088_Accel.Get_Raw_Accel_Y();
    float accel_z = BSP_BMI088.BMI088_Accel.Get_Raw_Accel_Z();
    float gyro_x = BSP_BMI088.BMI088_Gyro.Get_Raw_Gyro_X();
    float gyro_y = BSP_BMI088.BMI088_Gyro.Get_Raw_Gyro_Y();
    float gyro_z = BSP_BMI088.BMI088_Gyro.Get_Raw_Gyro_Z();
    float q0 = BSP_BMI088.EKF_Quaternion.Vector_X[0][0];
    float q1 = BSP_BMI088.EKF_Quaternion.Vector_X[1][0];
    float q2 = BSP_BMI088.EKF_Quaternion.Vector_X[2][0];
    float q3 = BSP_BMI088.EKF_Quaternion.Vector_X[3][0];
    float accel = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    float gyro = sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
    float yaw = BSP_BMI088.Get_Angle_Yaw();
    float pitch = BSP_BMI088.Get_Angle_Pitch();
    float roll = BSP_BMI088.Get_Angle_Roll();
    float loss = BSP_BMI088.Get_Accel_Chi_Square_Loss();
    float calculating_time = BSP_BMI088.Get_Calculating_Time();
    float temperature = BSP_BMI088.BMI088_Accel.Get_Now_Temperature();

    // 串口绘图
    Vofa_USB.Set_Data(6, &yaw, &pitch, &roll, &loss, &calculating_time, &temperature);
    // Serialplot_USB.Set_Data(6, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    Vofa_USB.TIM_1ms_Write_PeriodElapsedCallback();

    TIM_1ms_CAN_PeriodElapsedCallback();
    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
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
    SPI_Init(&hspi2, SPI2_Callback);
    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);
    // 电机的CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    // 电源的ADC
    ADC_Init(&hadc1, 1);

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    Vofa_USB.Init(4, reinterpret_cast<const char **>(Vofa_Variable_Assignment_List));

    BSP_WS2812.Init(0, 0, 0);

    BSP_Buzzer.Init();

    BSP_Power.Init();

    BSP_Key.Init();

    BSP_BMI088.Init();

    motor.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT);
    A[0][0] = 1.0f;
    A[0][1] = 0.001f;
    A[1][0] = 0.0f;
    A[1][1] = 1.0f;
    B[0][0] = 0.0f;
    B[1][0] = 0.0f;
    H[0][0] = 1.0f;
    H[0][1] = 0.0f;
    H[1][0] = 0.0f;
    H[1][1] = 1.0f;

    // 调参侠
    Q[0][0] = 0.001f;
    Q[0][1] = 0.0f;
    Q[1][0] = 0.0f;
    Q[1][1] = 0.1f;
    R[0][0] = 0.001f;
    R[0][1] = 0.0f;
    R[1][0] = 0.0f;
    R[1][1] = 1.0f;
    filter_kalman.Init(A, B, H, Q, R);

    Namespace_SYS_Timestamp::Delay_Millisecond(2000);

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
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
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
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
