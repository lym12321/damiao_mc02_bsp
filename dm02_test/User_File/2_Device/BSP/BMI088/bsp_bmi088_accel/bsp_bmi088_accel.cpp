/**
 * @file bsp_bmi088_accel.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-14 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化BMI088加速度计
 *
 * @param Heater_Enable 是否使能加热电阻
 */
void Class_BMI088_Accel::Init(const bool &__Heater_Enable)
{
    // 绑定SPI
    SPI_Manage_Object = &SPI2_Manage_Object;

    // 绑定片选
    CS_GPIO_Port = BMI088_ACCEL__SPI_CS_GPIO_Port;
    CS_Pin = BMI088_ACCEL__SPI_CS_Pin;
    Activate_Pin_State = GPIO_PIN_RESET;

    // 绑定加热电阻定时器
    htim = &htim3;
    TIM_Channel = TIM_CHANNEL_4;

    // 初始化PID
    PID_Temperature.Init(0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 0.1f);

    Heater_Enable = __Heater_Enable;

    uint8_t res;

    // 检测通信是否正常
    Register.ACC_CHIP_ID_RO = 0x00;
    while (Register.ACC_CHIP_ID_RO != 0x1e)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_CHIP_ID_RO));
        Namespace_Timestamp::Delay_Millisecond(100);
    }

    // 软重启
    res = 0xb6;
    Write_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_PWR_CTRL_RW), &res);
    Namespace_Timestamp::Delay_Millisecond(100);

    // 检测通信是否正常
    Register.ACC_CHIP_ID_RO = 0x00;
    while (Register.ACC_CHIP_ID_RO != 0x1e)
    {
        Read_Single_Register(offsetof(Struct_BMI088_Accel_Register, ACC_CHIP_ID_RO));
        Namespace_Timestamp::Delay_Millisecond(100);
    }

    for (uint8_t i = 0; i < BMI088_ACCEL_INIT_INSTRUCTION_NUM; i++)
    {
        ((uint8_t *) (&Register))[BMI088_GYRO_REGISTER_CONFIG[i][0]] = 0x00;
        while (((uint8_t *) (&Register))[BMI088_GYRO_REGISTER_CONFIG[i][0]] != BMI088_GYRO_REGISTER_CONFIG[i][1])
        {
            // 写入寄存器
            Write_Single_Register(BMI088_GYRO_REGISTER_CONFIG[i][0], &BMI088_GYRO_REGISTER_CONFIG[i][1]);
            Namespace_Timestamp::Delay_Millisecond(100);

            // 读取寄存器
            Read_Single_Register(BMI088_GYRO_REGISTER_CONFIG[i][0]);
            Namespace_Timestamp::Delay_Millisecond(100);
        }
    }

    // 预读取一次加速度计数据
    Read_Multi_Register(offsetof(Struct_BMI088_Accel_Register, ACC_X_RO), 6);
    Namespace_Timestamp::Delay_Millisecond(100);
}

/**
 * @brief SPI接收回调函数, 处理加速度计数据
 *
 */
void Class_BMI088_Accel::SPI_RxCpltCallback()
{
    uint8_t spi_init_address = SPI_Manage_Object->Tx_Buffer[0] & ~BMI088_GYRO_READ_MASK;

    memcpy((uint8_t *) (&Register) + spi_init_address, &SPI_Manage_Object->Rx_Buffer[1 + BMI088_GYRO_SPI_RX_RESERVED], SPI_Manage_Object->Rx_Buffer_Length);

    // 处理数据
    if (spi_init_address == offsetof(Struct_BMI088_Accel_Register, ACC_X_RO))
    {
        // 读取加速度计数据完成
        Raw_Accel_X = (float) (Register.ACC_X_RO);
        Raw_Accel_Y = (float) (Register.ACC_Y_RO);
        Raw_Accel_Z = (float) (Register.ACC_Z_RO);
    }
}

/**
 * @brief TIM定时器中断回调函数, 100us周期
 *
 */
void Class_BMI088_Accel::SPI_Request_Accel() const
{
    // 读取加速度计数据
    Read_Multi_Register(offsetof(Struct_BMI088_Accel_Register, ACC_X_RO), 6);
}

/**
 * @brief TIM定时器中断回调函数, 100ms周期
 *
 */
void Class_BMI088_Accel::TIM_100ms_Process_PeriodElapsedCallback()
{

}

/**
 * @brief 读取单个寄存器, 数据不会立即返回, 而是在SPI接收回调函数中处理
 *
 * @param Register_Address 寄存器地址
 */
void Class_BMI088_Accel::Read_Single_Register(const uint8_t &Register_Address) const
{
    SPI_Manage_Object->Tx_Buffer[0] = Register_Address | BMI088_GYRO_READ_MASK;

    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin, Activate_Pin_State, 1 + BMI088_GYRO_SPI_RX_RESERVED, 1);
}

/** * @brief 读取多个寄存器, 数据不会立即返回, 而是在SPI接收回调函数中处理
 *
 * @param Register_Address 寄存器地址
 * @param Rx_Length 接收数据长度
 */
void Class_BMI088_Accel::Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const
{
    SPI_Manage_Object->Tx_Buffer[0] = Register_Address | BMI088_GYRO_READ_MASK;

    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin, Activate_Pin_State, 1 + BMI088_GYRO_SPI_RX_RESERVED, Rx_Length);
}

/**
 * @brief 写入单个寄存器
 *
 * @param Register_Address 寄存器地址
 * @param Tx_Data_Buffer 发送数据缓冲区
 */
void Class_BMI088_Accel::Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const
{
    SPI_Manage_Object->Tx_Buffer[0] = Register_Address;
    SPI_Manage_Object->Tx_Buffer[1] = Tx_Data_Buffer[0];

    SPI_Send_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin, Activate_Pin_State, 2);
}

/**
 * @brief 写入多个寄存器
 *
 * @param Register_Address 寄存器地址
 * @param Tx_Data_Buffer 发送数据缓冲区
 * @param Tx_Length 发送数据长度
 */
void Class_BMI088_Accel::Write_Multi_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer, const uint32_t &Tx_Length) const
{
    SPI_Manage_Object->Tx_Buffer[0] = Register_Address;
    memcpy(&SPI_Manage_Object->Tx_Buffer[1], Tx_Data_Buffer, Tx_Length);

    SPI_Send_Data(SPI_Manage_Object->SPI_Handler, CS_GPIO_Port, CS_Pin, Activate_Pin_State, Tx_Length);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
