/**
 * @file bsp_bmi088_gyro.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之陀螺仪
 * @version 0.1
 * @date 2025-08-19 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_BMI088_GYRO_H
#define BSP_BMI088_GYRO_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_gyro_register.h"
#include "1_Middleware/Driver/SPI/drv_spi.h"
#include "1_Middleware/Algorithm/Basic/alg_basic.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 陀螺仪量程枚举类型
 *
 */
enum Enum_BSP_BMI088_Gyro_Range : uint8_t
{
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS,
    BMI088_GYRO_RANGE_500DPS,
    BMI088_GYRO_RANGE_250DPS,
    BMI088_GYRO_RANGE_125DPS,
};

/**
 * @brief Specialized, 陀螺仪
 *
 */
class Class_BMI088_Gyro
{
public:

    void Init(bool __Heater_Enable = false);

    inline float Get_Raw_Gyro_X() const;

    inline float Get_Raw_Gyro_Y() const;

    inline float Get_Raw_Gyro_Z() const;

    void SPI_RxCallback();

    void SPI_Request_Gyro();

protected:
    // 初始化相关常量

    // 绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;
    // 片选绑定的GPIO
    GPIO_TypeDef *CS_GPIO_Port;
    uint16_t CS_Pin;
    GPIO_PinState Activate_Pin_State;

    // 常量

    // 读取寄存器时需要设置的掩码
    const uint8_t BMI088_GYRO_READ_MASK = 0x80;
    // 初始化指令数
    const uint8_t BMI088_GYRO_INIT_INSTRUCTION_NUM = 5;
    // 陀螺仪量程, 默认2000°/s
    const Enum_BSP_BMI088_Gyro_Range BMI088_GYRO_RANGE = BMI088_GYRO_RANGE_2000DPS;
    // 寄存器配置相关
    const uint8_t BMI088_GYRO_REGISTER_CONFIG[5][2] = {
        // 设置陀螺仪量程
        {
            offsetof(Struct_BMI088_Gyro_Register, GYRO_RANGE_RW), BMI088_GYRO_RANGE
        },
        // 设置陀螺仪反馈频率为2000Hz, 带宽为532Hz, 0x80是只读位, 永久为该值, 可忽略
        {
            offsetof(Struct_BMI088_Gyro_Register, GYRO_BANDWODTH_RW), 0x01 | 0x80
        },
        // 配置为中断输出模式
        {
            offsetof(Struct_BMI088_Gyro_Register, GYRO_INT_CTRL_RW), 0x01 << 7
        },
        // 中断1号引脚配置推挽输出模式, 实际上写0才是配置部分, 默认是全1
        {
            offsetof(Struct_BMI088_Gyro_Register, INT3_INT4_IO_CONF_RW), 0x0c
        },
        // 配置为如果数据准备好就中断
        {
            offsetof(Struct_BMI088_Gyro_Register, INT3_INT4_IO_MAP_RW), 0x01
        },
    };

    // 内部变量

    // 寄存器结构体
    Struct_BMI088_Gyro_Register Register;

    // 读变量

    // 当前角速度
    float Raw_Gyro_X = 0.0f;
    float Raw_Gyro_Y = 0.0f;
    float Raw_Gyro_Z = 0.0f;

    // 写变量

    // 读写变量

    // 内部函数

    void Read_Single_Register(const uint8_t &Register_Address) const;

    void Read_Multi_Register(const uint8_t &Register_Address, const uint32_t &Rx_Length) const;

    void Write_Single_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer) const;

    void Write_Multi_Register(const uint8_t &Register_Address, const uint8_t *Tx_Data_Buffer, const uint32_t &Tx_Length) const;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前加速度计X轴原始数据
 *
 * @return 当前加速度计X轴原始数据
 */
inline float Class_BMI088_Gyro::Get_Raw_Gyro_X() const
{
    return (Raw_Gyro_X);
}

/**
 * @brief 获取当前加速度计Y轴原始数据
 *
 * @return 当前加速度计Y轴原始数据
 */
inline float Class_BMI088_Gyro::Get_Raw_Gyro_Y() const
{
    return (Raw_Gyro_Y);
}

/**
 * @brief 获取当前加速度计Z轴原始数据
 *
 * @return 当前加速度计Z轴原始数据
 */
inline float Class_BMI088_Gyro::Get_Raw_Gyro_Z() const
{
    return (Raw_Gyro_Z);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
