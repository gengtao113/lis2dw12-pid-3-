/**
  ******************************************************************************
  * @file    lis2dw12_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis2dw12_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIS2DW12_REGS_H
#define LIS2DW12_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LIS2DW12
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay;
  /** Customizable optional pointer **/
  void *handle;

  /** private data **/
  void *priv_data;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LIS2DW12_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS2DW12_I2C_ADD_L   0x31U
#define LIS2DW12_I2C_ADD_H   0x33U

/** Device Identification (Who am I) **/
#define LIS2DW12_ID            0x44U

/**
  * @}
  *
  */

#define LIS2DW12_OUT_T_L                     0x0DU
#define LIS2DW12_OUT_T_H                     0x0EU
#define LIS2DW12_WHO_AM_I                    0x0FU
#define LIS2DW12_CTRL1                       0x20U

/**
 * @brief 控制寄存器1结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器控制寄存器1的位域配置。
 * 包含低功耗模式、工作模式和输出数据率的设置。
 * 
 * @note 该寄存器是传感器的主要控制寄存器，影响传感器的基本工作状态。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lp_mode                    : 2;  /**< 低功耗模式选择：控制传感器的功耗模式 */
  uint8_t mode                       : 2;  /**< 工作模式选择：控制传感器的基本工作模式 */
  uint8_t odr                        : 4;  /**< 输出数据率：设置传感器的数据输出频率 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr                        : 4;  /**< 输出数据率：设置传感器的数据输出频率 */
  uint8_t mode                       : 2;  /**< 工作模式选择：控制传感器的基本工作模式 */
  uint8_t lp_mode                    : 2;  /**< 低功耗模式选择：控制传感器的功耗模式 */
#endif /* DRV_BYTE_ORDER */

} lis2dw12_ctrl1_t;

#define LIS2DW12_CTRL2                       0x21U

/**
 * @brief 控制寄存器2结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器控制寄存器2的位域配置。
 * 包含通信接口、数据更新、引脚配置和系统控制等设置。
 * 
 * @note 该寄存器控制传感器的通信接口和系统级配置。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim                        : 1;  /**< SPI接口模式：选择3线或4线SPI模式 */
  uint8_t i2c_disable                : 1;  /**< I2C接口禁用：控制I2C接口的启用状态 */
  uint8_t if_add_inc                 : 1;  /**< 地址自增：控制多字节读取时的地址自增 */
  uint8_t bdu                        : 1;  /**< 数据更新：控制数据寄存器的更新模式 */
  uint8_t cs_pu_disc                 : 1;  /**< CS引脚上拉：控制CS引脚内部上拉电阻 */
  uint8_t not_used_01                : 1;  /**< 保留位：未使用的位，应设置为0 */
  uint8_t soft_reset                 : 1;  /**< 软复位：触发传感器的软复位操作 */
  uint8_t boot                       : 1;  /**< 启动位：重新加载校准数据 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                       : 1;  /**< 启动位：重新加载校准数据 */
  uint8_t soft_reset                 : 1;  /**< 软复位：触发传感器的软复位操作 */
  uint8_t not_used_01                : 1;  /**< 保留位：未使用的位，应设置为0 */
  uint8_t cs_pu_disc                 : 1;  /**< CS引脚上拉：控制CS引脚内部上拉电阻 */
  uint8_t bdu                        : 1;  /**< 数据更新：控制数据寄存器的更新模式 */
  uint8_t if_add_inc                 : 1;  /**< 地址自增：控制多字节读取时的地址自增 */
  uint8_t i2c_disable                : 1;  /**< I2C接口禁用：控制I2C接口的启用状态 */
  uint8_t sim                        : 1;  /**< SPI接口模式：选择3线或4线SPI模式 */
#endif /* DRV_BYTE_ORDER */

} lis2dw12_ctrl2_t;

#define LIS2DW12_CTRL3                       0x22U

/**
 * @brief 控制寄存器3结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器控制寄存器3的位域配置。
 * 包含睡眠模式、中断配置、引脚特性和自检功能等设置。
 * 
 * @note 该寄存器主要控制传感器的中断行为和引脚配置。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slp_mode                   : 2;  /**< 睡眠模式：控制传感器的睡眠检测模式 */
  uint8_t not_used_01                : 1;  /**< 保留位：未使用的位，应设置为0 */
  uint8_t h_lactive                  : 1;  /**< 中断极性：设置中断引脚的有效电平 */
  uint8_t lir                        : 1;  /**< 中断锁存：控制中断信号的锁存模式 */
  uint8_t pp_od                      : 1;  /**< 引脚输出：选择推挽或开漏输出模式 */
  uint8_t st                         : 2;  /**< 自检模式：配置传感器的自检功能 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t st                         : 2;  /**< 自检模式：配置传感器的自检功能 */
  uint8_t pp_od                      : 1;  /**< 引脚输出：选择推挽或开漏输出模式 */
  uint8_t lir                        : 1;  /**< 中断锁存：控制中断信号的锁存模式 */
  uint8_t h_lactive                  : 1;  /**< 中断极性：设置中断引脚的有效电平 */
  uint8_t not_used_01                : 1;  /**< 保留位：未使用的位，应设置为0 */
  uint8_t slp_mode                   : 2;  /**< 睡眠模式：控制传感器的睡眠检测模式 */
#endif /* DRV_BYTE_ORDER */

} lis2dw12_ctrl3_t;

#define LIS2DW12_CTRL4_INT1_PAD_CTRL         0x23U

/**
 * @brief 中断1引脚控制寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器中断1引脚控制寄存器的位域配置。
 * 控制INT1引脚上各种中断信号的输出使能。
 * 
 * @note 该寄存器决定哪些中断事件会输出到INT1引脚。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy                  : 1;  /**< 数据就绪中断：使能数据就绪信号输出到INT1 */
  uint8_t int1_fth                   : 1;  /**< FIFO阈值中断：使能FIFO阈值中断输出到INT1 */
  uint8_t int1_diff5                 : 1;  /**< 差分中断：使能差分检测中断输出到INT1 */
  uint8_t int1_tap                   : 1;  /**< 双击中断：使能双击检测中断输出到INT1 */
  uint8_t int1_ff                    : 1;  /**< 自由落体中断：使能自由落体检测中断输出到INT1 */
  uint8_t int1_wu                    : 1;  /**< 唤醒中断：使能唤醒检测中断输出到INT1 */
  uint8_t int1_single_tap            : 1;  /**< 单击中断：使能单击检测中断输出到INT1 */
  uint8_t int1_6d                    : 1;  /**< 6D方向中断：使能6方向检测中断输出到INT1 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_6d                    : 1;  /**< 6D方向中断：使能6方向检测中断输出到INT1 */
  uint8_t int1_single_tap            : 1;  /**< 单击中断：使能单击检测中断输出到INT1 */
  uint8_t int1_wu                    : 1;  /**< 唤醒中断：使能唤醒检测中断输出到INT1 */
  uint8_t int1_ff                    : 1;  /**< 自由落体中断：使能自由落体检测中断输出到INT1 */
  uint8_t int1_tap                   : 1;  /**< 双击中断：使能双击检测中断输出到INT1 */
  uint8_t int1_diff5                 : 1;  /**< 差分中断：使能差分检测中断输出到INT1 */
  uint8_t int1_fth                   : 1;  /**< FIFO阈值中断：使能FIFO阈值中断输出到INT1 */
  uint8_t int1_drdy                  : 1;  /**< 数据就绪中断：使能数据就绪信号输出到INT1 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_ctrl4_int1_pad_ctrl_t;

#define LIS2DW12_CTRL5_INT2_PAD_CTRL         0x24U

/**
 * @brief 中断2引脚控制寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器中断2引脚控制寄存器的位域配置。
 * 控制INT2引脚上各种中断信号的输出使能。
 * 
 * @note 该寄存器决定哪些中断事件会输出到INT2引脚，与INT1引脚功能类似但支持不同的中断类型。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy                  : 1;  /**< 数据就绪中断：使能数据就绪信号输出到INT2 */
  uint8_t int2_fth                   : 1;  /**< FIFO阈值中断：使能FIFO阈值中断输出到INT2 */
  uint8_t int2_diff5                 : 1;  /**< 差分中断：使能差分检测中断输出到INT2 */
  uint8_t int2_ovr                   : 1;  /**< 数据溢出中断：使能数据溢出中断输出到INT2 */
  uint8_t int2_drdy_t                : 1;  /**< 温度数据就绪中断：使能温度数据就绪中断输出到INT2 */
  uint8_t int2_boot                  : 1;  /**< 启动中断：使能启动完成中断输出到INT2 */
  uint8_t int2_sleep_chg             : 1;  /**< 睡眠状态变化中断：使能睡眠状态变化中断输出到INT2 */
  uint8_t int2_sleep_state           : 1;  /**< 睡眠状态中断：使能睡眠状态指示中断输出到INT2 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_sleep_state           : 1;  /**< 睡眠状态中断：使能睡眠状态指示中断输出到INT2 */
  uint8_t int2_sleep_chg             : 1;  /**< 睡眠状态变化中断：使能睡眠状态变化中断输出到INT2 */
  uint8_t int2_boot                  : 1;  /**< 启动中断：使能启动完成中断输出到INT2 */
  uint8_t int2_drdy_t                : 1;  /**< 温度数据就绪中断：使能温度数据就绪中断输出到INT2 */
  uint8_t int2_ovr                   : 1;  /**< 数据溢出中断：使能数据溢出中断输出到INT2 */
  uint8_t int2_diff5                 : 1;  /**< 差分中断：使能差分检测中断输出到INT2 */
  uint8_t int2_fth                   : 1;  /**< FIFO阈值中断：使能FIFO阈值中断输出到INT2 */
  uint8_t int2_drdy                  : 1;  /**< 数据就绪中断：使能数据就绪信号输出到INT2 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_ctrl5_int2_pad_ctrl_t;

#define LIS2DW12_CTRL6                       0x25U

/**
 * @brief 控制寄存器6结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器控制寄存器6的位域配置。
 * 包含低噪声模式、滤波器数据选择、满量程范围和滤波器带宽等设置。
 * 
 * @note 该寄存器主要控制传感器的数据质量和测量范围。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01                : 2;  /**< 保留位：未使用的位，应设置为0 */
  uint8_t low_noise                  : 1;  /**< 低噪声模式：使能低噪声模式以提高数据质量 */
  uint8_t fds                        : 1;  /**< 滤波器数据选择：选择输出数据的滤波器路径 */
  uint8_t fs                         : 2;  /**< 满量程范围：设置加速度测量的量程范围 */
  uint8_t bw_filt                    : 2;  /**< 滤波器带宽：设置低通滤波器的带宽 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bw_filt                    : 2;  /**< 滤波器带宽：设置低通滤波器的带宽 */
  uint8_t fs                         : 2;  /**< 满量程范围：设置加速度测量的量程范围 */
  uint8_t fds                        : 1;  /**< 滤波器数据选择：选择输出数据的滤波器路径 */
  uint8_t low_noise                  : 1;  /**< 低噪声模式：使能低噪声模式以提高数据质量 */
  uint8_t not_used_01                : 2;  /**< 保留位：未使用的位，应设置为0 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_ctrl6_t;

#define LIS2DW12_OUT_T                       0x26U
#define LIS2DW12_STATUS                      0x27U

/**
 * @brief 状态寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器状态寄存器的位域配置。
 * 提供各种中断事件和数据状态的实时状态信息。
 * 
 * @note 该寄存器为只读寄存器，用于监控传感器的各种状态和事件。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                       : 1;  /**< 数据就绪：指示新的加速度数据可用 */
  uint8_t ff_ia                      : 1;  /**< 自由落体中断：指示检测到自由落体事件 */
  uint8_t _6d_ia                     : 1;  /**< 6D方向中断：指示检测到6方向变化事件 */
  uint8_t single_tap                 : 1;  /**< 单击中断：指示检测到单击事件 */
  uint8_t double_tap                 : 1;  /**< 双击中断：指示检测到双击事件 */
  uint8_t sleep_state                : 1;  /**< 睡眠状态：指示传感器当前处于睡眠状态 */
  uint8_t wu_ia                      : 1;  /**< 唤醒中断：指示检测到唤醒事件 */
  uint8_t fifo_ths                   : 1;  /**< FIFO阈值：指示FIFO达到设定阈值 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_ths                   : 1;  /**< FIFO阈值：指示FIFO达到设定阈值 */
  uint8_t wu_ia                      : 1;  /**< 唤醒中断：指示检测到唤醒事件 */
  uint8_t sleep_state                : 1;  /**< 睡眠状态：指示传感器当前处于睡眠状态 */
  uint8_t double_tap                 : 1;  /**< 双击中断：指示检测到双击事件 */
  uint8_t single_tap                 : 1;  /**< 单击中断：指示检测到单击事件 */
  uint8_t _6d_ia                     : 1;  /**< 6D方向中断：指示检测到6方向变化事件 */
  uint8_t ff_ia                      : 1;  /**< 自由落体中断：指示检测到自由落体事件 */
  uint8_t drdy                       : 1;  /**< 数据就绪：指示新的加速度数据可用 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_status_t;

#define LIS2DW12_OUT_X_L                     0x28U
#define LIS2DW12_OUT_X_H                     0x29U
#define LIS2DW12_OUT_Y_L                     0x2AU
#define LIS2DW12_OUT_Y_H                     0x2BU
#define LIS2DW12_OUT_Z_L                     0x2CU
#define LIS2DW12_OUT_Z_H                     0x2DU
#define LIS2DW12_FIFO_CTRL                   0x2EU

/**
 * @brief FIFO控制寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器FIFO控制寄存器的位域配置。
 * 控制FIFO缓冲区的阈值和工作模式。
 * 
 * @note 该寄存器用于配置FIFO缓冲区的行为和数据管理。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth                        : 5;  /**< FIFO阈值：设置FIFO触发中断的阈值级别 */
  uint8_t fmode                      : 3;  /**< FIFO模式：选择FIFO的工作模式 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fmode                      : 3;  /**< FIFO模式：选择FIFO的工作模式 */
  uint8_t fth                        : 5;  /**< FIFO阈值：设置FIFO触发中断的阈值级别 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_fifo_ctrl_t;

#define LIS2DW12_FIFO_SAMPLES                0x2FU

/**
 * @brief FIFO样本寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器FIFO样本寄存器的位域配置。
 * 提供FIFO缓冲区中存储的样本数量信息和状态标志。
 * 
 * @note 该寄存器为只读寄存器，用于监控FIFO缓冲区的状态。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff                       : 6;  /**< 差分样本数：FIFO中存储的样本数量 */
  uint8_t fifo_ovr                   : 1;  /**< FIFO溢出：指示FIFO缓冲区已满并发生溢出 */
  uint8_t fifo_fth                   : 1;  /**< FIFO阈值：指示FIFO达到设定阈值 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_fth                   : 1;  /**< FIFO阈值：指示FIFO达到设定阈值 */
  uint8_t fifo_ovr                   : 1;  /**< FIFO溢出：指示FIFO缓冲区已满并发生溢出 */
  uint8_t diff                       : 6;  /**< 差分样本数：FIFO中存储的样本数量 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_fifo_samples_t;

#define LIS2DW12_TAP_THS_X                   0x30U

/**
 * @brief X轴点击阈值寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器X轴点击阈值寄存器的位域配置。
 * 设置X轴方向的点击检测阈值和6D/4D方向检测参数。
 * 
 * @note 该寄存器用于配置点击检测的灵敏度和方向检测功能。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_thsx                    : 5;  /**< X轴点击阈值：设置X轴方向的点击检测阈值 */
  uint8_t _6d_ths                     : 2;  /**< 6D阈值：设置6方向检测的阈值 */
  uint8_t _4d_en                      : 1;  /**< 4D使能：使能4方向检测功能 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t _4d_en                      : 1;  /**< 4D使能：使能4方向检测功能 */
  uint8_t _6d_ths                     : 2;  /**< 6D阈值：设置6方向检测的阈值 */
  uint8_t tap_thsx                    : 5;  /**< X轴点击阈值：设置X轴方向的点击检测阈值 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_tap_ths_x_t;

#define LIS2DW12_TAP_THS_Y                   0x31U

/**
 * @brief Y轴点击阈值寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器Y轴点击阈值寄存器的位域配置。
 * 设置Y轴方向的点击检测阈值和点击优先级。
 * 
 * @note 该寄存器用于配置Y轴点击检测的灵敏度和优先级设置。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_thsy                   : 5;  /**< Y轴点击阈值：设置Y轴方向的点击检测阈值 */
  uint8_t tap_prior                  : 3;  /**< 点击优先级：设置点击检测的优先级 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tap_prior                  : 3;  /**< 点击优先级：设置点击检测的优先级 */
  uint8_t tap_thsy                   : 5;  /**< Y轴点击阈值：设置Y轴方向的点击检测阈值 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_tap_ths_y_t;

#define LIS2DW12_TAP_THS_Z                   0x32U

/**
 * @brief Z轴点击阈值寄存器结构体类型
 * 
 * 该结构体定义了LIS2DW12加速度传感器Z轴点击阈值寄存器的位域配置。
 * 设置Z轴方向的点击检测阈值和各轴点击检测的使能状态。
 * 
 * @note 该寄存器用于配置Z轴点击检测的灵敏度和各轴点击检测的开关。
 */
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_thsz                   : 5;  /**< Z轴点击阈值：设置Z轴方向的点击检测阈值 */
  uint8_t tap_z_en                   : 1;  /**< Z轴点击使能：使能Z轴方向的点击检测 */
  uint8_t tap_y_en                   : 1;  /**< Y轴点击使能：使能Y轴方向的点击检测 */
  uint8_t tap_x_en                   : 1;  /**< X轴点击使能：使能X轴方向的点击检测 */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tap_x_en                   : 1;  /**< X轴点击使能：使能X轴方向的点击检测 */
  uint8_t tap_y_en                   : 1;  /**< Y轴点击使能：使能Y轴方向的点击检测 */
  uint8_t tap_z_en                   : 1;  /**< Z轴点击使能：使能Z轴方向的点击检测 */
  uint8_t tap_thsz                   : 5;  /**< Z轴点击阈值：设置Z轴方向的点击检测阈值 */
#endif /* DRV_BYTE_ORDER */
} lis2dw12_tap_ths_z_t;

#define LIS2DW12_INT_DUR                     0x33U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock                      : 2;
  uint8_t quiet                      : 2;
  uint8_t latency                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t latency                    : 4;
  uint8_t quiet                      : 2;
  uint8_t shock                      : 2;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_int_dur_t;

#define LIS2DW12_WAKE_UP_THS                 0x34U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths                     : 6;
  uint8_t sleep_on                   : 1;
  uint8_t single_double_tap          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap          : 1;
  uint8_t sleep_on                   : 1;
  uint8_t wk_ths                     : 6;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_wake_up_ths_t;

#define LIS2DW12_WAKE_UP_DUR                 0x35U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                  : 4;
  uint8_t stationary                 : 1;
  uint8_t wake_dur                   : 2;
  uint8_t ff_dur                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                     : 1;
  uint8_t wake_dur                   : 2;
  uint8_t stationary                 : 1;
  uint8_t sleep_dur                  : 4;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_wake_up_dur_t;

#define LIS2DW12_FREE_FALL                   0x36U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths                     : 3;
  uint8_t ff_dur                     : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                     : 5;
  uint8_t ff_ths                     : 3;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_free_fall_t;

#define LIS2DW12_STATUS_DUP                  0x37U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                       : 1;
  uint8_t ff_ia                      : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t single_tap                 : 1;
  uint8_t double_tap                 : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t drdy_t                     : 1;
  uint8_t ovr                        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ovr                        : 1;
  uint8_t drdy_t                     : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t double_tap                 : 1;
  uint8_t single_tap                 : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t ff_ia                      : 1;
  uint8_t drdy                       : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_status_dup_t;

#define LIS2DW12_WAKE_UP_SRC                 0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                       : 1;
  uint8_t y_wu                       : 1;
  uint8_t x_wu                       : 1;
  uint8_t wu_ia                      : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t ff_ia                      : 1;
  uint8_t not_used_01                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 2;
  uint8_t ff_ia                      : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t wu_ia                      : 1;
  uint8_t x_wu                       : 1;
  uint8_t y_wu                       : 1;
  uint8_t z_wu                       : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_wake_up_src_t;

#define LIS2DW12_TAP_SRC                     0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap                      : 1;
  uint8_t y_tap                      : 1;
  uint8_t x_tap                      : 1;
  uint8_t tap_sign                   : 1;
  uint8_t double_tap                 : 1;
  uint8_t single_tap                 : 1;
  uint8_t tap_ia                     : 1;
  uint8_t not_used_01                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 1;
  uint8_t tap_ia                     : 1;
  uint8_t single_tap                 : 1;
  uint8_t double_tap                 : 1;
  uint8_t tap_sign                   : 1;
  uint8_t x_tap                      : 1;
  uint8_t y_tap                      : 1;
  uint8_t z_tap                      : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_tap_src_t;

#define LIS2DW12_SIXD_SRC                    0x3AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                         : 1;
  uint8_t xh                         : 1;
  uint8_t yl                         : 1;
  uint8_t yh                         : 1;
  uint8_t zl                         : 1;
  uint8_t zh                         : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t not_used_01                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t zh                         : 1;
  uint8_t zl                         : 1;
  uint8_t yh                         : 1;
  uint8_t xh                         : 1;
  uint8_t yl                         : 1;
  uint8_t xh                         : 1;
  uint8_t xl                         : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_sixd_src_t;

#define LIS2DW12_ALL_INT_SRC                 0x3BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ia                      : 1;
  uint8_t wu_ia                      : 1;
  uint8_t single_tap                 : 1;
  uint8_t double_tap                 : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t sleep_change_ia            : 1;
  uint8_t not_used_01                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 2;
  uint8_t sleep_change_ia            : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t double_tap                 : 1;
  uint8_t single_tap                 : 1;
  uint8_t wu_ia                      : 1;
  uint8_t ff_ia                      : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_all_int_src_t;

#define LIS2DW12_X_OFS_USR                   0x3CU
#define LIS2DW12_Y_OFS_USR                   0x3DU
#define LIS2DW12_Z_OFS_USR                   0x3EU
#define LIS2DW12_CTRL_REG7                   0x3FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpass_on6d                 : 1;
  uint8_t hp_ref_mode                : 1;
  uint8_t usr_off_w                  : 1;
  uint8_t usr_off_on_wu              : 1;
  uint8_t usr_off_on_out             : 1;
  uint8_t interrupts_enable          : 1;
  uint8_t int2_on_int1               : 1;
  uint8_t drdy_pulsed                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_pulsed                : 1;
  uint8_t int2_on_int1               : 1;
  uint8_t interrupts_enable          : 1;
  uint8_t usr_off_on_out             : 1;
  uint8_t usr_off_on_wu              : 1;
  uint8_t usr_off_w                  : 1;
  uint8_t hp_ref_mode                : 1;
  uint8_t lpass_on6d                 : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dw12_ctrl_reg7_t;

/**
  * @defgroup LIS2DW12_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lis2dw12_ctrl1_t                   ctrl1;
  lis2dw12_ctrl2_t                   ctrl2;
  lis2dw12_ctrl3_t                   ctrl3;
  lis2dw12_ctrl4_int1_pad_ctrl_t     ctrl4_int1_pad_ctrl;
  lis2dw12_ctrl5_int2_pad_ctrl_t     ctrl5_int2_pad_ctrl;
  lis2dw12_ctrl6_t                   ctrl6;
  lis2dw12_status_t                  status;
  lis2dw12_fifo_ctrl_t               fifo_ctrl;
  lis2dw12_fifo_samples_t            fifo_samples;
  lis2dw12_tap_ths_x_t               tap_ths_x;
  lis2dw12_tap_ths_y_t               tap_ths_y;
  lis2dw12_tap_ths_z_t               tap_ths_z;
  lis2dw12_int_dur_t                 int_dur;
  lis2dw12_wake_up_ths_t             wake_up_ths;
  lis2dw12_wake_up_dur_t             wake_up_dur;
  lis2dw12_free_fall_t               free_fall;
  lis2dw12_status_dup_t              status_dup;
  lis2dw12_wake_up_src_t             wake_up_src;
  lis2dw12_tap_src_t                 tap_src;
  lis2dw12_sixd_src_t                sixd_src;
  lis2dw12_all_int_src_t             all_int_src;
  lis2dw12_ctrl_reg7_t               ctrl_reg7;
  bitwise_t                          bitwise;
  uint8_t                            byte;
} lis2dw12_reg_t;

/**
  * @}
  *
  */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 * The __weak directive allows the final application to overwrite
 * them with a custom implementation.
 */

int32_t lis2dw12_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);
int32_t lis2dw12_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

float_t lis2dw12_from_fs2_to_mg(int16_t lsb);
float_t lis2dw12_from_fs4_to_mg(int16_t lsb);
float_t lis2dw12_from_fs8_to_mg(int16_t lsb);
float_t lis2dw12_from_fs16_to_mg(int16_t lsb);

float_t lis2dw12_from_fs2_lp1_to_mg(int16_t lsb);
float_t lis2dw12_from_fs4_lp1_to_mg(int16_t lsb);
float_t lis2dw12_from_fs8_lp1_to_mg(int16_t lsb);
float_t lis2dw12_from_fs16_lp1_to_mg(int16_t lsb);

float_t lis2dw12_from_lsb_to_celsius(int16_t lsb);

/**
 * @brief 电源模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器的各种电源工作模式。
 * 包括高性能模式、连续低功耗模式、单次低功耗模式等，每种模式在功耗和性能之间有不同的平衡。
 * 
 * @note 选择合适的电源模式对于优化系统功耗和性能至关重要。
 */
typedef enum
{
  LIS2DW12_HIGH_PERFORMANCE                    = 0x04,  /**< 高性能模式：最高精度和响应速度，功耗较高 */
  LIS2DW12_CONT_LOW_PWR_4                      = 0x03,  /**< 连续低功耗模式4：14位分辨率，中等功耗 */
  LIS2DW12_CONT_LOW_PWR_3                      = 0x02,  /**< 连续低功耗模式3：14位分辨率，较低功耗 */
  LIS2DW12_CONT_LOW_PWR_2                      = 0x01,  /**< 连续低功耗模式2：14位分辨率，低功耗 */
  LIS2DW12_CONT_LOW_PWR_12bit                  = 0x00,  /**< 连续低功耗模式：12位分辨率，最低功耗 */
  LIS2DW12_SINGLE_LOW_PWR_4                    = 0x0B,  /**< 单次低功耗模式4：14位分辨率，单次采样后休眠 */
  LIS2DW12_SINGLE_LOW_PWR_3                    = 0x0A,  /**< 单次低功耗模式3：14位分辨率，单次采样后休眠 */
  LIS2DW12_SINGLE_LOW_PWR_2                    = 0x09,  /**< 单次低功耗模式2：14位分辨率，单次采样后休眠 */
  LIS2DW12_SINGLE_LOW_PWR_12bit                = 0x08,  /**< 单次低功耗模式：12位分辨率，单次采样后休眠 */
  LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE          = 0x14,  /**< 高性能低噪声模式：最高精度，低噪声，功耗最高 */
  LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4            = 0x13,  /**< 连续低功耗低噪声模式4：14位分辨率，低噪声 */
  LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3            = 0x12,  /**< 连续低功耗低噪声模式3：14位分辨率，低噪声 */
  LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2            = 0x11,  /**< 连续低功耗低噪声模式2：14位分辨率，低噪声 */
  LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit        = 0x10,  /**< 连续低功耗低噪声模式：12位分辨率，低噪声 */
  LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4          = 0x1B,  /**< 单次低功耗低噪声模式4：14位分辨率，单次采样后休眠 */
  LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3          = 0x1A,  /**< 单次低功耗低噪声模式3：14位分辨率，单次采样后休眠 */
  LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2          = 0x19,  /**< 单次低功耗低噪声模式2：14位分辨率，单次采样后休眠 */
  LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit      = 0x18,  /**< 单次低功耗低噪声模式：12位分辨率，单次采样后休眠 */
} lis2dw12_mode_t;
int32_t lis2dw12_power_mode_set(const stmdev_ctx_t *ctx,
                                lis2dw12_mode_t val);
int32_t lis2dw12_power_mode_get(const stmdev_ctx_t *ctx,
                                lis2dw12_mode_t *val);

/**
 * @brief 输出数据率枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器的输出数据率（ODR）设置。
 * 包括从关闭状态到1.6kHz的各种采样频率，以及软件触发和引脚触发模式。
 * 
 * @note 数据率越高，功耗越大，但能提供更实时的数据更新。
 */
typedef enum
{
  LIS2DW12_XL_ODR_OFF            = 0x00,  /**< 关闭状态：传感器停止工作，最低功耗 */
  LIS2DW12_XL_ODR_1Hz6_LP_ONLY   = 0x01,  /**< 1.6Hz低功耗模式：仅适用于低功耗模式 */
  LIS2DW12_XL_ODR_12Hz5          = 0x02,  /**< 12.5Hz：适用于低频率应用场景 */
  LIS2DW12_XL_ODR_25Hz           = 0x03,  /**< 25Hz：适用于一般运动检测 */
  LIS2DW12_XL_ODR_50Hz           = 0x04,  /**< 50Hz：适用于中等频率应用 */
  LIS2DW12_XL_ODR_100Hz          = 0x05,  /**< 100Hz：适用于较高频率应用 */
  LIS2DW12_XL_ODR_200Hz          = 0x06,  /**< 200Hz：适用于高频率应用 */
  LIS2DW12_XL_ODR_400Hz          = 0x07,  /**< 400Hz：适用于高速应用 */
  LIS2DW12_XL_ODR_800Hz          = 0x08,  /**< 800Hz：适用于超高速应用 */
  LIS2DW12_XL_ODR_1k6Hz          = 0x09,  /**< 1.6kHz：最高数据率，适用于最高精度应用 */
  LIS2DW12_XL_SET_SW_TRIG        = 0x32,  /**< 软件触发模式：仅适用于单次模式 */
  LIS2DW12_XL_SET_PIN_TRIG       = 0x12,  /**< 引脚触发模式：仅适用于单次模式 */
} lis2dw12_odr_t;
int32_t lis2dw12_data_rate_set(const stmdev_ctx_t *ctx, lis2dw12_odr_t val);
int32_t lis2dw12_data_rate_get(const stmdev_ctx_t *ctx,
                               lis2dw12_odr_t *val);

int32_t lis2dw12_block_data_update_set(const stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lis2dw12_block_data_update_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val);

/**
 * @brief 满量程范围枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器的满量程范围设置。
 * 满量程范围决定了传感器能够测量的最大加速度值。
 * 
 * @note 满量程范围越大，测量范围越广，但分辨率相对降低。
 */
typedef enum
{
  LIS2DW12_2g     = 0,  /**< ±2g满量程：适用于低加速度应用，最高分辨率 */
  LIS2DW12_4g     = 1,  /**< ±4g满量程：适用于中等加速度应用 */
  LIS2DW12_8g     = 2,  /**< ±8g满量程：适用于高加速度应用 */
  LIS2DW12_16g    = 3,  /**< ±16g满量程：适用于超高加速度应用，最低分辨率 */
} lis2dw12_fs_t;
int32_t lis2dw12_full_scale_set(const stmdev_ctx_t *ctx, lis2dw12_fs_t val);
int32_t lis2dw12_full_scale_get(const stmdev_ctx_t *ctx,
                                lis2dw12_fs_t *val);

int32_t lis2dw12_status_reg_get(const stmdev_ctx_t *ctx,
                                lis2dw12_status_t *val);

int32_t lis2dw12_flag_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  lis2dw12_status_dup_t   status_dup;
  lis2dw12_wake_up_src_t  wake_up_src;
  lis2dw12_tap_src_t      tap_src;
  lis2dw12_sixd_src_t     sixd_src;
  lis2dw12_all_int_src_t  all_int_src;
} lis2dw12_all_sources_t;
int32_t lis2dw12_all_sources_get(const stmdev_ctx_t *ctx,
                                 lis2dw12_all_sources_t *val);

int32_t lis2dw12_usr_offset_x_set(const stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis2dw12_usr_offset_x_get(const stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2dw12_usr_offset_y_set(const stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis2dw12_usr_offset_y_get(const stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2dw12_usr_offset_z_set(const stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis2dw12_usr_offset_z_get(const stmdev_ctx_t *ctx, uint8_t *buff);

/**
 * @brief 用户偏移权重枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器用户偏移校准的权重设置。
 * 权重决定了偏移校准的精度和范围。
 * 
 * @note 权重越小，校准精度越高，但校准范围越小。
 */
typedef enum
{
  LIS2DW12_LSb_977ug    = 0,  /**< 977μg/LSB权重：高精度校准，适用于精密应用 */
  LIS2DW12_LSb_15mg6    = 1,  /**< 15.6mg/LSB权重：标准精度校准，适用于一般应用 */
} lis2dw12_usr_off_w_t;
int32_t lis2dw12_offset_weight_set(const stmdev_ctx_t *ctx,
                                   lis2dw12_usr_off_w_t val);
int32_t lis2dw12_offset_weight_get(const stmdev_ctx_t *ctx,
                                   lis2dw12_usr_off_w_t *val);

int32_t lis2dw12_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val);

int32_t lis2dw12_acceleration_raw_get(const stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t lis2dw12_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2dw12_auto_increment_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_auto_increment_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_reset_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_reset_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_boot_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_boot_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief 自检模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器的自检功能模式。
 * 自检功能用于验证传感器内部电路和机械结构的正常工作状态。
 * 
 * @note 自检功能有助于系统调试和传感器故障诊断。
 */
typedef enum
{
  LIS2DW12_XL_ST_DISABLE      = 0,  /**< 禁用自检：传感器正常工作模式 */
  LIS2DW12_XL_ST_POSITIVE     = 1,  /**< 正向自检：在输出信号上添加正向偏移进行自检 */
  LIS2DW12_XL_ST_NEGATIVE     = 2,  /**< 负向自检：在输出信号上添加负向偏移进行自检 */
} lis2dw12_st_t;
int32_t lis2dw12_self_test_set(const stmdev_ctx_t *ctx, lis2dw12_st_t val);
int32_t lis2dw12_self_test_get(const stmdev_ctx_t *ctx, lis2dw12_st_t *val);

/**
 * @brief 数据就绪信号模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器数据就绪（DRDY）信号的输出模式。
 * 控制DRDY引脚在数据更新时的信号行为。
 * 
 * @note 不同的信号模式适用于不同的系统架构和时序要求。
 */
typedef enum
{
  LIS2DW12_DRDY_LATCHED   = 0,  /**< 锁存模式：DRDY信号保持高电平直到数据被读取 */
  LIS2DW12_DRDY_PULSED    = 1,  /**< 脉冲模式：DRDY信号产生短脉冲，表示新数据可用 */
} lis2dw12_drdy_pulsed_t;
int32_t lis2dw12_data_ready_mode_set(const stmdev_ctx_t *ctx,
                                     lis2dw12_drdy_pulsed_t val);
int32_t lis2dw12_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                     lis2dw12_drdy_pulsed_t *val);

/**
 * @brief 滤波器数据选择枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器输出数据的滤波器选择。
 * 控制从传感器输出的数据经过哪种滤波处理。
 * 
 * @note 不同的滤波器选择影响输出数据的特性和应用场景。
 */
typedef enum
{
  LIS2DW12_LPF_ON_OUT         = 0x00,  /**< 低通滤波器输出：输出经过低通滤波的数据 */
  LIS2DW12_USER_OFFSET_ON_OUT  = 0x01,  /**< 用户偏移输出：输出经过用户偏移校准的数据 */
  LIS2DW12_HIGH_PASS_ON_OUT    = 0x10,  /**< 高通滤波器输出：输出经过高通滤波的数据 */
} lis2dw12_fds_t;
int32_t lis2dw12_filter_path_set(const stmdev_ctx_t *ctx,
                                 lis2dw12_fds_t val);
int32_t lis2dw12_filter_path_get(const stmdev_ctx_t *ctx,
                                 lis2dw12_fds_t *val);

/**
 * @brief 滤波器带宽枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器低通滤波器的带宽设置。
 * 带宽决定了滤波器的截止频率，影响输出数据的噪声和响应特性。
 * 
 * @note 带宽越低，噪声越小，但响应越慢；带宽越高，响应越快，但噪声越大。
 */
typedef enum
{
  LIS2DW12_ODR_DIV_2     = 0,  /**< ODR/2带宽：输出数据率的1/2作为截止频率 */
  LIS2DW12_ODR_DIV_4     = 1,  /**< ODR/4带宽：输出数据率的1/4作为截止频率 */
  LIS2DW12_ODR_DIV_10    = 2,  /**< ODR/10带宽：输出数据率的1/10作为截止频率 */
  LIS2DW12_ODR_DIV_20    = 3,  /**< ODR/20带宽：输出数据率的1/20作为截止频率 */
} lis2dw12_bw_filt_t;
int32_t lis2dw12_filter_bandwidth_set(const stmdev_ctx_t *ctx,
                                      lis2dw12_bw_filt_t val);
int32_t lis2dw12_filter_bandwidth_get(const stmdev_ctx_t *ctx,
                                      lis2dw12_bw_filt_t *val);

int32_t lis2dw12_reference_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_reference_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief SPI接口模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器SPI接口的工作模式。
 * 支持3线和4线SPI通信协议。
 * 
 * @note 不同的SPI模式适用于不同的硬件连接和通信要求。
 */
typedef enum
{
  LIS2DW12_SPI_4_WIRE   = 0,  /**< 4线SPI模式：标准SPI接口，包含MISO、MOSI、SCK、CS四根线 */
  LIS2DW12_SPI_3_WIRE   = 1,  /**< 3线SPI模式：单线双向SPI接口，节省引脚资源 */
} lis2dw12_sim_t;
int32_t lis2dw12_spi_mode_set(const stmdev_ctx_t *ctx, lis2dw12_sim_t val);
int32_t lis2dw12_spi_mode_get(const stmdev_ctx_t *ctx, lis2dw12_sim_t *val);

/**
 * @brief I2C接口使能状态枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器I2C接口的使能状态。
 * 控制I2C通信接口的启用或禁用。
 * 
 * @note 禁用I2C接口可以节省功耗，但会限制通信方式。
 */
typedef enum
{
  LIS2DW12_I2C_ENABLE    = 0,  /**< 启用I2C接口：允许通过I2C进行通信 */
  LIS2DW12_I2C_DISABLE   = 1,  /**< 禁用I2C接口：关闭I2C通信功能以节省功耗 */
} lis2dw12_i2c_disable_t;
int32_t lis2dw12_i2c_interface_set(const stmdev_ctx_t *ctx,
                                   lis2dw12_i2c_disable_t val);
int32_t lis2dw12_i2c_interface_get(const stmdev_ctx_t *ctx,
                                   lis2dw12_i2c_disable_t *val);

/**
 * @brief CS引脚上拉电阻状态枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器CS（片选）引脚内部上拉电阻的连接状态。
 * 控制CS引脚的内部上拉电阻是否启用。
 * 
 * @note 上拉电阻的设置影响CS引脚的电气特性和抗干扰能力。
 */
typedef enum
{
  LIS2DW12_PULL_UP_CONNECT     = 0,  /**< 连接上拉电阻：CS引脚内部上拉电阻启用 */
  LIS2DW12_PULL_UP_DISCONNECT  = 1,  /**< 断开上拉电阻：CS引脚内部上拉电阻禁用 */
} lis2dw12_cs_pu_disc_t;
int32_t lis2dw12_cs_mode_set(const stmdev_ctx_t *ctx,
                             lis2dw12_cs_pu_disc_t val);
int32_t lis2dw12_cs_mode_get(const stmdev_ctx_t *ctx,
                             lis2dw12_cs_pu_disc_t *val);

/**
 * @brief 中断引脚极性枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器中断引脚的极性设置。
 * 控制中断信号的有效电平是高电平还是低电平。
 * 
 * @note 引脚极性的选择需要与主控制器的中断配置保持一致。
 */
typedef enum
{
  LIS2DW12_ACTIVE_HIGH  = 0,  /**< 高电平有效：中断信号在高电平时表示有效状态 */
  LIS2DW12_ACTIVE_LOW   = 1,  /**< 低电平有效：中断信号在低电平时表示有效状态 */
} lis2dw12_h_lactive_t;
int32_t lis2dw12_pin_polarity_set(const stmdev_ctx_t *ctx,
                                  lis2dw12_h_lactive_t val);
int32_t lis2dw12_pin_polarity_get(const stmdev_ctx_t *ctx,
                                  lis2dw12_h_lactive_t *val);

/**
 * @brief 中断信号模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器中断信号的输出模式。
 * 控制中断信号是脉冲模式还是锁存模式。
 * 
 * @note 不同的中断模式适用于不同的系统架构和中断处理方式。
 */
typedef enum
{
  LIS2DW12_INT_PULSED   = 0,  /**< 脉冲模式：中断信号产生短脉冲，自动清除 */
  LIS2DW12_INT_LATCHED  = 1,  /**< 锁存模式：中断信号保持有效直到被读取清除 */
} lis2dw12_lir_t;
int32_t lis2dw12_int_notification_set(const stmdev_ctx_t *ctx,
                                      lis2dw12_lir_t val);
int32_t lis2dw12_int_notification_get(const stmdev_ctx_t *ctx,
                                      lis2dw12_lir_t *val);

/**
 * @brief 引脚输出模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器引脚的输出模式。
 * 控制引脚是推挽输出还是开漏输出。
 * 
 * @note 不同的输出模式适用于不同的电气特性和系统要求。
 */
typedef enum
{
  LIS2DW12_PUSH_PULL   = 0,  /**< 推挽输出：引脚可以主动驱动高电平和低电平 */
  LIS2DW12_OPEN_DRAIN  = 1,  /**< 开漏输出：引脚只能主动驱动低电平，高电平需要外部上拉 */
} lis2dw12_pp_od_t;
int32_t lis2dw12_pin_mode_set(const stmdev_ctx_t *ctx,
                              lis2dw12_pp_od_t val);
int32_t lis2dw12_pin_mode_get(const stmdev_ctx_t *ctx,
                              lis2dw12_pp_od_t *val);

int32_t lis2dw12_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl4_int1_pad_ctrl_t *val);
int32_t lis2dw12_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl4_int1_pad_ctrl_t *val);

int32_t lis2dw12_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl5_int2_pad_ctrl_t *val);
int32_t lis2dw12_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl5_int2_pad_ctrl_t *val);

int32_t lis2dw12_all_on_int1_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_all_on_int1_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_wkup_threshold_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_wkup_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_wkup_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_wkup_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief 唤醒检测数据源枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器唤醒检测功能使用的数据源。
 * 可以选择不同的数据路径来优化唤醒检测的性能。
 * 
 * @note 不同的数据源会影响唤醒检测的响应时间和功耗。
 */
typedef enum
{
  LIS2DW12_HP_FEED           = 0,  /**< 高通滤波器数据源：使用高通滤波器的输出进行唤醒检测 */
  LIS2DW12_USER_OFFSET_FEED  = 1,  /**< 用户偏移数据源：使用经过用户偏移校准的数据进行唤醒检测 */
} lis2dw12_usr_off_on_wu_t;
int32_t lis2dw12_wkup_feed_data_set(const stmdev_ctx_t *ctx,
                                    lis2dw12_usr_off_on_wu_t val);
int32_t lis2dw12_wkup_feed_data_get(const stmdev_ctx_t *ctx,
                                    lis2dw12_usr_off_on_wu_t *val);

/**
 * @brief 活动检测模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器的活动检测模式。
 * 控制传感器如何检测设备的运动和静止状态。
 * 
 * @note 不同的检测模式适用于不同的应用场景和功耗要求。
 */
typedef enum
{
  LIS2DW12_NO_DETECTION        = 0,  /**< 无检测：禁用活动检测功能 */
  LIS2DW12_DETECT_ACT_INACT    = 1,  /**< 活动/非活动检测：检测设备的运动和静止状态 */
  LIS2DW12_DETECT_STAT_MOTION  = 3,  /**< 静止/运动检测：检测设备从静止到运动或从运动到静止的状态变化 */
} lis2dw12_sleep_on_t;
int32_t lis2dw12_act_mode_set(const stmdev_ctx_t *ctx,
                              lis2dw12_sleep_on_t val);
int32_t lis2dw12_act_mode_get(const stmdev_ctx_t *ctx,
                              lis2dw12_sleep_on_t *val);

int32_t lis2dw12_act_sleep_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_act_sleep_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_tap_threshold_x_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_tap_threshold_x_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_tap_threshold_y_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_tap_threshold_y_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief 敲击检测轴优先级枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器敲击检测功能中各个轴的检测优先级顺序。
 * 当多个轴同时检测到敲击时，优先级高的轴将优先被识别。
 * 
 * @note 轴优先级设置影响敲击检测的准确性和响应性，应根据应用需求选择合适的优先级顺序。
 */
typedef enum
{
  LIS2DW12_XYZ    = 0,  /**< X轴最高优先级，Y轴次之，Z轴最低 */
  LIS2DW12_YXZ    = 1,  /**< Y轴最高优先级，X轴次之，Z轴最低 */
  LIS2DW12_XZY    = 2,  /**< X轴最高优先级，Z轴次之，Y轴最低 */
  LIS2DW12_ZYX    = 3,  /**< Z轴最高优先级，Y轴次之，X轴最低 */
  LIS2DW12_YZX    = 5,  /**< Y轴最高优先级，Z轴次之，X轴最低 */
  LIS2DW12_ZXY    = 6,  /**< Z轴最高优先级，X轴次之，Y轴最低 */
} lis2dw12_tap_prior_t;
int32_t lis2dw12_tap_axis_priority_set(const stmdev_ctx_t *ctx,
                                       lis2dw12_tap_prior_t val);
int32_t lis2dw12_tap_axis_priority_get(const stmdev_ctx_t *ctx,
                                       lis2dw12_tap_prior_t *val);

int32_t lis2dw12_tap_threshold_z_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_tap_threshold_z_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_tap_detection_on_z_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dw12_tap_detection_on_z_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dw12_tap_detection_on_y_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dw12_tap_detection_on_y_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dw12_tap_detection_on_x_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dw12_tap_detection_on_x_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lis2dw12_tap_shock_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_tap_shock_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_tap_quiet_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_tap_quiet_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_tap_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_tap_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief 敲击检测模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器敲击检测功能的模式设置。
 * 支持单次敲击检测或同时检测单次和双击。
 * 
 * @note 双击检测需要更长的检测时间，但能提供更丰富的用户交互功能。
 */
typedef enum
{
  LIS2DW12_ONLY_SINGLE          = 0,  /**< 仅单次敲击检测：只检测单次敲击事件 */
  LIS2DW12_BOTH_SINGLE_DOUBLE   = 1,  /**< 单次和双击检测：同时检测单次和双击事件 */
} lis2dw12_single_double_tap_t;
int32_t lis2dw12_tap_mode_set(const stmdev_ctx_t *ctx,
                              lis2dw12_single_double_tap_t val);
int32_t lis2dw12_tap_mode_get(const stmdev_ctx_t *ctx,
                              lis2dw12_single_double_tap_t *val);

int32_t lis2dw12_tap_src_get(const stmdev_ctx_t *ctx,
                             lis2dw12_tap_src_t *val);

int32_t lis2dw12_6d_threshold_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_6d_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_4d_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_4d_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_6d_src_get(const stmdev_ctx_t *ctx,
                            lis2dw12_sixd_src_t *val);

/**
 * @brief 6D方向检测数据源枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器6D方向检测功能使用的数据源。
 * 可以选择不同的数据路径来优化检测性能。
 * 
 * @note 不同的数据源会影响6D检测的响应时间和功耗。
 */
typedef enum
{
  LIS2DW12_ODR_DIV_2_FEED   = 0,  /**< ODR/2数据源：使用输出数据率的一半作为6D检测数据源 */
  LIS2DW12_LPF2_FEED        = 1,  /**< LPF2数据源：使用低通滤波器2的输出作为6D检测数据源 */
} lis2dw12_lpass_on6d_t;
int32_t lis2dw12_6d_feed_data_set(const stmdev_ctx_t *ctx,
                                  lis2dw12_lpass_on6d_t val);
int32_t lis2dw12_6d_feed_data_get(const stmdev_ctx_t *ctx,
                                  lis2dw12_lpass_on6d_t *val);

int32_t lis2dw12_ff_dur_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_ff_dur_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief 自由落体检测阈值枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器自由落体检测功能的阈值设置。
 * 阈值以LSb（最低有效位）为单位，基于±2g满量程范围。
 * 
 * @note 阈值越高，自由落体检测越不敏感；阈值越低，检测越敏感。
 */
typedef enum
{
  LIS2DW12_FF_TSH_5LSb_FS2g  = 0,  /**< 5 LSB阈值，约±0.156g (±2g量程下的5/32) */
  LIS2DW12_FF_TSH_7LSb_FS2g  = 1,  /**< 7 LSB阈值，约±0.219g (±2g量程下的7/32) */
  LIS2DW12_FF_TSH_8LSb_FS2g  = 2,  /**< 8 LSB阈值，约±0.250g (±2g量程下的8/32) */
  LIS2DW12_FF_TSH_10LSb_FS2g = 3,  /**< 10 LSB阈值，约±0.313g (±2g量程下的10/32) */
  LIS2DW12_FF_TSH_11LSb_FS2g = 4,  /**< 11 LSB阈值，约±0.344g (±2g量程下的11/32) */
  LIS2DW12_FF_TSH_13LSb_FS2g = 5,  /**< 13 LSB阈值，约±0.406g (±2g量程下的13/32) */
  LIS2DW12_FF_TSH_15LSb_FS2g = 6,  /**< 15 LSB阈值，约±0.469g (±2g量程下的15/32) */
  LIS2DW12_FF_TSH_16LSb_FS2g = 7,  /**< 16 LSB阈值，约±0.500g (±2g量程下的16/32) */
} lis2dw12_ff_ths_t;
int32_t lis2dw12_ff_threshold_set(const stmdev_ctx_t *ctx,
                                  lis2dw12_ff_ths_t val);
int32_t lis2dw12_ff_threshold_get(const stmdev_ctx_t *ctx,
                                  lis2dw12_ff_ths_t *val);

int32_t lis2dw12_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dw12_fifo_watermark_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief FIFO操作模式枚举类型
 * 
 * 该枚举定义了LIS2DW12加速度传感器FIFO（先进先出队列）的操作模式。
 * FIFO用于缓存传感器数据，支持不同的数据流处理方式。
 * 
 * @note 不同的FIFO模式适用于不同的应用场景，影响数据读取的实时性和缓存效率。
 */
typedef enum
{
  LIS2DW12_BYPASS_MODE             = 0,  /**< 旁路模式：FIFO被禁用，数据直接输出到输出寄存器 */
  LIS2DW12_FIFO_MODE               = 1,  /**< FIFO模式：FIFO被启用，数据存储在FIFO中直到满或触发阈值 */
  LIS2DW12_STREAM_TO_FIFO_MODE     = 3,  /**< 流到FIFO模式：连续数据流模式，FIFO满时停止采集新数据 */
  LIS2DW12_BYPASS_TO_STREAM_MODE   = 4,  /**< 旁路到流模式：FIFO满时切换到流模式，继续采集数据 */
  LIS2DW12_STREAM_MODE             = 6,  /**< 流模式：连续数据流模式，FIFO满时覆盖最旧的数据 */
} lis2dw12_fmode_t;
int32_t lis2dw12_fifo_mode_set(const stmdev_ctx_t *ctx,
                               lis2dw12_fmode_t val);
int32_t lis2dw12_fifo_mode_get(const stmdev_ctx_t *ctx,
                               lis2dw12_fmode_t *val);

int32_t lis2dw12_fifo_data_level_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dw12_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*LIS2DW12_REGS_H */
