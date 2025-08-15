/**
  ******************************************************************************
  * @file    lis2dw12_reg.c
  * @author  Sensors Software Solution Team
  * @brief   LIS2DW12 driver file
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

#include "lis2dw12_reg.h"

/**
  * @defgroup  LIS2DW12
  * @brief     This file provides a set of functions needed to drive the
  *            lis2dw12 enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  LIS2DW12_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak lis2dw12_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                 uint8_t *data,
                                 uint16_t len)
{
  int32_t ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak lis2dw12_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                  uint8_t *data,
                                  uint16_t len)
{
  int32_t ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2DW12_Sensitivity
  * @brief       这些函数将原始数据转换为工程单位。
  * @{
  *
  */

/**
 * @brief 将±2g满量程范围的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±2g满量程范围下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.061 mg/LSB，适用于高性能模式和低功耗模式。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数适用于所有工作模式下的±2g量程范围
 * @note 转换精度：0.061 mg/LSB
 * @note 测量范围：±2g (±2000 mg)
 */
float_t lis2dw12_from_fs2_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;  /**< 将LSB值乘以转换系数0.061 mg/LSB */
}

/**
 * @brief 将±4g满量程范围的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±4g满量程范围下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.122 mg/LSB，适用于高性能模式和低功耗模式。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数适用于所有工作模式下的±4g量程范围
 * @note 转换精度：0.122 mg/LSB
 * @note 测量范围：±4g (±4000 mg)
 */
float_t lis2dw12_from_fs4_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;  /**< 将LSB值乘以转换系数0.122 mg/LSB */
}

/**
 * @brief 将±8g满量程范围的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±8g满量程范围下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.244 mg/LSB，适用于高性能模式和低功耗模式。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数适用于所有工作模式下的±8g量程范围
 * @note 转换精度：0.244 mg/LSB
 * @note 测量范围：±8g (±8000 mg)
 */
float_t lis2dw12_from_fs8_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;  /**< 将LSB值乘以转换系数0.244 mg/LSB */
}

/**
 * @brief 将±16g满量程范围的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±16g满量程范围下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.488 mg/LSB，适用于高性能模式和低功耗模式。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数适用于所有工作模式下的±16g量程范围
 * @note 转换精度：0.488 mg/LSB
 * @note 测量范围：±16g (±16000 mg)
 */
float_t lis2dw12_from_fs16_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.488f;  /**< 将LSB值乘以转换系数0.488 mg/LSB */
}

/**
 * @brief 将±2g满量程范围低功耗模式1的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±2g满量程范围低功耗模式1下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.061 mg/LSB，专门用于低功耗模式1。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数专门用于低功耗模式1下的±2g量程范围
 * @note 转换精度：0.061 mg/LSB
 * @note 测量范围：±2g (±2000 mg)
 */
float_t lis2dw12_from_fs2_lp1_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;  /**< 将LSB值乘以转换系数0.061 mg/LSB */
}

/**
 * @brief 将±4g满量程范围低功耗模式1的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±4g满量程范围低功耗模式1下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.122 mg/LSB，专门用于低功耗模式1。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数专门用于低功耗模式1下的±4g量程范围
 * @note 转换精度：0.122 mg/LSB
 * @note 测量范围：±4g (±4000 mg)
 */
float_t lis2dw12_from_fs4_lp1_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;  /**< 将LSB值乘以转换系数0.122 mg/LSB */
}

/**
 * @brief 将±8g满量程范围低功耗模式1的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±8g满量程范围低功耗模式1下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.244 mg/LSB，专门用于低功耗模式1。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数专门用于低功耗模式1下的±8g量程范围
 * @note 转换精度：0.244 mg/LSB
 * @note 测量范围：±8g (±8000 mg)
 */
float_t lis2dw12_from_fs8_lp1_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;  /**< 将LSB值乘以转换系数0.244 mg/LSB */
}

/**
 * @brief 将±16g满量程范围低功耗模式1的原始数据转换为毫克单位
 * 
 * 该函数将LIS2DW12加速度传感器在±16g满量程范围低功耗模式1下的原始LSB值转换为毫克(mg)单位。
 * 转换系数为0.488 mg/LSB，专门用于低功耗模式1。
 * 
 * @param lsb 原始LSB值，16位有符号整数
 * @return 转换后的加速度值，单位为毫克(mg)
 * 
 * @note 该函数专门用于低功耗模式1下的±16g量程范围
 * @note 转换精度：0.488 mg/LSB
 * @note 测量范围：±16g (±16000 mg)
 */
float_t lis2dw12_from_fs16_lp1_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.488f;  /**< 将LSB值乘以转换系数0.488 mg/LSB */
}

/**
 * @brief 将温度传感器的原始数据转换为摄氏度
 * 
 * 该函数将LIS2DW12加速度传感器内置温度传感器的原始LSB值转换为摄氏度。
 * 转换公式：温度(°C) = (LSB / 256.0) + 25.0
 * 
 * @param lsb 温度传感器的原始LSB值，16位有符号整数
 * @return 转换后的温度值，单位为摄氏度(°C)
 * 
 * @note 温度传感器分辨率：1 LSB = 1/256 °C
 * @note 参考温度：25°C (LSB = 0时的温度)
 * @note 温度范围：-40°C 到 +85°C
 * @note 温度精度：±2°C
 */
float_t lis2dw12_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);  /**< 将LSB值除以256并加上25°C的偏移 */
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Data_Generation
  * @brief     This section groups all the functions concerning
  *            data generation
  * @{
  *
  */

/**
 * @brief 设置加速度传感器工作模式
 *
 * 该函数用于配置LIS2DW12加速度传感器的工作模式，包括基本工作模式、低功耗模式和低噪声模式。
 * 函数会同时修改CTRL1寄存器的mode和lp_mode位，以及CTRL6寄存器的low_noise位。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 工作模式枚举值，包含mode、lp_mode和low_noise的组合配置
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数会分两步配置：先配置CTRL1寄存器，再配置CTRL6寄存器
 * @note 如果任何一步失败，函数会立即返回错误状态
 * @note val参数的低2位用于lp_mode，位2-3用于mode，位4用于low_noise
 */
int32_t lis2dw12_power_mode_set(const stmdev_ctx_t *ctx,
                                lis2dw12_mode_t val)
{
  lis2dw12_ctrl1_t ctrl1;  /**< CTRL1寄存器结构体，用于配置基本工作模式和低功耗模式 */
  lis2dw12_ctrl6_t ctrl6;  /**< CTRL6寄存器结构体，用于配置低噪声模式 */
  int32_t ret;             /**< 函数返回值，用于错误处理 */

  /** 读取CTRL1寄存器的当前值到ctrl1结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, (uint8_t *) &ctrl1, 1);

  /** 如果读取成功，则配置CTRL1寄存器 */
  if (ret == 0)
  {
    /** 提取mode位：将val的位2-3右移2位得到mode值 (0x0C = 0b1100) */
    ctrl1.mode = ((uint8_t) val & 0x0CU) >> 2;
    /** 提取lp_mode位：取val的低2位 (0x03 = 0b0011) */
    ctrl1.lp_mode = (uint8_t) val & 0x03U ;
    /** 将配置写入CTRL1寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL1, (uint8_t *) &ctrl1, 1);
  }

  /** 如果CTRL1配置成功，则读取CTRL6寄存器 */
  if (ret == 0)
  {
    /** 读取CTRL6寄存器的当前值到ctrl6结构体 */
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &ctrl6, 1);
  }

  /** 如果读取CTRL6成功，则配置CTRL6寄存器 */
  if (ret == 0)
  {
    /** 提取low_noise位：将val的位4右移4位得到low_noise值 (0x10 = 0b10000) */
    ctrl6.low_noise = ((uint8_t) val & 0x10U) >> 4;
    /** 将配置写入CTRL6寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &ctrl6, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器工作模式
 *
 * 该函数用于读取LIS2DW12加速度传感器的当前工作模式配置。
 * 函数会读取CTRL1寄存器的mode和lp_mode位，以及CTRL6寄存器的low_noise位，
 * 然后根据这些位的组合值确定当前的工作模式。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前工作模式枚举值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数会读取两个寄存器：CTRL1和CTRL6
 * @note 如果读取失败，val参数不会被修改
 * @note 如果读取到的组合值不在预定义范围内，会默认返回HIGH_PERFORMANCE模式
 */
int32_t lis2dw12_power_mode_get(const stmdev_ctx_t *ctx,
                                lis2dw12_mode_t *val)
{
  lis2dw12_ctrl1_t ctrl1;  /**< CTRL1寄存器结构体，用于存储mode和lp_mode位 */
  lis2dw12_ctrl6_t ctrl6;  /**< CTRL6寄存器结构体，用于存储low_noise位 */
  int32_t ret;             /**< 函数返回值，用于错误处理 */

  /** 读取CTRL1寄存器的当前值到ctrl1结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, (uint8_t *) &ctrl1, 1);

  /** 如果CTRL1读取成功，则继续读取CTRL6寄存器 */
  if (ret == 0)
  {
    /** 读取CTRL6寄存器的当前值到ctrl6结构体 */
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &ctrl6, 1);

    /** 根据寄存器位的组合值确定工作模式 */
    /** 组合算法：low_noise左移4位 + mode左移2位 + lp_mode */
    /** 这与lis2dw12_power_mode_set函数中的位操作相对应 */
    switch (((ctrl6.low_noise << 4) + (ctrl1.mode << 2) +
             ctrl1.lp_mode))
    {
      /** 高性能模式：正常功耗，标准噪声水平 */
      case LIS2DW12_HIGH_PERFORMANCE:
        *val = LIS2DW12_HIGH_PERFORMANCE;
        break;

      /** 连续低功耗模式4：连续采样，低功耗，14位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_4:
        *val = LIS2DW12_CONT_LOW_PWR_4;
        break;

      /** 连续低功耗模式3：连续采样，低功耗，14位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_3:
        *val = LIS2DW12_CONT_LOW_PWR_3;
        break;

      /** 连续低功耗模式2：连续采样，低功耗，14位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_2:
        *val = LIS2DW12_CONT_LOW_PWR_2;
        break;

      /** 连续低功耗模式12位：连续采样，低功耗，12位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_12bit:
        *val = LIS2DW12_CONT_LOW_PWR_12bit;
        break;

      /** 单次低功耗模式4：单次采样，低功耗，14位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_4:
        *val = LIS2DW12_SINGLE_LOW_PWR_4;
        break;

      /** 单次低功耗模式3：单次采样，低功耗，14位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_3:
        *val = LIS2DW12_SINGLE_LOW_PWR_3;
        break;

      /** 单次低功耗模式2：单次采样，低功耗，14位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_2:
        *val = LIS2DW12_SINGLE_LOW_PWR_2;
        break;

      /** 单次低功耗模式12位：单次采样，低功耗，12位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_12bit:
        *val = LIS2DW12_SINGLE_LOW_PWR_12bit;
        break;

      /** 高性能低噪声模式：正常功耗，低噪声水平 */
      case LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE:
        *val = LIS2DW12_HIGH_PERFORMANCE_LOW_NOISE;
        break;

      /** 连续低功耗低噪声模式4：连续采样，低功耗，低噪声，14位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4:
        *val = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_4;
        break;

      /** 连续低功耗低噪声模式3：连续采样，低功耗，低噪声，14位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3:
        *val = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3;
        break;

      /** 连续低功耗低噪声模式2：连续采样，低功耗，低噪声，14位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2:
        *val = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2;
        break;

      /** 连续低功耗低噪声模式12位：连续采样，低功耗，低噪声，12位分辨率 */
      case LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit:
        *val = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit;
        break;

      /** 单次低功耗低噪声模式4：单次采样，低功耗，低噪声，14位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4:
        *val = LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_4;
        break;

      /** 单次低功耗低噪声模式3：单次采样，低功耗，低噪声，14位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3:
        *val = LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_3;
        break;

      /** 单次低功耗低噪声模式2：单次采样，低功耗，低噪声，14位分辨率 */
      case LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2:
        *val = LIS2DW12_SINGLE_LOW_PWR_LOW_NOISE_2;
        break;

      /** 单次低功耗低噪声模式12位：单次采样，低功耗，低噪声，12位分辨率 */
      case LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit:
        *val = LIS2DW12_SINGLE_LOW_LOW_NOISE_PWR_12bit;
        break;

      /** 默认情况：如果组合值不在预定义范围内，返回高性能模式 */
      default:
        *val = LIS2DW12_HIGH_PERFORMANCE;
        break;
    }
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置加速度传感器数据输出速率
 *
 * 该函数用于配置LIS2DW12加速度传感器的数据输出速率(ODR)和睡眠模式。
 * 函数会同时修改CTRL1寄存器的odr位和CTRL3寄存器的slp_mode位。
 * 数据输出速率决定了传感器的采样频率，影响功耗和响应速度。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 数据输出速率枚举值，包含odr和slp_mode的组合配置
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数会分两步配置：先配置CTRL1寄存器的odr位，再配置CTRL3寄存器的slp_mode位
 * @note 如果任何一步失败，函数会立即返回错误状态
 * @note val参数的低4位用于odr，位4-5用于slp_mode
 * @note 支持的数据速率：关闭、1.6Hz(仅低功耗)、12.5Hz、25Hz、50Hz、100Hz、200Hz、400Hz、800Hz、1.6kHz
 */
int32_t lis2dw12_data_rate_set(const stmdev_ctx_t *ctx, lis2dw12_odr_t val)
{
  lis2dw12_ctrl1_t ctrl1;  /**< CTRL1寄存器结构体，用于配置数据输出速率 */
  lis2dw12_ctrl3_t ctrl3;  /**< CTRL3寄存器结构体，用于配置睡眠模式 */
  int32_t ret;             /**< 函数返回值，用于错误处理 */

  /** 读取CTRL1寄存器的当前值到ctrl1结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, (uint8_t *) &ctrl1, 1);

  /** 如果CTRL1读取成功，则配置数据输出速率 */
  if (ret == 0)
  {
    /** 提取odr位：取val的低4位作为数据输出速率值 */
    ctrl1.odr = (uint8_t) val;
    /** 将配置写入CTRL1寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL1, (uint8_t *) &ctrl1, 1);
  }

  /** 如果CTRL1配置成功，则读取CTRL3寄存器 */
  if (ret == 0)
  {
    /** 读取CTRL3寄存器的当前值到ctrl3结构体 */
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &ctrl3, 1);
  }

  /** 如果CTRL3读取成功，则配置睡眠模式 */
  if (ret == 0)
  {
    /** 提取slp_mode位：将val的位4-5右移4位得到睡眠模式值 (0x30 = 0b110000) */
    ctrl3.slp_mode = ((uint8_t) val & 0x30U) >> 4;
    /** 将配置写入CTRL3寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &ctrl3, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器数据输出速率
 *
 * 该函数用于读取LIS2DW12加速度传感器的当前数据输出速率(ODR)和睡眠模式配置。
 * 函数会读取CTRL1寄存器的odr位和CTRL3寄存器的slp_mode位，
 * 然后根据这些位的组合值确定当前的数据输出速率配置。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前数据输出速率枚举值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数会读取两个寄存器：CTRL1和CTRL3
 * @note 如果读取失败，val参数不会被修改
 * @note 组合算法：slp_mode左移4位 + odr，这与lis2dw12_data_rate_set函数中的位操作相对应
 */
int32_t lis2dw12_data_rate_get(const stmdev_ctx_t *ctx, lis2dw12_odr_t *val)
{
  lis2dw12_ctrl1_t ctrl1;  /**< CTRL1寄存器结构体，用于存储odr位 */
  lis2dw12_ctrl3_t ctrl3;  /**< CTRL3寄存器结构体，用于存储slp_mode位 */
  int32_t ret;             /**< 函数返回值，用于错误处理 */

  /** 读取CTRL1寄存器的当前值到ctrl1结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, (uint8_t *) &ctrl1, 1);

  /** 如果CTRL1读取成功，则继续读取CTRL3寄存器 */
  if (ret == 0)
  {
    /** 读取CTRL3寄存器的当前值到ctrl3结构体 */
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &ctrl3, 1);

    /** 根据寄存器位的组合值确定数据输出速率 */
    /** 组合算法：slp_mode左移4位 + odr */
    /** 这与lis2dw12_data_rate_set函数中的位操作相对应 */
    switch ((ctrl3.slp_mode << 4) + ctrl1.odr)
    {
      /** 关闭模式：传感器停止工作，功耗最低 */
      case LIS2DW12_XL_ODR_OFF:
        *val = LIS2DW12_XL_ODR_OFF;
        break;

      /** 1.6Hz低功耗模式：仅支持低功耗模式，1.6Hz采样率 */
      case LIS2DW12_XL_ODR_1Hz6_LP_ONLY:
        *val = LIS2DW12_XL_ODR_1Hz6_LP_ONLY;
        break;

      /** 12.5Hz模式：12.5Hz采样率，平衡功耗和响应速度 */
      case LIS2DW12_XL_ODR_12Hz5:
        *val = LIS2DW12_XL_ODR_12Hz5;
        break;

      /** 25Hz模式：25Hz采样率，适用于一般应用 */
      case LIS2DW12_XL_ODR_25Hz:
        *val = LIS2DW12_XL_ODR_25Hz;
        break;

      /** 50Hz模式：50Hz采样率，适用于需要较快响应的应用 */
      case LIS2DW12_XL_ODR_50Hz:
        *val = LIS2DW12_XL_ODR_50Hz;
        break;

      /** 100Hz模式：100Hz采样率，适用于高动态应用 */
      case LIS2DW12_XL_ODR_100Hz:
        *val = LIS2DW12_XL_ODR_100Hz;
        break;

      /** 200Hz模式：200Hz采样率，适用于高速数据采集 */
      case LIS2DW12_XL_ODR_200Hz:
        *val = LIS2DW12_XL_ODR_200Hz;
        break;

      /** 400Hz模式：400Hz采样率，适用于高频振动检测 */
      case LIS2DW12_XL_ODR_400Hz:
        *val = LIS2DW12_XL_ODR_400Hz;
        break;

      /** 800Hz模式：800Hz采样率，适用于高速运动检测 */
      case LIS2DW12_XL_ODR_800Hz:
        *val = LIS2DW12_XL_ODR_800Hz;
        break;

      /** 1.6kHz模式：1.6kHz采样率，最高采样频率，适用于精密测量 */
      case LIS2DW12_XL_ODR_1k6Hz:
        *val = LIS2DW12_XL_ODR_1k6Hz;
        break;

      /** 软件触发模式：通过软件命令触发单次测量 */
      case LIS2DW12_XL_SET_SW_TRIG:
        *val = LIS2DW12_XL_SET_SW_TRIG;
        break;

      /** 引脚触发模式：通过外部引脚触发单次测量 */
      case LIS2DW12_XL_SET_PIN_TRIG:
        *val = LIS2DW12_XL_SET_PIN_TRIG;
        break;

      /** 默认情况：如果组合值不在预定义范围内，返回关闭模式 */
      default:
        *val = LIS2DW12_XL_ODR_OFF;
        break;
    }
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置块数据更新模式
 *
 * 该函数用于配置LIS2DW12加速度传感器的块数据更新(BDU)功能。
 * BDU功能确保在读取多字节数据时，数据的一致性得到保证。
 * 当BDU启用时，输出寄存器中的数据在完全更新之前不会被覆盖。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val BDU模式设置：0=禁用，1=启用
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 当BDU启用时，确保读取的数据来自同一次测量
 * @note 当BDU禁用时，可能在读取过程中数据被更新，导致数据不一致
 * @note 建议在读取多字节数据时启用BDU功能
 */
int32_t lis2dw12_block_data_update_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于配置BDU位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置BDU位 */
  if (ret == 0)
  {
    /** 设置BDU位：0=禁用块数据更新，1=启用块数据更新 */
    reg.bdu = val;
    /** 将配置写入CTRL2寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取块数据更新模式状态
 *
 * 该函数用于读取LIS2DW12加速度传感器的当前块数据更新(BDU)功能状态。
 * 通过读取CTRL2寄存器的bdu位，可以了解当前是否启用了块数据更新功能。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前BDU状态：0=禁用，1=启用
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note BDU功能影响多字节数据读取的一致性
 */
int32_t lis2dw12_block_data_update_get(const stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于存储BDU位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  /** 提取BDU位值：0=禁用块数据更新，1=启用块数据更新 */
  *val = reg.bdu;

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置加速度传感器满量程范围
 *
 * 该函数用于配置LIS2DW12加速度传感器的满量程范围(FS)。
 * 满量程范围决定了传感器的测量范围和分辨率，影响转换系数。
 * 支持的范围：±2g、±4g、±8g、±16g。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 满量程范围枚举值：LIS2DW12_2g、LIS2DW12_4g、LIS2DW12_8g、LIS2DW12_16g
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL6寄存器的fs位
 * @note 满量程范围影响数据转换系数，需要相应调整转换函数
 * @note 较大的满量程范围提供更大的测量范围但降低分辨率
 * @note 较小的满量程范围提供更高的分辨率但测量范围较小
 */
int32_t lis2dw12_full_scale_set(const stmdev_ctx_t *ctx, lis2dw12_fs_t val)
{
  lis2dw12_ctrl6_t reg;  /**< CTRL6寄存器结构体，用于配置满量程范围 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL6寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置满量程范围 */
  if (ret == 0)
  {
    /** 设置fs位：配置满量程范围值 */
    reg.fs = (uint8_t) val;
    /** 将配置写入CTRL6寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器满量程范围
 *
 * 该函数用于读取LIS2DW12加速度传感器的当前满量程范围(FS)配置。
 * 通过读取CTRL6寄存器的fs位，可以了解当前设置的测量范围。
 * 返回的枚举值对应不同的满量程范围：±2g、±4g、±8g、±16g。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前满量程范围枚举值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 满量程范围决定了数据转换系数和测量精度
 * @note 该函数通过switch语句将寄存器值映射到对应的枚举值
 */
int32_t lis2dw12_full_scale_get(const stmdev_ctx_t *ctx, lis2dw12_fs_t *val)
{
  lis2dw12_ctrl6_t reg;  /**< CTRL6寄存器结构体，用于存储fs位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL6寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &reg, 1);

  /** 根据fs位的值确定当前满量程范围 */
  switch (reg.fs)
  {
    /** ±2g满量程范围：最高分辨率，最小测量范围 */
    case LIS2DW12_2g:
      *val = LIS2DW12_2g;
      break;

    /** ±4g满量程范围：中等分辨率，中等测量范围 */
    case LIS2DW12_4g:
      *val = LIS2DW12_4g;
      break;

    /** ±8g满量程范围：较低分辨率，较大测量范围 */
    case LIS2DW12_8g:
      *val = LIS2DW12_8g;
      break;

    /** ±16g满量程范围：最低分辨率，最大测量范围 */
    case LIS2DW12_16g:
      *val = LIS2DW12_16g;
      break;

    /** 默认情况：如果fs位值不在预定义范围内，返回±2g范围 */
    default:
      *val = LIS2DW12_2g;
      break;
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 读取设备状态寄存器
 *
 * 该函数用于读取LIS2DW12加速度传感器的状态寄存器(STATUS)。
 * 状态寄存器包含各种状态标志位，如数据就绪标志、数据覆盖标志等。
 * 通过读取状态寄存器可以了解传感器的当前工作状态。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储状态寄存器的内容，包含各种状态标志位
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 状态寄存器是只读寄存器，包含实时状态信息
 * @note 该函数直接读取整个状态寄存器到val结构体中
 */
int32_t lis2dw12_status_reg_get(const stmdev_ctx_t *ctx,
                                lis2dw12_status_t *val)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 读取STATUS寄存器的内容到val结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_STATUS, (uint8_t *) val, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器数据就绪标志
 *
 * 该函数用于检查LIS2DW12加速度传感器是否有新的数据可用。
 * 通过读取状态寄存器的drdy位，可以判断传感器是否已经准备好新的加速度数据。
 * 当drdy位为1时，表示有新的数据可以读取。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储数据就绪标志：0=无新数据，1=有新数据
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 建议在读取加速度数据之前先检查此标志
 * @note drdy标志在数据更新时自动置位，读取数据后自动清零
 * @note 该函数专门用于检查数据就绪状态，比读取整个状态寄存器更高效
 */
int32_t lis2dw12_flag_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_status_t reg;  /**< 状态寄存器结构体，用于存储STATUS寄存器内容 */
  int32_t ret;            /**< 函数返回值，用于错误处理 */

  /** 读取STATUS寄存器的内容到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_STATUS, (uint8_t *) &reg, 1);
  /** 提取drdy位：0=无新数据，1=有新数据 */
  *val = reg.drdy;

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}
/**
 * @brief 读取设备所有中断/状态标志
 *
 * 该函数用于一次性读取LIS2DW12加速度传感器的所有中断和状态标志。
 * 函数会连续读取5个寄存器：STATUS_DUP、WAKE_UP_SRC、TAP_SRC、SIXD_SRC、ALL_INT_SRC。
 * 这些寄存器包含各种中断源的状态信息，如唤醒中断、敲击检测、6D方向检测等。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储所有中断/状态标志的联合体结构
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 该函数一次性读取5个字节，提高读取效率
 * @note 读取的寄存器包括：状态重复、唤醒源、敲击源、6D源、所有中断源
 * @note 建议在需要检查多种中断状态时使用此函数
 */
int32_t lis2dw12_all_sources_get(const stmdev_ctx_t *ctx,
                                 lis2dw12_all_sources_t *val)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 连续读取5个中断/状态寄存器到val联合体结构 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_STATUS_DUP, (uint8_t *) val, 5);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置加速度传感器X轴用户偏移校正
 *
 * 该函数用于设置LIS2DW12加速度传感器X轴的用户偏移校正值。
 * 偏移校正用于补偿传感器的零偏误差，提高测量精度。
 * 偏移值以二进制补码形式表示，有效范围为[-127, 127]。
 * 偏移值的权重取决于USR_OFF_W位的设置。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输入缓冲区，包含要写入的偏移校正值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 偏移值范围：-127到+127
 * @note 偏移值以二进制补码形式存储
 * @note 偏移值的实际权重取决于USR_OFF_W位的配置
 * @note 该函数直接写入X_OFS_USR寄存器
 * @note 偏移校正会在数据输出时自动应用
 */
int32_t lis2dw12_usr_offset_x_set(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 将偏移校正值写入X_OFS_USR寄存器 */
  ret = lis2dw12_write_reg(ctx, LIS2DW12_X_OFS_USR, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器X轴用户偏移校正
 *
 * 该函数用于读取LIS2DW12加速度传感器X轴的当前用户偏移校正值。
 * 偏移校正值以二进制补码形式存储，有效范围为[-127, 127]。
 * 通过读取当前偏移值，可以了解传感器的校正状态。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输出缓冲区，用于存储读取的偏移校正值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，buff参数不会被修改
 * @note 偏移值以二进制补码形式存储
 * @note 偏移值的实际权重取决于USR_OFF_W位的配置
 * @note 该函数直接读取X_OFS_USR寄存器
 * @note 读取的值是当前应用的偏移校正
 */
int32_t lis2dw12_usr_offset_x_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 从X_OFS_USR寄存器读取偏移校正值 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_X_OFS_USR, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置加速度传感器Y轴用户偏移校正
 *
 * 该函数用于设置LIS2DW12加速度传感器Y轴的用户偏移校正值。
 * 偏移校正用于补偿传感器的零偏误差，提高测量精度。
 * 偏移值以二进制补码形式表示，有效范围为[-127, 127]。
 * 偏移值的权重取决于USR_OFF_W位的设置。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输入缓冲区，包含要写入的偏移校正值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 偏移值范围：-127到+127
 * @note 偏移值以二进制补码形式存储
 * @note 偏移值的实际权重取决于USR_OFF_W位的配置
 * @note 该函数直接写入Y_OFS_USR寄存器
 * @note 偏移校正会在数据输出时自动应用
 */
int32_t lis2dw12_usr_offset_y_set(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 将偏移校正值写入Y_OFS_USR寄存器 */
  ret = lis2dw12_write_reg(ctx, LIS2DW12_Y_OFS_USR, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器Y轴用户偏移校正
 *
 * 该函数用于读取LIS2DW12加速度传感器Y轴的当前用户偏移校正值。
 * 偏移校正值以二进制补码形式存储，有效范围为[-127, 127]。
 * 通过读取当前偏移值，可以了解传感器的校正状态。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输出缓冲区，用于存储读取的偏移校正值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，buff参数不会被修改
 * @note 偏移值以二进制补码形式存储
 * @note 偏移值的实际权重取决于USR_OFF_W位的配置
 * @note 该函数直接读取Y_OFS_USR寄存器
 * @note 读取的值是当前应用的偏移校正
 */
int32_t lis2dw12_usr_offset_y_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 从Y_OFS_USR寄存器读取偏移校正值 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_Y_OFS_USR, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置加速度传感器Z轴用户偏移校正
 *
 * 该函数用于设置LIS2DW12加速度传感器Z轴的用户偏移校正值。
 * 偏移校正用于补偿传感器的零偏误差，提高测量精度。
 * 偏移值以二进制补码形式表示，有效范围为[-127, 127]。
 * 偏移值的权重取决于USR_OFF_W位的设置。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输入缓冲区，包含要写入的偏移校正值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 偏移值范围：-127到+127
 * @note 偏移值以二进制补码形式存储
 * @note 偏移值的实际权重取决于USR_OFF_W位的配置
 * @note 该函数直接写入Z_OFS_USR寄存器
 * @note 偏移校正会在数据输出时自动应用
 */
int32_t lis2dw12_usr_offset_z_set(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 将偏移校正值写入Z_OFS_USR寄存器 */
  ret = lis2dw12_write_reg(ctx, LIS2DW12_Z_OFS_USR, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器Z轴用户偏移校正
 *
 * 该函数用于读取LIS2DW12加速度传感器Z轴的当前用户偏移校正值。
 * 偏移校正值以二进制补码形式存储，有效范围为[-127, 127]。
 * 通过读取当前偏移值，可以了解传感器的校正状态。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输出缓冲区，用于存储读取的偏移校正值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，buff参数不会被修改
 * @note 偏移值以二进制补码形式存储
 * @note 偏移值的实际权重取决于USR_OFF_W位的配置
 * @note 该函数直接读取Z_OFS_USR寄存器
 * @note 读取的值是当前应用的偏移校正
 */
int32_t lis2dw12_usr_offset_z_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 从Z_OFS_USR寄存器读取偏移校正值 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_Z_OFS_USR, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置加速度传感器用户偏移权重
 *
 * 该函数用于设置LIS2DW12加速度传感器用户偏移校正的权重。
 * 权重设置影响X_OFS_USR、Y_OFS_USR、Z_OFS_USR寄存器中偏移值的实际效果。
 * 不同的权重设置提供不同的偏移校正精度和范围。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 偏移权重枚举值，决定偏移校正的精度和范围
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL_REG7寄存器的usr_off_w位
 * @note 权重设置影响所有轴的偏移校正效果
 * @note 不同的权重提供不同的偏移校正精度
 * @note 权重设置应与偏移值配合使用以获得最佳校正效果
 */
int32_t lis2dw12_offset_weight_set(const stmdev_ctx_t *ctx,
                                   lis2dw12_usr_off_w_t val)
{
  lis2dw12_ctrl_reg7_t reg;  /**< CTRL_REG7寄存器结构体，用于配置偏移权重 */
  int32_t ret;               /**< 函数返回值，用于错误处理 */

  /** 读取CTRL_REG7寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置偏移权重 */
  if (ret == 0)
  {
    /** 设置usr_off_w位：配置偏移校正权重 */
    reg.usr_off_w = (uint8_t) val;
    /** 将配置写入CTRL_REG7寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取加速度传感器用户偏移权重
 *
 * 该函数用于读取LIS2DW12加速度传感器用户偏移校正的当前权重设置。
 * 权重设置影响X_OFS_USR、Y_OFS_USR、Z_OFS_USR寄存器中偏移值的实际效果。
 * 通过读取当前权重值，可以了解偏移校正的精度和范围配置。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前偏移权重枚举值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 权重设置影响所有轴的偏移校正效果
 * @note 该函数通过switch语句将寄存器值映射到对应的枚举值
 * @note 权重值决定了偏移校正的精度和范围
 */
int32_t lis2dw12_offset_weight_get(const stmdev_ctx_t *ctx,
                                   lis2dw12_usr_off_w_t *val)
{
  lis2dw12_ctrl_reg7_t reg;  /**< CTRL_REG7寄存器结构体，用于存储usr_off_w位 */
  int32_t ret;               /**< 函数返回值，用于错误处理 */

  /** 读取CTRL_REG7寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  /** 根据usr_off_w位的值确定当前偏移权重设置 */
  switch (reg.usr_off_w)
  {
    /** 高精度模式：每个LSB对应977微克，提供更高精度的偏移校正 */
    case LIS2DW12_LSb_977ug:
      *val = LIS2DW12_LSb_977ug;
      break;

    /** 标准模式：每个LSB对应15.6毫克，提供标准精度的偏移校正 */
    case LIS2DW12_LSb_15mg6:
      *val = LIS2DW12_LSb_15mg6;
      break;

    /** 默认情况：如果权重值不在预定义范围内，返回高精度模式 */
    default:
      *val = LIS2DW12_LSb_977ug;
      break;
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Data_Output
  * @brief     This section groups all the data output functions.
  * @{
  *
  */

/**
 * @brief 获取温度传感器原始数据
 *
 * 该函数用于读取LIS2DW12加速度传感器内部温度传感器的原始数据。
 * 温度数据存储在OUT_T_L和OUT_T_H寄存器中，以16位二进制补码形式表示。
 * 函数将两个8位寄存器组合成16位温度值。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储16位温度原始数据（二进制补码格式）
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 温度数据以二进制补码形式存储，需要转换为实际温度值
 * @note 该函数连续读取2个字节：OUT_T_L（低字节）和OUT_T_H（高字节）
 * @note 温度分辨率约为1LSB = 1°C
 * @note 温度范围约为-40°C到+85°C
 */
int32_t lis2dw12_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];  /**< 临时缓冲区，用于存储温度寄存器的2个字节 */
  int32_t ret;      /**< 函数返回值，用于错误处理 */

  /** 连续读取温度寄存器的低字节和高字节到buff缓冲区 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_OUT_T_L, buff, 2);
  
  /** 将高字节（buff[1]）转换为16位有符号整数 */
  *val = (int16_t)buff[1];
  /** 组合高低字节：高字节左移8位 + 低字节，形成完整的16位温度值 */
  *val = (*val * 256) + (int16_t)buff[0];

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取三轴加速度传感器原始数据
 *
 * 该函数用于读取LIS2DW12加速度传感器X、Y、Z三轴的原始加速度数据。
 * 每个轴的数据存储在对应的OUT_X_L/H、OUT_Y_L/H、OUT_Z_L/H寄存器中，
 * 以16位二进制补码形式表示。函数将6个8位寄存器组合成3个16位加速度值。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数数组，用于存储X、Y、Z三轴的16位加速度原始数据（二进制补码格式）
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 加速度数据以二进制补码形式存储，需要转换为实际加速度值
 * @note 该函数连续读取6个字节：X轴低高字节、Y轴低高字节、Z轴低高字节
 * @note 数据顺序：val[0]=X轴，val[1]=Y轴，val[2]=Z轴
 * @note 数据分辨率取决于满量程范围设置
 * @note 建议在读取前检查数据就绪标志
 */
int32_t lis2dw12_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];  /**< 临时缓冲区，用于存储6个字节的加速度数据 */
  int32_t ret;      /**< 函数返回值，用于错误处理 */

  /** 连续读取X、Y、Z三轴的加速度数据寄存器到buff缓冲区 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_OUT_X_L, buff, 6);
  
  /** 处理X轴数据：将高字节转换为16位有符号整数 */
  val[0] = (int16_t)buff[1];
  /** 组合X轴高低字节：高字节左移8位 + 低字节，形成完整的16位X轴加速度值 */
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  
  /** 处理Y轴数据：将高字节转换为16位有符号整数 */
  val[1] = (int16_t)buff[3];
  /** 组合Y轴高低字节：高字节左移8位 + 低字节，形成完整的16位Y轴加速度值 */
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  
  /** 处理Z轴数据：将高字节转换为16位有符号整数 */
  val[2] = (int16_t)buff[5];
  /** 组合Z轴高低字节：高字节左移8位 + 低字节，形成完整的16位Z轴加速度值 */
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Common
  * @brief     This section groups common useful functions.
  * @{
  *
  */

/**
 * @brief 获取设备ID
 *
 * 该函数用于读取LIS2DW12加速度传感器的设备标识符。
 * 设备ID存储在WHO_AM_I寄存器中，用于验证设备类型和通信连接。
 * LIS2DW12的设备ID固定为0x44。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param buff 输出缓冲区，用于存储设备ID值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，buff参数不会被修改
 * @note 设备ID用于验证传感器类型和通信连接状态
 * @note LIS2DW12的设备ID固定为0x44
 * @note 建议在初始化时调用此函数验证设备连接
 * @note 该函数直接读取WHO_AM_I寄存器
 */
int32_t lis2dw12_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;  /**< 函数返回值，用于错误处理 */

  /** 从WHO_AM_I寄存器读取设备ID */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_WHO_AM_I, buff, 1);

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置寄存器地址自动递增功能
 *
 * 该函数用于配置LIS2DW12加速度传感器在串行接口多字节访问时的地址自动递增功能。
 * 当启用自动递增时，在读取或写入多个连续寄存器时，寄存器地址会自动递增。
 * 这提高了多字节数据传输的效率。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 自动递增功能开关：0=禁用，1=启用
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL2寄存器的if_add_inc位
 * @note 启用自动递增可以提高多字节数据传输效率
 * @note 建议在需要连续读取多个寄存器时启用此功能
 * @note 自动递增功能适用于SPI和I2C接口
 */
int32_t lis2dw12_auto_increment_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于配置自动递增功能 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置自动递增功能 */
  if (ret == 0)
  {
    /** 设置if_add_inc位：配置寄存器地址自动递增功能 */
    reg.if_add_inc = val;
    /** 将配置写入CTRL2寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取寄存器地址自动递增功能状态
 *
 * 该函数用于读取LIS2DW12加速度传感器寄存器地址自动递增功能的当前状态。
 * 通过读取CTRL2寄存器的if_add_inc位，可以了解当前是否启用了自动递增功能。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储自动递增功能状态：0=禁用，1=启用
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 自动递增功能影响多字节数据传输的效率
 * @note 该函数直接读取CTRL2寄存器的if_add_inc位
 * @note 返回值表示当前自动递增功能的配置状态
 */
int32_t lis2dw12_auto_increment_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于存储if_add_inc位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  /** 提取if_add_inc位：0=禁用自动递增，1=启用自动递增 */
  *val = reg.if_add_inc;

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置软件复位功能
 *
 * 该函数用于触发LIS2DW12加速度传感器的软件复位。
 * 软件复位会恢复用户寄存器中的默认值，将传感器恢复到出厂配置状态。
 * 复位后需要重新配置传感器的各项参数。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 软件复位触发：0=正常模式，1=触发软件复位
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL2寄存器的soft_reset位
 * @note 软件复位会清除所有用户配置，恢复出厂默认值
 * @note 复位后需要等待一段时间让传感器重新初始化
 * @note 建议在传感器出现异常时使用软件复位功能
 * @note 复位后需要重新配置数据输出率、满量程范围等参数
 */
int32_t lis2dw12_reset_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于配置软件复位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置软件复位 */
  if (ret == 0)
  {
    /** 设置soft_reset位：触发软件复位或恢复正常模式 */
    reg.soft_reset = val;
    /** 将配置写入CTRL2寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取软件复位功能状态
 *
 * 该函数用于读取LIS2DW12加速度传感器软件复位功能的当前状态。
 * 通过读取CTRL2寄存器的soft_reset位，可以了解当前是否处于复位状态。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储软件复位状态：0=正常模式，1=复位状态
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 复位状态表示传感器正在进行软件复位操作
 * @note 该函数直接读取CTRL2寄存器的soft_reset位
 * @note 复位完成后，该位会自动清零
 * @note 建议在复位后检查此状态确认复位完成
 */
int32_t lis2dw12_reset_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于存储soft_reset位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  /** 提取soft_reset位：0=正常模式，1=复位状态 */
  *val = reg.soft_reset;

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置内存重启功能
 *
 * 该函数用于触发LIS2DW12加速度传感器的内存重启功能。
 * 内存重启会重新加载校准参数，恢复存储在传感器内部存储器中的校准数据。
 * 这有助于恢复传感器的校准状态，提高测量精度。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 内存重启触发：0=正常模式，1=触发内存重启
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL2寄存器的boot位
 * @note 内存重启会重新加载内部校准参数
 * @note 重启后需要等待一段时间让传感器重新加载校准数据
 * @note 建议在传感器精度下降时使用内存重启功能
 * @note 内存重启不会清除用户配置，只重新加载校准参数
 */
int32_t lis2dw12_boot_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于配置内存重启 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置内存重启 */
  if (ret == 0)
  {
    /** 设置boot位：触发内存重启或恢复正常模式 */
    reg.boot = val;
    /** 将配置写入CTRL2寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取内存重启功能状态
 *
 * 该函数用于读取LIS2DW12加速度传感器内存重启功能的当前状态。
 * 通过读取CTRL2寄存器的boot位，可以了解当前是否正在进行内存重启操作。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储内存重启状态：0=正常模式，1=重启状态
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 重启状态表示传感器正在进行内存重启操作
 * @note 该函数直接读取CTRL2寄存器的boot位
 * @note 重启完成后，该位会自动清零
 * @note 建议在重启后检查此状态确认重启完成
 */
int32_t lis2dw12_boot_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_ctrl2_t reg;  /**< CTRL2寄存器结构体，用于存储boot位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL2寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  /** 提取boot位：0=正常模式，1=重启状态 */
  *val = reg.boot;

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置传感器自检功能
 *
 * 该函数用于配置LIS2DW12加速度传感器的自检功能。
 * 自检功能可以验证传感器的内部功能是否正常工作，
 * 通过比较正常模式和自检模式下的输出数据来检测传感器状态。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 自检模式枚举值：LIS2DW12_ST_DISABLE=禁用，LIS2DW12_ST_POSITIVE=正自检，LIS2DW12_ST_NEGATIVE=负自检
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL3寄存器的st位
 * @note 自检功能用于验证传感器内部功能是否正常
 * @note 正自检和负自检会产生不同的输出偏移
 * @note 建议在传感器初始化或故障诊断时使用自检功能
 * @note 自检完成后应恢复到正常模式
 */
int32_t lis2dw12_self_test_set(const stmdev_ctx_t *ctx, lis2dw12_st_t val)
{
  lis2dw12_ctrl3_t reg;  /**< CTRL3寄存器结构体，用于配置自检功能 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL3寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置自检功能 */
  if (ret == 0)
  {
    /** 设置st位：配置传感器自检模式 */
    reg.st = (uint8_t) val;
    /** 将配置写入CTRL3寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取传感器自检功能状态
 *
 * 该函数用于读取LIS2DW12加速度传感器自检功能的当前配置。
 * 通过读取CTRL3寄存器的st位，可以了解当前的自检模式设置。
 * 函数使用switch语句将寄存器值映射到对应的枚举值。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前自检模式枚举值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 该函数通过switch语句将寄存器值映射到对应的枚举值
 * @note 自检模式包括：禁用、正自检、负自检
 * @note 默认情况下返回禁用自检模式
 * @note 该函数直接读取CTRL3寄存器的st位
 */
int32_t lis2dw12_self_test_get(const stmdev_ctx_t *ctx, lis2dw12_st_t *val)
{
  lis2dw12_ctrl3_t reg;  /**< CTRL3寄存器结构体，用于存储st位 */
  int32_t ret;           /**< 函数返回值，用于错误处理 */

  /** 读取CTRL3寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  /** 根据st位的值确定当前自检模式 */
  switch (reg.st)
  {
    /** 禁用自检模式：传感器正常工作模式 */
    case LIS2DW12_XL_ST_DISABLE:
      *val = LIS2DW12_XL_ST_DISABLE;
      break;

    /** 正自检模式：产生正向输出偏移 */
    case LIS2DW12_XL_ST_POSITIVE:
      *val = LIS2DW12_XL_ST_POSITIVE;
      break;

    /** 负自检模式：产生负向输出偏移 */
    case LIS2DW12_XL_ST_NEGATIVE:
      *val = LIS2DW12_XL_ST_NEGATIVE;
      break;

    /** 默认情况：如果st位值不在预定义范围内，返回禁用自检模式 */
    default:
      *val = LIS2DW12_XL_ST_DISABLE;
      break;
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 设置数据就绪信号模式
 *
 * 该函数用于配置LIS2DW12加速度传感器数据就绪(DRDY)信号的输出模式。
 * DRDY信号用于指示传感器有新的数据可用，支持脉冲模式和锁存模式。
 * 脉冲模式下DRDY信号在数据更新时产生短脉冲，锁存模式下信号保持高电平直到数据被读取。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 数据就绪信号模式枚举值：LIS2DW12_DRDY_LATCHED=锁存模式，LIS2DW12_DRDY_PULSED=脉冲模式
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数使用读-修改-写模式配置CTRL_REG7寄存器的drdy_pulsed位
 * @note 脉冲模式适合中断驱动的数据读取
 * @note 锁存模式适合轮询方式的数据读取
 * @note DRDY信号模式影响数据读取的时序要求
 * @note 建议根据应用需求选择合适的DRDY模式
 */
int32_t lis2dw12_data_ready_mode_set(const stmdev_ctx_t *ctx,
                                     lis2dw12_drdy_pulsed_t val)
{
  lis2dw12_ctrl_reg7_t reg;  /**< CTRL_REG7寄存器结构体，用于配置DRDY模式 */
  int32_t ret;               /**< 函数返回值，用于错误处理 */

  /** 读取CTRL_REG7寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  /** 如果读取成功，则配置数据就绪信号模式 */
  if (ret == 0)
  {
    /** 设置drdy_pulsed位：配置DRDY信号输出模式 */
    reg.drdy_pulsed = (uint8_t) val;
    /** 将配置写入CTRL_REG7寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
 * @brief 获取数据就绪信号模式
 *
 * 该函数用于读取LIS2DW12加速度传感器数据就绪(DRDY)信号的当前配置。
 * 通过读取CTRL_REG7寄存器的drdy_pulsed位，可以了解当前的DRDY信号输出模式。
 * 函数使用switch语句将寄存器值映射到对应的枚举值。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 输出参数，用于存储当前数据就绪信号模式枚举值
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 如果读取失败，val参数不会被修改
 * @note 该函数通过switch语句将寄存器值映射到对应的枚举值
 * @note DRDY模式包括：锁存模式和脉冲模式
 * @note 默认情况下返回锁存模式
 * @note 该函数直接读取CTRL_REG7寄存器的drdy_pulsed位
 */
int32_t lis2dw12_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                     lis2dw12_drdy_pulsed_t *val)
{
  lis2dw12_ctrl_reg7_t reg;  /**< CTRL_REG7寄存器结构体，用于存储drdy_pulsed位 */
  int32_t ret;               /**< 函数返回值，用于错误处理 */

  /** 读取CTRL_REG7寄存器的当前值到reg结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  /** 根据drdy_pulsed位的值确定当前数据就绪信号模式 */
  switch (reg.drdy_pulsed)
  {
    /** 锁存模式：DRDY信号保持高电平直到数据被读取 */
    case LIS2DW12_DRDY_LATCHED:
      *val = LIS2DW12_DRDY_LATCHED;
      break;

    /** 脉冲模式：DRDY信号在数据更新时产生短脉冲 */
    case LIS2DW12_DRDY_PULSED:
      *val = LIS2DW12_DRDY_PULSED;
      break;

    /** 默认情况：如果drdy_pulsed位值不在预定义范围内，返回锁存模式 */
    default:
      *val = LIS2DW12_DRDY_LATCHED;
      break;
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Filters
  * @brief     This section group all the functions concerning the filters
  *            configuration.
  * @{
  *
  */

/**
 * @brief 设置加速度传感器滤波数据路径
 *
 * 该函数用于配置LIS2DW12加速度传感器的滤波数据路径。
 * 滤波数据路径决定了传感器输出的数据来源，可以选择原始数据或经过滤波处理的数据。
 * 该配置涉及CTRL6寄存器的fds位和CTRL_REG7寄存器的usr_off_on_out位。
 *
 * @param ctx 读写接口定义结构体指针，包含设备句柄和读写函数指针
 * @param val 滤波数据路径枚举值，决定数据输出路径和用户偏移应用方式
 * @retval 接口状态 (MANDATORY: 返回0表示无错误)
 *
 * @note 该函数需要配置两个寄存器：CTRL6和CTRL_REG7
 * @note fds位控制滤波数据选择，usr_off_on_out位控制用户偏移应用
 * @note 不同的滤波路径提供不同的数据质量和处理方式
 * @note 滤波路径设置影响数据输出的精度和响应特性
 * @note 建议根据应用需求选择合适的滤波数据路径
 */
int32_t lis2dw12_filter_path_set(const stmdev_ctx_t *ctx,
                                 lis2dw12_fds_t val)
{
  lis2dw12_ctrl6_t ctrl6;        /**< CTRL6寄存器结构体，用于配置fds位 */
  lis2dw12_ctrl_reg7_t ctrl_reg7; /**< CTRL_REG7寄存器结构体，用于配置usr_off_on_out位 */
  int32_t ret;                    /**< 函数返回值，用于错误处理 */

  /** 读取CTRL6寄存器的当前值到ctrl6结构体 */
  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &ctrl6, 1);

  /** 如果读取成功，则配置CTRL6寄存器的fds位 */
  if (ret == 0)
  {
    /** 设置fds位：从val参数提取第4位(0x10)并右移4位 */
    ctrl6.fds = ((uint8_t) val & 0x10U) >> 4;
    /** 将配置写入CTRL6寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &ctrl6, 1);
  }

  /** 如果CTRL6配置成功，则读取CTRL_REG7寄存器 */
  if (ret == 0)
  {
    /** 读取CTRL_REG7寄存器的当前值到ctrl_reg7结构体 */
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7,
                            (uint8_t *) &ctrl_reg7, 1);
  }

  /** 如果读取成功，则配置CTRL_REG7寄存器的usr_off_on_out位 */
  if (ret == 0)
  {
    /** 设置usr_off_on_out位：从val参数提取第0位(0x01) */
    ctrl_reg7.usr_off_on_out = (uint8_t) val & 0x01U;
    /** 将配置写入CTRL_REG7寄存器 */
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7,
                             (uint8_t *) &ctrl_reg7, 1);
  }

  /** 返回操作结果：0表示成功，非0表示错误 */
  return ret;
}

/**
  * @brief  Accelerometer filtering path for outputs.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fds in reg CTRL6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_filter_path_get(const stmdev_ctx_t *ctx,
                                 lis2dw12_fds_t *val)
{
  lis2dw12_ctrl6_t ctrl6;
  lis2dw12_ctrl_reg7_t ctrl_reg7;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &ctrl6, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7,
                            (uint8_t *) &ctrl_reg7, 1);

    switch ((ctrl6.fds << 4) + ctrl_reg7.usr_off_on_out)
    {
      case LIS2DW12_LPF_ON_OUT:
        *val = LIS2DW12_LPF_ON_OUT;
        break;

      case LIS2DW12_USER_OFFSET_ON_OUT:
        *val = LIS2DW12_USER_OFFSET_ON_OUT;
        break;

      case LIS2DW12_HIGH_PASS_ON_OUT:
        *val = LIS2DW12_HIGH_PASS_ON_OUT;
        break;

      default:
        *val = LIS2DW12_LPF_ON_OUT;
        break;
    }
  }

  return ret;
}

/**
  * @brief   Accelerometer cutoff filter frequency. Valid for low and high
  *          pass filter.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bw_filt in reg CTRL6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_filter_bandwidth_set(const stmdev_ctx_t *ctx,
                                      lis2dw12_bw_filt_t val)
{
  lis2dw12_ctrl6_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.bw_filt = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief   Accelerometer cutoff filter frequency. Valid for low and
  *          high pass filter.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of bw_filt in reg CTRL6
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_filter_bandwidth_get(const stmdev_ctx_t *ctx,
                                      lis2dw12_bw_filt_t *val)
{
  lis2dw12_ctrl6_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, (uint8_t *) &reg, 1);

  switch (reg.bw_filt)
  {
    case LIS2DW12_ODR_DIV_2:
      *val = LIS2DW12_ODR_DIV_2;
      break;

    case LIS2DW12_ODR_DIV_4:
      *val = LIS2DW12_ODR_DIV_4;
      break;

    case LIS2DW12_ODR_DIV_10:
      *val = LIS2DW12_ODR_DIV_10;
      break;

    case LIS2DW12_ODR_DIV_20:
      *val = LIS2DW12_ODR_DIV_20;
      break;

    default:
      *val = LIS2DW12_ODR_DIV_2;
      break;
  }

  return ret;
}

/**
  * @brief  Enable HP filter reference mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hp_ref_mode in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_reference_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.hp_ref_mode = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Enable HP filter reference mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hp_ref_mode in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_reference_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  *val = reg.hp_ref_mode;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LIS2DW12_Serial_Interface
  * @brief      This section groups all the functions concerning main serial
  *             interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sim in reg CTRL2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_spi_mode_set(const stmdev_ctx_t *ctx, lis2dw12_sim_t val)
{
  lis2dw12_ctrl2_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.sim = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sim in reg CTRL2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_spi_mode_get(const stmdev_ctx_t *ctx, lis2dw12_sim_t *val)
{
  lis2dw12_ctrl2_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  switch (reg.sim)
  {
    case LIS2DW12_SPI_4_WIRE:
      *val = LIS2DW12_SPI_4_WIRE;
      break;

    case LIS2DW12_SPI_3_WIRE:
      *val = LIS2DW12_SPI_3_WIRE;
      break;

    default:
      *val = LIS2DW12_SPI_4_WIRE;
      break;
  }

  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of i2c_disable in
  *                                 reg CTRL2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_i2c_interface_set(const stmdev_ctx_t *ctx,
                                   lis2dw12_i2c_disable_t val)
{
  lis2dw12_ctrl2_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.i2c_disable = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of i2c_disable in reg CTRL2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_i2c_interface_get(const stmdev_ctx_t *ctx,
                                   lis2dw12_i2c_disable_t *val)
{
  lis2dw12_ctrl2_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  switch (reg.i2c_disable)
  {
    case LIS2DW12_I2C_ENABLE:
      *val = LIS2DW12_I2C_ENABLE;
      break;

    case LIS2DW12_I2C_DISABLE:
      *val = LIS2DW12_I2C_DISABLE;
      break;

    default:
      *val = LIS2DW12_I2C_ENABLE;
      break;
  }

  return ret;
}

/**
  * @brief  Disconnect CS pull-up.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of cs_pu_disc in reg CTRL2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_cs_mode_set(const stmdev_ctx_t *ctx,
                             lis2dw12_cs_pu_disc_t val)
{
  lis2dw12_ctrl2_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.cs_pu_disc = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Disconnect CS pull-up.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of cs_pu_disc in reg CTRL2
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_cs_mode_get(const stmdev_ctx_t *ctx,
                             lis2dw12_cs_pu_disc_t *val)
{
  lis2dw12_ctrl2_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, (uint8_t *) &reg, 1);

  switch (reg.cs_pu_disc)
  {
    case LIS2DW12_PULL_UP_CONNECT:
      *val = LIS2DW12_PULL_UP_CONNECT;
      break;

    case LIS2DW12_PULL_UP_DISCONNECT:
      *val = LIS2DW12_PULL_UP_DISCONNECT;
      break;

    default:
      *val = LIS2DW12_PULL_UP_CONNECT;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Interrupt_Pins
  * @brief     This section groups all the functions that manage interrupt pins
  * @{
  *
  */

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of h_lactive in reg CTRL3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_polarity_set(const stmdev_ctx_t *ctx,
                                  lis2dw12_h_lactive_t val)
{
  lis2dw12_ctrl3_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.h_lactive = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of h_lactive in reg CTRL3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_polarity_get(const stmdev_ctx_t *ctx,
                                  lis2dw12_h_lactive_t *val)
{
  lis2dw12_ctrl3_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  switch (reg.h_lactive)
  {
    case LIS2DW12_ACTIVE_HIGH:
      *val = LIS2DW12_ACTIVE_HIGH;
      break;

    case LIS2DW12_ACTIVE_LOW:
      *val = LIS2DW12_ACTIVE_LOW;
      break;

    default:
      *val = LIS2DW12_ACTIVE_HIGH;
      break;
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lir in reg CTRL3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_int_notification_set(const stmdev_ctx_t *ctx,
                                      lis2dw12_lir_t val)
{
  lis2dw12_ctrl3_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.lir = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lir in reg CTRL3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_int_notification_get(const stmdev_ctx_t *ctx,
                                      lis2dw12_lir_t *val)
{
  lis2dw12_ctrl3_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  switch (reg.lir)
  {
    case LIS2DW12_INT_PULSED:
      *val = LIS2DW12_INT_PULSED;
      break;

    case LIS2DW12_INT_LATCHED:
      *val = LIS2DW12_INT_LATCHED;
      break;

    default:
      *val = LIS2DW12_INT_PULSED;
      break;
  }

  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of pp_od in reg CTRL3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_mode_set(const stmdev_ctx_t *ctx, lis2dw12_pp_od_t val)
{
  lis2dw12_ctrl3_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.pp_od = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of pp_od in reg CTRL3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_mode_get(const stmdev_ctx_t *ctx,
                              lis2dw12_pp_od_t *val)
{
  lis2dw12_ctrl3_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, (uint8_t *) &reg, 1);

  switch (reg.pp_od)
  {
    case LIS2DW12_PUSH_PULL:
      *val = LIS2DW12_PUSH_PULL;
      break;

    case LIS2DW12_OPEN_DRAIN:
      *val = LIS2DW12_OPEN_DRAIN;
      break;

    default:
      *val = LIS2DW12_PUSH_PULL;
      break;
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register CTRL4_INT1_PAD_CTRL.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl4_int1_pad_ctrl_t *val)
{
  lis2dw12_ctrl5_int2_pad_ctrl_t ctrl5_int2_pad_ctrl;
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL,
                          (uint8_t *)&ctrl5_int2_pad_ctrl, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  if (ret == 0)
  {
    if ((val->int1_tap |
         val->int1_ff |
         val->int1_wu |
         val->int1_single_tap |
         val->int1_6d |
         ctrl5_int2_pad_ctrl.int2_sleep_state |
         ctrl5_int2_pad_ctrl.int2_sleep_chg) != PROPERTY_DISABLE)
    {
      reg.interrupts_enable = PROPERTY_ENABLE;
    }

    else
    {
      reg.interrupts_enable = PROPERTY_DISABLE;
    }

    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL,
                             (uint8_t *) val, 1);
  }

  if (ret == 0)
  {
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register CTRL4_INT1_PAD_CTRL.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl4_int1_pad_ctrl_t *val)
{
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL,
                          (uint8_t *) val, 1);

  return ret;
}

/**
  * @brief   Select the signal that need to route on int2 pad.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register CTRL5_INT2_PAD_CTRL.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl5_int2_pad_ctrl_t *val)
{
  lis2dw12_ctrl4_int1_pad_ctrl_t ctrl4_int1_pad_ctrl;
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL,
                          (uint8_t *) &ctrl4_int1_pad_ctrl, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  if (ret == 0)
  {
    if ((ctrl4_int1_pad_ctrl.int1_tap |
         ctrl4_int1_pad_ctrl.int1_ff |
         ctrl4_int1_pad_ctrl.int1_wu |
         ctrl4_int1_pad_ctrl.int1_single_tap |
         ctrl4_int1_pad_ctrl.int1_6d |
         val->int2_sleep_state | val->int2_sleep_chg) != PROPERTY_DISABLE)
    {
      reg.interrupts_enable = PROPERTY_ENABLE;
    }

    else
    {
      reg.interrupts_enable = PROPERTY_DISABLE;
    }

    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL,
                             (uint8_t *) val, 1);
  }

  if (ret == 0)
  {
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register CTRL5_INT2_PAD_CTRL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                    lis2dw12_ctrl5_int2_pad_ctrl_t *val)
{
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL,
                          (uint8_t *) val, 1);

  return ret;
}
/**
  * @brief All interrupt signals become available on INT1 pin.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of int2_on_int1 in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_all_on_int1_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.int2_on_int1 = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of int2_on_int1 in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_all_on_int1_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  *val = reg.int2_on_int1;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Wake_Up_Event
  * @brief     This section groups all the functions that manage the Wake
  *            Up event generation.
  * @{
  *
  */

/**
  * @brief  Threshold for wakeup.1 LSB = FS_XL / 64.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wk_ths in reg WAKE_UP_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_wkup_threshold_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_wake_up_ths_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.wk_ths = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_THS, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for wakeup.1 LSB = FS_XL / 64.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wk_ths in reg WAKE_UP_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_wkup_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_wake_up_ths_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, (uint8_t *) &reg, 1);
  *val = reg.wk_ths;

  return ret;
}

/**
  * @brief  Wake up duration event.1LSb = 1 / ODR.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wake_dur in reg WAKE_UP_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_wkup_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_wake_up_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.wake_dur = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Wake up duration event.1LSb = 1 / ODR.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wake_dur in reg WAKE_UP_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_wkup_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_wake_up_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, (uint8_t *) &reg, 1);
  *val = reg.wake_dur;

  return ret;
}

/**
  * @brief  Data sent to wake-up interrupt function.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_on_wu in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_wkup_feed_data_set(const stmdev_ctx_t *ctx,
                                    lis2dw12_usr_off_on_wu_t val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.usr_off_on_wu = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Data sent to wake-up interrupt function.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of usr_off_on_wu in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_wkup_feed_data_get(const stmdev_ctx_t *ctx,
                                    lis2dw12_usr_off_on_wu_t *val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  switch (reg.usr_off_on_wu)
  {
    case LIS2DW12_HP_FEED:
      *val = LIS2DW12_HP_FEED;
      break;

    case LIS2DW12_USER_OFFSET_FEED:
      *val = LIS2DW12_USER_OFFSET_FEED;
      break;

    default:
      *val = LIS2DW12_HP_FEED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LIS2DW12_Activity/Inactivity_Detection
  * @brief      This section groups all the functions concerning
  *             activity/inactivity detection.
  * @{
  *
  */

/**
  * @brief  Config activity / inactivity or
  *         stationary / motion detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_on / stationary in
  *                  reg WAKE_UP_THS / WAKE_UP_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_act_mode_set(const stmdev_ctx_t *ctx,
                              lis2dw12_sleep_on_t val)
{
  lis2dw12_wake_up_ths_t wake_up_ths;
  lis2dw12_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS,
                          (uint8_t *) &wake_up_ths, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR,
                            (uint8_t *) &wake_up_dur, 1);
  }

  if (ret == 0)
  {
    wake_up_ths.sleep_on = (uint8_t) val & 0x01U;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_THS,
                             (uint8_t *) &wake_up_ths, 1);
  }

  if (ret == 0)
  {
    wake_up_dur.stationary = ((uint8_t)val & 0x02U) >> 1;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR,
                             (uint8_t *) &wake_up_dur, 1);
  }

  return ret;
}

/**
  * @brief  Config activity / inactivity or
  *         stationary / motion detection. [get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sleep_on in reg WAKE_UP_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_act_mode_get(const stmdev_ctx_t *ctx,
                              lis2dw12_sleep_on_t *val)
{
  lis2dw12_wake_up_ths_t wake_up_ths;
  lis2dw12_wake_up_dur_t wake_up_dur;;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS,
                          (uint8_t *) &wake_up_ths, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR,
                            (uint8_t *) &wake_up_dur, 1);

    switch ((wake_up_dur.stationary << 1) + wake_up_ths.sleep_on)
    {
      case LIS2DW12_NO_DETECTION:
        *val = LIS2DW12_NO_DETECTION;
        break;

      case LIS2DW12_DETECT_ACT_INACT:
        *val = LIS2DW12_DETECT_ACT_INACT;
        break;

      case LIS2DW12_DETECT_STAT_MOTION:
        *val = LIS2DW12_DETECT_STAT_MOTION;
        break;

      default:
        *val = LIS2DW12_NO_DETECTION;
        break;
    }
  }

  return ret;
}

/**
  * @brief  Duration to go in sleep mode (1 LSb = 512 / ODR).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_act_sleep_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_wake_up_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.sleep_dur = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Duration to go in sleep mode (1 LSb = 512 / ODR).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_act_sleep_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_wake_up_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, (uint8_t *) &reg, 1);
  *val = reg.sleep_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Tap_Generator
  * @brief     This section groups all the functions that manage the tap
  *            and double tap event generation.
  * @{
  *
  */

/**
  * @brief  Threshold for tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_thsx in reg TAP_THS_X
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_threshold_x_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_tap_ths_x_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_thsx = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_thsx in reg TAP_THS_X
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_threshold_x_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_tap_ths_x_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);
  *val = reg.tap_thsx;

  return ret;
}

/**
  * @brief  Threshold for tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_thsy in reg TAP_THS_Y
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_threshold_y_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_tap_ths_y_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_thsy = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Y, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_thsy in reg TAP_THS_Y
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_threshold_y_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_tap_ths_y_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, (uint8_t *) &reg, 1);
  *val = reg.tap_thsy;

  return ret;
}

/**
  * @brief  Selection of axis priority for TAP detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_prior in reg TAP_THS_Y
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_axis_priority_set(const stmdev_ctx_t *ctx,
                                       lis2dw12_tap_prior_t val)
{
  lis2dw12_tap_ths_y_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_prior = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Y, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Selection of axis priority for TAP detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of tap_prior in reg TAP_THS_Y
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_axis_priority_get(const stmdev_ctx_t *ctx,
                                       lis2dw12_tap_prior_t *val)
{
  lis2dw12_tap_ths_y_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, (uint8_t *) &reg, 1);

  switch (reg.tap_prior)
  {
    case LIS2DW12_XYZ:
      *val = LIS2DW12_XYZ;
      break;

    case LIS2DW12_YXZ:
      *val = LIS2DW12_YXZ;
      break;

    case LIS2DW12_XZY:
      *val = LIS2DW12_XZY;
      break;

    case LIS2DW12_ZYX:
      *val = LIS2DW12_ZYX;
      break;

    case LIS2DW12_YZX:
      *val = LIS2DW12_YZX;
      break;

    case LIS2DW12_ZXY:
      *val = LIS2DW12_ZXY;
      break;

    default:
      *val = LIS2DW12_XYZ;
      break;
  }

  return ret;
}

/**
  * @brief  Threshold for tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_thsz in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_threshold_z_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_thsz = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_thsz in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_threshold_z_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  *val = reg.tap_thsz;

  return ret;
}

/**
  * @brief  Enable Z direction in tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_z_en in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_detection_on_z_set(const stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_z_en = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Enable Z direction in tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_z_en in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_detection_on_z_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  *val = reg.tap_z_en;

  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_y_en in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_detection_on_y_set(const stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_y_en = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_y_en in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_detection_on_y_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  *val = reg.tap_y_en;

  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_x_en in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_detection_on_x_set(const stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.tap_x_en = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_x_en in reg TAP_THS_Z
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_detection_on_x_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  lis2dw12_tap_ths_z_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, (uint8_t *) &reg, 1);
  *val = reg.tap_x_en;

  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value
  *         of these bits is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different value, 1LSB
  *         corresponds to 8*ODR_XL time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of shock in reg INT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_shock_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_int_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.shock = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value
  *         of these bits is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different value, 1LSB
  *         corresponds to 8*ODR_XL time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of shock in reg INT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_shock_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_int_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);
  *val = reg.shock;

  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which
  *         there must not be any overthreshold event.
  *         The default value of these bits is 00b which corresponds
  *         to 2*ODR_XL time. If the QUIET[1:0] bits are set to a different
  *         value, 1LSB corresponds to 4*ODR_XL time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of quiet in reg INT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_quiet_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_int_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.quiet = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which
  *         there must not be any overthreshold event.
  *         The default value of these bits is 00b which corresponds
  *         to 2*ODR_XL time. If the QUIET[1:0] bits are set to a different
  *         value, 1LSB corresponds to 4*ODR_XL time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of quiet in reg INT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_quiet_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_int_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);
  *val = reg.quiet;

  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses
  *         the maximum time between two consecutive detected taps to
  *         determine a double tap event.
  *         The default value of these bits is 0000b which corresponds
  *         to 16*ODR_XL time. If the DUR[3:0] bits are set to a different
  *         value, 1LSB corresponds to 32*ODR_XL time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of latency in reg INT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_int_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.latency = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses
  *         the maximum time between two consecutive detected taps to
  *         determine a double tap event.
  *         The default value of these bits is 0000b which corresponds
  *         to 16*ODR_XL time. If the DUR[3:0] bits are set to a different
  *         value, 1LSB corresponds to 32*ODR_XL time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of latency in reg INT_DUR
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_int_dur_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, (uint8_t *) &reg, 1);
  *val = reg.latency;

  return ret;
}

/**
  * @brief  Single/double-tap event enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of single_double_tap in reg WAKE_UP_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_mode_set(const stmdev_ctx_t *ctx,
                              lis2dw12_single_double_tap_t val)
{
  lis2dw12_wake_up_ths_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.single_double_tap = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_THS, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Single/double-tap event enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of single_double_tap in reg WAKE_UP_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_mode_get(const stmdev_ctx_t *ctx,
                              lis2dw12_single_double_tap_t *val)
{
  lis2dw12_wake_up_ths_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, (uint8_t *) &reg, 1);

  switch (reg.single_double_tap)
  {
    case LIS2DW12_ONLY_SINGLE:
      *val = LIS2DW12_ONLY_SINGLE;
      break;

    case LIS2DW12_BOTH_SINGLE_DOUBLE:
      *val = LIS2DW12_BOTH_SINGLE_DOUBLE;
      break;

    default:
      *val = LIS2DW12_ONLY_SINGLE;
      break;
  }

  return ret;
}

/**
  * @brief  Read the tap / double tap source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  lis2dw12_tap_src: union of registers from TAP_SRC to
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_tap_src_get(const stmdev_ctx_t *ctx,
                             lis2dw12_tap_src_t *val)
{
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_SRC, (uint8_t *) val, 1);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LIS2DW12_Six_Position_Detection(6D/4D)
  * @brief      This section groups all the functions concerning six
  *             position detection (6D).
  * @{
  *
  */

/**
  * @brief  Threshold for 4D/6D function.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of 6d_ths in reg TAP_THS_X
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_6d_threshold_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_tap_ths_x_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg._6d_ths = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Threshold for 4D/6D function.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of 6d_ths in reg TAP_THS_X
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_6d_threshold_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_tap_ths_x_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);
  *val = reg._6d_ths;

  return ret;
}

/**
  * @brief  4D orientation detection enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of 4d_en in reg TAP_THS_X
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_4d_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_tap_ths_x_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg._4d_en = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  4D orientation detection enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of 4d_en in reg TAP_THS_X
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_4d_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_tap_ths_x_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, (uint8_t *) &reg, 1);
  *val = reg._4d_en;

  return ret;
}

/**
  * @brief  Read the 6D tap source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      union of registers from SIXD_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_6d_src_get(const stmdev_ctx_t *ctx,
                            lis2dw12_sixd_src_t *val)
{
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_SIXD_SRC, (uint8_t *) val, 1);

  return ret;
}
/**
  * @brief  Data sent to 6D interrupt function.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpass_on6d in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_6d_feed_data_set(const stmdev_ctx_t *ctx,
                                  lis2dw12_lpass_on6d_t val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.lpass_on6d = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Data sent to 6D interrupt function.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lpass_on6d in reg CTRL_REG7
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_6d_feed_data_get(const stmdev_ctx_t *ctx,
                                  lis2dw12_lpass_on6d_t *val)
{
  lis2dw12_ctrl_reg7_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, (uint8_t *) &reg, 1);

  switch (reg.lpass_on6d)
  {
    case LIS2DW12_ODR_DIV_2_FEED:
      *val = LIS2DW12_ODR_DIV_2_FEED;
      break;

    case LIS2DW12_LPF2_FEED:
      *val = LIS2DW12_LPF2_FEED;
      break;

    default:
      *val = LIS2DW12_ODR_DIV_2_FEED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Free_Fall
  * @brief     This section group all the functions concerning
  *            the free fall detection.
  * @{
  *
  */

/**
  * @brief  Free fall duration event(1LSb = 1 / ODR).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ff_dur in reg
  *                  WAKE_UP_DUR /F REE_FALL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_ff_dur_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_wake_up_dur_t wake_up_dur;
  lis2dw12_free_fall_t free_fall;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR,
                          (uint8_t *) &wake_up_dur, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_FREE_FALL,
                            (uint8_t *) &free_fall, 1);
  }

  if (ret == 0)
  {
    wake_up_dur.ff_dur = ((uint8_t) val & 0x20U) >> 5;
    free_fall.ff_dur = (uint8_t) val & 0x1FU;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR,
                             (uint8_t *) &wake_up_dur, 1);
  }

  if (ret == 0)
  {
    ret = lis2dw12_write_reg(ctx, LIS2DW12_FREE_FALL,
                             (uint8_t *) &free_fall, 1);
  }

  return ret;
}

/**
  * @brief  Free fall duration event(1LSb = 1 / ODR).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ff_dur in
  *                  reg WAKE_UP_DUR /F REE_FALL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_ff_dur_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_wake_up_dur_t wake_up_dur;
  lis2dw12_free_fall_t free_fall;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR,
                          (uint8_t *) &wake_up_dur, 1);

  if (ret == 0)
  {
    ret = lis2dw12_read_reg(ctx, LIS2DW12_FREE_FALL,
                            (uint8_t *) &free_fall, 1);
    *val = (wake_up_dur.ff_dur << 5) + free_fall.ff_dur;
  }

  return ret;
}

/**
  * @brief  Free fall threshold setting.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ff_ths in reg FREE_FALL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_ff_threshold_set(const stmdev_ctx_t *ctx,
                                  lis2dw12_ff_ths_t val)
{
  lis2dw12_free_fall_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FREE_FALL, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.ff_ths = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_FREE_FALL, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  Free fall threshold setting.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of ff_ths in reg FREE_FALL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_ff_threshold_get(const stmdev_ctx_t *ctx,
                                  lis2dw12_ff_ths_t *val)
{
  lis2dw12_free_fall_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FREE_FALL, (uint8_t *) &reg, 1);

  switch (reg.ff_ths)
  {
    case LIS2DW12_FF_TSH_5LSb_FS2g:
      *val = LIS2DW12_FF_TSH_5LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_7LSb_FS2g:
      *val = LIS2DW12_FF_TSH_7LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_8LSb_FS2g:
      *val = LIS2DW12_FF_TSH_8LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_10LSb_FS2g:
      *val = LIS2DW12_FF_TSH_10LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_11LSb_FS2g:
      *val = LIS2DW12_FF_TSH_11LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_13LSb_FS2g:
      *val = LIS2DW12_FF_TSH_13LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_15LSb_FS2g:
      *val = LIS2DW12_FF_TSH_15LSb_FS2g;
      break;

    case LIS2DW12_FF_TSH_16LSb_FS2g:
      *val = LIS2DW12_FF_TSH_16LSb_FS2g;
      break;

    default:
      *val = LIS2DW12_FF_TSH_5LSb_FS2g;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LIS2DW12_Fifo
  * @brief     This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  lis2dw12_fifo_ctrl_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.fth = val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_FIFO_CTRL, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_watermark_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_fifo_ctrl_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, (uint8_t *) &reg, 1);
  *val = reg.fth;

  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fmode in reg FIFO_CTRL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_mode_set(const stmdev_ctx_t *ctx,
                               lis2dw12_fmode_t val)
{
  lis2dw12_fifo_ctrl_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, (uint8_t *) &reg, 1);

  if (ret == 0)
  {
    reg.fmode = (uint8_t) val;
    ret = lis2dw12_write_reg(ctx, LIS2DW12_FIFO_CTRL, (uint8_t *) &reg, 1);
  }

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fmode in reg FIFO_CTRL
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_mode_get(const stmdev_ctx_t *ctx,
                               lis2dw12_fmode_t *val)
{
  lis2dw12_fifo_ctrl_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, (uint8_t *) &reg, 1);

  switch (reg.fmode)
  {
    case LIS2DW12_BYPASS_MODE:
      *val = LIS2DW12_BYPASS_MODE;
      break;

    case LIS2DW12_FIFO_MODE:
      *val = LIS2DW12_FIFO_MODE;
      break;

    case LIS2DW12_STREAM_TO_FIFO_MODE:
      *val = LIS2DW12_STREAM_TO_FIFO_MODE;
      break;

    case LIS2DW12_BYPASS_TO_STREAM_MODE:
      *val = LIS2DW12_BYPASS_TO_STREAM_MODE;
      break;

    case LIS2DW12_STREAM_MODE:
      *val = LIS2DW12_STREAM_MODE;
      break;

    default:
      *val = LIS2DW12_BYPASS_MODE;
      break;
  }

  return ret;
}

/**
  * @brief  Number of unread samples stored in FIFO.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of diff in reg FIFO_SAMPLES
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_data_level_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_fifo_samples_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_SAMPLES, (uint8_t *) &reg, 1);
  *val = reg.diff;

  return ret;
}
/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_ovr in reg FIFO_SAMPLES
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_fifo_samples_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_SAMPLES, (uint8_t *) &reg, 1);
  *val = reg.fifo_ovr;

  return ret;
}
/**
  * @brief  FIFO threshold status flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_fth in reg FIFO_SAMPLES
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2dw12_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_fifo_samples_t reg;
  int32_t ret;

  ret = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_SAMPLES, (uint8_t *) &reg, 1);
  *val = reg.fifo_fth;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @}
  *
  */
