/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2018 Robert Bosch Kft  All Rights Reserved
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch (China) Investment Ltd. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2018 Robert Bosch Kft  All Rights Reserved
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch (China) Investment Ltd. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/
/*! \file smi130_gyro.h
    \brief smi130_gyro Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef _SMI130_GYRO_H
#define _SMI130_GYRO_H

#include "smi130_defs.h"

#define SMI130_GYRO_U16 uint16_t       /* 16 bit achieved with short */
#define SMI130_GYRO_S16 int16_t
#define SMI130_GYRO_S32 int32_t        /* 32 bit achieved with int   */

/**\brief defines the calling parameter types of the SMI130_GYRO_WR_FUNCTION */
#define SMI130_GYRO_BUS_WR_RETURN_TYPE int8_t

/**\brief links the order of parameters defined in
SMI130_GYRO_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define SMI130_GYRO_BUS_WR_PARAM_TYPES uint8_t, uint8_t,\
uint8_t *, uint8_t

/**\brief links the order of parameters defined in
SMI130_GYRO_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define SMI130_GYRO_BUS_WR_PARAM_ORDER(device_addr, register_addr,\
register_data, wr_len)

/* never change this line */
#define SMI130_GYRO_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
register_data, wr_len)
/**\brief defines the return parameter type of the SMI130_GYRO_RD_FUNCTION
*/
#define SMI130_GYRO_BUS_RD_RETURN_TYPE int8_t
/**\brief defines the calling parameter types of the SMI130_GYRO_RD_FUNCTION
*/
#define SMI130_GYRO_BUS_RD_PARAM_TYPES uint8_t, uint8_t,\
uint8_t *, uint8_t
/**\brief links the order of parameters defined in \
SMI130_GYRO_BUS_RD_PARAM_TYPE to function calls used inside the API
*/
#define SMI130_GYRO_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
register_data)
/* never change this line */
#define SMI130_GYRO_BUS_READ_FUNC(device_addr, register_addr,\
register_data, rd_len)bus_read(device_addr, register_addr,\
register_data, rd_len)
/**\brief defines the return parameter type of the SMI130_GYRO_RD_FUNCTION
*/
#define SMI130_GYRO_BURST_RD_RETURN_TYPE int8_t
/**\brief defines the calling parameter types of the SMI130_GYRO_RD_FUNCTION
*/
#define SMI130_GYRO_BURST_RD_PARAM_TYPES uint8_t,\
uint8_t, uint8_t *, int32_t
/**\brief links the order of parameters defined in \
SMI130_GYRO_BURST_RD_PARAM_TYPE to function calls used inside the API
*/
#define SMI130_GYRO_BURST_RD_PARAM_ORDER (device_addr, register_addr,\
register_data)
/* never change this line */
#define SMI130_GYRO_BURST_READ_FUNC(device_addr, register_addr,\
register_data, rd_len)burst_read(device_addr, \
register_addr, register_data, rd_len)
/**\brief defines the return parameter type of the SMI130_GYRO_DELAY_FUNCTION
*/
#define SMI130_GYRO_DELAY_RETURN_TYPE void
/* never change this line */
#define SMI130_GYRO_DELAY_FUNC(delay_in_msec)\
		delay_func(delay_in_msec)
#define SMI130_GYRO_RETURN_FUNCTION_TYPE			  int8_t
/**< This refers SMI130_GYRO return type as int8_t */

#define	SMI130_GYRO_I2C_ADDR1				          (0x68)
#define	SMI130_GYRO_I2C_ADDR					      SMI130_GYRO_I2C_ADDR1
#define	SMI130_GYRO_I2C_ADDR2				          (0x69)

#define SMI130_GYRO_CHIP_ID                           (0x0F)

/*Define of registers*/

/* Hard Wired */
#define SMI130_GYRO_CHIP_ID_ADDR					  (0x00)
/**<Address of Chip ID Register*/

/* Data Register */
#define SMI130_GYRO_RATE_X_LSB_ADDR                   (0x02)
/**<        Address of X axis Rate LSB Register       */
#define SMI130_GYRO_RATE_X_MSB_ADDR                   (0x03)
/**<        Address of X axis Rate MSB Register       */
#define SMI130_GYRO_RATE_Y_LSB_ADDR                   (0x04)
/**<        Address of Y axis Rate LSB Register       */
#define SMI130_GYRO_RATE_Y_MSB_ADDR                   (0x05)
/**<        Address of Y axis Rate MSB Register       */
#define SMI130_GYRO_RATE_Z_LSB_ADDR                   (0x06)
/**<        Address of Z axis Rate LSB Register       */
#define SMI130_GYRO_RATE_Z_MSB_ADDR                   (0x07)
/**<        Address of Z axis Rate MSB Register       */
#define SMI130_GYRO_TEMP_ADDR                         (0x08)
/**<        Address of Temperature Data LSB Register  */

/* Status Register */
#define SMI130_GYRO_INT_STATUS1_ADDR                  (0x0A)
/**<        Address of Interrupt status Register 1    */

/* Control Register */
#define SMI130_GYRO_RANGE_ADDR                        (0x0F)
/**<        Address of Range address Register         */
#define SMI130_GYRO_BW_ADDR                           (0x10)
/**<        Address of Bandwidth Register             */
#define SMI130_GYRO_MODE_LPM1_ADDR                    (0x11)
/**<        Address of Mode LPM1 Register             */
#define SMI130_GYRO_MODE_LPM2_ADDR                    (0x12)
/**<        Address of Mode LPM2 Register             */
#define SMI130_GYRO_RATED_HBW_ADDR                    (0x13)
/**<        Address of Rate HBW Register              */
#define SMI130_GYRO_BGW_SOFTRESET_ADDR                (0x14)
/**<        Address of BGW Softreset Register         */
#define SMI130_GYRO_INT_ENABLE0_ADDR                  (0x15)
/**<        Address of Interrupt Enable 0             */
#define SMI130_GYRO_INT_ENABLE1_ADDR                  (0x16)
/**<        Address of Interrupt Enable 1             */
#define SMI130_GYRO_INT_MAP_1_ADDR                    (0x18)
/**<        Address of Interrupt MAP 1                */
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR                 (0x34)
/**<        Address of BGW SPI3,WDT Register          */
#define SMI130_GYRO_SELF_TEST_ADDR                    (0x3C)
/**<        Address of BGW Self test Register         */

/* Rate X LSB Register */
#define SMI130_GYRO_RATE_X_LSB_VALUEX__POS        (0)

/**< Last 8 bits of RateX LSB Registers */
#define SMI130_GYRO_RATE_X_LSB_VALUEX__LEN        (8)
#define SMI130_GYRO_RATE_X_LSB_VALUEX__MSK        (0xFF)
#define SMI130_GYRO_RATE_X_LSB_VALUEX__REG        SMI130_GYRO_RATE_X_LSB_ADDR

/* Rate Y LSB Register */
/**<  Last 8 bits of RateY LSB Registers */
#define SMI130_GYRO_RATE_Y_LSB_VALUEY__POS        (0)
#define SMI130_GYRO_RATE_Y_LSB_VALUEY__LEN        (8)
#define SMI130_GYRO_RATE_Y_LSB_VALUEY__MSK        (0xFF)
#define SMI130_GYRO_RATE_Y_LSB_VALUEY__REG        SMI130_GYRO_RATE_Y_LSB_ADDR

/* Rate Z LSB Register */
/**< Last 8 bits of RateZ LSB Registers */
#define SMI130_GYRO_RATE_Z_LSB_VALUEZ__POS        (0)
#define SMI130_GYRO_RATE_Z_LSB_VALUEZ__LEN        (8)
#define SMI130_GYRO_RATE_Z_LSB_VALUEZ__MSK        (0xFF)
#define SMI130_GYRO_RATE_Z_LSB_VALUEZ__REG        SMI130_GYRO_RATE_Z_LSB_ADDR

/* Interrupt status 1 Register */
/**< 7th bit of Interrupt status 1 register */
#define SMI130_GYRO_INT_STATUS1_DATA_INT__POS           (7)
#define SMI130_GYRO_INT_STATUS1_DATA_INT__LEN           (1)
#define SMI130_GYRO_INT_STATUS1_DATA_INT__MSK           (0x80)
#define SMI130_GYRO_INT_STATUS1_DATA_INT__REG           SMI130_GYRO_INT_STATUS1_ADDR

/**< First 3 bits of range Registers */
#define SMI130_GYRO_RANGE_ADDR_RANGE__POS           (0)
#define SMI130_GYRO_RANGE_ADDR_RANGE__LEN           (3)
#define SMI130_GYRO_RANGE_ADDR_RANGE__MSK           (0x07)
#define SMI130_GYRO_RANGE_ADDR_RANGE__REG           SMI130_GYRO_RANGE_ADDR

/**< First 3 bits of Bandwidth Registers */
#define SMI130_GYRO_BW_ADDR__POS             (0)
#define SMI130_GYRO_BW_ADDR__LEN             (3)
#define SMI130_GYRO_BW_ADDR__MSK             (0x07)
#define SMI130_GYRO_BW_ADDR__REG             SMI130_GYRO_BW_ADDR

/**< 6th bit of Bandwidth Registers */
#define SMI130_GYRO_BW_ADDR_IMG_STB__POS             (6)
#define SMI130_GYRO_BW_ADDR_IMG_STB__LEN             (1)
#define SMI130_GYRO_BW_ADDR_IMG_STB__MSK             (0x40)
#define SMI130_GYRO_BW_ADDR_IMG_STB__REG             SMI130_GYRO_BW_ADDR

/**< 5th and 7th bit of LPM1 Register */
#define SMI130_GYRO_MODE_LPM1__POS             (5)
#define SMI130_GYRO_MODE_LPM1__LEN             (3)
#define SMI130_GYRO_MODE_LPM1__MSK             (0xA0)
#define SMI130_GYRO_MODE_LPM1__REG             SMI130_GYRO_MODE_LPM1_ADDR

/**< 4th & 5th bit of Mode LPM2 Register */
#define SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__POS          (4)
#define SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__LEN          (2)
#define SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__MSK          (0x30)
#define SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__REG          SMI130_GYRO_MODE_LPM2_ADDR

/**< 7th bit of HBW Register */
#define SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__POS         (7)
#define SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__LEN         (1)
#define SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__MSK         (0x80)
#define SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__REG         SMI130_GYRO_RATED_HBW_ADDR

/**< 6th bit of HBW Register */
#define SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__POS          (6)
#define SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__LEN          (1)
#define SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__MSK          (0x40)
#define SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__REG          SMI130_GYRO_RATED_HBW_ADDR

/**< 7th bit of Interrupt Enable 0 Registers */
#define SMI130_GYRO_INT_ENABLE0_DATAEN__POS               (7)
#define SMI130_GYRO_INT_ENABLE0_DATAEN__LEN               (1)
#define SMI130_GYRO_INT_ENABLE0_DATAEN__MSK               (0x80)
#define SMI130_GYRO_INT_ENABLE0_DATAEN__REG               SMI130_GYRO_INT_ENABLE0_ADDR

/**< 1st bit of Interrupt Enable 1 Registers */
#define SMI130_GYRO_INT_ENABLE1_IT1_OD__POS               (1)
#define SMI130_GYRO_INT_ENABLE1_IT1_OD__LEN               (1)
#define SMI130_GYRO_INT_ENABLE1_IT1_OD__MSK               (0x02)
#define SMI130_GYRO_INT_ENABLE1_IT1_OD__REG               SMI130_GYRO_INT_ENABLE1_ADDR

/**< 0th bit of Interrupt Enable 1 Registers */
#define SMI130_GYRO_INT_ENABLE1_IT1_LVL__POS              (0)
#define SMI130_GYRO_INT_ENABLE1_IT1_LVL__LEN              (1)
#define SMI130_GYRO_INT_ENABLE1_IT1_LVL__MSK              (0x01)
#define SMI130_GYRO_INT_ENABLE1_IT1_LVL__REG              SMI130_GYRO_INT_ENABLE1_ADDR

/**< 0th bit of MAP_1Registers */
#define SMI130_GYRO_MAP_1_INT1_DATA__POS                  (0)
#define SMI130_GYRO_MAP_1_INT1_DATA__LEN                  (1)
#define SMI130_GYRO_MAP_1_INT1_DATA__MSK                  (0x01)
#define SMI130_GYRO_MAP_1_INT1_DATA__REG                  SMI130_GYRO_INT_MAP_1_ADDR

/**< 2nd bit of SPI3 WDT Registers */
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__POS      (2)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__LEN      (1)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__MSK      (0x04)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__REG      \
SMI130_GYRO_BGW_SPI3_WDT_ADDR

/**< 1st bit of SPI3 WDT Registers */
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__POS     (1)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__LEN     (1)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__MSK     (0x02)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__REG     \
SMI130_GYRO_BGW_SPI3_WDT_ADDR

/**< 0th bit of SPI3 WDT Registers */
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__POS            (0)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__LEN            (1)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__MSK            (0x01)
#define SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__REG            \
SMI130_GYRO_BGW_SPI3_WDT_ADDR

/**< 4th bit of Self test Registers */
#define SMI130_GYRO_SELF_TEST_ADDR_RATEOK__POS            (4)
#define SMI130_GYRO_SELF_TEST_ADDR_RATEOK__LEN            (1)
#define SMI130_GYRO_SELF_TEST_ADDR_RATEOK__MSK            (0x10)
#define SMI130_GYRO_SELF_TEST_ADDR_RATEOK__REG            \
SMI130_GYRO_SELF_TEST_ADDR

/**< 2nd bit of Self test Registers */
#define SMI130_GYRO_SELF_TEST_ADDR_BISTFAIL__POS          (2)
#define SMI130_GYRO_SELF_TEST_ADDR_BISTFAIL__LEN          (1)
#define SMI130_GYRO_SELF_TEST_ADDR_BISTFAIL__MSK          (0x04)
#define SMI130_GYRO_SELF_TEST_ADDR_BISTFAIL__REG          \
SMI130_GYRO_SELF_TEST_ADDR

/**< 1st bit of Self test Registers */
#define SMI130_GYRO_SELF_TEST_ADDR_BISTRDY__POS           (1)
#define SMI130_GYRO_SELF_TEST_ADDR_BISTRDY__LEN           (1)
#define SMI130_GYRO_SELF_TEST_ADDR_BISTRDY__MSK           (0x02)
#define SMI130_GYRO_SELF_TEST_ADDR_BISTRDY__REG           \
SMI130_GYRO_SELF_TEST_ADDR

/**< 0th bit of Self test Registers */
#define SMI130_GYRO_SELF_TEST_ADDR_TRIGBIST__POS          (0)
#define SMI130_GYRO_SELF_TEST_ADDR_TRIGBIST__LEN          (1)
#define SMI130_GYRO_SELF_TEST_ADDR_TRIGBIST__MSK          (0x01)
#define SMI130_GYRO_SELF_TEST_ADDR_TRIGBIST__REG          \
SMI130_GYRO_SELF_TEST_ADDR

/* For Axis Selection   */
/**< It refers SMI130_GYRO X-axis */
#define SMI130_GYRO_X_AXIS           (0)
/**< It refers SMI130_GYRO Y-axis */
#define SMI130_GYRO_Y_AXIS           (1)
/**< It refers SMI130_GYRO Z-axis */
#define SMI130_GYRO_Z_AXIS           (2)

/* For Mode Settings    */
#define SMI130_GYRO_MODE_NORMAL              (0)
#define SMI130_GYRO_MODE_DEEPSUSPEND         (1)

/* get bit slice  */
#define SMI130_GYRO_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define SMI130_GYRO_SET_BITSLICE(regvar, bitname, val)\
((regvar&~bitname##__MSK)|((val<<bitname##__POS)&bitname##__MSK))
/* Constants */

#define SMI130_GYRO_NULL                             (0)
/**< constant declaration of NULL */
#define SMI130_GYRO_DISABLE                          (0)
/**< It refers SMI130_GYRO disable */
#define SMI130_GYRO_ENABLE                           (1)
/**< It refers SMI130_GYRO enable */
#define SMI130_GYRO_OFF                              (0)
/**< It refers SMI130_GYRO OFF state */
#define SMI130_GYRO_ON                               (1)
/**< It refers SMI130_GYRO ON state  */


#define SMI130_GYRO_TURN1                            (0)
/**< It refers SMI130_GYRO TURN1 */
#define SMI130_GYRO_TURN2                            (1)
/**< It refers SMI130_GYRO TURN2 */

#define SMI130_GYRO_INT1                             (0)
/**< It refers SMI130_GYRO INT1 */
#define SMI130_GYRO_INT2                             (1)
/**< It refers SMI130_GYRO INT2 */

#define SMI130_GYRO_SLOW_OFFSET                      (0)
/**< It refers SMI130_GYRO Slow Offset */
#define SMI130_GYRO_AUTO_OFFSET                      (1)
/**< It refers SMI130_GYRO Auto Offset */
#define SMI130_GYRO_FAST_OFFSET                      (2)
/**< It refers SMI130_GYRO Fast Offset */
#define SMI130_GYRO_S_TAP                            (0)
/**< It refers SMI130_GYRO Single Tap */
#define SMI130_GYRO_D_TAP                            (1)
/**< It refers SMI130_GYRO Double Tap */
#define SMI130_GYRO_INT1_DATA                        (0)
/**< It refers SMI130_GYRO Int1 Data */
#define SMI130_GYRO_INT2_DATA                        (1)
/**< It refers SMI130_GYRO Int2 Data */
#define SMI130_GYRO_TAP_UNFILT_DATA                  (0)
/**< It refers SMI130_GYRO Tap unfilt data */
#define SMI130_GYRO_HIGH_UNFILT_DATA                 (1)
/**< It refers SMI130_GYRO High unfilt data */
#define SMI130_GYRO_CONST_UNFILT_DATA                (2)
/**< It refers SMI130_GYRO Const unfilt data */
#define SMI130_GYRO_ANY_UNFILT_DATA                  (3)
/**< It refers SMI130_GYRO Any unfilt data */
#define SMI130_GYRO_SHAKE_UNFILT_DATA                (4)
/**< It refers SMI130_GYRO Shake unfilt data */
#define SMI130_GYRO_SHAKE_TH                         (0)
/**< It refers SMI130_GYRO Shake Threshold */
#define SMI130_GYRO_SHAKE_TH2                        (1)
/**< It refers SMI130_GYRO Shake Threshold2 */
#define SMI130_GYRO_AUTO_OFFSET_WL                   (0)
/**< It refers SMI130_GYRO Auto Offset word length */
#define SMI130_GYRO_FAST_OFFSET_WL                   (1)
/**< It refers SMI130_GYRO Fast Offset word length */
#define SMI130_GYRO_I2C_WDT_EN                       (0)
/**< It refers SMI130_GYRO I2C WDT En */
#define SMI130_GYRO_I2C_WDT_SEL                      (1)
/**< It refers SMI130_GYRO I2C WDT Sel */
#define SMI130_GYRO_EXT_MODE                         (0)
/**< It refers SMI130_GYRO Ext Mode */
#define SMI130_GYRO_EXT_PAGE                         (1)
/**< It refers SMI130_GYRO Ext page */
#define SMI130_GYRO_START_ADDR                       (0)
/**< It refers SMI130_GYRO Start Address */
#define SMI130_GYRO_STOP_ADDR                        (1)
/**< It refers SMI130_GYRO Stop Address */
#define SMI130_GYRO_SLOW_CMD                         (0)
/**< It refers SMI130_GYRO Slow Command */
#define SMI130_GYRO_FAST_CMD                         (1)
/**< It refers SMI130_GYRO Fast Command */
#define SMI130_GYRO_TRIM_VRA                         (0)
/**< It refers SMI130_GYRO Trim VRA */
#define SMI130_GYRO_TRIM_VRD                         (1)
/**< It refers SMI130_GYRO Trim VRD */
#define SMI130_GYRO_LOGBIT_EM                        (0)
/**< It refers SMI130_GYRO LogBit Em */
#define SMI130_GYRO_LOGBIT_VM                        (1)
/**< It refers SMI130_GYRO LogBit VM */
#define SMI130_GYRO_GP0                              (0)
/**< It refers SMI130_GYRO GP0 */
#define SMI130_GYRO_GP1                              (1)
/**< It refers SMI130_GYRO GP1*/
#define SMI130_GYRO_LOW_SPEED                        (0)
/**< It refers SMI130_GYRO Low Speed Oscillator */
#define SMI130_GYRO_HIGH_SPEED                       (1)
/**< It refers SMI130_GYRO High Speed Oscillator */
#define SMI130_GYRO_DRIVE_OFFSET_P                   (0)
/**< It refers SMI130_GYRO Drive Offset P */
#define SMI130_GYRO_DRIVE_OFFSET_N                   (1)
/**< It refers SMI130_GYRO Drive Offset N */
#define SMI130_GYRO_TEST_MODE_EN                     (0)
/**< It refers SMI130_GYRO Test Mode Enable */
#define SMI130_GYRO_TEST_MODE_REG                    (1)
/**< It refers SMI130_GYRO Test Mode reg */
#define SMI130_GYRO_IBIAS_DRIVE_TRIM                 (0)
/**< It refers SMI130_GYRO IBIAS Drive Trim */
#define SMI130_GYRO_IBIAS_RATE_TRIM                  (1)
/**< It refers SMI130_GYRO IBIAS Rate Trim */
#define SMI130_GYRO_BAA_MODE                         (0)
/**< It refers SMI130_GYRO BAA Mode Trim */
#define SMI130_GYRO_SMI_ACC_MODE                     (1)
/**< It refers SMI130_GYRO SMI_ACC Mode Trim */
#define SMI130_GYRO_PI_KP                            (0)
/**< It refers SMI130_GYRO PI KP */
#define SMI130_GYRO_PI_KI                            (1)
/**< It refers SMI130_GYRO PI KI */


#define C_SMI130_GYRO_SUCCESS						(0)
/**< It refers SMI130_GYRO operation is success */
#define C_SMI130_GYRO_FAILURE						(1)
/**< It refers SMI130_GYRO operation is Failure */

#define SMI130_GYRO_SPI_RD_MASK                     (0x80)
/**< Read mask **/
#define SMI130_GYRO_READ_SET                        (0x01)
/**< Setting for rading data **/

#define SMI130_GYRO_SHIFT_1_POSITION                 (1)
/**< Shift bit by 1 Position **/
#define SMI130_GYRO_SHIFT_2_POSITION                 (2)
/**< Shift bit by 2 Position **/
#define SMI130_GYRO_SHIFT_3_POSITION                 (3)
/**< Shift bit by 3 Position **/
#define SMI130_GYRO_SHIFT_4_POSITION                 (4)
/**< Shift bit by 4 Position **/
#define SMI130_GYRO_SHIFT_5_POSITION                 (5)
/**< Shift bit by 5 Position **/
#define SMI130_GYRO_SHIFT_6_POSITION                 (6)
/**< Shift bit by 6 Position **/
#define SMI130_GYRO_SHIFT_7_POSITION                 (7)
/**< Shift bit by 7 Position **/
#define SMI130_GYRO_SHIFT_8_POSITION                 (8)
/**< Shift bit by 8 Position **/
#define SMI130_GYRO_SHIFT_12_POSITION                (12)
/**< Shift bit by 12 Position **/

#define C_SMI130_GYRO_Null_U8X                              (0)
#define C_SMI130_GYRO_Zero_U8X                              (0)
#define C_SMI130_GYRO_One_U8X                               (1)
#define C_SMI130_GYRO_Two_U8X                               (2)
#define C_SMI130_GYRO_Three_U8X                             (3)
#define C_SMI130_GYRO_Four_U8X                              (4)
#define C_SMI130_GYRO_Five_U8X                              (5)
#define C_SMI130_GYRO_Six_U8X                               (6)
#define C_SMI130_GYRO_Seven_U8X                             (7)
#define C_SMI130_GYRO_Eight_U8X                             (8)
#define C_SMI130_GYRO_Nine_U8X                              (9)
#define C_SMI130_GYRO_Ten_U8X                               (10)
#define C_SMI130_GYRO_Eleven_U8X                            (11)
#define C_SMI130_GYRO_Twelve_U8X                            (12)
#define C_SMI130_GYRO_Thirteen_U8X                          (13)
#define C_SMI130_GYRO_Fifteen_U8X                           (15)
#define C_SMI130_GYRO_Sixteen_U8X                           (16)
#define C_SMI130_GYRO_TwentyTwo_U8X                         (22)
#define C_SMI130_GYRO_TwentyThree_U8X                       (23)
#define C_SMI130_GYRO_TwentyFour_U8X                        (24)
#define C_SMI130_GYRO_TwentyFive_U8X                        (25)
#define C_SMI130_GYRO_ThirtyTwo_U8X                         (32)
#define C_SMI130_GYRO_Hundred_U8X                           (100)
#define C_SMI130_GYRO_OneTwentySeven_U8X                    (127)
#define C_SMI130_GYRO_OneTwentyEight_U8X                    (128)
#define C_SMI130_GYRO_TwoFiftyFive_U8X                      (255)
#define C_SMI130_GYRO_TwoFiftySix_U16X                      (256)

#define E_SMI130_GYRO_NULL_PTR                              (int8_t)(-127)
#define E_SMI130_GYRO_COMM_RES                              (int8_t)(-1)
#define E_SMI130_GYRO_OUT_OF_RANGE                          (int8_t)(-2)

#define	C_SMI130_GYRO_No_Filter_U8X			(0)
#define	C_SMI130_GYRO_BW_230Hz_U8X			(1)
#define	C_SMI130_GYRO_BW_116Hz_U8X			(1)
#define	C_SMI130_GYRO_BW_47Hz_U8X			(3)
#define	C_SMI130_GYRO_BW_23Hz_U8X			(4)
#define	C_SMI130_GYRO_BW_12Hz_U8X			(5)
#define	C_SMI130_GYRO_BW_64Hz_U8X			(6)
#define	C_SMI130_GYRO_BW_32Hz_U8X			(7)

#define C_SMI130_GYRO_No_Autosleepdur_U8X	(0)
#define	C_SMI130_GYRO_4ms_Autosleepdur_U8X	(1)
#define	C_SMI130_GYRO_5ms_Autosleepdur_U8X	(1)
#define	C_SMI130_GYRO_8ms_Autosleepdur_U8X	(3)
#define	C_SMI130_GYRO_10ms_Autosleepdur_U8X	(4)
#define	C_SMI130_GYRO_15ms_Autosleepdur_U8X	(5)
#define	C_SMI130_GYRO_20ms_Autosleepdur_U8X	(6)
#define	C_SMI130_GYRO_40ms_Autosleepdur_U8X	(7)

#define C_SMI130_GYRO_RANGE_2000            (0)
#define C_SMI130_GYRO_RANGE_1000            (1)
#define C_SMI130_GYRO_RANGE_500             (1)
#define C_SMI130_GYRO_RANGE_250             (3)
#define C_SMI130_GYRO_RANGE_125             (4)

#define SMI130_GYRO_MDELAY_DATA_TYPE SMI130_GYRO_U16

#define SMI130_GYRO_WR_FUNC_PTR int8_t (*bus_write)\
(uint8_t, uint8_t, uint8_t*, uint8_t)
#define SMI130_GYRO_RD_FUNC_PTR int8_t (*bus_read)\
(uint8_t, uint8_t, uint8_t*, uint8_t)
#define SMI130_GYRO_MDELAY_DATA_TYPE SMI130_GYRO_U16


/*user defined Structures*/
struct smi130_gyro_data_t {
	SMI130_GYRO_S16 datax;
	SMI130_GYRO_S16 datay;
	SMI130_GYRO_S16 dataz;
	int8_t intstatus[5];
};


struct smi130_gyro_offset_t {
	SMI130_GYRO_U16 datax;
	SMI130_GYRO_U16 datay;
	SMI130_GYRO_U16 dataz;
};

struct smi130_gyro_t {
	uint8_t chip_id;
	uint8_t dev_addr;
	SMI130_GYRO_WR_FUNC_PTR;
	SMI130_GYRO_RD_FUNC_PTR;
	void(*delay_msec)(SMI130_GYRO_MDELAY_DATA_TYPE);
};

/***************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *
 *
 *  \return
 *
 *
 ***************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_init(struct smi130_gyro_t *p_smi130_gyro);
/***************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *
 *
 *  \return
 *
 *
 ***************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataX(SMI130_GYRO_S16 *data_x);
/****************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataY(SMI130_GYRO_S16 *data_y);
/***************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ***************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataZ(SMI130_GYRO_S16 *data_z);
/************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 *************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataXYZ(struct smi130_gyro_data_t *data);
/***************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ********************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataXYZI(struct smi130_gyro_data_t *data);
/********************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ********************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_Temperature(uint8_t *temperature);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_read_register(uint8_t addr,
                uint8_t *data, uint8_t len);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_burst_read(uint8_t addr,
                uint8_t *data, SMI130_GYRO_S32 len);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_write_register(uint8_t addr,
                uint8_t *data, uint8_t len);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_range_reg
(uint8_t *range);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_range_reg
(uint8_t range);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_bw(uint8_t *bandwidth);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_bw(uint8_t bandwidth);
/****************************************************************************
 * Description: *//**\brief
 *
 *

 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_pmu_ext_tri_sel
(uint8_t *pwu_ext_tri_sel);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_pmu_ext_tri_sel
(uint8_t pwu_ext_tri_sel);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_high_bw
(uint8_t *high_bw);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_high_bw
(uint8_t high_bw);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_shadow_dis
(uint8_t *shadow_dis);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_shadow_dis
(uint8_t shadow_dis);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_soft_reset(void);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_data_enable(uint8_t *data_en);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_data_en(uint8_t data_en);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_int_od
(uint8_t param, uint8_t *int_od);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_int_od
(uint8_t param, uint8_t int_od);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_int_lvl
(uint8_t param, uint8_t *int_lvl);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_int_lvl
(uint8_t param, uint8_t int_lvl);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_int_data
(uint8_t axis, uint8_t *int_data);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_int_data
(uint8_t axis, uint8_t int_data);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_i2c_wdt
(uint8_t i2c_wdt, uint8_t *prog_mode);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_i2c_wdt
(uint8_t i2c_wdt, uint8_t prog_mode);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_spi3(uint8_t *spi3);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_spi3(uint8_t spi3);
/****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_mode(uint8_t *mode);
/*****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_mode(uint8_t mode);
/*****************************************************************************
 * Description: *//**\brief
 *
 *
 *
 *
 *  \param
 *
 *
 *  \return
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_selftest(uint8_t *result);

#endif
