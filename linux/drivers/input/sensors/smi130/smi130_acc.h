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
/*! \file smi130_acc.h
    \brief smi130_acc Sensor Driver Support Header File */
#ifndef _SMI130_ACC_H
#define _SMI130_ACC_H

#include "smi130_defs.h"

/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
/*!
* @brief Define the calling convention of YOUR bus communication routine.
* @note This includes types of parameters.
* This example shows the configuration for an SPI bus link.

* If your communication function looks like this:

* write_my_bus_xy(uint8_t device_addr,
* uint8_t register_addr, uint8_t * data, uint8_t length);

* The SMI130_ACC_WR_FUNC_PTR would equal:

* SMI130_ACC_WR_FUNC_PTR int8_t
* (* bus_write)(uint8_t, uint8_t, uint8_t *, uint8_t)

* Parameters can be mixed as needed refer to
* the \ref SMI130_ACC_BUS_WRITE_FUNC  macro.
*/
#define SMI130_ACC_WR_FUNC_PTR int8_t (*bus_write)\
(uint8_t, uint8_t, uint8_t *, uint8_t)
/*!
* @brief link macro between API function calls and bus write function
* @note The bus write function can change since
* this is a system dependant issue.

* If the bus_write parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

* If the parameters are differently ordered or your communication function
* like I2C need to know the device address,
* you can change this macro accordingly.


* define SMI130_ACC_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_write(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR communication routine in
* a way that equals your definition in the
* ref SMI130_ACC_WR_FUNC_PTR definition.

*/
#define SMI130_ACC_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
bus_write(dev_addr, reg_addr, reg_data, wr_len)


/*!
* @brief Define the calling convention of YOUR bus communication routine.
* @note This includes types of parameters. This example
*shows the configuration for an SPI bus link.

*If your communication function looks like this:

*read_my_bus_xy(uint8_t device_addr,
*uint8_t register_addr, uint8_t* data, uint8_t length);

*The SMI130_ACC_RD_FUNC_PTR would equal:

*SMI130_ACC_RD_FUNC_PTR int8_t
*(* bus_read)(uint8_t, uint8_t, uint8_t*, uint8_t)

*Parameters can be mixed as needed refer to the
 ref SMI130_ACC_BUS_READ_FUNC  macro.
*/

#define SMI130_ACC_SPI_RD_MASK (0x80)
/* for spi read transactions on SPI the MSB has to be set */
#define SMI130_ACC_RD_FUNC_PTR int8_t (*bus_read)\
(uint8_t, uint8_t, uint8_t *, uint8_t)
#define SMI130_ACC_BRD_FUNC_PTR int8_t(*burst_read)\
(uint8_t, uint8_t, uint8_t *, unsigned short)

/*!
* @brief link macro between API function calls and bus read function
* @note The bus write function can change since
* this is a system dependant issue.

* If the bus_read parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

*  If the parameters are differently ordered or your
*  communication function like I2C need to know the device address,
*  you can change this macro accordingly.


*  define SMI130_ACC_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_read(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR
* communication routine in a way that equals your definition in the
* ref SMI130_ACC_WR_FUNC_PTR definition.

* @note: this macro also includes the "MSB='1'" for reading SMI130_ACC addresses.
*/



#define SMI130_ACC_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)
#define SMI130_ACC_BURST_READ_FUNC(device_addr,\
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)
/**************************************************************/
/**\name	I2C ADDRESS DEFINITIONS    */
/**************************************************************/
/**< The following definition of I2C address is used for the following sensors
* SMI130_ACC
*/
#define SMI130_ACC_I2C_ADDR1                		(0x18)
#define SMI130_ACC_I2C_ADDR2                		(0x19)
/**************************************************************/
/**\name	SMI130 ACC NAME DEFINITIONS    */
/**************************************************************/
#define SMI130_ACC_NAME  "smi130_acc"

/**************************************************************/
/**\name	CONSTANTS DEFINITION    */
/**************************************************************/
#define	SMI130_ACC_INIT_VALUE                           ((uint8_t)0)
#define	SMI130_ACC_GEN_READ_WRITE_LENGTH                ((uint8_t)1)
#define	SMI130_ACC_INTERFACE_IDLE_TIME_DELAY		((uint8_t)1)
#define	SMI130_ACC_LSB_MSB_READ_LENGTH			((uint8_t)2)
/**	BIT SHIFT DEFINITIONS    */
#define	SMI130_ACC_SHIFT_TWO_BITS                       ((uint8_t)2)
#define	SMI130_ACC_SHIFT_FOUR_BITS                      ((uint8_t)4)
#define	SMI130_ACC_SHIFT_FIVE_BITS                      ((uint8_t)5)
#define	SMI130_ACC_SHIFT_SIX_BITS                       ((uint8_t)6)
#define	SMI130_ACC_SHIFT_EIGHT_BITS                     ((uint8_t)8)
/**	MODE RANGES    */
#define	SMI130_ACC_BW_MIN_RANGE               		((uint8_t)7)
#define	SMI130_ACC_BW_1000HZ_RANGE            		((uint8_t)15)
#define	SMI130_ACC_BW_MAX_RANGE               		((uint8_t)16)
#define	SMI130_ACC_SLEEP_DURN_MIN_RANGE			((uint8_t)4)
#define	SMI130_ACC_SLEEP_TIMER_MODE_RANGE		((uint8_t)2)
#define	SMI130_ACC_SLEEP_DURN_MAX_RANGE			((uint8_t)16)
#define	SMI130_ACC_POWER_MODE_RANGE			((uint8_t)6)
#define	SMI130_ACC_SELF_TEST_AXIS_RANGE			((uint8_t)4)
#define	SMI130_ACC_SELF_TEST_SIGN_RANGE			((uint8_t)2)
#define	SMI130_ACC_SELF_TEST_AMP_RANGE			((uint8_t)2)


/**************************************************************/
/**\name	ERROR CODE DEFINITIONS    */
/**************************************************************/
#define E_OUT_OF_RANGE          			((int8_t)-2)
#define E_SMI130_ACC_NULL_PTR       			((int8_t)-127)
#define SMI130_ACC_NULL             			((void *)0)
#define ERROR						((int8_t)-1)
#define	SUCCESS						((uint8_t)0)
/**************************************************************/
/**\name	RETURN TYPE DEFINITION    */
/**************************************************************/
#define	SMI130_ACC_RETURN_FUNCTION_TYPE                 int8_t
/**< This refers SMI130_ACC return type as int8_t */

/**************************************************************/
/**\name	REGISTER ADDRESS DEFINITIONS    */
/**************************************************************/
#define SMI130_ACC_EEP_OFFSET                           (0x16)
#define SMI130_ACC_IMAGE_BASE                           (0x38)
#define SMI130_ACC_IMAGE_LEN                            (22)
#define SMI130_ACC_CHIP_ID_ADDR				(0x00)
/** DATA ADDRESS DEFINITIONS */
#define SMI130_ACC_X_AXIS_LSB_ADDR                      (0x02)
#define SMI130_ACC_X_AXIS_MSB_ADDR                      (0x03)
#define SMI130_ACC_Y_AXIS_LSB_ADDR                      (0x04)
#define SMI130_ACC_Y_AXIS_MSB_ADDR                      (0x05)
#define SMI130_ACC_Z_AXIS_LSB_ADDR                      (0x06)
#define SMI130_ACC_Z_AXIS_MSB_ADDR                      (0x07)
#define SMI130_ACC_TEMP_ADDR				(0x08)
/**STATUS ADDRESS DEFINITIONS */
#define SMI130_ACC_STAT1_ADDR				(0x09)
#define SMI130_ACC_STAT_SLOPE_ADDR			(0x0B)
/**STATUS ADDRESS DEFINITIONS */
#define SMI130_ACC_RANGE_SELECT_ADDR			(0x0F)
#define SMI130_ACC_BW_SELECT_ADDR                       (0x10)
#define SMI130_ACC_MODE_CTRL_ADDR                       (0x11)
#define SMI130_ACC_LOW_NOISE_CTRL_ADDR                  (0x12)
#define SMI130_ACC_DATA_CTRL_ADDR                       (0x13)
#define SMI130_ACC_RST_ADDR                             (0x14)
/**INTERUPT ADDRESS DEFINITIONS */
#define SMI130_ACC_INTR_ENABLE1_ADDR                    (0x16)
#define SMI130_ACC_INTR_ENABLE2_ADDR                    (0x17)
#define SMI130_ACC_INTR_DATA_SELECT_ADDR                (0x1A)
#define SMI130_ACC_INTR2_PAD_SELECT_ADDR                (0x1B)
#define SMI130_ACC_INTR_SOURCE_ADDR                     (0x1E)
#define SMI130_ACC_INTR_SET_ADDR                        (0x20)
#define SMI130_ACC_INTR_CTRL_ADDR                       (0x21)
/** FEATURE ADDRESS DEFINITIONS */
#define SMI130_ACC_SLOPE_DURN_ADDR                      (0x27)
#define SMI130_ACC_SLOPE_THRES_ADDR                     (0x28)
#define SMI130_ACC_SELFTEST_ADDR                        (0x32)
#define SMI130_ACC_SERIAL_CTRL_ADDR                     (0x34)
/**OFFSET ADDRESS DEFINITIONS */
#define SMI130_ACC_OFFSET_CTRL_ADDR                     (0x36)
#define SMI130_ACC_OFFSET_PARAMS_ADDR                   (0x37)
#define SMI130_ACC_OFFSET_X_AXIS_ADDR                   (0x38)
#define SMI130_ACC_OFFSET_Y_AXIS_ADDR                   (0x39)
#define SMI130_ACC_OFFSET_Z_AXIS_ADDR                   (0x3A)

/**************************************************************/
/**\name	ACCEL RESOLUTION DEFINITION   */
/**************************************************************/
#define SMI130_ACC_12_RESOLUTION                        (0)

/**************************************************************/
/**\name	ACCEL DELAY DEFINITION   */
/**************************************************************/
/* register write and read delays */
#define SMI130_ACC_MDELAY_DATA_TYPE                     uint32_t
#define SMI130_ACC_EE_W_DELAY                           (28)

/**************************************************************/
/**\name	STRUCTURE DEFINITIONS    */
/**************************************************************/
/*!
*	@brief read accel xyz data for 10,14 and 12 bit resolution
*/
struct smi130_acc_data {
	int16_t x,/**< accel x data 10,14 and 12 resolution*/
	        y,/**< accel y data 10,14 and 12 resolution*/
	        z;/**< accel z data 10,14 and 12 resolution*/
};
/*!
*	@brief read accel xyz data for 10,14 and 12 bit resolution
*	and temperature output
*/
struct smi130_acc_data_temp {
	int16_t x,/**< accel x data 10,14 and 12 resolution*/
	        y,/**< accel y data 10,14 and 12 resolution*/
	        z;/**< accel z data 10,14 and 12 resolution*/
	int8_t temp;/**< accel temperature data*/
};

/*!
 *	@brief smi130_acc initialization struct
 *	struct smi130_acc_t is used for assigning the following parameters.
 *
 *	Bus write function pointer: SMI130_ACC_WR_FUNC_PTR
 *	Bus read function pointer: SMI130_ACC_RD_FUNC_PTR
 *	Burst read function pointer: SMI130_ACC_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 */
struct smi130_acc_t {
	/*! save current smi130_acc operation mode */
	uint8_t power_mode;
	/*! chip_id of smi130_acc */
	uint8_t chip_id;
	/*! the value of power mode register 0x11*/
	uint8_t ctrl_mode_reg;
	/*! the value of power mode register 0x12*/
	uint8_t low_mode_reg;
	/*! initializes smi130_acc's I2C device address*/
	uint8_t dev_addr;
	/*! function pointer to the SPI/I2C write function */
	SMI130_ACC_WR_FUNC_PTR;
	/*! function pointer to the SPI/I2C read function */
	SMI130_ACC_RD_FUNC_PTR;
	/*! function pointer to the SPI/I2C burst read function */
	SMI130_ACC_BRD_FUNC_PTR;
	/*! delay(in ms) function pointer */
	void (*delay_msec)(SMI130_ACC_MDELAY_DATA_TYPE);
};

/*********************************************************************/
/**\name REGISTER BIT MASK, BIT LENGTH, BIT POSITION DEFINITIONS  */
/********************************************************************/
/******************************/
/**\name CHIP ID  */
/******************************/
#define SMI130_ACC_CHIP_ID_POS             (0)
#define SMI130_ACC_CHIP_ID_MSK             (0xFF)
#define SMI130_ACC_CHIP_ID_LEN             (8)
#define SMI130_ACC_CHIP_ID_REG             SMI130_ACC_CHIP_ID_ADDR

/******************************/
/**\name DATA REGISTER-X  */
/******************************/
#define SMI130_ACC_NEW_DATA_X_POS          (0)
#define SMI130_ACC_NEW_DATA_X_LEN          (1)
#define SMI130_ACC_NEW_DATA_X_MSK          (0x01)
#define SMI130_ACC_NEW_DATA_X_REG          SMI130_ACC_X_AXIS_LSB_ADDR

#define SMI130_ACC_X14_LSB_POS          (2)
#define SMI130_ACC_X14_LSB_LEN          (6)
#define SMI130_ACC_X14_LSB_MSK          (0xFC)
#define SMI130_ACC_X14_LSB_REG           SMI130_ACC_X_AXIS_LSB_ADDR

#define SMI130_ACC_X12_LSB_POS           (4)
#define SMI130_ACC_X12_LSB_LEN           (4)
#define SMI130_ACC_X12_LSB_MSK           (0xF0)
#define SMI130_ACC_X12_LSB_REG           SMI130_ACC_X_AXIS_LSB_ADDR

#define SMI130_ACC_X10_LSB_POS           (6)
#define SMI130_ACC_X10_LSB_LEN           (2)
#define SMI130_ACC_X10_LSB_MSK           (0xC0)
#define SMI130_ACC_X10_LSB_REG           SMI130_ACC_X_AXIS_LSB_ADDR

#define SMI130_ACC_X8_LSB_POS           (0)
#define SMI130_ACC_X8_LSB_LEN           (0)
#define SMI130_ACC_X8_LSB_MSK           (0x00)
#define SMI130_ACC_X8_LSB_REG           SMI130_ACC_X_AXIS_LSB_ADDR

#define SMI130_ACC_X_MSB_POS           (0)
#define SMI130_ACC_X_MSB_LEN           (8)
#define SMI130_ACC_X_MSB_MSK           (0xFF)
#define SMI130_ACC_X_MSB_REG           SMI130_ACC_X_AXIS_MSB_ADDR
/******************************/
/**\name DATA REGISTER-Y  */
/******************************/
#define SMI130_ACC_NEW_DATA_Y_POS          (0)
#define SMI130_ACC_NEW_DATA_Y_LEN          (1)
#define SMI130_ACC_NEW_DATA_Y_MSK          (0x01)
#define SMI130_ACC_NEW_DATA_Y_REG          SMI130_ACC_Y_AXIS_LSB_ADDR

#define SMI130_ACC_Y14_LSB_POS           (2)
#define SMI130_ACC_Y14_LSB_LEN           (6)
#define SMI130_ACC_Y14_LSB_MSK           (0xFC)
#define SMI130_ACC_Y14_LSB_REG           SMI130_ACC_Y_AXIS_LSB_ADDR

#define SMI130_ACC_Y12_LSB_POS           (4)
#define SMI130_ACC_Y12_LSB_LEN           (4)
#define SMI130_ACC_Y12_LSB_MSK           (0xF0)
#define SMI130_ACC_Y12_LSB_REG           SMI130_ACC_Y_AXIS_LSB_ADDR

#define SMI130_ACC_Y10_LSB_POS           (6)
#define SMI130_ACC_Y10_LSB_LEN           (2)
#define SMI130_ACC_Y10_LSB_MSK           (0xC0)
#define SMI130_ACC_Y10_LSB_REG           SMI130_ACC_Y_AXIS_LSB_ADDR

#define SMI130_ACC_Y8_LSB_POS           (0)
#define SMI130_ACC_Y8_LSB_LEN           (0)
#define SMI130_ACC_Y8_LSB_MSK           (0x00)
#define SMI130_ACC_Y8_LSB_REG           SMI130_ACC_Y_AXIS_LSB_ADDR

#define SMI130_ACC_Y_MSB_POS           (0)
#define SMI130_ACC_Y_MSB_LEN           (8)
#define SMI130_ACC_Y_MSB_MSK           (0xFF)
#define SMI130_ACC_Y_MSB_REG           SMI130_ACC_Y_AXIS_MSB_ADDR
/******************************/
/**\name DATA REGISTER-Z  */
/******************************/
#define SMI130_ACC_NEW_DATA_Z_POS          (0)
#define SMI130_ACC_NEW_DATA_Z_LEN          (1)
#define SMI130_ACC_NEW_DATA_Z_MSK          (0x01)
#define SMI130_ACC_NEW_DATA_Z_REG          SMI130_ACC_Z_AXIS_LSB_ADDR

#define SMI130_ACC_Z14_LSB_POS           (2)
#define SMI130_ACC_Z14_LSB_LEN           (6)
#define SMI130_ACC_Z14_LSB_MSK           (0xFC)
#define SMI130_ACC_Z14_LSB_REG           SMI130_ACC_Z_AXIS_LSB_ADDR

#define SMI130_ACC_Z12_LSB_POS           (4)
#define SMI130_ACC_Z12_LSB_LEN           (4)
#define SMI130_ACC_Z12_LSB_MSK           (0xF0)
#define SMI130_ACC_Z12_LSB_REG           SMI130_ACC_Z_AXIS_LSB_ADDR

#define SMI130_ACC_Z10_LSB_POS           (6)
#define SMI130_ACC_Z10_LSB_LEN           (2)
#define SMI130_ACC_Z10_LSB_MSK           (0xC0)
#define SMI130_ACC_Z10_LSB_REG           SMI130_ACC_Z_AXIS_LSB_ADDR

#define SMI130_ACC_Z8_LSB_POS           (0)
#define SMI130_ACC_Z8_LSB_LEN           (0)
#define SMI130_ACC_Z8_LSB_MSK           (0x00)
#define SMI130_ACC_Z8_LSB_REG           SMI130_ACC_Z_AXIS_LSB_ADDR

#define SMI130_ACC_Z_MSB_POS           (0)
#define SMI130_ACC_Z_MSB_LEN           (8)
#define SMI130_ACC_Z_MSB_MSK           (0xFF)
#define SMI130_ACC_Z_MSB_REG           SMI130_ACC_Z_AXIS_MSB_ADDR

/******************************/
/**\name TEMPERATURE */
/******************************/
#define SMI130_ACC_TEMP_MSB_POS           (0)
#define SMI130_ACC_TEMP_MSB_LEN           (8)
#define SMI130_ACC_TEMP_MSB_MSK           (0xFF)
#define SMI130_ACC_TEMP_MSB_REG           SMI130_ACC_TEMP_ADDR

/***************************************/
/**\name INTERRUPT STATUS OF LOW-G */
/**************************************/
#define SMI130_ACC_LOW_G_INTR_STAT_POS          (0)
#define SMI130_ACC_LOW_G_INTR_STAT_LEN          (1)
#define SMI130_ACC_LOW_G_INTR_STAT_MSK          (0x01)
#define SMI130_ACC_LOW_G_INTR_STAT_REG          SMI130_ACC_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF HIGH-G */
/**************************************/
#define SMI130_ACC_HIGH_G_INTR_STAT_POS          (1)
#define SMI130_ACC_HIGH_G_INTR_STAT_LEN          (1)
#define SMI130_ACC_HIGH_G_INTR_STAT_MSK          (0x02)
#define SMI130_ACC_HIGH_G_INTR_STAT_REG          SMI130_ACC_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF SLOPE */
/**************************************/
#define SMI130_ACC_SLOPE_INTR_STAT_POS          (2)
#define SMI130_ACC_SLOPE_INTR_STAT_LEN          (1)
#define SMI130_ACC_SLOPE_INTR_STAT_MSK          (0x04)
#define SMI130_ACC_SLOPE_INTR_STAT_REG          SMI130_ACC_STAT1_ADDR
/*******************************************/
/**\name INTERRUPT STATUS OF SLOW NO MOTION*/
/*******************************************/
#define SMI130_ACC_SLOW_NO_MOTION_INTR_STAT_POS          (3)
#define SMI130_ACC_SLOW_NO_MOTION_INTR_STAT_LEN          (1)
#define SMI130_ACC_SLOW_NO_MOTION_INTR_STAT_MSK          (0x08)
#define SMI130_ACC_SLOW_NO_MOTION_INTR_STAT_REG          SMI130_ACC_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF DOUBLE TAP */
/**************************************/
#define SMI130_ACC_DOUBLE_TAP_INTR_STAT_POS     (4)
#define SMI130_ACC_DOUBLE_TAP_INTR_STAT_LEN     (1)
#define SMI130_ACC_DOUBLE_TAP_INTR_STAT_MSK     (0x10)
#define SMI130_ACC_DOUBLE_TAP_INTR_STAT_REG     SMI130_ACC_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF SINGLE TAP */
/**************************************/
#define SMI130_ACC_SINGLE_TAP_INTR_STAT_POS     (5)
#define SMI130_ACC_SINGLE_TAP_INTR_STAT_LEN     (1)
#define SMI130_ACC_SINGLE_TAP_INTR_STAT_MSK     (0x20)
#define SMI130_ACC_SINGLE_TAP_INTR_STAT_REG     SMI130_ACC_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF ORIENT*/
/**************************************/
#define SMI130_ACC_ORIENT_INTR_STAT_POS         (6)
#define SMI130_ACC_ORIENT_INTR_STAT_LEN         (1)
#define SMI130_ACC_ORIENT_INTR_STAT_MSK         (0x40)
#define SMI130_ACC_ORIENT_INTR_STAT_REG         SMI130_ACC_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF FLAT */
/**************************************/
#define SMI130_ACC_FLAT_INTR_STAT_POS           (7)
#define SMI130_ACC_FLAT_INTR_STAT_LEN           (1)
#define SMI130_ACC_FLAT_INTR_STAT_MSK           (0x80)
#define SMI130_ACC_FLAT_INTR_STAT_REG           SMI130_ACC_STAT1_ADDR
/*********************************************/
/**\name INTERRUPT STATUS SLOPE XYZ AND SIGN */
/*********************************************/
#define SMI130_ACC_SLOPE_FIRST_X_POS        (0)
#define SMI130_ACC_SLOPE_FIRST_X_LEN        (1)
#define SMI130_ACC_SLOPE_FIRST_X_MSK        (0x01)
#define SMI130_ACC_SLOPE_FIRST_X_REG        SMI130_ACC_STAT_SLOPE_ADDR

#define SMI130_ACC_SLOPE_FIRST_Y_POS        (1)
#define SMI130_ACC_SLOPE_FIRST_Y_LEN        (1)
#define SMI130_ACC_SLOPE_FIRST_Y_MSK        (0x02)
#define SMI130_ACC_SLOPE_FIRST_Y_REG        SMI130_ACC_STAT_SLOPE_ADDR

#define SMI130_ACC_SLOPE_FIRST_Z_POS        (2)
#define SMI130_ACC_SLOPE_FIRST_Z_LEN        (1)
#define SMI130_ACC_SLOPE_FIRST_Z_MSK        (0x04)
#define SMI130_ACC_SLOPE_FIRST_Z_REG        SMI130_ACC_STAT_SLOPE_ADDR

#define SMI130_ACC_SLOPE_SIGN_STAT_POS         (3)
#define SMI130_ACC_SLOPE_SIGN_STAT_LEN         (1)
#define SMI130_ACC_SLOPE_SIGN_STAT_MSK         (0x08)
#define SMI130_ACC_SLOPE_SIGN_STAT_REG         SMI130_ACC_STAT_SLOPE_ADDR

/****************************/
/**\name RANGE */
/****************************/
#define SMI130_ACC_RANGE_SELECT_POS             (0)
#define SMI130_ACC_RANGE_SELECT_LEN             (4)
#define SMI130_ACC_RANGE_SELECT_MSK             (0x0F)
#define SMI130_ACC_RANGE_SELECT_REG             SMI130_ACC_RANGE_SELECT_ADDR
/****************************/
/**\name BANDWIDTH */
/****************************/
#define SMI130_ACC_BW_POS             (0)
#define SMI130_ACC_BW_LEN             (5)
#define SMI130_ACC_BW_MSK             (0x1F)
#define SMI130_ACC_BW_REG             SMI130_ACC_BW_SELECT_ADDR
/****************************/
/**\name SLEEP DURATION */
/****************************/
#define SMI130_ACC_SLEEP_DURN_POS             (1)
#define SMI130_ACC_SLEEP_DURN_LEN             (4)
#define SMI130_ACC_SLEEP_DURN_MSK             (0x1E)
#define SMI130_ACC_SLEEP_DURN_REG             SMI130_ACC_MODE_CTRL_ADDR
/****************************/
/**\name POWER MODEPOWER MODE */
/****************************/
#define SMI130_ACC_MODE_CTRL_POS             (5)
#define SMI130_ACC_MODE_CTRL_LEN             (3)
#define SMI130_ACC_MODE_CTRL_MSK             (0xE0)
#define SMI130_ACC_MODE_CTRL_REG             SMI130_ACC_MODE_CTRL_ADDR
/****************************/
/**\name SLEEP TIMER */
/****************************/
#define SMI130_ACC_SLEEP_TIMER_POS          (5)
#define SMI130_ACC_SLEEP_TIMER_LEN          (1)
#define SMI130_ACC_SLEEP_TIMER_MSK          (0x20)
#define SMI130_ACC_SLEEP_TIMER_REG          SMI130_ACC_LOW_NOISE_CTRL_ADDR
/****************************/
/**\name LOWPOWER MODE */
/****************************/
#define SMI130_ACC_LOW_POWER_MODE_POS          (6)
#define SMI130_ACC_LOW_POWER_MODE_LEN          (1)
#define SMI130_ACC_LOW_POWER_MODE_MSK          (0x40)
#define SMI130_ACC_LOW_POWER_MODE_REG          SMI130_ACC_LOW_NOISE_CTRL_ADDR
/*******************************************/
/**\name DISABLE MSB SHADOWING PROCEDURE  */
/*******************************************/
#define SMI130_ACC_DIS_SHADOW_PROC_POS       (6)
#define SMI130_ACC_DIS_SHADOW_PROC_LEN       (1)
#define SMI130_ACC_DIS_SHADOW_PROC_MSK       (0x40)
#define SMI130_ACC_DIS_SHADOW_PROC_REG       SMI130_ACC_DATA_CTRL_ADDR
/***************************************************/
/**\name FILTERED OR UNFILTERED ACCELERATION DATA   */
/***************************************************/
#define SMI130_ACC_ENABLE_DATA_HIGH_BW_POS         (7)
#define SMI130_ACC_ENABLE_DATA_HIGH_BW_LEN         (1)
#define SMI130_ACC_ENABLE_DATA_HIGH_BW_MSK         (0x80)
#define SMI130_ACC_ENABLE_DATA_HIGH_BW_REG         SMI130_ACC_DATA_CTRL_ADDR
/***************************************************/
/**\name SOFT RESET VALUE   */
/***************************************************/
#define SMI130_ACC_ENABLE_SOFT_RESET_VALUE        (0xB6)
/**********************************************/
/**\name INTERRUPT ENABLE OF SLOPE-XYZ   */
/**********************************************/
#define SMI130_ACC_ENABLE_SLOPE_X_INTR_POS         (0)
#define SMI130_ACC_ENABLE_SLOPE_X_INTR_LEN         (1)
#define SMI130_ACC_ENABLE_SLOPE_X_INTR_MSK         (0x01)
#define SMI130_ACC_ENABLE_SLOPE_X_INTR_REG         SMI130_ACC_INTR_ENABLE1_ADDR

#define SMI130_ACC_ENABLE_SLOPE_Y_INTR_POS         (1)
#define SMI130_ACC_ENABLE_SLOPE_Y_INTR_LEN         (1)
#define SMI130_ACC_ENABLE_SLOPE_Y_INTR_MSK         (0x02)
#define SMI130_ACC_ENABLE_SLOPE_Y_INTR_REG         SMI130_ACC_INTR_ENABLE1_ADDR

#define SMI130_ACC_ENABLE_SLOPE_Z_INTR_POS         (2)
#define SMI130_ACC_ENABLE_SLOPE_Z_INTR_LEN         (1)
#define SMI130_ACC_ENABLE_SLOPE_Z_INTR_MSK         (0x04)
#define SMI130_ACC_ENABLE_SLOPE_Z_INTR_REG         SMI130_ACC_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF DOUBLE TAP   */
/**********************************************/
#define SMI130_ACC_ENABLE_DOUBLE_TAP_INTR_POS      (4)
#define SMI130_ACC_ENABLE_DOUBLE_TAP_INTR_LEN      (1)
#define SMI130_ACC_ENABLE_DOUBLE_TAP_INTR_MSK      (0x10)
#define SMI130_ACC_ENABLE_DOUBLE_TAP_INTR_REG      SMI130_ACC_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF SINGLE TAP   */
/**********************************************/
#define SMI130_ACC_ENABLE_SINGLE_TAP_INTR_POS      (5)
#define SMI130_ACC_ENABLE_SINGLE_TAP_INTR_LEN      (1)
#define SMI130_ACC_ENABLE_SINGLE_TAP_INTR_MSK      (0x20)
#define SMI130_ACC_ENABLE_SINGLE_TAP_INTR_REG      SMI130_ACC_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF ORIENT  */
/**********************************************/
#define SMI130_ACC_ENABLE_ORIENT_INTR_POS          (6)
#define SMI130_ACC_ENABLE_ORIENT_INTR_LEN          (1)
#define SMI130_ACC_ENABLE_ORIENT_INTR_MSK          (0x40)
#define SMI130_ACC_ENABLE_ORIENT_INTR_REG          SMI130_ACC_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF FLAT  */
/**********************************************/
#define SMI130_ACC_ENABLE_FLAT_INTR_POS            (7)
#define SMI130_ACC_ENABLE_FLAT_INTR_LEN            (1)
#define SMI130_ACC_ENABLE_FLAT_INTR_MSK            (0x80)
#define SMI130_ACC_ENABLE_FLAT_INTR_REG            SMI130_ACC_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF HIGH_G-XYZ   */
/**********************************************/
#define SMI130_ACC_ENABLE_HIGH_G_X_INTR_POS         (0)
#define SMI130_ACC_ENABLE_HIGH_G_X_INTR_LEN         (1)
#define SMI130_ACC_ENABLE_HIGH_G_X_INTR_MSK         (0x01)
#define SMI130_ACC_ENABLE_HIGH_G_X_INTR_REG         SMI130_ACC_INTR_ENABLE2_ADDR

#define SMI130_ACC_ENABLE_HIGH_G_Y_INTR_POS         (1)
#define SMI130_ACC_ENABLE_HIGH_G_Y_INTR_LEN         (1)
#define SMI130_ACC_ENABLE_HIGH_G_Y_INTR_MSK         (0x02)
#define SMI130_ACC_ENABLE_HIGH_G_Y_INTR_REG         SMI130_ACC_INTR_ENABLE2_ADDR

#define SMI130_ACC_ENABLE_HIGH_G_Z_INTR_POS         (2)
#define SMI130_ACC_ENABLE_HIGH_G_Z_INTR_LEN         (1)
#define SMI130_ACC_ENABLE_HIGH_G_Z_INTR_MSK         (0x04)
#define SMI130_ACC_ENABLE_HIGH_G_Z_INTR_REG         SMI130_ACC_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF LOW_G  */
/**********************************************/
#define SMI130_ACC_ENABLE_LOW_G_INTR_POS            (3)
#define SMI130_ACC_ENABLE_LOW_G_INTR_LEN            (1)
#define SMI130_ACC_ENABLE_LOW_G_INTR_MSK            (0x08)
#define SMI130_ACC_ENABLE_LOW_G_INTR_REG            SMI130_ACC_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF DATA   */
/**********************************************/
#define SMI130_ACC_ENABLE_NEW_DATA_INTR_POS        (4)
#define SMI130_ACC_ENABLE_NEW_DATA_INTR_LEN        (1)
#define SMI130_ACC_ENABLE_NEW_DATA_INTR_MSK        (0x10)
#define SMI130_ACC_ENABLE_NEW_DATA_INTR_REG        SMI130_ACC_INTR_ENABLE2_ADDR

/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD LOW_G */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_LOW_G_POS        (0)
#define SMI130_ACC_ENABLE_INTR2_PAD_LOW_G_LEN        (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_LOW_G_MSK        (0x01)
#define SMI130_ACC_ENABLE_INTR2_PAD_LOW_G_REG        SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD HIGH_G */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_HIGH_G_POS       (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_HIGH_G_LEN       (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_HIGH_G_MSK       (0x02)
#define SMI130_ACC_ENABLE_INTR2_PAD_HIGH_G_REG       SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD SLOPE */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_POS       (2)
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_LEN       (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_MSK       (0x04)
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_REG       SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD SLOW NO MOTION */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOW_NO_MOTION_POS        (3)
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOW_NO_MOTION_LEN        (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOW_NO_MOTION_MSK        (0x08)
#define SMI130_ACC_ENABLE_INTR2_PAD_SLOW_NO_MOTION_REG        \
SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD DOUBLE_TAP */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_DOUBLE_TAP_POS      (4)
#define SMI130_ACC_ENABLE_INTR2_PAD_DOUBLE_TAP_LEN      (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_DOUBLE_TAP_MSK      (0x10)
#define SMI130_ACC_ENABLE_INTR2_PAD_DOUBLE_TAP_REG      SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD SINGLE_TAP */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_SINGLE_TAP_POS     (5)
#define SMI130_ACC_ENABLE_INTR2_PAD_SINGLE_TAP_LEN     (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_SINGLE_TAP_MSK     (0x20)
#define SMI130_ACC_ENABLE_INTR2_PAD_SINGLE_TAP_REG     SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD ORIENT */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_ORIENT_POS      (6)
#define SMI130_ACC_ENABLE_INTR2_PAD_ORIENT_LEN      (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_ORIENT_MSK      (0x40)
#define SMI130_ACC_ENABLE_INTR2_PAD_ORIENT_REG      SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD FLAT */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_FLAT_POS        (7)
#define SMI130_ACC_ENABLE_INTR2_PAD_FLAT_LEN        (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_FLAT_MSK        (0x80)
#define SMI130_ACC_ENABLE_INTR2_PAD_FLAT_REG        SMI130_ACC_INTR2_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD DATA */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR1_PAD_NEWDATA_POS     (0)
#define SMI130_ACC_ENABLE_INTR1_PAD_NEWDATA_LEN     (1)
#define SMI130_ACC_ENABLE_INTR1_PAD_NEWDATA_MSK     (0x01)
#define SMI130_ACC_ENABLE_INTR1_PAD_NEWDATA_REG     SMI130_ACC_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT2 ENABLE OF PAD DATA */
/**********************************************/
#define SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_POS     (7)
#define SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_LEN     (1)
#define SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_MSK     (0x80)
#define SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_REG     SMI130_ACC_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF LOW_G*/
/**********************************************/
#define SMI130_ACC_UNFILT_INTR_SOURCE_LOW_G_POS        (0)
#define SMI130_ACC_UNFILT_INTR_SOURCE_LOW_G_LEN        (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_LOW_G_MSK        (0x01)
#define SMI130_ACC_UNFILT_INTR_SOURCE_LOW_G_REG        SMI130_ACC_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF HIGH_G*/
/**********************************************/
#define SMI130_ACC_UNFILT_INTR_SOURCE_HIGH_G_POS       (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_HIGH_G_LEN       (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_HIGH_G_MSK       (0x02)
#define SMI130_ACC_UNFILT_INTR_SOURCE_HIGH_G_REG       SMI130_ACC_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF SLOPE*/
/**********************************************/
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_POS       (2)
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_LEN       (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_MSK       (0x04)
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_REG       SMI130_ACC_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF SLOW NO MOTION*/
/**********************************************/
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_POS        (3)
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_LEN        (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_MSK        (0x08)
#define SMI130_ACC_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_REG        \
SMI130_ACC_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF TAP*/
/**********************************************/
#define SMI130_ACC_UNFILT_INTR_SOURCE_TAP_POS         (4)
#define SMI130_ACC_UNFILT_INTR_SOURCE_TAP_LEN         (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_TAP_MSK         (0x10)
#define SMI130_ACC_UNFILT_INTR_SOURCE_TAP_REG         SMI130_ACC_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF DATA*/
/**********************************************/
#define SMI130_ACC_UNFILT_INTR_SOURCE_DATA_POS        (5)
#define SMI130_ACC_UNFILT_INTR_SOURCE_DATA_LEN        (1)
#define SMI130_ACC_UNFILT_INTR_SOURCE_DATA_MSK        (0x20)
#define SMI130_ACC_UNFILT_INTR_SOURCE_DATA_REG        SMI130_ACC_INTR_SOURCE_ADDR
/****************************************************/
/**\name  INTERRUPT PAD ACTIVE LEVEL AND OUTPUT TYPE*/
/****************************************************/
#define SMI130_ACC_INTR1_PAD_ACTIVE_LEVEL_POS       (0)
#define SMI130_ACC_INTR1_PAD_ACTIVE_LEVEL_LEN       (1)
#define SMI130_ACC_INTR1_PAD_ACTIVE_LEVEL_MSK       (0x01)
#define SMI130_ACC_INTR1_PAD_ACTIVE_LEVEL_REG       SMI130_ACC_INTR_SET_ADDR

#define SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_POS       (2)
#define SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_LEN       (1)
#define SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_MSK       (0x04)
#define SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_REG       SMI130_ACC_INTR_SET_ADDR

#define SMI130_ACC_INTR1_PAD_OUTPUT_TYPE_POS        (1)
#define SMI130_ACC_INTR1_PAD_OUTPUT_TYPE_LEN        (1)
#define SMI130_ACC_INTR1_PAD_OUTPUT_TYPE_MSK        (0x02)
#define SMI130_ACC_INTR1_PAD_OUTPUT_TYPE_REG        SMI130_ACC_INTR_SET_ADDR

#define SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_POS        (3)
#define SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_LEN        (1)
#define SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_MSK        (0x08)
#define SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_REG        SMI130_ACC_INTR_SET_ADDR
/****************************************************/
/**\name   LATCH INTERRUPT */
/****************************************************/
#define SMI130_ACC_LATCH_INTR_POS                (0)
#define SMI130_ACC_LATCH_INTR_LEN                (4)
#define SMI130_ACC_LATCH_INTR_MSK                (0x0F)
#define SMI130_ACC_LATCH_INTR_REG                SMI130_ACC_INTR_CTRL_ADDR
/****************************************************/
/**\name   RESET LATCH INTERRUPT */
/****************************************************/
#define SMI130_ACC_RESET_INTR_POS           (7)
#define SMI130_ACC_RESET_INTR_LEN           (1)
#define SMI130_ACC_RESET_INTR_MSK           (0x80)
#define SMI130_ACC_RESET_INTR_REG           SMI130_ACC_INTR_CTRL_ADDR

/****************************************************/
/**\name   SLOPE DURATION */
/****************************************************/
#define SMI130_ACC_SLOPE_DURN_POS                    (0)
#define SMI130_ACC_SLOPE_DURN_LEN                    (2)
#define SMI130_ACC_SLOPE_DURN_MSK                    (0x03)
#define SMI130_ACC_SLOPE_DURN_REG                    SMI130_ACC_SLOPE_DURN_ADDR
/****************************************************/
/**\name   SLOW NO MOTION DURATION */
/****************************************************/
#define SMI130_ACC_SLOW_NO_MOTION_DURN_POS                    (2)
#define SMI130_ACC_SLOW_NO_MOTION_DURN_LEN                    (6)
#define SMI130_ACC_SLOW_NO_MOTION_DURN_MSK                    (0xFC)
#define SMI130_ACC_SLOW_NO_MOTION_DURN_REG                    SMI130_ACC_SLOPE_DURN_ADDR

/****************************************************/
/**\name   ACTIVATE SELF TEST  */
/****************************************************/
#define SMI130_ACC_ENABLE_SELFTEST_POS                (0)
#define SMI130_ACC_ENABLE_SELFTEST_LEN                (2)
#define SMI130_ACC_ENABLE_SELFTEST_MSK                (0x03)
#define SMI130_ACC_ENABLE_SELFTEST_REG                SMI130_ACC_SELFTEST_ADDR
/****************************************************/
/**\name   SELF TEST -- NEGATIVE   */
/****************************************************/
#define SMI130_ACC_NEG_SELFTEST_POS               (2)
#define SMI130_ACC_NEG_SELFTEST_LEN               (1)
#define SMI130_ACC_NEG_SELFTEST_MSK               (0x04)
#define SMI130_ACC_NEG_SELFTEST_REG               SMI130_ACC_SELFTEST_ADDR
/****************************************************/
/**\name   SELF TEST -- AMPLITUDE   */
/****************************************************/
#define SMI130_ACC_AMP_SELFTEST_POS               (4)
#define SMI130_ACC_AMP_SELFTEST_LEN               (1)
#define SMI130_ACC_AMP_SELFTEST_MSK               (0x10)
#define SMI130_ACC_AMP_SELFTEST_REG               SMI130_ACC_SELFTEST_ADDR
/****************************************************/
/**\name   SPI INTERFACE MODE SELECTION   */
/***************************************************/
#define SMI130_ACC_ENABLE_SPI_MODE_3_POS              (0)
#define SMI130_ACC_ENABLE_SPI_MODE_3_LEN              (1)
#define SMI130_ACC_ENABLE_SPI_MODE_3_MSK              (0x01)
#define SMI130_ACC_ENABLE_SPI_MODE_3_REG              SMI130_ACC_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   I2C WATCHDOG PERIOD SELECTION   */
/***************************************************/
#define SMI130_ACC_I2C_WDT_PERIOD_POS        (1)
#define SMI130_ACC_I2C_WDT_PERIOD_LEN        (1)
#define SMI130_ACC_I2C_WDT_PERIOD_MSK        (0x02)
#define SMI130_ACC_I2C_WDT_PERIOD_REG        SMI130_ACC_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   I2C WATCHDOG ENABLE   */
/***************************************************/
#define SMI130_ACC_ENABLE_I2C_WDT_POS            (2)
#define SMI130_ACC_ENABLE_I2C_WDT_LEN            (1)
#define SMI130_ACC_ENABLE_I2C_WDT_MSK            (0x04)
#define SMI130_ACC_ENABLE_I2C_WDT_REG            SMI130_ACC_SERIAL_CTRL_ADDR
/******************************************************************/
/**\name   OFFSET  COMPENSATION/SLOW COMPENSATION FOR X,Y,Z AXIS */
/*****************************************************************/
#define SMI130_ACC_ENABLE_SLOW_COMP_X_POS              (0)
#define SMI130_ACC_ENABLE_SLOW_COMP_X_LEN              (1)
#define SMI130_ACC_ENABLE_SLOW_COMP_X_MSK              (0x01)
#define SMI130_ACC_ENABLE_SLOW_COMP_X_REG              SMI130_ACC_OFFSET_CTRL_ADDR

#define SMI130_ACC_ENABLE_SLOW_COMP_Y_POS              (1)
#define SMI130_ACC_ENABLE_SLOW_COMP_Y_LEN              (1)
#define SMI130_ACC_ENABLE_SLOW_COMP_Y_MSK              (0x02)
#define SMI130_ACC_ENABLE_SLOW_COMP_Y_REG              SMI130_ACC_OFFSET_CTRL_ADDR

#define SMI130_ACC_ENABLE_SLOW_COMP_Z_POS              (2)
#define SMI130_ACC_ENABLE_SLOW_COMP_Z_LEN              (1)
#define SMI130_ACC_ENABLE_SLOW_COMP_Z_MSK              (0x04)
#define SMI130_ACC_ENABLE_SLOW_COMP_Z_REG              SMI130_ACC_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   FAST COMPENSATION READY FLAG            */
/***************************************************/
#define SMI130_ACC_FAST_CAL_RDY_STAT_POS             (4)
#define SMI130_ACC_FAST_CAL_RDY_STAT_LEN             (1)
#define SMI130_ACC_FAST_CAL_RDY_STAT_MSK             (0x10)
#define SMI130_ACC_FAST_CAL_RDY_STAT_REG             SMI130_ACC_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   FAST COMPENSATION FOR X,Y,Z AXIS         */
/***************************************************/
#define SMI130_ACC_CAL_TRIGGER_POS                (5)
#define SMI130_ACC_CAL_TRIGGER_LEN                (2)
#define SMI130_ACC_CAL_TRIGGER_MSK                (0x60)
#define SMI130_ACC_CAL_TRIGGER_REG                SMI130_ACC_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   RESET OFFSET REGISTERS         */
/***************************************************/
#define SMI130_ACC_RST_OFFSET_POS           (7)
#define SMI130_ACC_RST_OFFSET_LEN           (1)
#define SMI130_ACC_RST_OFFSET_MSK           (0x80)
#define SMI130_ACC_RST_OFFSET_REG           SMI130_ACC_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   SLOW COMPENSATION  CUTOFF        */
/***************************************************/
#define SMI130_ACC_COMP_CUTOFF_POS                 (0)
#define SMI130_ACC_COMP_CUTOFF_LEN                 (1)
#define SMI130_ACC_COMP_CUTOFF_MSK                 (0x01)
#define SMI130_ACC_COMP_CUTOFF_REG                 SMI130_ACC_OFFSET_PARAMS_ADDR
/****************************************************/
/**\name    COMPENSATION TARGET       */
/***************************************************/
#define SMI130_ACC_COMP_TARGET_OFFSET_X_POS        (1)
#define SMI130_ACC_COMP_TARGET_OFFSET_X_LEN        (2)
#define SMI130_ACC_COMP_TARGET_OFFSET_X_MSK        (0x06)
#define SMI130_ACC_COMP_TARGET_OFFSET_X_REG        SMI130_ACC_OFFSET_PARAMS_ADDR

#define SMI130_ACC_COMP_TARGET_OFFSET_Y_POS        (3)
#define SMI130_ACC_COMP_TARGET_OFFSET_Y_LEN        (2)
#define SMI130_ACC_COMP_TARGET_OFFSET_Y_MSK        (0x18)
#define SMI130_ACC_COMP_TARGET_OFFSET_Y_REG        SMI130_ACC_OFFSET_PARAMS_ADDR

#define SMI130_ACC_COMP_TARGET_OFFSET_Z_POS        (5)
#define SMI130_ACC_COMP_TARGET_OFFSET_Z_LEN        (2)
#define SMI130_ACC_COMP_TARGET_OFFSET_Z_MSK        (0x60)
#define SMI130_ACC_COMP_TARGET_OFFSET_Z_REG        SMI130_ACC_OFFSET_PARAMS_ADDR
/****************************************************/
/**\name  BITSLICE FUNCTIONS      */
/***************************************************/
#define SMI130_ACC_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##_MSK) >> bitname##_POS)


#define SMI130_ACC_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##_MSK) | ((val<<bitname##_POS)&bitname##_MSK))

/****************************************************/
/**\name   CONSTANTS      */
/***************************************************/
#define SMI130_ACC_CHIP_ID		(0xFA)
/****************************************************/
/**\name  RESOLUTION SELECTION      */
/***************************************************/
/* Definitions used for accel resolution bit shifting*/
#define SMI130_ACC_14_BIT_SHIFT		(0xFC)
/**< It refers 14bit accel resolution*/
#define SMI130_ACC_10_BIT_SHIFT		(0xC0)
/**< It refers 10bit accel resolution*/
#define SMI130_ACC_12_BIT_SHIFT		(0xF0)
/**< It refers 12bit accel resolution*/
#define BANDWIDTH_DEFINE		(0xFA)
/**< Chip id set for accel bandwidth define*/

/****************************************************/
/**\name  ENABLE DISABLE SELECTION     */
/***************************************************/
#define INTR_ENABLE	(0X01)
/**< Enable selection for bit */
#define INTR_DISABLE	(0x00)
/**< Disable selection for bit */

/****************************************************/
/**\name  OUTPUT TYPE SELECT     */
/***************************************************/
#define OPEN_DRAIN	(0x01)
/**< It refers open drain selection*/
#define PUSH_PULL	(0x00)
/**< It refers push pull selection*/

/****************************************************/
/**\name  LEVEL SELECT     */
/***************************************************/
#define	ACTIVE_LOW	(0x00)
/**< It refers active low selection*/
#define	ACTIVE_HIGH	(0x01)
/**< It refers active high selection*/

/****************************************************/
/**\name  STATUS SELECT     */
/***************************************************/
#define SMI130_ACC_STAT1                             (0)
/**< It refers Status interrupt1 */
#define SMI130_ACC_STAT2                             (1)
/**< It refers Status interrupt2 */
#define SMI130_ACC_STAT3                             (2)
/**< It refers Status interrupt3  */
#define SMI130_ACC_STAT4                             (3)
/**< It refers Status interrupt4  */
#define SMI130_ACC_STAT5                             (4)
/**< It refers Status interrupt5  */

/****************************************************/
/**\name  RANGE AND BANDWIDTH SELECT     */
/***************************************************/
#define SMI130_ACC_RANGE_2G                 (3)
/**< sets range to +/- 2G mode */
#define SMI130_ACC_RANGE_4G                 (5)
/**< sets range to +/- 4G mode */
#define SMI130_ACC_RANGE_8G                 (8)
/**< sets range to +/- 8G mode */
#define SMI130_ACC_RANGE_16G                (12)
/**< sets range to +/- 16G mode */


#define SMI130_ACC_BW_7_81HZ        (0x08)
/**< sets bandwidth to LowPass 7.81HZ  */
#define SMI130_ACC_BW_15_63HZ       (0x09)
/**< sets bandwidth to LowPass 15.63HZ  */
#define SMI130_ACC_BW_31_25HZ       (0x0A)
/**< sets bandwidth to LowPass 31.25HZ  */
#define SMI130_ACC_BW_62_50HZ       (0x0B)
/**< sets bandwidth to LowPass 62.50HZ  */
#define SMI130_ACC_BW_125HZ         (0x0C)
/**< sets bandwidth to LowPass 125HZ  */
#define SMI130_ACC_BW_250HZ         (0x0D)
/**< sets bandwidth to LowPass 250HZ  */
#define SMI130_ACC_BW_500HZ         (0x0E)
/**< sets bandwidth to LowPass 500HZ  */
#define SMI130_ACC_BW_1000HZ        (0x0F)
/**< sets bandwidth to LowPass 1000HZ  */

/******************************************/
/**\name  SLEEP DURATION SELECT     */
/******************************************/
#define SMI130_ACC_SLEEP_DURN_0_5MS        (0x05)
/* sets sleep duration to 0.5 ms  */
#define SMI130_ACC_SLEEP_DURN_1MS          (0x06)
/* sets sleep duration to 1 ms */
#define SMI130_ACC_SLEEP_DURN_2MS          (0x07)
/* sets sleep duration to 2 ms */
#define SMI130_ACC_SLEEP_DURN_4MS          (0x08)
/* sets sleep duration to 4 ms */
#define SMI130_ACC_SLEEP_DURN_6MS          (0x09)
/* sets sleep duration to 6 ms*/
#define SMI130_ACC_SLEEP_DURN_10MS         (0x0A)
/* sets sleep duration to 10 ms */
#define SMI130_ACC_SLEEP_DURN_25MS         (0x0B)
/* sets sleep duration to 25 ms */
#define SMI130_ACC_SLEEP_DURN_50MS         (0x0C)
/* sets sleep duration to 50 ms */
#define SMI130_ACC_SLEEP_DURN_100MS        (0x0D)
/* sets sleep duration to 100 ms */
#define SMI130_ACC_SLEEP_DURN_500MS        (0x0E)
/* sets sleep duration to 500 ms */
#define SMI130_ACC_SLEEP_DURN_1S           (0x0F)
/* sets sleep duration to 1 s */

/******************************************/
/**\name  LATCH DURATION     */
/******************************************/
#define SMI130_ACC_LATCH_DURN_NON_LATCH    (0x00)
/* sets LATCH duration to NON LATCH  */
#define SMI130_ACC_LATCH_DURN_250MS        (0x01)
/* sets LATCH duration to 250 ms */
#define SMI130_ACC_LATCH_DURN_500MS        (0x02)
/* sets LATCH duration to 500 ms */
#define SMI130_ACC_LATCH_DURN_1S           (0x03)
/* sets LATCH duration to 1 s */
#define SMI130_ACC_LATCH_DURN_2S           (0x04)
/* sets LATCH duration to 2 s*/
#define SMI130_ACC_LATCH_DURN_4S           (0x05)
/* sets LATCH duration to 4 s */
#define SMI130_ACC_LATCH_DURN_8S           (0x06)
/* sets LATCH duration to 8 s */
#define SMI130_ACC_LATCH_DURN_LATCH        (0x07)
/* sets LATCH duration to LATCH */
#define SMI130_ACC_LATCH_DURN_NON_LATCH1   (0x08)
/* sets LATCH duration to NON LATCH1 */
#define SMI130_ACC_LATCH_DURN_250US        (0x09)
/* sets LATCH duration to 250 Us */
#define SMI130_ACC_LATCH_DURN_500US        (0x0A)
/* sets LATCH duration to 500 Us */
#define SMI130_ACC_LATCH_DURN_1MS          (0x0B)
/* sets LATCH duration to 1 Ms */
#define SMI130_ACC_LATCH_DURN_12_5MS       (0x0C)
/* sets LATCH duration to 12.5 Ms */
#define SMI130_ACC_LATCH_DURN_25MS         (0x0D)
/* sets LATCH duration to 25 Ms */
#define SMI130_ACC_LATCH_DURN_50MS         (0x0E)
/* sets LATCH duration to 50 Ms */
#define SMI130_ACC_LATCH_DURN_LATCH1       (0x0F)
/* sets LATCH duration to LATCH*/

/******************************************/
/**\name  MODE SETTINGS     */
/******************************************/
#define SMI130_ACC_MODE_NORMAL             (0)
#define SMI130_ACC_MODE_DEEP_SUSPEND       (3)
#define SMI130_ACC_MODE_LOWPOWER2          (4)
#define SMI130_ACC_MODE_STANDBY            (5)

/******************************************/
/**\name  AXIS SELECTION     */
/******************************************/
#define SMI130_ACC_X_AXIS           (0)
/**< It refers SMI130_ACC X-axis */
#define SMI130_ACC_Y_AXIS           (1)
/**< It refers SMI130_ACC Y-axis */
#define SMI130_ACC_Z_AXIS           (2)
/**< It refers SMI130_ACC Z-axis */

/******************************************/
/**\name  INTERRUPT TYPE SELECTION     */
/******************************************/
#define SMI130_ACC_LOW_G_INTR       (0)
/**< enable/disable low-g interrupt*/
#define SMI130_ACC_HIGH_G_X_INTR    (1)
/**< enable/disable high_g X interrupt*/
#define SMI130_ACC_HIGH_G_Y_INTR    (2)
/**< enable/disable high_g Y interrupt*/
#define SMI130_ACC_HIGH_G_Z_INTR    (3)
/**< enable/disable high_g Z interrupt*/
#define SMI130_ACC_DATA_ENABLE      (4)
/**< enable/disable data interrupt*/
#define SMI130_ACC_SLOPE_X_INTR     (5)
/**< enable/disable slope X interrupt*/
#define SMI130_ACC_SLOPE_Y_INTR     (6)
/**< enable/disable slope X interrupt*/
#define SMI130_ACC_SLOPE_Z_INTR     (7)
/**< enable/disable slope X interrupt*/
#define SMI130_ACC_SINGLE_TAP_INTR  (8)
/**< enable/disable single tap interrupt*/
#define SMI130_ACC_DOUBLE_TAP_INTR  (9)
/**< enable/disable double tap interrupt*/
#define SMI130_ACC_ORIENT_INTR      (10)
/**< enable/disable orient interrupt*/
#define SMI130_ACC_FLAT_INTR        (11)
/******************************************/
/**\name  INTERRUPTS PADS     */
/******************************************/
#define SMI130_ACC_INTR1_LOW_G             (0)
/**< disable low-g interrupt*/
#define SMI130_ACC_INTR2_LOW_G             (1)
/**< enable low-g interrupt*/
#define SMI130_ACC_INTR1_HIGH_G            (0)
/**< disable high-g interrupt*/
#define SMI130_ACC_INTR2_HIGH_G            (1)
/**< enable high-g interrupt*/
#define SMI130_ACC_INTR1_SLOPE             (0)
/**< disable slope interrupt*/
#define SMI130_ACC_INTR2_SLOPE             (1)
/**< enable slope interrupt*/
#define SMI130_ACC_INTR1_SLOW_NO_MOTION    (0)
/**< disable slow no motion interrupt*/
#define SMI130_ACC_INTR2_SLOW_NO_MOTION    (1)
/**< enable slow no motion  interrupt*/
#define SMI130_ACC_INTR1_DOUBLE_TAP        (0)
/**< disable double tap  interrupt*/
#define SMI130_ACC_INTR2_DOUBLE_TAP        (1)
/**< enable double tap  interrupt*/
#define SMI130_ACC_INTR1_SINGLE_TAP        (0)
/**< disable single tap  interrupt*/
#define SMI130_ACC_INTR2_SINGLE_TAP        (1)
/**< enable single tap  interrupt*/
#define SMI130_ACC_INTR1_ORIENT            (0)
/**< disable orient  interrupt*/
#define SMI130_ACC_INTR2_ORIENT            (1)
/**< enable orient  interrupt*/
#define SMI130_ACC_INTR1_FLAT              (0)
/**< disable flat  interrupt*/
#define SMI130_ACC_INTR2_FLAT              (1)
/**< enable flat  interrupt*/
#define SMI130_ACC_INTR1_NEWDATA           (0)
/**< disable data  interrupt*/
#define SMI130_ACC_INTR2_NEWDATA           (1)
/**< enable data interrupt*/

/******************************************/
/**\name  SOURCE REGISTER     */
/******************************************/
#define SMI130_ACC_SOURCE_LOW_G            (0)
#define SMI130_ACC_SOURCE_HIGH_G           (1)
#define SMI130_ACC_SOURCE_SLOPE            (2)
#define SMI130_ACC_SOURCE_SLOW_NO_MOTION   (3)
#define SMI130_ACC_SOURCE_TAP              (4)
#define SMI130_ACC_SOURCE_DATA             (5)

#define SMI130_ACC_INTR1_OUTPUT      (0)
#define SMI130_ACC_INTR2_OUTPUT      (1)
#define SMI130_ACC_INTR1_LEVEL       (0)
#define SMI130_ACC_INTR2_LEVEL       (1)

/******************************************/
/**\name  DURATION     */
/******************************************/
#define SMI130_ACC_LOW_DURN                (0)
#define SMI130_ACC_HIGH_DURN               (1)
#define SMI130_ACC_SLOPE_DURN              (2)
#define SMI130_ACC_SLOW_NO_MOTION_DURN     (3)

/******************************************/
/**\name  THRESHOLD     */
/******************************************/
#define SMI130_ACC_LOW_THRES                (0)
#define SMI130_ACC_HIGH_THRES               (1)
#define SMI130_ACC_SLOPE_THRES              (2)
#define SMI130_ACC_SLOW_NO_MOTION_THRES     (3)


#define SMI130_ACC_LOW_G_HYST                (0)
#define SMI130_ACC_HIGH_G_HYST               (1)

#define SMI130_ACC_ORIENT_THETA             (0)
#define SMI130_ACC_FLAT_THETA               (1)

#define SMI130_ACC_I2C_SELECT               (0)
#define SMI130_ACC_I2C_ENABLE               (1)
/******************************************/
/**\name  COMPENSATION     */
/******************************************/
#define SMI130_ACC_SLOW_COMP_X              (0)
#define SMI130_ACC_SLOW_COMP_Y              (1)
#define SMI130_ACC_SLOW_COMP_Z              (2)
/******************************************/
/**\name  OFFSET TRIGGER     */
/******************************************/
#define SMI130_ACC_CUT_OFF                  (0)
#define SMI130_ACC_OFFSET_TRIGGER_X         (1)
#define SMI130_ACC_OFFSET_TRIGGER_Y         (2)
#define SMI130_ACC_OFFSET_TRIGGER_Z         (3)
/******************************************/
/**\name  GP REGISTERS     */
/******************************************/
#define SMI130_ACC_GP0                      (0)
#define SMI130_ACC_GP1                      (1)
/******************************************/
/**\name  SLO NO MOTION REGISTER      */
/******************************************/
#define SMI130_ACC_SLOW_NO_MOTION_ENABLE_X          (0)
#define SMI130_ACC_SLOW_NO_MOTION_ENABLE_Y          (1)
#define SMI130_ACC_SLOW_NO_MOTION_ENABLE_Z          (2)
#define SMI130_ACC_SLOW_NO_MOTION_ENABLE_SELECT     (3)
/******************************************/
/**\name  WAKE UP      */
/******************************************/
#define SMI130_ACC_WAKE_UP_DURN_20MS         (0)
#define SMI130_ACC_WAKE_UP_DURN_80MS         (1)
#define SMI130_ACC_WAKE_UP_DURN_320MS        (2)
#define SMI130_ACC_WAKE_UP_DURN_2560MS       (3)


/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */

#define SMI130_ACC_SELFTEST0_ON            (1)
#define SMI130_ACC_SELFTEST1_ON            (2)

#define SMI130_ACC_EE_W_OFF                 (0)
#define SMI130_ACC_EE_W_ON                  (1)
/******************************************/
/**\name  RESOLUTION SETTINGS      */
/******************************************/
#define SMI130_ACC_RESOLUTION_12_BIT        (0)
#define SMI130_ACC_RESOLUTION_10_BIT        (1)
#define SMI130_ACC_RESOLUTION_14_BIT        (3)

/******************************************/
/**\name  CHIP ID SELECTION      */
/******************************************/
#define SMI130_ACC           (0x16)
/******************************************/
/**\name  LOW-G MODE SELECTION    */
/******************************************/
#define LOW_G_SINGLE_AXIS_MODE		(0x00)
#define LOW_G_SUMMING_MODE		(0x01)
/******************************************/
/**\name TAP DURATION DEFINITION    */
/******************************************/
#define TAP_DURN_50_MS			(0x00)
#define TAP_DURN_100_MS			(0x01)
#define TAP_DURN_150_MS			(0x02)
#define TAP_DURN_200_MS			(0x03)
#define TAP_DURN_250_MS			(0x04)
#define TAP_DURN_375_MS			(0x05)
#define TAP_DURN_500_MS			(0x06)
#define TAP_DURN_700_MS			(0x07)
/******************************************/
/**\name TAP SHOCK DEFINITION    */
/******************************************/
#define TAP_SHOCK_50_MS		(0x00)
#define TAP_SHOCK_75_MS		(0x01)
/******************************************/
/**\name TAP QUIET DEFINITION    */
/******************************************/
#define	TAP_QUIET_30_MS		(0x00)
#define	TAP_QUIET_20_MS		(0x01)
/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define SMI130_ACC_DATA_SIZE			(2)
#define SMI130_ACC_XYZ_DATA_SIZE		(6)
#define SMI130_ACC_XYZ_TEMP_DATA_SIZE		(7)
/****************************************************/
/**\name	ARRAY PARAMETERS      */
/***************************************************/

#define SMI130_ACC_SENSOR_DATA_ACCEL_LSB			(0)
#define SMI130_ACC_SENSOR_DATA_ACCEL_MSB			(1)

#define SMI130_ACC_SENSOR_DATA_XYZ_X_LSB			(0)
#define SMI130_ACC_SENSOR_DATA_XYZ_X_MSB			(1)
#define SMI130_ACC_SENSOR_DATA_XYZ_Y_LSB			(2)
#define SMI130_ACC_SENSOR_DATA_XYZ_Y_MSB			(3)
#define SMI130_ACC_SENSOR_DATA_XYZ_Z_LSB			(4)
#define SMI130_ACC_SENSOR_DATA_XYZ_Z_MSB			(5)
#define SMI130_ACC_SENSOR_DATA_TEMP				(6)

#define SMI130_ACC_RESOLUTION_12_MASK				(0xF0)
#define SMI130_ACC_RESOLUTION_10_MASK				(0xC0)
#define SMI130_ACC_RESOLUTION_14_MASK				(0xFC)

#define	SMI130_ACC_POWER_MODE_HEX_E_ZERO_MASK		(0xE0)
#define	SMI130_ACC_POWER_MODE_HEX_4_ZERO_MASK		(0x40)
#define	SMI130_ACC_POWER_MODE_HEX_ZERO_ZERO_MASK	(0x00)
#define	SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK		(0x01)
#define	SMI130_ACC_POWER_MODE_HEX_ZERO_TWO_MASK		(0x02)
#define	SMI130_ACC_POWER_MODE_HEX_ZERO_FOUR_MASK	(0x04)
#define	SMI130_ACC_POWER_MODE_HEX_ZERO_SIX_MASK		(0x06)

/** Macro to convert floating point
low-g-thresholds in G to 8-bit register values.<br>
  * Example: SMI130_ACC_LOW_TH_IN_G( 0.3, 2.0) generates
  * the register value for 0.3G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define SMI130_ACC_LOW_THRES_IN_G(gthres, range)  ((256 * gthres) / range)

/** Macro to convert floating point high-g-thresholds
    in G to 8-bit register values.<br>
  * Example: SMI130_ACC_HIGH_TH_IN_G( 1.4, 2.0)
  * generates the register value for 1.4G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define SMI130_ACC_HIGH_THRES_IN_G(gthres, range)   ((256 * gthres) / range)

/** Macro to convert floating point low-g-hysteresis
in G to 8-bit register values.<br>
  * Example: SMI130_ACC_LOW_HY_IN_G( 0.2, 2.0)
  *generates the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define SMI130_ACC_LOW_HYST_IN_G(ghyst, range)   ((32 * ghyst) / range)

/** Macro to convert floating point high-g-hysteresis
   in G to 8-bit register values.<br>
  * Example: SMI130_ACC_HIGH_HY_IN_G( 0.2, 2.0) generates
  *the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define SMI130_ACC_HIGH_HYST_IN_G(ghyst, range)    ((32 * ghyst) / range)


/** Macro to convert floating point G-thresholds
    to 8-bit register values<br>
  * Example: SMI130_ACC_SLOPE_TH_IN_G( 1.2, 2.0)
  * generates the register value for 1.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */

#define SMI130_ACC_SLOPE_THRES_IN_G(gthres, range)    ((128 * gthres) / range)
/******************************************/
/**\name FUNCTION DECLARATION  */
/******************************************/
/******************************************/
/**\name FUNCTION FOR COMMON READ AND WRITE   */
/******************************************/
/*!
 * @brief
 *	This API reads the data from
 *	the given register continuously
 *
 *
 *	@param addr -> Address of the register
 *	@param data -> The data from the register
 *	@param len -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_burst_read(uint8_t addr,
                uint8_t *data, uint32_t len);
/******************************************/
/**\name FUNCTION FOR INTIALIZE  */
/******************************************/
/*!
 *	@brief
 *	This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	@param smi130_acc : structure pointer
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	While changing the parameter of the smi130_acc_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_init(struct smi130_acc_t *smi130_acc);
/*!
 * @brief
 *	This API gives data to the given register and
 *	the data is written in the corresponding register address
 *
 *
 *	@param adr  -> Address of the register
 *	@param data -> The data to the register
 *	@param len -> no of bytes to read
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_write_reg(uint8_t adr,
                uint8_t *data, uint8_t len);
/*!
 * @brief This API reads the data from
 *           the given register address
 *
 *
 *	@param adr -> Address of the register
 *	@param data -> The data from the register
 *	@param len -> no of bytes to read
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_reg(uint8_t adr,
                uint8_t *data, uint8_t len);
/******************************************/
/**\name FUNCTION FOR   DATA READ*/
/******************************************/
/*!
 * @brief
 *	This API reads acceleration data X values
 *	from location 02h and 03h
 *
 *
 *  @param   accel_x_s16 : pointer holding the data of accel X
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_read_accel_x(int16_t *accel_x_s16);
/*!
 * @brief
 *	This API reads acceleration data Y values
 *	from location 04h and 05h
 *
 *  @param   accel_y_s16 : pointer holding the data of accel Y
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_read_accel_y(int16_t *accel_y_s16);
/*!
 * @brief This API reads acceleration data Z values
 *                          from location 06h and 07h
 *
 *
 *  @param   accel_z_s16 : pointer holding the data of accel Z
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_read_accel_z(int16_t *accel_z_s16);
/*!
 *	@brief This API reads acceleration data X,Y,Z values
 *	from location 02h to 07h
 *
 *  @param accel : pointer holding the data of accel
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_read_accel_xyz(
        struct smi130_acc_data *accel);
/******************************************/
/**\name FUNCTION FOR INTERRUPT STATUS*/
/******************************************/
/*!
 *	@brief This API read tap-sign, tap-first-xyz
 *	slope-sign, slope-first-xyz status register byte
 *	from location 0Bh
 *
 *   @param stat_slope : The status of slope
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_slope_stat(
        uint8_t *stat_slope);
/******************************************/
/**\name FUNCTION FOR  RANGE */
/******************************************/
/*!
 *	@brief This API read interrupt status of slope from location 09h
 *
 *
 *
 *	@param  intr_stat : The value of interrupt status
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_stat(
        uint8_t *intr_stat);
/*!
 * @brief This API is used to get the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range : The value of range
 *		  range       |   result
 *       ----------------- | --------------
 *              0x03       | SMI130_ACC_RANGE_2G
 *              0x05       | SMI130_ACC_RANGE_4G
 *              0x08       | SMI130_ACC_RANGE_8G
 *              0x0C       | SMI130_ACC_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_range(uint8_t *range);
/*!
 * @brief This API is used to set the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range : The value of range
 *		  range       |   result
 *       ----------------- | --------------
 *              0x03       | SMI130_ACC_RANGE_2G
 *              0x05       | SMI130_ACC_RANGE_4G
 *              0x08       | SMI130_ACC_RANGE_8G
 *              0x0C       | SMI130_ACC_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_range(uint8_t range);
/******************************************/
/**\name FUNCTION FOR   BANDWIDTH*/
/******************************************/
/*!
 *	@brief This API is used to get the bandwidth of the sensor in the register
 *	0x10 bit from 0 to 4
 *
 *
 *  @param bw : The value of bandwidth
 *		  bw          |   result
 *       ----------------- | --------------
 *              0x08       | SMI130_ACC_BW_7_81HZ
 *              0x09       | SMI130_ACC_BW_15_63HZ
 *              0x0A       | SMI130_ACC_BW_31_25HZ
 *              0x0B       | SMI130_ACC_BW_62_50HZ
 *              0x0C       | SMI130_ACC_BW_125HZ
 *              0x0D       | SMI130_ACC_BW_250HZ
 *              0x0E       | SMI130_ACC_BW_500HZ
 *              0x0F       | SMI130_ACC_BW_1000HZ
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_bw(uint8_t *bw);
/*!
 *	@brief This API is used to set the bandwidth of the sensor in the register
 *	0x10 bit from 0 to 4
 *
 *
 *  @param bw : The value of bandwidth
 *		  bw          |   result
 *       ----------------- | --------------
 *              0x08       | SMI130_ACC_BW_7_81HZ
 *              0x09       | SMI130_ACC_BW_15_63HZ
 *              0x0A       | SMI130_ACC_BW_31_25HZ
 *              0x0B       | SMI130_ACC_BW_62_50HZ
 *              0x0C       | SMI130_ACC_BW_125HZ
 *              0x0D       | SMI130_ACC_BW_250HZ
 *              0x0E       | SMI130_ACC_BW_500HZ
 *              0x0F       | SMI130_ACC_BW_1000HZ
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_bw(uint8_t bw);
/******************************************/
/**\name FUNCTION FOR   POWER MODE*/
/******************************************/
/*!
 *	@brief This API is used to get the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode : The value of power mode
 *	power_mode           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  SMI130_ACC_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  SMI130_ACC_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  SMI130_ACC_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  SMI130_ACC_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_power_mode(
        uint8_t *power_mode);
/*!
 *	@brief This API is used to set the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode : The value of power mode
 *	power_mode           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  SMI130_ACC_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  SMI130_ACC_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  SMI130_ACC_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  SMI130_ACC_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_power_mode(uint8_t power_mode);
/*!
 *	@brief This API is used to assign the power mode values
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode : The value of power mode
 *	power_mode           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  SMI130_ACC_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  SMI130_ACC_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *  SMI130_ACC_MODE_LOWPOWER2     |  4    |  0x02   |  0x01
 *  SMI130_ACC_MODE_STANDBY       |  5    |  0x04   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_mode_value(uint8_t power_mode);
/******************************************/
/**\name FUNCTION FOR  SLEEP CONFIGURATION */
/******************************************/
/*!
 *	@brief This API is used to get
 *	the sleep duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  @param  sleep_durn : The value of sleep duration time
 *        sleep_durn  |   result
 *       ----------------- | ----------------------
 *              0x05       | SMI130_ACC_SLEEP_DURN_0_5MS
 *              0x06       | SMI130_ACC_SLEEP_DURN_1MS
 *              0x07       | SMI130_ACC_SLEEP_DURN_2MS
 *              0x08       | SMI130_ACC_SLEEP_DURN_4MS
 *              0x09       | SMI130_ACC_SLEEP_DURN_6MS
 *              0x0A       | SMI130_ACC_SLEEP_DURN_10MS
 *              0x0B       | SMI130_ACC_SLEEP_DURN_25MS
 *              0x0C       | SMI130_ACC_SLEEP_DURN_50MS
 *              0x0D       | SMI130_ACC_SLEEP_DURN_100MS
 *              0x0E       | SMI130_ACC_SLEEP_DURN_500MS
 *              0x0F       | SMI130_ACC_SLEEP_DURN_1S
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_sleep_durn(uint8_t *sleep_durn);
/*!
 *	@brief This API is used to set
 *	the sleep duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  @param  sleep_durn : The value of sleep duration time
 *        sleep_durn  |   result
 *       ----------------- | ----------------------
 *              0x05       | SMI130_ACC_SLEEP_DURN_0_5MS
 *              0x06       | SMI130_ACC_SLEEP_DURN_1MS
 *              0x07       | SMI130_ACC_SLEEP_DURN_2MS
 *              0x08       | SMI130_ACC_SLEEP_DURN_4MS
 *              0x09       | SMI130_ACC_SLEEP_DURN_6MS
 *              0x0A       | SMI130_ACC_SLEEP_DURN_10MS
 *              0x0B       | SMI130_ACC_SLEEP_DURN_25MS
 *              0x0C       | SMI130_ACC_SLEEP_DURN_50MS
 *              0x0D       | SMI130_ACC_SLEEP_DURN_100MS
 *              0x0E       | SMI130_ACC_SLEEP_DURN_500MS
 *              0x0F       | SMI130_ACC_SLEEP_DURN_1S
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_sleep_durn(uint8_t sleep_durn);
/*!
 * @brief This API is used to get the sleep timer mode
 *	in the register 0x12 bit 5
 *
 *
 *
 *
 *  @param  sleep_timer : The value of sleep timer mode
 *        sleep_timer |   result
 *       ----------------- | ----------------------
 *              0          | enable EventDrivenSampling(EDT)
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_sleep_timer_mode(
        uint8_t *sleep_timer);
/*!
 * @brief This API is used to set the sleep timer mode
 *	in the register 0x12 bit 5
 *
 *
 *
 *
 *  @param  sleep_timer : The value of sleep timer mode
 *        sleep_timer |   result
 *       ----------------- | ----------------------
 *              0          | enable EventDrivenSampling(EDT)
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_sleep_timer_mode(uint8_t sleep_timer);

/*!
 *	@brief This API is used to get shadow dis
 *	in the register 0x13 bit 6
 *
 *  @param  shadow_dis : The value of shadow dis
 *        shadow_dis  |   result
 *       ----------------- | ------------------
 *              0          | MSB is Locked
 *              1          | No MSB Locking
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_shadow_dis(uint8_t *shadow_dis);
/*!
 *	@brief This API is used to set shadow dis
 *	in the register 0x13 bit 6
 *
 *  @param  shadow_dis : The value of shadow dis
 *        shadow_dis  |   result
 *       ----------------- | ------------------
 *              0          | MSB is Locked
 *              1          | No MSB Locking
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_shadow_dis(uint8_t shadow_dis);
/******************************************/
/**\name FUNCTION FOR  SOFT RESET */
/******************************************/
/*!
 *	@brief This function is used for the soft reset
 *	The soft reset register will be written
 *	with 0xB6 in the register 0x14.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_soft_rst(void);
/******************************************/
/**\name FUNCTION FOR  INTERRUPT ENABLE */
/******************************************/
/*!
 *	@brief This API is used to get
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note It reads the flat enable, orient enable,
 *	@note single tap enable, double tap enable
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note high-z enable, high-y enable
 *	@note high-z enable
 *
 *
 *
 *  @param intr_type: The value of interrupts
 *        intr_type   |   result
 *       ----------------- | ------------------
 *              4          | SMI130_ACC_DATA_ENABLE
 *              5          | SLOPE_X_INTR
 *              6          | SLOPE_Y_INTR
 *              7          | SLOPE_Z_INTR
 *
 *  @param value : The value of interrupts enable
 *        value       |   result
 *       ----------------- | ------------------
 *              0x00       | INTR_DISABLE
 *              0x01       | INTR_ENABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_enable(uint8_t intr_type,
                uint8_t *value);
/*!
 *	@brief This API is used to set
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note It reads the flat enable, orient enable,
 *	@note single tap enable, double tap enable
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note high-z enable, high-y enable
 *	@note high-z enable
 *
 *
 *
 *  @param intr_type: The value of interrupts
 *        intr_type   |   result
 *       ----------------- | ------------------
 *              4          | SMI130_ACC_DATA_ENABLE
 *              5          | SLOPE_X_INTR
 *              6          | SLOPE_Y_INTR
 *              7          | SLOPE_Z_INTR
 *
 *  @param value : The value of interrupts enable
 *        value       |   result
 *       ----------------- | ------------------
 *              0x00       | INTR_DISABLE
 *              0x01       | INTR_ENABLE
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_intr_enable(uint8_t intr_type,
                uint8_t value);

/*!
 * @brief This API is used to get
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
 * @note INTR1_slope -> register 0x19 bit 2
 * @note INTR2_slope -> register 0x1B bit 2
 *
 *
 *
 * @param channel: the value of slope channel select
 *        channel     |   result
 *       ----------------- | ------------------
 *              1          | SMI130_ACC_INTR2_SLOPE
 *
 * @param intr_slope : The slope value enable value
 *        intr_slope         |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_slope(uint8_t channel,
                uint8_t *intr_slope);
/*!
 * @brief This API is used to set
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
 * @note INTR1_slope -> register 0x19 bit 2
 * @note INTR2_slope -> register 0x1B bit 2
 *
 *
 *
 * @param channel: the value of slope channel select
 *        channel     |   result
 *       ----------------- | ------------------
 *              1          | SMI130_ACC_INTR2_SLOPE
 *
 * @param intr_slope : The slope value enable value
 *        intr_slope         |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_intr_slope(uint8_t channel,
                uint8_t intr_slope);

/*!
 * @brief This API is used to get
 * the interrupt status of new data in the register 0x19
 * @note INTR1_data -> register 0x19 bit 0
 * @note INTR2_data -> register 0x19 bit 7
 *
 *
 *
 *  @param channel: The value of new data interrupt select
 *        channel     |   result
 *       ----------------- | ------------------
 *              1          | SMI130_ACC_INTR2_NEWDATA
 *
 *	@param intr_newdata: The new data interrupt enable value
 *       intr_newdata          |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_new_data(uint8_t channel,
                uint8_t *intr_newdata);
/*!
 * @brief This API is used to set
 * the interrupt status of new data in the register 0x19
 * @note INTR1_data -> register 0x19 bit 0
 * @note INTR2_data -> register 0x19 bit 7
 *
 *
 *
 *  @param channel: The value of new data interrupt select
 *        channel     |   result
 *       ----------------- | ------------------
 *              1          | SMI130_ACC_INTR2_NEWDATA
 *
 *	@param intr_newdata: The new data interrupt enable value
 *       intr_newdata          |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_new_data(uint8_t channel,
                uint8_t intr_newdata);

/******************************************/
/**\name FUNCTION FOR  SOURCE CONFIGURATION */
/******************************************/
/*!
 *	@brief This API is used to get
 *	the source data status of source data,
 *	source slow no motion, source slope, source high
 *	and source low in the register 0x1E bit from 0 to 5
 *
 *
 *
 *  @param channel : The value of source select
 *       channel     |    result
 *       -----------------| ------------------
 *               2        | SMI130_ACC_SOURCE_SLOPE
 *               5        | SMI130_ACC_SOURCE_DATA
 *
 *	@param intr_source: The source status enable value
 *       intr_source         |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_source(uint8_t channel,
                uint8_t *intr_source);
/*!
 *	@brief This API is used to set
 *	the source data status of source data,
 *	source slow no motion, source slope, source high
 *	and source low in the register 0x1E bit from 0 to 5
 *
 *
 *
 *  @param channel : The value of source select
 *       channel     |    result
 *       -----------------| ------------------
 *               2        | SMI130_ACC_SOURCE_SLOPE
 *               5        | SMI130_ACC_SOURCE_DATA
 *
 *	@param intr_source: The source status enable value
 *       intr_source         |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_source(uint8_t channel,
                uint8_t intr_source);
/******************************************/
/**\name FUNCTION FOR   OUTPUT TYPE AND LEVEL*/
/******************************************/
/*!
 *	@brief This API is used to get
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *	@note INTR2 -> bit 3
 *
 *  @param channel: The value of output type select
 *       channel     |    result
 *       -----------------| ------------------
 *               1        | SMI130_ACC_INTR2_OUTPUT
 *
 *	@param intr_output_type: The value of output type select
 *       intr_source         |    result
 *       ------------------------ | ------------------
 *              0x01              | OPEN_DRAIN
 *              0x00              | PUSS_PULL
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_output_type(uint8_t channel,
                uint8_t *intr_output_type);
/*!
 *	@brief This API is used to set
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *	@note INTR2 -> bit 3
 *
 *  @param channel: The value of output type select
 *         channel   |    result
 *       -----------------| ------------------
 *               1        | SMI130_ACC_INTR2_OUTPUT
 *
 *	@param intr_output_type: The value of output type select
 *       intr_source         |    result
 *       ------------------------ | ------------------
 *              0x01              | OPEN_DRAIN
 *              0x00              | PUSS_PULL
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_intr_output_type(uint8_t channel,
                uint8_t intr_output_type);
/*!
 *	@brief This API is used to get
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *	@note INTR2 -> bit 2
 *
 *  @param channel: The value of Active Level select
 *       channel     |    result
 *       -----------------| ------------------
 *               1        | SMI130_ACC_INTR2_LEVEL
 *
 *  @param intr_level: The Active Level status enable value
 *       intr_level          |    result
 *       ------------------------ | ------------------
 *              0x01              | ACTIVE_HIGH
 *              0x00              | ACTIVE_LOW
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_level(uint8_t channel,
                uint8_t *intr_level);
/*!
 *	@brief This API is used to set
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *	@note INTR2 -> bit 2
 *
 *  @param channel: The value of Active Level select
 *       channel     |    result
 *       -----------------| ------------------
 *               1        | SMI130_ACC_INTR2_LEVEL
 *
 *  @param intr_level: The Active Level status enable value
 *       intr_level          |    result
 *       ------------------------ | ------------------
 *              0x01              | ACTIVE_HIGH
 *              0x00              | ACTIVE_LOW
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_intr_level(uint8_t channel,
                uint8_t intr_level);
/******************************************/
/**\name FUNCTION FOR  RESET INTERRUPT*/
/******************************************/
/*!
 *	@brief This API is used to set
 *	the reset interrupt in the register 0x21 bit 7
 *
 *
 *
 *  @param  rst_intr: The value of reset interrupt
 *          rst_intr         |  result
 *       ------------------------ | ------------------
 *              0x01              | clear any latch interrupt
 *              0x00              | keep latch interrupt active
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_rst_intr(uint8_t rst_intr);
/******************************************/
/**\name FUNCTION FOR   LATCH INTERRUPT INTERRUPT*/
/******************************************/
/*!
 *	@brief This API is used to get
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *	@param latch_intr: The value of latch duration
 *        latch_intr |  result
 *       -----------------| ------------------
 *               0x00     | SMI130_ACC_LATCH_DURN_NON_LATCH
 *               0x01     | SMI130_ACC_LATCH_DURN_250MS
 *               0x02     | SMI130_ACC_LATCH_DURN_500MS
 *               0x03     | SMI130_ACC_LATCH_DURN_1S
 *               0x04     | SMI130_ACC_LATCH_DURN_2S
 *               0x05     | SMI130_ACC_LATCH_DURN_4S
 *               0x06     | SMI130_ACC_LATCH_DURN_8S
 *               0x07     | SMI130_ACC_LATCH_DURN_LATCH
 *               0x08     | SMI130_ACC_LATCH_DURN_NON_LATCH1
 *               0x09     | SMI130_ACC_LATCH_DURN_250US
 *               0x0A     | SMI130_ACC_LATCH_DURN_500US
 *               0x0B     | SMI130_ACC_LATCH_DURN_1MS
 *               0x0C     | SMI130_ACC_LATCH_DURN_12_5MS
 *               0x0D     | SMI130_ACC_LATCH_DURN_25MS
 *               0x0E     | SMI130_ACC_LATCH_DURN_50MS
 *               0x0F     | SMI130_ACC_LATCH_DURN_LATCH1
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_latch_intr(uint8_t *latch_intr);
/*!
 *	@brief This API is used to set
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *	@param latch_intr: The value of latch duration
 *        latch_intr |  result
 *       -----------------| ------------------
 *               0x00     | SMI130_ACC_LATCH_DURN_NON_LATCH
 *               0x01     | SMI130_ACC_LATCH_DURN_250MS
 *               0x02     | SMI130_ACC_LATCH_DURN_500MS
 *               0x03     | SMI130_ACC_LATCH_DURN_1S
 *               0x04     | SMI130_ACC_LATCH_DURN_2S
 *               0x05     | SMI130_ACC_LATCH_DURN_4S
 *               0x06     | SMI130_ACC_LATCH_DURN_8S
 *               0x07     | SMI130_ACC_LATCH_DURN_LATCH
 *               0x08     | SMI130_ACC_LATCH_DURN_NON_LATCH1
 *               0x09     | SMI130_ACC_LATCH_DURN_250US
 *               0x0A     | SMI130_ACC_LATCH_DURN_500US
 *               0x0B     | SMI130_ACC_LATCH_DURN_1MS
 *               0x0C     | SMI130_ACC_LATCH_DURN_12_5MS
 *               0x0D     | SMI130_ACC_LATCH_DURN_25MS
 *               0x0E     | SMI130_ACC_LATCH_DURN_50MS
 *               0x0F     | SMI130_ACC_LATCH_DURN_LATCH1
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_latch_intr(uint8_t latch_intr);
/******************************************/
/**\name FUNCTION FOR   INTERRUPT DURATION CONFIGURATION*/
/******************************************/
/*!
 *	@brief This API is used to get the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_DURN		-> register 0x22 bit form 0 to 7
 *	@note HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	@note SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *  @param channel: The value of duration select
 *     channel   | result
 *   -----------------| ------------------
 *               2    | SMI130_ACC_SLOPE_DURN
 *
 *	@param durn: The value of duration
 *
 *	@note :
 *     Duration           |    result
 * -----------------------| ------------------
 * SMI130_ACC_SLOPE_DURN| slope interrupt trigger
 *         -              | if[durn<1:0>+1] consecutive data points
 *         -              | are above the slope interrupt threshold
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_durn(uint8_t channel,
                uint8_t *durn);
/*!
 *	@brief This API is used to set the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_DURN		-> register 0x22 bit form 0 to 7
 *	@note HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	@note SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *  @param channel: The value of duration select
 *     channel   | result
 *   -----------------| ------------------
 *               2    | SMI130_ACC_SLOPE_DURN
 *
 *	@param durn: The value of duration
 *
 *	@note :
 *     Duration           |    result
 * -----------------------| ------------------
 * SMI130_ACC_SLOPE_DURN| slope interrupt trigger
 *         -              | if[durn<1:0>+1] consecutive data points
 *         -              | are above the slope interrupt threshold
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_durn(uint8_t channel,
                uint8_t durn);
/******************************************/
/**\name FUNCTION FOR  INTERRUPT THRESHOLD CONFIGURATION */
/******************************************/
/*!
 * @brief This API is used to get the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_THRES		-> register 0x23 bit form 0 to 7
 *	@note HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	@note SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *  @param channel: The value of threshold selection
 *     channel   | result
 *   -----------------| ------------------
 *               2    | SMI130_ACC_SLOPE_THRES
 *
 *  @param thres: The threshold value of selected interrupts
 *
 *	@note : SLOPE THRESHOLD
 *	@note Threshold of slope interrupt according to accel g range
 *    g-range           |      Slope threshold
 *  --------------------|----------------------------
 *     2g               |    (thres * 3.19) mg
 *     4g               |    (thres * 7.81) mg
 *     8g               |    (thres * 15.63) mg
 *     16g              |    (thres * 31.25) mg
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_thres(uint8_t channel,
                uint8_t *thres);
/*!
 * @brief This API is used to set the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_THRES		-> register 0x23 bit form 0 to 7
 *	@note HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	@note SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *  @param channel: The value of threshold selection
 *     channel   | result
 *   -----------------| ------------------
 *               2    | SMI130_ACC_SLOPE_THRES
 *
 *  @param thres: The threshold value of selected interrupts
 *
 *	@note : SLOPE THRESHOLD
 *	@note Threshold of slope interrupt according to accel g range
 *    g-range           |      Slope threshold
 *  --------------------|----------------------------
 *     2g               |    (thres * 3.19) mg
 *     4g               |    (thres * 7.81) mg
 *     8g               |    (thres * 15.63) mg
 *     16g              |    (thres * 31.25) mg
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_thres(uint8_t channel,
                uint8_t thres);

/******************************************/
/**\name FUNCTION FOR SELFTEST   */
/******************************************/
/*!
 *	@brief This API is for to get
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  @param selftest_axis : The value of selftest axis
 *     selftest_axis     |    result
 *  ------------------------- |------------------
 *     0x00                   | self test disable
 *     0x01                   | x-axis
 *     0x02                   | y-axis
 *     0x03                   | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_selftest_axis(
        uint8_t *selftest_axis);
/*!
 *	@brief This API is for to set
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  @param selftest_axis : The value of selftest axis
 *     selftest_axis     |    result
 *  ------------------------- |------------------
 *     0x00                   | self test disable
 *     0x01                   | x-axis
 *     0x02                   | y-axis
 *     0x03                   | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_selftest_axis(
        uint8_t selftest_axis);
/*!
 *	@brief This API is for to get
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  @param selftest_sign : The value of self test sign
 *     selftest_sign     |    result
 *  ------------------------- |------------------
 *     0x00                   | negative sign
 *     0x01                   | positive sign
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_selftest_sign(
        uint8_t *selftest_sign);
/*!
 *	@brief This API is for to set
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  @param selftest_sign : The value of self test sign
 *     selftest_sign     |    result
 *  ------------------------- |------------------
 *     0x00                   | negative sign
 *     0x01                   | positive sign
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_selftest_sign(
        uint8_t selftest_sign);
/*!
 *	@brief This API is for to set
 *	the Self Test sign(selftest_amp) in the register 0x32 bit 4
 *
 *
 *
 *  @param selftest_amp : The value of self test amp
 *     selftest_amp        |    result
 *  ------------------------- |------------------
 *     0x00                   | low
 *     0x01                   | high
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_selftest_amp(
        uint8_t selftest_amp);


/*!
 *	@brief This API is used to get the i2c
 *	watch dog timer period and I2C interface mode is selected
 *	in the register 0x34 bit 1 and 2
 *
 *
 *  @param channel: The i2c option selection
 *     channel           |    result
 *  ------------------------- |------------------
 *        0                   |   SMI130_ACC_I2C_SELECT
 *        1                   |   SMI130_ACC_I2C_ENABLE
 *
 *  @param i2c_wdt: watch dog timer period
 *	and I2C interface mode is selected
 *     SMI130_ACC_I2C_SELECT|    result
 *  ------------------------- |------------------
 *     0x00                   | Disable the watchdog at SDI pin
 *     0x01                   | Enable watchdog
 *
 *     SMI130_ACC_I2C_ENABLE      |    result
 *  ------------------------- |------------------
 *     0x00                   | 1ms
 *     0x01                   | 50ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_i2c_wdt(uint8_t channel,
                uint8_t *i2c_wdt);
/*!
 *	@brief This API is used to set the i2c
 *	watch dog timer period and I2C interface mode is selected
 *	in the register 0x34 bit 1 and 2
 *
 *
 *  @param channel: The i2c option selection
 *     channel           |    result
 *  ------------------------- |------------------
 *        0                   |   SMI130_ACC_I2C_SELECT
 *        1                   |   SMI130_ACC_I2C_ENABLE
 *
 *  @param i2c_wdt: watch dog timer period
 *	and I2C interface mode is selected
 *     SMI130_ACC_I2C_SELECT|    result
 *  ------------------------- |------------------
 *     0x00                   | Disable the watchdog at SDI pin
 *     0x01                   | Enable watchdog
 *
 *     SMI130_ACC_I2C_ENABLE      |    result
 *  ------------------------- |------------------
 *     0x00                   | 1ms
 *     0x01                   | 50ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_i2c_wdt(uint8_t channel,
                uint8_t i2c_wdt);

/*!
 *	@brief This API is used to get
 *	the status of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)
 *
 *
 *
 *  @param  cal_rdy: The value of cal_ready
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_cal_rdy(uint8_t *cal_rdy);
/*!
 *	@brief This API is used to set
 *	the status of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)
 *
 *
 *
 *  @param  cal_trigger: The value of cal_ready
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_cal_trigger(uint8_t cal_trigger);
/*!
 *	@brief This API is used to set
 *	the offset reset(offset_reset) in the register 0x36
 *	bit 7(Write only possible)
 *
 *
 *
 *  @param  offset_rst: The offset reset value
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_offset_rst(uint8_t offset_rst);
/*!
 *	@brief This API is used to get
 *	the status of offset target axis(offset_target_x, offset_target_y and
 *	offset_target_z) and cut_off in the register 0x37
 *	@note CUT_OFF -> bit 0
 *	@note OFFSET_TRIGGER_X -> bit 1 and 2
 *	@note OFFSET_TRIGGER_Y -> bit 3 and 4
 *	@note OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *
 *  @param channel: The value of offset axis selection
 *     channel           |    result
 *  ------------------------- |------------------
 *        0                   |   SMI130_ACC_CUT_OFF
 *        1                   |   SMI130_ACC_OFFSET_TRIGGER_X
 *        2                   |   SMI130_ACC_OFFSET_TRIGGER_Y
 *        2                   |   SMI130_ACC_OFFSET_TRIGGER_Z
 *
 *  @param  offset: The offset target value
 *     CUT_OFF                |    result
 *  ------------------------- |------------------
 *        0                   |   1Hz
 *        1                   |   10Hz
 *
 *
 *     OFFSET_TRIGGER         |    result
 *  ------------------------- |------------------
 *        0x00                |   0g
 *        0x01                |   +1g
 *        0x02                |   -1g
 *        0x03                |   0g
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_offset_target(uint8_t channel,
                uint8_t *offset);
/*!
 *	@brief This API is used to set
 *	the status of offset target axis(offset_target_x, offset_target_y and
 *	offset_target_z) and cut_off in the register 0x37
 *	@note CUT_OFF -> bit 0
 *	@note OFFSET_TRIGGER_X -> bit 1 and 2
 *	@note OFFSET_TRIGGER_Y -> bit 3 and 4
 *	@note OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *
 *  @param channel: The value of offset axis selection
 *     channel           |    result
 *  ------------------------- |------------------
 *        0                   |   SMI130_ACC_CUT_OFF
 *        1                   |   SMI130_ACC_OFFSET_TRIGGER_X
 *        2                   |   SMI130_ACC_OFFSET_TRIGGER_Y
 *        2                   |   SMI130_ACC_OFFSET_TRIGGER_Z
 *
 *  @param  offset: The offset target value
 *     CUT_OFF                |    result
 *  ------------------------- |------------------
 *        0                   |   1Hz
 *        1                   |   10Hz
 *
 *
 *     OFFSET_TRIGGER         |    result
 *  ------------------------- |------------------
 *        0x00                |   0g
 *        0x01                |   +1g
 *        0x02                |   -1g
 *        0x03                |   0g
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_offset_target(uint8_t channel,
                uint8_t offset);
/*!
 *	@brief This API is used to get the status of offset
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	@note offset_x -> register 0x38 bit 0 to 7
 *	@note offset_y -> register 0x39 bit 0 to 7
 *	@note offset_z -> register 0x3A bit 0 to 7
 *
 *
 *  @param channel: The value of offset selection
 *     channel           |    result
 *  ------------------------- |------------------
 *        0                   |   SMI130_ACC_X_AXIS
 *        1                   |   SMI130_ACC_Y_AXIS
 *        2                   |   SMI130_ACC_Z_AXIS
 *
 *  @param offset: The value of offset
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_offset(uint8_t channel,
                int8_t *offset);
/*!
 *	@brief This API is used to set the status of offset
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	@note offset_x -> register 0x38 bit 0 to 7
 *	@note offset_y -> register 0x39 bit 0 to 7
 *	@note offset_z -> register 0x3A bit 0 to 7
 *
 *
 *  @param channel: The value of offset selection
 *     channel           |    result
 *  ------------------------- |------------------
 *        0                   |   SMI130_ACC_X_AXIS
 *        1                   |   SMI130_ACC_Y_AXIS
 *        2                   |   SMI130_ACC_Z_AXIS
 *
 *  @param offset: The value of offset
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_offset(uint8_t channel,
                int8_t offset);

/******************************************/
/**\name FUNCTION FOR  TEMPERATURE DATA READ */
/******************************************/
/*!
 * @brief This API is used to read the temp
 * from register 0x08
 *
 *
 *
 *  @param  temp_s8: The value of temperature
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_temp(int8_t *temp_s8);
/******************************************/
/**\name FUNCTION FOR  DATA XYZ WITH TEMPERATURE */
/******************************************/
/*!
 * @brief This API reads accelerometer data X,Y,Z values and
 * temperature data from location 02h to 08h
 *
 *
 *
 *
 *  @param accel : The value of accel xyz and temperature data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_xyzt(
        struct smi130_acc_data_temp *accel);
/*!
 *	@brief This API reads acceleration data X,Y,Z values
 *	from location 02h to 07h
 *
 *  @param accel : pointer holding the data of accel
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_xyz(
        struct smi130_acc_data *accel);
/*!
 * @brief
 *	This API reads acceleration data X values
 *	from location 02h and 03h
 *
 *
 *  @param   accel_x_s16 : pointer holding the data of accel X
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_x(int16_t *accel_x_s16);
/*!
 * @brief
 *	This API reads acceleration data Y values
 *	from location 04h and 05h
 *
 *  @param   accel_y_s16 : pointer holding the data of accel Y
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_y(int16_t *accel_y_s16);
/*!
 * @brief This API reads acceleration data Z values
 *                          from location 06h and 07h
 *
 *
 *  @param   accel_z_s16 : pointer holding the data of accel Z
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMI130_ACC_12_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_z(int16_t *accel_z_s16);

#endif

