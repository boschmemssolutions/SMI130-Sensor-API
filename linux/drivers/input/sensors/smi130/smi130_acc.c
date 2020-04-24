// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
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
/*! file <SMI130_ACC >
    brief <Sensor driver for SMI130_ACC> */
#include "smi130_acc.h"
/*! user defined code to be added here ... */
static struct smi130_acc_t *p_smi130_acc;
/*! Based on Bit resolution value should be modified */
uint8_t V_SMI130_ACC_RESOLUTION_U8 = SMI130_ACC_12_RESOLUTION;

/****************************************************************************/
/*!	Static Function Declarations
*****************************************************************************/
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
                uint8_t *data, uint32_t len)
{
	/* Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the data from the register*/
		com_rslt = p_smi130_acc->SMI130_ACC_BURST_READ_FUNC
		           (p_smi130_acc->dev_addr, addr, data, len);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_init(struct smi130_acc_t *smi130_acc)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/* assign smi130_acc ptr */
	p_smi130_acc = smi130_acc;
	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		/* read Chip Id */
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_CHIP_ID_REG, &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		p_smi130_acc->chip_id = data;    /* get bit slice */
	}
	return com_rslt;
}
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
                uint8_t *data, uint8_t len)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Write the data to the register*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_WRITE_FUNC
		           (p_smi130_acc->dev_addr, adr, data, len);

		if (p_smi130_acc->power_mode != SMI130_ACC_MODE_NORMAL) {
			/*A minimum interface idle time delay
			of atleast 450us is required as per the data sheet.*/
			p_smi130_acc->delay_msec(SMI130_ACC_INTERFACE_IDLE_TIME_DELAY);
		}
	}
	return com_rslt;
}
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
                uint8_t *data, uint8_t len)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/*Read the data from the register*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, adr, data, len);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_x(int16_t *accel_x_s16)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel x value
	data[0] - x->LSB
	data[1] - x->MSB
	*/
	uint8_t	data[SMI130_ACC_DATA_SIZE] = {
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE
	};
	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* This case used for the resolution bit 12*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_X12_LSB_REG, data,
		            SMI130_ACC_LSB_MSB_READ_LENGTH);
		*accel_x_s16 = (int16_t)((((int32_t)((int8_t)
		                                     data[SMI130_ACC_SENSOR_DATA_ACCEL_MSB]))
		                          << SMI130_ACC_SHIFT_EIGHT_BITS) |
		                         (data[SMI130_ACC_SENSOR_DATA_ACCEL_LSB] &
		                          SMI130_ACC_RESOLUTION_12_MASK));
		*accel_x_s16 = *accel_x_s16 >>
		               SMI130_ACC_SHIFT_FOUR_BITS;

	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_y(int16_t *accel_y_s16)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel y value
	data[0] - y->LSB
	data[1] - y->MSB
	*/
	uint8_t data[SMI130_ACC_DATA_SIZE] = {SMI130_ACC_INIT_VALUE,
	                                      SMI130_ACC_INIT_VALUE
	                                     };

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_Y12_LSB_REG, data,
		            SMI130_ACC_LSB_MSB_READ_LENGTH);
		*accel_y_s16 = (int16_t)((((int32_t)((int8_t)
		                                     data[SMI130_ACC_SENSOR_DATA_ACCEL_MSB]))
		                          << SMI130_ACC_SHIFT_EIGHT_BITS) |
		                         (data[SMI130_ACC_SENSOR_DATA_ACCEL_LSB] &
		                          SMI130_ACC_12_BIT_SHIFT));
		*accel_y_s16 = *accel_y_s16 >>
		               SMI130_ACC_SHIFT_FOUR_BITS;

	}
	return com_rslt;
}

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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_z(int16_t *accel_z_s16)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel z value
	data[0] - z->LSB
	data[1] - z->MSB
	*/
	uint8_t data[SMI130_ACC_DATA_SIZE] = {SMI130_ACC_INIT_VALUE,
	                                      SMI130_ACC_INIT_VALUE
	                                     };

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* This case used for the resolution bit 12*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_Z12_LSB_REG, data,
		            SMI130_ACC_LSB_MSB_READ_LENGTH);
		*accel_z_s16 = (int16_t)((((int32_t)((int8_t)
		                                     data[SMI130_ACC_SENSOR_DATA_ACCEL_MSB]))
		                          << SMI130_ACC_SHIFT_EIGHT_BITS) |
		                         (data[SMI130_ACC_SENSOR_DATA_ACCEL_LSB]
		                          & SMI130_ACC_12_BIT_SHIFT));
		*accel_z_s16 = *accel_z_s16 >>
		               SMI130_ACC_SHIFT_FOUR_BITS;

	}
	return com_rslt;
}

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
        struct smi130_acc_data *accel)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel xyz value
	data[0] - x->LSB
	data[1] - x->MSB
	data[2] - y->MSB
	data[3] - y->MSB
	data[4] - z->MSB
	data[5] - z->MSB
	*/
	uint8_t data[SMI130_ACC_XYZ_DATA_SIZE] = {
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE,
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE,
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE
	};

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_X12_LSB_REG,
		            data, SMI130_ACC_SHIFT_SIX_BITS);
		/* read the x data*/
		accel->x = (int16_t)((((int32_t)((int8_t)
		                                 data[SMI130_ACC_SENSOR_DATA_XYZ_X_MSB]))
		                      << SMI130_ACC_SHIFT_EIGHT_BITS) |
		                     (data[SMI130_ACC_SENSOR_DATA_XYZ_X_LSB] &
		                      SMI130_ACC_12_BIT_SHIFT));
		accel->x = accel->x >> SMI130_ACC_SHIFT_FOUR_BITS;

		/* read the y data*/
		accel->y = (int16_t)((((int32_t)((int8_t)
		                                 data[SMI130_ACC_SENSOR_DATA_XYZ_Y_MSB]))
		                      << SMI130_ACC_SHIFT_EIGHT_BITS) |
		                     (data[SMI130_ACC_SENSOR_DATA_XYZ_Y_LSB] &
		                      SMI130_ACC_12_BIT_SHIFT));
		accel->y = accel->y >> SMI130_ACC_SHIFT_FOUR_BITS;

		/* read the z data*/
		accel->z = (int16_t)((((int32_t)((int8_t)
		                                 data[SMI130_ACC_SENSOR_DATA_XYZ_Z_MSB]))
		                      << SMI130_ACC_SHIFT_EIGHT_BITS) |
		                     (data[SMI130_ACC_SENSOR_DATA_XYZ_Z_LSB] &
		                      SMI130_ACC_12_BIT_SHIFT));
		accel->z = accel->z >> SMI130_ACC_SHIFT_FOUR_BITS;

	}
	return com_rslt;
}

/*!
 *	@brief This API read slope-sign, slope-first-xyz
 *	status register byte from location 0Bh
 *
 *
 *   @param stat_slope : The status of slope
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_intr_slope_stat(
        uint8_t *stat_slope)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the interrupt status register 0x0B*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_STAT_SLOPE_ADDR,
		            stat_slope, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}

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
        uint8_t *intr_stat)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the interrupt status register 0x09*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC(
		                   p_smi130_acc->dev_addr,
		                   SMI130_ACC_STAT1_ADDR, intr_stat,
		                   SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_range(uint8_t *range)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the range register 0x0F*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC(p_smi130_acc->dev_addr,
		                SMI130_ACC_RANGE_SELECT_REG, &data,
		                SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_GET_BITSLICE(data, SMI130_ACC_RANGE_SELECT);
		*range = data;
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range : The value of range
 *		  range |   result
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_range(uint8_t range)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		if ((range == SMI130_ACC_RANGE_2G) ||
		    (range == SMI130_ACC_RANGE_4G) ||
		    (range == SMI130_ACC_RANGE_8G) ||
		    (range == SMI130_ACC_RANGE_16G)) {
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_RANGE_SELECT_REG, &data,
			            SMI130_ACC_GEN_READ_WRITE_LENGTH);
			switch (range) {
			case SMI130_ACC_RANGE_2G:
				data  = SMI130_ACC_SET_BITSLICE(data,
				                                SMI130_ACC_RANGE_SELECT,
				                                SMI130_ACC_RANGE_2G);
				break;
			case SMI130_ACC_RANGE_4G:
				data  = SMI130_ACC_SET_BITSLICE(data,
				                                SMI130_ACC_RANGE_SELECT,
				                                SMI130_ACC_RANGE_4G);
				break;
			case SMI130_ACC_RANGE_8G:
				data  = SMI130_ACC_SET_BITSLICE(data,
				                                SMI130_ACC_RANGE_SELECT,
				                                SMI130_ACC_RANGE_8G);
				break;
			case SMI130_ACC_RANGE_16G:
				data  = SMI130_ACC_SET_BITSLICE(data,
				                                SMI130_ACC_RANGE_SELECT,
				                                SMI130_ACC_RANGE_16G);
				break;
			default:
				break;
			}
			/* Write the range register 0x0F*/
			com_rslt += smi130_acc_write_reg(SMI130_ACC_RANGE_SELECT_REG,
			                                 &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *  @brief This API is used to get the bandwidth of the sensor in the register
 *  0x10 bit from 0 to 4
 *
 *
 *  @param bw : The value of bandwidth
 *          bw          |   result
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
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 *
*/
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_bw(uint8_t *bw)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the bandwidth register 0x10*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_BW_REG, &data,
		            SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_GET_BITSLICE(data, SMI130_ACC_BW);
		*bw = data;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the bandwidth of the sensor
 *      in the register
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_bw(uint8_t bw)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;
	uint8_t data_bw = SMI130_ACC_INIT_VALUE;
	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		/* Check the chip id 0xFB, it support upto 500Hz*/
		if (p_smi130_acc->chip_id == BANDWIDTH_DEFINE) {
			if (bw > SMI130_ACC_BW_MIN_RANGE &&
			    bw < SMI130_ACC_BW_1000HZ_RANGE) {
				switch (bw) {
				case SMI130_ACC_BW_7_81HZ:
					data_bw = SMI130_ACC_BW_7_81HZ;

					/*  7.81 Hz      64000 uS   */
					break;
				case SMI130_ACC_BW_15_63HZ:
					data_bw = SMI130_ACC_BW_15_63HZ;

					/*  15.63 Hz     32000 uS   */
					break;
				case SMI130_ACC_BW_31_25HZ:
					data_bw = SMI130_ACC_BW_31_25HZ;

					/*  31.25 Hz     16000 uS   */
					break;
				case SMI130_ACC_BW_62_50HZ:
					data_bw = SMI130_ACC_BW_62_50HZ;

					/*  62.50 Hz     8000 uS   */
					break;
				case SMI130_ACC_BW_125HZ:
					data_bw = SMI130_ACC_BW_125HZ;

					/*  125 Hz       4000 uS   */
					break;
				case SMI130_ACC_BW_250HZ:
					data_bw = SMI130_ACC_BW_250HZ;

					/*  250 Hz       2000 uS   */
					break;
				case SMI130_ACC_BW_500HZ:
					data_bw = SMI130_ACC_BW_500HZ;

					/*  500 Hz       1000 uS   */
					break;
				default:
					break;
				}
				/* Write the bandwidth register */
				com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
				           (p_smi130_acc->dev_addr,
				            SMI130_ACC_BW_REG, &data,
				            SMI130_ACC_GEN_READ_WRITE_LENGTH);
				data = SMI130_ACC_SET_BITSLICE(data,
				                               SMI130_ACC_BW, data_bw);
				com_rslt += smi130_acc_write_reg
				            (SMI130_ACC_BW_REG, &data,
				             SMI130_ACC_GEN_READ_WRITE_LENGTH);
			} else {
				com_rslt = E_OUT_OF_RANGE;
			}
		} else {
			if (bw > SMI130_ACC_BW_MIN_RANGE &&
			    bw < SMI130_ACC_BW_MAX_RANGE) {
				switch (bw) {
				case SMI130_ACC_BW_7_81HZ:
					data_bw = SMI130_ACC_BW_7_81HZ;

					/*  7.81 Hz      64000 uS   */
					break;
				case SMI130_ACC_BW_15_63HZ:
					data_bw = SMI130_ACC_BW_15_63HZ;

					/*  15.63 Hz     32000 uS   */
					break;
				case SMI130_ACC_BW_31_25HZ:
					data_bw = SMI130_ACC_BW_31_25HZ;

					/*  31.25 Hz     16000 uS   */
					break;
				case SMI130_ACC_BW_62_50HZ:
					data_bw = SMI130_ACC_BW_62_50HZ;

					/*  62.50 Hz     8000 uS   */
					break;
				case SMI130_ACC_BW_125HZ:
					data_bw = SMI130_ACC_BW_125HZ;

					/*  125 Hz       4000 uS   */
					break;
				case SMI130_ACC_BW_250HZ:
					data_bw = SMI130_ACC_BW_250HZ;

					/*  250 Hz       2000 uS   */
					break;
				case SMI130_ACC_BW_500HZ:
					data_bw = SMI130_ACC_BW_500HZ;

					/*!  500 Hz       1000 uS   */
					break;
				case SMI130_ACC_BW_1000HZ:
					data_bw = SMI130_ACC_BW_1000HZ;

					/*  1000 Hz      500 uS   */
					break;
				default:
					break;
				}
				/* Write the bandwidth register */
				com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
				           (p_smi130_acc->dev_addr,
				            SMI130_ACC_BW_REG, &data,
				            SMI130_ACC_GEN_READ_WRITE_LENGTH);
				data = SMI130_ACC_SET_BITSLICE
				       (data, SMI130_ACC_BW, data_bw);
				com_rslt += smi130_acc_write_reg(
				                    SMI130_ACC_BW_REG, &data,
				                    SMI130_ACC_GEN_READ_WRITE_LENGTH);
			} else {
				com_rslt = E_OUT_OF_RANGE;
			}
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode : The value of power mode
 *	power_mode           	      | value |  0x11   |  0x12
 *  ------------------------------|-------| --------|--------
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
        uint8_t *power_mode)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;
	uint8_t data2 = SMI130_ACC_INIT_VALUE;
	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_MODE_CTRL_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		com_rslt += p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		            (p_smi130_acc->dev_addr, SMI130_ACC_LOW_NOISE_CTRL_ADDR,
		             &data2, SMI130_ACC_GEN_READ_WRITE_LENGTH);

		data  = (data &
		         SMI130_ACC_POWER_MODE_HEX_E_ZERO_MASK) >>
		        SMI130_ACC_SHIFT_FIVE_BITS;
		data2  = (data2 &
		          SMI130_ACC_POWER_MODE_HEX_4_ZERO_MASK) >>
		         SMI130_ACC_SHIFT_SIX_BITS;

		if ((data ==
		     SMI130_ACC_POWER_MODE_HEX_ZERO_ZERO_MASK) &&
		    (data2 ==
		     SMI130_ACC_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
			*power_mode  = SMI130_ACC_MODE_NORMAL;
		} else {
			if (((data &
			      SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK)
			     == SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK)) {
				*power_mode  =
				        SMI130_ACC_MODE_DEEP_SUSPEND;
			} else {
				if ((data ==
				     SMI130_ACC_POWER_MODE_HEX_ZERO_TWO_MASK)
				    && (data2 ==
				        SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK)) {
					*power_mode  =
					        SMI130_ACC_MODE_LOWPOWER2;
				} else {
					if ((data ==
					     SMI130_ACC_POWER_MODE_HEX_ZERO_FOUR_MASK) &&
					    (data2 ==
					     SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK))
						*power_mode  =
						        SMI130_ACC_MODE_STANDBY;
					else
						*power_mode =
						        SMI130_ACC_MODE_DEEP_SUSPEND;
				}
			}
		}
	}
	p_smi130_acc->power_mode = *power_mode;
	return com_rslt;
}
/*!
 *	@brief This API is used to set the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode : The value of power mode
 *	power_mode         		      |value  |   0x11  |   0x12
 *  ------------------------------|-------| --------|--------
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_power_mode(uint8_t power_mode)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t mode_ctr_eleven_reg = SMI130_ACC_INIT_VALUE;
	uint8_t mode_ctr_twel_reg = SMI130_ACC_INIT_VALUE;
	uint8_t data = SMI130_ACC_INIT_VALUE;
	uint8_t data2 = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC(p_smi130_acc->dev_addr,
		                SMI130_ACC_MODE_CTRL_REG, &data, 1);
		com_rslt += p_smi130_acc->SMI130_ACC_BUS_READ_FUNC(p_smi130_acc->dev_addr,
		                SMI130_ACC_LOW_POWER_MODE_REG, &data2, 1);


		com_rslt += smi130_acc_set_mode_value(power_mode);
		mode_ctr_eleven_reg = p_smi130_acc->ctrl_mode_reg;
		mode_ctr_twel_reg =  p_smi130_acc->low_mode_reg;

		/* write the power mode to the register 0x12*/
		data2  = SMI130_ACC_SET_BITSLICE(data2, SMI130_ACC_LOW_POWER_MODE,
		                                 mode_ctr_twel_reg);
		com_rslt += smi130_acc_write_reg(SMI130_ACC_LOW_POWER_MODE_REG,
		                                 &data2, 1);

		/*A minimum delay of atleast 450us is required for
		the low power modes, as per the data sheet.*/
		p_smi130_acc->delay_msec(SMI130_ACC_INTERFACE_IDLE_TIME_DELAY);

		if (((p_smi130_acc->power_mode == SMI130_ACC_MODE_LOWPOWER2)) &&
		    (power_mode == SMI130_ACC_MODE_NORMAL)) {
			/* Enter the power mode to suspend*/
			data  = SMI130_ACC_SET_BITSLICE(data,
			                                SMI130_ACC_MODE_CTRL, SMI130_ACC_SHIFT_FOUR_BITS);
			/* write the power mode to suspend*/
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_MODE_CTRL_REG, &data,
			                    SMI130_ACC_GEN_READ_WRITE_LENGTH);
		}

		/* write the power mode to 0x11 register*/
		data  = SMI130_ACC_SET_BITSLICE(data, SMI130_ACC_MODE_CTRL,
		                                mode_ctr_eleven_reg);
		com_rslt += smi130_acc_write_reg(SMI130_ACC_MODE_CTRL_REG, &data, 1);
		/*A minimum delay of atleast 450us is required for
		the low power modes, as per the data sheet.*/
		p_smi130_acc->delay_msec(SMI130_ACC_INTERFACE_IDLE_TIME_DELAY);

		/*Assigning the power mode to the global variable*/
		p_smi130_acc->power_mode = power_mode;
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_mode_value(uint8_t power_mode)
{
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		if (power_mode < SMI130_ACC_POWER_MODE_RANGE) {
			switch (power_mode)	{
			case SMI130_ACC_MODE_NORMAL:
				p_smi130_acc->ctrl_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_ZERO_MASK;
				p_smi130_acc->low_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_ZERO_MASK;
				break;
			case SMI130_ACC_MODE_LOWPOWER2:
				p_smi130_acc->ctrl_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_TWO_MASK;
				p_smi130_acc->low_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK;
				break;
			case SMI130_ACC_MODE_STANDBY:
				p_smi130_acc->ctrl_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_FOUR_MASK;
				p_smi130_acc->low_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK;
				break;
			case SMI130_ACC_MODE_DEEP_SUSPEND:
				p_smi130_acc->ctrl_mode_reg =
				        SMI130_ACC_POWER_MODE_HEX_ZERO_ONE_MASK;
				break;
			}
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the sleep duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *  @param  sleep_durn : The value of sleep duration time
 *         sleep_durn |   result
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_sleep_durn(uint8_t *sleep_durn)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* read the sleep duration */
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_SLEEP_DURN_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*sleep_durn = SMI130_ACC_GET_BITSLICE
		              (data, SMI130_ACC_SLEEP_DURN);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_sleep_durn(uint8_t sleep_durn)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data_sleep_durn = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		if (sleep_durn > SMI130_ACC_SLEEP_DURN_MIN_RANGE &&
		    sleep_durn < SMI130_ACC_SLEEP_DURN_MAX_RANGE) {
			switch (sleep_durn) {
			case SMI130_ACC_SLEEP_DURN_0_5MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_0_5MS;

				/*  0.5 MS   */
				break;
			case SMI130_ACC_SLEEP_DURN_1MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_1MS;

				/*  1 MS  */
				break;
			case SMI130_ACC_SLEEP_DURN_2MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_2MS;

				/*  2 MS  */
				break;
			case SMI130_ACC_SLEEP_DURN_4MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_4MS;

				/*  4 MS   */
				break;
			case SMI130_ACC_SLEEP_DURN_6MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_6MS;

				/*  6 MS  */
				break;
			case SMI130_ACC_SLEEP_DURN_10MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_10MS;

				/*  10 MS  */
				break;
			case SMI130_ACC_SLEEP_DURN_25MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_25MS;

				/*  25 MS  */
				break;
			case SMI130_ACC_SLEEP_DURN_50MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_50MS;

				/*  50 MS   */
				break;
			case SMI130_ACC_SLEEP_DURN_100MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_100MS;

				/*  100 MS  */
				break;
			case SMI130_ACC_SLEEP_DURN_500MS:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_500MS;

				/*  500 MS   */
				break;
			case SMI130_ACC_SLEEP_DURN_1S:
				data_sleep_durn = SMI130_ACC_SLEEP_DURN_1S;

				/*!  1 SECS   */
				break;
			default:
				break;
			}
			/* write the sleep duration */
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr, SMI130_ACC_SLEEP_DURN_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_SLEEP_DURN, data_sleep_durn);
			com_rslt += smi130_acc_write_reg(SMI130_ACC_SLEEP_DURN_REG,
			                                 &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
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
        uint8_t *sleep_timer)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/*Read the SLEEP TIMER MODE*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_SLEEP_TIMER_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*sleep_timer = SMI130_ACC_GET_BITSLICE
		               (data, SMI130_ACC_SLEEP_TIMER);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_sleep_timer_mode(uint8_t sleep_timer)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		if (sleep_timer < SMI130_ACC_SLEEP_TIMER_MODE_RANGE) {
			/* write the SLEEP TIMER MODE*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr, SMI130_ACC_SLEEP_TIMER_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_SLEEP_TIMER, sleep_timer);
			com_rslt += smi130_acc_write_reg(SMI130_ACC_SLEEP_TIMER_REG,
			                                 &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get high bandwidth
 *		in the register 0x13 bit 7
 *
 *  @param  high_bw : The value of high bandwidth
 *         high_bw    |   result
 *       ----------------- | ----------------------
 *              0          | Unfiltered High Bandwidth
 *              1          | Filtered Low Bandwidth
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_high_bw(uint8_t *high_bw)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		return  E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the high bandwidth*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_ENABLE_DATA_HIGH_BW_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*high_bw = SMI130_ACC_GET_BITSLICE
		           (data, SMI130_ACC_ENABLE_DATA_HIGH_BW);
	}
	return com_rslt;
}
/*!
 * @brief This API is used to write high bandwidth
 *		in the register 0x13 bit 7
 *
 *  @param  high_bw : The value of high bandwidth
 *         high_bw    |   result
 *       ----------------- | ----------------------
 *              0          | Unfiltered High Bandwidth
 *              1          | Filtered Low Bandwidth
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_high_bw(uint8_t high_bw)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		return  E_SMI130_ACC_NULL_PTR;
	}  else {
		/* Write the high bandwidth*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_ENABLE_DATA_HIGH_BW_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_SET_BITSLICE(data,
		                               SMI130_ACC_ENABLE_DATA_HIGH_BW, high_bw);
		com_rslt += smi130_acc_write_reg(
		                    SMI130_ACC_ENABLE_DATA_HIGH_BW_REG,
		                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_shadow_dis(uint8_t *shadow_dis)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		return  E_SMI130_ACC_NULL_PTR;
	} else {
		/*Read the shadow dis*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_DIS_SHADOW_PROC_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*shadow_dis = SMI130_ACC_GET_BITSLICE
		              (data, SMI130_ACC_DIS_SHADOW_PROC);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_shadow_dis(uint8_t shadow_dis)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		return  E_SMI130_ACC_NULL_PTR;
	} else {
		/* Write the shadow dis*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_DIS_SHADOW_PROC_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_SET_BITSLICE
		       (data, SMI130_ACC_DIS_SHADOW_PROC, shadow_dis);
		com_rslt += smi130_acc_write_reg(SMI130_ACC_DIS_SHADOW_PROC_REG,
		                                 &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
/*!
 *	@brief This function is used for the soft reset
 *	The soft reset register will be written
 *	with 0xB6 in the register 0x14.
 *
 *
 *
 *  \param : None
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_soft_rst(void)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_ENABLE_SOFT_RESET_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	}  else {
		/*! To reset the sensor
		0xB6 value will be written */
		com_rslt = smi130_acc_write_reg(SMI130_ACC_RST_ADDR,
		                                &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note slope-x enable, slope-y enable, slope-z enable,
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
                uint8_t *value)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (intr_type) {
		case SMI130_ACC_DATA_ENABLE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_NEW_DATA_INTR_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*value = SMI130_ACC_GET_BITSLICE
			         (data, SMI130_ACC_ENABLE_NEW_DATA_INTR);
			break;
		case SMI130_ACC_SLOPE_X_INTR:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_SLOPE_X_INTR_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*value = SMI130_ACC_GET_BITSLICE
			         (data, SMI130_ACC_ENABLE_SLOPE_X_INTR);
			break;
		case SMI130_ACC_SLOPE_Y_INTR:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_SLOPE_Y_INTR_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*value = SMI130_ACC_GET_BITSLICE
			         (data, SMI130_ACC_ENABLE_SLOPE_Y_INTR);
			break;
		case SMI130_ACC_SLOPE_Z_INTR:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_SLOPE_Z_INTR_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*value = SMI130_ACC_GET_BITSLICE
			         (data, SMI130_ACC_ENABLE_SLOPE_Z_INTR);
			break;
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                uint8_t value)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data = SMI130_ACC_INIT_VALUE;
	uint8_t data2 = SMI130_ACC_INIT_VALUE;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_INTR_ENABLE1_ADDR,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		com_rslt += p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		            (p_smi130_acc->dev_addr, SMI130_ACC_INTR_ENABLE2_ADDR,
		             &data2, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		value = value & SMI130_ACC_GEN_READ_WRITE_LENGTH;
		switch (intr_type) {
		case SMI130_ACC_DATA_ENABLE:
			/*Data En Interrupt  */
			data2 = SMI130_ACC_SET_BITSLICE(data2,
			                                SMI130_ACC_ENABLE_NEW_DATA_INTR, value);
			break;
		case SMI130_ACC_SLOPE_X_INTR:
			/* Slope X Interrupt */
			data = SMI130_ACC_SET_BITSLICE(data,
			                               SMI130_ACC_ENABLE_SLOPE_X_INTR, value);
			break;
		case SMI130_ACC_SLOPE_Y_INTR:
			/* Slope Y Interrupt */
			data = SMI130_ACC_SET_BITSLICE(data,
			                               SMI130_ACC_ENABLE_SLOPE_Y_INTR, value);
			break;
		case SMI130_ACC_SLOPE_Z_INTR:
			/* Slope Z Interrupt */
			data = SMI130_ACC_SET_BITSLICE(data,
			                               SMI130_ACC_ENABLE_SLOPE_Z_INTR, value);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
		/* write the interrupt*/
		com_rslt += smi130_acc_write_reg
		            (SMI130_ACC_INTR_ENABLE1_ADDR,
		             &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		com_rslt += smi130_acc_write_reg
		            (SMI130_ACC_INTR_ENABLE2_ADDR,
		             &data2, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
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
                uint8_t *intr_slope)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* Read the slope value */
		switch (channel) {
		case SMI130_ACC_INTR2_SLOPE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*intr_slope = SMI130_ACC_GET_BITSLICE
			              (data, SMI130_ACC_ENABLE_INTR2_PAD_SLOPE);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
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
                uint8_t intr_slope)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_INTR2_SLOPE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_ENABLE_INTR2_PAD_SLOPE,
			        intr_slope);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_ENABLE_INTR2_PAD_SLOPE_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                uint8_t *intr_newdata)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_INTR2_NEWDATA:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*intr_newdata = SMI130_ACC_GET_BITSLICE
			                (data, SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                uint8_t intr_newdata)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_INTR2_NEWDATA:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data,
			        SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA, intr_newdata);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_ENABLE_INTR2_PAD_NEWDATA_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to get
 *	the source data status of source data, source source slope
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
                uint8_t *intr_source)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		return  E_SMI130_ACC_NULL_PTR;
	} else {
		/* read the source interrupt register */
		switch (channel) {
		case SMI130_ACC_SOURCE_SLOPE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*intr_source = SMI130_ACC_GET_BITSLICE
			               (data, SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE);
			break;
		case SMI130_ACC_SOURCE_DATA:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_UNFILT_INTR_SOURCE_DATA_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*intr_source = SMI130_ACC_GET_BITSLICE
			               (data, SMI130_ACC_UNFILT_INTR_SOURCE_DATA);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the source data status of source data, source source slope
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
                uint8_t intr_source)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	if (p_smi130_acc == SMI130_ACC_NULL) {
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
			/* write the source interrupt register*/
		case SMI130_ACC_SOURCE_SLOPE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data,
			        SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE, intr_source);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_UNFILT_INTR_SOURCE_SLOPE_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		case SMI130_ACC_SOURCE_DATA:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_UNFILT_INTR_SOURCE_DATA_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_UNFILT_INTR_SOURCE_DATA,
			        intr_source);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_UNFILT_INTR_SOURCE_DATA_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                uint8_t *intr_output_type)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
			/* read the output type */
		case SMI130_ACC_INTR2_OUTPUT:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*intr_output_type = SMI130_ACC_GET_BITSLICE
			                    (data, SMI130_ACC_INTR2_PAD_OUTPUT_TYPE);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
 *              0x00              | PUSH_PULL
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
                uint8_t intr_output_type)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		com_rslt = E_SMI130_ACC_NULL_PTR;
	}  else {
		switch (channel) {
			/* write the output type*/
		case SMI130_ACC_INTR2_OUTPUT:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data,
			        SMI130_ACC_INTR2_PAD_OUTPUT_TYPE, intr_output_type);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_INTR2_PAD_OUTPUT_TYPE_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
 *        intr_level         |    result
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
                uint8_t *intr_level)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
			/* read the active level*/
		case SMI130_ACC_INTR2_LEVEL:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*intr_level = SMI130_ACC_GET_BITSLICE
			              (data, SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                uint8_t intr_level)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		com_rslt = E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
			/* write the active level */
		case SMI130_ACC_INTR2_LEVEL:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data,
			        SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL, intr_level);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_INTR2_PAD_ACTIVE_LEVEL_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_rst_intr(uint8_t rst_intr)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_RESET_INTR_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_SET_BITSLICE
		       (data, SMI130_ACC_RESET_INTR, rst_intr);
		com_rslt += smi130_acc_write_reg(SMI130_ACC_RESET_INTR_REG,
		                                 &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_latch_intr(uint8_t *latch_intr)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* read the latch duration */
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr, SMI130_ACC_LATCH_INTR_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*latch_intr = SMI130_ACC_GET_BITSLICE
		              (data, SMI130_ACC_LATCH_INTR);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_latch_intr(uint8_t latch_intr)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t latch_durn = SMI130_ACC_INIT_VALUE;
	if (p_smi130_acc == SMI130_ACC_NULL)  {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else  {
		if (latch_intr < SMI130_ACC_BW_MAX_RANGE) {
			switch (latch_intr) {
			case SMI130_ACC_LATCH_DURN_NON_LATCH:
				latch_durn = SMI130_ACC_LATCH_DURN_NON_LATCH;

				/*  NON LATCH   */
				break;
			case SMI130_ACC_LATCH_DURN_250MS:
				latch_durn = SMI130_ACC_LATCH_DURN_250MS;

				/*  250 MS  */
				break;
			case SMI130_ACC_LATCH_DURN_500MS:
				latch_durn = SMI130_ACC_LATCH_DURN_500MS;

				/*  500 MS  */
				break;
			case SMI130_ACC_LATCH_DURN_1S:
				latch_durn = SMI130_ACC_LATCH_DURN_1S;

				/*  1 S   */
				break;
			case SMI130_ACC_LATCH_DURN_2S:
				latch_durn = SMI130_ACC_LATCH_DURN_2S;

				/*  2 S  */
				break;
			case SMI130_ACC_LATCH_DURN_4S:
				latch_durn = SMI130_ACC_LATCH_DURN_4S;

				/*  4 S  */
				break;
			case SMI130_ACC_LATCH_DURN_8S:
				latch_durn = SMI130_ACC_LATCH_DURN_8S;

				/*  8 S  */
				break;
			case SMI130_ACC_LATCH_DURN_LATCH:
				latch_durn = SMI130_ACC_LATCH_DURN_LATCH;

				/*  LATCH  */
				break;
			case SMI130_ACC_LATCH_DURN_NON_LATCH1:
				latch_durn = SMI130_ACC_LATCH_DURN_NON_LATCH1;

				/*  NON LATCH1  */
				break;
			case SMI130_ACC_LATCH_DURN_250US:
				latch_durn = SMI130_ACC_LATCH_DURN_250US;

				/*  250 US   */
				break;
			case SMI130_ACC_LATCH_DURN_500US:
				latch_durn = SMI130_ACC_LATCH_DURN_500US;

				/*  500 US   */
				break;
			case SMI130_ACC_LATCH_DURN_1MS:
				latch_durn = SMI130_ACC_LATCH_DURN_1MS;

				/*  1 MS   */
				break;
			case SMI130_ACC_LATCH_DURN_12_5MS:
				latch_durn = SMI130_ACC_LATCH_DURN_12_5MS;

				/*  12.5 MS   */
				break;
			case SMI130_ACC_LATCH_DURN_25MS:
				latch_durn = SMI130_ACC_LATCH_DURN_25MS;

				/*  25 MS   */
				break;
			case SMI130_ACC_LATCH_DURN_50MS:
				latch_durn = SMI130_ACC_LATCH_DURN_50MS;

				/*  50 MS   */
				break;
			case SMI130_ACC_LATCH_DURN_LATCH1:
				latch_durn = SMI130_ACC_LATCH_DURN_LATCH1;

				/*  LATCH1   */
				break;
			default:
				break;
			}
			/* write the latch duration */
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr, SMI130_ACC_LATCH_INTR_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_LATCH_INTR, latch_durn);
			com_rslt += smi130_acc_write_reg(SMI130_ACC_LATCH_INTR_REG,
			                                 &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
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
                uint8_t *durn)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* write the duration data */
		switch (channel) {
		case SMI130_ACC_SLOPE_DURN:
			/*SLOPE DURATION*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr, SMI130_ACC_SLOPE_DURN_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*durn = SMI130_ACC_GET_BITSLICE
			        (data, SMI130_ACC_SLOPE_DURN);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the duration of Slope interrupts in the registers
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
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
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_durn(uint8_t channel,
                uint8_t durn)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL)  {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	}  else  {
		/* write duration data */
		switch (channel)   {
		case SMI130_ACC_SLOPE_DURN:
			/*SLOPE DURATION*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_SLOPE_DURN_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_SLOPE_DURN, durn);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_SLOPE_DURN_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get the threshold of Slope interrupts in the registers
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
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
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_thres(uint8_t channel,
                uint8_t *thres)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
			/* Read the threshold value */
		case SMI130_ACC_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_SLOPE_THRES_ADDR,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*thres = data;
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set the threshold of Slope interrupts in the registers
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
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
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_thres(uint8_t channel,
                uint8_t thres)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
			/* write the threshold value*/
		case SMI130_ACC_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			data = thres;
			com_rslt = smi130_acc_write_reg(
			                   SMI130_ACC_SLOPE_THRES_ADDR, &data,
			                   SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}

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
        uint8_t *selftest_axis)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* read the self test axis*/
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_ENABLE_SELFTEST_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*selftest_axis = SMI130_ACC_GET_BITSLICE
		                 (data, SMI130_ACC_ENABLE_SELFTEST);
	}
	return com_rslt;
}
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
        uint8_t selftest_axis)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		if (selftest_axis < SMI130_ACC_SELF_TEST_AXIS_RANGE) {
			/* write the self test axis*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_SELFTEST_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_ENABLE_SELFTEST, selftest_axis);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_ENABLE_SELFTEST_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
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
        uint8_t *selftest_sign)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		/* read self test sign */
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_NEG_SELFTEST_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*selftest_sign = SMI130_ACC_GET_BITSLICE
		                 (data, SMI130_ACC_NEG_SELFTEST);
	}
	return com_rslt;
}
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
        uint8_t selftest_sign)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		if (selftest_sign <
		    SMI130_ACC_SELF_TEST_SIGN_RANGE) {
			/* write self test sign */
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_NEG_SELFTEST_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_NEG_SELFTEST, selftest_sign);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_NEG_SELFTEST_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
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
        uint8_t selftest_amp)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		if (selftest_amp <
		    SMI130_ACC_SELF_TEST_AMP_RANGE) {
			/* write self test sign */
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_AMP_SELFTEST_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_AMP_SELFTEST, selftest_amp);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_AMP_SELFTEST_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
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
                uint8_t *i2c_wdt)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_I2C_SELECT:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_I2C_WDT_PERIOD_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*i2c_wdt = SMI130_ACC_GET_BITSLICE(data,
			                                   SMI130_ACC_I2C_WDT_PERIOD);
			break;
		case SMI130_ACC_I2C_ENABLE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_I2C_WDT_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*i2c_wdt = SMI130_ACC_GET_BITSLICE
			           (data, SMI130_ACC_ENABLE_I2C_WDT);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                uint8_t i2c_wdt)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_I2C_SELECT:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_I2C_WDT_PERIOD_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data,
			        SMI130_ACC_I2C_WDT_PERIOD, i2c_wdt);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_I2C_WDT_PERIOD_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		case SMI130_ACC_I2C_ENABLE:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_ENABLE_I2C_WDT_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data,
			        SMI130_ACC_ENABLE_I2C_WDT, i2c_wdt);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_ENABLE_I2C_WDT_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_get_cal_rdy(uint8_t *cal_rdy)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_FAST_CAL_RDY_STAT_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*cal_rdy = SMI130_ACC_GET_BITSLICE(data,
		                                   SMI130_ACC_FAST_CAL_RDY_STAT);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_cal_trigger(uint8_t cal_trigger)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_CAL_TRIGGER_REG, &data,
		            SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_SET_BITSLICE(data,
		                               SMI130_ACC_CAL_TRIGGER, cal_trigger);
		com_rslt += smi130_acc_write_reg(
		                    SMI130_ACC_CAL_TRIGGER_REG,
		                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_set_offset_rst(uint8_t offset_rst)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
		           (p_smi130_acc->dev_addr,
		            SMI130_ACC_RST_OFFSET_REG,
		            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		data = SMI130_ACC_SET_BITSLICE
		       (data, SMI130_ACC_RST_OFFSET,
		        offset_rst);
		com_rslt += smi130_acc_write_reg(
		                    SMI130_ACC_RST_OFFSET_REG,
		                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
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
                uint8_t *offset)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_OFFSET_TRIGGER_X:
			/*OFFSET TRIGGER X*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_COMP_TARGET_OFFSET_X_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*offset = SMI130_ACC_GET_BITSLICE(data,
			                                  SMI130_ACC_COMP_TARGET_OFFSET_X);
			break;
		case SMI130_ACC_OFFSET_TRIGGER_Y:
			/*OFFSET TRIGGER Y*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_COMP_TARGET_OFFSET_Y_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*offset = SMI130_ACC_GET_BITSLICE(data,
			                                  SMI130_ACC_COMP_TARGET_OFFSET_Y);
			break;
		case SMI130_ACC_OFFSET_TRIGGER_Z:
			/*OFFSET TRIGGER Z*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_COMP_TARGET_OFFSET_Z_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*offset = SMI130_ACC_GET_BITSLICE
			          (data, SMI130_ACC_COMP_TARGET_OFFSET_Z);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
 *        3                   |   SMI130_ACC_OFFSET_TRIGGER_Z
 *
 *  @param  offset: The offset target value
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
                uint8_t offset)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_OFFSET_TRIGGER_X:
			/*OFFSET TARGET X*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_COMP_TARGET_OFFSET_X_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_COMP_TARGET_OFFSET_X, offset);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_COMP_TARGET_OFFSET_X_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		case SMI130_ACC_OFFSET_TRIGGER_Y:
			/*OFFSET TARGET Y*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_COMP_TARGET_OFFSET_Y_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_COMP_TARGET_OFFSET_Y, offset);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_COMP_TARGET_OFFSET_Y_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		case SMI130_ACC_OFFSET_TRIGGER_Z:
			/*OFFSET TARGET Z*/
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_COMP_TARGET_OFFSET_Z_REG,
			            &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			data = SMI130_ACC_SET_BITSLICE
			       (data, SMI130_ACC_COMP_TARGET_OFFSET_Z, offset);
			com_rslt += smi130_acc_write_reg(
			                    SMI130_ACC_COMP_TARGET_OFFSET_Z_REG,
			                    &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                int8_t *offset)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_X_AXIS:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_OFFSET_X_AXIS_ADDR, &data,
			            SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*offset = (int8_t)data;
			break;
		case SMI130_ACC_Y_AXIS:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_OFFSET_Y_AXIS_ADDR, &data,
			            SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*offset = (int8_t)data;
			break;
		case SMI130_ACC_Z_AXIS:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr,
			            SMI130_ACC_OFFSET_Z_AXIS_ADDR, &data,
			            SMI130_ACC_GEN_READ_WRITE_LENGTH);
			*offset = (int8_t)data;
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
                int8_t offset)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (channel) {
		case SMI130_ACC_X_AXIS:
			data = offset;
			com_rslt = smi130_acc_write_reg(
			                   SMI130_ACC_OFFSET_X_AXIS_ADDR, &data,
			                   SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		case SMI130_ACC_Y_AXIS:
			data = offset;
			com_rslt = smi130_acc_write_reg(
			                   SMI130_ACC_OFFSET_Y_AXIS_ADDR, &data,
			                   SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		case SMI130_ACC_Z_AXIS:
			data = offset;
			com_rslt = smi130_acc_write_reg(
			                   SMI130_ACC_OFFSET_Z_AXIS_ADDR, &data,
			                   SMI130_ACC_GEN_READ_WRITE_LENGTH);
			break;
		default:
			com_rslt = E_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}

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
SMI130_ACC_RETURN_FUNCTION_TYPE smi130_acc_read_temp(int8_t *temp_s8)
{
	uint8_t data = SMI130_ACC_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC(
		                   p_smi130_acc->dev_addr,
		                   SMI130_ACC_TEMP_ADDR,
		                   &data, SMI130_ACC_GEN_READ_WRITE_LENGTH);
		*temp_s8 = (int8_t)data;
	}
	return com_rslt;
}
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
        struct smi130_acc_data_temp *accel)
{
	/*  Variable used to return value of
	communication routine*/
	SMI130_ACC_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	uint8_t data[SMI130_ACC_XYZ_TEMP_DATA_SIZE] = {
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE,
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE,
		SMI130_ACC_INIT_VALUE, SMI130_ACC_INIT_VALUE,
		SMI130_ACC_INIT_VALUE
	};
	if (p_smi130_acc == SMI130_ACC_NULL) {
		/* Check the struct p_smi130_acc is empty */
		return E_SMI130_ACC_NULL_PTR;
	} else {
		switch (V_SMI130_ACC_RESOLUTION_U8) {
		case SMI130_ACC_12_RESOLUTION:
			com_rslt = p_smi130_acc->SMI130_ACC_BUS_READ_FUNC
			           (p_smi130_acc->dev_addr, SMI130_ACC_X12_LSB_REG,
			            data, SMI130_ACC_BW_MIN_RANGE);

			/* read x data*/
			accel->x = (int16_t)((((int32_t)((int8_t)
			                                 data[SMI130_ACC_SENSOR_DATA_XYZ_X_MSB]))
			                      << SMI130_ACC_SHIFT_EIGHT_BITS)|
			                     (data[SMI130_ACC_SENSOR_DATA_XYZ_X_LSB]
			                      & SMI130_ACC_12_BIT_SHIFT));
			accel->x = accel->x >> SMI130_ACC_SHIFT_FOUR_BITS;

			/* read y data*/
			accel->y = (int16_t)((((int32_t)((int8_t)
			                                 data[SMI130_ACC_SENSOR_DATA_XYZ_Y_MSB]))
			                      << SMI130_ACC_SHIFT_EIGHT_BITS)|
			                     (data[SMI130_ACC_SENSOR_DATA_XYZ_Y_LSB]
			                      & SMI130_ACC_12_BIT_SHIFT));
			accel->y = accel->y >> SMI130_ACC_SHIFT_FOUR_BITS;

			/* read z data*/
			accel->z = (int16_t)((((int32_t)((int8_t)
			                                 data[SMI130_ACC_SENSOR_DATA_XYZ_Z_MSB]))
			                      << SMI130_ACC_SHIFT_EIGHT_BITS)|
			                     (data[SMI130_ACC_SENSOR_DATA_XYZ_Z_LSB]
			                      & SMI130_ACC_12_BIT_SHIFT));
			accel->z = accel->z >> SMI130_ACC_SHIFT_FOUR_BITS;
			/*Accessing the sixth element of array*/
			accel->temp = (int8_t)data[SMI130_ACC_SENSOR_DATA_TEMP];
			break;
		default:
			break;
		}
	}
	return com_rslt;
}

