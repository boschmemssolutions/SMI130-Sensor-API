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
/*! file <smi130_gyro.c >
    brief <Sensor driver for SMI130_GYRO> */
#include "smi130_gyro.h"
static struct smi130_gyro_t *p_smi130_gyro;


/*****************************************************************************
 * Description: *//**brief API Initialization routine
 *
 *
 *
 *
* \param smi130_gyro_t *smi130_gyro
 *      Pointer to a structure.
 *
 *       structure members are
 *
 *       uint8_t chip_id;
 *       uint8_t dev_addr;
 *       SMI130_GYRO_BRD_FUNC_PTR;
 *       SMI130_GYRO_WR_FUNC_PTR;
 *       SMI130_GYRO_RD_FUNC_PTR;
 *       void(*delay_msec)( SMI130_GYRO_MDELAY_DATA_TYPE );
 *
 *
 *
 *
 *
 *  \return result of communication routines
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_init(struct smi130_gyro_t *smi130_gyro)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t a_data  = C_SMI130_GYRO_Zero_U8X;
	p_smi130_gyro = smi130_gyro;

	p_smi130_gyro->dev_addr = SMI130_GYRO_I2C_ADDR;

	/*Read CHIP_ID */
	comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
	                SMI130_GYRO_CHIP_ID_ADDR, &a_data, 1);
	p_smi130_gyro->chip_id = a_data;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief Reads Rate dataX from location 02h and 03h
 * registers
 *
 *
 *
 *
 *  \param
 *      SMI130_GYRO_S16  *data_x   :  Address of data_x
 *
 *
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataX(SMI130_GYRO_S16 *data_x)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t a_data[2] = {0, 0};
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATE_X_LSB_VALUEX__REG, a_data, 2);
		a_data[0] = SMI130_GYRO_GET_BITSLICE(a_data[0],
		                                     SMI130_GYRO_RATE_X_LSB_VALUEX);
		*data_x = (SMI130_GYRO_S16)
		          ((((SMI130_GYRO_S16)((int8_t)a_data[1])) <<
		            SMI130_GYRO_SHIFT_8_POSITION) | (a_data[0]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief Reads rate dataY from location 04h and 05h
 * registers
 *
 *
 *
 *
 *  \param
 *      SMI130_GYRO_S16  *data_y   :  Address of data_y
 *
 *
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataY(SMI130_GYRO_S16 *data_y)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t a_data[2] = {0, 0};
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATE_Y_LSB_VALUEY__REG, a_data, 2);
		a_data[0] = SMI130_GYRO_GET_BITSLICE(a_data[0],
		                                     SMI130_GYRO_RATE_Y_LSB_VALUEY);
		*data_y = (SMI130_GYRO_S16)
		          ((((SMI130_GYRO_S16)((int8_t)a_data[1]))
		            << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[0]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief Reads rate dataZ from location 06h and 07h
 * registers
 *
 *
 *
 *
 *  \param
 *      SMI130_GYRO_S16  *data_z   :  Address of data_z
 *
 *
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataZ(SMI130_GYRO_S16 *data_z)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t a_data[2] = {0, 0};
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATE_Z_LSB_VALUEZ__REG, a_data, 2);
		a_data[0] = SMI130_GYRO_GET_BITSLICE(a_data[0],
		                                     SMI130_GYRO_RATE_Z_LSB_VALUEZ);
		*data_z = (SMI130_GYRO_S16)
		          ((((SMI130_GYRO_S16)((int8_t)a_data[1]))
		            << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[0]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief Reads data X,Y and Z from location 02h to 07h
 *
 *
 *
 *
 *  \param
 *      smi130_gyro_data_t *data   :  Address of smi130_gyro_data_t
 *
 *
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataXYZ(struct smi130_gyro_data_t *data)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t a_data[6] = {0, 0, 0, 0, 0, 0};
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATE_X_LSB_VALUEX__REG, a_data, 6);
		/* Data X */
		a_data[0] =
		        SMI130_GYRO_GET_BITSLICE(a_data[0], SMI130_GYRO_RATE_X_LSB_VALUEX);
		data->datax = (SMI130_GYRO_S16)
		              ((((SMI130_GYRO_S16)((int8_t)a_data[1]))
		                << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[0]));
		/* Data Y */
		a_data[2] = SMI130_GYRO_GET_BITSLICE(a_data[2],
		                                     SMI130_GYRO_RATE_Y_LSB_VALUEY);
		data->datay = (SMI130_GYRO_S16)
		              ((((SMI130_GYRO_S16)((int8_t)a_data[3]))
		                << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[2]));
		/* Data Z */
		a_data[4] = SMI130_GYRO_GET_BITSLICE(a_data[4],
		                                     SMI130_GYRO_RATE_Z_LSB_VALUEZ);
		data->dataz = (SMI130_GYRO_S16)
		              ((((SMI130_GYRO_S16)((int8_t)a_data[5]))
		                << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[4]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief Reads data X,Y,Z and Interrupts
 *							from location 02h to 07h
 *
 *
 *
 *
 *  \param
 *      smi130_gyro_data_t *data   :  Address of smi130_gyro_data_t
 *
 *
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_dataXYZI(struct smi130_gyro_data_t *data)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t a_data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATE_X_LSB_VALUEX__REG, a_data, 12);
		/* Data X */
		a_data[0] = SMI130_GYRO_GET_BITSLICE(a_data[0],
		                                     SMI130_GYRO_RATE_X_LSB_VALUEX);
		data->datax = (SMI130_GYRO_S16)
		              ((((SMI130_GYRO_S16)((int8_t)a_data[1]))
		                << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[0]));
		/* Data Y */
		a_data[2] = SMI130_GYRO_GET_BITSLICE(a_data[2],
		                                     SMI130_GYRO_RATE_Y_LSB_VALUEY);
		data->datay = (SMI130_GYRO_S16)
		              ((((SMI130_GYRO_S16)((int8_t)a_data[3]))
		                << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[2]));
		/* Data Z */
		a_data[4] = SMI130_GYRO_GET_BITSLICE(a_data[4],
		                                     SMI130_GYRO_RATE_Z_LSB_VALUEZ);
		data->dataz = (SMI130_GYRO_S16)
		              ((((SMI130_GYRO_S16)((int8_t)a_data[5]))
		                << SMI130_GYRO_SHIFT_8_POSITION) | (a_data[4]));
		data->intstatus[0] = a_data[7];
		data->intstatus[1] = a_data[8];
		data->intstatus[2] = a_data[9];
		data->intstatus[3] = a_data[10];
		data->intstatus[4] = a_data[11];
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief Reads Temperature from location 08h
 *
 *
 *
 *
 *  \param
 *      uint8_t *temp   :  Address of temperature
 *
 *
 *  \return
 *      result of communication routines
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_Temperature(uint8_t *temperature)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_TEMP_ADDR, &v_data, 1);
		*temperature = v_data;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API reads the data from the given register
 *
 *
 *
 *
 *\param uint8_t addr, uint8_t *data uint8_t len
 *                       addr -> Address of the register
 *                       data -> address of the variable, read value will be
 *								kept
 *						len -> No of byte to be read.
 *  \return  results of bus communication function
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_read_register(uint8_t addr,
                uint8_t *data, uint8_t len)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
		         (p_smi130_gyro->dev_addr, addr, data, len);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API reads the data from the given register
 *
 *
 *
 *
 *\param uint8_t addr, uint8_t *data SMI130_GYRO_S32 len
 *                       addr -> Address of the register
 *                       data -> address of the variable, read value will be
 *								kept
 *						len -> No of byte to be read.
 *  \return  results of bus communication function
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_burst_read(uint8_t addr,
                uint8_t *data, SMI130_GYRO_S32 len)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                addr, data, len);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API given data to the given register
 *
 *
 *
 *
 *\param uint8_t addr, uint8_t data,uint8_t len
 *                   addr -> Address of the register
 *                   data -> Data to be written to the register
 *					len -> No of byte to be read.
 *
 *  \return Results of bus communication function
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_write_register(uint8_t addr,
                uint8_t *data, uint8_t len)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
		         (p_smi130_gyro->dev_addr, addr, data, len);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API reads the range from register 0x0Fh of
 * (0 to 2) bits
 *
 *
 *
 *
 *\param uint8_t *range
 *      Range[0....7]
 *      0 2000/s
 *      1 1000/s
 *      2 500/s
 *      3 250/s
 *      4 125/s
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_range_reg(uint8_t *range)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
		         (p_smi130_gyro->dev_addr,
		          SMI130_GYRO_RANGE_ADDR_RANGE__REG, &v_data, 1);
		*range =
		        SMI130_GYRO_GET_BITSLICE(v_data, SMI130_GYRO_RANGE_ADDR_RANGE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API sets the range register 0x0Fh
 * (0 to 2 bits)
 *
 *
 *
 *
 *\param uint8_t range
 *
 *      Range[0....7]
 *      0 2000/s
 *      1 1000/s
 *      2 500/s
 *      3 250/s
 *      4 125/s
 *
 *
 *
 *
 *  \return Communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_range_reg(uint8_t range)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		if (range < C_SMI130_GYRO_Five_U8X) {
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_RANGE_ADDR_RANGE__REG, &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_RANGE_ADDR_RANGE,
			                                  range);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_RANGE_ADDR_RANGE__REG, &v_data, 1);
		} else {
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API reads the bandwidth register of 0x10h 0 to
 *  3 bits
 *
 *
 *
 *
* \param uint8_t *bandwidth
 *              pointer to a variable passed as a parameter
 *
 *              0 no filter(523 Hz)
 *              1 230Hz
 *              2 116Hz
 *              3 47Hz
 *              4 23Hz
 *              5 12Hz
 *              6 64Hz
 *              7 32Hz
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_bw(uint8_t *bandwidth)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
		         (p_smi130_gyro->dev_addr, SMI130_GYRO_BW_ADDR__REG, &v_data, 1);
		*bandwidth = SMI130_GYRO_GET_BITSLICE(v_data,
		                                      SMI130_GYRO_BW_ADDR);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API writes the Bandwidth register (0x10h of 0
 * to 3 bits)
 *
 *
 *
 *
 *\param uint8_t bandwidth,
 *              The bandwidth to be set passed as a parameter
 *
 *              0 no filter(523 Hz)
 *              1 230Hz
 *              2 116Hz
 *              3 47Hz
 *              4 23Hz
 *              5 12Hz
 *              6 64Hz
 *              7 32Hz
 *
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_bw(uint8_t bandwidth)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		if (bandwidth < C_SMI130_GYRO_Eight_U8X) {
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_BW_ADDR__REG, &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_BW_ADDR, bandwidth);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_BW_ADDR__REG, &v_data, 1);
		} else {
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API reads the status of External Trigger
 * selection bits (4 and 5) of 0x12h registers
 *
 *
 *
 *
 *\param uint8_t *pwu_ext_tri_sel
 *                      Pointer to a variable passed as a parameter
 *
 *
 *
 *  \return Communication Results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_pmu_ext_tri_sel(
        uint8_t *pwu_ext_tri_sel)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__REG, &v_data, 1);
		*pwu_ext_tri_sel = SMI130_GYRO_GET_BITSLICE(v_data,
		                   SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API writes the External Trigger selection
 * bits (4 and 5) of 0x12h registers
 *
 *
 *
 *
 *\param uint8_t pwu_ext_tri_sel
 *               Value to be written passed as a parameter
 *
 *
 *
 *  \return Communication Results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_pmu_ext_tri_sel(
        uint8_t pwu_ext_tri_sel)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__REG, &v_data, 1);
		v_data = SMI130_GYRO_SET_BITSLICE(v_data,
		                                  SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL, pwu_ext_tri_sel);
		comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_MODE_LPM2_ADDR_EXT_TRI_SEL__REG, &v_data, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief  This API is used to get data high bandwidth
 *
 *
 *
 *
 *\param uint8_t *high_bw : Address of high_bw
 *                         Pointer to a variable passed as a parameter
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_high_bw(uint8_t *high_bw)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__REG, &v_data, 1);
		*high_bw = SMI130_GYRO_GET_BITSLICE(v_data,
		                                    SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set data high bandwidth
 *
 *
 *
 *
 *\param uint8_t high_bw:
 *          Value to be written passed as a parameter
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_high_bw(uint8_t high_bw)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		if (high_bw < C_SMI130_GYRO_Two_U8X) {
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__REG,
			          &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW, high_bw);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_RATED_HBW_ADDR_DATA_HIGHBW__REG,
			           &v_data, 1);
		} else {
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get shadow dis
 *
 *
 *
 *
 *\param uint8_t *shadow_dis : Address of shadow_dis
 *                       Pointer to a variable passed as a parameter
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_shadow_dis(uint8_t *shadow_dis)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__REG, &v_data, 1);
		*shadow_dis = SMI130_GYRO_GET_BITSLICE(v_data,
		                                       SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set shadow dis
 *
 *
 *
 *
 *\param uint8_t shadow_dis
 *         Value to be written passed as a parameter
 *
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_shadow_dis(uint8_t shadow_dis)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		if (shadow_dis < C_SMI130_GYRO_Two_U8X) {
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__REG, &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS, shadow_dis);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_RATED_HBW_ADDR_SHADOW_DIS__REG, &v_data, 1);
		} else {
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief
 *               This function is used for the soft reset
 *     The soft reset register will be written with 0xB6.
 *
 *
 *
* \param None
 *
 *
 *
 *  \return Communication results.
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_soft_reset()
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_SoftReset  = C_SMI130_GYRO_Zero_U8X;
	v_SoftReset = 0xB6;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_BGW_SOFTRESET_ADDR, &v_SoftReset, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get data enable data
 *
 *
 *
 *
 *\param uint8_t *data_en : Address of data_en
 *                         Pointer to a variable passed as a parameter
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_data_enable(uint8_t *data_en)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_INT_ENABLE0_DATAEN__REG, &v_data, 1);
		*data_en = SMI130_GYRO_GET_BITSLICE(v_data,
		                                    SMI130_GYRO_INT_ENABLE0_DATAEN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set data enable data
 *
 *
 *
 *
 *  \param uint8_t data_en:
 *          Value to be written passed as a \parameter
 *           0 --> Disable
 *           1 --> Enable
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_data_en(uint8_t data_en)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
		         (p_smi130_gyro->dev_addr,
		          SMI130_GYRO_INT_ENABLE0_DATAEN__REG, &v_data, 1);
		v_data = SMI130_GYRO_SET_BITSLICE(v_data,
		                                  SMI130_GYRO_INT_ENABLE0_DATAEN, data_en);
		comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
		          (p_smi130_gyro->dev_addr,
		           SMI130_GYRO_INT_ENABLE0_DATAEN__REG, &v_data, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get the output type status
 *
 *
 *
 *
 *  \param uint8_t channel,uint8_t *int_od
 *                  SMI130_GYRO_INT1    ->   0
 *                  int_od : open drain   ->   1
 *                           push pull    ->   0
 *
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_int_od(uint8_t param,
                uint8_t *int_od)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (param) {
		case SMI130_GYRO_INT1:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_INT_ENABLE1_IT1_OD__REG, &v_data, 1);
			*int_od = SMI130_GYRO_GET_BITSLICE(v_data,
			                                   SMI130_GYRO_INT_ENABLE1_IT1_OD);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set the output type status
 *
 *
 *
 *
 *  \param uint8_t channel,uint8_t *int_od
 *                  SMI130_GYRO_INT1    ->   0
 *                  int_od : open drain   ->   1
 *                           push pull    ->   0
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_int_od(uint8_t param,
                uint8_t int_od)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (param) {
		case SMI130_GYRO_INT1:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_INT_ENABLE1_IT1_OD__REG, &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_INT_ENABLE1_IT1_OD, int_od);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_INT_ENABLE1_IT1_OD__REG, &v_data, 1);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get Active Level status
 *
 *
 *
 *
 *  \param uint8_t channel,uint8_t *int_lvl
 *                  SMI130_GYRO_INT1    ->    0
 *                  int_lvl : Active HI   ->   1
 *                            Active LO   ->   0
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_int_lvl(uint8_t param,
                uint8_t *int_lvl)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (param) {
		case SMI130_GYRO_INT1:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_INT_ENABLE1_IT1_LVL__REG, &v_data, 1);
			*int_lvl = SMI130_GYRO_GET_BITSLICE(v_data,
			                                    SMI130_GYRO_INT_ENABLE1_IT1_LVL);
			break;
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set Active Level status
 *
 *
 *
 *
 *  \param uint8_t channel,uint8_t *int_lvl
 *                  SMI130_GYRO_INT1    ->    0
 *                  int_lvl : Active HI   ->   1
 *                            Active LO   ->   0
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_int_lvl(uint8_t param,
                uint8_t int_lvl)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (param) {
		case SMI130_GYRO_INT1:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_INT_ENABLE1_IT1_LVL__REG, &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_INT_ENABLE1_IT1_LVL, int_lvl);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_INT_ENABLE1_IT1_LVL__REG, &v_data, 1);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get data Interrupt1 and data
 * Interrupt2
 *
 *
 *
 *
 *  \param uint8_t axis,uint8_t *int_data
 *                       axis :
 *                       SMI130_GYRO_INT1_DATA -> 0
 *                       int_data :
 *                       Disable     -> 0
 *                       Enable      -> 1
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_int_data(uint8_t axis,
                uint8_t *int_data)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (axis) {
		case SMI130_GYRO_INT1_DATA:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_MAP_1_INT1_DATA__REG, &v_data, 1);
			*int_data = SMI130_GYRO_GET_BITSLICE(v_data,
			                                     SMI130_GYRO_MAP_1_INT1_DATA);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set data Interrupt1 and data
 * Interrupt2
 *
 *
 *
 *
 * \param uint8_t axis,uint8_t *int_data
 *                       axis :
 *                       SMI130_GYRO_INT1_DATA -> 0
 *                       int_data :
 *                       Disable     -> 0
 *                       Enable      -> 1
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_int_data(uint8_t axis,
                uint8_t int_data)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres  = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data  = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	}   else {
		switch (axis) {
		case SMI130_GYRO_INT1_DATA:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_MAP_1_INT1_DATA__REG, &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_MAP_1_INT1_DATA, int_data);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_MAP_1_INT1_DATA__REG, &v_data, 1);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get the status of i2c wdt
 *
 *
 *
 *
 *\param uint8_t channel,uint8_t *prog_mode
 *            SMI130_GYRO_I2C_WDT_SEL               1
 *            SMI130_GYRO_I2C_WDT_EN                0
 *            *prog_mode : Address of prog_mode
 *                         Pointer to a variable passed as a parameter
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_i2c_wdt(uint8_t i2c_wdt,
                uint8_t *prog_mode)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (i2c_wdt) {
		case SMI130_GYRO_I2C_WDT_EN:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__REG,
			          &v_data, 1);
			*prog_mode = SMI130_GYRO_GET_BITSLICE(v_data,
			                                      SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN);
			break;
		case SMI130_GYRO_I2C_WDT_SEL:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__REG,
			          &v_data, 1);
			*prog_mode = SMI130_GYRO_GET_BITSLICE(v_data,
			                                      SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set the status of i2c wdt
 *
 *
 *
 *
 *\param uint8_t channel,uint8_t prog_mode
 *            SMI130_GYRO_I2C_WDT_SEL               1
 *            SMI130_GYRO_I2C_WDT_EN                0
 *            prog_mode : Value to be written passed as a parameter
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_i2c_wdt(uint8_t i2c_wdt,
                uint8_t prog_mode)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		switch (i2c_wdt) {
		case SMI130_GYRO_I2C_WDT_EN:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__REG,
			          &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN, prog_mode);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_EN__REG,
			           &v_data, 1);
			break;
		case SMI130_GYRO_I2C_WDT_SEL:
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__REG,
			          &v_data, 1);
			v_data = SMI130_GYRO_SET_BITSLICE(v_data,
			                                  SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL, prog_mode);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_BGW_SPI3_WDT_ADDR_I2C_WDT_SEL__REG,
			           &v_data, 1);
			break;
		default:
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief  This API is used to get the status of spi3
 *
 *
 *
 *
* \param uint8_t *spi3 : Address of spi3
 *                                Pointer to a variable passed as a parameter
 *
 *
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_spi3(uint8_t *spi3)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__REG, &v_data, 1);
		*spi3 = SMI130_GYRO_GET_BITSLICE(v_data,
		                                 SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set the status of spi3
 *
 *
 *
 *
 *\param uint8_t spi3
 *
 *
 *
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_spi3(uint8_t spi3)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t v_data = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == SMI130_GYRO_NULL) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__REG, &v_data, 1);
		v_data = SMI130_GYRO_SET_BITSLICE(v_data,
		                                  SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3, spi3);
		comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_BGW_SPI3_WDT_ADDR_SPI3__REG, &v_data, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to get the operating modes of the
 * sensor
 *
 *
 *
 *
 *\param uint8_t * mode : Address of mode
 *                       0 -> NORMAL
 *                       1 -> DEEP SUSPEND
 *
 *
 *  \return
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_get_mode(uint8_t *mode)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t data1 = C_SMI130_GYRO_Zero_U8X;
	uint8_t data2 = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == C_SMI130_GYRO_Zero_U8X) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_MODE_LPM1_ADDR, &data1, C_SMI130_GYRO_One_U8X);
		comres += p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
		                SMI130_GYRO_MODE_LPM2_ADDR, &data2, C_SMI130_GYRO_One_U8X);
		data1  = (data1 & 0x20) >> 5;
		if (data1 == 0x01) {
			*mode = SMI130_GYRO_MODE_DEEPSUSPEND;
		} else {
			*mode = SMI130_GYRO_MODE_NORMAL;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to set the operating Modes of the
 * sensor
 *
 *
 *
 *
 *\param uint8_t Mode
 *                       0 -> NORMAL
 *                       1 -> DEEPSUSPEND
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_set_mode(uint8_t mode)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t data1 = C_SMI130_GYRO_Zero_U8X;
	uint8_t data2 = C_SMI130_GYRO_Zero_U8X;
	uint8_t data3 = C_SMI130_GYRO_Zero_U8X;
	if (p_smi130_gyro == C_SMI130_GYRO_Zero_U8X) {
		return  E_SMI130_GYRO_NULL_PTR;
	} else {
		if (mode < C_SMI130_GYRO_Five_U8X) {
			comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			         (p_smi130_gyro->dev_addr,
			          SMI130_GYRO_MODE_LPM1_ADDR, &data1, C_SMI130_GYRO_One_U8X);
			comres += p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC
			          (p_smi130_gyro->dev_addr,
			           SMI130_GYRO_MODE_LPM2_ADDR, &data2, C_SMI130_GYRO_One_U8X);
			switch (mode) {
			case SMI130_GYRO_MODE_NORMAL:
				data1  = SMI130_GYRO_SET_BITSLICE(data1,
				                                  SMI130_GYRO_MODE_LPM1, C_SMI130_GYRO_Zero_U8X);
				comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
				          (p_smi130_gyro->dev_addr,
				           SMI130_GYRO_MODE_LPM1_ADDR, &data1, C_SMI130_GYRO_One_U8X);
				p_smi130_gyro->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
				comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
				          (p_smi130_gyro->dev_addr,
				           SMI130_GYRO_MODE_LPM2_ADDR, &data3, C_SMI130_GYRO_One_U8X);
				break;
			case SMI130_GYRO_MODE_DEEPSUSPEND:
				data1  = SMI130_GYRO_SET_BITSLICE(data1,
				                                  SMI130_GYRO_MODE_LPM1, C_SMI130_GYRO_One_U8X);
				comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
				          (p_smi130_gyro->dev_addr,
				           SMI130_GYRO_MODE_LPM1_ADDR, &data1, C_SMI130_GYRO_One_U8X);
				p_smi130_gyro->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
				comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC
				          (p_smi130_gyro->dev_addr,
				           SMI130_GYRO_MODE_LPM2_ADDR, &data3, C_SMI130_GYRO_One_U8X);
				break;
			default:
				comres = E_SMI130_GYRO_OUT_OF_RANGE;
				break;
			}
		} else {
			comres = E_SMI130_GYRO_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API is used to to do selftest to sensor
 * sensor
 *
 *
 *
 *
 *\param uint8_t *result
 *
 *
 *
 *
 *  \return communication results
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
SMI130_GYRO_RETURN_FUNCTION_TYPE smi130_gyro_selftest(uint8_t *result)
{
	SMI130_GYRO_RETURN_FUNCTION_TYPE comres = C_SMI130_GYRO_Zero_U8X;
	uint8_t data1 = C_SMI130_GYRO_Zero_U8X;
	uint8_t data2 = C_SMI130_GYRO_Zero_U8X;

	comres = p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
	                SMI130_GYRO_SELF_TEST_ADDR, &data1, C_SMI130_GYRO_One_U8X);
	data2  = SMI130_GYRO_GET_BITSLICE(data1, SMI130_GYRO_SELF_TEST_ADDR_RATEOK);
	data1  = SMI130_GYRO_SET_BITSLICE(data1, SMI130_GYRO_SELF_TEST_ADDR_TRIGBIST,
	                                  C_SMI130_GYRO_One_U8X);
	comres += p_smi130_gyro->SMI130_GYRO_BUS_WRITE_FUNC(p_smi130_gyro->dev_addr,
	                SMI130_GYRO_SELF_TEST_ADDR_TRIGBIST__REG, &data1, C_SMI130_GYRO_One_U8X);

	/* Waiting time to complete the selftest process */
	p_smi130_gyro->delay_msec(10);

	/* Reading Selftest result bir bist_failure */
	comres += p_smi130_gyro->SMI130_GYRO_BUS_READ_FUNC(p_smi130_gyro->dev_addr,
	                SMI130_GYRO_SELF_TEST_ADDR_BISTFAIL__REG, &data1, C_SMI130_GYRO_One_U8X);
	data1  = SMI130_GYRO_GET_BITSLICE(data1, SMI130_GYRO_SELF_TEST_ADDR_BISTFAIL);
	if ((data1 == 0x00) && (data2 == 0x01))
		*result = C_SMI130_GYRO_SUCCESS;
	else
		*result = C_SMI130_GYRO_FAILURE;
	return comres;
}
