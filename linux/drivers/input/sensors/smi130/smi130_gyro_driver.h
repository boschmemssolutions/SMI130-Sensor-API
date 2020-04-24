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
/*! \file smi130_gyro_driver.h
    \brief smi130_gyro Linux Sensor Driver Support Header File */
#ifndef _SMI130_GYRO_DRIVER_H
#define _SMI130_GYRO_DRIVER_H

#include "smi130_gyro.h"

/* sensor specific */
#define SMI130_GYRO_NAME                "smi130_gyro"
#define SMI_GYRO_DEBUG			        1
#define SMI_GYRO_REG_NAME(name)         SMI130_GYRO_##name
#define SMI_GYRO_VAL_NAME(name)         SMI130_GYRO_##name
#define SMI_GYRO_CALL_API(name)         smi130_gyro_##name
#define MSC_TIME                        6
#define INPUT_EVENT_TYPE                EV_MSC
#define INPUT_EVENT_X                   MSC_GESTURE
#define INPUT_EVENT_Y                   MSC_GESTURE
#define INPUT_EVENT_Z                   MSC_GESTURE
#define INPUT_EVENT_TIME                MSC_TIMESTAMP

/* generic */
#define SMI_GYRO_MAX_RETRY_I2C_XFER     (100)
#define SMI_GYRO_MAX_RETRY_WAKEUP       (5)
#define SMI_GYRO_MAX_RETRY_WAIT_DRDY    (100)
#define SMI_GYRO_DELAY_MIN              (1)
#define SMI_GYRO_DELAY_DEFAULT          (200)
#define SMI_GYRO_VALUE_MAX              (32767)
#define SMI_GYRO_VALUE_MIN              (-32768)
#define BYTES_PER_LINE                  (16)
#define SMI_GYRO_SELF_TEST              (0)
#define SMI_GYRO_SOFT_RESET_VALUE       (0xB6)
#define SMI_INT1_LVL_ACTIVE_LOW         (0)
#define SMI_INT1_LVL_ACTIVE_HIGH        (1)
#define SMI_INT1_TYPE_PUSH_PULL         (0)
#define SMI_INT1_TYPE_OPEN_DRAIN        (1)


struct smi_gyro_client_data {
	struct smi130_gyro_t device;
	struct device *dev;
	struct input_dev *input;
	struct delayed_work work;

	atomic_t delay;
	uint8_t debug_level;
	struct smi130_gyro_data_t value;
	uint8_t enable:1;
	uint64_t timestamp;
	uint64_t base_time;
	uint64_t gyro_count;
	uint64_t time_odr;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct work_struct report_data_work;
	int32_t is_timer_running;
	struct hrtimer timer;
	ktime_t work_delay_kt;
	uint8_t gpio_pin;
	int16_t IRQ;
	struct work_struct irq_work;
};






extern int32_t smi_gyro_probe(struct smi_gyro_client_data *client_data, struct device *dev);
extern void smi_gyro_shutdown(struct device *dev);
extern int32_t smi_gyro_remove(struct device *dev);
extern int32_t smi_gyro_resume(struct device *dev);

#endif