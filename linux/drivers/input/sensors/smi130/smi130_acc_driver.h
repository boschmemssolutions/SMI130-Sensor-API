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
/*! file <smi130_acc_driver.h >
    brief <smi130_acc Linux Sensor Driver Support Header File> */
#ifndef _SMI130_ACC_DRIVER_H
#define _SMI130_ACC_DRIVER_H

#include "smi130_acc.h"


#define SMI130_ACC_DELAY_DEFAULT           200
#define SMI130_ACC_MAX_DELAY               200

/*smi interrupt about sgm*/
#define INPUT_EVENT_SGM                     3/*7*/
#define INPUT_EVENT_FAST_GYRO_CALIB_DONE    4
#define INPUT_EVENT_FAST_ACC_CALIB_DONE     6
#define INPUT_EVENT_TYPE                    EV_MSC
#define INPUT_EVENT_X                       MSC_GESTURE
#define INPUT_EVENT_Y                       MSC_GESTURE
#define INPUT_EVENT_Z                       MSC_GESTURE
#define INPUT_EVENT_TIME                    MSC_TIMESTAMP
#define SLOPE_INTERRUPT                     REL_DIAL



/*
1: slope status
7: slope x
8: slope y
9: slope z
10: slope x negative
11: slope y negative
12: slope z negative
*/
#define SLOPE_INTERRUPT_HAPPENED                    1
#define SLOPE_INTERRUPT_X_HAPPENED                  7
#define SLOPE_INTERRUPT_Y_HAPPENED                  8
#define SLOPE_INTERRUPT_Z_HAPPENED                  9
#define SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED         10
#define SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED         11
#define SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED         12

/*! Slope interrupt of x, y, z axis happened */
#define SLOPE_INTERRUPT_X             SLOPE_INTERRUPT_X_HAPPENED
#define SLOPE_INTERRUPT_Y             SLOPE_INTERRUPT_Y_HAPPENED
#define SLOPE_INTERRUPT_Z             SLOPE_INTERRUPT_Z_HAPPENED
/*! Slope interrupt of x, y, z negative axis happened */
#define SLOPE_INTERRUPT_X_N           SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED
#define SLOPE_INTERRUPT_Y_N           SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED
#define SLOPE_INTERRUPT_Z_N           SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED

struct smi_client_data {
	struct smi130_acc_t device;
	struct device *dev;
	struct input_dev *input;
	struct smi130_acc_data value;

	struct input_dev *dev_interrupt;

	struct delayed_work work;
	struct work_struct irq_work;
	struct work_struct report_data_work;

	int32_t is_timer_running;
	struct hrtimer timer;
	ktime_t work_delay_kt;

	uint8_t selftest;
	struct delayed_work delay_work_sig;

	atomic_t in_suspend;
	atomic_t wkqueue_en;
	atomic_t delay;
	atomic_t selftest_result;
	atomic_t enable;
	atomic_t en_sig_motion;

	uint8_t stc_enable;
	uint16_t gpio_pin;
	uint8_t std;
	uint8_t sig_flag;
	uint8_t calib_status;
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct mutex mutex_value;

	int32_t IRQ;
	int32_t reg_sel;
	int32_t reg_len;
	int32_t smi_acc_mode_enabled;
	uint64_t timestamp;
	uint64_t time_odr;
	uint64_t base_time;
	uint64_t acc_count;
	uint8_t debug_level;
};



extern int32_t smi130_acc_probe(struct smi_client_data *client_data, struct device *dev);
extern int32_t smi130_acc_remove(struct device *dev);
extern int32_t smi130_acc_suspend(struct device *dev);
extern int32_t smi130_acc_resume(struct device *dev);
extern void smi130_acc_delay(u32 msec);


#endif