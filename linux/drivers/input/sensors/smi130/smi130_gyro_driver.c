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
/*! file <smi130_gyro_driver.c >
    brief <Linux Sensor driver for SMI130_GYRO> */
#include "smi130_gyro_driver.h"

static struct spi_device *smi_gyro_client;

static int32_t smi130_check_chip_id(struct smi_gyro_client_data *client_data);
static int32_t smi_gyro_reset(struct smi_gyro_client_data *client_data);
static void smi130_gyro_delay(SMI130_GYRO_U16 msec);
static uint64_t smi130_gyro_get_alarm_timestamp(void);
static void smi130_gyro_work_func(struct work_struct *work);
static enum hrtimer_restart reportdata_timer_fun(
        struct hrtimer *hrtimer);


static void smi130_gyro_delay(SMI130_GYRO_U16 msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, msec * 1000);
	else
		msleep(msec);
}

static void smi_gyro_work_func(struct work_struct *work)
{
	struct smi_gyro_client_data *client_data =
	        container_of((struct delayed_work *)work,
	                     struct smi_gyro_client_data, work);

	unsigned long delay =
	        msecs_to_jiffies(atomic_read(&client_data->delay));
	struct smi130_gyro_data_t gyro_data;

	SMI_GYRO_CALL_API(get_dataXYZ)(&gyro_data);

	input_event(client_data->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_X, gyro_data.datax);
	input_event(client_data->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_Y, gyro_data.datay);
	input_event(client_data->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_Z, gyro_data.dataz);
	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);
}

static struct workqueue_struct *reportdata_wq;

static uint64_t smi130_gyro_get_alarm_timestamp(void)
{
	uint64_t ts_ap;
	struct timespec tmp_time;
	get_monotonic_boottime(&tmp_time);
	ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000 + tmp_time.tv_nsec;
	return ts_ap;
}

static void smi130_gyro_work_func(struct work_struct *work)
{
	struct	smi_gyro_client_data *smi130_gyro =
	        container_of(work,
	                     struct smi_gyro_client_data, report_data_work);
	struct smi130_gyro_data_t gyro_lsb;

	smi130_gyro_get_dataXYZ(&gyro_lsb);

	input_event(smi130_gyro->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_X, gyro_lsb.datax);
	input_event(smi130_gyro->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_Y, gyro_lsb.datay);
	input_event(smi130_gyro->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_Z, gyro_lsb.dataz);
	input_sync(smi130_gyro->input);

}


static enum hrtimer_restart reportdata_timer_fun(
        struct hrtimer *hrtimer)
{
	struct smi_gyro_client_data *client_data =
	        container_of(hrtimer, struct smi_gyro_client_data, timer);
	int32_t delay = 0;
	delay = 10;
	queue_work(reportdata_wq, &(client_data->report_data_work));
	client_data->work_delay_kt = ns_to_ktime(delay*1000000);
	hrtimer_forward(hrtimer, ktime_get(), client_data->work_delay_kt);

	return HRTIMER_RESTART;
}

static ssize_t smi_gyro_show_enable_timer(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 16, "%d\n", client_data->is_timer_running);
}

static ssize_t smi_gyro_store_enable_timer(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	unsigned long data;
	int32_t error;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data) {
		if (0 == client_data->is_timer_running) {
			hrtimer_start(&client_data->timer,
			              ns_to_ktime(10000000),
			              HRTIMER_MODE_REL);
			client_data->is_timer_running = 1;
			client_data->base_time = 0;
			client_data->timestamp = 0;
			client_data->gyro_count = 0;
		}
	} else {
		if (1 == client_data->is_timer_running) {
			hrtimer_cancel(&client_data->timer);
			client_data->is_timer_running = 0;
			client_data->base_time = 0;
			client_data->timestamp = 0;
			client_data->gyro_count = 0;
		}
	}
	return count;
}

static ssize_t smi130_gyro_show_debug_level(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int32_t err;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	client_data->debug_level=get_debug_log_level();
	err = snprintf(buf, 8, "%d\n", client_data->debug_level);
	return err;
}
static ssize_t smi130_gyro_store_debug_level(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	int32_t ret = 0;
	unsigned long data;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	ret = kstrtoul(buf, 16, &data);
	if (ret)
		return ret;
	client_data->debug_level = (uint8_t)data;
	set_debug_log_level(client_data->debug_level);
	return count;
}

static ssize_t smi_gyro_show_chip_id(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	int32_t ret = 0;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	ret = smi130_check_chip_id(client_data);

	return snprintf(buf, 16, "0x%02x\n", ret);

}

static ssize_t smi_gyro_show_op_mode(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	int32_t ret;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = 0xff;

	mutex_lock(&client_data->mutex_op_mode);
	ret = SMI_GYRO_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&client_data->mutex_op_mode);

	ret = snprintf(buf, 16, "%d\n", op_mode);

	return ret;
}

static ssize_t smi_gyro_store_op_mode(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
	int32_t err;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;
	mutex_lock(&client_data->mutex_op_mode);

	err = SMI_GYRO_CALL_API(set_mode)(op_mode);

	mutex_unlock(&client_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}



static ssize_t smi_gyro_show_value(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	int32_t count;
	int32_t err;

	struct smi130_gyro_data_t value_data;
	err = SMI_GYRO_CALL_API(get_dataXYZ)(&value_data);

	count = snprintf(buf, 96, "%hd %hd %hd\n",
	                 value_data.datax,
	                 value_data.datay,
	                 value_data.dataz);

	return count;
}

static ssize_t smi_gyro_show_range(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	int32_t err;
	uint8_t range = 0;
	SMI_GYRO_CALL_API(get_range_reg)(&range);
	err = snprintf(buf, 16, "%d\n", range);
	return err;
}

static ssize_t smi_gyro_store_range(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	int32_t err;
	unsigned long range;
	err = kstrtoul(buf, 10, &range);
	if (err)
		return err;
	SMI_GYRO_CALL_API(set_range_reg)(range);
	return count;
}

/*
decimation    odr     filter bandwidth     bits
20	100HZ		32HZ		7
10	200Hz		64HZ		6
20	100HZ		12HZ		5
10	200hz		23HZ		4
5	400HZ		47HZ		3
2	1000HZ		116HZ		2
0	2000HZ		230HZ		1
0	2000HZ		Unfiltered(523HZ)	0
*/

static const uint64_t odr_map[8] = {
	500000, 500000, 1000000, 2500000, 5000000, 10000000, 5000000, 10000000
};

static ssize_t smi_gyro_show_bandwidth(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
	int32_t err;
	uint8_t bandwidth = 0;
	SMI_GYRO_CALL_API(get_bw)(&bandwidth);
	err = snprintf(buf, 16, "%d\n", bandwidth);
	return err;
}

static ssize_t smi_gyro_store_bandwidth(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int32_t err;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	unsigned long bandwidth;
	uint8_t op_mode = 0xff;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	/*
	set bandwidth only in the op_mode=0
	*/
	err = SMI_GYRO_CALL_API(get_mode)(&op_mode);
	if (op_mode == 0) {
		err += SMI_GYRO_CALL_API(set_bw)(bandwidth);
	} else {
		err += SMI_GYRO_CALL_API(set_mode)(0);
		err += SMI_GYRO_CALL_API(set_bw)(bandwidth);
		smi130_gyro_delay(1);
		err += SMI_GYRO_CALL_API(set_mode)(2);
		smi130_gyro_delay(3);
	}

	if (err)
		PERR("set failed");
	client_data->time_odr = odr_map[bandwidth];
	client_data->base_time = 0;
	client_data->gyro_count = 0;
	return count;
}


static ssize_t smi_gyro_show_enable(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	int32_t err;

	mutex_lock(&client_data->mutex_enable);
	err = snprintf(buf, 16, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t smi_gyro_store_enable(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	unsigned long data;
	int32_t err;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {
			schedule_delayed_work(
			        &client_data->work,
			        msecs_to_jiffies(atomic_read(
			                                 &client_data->delay)));
		} else {
			cancel_delayed_work_sync(&client_data->work);
		}

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t smi_gyro_show_delay(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, 16, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t smi_gyro_store_delay(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	unsigned long data;
	int32_t err;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < SMI_GYRO_DELAY_MIN)
		data = SMI_GYRO_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static ssize_t smi_gyro_show_selftest(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	int32_t err;
	uint8_t selftest;
	SMI_GYRO_CALL_API(selftest)(&selftest);
	err = snprintf(buf, 16, "%d\n", selftest);
	return err;
}

#ifdef SMI_GYRO_DEBUG
static ssize_t smi_gyro_store_softreset(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int32_t err;
	unsigned long softreset;
	err = kstrtoul(buf, 10, &softreset);
	if (err)
		return err;
	SMI_GYRO_CALL_API(set_soft_reset)();
	PINFO("softreset done");
	return count;
}

static ssize_t smi_gyro_show_register(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{

	size_t count = 0;
	uint8_t reg[0x40];
	int32_t i;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	for (i = 0; i < 0x40; i++) {
		client_data->device.bus_read(client_data->device.dev_addr, i, reg+i,1);
		count += snprintf(&buf[count], 0x40, "0x%02x: 0x%02x\n", i, reg[i]);
	}
	return count;

}

static ssize_t smi_gyro_store_register(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
	int32_t address, value;
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);

	sscanf(buf, "%3d %3d", &address, &value);

	if (client_data->device.bus_write(client_data->device.dev_addr, (uint8_t)address,
	                                  (uint8_t *)&value,1) < 0)
		return -EINVAL;
	return count;
}

#endif


static ssize_t smi130_gyro_driver_version_show(struct device *dev
                , struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct smi_gyro_client_data *client_data = input_get_drvdata(input);
	int32_t ret;

	if (client_data == NULL) {
		printk(KERN_ERR "Invalid client_data pointer");
		return -ENODEV;
	}

	ret = snprintf(buf, 128, "Driver version: %s\n",
	               DRIVER_VERSION);
	return ret;
}
static DEVICE_ATTR(chip_id, S_IRUSR,
                   smi_gyro_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO | S_IWUSR,
                   smi_gyro_show_op_mode, smi_gyro_store_op_mode);
static DEVICE_ATTR(value, S_IRUSR,
                   smi_gyro_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO | S_IWUSR,
                   smi_gyro_show_range, smi_gyro_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO | S_IWUSR,
                   smi_gyro_show_bandwidth, smi_gyro_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
                   smi_gyro_show_enable, smi_gyro_store_enable);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR,
                   smi_gyro_show_delay, smi_gyro_store_delay);
static DEVICE_ATTR(selftest, S_IRUGO,
                   smi_gyro_show_selftest, NULL);
static DEVICE_ATTR(enable_timer, S_IRUGO | S_IWUSR,
                   smi_gyro_show_enable_timer, smi_gyro_store_enable_timer);
static DEVICE_ATTR(debug_level, S_IRUGO | S_IWUSR,
                   smi130_gyro_show_debug_level, smi130_gyro_store_debug_level);
static DEVICE_ATTR(driver_version, S_IRUSR,
                   smi130_gyro_driver_version_show, NULL);
#ifdef SMI_GYRO_DEBUG
static DEVICE_ATTR(softreset, S_IWUSR,
                   NULL, smi_gyro_store_softreset);
static DEVICE_ATTR(reg_value, S_IRUSR| S_IWUSR,
                   smi_gyro_show_register, smi_gyro_store_register);
#endif


static struct attribute *smi_gyro_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_selftest.attr,
	&dev_attr_enable_timer.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_driver_version.attr,
#ifdef SMI_GYRO_DEBUG
	&dev_attr_softreset.attr,
	&dev_attr_reg_value.attr,
#endif
	NULL
};

static struct attribute_group smi_gyro_attribute_group = {
	.attrs = smi_gyro_attributes
};


static int32_t smi_gyro_input_init(struct smi_gyro_client_data *client_data)
{
	struct input_dev *dev;
	int32_t err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SMI130_GYRO_NAME;
	//dev->id.bustype = BUS_SPI;

	input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_X);
	input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y);
	input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z);
	input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void smi_gyro_input_destroy(struct smi_gyro_client_data *client_data)
{
	struct input_dev *dev = client_data->input;
	input_unregister_device(dev);
}

#ifdef CONFIG_SENSORS_SMI130_GYRO_ENABLE_NEWDATA_INT
static void smi130_gyro_irq_work_func(struct work_struct *work)
{
	struct smi_gyro_client_data *client_data = container_of(work,
	                struct smi_gyro_client_data, irq_work);
	struct smi130_gyro_data_t gyro_data;
	struct timespec ts;
	ts = ns_to_timespec(client_data->timestamp);
	SMI_GYRO_CALL_API(get_dataXYZ)(&gyro_data);

	input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME,
	            ts.tv_sec);
	input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME,
	            ts.tv_nsec);
	input_event(client_data->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_X, gyro_data.datax);
	input_event(client_data->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_Y, gyro_data.datay);
	input_event(client_data->input, INPUT_EVENT_TYPE,
	            INPUT_EVENT_Z, gyro_data.dataz);
	input_sync(client_data->input);
}

static irqreturn_t smi_gyro_irq_handler(int32_t irq, void *handle)
{
	struct smi_gyro_client_data *client_data = handle;
	client_data->timestamp= smi130_gyro_get_alarm_timestamp();
	schedule_work(&client_data->irq_work);
	return IRQ_HANDLED;
}
#endif

static int32_t smi130_check_chip_id(struct smi_gyro_client_data *client_data)
{
	int32_t err = 0;
	uint8_t chip_id;

	err = client_data->device.bus_read(client_data->device.dev_addr, SMI130_GYRO_CHIP_ID_ADDR,
	                                   &chip_id,1);
	if (err<0) {
		dev_err(client_data->dev, "failed to read chip_id register\n");
		return err;
	}
	return chip_id;
}
static int32_t smi_gyro_reset(struct smi_gyro_client_data *client_data)
{
	int32_t err = 0;
	uint8_t reset_value = 0xb6;
	err = client_data->device.bus_write(client_data->device.dev_addr, SMI130_GYRO_BGW_SOFTRESET_ADDR,
	                                    &reset_value,1);
	return err;

}
int32_t smi_gyro_probe(struct smi_gyro_client_data *client_data, struct device *dev)
{
	int32_t err = 0;
	PINFO("function entrance");

	err = smi130_check_chip_id(client_data);
	if(err==0x0F)
		PINFO("find Bosch smi130_gyro");
	else
		goto exit_err_clean;

	dev_set_drvdata(dev, client_data);
	client_data->dev = dev;

	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);

	/* do soft reset */
	smi130_gyro_delay(5);
	PINFO("gyro soft reset");
	err = smi_gyro_reset(client_data);
	smi130_gyro_delay(30);
	if (err < 0) {
		PERR("error soft reset!\n");
		err = -EINVAL;
		goto exit_err_clean;
	}
	smi130_gyro_delay(30);

	/* input device init */
	err = smi_gyro_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
	                         &smi_gyro_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;

	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, smi_gyro_work_func);
	atomic_set(&client_data->delay, SMI_GYRO_DELAY_DEFAULT);

	client_data->device.delay_msec = smi130_gyro_delay;
	err = SMI_GYRO_CALL_API(init)(&client_data->device);
	if(err<0)
		PINFO("failed to read chip_id\n");

	client_data->enable = 0;

	/*workqueue init*/
	INIT_WORK(&client_data->report_data_work,
	          smi130_gyro_work_func);
	reportdata_wq = create_singlethread_workqueue("smi130_gyro_wq");
	if (NULL == reportdata_wq)
		PERR("fail to create the reportdta_wq %d", -ENOMEM);
	hrtimer_init(&client_data->timer, CLOCK_MONOTONIC,
	             HRTIMER_MODE_REL);
	client_data->timer.function = reportdata_timer_fun;
	client_data->work_delay_kt = ns_to_ktime(10000000);
	client_data->is_timer_running = 0;
	client_data->time_odr = 500000;


	/*default odr is 100HZ, filter BW 32Hz*/
	err += SMI_GYRO_CALL_API(set_bw)(C_SMI130_GYRO_BW_32Hz_U8X);
	/*set gyro range to 2000 */
	err += SMI_GYRO_CALL_API(set_range_reg)(C_SMI130_GYRO_RANGE_2000);
	smi130_gyro_delay(5);
#ifdef CONFIG_SENSORS_SMI130_GYRO_ENABLE_NEWDATA_INT
	/*config the interrupt and map the interrupt*/
	/*high level trigger*/
	err += smi130_gyro_set_int_lvl(SMI130_GYRO_INT1_DATA, SMI_INT1_LVL_ACTIVE_HIGH);
	smi130_gyro_delay(5);
	err += smi130_gyro_set_int_od(SMI130_GYRO_INT1, SMI_INT1_TYPE_PUSH_PULL);
	smi130_gyro_delay(5);
	err += smi130_gyro_set_int_data(SMI130_GYRO_INT1_DATA, SMI130_GYRO_ENABLE);
	smi130_gyro_delay(5);
	/*enable new data interrupt*/
	err += smi130_gyro_set_data_en(SMI130_GYRO_ENABLE);
	if (err<0)
		PERR("config sensor data ready interrupt failed");
	smi130_gyro_delay(5);
	client_data->gpio_pin = of_get_named_gpio_flags(
	                                dev->of_node,
	                                "smi130_gyro,gpio_irq", 0, NULL);
	PDEBUG("smi130_gyro gpio number:%d\n", client_data->gpio_pin);
	err = gpio_request_one(client_data->gpio_pin,
	                       GPIOF_IN, "smi130_gyro");
	if (err < 0) {
		PDEBUG("request gpio failed\n");
		client_data->gpio_pin = 0;
	}
	if (client_data->gpio_pin != 0) {
		err = gpio_direction_input(client_data->gpio_pin);
		if (err < 0) {
			PDEBUG("request failed\n");
		}
		client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
		err = request_irq(client_data->IRQ, smi_gyro_irq_handler,
		                  IRQF_TRIGGER_RISING,
		                  SMI130_GYRO_NAME, client_data);
		if (err < 0)
			PDEBUG("request handle failed\n");
	}
	INIT_WORK(&client_data->irq_work, smi130_gyro_irq_work_func);
#endif

	/*
	    err = SMI_GYRO_CALL_API(set_mode)(SMI130_GYRO_MODE_DEEPSUSPEND);
	    if (err<0)
	    dev_err(client_data->dev, "Failed sensor config\n");
	    else
	    PINFO("set smi130_gyro to deep_suspend mode\n");
	*/
	PINFO("sensor %s probed successfully", SMI130_GYRO_NAME);

	return 0;

exit_err_sysfs:
	if (err)
		smi_gyro_input_destroy(client_data);

exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			kfree(client_data);
			client_data = NULL;
		}

		smi_gyro_client = NULL;
	}

	return err;
}

int32_t smi_gyro_pre_suspend(struct device *dev)
{
	int32_t err = 0;
	struct smi_gyro_client_data *client_data = dev_get_drvdata(dev);
	PINFO("pre_suspend function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		PINFO("cancel work");
	}
	mutex_unlock(&client_data->mutex_enable);
	if (client_data->is_timer_running) {
		hrtimer_cancel(&client_data->timer);
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->gyro_count = 0;
	}
	return err;
}

static int32_t smi_gyro_post_resume(struct device *dev)
{
	int32_t err = 0;
	struct smi_gyro_client_data *client_data = dev_get_drvdata(dev);
	PINFO("post_resume function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
		                      msecs_to_jiffies(
		                              atomic_read(&client_data->delay)));
	}
	mutex_unlock(&client_data->mutex_enable);
	if (client_data->is_timer_running) {
		hrtimer_start(&client_data->timer,
		              ns_to_ktime(client_data->time_odr),
		              HRTIMER_MODE_REL);
		client_data->base_time = 0;
		client_data->timestamp = 0;
		client_data->is_timer_running = 1;
		client_data->gyro_count = 0;
	}
	return err;
}


int32_t smi_gyro_suspend(struct device *dev, pm_message_t mesg)
{
	int32_t err = 0;

	struct smi_gyro_client_data *client_data = dev_get_drvdata(dev);
	PINFO("suspend function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	if (client_data->enable) {
		err = smi_gyro_pre_suspend(dev);
		err = SMI_GYRO_CALL_API(set_mode)(
		              SMI_GYRO_VAL_NAME(MODE_DEEPSUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}

int32_t smi_gyro_resume(struct device *dev)
{

	int32_t err = 0;
	struct smi_gyro_client_data *client_data = dev_get_drvdata(dev);
	PINFO("resume function entrance");
	mutex_lock(&client_data->mutex_op_mode);

	if (client_data->enable)
		err = SMI_GYRO_CALL_API(set_mode)(SMI_GYRO_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	smi_gyro_post_resume(dev);

	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}

void smi_gyro_shutdown(struct device *dev)
{

	struct smi_gyro_client_data *client_data = dev_get_drvdata(dev);
	PINFO("shutdown function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	SMI_GYRO_CALL_API(set_mode)(
	        SMI_GYRO_VAL_NAME(MODE_DEEPSUSPEND));
	mutex_unlock(&client_data->mutex_op_mode);
}

int32_t smi_gyro_remove(struct device *dev)
{
	int32_t err = 0;
	uint8_t op_mode;

	struct smi_gyro_client_data *client_data = dev_get_drvdata(dev);
	PINFO("pre_suspend function entrance");

	if (NULL != client_data) {
		mutex_lock(&client_data->mutex_op_mode);
		SMI_GYRO_CALL_API(get_mode)(&op_mode);
		if (SMI_GYRO_VAL_NAME(MODE_NORMAL) == op_mode) {
			cancel_delayed_work_sync(&client_data->work);
			PINFO("cancel work");
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = SMI_GYRO_CALL_API(set_mode)(
		              SMI_GYRO_VAL_NAME(MODE_DEEPSUSPEND));
		smi130_gyro_delay(1);

		sysfs_remove_group(&client_data->input->dev.kobj,
		                   &smi_gyro_attribute_group);
		smi_gyro_input_destroy(client_data);
		kfree(client_data);

		smi_gyro_client = NULL;
	}

	return err;
}

