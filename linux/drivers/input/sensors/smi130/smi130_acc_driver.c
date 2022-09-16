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
/*! file <smi130_acc_driver.c >
    brief <Linux Sensor driver for SMI130_ACC> */

#include "smi130_acc_driver.h"

void smi130_acc_delay(uint32_t msec) {
  if (msec <= 20)
    usleep_range(msec * 1000, msec * 1000);
  else
    msleep(msec);
}

static ssize_t smi130_acc_enable_int_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count) {
  int32_t type, value;

  sscanf(buf, "%3d %3d", &type, &value);

  if (smi130_acc_set_intr_enable(type, value) < 0) return -EINVAL;

  return count;
}

static ssize_t smi130_acc_int_mode_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_latch_intr(&data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_int_mode_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_latch_intr((uint8_t)data) < 0) return -EINVAL;

  return count;
}

static ssize_t smi130_acc_slope_duration_show(struct device *dev,
                                              struct device_attribute *attr,
                                              char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_durn(SMI130_ACC_SLOPE_DURN, &data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_slope_duration_store(struct device *dev,
                                               struct device_attribute *attr,
                                               const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_durn(SMI130_ACC_SLOPE_DURN, (uint8_t)data) < 0)
    return -EINVAL;

  return count;
}

static ssize_t smi130_acc_slope_threshold_show(struct device *dev,
                                               struct device_attribute *attr,
                                               char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_thres(SMI130_ACC_SLOPE_THRES, &data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_slope_threshold_store(struct device *dev,
                                                struct device_attribute *attr,
                                                const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;
  if (smi130_acc_set_thres(SMI130_ACC_SLOPE_THRES, (uint8_t)data) < 0)
    return -EINVAL;

  return count;
}

static ssize_t smi130_acc_softreset_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count) {
  if (smi130_acc_soft_rst() < 0) return -EINVAL;

  return count;
}

static ssize_t smi130_acc_selftest_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  return snprintf(buf, 16, "%d\n", atomic_read(&client_data->selftest_result));
}

static ssize_t smi130_acc_selftest_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  unsigned long data;
  uint8_t clear_value = 0;
  int32_t error;
  short value1 = 0;
  short value2 = 0;
  short diff = 0;
  unsigned long result = 0;
  uint8_t test_result_branch = 0;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  smi130_acc_soft_rst();
  smi130_acc_delay(200);
  error = kstrtoul(buf, 10, &data);
  if (error) return error;
  if (data != 1) return -EINVAL;

  smi130_acc_write_reg(SMI130_ACC_SELFTEST_ADDR, &clear_value,
                       SMI130_ACC_GEN_READ_WRITE_LENGTH);
  /* set to 8 G range */
  if (smi130_acc_set_range(SMI130_ACC_RANGE_8G) < 0) return -EINVAL;
  /* set bandwidth to 1000Hz  */
  if (smi130_acc_set_bw(SMI130_ACC_BW_1000HZ) < 0) return -EINVAL;
  smi130_acc_set_selftest_amp(1);
  /* 1 for x-axis*/
  smi130_acc_set_selftest_axis(1);
  smi130_acc_set_selftest_sign(1);
  smi130_acc_delay(50);
  smi130_acc_read_x(&value1);
  smi130_acc_set_selftest_sign(0);
  smi130_acc_delay(50);
  smi130_acc_read_x(&value2);
  diff = value1 - value2;

  PINFO("diff x is %d,value1 is %d, value2 is %d\n", diff, value1, value2);
  test_result_branch = 1;
  /*x-axis LSB diff should >= (256LSB/g * 800mg/1000 = 205LSB)*/
  if (abs(diff) < 205) result |= test_result_branch;

  /* 2 for y-axis*/
  smi130_acc_set_selftest_axis(2);
  smi130_acc_set_selftest_sign(1);
  smi130_acc_delay(50);
  smi130_acc_read_y(&value1);
  smi130_acc_set_selftest_sign(0);
  smi130_acc_delay(50);
  smi130_acc_read_y(&value2);
  diff = value1 - value2;
  PINFO("diff y is %d,value1 is %d, value2 is %d\n", diff, value1, value2);
  test_result_branch = 2;
  /*y-axis LSB diff should >= (256LSB/g * 800mg/1000 = 205LSB)*/
  if (abs(diff) < 205) result |= test_result_branch;

  smi130_acc_delay(50);
  /* 3 for z-axis*/
  smi130_acc_set_selftest_axis(3);
  smi130_acc_set_selftest_sign(1);
  smi130_acc_delay(50);
  smi130_acc_read_z(&value1);
  smi130_acc_set_selftest_sign(0);
  smi130_acc_delay(50);
  smi130_acc_read_z(&value2);
  diff = value1 - value2;

  PINFO("diff z is %d,value1 is %d, value2 is %d\n", diff, value1, value2);
  test_result_branch = 4;
  /*z-axis LSB diff should >= (256LSB/g * 400mg/1000 = 103LSB)*/
  if (abs(diff) < 103) result |= test_result_branch;

  atomic_set(&client_data->selftest_result, (uint32_t)result);

  smi130_acc_soft_rst();
  smi130_acc_delay(200);
  PINFO("self test result:%ld\n", result);

  return count;
}

static struct workqueue_struct *reportdata_wq;

uint64_t smi130_acc_get_alarm_timestamp(void) {
  uint64_t ts_ap;
  struct timespec64 tmp_time;          // adapt to Kernel v5.15
  ktime_get_boottime_ts64(&tmp_time);  // adapt to Kernel v5.15
  ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000 + tmp_time.tv_nsec;
  return ts_ap;
}

static void smi130_acc_timer_work_fun(struct work_struct *work) {
  struct smi_client_data *client_data =
      container_of(work, struct smi_client_data, report_data_work);

  struct smi130_acc_data acc_lsb;
  struct timespec64 ts;  // adapt to Kernel v5.15
  smi130_acc_read_xyz(&acc_lsb);
  ts = ns_to_timespec64(client_data->timestamp);  // adapt to Kernel v5.15
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME,
              ts.tv_sec);
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME,
              ts.tv_nsec);
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_X, acc_lsb.x);
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, acc_lsb.y);
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, acc_lsb.z);
  input_sync(client_data->input);
}
static enum hrtimer_restart reportdata_timer_fun(struct hrtimer *hrtimer) {
  struct smi_client_data *client_data =
      container_of(hrtimer, struct smi_client_data, timer);
  int32_t delay = 0;
  delay = 8;
  queue_work(reportdata_wq, &(client_data->report_data_work));
  /*set delay 8ms*/
  client_data->work_delay_kt = ns_to_ktime(delay * 1000000);
  hrtimer_forward(hrtimer, ktime_get(), client_data->work_delay_kt);

  return HRTIMER_RESTART;
}

static ssize_t smi130_acc_enable_timer_show(struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  return snprintf(buf, 16, "%d\n", client_data->is_timer_running);
}

static ssize_t smi130_acc_enable_timer_store(struct device *dev,
                                             struct device_attribute *attr,
                                             const char *buf, size_t count) {
  unsigned long data;
  int32_t error;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);
  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (data) {
    if (0 == client_data->is_timer_running) {
      hrtimer_start(&client_data->timer, ns_to_ktime(1000000),
                    HRTIMER_MODE_REL);
      client_data->base_time = 0;
      client_data->timestamp = 0;
      client_data->is_timer_running = 1;
    }
  } else {
    if (1 == client_data->is_timer_running) {
      hrtimer_cancel(&client_data->timer);
      client_data->is_timer_running = 0;
      client_data->base_time = 0;
      client_data->timestamp = 0;
      client_data->acc_count = 0;
    }
  }
  return count;
}

static ssize_t smi130_acc_debug_level_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf) {
  int32_t err;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);
  client_data->debug_level = get_debug_log_level();
  err = snprintf(buf, 8, "%d\n", client_data->debug_level);
  return err;
}
static ssize_t smi130_acc_debug_level_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t count) {
  int32_t ret = 0;
  unsigned long data;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  ret = kstrtoul(buf, 16, &data);
  if (ret) return ret;
  client_data->debug_level = data;
  set_debug_log_level(client_data->debug_level);
  return count;
}

static ssize_t smi130_acc_register_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  int32_t address, value;

  sscanf(buf, "%3d %3d", &address, &value);
  if (smi130_acc_write_reg((uint8_t)address, (uint8_t *)&value,
                           SMI130_ACC_GEN_READ_WRITE_LENGTH) < 0)
    return -EINVAL;
  return count;
}
static ssize_t smi130_acc_register_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  size_t count = 0;
  u8 reg[0x40];
  int32_t i;

  for (i = 0; i < 0x40; i++) {
    smi130_acc_read_reg(i, reg + i, 1);
    count += snprintf(&buf[count], 0x40, "0x%02x: 0x%02x\n", i, reg[i]);
  }
  return count;
}

static ssize_t smi130_acc_range_show(struct device *dev,
                                     struct device_attribute *attr, char *buf) {
  uint8_t data;

  if (smi130_acc_get_range(&data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_range_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count) {
  unsigned long data;
  int32_t err;

  err = kstrtoul(buf, 10, &data);
  if (err) return err;
  if (smi130_acc_set_range((uint8_t)data) < 0) return -EINVAL;

  return count;
}

static ssize_t smi130_acc_bandwidth_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_bw(&data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_bandwidth_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_bw((uint8_t)data) < 0) return -EINVAL;
  return count;
}

static ssize_t smi130_acc_mode_show(struct device *dev,
                                    struct device_attribute *attr, char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_power_mode(&data) < 0) return -EINVAL;

  return snprintf(buf, 32, "%d\n", data);
}

static ssize_t smi130_acc_mode_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;
  if (smi130_acc_set_power_mode(data) < 0) return -EINVAL;

  return count;
}

static ssize_t smi130_acc_value_show(struct device *dev,
                                     struct device_attribute *attr, char *buf) {
  struct smi130_acc_data acc_value;
  smi130_acc_read_xyz(&acc_value);
  return snprintf(buf, 96, "%d %d %d\n", acc_value.x, acc_value.y, acc_value.z);
}
static ssize_t smi130_acc_delay_show(struct device *dev,
                                     struct device_attribute *attr, char *buf) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  return snprintf(buf, 16, "%d\n", atomic_read(&client_data->delay));
}

static ssize_t smi130_acc_chip_id_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf) {
  int32_t ret = 0;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  ret = client_data->device.chip_id;

  return snprintf(buf, 16, "0x%x\n", ret);
}

static ssize_t smi130_acc_delay_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count) {
  unsigned long data;
  int32_t error;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  error = kstrtoul(buf, 10, &data);
  if (error) return error;
  if (data > SMI130_ACC_MAX_DELAY) data = SMI130_ACC_MAX_DELAY;
  atomic_set(&client_data->delay, (uint32_t)data);

  return count;
}

static ssize_t smi130_acc_enable_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  return snprintf(buf, 16, "%d\n", atomic_read(&client_data->wkqueue_en));
}

static void smi130_acc_set_enable(struct device *dev, int32_t enable) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);
  int32_t pre_enable = atomic_read(&client_data->wkqueue_en);

  mutex_lock(&client_data->mutex_enable);
  if (enable) {
    if (pre_enable == 0) {
      smi130_acc_set_power_mode(SMI130_ACC_MODE_NORMAL);

#ifndef CONFIG_SENSORS_SMI130_ACC_ENABLE_NEWDATA_INT
      schedule_delayed_work(&client_data->work,
                            msecs_to_jiffies(atomic_read(&client_data->delay)));
#endif
      atomic_set(&client_data->wkqueue_en, 1);
    }

  } else {
    if (pre_enable == 1) {
      smi130_acc_set_power_mode(SMI130_ACC_MODE_STANDBY);

#ifndef CONFIG_SENSORS_SMI130_ACC_ENABLE_NEWDATA_INT
      cancel_delayed_work_sync(&client_data->work);
#endif
      atomic_set(&client_data->wkqueue_en, 0);
    }
  }
  mutex_unlock(&client_data->mutex_enable);
}

static ssize_t smi130_acc_enable_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;
  if ((data == 0) || (data == 1)) smi130_acc_set_enable(dev, data);

  return count;
}
static ssize_t smi130_acc_fast_calibration_x_show(struct device *dev,
                                                  struct device_attribute *attr,
                                                  char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_offset_target(SMI130_ACC_OFFSET_TRIGGER_X, &data) < 0)
    return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_fast_calibration_x_store(
    struct device *dev, struct device_attribute *attr, const char *buf,
    size_t count) {
  unsigned long data;
  int8_t tmp;
  uint8_t timeout = 0;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_offset_target(SMI130_ACC_OFFSET_TRIGGER_X, (uint8_t)data) <
      0)
    return -EINVAL;

  if (smi130_acc_set_cal_trigger(SMI130_ACC_OFFSET_TRIGGER_X) < 0)
    return -EINVAL;

  do {
    smi130_acc_delay(2);
    smi130_acc_get_cal_rdy(&tmp);

    /*PINFO("wait 2ms cal ready flag is %d\n", tmp); */
    timeout++;
    if (timeout == 50) {
      PINFO("get fast calibration ready error\n");
      return -EINVAL;
    };

  } while (tmp == 0);

  PINFO("x axis fast calibration finished\n");
  return count;
}

static ssize_t smi130_acc_fast_calibration_y_show(struct device *dev,
                                                  struct device_attribute *attr,
                                                  char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_offset_target(SMI130_ACC_OFFSET_TRIGGER_Y, &data) < 0)
    return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_fast_calibration_y_store(
    struct device *dev, struct device_attribute *attr, const char *buf,
    size_t count) {
  unsigned long data;
  int8_t tmp;
  uint8_t timeout = 0;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_offset_target(SMI130_ACC_OFFSET_TRIGGER_Y, (uint8_t)data) <
      0)
    return -EINVAL;

  if (smi130_acc_set_cal_trigger(SMI130_ACC_OFFSET_TRIGGER_Y) < 0)
    return -EINVAL;

  do {
    smi130_acc_delay(2);
    smi130_acc_get_cal_rdy(&tmp);

    /*PINFO("wait 2ms cal ready flag is %d\n", tmp);*/
    timeout++;
    if (timeout == 50) {
      PINFO("get fast calibration ready error\n");
      return -EINVAL;
    };

  } while (tmp == 0);

  PINFO("y axis fast calibration finished\n");
  return count;
}

static ssize_t smi130_acc_fast_calibration_z_show(struct device *dev,
                                                  struct device_attribute *attr,
                                                  char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_offset_target(SMI130_ACC_OFFSET_TRIGGER_Z, &data) < 0)
    return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_fast_calibration_z_store(
    struct device *dev, struct device_attribute *attr, const char *buf,
    size_t count) {
  unsigned long data;
  int8_t tmp;
  uint8_t timeout = 0;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_offset_target(SMI130_ACC_OFFSET_TRIGGER_Z,
                                   (unsigned char)data) < 0)
    return -EINVAL;

  if (smi130_acc_set_cal_trigger(SMI130_ACC_OFFSET_TRIGGER_Z) < 0)
    return -EINVAL;

  do {
    smi130_acc_delay(2);
    smi130_acc_get_cal_rdy(&tmp);

    /*PINFO("wait 2ms cal ready flag is %d\n", tmp);*/
    timeout++;
    if (timeout == 50) {
      PINFO("get fast calibration ready error\n");
      return -EINVAL;
    };

  } while (tmp == 0);

  PINFO("z axis fast calibration finished\n");
  return count;
}

static ssize_t smi130_acc_sleepdur_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_sleep_durn(&data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_sleepdur_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;
  if (smi130_acc_set_sleep_durn((uint8_t)data) < 0) return -EINVAL;

  return count;
}

static ssize_t smi130_acc_temperature_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf) {
  uint8_t data = 0;

  if (smi130_acc_read_temp(&data) < 0) return -EINVAL;

  return snprintf(buf, 16, "%d\n", data);
}
static ssize_t smi130_acc_offset_x_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_offset(SMI130_ACC_X_AXIS, &data) < 0)
    return snprintf(buf, 48, "Read error\n");

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_offset_x_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_offset(SMI130_ACC_X_AXIS, (unsigned char)data) < 0)
    return -EINVAL;

  return count;
}

static ssize_t smi130_acc_offset_y_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_offset(SMI130_ACC_Y_AXIS, &data) < 0)
    return snprintf(buf, 48, "Read error\n");

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_offset_y_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_offset(SMI130_ACC_Y_AXIS, (unsigned char)data) < 0)
    return -EINVAL;

  return count;
}

static ssize_t smi130_acc_offset_z_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  uint8_t data = 0;

  if (smi130_acc_get_offset(SMI130_ACC_Z_AXIS, &data) < 0)
    return snprintf(buf, 48, "Read error\n");

  return snprintf(buf, 16, "%d\n", data);
}

static ssize_t smi130_acc_offset_z_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count) {
  unsigned long data;
  int32_t error;

  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if (smi130_acc_set_offset(SMI130_ACC_Z_AXIS, (unsigned char)data) < 0)
    return -EINVAL;

  return count;
}

static ssize_t smi130_acc_driver_version_show(struct device *dev,
                                              struct device_attribute *attr,
                                              char *buf) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);
  int32_t ret;

  if (client_data == NULL) {
    printk(KERN_ERR "Invalid client_data pointer");
    return -ENODEV;
  }

  ret = snprintf(buf, 128, "Driver version: %s\n", DRIVER_VERSION);
  return ret;
}

#ifdef CONFIG_SENSORS_SMI130_ACC_ENABLE_MOTION
static int32_t smi130_acc_set_en_slope_int(int32_t en) {
  int32_t err = 0;

  if (en) {
    /* Set the related parameters which needs to be fine tuned by
     * interfaces: slope_threshold and slope_duration
     */
    err = smi130_acc_set_durn(SMI130_ACC_SLOPE_DURN, 0x0);
    err += smi130_acc_set_thres(SMI130_ACC_SLOPE_THRES, 0x16);

    /*Enable the interrupts*/
    err += smi130_acc_set_intr_enable(SMI130_ACC_SLOPE_X_INTR,
                                      INTR_ENABLE); /*Slope X*/
    err += smi130_acc_set_intr_enable(SMI130_ACC_SLOPE_Y_INTR,
                                      INTR_ENABLE); /*Slope Y*/
    err += smi130_acc_set_intr_enable(SMI130_ACC_SLOPE_Z_INTR,
                                      INTR_ENABLE); /*Slope Z*/

    err += smi130_acc_set_intr_slope(SMI130_ACC_INTR2_SLOPE, INTR_ENABLE);

  } else {
    err += smi130_acc_set_intr_enable(SMI130_ACC_SLOPE_X_INTR,
                                      INTR_DISABLE); /*Slope X*/
    err += smi130_acc_set_intr_enable(SMI130_ACC_SLOPE_Y_INTR,
                                      INTR_DISABLE); /*Slope Y*/
    err += smi130_acc_set_intr_enable(SMI130_ACC_SLOPE_Z_INTR,
                                      INTR_DISABLE); /*Slope Z*/
  }
  return err;
}

static ssize_t smi130_acc_en_sig_motion_show(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf) {
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);

  return snprintf(buf, 16, "%d\n", atomic_read(&client_data->en_sig_motion));
}

static int32_t smi130_acc_set_en_sig_motion(struct smi_client_data *client_data,
                                            int32_t en) {
  int32_t err = 0;

  en = (en >= 1) ? 1 : 0; /* set sig motion sensor status */

  if (atomic_read(&client_data->en_sig_motion) != en) {
    if (en) {
      err = smi130_acc_set_power_mode(SMI130_ACC_MODE_NORMAL);
      err = smi130_acc_set_en_slope_int(en);
      enable_irq_wake(client_data->IRQ);
    } else {
      disable_irq_wake(client_data->IRQ);
      err = smi130_acc_set_en_slope_int(en);
      err = smi130_acc_set_power_mode(SMI130_ACC_MODE_STANDBY);
    }
    atomic_set(&client_data->en_sig_motion, en);
  }
  return err;
}

static ssize_t smi130_acc_en_sig_motion_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf, size_t count) {
  unsigned long data;
  int32_t error;
  struct input_dev *input = to_input_dev(dev);
  struct smi_client_data *client_data = input_get_drvdata(input);
  error = kstrtoul(buf, 10, &data);
  if (error) return error;

  if ((data == 0) || (data == 1))
    smi130_acc_set_en_sig_motion(client_data, data);

  return count;
}
#endif

static DEVICE_ATTR(range, S_IRUGO | S_IWUSR, smi130_acc_range_show,
                   smi130_acc_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO | S_IWUSR, smi130_acc_bandwidth_show,
                   smi130_acc_bandwidth_store);
static DEVICE_ATTR(op_mode, S_IRUGO | S_IWUSR, smi130_acc_mode_show,
                   smi130_acc_mode_store);
static DEVICE_ATTR(value, S_IRUSR, smi130_acc_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR, smi130_acc_delay_show,
                   smi130_acc_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, smi130_acc_enable_show,
                   smi130_acc_enable_store);
static DEVICE_ATTR(sleepdur, S_IRUGO | S_IWUSR, smi130_acc_sleepdur_show,
                   smi130_acc_sleepdur_store);
static DEVICE_ATTR(fast_calibration_x, S_IRUGO | S_IWUSR,
                   smi130_acc_fast_calibration_x_show,
                   smi130_acc_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, S_IRUGO | S_IWUSR,
                   smi130_acc_fast_calibration_y_show,
                   smi130_acc_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, S_IRUGO | S_IWUSR,
                   smi130_acc_fast_calibration_z_show,
                   smi130_acc_fast_calibration_z_store);
static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, smi130_acc_register_show,
                   smi130_acc_register_store);
static DEVICE_ATTR(chip_id, S_IRUSR, smi130_acc_chip_id_show, NULL);
static DEVICE_ATTR(offset_x, S_IRUGO | S_IWUSR, smi130_acc_offset_x_show,
                   smi130_acc_offset_x_store);
static DEVICE_ATTR(offset_y, S_IRUGO | S_IWUSR, smi130_acc_offset_y_show,
                   smi130_acc_offset_y_store);
static DEVICE_ATTR(offset_z, S_IRUGO | S_IWUSR, smi130_acc_offset_z_show,
                   smi130_acc_offset_z_store);
static DEVICE_ATTR(enable_int, S_IWUSR, NULL, smi130_acc_enable_int_store);
static DEVICE_ATTR(int_mode, S_IRUGO | S_IWUSR, smi130_acc_int_mode_show,
                   smi130_acc_int_mode_store);
static DEVICE_ATTR(slope_duration, S_IRUGO | S_IWUSR,
                   smi130_acc_slope_duration_show,
                   smi130_acc_slope_duration_store);
static DEVICE_ATTR(slope_threshold, S_IRUGO | S_IWUSR,
                   smi130_acc_slope_threshold_show,
                   smi130_acc_slope_threshold_store);
static DEVICE_ATTR(selftest, S_IRUGO | S_IWUSR, smi130_acc_selftest_show,
                   smi130_acc_selftest_store);
static DEVICE_ATTR(softreset, S_IWUSR, NULL, smi130_acc_softreset_store);
static DEVICE_ATTR(enable_timer, S_IRUGO | S_IWUSR,
                   smi130_acc_enable_timer_show, smi130_acc_enable_timer_store);
static DEVICE_ATTR(debug_level, S_IRUGO | S_IWUSR, smi130_acc_debug_level_show,
                   smi130_acc_debug_level_store);
static DEVICE_ATTR(temperature, S_IRUSR, smi130_acc_temperature_show, NULL);
static DEVICE_ATTR(driver_version, S_IRUSR, smi130_acc_driver_version_show,
                   NULL);

#ifdef CONFIG_SENSORS_SMI130_ACC_ENABLE_MOTION
static DEVICE_ATTR(en_sig_motion, S_IRUGO | S_IWUSR,
                   smi130_acc_en_sig_motion_show,
                   smi130_acc_en_sig_motion_store);
#endif

static struct attribute *smi130_acc_attributes[] = {
    &dev_attr_range.attr,
    &dev_attr_bandwidth.attr,
    &dev_attr_op_mode.attr,
    &dev_attr_value.attr,
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_sleepdur.attr,
    &dev_attr_reg.attr,
    &dev_attr_fast_calibration_x.attr,
    &dev_attr_fast_calibration_y.attr,
    &dev_attr_fast_calibration_z.attr,
    &dev_attr_chip_id.attr,
    &dev_attr_offset_x.attr,
    &dev_attr_offset_y.attr,
    &dev_attr_offset_z.attr,
    &dev_attr_enable_int.attr,
    &dev_attr_enable_timer.attr,
    &dev_attr_debug_level.attr,
    &dev_attr_int_mode.attr,
    &dev_attr_slope_duration.attr,
    &dev_attr_slope_threshold.attr,
    &dev_attr_selftest.attr,
    &dev_attr_softreset.attr,
    &dev_attr_temperature.attr,
    &dev_attr_driver_version.attr,
#ifdef CONFIG_SENSORS_SMI130_ACC_ENABLE_MOTION
    &dev_attr_en_sig_motion.attr,
#endif
    NULL};

static struct attribute_group smi130_acc_attribute_group = {
    .attrs = smi130_acc_attributes};

#if defined(CONFIG_SENSORS_SMI130_ENABLE_INTERRUPT)

#ifdef CONFIG_SENSORS_SMI130_ACC_ENABLE_MOTION

static void smi130_acc_slope_interrupt_handle(
    struct smi_client_data *client_data) {
  uint8_t slope_sign = 0;
  uint8_t slope_first_x = 0;
  uint8_t slope_first_y = 0;
  uint8_t slope_first_z = 0;
  uint8_t data = 0;
  smi130_acc_get_intr_slope_stat(&data);
  slope_sign = SMI130_ACC_GET_BITSLICE(data, SMI130_ACC_SLOPE_SIGN_STAT);
  slope_first_x = SMI130_ACC_GET_BITSLICE(data, SMI130_ACC_SLOPE_FIRST_X);
  slope_first_y = SMI130_ACC_GET_BITSLICE(data, SMI130_ACC_SLOPE_FIRST_Y);
  slope_first_z = SMI130_ACC_GET_BITSLICE(data, SMI130_ACC_SLOPE_FIRST_Z);
  if ((slope_sign == POSITIVE) && (slope_first_x == TRUE)) {
    input_report_rel(client_data->input, SLOPE_INTERRUPT, SLOPE_INTERRUPT_X);
  }
  if ((slope_sign == POSITIVE) && (slope_first_y == TRUE)) {
    input_report_rel(client_data->input, SLOPE_INTERRUPT, SLOPE_INTERRUPT_Y);
  }
  if ((slope_sign == POSITIVE) && (slope_first_z == TRUE)) {
    input_report_rel(client_data->input, SLOPE_INTERRUPT, SLOPE_INTERRUPT_Z);
  }
  if ((slope_sign == NEGATIVE) && (slope_first_x == TRUE)) {
    input_report_rel(client_data->input, SLOPE_INTERRUPT, SLOPE_INTERRUPT_X_N);
  }
  if ((slope_sign == NEGATIVE) && (slope_first_x == TRUE)) {
    input_report_rel(client_data->input, SLOPE_INTERRUPT, SLOPE_INTERRUPT_Y_N);
  }
  if ((slope_sign == NEGATIVE) && (slope_first_x == TRUE)) {
    input_report_rel(client_data->input, SLOPE_INTERRUPT, SLOPE_INTERRUPT_Z_N);
  }
  PINFO("slope status is %d,\n", data);
}
#endif

static void smi130_acc_irq_work_func(struct work_struct *work) {
  struct smi_client_data *client_data = container_of(
      (struct work_struct *)work, struct smi_client_data, irq_work);

  uint8_t status = 0;

#ifdef CONFIG_SENSORS_SMI130_ACC_ENABLE_NEWDATA_INT
  static struct smi130_acc_data acc;
  struct timespec64 ts;

  smi130_acc_read_xyz(&acc);
  ts = ns_to_timespec64(client_data->timestamp);
  if ((acc.x & SMI130_ACC_NEW_DATA_X_MSK) &&
      (acc.y & SMI130_ACC_NEW_DATA_Y_MSK) &&
      (acc.x & SMI130_ACC_NEW_DATA_Z_MSK)) {
    input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME,
                ts.tv_sec);
    input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_TIME,
                ts.tv_nsec);
    input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_X, acc.x);
    input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, acc.y);
    input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, acc.z);
    input_sync(client_data->input);
    mutex_lock(&client_data->mutex_value);
    client_data->value = acc;
    mutex_unlock(&client_data->mutex_value);
    // PINFO("acc_value, x = %d, y = %d, z = %d\n", acc.x, acc.y, acc.z);
  }
#endif

  smi130_acc_get_intr_stat(&status);

#ifdef CONFIG_SENSORS_SMI130_ACC_ENABLE_MOTION
  if (status & 0x04) {
    if (atomic_read(&client_data->en_sig_motion) == 1) {
      PINFO("slope interrupt happened\n");

      input_report_rel(client_data->input, SLOPE_INTERRUPT,
                       SLOPE_INTERRUPT_HAPPENED);
      input_sync(client_data->input);
      smi130_acc_slope_interrupt_handle(client_data);
    }
    /* close sig sensor,
    it will be open again if APP wants */
    /*smi130_acc_set_en_sig_motion(client_data, 0);*/
  }
#endif
}

static irqreturn_t smi130_acc_irq_handler(int32_t irq, void *handle) {
  struct smi_client_data *data = handle;

  if (data == NULL) return IRQ_HANDLED;
  data->timestamp = smi130_acc_get_alarm_timestamp();

  schedule_work(&data->irq_work);

  return IRQ_HANDLED;
}
#endif /* defined(CONFIG_SENSORS_SMI130_ENABLE_INTERRUPT) */

static int32_t smi130_acc_input_init(struct smi_client_data *client_data) {
  struct input_dev *dev;
  int32_t err = 0;

  dev = input_allocate_device();
  if (NULL == dev) return -ENOMEM;

  dev->name = SMI130_ACC_NAME;

  input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_X);
  input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_Y);
  input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_Z);
  input_set_capability(dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME);

  input_set_capability(dev, EV_REL, SLOPE_INTERRUPT);
  input_set_drvdata(dev, client_data);

  err = input_register_device(dev);
  if (err < 0) {
    input_free_device(dev);
    dev_notice(client_data->dev, "smi130_acc input free!\n");
    return err;
  }
  client_data->input = dev;
  dev_notice(client_data->dev, "smi130_acc input register successfully, %s!\n",
             client_data->input->name);
  return err;
}

static void smi130_acc_input_destroy(struct smi_client_data *client_data) {
  struct input_dev *dev = client_data->input;

  input_unregister_device(dev);
}

static void smi130_acc_work_func(struct work_struct *work) {
  struct smi_client_data *client_data =
      container_of((struct delayed_work *)work, struct smi_client_data, work);
  unsigned long delay = msecs_to_jiffies(atomic_read(&client_data->delay));
  struct smi130_acc_data data;
  int32_t err;

  err = smi130_acc_read_xyz(&data);
  if (err < 0) return;
  /*report current frame via input event*/
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_X, data.x);
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_Y, data.y);
  input_event(client_data->input, INPUT_EVENT_TYPE, INPUT_EVENT_Z, data.z);
  input_sync(client_data->input);
  schedule_delayed_work(&client_data->work, delay);
}

static void smi_delay(uint32_t msec) {
  if (msec <= 20)
    usleep_range(msec * 1000, msec * 1000);
  else
    msleep(msec);
}

int32_t smi130_acc_probe(struct smi_client_data *client_data,
                         struct device *dev) {
  int32_t err = 0;
  PINFO("enter acc probe");

  err = smi130_acc_init(&client_data->device);
  if (client_data->device.chip_id == SMI130_ACC_CHIP_ID)
    PINFO("find Bosch smi130_acc");
  else
    goto exit_err_clean;

  dev_set_drvdata(dev, client_data);
  client_data->dev = dev;

  mutex_init(&client_data->mutex_enable);
  mutex_init(&client_data->mutex_op_mode);
  mutex_init(&client_data->mutex_value);
  /* input device init */
  err = smi130_acc_input_init(client_data);
  if (err < 0) goto exit_err_clean;
  /* sysfs node creation */
  err = sysfs_create_group(&client_data->input->dev.kobj,
                           &smi130_acc_attribute_group);
  if (err < 0) goto exit_err_sysfs;

  /* workqueue init */
  INIT_DELAYED_WORK(&client_data->work, smi130_acc_work_func);
  atomic_set(&client_data->delay, SMI130_ACC_DELAY_DEFAULT);
  atomic_set(&client_data->wkqueue_en, 0);

  /* h/w init */
  client_data->device.delay_msec = smi_delay;

  INIT_WORK(&client_data->report_data_work, smi130_acc_timer_work_fun);
  reportdata_wq = create_singlethread_workqueue("smi130_acc_wq");
  if (NULL == reportdata_wq) PERR("fail to create the reportdta_wq");
  hrtimer_init(&client_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  client_data->timer.function = reportdata_timer_fun;
  client_data->work_delay_kt = ns_to_ktime(4000000);
  client_data->is_timer_running = 0;
  client_data->timestamp = 0;
  client_data->time_odr = 4000000; /*default bandwidth 125HZ*/
  client_data->smi_acc_mode_enabled = 0;
  /*soft reset*/
  smi_delay(5);
  err = smi130_acc_soft_rst();
  /*200ms delay is recommended after soft reset*/
  smi_delay(200);
  if (err < 0) dev_err(client_data->dev, "Failed soft reset, er=%d", err);

  err += smi130_acc_set_range(SMI130_ACC_RANGE_2G);
  err += smi130_acc_set_bw(SMI130_ACC_BW_125HZ);
  if (err < 0) dev_err(client_data->dev, "Failed sensor config\n");

#if defined(CONFIG_SENSORS_SMI130_ENABLE_INTERRUPT)

  /* maps interrupt to int2 pin */
  err += smi130_acc_set_new_data(SMI130_ACC_INTR2_NEWDATA, INTR_ENABLE);
  /* enable data ready interrupt */
  err += smi130_acc_set_intr_enable(SMI130_ACC_DATA_ENABLE, INTR_ENABLE);
  /*Set interrupt behavior */
  err += smi130_acc_set_intr_output_type(SMI130_ACC_INTR2_OUTPUT, PUSH_PULL);
  /*Set interrupt active level*/
  err += smi130_acc_set_intr_level(SMI130_ACC_INTR2_LEVEL, ACTIVE_HIGH);
  /*set interrupt temporary, 250 ms*/
  err += smi130_acc_set_latch_intr(SMI130_ACC_LATCH_DURN_250MS);
  if (err < 0) PERR("config sensor data ready interrupt failed");
  client_data->gpio_pin =
      of_get_named_gpio_flags(dev->of_node, "smi130_acc,gpio_irq", 0, NULL);
  dev_info(client_data->dev, "smi130_acc int gpio number:%d\n",
           client_data->gpio_pin);
  err = gpio_request_one(client_data->gpio_pin, GPIOF_IN, "smi130_int");
  err += gpio_direction_input(client_data->gpio_pin);
  client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
  if (err < 0) {
    dev_err(client_data->dev, "can not request gpio to irq number\n");
    client_data->gpio_pin = 0;
  }
  if (client_data->gpio_pin != 0) {
    err = request_irq(client_data->IRQ, smi130_acc_irq_handler,
                      IRQF_TRIGGER_RISING, "smi130_acc", client_data);
    if (err < 0) dev_err(client_data->dev, "could not request irq\n");

    INIT_WORK(&client_data->irq_work, smi130_acc_irq_work_func);
  }
#endif

  err = smi130_acc_set_power_mode(SMI130_ACC_MODE_STANDBY);
  if (err < 0)
    dev_err(client_data->dev, "Failed sensor config\n");
  else
    PINFO("set smi130_acc to standby mode\n");

  client_data->selftest = 0;
  dev_notice(dev, "sensor %s probed successfully", SMI130_ACC_NAME);

  return 0;

exit_err_sysfs:
  if (err) smi130_acc_input_destroy(client_data);

exit_err_clean:
  if (err) {
    PINFO("enter probe");
    if (client_data != NULL) kfree(client_data);
  }
  return err;
}

/*!
 * @brief remove smi client
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
 */
int32_t smi130_acc_remove(struct device *dev) {
  int32_t err = 0;
  struct smi_client_data *client_data = dev_get_drvdata(dev);

  if (NULL != client_data) {
    smi_delay(1);
    sysfs_remove_group(&client_data->input->dev.kobj,
                       &smi130_acc_attribute_group);
    smi130_acc_input_destroy(client_data);
    kfree(client_data);
  }
  return err;
}

int32_t smi130_acc_suspend(struct device *dev) {
  int32_t err = 0;
  struct smi_client_data *client_data = dev_get_drvdata(dev);

  PINFO("suspend function entrance");
  err = enable_irq_wake(client_data->IRQ);
  if (err != 0) {
    PERR("enable_irq_wake failed %d\n", err);
    return err;
  }
  atomic_set(&client_data->in_suspend, 1);
  return err;
}

int32_t smi130_acc_resume(struct device *dev) {
  int32_t err = 0;
  struct smi_client_data *client_data = dev_get_drvdata(dev);

  PINFO("resume function entrance");
  err = disable_irq_wake(client_data->IRQ);
  if (err != 0) {
    PERR("enable_irq_wake failed %d\n", err);
    return err;
  }
  atomic_set(&client_data->in_suspend, 0);
  return err;
}
