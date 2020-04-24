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
/*! file <smi130_i2c_driver.c >
    brief <Linux Sensor interface i2c driver for SMI130> */

#include "smi130_acc_driver.h"
#include "smi130_gyro_driver.h"

#define SMI130_MAX_RETRY_I2C_XFER          (10)
#define SMI130_USE_BASIC_I2C_FUNC          (1)

/****************** Static Function Definitions *******************************/
static struct i2c_client *smi130_acc_i2c_client;
static struct i2c_client *smi130_gyro_i2c_client;

static int8_t smi130_i2c_read_byte(struct i2c_client *client,
                                   uint8_t reg_addr, uint8_t *data, uint8_t len);
static int8_t smi130_i2c_write_byte(struct i2c_client *client,
                                    uint8_t reg_addr, uint8_t *data, uint8_t len);
static int8_t smi130_i2c_read_block (uint8_t dev_addr,
                                     uint8_t reg_addr, uint8_t *data, uint8_t len);

static int8_t smi130_i2c_write_block (uint8_t dev_addr,
                                      uint8_t reg_addr, uint8_t *data, uint8_t len);
static int32_t smi130_acc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static void smi130_acc_i2c_shutdown(struct i2c_client *client);
static int32_t smi130_acc_i2c_remove(struct i2c_client *client);
static int32_t smi130_gyro_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static void smi130_gyro_i2c_shutdown(struct i2c_client *client);
static int32_t smi130_gyro_i2c_remove(struct i2c_client *client);

static int8_t smi130_i2c_read_byte(struct i2c_client *client,
                                   uint8_t reg_addr, uint8_t *data, uint8_t len)
{

	int32_t retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},

		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	for (retry = 0; retry < SMI130_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			smi130_acc_delay(1);
	}

	if (SMI130_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

static int8_t smi130_i2c_write_byte(struct i2c_client *client,
                                    uint8_t reg_addr, uint8_t *data, uint8_t len)
{

	uint8_t buffer[2];
	int32_t retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buffer,
		},
	};
	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < SMI130_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
			                 ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				smi130_acc_delay(1);
			}
		}
		if (SMI130_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
}

static int8_t smi130_i2c_read_block (uint8_t dev_addr,
                                     uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int8_t err = 0;


	if ((dev_addr==SMI130_ACC_I2C_ADDR1)||(dev_addr==SMI130_ACC_I2C_ADDR2)) {
		err = smi130_i2c_read_byte(smi130_acc_i2c_client,reg_addr,data,len);
	}

	else {
		err = smi130_i2c_read_byte(smi130_gyro_i2c_client,reg_addr,data,len);
	}

	return err;

}
static int8_t smi130_i2c_write_block (uint8_t dev_addr,
                                      uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int8_t err = 0;

	if ((dev_addr==SMI130_ACC_I2C_ADDR1)||(dev_addr==SMI130_ACC_I2C_ADDR2)) {
		err = smi130_i2c_write_byte(smi130_acc_i2c_client,reg_addr,data,len);
	} else {
		err = smi130_i2c_write_byte(smi130_gyro_i2c_client,reg_addr,data,len);
	}

	return err;
}

/* ACC driver */
static int32_t smi130_acc_i2c_probe(struct i2c_client *client,
                                    const struct i2c_device_id *id)
{
	int32_t err = 0;

	struct smi_client_data *client_data = NULL;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		return err;
	}
	client_data = kzalloc(sizeof(struct smi_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		dev_err(&client->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	smi130_acc_i2c_client = client;
	client_data->device.dev_addr = smi130_acc_i2c_client->addr;
	client_data->device.bus_read  =  smi130_i2c_read_block;
	client_data->device.bus_write =  smi130_i2c_write_block;

	i2c_set_clientdata(client, client_data);

	return smi130_acc_probe(client_data, &client->dev);
exit_err_clean:
	if (err)
		smi130_acc_i2c_client = NULL;
	return err;
}



/*!
 * @brief shutdown smi130_acc device in i2c driver
 *
 * @param client the pointer of i2c client
 *
 * @return no return value
*/
static void smi130_acc_i2c_shutdown(struct i2c_client *client)
{
	smi130_acc_suspend(&client->dev);
}

/*!
 * @brief remove smi130_acc i2c client
 *
 * @param client the pointer of i2c client
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_acc_i2c_remove(struct i2c_client *client)
{
	int32_t err = 0;
	err = smi130_acc_remove(&client->dev);
	smi130_acc_i2c_client = NULL;

	return err;
}

#ifdef CONFIG_PM
/*!
 * @brief suspend smi130_acc device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_acc_i2c_suspend(struct device *dev)
{
	int32_t err = 0;
	err = smi130_acc_suspend(dev);
	return err;
}

/*!
 * @brief resume smi130_acc device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_acc_i2c_resume(struct device *dev)
{
	int32_t err = 0;
	/* post resume operation */
	err = smi130_acc_resume(dev);

	return err;
}

/*!
 * @brief register i2c device power manager hooks
*/
static const struct dev_pm_ops smi130_acc_i2c_pm_ops = {
	/**< device suspend */
	.suspend = smi130_acc_i2c_suspend,
	/**< device resume */
	.resume  = smi130_acc_i2c_resume
};
#define smi130_acc_pm_ops	(&smi130_acc_i2c_pm_ops)
#else /* CONFIG_PM */
#define smi130_acc_pm_ops	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id smi130_acc_ids[] = {
	{ SMI130_ACC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smi130_acc_ids);

#ifdef CONFIG_OF
static const struct of_device_id smi130_acc_id_table[] = {
	{ .compatible = "bosch,smi130_acc", },
	{ },
};
MODULE_DEVICE_TABLE(of, smi130_acc_id_table);
#endif
/*!
 * @brief register i2c driver hooks
*/
static struct i2c_driver smi130_acc_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SMI130_ACC_NAME,
#ifdef CONFIG_PM
		.pm = smi130_acc_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = smi130_acc_id_table,
#endif
	},
	.id_table = smi130_acc_ids,
	.probe = smi130_acc_i2c_probe,
	.shutdown = smi130_acc_i2c_shutdown,
	.remove = smi130_acc_i2c_remove,
};


/*!
 * @brief smi130_gyro probe function via i2c bus
 *
 * @param client the pointer of i2c client
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t smi130_gyro_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t err = 0;
	struct smi_gyro_client_data *gyro_client_data = NULL;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		return err;
	}
	gyro_client_data = kzalloc(sizeof(struct smi_gyro_client_data), GFP_KERNEL);
	if (NULL == gyro_client_data) {
		dev_err(&client->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	smi130_gyro_i2c_client = client;
	gyro_client_data->device.dev_addr = smi130_gyro_i2c_client->addr;
	gyro_client_data->device.bus_read  =  smi130_i2c_read_block;
	gyro_client_data->device.bus_write =  smi130_i2c_write_block;

	i2c_set_clientdata(client, gyro_client_data);

	return smi_gyro_probe(gyro_client_data, &client->dev);
exit_err_clean:
	if (err)
		smi130_gyro_i2c_client = NULL;
	return err;
}


/*!
 * @brief shutdown smi130_acc device in i2c driver
 *
 * @param client the pointer of i2c client
 *
 * @return no return value
*/
static void smi130_gyro_i2c_shutdown(struct i2c_client *client)
{
	smi_gyro_shutdown(&client->dev);
}

/*!
 * @brief remove smi130_acc i2c client
 *
 * @param client the pointer of i2c client
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_gyro_i2c_remove(struct i2c_client *client)
{
	int32_t err = 0;
	err = smi_gyro_remove(&client->dev);
	smi130_gyro_i2c_client = NULL;

	return err;
}

#ifdef CONFIG_PM
/*!
 * @brief suspend smi130_acc device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_gyro_i2c_suspend(struct device *dev)
{
	int32_t err = 0;
	smi_gyro_shutdown(dev);
	return err;
}

/*!
 * @brief resume smi130_acc device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_gyro_i2c_resume(struct device *dev)
{
	int32_t err = 0;
	/* post resume operation */
	err = smi_gyro_resume(dev);

	return err;
}

/*!
 * @brief register i2c device power manager hooks
*/
static const struct dev_pm_ops smi130_i2c_pm_ops = {
	/**< device suspend */
	.suspend = smi130_gyro_i2c_suspend,
	/**< device resume */
	.resume  = smi130_gyro_i2c_resume
};
#define smi130_i2c_pm_ops	(&smi130_i2c_pm_ops)
#else /* CONFIG_PM */
#define smi130_i2c_pm_ops	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id smi130_gyro_ids[] = {
	{ SMI130_GYRO_NAME },
};
MODULE_DEVICE_TABLE(i2c, smi130_gyro_ids);

#ifdef CONFIG_OF
static const struct of_device_id smi130_id_table[] = {
	{ .compatible = "bosch,smi130_gyro", },
	{ },
};
MODULE_DEVICE_TABLE(of, smi130_id_table);
#endif
/*!
 * @brief register i2c driver hooks
*/
static struct i2c_driver smi130_gyro_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "smi130_gyro",
#ifdef CONFIG_PM
		.pm = smi130_i2c_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = smi130_id_table,
#endif
	},
	.id_table = smi130_gyro_ids,
	.probe = smi130_gyro_i2c_probe,
	.shutdown = smi130_gyro_i2c_shutdown,
	.remove = smi130_gyro_i2c_remove,
};

/*!
 * @brief initialize smi130 i2c module
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t __init smi130_i2c_init(void)
{
	i2c_add_driver(&smi130_acc_i2c_driver);
	return i2c_add_driver(&smi130_gyro_i2c_driver);
}

/*!
 * @brief remove smi130 i2c module
 *
 * @return no return value
*/
static void __exit smi130_i2c_exit(void)
{
	i2c_del_driver(&smi130_acc_i2c_driver);
	i2c_del_driver(&smi130_gyro_i2c_driver);
}

MODULE_DESCRIPTION("SMI130 I2C driver");
MODULE_LICENSE("Dual BSD/GPL");

module_init(smi130_i2c_init);
module_exit(smi130_i2c_exit);




