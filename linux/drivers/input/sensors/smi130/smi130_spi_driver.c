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
    brief <Linux Sensor interface spi driver for SMI130> */

#include "smi130_acc_driver.h"
#include "smi130_gyro_driver.h"



/*! @defgroup smi130_acc_spi_src
 *  @brief smi130_acc spi driver module
 @{*/
/*! the maximum of transfer buffer size */
#define SMI130_ACC_MAX_BUFFER_SIZE      32
#define SMI130_MAX_BUFFER_SIZE          32
#define SENSORS_SPI_READ	            0x80


/****************** Static Function Definitions *******************************/
static struct spi_device *smi130_acc_spi_client;
static struct spi_device *smi130_gyro_spi_client;


static int8_t smi130_spi_read_block (uint8_t dev_addr,
                                     uint8_t reg_addr, uint8_t *data, uint8_t len);
static int8_t smi130_spi_write_block (uint8_t dev_addr,
                                      uint8_t reg_addr, uint8_t *data, uint8_t len);
static int32_t smi130_acc_spi_probe(struct spi_device *spi);
static void smi130_acc_spi_shutdown(struct spi_device *client);
static int32_t smi130_acc_spi_remove(struct spi_device *client);

#ifdef CONFIG_PM
static int32_t smi130_acc_spi_suspend(struct device *dev);

static int32_t smi130_acc_spi_resume(struct device *dev);
#endif

static int32_t smi130_gyro_spi_probe(struct spi_device *spi);
static void smi130_gyro_spi_shutdown(struct spi_device *client);
static int32_t smi130_gyro_spi_remove(struct spi_device *client);

#ifdef CONFIG_PM
static int32_t smi130_gyro_spi_suspend(struct device *dev);

static int32_t smi130_gyro_spi_resume(struct device *dev);
#endif

static int8_t smi130_spi_write_block (uint8_t dev_addr,
                                      uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	uint8_t buffer[SMI130_MAX_BUFFER_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf = buffer,
		.len = len + 1,
	};
	struct spi_message msg;

	if (len > SMI130_MAX_BUFFER_SIZE)
		return -EINVAL;

	buffer[0] = reg_addr & 0x7F;/* write: MSB = 0 */
	memcpy(&buffer[1], data, len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	if (dev_addr == 0)
		return spi_sync(smi130_acc_spi_client, &msg);
	else
		return spi_sync(smi130_gyro_spi_client, &msg);
}

static int8_t smi130_spi_read_block (uint8_t dev_addr,
                                     uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	u8 reg = reg_addr | SENSORS_SPI_READ;/* read: MSB = 1 */
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		}
	};
	struct spi_message msg;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);
	if (dev_addr == 0)
		return spi_sync(smi130_acc_spi_client, &msg);
	else
		return spi_sync(smi130_gyro_spi_client, &msg);
}



/*!
 * @brief smi130_acc probe function via spi bus
 *
 * @param client the pointer of spi client
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t smi130_acc_spi_probe(struct spi_device *client)
{

	int32_t err = 0;
	struct smi_client_data *client_data = NULL;

	if (NULL == smi130_acc_spi_client)
		smi130_acc_spi_client = client;
	else {
		dev_err(&client->dev, "This driver does not support multiple clients!\n");
		return -EBUSY;
	}

	client->bits_per_word = 8;
	err = spi_setup(client);
	if (err < 0) {
		dev_err(&client->dev, "spi_setup failed!\n");
		return err;
	}
	client_data = kzalloc(sizeof(struct smi_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		dev_err(&client->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	client_data->device.dev_addr = smi130_acc_spi_client->chip_select;
	client_data->device.bus_read = smi130_spi_read_block;
	client_data->device.bus_write = smi130_spi_write_block;

	return smi130_acc_probe(client_data, &client->dev);
exit_err_clean:
	if (err)
		smi130_acc_spi_client = NULL;
	return err;
}

/*!
 * @brief shutdown smi130_acc device in spi driver
 *
 * @param client the pointer of spi client
 *
 * @return no return value
*/
static void smi130_acc_spi_shutdown(struct spi_device *client)
{
	smi130_acc_suspend(&client->dev);
}

/*!
 * @brief remove smi130_acc spi client
 *
 * @param client the pointer of spi client
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_acc_spi_remove(struct spi_device *client)
{
	int32_t err = 0;
	err = smi130_acc_remove(&client->dev);
	smi130_acc_spi_client = NULL;

	return err;
}

#ifdef CONFIG_PM
/*!
 * @brief suspend smi130_acc device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_acc_spi_suspend(struct device *dev)
{
	int32_t err = 0;
	err = smi130_acc_suspend(dev);
	return err;
}

/*!
 * @brief resume smi130_acc device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_acc_spi_resume(struct device *dev)
{
	int32_t err = 0;
	/* post resume operation */
	err = smi130_acc_resume(dev);

	return err;
}

/*!
 * @brief register spi device power manager hooks
*/
static const struct dev_pm_ops smi130_acc_spi_pm_ops = {
	/**< device suspend */
	.suspend = smi130_acc_spi_suspend,
	/**< device resume */
	.resume  = smi130_acc_spi_resume
};
#define smi130_acc_pm_ops	(&smi130_acc_spi_pm_ops)
#else /* CONFIG_PM */
#define smi130_acc_pm_ops	NULL
#endif /* CONFIG_PM */

static const struct spi_device_id smi130_acc_ids[] = {
	{ SMI130_ACC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, smi130_acc_ids);

#ifdef CONFIG_OF
static const struct of_device_id smi130_acc_id_table[] = {
	{ .compatible = "bosch,smi130_acc", },
	{ },
};
MODULE_DEVICE_TABLE(of, smi130_acc_id_table);
#endif
/*!
 * @brief register spi driver hooks
*/
static struct spi_driver smi130_acc_spi_driver = {
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
	.probe = smi130_acc_spi_probe,
	.shutdown = smi130_acc_spi_shutdown,
	.remove = smi130_acc_spi_remove,
};


/*!
 * @brief smi130_gyro probe function via spi bus
 *
 * @param client the pointer of spi client
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t smi130_gyro_spi_probe(struct spi_device *client)
{
	int32_t err = 0;
	struct smi_gyro_client_data *gyro_client_data = NULL;

	if (NULL == smi130_gyro_spi_client)
		smi130_gyro_spi_client = client;
	else {
		dev_err(&client->dev, "This driver does not support multiple clients!\n");
		return -EBUSY;
	}
	client->bits_per_word = 8;
	err = spi_setup(client);
	if (err < 0) {
		dev_err(&client->dev, "spi_setup failed!\n");
		return err;
	}
	gyro_client_data = kzalloc(sizeof(struct smi_gyro_client_data), GFP_KERNEL);
	if (NULL == gyro_client_data) {
		dev_err(&client->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	gyro_client_data->device.dev_addr = smi130_gyro_spi_client->chip_select;
	gyro_client_data->device.bus_read = smi130_spi_read_block;
	gyro_client_data->device.bus_write = smi130_spi_write_block;

	return smi_gyro_probe(gyro_client_data, &client->dev);

exit_err_clean:
	if (err)
		smi130_gyro_spi_client = NULL;
	return err;
}


/*!
 * @brief shutdown smi130_acc device in spi driver
 *
 * @param client the pointer of spi client
 *
 * @return no return value
*/
static void smi130_gyro_spi_shutdown(struct spi_device *client)
{
	smi_gyro_shutdown(&client->dev);
}

/*!
 * @brief remove smi130_acc spi client
 *
 * @param client the pointer of spi client
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_gyro_spi_remove(struct spi_device *client)
{
	int32_t err = 0;
	err = smi_gyro_remove(&client->dev);
	smi130_gyro_spi_client = NULL;

	return err;
}

#ifdef CONFIG_PM
/*!
 * @brief suspend smi130_acc device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_gyro_spi_suspend(struct device *dev)
{
	int32_t err = 0;
	smi_gyro_shutdown(dev);
	return err;
}

/*!
 * @brief resume smi130_acc device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t smi130_gyro_spi_resume(struct device *dev)
{
	int32_t err = 0;
	/* post resume operation */
	err = smi_gyro_resume(dev);

	return err;
}

/*!
 * @brief register spi device power manager hooks
*/
static const struct dev_pm_ops smi130_spi_pm_ops = {
	/**< device suspend */
	.suspend = smi130_gyro_spi_suspend,
	/**< device resume */
	.resume  = smi130_gyro_spi_resume
};
#define smi130_spi_pm_ops	(&smi130_spi_pm_ops)
#else /* CONFIG_PM */
#define smi130_spi_pm_ops	NULL
#endif /* CONFIG_PM */

static const struct spi_device_id smi130_gyro_ids[] = {
	{ SMI130_GYRO_NAME },
};
MODULE_DEVICE_TABLE(spi, smi130_gyro_ids);

#ifdef CONFIG_OF
static const struct of_device_id smi130_id_table[] = {
	{ .compatible = "bosch,smi130_gyro", },
	{ },
};
MODULE_DEVICE_TABLE(of, smi130_id_table);
#endif
/*!
 * @brief register spi driver hooks
*/
static struct spi_driver smi130_gyro_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "smi130_gyro",
#ifdef CONFIG_PM
		.pm = smi130_spi_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = smi130_id_table,
#endif
	},
	.id_table = smi130_gyro_ids,
	.probe = smi130_gyro_spi_probe,
	.shutdown = smi130_gyro_spi_shutdown,
	.remove = smi130_gyro_spi_remove,
};

/*!
 * @brief initialize smi130 spi module
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t __init smi130_spi_init(void)
{
	spi_register_driver(&smi130_acc_spi_driver);
	return spi_register_driver(&smi130_gyro_spi_driver);
}

/*!
 * @brief remove smi130 spi module
 *
 * @return no return value
*/
static void __exit smi130_spi_exit(void)
{
	spi_unregister_driver(&smi130_acc_spi_driver);
	spi_unregister_driver(&smi130_gyro_spi_driver);
}

MODULE_DESCRIPTION("SMI130 SPI driver");
MODULE_LICENSE("Dual BSD/GPL");

module_init(smi130_spi_init);
module_exit(smi130_spi_exit);


