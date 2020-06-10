# SMI130-Sensor-API and Sensor Driver

## Table of Contents

 - [General Description](#Description)
 - [License](#License)
 - [System Configuration](#Configuration)
 - [Linux driver files](#files)
 - [Linux driver principle](#principle)
 - [The sysfs interface attributes](#interface)
 - [Input Event](#Event)
 - [Interrupt support](#Interrupt)
 - [Other configurations](#Other)
   - [Device Tree configuration](#DeviceTree)
   - [Kconfig configuration](#Kconfig)
   - [Makefile configuration](#Makefile)

## General Description <a name=Description></a>

This document introduces the usage of SMI130 Linux driver

## License <a name=License></a>
See [LICENSE](LICENSE.md) file

## System Configuration <a name=Configuration></a>

Table 1: System Configuration

System Configuration|Settings|
:--|:--|
Linux Kernel configuration|CONFIG_SYSFS=y
I2C interface configuration|CONFIG_I2C=y
SPI interface configuration|CONFIG_SPI_MASTER=y

## Linux driver files <a name=files></a>

Table 2: Driver Description

Files name|Description|
:--|:--|
bs_log.c<br>bs_log.h|This file implements Bosch sensor log driver
Kconfig|Kconfig
Makefile|Makefile
LICENSE|License
README.md|Brief info of SMI130 Linux driver
smi130_i2c_driver.c<br>smi130_spi_driver.c|This file implements i2c/spi interface function of SMI130
smi130_acc_driver.c<br>smi130_acc_driver.h|This file implements the Linux input level driver of SMI130 ACC
smi130_acc.c<br>smi130_acc.h|This file implements the low level API of SMI130 ACC
smi130_gyro_driver.c<br>smi130_gyro_driver.h|This file implements the Linux input level driver of SMI130 GYRO
smi130_gyro.c<br>smi130_gyro.h|This file implements the low level API of SMI130 GYRO
smi130_defs.h|This file implements the common definition of SMI130 
## Linux driver principle <a name=principle></a>

By sysfs interface, application enables sensor to launch a delay task on work queue. The task delay value can be configured by sysfs interface. The default delay value is 200ms. The delay task will get the sensor values and update them as input event on input device.
If compiled with interrupt support, the interrupt service routine will launch an IRQ task on work queue.The IRQ task will identify the interrupt type and send the interrupt event on input device. Application will block read the input device to get the input events. The input event will carry the sensor values or the interrupt type data.

```
              User space
----------------------------------------------
             |          |
           sysfs       dev
             \          /
           input-subsystem
                  |
i2c/spi bus <-- smi130_driver --> sensor_API
                  |
----------------------------------------------
               Hardware

```

## The sysfs interface attributes <a name=interface></a>



SMI130_ACC driver is registered as input class.
In /sys/devices/virtual/input/inputX(X indicates the sensor number) on sysfs, the registered name is “smi130_acc”.

Table 3: sysfs interface attributes

| Name | Descriptions | RW | Usage |
| :------------ | :------------ | :------------ | :------------ |
| Range | Set and get sensor G-range  |RW   |  Write: <br>3------ +/- 2G<br>5 ------ +/- 4G<br>8 ------ +/- 8G<br>12 ------ +/- 16G<br>Read: <br>3------ +/- 2G<br>5 ------ +/- 4G<br>8 ------ +/- 8G<br>12 ------ +/- 16G<br>Default : 3|
|bandwidth   | Set and get sensor bandwidth  |RW   |Write:<br>8 ------ 7.81 Hz<br>9 ------ 15.63 H<br>10 ------ 31.25 Hz<br>11 ------ 62.5 Hz<br>12 ------ 125 Hz<br>13 ------ 250 Hz<br>14 ------ 500 Hz<br>15 ------ 1000 Hz<br>Read:<br>8 ------ 7.81 Hz<br>9 ------ 15.63 Hz<br>10 ------ 31.25 Hz<br>11 ------ 62.5 Hz<br>12 ------ 125 Hz<br>13 ------ 250 Hz<br>14 ------ 500 Hz<br>15 ------ 1000 Hz<br>Default : 12|
| op_mode   |Set and get sensor mode    | RW  | Write :<br>0 ------ normal Mode<br>3 ------ deep suspend<br>4 ------ low power 2<br>5 ------ standby<br> Read:<br>0 ------ normal Mode<br>3 ------ deep suspend<br>4 ------ low power 2<br>5 ------ standby Mode<br>Default : 5|
| Value  |Get sensor x, y, z axis valus   | R  |  Read return x, y, z axis values |
| Delay  |Set and get delay time for delay task |RW   |  value Range (0- 200) ms<br>Default : 200  |
|Enable   |Enable or disable sensor work task |RW   |Write :<br>0 ------ disable sensor<br>1 ------ enable sensor<br>Read :<br>0 ------ disable sensor<br>1 ------ enable sensor<br>Default : 0 |
|sleepdur   |Set and get sleep phase<br>duration in Low Power mode |RW   |Write :<br>5 ------ 0.5ms<br>6 ------ 1ms<br>7 ------ 2ms<br>8 ------ 4ms<br>9 ------ 6ms<br>10 ------ 10ms<br>11 ------ 25ms<br>12 ------ 50ms<br>13 ------ 100ms<br>14 ------ 500ms<br>15 ------ 1s<br>Read :<br>5 ------ 0.5ms<br>6 ------ 1ms<br>7 ------ 2ms<br>8 ------ 4ms<br>9 ------ 6ms<br>10 ------ 10ms<br>11 ------ 25ms<br>12 ------ 50ms<br>13 ------ 100ms<br>14 ------ 500ms<br>15 ------ 1s<br>Default : 0<br> |
|reg   |Set value to specific<br>register and get whole<br>register value|RW   | Write :<br>para1 para2<br>Para1: register address<br>Para2: value to be set<br>Read:<br>whole register and value|
|fast_calibration_x   |Set and get fast calibration<br>configure value for x axis|RW   |Write :<br>0 ------ 0 g<br>1 ------ 1 g<br>2 ------ -1 g<br>3 ------ 0 g<br>Read :<br>0 ------ 0 g<br>1 ------ 1 g<br>2 ------ -1 g<br>3 ------ 0 g<br>Default : 0|
|fast_calibration_y   |Set and get fast calibration<br>configure value for y axis|RW   |Write :<br>0 ------ 0 g<br>1 ------ 1 g<br>2 ------ -1 g<br>3 ------ 0 g<br>Read :<br>0 ------ 0 g<br>1 ------ 1 g<br>2 ------ -1 g<br>3 ------ 0 g<br>Default : 0|
|fast_calibration_z   |Set and get fast calibration<br>configure value for z axis|RW   |Write :<br>0 ------ 0 g<br>1 ------ 1 g<br>2 ------ -1 g<br>3 ------ 0 g<br>Read :<br>0 ------ 0 g<br>1 ------ 1 g<br>2 ------ -1 g<br>3 ------ 0 g<br>Default : 0|
|Chip_id   |Get sensor chip id   |R   |0xFA   |
|offset_x   |Set and get offset x register|RW   |value Range (0- 255)   |
|offset_y   |Set and get offset y register|RW   |value Range (0- 255)   |
|offset_z   |Set and get offset z register|RW   |value Range (0- 255)   |
|enable_int   |Enable or disable sensor interrupt|W   |Write : para1 para2<br>Para1:<br>4 ------ Data Ready Interrupt<br>5 ------ Slope X Interrupt<br>6 ------ Slope Y Interrupt<br>7 ------ Slope Z Interrupt<br>Para2:<br>0 ------ disable interrupt<br>1 ------ enable interrupt |
|int_mode   |Set and get interrupt latched time|RW   |Write :<br>0 ------ non-latched<br>1 ------ temporary, 250 ms<br>2 ------ temporary, 500 ms<br>3 ------ temporary, 1 s<br>4 ------ temporary, 2 s<br>5 ------ temporary, 4 s<br>6 ------ temporary, 8 s<br>7 ------ latched<br>8 ------ non-latched<br>9 ------ temporary, 500 us<br>10 ------ temporary, 500 us<br>11 ------ temporary, 1 ms<br>12 ------ temporary, 12.5 ms<br>13 ------ temporary, 25 ms<br>14 ------ temporary, 50 ms<br>15 ------ latched<br>Read :<br>0 ------ non-latched<br>1 ------ temporary, 250 ms<br>2 ------ temporary, 500 ms<br>3 ------ temporary, 1 s<br>4 ------ temporary, 2 s<br>5 ------ temporary, 4 s<br>6 ------ temporary, 8 s<br>7 ------ latched<br>8 ------ non-latched<br>9 ------ temporary, 250 us<br>10 ------ temporary, 500 us<br>11 ------ temporary, 1 ms<br>12 ------ temporary, 12.5 ms<br>13 ------ temporary, 25 ms<br>14 ------ temporary, 50 ms<br>15 ------ latched<br>Default : 1<br>|
|slope_duration   |Set and get slope interrupt duration|RW   |value Range (0- 3)<br>Default : 0|
|slope_threshold   |Set and get slope interrupt threshold|RW   |value Range (0- 255)<br>Default : 0|
|en_sig_motion   |enable and disable slope function |RW   |Write :<br>0 ------ disable <br>1 ------ enable <br>Read :<br>0 ------ disable <br>1 ------ enable <br>Default : 0 |
|selftest   |Set selftest and get result|RW   |Write:<br>1 ------ start self-test<br>Read:<br>0 ------ success<br>1 ------ x axis failed<br>2 ------ y axis failed<br>3 ------ x and y axis failed<br>4 ------ z axis failed<br>5 ------ x and z axis failed<br>6 ------ y and z axis failed<br>7 ------ x, y and z axis failed<br>|
|softreset   |set soft reset   |W   |Write:<br>1 ------ start soft reset |
|temperature   |Read temperature in LSB   |R   |value Range (-127- 127)   |

SMI130_GYRO driver is registered as input class.
In /sys/devices/virtual/input/inputX(X indicates the sensor number) on sysfs, the registered name is “smi130_gyro”.

The gyro sysfs interfaces are shown below.

| Name | Descriptions | RW | Usage |
| :------------ | :------------ | :------------ | :------------ |
|chip_id   |Get sensor chip id   |R   |0x0F   |
|name   |Get sensor name   |R   |smi130_gyro   |
| range | Set and get sensor range | RW | Write:<br> 0 ------ +/- 2000 º/s<br>1 ------ +/- 1000 º/s<br>2 ------ +/- 500 º/s<br>3 ------ +/- 250 º/s<br>4 ------ +/- 125 º/s<br>Read:<br> 0 ------ +/- 2000 º/s<br>1 ------ +/- 1000 º/s<br>2 ------ +/- 500 º/s<br>3 ------ +/- 250 º/s<br>4 ------ +/- 125 º/s<br>Default : 0|
|bandwidth| set and get sensor bandwidth|RW |Write:<br> 0 ------ 523 Hz<br>1 ------ 230 Hz<br>2 ------ 116 Hz<br>3 ------ 47 Hz<br>4 ------ 23 Hz<br>5 ------ 12 Hz<br>6 ------ 64 Hz<br>7 ------ 32 Hz<br>Read:<br> 0 ------ 523 Hz<br>1 ------ 230 Hz<br>2 ------ 116 Hz<br>3 ------ 47 Hz<br>4 ------ 23 Hz<br>5 ------ 12 Hz<br>6 ------ 64 Hz<br>7 ------ 32 Hz<br>Default : 7 |
|op_mode|Set and get sensor mode|RW|Read :<br> 0 ------ Normal Mode<br>1 ------ Deep suspend Mode<br>Write :<br> 0 ------ normal Mode<br>1 ------ Deep suspend Mode<br>Default : 0|
|value|Get sensor x, y, z axis values|R|Read return LSB values of x, y, z axis|
|reg   |Set value to specific<br>register and get whole<br>register value|RW   | Write :<br>para1 para2<br>Para1: register address<br>Para2: value to be set<br>Read:<br>whole register and value|
|selftest|Perform a selftest for sensor|R|Read :<br> 0 ------ selftest passed<br>1 ------ selftest failed|
|delay|Set and get delay time for delay task|RW|value Range (0- 200) ms|
|enable|Enable or disable sensor work task|RW|Write :<br> 0 ------ disable sensor<br>1 ------ enable sensor<br>Read :<br> 0 ------ disable sensor<br>1 ------ enable sensor<br>Default : 0|

## Input Event <a name=Event></a>
Application should read input event for the sensor value and interrupt report.
The input event descriptions are shown below.
Table 4: Input event

| Type  |Code   |Description    |Value   |
| :------------ | :------------ | :------------ | :------------ |
|EV_MSC   |MSC_GESTURE   |Event for x axis   |LSB Value of X axis   |
|EV_MSC   |MSC_RAW   |Event for x axis   |LSB Value of X axis   |
|EV_MSC   |MSC_SCAN   |Event for x axis   |LSB Value of X axis   |
|EV_MSC      |MSC_TIMESTAMP   |Event for Time Stamp   |Timestamp value   |
|EV_REL|REL_DIAL|slope/any motion interrupt|Interrupt notification 7-12|

## Interrupt support <a name=Interrupt></a>

Set SENSORS_SMI130_ENABLE_INTERRUPT to 'yes' in Kconfig file is prerequisite before using other interrupt features

### SMI130 ACC interrupt

#### New data interrupt support

Set SENSORS_SMI130_ACC_ENABLE_NEWDATA_INT to ‘yes’ in Kconfig file

#### MOTION detection function support

Set SENSORS_SMI130_ACC_ENABLE_MOTION to ‘yes’ in Kconfig file

### SMI130 GYRO interrupt

#### New data interrupt support

Set SENSORS_SMI130_GYRO_ENABLE_NEWDATA_INT to ‘yes’ in Kconfig file

## Other configurations <a name=Other></a>

###  Device Tree configuration <a name=DeviceTree></a>

To enable driver probing, add the smi130_acc and smi130_gyro node to the platform device tree as described below.

**Required properties:**

*- compatible*: "bosch,smi130_acc"

*- compatible*: "bosch,smi130_gyro"

*- reg*: the I2C address or SPI chip select the device will respond to

*- interrupt-parent*: phandle to the parent interrupt controller as documented in interrupts

*- interrupts*: interrupt mapping for IRQ as documented in interrupts

I2C example (Qualcomm DragonBoard 410c):
>			status = "okay";
>			smi130_acc@19{
>					compatible = "smi130_acc";
>					reg = <0x19>;
>					interrupt-parent = <&msmgpio>;
>					interrupts = <115 0>; 
>					smi130_acc,gpio_irq = <&msmgpio 115 0>;
>					};
>			smi130_gyro@69{
>					compatible = "smi130_gyro";
>					reg = <0x69>;
>					interrupt-parent = <&msmgpio>;
>					interrupts = <13 0>; 
>					smi130_gyro,gpio_irq = <&msmgpio 13 0>;
>					};	

SPI example (Qualcomm DragonBoard 410c):

>			status = "okay";
>			cs-gpios = <&msmgpio 18 0>,<&msmgpio 110 0>;
>			spi-cpol = <0>;
>			spi-cpha = <0>;
>			smi130_acc@0{
>			 			spi-max-frequency = <500000>;					
>						compatible = "smi130_acc";
>						reg = <0>;
>						interrupt-parent = <&msmgpio>;
>						interrupts = <115 0>; 
>						smi130_acc,gpio_irq = <&msmgpio 115 0>;			
>			    };
>			smi130_gyro@1{
>						spi-max-frequency = <500000>;					
>						compatible = "smi130_gyro";
>						reg = <1>;
>						interrupt-parent = <&msmgpio>;
>						interrupts = <13 0>; 
>						smi130_gyro,gpio_irq = <&msmgpio 13 0>;
>				};
>

### Kconfig configuration <a name=Kconfig></a>

Configure kernel with *make menuconfig*<br>
>		Device Drivers  --->
>			Input device support  --->
>			[*] Enable Bosch SMI130 IMU sensor  ---> 
>				Select communication interface (Enable I2C connection)  --->
>				  ( ) Enable SPI connection
>				  (X) Enable I2C connection
>				<*>  Bosch Sensor driver smart log function support  
>				<*>  SMI130 sensor support
>				  <*>  SMI130_ACC acceleration sensor interrupt INT2 support
>					  <*> enable acc data ready interrupt
>					  <*> support sensor slope/any motion function
>					  <*> enable gyro data ready interrupt 


###  Makefile configuration <a name=Makefile></a>

* Copy driver source code into the target directory (e.g. *drivers/input/sensors/smi130*)
* Edit related Kconfig to include *smi130* support:

>         source "drivers/input/sensors/smi130/Kconfig"

* Edit related Makefile adding the following line:

>         obj-$(CONFIG_INPUT_SMI130)	+= sensors/smi130/
