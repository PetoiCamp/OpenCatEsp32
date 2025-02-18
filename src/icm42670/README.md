# Overview

The eMD driver is TDK Invensense's reference code to drive our IMU from a microcontroller-based system. It is coded in C language and organized around modules. 

## Common files

All modules rely on the following files. 

Files:
* `imu/inv_imu_transport.h`: Definition of the abstraction layer used to communicate with the IMU.
* `imu/inv_imu_transport.c`: Implementation of the abstraction layer used to communicate with the IMU.
* `imu/inv_imu.h`: Describes IMU specificities and capabilities.
* `imu/inv_imu_regmap_rev_a.h`: Exposes register map using positions and masks.
* `imu/inv_imu_defs.h`: Defines possible values for most used registers.
* `imu/inv_imu_extfunc.h`: Defines system functions required by the driver, which shall be implemented in the application layer.

## Driver

The **driver** contains the generic functionalities required to operate our IMU.

Depends on:
* **Common files**

Files:
* `imu/inv_imu_driver.h`: Definition of the driver API.
* `imu/inv_imu_driver.c`: Implementation of the driver API.
* `imu/inv_imu_version.h`: Contains the driver's version as a string. 

## Self-test

The **self-test** module contains the code required to operate IMU's self-test.

Depends on:
* **Driver**
* **Common files**

Files:
* `imu/inv_imu_selftest.h`: Definition of the self-test module API.
* `imu/inv_imu_selftest.c`: Implementation of the self-test module API.

## APEX

The **APEX** module provides API to operate algorithm features (APEX) on-chip. The following features are available:
* Pedometer (step count and step detector)
* FreeFall detection, including Low-G and High-G detection.
* Tilt detection
* Significant Motion Detection

Depends on:
* **Driver**
* **Common files**

Files:
* `imu/inv_imu_apex.h`: Definition of the APEX module API.
* `imu/inv_imu_apex.c`: Implementation of the APEX module API.

# Initializing driver

Please, follow these steps:
* On your application code, create a local variable of type `inv_imu_device_t` and another one of type `inv_imu_serif_t`: `inv_imu_device_t imu_dev;` and `inv_imu_serif_t imu_serif;`
* Initialize serif structure:
  * Provide an implementation of the `read_reg` and `write_reg` functions and initialize the corresponding pointers.
  * Configure the `max_read` and `max_write` fields which indicates the maximum numbers of bytes allowed per transaction.
  * Configure the `serif_type` field which indicates the serial interface used. Available options are listed in `inv_imu_transport.h`.
  * The `context` field can be set to 0.
* Initialize sensor event callback pointer in `inv_imu_adv_var_t` structure. This function will be called when a new sensor data will be available.
* Call the `inv_imu_adv_init` function, providing the device and serif objects as well as a callback which will be executed when a new sample is received.

Example of initialization in C:
```C
inv_imu_device_t imu_dev;
inv_imu_serif_t  imu_serif;

/* Initialize serial interface */
imu_serif.read_reg   = si_io_imu_read_reg;
imu_serif.write_reg  = si_io_imu_write_reg;
imu_serif.max_read   = 32768;
imu_serif.max_write  = 32768;
imu_serif.serif_type = SERIF_TYPE;
imu_serif.context    = 0;

/* Init device */
rc |= inv_imu_init(&imu_dev, &imu_serif, sensor_event_cb);
```
