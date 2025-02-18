/*
 *
 * Copyright (c) [2018] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "inv_imu_driver.h"
#include "inv_imu_extfunc.h"
#include "inv_imu_version.h"

#include <stddef.h> /* NULL */
#include <string.h> /* memset */

#if INV_IMU_HFSR_SUPPORTED
/* Address of DMP_CONFIG1 register */
#define DMP_CONFIG1_MREG1  0x2c
/* SRAM first bank ID */
#define SRAM_START_BANK    0x50
#endif

/* Static functions declaration */
static int select_rcosc(inv_imu_device_t *s);
static int select_wuosc(inv_imu_device_t *s);
static int configure_serial_interface(inv_imu_device_t *s);
static int init_hardware_from_ui(inv_imu_device_t *s);
#if INV_IMU_HFSR_SUPPORTED
static int read_and_check_sram(struct inv_imu_device *self, const uint8_t *data, uint32_t offset,
                               uint32_t size);
#endif

int inv_imu_init(inv_imu_device_t *s, const struct inv_imu_serif *serif,
                 void (*sensor_event_cb)(inv_imu_sensor_event_t *event))
{
	int status = 0;

	memset(s, 0, sizeof(*s));

	/* Verify validity of `serif` variable */
	if (serif == NULL || serif->read_reg == NULL || serif->write_reg == NULL)
		return INV_ERROR;

	s->transport.serif = *serif;

	/* Supply ramp time max is 3 ms */
	inv_imu_sleep_us(3000);

	/* Register sensor event callback */
	s->sensor_event_cb = sensor_event_cb;

	/* Make sure `need_mclk_cnt` is cleared */
	s->transport.need_mclk_cnt = 0;

	/* Configure serial interface so we can trigger device reset */
	status |= configure_serial_interface(s);

	/* Reset device */
	status |= inv_imu_device_reset(s);

	/* Init transport layer */
	status |= inv_imu_init_transport(s);

	/* Read and set endianness for further processing */
	status |= inv_imu_get_endianness(s);

	/* Initialize hardware */
	status |= init_hardware_from_ui(s);

	/* Set default value for sensor start/stop time */
#if INV_IMU_IS_GYRO_SUPPORTED
	s->gyro_start_time_us = UINT32_MAX;
#endif
	s->accel_start_time_us = UINT32_MAX;

	return status;
}

int inv_imu_device_reset(inv_imu_device_t *s)
{
	int     status = INV_ERROR_SUCCESS;
	uint8_t data;

	/* Ensure BLK_SEL_R and BLK_SEL_W are set to 0 */
	data = 0;
	status |= inv_imu_write_reg(s, BLK_SEL_R, 1, &data);
	status |= inv_imu_write_reg(s, BLK_SEL_W, 1, &data);

	/* Trigger soft reset */
	data = (uint8_t)SIGNAL_PATH_RESET_SOFT_RESET_DEVICE_CONFIG_EN;
	status |= inv_imu_write_reg(s, SIGNAL_PATH_RESET, 1, &data);

	/* Wait 1ms for soft reset to be effective */
	inv_imu_sleep_us(1000);

	/* Re-configure serial interface since it was reset */
	status |= configure_serial_interface(s);

	/* Clear the reset done interrupt */
	status |= inv_imu_read_reg(s, INT_STATUS, 1, &data);
	if (data != INT_STATUS_RESET_DONE_INT_MASK)
		status |= INV_ERROR_UNEXPECTED;

	return status;
}

int inv_imu_get_who_am_i(inv_imu_device_t *s, uint8_t *who_am_i)
{
	return inv_imu_read_reg(s, WHO_AM_I, 1, who_am_i);
}

int inv_imu_enable_accel_low_power_mode(inv_imu_device_t *s)
{
	int                    status = 0;
	PWR_MGMT0_ACCEL_MODE_t accel_mode;
#if INV_IMU_IS_GYRO_SUPPORTED
	PWR_MGMT0_GYRO_MODE_t gyro_mode;
#endif
	ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint32_t            accel_odr_us = 0;
	uint8_t             pwr_mgmt0_reg;
	uint8_t             accel_config0_reg;
	uint8_t             value;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	accel_mode = (PWR_MGMT0_ACCEL_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_ACCEL_MODE_MASK);
#if INV_IMU_IS_GYRO_SUPPORTED
	gyro_mode = (PWR_MGMT0_GYRO_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_GYRO_MODE_MASK);
#endif

	/* Check if the accelerometer is the only one enabled */
	if ((accel_mode != PWR_MGMT0_ACCEL_MODE_LP)
#if INV_IMU_IS_GYRO_SUPPORTED
	    && ((gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) || (gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY))
#endif
	) {
		/* Get accelerometer's ODR for next required wait */
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &accel_config0_reg);
		acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
		accel_odr_us     = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= select_rcosc(s);
	}

	/* Enable/Switch the accelerometer in/to low power mode */
	/* Read a new time because select_rcosc() modified it */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	pwr_mgmt0_reg &= ~PWR_MGMT0_ACCEL_MODE_MASK;
	pwr_mgmt0_reg |= PWR_MGMT0_ACCEL_MODE_LP;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	inv_imu_sleep_us(200);

	if ((accel_mode != PWR_MGMT0_ACCEL_MODE_LP)
#if INV_IMU_IS_GYRO_SUPPORTED
	    && ((gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) || (gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY))
#endif
	) {
		/* 
		 * Wait one accelerometer ODR before switching to the WU OSC.
		 * if ODR is smaller than 200 us, we already waited for one ODR above. 
		 */
		if (accel_odr_us > 200)
			inv_imu_sleep_us(accel_odr_us - 200);
		status |= select_wuosc(s);
	}

	if (accel_mode == PWR_MGMT0_ACCEL_MODE_OFF && s->fifo_is_used) {
		/*
		 * First data are wrong after accel enable using IIR filter
		 * There is no signal that says accel start-up has completed and data are stable 
		 * using FIR filter. So keep track of the time at start-up to discard the invalid data, 
		 * about 20 ms after enable.
		 */
		s->accel_start_time_us = inv_imu_get_time_us();
	}

	/* Enable the automatic RCOSC power on so that FIFO is entirely powered on */
	status |= inv_imu_read_reg(s, FIFO_CONFIG6_MREG1, 1, &value);
	value &= ~FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_MASK;
	status |= inv_imu_write_reg(s, FIFO_CONFIG6_MREG1, 1, &value);

	return status;
}

int inv_imu_enable_accel_low_noise_mode(inv_imu_device_t *s)
{
	int                    status = 0;
	PWR_MGMT0_ACCEL_MODE_t accel_mode;
#if INV_IMU_IS_GYRO_SUPPORTED
	PWR_MGMT0_GYRO_MODE_t gyro_mode;
#endif
	ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint8_t             pwr_mgmt0_reg;
	uint8_t             accel_config0_reg;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	accel_mode = (PWR_MGMT0_ACCEL_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_ACCEL_MODE_MASK);
#if INV_IMU_IS_GYRO_SUPPORTED
	gyro_mode = (PWR_MGMT0_GYRO_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_GYRO_MODE_MASK);
#endif

	/* Check if the accelerometer is the only one enabled */
	if ((accel_mode == PWR_MGMT0_ACCEL_MODE_LP)
#if INV_IMU_IS_GYRO_SUPPORTED
	    && ((gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) || (gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY))
#endif
	) {
		uint32_t accel_odr_us;
		/* Get accelerometer's ODR for next required wait */
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &accel_config0_reg);
		acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
		accel_odr_us     = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= select_rcosc(s);
		/* Wait one accel ODR before switching to low noise mode */
		inv_imu_sleep_us(accel_odr_us);
	}

	/* Enable/Switch the accelerometer in/to low noise mode */
	/* Read a new time because select_rcosc() modified it */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	pwr_mgmt0_reg &= ~PWR_MGMT0_ACCEL_MODE_MASK;
	pwr_mgmt0_reg |= PWR_MGMT0_ACCEL_MODE_LN;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	inv_imu_sleep_us(200);

	if (accel_mode == PWR_MGMT0_ACCEL_MODE_OFF && s->fifo_is_used) {
		/* 
		 * First data are wrong after accel enable using IIR filter
		 * There is no signal that says accel start-up has completed and data are stable
		 * using FIR filter. So keep track of the time at start-up to discard the invalid data, 
		 * about 20ms after enable.
		 */
		s->accel_start_time_us = inv_imu_get_time_us();
	}

	return status;
}

int inv_imu_disable_accel(inv_imu_device_t *s)
{
	int      status          = 0;
	int      stop_fifo_usage = 0;
	uint32_t accel_odr_us;
#if INV_IMU_IS_GYRO_SUPPORTED
	PWR_MGMT0_GYRO_MODE_t gyro_mode;
#endif
	ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint8_t             pwr_mgmt0_reg;
	uint8_t             accel_config0_reg;
	uint8_t             fifo_cfg_6_reg;

	/* Get accelerometer's ODR for next required wait */
	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &accel_config0_reg);
	acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
	accel_odr_us     = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
#if INV_IMU_IS_GYRO_SUPPORTED
	gyro_mode = (PWR_MGMT0_GYRO_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_GYRO_MODE_MASK);
#endif
	if (s->fifo_is_used
#if INV_IMU_IS_GYRO_SUPPORTED
	    && (gyro_mode == PWR_MGMT0_GYRO_MODE_OFF)
#endif
	) {
		/* 
		 * Accel is off and gyro is about to be turned off. 
		 * Flush FIFO so that there is no old data at next enable time
		 */
		stop_fifo_usage = 1;
	}

	/* Check if accel is the last sensor enabled and bit rcosc dis is not set */
	status |= inv_imu_read_reg(s, FIFO_CONFIG6_MREG1, 1, &fifo_cfg_6_reg);
	if (((fifo_cfg_6_reg & FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_MASK) == 0)
#if INV_IMU_IS_GYRO_SUPPORTED
	    && (gyro_mode == PWR_MGMT0_GYRO_MODE_OFF)
#endif
	) {
		/* 
		 * Disable the automatic RCOSC power on to avoid extra power consumption 
		 * in sleep mode (all sensors and clocks off) 
		 */
		fifo_cfg_6_reg |= ((1 & FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_MASK)
		                   << FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_POS);
		status |= inv_imu_write_reg(s, FIFO_CONFIG6_MREG1, 1, &fifo_cfg_6_reg);
		inv_imu_sleep_us(accel_odr_us);
	}

	pwr_mgmt0_reg &= ~PWR_MGMT0_ACCEL_MODE_MASK;
	pwr_mgmt0_reg |= PWR_MGMT0_ACCEL_MODE_OFF;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);

	if (stop_fifo_usage && s->fifo_is_used) {
		/* Reset FIFO explicitly so there is no data left in FIFO once all sensors are off */
		status |= inv_imu_reset_fifo(s);
	}

	return status;
}

int inv_imu_set_accel_frequency(inv_imu_device_t *s, const ACCEL_CONFIG0_ODR_t frequency)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &data);
	data &= ~ACCEL_CONFIG0_ACCEL_ODR_MASK;
	data |= frequency;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG0, 1, &data);

	return status;
}

int inv_imu_set_accel_lp_avg(inv_imu_device_t *s, ACCEL_CONFIG1_ACCEL_FILT_AVG_t acc_avg)
{
	uint8_t value;
	int     status = 0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~ACCEL_CONFIG1_ACCEL_UI_AVG_MASK;
	value |= acc_avg;

	status |= inv_imu_write_reg(s, ACCEL_CONFIG1, 1, &value);

	return status;
}

int inv_imu_set_accel_ln_bw(inv_imu_device_t *s, ACCEL_CONFIG1_ACCEL_FILT_BW_t acc_bw)
{
	uint8_t value;
	int     status = 0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~ACCEL_CONFIG1_ACCEL_UI_FILT_BW_MASK;
	value |= acc_bw;

	status |= inv_imu_write_reg(s, ACCEL_CONFIG1, 1, &value);

	return status;
}

int inv_imu_set_accel_fsr(inv_imu_device_t *s, ACCEL_CONFIG0_FS_SEL_t accel_fsr_g)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &data);
	data &= ~ACCEL_CONFIG0_ACCEL_UI_FS_SEL_MASK;
	data |= accel_fsr_g;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG0, 1, &data);

	return status;
}

int inv_imu_get_accel_fsr(inv_imu_device_t *s, ACCEL_CONFIG0_FS_SEL_t *accel_fsr_g)
{
	int     status = 0;
	uint8_t accel_config0_reg;

	if ((s->fifo_highres_enabled) && (s->fifo_is_used == INV_IMU_FIFO_ENABLED))
		*accel_fsr_g = ACCEL_CONFIG0_FS_SEL_MAX;
	else {
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &accel_config0_reg);
		*accel_fsr_g =
		    (ACCEL_CONFIG0_FS_SEL_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_UI_FS_SEL_MASK);
	}

	return status;
}

#if INV_IMU_IS_GYRO_SUPPORTED
int inv_imu_enable_gyro_low_noise_mode(inv_imu_device_t *s)
{
	int                    status = 0;
	PWR_MGMT0_ACCEL_MODE_t accel_mode;
	PWR_MGMT0_GYRO_MODE_t  gyro_mode;
	ACCEL_CONFIG0_ODR_t    acc_odr_bitfield;
	uint8_t                pwr_mgmt0_reg;
	uint8_t                accel_config0_reg;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	accel_mode = (PWR_MGMT0_ACCEL_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_ACCEL_MODE_MASK);
	gyro_mode  = (PWR_MGMT0_GYRO_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_GYRO_MODE_MASK);
	/* Check if the accelerometer is the only one enabled */
	if ((accel_mode == PWR_MGMT0_ACCEL_MODE_LP) &&
	    ((gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) || (gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY))) {
		uint32_t accel_odr_us;
		/* Get accelerometer's ODR for next required wait */
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &accel_config0_reg);
		acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
		accel_odr_us     = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= select_rcosc(s);
		/* Wait one accel ODR before enabling the gyroscope */
		inv_imu_sleep_us(accel_odr_us);
	}

	/* Enable/Switch the gyroscope in/to low noise mode */
	/* Read a new time because select_rcosc() modified it */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	pwr_mgmt0_reg &= ~PWR_MGMT0_GYRO_MODE_MASK;
	pwr_mgmt0_reg |= (uint8_t)PWR_MGMT0_GYRO_MODE_LN;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	inv_imu_sleep_us(200);

	if (gyro_mode == PWR_MGMT0_GYRO_MODE_OFF && s->fifo_is_used) {
		/* 
		 * First data are wrong after gyro enable using IIR filter
		 * There is no signal that says Gyro start-up has completed and data are stable
		 * using FIR filter and the Gyro max start-up time is 40ms. So keep track of the time 
		 * at start-up to discard the invalid data, about 60ms after enable.
		 */
		s->gyro_start_time_us = inv_imu_get_time_us();
	}

	return status;
}

int inv_imu_disable_gyro(inv_imu_device_t *s)
{
	int                    status          = 0;
	int                    stop_fifo_usage = 0;
	ACCEL_CONFIG0_ODR_t    acc_odr_bitfield;
	uint32_t               accel_odr_us;
	PWR_MGMT0_ACCEL_MODE_t accel_mode;
	uint8_t                pwr_mgmt0_reg;
	uint8_t                accel_config0_reg;
	uint8_t                fifo_cfg_6_reg;

	/* Get accelerometer's ODR for next required wait */
	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, &accel_config0_reg);
	acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
	accel_odr_us     = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	accel_mode = (PWR_MGMT0_ACCEL_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_ACCEL_MODE_MASK);

	if ((accel_mode == PWR_MGMT0_ACCEL_MODE_OFF) && s->fifo_is_used) {
		/* 
		 * Accel is off and gyro is about to be turned off. 
		 * Flush FIFO so that there is no old data at next enable time
		 */
		stop_fifo_usage = 1;
	}

	/* Check if the accelerometer is enabled in low power mode */
	if (accel_mode == PWR_MGMT0_ACCEL_MODE_LP) {
		/* Select the RC OSC as clock source for the accelerometer */
		status |= select_rcosc(s);
	}

	/* Check if gyro is the last sensor enabled and bit rcosc dis is not set */
	status |= inv_imu_read_reg(s, FIFO_CONFIG6_MREG1, 1, &fifo_cfg_6_reg);
	if ((accel_mode == PWR_MGMT0_ACCEL_MODE_OFF) &&
	    ((fifo_cfg_6_reg & FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_MASK) == 0)) {
		GYRO_CONFIG0_ODR_t gyro_odr_bitfield;
		uint32_t           gyro_odr_us;
		uint8_t            gyro_config0_reg;

		/* Read gyro odr to apply it to the sleep */
		status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, &gyro_config0_reg);
		gyro_odr_bitfield = (GYRO_CONFIG0_ODR_t)(gyro_config0_reg & GYRO_CONFIG0_GYRO_ODR_MASK);
		gyro_odr_us       = inv_imu_convert_odr_bitfield_to_us(gyro_odr_bitfield);

		/* 
		 * Disable the automatic RCOSC power on to avoid extra power consumption 
		 * in sleep mode (all sensors and clocks off) 
		 */
		fifo_cfg_6_reg |= ((1 & FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_MASK)
		                   << FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_POS);
		status |= inv_imu_write_reg(s, FIFO_CONFIG6_MREG1, 1, &fifo_cfg_6_reg);
		inv_imu_sleep_us(gyro_odr_us);
	}

	/* Read a new time because select_rcosc() modified it */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);
	pwr_mgmt0_reg &= ~PWR_MGMT0_GYRO_MODE_MASK;
	pwr_mgmt0_reg |= PWR_MGMT0_GYRO_MODE_OFF;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &pwr_mgmt0_reg);

	if (accel_mode == PWR_MGMT0_ACCEL_MODE_LP) {
		/* Wait based on accelerometer ODR */
		inv_imu_sleep_us(2 * accel_odr_us);
		/* Select the WU OSC as clock source for the accelerometer */
		status |= select_wuosc(s);
	}

	if (stop_fifo_usage && s->fifo_is_used) {
		/* Reset FIFO explicitly so there is no data left in FIFO once all sensors are off */
		status |= inv_imu_reset_fifo(s);
	}

	return status;
}

int inv_imu_set_gyro_frequency(inv_imu_device_t *s, const GYRO_CONFIG0_ODR_t frequency)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, &data);
	data &= ~GYRO_CONFIG0_GYRO_ODR_MASK;
	data |= frequency;
	status |= inv_imu_write_reg(s, GYRO_CONFIG0, 1, &data);

	return status;
}

int inv_imu_set_gyro_ln_bw(inv_imu_device_t *s, GYRO_CONFIG1_GYRO_FILT_BW_t gyr_bw)
{
	uint8_t value;
	int     status = 0;

	status |= inv_imu_read_reg(s, GYRO_CONFIG1, 1, &value);
	if (status)
		return status;

	value &= ~GYRO_CONFIG1_GYRO_UI_FILT_BW_MASK;
	value |= gyr_bw;

	status |= inv_imu_write_reg(s, GYRO_CONFIG1, 1, &value);

	return status;
}

int inv_imu_set_gyro_fsr(inv_imu_device_t *s, GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, &data);
	data &= ~GYRO_CONFIG0_GYRO_UI_FS_SEL_MASK;
	data |= gyro_fsr_dps;
	status |= inv_imu_write_reg(s, GYRO_CONFIG0, 1, &data);

	return status;
}

int inv_imu_get_gyro_fsr(inv_imu_device_t *s, GYRO_CONFIG0_FS_SEL_t *gyro_fsr_dps)
{
	int     status = 0;
	uint8_t gyro_config0_reg;

	if ((s->fifo_highres_enabled) && (s->fifo_is_used == INV_IMU_FIFO_ENABLED))
		*gyro_fsr_dps = GYRO_CONFIG0_FS_SEL_MAX;
	else {
		status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, &gyro_config0_reg);
		*gyro_fsr_dps =
		    (GYRO_CONFIG0_FS_SEL_t)(gyro_config0_reg & GYRO_CONFIG0_GYRO_UI_FS_SEL_MASK);
	}

	return status;
}

int inv_imu_enable_fsync(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_switch_on_mclk(s);

	//Enable Fsync
	status |= inv_imu_read_reg(s, FSYNC_CONFIG_MREG1, 1, &value);
	value &= ~FSYNC_CONFIG_FSYNC_UI_SEL_MASK;
	value |= (uint8_t)FSYNC_CONFIG_UI_SEL_TEMP;
	status |= inv_imu_write_reg(s, FSYNC_CONFIG_MREG1, 1, &value);

	status |= inv_imu_read_reg(s, TMST_CONFIG1_MREG1, 1, &value);
	value &= ~TMST_CONFIG1_TMST_FSYNC_EN_MASK;
	value |= TMST_CONFIG1_TMST_FSYNC_EN;
	status |= inv_imu_write_reg(s, TMST_CONFIG1_MREG1, 1, &value);

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_disable_fsync(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_switch_on_mclk(s);

	// Disable Fsync
	status |= inv_imu_read_reg(s, FSYNC_CONFIG_MREG1, 1, &value);
	value &= ~FSYNC_CONFIG_FSYNC_UI_SEL_MASK;
	value |= (uint8_t)FSYNC_CONFIG_UI_SEL_NO;
	status |= inv_imu_write_reg(s, FSYNC_CONFIG_MREG1, 1, &value);

	status |= inv_imu_read_reg(s, TMST_CONFIG1_MREG1, 1, &value);
	value &= ~TMST_CONFIG1_TMST_FSYNC_EN_MASK;
	value |= TMST_CONFIG1_TMST_FSYNC_DIS;
	status |= inv_imu_write_reg(s, TMST_CONFIG1_MREG1, 1, &value);

	status |= inv_imu_switch_off_mclk(s);

	return status;
}
#endif /* INV_IMU_IS_GYRO_SUPPORTED */

int inv_imu_set_spi_slew_rate(inv_imu_device_t *s, const DRIVE_CONFIG3_SPI_SLEW_RATE_t slew_rate)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, DRIVE_CONFIG3, 1, &value);
	value &= ~DRIVE_CONFIG3_SPI_SLEW_RATE_MASK;
	value |= slew_rate;
	status |= inv_imu_write_reg(s, DRIVE_CONFIG3, 1, &value);

	return status;
}

int inv_imu_set_pin_config_int1(inv_imu_device_t *s, const inv_imu_int1_pin_config_t *conf)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, INT_CONFIG, 1, &value);
	value &= ~INT_CONFIG_INT1_POLARITY_MASK;
	value &= ~INT_CONFIG_INT1_MODE_MASK;
	value &= ~INT_CONFIG_INT1_DRIVE_CIRCUIT_MASK;
	value |= conf->int_polarity;
	value |= conf->int_mode;
	value |= conf->int_drive;
	status |= inv_imu_write_reg(s, INT_CONFIG, 1, &value);

	return status;
}

int inv_imu_set_pin_config_int2(inv_imu_device_t *s, const inv_imu_int2_pin_config_t *conf)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, INT_CONFIG, 1, &value);
	value &= ~INT_CONFIG_INT2_POLARITY_MASK;
	value &= ~INT_CONFIG_INT2_MODE_MASK;
	value &= ~INT_CONFIG_INT2_DRIVE_CIRCUIT_MASK;
	value |= conf->int_polarity;
	value |= conf->int_mode;
	value |= conf->int_drive;
	status |= inv_imu_write_reg(s, INT_CONFIG, 1, &value);

	return status;
}

int inv_imu_get_config_int1(inv_imu_device_t *s, inv_imu_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, INT_SOURCE0, 1, &data);
	it->INV_UI_FSYNC  = (inv_imu_interrupt_value)((data & INT_SOURCE0_FSYNC_INT1_EN_MASK) >>
                                                 INT_SOURCE0_FSYNC_INT1_EN_POS);
	it->INV_UI_DRDY   = (inv_imu_interrupt_value)((data & INT_SOURCE0_DRDY_INT1_EN_MASK) >>
                                                INT_SOURCE0_DRDY_INT1_EN_POS);
	it->INV_FIFO_THS  = (inv_imu_interrupt_value)((data & INT_SOURCE0_FIFO_THS_INT1_EN_MASK) >>
                                                 INT_SOURCE0_FIFO_THS_INT1_EN_POS);
	it->INV_FIFO_FULL = (inv_imu_interrupt_value)((data & INT_SOURCE0_FIFO_FULL_INT1_EN_MASK) >>
	                                              INT_SOURCE0_FIFO_FULL_INT1_EN_POS);

	status |= inv_imu_read_reg(s, INT_SOURCE1, 1, &data);
	it->INV_SMD   = (inv_imu_interrupt_value)((data & INT_SOURCE1_SMD_INT1_EN_MASK) >>
                                            INT_SOURCE1_SMD_INT1_EN_POS);
	it->INV_WOM_X = (inv_imu_interrupt_value)((data & INT_SOURCE1_WOM_X_INT1_EN_MASK) >>
	                                          INT_SOURCE1_WOM_X_INT1_EN_POS);
	it->INV_WOM_Y = (inv_imu_interrupt_value)((data & INT_SOURCE1_WOM_Y_INT1_EN_MASK) >>
	                                          INT_SOURCE1_WOM_Y_INT1_EN_POS);
	it->INV_WOM_Z = (inv_imu_interrupt_value)((data & INT_SOURCE1_WOM_Z_INT1_EN_MASK) >>
	                                          INT_SOURCE1_WOM_Z_INT1_EN_POS);

	status |= inv_imu_read_reg(s, INT_SOURCE6_MREG1, 1, &data);
	it->INV_FF            = (inv_imu_interrupt_value)((data & INT_SOURCE6_FF_INT1_EN_MASK) >>
                                           INT_SOURCE6_FF_INT1_EN_POS);
	it->INV_LOWG          = (inv_imu_interrupt_value)((data & INT_SOURCE6_LOWG_INT1_EN_MASK) >>
                                             INT_SOURCE6_LOWG_INT1_EN_POS);
	it->INV_STEP_DET      = (inv_imu_interrupt_value)((data & INT_SOURCE6_STEP_DET_INT1_EN_MASK) >>
                                                 INT_SOURCE6_STEP_DET_INT1_EN_POS);
	it->INV_STEP_CNT_OVFL = (inv_imu_interrupt_value)(
	    (data & INT_SOURCE6_STEP_CNT_OFL_INT1_EN_MASK) >> INT_SOURCE6_STEP_CNT_OFL_INT1_EN_POS);
	it->INV_TILT_DET = (inv_imu_interrupt_value)((data & INT_SOURCE6_TILT_DET_INT1_EN_MASK) >>
	                                             INT_SOURCE6_TILT_DET_INT1_EN_POS);

	return status;
}

int inv_imu_get_config_int2(inv_imu_device_t *s, inv_imu_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, INT_SOURCE3, 1, &data);
	it->INV_UI_FSYNC  = (inv_imu_interrupt_value)((data & INT_SOURCE3_FSYNC_INT2_EN_MASK) >>
                                                 INT_SOURCE3_FSYNC_INT2_EN_POS);
	it->INV_UI_DRDY   = (inv_imu_interrupt_value)((data & INT_SOURCE3_DRDY_INT2_EN_MASK) >>
                                                INT_SOURCE3_DRDY_INT2_EN_POS);
	it->INV_FIFO_THS  = (inv_imu_interrupt_value)((data & INT_SOURCE3_FIFO_THS_INT2_EN_MASK) >>
                                                 INT_SOURCE3_FIFO_THS_INT2_EN_POS);
	it->INV_FIFO_FULL = (inv_imu_interrupt_value)((data & INT_SOURCE3_FIFO_FULL_INT2_EN_MASK) >>
	                                              INT_SOURCE3_FIFO_FULL_INT2_EN_POS);

	status |= inv_imu_read_reg(s, INT_SOURCE4, 1, &data);
	it->INV_SMD   = (inv_imu_interrupt_value)((data & INT_SOURCE4_SMD_INT2_EN_MASK) >>
                                            INT_SOURCE4_SMD_INT2_EN_POS);
	it->INV_WOM_X = (inv_imu_interrupt_value)((data & INT_SOURCE4_WOM_X_INT2_EN_MASK) >>
	                                          INT_SOURCE4_WOM_X_INT2_EN_POS);
	it->INV_WOM_Y = (inv_imu_interrupt_value)((data & INT_SOURCE4_WOM_Y_INT2_EN_MASK) >>
	                                          INT_SOURCE4_WOM_Y_INT2_EN_POS);
	it->INV_WOM_Z = (inv_imu_interrupt_value)((data & INT_SOURCE4_WOM_Z_INT2_EN_MASK) >>
	                                          INT_SOURCE4_WOM_Z_INT2_EN_POS);

	status |= inv_imu_read_reg(s, INT_SOURCE7_MREG1, 1, &data);
	it->INV_FF            = (inv_imu_interrupt_value)((data & INT_SOURCE7_FF_INT2_EN_MASK) >>
                                           INT_SOURCE7_FF_INT2_EN_POS);
	it->INV_LOWG          = (inv_imu_interrupt_value)((data & INT_SOURCE7_LOWG_INT2_EN_MASK) >>
                                             INT_SOURCE7_LOWG_INT2_EN_POS);
	it->INV_STEP_DET      = (inv_imu_interrupt_value)((data & INT_SOURCE7_STEP_DET_INT2_EN_MASK) >>
                                                 INT_SOURCE7_STEP_DET_INT2_EN_POS);
	it->INV_STEP_CNT_OVFL = (inv_imu_interrupt_value)(
	    (data & INT_SOURCE7_STEP_CNT_OFL_INT2_EN_MASK) >> INT_SOURCE7_STEP_CNT_OFL_INT2_EN_POS);
	it->INV_TILT_DET = (inv_imu_interrupt_value)((data & INT_SOURCE7_TILT_DET_INT2_EN_MASK) >>
	                                             INT_SOURCE7_TILT_DET_INT2_EN_POS);

	return status;
}

int inv_imu_set_config_int1(inv_imu_device_t *s, const inv_imu_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data[2];

	status |= inv_imu_read_reg(s, INT_SOURCE0, 2, &data[0]);

	data[0] &= ~(INT_SOURCE0_FSYNC_INT1_EN_MASK | INT_SOURCE0_DRDY_INT1_EN_MASK |
	             INT_SOURCE0_FIFO_THS_INT1_EN_MASK | INT_SOURCE0_FIFO_FULL_INT1_EN_MASK);
	data[0] |= ((it->INV_UI_FSYNC != 0) << INT_SOURCE0_FSYNC_INT1_EN_POS);
	data[0] |= ((it->INV_UI_DRDY != 0) << INT_SOURCE0_DRDY_INT1_EN_POS);
	data[0] |= ((it->INV_FIFO_THS != 0) << INT_SOURCE0_FIFO_THS_INT1_EN_POS);
	data[0] |= ((it->INV_FIFO_FULL != 0) << INT_SOURCE0_FIFO_FULL_INT1_EN_POS);

	data[1] &= ~(INT_SOURCE1_SMD_INT1_EN_MASK | INT_SOURCE1_WOM_X_INT1_EN_MASK |
	             INT_SOURCE1_WOM_Y_INT1_EN_MASK | INT_SOURCE1_WOM_Z_INT1_EN_MASK);
	data[1] |= ((it->INV_SMD != 0) << INT_SOURCE1_SMD_INT1_EN_POS);
	data[1] |= ((it->INV_WOM_X != 0) << INT_SOURCE1_WOM_X_INT1_EN_POS);
	data[1] |= ((it->INV_WOM_Y != 0) << INT_SOURCE1_WOM_Y_INT1_EN_POS);
	data[1] |= ((it->INV_WOM_Z != 0) << INT_SOURCE1_WOM_Z_INT1_EN_POS);

	status |= inv_imu_write_reg(s, INT_SOURCE0, 2, &data[0]);

	status |= inv_imu_read_reg(s, INT_SOURCE6_MREG1, 1, &data[0]);

	data[0] &= ~(INT_SOURCE6_FF_INT1_EN_MASK | INT_SOURCE6_LOWG_INT1_EN_MASK |
	             INT_SOURCE6_STEP_DET_INT1_EN_MASK | INT_SOURCE6_STEP_CNT_OFL_INT1_EN_MASK |
	             INT_SOURCE6_TILT_DET_INT1_EN_MASK);
	data[0] |= ((it->INV_FF != 0) << INT_SOURCE6_FF_INT1_EN_POS);
	data[0] |= ((it->INV_LOWG != 0) << INT_SOURCE6_LOWG_INT1_EN_POS);
	data[0] |= ((it->INV_STEP_DET != 0) << INT_SOURCE6_STEP_DET_INT1_EN_POS);
	data[0] |= ((it->INV_STEP_CNT_OVFL != 0) << INT_SOURCE6_STEP_CNT_OFL_INT1_EN_POS);
	data[0] |= ((it->INV_TILT_DET != 0) << INT_SOURCE6_TILT_DET_INT1_EN_POS);
	status |= inv_imu_write_reg(s, INT_SOURCE6_MREG1, 1, &data[0]);

	return status;
}

int inv_imu_set_config_int2(inv_imu_device_t *s, const inv_imu_interrupt_parameter_t *it)
{
	int     status = 0;
	uint8_t data[2];

	status |= inv_imu_read_reg(s, INT_SOURCE3, 2, &data[0]);

	data[0] &= ~(INT_SOURCE3_FSYNC_INT2_EN_MASK | INT_SOURCE3_DRDY_INT2_EN_MASK |
	             INT_SOURCE3_FIFO_THS_INT2_EN_MASK | INT_SOURCE3_FIFO_FULL_INT2_EN_MASK);
	data[0] |= ((it->INV_UI_FSYNC != 0) << INT_SOURCE3_FSYNC_INT2_EN_POS);
	data[0] |= ((it->INV_UI_DRDY != 0) << INT_SOURCE3_DRDY_INT2_EN_POS);
	data[0] |= ((it->INV_FIFO_THS != 0) << INT_SOURCE3_FIFO_THS_INT2_EN_POS);
	data[0] |= ((it->INV_FIFO_FULL != 0) << INT_SOURCE3_FIFO_FULL_INT2_EN_POS);

	data[1] &= ~(INT_SOURCE4_SMD_INT2_EN_MASK | INT_SOURCE4_WOM_X_INT2_EN_MASK |
	             INT_SOURCE4_WOM_Y_INT2_EN_MASK | INT_SOURCE4_WOM_Z_INT2_EN_MASK);
	data[1] |= ((it->INV_SMD != 0) << INT_SOURCE4_SMD_INT2_EN_POS);
	data[1] |= ((it->INV_WOM_X != 0) << INT_SOURCE4_WOM_X_INT2_EN_POS);
	data[1] |= ((it->INV_WOM_Y != 0) << INT_SOURCE4_WOM_Y_INT2_EN_POS);
	data[1] |= ((it->INV_WOM_Z != 0) << INT_SOURCE4_WOM_Z_INT2_EN_POS);

	status |= inv_imu_write_reg(s, INT_SOURCE3, 2, &data[0]);

	status |= inv_imu_read_reg(s, INT_SOURCE7_MREG1, 1, &data[0]);

	data[0] &= ~(INT_SOURCE7_FF_INT2_EN_MASK | INT_SOURCE7_LOWG_INT2_EN_MASK |
	             INT_SOURCE7_STEP_DET_INT2_EN_MASK | INT_SOURCE7_STEP_CNT_OFL_INT2_EN_MASK |
	             INT_SOURCE7_TILT_DET_INT2_EN_MASK);
	data[0] |= ((it->INV_FF != 0) << INT_SOURCE7_FF_INT2_EN_POS);
	data[0] |= ((it->INV_LOWG != 0) << INT_SOURCE7_LOWG_INT2_EN_POS);
	data[0] |= ((it->INV_STEP_DET != 0) << INT_SOURCE7_STEP_DET_INT2_EN_POS);
	data[0] |= ((it->INV_STEP_CNT_OVFL != 0) << INT_SOURCE7_STEP_CNT_OFL_INT2_EN_POS);
	data[0] |= ((it->INV_TILT_DET != 0) << INT_SOURCE7_TILT_DET_INT2_EN_POS);

	status |= inv_imu_write_reg(s, INT_SOURCE7_MREG1, 1, &data[0]);

	return status;
}

int inv_imu_get_data_from_registers(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t int_status;
	uint8_t accel[ACCEL_DATA_SIZE];
#if INV_IMU_IS_GYRO_SUPPORTED
	uint8_t gyro[GYRO_DATA_SIZE];
#endif
	inv_imu_sensor_event_t event;

	/* Ensure data ready status bit is set */
	if (status |= inv_imu_read_reg(s, INT_STATUS_DRDY, 1, &int_status))
		return status;

	if (int_status & INT_STATUS_DRDY_DATA_RDY_INT_MASK) {
		uint8_t temperature[2];
		/* Read temperature */
		status |= inv_imu_read_reg(s, TEMP_DATA1, 2, &temperature[0]);
		format_s16_data(s->endianness_data, &temperature[0], &event.temperature);

		/* Read accel */
		status |= inv_imu_read_reg(s, ACCEL_DATA_X1, ACCEL_DATA_SIZE, &accel[0]);
		format_s16_data(s->endianness_data, &accel[0], &event.accel[0]);
		format_s16_data(s->endianness_data, &accel[2], &event.accel[1]);
		format_s16_data(s->endianness_data, &accel[4], &event.accel[2]);

#if INV_IMU_IS_GYRO_SUPPORTED
		status |= inv_imu_read_reg(s, GYRO_DATA_X1, GYRO_DATA_SIZE, &gyro[0]);
		format_s16_data(s->endianness_data, &gyro[0], &event.gyro[0]);
		format_s16_data(s->endianness_data, &gyro[2], &event.gyro[1]);
		format_s16_data(s->endianness_data, &gyro[4], &event.gyro[2]);
#endif

		/* call sensor event callback */
		if (s->sensor_event_cb)
			s->sensor_event_cb(&event);
	}
	/*else: Data Ready was not reached*/

	return status;
}

int inv_imu_get_frame_count(inv_imu_device_t *s, uint16_t *frame_count)
{
	int status = 0;

	status |= inv_imu_read_reg(s, FIFO_COUNTH, 2, (uint8_t *)frame_count);
	format_u16_data(INTF_CONFIG0_DATA_LITTLE_ENDIAN, (uint8_t *)frame_count, frame_count);

	return status;
}

int inv_imu_decode_fifo_frame(inv_imu_device_t *s, const uint8_t *frame,
                              inv_imu_sensor_event_t *event)
{
	int                  status    = 0;
	uint16_t             frame_idx = 0;
	const fifo_header_t *header;

	event->sensor_mask = 0;
	header             = (fifo_header_t *)frame;

	frame_idx += FIFO_HEADER_SIZE;

	/* Check if frame is invalid */
	if (header->Byte == 0x80) {
		/* Header shows that frame is invalid, no need to go further */
		return 0;
	}

	/* Check MSG BIT */
	if (header->bits.msg_bit) {
		/* MSG BIT set in FIFO header, return error */
		return INV_ERROR;
	}

	/* Read Accel */
	if (header->bits.accel_bit) {
		format_s16_data(s->endianness_data, &(frame[0 + frame_idx]), &event->accel[0]);
		format_s16_data(s->endianness_data, &(frame[2 + frame_idx]), &event->accel[1]);
		format_s16_data(s->endianness_data, &(frame[4 + frame_idx]), &event->accel[2]);
		frame_idx += FIFO_ACCEL_DATA_SIZE;
	}

#if INV_IMU_IS_GYRO_SUPPORTED
	/* Read Gyro */
	if (header->bits.gyro_bit) {
		format_s16_data(s->endianness_data, &(frame[0 + frame_idx]), &event->gyro[0]);
		format_s16_data(s->endianness_data, &(frame[2 + frame_idx]), &event->gyro[1]);
		format_s16_data(s->endianness_data, &(frame[4 + frame_idx]), &event->gyro[2]);
		frame_idx += FIFO_GYRO_DATA_SIZE;
	}
#else
	/* 
	 * With 20-Bytes packets, 6B are reserved after accel value. 
	 * Therefore, increment `fifo_idx` accordingly.
	 */
	if (header->bits.twentybits_bit)
		frame_idx += FIFO_GYRO_DATA_SIZE;
#endif

	/* 
	 * The coarse temperature (8 or 16B FIFO packet format) 
	 * range is ± 64 degrees with 0.5°C resolution.
	 * but the fine temperature range (2 bytes) (20B FIFO packet format) is 
	 * ± 256 degrees with (1/128)°C resolution
	 */
	if (header->bits.twentybits_bit) {
		format_s16_data(s->endianness_data, &(frame[0 + frame_idx]), &event->temperature);
		frame_idx += FIFO_TEMP_DATA_SIZE + FIFO_TEMP_HIGH_RES_SIZE;

		/* new temperature data */
		if (event->temperature != INVALID_VALUE_FIFO)
			event->sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
	} else {
		/* cast to int8_t since FIFO is in 16 bits mode (temperature on 8 bits) */
		event->temperature = (int8_t)(frame[0 + frame_idx]);
		frame_idx += FIFO_TEMP_DATA_SIZE;

		/* new temperature data */
		if (event->temperature != INVALID_VALUE_FIFO_1B)
			event->sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
	}

	if (header->bits.timestamp_bit
#if INV_IMU_IS_GYRO_SUPPORTED
	    || header->bits.fsync_bit
#endif
	) {
		format_u16_data(s->endianness_data, &(frame[0 + frame_idx]), &event->timestamp_fsync);
		frame_idx += FIFO_TS_FSYNC_SIZE;
#if INV_IMU_IS_GYRO_SUPPORTED
		/* new fsync event */
		if (header->bits.fsync_bit)
			event->sensor_mask |= (1 << INV_SENSOR_FSYNC_EVENT);
#endif
	}

	if (header->bits.accel_bit &&
	    ((event->accel[0] != INVALID_VALUE_FIFO) && (event->accel[1] != INVALID_VALUE_FIFO) &&
	     (event->accel[2] != INVALID_VALUE_FIFO))) {
		if (s->accel_start_time_us == UINT32_MAX) {
			event->sensor_mask |= (1 << INV_SENSOR_ACCEL);
		} else {
			if (((inv_imu_get_time_us() - s->accel_start_time_us) >= ACC_STARTUP_TIME_US)
#if INV_IMU_IS_GYRO_SUPPORTED
			    && !header->bits.fsync_bit
#endif
			) {
				/* Discard first data after startup to let output to settle */
				s->accel_start_time_us = UINT32_MAX;
				event->sensor_mask |= (1 << INV_SENSOR_ACCEL);
			}
		}

		if ((event->sensor_mask & (1 << INV_SENSOR_ACCEL)) && (header->bits.twentybits_bit)) {
			event->accel_high_res[0] = (frame[0 + frame_idx] >> 4) & 0xF;
			event->accel_high_res[1] = (frame[1 + frame_idx] >> 4) & 0xF;
			event->accel_high_res[2] = (frame[2 + frame_idx] >> 4) & 0xF;
		}
	}

#if INV_IMU_IS_GYRO_SUPPORTED
	if (header->bits.gyro_bit &&
	    ((event->gyro[0] != INVALID_VALUE_FIFO) && (event->gyro[1] != INVALID_VALUE_FIFO) &&
	     (event->gyro[2] != INVALID_VALUE_FIFO))) {
		if (s->gyro_start_time_us == UINT32_MAX) {
			event->sensor_mask |= (1 << INV_SENSOR_GYRO);
		} else {
			if (!header->bits.fsync_bit &&
			    ((inv_imu_get_time_us() - s->gyro_start_time_us) >= GYR_STARTUP_TIME_US)) {
				/* Discard first data after startup to let output to settle */
				s->gyro_start_time_us = UINT32_MAX;
				event->sensor_mask |= (1 << INV_SENSOR_GYRO);
			}
		}

		if ((event->sensor_mask & (1 << INV_SENSOR_GYRO)) && (header->bits.twentybits_bit)) {
			event->gyro_high_res[0] = (frame[0 + frame_idx]) & 0xF;
			event->gyro_high_res[1] = (frame[1 + frame_idx]) & 0xF;
			event->gyro_high_res[2] = (frame[2 + frame_idx]) & 0xF;
		}
	}
#endif

	return status;
}

int inv_imu_get_data_from_fifo(inv_imu_device_t *s)
{
	int      status = 0;
	uint8_t  int_status;
	uint16_t packet_count = 0;
	uint16_t packet_size =
	    s->fifo_highres_enabled ? FIFO_20BYTES_PACKET_SIZE : FIFO_16BYTES_PACKET_SIZE;
	uint16_t fifo_idx = 0;

	/* Ensure data ready status bit is set */
	status |= inv_imu_read_reg(s, INT_STATUS, 1, &int_status);
	if (!(int_status & INT_STATUS_FIFO_THS_INT_MASK) &&
	    !(int_status & INT_STATUS_FIFO_FULL_INT_MASK))
		return 0; /* Neither FIFO THS nor FIFO_FULL is set, simply return here */

	/*
	 * Make sure RCOSC is enabled to guarantee FIFO read.
	 * For power optimization, this call can be omitted under specific conditions:
	 *  - If using WM interrupt and you can guarantee entire FIFO will be read at once.
	 *  - If gyro is enabled or accel is in LN or LP+RCOSC mode.
	 *  - In accel LP+WUOSC mode, if you wait 100 us after reading FIFO_COUNT and 
	 *    you can guarantee that the FIFO will be read within 1 ms.
	 * Please refer to the AN-000324 for more information.
	 */
	status |= inv_imu_switch_on_mclk(s);

	/* Read FIFO frame count */
	status |= inv_imu_get_frame_count(s, &packet_count);

	/* Check for error */
	if (status != INV_ERROR_SUCCESS) {
		status |= inv_imu_switch_off_mclk(s);
		return status;
	}

	/* Read FIFO data */
	status |= inv_imu_read_reg(s, FIFO_DATA, packet_size * packet_count, s->fifo_data);

	/* Check for error */
	if (status != INV_ERROR_SUCCESS) {
		status |= inv_imu_reset_fifo(s);
		status |= inv_imu_switch_off_mclk(s);
		return status;
	}

	/* Process FIFO packets */
	for (uint16_t i = 0; i < packet_count; i++) {
		inv_imu_sensor_event_t event;

		status |= inv_imu_decode_fifo_frame(s, &s->fifo_data[fifo_idx], &event);
		fifo_idx += packet_size;

		/* Check for error */
		if (status != INV_ERROR_SUCCESS) {
			status |= inv_imu_reset_fifo(s);
			status |= inv_imu_switch_off_mclk(s);
			return status;
		}

		/* call sensor event callback */
		if (s->sensor_event_cb)
			s->sensor_event_cb(&event);
	}

	status |= inv_imu_switch_off_mclk(s);

	if (status < 0)
		return status;

	return packet_count;
}

uint32_t inv_imu_convert_odr_bitfield_to_us(uint32_t odr_bitfield)
{
	/*
 odr bitfield - frequency : odr ms
			0 - N/A
			1 - N/A
			2 - N/A
			3 - N/A
			4 - N/A
			5 - 1.6k      : 0.625ms
  (default) 6 - 800       : 1.25ms
			7 - 400       : 2.5 ms
			8 - 200       : 5 ms
			9 - 100       : 10 ms
			10 - 50       : 20 ms
			11 - 25       : 40 ms
			12 - 12.5     : 80 ms
			13 - 6.25     : 160 ms
			14 - 3.125    : 320 ms
			15 - 1.5625   : 640 ms
		*/

	switch (odr_bitfield) {
	case ACCEL_CONFIG0_ODR_1600_HZ:
		return 625;
	case ACCEL_CONFIG0_ODR_800_HZ:
		return 1250;
	case ACCEL_CONFIG0_ODR_400_HZ:
		return 2500;
	case ACCEL_CONFIG0_ODR_200_HZ:
		return 5000;
	case ACCEL_CONFIG0_ODR_100_HZ:
		return 10000;
	case ACCEL_CONFIG0_ODR_50_HZ:
		return 20000;
	case ACCEL_CONFIG0_ODR_25_HZ:
		return 40000;
	case ACCEL_CONFIG0_ODR_12_5_HZ:
		return 80000;
	case ACCEL_CONFIG0_ODR_6_25_HZ:
		return 160000;
	case ACCEL_CONFIG0_ODR_3_125_HZ:
		return 320000;
	case ACCEL_CONFIG0_ODR_1_5625_HZ:
	default:
		return 640000;
	}
}

int inv_imu_set_timestamp_resolution(inv_imu_device_t *         s,
                                     const TMST_CONFIG1_RESOL_t timestamp_resol)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, TMST_CONFIG1_MREG1, 1, &data);
	data &= ~TMST_CONFIG1_TMST_RES_MASK;
	data |= timestamp_resol;
	status |= inv_imu_write_reg(s, TMST_CONFIG1_MREG1, 1, &data);

	return status;
}

int inv_imu_reset_fifo(inv_imu_device_t *s)
{
	int     status            = 0;
	uint8_t fifo_flush_status = (uint8_t)SIGNAL_PATH_RESET_FIFO_FLUSH_EN;

	status |= inv_imu_switch_on_mclk(s);

	status |= inv_imu_write_reg(s, SIGNAL_PATH_RESET, 1, &fifo_flush_status);
	inv_imu_sleep_us(10);

	/* Wait for FIFO flush (idle bit will go high at appropriate time and unlock flush) */
	while ((status == 0) && ((fifo_flush_status & SIGNAL_PATH_RESET_FIFO_FLUSH_MASK) ==
	                         (uint8_t)SIGNAL_PATH_RESET_FIFO_FLUSH_EN)) {
		status |= inv_imu_read_reg(s, SIGNAL_PATH_RESET, 1, &fifo_flush_status);
	}

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_enable_high_resolution_fifo(inv_imu_device_t *s)
{
	uint8_t value;
	int     status = 0;

	/* set FIFO packets to 20bit format (i.e. high res is enabled) */
	s->fifo_highres_enabled = 1;

	status |= inv_imu_read_reg(s, FIFO_CONFIG5_MREG1, 1, &value);
	value &= ~FIFO_CONFIG5_FIFO_HIRES_EN_MASK;
	value |= FIFO_CONFIG5_HIRES_EN;
	status |= inv_imu_write_reg(s, FIFO_CONFIG5_MREG1, 1, &value);

	return status;
}

int inv_imu_disable_high_resolution_fifo(inv_imu_device_t *s)
{
	uint8_t value;
	int     status = 0;

	/* set FIFO packets to 16bit format (i.e. high res is disabled) */
	s->fifo_highres_enabled = 0;

	status |= inv_imu_read_reg(s, FIFO_CONFIG5_MREG1, 1, &value);
	value &= ~FIFO_CONFIG5_FIFO_HIRES_EN_MASK;
	value |= FIFO_CONFIG5_HIRES_DIS;
	status |= inv_imu_write_reg(s, FIFO_CONFIG5_MREG1, 1, &value);

	return status;
}

int inv_imu_configure_fifo(inv_imu_device_t *s, INV_IMU_FIFO_CONFIG_t fifo_config)
{
	int     status = 0;
	uint8_t data;

	s->fifo_is_used = fifo_config;

	inv_imu_switch_on_mclk(s);

	switch (fifo_config) {
	case INV_IMU_FIFO_ENABLED:
		/* Configure:
		  - FIFO record mode i.e FIFO count unit is packet 
		  - FIFO snapshot mode i.e drop the data when the FIFO overflows
		  - Timestamp is logged in FIFO
		  - Little Endian fifo_count
		*/

		status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, &data);
		data &= ~(INTF_CONFIG0_FIFO_COUNT_FORMAT_MASK | INTF_CONFIG0_FIFO_COUNT_ENDIAN_MASK);
		data |= (uint8_t)INTF_CONFIG0_FIFO_COUNT_REC_RECORD |
		        (uint8_t)INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN;
		status |= inv_imu_write_reg(s, INTF_CONFIG0, 1, &data);

		status |= inv_imu_read_reg(s, FIFO_CONFIG1, 1, &data);
		data &= ~(FIFO_CONFIG1_FIFO_MODE_MASK | FIFO_CONFIG1_FIFO_BYPASS_MASK);
		data |= (uint8_t)FIFO_CONFIG1_FIFO_MODE_SNAPSHOT | (uint8_t)FIFO_CONFIG1_FIFO_BYPASS_OFF;
		status |= inv_imu_write_reg(s, FIFO_CONFIG1, 1, &data);

		status |= inv_imu_read_reg(s, TMST_CONFIG1_MREG1, 1, &data);
		data &= ~TMST_CONFIG1_TMST_EN_MASK;
		data |= TMST_CONFIG1_TMST_EN;
		status |= inv_imu_write_reg(s, TMST_CONFIG1_MREG1, 1, &data);

		/* restart and reset FIFO configuration */
		status |= inv_imu_read_reg(s, FIFO_CONFIG5_MREG1, 1, &data);
		data &= ~FIFO_CONFIG5_FIFO_ACCEL_EN_MASK;
		data &= ~FIFO_CONFIG5_FIFO_HIRES_EN_MASK;
#if INV_IMU_IS_GYRO_SUPPORTED
		data &= ~FIFO_CONFIG5_FIFO_GYRO_EN_MASK;
		data &= ~FIFO_CONFIG5_FIFO_TMST_FSYNC_EN_MASK;
		data |= (uint8_t)FIFO_CONFIG5_GYRO_EN;
		data |= (uint8_t)FIFO_CONFIG5_TMST_FSYNC_EN;
#endif
		data |= (uint8_t)FIFO_CONFIG5_ACCEL_EN;
		data |= (uint8_t)FIFO_CONFIG5_WM_GT_TH_EN;
		status |= inv_imu_write_reg(s, FIFO_CONFIG5_MREG1, 1, &data);

		/* Configure FIFO WM so that INT is triggered for each packet */
		data = 0x1;
		status |= inv_imu_write_reg(s, FIFO_CONFIG2, 1, &data);
		break;

	case INV_IMU_FIFO_DISABLED:
		/* make sure FIFO is disabled */
		status |= inv_imu_read_reg(s, FIFO_CONFIG1, 1, &data);
		data &= ~FIFO_CONFIG1_FIFO_BYPASS_MASK;
		data |= (uint8_t)FIFO_CONFIG1_FIFO_BYPASS_ON;
		status |= inv_imu_write_reg(s, FIFO_CONFIG1, 1, &data);

		/* restart and reset FIFO configuration */
		status |= inv_imu_read_reg(s, FIFO_CONFIG5_MREG1, 1, &data);
		data &= ~FIFO_CONFIG5_FIFO_ACCEL_EN_MASK;
#if INV_IMU_IS_GYRO_SUPPORTED
		data &= ~FIFO_CONFIG5_FIFO_GYRO_EN_MASK;
		data &= ~FIFO_CONFIG5_FIFO_TMST_FSYNC_EN_MASK;
		data |= (uint8_t)FIFO_CONFIG5_GYRO_DIS;
		data |= (uint8_t)FIFO_CONFIG5_TMST_FSYNC_EN;
#endif
		data |= (uint8_t)FIFO_CONFIG5_ACCEL_DIS;
		status |= inv_imu_write_reg(s, FIFO_CONFIG5_MREG1, 1, &data);
		break;

	default:
		status = -1;
	}

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

uint32_t inv_imu_get_timestamp_resolution_us(inv_imu_device_t *s)
{
	int                  status = 0;
	uint8_t              tmst_config1_reg;
	TMST_CONFIG1_RESOL_t tmst_resol;

	status |= inv_imu_read_reg(s, TMST_CONFIG1_MREG1, 1, &tmst_config1_reg);
	if (status < 0)
		return INV_ERROR;

	tmst_resol = (TMST_CONFIG1_RESOL_t)(tmst_config1_reg & TMST_CONFIG1_TMST_RES_MASK);

	if (tmst_resol == TMST_CONFIG1_RESOL_16us)
		return 16;
	else if (tmst_resol == TMST_CONFIG1_RESOL_1us)
		return 1;

	// Should not happen, return 0
	return 0;
}

int inv_imu_configure_wom(inv_imu_device_t *s, const uint8_t wom_x_th, const uint8_t wom_y_th,
                          const uint8_t wom_z_th, WOM_CONFIG_WOM_INT_MODE_t wom_int,
                          WOM_CONFIG_WOM_INT_DUR_t wom_dur)
{
	int     status = 0;
	uint8_t data[3];
	uint8_t value;

	data[0] = wom_x_th; // Set X threshold
	data[1] = wom_y_th; // Set Y threshold
	data[2] = wom_z_th; // Set Z threshold
	status |= inv_imu_write_reg(s, ACCEL_WOM_X_THR_MREG1, sizeof(data), &data[0]);

	// Compare current sample with the previous sample and WOM from the 3 axis are ORed or ANDed to produce WOM signal.
	status |= inv_imu_read_reg(s, WOM_CONFIG, 1, &value);
	value &= ~WOM_CONFIG_WOM_INT_MODE_MASK;
	value |= (uint8_t)WOM_CONFIG_WOM_MODE_CMP_PREV | (uint8_t)wom_int;

	// Configure the number of overthreshold event to wait before producing the WOM signal.
	value &= ~WOM_CONFIG_WOM_INT_DUR_MASK;
	value |= (uint8_t)wom_dur;
	status |= inv_imu_write_reg(s, WOM_CONFIG, 1, &value);

	return status;
}

int inv_imu_enable_wom(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	/* Enable WOM */
	status |= inv_imu_read_reg(s, WOM_CONFIG, 1, &value);
	value &= ~WOM_CONFIG_WOM_EN_MASK;
	value |= (uint8_t)WOM_CONFIG_WOM_EN_ENABLE;
	status |= inv_imu_write_reg(s, WOM_CONFIG, 1, &value);

	return status;
}

int inv_imu_disable_wom(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	/* Disable WOM */
	status |= inv_imu_read_reg(s, WOM_CONFIG, 1, &value);
	value &= ~WOM_CONFIG_WOM_EN_MASK;
	value |= WOM_CONFIG_WOM_EN_DISABLE;
	status |= inv_imu_write_reg(s, WOM_CONFIG, 1, &value);

	return status;
}

int inv_imu_start_dmp(inv_imu_device_t *s)
{
	int status = 0;

	// On first enabling of DMP, reset internal state
	if (!s->dmp_is_on) {
		// Reset SRAM to 0's
		status |= inv_imu_reset_dmp(s, APEX_CONFIG0_DMP_MEM_RESET_APEX_ST_EN);
		if (status)
			return status;
		s->dmp_is_on = 1;

#if INV_IMU_HFSR_SUPPORTED
		{
			uint8_t data;
			static uint8_t ram_img[] = {
				#include "dmp3Default_xian_hfsr_rom_patch.txt"
			};

			/* HFSR parts requires to prescale accel data using a patch in SRAM */
			status |= inv_imu_write_sram(s, ram_img, 320, sizeof(ram_img));

			/* Set DMP start point to beginning of the patch i.e. SRAM start + offset = 320 */
			data = (320 / 32);
			status |= inv_imu_write_reg(s, DMP_CONFIG1_MREG1, 1, &data);
		}
#endif
	}

	// Initialize DMP
	status |= inv_imu_resume_dmp(s);

	return status;
}

int inv_imu_resume_dmp(struct inv_imu_device *s)
{
	int      status = 0;
	uint8_t  value;
	uint64_t start;

	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, &value);
	value &= ~APEX_CONFIG0_DMP_INIT_EN_MASK;
	value |= (uint8_t)APEX_CONFIG0_DMP_INIT_EN;
	status |= inv_imu_write_reg(s, APEX_CONFIG0, 1, &value);

	/* wait to make sure dmp_init_en = 0 */
	start = inv_imu_get_time_us();
	do {
		inv_imu_read_reg(s, APEX_CONFIG0, 1, &value);
		inv_imu_sleep_us(100);

		if ((value & APEX_CONFIG0_DMP_INIT_EN_MASK) == 0)
			break;

	} while (inv_imu_get_time_us() - start < 50000);

	return status;
}

int inv_imu_reset_dmp(inv_imu_device_t *s, const APEX_CONFIG0_DMP_MEM_RESET_t sram_reset)
{
	const int ref_timeout = 5000; /*50 ms*/
	int       status      = 0;
	int       timeout     = ref_timeout;
	uint8_t   data_dmp_reset;
	uint8_t   value = 0;

	status |= inv_imu_switch_on_mclk(s);

	// Reset DMP internal memories
	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, &value);
	value &= ~APEX_CONFIG0_DMP_MEM_RESET_EN_MASK;
	value |= (sram_reset & APEX_CONFIG0_DMP_MEM_RESET_EN_MASK);
	status |= inv_imu_write_reg(s, APEX_CONFIG0, 1, &value);

	inv_imu_sleep_us(1000);

	// Make sure reset procedure has finished by reading back mem_reset_en bit
	do {
		inv_imu_sleep_us(10);
		status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, &data_dmp_reset);
	} while (
	    ((data_dmp_reset & APEX_CONFIG0_DMP_MEM_RESET_EN_MASK) != APEX_CONFIG0_DMP_MEM_RESET_DIS) &&
	    timeout-- && !status);

	status |= inv_imu_switch_off_mclk(s);

	if (timeout <= 0)
		return INV_ERROR_TIMEOUT;

	return status;
}

int inv_imu_set_endianness(inv_imu_device_t *s, INTF_CONFIG0_DATA_ENDIAN_t endianness)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, &value);
	value &= ~INTF_CONFIG0_SENSOR_DATA_ENDIAN_MASK;
	value |= (uint8_t)endianness;
	status |= inv_imu_write_reg(s, INTF_CONFIG0, 1, &value);

	if (!status)
		s->endianness_data = (uint8_t)endianness;

	return status;
}

int inv_imu_get_endianness(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, &value);
	if (!status)
		s->endianness_data = value & INTF_CONFIG0_SENSOR_DATA_ENDIAN_MASK;

	return status;
}

int inv_imu_configure_fifo_data_rate(inv_imu_device_t *s, FDR_CONFIG_FDR_SEL_t dec_factor)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, FDR_CONFIG_MREG1, 1, &data);
	data &= (uint8_t)~FDR_CONFIG_FDR_SEL_MASK;
	data |= (uint8_t)dec_factor;
	status |= inv_imu_write_reg(s, FDR_CONFIG_MREG1, 1, &data);

	return status;
}

const char *inv_imu_get_version(void)
{
	return INV_IMU_VERSION_STRING;
}

#if INV_IMU_HFSR_SUPPORTED
int inv_imu_write_sram(struct inv_imu_device *s, const uint8_t *data, uint32_t offset,
                       uint32_t size)
{
	int     rc = 0;
	uint8_t memory_bank;
	uint8_t dmp_memory_address;

	if (size + offset > 1280U)
		return INV_ERROR_SIZE;

	/* make sure mclk is on */
	rc |= inv_imu_switch_on_mclk(s);

	/* Write memory pointed by data into DMP memory */
	memory_bank        = (uint8_t)(SRAM_START_BANK + (offset / 256));
	dmp_memory_address = (uint8_t)(offset % 256);
	rc |= inv_imu_write_reg(s, BLK_SEL_W, 1, &memory_bank);
	inv_imu_sleep_us(10);
	rc |= inv_imu_write_reg(s, MADDR_W, 1, &dmp_memory_address);
	inv_imu_sleep_us(10);

	for (uint32_t i = offset; i < size + offset; i++) {
		if (0 == (i % 256)) {
			memory_bank        = (uint8_t)(SRAM_START_BANK + (i / 256));
			dmp_memory_address = 0;
			rc |= inv_imu_write_reg(s, BLK_SEL_W, 1, &memory_bank);
			inv_imu_sleep_us(10);
			rc |= inv_imu_write_reg(s, MADDR_W, 1, &dmp_memory_address);
			inv_imu_sleep_us(10);
		}

		rc |= inv_imu_write_reg(s, M_W, 1, &data[i - offset]);
		inv_imu_sleep_us(10);
	}

	memory_bank = 0;
	rc |= inv_imu_write_reg(s, BLK_SEL_W, 1, &memory_bank);

	/* cancel mclk request */
	rc |= inv_imu_switch_off_mclk(s);

	rc |= read_and_check_sram(s, data, offset, size);

	return rc;
}
#endif

/*
 * Static functions definition
 */
static int select_rcosc(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &data);
	data &= ~PWR_MGMT0_ACCEL_LP_CLK_SEL_MASK;
	data |= PWR_MGMT0_ACCEL_LP_CLK_RCOSC;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &data);

	return status;
}

static int select_wuosc(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t data;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &data);
	data &= ~PWR_MGMT0_ACCEL_LP_CLK_SEL_MASK;
	data |= PWR_MGMT0_ACCEL_LP_CLK_WUOSC;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &data);

	return status;
}

static int configure_serial_interface(inv_imu_device_t *s)
{
	uint8_t value;
	int     status = 0;

	/* Ensure BLK_SEL_R and BLK_SEL_W are set to 0 */
	value = 0;
	status |= inv_imu_write_reg(s, BLK_SEL_R, 1, &value);
	status |= inv_imu_write_reg(s, BLK_SEL_W, 1, &value);

	if (s->transport.serif.serif_type == UI_I2C) {
		/* Enable I2C 50ns spike filtering */
		status |= inv_imu_read_reg(s, INTF_CONFIG1, 1, &value);
		value &= ~(INTF_CONFIG1_I3C_SDR_EN_MASK | INTF_CONFIG1_I3C_DDR_EN_MASK);
		status |= inv_imu_write_reg(s, INTF_CONFIG1, 1, &value);
	} else {
		/* Configure SPI */
		if (s->transport.serif.serif_type == UI_SPI4)
			value = (uint8_t)DEVICE_CONFIG_SPI_4WIRE | (uint8_t)DEVICE_CONFIG_SPI_MODE_0_3;
		else if (s->transport.serif.serif_type == UI_SPI3)
			value = (uint8_t)DEVICE_CONFIG_SPI_3WIRE | (uint8_t)DEVICE_CONFIG_SPI_MODE_0_3;
		else
			return INV_ERROR_BAD_ARG; /* Not supported */
		status |= inv_imu_write_reg(s, DEVICE_CONFIG, 1, &value);

#if INV_IMU_REV == INV_IMU_REV_A
		/* Device operation in shared spi bus configuration (AN-000352) */
		status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, &value);
		value |= 0x3;
		status |= inv_imu_write_reg(s, INTF_CONFIG0, 1, &value);
#endif
	}

	return status;
}

static int init_hardware_from_ui(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

#if INV_IMU_IS_GYRO_SUPPORTED
	/* Deactivate FSYNC by default */
	status |= inv_imu_disable_fsync(s);
#endif

	/* Set default timestamp resolution 16us (Mobile use cases) */
	status |= inv_imu_set_timestamp_resolution(s, TMST_CONFIG1_RESOL_16us);

	/* Enable FIFO: use 16-bit format by default (i.e. high res is disabled) */
	status |= inv_imu_configure_fifo(s, INV_IMU_FIFO_ENABLED);

	/* 
	 * Disable the automatic RCOSC power on to avoid 
	 * extra power consumption in sleep mode (all sensors and clocks off) 
	 */
	status |= inv_imu_read_reg(s, FIFO_CONFIG6_MREG1, 1, &value);
	value |= ((1 & FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_MASK)
	          << FIFO_CONFIG6_RCOSC_REQ_ON_FIFO_THS_DIS_POS);
	status |= inv_imu_write_reg(s, FIFO_CONFIG6_MREG1, 1, &value);

	return status;
}

#if INV_IMU_HFSR_SUPPORTED
static int read_and_check_sram(struct inv_imu_device *s, const uint8_t *data, uint32_t offset,
                               uint32_t size)
{
	int     rc = 0;
	uint8_t memory_bank;
	uint8_t dmp_memory_address;

	/* make sure mclk is on */
	rc |= inv_imu_switch_on_mclk(s);

	/* Read DMP memory and check it against memory pointed by input parameter */
	memory_bank        = (uint8_t)(SRAM_START_BANK + (offset / 256));
	dmp_memory_address = (uint8_t)(offset % 256);

	rc |= inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);
	inv_imu_sleep_us(10);
	rc |= inv_imu_write_reg(s, MADDR_R, 1, &dmp_memory_address);
	inv_imu_sleep_us(10);

	for (uint32_t i = offset; i < size + offset; i++) {
		uint8_t readByte;

		if (0 == (i % 256)) {
			memory_bank        = (uint8_t)(SRAM_START_BANK + (i / 256));
			dmp_memory_address = 0;
			rc |= inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);
			inv_imu_sleep_us(10);
			rc |= inv_imu_write_reg(s, MADDR_R, 1, &dmp_memory_address);
			inv_imu_sleep_us(10);
		}

		rc |= inv_imu_read_reg(s, M_R, 1, &readByte);
		inv_imu_sleep_us(10);
		if (readByte != data[i - offset]) {
			rc = -1;
			break;
		}
	}

	memory_bank = 0;
	rc |= inv_imu_write_reg(s, BLK_SEL_R, 1, &memory_bank);

	/* cancel mclk request */
	rc |= inv_imu_switch_off_mclk(s);

	return rc;
}
#endif
