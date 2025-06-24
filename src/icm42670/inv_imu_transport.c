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

#include "inv_imu_transport.h"
#include "inv_imu_extfunc.h"

#include <stddef.h> /* NULL */

#define TIMEOUT_US 1000000 /* 1 sec */

/* Function definition */
static uint8_t *get_register_cache_addr(void *t, const uint32_t reg);
static int      write_sreg(void *t, uint8_t reg, uint32_t len, const uint8_t *buf);
static int      read_sreg(void *t, uint8_t reg, uint32_t len, uint8_t *buf);
static int      write_mclk_reg(void *t, uint16_t regaddr, uint8_t wr_cnt, const uint8_t *buf);
static int      read_mclk_reg(void *t, uint16_t regaddr, uint8_t rd_cnt, uint8_t *buf);

int inv_imu_init_transport(void *t)
{
	int                       status = 0;
	struct inv_imu_transport *tr     = (struct inv_imu_transport *)t;

	if (tr == NULL)
		return INV_ERROR_BAD_ARG;

	status |= read_sreg(t, (uint8_t)PWR_MGMT0, 1, &(tr->register_cache.pwr_mgmt0_reg));
#if INV_IMU_IS_GYRO_SUPPORTED
	status |= read_sreg(t, (uint8_t)GYRO_CONFIG0, 1, &(tr->register_cache.gyro_config0_reg));
#endif
	status |= read_sreg(t, (uint8_t)ACCEL_CONFIG0, 1, &(tr->register_cache.accel_config0_reg));

	status |=
	    read_mclk_reg(t, (TMST_CONFIG1_MREG1 & 0xFFFF), 1, &(tr->register_cache.tmst_config1_reg));

	tr->need_mclk_cnt = 0;

	return status;
}

int inv_imu_read_reg(void *t, uint32_t reg, uint32_t len, uint8_t *buf)
{
	int rc = 0;

	if (t == NULL)
		return INV_ERROR_BAD_ARG;

	for (uint32_t i = 0; i < len; i++) {
		const uint8_t *cache_addr = get_register_cache_addr(t, reg + i);

		if (cache_addr) {
			buf[i] = *cache_addr;
		} else {
			if (!(reg & 0x10000)) {
				rc |= read_mclk_reg(t, ((reg + i) & 0xFFFF), 1, &buf[i]);
			} else {
				rc |= read_sreg(t, (uint8_t)(reg + i), len - i, &buf[i]);
				break;
			}
		}
	}

	return rc;
}

int inv_imu_write_reg(void *t, uint32_t reg, uint32_t len, const uint8_t *buf)
{
	int rc = 0;

	if (t == NULL)
		return INV_ERROR_BAD_ARG;

	for (uint32_t i = 0; i < len; i++) {
		uint8_t *cache_addr = get_register_cache_addr(t, reg + i);

		if (cache_addr)
			*cache_addr = buf[i];

		if (!(reg & 0x10000))
			rc |= write_mclk_reg(t, ((reg + i) & 0xFFFF), 1, &buf[i]);
	}

	if (reg & 0x10000)
		rc |= write_sreg(t, (uint8_t)reg, len, buf);

	return rc;
}

int inv_imu_switch_on_mclk(void *t)
{
	int                       status = 0;
	uint8_t                   data;
	struct inv_imu_transport *tr = (struct inv_imu_transport *)t;

	if (tr == NULL)
		return INV_ERROR_BAD_ARG;

	/* set IDLE bit only if it is not set yet */
	if (tr->need_mclk_cnt == 0) {
		uint64_t start;

		status |= inv_imu_read_reg(t, PWR_MGMT0, 1, &data);
		data |= PWR_MGMT0_IDLE_MASK;
		status |= inv_imu_write_reg(t, PWR_MGMT0, 1, &data);

		if (status)
			return status;

		/* Check if MCLK is ready */
		start = inv_imu_get_time_us();
		do {
			status = inv_imu_read_reg(t, MCLK_RDY, 1, &data);

			if (status)
				return status;

			/* Timeout */
			if (inv_imu_get_time_us() - start > TIMEOUT_US)
				return INV_ERROR_TIMEOUT;

		} while (!(data & MCLK_RDY_MCLK_RDY_MASK));
	} else {
		/* Make sure it is already on */
		status |= inv_imu_read_reg(t, PWR_MGMT0, 1, &data);
		if (0 == (data &= PWR_MGMT0_IDLE_MASK))
			status |= INV_ERROR;
	}

	/* Increment the counter to keep track of number of MCLK requesters */
	tr->need_mclk_cnt++;

	return status;
}

int inv_imu_switch_off_mclk(void *t)
{
	int                       status = 0;
	uint8_t                   data;
	struct inv_imu_transport *tr = (struct inv_imu_transport *)t;

	if (tr == NULL)
		return INV_ERROR_BAD_ARG;

	/* Reset the IDLE but only if there is one requester left */
	if (tr->need_mclk_cnt == 1) {
		status |= inv_imu_read_reg(t, PWR_MGMT0, 1, &data);
		data &= ~PWR_MGMT0_IDLE_MASK;
		status |= inv_imu_write_reg(t, PWR_MGMT0, 1, &data);
	} else {
		/* Make sure it is still on */
		status |= inv_imu_read_reg(t, PWR_MGMT0, 1, &data);
		if (0 == (data &= PWR_MGMT0_IDLE_MASK))
			status |= INV_ERROR;
	}

	/* Decrement the counter */
	tr->need_mclk_cnt--;

	return status;
}

/* Static function */

static uint8_t *get_register_cache_addr(void *t, const uint32_t reg)
{
	struct inv_imu_transport *tr = (struct inv_imu_transport *)t;

	if (tr == NULL)
		return (uint8_t *)0; /* error */

	switch (reg) {
	case PWR_MGMT0:
		return &(tr->register_cache.pwr_mgmt0_reg);
#if INV_IMU_IS_GYRO_SUPPORTED
	case GYRO_CONFIG0:
		return &(tr->register_cache.gyro_config0_reg);
#endif
	case ACCEL_CONFIG0:
		return &(tr->register_cache.accel_config0_reg);
	case TMST_CONFIG1_MREG1:
		return &(tr->register_cache.tmst_config1_reg);
	default:
		return (uint8_t *)0; // Not found
	}
}

static int read_sreg(void *t, uint8_t reg, uint32_t len, uint8_t *buf)
{
	struct inv_imu_serif *serif = (struct inv_imu_serif *)t;

	if (serif == NULL)
		return INV_ERROR_BAD_ARG;

	if (len > serif->max_read)
		return INV_ERROR_SIZE;

	if (serif->read_reg(serif, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static int write_sreg(void *t, uint8_t reg, uint32_t len, const uint8_t *buf)
{
	struct inv_imu_serif *serif = (struct inv_imu_serif *)t;

	if (serif == NULL)
		return INV_ERROR_BAD_ARG;

	if (len > serif->max_write)
		return INV_ERROR_SIZE;

	if (serif->write_reg(serif, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static int read_mclk_reg(void *t, uint16_t regaddr, uint8_t rd_cnt, uint8_t *buf)
{
	uint8_t data;
	uint8_t blk_sel = (regaddr & 0xFF00) >> 8;
	int     status  = 0;

	if (t == NULL)
		return INV_ERROR_BAD_ARG;

	/* Have IMU not in IDLE mode to access MCLK domain */
	status |= inv_imu_switch_on_mclk(t);

	/* optimize by changing BLK_SEL only if not NULL */
	if (blk_sel)
		status |= write_sreg(t, (uint8_t)BLK_SEL_R & 0xff, 1, &blk_sel);

	data = (regaddr & 0x00FF);
	status |= write_sreg(t, (uint8_t)MADDR_R, 1, &data);
	inv_imu_sleep_us(10);
	status |= read_sreg(t, (uint8_t)M_R, rd_cnt, buf);
	inv_imu_sleep_us(10);

	if (blk_sel) {
		data = 0;
		status |= write_sreg(t, (uint8_t)BLK_SEL_R, 1, &data);
	}

	/* switch OFF MCLK if needed */
	status |= inv_imu_switch_off_mclk(t);

	return status;
}

static int write_mclk_reg(void *t, uint16_t regaddr, uint8_t wr_cnt, const uint8_t *buf)
{
	uint8_t data;
	uint8_t blk_sel = (regaddr & 0xFF00) >> 8;
	int     status  = 0;

	if (t == NULL)
		return INV_ERROR_BAD_ARG;

	/* Have IMU not in IDLE mode to access MCLK domain */
	status |= inv_imu_switch_on_mclk(t);

	/* optimize by changing BLK_SEL only if not NULL */
	if (blk_sel)
		status |= write_sreg(t, (uint8_t)BLK_SEL_W, 1, &blk_sel);

	data = (regaddr & 0x00FF);
	status |= write_sreg(t, (uint8_t)MADDR_W, 1, &data);
	for (uint8_t i = 0; i < wr_cnt; i++) {
		status |= write_sreg(t, (uint8_t)M_W, 1, &buf[i]);
		inv_imu_sleep_us(10);
	}

	if (blk_sel) {
		data   = 0;
		status = write_sreg(t, (uint8_t)BLK_SEL_W, 1, &data);
	}

	status |= inv_imu_switch_off_mclk(t);

	return status;
}
