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

/** @defgroup selftest Self-Test
 *  @brief High-level functions for Self-Test procedures
 *  @{
 */

/** @file inv_imu_selftest.h */

#ifndef _INV_IMU_SELFTEST_H_
#define _INV_IMU_SELFTEST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "inv_imu_driver.h"

/** Self-test input parameters */
typedef struct {
	ST_CONFIG_NUM_SAMPLES_t     st_num_samples; /**< Number of samples used to perform self-test */
	SELFTEST_ACCEL_GYRO_ST_EN_t st_control; /**< Define which sensor is under self-test */
} inv_imu_selftest_parameters_t;

/** Self-test routine outputs */
typedef struct {
	int8_t accel_status; /**< global accel self-test passed */
	int8_t ax_status; /**< AX self-test status */
	int8_t ay_status; /**< AY self-test status */
	int8_t az_status; /**< AZ self-test status */
#if INV_IMU_IS_GYRO_SUPPORTED
	int8_t gyro_status; /**< global gyro self-test status: st_pass (bit0), st_incomplete (bit1) */
	int8_t gx_status; /**< GX self-test status */
	int8_t gy_status; /**< GY self-test status */
	int8_t gz_status; /**< GZ self-test status */
#endif
} inv_imu_selftest_output_t;

/** @brief Execute self-test.
 *  @param[in] s           Pointer to device.
 *  @param[in] st_params   Self-test parameters.
 *  @param[out] st_output  Self-test results.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_run_selftest(inv_imu_device_t *s, const inv_imu_selftest_parameters_t st_params,
                         inv_imu_selftest_output_t *st_output);

/** @brief Fill the self-test configuration structure with default configuration.
 *  @param[in] s                Pointer to device.
 *  @param[in] selftest_params  Self-test parameters to be initialized.
 *  @return                     0 on success, negative value on error.
 */
int inv_imu_init_selftest_parameters_struct(inv_imu_device_t *             s,
                                            inv_imu_selftest_parameters_t *selftest_params);

/** @brief Load self-test data.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_load_selftest_data(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_SELFTEST_H_ */

/** @} */
