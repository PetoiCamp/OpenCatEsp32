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

#ifndef _INV_IMU_H_
#define _INV_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IMU IMU
 *  @brief Describes IMU
 *  @{
 */

/** @file inv_imu.h */


/* Device ID */
#define ICM42670P

/* Device description */
#define INV_IMU_STRING_ID         "ICM42670P"
#define INV_IMU_WHOAMI            0x67
#define INV_IMU_REV               INV_IMU_REV_A
#define INV_IMU_IS_GYRO_SUPPORTED 1
#define INV_IMU_HFSR_SUPPORTED    0


#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
