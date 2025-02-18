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

/** @defgroup ExtFunc ExtFunc
 *  @brief External functions (to be implemented in application) required by the driver.
 *  @{
 */

/** @file  inv_imu_extfunc.h */

#ifndef _INV_IMU_EXTFUNC_H_
#define _INV_IMU_EXTFUNC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Sleep function.
 *  @param[in] us  Sleep duration in microseconds (us).
 */
extern void inv_imu_sleep_us(uint32_t us);

/** @brief Get time function. Value is expected to be on 64 bits with a 1 us resolution.
 *  @return  The current time in us.
 */
extern uint64_t inv_imu_get_time_us(void);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EXTFUNC_H_ */

/** @} */
