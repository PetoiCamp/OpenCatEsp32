/*
 *
 * Copyright (c) [2017] by InvenSense, Inc.
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

/** @defgroup Apex APEX
 *  @brief High-level functions to drive APEX features
 *  @{
 */

/** @file inv_imu_apex.h */

#ifndef _INV_IMU_APEX_H_
#define _INV_IMU_APEX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "inv_imu_driver.h"

/** IMU APEX inputs parameters definition */
typedef struct {
	APEX_CONFIG3_PEDO_AMP_TH_t            pedo_amp_th;
	uint8_t                               pedo_step_cnt_th;
	uint8_t                               pedo_step_det_th;
	APEX_CONFIG4_PEDO_SB_TIMER_TH_t       pedo_sb_timer_th;
	APEX_CONFIG4_PEDO_HI_ENRGY_TH_t       pedo_hi_enrgy_th;
	APEX_CONFIG5_TILT_WAIT_TIME_t         tilt_wait_time;
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_t    power_save_time;
	APEX_CONFIG0_DMP_POWER_SAVE_t         power_save;
	APEX_CONFIG9_SENSITIVITY_MODE_t       sensitivity_mode;
	APEX_CONFIG2_LOW_ENERGY_AMP_TH_t      low_energy_amp_th;
	APEX_CONFIG9_SMD_SENSITIVITY_t        smd_sensitivity;
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_t   ff_debounce_duration;
	APEX_CONFIG12_FF_MAX_DURATION_t       ff_max_duration_cm;
	APEX_CONFIG12_FF_MIN_DURATION_t       ff_min_duration_cm;
	APEX_CONFIG10_LOWG_PEAK_TH_t          lowg_peak_th;
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_t      lowg_peak_hyst;
	APEX_CONFIG10_LOWG_TIME_TH_SAMPLES_t  lowg_samples_th;
	APEX_CONFIG11_HIGHG_PEAK_TH_t         highg_peak_th;
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_t     highg_peak_hyst;
	APEX_CONFIG11_HIGHG_TIME_TH_SAMPLES_t highg_samples_th;
} inv_imu_apex_parameters_t;

/** APEX pedometer outputs */
typedef struct inv_imu_apex_step_activity {
	/** Number of steps taken */
	uint16_t step_cnt;

	/** Walk/run cadence in number of samples. 
	 *  Format is u6.2. (at 50Hz and 2Hz walk frequency, if the cadence 
	 *  is 25 samples, the register will output 100). 
	 */
	uint8_t step_cadence;

	/** Detected activity. 
	 *  Unknown (0), Walk (1) or Run (2) 
	 */
	uint8_t activity_class;
} inv_imu_apex_step_activity_t;

/** @brief Enable Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_ff(inv_imu_device_t *s);

/** @brief Disable Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_ff(inv_imu_device_t *s);

/** @brief Enable Significant Motion Detection.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 *  @warning SMD requires to have the pedometer enabled to work.
 */
int inv_imu_apex_enable_smd(inv_imu_device_t *s);

/** @brief Disable Significant Motion Detection.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_smd(inv_imu_device_t *s);

/** @brief Fill the APEX parameters structure with all the default parameters for APEX algorithms.
 *  @param[in] s             Pointer to device.
 *  @param[out] apex_inputs  Default input parameters.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_apex_init_parameters_struct(inv_imu_device_t *         s,
                                        inv_imu_apex_parameters_t *apex_inputs);

/** @brief Configures DMP parameters for APEX algorithms. 
 *  @param[in] s            Pointer to device.
 *  @param[in] apex_inputs  The requested input parameters.
 *  @return                 0 on success, negative value on error.
 *  @warning APEX inputs can't change on the fly, this should be called before enabling 
 *           any APEX features.
 *  @warning This API can't be called twice within 10 ms.
 */
int inv_imu_apex_configure_parameters(inv_imu_device_t *               s,
                                      const inv_imu_apex_parameters_t *apex_inputs);

/** @brief Returns current DMP parameters for APEX algorithms.
 *  @param[in] s             Pointer to device.
 *  @param[out] apex_params  The current parameter, fetched from registers.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_apex_get_parameters(inv_imu_device_t *s, inv_imu_apex_parameters_t *apex_params);

/** @brief Configure DMP Output Data Rate for APEX algorithms.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 *  @warning Accel frequency must be higher or equal to DMP frequency.
 */
int inv_imu_apex_set_frequency(inv_imu_device_t *s, const APEX_CONFIG1_DMP_ODR_t frequency);

/** @brief Enable APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_pedometer(inv_imu_device_t *s);

/** @brief Disable APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_pedometer(inv_imu_device_t *s);

/** @brief Enable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_tilt(inv_imu_device_t *s);

/** @brief Disable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_tilt(inv_imu_device_t *s);

/** @brief  Retrieve APEX pedometer outputs and format them
 *  @param[in] s              Pointer to device.
 *  @param[out] apex_activity Apex step and activity data value.
 *  @return                   0 on success, negative value on error.
 */
int inv_imu_apex_get_data_activity(inv_imu_device_t *            s,
                                   inv_imu_apex_step_activity_t *apex_activity);

/** @brief  Retrieve APEX free fall outputs and format them
 *  @param[in] s                   Pointer to device.
 *  @param[out] freefall_duration  Free fall duration in number of sample.
 *  @return                        0 on success, negative value on error.
 */
int inv_imu_apex_get_data_free_fall(inv_imu_device_t *s, uint16_t *freefall_duration);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_APEX_H_ */

/** @} */
