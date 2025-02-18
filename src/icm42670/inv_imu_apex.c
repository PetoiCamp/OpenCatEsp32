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

#include "inv_imu_apex.h"
#include "inv_imu_extfunc.h"

int inv_imu_apex_enable_ff(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_start_dmp(s);

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_FF_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_FF_ENABLE_EN;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_disable_ff(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_FF_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_FF_ENABLE_DIS;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_enable_smd(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_start_dmp(s);

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_SMD_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_SMD_ENABLE_EN;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_disable_smd(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_SMD_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_SMD_ENABLE_DIS;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_init_parameters_struct(inv_imu_device_t *s, inv_imu_apex_parameters_t *apex_inputs)
{
	int status = 0;
	(void)s;

	/* Default parameters at POR */
	apex_inputs->pedo_amp_th          = APEX_CONFIG3_PEDO_AMP_TH_62_MG;
	apex_inputs->pedo_step_cnt_th     = 0x5;
	apex_inputs->pedo_step_det_th     = 0x2;
	apex_inputs->pedo_sb_timer_th     = APEX_CONFIG4_PEDO_SB_TIMER_TH_150_SAMPLES;
	apex_inputs->pedo_hi_enrgy_th     = APEX_CONFIG4_PEDO_HI_ENRGY_TH_104_MG;
	apex_inputs->tilt_wait_time       = APEX_CONFIG5_TILT_WAIT_TIME_4_S;
	apex_inputs->power_save_time      = APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_8_S;
	apex_inputs->power_save           = APEX_CONFIG0_DMP_POWER_SAVE_EN;
	apex_inputs->sensitivity_mode     = APEX_CONFIG9_SENSITIVITY_MODE_NORMAL;
	apex_inputs->low_energy_amp_th    = APEX_CONFIG2_LOW_ENERGY_AMP_TH_SEL_80_MG;
	apex_inputs->smd_sensitivity      = APEX_CONFIG9_SMD_SENSITIVITY_0;
	apex_inputs->ff_debounce_duration = APEX_CONFIG9_FF_DEBOUNCE_DURATION_2000_MS;
	apex_inputs->ff_max_duration_cm   = APEX_CONFIG12_FF_MAX_DURATION_204_CM;
	apex_inputs->ff_min_duration_cm   = APEX_CONFIG12_FF_MIN_DURATION_10_CM;
	apex_inputs->lowg_peak_th         = APEX_CONFIG10_LOWG_PEAK_TH_563_MG;
	apex_inputs->lowg_peak_hyst       = APEX_CONFIG5_LOWG_PEAK_TH_HYST_156_MG;
	apex_inputs->lowg_samples_th      = APEX_CONFIG10_LOWG_TIME_TH_1_SAMPLE;
	apex_inputs->highg_peak_th        = APEX_CONFIG11_HIGHG_PEAK_TH_2500_MG;
	apex_inputs->highg_peak_hyst      = APEX_CONFIG5_HIGHG_PEAK_TH_HYST_156_MG;
	apex_inputs->highg_samples_th     = APEX_CONFIG11_HIGHG_TIME_TH_1_SAMPLE;

	return status;
}

int inv_imu_apex_configure_parameters(inv_imu_device_t *               s,
                                      const inv_imu_apex_parameters_t *apex_inputs)
{
	int                        status = 0;
	uint8_t                    data;
	uint8_t                    apexConfig[7];
	APEX_CONFIG1_PED_ENABLE_t  pedo_state;
	APEX_CONFIG1_TILT_ENABLE_t tilt_state;
	APEX_CONFIG1_FF_ENABLE_t   ff_state;
	APEX_CONFIG1_SMD_ENABLE_t  smd_state;

	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &data);
	pedo_state = (APEX_CONFIG1_PED_ENABLE_t)(data & APEX_CONFIG1_PED_ENABLE_MASK);
	tilt_state = (APEX_CONFIG1_TILT_ENABLE_t)(data & APEX_CONFIG1_TILT_ENABLE_MASK);
	ff_state   = (APEX_CONFIG1_FF_ENABLE_t)(data & APEX_CONFIG1_FF_ENABLE_MASK);
	smd_state  = (APEX_CONFIG1_SMD_ENABLE_t)(data & APEX_CONFIG1_SMD_ENABLE_MASK);
	if (pedo_state == APEX_CONFIG1_PED_ENABLE_EN)
		return INV_ERROR;
	if (tilt_state == APEX_CONFIG1_TILT_ENABLE_EN)
		return INV_ERROR;
	if (ff_state == APEX_CONFIG1_FF_ENABLE_EN)
		return INV_ERROR;
	if (smd_state == APEX_CONFIG1_SMD_ENABLE_EN)
		return INV_ERROR;

	status |= inv_imu_switch_on_mclk(s);

	/* Power Save mode and low energy amplitude threshold (for Pedometer in Slow Walk mode) */
	/* APEX_CONFIG2_MREG1 */
	apexConfig[0] = (uint8_t)apex_inputs->power_save_time | (uint8_t)apex_inputs->low_energy_amp_th;

	/* Pedometer parameters */
	/* APEX_CONFIG3_MREG1 */
	apexConfig[1] = (uint8_t)apex_inputs->pedo_amp_th |
	                (apex_inputs->pedo_step_cnt_th & APEX_CONFIG3_PED_STEP_CNT_TH_SEL_MASK);

	/* APEX_CONFIG4_MREG1 */
	apexConfig[2] = ((apex_inputs->pedo_step_det_th << APEX_CONFIG4_PED_STEP_DET_TH_SEL_POS) &
	                 APEX_CONFIG4_PED_STEP_DET_TH_SEL_MASK) |
	                (uint8_t)apex_inputs->pedo_sb_timer_th | (uint8_t)apex_inputs->pedo_hi_enrgy_th;

	/* Tilt, Lowg and highg parameters */
	/* APEX_CONFIG5_MREG1 */
	apexConfig[3] = (uint8_t)apex_inputs->tilt_wait_time | (uint8_t)apex_inputs->lowg_peak_hyst |
	                (uint8_t)apex_inputs->highg_peak_hyst;

	status |= inv_imu_write_reg(s, APEX_CONFIG2_MREG1, 4, &apexConfig[0]);

	/* APEX_CONFIG0 */
	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, &apexConfig[0]);
	apexConfig[0] &= ~APEX_CONFIG0_DMP_POWER_SAVE_EN_MASK;
	apexConfig[0] |= apex_inputs->power_save;
	status |= inv_imu_write_reg(s, APEX_CONFIG0, 1, &apexConfig[0]);

	/* free fall parameter, SMD parameter and parameters for Pedometer in Slow Walk mode */
	/* APEX_CONFIG9_MREG1 */
	apexConfig[0] = (uint8_t)apex_inputs->ff_debounce_duration |
	                (uint8_t)apex_inputs->smd_sensitivity | (uint8_t)apex_inputs->sensitivity_mode;

	/* Lowg and highg parameters and free fall parameters */
	/* APEX_CONFIG10_MREG1 */
	apexConfig[1] = (uint8_t)apex_inputs->lowg_peak_th | (uint8_t)apex_inputs->lowg_samples_th;

	/* APEX_CONFIG11_MREG1 */
	apexConfig[2] = (uint8_t)apex_inputs->highg_peak_th | (uint8_t)apex_inputs->highg_samples_th;

	status |= inv_imu_write_reg(s, APEX_CONFIG9_MREG1, 3, &apexConfig[0]);

	/* APEX_CONFIG12_MREG1 */
	apexConfig[0] =
	    (uint8_t)apex_inputs->ff_max_duration_cm | (uint8_t)apex_inputs->ff_min_duration_cm;

	status |= inv_imu_write_reg(s, APEX_CONFIG12_MREG1, 1, &apexConfig[0]);

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_apex_get_parameters(inv_imu_device_t *s, inv_imu_apex_parameters_t *apex_params)
{
	int     status = 0;
	uint8_t data[7];
	uint8_t value;

	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, &value);
	apex_params->power_save =
	    (APEX_CONFIG0_DMP_POWER_SAVE_t)(value & APEX_CONFIG0_DMP_POWER_SAVE_EN_MASK);

	/* Access continuous config registers (CONFIG2-CONFIG11) */
	status |= inv_imu_read_reg(s, APEX_CONFIG2_MREG1, sizeof(data), &data[0]);

	/* Get params from apex_config2 : dmp_power_save_time and low_energy_amp_th */
	apex_params->power_save_time =
	    (APEX_CONFIG2_DMP_POWER_SAVE_TIME_t)(data[0] & APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_MASK);
	apex_params->low_energy_amp_th =
	    (APEX_CONFIG2_LOW_ENERGY_AMP_TH_t)(data[0] & APEX_CONFIG2_LOW_ENERGY_AMP_TH_SEL_MASK);

	/* Get params from apex_config3 : pedo_amp_th and pedo_step_cnt_th */
	apex_params->pedo_amp_th =
	    (APEX_CONFIG3_PEDO_AMP_TH_t)(data[1] & APEX_CONFIG3_PED_AMP_TH_SEL_MASK);
	apex_params->pedo_step_cnt_th =
	    (data[1] & APEX_CONFIG3_PED_STEP_CNT_TH_SEL_MASK) >> APEX_CONFIG3_PED_STEP_CNT_TH_SEL_POS;

	/* Get params from apex_config4 : pedo_step_det_th, pedo_sb_timer_th and pedo_hi_enrgy_th */
	apex_params->pedo_step_det_th =
	    (data[2] & APEX_CONFIG4_PED_STEP_DET_TH_SEL_MASK) >> APEX_CONFIG4_PED_STEP_DET_TH_SEL_POS;
	apex_params->pedo_sb_timer_th =
	    (APEX_CONFIG4_PEDO_SB_TIMER_TH_t)(data[2] & APEX_CONFIG4_PED_SB_TIMER_TH_SEL_MASK);
	apex_params->pedo_hi_enrgy_th =
	    (APEX_CONFIG4_PEDO_HI_ENRGY_TH_t)(data[2] & APEX_CONFIG4_PED_HI_EN_TH_SEL_MASK);

	/* Get params from apex_config5 : tilt_wait_time, lowg_peak_hyst and highg_peak_hyst */
	apex_params->tilt_wait_time =
	    (APEX_CONFIG5_TILT_WAIT_TIME_t)(data[3] & APEX_CONFIG5_TILT_WAIT_TIME_SEL_MASK);
	apex_params->lowg_peak_hyst =
	    (APEX_CONFIG5_LOWG_PEAK_TH_HYST_t)(data[3] & APEX_CONFIG5_LOWG_PEAK_TH_HYST_SEL_MASK);
	apex_params->highg_peak_hyst =
	    (APEX_CONFIG5_HIGHG_PEAK_TH_HYST_t)(data[3] & APEX_CONFIG5_HIGHG_PEAK_TH_HYST_SEL_MASK);

	/* Get params from apex_config9 : ff_debounce_duration, smd_sensitivity and sensitivity_mode */
	apex_params->ff_debounce_duration =
	    (APEX_CONFIG9_FF_DEBOUNCE_DURATION_t)(data[4] & APEX_CONFIG9_FF_DEBOUNCE_DURATION_SEL_MASK);
	apex_params->smd_sensitivity =
	    (APEX_CONFIG9_SMD_SENSITIVITY_t)(data[4] & APEX_CONFIG9_SMD_SENSITIVITY_SEL_MASK);
	apex_params->sensitivity_mode =
	    (APEX_CONFIG9_SENSITIVITY_MODE_t)(data[4] & APEX_CONFIG9_SENSITIVITY_MODE_MASK);

	/* Get params from apex_config10 : lowg_peak_th and lowg_samples_th */
	apex_params->lowg_peak_th =
	    (APEX_CONFIG10_LOWG_PEAK_TH_t)(data[5] & APEX_CONFIG10_LOWG_PEAK_TH_SEL_MASK);
	apex_params->lowg_samples_th =
	    (APEX_CONFIG10_LOWG_TIME_TH_SAMPLES_t)(data[5] & APEX_CONFIG10_LOWG_TIME_TH_SEL_MASK);

	/* Get params from apex_config11 : highg_peak_th and highg_samples_th */
	apex_params->highg_peak_th =
	    (APEX_CONFIG11_HIGHG_PEAK_TH_t)(data[6] & APEX_CONFIG11_HIGHG_PEAK_TH_SEL_MASK);
	apex_params->highg_samples_th =
	    (APEX_CONFIG11_HIGHG_TIME_TH_SAMPLES_t)(data[6] & APEX_CONFIG11_HIGHG_TIME_TH_SEL_MASK);

	/* Access apex reg 12 */
	status |= inv_imu_read_reg(s, APEX_CONFIG12_MREG1, 1, &data[0]);

	/* Get params from apex_config12 : ff_max_duration_cm and ff_min_duration_cm */
	apex_params->ff_max_duration_cm =
	    (APEX_CONFIG12_FF_MAX_DURATION_t)(data[0] & APEX_CONFIG12_FF_MAX_DURATION_SEL_MASK);
	apex_params->ff_min_duration_cm =
	    (APEX_CONFIG12_FF_MIN_DURATION_t)(data[0] & APEX_CONFIG12_FF_MIN_DURATION_SEL_MASK);

	return status;
}

int inv_imu_apex_set_frequency(inv_imu_device_t *s, const APEX_CONFIG1_DMP_ODR_t frequency)
{
	uint8_t value;
	int     status = 0;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);

	value &= ~APEX_CONFIG1_DMP_ODR_MASK;
	value |= frequency;

	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);
	return status;
}

int inv_imu_apex_enable_pedometer(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_start_dmp(s);

	/* Enable Pedometer */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_PED_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_PED_ENABLE_EN;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_disable_pedometer(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	/* Disable Pedometer */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_PED_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_PED_ENABLE_DIS;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_enable_tilt(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	status |= inv_imu_start_dmp(s);

	/* Enable Tilt */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_TILT_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_TILT_ENABLE_EN;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_disable_tilt(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t value;

	/* Disable Tilt */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, &value);
	value &= ~APEX_CONFIG1_TILT_ENABLE_MASK;
	value |= (uint8_t)APEX_CONFIG1_TILT_ENABLE_DIS;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, &value);

	return status;
}

int inv_imu_apex_get_data_activity(inv_imu_device_t *s, inv_imu_apex_step_activity_t *apex_activity)
{
	uint8_t data[4];
	int     status = inv_imu_read_reg(s, APEX_DATA0, 4, data);

	apex_activity->step_cnt       = (uint16_t)((data[1] << 8) | data[0]);
	apex_activity->step_cadence   = data[2];
	apex_activity->activity_class = data[3] & APEX_DATA3_ACTIVITY_CLASS_MASK;

	return status;
}

int inv_imu_apex_get_data_free_fall(inv_imu_device_t *s, uint16_t *freefall_duration)
{
	uint8_t data[2];
	int     status = inv_imu_read_reg(s, APEX_DATA4, 2, &data[0]);

	*freefall_duration = (uint16_t)((data[1] << 8) | data[0]);

	return status;
}
