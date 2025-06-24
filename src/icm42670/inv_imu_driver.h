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

/** @defgroup Driver Driver
 *  @brief High-level functions to drive the device
 *  @{
 */

/** @file inv_imu_driver.h */

#ifndef _INV_IMU_DRIVER_H_
#define _INV_IMU_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "inv_imu_defs.h"
#include "inv_imu_transport.h"

/** Max FSR values for accel */
#if INV_IMU_HFSR_SUPPORTED
#define ACCEL_CONFIG0_FS_SEL_MAX ACCEL_CONFIG0_FS_SEL_32g
#else
#define ACCEL_CONFIG0_FS_SEL_MAX ACCEL_CONFIG0_FS_SEL_16g
#endif
/** Max user offset value for accel (mg) */
#define ACCEL_OFFUSER_MAX_MG 1000
/** Accel start-up time */
#define ACC_STARTUP_TIME_US 10000

#if INV_IMU_IS_GYRO_SUPPORTED
/** Max FSR values for gyro */
#if INV_IMU_HFSR_SUPPORTED
#define GYRO_CONFIG0_FS_SEL_MAX GYRO_CONFIG0_FS_SEL_4000dps
#else
#define GYRO_CONFIG0_FS_SEL_MAX GYRO_CONFIG0_FS_SEL_2000dps
#endif
/** Max user offset value for gyro (dps) */
#define GYRO_OFFUSER_MAX_DPS 64
/** Gyro start-up time */
#define GYR_STARTUP_TIME_US 70000
#endif

/** Max buffer size mirrored from FIFO at polling time */
#define FIFO_MIRRORING_SIZE 16 * 258 // packet size * max_count = 4kB

/** Sensor identifier for UI control function */
enum inv_imu_sensor {
	INV_SENSOR_ACCEL, /**< Accelerometer */
#if INV_IMU_IS_GYRO_SUPPORTED
	INV_SENSOR_GYRO, /**< Gyroscope */
	INV_SENSOR_FSYNC_EVENT, /**< FSYNC */
#else
	INV_SENSOR_RESERVED1,
	INV_SENSOR_RESERVED2,
#endif
	INV_SENSOR_TEMPERATURE, /**< Chip temperature */
	INV_SENSOR_DMP_PEDOMETER_EVENT, /**< Pedometer: step detected */
	INV_SENSOR_DMP_PEDOMETER_COUNT, /**< Pedometer: step counter */
	INV_SENSOR_DMP_TILT, /**< Tilt */
	INV_SENSOR_DMP_FF, /**< FreeFall */
	INV_SENSOR_DMP_LOWG, /**< Low G */
	INV_SENSOR_DMP_SMD, /**< Significant Motion Detection */
	INV_SENSOR_MAX
};

/** Configure FIFO usage */
typedef enum {
	INV_IMU_FIFO_DISABLED = 0, /**< FIFO is disabled and data source is sensors registers */
	INV_IMU_FIFO_ENABLED  = 1, /**< FIFO is used as data source */
} INV_IMU_FIFO_CONFIG_t;

/** Sensor event structure definition */
typedef struct {
	int      sensor_mask;
	uint16_t timestamp_fsync;
	int16_t  accel[3];
#if INV_IMU_IS_GYRO_SUPPORTED
	int16_t gyro[3];
#endif
	int16_t temperature;
	int8_t  accel_high_res[3];
#if INV_IMU_IS_GYRO_SUPPORTED
	int8_t gyro_high_res[3];
#endif
} inv_imu_sensor_event_t;

/** IMU driver states definition */
typedef struct inv_imu_device {
	/** Transport layer. 
	 *  @warning Must be the first one of inv_imu_device_t 
	 */
	inv_imu_transport_t transport;

	/** callback executed by:
	 *  * inv_imu_get_data_from_fifo (if FIFO is used).
	 *  * inv_imu_get_data_from_registers (if FIFO isn't used).
	 *  May be NULL if above API are not used by application 
	 */
	void (*sensor_event_cb)(inv_imu_sensor_event_t *event);

	uint8_t               fifo_data[FIFO_MIRRORING_SIZE]; /**< FIFO mirroring memory area */
	uint8_t               dmp_is_on; /**< DMP started status */
	uint8_t               endianness_data; /**< Data endianness configuration */
	uint8_t               fifo_highres_enabled; /**< Highres mode configuration */
	INV_IMU_FIFO_CONFIG_t fifo_is_used; /**< FIFO configuration */
#if INV_IMU_IS_GYRO_SUPPORTED
	uint64_t gyro_start_time_us; /**< Gyro start time to discard first samples */
#endif
	uint64_t accel_start_time_us; /**< Accel start time to discard first samples */
} inv_imu_device_t;

/** Interrupt enum state for INT1, INT2, and IBI */
typedef enum { INV_IMU_DISABLE = 0, INV_IMU_ENABLE } inv_imu_interrupt_value;

/** Interrupt definition */
typedef struct {
	inv_imu_interrupt_value INV_UI_FSYNC;
	inv_imu_interrupt_value INV_UI_DRDY;
	inv_imu_interrupt_value INV_FIFO_THS;
	inv_imu_interrupt_value INV_FIFO_FULL;
	inv_imu_interrupt_value INV_SMD;
	inv_imu_interrupt_value INV_WOM_X;
	inv_imu_interrupt_value INV_WOM_Y;
	inv_imu_interrupt_value INV_WOM_Z;
	inv_imu_interrupt_value INV_FF;
	inv_imu_interrupt_value INV_LOWG;
	inv_imu_interrupt_value INV_STEP_DET;
	inv_imu_interrupt_value INV_STEP_CNT_OVFL;
	inv_imu_interrupt_value INV_TILT_DET;
} inv_imu_interrupt_parameter_t;

/** INT1 pin configuration */
typedef struct {
	INT_CONFIG_INT1_POLARITY_t      int_polarity;
	INT_CONFIG_INT1_MODE_t          int_mode;
	INT_CONFIG_INT1_DRIVE_CIRCUIT_t int_drive;
} inv_imu_int1_pin_config_t;

/** INT2 pin configuration */
typedef struct {
	INT_CONFIG_INT2_POLARITY_t      int_polarity;
	INT_CONFIG_INT2_MODE_t          int_mode;
	INT_CONFIG_INT2_DRIVE_CIRCUIT_t int_drive;
} inv_imu_int2_pin_config_t;

/** @brief Initializes device.
 *  @param[in] s                Pointer to device.
 *  @param[in] serif            Pointer on serial interface structure.
 *  @param[in] sensor_event_cb  Callback executed when a new sensor event is available. *
 *  @return                     0 on success, negative value on error.
 */
int inv_imu_init(inv_imu_device_t *s, const struct inv_imu_serif *serif,
                 void (*sensor_event_cb)(inv_imu_sensor_event_t *event));

/** @brief Reset device by reloading OTPs.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_device_reset(inv_imu_device_t *s);

/** @brief return WHOAMI value.
 *  @param[in] s          Pointer to device.
 *  @param[out] who_am_i  WHOAMI for device.
 *  @return 0 on success, negative value on error.
 */
int inv_imu_get_who_am_i(inv_imu_device_t *s, uint8_t *who_am_i);

/** @brief Enable/put accel in low power mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_enable_accel_low_power_mode(inv_imu_device_t *s);

/** @brief Enable/put accel in low noise mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_enable_accel_low_noise_mode(inv_imu_device_t *s);

/** @brief Disable all 3 axes of accel.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_disable_accel(inv_imu_device_t *s);

/** @brief Configure accel Output Data Rate.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_accel_frequency(inv_imu_device_t *s, const ACCEL_CONFIG0_ODR_t frequency);

/** @brief Set accel Low-Power averaging value.
 *  @param[in] s        Pointer to device.
 *  @param[in] acc_avg  Requested averaging value.
 *  @return             0 on success, negative value on error.
 */
int inv_imu_set_accel_lp_avg(inv_imu_device_t *s, ACCEL_CONFIG1_ACCEL_FILT_AVG_t acc_avg);

/** @brief Set accel Low-Noise bandwidth value.
 *  @param[in] s       Pointer to device.
 *  @param[in] acc_bw  Requested averaging value.
 *  @return            0 on success, negative value on error.
 */
int inv_imu_set_accel_ln_bw(inv_imu_device_t *s, ACCEL_CONFIG1_ACCEL_FILT_BW_t acc_bw);

/** @brief Set accel full scale range.
 *  @param[in] s            Pointer to device.
 *  @param[in] accel_fsr_g  Requested full scale range.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_set_accel_fsr(inv_imu_device_t *s, ACCEL_CONFIG0_FS_SEL_t accel_fsr_g);

/** @brief Access accel full scale range.
 *  @param[in] s             Pointer to device.
 *  @param[out] accel_fsr_g  Current full scale range.
 *  @return 0 on success, negative value on error.
 */
int inv_imu_get_accel_fsr(inv_imu_device_t *s, ACCEL_CONFIG0_FS_SEL_t *accel_fsr_g);

#if INV_IMU_IS_GYRO_SUPPORTED
/** @brief Enable/put gyro in low noise mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_enable_gyro_low_noise_mode(inv_imu_device_t *s);

/** @brief Disable all 3 axes of gyro.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_disable_gyro(inv_imu_device_t *s);

/** @brief Configure gyro Output Data Rate.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_gyro_frequency(inv_imu_device_t *s, const GYRO_CONFIG0_ODR_t frequency);

/** @brief Set gyro Low-Noise bandwidth value.
 *  @param[in] s       Pointer to device.
 *  @param[in] gyr_bw  Requested averaging value.
 *  @return            0 on success, negative value on error.
 */
int inv_imu_set_gyro_ln_bw(inv_imu_device_t *s, GYRO_CONFIG1_GYRO_FILT_BW_t gyr_bw);

/** @brief Set gyro full scale range.
 *  @param[in] s             Pointer to device.
 *  @param[in] gyro_fsr_dps  Requested full scale range.
*   @return                  0 on success, negative value on error.
 */
int inv_imu_set_gyro_fsr(inv_imu_device_t *s, GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps);

/** @brief Access gyro full scale range.
 *  @param[in] s              Pointer to device.
 *  @param[out] gyro_fsr_dps  Current full scale range.
 *  @return                   0 on success, negative value on error.
 */
int inv_imu_get_gyro_fsr(inv_imu_device_t *s, GYRO_CONFIG0_FS_SEL_t *gyro_fsr_dps);

/** @brief Enable fsync tagging functionality.
 *         * Enables fsync.
 *         * Enables timestamp to registers. Once fsync is enabled fsync counter is pushed to 
 *           FIFO instead of timestamp. So timestamp is made available in registers. Note that 
 *           this increase power consumption.
 *         * Enables fsync related interrupt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_enable_fsync(inv_imu_device_t *s);

/** @brief Disable fsync tagging functionality.
 *         * Disables fsync.
 *         * Disables timestamp to registers. Once fsync is disabled  timestamp is pushed to FIFO 
 *           instead of fsync counter. So in order to decrease power consumption, timestamp is no 
 *           more available in registers.
 *         * Disables fsync related interrupt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_disable_fsync(inv_imu_device_t *s);
#endif

/** @brief Configure SPI slew-rate.
 *  @param[in] s          Pointer to device.
 *  @param[in] slew_rate  Requested slew-rate.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_spi_slew_rate(inv_imu_device_t *s, const DRIVE_CONFIG3_SPI_SLEW_RATE_t slew_rate);

/** @brief Configure INT1 pin behavior.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure with the requested configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_set_pin_config_int1(inv_imu_device_t *s, const inv_imu_int1_pin_config_t *conf);

/** @brief Configure INT2 pin behavior.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure with the requested configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_set_pin_config_int2(inv_imu_device_t *s, const inv_imu_int2_pin_config_t *conf);

/** @brief Configure which interrupt source can trigger INT1.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  Structure with the corresponding state to manage INT1.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_config_int1(inv_imu_device_t *s, const inv_imu_interrupt_parameter_t *it);

/** @brief Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Structure with the corresponding state to manage INT1.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_get_config_int1(inv_imu_device_t *s, inv_imu_interrupt_parameter_t *it);

/** @brief  Configure which interrupt source can trigger INT2.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  Structure with the corresponding state to INT2.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_config_int2(inv_imu_device_t *s, const inv_imu_interrupt_parameter_t *it);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Structure with the corresponding state to manage INT2.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_get_config_int2(inv_imu_device_t *s, inv_imu_interrupt_parameter_t *it);

/** @brief Read all data registers. 
 *         Then it calls sensor_event_cb function passed at init for each packet.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_get_data_from_registers(inv_imu_device_t *s);

/** @brief Read FIFO frame count. 
 *  @param[in] s             Pointer to device.
 *  @param[out] frame_count  Number of frame available in the FIFO.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_get_frame_count(inv_imu_device_t *s, uint16_t *frame_count);

/** @brief Decode FIFO frame. 
 *  @param[in] s       Pointer to device.
 *  @param[in] frame   FIFO frame data.
 *  @param[out] event  Data content coded as an event.
 *  @return            0 on success, negative value on error.
 */
int inv_imu_decode_fifo_frame(inv_imu_device_t *s, const uint8_t *frame,
                              inv_imu_sensor_event_t *event);

/** @brief Read all available packets from the FIFO. 
 *         For each packet function builds a sensor event containing packet data 
 *         and validity information. Then it calls sensor_event_cb function passed 
 *         at init for each packet.
 *  @param[in] s  Pointer to device.
 *  @return       Number of valid packets read on success, negative value on error.
 */
int inv_imu_get_data_from_fifo(inv_imu_device_t *s);

/** @brief Converts ODR's enum to period in us.
 *  @param[in] odr_bitfield ACCEL_CONFIG0_ODR_t or GYRO_CONFIG0_ODR_t enum.
 *  @return    The corresponding period in us.
 */
uint32_t inv_imu_convert_odr_bitfield_to_us(uint32_t odr_bitfield);

/** @brief Set timestamp resolution.
 *  @param[in] s                Pointer to device.
 *  @param[in] timestamp_resol  Requested timestamp resolution.
 *  @return                     0 on success, negative value on error.
 */
int inv_imu_set_timestamp_resolution(inv_imu_device_t *         s,
                                     const TMST_CONFIG1_RESOL_t timestamp_resol);

/** @brief Reset IMU FIFO.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_reset_fifo(inv_imu_device_t *s);

/** @brief Enable 20 bits raw data in FIFO.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_enable_high_resolution_fifo(inv_imu_device_t *s);

/** @brief Disable 20 bits raw data in FIFO.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_disable_high_resolution_fifo(inv_imu_device_t *s);

/** @brief Configure FIFO.
  *  @param[in] s            Pointer to device.
  *  @param[in] fifo_config  FIFO configuration method.
  *                          Enabled: data are pushed to FIFO and FIFO THS interrupt is set.
  *                          Disabled: data are not pushed to FIFO and DRDY interrupt is set.
  *  @return                 0 on success, negative value on error.
  */
int inv_imu_configure_fifo(inv_imu_device_t *s, INV_IMU_FIFO_CONFIG_t fifo_config);

/** @brief Get timestamp resolution
 *  @param[in] s  Pointer to device.
 *  @return       The timestamp resolution in us or 0 in case of error
 */
uint32_t inv_imu_get_timestamp_resolution_us(inv_imu_device_t *s);

/** @brief  Enable Wake On Motion.
 *  @param[in] s         Pointer to device.
 *  @param[in] wom_x_th  Threshold value for the Wake on Motion Interrupt for X-axis accel.
 *  @param[in] wom_y_th  Threshold value for the Wake on Motion Interrupt for Y-axis accel.
 *  @param[in] wom_z_th  Threshold value for the Wake on Motion Interrupt for Z-axis accel.
 *  @param[in] wom_int   Select which mode between AND/OR is used to generate interrupt.
 *  @param[in] wom_dur   Select the number of over-threshold events to wait before generating
 *                       interrupt.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_configure_wom(inv_imu_device_t *s, const uint8_t wom_x_th, const uint8_t wom_y_th,
                          const uint8_t wom_z_th, WOM_CONFIG_WOM_INT_MODE_t wom_int,
                          WOM_CONFIG_WOM_INT_DUR_t wom_dur);

/** @brief Enable Wake On Motion.
 *         WoM requests to have the accelerometer enabled to work. 
 *         As a consequence FIFO water-mark interrupt is disabled to only trigger WoM interrupts.
 *         To have good performance, it's recommended to set accel ODR to 20 ms and in Low Power
 *         Mode. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_enable_wom(inv_imu_device_t *s);

/** @brief Disable Wake On Motion.
 *         Fifo water-mark interrupt is re-enabled when WoM is disabled.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_disable_wom(inv_imu_device_t *s);

/** @brief Start DMP for APEX algorithms and self-test.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_start_dmp(inv_imu_device_t *s);

/** @brief Resume DMP operations.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_resume_dmp(struct inv_imu_device *s);

/** @brief Reset DMP for APEX algorithms and self-test.
 *  @param[in] s           Pointer to device.
 *  @param[in] sram_reset  Reset mode for the SRAM.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_reset_dmp(inv_imu_device_t *s, const APEX_CONFIG0_DMP_MEM_RESET_t sram_reset);

/** @brief Set the UI endianness and set the inv_device endianness field.
 *  @param[in] s           Pointer to device.
 *  @param[in] endianness  Endianness to be set.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_set_endianness(inv_imu_device_t *s, INTF_CONFIG0_DATA_ENDIAN_t endianness);

/** @brief Read the UI endianness and set the inv_device endianness field.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_get_endianness(inv_imu_device_t *s);

/** @brief Configure FIFO decimation.
 *  @param[in] s           Pointer to device.
 *  @param[in] dec_factor  Requested decimation factor value from 2 to 256.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_configure_fifo_data_rate(inv_imu_device_t *s, FDR_CONFIG_FDR_SEL_t dec_factor);

/** @brief Return driver version x.y.z-suffix as a char array
 *  @return driver version a char array "x.y.z-suffix"
 */
const char *inv_imu_get_version(void);

/** @brief Converts two bytes in one unsigned half-word depending on endianness 
 *  @param[in] endianness  IMU's endianness.
 *  @param[in]  in         Pointer to input.
 *  @param[out] out        Pointer to output.
 */
static inline void format_u16_data(uint8_t endianness, const uint8_t *in, uint16_t *out)
{
	*out = (uint16_t)(endianness == INTF_CONFIG0_DATA_BIG_ENDIAN ? (in[0] << 8) | in[1] :
                                                                   (in[1] << 8) | in[0]);
}

/** @brief Converts two bytes in one signed half-word depending on endianness 
 *  @param[in] endianness  IMU's endianness.
 *  @param[in]  in         Pointer to input.
 *  @param[out] out        Pointer to output.
 */
static inline void format_s16_data(uint8_t endianness, const uint8_t *in, int16_t *out)
{
	*out = (int16_t)(endianness == INTF_CONFIG0_DATA_BIG_ENDIAN ? (in[0] << 8) | in[1] :
                                                                  (in[1] << 8) | in[0]);
}

#if INV_IMU_HFSR_SUPPORTED
/** @brief  Write 'size' bytes pointed by 'data' in SRAM at offset given in parameters.
 *  @param[in] data    pointer to data to be written in SRAM
 *  @param[in] offset  offset in bytes from SRAM start address where data should be written
 *  @param[in] size    number of bytes to write
 *  @return            0 in case of success, negative value on error.
 */
int inv_imu_write_sram(struct inv_imu_device *s, const uint8_t *data, uint32_t offset,
                       uint32_t size);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_H_ */

/** @} */
