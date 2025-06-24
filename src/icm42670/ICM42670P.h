/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
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
 
#ifndef ICM42670_H
#define ICM42670_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

extern "C" {
#include "inv_imu_driver.h"
#undef ICM42670
}

// This defines the handler called when retrieving a sample from the FIFO
typedef void (*ICM42670_sensor_event_cb)(inv_imu_sensor_event_t *event);
// This defines the handler called when receiving an irq
typedef void (*ICM42670_irq_handler)(void);

class ICM42670 {
  public:
    ICM42670(TwoWire &i2c,bool address_lsb);
    ICM42670(TwoWire &i2c,bool lsb, uint32_t freq);
    ICM42670(SPIClass &spi,uint8_t chip_select_id);
    ICM42670(SPIClass &spi,uint8_t cs_id, uint32_t freq);
    int begin();
    int startAccel(uint16_t odr, uint16_t fsr);
    int startGyro(uint16_t odr, uint16_t fsr);
    int getDataFromRegisters(inv_imu_sensor_event_t& evt);
    int enableFifoInterrupt(uint8_t intpin, ICM42670_irq_handler handler, uint8_t fifo_watermark);
    int getDataFromFifo(ICM42670_sensor_event_cb event_cb);
    bool isAccelDataValid(inv_imu_sensor_event_t *evt);
    bool isGyroDataValid(inv_imu_sensor_event_t *evt);
    int startTiltDetection(uint8_t intpin=2, ICM42670_irq_handler handler=NULL);
    bool getTilt(void);
    int startPedometer(uint8_t intpin=2, ICM42670_irq_handler handler=NULL);
    int getPedometer(uint32_t& step_count, float& step_cadence, const char*& activity);
    int startWakeOnMotion(uint8_t intpin, ICM42670_irq_handler handler);
    int updateApex(void);
    void enableInterrupt(uint8_t intpin, ICM42670_irq_handler handler);

    uint8_t i2c_address;
    TwoWire *i2c;
    uint8_t spi_cs;
    SPIClass *spi;
    uint32_t clk_freq;
    uint8_t int_status3;
  protected:
    struct inv_imu_device icm_driver;
    inv_imu_interrupt_parameter_t int1_config;
    bool use_spi;
    ACCEL_CONFIG0_ODR_t accel_freq_to_param(uint16_t accel_freq_hz);
    GYRO_CONFIG0_ODR_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    ACCEL_CONFIG0_FS_SEL_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);
    int initApex(uint8_t intpin, ICM42670_irq_handler handler);
    uint32_t step_cnt_ovflw;
    bool apex_tilt_enable;
    bool apex_pedometer_enable;
};

#endif // ICM42670_H
