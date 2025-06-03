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

#include "Arduino.h"
#include "ICM42670P.h"
#include "inv_imu_apex.h"

static int i2c_write(inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
static int i2c_read(inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
static int spi_write(inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
static int spi_read(inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
static void event_cb(inv_imu_sensor_event_t *event);

static const char *APEX_ACTIVITY[3] = {"IDLE", "WALK", "RUN"};

// i2c
#define I2C_DEFAULT_CLOCK 400000
#define I2C_MAX_CLOCK 1000000
#define ICM42670_I2C_ADDRESS 0x68
#define ARDUINO_I2C_BUFFER_LENGTH 32
// spi
#define SPI_READ 0x80
#define SPI_DEFAULT_CLOCK 6000000
#define SPI_MAX_CLOCK 24000000
// WOM threshold in mg
#define WOM_THRESHOLD 50 /* = 50 * 1000 / 256 = 195 mg */
// This is used by the event callback (not object aware), declared static
static inv_imu_sensor_event_t *event;

// ICM42670 constructor for I2c interface
ICM42670::ICM42670(TwoWire &i2c_ref, bool lsb, uint32_t freq)
{
  i2c = &i2c_ref;
  i2c_address = ICM42670_I2C_ADDRESS | (lsb ? 0x1 : 0);
  use_spi = false;
  spi = NULL;
  spi_cs = 0;
  if ((freq <= I2C_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  }
  else
  {
    clk_freq = I2C_DEFAULT_CLOCK;
  }
}

// ICM42670 constructor for i2c interface, default frequency
ICM42670::ICM42670(TwoWire &i2c_ref, bool lsb)
{
  i2c = &i2c_ref;
  i2c_address = ICM42670_I2C_ADDRESS | (lsb ? 0x1 : 0);
  use_spi = false;
  spi = NULL;
  spi_cs = 0;
  clk_freq = I2C_DEFAULT_CLOCK;
}

// ICM42670 constructor for spi interface
ICM42670::ICM42670(SPIClass &spi_ref, uint8_t cs_id, uint32_t freq)
{
  spi = &spi_ref;
  spi_cs = cs_id;
  use_spi = true;
  i2c = NULL;
  i2c_address = 0;
  if ((freq <= SPI_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  }
  else
  {
    clk_freq = SPI_DEFAULT_CLOCK;
  }
}

// ICM42670 constructor for spi interface, default frequency
ICM42670::ICM42670(SPIClass &spi_ref, uint8_t cs_id)
{
  spi = &spi_ref;
  spi_cs = cs_id;
  use_spi = true;
  i2c = NULL;
  i2c_address = 0;
  clk_freq = SPI_DEFAULT_CLOCK;
}

/* starts communication with the ICM42670 */
int ICM42670::begin()
{
  struct inv_imu_serif icm_serif;
  int rc = 0;
  uint8_t who_am_i;
  inv_imu_int1_pin_config_t int1_pin_config;

  if (i2c != NULL)
  {
    i2c->begin();
    i2c->setClock(clk_freq);
    icm_serif.serif_type = UI_I2C;
    icm_serif.read_reg = i2c_read;
    icm_serif.write_reg = i2c_write;
  }
  else
  {
    spi->begin();
    pinMode(spi_cs, OUTPUT);
    digitalWrite(spi_cs, HIGH);
    icm_serif.serif_type = UI_SPI4;
    icm_serif.read_reg = spi_read;
    icm_serif.write_reg = spi_write;
  }
  /* Initialize serial interface between MCU and Icm43xxx */
  icm_serif.context = (void *)this;
  icm_serif.max_read = 2560;  /* maximum number of bytes allowed per serial read */
  icm_serif.max_write = 2560; /* maximum number of bytes allowed per serial write */
  rc = inv_imu_init(&icm_driver, &icm_serif, NULL);
  if (rc != INV_ERROR_SUCCESS)
  {
    return rc;
  }
  icm_driver.sensor_event_cb = event_cb;
  int1_config = {
      .INV_UI_FSYNC = INV_IMU_DISABLE,
      .INV_UI_DRDY = INV_IMU_DISABLE,
      .INV_FIFO_THS = INV_IMU_DISABLE,
      .INV_FIFO_FULL = INV_IMU_DISABLE,
      .INV_SMD = INV_IMU_DISABLE,
      .INV_WOM_X = INV_IMU_DISABLE,
      .INV_WOM_Y = INV_IMU_DISABLE,
      .INV_WOM_Z = INV_IMU_DISABLE,
      .INV_FF = INV_IMU_DISABLE,
      .INV_LOWG = INV_IMU_DISABLE,
      .INV_STEP_DET = INV_IMU_DISABLE,
      .INV_STEP_CNT_OVFL = INV_IMU_DISABLE,
      .INV_TILT_DET = INV_IMU_DISABLE};

  /* Check WHOAMI */
  rc = inv_imu_get_who_am_i(&icm_driver, &who_am_i);
  if (rc != 0)
  {
    return -2;
  }
  if (who_am_i != INV_IMU_WHOAMI)
  {
    return -3;
  }

  /*
   * Configure interrupts pins
   * - Polarity High
   * - Pulse mode
   * - Push-Pull drive
   */
  int1_pin_config.int_polarity = INT_CONFIG_INT1_POLARITY_HIGH;
  int1_pin_config.int_mode = INT_CONFIG_INT1_MODE_PULSED;
  int1_pin_config.int_drive = INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
  inv_imu_set_pin_config_int1(&icm_driver, &int1_pin_config);

  /* All APEX off */
  apex_tilt_enable = false;
  apex_pedometer_enable = false;

  // successful init, return 0
  return 0;
}

int ICM42670::startAccel(uint16_t odr, uint16_t fsr)
{
  int rc = 0;
  rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  rc |= inv_imu_enable_accel_low_noise_mode(&icm_driver);
  return rc;
}

int ICM42670::startGyro(uint16_t odr, uint16_t fsr)
{
  int rc = 0;
  rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_enable_gyro_low_noise_mode(&icm_driver);
  return rc;
}

int ICM42670::getDataFromRegisters(inv_imu_sensor_event_t &evt)
{
  // Set event buffer to be used by the callback
  event = &evt;
  return inv_imu_get_data_from_registers(&icm_driver);
}

void ICM42670::enableInterrupt(uint8_t intpin, ICM42670_irq_handler handler)
{
  if (handler != NULL)
  {
    pinMode(intpin, INPUT);
    attachInterrupt(intpin, handler, RISING);
  }
}

int ICM42670::enableFifoInterrupt(uint8_t intpin, ICM42670_irq_handler handler, uint8_t fifo_watermark)
{
  int rc = 0;
  uint8_t data;

  if (handler == NULL)
  {
    return -1;
  }
  enableInterrupt(intpin, handler);
  rc |= inv_imu_configure_fifo(&icm_driver, INV_IMU_FIFO_ENABLED);
  // Configure interrupts sources
  int1_config.INV_FIFO_THS = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int1(&icm_driver, &int1_config);
  rc |= inv_imu_write_reg(&icm_driver, FIFO_CONFIG2, 1, &fifo_watermark);
  // Set fifo_wm_int_w generating condition : fifo_wm_int_w generated when counter == threshold
  rc |= inv_imu_read_reg(&icm_driver, FIFO_CONFIG5_MREG1, 1, &data);
  data &= (uint8_t)~FIFO_CONFIG5_WM_GT_TH_EN;
  rc |= inv_imu_write_reg(&icm_driver, FIFO_CONFIG5_MREG1, 1, &data);
  // Disable APEX to use 2.25kB of fifo for raw data
  data = SENSOR_CONFIG3_APEX_DISABLE_MASK;
  rc |= inv_imu_write_reg(&icm_driver, SENSOR_CONFIG3_MREG1, 1, &data);
  return rc;
}

int ICM42670::getDataFromFifo(ICM42670_sensor_event_cb event_cb)
{
  if (event_cb == NULL)
  {
    return -1;
  }
  icm_driver.sensor_event_cb = event_cb;
  return inv_imu_get_data_from_fifo(&icm_driver);
}

bool ICM42670::isAccelDataValid(inv_imu_sensor_event_t *evt)
{
  return (evt->sensor_mask & (1 << INV_SENSOR_ACCEL));
}

bool ICM42670::isGyroDataValid(inv_imu_sensor_event_t *evt)
{
  return (evt->sensor_mask & (1 << INV_SENSOR_GYRO));
}

int ICM42670::initApex(uint8_t intpin, ICM42670_irq_handler handler)
{
  int rc = 0;
  inv_imu_apex_parameters_t apex_inputs;

  /* Disabling FIFO usage to optimize power consumption */
  rc |= inv_imu_configure_fifo(&icm_driver, INV_IMU_FIFO_DISABLED);

  /* Enable accel in LP mode */
  rc |= inv_imu_enable_accel_low_power_mode(&icm_driver);

  /* Disable Pedometer before configuring it */
  rc |= inv_imu_apex_disable_pedometer(&icm_driver);
  rc |= inv_imu_apex_disable_tilt(&icm_driver);

  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ODR_50_HZ);
  rc |= inv_imu_apex_set_frequency(&icm_driver, APEX_CONFIG1_DMP_ODR_50Hz);

  /* Set APEX parameters */
  rc |= inv_imu_apex_init_parameters_struct(&icm_driver, &apex_inputs);
  apex_inputs.power_save = APEX_CONFIG0_DMP_POWER_SAVE_DIS;
  rc |= inv_imu_apex_configure_parameters(&icm_driver, &apex_inputs);

  rc |= inv_imu_set_config_int1(&icm_driver, &int1_config);
  enableInterrupt(intpin, handler);

  if (apex_tilt_enable)
    rc |= inv_imu_apex_enable_tilt(&icm_driver);
  if (apex_pedometer_enable)
    rc |= inv_imu_apex_enable_pedometer(&icm_driver);

  return rc;
}

int ICM42670::updateApex(void)
{
  int rc = 0;
  uint8_t data;
  rc = inv_imu_read_reg(&icm_driver, INT_STATUS3, 1, &data);
  if (rc == 0)
  {
    /* Update interrupt status */
    int_status3 |= data;
  }
  return rc;
}

int ICM42670::startTiltDetection(uint8_t intpin, ICM42670_irq_handler handler)
{
  int rc = 0;
  apex_tilt_enable = true;
  /* Configure interrupts sources */
  int1_config.INV_TILT_DET = INV_IMU_ENABLE;
  rc |= initApex(intpin, handler);
  return rc;
}

bool ICM42670::getTilt(void)
{
  updateApex();
  if (int_status3 & INT_STATUS3_TILT_DET_INT_MASK)
  {
    /* Reset tilt internal status */
    int_status3 &= ~INT_STATUS3_TILT_DET_INT_MASK;
    return true;
  }
  return false;
}

int ICM42670::startPedometer(uint8_t intpin, ICM42670_irq_handler handler)
{
  int rc = 0;
  step_cnt_ovflw = 0;
  apex_pedometer_enable = true;
  /* Configure interrupts sources */
  int1_config.INV_STEP_DET = INV_IMU_ENABLE;
  int1_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
  rc |= initApex(intpin, handler);
  return rc;
}

int ICM42670::getPedometer(uint32_t &step_count, float &step_cadence, const char *&activity)
{
  int rc = 0;

  /* Read APEX interrupt status */
  rc |= updateApex();

  if (int_status3 & INT_STATUS3_STEP_CNT_OVF_INT_MASK)
  {
    step_cnt_ovflw++;
    /* Reset pedometer overflow internal status */
    int_status3 &= ~INT_STATUS3_STEP_CNT_OVF_INT_MASK;
  }

  if (int_status3 & (INT_STATUS3_STEP_DET_INT_MASK))
  {
    inv_imu_apex_step_activity_t apex_data0;
    float nb_samples = 0;

    /* Reset pedometer internal status */
    int_status3 &= ~INT_STATUS3_STEP_DET_INT_MASK;

    rc |= inv_imu_apex_get_data_activity(&icm_driver, &apex_data0);
    // to do: detect step counter overflow?
    step_count = apex_data0.step_cnt + step_cnt_ovflw * (uint32_t)UINT16_MAX;
    /* Converting u6.2 to float */
    nb_samples = (apex_data0.step_cadence >> 2) +
                 (float)(apex_data0.step_cadence & 0x03) * 0.25f;
    if (nb_samples != 0)
    {
      step_cadence = (float)50 / nb_samples;
    }
    else
    {
      step_cadence = 0;
    }
    activity = APEX_ACTIVITY[apex_data0.activity_class];
  }
  else
  {
    return -11;
  }

  return rc;
}

int ICM42670::startWakeOnMotion(uint8_t intpin, ICM42670_irq_handler handler)
{
  int rc = 0;

  /* Configure interrupts sources */
  int1_config.INV_WOM_X = INV_IMU_ENABLE;
  int1_config.INV_WOM_Y = INV_IMU_ENABLE;
  int1_config.INV_WOM_Z = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int1(&icm_driver, &int1_config);

  /* Disabling FIFO usage to optimize power consumption */
  rc |= inv_imu_configure_fifo(&icm_driver, INV_IMU_FIFO_DISABLED);

  /* Disable Pedometer before configuring it */
  rc |= inv_imu_apex_disable_pedometer(&icm_driver);
  rc |= inv_imu_apex_disable_tilt(&icm_driver);

  apex_tilt_enable = false;
  apex_pedometer_enable = false;

  enableInterrupt(intpin, handler);

  /*
   * Optimize power consumption:
   * - Disable FIFO usage.
   * - Set 2X averaging.
   * - Use Low-Power mode at low frequency.
   */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_AVG_2);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ODR_12_5_HZ);
  rc |= inv_imu_enable_accel_low_power_mode(&icm_driver);

  /* Configure and enable WOM */
  rc |= inv_imu_configure_wom(&icm_driver, WOM_THRESHOLD, WOM_THRESHOLD, WOM_THRESHOLD,
                              WOM_CONFIG_WOM_INT_MODE_ORED, WOM_CONFIG_WOM_INT_DUR_1_SMPL);
  rc |= inv_imu_enable_wom(&icm_driver);
  return rc;
}

static int i2c_write(inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
  ICM42670 *obj = (ICM42670 *)serif->context;
  obj->i2c->beginTransmission(obj->i2c_address);
  obj->i2c->write(reg);
  for (uint8_t i = 0; i < wlen; i++)
  {
    obj->i2c->write(wbuffer[i]);
  }
  obj->i2c->endTransmission();
  return 0;
}

static int i2c_read(inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
  ICM42670 *obj = (ICM42670 *)serif->context;
  uint16_t offset = 0;

  obj->i2c->beginTransmission(obj->i2c_address);
  obj->i2c->write(reg);
  obj->i2c->endTransmission(false);
  while (offset < rlen)
  {
    uint16_t rx_bytes = 0;
    if (offset != 0)
      obj->i2c->beginTransmission(obj->i2c_address);
    uint16_t length = ((rlen - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (rlen - offset);
    rx_bytes = obj->i2c->requestFrom((uint8_t)obj->i2c_address, (uint8_t)length);
    if (rx_bytes == length)
    {
      for (uint8_t i = 0; i < length; i++)
      {
        rbuffer[offset + i] = obj->i2c->read();
      }
      offset += length;
      obj->i2c->endTransmission((offset == rlen));
    }
    else
    {
      obj->i2c->endTransmission((offset == rlen));
    }
  }
  if (offset == rlen)
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

static int spi_write(inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
  ICM42670 *obj = (ICM42670 *)serif->context;
  obj->spi->beginTransaction(SPISettings(obj->clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(obj->spi_cs, LOW);
  obj->spi->transfer(reg);
  for (uint8_t i = 0; i < wlen; i++)
  {
    obj->spi->transfer(wbuffer[i]);
  }
  digitalWrite(obj->spi_cs, HIGH);
  obj->spi->endTransaction();
  return 0;
}

static int spi_read(inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
  ICM42670 *obj = (ICM42670 *)serif->context;
  obj->spi->beginTransaction(SPISettings(obj->clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(obj->spi_cs, LOW);
  obj->spi->transfer(reg | SPI_READ);
  obj->spi->transfer(rbuffer, rlen);
  digitalWrite(obj->spi_cs, HIGH);
  obj->spi->endTransaction();
  return 0;
}

ACCEL_CONFIG0_FS_SEL_t ICM42670::accel_fsr_g_to_param(uint16_t accel_fsr_g)
{
  ACCEL_CONFIG0_FS_SEL_t ret = ACCEL_CONFIG0_FS_SEL_16g;

  switch (accel_fsr_g)
  {
  case 2:
    ret = ACCEL_CONFIG0_FS_SEL_2g;
    break;
  case 4:
    ret = ACCEL_CONFIG0_FS_SEL_4g;
    break;
  case 8:
    ret = ACCEL_CONFIG0_FS_SEL_8g;
    break;
  case 16:
    ret = ACCEL_CONFIG0_FS_SEL_16g;
    break;
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

GYRO_CONFIG0_FS_SEL_t ICM42670::gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps)
{
  GYRO_CONFIG0_FS_SEL_t ret = GYRO_CONFIG0_FS_SEL_2000dps;

  switch (gyro_fsr_dps)
  {
  case 250:
    ret = GYRO_CONFIG0_FS_SEL_250dps;
    break;
  case 500:
    ret = GYRO_CONFIG0_FS_SEL_500dps;
    break;
  case 1000:
    ret = GYRO_CONFIG0_FS_SEL_1000dps;
    break;
  case 2000:
    ret = GYRO_CONFIG0_FS_SEL_2000dps;
    break;
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

ACCEL_CONFIG0_ODR_t ICM42670::accel_freq_to_param(uint16_t accel_freq_hz)
{
  ACCEL_CONFIG0_ODR_t ret = ACCEL_CONFIG0_ODR_100_HZ;

  switch (accel_freq_hz)
  {
  case 12:
    ret = ACCEL_CONFIG0_ODR_12_5_HZ;
    break;
  case 25:
    ret = ACCEL_CONFIG0_ODR_25_HZ;
    break;
  case 50:
    ret = ACCEL_CONFIG0_ODR_50_HZ;
    break;
  case 100:
    ret = ACCEL_CONFIG0_ODR_100_HZ;
    break;
  case 200:
    ret = ACCEL_CONFIG0_ODR_200_HZ;
    break;
  case 400:
    ret = ACCEL_CONFIG0_ODR_400_HZ;
    break;
  case 800:
    ret = ACCEL_CONFIG0_ODR_800_HZ;
    break;
  case 1600:
    ret = ACCEL_CONFIG0_ODR_1600_HZ;
    break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

GYRO_CONFIG0_ODR_t ICM42670::gyro_freq_to_param(uint16_t gyro_freq_hz)
{
  GYRO_CONFIG0_ODR_t ret = GYRO_CONFIG0_ODR_100_HZ;

  switch (gyro_freq_hz)
  {
  case 12:
    ret = GYRO_CONFIG0_ODR_12_5_HZ;
    break;
  case 25:
    ret = GYRO_CONFIG0_ODR_25_HZ;
    break;
  case 50:
    ret = GYRO_CONFIG0_ODR_50_HZ;
    break;
  case 100:
    ret = GYRO_CONFIG0_ODR_100_HZ;
    break;
  case 200:
    ret = GYRO_CONFIG0_ODR_200_HZ;
    break;
  case 400:
    ret = GYRO_CONFIG0_ODR_400_HZ;
    break;
  case 800:
    ret = GYRO_CONFIG0_ODR_800_HZ;
    break;
  case 1600:
    ret = GYRO_CONFIG0_ODR_1600_HZ;
    break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}

static void event_cb(inv_imu_sensor_event_t *evt)
{
  memcpy(event, evt, sizeof(inv_imu_sensor_event_t));
}
