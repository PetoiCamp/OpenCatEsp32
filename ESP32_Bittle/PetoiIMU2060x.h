#ifndef _PETOIIMU2060X_H
#define PETOIIMU2060X_H

#include <Wire.h>
#include <Arduino.h>

/***************************************************************
    ICM20600 I2C Register
 ***************************************************************/
#define ICM20600_XG_OFFS_TC_H           0x04
#define ICM20600_XG_OFFS_TC_L           0x05
#define ICM20600_YG_OFFS_TC_H           0x07
#define ICM20600_YG_OFFS_TC_L           0x08
#define ICM20600_ZG_OFFS_TC_H           0x0a
#define ICM20600_ZG_OFFS_TC_L           0x0b
#define ICM20600_SELF_TEST_X_ACCEL      0x0d
#define ICM20600_SELF_TEST_Y_ACCEL      0x0e
#define ICM20600_SELF_TEST_Z_ACCEL      0x0f
#define ICM20600_XG_OFFS_USRH           0x13
#define ICM20600_XG_OFFS_USRL           0x14
#define ICM20600_YG_OFFS_USRH           0x15
#define ICM20600_YG_OFFS_USRL           0x16
#define ICM20600_ZG_OFFS_USRH           0x17
#define ICM20600_ZG_OFFS_USRL           0x18
#define ICM20600_SMPLRT_DIV             0x19
#define ICM20600_CONFIG                 0x1a
#define ICM20600_GYRO_CONFIG            0x1b
#define ICM20600_ACCEL_CONFIG           0x1c
#define ICM20600_ACCEL_CONFIG2          0x1d
#define ICM20600_GYRO_LP_MODE_CFG       0x1e
#define ICM20600_ACCEL_WOM_X_THR        0x20
#define ICM20600_ACCEL_WOM_Y_THR        0x21
#define ICM20600_ACCEL_WOM_Z_THR        0x22
#define ICM20600_FIFO_EN                0x23
#define ICM20600_FSYNC_INT              0x36
#define ICM20600_INT_PIN_CFG            0x37
#define ICM20600_INT_ENABLE             0x38
#define ICM20600_FIFO_WM_INT_STATUS     0x39
#define ICM20600_INT_STATUS             0x3a
#define ICM20600_ACCEL_XOUT_H           0x3b
#define ICM20600_ACCEL_XOUT_L           0x3c
#define ICM20600_ACCEL_YOUT_H           0x3d
#define ICM20600_ACCEL_YOUT_L           0x3e
#define ICM20600_ACCEL_ZOUT_H           0x3f
#define ICM20600_ACCEL_ZOUT_L           0x40
#define ICM20600_TEMP_OUT_H             0x41
#define ICM20600_TEMP_OUT_L             0x42
#define ICM20600_GYRO_XOUT_H            0x43
#define ICM20600_GYRO_XOUT_L            0x44
#define ICM20600_GYRO_YOUT_H            0x45
#define ICM20600_GYRO_YOUT_L            0x46
#define ICM20600_GYRO_ZOUT_H            0x47
#define ICM20600_GYRO_ZOUT_L            0x48
#define ICM20600_SELF_TEST_X_GYRO       0x50
#define ICM20600_SELF_TEST_Y_GYRO       0x51
#define ICM20600_SELF_TEST_Z_GYRO       0x52
#define ICM20600_FIFO_WM_TH1            0x60
#define ICM20600_FIFO_WM_TH2            0x61
#define ICM20600_SIGNAL_PATH_RESET      0x68
#define ICM20600_ACCEL_INTEL_CTRL       0x69
#define ICM20600_USER_CTRL              0x6A
#define ICM20600_PWR_MGMT_1             0x6b
#define ICM20600_PWR_MGMT_2             0x6c
#define ICM20600_I2C_IF                 0x70
#define ICM20600_FIFO_COUNTH            0x72
#define ICM20600_FIFO_COUNTL            0x73
#define ICM20600_FIFO_R_W               0x74
#define ICM20600_WHO_AM_I               0x75
#define ICM20600_XA_OFFSET_H            0x77
#define ICM20600_XA_OFFSET_L            0x78
#define ICM20600_YA_OFFSET_H            0x7a
#define ICM20600_YA_OFFSET_L            0x7b
#define ICM20600_ZA_OFFSET_H            0x7d
#define ICM20600_ZA_OFFSET_L            0x7e

#define ICM20600_I2C_ADDR1              0x68
#define ICM20600_I2C_ADDR2              0x69

#define ICM20600_FIFO_EN_BIT            (1 << 6)
#define ICM20600_FIFO_RST_BIT           (1 << 2)
#define ICM20600_RESET_BIT              (1 << 0)
#define ICM20600_DEVICE_RESET_BIT       (1 << 7)

// Gyroscope scale range
enum gyro_scale_type_t {
    RANGE_250_DPS = 0,
    RANGE_500_DPS,
    RANGE_1K_DPS,
    RANGE_2K_DPS,
};

// Accelerometer scale range
enum acc_scale_type_t {
    RANGE_2G = 0,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G,
};

// Gyroscope output data rate
enum gyro_lownoise_odr_type_t {
    GYRO_RATE_8K_BW_3281 = 0,
    GYRO_RATE_8K_BW_250,
    GYRO_RATE_1K_BW_176,
    GYRO_RATE_1K_BW_92,
    GYRO_RATE_1K_BW_41,
    GYRO_RATE_1K_BW_20,
    GYRO_RATE_1K_BW_10,
    GYRO_RATE_1K_BW_5,
};

// Accelerometer output data rate
enum acc_lownoise_odr_type_t {
    ACC_RATE_4K_BW_1046 = 0,
    ACC_RATE_1K_BW_420,
    ACC_RATE_1K_BW_218,
    ACC_RATE_1K_BW_99,
    ACC_RATE_1K_BW_44,
    ACC_RATE_1K_BW_21,
    ACC_RATE_1K_BW_10,
    ACC_RATE_1K_BW_5,
};

// Averaging filter settings for Low Power Accelerometer mode
enum acc_averaging_sample_type_t {
    ACC_AVERAGE_4 = 0,
    ACC_AVERAGE_8,
    ACC_AVERAGE_16,
    ACC_AVERAGE_32,
};

// Averaging filter configuration for low-power gyroscope mode
enum gyro_averaging_sample_type_t {
    GYRO_AVERAGE_1 = 0,
    GYRO_AVERAGE_2,
    GYRO_AVERAGE_4,
    GYRO_AVERAGE_8,
    GYRO_AVERAGE_16,
    GYRO_AVERAGE_32,
    GYRO_AVERAGE_64,
    GYRO_AVERAGE_128,
};

// ICM20600 power mode
enum icm20600_power_type_t {
    ICM_SLEEP_MODE = 0,
    ICM_STANDYBY_MODE,
    ICM_ACC_LOW_POWER,
    ICM_ACC_LOW_NOISE,
    ICM_GYRO_LOW_POWER,
    ICM_GYRO_LOW_NOISE,
    ICM_6AXIS_LOW_POWER,
    ICM_6AXIS_LOW_NOISE,
};

class PetoiICM20600 {
    
  public:
    PetoiICM20600();
    
    uint8_t getDeviceID();
    void initialize();
    void reset();

    // Set power mode, the default mode is ICM_6AXIS_LOW_POWER(set in initialize() function)
    void setPowerMode(icm20600_power_type_t mode);
    // Divides the internal sample rate to generate the sample rate that controls
    // sensor data output rate. Only works at mode: ICM_GYRO_LOW_POWER / ICM_ACC_LOW_POWER / ICM_ACC_LOW_NOISE
    // SAMPLE_RATE = 1KHz / (1 + div)
    void setSampleRateDivier(uint8_t div);

    void setAccScaleRange(acc_scale_type_t range);
    // Averaging filter settings for Low Power Accelerometer mode
    void setAccAverageSample(acc_averaging_sample_type_t sample);
    void setAccOutputDataRate(acc_lownoise_odr_type_t odr);

    void setGyroScaleRange(gyro_scale_type_t range);
    // Averaging filter configuration for low-power gyroscope mode
    void setGyroAverageSample(gyro_averaging_sample_type_t sample);
    void setGyroOutputDataRate(gyro_lownoise_odr_type_t odr);

    void getAcceleration(float* x, float* y, float* z);
    int16_t getAccelerationX(void);
    int16_t getAccelerationY(void);
    int16_t getAccelerationZ(void);
    int16_t getRawAccelerationX(void);
    int16_t getRawAccelerationY(void);
    int16_t getRawAccelerationZ(void);

    void getGyroscope(float* x, float* y, float* z);
    int16_t getGyroscopeX(void);
    int16_t getGyroscopeY(void);
    int16_t getGyroscopeZ(void);
    int16_t getRawGyroscopeX(void);
    int16_t getRawGyroscopeY(void);
    int16_t getRawGyroscopeZ(void);

    // Yes there is a digital-output temperature sensor in ICM20600,
    // return a integer centigrade degree
    int16_t getTemperature(void);

  private:
    uint8_t _addr;
    uint16_t _acc_scale, _gyro_scale;
    float _pitch, _roll;

    void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
    void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);

};



#endif
