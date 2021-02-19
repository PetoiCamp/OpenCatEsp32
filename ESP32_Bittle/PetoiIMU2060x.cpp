#include "PetoiIMU2060x.h"

PetoiICM20600::PetoiICM20600(){

     // Nyboard V2 default 20600 I2C ADDR = 0x69
    _addr = ICM20600_I2C_ADDR2;        
}

void PetoiICM20600::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer){
        
  Wire.beginTransmission(driver_Addr);
  Wire.write(start_Addr);  
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(driver_Addr,number_Bytes);
  
  //! Put read results in the Rx buffer
  while (Wire.available()) {
    read_Buffer[i++] = Wire.read();
  }        
}

void PetoiICM20600::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer){
  
  Wire.beginTransmission(driver_Addr);
  Wire.write(start_Addr);
  Wire.write(*write_Buffer);
  Wire.endTransmission();
}   

uint8_t PetoiICM20600::getDeviceID(){

  uint8_t regdata;

  I2C_Read_NBytes(_addr, ICM20600_WHO_AM_I, 1, &regdata);
  Serial.print("DeviceID = :");
  Serial.println(regdata, HEX);

  if(regdata!= 0x11){
    Serial.println("Error, please check connection and IMU address settings!");
    while(1);
  }

  return 0;
}

void PetoiICM20600::reset(){

    uint8_t regdata;
    I2C_Read_NBytes(_addr, ICM20600_USER_CTRL, 1, &regdata);

    regdata &= 0xfe;        //0b11111110

    regdata |= ICM20600_RESET_BIT;

    I2C_Write_NBytes(_addr, ICM20600_USER_CTRL, 1, &regdata);
}

void PetoiICM20600::initialize() {

    unsigned char regdata;

    // configuration
    regdata = 0x00;
    I2C_Write_NBytes(_addr, ICM20600_CONFIG, 1, &regdata);

    // disable fifo
    regdata = 0x00;
    I2C_Write_NBytes(_addr, ICM20600_FIFO_EN, 1, &regdata);

    // set default power mode
    PetoiICM20600::setPowerMode(ICM_6AXIS_LOW_POWER);

    // gyro config
    PetoiICM20600::setGyroScaleRange(RANGE_1K_DPS);
    PetoiICM20600::setGyroOutputDataRate(GYRO_RATE_1K_BW_176);
    PetoiICM20600::setGyroAverageSample(GYRO_AVERAGE_1);

    // accel config
    PetoiICM20600::setAccScaleRange(RANGE_4G);
    PetoiICM20600::setAccOutputDataRate(ACC_RATE_1K_BW_420);
    PetoiICM20600::setAccAverageSample(ACC_AVERAGE_4);
}

void PetoiICM20600::setPowerMode(icm20600_power_type_t mode){

    unsigned char data_pwr1;
    unsigned char data_pwr2;
    unsigned char data_lp;

    I2C_Read_NBytes(_addr, ICM20600_PWR_MGMT_1, 1, &data_pwr1);
    // Clear config 
    data_pwr1 &= 0x8f;                  // 0b10001111

    I2C_Read_NBytes(_addr, ICM20600_GYRO_LP_MODE_CFG, 1, &data_lp);
    // When set to "1" low-power gyroscope mode is enabled. Default setting is 0
    data_lp &= 0x7f;               // 0b01111111

    switch (mode) {
        case ICM_SLEEP_MODE:
            data_pwr1 |= 0x40;          // set 0b01000000
            break;

        case ICM_STANDYBY_MODE:
            data_pwr1 |= 0x10;          // set 0b00010000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_ACC_LOW_POWER:
            data_pwr1 |= 0x20;          // set bit5 0b00100000
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_ACC_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_GYRO_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00000000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            data_lp |= 0x80;
            break;

        case ICM_GYRO_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_6AXIS_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00100000
            data_lp |= 0x80;
            break;

        case ICM_6AXIS_LOW_NOISE:
            data_pwr1 |= 0x00;
            break;

        default:
            break;
    }
    I2C_Write_NBytes(_addr, ICM20600_PWR_MGMT_1, 1, &data_pwr1);
    I2C_Write_NBytes(_addr, ICM20600_PWR_MGMT_2, 1, &data_pwr2);
    I2C_Write_NBytes(_addr, ICM20600_GYRO_LP_MODE_CFG, 1, &data_lp);
}  


void PetoiICM20600::setSampleRateDivier(uint8_t div){

    uint8_t regdata = div;
    I2C_Write_NBytes(_addr, ICM20600_SMPLRT_DIV, 1, &regdata);
}


void PetoiICM20600::setAccScaleRange(acc_scale_type_t range){
    
    /*
        Reg: ACCEL_CONFIG[4:3]
        00: +-250dps
        01: +-500dps
        10: +-1000dps
        11: +-2000dps
    */
    uint8_t regdata;
    I2C_Read_NBytes(_addr, ICM20600_ACCEL_CONFIG, 1, &regdata);

    // Clear ACCEL_CONFIG[4:3]
    regdata &= 0xe7;   // 0b11100111

    switch (range)
    {
        case RANGE_2G:
            regdata |= 0x00;        // or: 0bxxx00xxx
            _acc_scale = 4000;
            break;

        case RANGE_4G:
            regdata |= 0x08;        // or: 0bxxx01xxx
            _acc_scale = 8000;
            break;

        case RANGE_8G:
            regdata |= 0x10;        // or: 0bxxx10xxx
            _acc_scale = 160000;
            break;

        case RANGE_16G:
            regdata |= 0x18;        // or: 0bxxx11xxx
            _acc_scale = 32000;
            break;
        
        default:
            break;
        }

    I2C_Write_NBytes(_addr, ICM20600_ACCEL_CONFIG, 1, &regdata);
}

void PetoiICM20600::setAccAverageSample(acc_averaging_sample_type_t sample){

    /*
        Reg: ACCEL_CONFIG2[5:4]
        00: Average 4 samples
        01: Average 8 samples
        10: Average 16 samples
        11: Average 32 samples
    */
    uint8_t regdata = 0;
    I2C_Read_NBytes(_addr, ICM20600_ACCEL_CONFIG2, 1, &regdata);

    // Clear ACCEL_CONFIG2[5:4]
    regdata &= 0xcf;        // and 0b11001111

    switch (sample)
    {
        case ACC_AVERAGE_4:
            regdata |= 0x00;        // or 0bxx00xxxx
            break;

        case ACC_AVERAGE_8:
            regdata |= 0x10;        // or 0bxx01xxxx
            break;

        case ACC_AVERAGE_16:
            regdata |= 0x20;        // or 0bxx10xxxx
            break;

        case ACC_AVERAGE_32:
            regdata |= 0x30;        // or 0bxx11xxxx
            break;
    
    default:
        break;
    }

    I2C_Write_NBytes(_addr, ICM20600_ACCEL_CONFIG2, 1, &regdata);

}


void PetoiICM20600::setAccOutputDataRate(acc_lownoise_odr_type_t odr){

    /*
        Reg: ACCEL_CONFIG2[3:0](ACCEL_FCHOICE_B & A_DLPF_CFG)
        See ICM20600 Datasheet P36
    */

    // Read register 
    uint8_t regdata;
    I2C_Read_NBytes(_addr, ICM20600_ACCEL_CONFIG2, 1, &regdata);
    
    // Clear ACCEL_CONFIG2[3:0]
    regdata &= 0xf0;        //0b11110000

    // Set new value 
    switch (odr) {
        case ACC_RATE_4K_BW_1046:
            regdata |= 0x08;
            break;

        case ACC_RATE_1K_BW_420:
            regdata |= 0x07;
            break;

        case ACC_RATE_1K_BW_218:
            regdata |= 0x01;
            break;

        case ACC_RATE_1K_BW_99:
            regdata |= 0x02;
            break;

        case ACC_RATE_1K_BW_44:
            regdata |= 0x03;
            break;

        case ACC_RATE_1K_BW_21:
            regdata |= 0x04;
            break;

        case ACC_RATE_1K_BW_10:
            regdata |= 0x05;
            break;

        case ACC_RATE_1K_BW_5:
            regdata |= 0x06;
            break;

        default:
            break;
    }

    // Write to register
    I2C_Write_NBytes(_addr, ICM20600_ACCEL_CONFIG2, 1, &regdata);
}

void PetoiICM20600::setGyroScaleRange(gyro_scale_type_t range){
    
    /*
        Reg: GYRO_CONFIG[4:3], FS_SEL[1:0]
        00: +-250dps
        01: +-500dps
        10: +-1000dps
        11: +-2000dps
    */

    // Read register 
    uint8_t regdata;
    I2C_Read_NBytes(_addr, ICM20600_GYRO_CONFIG, 1, &regdata);

    // Clear FS_SEL[4:3] config
    regdata &= 0xe7;    // 0b11100111

    switch (range) {
        case RANGE_250_DPS:
            regdata |= 0x00;   // 0bxxx00xxx
            _gyro_scale = 500;
            break;

        case RANGE_500_DPS:
            regdata |= 0x08;   // 0bxxx00xxx
            _gyro_scale = 1000;
            break;

        case RANGE_1K_DPS:
            regdata |= 0x10;   // 0bxxx10xxx
            _gyro_scale = 2000;
            break;

        case RANGE_2K_DPS:
            regdata |= 0x18;   // 0bxxx11xxx
            _gyro_scale = 4000;
            break;

        default:
            break;
    }

    I2C_Write_NBytes(_addr, ICM20600_GYRO_CONFIG, 1, &regdata);
}

void PetoiICM20600::setGyroAverageSample(gyro_averaging_sample_type_t sample){

    /*
        Reg: LP_MODE_CFG[6:4] G_AVGCFG[2:0]
        ICM20600 Datasheet P37
    */

    // Read register 
    uint8_t regdata;
    I2C_Read_NBytes(_addr, ICM20600_GYRO_LP_MODE_CFG, 1, &regdata);

    // Clear G_AVGCFG[6:4] 
    regdata &= 0x8f;        //0b10001111

    switch (sample) {
        case GYRO_AVERAGE_1:
            regdata |= 0x00; // 0bx000xxxx
            break;

        case GYRO_AVERAGE_2:
            regdata |= 0x10; // 0bx001xxxx
            break;

        case GYRO_AVERAGE_4:
            regdata |= 0x20; // 0bx010xxxx
            break;

        case GYRO_AVERAGE_8:
            regdata |= 0x30; // 0bx011xxxx
            break;

        case GYRO_AVERAGE_16:
            regdata |= 0x40; // 0bx100xxxx
            break;

        case GYRO_AVERAGE_32:
            regdata |= 0x50; // 0bx101xxxx
            break;

        case GYRO_AVERAGE_64:
            regdata |= 0x60; // 0bx110xxxx
            break;

        case GYRO_AVERAGE_128:
            regdata |= 0x70; // 0bx111xxxx
            break;

        default:
            break;
    }

    I2C_Write_NBytes(_addr, ICM20600_GYRO_LP_MODE_CFG, 1, &regdata);

}
    
    
void PetoiICM20600::setGyroOutputDataRate(gyro_lownoise_odr_type_t odr){

    /*
        Reg: CONFIG 
        DLPF_CFG, FCHOICE[2:1]+DLPFCFG[0]
        Datasheet P35
    */

    // Read initial Register value
    uint8_t regdata;
    I2C_Read_NBytes(_addr, ICM20600_CONFIG, 1, &regdata);

    // initialize DLPF Config
    regdata &= 0xf8;       // DLPF_CFG[2:0] 0b11111000

    switch (odr) {
        case GYRO_RATE_8K_BW_3281:
            regdata |= 0x07;
            break;

        case GYRO_RATE_8K_BW_250:
            regdata |= 0x00;
            break;

        case GYRO_RATE_1K_BW_176:
            regdata |= 0x01;
            break;

        case GYRO_RATE_1K_BW_92:
            regdata |= 0x02;
            break;
            
        case GYRO_RATE_1K_BW_41:
            regdata |= 0x03;
            break;

        case GYRO_RATE_1K_BW_20:
            regdata |= 0x04;
            break;

        case GYRO_RATE_1K_BW_10:
            regdata |= 0x05;
            break;

        case GYRO_RATE_1K_BW_5:
            regdata |= 0x06;
            break;
    }    

    I2C_Write_NBytes(_addr, ICM20600_CONFIG, 1, &regdata);
}

void PetoiICM20600::getAcceleration(float* x, float* y, float* z){

    int32_t rawData;
    rawData = getRawAccelerationX();
    *x = (float)(rawData * _acc_scale) / 65536 / 100;   //  m/s^2

    rawData = getRawAccelerationY();
    *y = (float)(rawData * _acc_scale) / 65536 / 100;

    rawData = getRawAccelerationZ();
    *z = (float)(rawData * _acc_scale) / 65536 / 100;
}


int16_t PetoiICM20600::getAccelerationX(void){

    int32_t raw_data = getRawAccelerationX();
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data;
}

int16_t PetoiICM20600::getAccelerationY(void){

    int32_t raw_data = getRawAccelerationY();
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data;
}
    
int16_t PetoiICM20600::getAccelerationZ(void){

    int32_t raw_data = getRawAccelerationZ();
    raw_data = (raw_data * _acc_scale) >> 16;
    return (int16_t)raw_data;
}
    
int16_t PetoiICM20600::getRawAccelerationX(void){

    // Output register H & L
    uint8_t regdata[2];
    I2C_Read_NBytes(_addr, ICM20600_ACCEL_XOUT_H, 2, regdata);
    return ((uint16_t)regdata[0] << 8) + regdata[1];
}
    
int16_t PetoiICM20600::getRawAccelerationY(void){

    // Output register H & L
    uint8_t regdata[2];
    I2C_Read_NBytes(_addr, ICM20600_ACCEL_YOUT_H, 2, regdata);
    return ((uint16_t)regdata[0] << 8) + regdata[1];
}
    
int16_t PetoiICM20600::getRawAccelerationZ(void){

    // Output register H & L
    uint8_t regdata[2];
    I2C_Read_NBytes(_addr, ICM20600_ACCEL_ZOUT_H, 2, regdata);
    return ((uint16_t)regdata[0] << 8) + regdata[1];
}

void PetoiICM20600::getGyroscope(float* x, float* y, float* z){

    int32_t rawData;
    rawData = getRawGyroscopeX(); 
    *x = (float)(rawData * _gyro_scale) / 65536;

    rawData = getRawGyroscopeY(); 
    *y = (float)(rawData * _gyro_scale) / 65536;

    rawData = getRawGyroscopeZ(); 
    *z = (float)(rawData * _gyro_scale) / 65536;
}
    
int16_t PetoiICM20600::getGyroscopeX(void){

    int32_t raw_data = getRawGyroscopeX();
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}
    
int16_t PetoiICM20600::getGyroscopeY(void){

    int32_t raw_data = getRawGyroscopeY();
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}
    
int16_t PetoiICM20600::getGyroscopeZ(void){

    int32_t raw_data = getRawGyroscopeZ();
    raw_data = (raw_data * _gyro_scale) >> 16;
    return (int16_t)raw_data;
}
    
int16_t PetoiICM20600::getRawGyroscopeX(void){

    // Output register H & L
    uint8_t regdata[2];
    I2C_Read_NBytes(_addr, ICM20600_GYRO_XOUT_H, 2, regdata);
    return ((uint16_t)regdata[0] << 8) + regdata[1];
}

int16_t PetoiICM20600::getRawGyroscopeY(void){

    // Output register H & L
    uint8_t regdata[2];
    I2C_Read_NBytes(_addr, ICM20600_GYRO_YOUT_H, 2, regdata);
    return ((uint16_t)regdata[0] << 8) + regdata[1];
}
    
int16_t PetoiICM20600::getRawGyroscopeZ(void){

    // Output register H & L
    uint8_t regdata[2];
    I2C_Read_NBytes(_addr, ICM20600_GYRO_ZOUT_H, 2, regdata);
    return ((uint16_t)regdata[0] << 8) + regdata[1];
}

int16_t PetoiICM20600::getTemperature(void){

    // Output register H & L
    uint8_t regdata[2];
    uint16_t tempRawData;
    I2C_Read_NBytes(_addr, ICM20600_TEMP_OUT_H, 2, regdata);
    tempRawData = ((uint16_t)regdata[0] << 8) + regdata[1];
    return (uint16_t)(tempRawData / 327 + 25);
}
