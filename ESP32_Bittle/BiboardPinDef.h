/*
    Nyboard V2_3 ESP32 Pin definition file

*/

/* 
    Serial:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    Serial-TX       GPIO1           1                       Fixed - CP2102
    Serial-RX       GPIO3           3                       Fixed - CP2102  
    Serial2-TX      GPIO17          17                      GPIO / PWM
    Serial2-RX      GPIO16          16                      GPIO / PWM

    System default, nothing to declaration!
*/

/* 
    Analog in:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    Ain[0]            GPI35           35                      GPI 
    Ain[1]            GPI34           34                      GPI
    Ain[2]            GPI39           39                      GPI / SAR in (SVN)
    Ain[3]            GPI36           36                      GPI / SAR in (SVP)

*/


/* 
    PWM:
                        |--------------------------------
                        |    PWM[0]           PWM[6]    |    
                        |    PWM[1]           PWM[7]    |
                        |    PWM[2]           PWM[8]    | 
                        |-----------                    |                 
                        |           |                   |
                        |   ESP32   |  IMU         USB-C|~~~~Tail~~~~
                        |           |                   |
                        |-----------                    |
                        |    PWM[3]           PWM[9]    |
                        |    PWM[4]           PWM[10]   |
                        |    PWM[5]           PWM[11]   |
                        |-------------------------------|


    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    PWM[0]          GPIO18              4                   GPIO / Ain / Touch
    PWM[1]          GPIO5               5                   GPIO / VSPI SS
    PWM[2]          GPIO4               18                  GPIO / VSPI SCK 
    -----------------------------------------------------------------------------
    PWM[3]          GPIO32              32                  GPIO / Ain / Touch
    PWM[4]          GPIO33              33                  GPIO / Ain / Touch
    PWM[5]          GPIO19              19                  GPIO / VSPI MISO
    -----------------------------------------------------------------------------
    PWM[6]          GPIO2               2                   boot pin, DO NOT PUT HIGH WHEN BOOT! 
    PWM[7]          GPIO15              15                  GPIO / HSPI SS / Ain Touch
    PWM[8]          GPIO13              13                  built-in LED / GPIO / HSPI MOSI / Ain / Touch 
    -----------------------------------------------------------------------------
    PWM[9]          GPIO12              12                  GPIO / HSPI MISO / Ain / Touch
    PWM[10]         GPIO14              14                  GPIO / HSPI SCK / Ain / Touch
    PWM[11]         GPIO27              27                  GPIO / Ain / Touch

*/

const uint8_t PWM_pin[12] = {4, 5, 18, 32, 33, 19, 2, 15, 13, 12, 14, 27};

/*
    I2C:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    I2C-SCL         GPIO22              22                  Fixed - ICM20600 - Pulled
    I2C-SDA         GPIO21              21                  Fixed = ICM20600 - Pulled

    System default, nothing to declaration!

*/

/*
    Other Peripherals:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    IR_Remote       GPIO23              23                  Fixed - VS1838B IR 
    DAC_Out         GPIO25              25                  Fixed - PAM8302
    IMU_Int         GPIO26              26                  Fixed - MPU6050 Interrupt

    System default, nothing to declaration!
*/

const uint8_t IR_Remote_pin = 23;
const uint8_t DAC_Out_pin = 25;
const uint8_t IMU_Int = 26;
