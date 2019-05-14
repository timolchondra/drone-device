#ifndef __MULTICHANNELGASSENSOR_H__
#define __MULTICHANNELGASSENSOR_H__

#define DEFAULT_I2C_ADDR    0x04

#define ADDR_IS_SET             0           // if this is the first time to run, if 1126, set 
#define ADDR_FACTORY_ADC_NH3    2
#define ADDR_FACTORY_ADC_CO     4
#define ADDR_FACTORY_ADC_NO2    6

#define ADDR_USER_ADC_HN3       8
#define ADDR_USER_ADC_CO        10
#define ADDR_USER_ADC_NO2       12
#define ADDR_IF_CALI            14          // IF USER HAD CALI

#define ADDR_I2C_ADDRESS        20

#define CH_VALUE_NH3            1
#define CH_VALUE_CO             2
#define CH_VALUE_NO2            3

#define CMD_ADC_RES0            1           // NH3
#define CMD_ADC_RES1            2           // CO
#define CMD_ADC_RES2            3           // NO2
#define CMD_ADC_RESALL          4           // ALL CHANNEL
#define CMD_CHANGE_I2C          5           // CHANGE I2C
#define CMD_READ_EEPROM         6           // READ EEPROM VALUE, RETURN UNSIGNED INT
#define CMD_SET_R0_ADC          7           // SET R0 ADC VALUE
#define CMD_GET_R0_ADC          8           // GET R0 ADC VALUE
#define CMD_GET_R0_ADC_FACTORY  9           // GET FACTORY R0 ADC VALUE
#define CMD_CONTROL_LED         10
#define CMD_CONTROL_PWR         11


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_SPEED_STANDARD  100000
#define WRITE_BIT           I2C_MASTER_WRITE
#define READ_BIT            I2C_MASTER_READ
#define ACK_CHECK_EN        0x1
#define ACK_CHECK_DIS       0x0
#define ACK_VAL             0x0
#define NACK_VAL            0x1

#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0


#include <iostream>

enum{CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH};
esp_err_t i2c_master_read_slave(uint8_t i2cAddress, uint8_t *data_rd, size_t size);
void init_i2c();

class MultiChannelGasSensor{

private:

    int __version;
    unsigned char dta_test[20];

    unsigned int readChAdcValue(int ch);
    unsigned int adcValueR0_NH3_Buf;
    unsigned int adcValueR0_CO_Buf;
    unsigned int adcValueR0_NO2_Buf;

public:

    uint8_t i2cAddress;     //I2C address of this MCU
    uint16_t res0[3];       //sensors res0
    uint16_t res[3];        //sensors res
    bool r0_inited;


    inline unsigned int get_addr_dta(unsigned char addr_reg);
    inline unsigned int get_addr_dta(unsigned char addr_reg, unsigned char __dta);
    void write_i2c(unsigned char addr, unsigned char *dta, unsigned char dta_len);

    esp_err_t sendI2C(unsigned char dta);
    int16_t readData(uint8_t cmd);
    int16_t readR0(void);
    int16_t readR(void);
    float calcGas(int gas);

public:

    void begin(int address);
    void begin();
    void changeI2CAddr(uint8_t newAddr);
    void powerOn(void);
    void powerOff(void);
    void doCalibrate(void);

    //get gas concentration, unit: ppm
    float measure_CO(){return calcGas(CO);}
    float measure_NO2(){return calcGas(NO2);}
    float measure_NH3(){return calcGas(NH3);}
    float measure_C3H8(){return calcGas(C3H8);}
    float measure_C4H10(){return calcGas(C4H10);}
    float measure_CH4(){return calcGas(CH4);}
    float measure_H2(){return calcGas(H2);}
    float measure_C2H5OH(){return calcGas(C2H5OH);}

    float getR0(unsigned char ch);      // 0:CH3, 1:CO, 2:NO2
    float getRs(unsigned char ch);      // 0:CH3, 1:CO, 2:NO2

public:

    void ledOn()
    {
        dta_test[0] = CMD_CONTROL_LED;
        dta_test[1] = 1;
        write_i2c(i2cAddress, dta_test, 2);
    }

    void ledOff()
    {
        dta_test[0] = CMD_CONTROL_LED;
        dta_test[1] = 0;
        write_i2c(i2cAddress, dta_test, 2);
    }

   // void display_eeprom();
   // void factory_setting();
   //void change_i2c_address(unsigned char addr);
   unsigned char getVersion();

};

extern MultiChannelGasSensor gas;

#endif

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
