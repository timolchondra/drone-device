/* Timothy Olchondra
 * This code is meant to re write the existing library for the Seeed MultichannelGasSensor
 * for the ESP32 using ESP-IDF
 *
 *
 */

#include <math.h>
#include "MultiChannelGasSensor.h"
#include <iostream>
#include "esp_task.h"
#include "driver/i2c.h"

using namespace std;
/*Function name:            begin
 *Description:              intialize I2C
 */

void MultiChannelGasSensor::begin(int address) {
    __version = 1;
    r0_inited = false;
    init_I2C();
    i2cAddress = address;
    __version = getVersion(); 

}
unsigned char MultiChannelGasSensor::getVersion() {
    if(get_addr_dta(CMD_READ_EEPROM, ADDR_IS_SET) == 1126) {
        __version = 1;
        printf("version = 2\n");
        return 2;
    }

    __version = 1;
    printf("version = 1\n");
    return 1;

}

void init_I2C() {
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t) I2C_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t) I2C_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_SPEED_STANDARD;
  
    i2c_param_config((i2c_port_t) i2c_master_port, &conf);
  
    i2c_driver_install(i2c_master_port, conf.mode,
                    I2C_MASTER_RX_BUF_DISABLE,
                    I2C_MASTER_TX_BUF_DISABLE, 0);
}
void MultiChannelGasSensor::begin() {
    begin(DEFAULT_I2C_ADDR);
}
/* Function: sendI2C
 * Description: send one byte to I2C Wire
 *
 */

esp_err_t MultiChannelGasSensor::sendI2C(unsigned char dta) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEFAULT_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)dta, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;

}

unsigned int MultiChannelGasSensor::get_addr_dta(unsigned char addr_reg) {
    unsigned int dta = 0;
    unsigned char raw[10];

    sendI2C(addr_reg);

   //now request from the address
   //Read two bytes from the slave and store it into raw (some buffer with arbitrary size 10)
    esp_err_t ret = i2c_master_read_slave(raw, 2);
    if(ret != ESP_OK) {
        printf("I2C READ FAILED...%d\n", ret);
        return -1;
    }

   //now store those two bytes into an unsigned int, dta 
    dta = raw[0];
    dta <<= 8;
    dta += raw[1];

    switch(addr_reg) {
        case CH_VALUE_NH3:
            if(dta > 0) {
                adcValueR0_NH3_Buf = dta;
            } else {
                dta = adcValueR0_NH3_Buf;
            }
            break;
        case CH_VALUE_CO:
            if (dta > 0) {
                adcValueR0_CO_Buf = dta;
            } else {
                dta = adcValueR0_CO_Buf;
            }
            break;
        case CH_VALUE_NO2:
            if (dta > 0) {
                adcValueR0_NO2_Buf = dta;
            } else {
                dta = adcValueR0_NO2_Buf;
            }
            break;
        default:;

    }
    return dta;

}

unsigned int MultiChannelGasSensor::get_addr_dta(unsigned char addr_reg, unsigned char __dta) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEFAULT_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr_reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, __dta, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
   
    if (ret != ESP_OK) {
        printf("I2C Failed to read\n");
        return -1;
    }
    
    unsigned int dta = 0;
    unsigned char raw[10];

    ret = i2c_master_read_slave(raw, 2);

    if (ret != ESP_OK) {
        printf("I2C failed to read...\n");
        return -1;

    }

    dta = raw[0];
    dta <<= 8;
    dta += raw[1];

    return dta;
}
int16_t MultiChannelGasSensor::readData(uint8_t cmd) {
    uint16_t timeout = 0;
    uint8_t buffer[4];
    uint8_t checksum = 0;
    int16_t rtnData = 0;

    sendI2C(cmd);
    vTaskDelay(2/portTICK_RATE_MS);

    i2c_master_read_slave(buffer, 4);

    checksum = (uint8_t)(buffer[0] + buffer[1] + buffer[2]);
    if(checksum != buffer[3])
        return -4; //checksum wrong

    rtnData = ((buffer[1] << 8) + buffer[2]);

    return rtnData;

}

/* Function name: readR0
 * Descriptions: read R0 stored in slave MCU
 *
 */
int16_t MultiChannelGasSensor::readR0(void) {
    int16_t rtnData = 0;
    rtnData = readData(0x11);
    if(rtnData > 0)
        res0[0] = rtnData;
    else
        return rtnData;     //unsuccessful

    rtnData = readData(0x12);
    if(rtnData > 0)
        res0[1] = rtnData;
    else
        return rtnData;     //unsuccessful

    rtnData = readData(0x13);
    if(rtnData > 0)
        res0[2] = rtnData;
    else
        return rtnData;

    return 1; //successful

}

int16_t MultiChannelGasSensor::readR(void) {
    int16_t rtnData = 9;
    rtnData = readData(0x01);
    if(rtnData >= 0)
        res[0] = rtnData;
    else
        return rtnData; //unsuccessful

    rtnData = readData(0x02);
    if(rtnData >= 0)
        res[1] = rtnData;
    else
        return rtnData; //uncessful

    if(rtnData >= 0)
        res[2] = rtnData;
    else
        return rtnData; //uncessfull

    return 0; //successful

}
void MultiChannelGasSensor::write_i2c(unsigned char addr, unsigned char* dta, unsigned char dta_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, dta, dta_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

}
float MultiChannelGasSensor::calcGas(int gas) {
    float ratio0 = 1, ratio1 = 1, ratio2 = 1;
    if(1 == __version) {
        if(!r0_inited) {
            if(readR0() >= 0) r0_inited = true;
            else return -1.0f;

        }
        if(readR() < 0)
            return -2.0f;
        
        ratio0 = (float)res[0] / res0[0];
        ratio1 = (float)res[1] / res0[1];
        ratio2 = (float)res[2] / res0[2];

    } else if(2 == __version) {
        ledOn();
        int A0_0 = get_addr_dta(6, ADDR_USER_ADC_HN3);
        int A0_1 = get_addr_dta(6, ADDR_USER_ADC_CO);
        int A0_2 = get_addr_dta(6, ADDR_USER_ADC_NO2);

        int An_0 = get_addr_dta(CH_VALUE_NH3);
        int An_1 = get_addr_dta(CH_VALUE_CO);
        int An_2 = get_addr_dta(CH_VALUE_NO2);

        ratio0 = (float)An_0/(float)A0_0*(1023.0-A0_0)/(1023.0-An_0);
        ratio1 = (float)An_1/(float)A0_1*(1023.0-A0_1)/(1023.0-An_1);
        ratio2 = (float)An_2/(float)A0_2*(1023.0-A0_2)/(1023.0-An_2);

    }
    float c = 0;

    switch(gas)
    {
        case CO:
        {
            c = pow(ratio1, -1.179)*4.385;  //mod by jack
            break;
        }
        case NO2:
        {
            c = pow(ratio2, 1.007)/6.855;  //mod by jack
            break;
        }
        case NH3:
        {
            c = pow(ratio0, -1.67)/1.47;  //modi by jack
            break;
        }
        case C3H8:  //add by jack
        {
            c = pow(ratio0, -2.518)*570.164;
            break;
        }
        case C4H10:  //add by jack
        {
            c = pow(ratio0, -2.138)*398.107;
            break;
        }
        case CH4:  //add by jack
        {
            c = pow(ratio1, -4.363)*630.957;
            break;
        }
        case H2:  //add by jack
        {
            c = pow(ratio1, -1.8)*0.73;
            break;
        }
        case C2H5OH:  //add by jack
        {
            c = pow(ratio1, -1.552)*1.622;
            break;
        }
        default:
            break;
    }
    
    if(2==__version)ledOff();
    return isnan(c)?-3:c;
}

void MultiChannelGasSensor::changeI2CAddr(uint8_t newAddr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEFAULT_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, newAddr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


}
void MultiChannelGasSensor::doCalibrate(void) {
    if(1==__version) {
    START:
        sendI2C(0x22);
        if(readR0() > 0) {
            for(int i = 0; i < 3; i++) {
                printf("%d \t", res0[i]);

            }

        } else {
            vTaskDelay(5000/portTICK_RATE_MS);
            printf("continue...\n");
            for(int i = 0; i < 3; i++) {
                printf("%d \t", res0[i]);
            }
            printf("\n");
            goto START;

        }
    }
    else if(2 == __version) {
        unsigned int i, a0, a1, a2;
        while(1) {
            a0 = get_addr_dta(CH_VALUE_NH3);
            a1 = get_addr_dta(CH_VALUE_CO);
            a2 = get_addr_dta(CH_VALUE_NO2);
            
            printf("%d\t%d\t%d\t\n",a0,a1,a2);
            ledOn();
            int cnt = 0;
            for(i=0; i<20; i++) {
                if((a0 - get_addr_dta(CH_VALUE_NH3)) > 2 || (get_addr_dta(CH_VALUE_NH3) - a0) > 2)cnt++;
                if((a1 - get_addr_dta(CH_VALUE_CO)) > 2 || (get_addr_dta(CH_VALUE_CO) - a1) > 2)cnt++;
                if((a2 - get_addr_dta(CH_VALUE_NO2)) > 2 || (get_addr_dta(CH_VALUE_NO2) - a2) > 2)cnt++;
                
                if(cnt>5) {
                    break;
                }
                vTaskDelay(1000/portTICK_RATE_MS);
            }
                        
            ledOff();
            if(cnt <= 5)break;
            vTaskDelay(200/portTICK_RATE_MS);
        }
        
        printf("write user adc value: %d\t%d\t%d\t\n", a0,a1,a2);
        
        unsigned char tmp[7];
    
        tmp[0] = 7;

        tmp[1] = a0>>8;
        tmp[2] = a0&0xff;
           
        tmp[3] = a1>>8;
        tmp[4] = a1&0xff;

        tmp[5] = a2>>8;
        tmp[6] = a2&0xff;
            
        write_i2c(i2cAddress, tmp, 7);
    }
}

void MultiChannelGasSensor::powerOn() {
    if(__version == 1)
        sendI2C(0x21);

}
void MultiChannelGasSensor::powerOff() {
    if(__version == 1)
        sendI2C(0x20);
    else if(__version == 2) {
        dta_test[0] = 11;
        dta_test[1] = 0;
        write_i2c(DEFAULT_I2C_ADDR, dta_test, 2);

    }
}

esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEFAULT_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + (size - 1), (i2c_ack_type_t)NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
