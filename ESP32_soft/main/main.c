#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "driver/i2c.h"

//Defined GPIO for I2C
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
//FREQUENCY 100kHz
#define I2C_MASTER_FREQ_HZ          400000
//???
#define I2C_MASTER_NUM              0                          /*!< I2C Master mode */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*size of slave receiver buffer?*/
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*size of slave rx_buffer?*/
#define INTR_ALLOC_FLAGS            0                          /*Flags used to allocate the interrupt*/

#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_SLAVE_ADDRESS           0x48                        /*IF ADDR pino of GY-ADS1115 is connected to ground*/
#define I2C_WHO_I_AM_REG            0x00

#define VoltPerBit                  187.5*0.000001

#define CONFIG_REGISTER             0b00000001                  //wybor rejestru z P[1:0]

//Config Register LOW BYTE
#define SamplePerSecond_128         0b01000000                  //Ustawianie Sample Per Second
//Config Register HIGH BYTE
#define _4096V_Input                0b00000001                  //Zakres napiecia mierzalnego
#define _2048V_Input                0b00000010
#define AIN0                        0b00100000                  //Zadamy sczyt pomiaru z pinu AIN0
#define AIN1                        0b00101000
#define AIN2                        0b00110000
#define AIN3                        0b00111000

//Config register Example
#define CONF_1_1                    0b10010000                  //first 7-bit I2c address followed by a low read/write bit
#define CONF_1_2                    0b00000001                  //Points to Config register
#define CONF_1_3                    0b10000100                  //MSB of the Config register to be written
#define CONF_1_4                    0b10000011                  //LSB of the Config register to be written

uint8_t data_1[3]={CONF_1_2, CONF_1_3, CONF_1_4};

//Write to Pointer register
#define CONF_2_1                    0b10010000                  //first 7-bit I2c address followed by a low read/write bit
#define CONF_2_2                    0b00000000                  //points to Conversion regitster

uint8_t data_2[1]={CONF_2_2};

//Read Conversion register
#define CONF_3_1                    0b10010001                  //first 7-bit I2c address followed by a low read/write bit


#define SCALE                       6.144/65535


const uint8_t data_reg[2]={SamplePerSecond_128,_4096V_Input|AIN0};
static const char *TAG = "i2c-testowa-komunikacja";

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, INTR_ALLOC_FLAGS);
}


void app_main(void)
{
    uint8_t size_data=2;
    uint8_t data[size_data];
    //char * pKoniec;
    bool state;
    float Voltage=0;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    state=i2c_master_write_to_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_1,sizeof(data_1),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));

    state=i2c_master_write_to_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_2,sizeof(data_2),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));
    
    state=i2c_master_read_from_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data,sizeof(data),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));

    if(state==0)
        {
        for(int i=0;i<size_data;i++)
        {
            ESP_LOGI(TAG, "Message%d: dec:%X  hex:%d",i, data[i], data[i]);
        }
        
        Voltage =(data[0]*256+data[1])*SCALE;
        ESP_LOGI(TAG, "Voltage: %f",Voltage);
    
    }

    while(1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
