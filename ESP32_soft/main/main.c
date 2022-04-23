#include <stdio.h>

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

const uint8_t data_reg[3]={CONFIG_REGISTER,SamplePerSecond_128,_4096V_Input|AIN0};
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

static esp_err_t i2c_read_byte(const uint8_t reg_addr,size_t len_reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, I2C_SLAVE_ADDRESS, &reg_addr, len_reg, data, len, (I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));
}

void app_main(void)
{
    uint8_t data[20];
    uint8_t temp=0;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_read_byte(I2C_WHO_I_AM_REG,1,data,20));
    
    while(temp<20)
    {
        ESP_LOGI(TAG, "Message%d: %X",temp, data[temp]);
        temp++;
    }
    

    while(1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
