/* SPIFFS Image Generation on Build Example
 *
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense or CC0-1.0
 */

#include "esp_event.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/param.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"

#include "bn_mul.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_netif.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "esp_http_server.h"

#include <dirent.h>

#include "driver/i2c.h"


static const char *TAG = "SAW_Project";

///////////////////////////// KOMUNIKACJA I2C///////////////////////////////////////

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
#define AIN0                        0b01000000                  //Zadamy sczyt pomiaru z pinu AIN0
#define AIN1                        0b01010000
#define AIN2                        0b01100000
#define AIN3                        0b01110000

//Config register Example
#define CONF_1_1                    0b10010000                  //first 7-bit I2c address followed by a low read/write bit
#define CONF_1_2                    0b00000001                  //Points to Config register
#define CONF_1_3                    0b10000010                  //MSB of the Config register to be written
#define CONF_1_4                    0b10000011                  //LSB of the Config register to be written

uint8_t data_1_AIN0[3]={CONF_1_2, CONF_1_3|AIN0, CONF_1_4};
uint8_t data_1_AIN1[3]={CONF_1_2, CONF_1_3|AIN1, CONF_1_4};
uint8_t data_1_AIN2[3]={CONF_1_2, CONF_1_3|AIN2, CONF_1_4};
uint8_t data_1_AIN3[3]={CONF_1_2, CONF_1_3|AIN3, CONF_1_4};


uint8_t data_ADC[2];
//Write to Pointer register
#define CONF_2_1                    0b10010000                  //first 7-bit I2c address followed by a low read/write bit
#define CONF_2_2                    0b00000000                  //points to Conversion regitster

uint8_t data_2[1]={CONF_2_2};

//Read Conversion register
#define CONF_3_1                    0b10010001                  //first 7-bit I2c address followed by a low read/write bit


#define SCALE                       4.096/32768

#define I2C_MASTER_WRITE            0
#define I2C_MASTER_READ             1


char * Read_ADC_V(int pin)
{
    bool state;
    float Voltage=0;
    char * buffer=(char *)malloc(sizeof(char)*8);
    
    switch(pin)
    {
        case 1:
            state=i2c_master_write_to_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_1_AIN0,sizeof(data_1_AIN0),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));
        break;
        case 2:
            state=i2c_master_write_to_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_1_AIN1,sizeof(data_1_AIN0),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));
        break;
        case 3:
            state=i2c_master_write_to_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_1_AIN2,sizeof(data_1_AIN0),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));
        break;
        default:
        return 0;
    }

    state=i2c_master_write_to_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_2,sizeof(data_2),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));
    state=i2c_master_read_from_device(I2C_MASTER_NUM,I2C_SLAVE_ADDRESS,data_ADC,sizeof(data_ADC),(I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS));

    if(state==0)
        {
        for(int i=0;i<2;i++)
        {
            ESP_LOGI(TAG, "Message%d: dec:%X  hex:%d",i, data_ADC[i], data_ADC[i]);
        }
        
        Voltage =(data_ADC[0]*256+data_ADC[1])*SCALE;
        ESP_LOGI(TAG, "Voltage: %f",Voltage);
        
        if(snprintf(buffer, 8, "%f", Voltage))
        return buffer;

        else
        {
            ESP_LOGI(TAG,"Problem z snprintf");
            return 0;
        }
    }
    return 0;
}

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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////SPIFFS///////////////////////////////////


static void read_hello_txt(void)
{
    ESP_LOGI(TAG, "Reading index.html");

    // Open for reading hello.txt
    FILE* f = fopen("/web_server_V2/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open hello.txt");
        return;
    }

    char buf[64];
    memset(buf, 0, sizeof(buf));
    fread(buf, 1, sizeof(buf), f);
    fclose(f);

    // Display the read contents from the file
    ESP_LOGI(TAG, "Read from hello.txt: %s", buf);
}

/* Function to initialize SPIFFS */
esp_err_t mount_html_page(const char * base_path)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = base_path,
        .partition_label = NULL,
        .max_files = 5,   // This sets the maximum number of files that can be open at the same time
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////HTTP_SERVER_MANAGER///////////////////////////////////////
/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
};


//* Copies the full path into destination buffer and returns
// * pointer to path (skipping the preceding base path) */
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

int which_file(const char * str)
{
    if(!strcmp(str,"/L1V"))
        return 1;
    if(!strcmp(str,"/L2V"))
        return 2;
    if(!strcmp(str,"/L3V"))
        return 3;
    if(!strcmp(str,"/L1C"))
        return 4;
    if(!strcmp(str,"/L2C"))
        return 5;
    if(!strcmp(str,"/L3C"))
        return 6;
    if(!strcmp(str,"/Fi"))
        return 7;
    
    return 0;
}

/* Handler to download a file kept on the server */
static esp_err_t file_root_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    FILE *fd = NULL;
  

    const char *filename = get_path_from_uri(filepath, ((struct file_server_data *)req->user_ctx)->base_path,
                                             req->uri, sizeof(filepath));

    
    ESP_LOGI(TAG,"sciezka do pliku: %s", filepath);
    
    if (!filename) {
        ESP_LOGE(TAG, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    switch(which_file(filename))
    {
        //L1V
        case 1:
            httpd_resp_send(req,Read_ADC_V(1), 8*sizeof(char));
            return ESP_OK;
        break;
        //L1C
        case 2:
            httpd_resp_send(req,Read_ADC_V(2), 8*sizeof(char));
            return ESP_OK;
        break;
        //L2V
        case 3:
            httpd_resp_send(req,Read_ADC_V(3), 8*sizeof(char));
            return ESP_OK;
        break;
        //L2C
        case 4:
            httpd_resp_send(req,"69", 2);
            return ESP_OK;
        break;
        //L3V
        case 5:
            httpd_resp_send(req,"69", 2);
            return ESP_OK;
        break;
        //L3C
        case 6:
            httpd_resp_send(req,"69", 2);
            return ESP_OK;
        break;
        //Fi
        case 7:
            httpd_resp_send(req,"69", 2);
            return ESP_OK;
        break;

        default:
        break;
    }

    fd = fopen("/web_server_V2/index.html", "r");
        if (!fd) {
            ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
            return ESP_FAIL;
        }
    
        ESP_LOGI(TAG, "Sending file : %s", filename);
        httpd_resp_set_type(req, "text/html");

        /* Retrieve the pointer to scratch buffer for temporary storage */
        char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
        size_t chunksize;
        do {
            /* Read file in chunks into the scratch buffer */
            chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

            if (chunksize > 0) {
                /* Send the buffer contents as HTTP response chunk */
                if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                    fclose(fd);
                    ESP_LOGE(TAG, "File sending failed!");
                    /* Abort sending file */
                    httpd_resp_sendstr_chunk(req, NULL);
                    /* Respond with 500 Internal Server Error */
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
            }

            /* Keep looping till the whole file is sent */
        } while (chunksize != 0);

        /* Close file after sending complete */
        fclose(fd);
        ESP_LOGI(TAG, "File sending complete");

        /* Respond with an empty chunk to signal HTTP response completion */
    #ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
        httpd_resp_set_hdr(req, "Connection", "close");
    #endif
        httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

/* Function to start the file server */
esp_err_t example_start_file_server(const char *base_path)
{
    static struct file_server_data *server_data = NULL;

    if (server_data) {
        ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate memory for server data */
    server_data = calloc(1, sizeof(struct file_server_data));
    if (!server_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path,
            sizeof(server_data->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }

    /* URI handler do roota */
    httpd_uri_t file_root = {
        .uri       = "/*",  
        .method    = HTTP_GET,
        .handler   = file_root_handler,
        .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_root);

    return ESP_OK;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////



void app_main(void)
{

    ESP_LOGI(TAG, "Starting example");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    const char * base_path="/web_server_V2";

    ESP_ERROR_CHECK(mount_html_page(base_path));

    read_hello_txt();

    ESP_ERROR_CHECK(example_connect());

    // Start the file server 
    ESP_ERROR_CHECK(example_start_file_server(base_path));
    ESP_LOGI(TAG, "File server started");

    while(1)
    {
            vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}
