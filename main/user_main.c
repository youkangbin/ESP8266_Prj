/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_system.h"
#include "esp_err.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include "inc/uni_io.h"

#include "inc/user_main.h"
#include "inc/BH1750.h"
#include "inc/SHT30.h"
#include "inc/BMP280.h"

/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/


#define GPIO_OUTPUT_PIN_SEL  (1ULL<<16) 


void gpio_init(void)
{

    gpio_config_t conf;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    conf.pull_down_en = 0;
    conf.pull_up_en = 0;
    gpio_config(&conf);
    gpio_set_level(GPIO_D0, 0);

}


void sensor_read(float *rh, float *temp, float *press)
{
    esp_err_t ret;
    ret = SHT30_read_temp(rh, temp);
    if(ret != ESP_OK) {
        vTaskDelay(100 / portTICK_RATE_MS);
        SHT30_read_temp(rh, temp);
    }

    ret = BMP280_data_get(press, NULL);
    if(ret != ESP_OK) {
        vTaskDelay(100 / portTICK_RATE_MS);
        BMP280_data_get(press, NULL);
    }

}

void LED_control(void *pvParameters)
{

    uint16_t lx;
    for(;;) {
        BH1750_read_data(&lx);
        if(lx < 20)
            gpio_set_level(GPIO_D0, 1);
        else
            gpio_set_level(GPIO_D0, 0);

        vTaskDelay(200/ portTICK_RATE_MS);     
    }

}
char p[] = "hellxxxxxx";
void app_main(void)
{
    float p, t, t30, rh;
    char str[] = "####.###";

    // uint32_t dax[4] = {0x3d, 0x2c, 0x7b, 0x9d};
    //gpio_init();
    
    
    esp_err_t ret;
    uint16_t count = 0;
    
    gpio_init();
    I2C_driver_init(I2C_NUM_0, IO_SDA, IO_SCL);
    
    ESP_ERROR_CHECK(BH1750_init());
    ESP_ERROR_CHECK(SHT30_init());
    ESP_ERROR_CHECK(BMP280_init());
    vTaskDelay(200 / portTICK_RATE_MS);
    xTaskCreate(LED_control, "LED Control", 512, NULL, 2, NULL);
    for(;;) {


        
        // sensor_read(&rh, &t, &p);
 

        // float_to_str(str, p);
        // printf("PRESS:%s ", str);

        // float_to_str(str, t);
        // printf("SHT30 Temp:%s ", str);

        // float_to_str(str, rh);
        // printf("SHT30 RH:%s", str);

        // printf("\n");
        // count++;

        printf("%d\n", sizeof(p));
        vTaskDelay(1500/portTICK_RATE_MS);

    };

    
}

/*

Pin	        Function	                    ESP-8266 Pin
TX	        TXD	                            TXD
RX	        RXD	                            RXD
A0	        Analog input, max 3.3V input    A0
D0	        IO	                            GPIO16
D1	        IO,SCL	                        GPIO5
D2	        IO,SDA                          GPIO4
D3	        IO,10k Pull-up	                GPIO0
D4	        IO,10k Pull-up, BUILTIN_LED	    GPIO2
D5	        IO, SCK	                        GPIO14
D6	        IO, MISO	                    GPIO12
D7	        IO, MOSI	                    GPIO13
D8	        IO, 10k Pull-down, SS	        GPIO15
G	        Ground	                        GND
5V	        5V	                            -
3V3	        3.3V	                        3.3V
RST	        Reset	                        RST

*/

