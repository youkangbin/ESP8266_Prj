#include "inc/uni_io.h"
#include "inc/BMP280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
//SDO to GND results in slave address 1110110 (0x76);
//to VDDIO results in slave address 1110111 (0x77),
//If CSB is connected to VDDIO, the I²C interface is active.

#define BMP_ADDR 0x76

//MP280 reg address 
#define CHIP_ID     0xD0
#define CTRL_MEAS   0xF4
#define RESET       0xE0

#define TEMP_LSB    0xFB
#define TEMP_MSB    0xFA

#define PRESS_LSB   0xF8
#define PRESS_MSB   0xF7


struct {

      uint16_t digT1;
      int16_t  digT2;
      int16_t  digT3;

      uint16_t digP1;
      int16_t  digP2;
      int16_t  digP3;
      int16_t  digP4;
      int16_t  digP5;
      int16_t  digP6;
      int16_t  digP7;
      int16_t  digP8;
      int16_t  digP9;

} BMP280CalibData;


static esp_err_t BMP280_read_two(uint8_t reg_address, uint8_t *data)
{
    esp_err_t ret;
    ret = I2C_read(BMP_ADDR, &reg_address, 1, data, 2);
    return ret;
}


esp_err_t BMP280_init(void)
{

    uint8_t reg_data = 0xff;
    uint8_t reset_cmd = 0xb6;
    memset(&BMP280CalibData, 0,sizeof(BMP280CalibData));

    I2C_write(BMP_ADDR, RESET, &reset_cmd, 1);
    I2C_write(BMP_ADDR, CTRL_MEAS, &reg_data, 1);

    ESP_ERROR_CHECK(BMP280_read_two(0x88, (uint8_t*)&BMP280CalibData.digT1));
    ESP_ERROR_CHECK(BMP280_read_two(0x8A, (uint8_t*)&BMP280CalibData.digT2));
    ESP_ERROR_CHECK(BMP280_read_two(0x8C, (uint8_t*)&BMP280CalibData.digT3));

    ESP_ERROR_CHECK(BMP280_read_two(0x8E, (uint8_t*)&BMP280CalibData.digP1));
    ESP_ERROR_CHECK(BMP280_read_two(0x90, (uint8_t*)&BMP280CalibData.digP2));
    ESP_ERROR_CHECK(BMP280_read_two(0x92, (uint8_t*)&BMP280CalibData.digP3));
    ESP_ERROR_CHECK(BMP280_read_two(0x94, (uint8_t*)&BMP280CalibData.digP4));
    ESP_ERROR_CHECK(BMP280_read_two(0x96, (uint8_t*)&BMP280CalibData.digP5));
    ESP_ERROR_CHECK(BMP280_read_two(0x98, (uint8_t*)&BMP280CalibData.digP6));
    ESP_ERROR_CHECK(BMP280_read_two(0x9A, (uint8_t*)&BMP280CalibData.digP7));
    ESP_ERROR_CHECK(BMP280_read_two(0x9C, (uint8_t*)&BMP280CalibData.digP8));
    ESP_ERROR_CHECK(BMP280_read_two(0x9E, (uint8_t*)&BMP280CalibData.digP9));


    return ESP_OK;

}



esp_err_t BMP280_data_get(float *press, float *temp) 
{
    esp_err_t ret;
    int32_t adc_T, adc_P, t_fine;
    int64_t var1, var2, p;

    ret = BMP280_read_data((uint32_t*)&adc_T, (uint32_t*)&adc_P);

    var1  = ((((adc_T>>3) - ((int32_t)BMP280CalibData.digT1 <<1))) *
        ((int32_t)BMP280CalibData.digT2)) >> 11;

    var2  = (((((adc_T>>4) - ((int32_t)BMP280CalibData.digT1)) *
            ((adc_T>>4) - ((int32_t)BMP280CalibData.digT1))) >> 12) *
        ((int32_t)BMP280CalibData.digT3)) >> 14;

    t_fine = var1 + var2;

    if(temp != NULL)
        *temp  = (float)((t_fine * 5 + 128) >> 8) / 100;


    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)BMP280CalibData.digP6;
    var2 = var2 + ((var1*(int64_t)BMP280CalibData.digP5)<<17);
    var2 = var2 + (((int64_t)BMP280CalibData.digP4)<<35);
    var1 = ((var1 * var1 * (int64_t)BMP280CalibData.digP3)>>8) +
        ((var1 * (int64_t)BMP280CalibData.digP2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)BMP280CalibData.digP1)>>33;

    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)BMP280CalibData.digP9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)BMP280CalibData.digP8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)BMP280CalibData.digP7)<<4);
    if(press != NULL)
        *press =  ((float)p / 256) / 100;

    return ret;

}

esp_err_t BMP280_read_data(uint32_t *temp, uint32_t *press)
{
    esp_err_t ret;
    uint8_t temp_addr = TEMP_MSB;
    uint8_t press_addr = PRESS_MSB;
    uint8_t data[3];
    //data[0] msb data[1] lsb data[2] xlsb
    
    ret = I2C_read(BMP_ADDR, &temp_addr, 1, data, 3);

    *temp = (uint32_t)(((uint32_t)data[0] << 12)|((uint32_t)data[1] << 4)|((uint32_t)data[2] >> 4));

    if(ret != ESP_OK)
        return ret;

    ret = I2C_read(BMP_ADDR, &press_addr, 1, data, 3);

    *press = (uint32_t)(((uint32_t)data[0] << 12)|((uint32_t)data[1] << 4)|((uint32_t)data[2] >> 4));

    return ret;

}

//0xd0 , 0x58
esp_err_t BMP280_check(uint8_t *data)
{

    int ret;
    uint8_t reg = CHIP_ID;
    ret = I2C_read(BMP_ADDR, &reg, 1, data, 1);
    return ret;

}


