#ifndef __BMP280_H__
#define __BMP280_H__

//vcc gnd scl sda csb sdo

esp_err_t BMP280_init(void);

esp_err_t BMP280_read(uint8_t *data);

esp_err_t BMP280_check(uint8_t *data);

void print_cali(void);

float press_data_convert(int64_t *xg, int32_t adc_P, int32_t t_fine);  


esp_err_t BMP280_read_data(uint32_t *temp, uint32_t *press);

esp_err_t BMP280_data_get(float *press, float *temp);


#endif