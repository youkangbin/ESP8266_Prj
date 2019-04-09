#ifndef __SHT30_H__
#define __SHT30_H__


esp_err_t I2C_init(void);
esp_err_t SHT30_init(void);
esp_err_t SHT30_read_temp(float *rh, float *t);
esp_err_t SHT30_read_buffer(uint8_t *data);

void data_convert(float *rh, float *temp, uint8_t *data);



#endif