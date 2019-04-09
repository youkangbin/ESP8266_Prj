#ifndef __BH1750_H__
#define __BH1750_H__

#define I2C_SCL 5
#define I2C_SDA 4


esp_err_t BH1750_init(void);

esp_err_t BH1750_read_data(uint16_t *lx);



#endif