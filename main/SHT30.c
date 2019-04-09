#include "inc/uni_io.h"
#include "inc/SHT30.h"


#define SHT_ADDR 0x44





esp_err_t SHT30_init(void)
{

    int ret;

    uint8_t cmd_next = 0x30;
    ret = I2C_write(SHT_ADDR, 0x21, &cmd_next, 1);

    return ret;
    
}

esp_err_t SHT30_read_buffer(uint8_t *data)
{


    int ret;

    uint8_t cmd[2] = {0xe0, 0x00};
    ret = I2C_read(SHT_ADDR, cmd, 2, data, 6);

    return ret;

}


esp_err_t SHT30_read_temp(float *rh, float *t)
{

    esp_err_t ret;
    uint8_t data[6];
    ret = SHT30_read_buffer(data);
    data_convert(rh, t, data);
    return ret;

}

void data_convert(float *rh, float *temp, uint8_t *data)
{

    uint16_t St = data[0]<<8 | data[1];
    uint16_t Srh =  data[3]<<8 | data[4];
    *rh = ((float)Srh / 65535.0) * 100.0;
    *temp = ((float)St / 65535.0) * 175.0 - 45;

}


