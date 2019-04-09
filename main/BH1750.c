#include "inc/uni_io.h"
#include "inc/BH1750.h"

//ADDR to LOW 0x23 default
//ADDR to HIGH 0x5c
#define BH_ADDR 0x23







// static esp_err_t SHT30_write(uint8_t reg_address, uint8_t *data, size_t data_len)
// {

//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ADDR << 1, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
//     i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     return ret;
    
// }


esp_err_t BH1750_init(void)
{

    ESP_ERROR_CHECK( I2C_write(BH_ADDR, 0x1, NULL, 0));
    ESP_ERROR_CHECK( I2C_write(BH_ADDR, 0x10, NULL, 0));

    return ESP_OK;
}

esp_err_t BH1750_read_data(uint16_t *lx)
{
    int ret;
    uint8_t data[2];

    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, 2, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    *lx =(uint16_t)((float)((uint16_t)data[0] << 8 | data[1]) / 1.2); 

    return ret;
}





