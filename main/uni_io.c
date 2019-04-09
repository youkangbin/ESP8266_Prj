#include "inc/uni_io.h"





esp_err_t I2C_write(uint8_t slave_address, uint8_t reg_address, uint8_t *data, uint8_t data_len)
{

    int ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_address<<1|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    if(data != NULL)
        i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;

}


esp_err_t I2C_read(uint8_t slave_address, uint8_t *reg_address, uint8_t reg_len, uint8_t *data, uint8_t data_len)
{

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_address << 1 | WRITE_BIT, ACK_CHECK_EN);
    if(reg_address != NULL)
        i2c_master_write(cmd, reg_address, reg_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_address << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;

}



esp_err_t I2C_driver_init(i2c_port_t I2C_NUM, gpio_num_t SDA_IO, gpio_num_t SCL_IO)
{

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = SCL_IO;
    conf.scl_pullup_en = 0;
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &conf));
    return ESP_OK;

}

//012.456
void float_to_str(char *str, double num)
{

    if(str[8] != '\0')
        return;
    str[0] = (int)num / 1000 + '0';
    str[1] = ((int)num / 100) %10 + '0';
    str[2] = ((int)num / 10) % 10 + '0';
    str[3] = (int)num % 10 + '0';
    num = (num - (int)num) * 1000;

    str[5] = (int)num / 100 + '0';
    str[6] = ((int)num / 10) % 10 + '0';
    str[7] = (int)num % 10 + '0';
    
}

