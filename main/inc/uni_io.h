#ifndef __UNI_IO__
#define __UNI_IO__



#include "driver/i2c.h"
#include "driver/spi.h"

//GPIO define


#define GPIO_D0     16
#define GPIO_D1     5       //SCL
#define GPIO_D2     4       //SDA
#define GPIO_D3     0
#define GPIO_D4     2
#define GPIO_D5     14      //IO, SCK
#define GPIO_D6     12      //IO, MISO
#define GPIO_D7     13      //IO, MOSI
#define GPIO_D8     15

#define IO_MOSI     13
#define IO_MISO     12
#define IO_CLK      14


//I2C define

#define IO_SDA 4
#define IO_SCL 5


#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

#define WRITE_BIT   I2C_MASTER_WRITE
#define READ_BIT    I2C_MASTER_READ  

#define I2C_PORT I2C_NUM_0





esp_err_t I2C_write(uint8_t slave_address, uint8_t reg_address, uint8_t *data, uint8_t data_len);


esp_err_t I2C_read(uint8_t slave_address, uint8_t *reg_address, uint8_t reg_len, uint8_t *data, uint8_t data_len);


esp_err_t I2C_driver_init(i2c_port_t I2C_NUM, gpio_num_t SDA_IO, gpio_num_t SCL_IO);

void float_to_str(char *str, double num);


#endif

