#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"

#include "driver/i2c.h"
#include "driver/gpio.h"


#define I2C_MASTER_NUM I2C_NUMBER(0) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */

#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static esp_err_t two_wire_master_init(gpio_num_t sda, gpio_num_t scl);

static void two_wire_select_register(uint8_t dev, uint8_t reg);

static int8_t two_wire_read_bit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);

static int8_t two_wire_read_bits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);

static int8_t two_wire_read_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);

static int8_t two_wire_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);

static int8_t two_wire_read_word(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout);

static esp_err_t two_wire_write_bit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);

static esp_err_t two_wire_write_bits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

static esp_err_t two_wire_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

static esp_err_t two_wire_write_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
