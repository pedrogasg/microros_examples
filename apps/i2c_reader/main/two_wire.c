#include <rcl/rcl.h>

#include "freertos/FreeRTOS.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "two_wire.h"


static esp_err_t two_wire_master_init(gpio_num_t sda, gpio_num_t scl)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void two_wire_select_register(uint8_t dev, uint8_t reg)
{

	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev << 1) | WRITE_BIT, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

}

static int8_t two_wire_read_bit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout)
{
	uint8_t b;
    uint8_t count = two_wire_read_byte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}


static int8_t two_wire_read_bits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout)
{

    uint8_t count, b;
    if ((count = two_wire_read_byte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

static int8_t two_wire_read_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) 
{
    return two_wire_read_bytes(devAddr, regAddr, 1, data, timeout);
}

static int8_t two_wire_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout)
{
	i2c_cmd_handle_t cmd;
	two_wire_select_register(devAddr, regAddr);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | READ_BIT, ACK_CHECK_EN));

	if(length>1)
		ESP_ERROR_CHECK(i2c_master_read(cmd, data, length-1, ACK_VAL));

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+length-1, NACK_VAL));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS));

	i2c_cmd_link_delete(cmd);

	return length;
}

static int8_t two_wire_read_word(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout)
{
	uint8_t msb[2] = {0,0};
	readBytes(devAddr, regAddr, 2, msb);
	*data = (int16_t)((msb[0] << 8) | msb[1]);
	return 0;
}

static esp_err_t two_wire_write_bit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    two_wire_read_byte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return two_wire_write_byte(devAddr, regAddr, b);
}


static esp_err_t two_wire_write_bits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b = 0;
    if (two_wire_read_byte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); 
        data &= mask; 
        b &= ~(mask); 
        b |= data; 
        return two_wire_write_byte(devAddr, regAddr, b);
    } else {
        return ESP_FAIL;
    }
}

static esp_err_t two_wire_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | WRITE_BIT, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, ACK_CHECK_EN));
	
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS));

	i2c_cmd_link_delete(cmd);

	return true;
}

static esp_err_t two_wire_write_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	
    i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | WRITE_BIT, ACK_CHECK_EN));


	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_write(cmd, data, length-1, 0));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length-1], ACK_CHECK_EN));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS));

	i2c_cmd_link_delete(cmd);

	return ESP_OK;
}

