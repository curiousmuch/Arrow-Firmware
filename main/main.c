/*
 * Project: Arrow
 * Author: 	curiousmuch
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// Includes for CC1120 Driver
#include "driver/spi_master.h"
#include "esp_err.h"
#include "cc1120.h"
#include "cc1120_protocol.h"

// CC1120 - ESP32 I/O
// NOTE: Logic Probe is connecting to RESET - Pin1
#define CC1120_RESET		22
#define CC1120_CS 			5
#define CC1120_SCLK			18
#define CC1120_MOSI			23
#define CC1120_MISO			19
#define CC1120_GPIO0		36
#define CC1120_GPIO0_RTC	0
#define CC1120_GPIO2		39
#define CC1120_GPIO2_RTC	3
#define CC1120_GPIO3 		34
#define CC1120_GPIO3_RTC	4

#define CC1120_WRITE_BIT 	0
#define CC1120_READ_BIT 	BIT(1)
#define CC1120_BURST_BIT 	BIT(0)

// Public Configurations for CC1120 SPI Driver
spi_bus_config_t bus_config =
{
	.miso_io_num = CC1120_MISO,
	.mosi_io_num = CC1120_MOSI,
	.sclk_io_num = CC1120_SCLK,
	.quadwp_io_num = -1,
	.quadhd_io_num = -1,
	.max_transfer_sz = 150
};

spi_device_interface_config_t interface_config =
{
	.command_bits = 2,
	.address_bits = 6,
	.dummy_bits = 0,
	.mode = 0,
	.spics_io_num = CC1120_CS,
	.clock_speed_hz = (APB_CLK_FREQ/20),
	.flags = 0,
	.queue_size = 1

};

spi_device_handle_t spi;

// Private CC1120 Driver Functions
void cc1120_gpio_init(void)
{
	gpio_config_t reset_pin_config =
	{
			.pin_bit_mask = (uint64_t)BIT64(CC1120_RESET),
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE

	};
	gpio_config_t gpio_pin_config =
	{
			.pin_bit_mask = (uint64_t) (BIT64(CC1120_GPIO0)|BIT64(CC1120_GPIO2)|BIT64(CC1120_GPIO3)),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
	};
	gpio_config(&reset_pin_config);
	gpio_config(&gpio_pin_config);

	gpio_set_level(CC1120_RESET, 1);

}

void cc1120_spi_init(void)
{
	esp_err_t ret;
	ret = spi_bus_initialize(VSPI_HOST, &bus_config, 1);	// this uses DMA channel 1
	ESP_ERROR_CHECK(ret);
	ret = spi_bus_add_device(VSPI_HOST, &interface_config, &spi);
	ESP_ERROR_CHECK(ret);
}

void cc1120_spi_write_byte(uint16_t addr, uint8_t data)
{
	esp_err_t ret;
	spi_transaction_t tx_trans =
	{
		.flags = SPI_TRANS_USE_TXDATA,
		.cmd = CC1120_WRITE_BIT,
		.addr = addr,
		.length = 8,
		.rxlength = 0,
		.tx_data[0] = data

	};

	if ((addr & 0xFF00) != 0) // send data with extended address in command field
	{
		tx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t tx_trans_ext =
		{
				.base = tx_trans,
				.command_bits = 2,
				.address_bits = 14
		};
		ret = spi_device_transmit(spi, (spi_transaction_t*)&tx_trans_ext);
	}
	else
	{
		ret = spi_device_transmit(spi, &tx_trans);
	}
	ESP_ERROR_CHECK(ret);
}

void cc1120_spi_write_bytes(uint16_t addr, uint8_t* data, uint8_t len)
{
	esp_err_t ret;
	spi_transaction_t tx_trans =
	{
		.cmd = (CC1120_WRITE_BIT | CC1120_BURST_BIT),
		.addr = addr,
		.length = 8*len,
		.tx_buffer = data
	};
	if ((addr & 0xFF00) != 0) // send data with extended address in command field
	{
		tx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t tx_trans_ext =
		{
				.base = tx_trans,
				.command_bits = 2,
				.address_bits = 14
		};
		ret = spi_device_transmit(spi, (spi_transaction_t*)&tx_trans_ext);
	}
	else
	{
		ret = spi_device_transmit(spi, &tx_trans);
	}
	ESP_ERROR_CHECK(ret);
}

void cc1120_spi_read_byte(uint16_t addr, uint8_t* data)
{
	esp_err_t ret;
	spi_transaction_t rx_trans =
	{
		.cmd = CC1120_READ_BIT,
		.addr = addr,
		.length = 8,
		.rxlength = 8,
		.rx_buffer = data
	};
	if ((addr & 0xFF00) != 0) // read data with extended address in command field
	{
		rx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t rx_trans_ext =
		{
				.base = rx_trans,
				.command_bits = 2,
				.address_bits = 14
		};
		ret = spi_device_transmit(spi, (spi_transaction_t*)&rx_trans_ext);
	}
	else
	{
		ret = spi_device_transmit(spi, &rx_trans);
	}
	ESP_ERROR_CHECK(ret);
}

void cc1120_spi_read_bytes(uint16_t addr, uint8_t* data, uint8_t len)
{
	esp_err_t ret;
	spi_transaction_t rx_trans =
	{
		.cmd = (CC1120_READ_BIT | CC1120_BURST_BIT),
		.addr = addr,
		.length = 8*len,
		.rxlength = 8*len,
		.rx_buffer = data
	};
	if ((addr & 0xFF00) != 0) // read data with extended address in command field
	{
		rx_trans.flags |= (SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR);
		spi_transaction_ext_t rx_trans_ext =
		{
				.base = rx_trans,
				.command_bits = 2,
				.address_bits = 14
		};
		ret = spi_device_transmit(spi, (spi_transaction_t*)&rx_trans_ext);
	}
	else
	{
		ret = spi_device_transmit(spi, &rx_trans);
	}
	ESP_ERROR_CHECK(ret);
}

rf_status_t cc1120_spi_strobe(uint8_t cmd)
{
	esp_err_t ret;
	uint8_t temp=0;
	spi_transaction_t rx_trans =
	{
		.flags = (SPI_TRANS_USE_TXDATA | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR),
		.length = 8,
		.rxlength = 8,
		.rx_buffer = &temp,
		.tx_data[0] = cmd
	};
	spi_transaction_ext_t rx_trans_ext =
	{
			.base = rx_trans,
			.command_bits = 0,
			.address_bits = 0
	};
	ret = spi_device_transmit(spi, (spi_transaction_t*)&rx_trans_ext);
	ESP_ERROR_CHECK(ret);
	return (temp & 0xF0);
}

// Public CC1120 Driver Functions
// These function should have there own error codes as they're dependent upon the radio and
// not the ESP32 :)

esp_err_t cc1120_radio_reset(void)
{
	return ESP_OK;
}

esp_err_t cc1120_radio_frequency(uint32_t freq)
{
	return ESP_OK;
}

esp_err_t cc1120_radio_sleep(void)
{
	return ESP_OK;
}

esp_err_t cc1120_radio_power(uint8_t txPower)
{
	return ESP_OK;
}

esp_err_t cc1120_radio_init(void)
{
	cc1120_gpio_init();
	cc1120_spi_init();
	return ESP_OK;
}

void app_main()
{
	cc1120_radio_init();
	uint8_t len, i;
	len = sizeof(CW_SETTINGS) / sizeof(cc1120_reg_settings_t);

	cc1120_spi_strobe(CC112X_SRES);

	vTaskDelay(500/portTICK_PERIOD_MS);

	//cc1120_spi_strobe(CC112X_SFSTXON);

	vTaskDelay(500/portTICK_PERIOD_MS);

	for (i=0;i<len;i++)
	{
		cc1120_spi_write_byte(CW_SETTINGS[i].addr, CW_SETTINGS[i].data);
	}

	vTaskDelay(500/portTICK_PERIOD_MS);


	cc1120_spi_strobe(CC112X_STX);

	while(1)
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
		printf("Status: %x\n", cc1120_spi_strobe(CC112X_STX));
		printf("Status: %x\n", cc1120_spi_strobe(CC112X_SNOP));
	}
}
