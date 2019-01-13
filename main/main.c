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
	.clock_speed_hz = SPI_MASTER_FREQ_8M,
	.flags = SPI_DEVICE_HALFDUPLEX,
	.queue_size = 1

};

spi_device_handle_t spi;

// Private CC1120 Driver Functions
esp_err_t cc1120_gpio_init(void)
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

	return ESP_OK;
}

esp_err_t cc1120_spi_init(void)
{
	esp_err_t ret;
	ret = spi_bus_initialize(VSPI_HOST, &bus_config, 1);	// this uses DMA channel 1
	ESP_ERROR_CHECK(ret);
	ret = spi_bus_add_device(VSPI_HOST, &interface_config, &spi);
	ESP_ERROR_CHECK(ret);
	return ESP_OK;
}

esp_err_t cc1120_spi_write_byte(uint16_t addr, uint8_t data)
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
	return ESP_OK;
}

esp_err_t cc1120_spi_write_bytes(uint16_t addr, uint8_t* data, uint8_t len)
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
	return ESP_OK;
}

esp_err_t cc1120_spi_read_byte(uint16_t addr, uint8_t* data)
{
	esp_err_t ret;
	spi_transaction_t rx_trans =
	{
		.flags = SPI_TRANS_USE_RXDATA,
		.cmd = CC1120_READ_BIT,
		.addr = addr,
		.length = 8,
		.rxlength = 8,
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
	*data = rx_trans.rx_data[0];
	return ESP_OK;
}

esp_err_t cc1120_spi_read_bytes(uint16_t addr, uint8_t* data, uint8_t len)
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
	return ESP_OK;
}

esp_err_t cc1120_spi_strobe(uint8_t cmd)
{
	esp_err_t ret;
	spi_transaction_t strobe_trans =
	{
		.cmd = CC1120_WRITE_BIT,
		.addr = cmd,
		.length = 0,
	};
	ret = spi_device_transmit(spi, &strobe_trans);
	ESP_ERROR_CHECK(ret);
	return ESP_OK;

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
	printf("Hello\n");
	uint8_t data;
	// function test loop
	while(1)
	{
		//cc1120_spi_write_byte(CC112X_PKT_CFG2, 0x0F);
		//cc1120_spi_write_byte(CC112X_FS_CAL0, 0x0F);
		cc1120_spi_read_byte(CC112X_PARTNUMBER, &data);
		cc1120_spi_strobe(CC112X_STX);
		vTaskDelay(30/portTICK_RATE_MS);
		cc1120_spi_strobe(CC112X_SRX);
		printf("data: %x\n", data);
		//esp_task_wdt_reset();
		vTaskDelay(1000/portTICK_RATE_MS);
	}
}
