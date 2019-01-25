/*
 * Project: Arrow
 * Author: 	curiousmuch
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "cc1120.h"
#include "cc1120_protocol.h"
#include "board.h"

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
	.max_transfer_sz = 150,
	.flags = ESP_INTR_FLAG_IRAM
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
	.queue_size = 20

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
	ret = spi_bus_initialize(VSPI_HOST, &bus_config, 0);	// this uses DMA channel 1
	ESP_ERROR_CHECK(ret);
	ret = spi_bus_add_device(VSPI_HOST, &interface_config, &spi);
	ESP_ERROR_CHECK(ret);
}

void IRAM_ATTR cc1120_spi_write_byte(uint16_t addr, uint8_t data)
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
		ret = spi_device_polling_transmit(spi, (spi_transaction_t*)&tx_trans_ext);
	}
	else
	{
		ret = spi_device_polling_transmit(spi, &tx_trans);
	}
	ESP_ERROR_CHECK(ret);
}

void IRAM_ATTR cc1120_spi_write_bytes(uint16_t addr, uint8_t* data, uint8_t len)
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

rf_status_t IRAM_ATTR cc1120_spi_strobe(uint8_t cmd)
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

rf_status_t cc1120_radio_reset(void)
{
	rf_status_t status;
	uint8_t retry_count = 0;
	cc1120_spi_strobe(CC112X_SRES);
	status = cc1120_spi_strobe(CC112X_SNOP);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	while((CC112X_RDYn_BIT & (status & 0x80)))
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if (retry_count > 3)
		{
			// place error CC1120 timeout
			printf("CC1120 Reset Failure\n");
			break;
		}
		status = cc1120_spi_strobe(CC112X_SNOP);
		retry_count++;
	}
	printf("%x\n", retry_count);
	return status;
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


#define HDLC_FLAG 0x7E
#define HDLC_FLAG_LEN 10

uint8_t packet_len = 0;
uint8_t test_vector[] = {0x71, 0x01, 023, 0xAE, 0x75};
uint8_t sample_count = 0;
uint32_t tx_symbol = 0;
uint8_t prev_tx_symbol = 0;



#define SAMPLE_FREQUENCY 13200
#define DAC_MAX 64
#define LUT_SIZE DAC_MAX*2

DRAM_ATTR int8_t LUT[LUT_SIZE];

int8_t phase_i = 0;
float phase, delta_phi;
float const delta_phi_1 = (float) 1200 / SAMPLE_FREQUENCY * LUT_SIZE;
float const delta_phi_2 = (float) 2200 / SAMPLE_FREQUENCY * LUT_SIZE;


// The output needs to be continous phase.

static void IRAM_ATTR cc1120_aprs_tx_isr(void* arg)
{
	if (tx_symbol)
		delta_phi = delta_phi_1;
	else
		delta_phi = delta_phi_2;

    phase_i = (int8_t)phase;        // get integer part of our phase

    cc1120_spi_write_byte(CC112X_CFM_TX_DATA_IN, LUT[phase_i]);

    phase += delta_phi;              // increment phase

    if (phase >= (float)LUT_SIZE)    // handle wraparound
        phase -= (float)LUT_SIZE;

	sample_count++;
}

void cc1120_lut_init(void)
{
	int16_t i=0;
	for (i=0; i<LUT_SIZE; i++)
	{
		LUT[i] = (int8_t)roundf(DAC_MAX * sinf(2.0f * M_PI * (float)i / LUT_SIZE));
	}
}

// test function to generate APRS s1 or s2
void cc1120_radio_APRSTXPacket(void)
{
	// start CW transmission
	cc1120_spi_write_byte(CC112X_FIFO, 0x12);
	cc1120_spi_strobe(CC112X_STX);

	// enable interrupt pin for CC1120 for timing packets
	gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

	gpio_isr_handler_add(CC1120_GPIO3, cc1120_aprs_tx_isr, NULL);
	gpio_set_intr_type(CC1120_GPIO3, GPIO_INTR_POSEDGE);

	// acquire SPI bus for fastest possible SPI transactions
	spi_device_acquire_bus(spi, portMAX_DELAY);

	// send Flag
	// send Loaded Packet
	// send Flag
}

void cc1120_radio_init(const cc1120_reg_settings_t* rf_settings, uint8_t len)
{
	cc1120_gpio_init();
	cc1120_spi_init();
	cc1120_lut_init();

	cc1120_radio_reset();

	uint8_t i;

	for (i=0;i<len;i++)
	{
		cc1120_spi_write_byte(rf_settings[i].addr, rf_settings[i].data);
	}
}
