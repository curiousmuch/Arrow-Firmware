/*
 * Project: Arrow
 * Author: 	curiousmuch
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_task_wdt.h"
#include "cc1120.h"
#include "cc1120_protocol.h"
#include "board.h"

extern uint8_t tx_symbol;
extern uint8_t sample_count;


void IRAM_ATTR app_main()
{
	cc1120_radio_init(CW_SETTINGS, sizeof(CW_SETTINGS)/sizeof(cc1120_reg_settings_t));

	vTaskDelay(500/portTICK_PERIOD_MS);
	cc1120_radio_APRSTXPacket();

	while(1)
	{
		if (sample_count == 11)
		{
			tx_symbol = tx_symbol ^ 1;
			//tx_symbol = 1;
			sample_count = 0;
		}
		esp_task_wdt_reset();
	}

}
