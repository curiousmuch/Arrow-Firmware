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

#include "ax25_pad2.h"
#include "ax25_pad.h"
#include "fcs_calc.h"

extern uint8_t tx_symbol;
extern uint8_t sample_count;

void IRAM_ATTR app_main()
{
	cc1120_radio_init(APRS_SETTINGS, sizeof(APRS_SETTINGS)/sizeof(cc1120_reg_settings_t));
	vTaskDelay(500/portTICK_PERIOD_MS);

	// generate sample packet
	packet_t pp;
	unsigned char fbuf[AX25_MAX_PACKET_LEN+2];
	uint32_t flen;
	uint32_t c;

	pp = ax25_from_text("WB2OSZ-15>TEST:,The quick brown fox jumps over the lazy dog!  1 of 4", 1);
	flen = ax25_pack(pp, fbuf);

	uint32_t fcs = fcs_calc(fbuf, flen);

	ax25_hex_dump(pp);
	printf("FCS: %x\n", fcs);


	//cc1120_radio_APRSTXPacket();
	//xTaskCreatePinnedToCore();



}
