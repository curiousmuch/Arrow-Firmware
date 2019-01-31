/*
 * board.h
 *
 *  Created on: Jan 11, 2019
 *      Author: curiousmuch
 */

#ifndef MAIN_BOARD_H_
#define MAIN_BOARD_H_

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

#define DEBUG_0 			2
#define DEBUG_1				4


#endif /* MAIN_BOARD_H_ */
