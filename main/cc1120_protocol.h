/*
 * cc1120_aprs.h
 *
 *  Created on: Jan 12, 2019
 *      Author: curiousmuch
 */

#ifndef MAIN_CC1120_PROTOCOL_H_
#define MAIN_CC1120_PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>
#include "cc1120.h"

typedef struct {
	uint16_t addr;
	uint8_t data;
} cc1120_reg_settings_t;

/* APRS Radio Configuration */
// Address Config = No address check
// Bit Rate = 6
// Carrier Frequency = 144.389999
// Deviation = 5.004883
// Device Address = 0
// Manchester Enable = false
// Modulation Format = 2-FSK
// PA Ramping = true
// Packet Bit Length = 0
// Packet Length = 255
// Packet Length Mode = Variable
// Performance Mode = High Performance
// RX Filter BW = 20.000000
// Symbol rate = 6
// TX Power = 15
// Whitening = false
static const cc1120_reg_settings_t ARPS_SETTINGS[]=
{
  {CC112X_IOCFG3,            0xB0},
  {CC112X_IOCFG2,            0x06},
  {CC112X_IOCFG1,            0xB0},
  {CC112X_IOCFG0,            0x40},
  {CC112X_SYNC_CFG1,         0x0B},
  {CC112X_DEVIATION_M,       0x48},
  {CC112X_DCFILT_CFG,        0x1C},
  {CC112X_PREAMBLE_CFG1,     0x18},
  {CC112X_IQIC,              0xC6},
  {CC112X_CHAN_BW,           0x0A},
  {CC112X_MDMCFG0,           0x05},
  {CC112X_SYMBOL_RATE2,      0x68},
  {CC112X_SYMBOL_RATE1,      0x93},
  {CC112X_SYMBOL_RATE0,      0x75},
  {CC112X_AGC_REF,           0x20},
  {CC112X_AGC_CS_THR,        0x19},
  {CC112X_AGC_CFG1,          0xA9},
  {CC112X_AGC_CFG0,          0xCF},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_FS_CFG,            0x1B},
  {CC112X_PKT_CFG0,          0x20},
  {CC112X_PA_CFG0,           0x7E},
  {CC112X_PKT_LEN,           0xFF},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x22},
  {CC112X_FREQ2,             0x6C},
  {CC112X_FREQ1,             0x4A},
  {CC112X_FREQ0,             0xE1},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL1,           0x40},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC1,             0x03},
};

/* 900MHz CW Configuration */
// Address Config = No address check
// Bit Rate = 1.2
// Carrier Frequency = 915.000000
// Deviation = 3.997803
// Device Address = 0
// Manchester Enable = false
// Modulation Format = 2-FSK
// PA Ramping = false
// Packet Bit Length = 0
// Packet Length = 3
// Packet Length Mode = Not supported
// Performance Mode = High Performance
// RX Filter BW = 25.000000
// Symbol rate = 1.2
// TX Power = 15
// Whitening = false

static const cc1120_reg_settings_t CW_SETTINGS[]=
{
  {CC112X_IOCFG3,            0xB0},
  {CC112X_IOCFG2,            0x08},
  {CC112X_IOCFG1,            0xB0},
  {CC112X_IOCFG0,            0x09},
  {CC112X_SYNC_CFG1,         0x0B},
  {CC112X_DCFILT_CFG,        0x1C},
  {CC112X_PREAMBLE_CFG1,     0x00},
  {CC112X_IQIC,              0xC6},
  {CC112X_CHAN_BW,           0x08},
  {CC112X_MDMCFG0,           0x05},
  {CC112X_AGC_REF,           0x20},
  {CC112X_AGC_CS_THR,        0x19},
  {CC112X_AGC_CFG1,          0xA9},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_FS_CFG,            0x12},
  {CC112X_PKT_CFG2,          0x06},
  {CC112X_PKT_CFG1,          0x00},
  {CC112X_PKT_CFG0,          0x40},
  {CC112X_PA_CFG2,           0x3F},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x22},
  {CC112X_CFM_DATA_CFG,      0x01},
  {CC112X_FREQ2,             0x72},
  {CC112X_FREQ1,             0x60},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL1,           0x40},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC1,             0x03},
};
#endif /* MAIN_CC1120_PROTOCOL_H_ */
