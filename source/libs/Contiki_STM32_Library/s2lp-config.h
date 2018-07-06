/*
 * Copyright (c) 2012, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
#ifndef __RADIO_CONFIG_H__
#define __RADIO_CONFIG_H__
/*---------------------------------------------------------------------------*/

#include "s2lp-const.h"
#include "radio_shield_config.h"
#include "MCU_Interface.h"

/*---------------------------------------------------------------------------*/   
extern const struct radio_driver subGHz_radio_driver;
/*---------------------------------------------------------------------------*/
/**
 * @addtogroup ST_Radio
 * @ingroup STM32_Contiki_Library
 * @{
 * @file Spirit1 configuration file for Contiki
 */
/*---------------------------------------------------------------------------*/
void Radio_interrupt_callback(void);
/*---------------------------------------------------------------------------*/
#if defined(X_NUCLEO_IDS01A3)
	 #define USE_RADIO_433MHz
#elif defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_S2868A1)
         #define USE_RADIO_868MHz
#elif defined(X_NUCLEO_IDS01A5) || defined(X_NUCLEO_S2915A1)
         #define USE_RADIO_915MHz
#else
#error RADIO Nucleo Shield undefined or unsupported
#endif

/*  Uncomment the Link Layer features to be used */
// #define USE_AUTO_ACK
// #define USE_AUTO_ACK_PIGGYBACKING
// #define USE_AUTO_RETRANSMISSION

#if defined(USE_AUTO_ACK)&& defined(USE_AUTO_ACK_PIGGYBACKING)&& defined(USE_AUTO_RETRANSMISSION)

/* LLP configuration parameters */
#define EN_AUTOACK                      S_ENABLE
#define EN_PIGGYBACKING             	S_ENABLE
#define MAX_RETRANSMISSIONS         	PKT_N_RETX_2

#else
#define USE_BASIC_PROTOCOL
#endif

/*  Uncomment the system Operating mode */
//#define USE_LOW_POWER_MODE

#if defined (USE_LOW_POWER_MODE)
#define LPM_ENABLE
#define MCU_STOP_MODE
//#define MCU_SLEEP_MODE
//#define RF_STANDBY
#endif


/* Exported constants --------------------------------------------------------*/

/*  Radio configuration parameters  */
#define XTAL_OFFSET_PPM             0
#define INFINITE_TIMEOUT            0.0

#ifdef USE_RADIO_433MHz
#define BASE_FREQUENCY              433.0e6
#endif

#ifdef USE_RADIO_868MHz
#define BASE_FREQUENCY              868.0e6
#endif

#ifdef USE_RADIO_915MHz
//#define BASE_FREQUENCY            915.0e6
#define BASE_FREQUENCY              902.0e6
#endif

#define RADIO_IRQ_ENABLE()   RadioGpioInterruptCmd(RADIO_GPIO_IRQ,0x04,0x04,ENABLE)
#define RADIO_IRQ_DISABLE()  RadioGpioInterruptCmd(RADIO_GPIO_IRQ,0x04,0x04,DISABLE) 

#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
#define CHANNEL_SPACE               100e3
#define CHANNEL_NUMBER              0 
#define MODULATION_SELECT           MOD_2FSK
#define DATARATE                    38400
#define FREQ_DEVIATION              20e3
#define BANDWIDTH                   100.0e3
   
#define POWER_DBM                   12.0

/*  Packet configuration parameters  */
#define PREAMBLE_LENGTH             PREAMBLE_BYTE(4)
#define SYNC_LENGTH                 SYNC_BYTE(4)
#define SYNC_WORD                   0x88888888
#define VARIABLE_LENGTH             S_ENABLE
#define EXTENDED_LENGTH_FIELD       S_DISABLE
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define EN_ADDRESS                  S_DISABLE //S_ENABLE 
#define EN_FEC                      S_DISABLE
#define EN_WHITENING                S_ENABLE

#define PREAMBLE_BYTE(v)           (4*v)
#define SYNC_BYTE(v)               (8*v)


#define POWER_INDEX                 7
#define RECEIVE_TIMEOUT             2000.0 /*change the value for required timeout period*/

#define RSSI_RX_THRESHOLD          -118.0   /* dBm */
#define RSSI_TX_THRESHOLD          -90.0   /* dBm */


//#define LENGTH_WIDTH                7
//#define CONTROL_LENGTH              0x00 


/*  Addresses configuration parameters  */
#define EN_FILT_MY_ADDRESS          S_ENABLE
#define MY_ADDRESS                  0x24
#define EN_FILT_MULTICAST_ADDRESS   S_ENABLE
#define MULTICAST_ADDRESS           0xEE
#define EN_FILT_BROADCAST_ADDRESS   S_ENABLE
#define BROADCAST_ADDRESS           0xFF
#define DESTINATION_ADDRESS         0x44
#define EN_FILT_SOURCE_ADDRESS      S_ENABLE

#define LPM_WAKEUP_TIME                 100
#define DATA_SEND_TIME               	50

#define RADIO_MAX_FIFO_LEN         128
#endif

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
/*  Addresses configuration parameters  */
#define EN_FILT_MY_ADDRESS          S_DISABLE
#define MY_ADDRESS                  0x24
#define EN_FILT_MULTICAST_ADDRESS   S_DISABLE
#define MULTICAST_ADDRESS           0xEE
#define EN_FILT_BROADCAST_ADDRESS   S_DISABLE
#define BROADCAST_ADDRESS           0xFF
#define DESTINATION_ADDRESS         0x44
#define EN_FILT_SOURCE_ADDRESS      S_DISABLE
#define SOURCE_ADDR_MASK            0xf0
#define SOURCE_ADDR_REF             0x37

#define APPLI_CMD                       0x11
#define NWK_CMD                         0x22
#define LED_TOGGLE                      0xff
#define ACK_OK                          0x01
#define MAX_BUFFER_LEN                  96
#define TIME_TO_EXIT_RX                 3000
#define DELAY_RX_LED_TOGGLE             200
#define DELAY_TX_LED_GLOW               1000
#define LPM_WAKEUP_TIME                 100
#define DATA_SEND_TIME               	30

#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_ADDRESS                  S_DISABLE
#define EN_FEC                      S_DISABLE
#define CHANNEL_NUMBER              0
#define LENGTH_TYPE                 PKT_LENGTH_VAR
#define POWER_INDEX                 7
#define RECEIVE_TIMEOUT             2000.0 /*change the value for required timeout period*/

#define RSSI_RX_THRESHOLD          -118.0   /* dBm */
#define RSSI_TX_THRESHOLD          -107.0   /* dBm */

#define POWER_DBM                   11.6
#define CHANNEL_SPACE               20e3
#define FREQ_DEVIATION              20e3
#define BANDWIDTH                   100.0e3
#define MODULATION_SELECT           FSK
#define DATARATE                    38400
#define XTAL_OFFSET_PPM             0
#define SYNC_WORD                   0x1A2635A8
#define LENGTH_WIDTH                7
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define EN_WHITENING                S_ENABLE
#define INFINITE_TIMEOUT            0.0

#define RADIO_MAX_FIFO_LEN         96
#endif

#define XTAL_FREQUENCY              50000000    /* Hz */ 


/**    
 * The MAX_PACKET_LEN is an arbitrary value used to define the two array
 * radio_txbuf and radio_rxbuf.
 * The RADIO supports with its packet handler a length of 65,535 bytes,
 * and in direct mode (without packet handler) there is no limit of data.
 */  
#define MAX_PACKET_LEN              RADIO_MAX_FIFO_LEN


extern volatile FlagStatus PushButtonStatusWakeup;
extern uint16_t wakeupCounter;
extern uint16_t dataSendCounter ;
extern volatile FlagStatus PushButtonStatusData;

/*---------------------------------------------------------------------------*/
#endif /* __RADIO_CONFIG_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
