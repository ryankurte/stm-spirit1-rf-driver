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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*---------------------------------------------------------------------------*/
#ifndef __S2LP_CONST_H__
#define __S2LP_CONST_H__
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

/**
 * @addtogroup RADIO
 * @ingroup STM32_Contiki_Library
 * @{
 * @file States configuration for the RADIO
 */

/* The state bitfield and values for different states, as registers,
which are returned on any SPI read or write operation. */
#define S2LP_STATE_STATEBITS           (0x00FE)
/*---------------------------------------------------------------------------*/
#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
#define S2LP_STATE_STANDBY             ((0x0002)<<1)
#define S2LP_STATE_SLEEP               ((0x0001)<<1)
#define S2LP_STATE_READY               ((0x0000)<<1)
#define S2LP_STATE_LOCK                ((0x000C)<<1)
#define S2LP_STATE_RX                  ((0x0030)<<1)
#define S2LP_STATE_TX                  ((0x005C)<<1)

/* S2-LP States */

/*
#define S2LP_STATE_STANDBY             0x02
#define S2LP_STATE_SLEEP               0x01
#define S2LP_STATE_READY               0x00
#define S2LP_STATE_LOCK                0x0C
#define S2LP_STATE_RX                  0x30
#define S2LP_STATE_TX                  0x5C
*/
#endif


#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)

#define S2LP_STATE_STANDBY             ((0x0040)<<1)
#define S2LP_STATE_SLEEP               ((0x0036)<<1)
#define S2LP_STATE_READY               ((0x0003)<<1)
#define S2LP_STATE_LOCK                ((0x000F)<<1)
#define S2LP_STATE_RX                  ((0x0033)<<1)
#define S2LP_STATE_TX                  ((0x005F)<<1)


#endif


/* NB the below states were extracted from ST drivers, but are not specified in the datasheet */
/*---------------------------------------------------------------------------*/
/* strobe commands */
#define S2LP_STROBE_TX             0x60
#define S2LP_STROBE_RX             0x61
#define S2LP_STROBE_READY          0x62
#define S2LP_STROBE_STANDBY        0x63
#define S2LP_STROBE_SLEEP          0x64
#define S2LP_STROBE_SABORT         0x67
#define S2LP_STROBE_SRES           0x70
#define S2LP_STROBE_FRX            0x71
#define S2LP_STROBE_FTX            0x72
/*---------------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/
/*------------------------------------------------------------------*/


#endif /* __S2LP_CONST_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
