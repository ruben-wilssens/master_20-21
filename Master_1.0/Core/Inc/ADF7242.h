/*
 * ADF7242.h
 *
 *  Created on: Feb 28, 2020
 *  Author: Jelle
 *  Edited: Ruben
 */

#ifndef INC_ADF7242_H_
#define INC_ADF7242_H_

#ifdef __cplusplus
extern C {
#endif

/*
 * The ADF7242 uses SPI1 for communication
 *
 * Default pinout
 *
	ADF7242   |STM32F4    |DESCRIPTION

	CS        |PB12       |Chip select (active low)
	MOSI      |PB15       |Master output, slave input
	SCLK      |PB10       |Serial clock line
	MISO      |PB14       |Master input, slave output
	IRQ1      |PA8        |Interrupt request 1
	IRQ2      |PA9        |Interrupt request 2
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "CBUF.h"

/* Defines -------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void ADF_Init(uint32_t);
void ADF_reset(void);
void ADF_sleep(void);

void ADF_SET_FREQ_kHz(uint32_t);
uint32_t ADF_RD_Frequency_MHz(void);

void ADF_SPI_MEM_WR(uint16_t, uint8_t);
uint8_t ADF_SPI_MEM_RD(uint16_t);

void ADF_set_turnaround_Tx_Rx(void);
void ADF_set_turnaround_Rx_Tx(void);

void ADF_set_IDLE_mode(void);
void ADF_set_PHY_RDY_mode(void);
void ADF_set_Tx_mode(void);
void ADF_set_Rx_mode(void);

uint8_t ADF_SPI_READY(void);
uint8_t ADF_RC_READY(void);

uint8_t ADF_Rx_READY(void);
uint8_t ADF_PHY_RDY_READY(void);
uint8_t ADF_IDLE_READY(void);
uint8_t ADF_status_word(void);

void ADF_clear_Rx_flag(void);
void ADF_clear_Tx_flag(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADF7242_H_ */
