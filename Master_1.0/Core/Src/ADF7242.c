/*
 * ADF7242.c
 *
 *  Created on: Feb 28, 2020
 *  Author: Jelle
 *  Edited: Ruben
 */
#include "ADF7242.h"

/* Variables -------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern uint8_t RX_BUFFER_BASE;
extern uint8_t TX_BUFFER_BASE;

extern uint8_t ADF_status;

/* Functions -------------------------------------------------------------------*/

/* ADF7242 config after cold start or wake-up from sleep */
void ADF_Init(uint32_t frequency){
	HAL_GPIO_WritePin(ADF7242_GP1_GPIO_Port, ADF7242_GP1_Pin, GPIO_PIN_RESET); //transmit data input low SPORT mode
	HAL_GPIO_WritePin(PA_LNA_HGM_GPIO_Port, PA_LNA_HGM_Pin, GPIO_PIN_RESET); //HGM off

	uint8_t res = 0;
	// Reset transceiver at startup with RC_RESET command (p.37)
	ADF_reset();
	HAL_Delay(10);

	// RC_mode = IEEE802.15.4 packet with automatic preamble and SFD generation
	ADF_SPI_MEM_WR(0x13e,0x00);

	// Interrupt setup
	// irq1_en0 register -> no battery monitor, chip ready for access... interrupt on IRQ1
	ADF_SPI_MEM_WR(0x3c7,0x00);
	// irq1_en1 register -> packet transmission complete interrupt on IRQ1
	ADF_SPI_MEM_WR(0x3c8,0x10);
	// irq2_en0 register -> no battery monitor, chip ready for access... interrupt on IRQ2
	ADF_SPI_MEM_WR(0x3c9,0x00);
	// irq2_en1 register -> packet received in RX_BUFFER interrupt on IRQ2
	ADF_SPI_MEM_WR(0x3ca,0x08);

	// Clear all interrupt flags
	ADF_SPI_MEM_WR(0x3cb,0xff);
	ADF_SPI_MEM_WR(0x3cc,0xff);

	// Set transceiver in idle state
	ADF_set_IDLE_mode();
	HAL_Delay(10);

	// Set frequency
	ADF_SET_FREQ_kHz(frequency);

	// Read register txpb (Transmit packet storage base address)
	TX_BUFFER_BASE = ADF_SPI_MEM_RD(0x314);
	// Read register rxpb (Receive packet storage base address)
	RX_BUFFER_BASE = ADF_SPI_MEM_RD(0x315);

	// Set transceiver in idle state
	ADF_set_IDLE_mode();
	HAL_Delay(10);


	// PA power [7:4] min=3, max=15 & [3]=0 & [2:0]=1
	ADF_SPI_MEM_WR(0x3aa, 0xf1);

	// Setup for external PA and LNA (ext_ctrl register)
	ADF_SPI_MEM_WR(0x100, 0x1C);

	HAL_Delay(50);
	ADF_set_PHY_RDY_mode();
}

void ADF_reset(void){
	// Write RC_RESET command
	uint8_t byte[] = {0xC8};
	uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, byte, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

void ADF_sleep(void){
	// Sleep BBRAM in tmr_cfg1 register
	ADF_SPI_MEM_WR(0x317,0x08);

	// Write RC_sleep command
	uint8_t bytes[] = {0xb1};

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

void ADF_SET_FREQ_kHz(uint32_t frequency){
	uint8_t LSB = frequency&0xff;
	uint8_t MSB = (frequency>>8)&0xff;
	uint8_t HSB = (frequency>>16)&0xff;
	ADF_SPI_MEM_WR(0x302,HSB);
	ADF_SPI_MEM_WR(0x301,MSB);
	ADF_SPI_MEM_WR(0x300,LSB);
}

uint32_t ADF_RD_Frequency_MHz(void){
	uint32_t frequency = ADF_SPI_MEM_RD(0x302);
	frequency = (frequency<<8) | ADF_SPI_MEM_RD(0x301);
	frequency = (frequency<<8) | ADF_SPI_MEM_RD(0x300);

	frequency /= 100;

	return frequency;
}

void ADF_SPI_MEM_WR(uint16_t reg, uint8_t data){
	uint8_t SPI_MEM_WR_MODES[3] = {0x08, 0x09, 0x0b};
	uint8_t mode;

	if (reg < 0x100)
		mode = 0;
	else if (reg > 0x13f)
		mode = 2;
	else
		mode = 1;

	uint8_t bytes[3];
	bytes[0] = SPI_MEM_WR_MODES[mode] | ((reg>>8)&0x07);
	bytes[1] = reg&0xff;
	bytes[2] = data;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, bytes, 3, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
}

uint8_t ADF_SPI_MEM_RD(uint16_t reg){
	uint8_t SPI_MEM_RD_MODES[3] = {0x28, 0x29, 0x2b};
	uint8_t mode;
	uint8_t value;

	if (reg < 0x100)
		mode = 0;
	else if (reg > 0x13f)
		mode = 2;
	else
		mode = 1;

	uint8_t bytes[3];
	bytes[0] = SPI_MEM_RD_MODES[mode] | ((reg>>8)&0x07);
	bytes[1] = reg&0xff;
	bytes[2] = 0xff;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, bytes, 3, 50);
	HAL_SPI_Receive(&hspi1, &value, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	return value;
}

void ADF_set_turnaround_Tx_Rx(void){
	// Set auto_tx_to_rx_turnaround
	ADF_SPI_MEM_WR(0x107,0x08);
}

void ADF_set_turnaround_Rx_Tx(void){
	// Set auto_rx_to_tx_turnaround
	ADF_SPI_MEM_WR(0x107,0x04);
}

void ADF_set_IDLE_mode(void){
	uint8_t bytes[] = {0xB2};
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	//delay_us(31); // TX to Idle state (max transition timing) p.14
}

void ADF_set_PHY_RDY_mode(void){
	uint8_t bytes[] = {0xb3};
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	//delay_us(145); // Idle to PHY_RDY state (max transition timing) p.14
}

void ADF_set_Tx_mode(void){
	uint8_t bytes[] = {0xb5};
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, bytes, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	//delay_us(200); // (max transition timing) p.14
}

void ADF_set_Rx_mode(void){
	uint8_t bytes[] = {0xb4};
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, bytes, 1,50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	//delay_us(200); // (max transition timing) p.14
}

/* Check if transceiver is ready for SPI access (p.73) */
uint8_t ADF_SPI_READY(void){
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((ADF_status&0x80) == 0x80)
		return 1;
	else
		return 0;
}

/* Check if transceiver is ready for RC command (p.73) */
uint8_t ADF_RC_READY(void){
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((ADF_status&0xA0) == 0xA0)
		return 1;
	else
		//ADF_clear_Tx_flag(); //debug test
		//ADF_clear_Rx_flag(); //debug test
		return 0;
}

/* Check RC_status of SPI status word (p.73) */
uint8_t ADF_Rx_READY(void){
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((ADF_status&0xA4) == 0xA4)
		return 1;
	else
		return 0;
}

/* Check RC_STATUS of SPI status word (p.73) */
uint8_t ADF_PHY_RDY_READY(void){
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((ADF_status&0xA3) == 0xA3)
		return 1;
	else
		return 0;
}

/* Check RC_STATUS of SPI status word (p.73) */
uint8_t ADF_IDLE_READY(void){
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	if ((ADF_status&0xA1) == 0xA1)
		return 1;
	else
		return 0;
}

/* Check SPI status word (p.73) */
uint8_t ADF_status_word(void){
	//uint8_t ADF_status;

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, 0xff, 1, 50);
	HAL_SPI_Receive(&hspi1, &ADF_status, 1, 50);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	return ADF_status;
}

void ADF_clear_Rx_flag(void){
	ADF_SPI_MEM_WR(0x3cc,0x08);
}

void ADF_clear_Tx_flag(void){
	ADF_SPI_MEM_WR(0x3cc,0x10);
}
