/*
 * NRF24L01.cpp
 *
 *  Created on: Jul 31, 2021
 *      Author: Siddharth
 */

#include "stm32f4xx_hal.h"
#include "NRF24L01.h"

GPIO_TypeDef *PORT_CE, *PORT_CS;
uint16_t CE, CS;
SPI_HandleTypeDef *NRF24_SPI;


NRF24::NRF24(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CE_PORT, uint16_t CE_Pin, GPIO_TypeDef*CSN_PORT, uint16_t CSN_Pin){
	NRF24_SPI = hspi;
	PORT_CE = CE_PORT;
	CE = CE_Pin;
	PORT_CS = CSN_PORT;
	CS = CSN_Pin;
	// Write Initialization below
	CE_Disable();
	reset (0);
	WriteReg(CONFIG, 0);
	WriteReg(EN_AA, 0);
	WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now
	WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address
	WriteReg (SETUP_RETR, 0);   // No retransmission
	WriteReg (RF_CH, 0);  // will be setup during Tx or RX
	WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps
	CE_Enable();
};

void CS_Select(void){
	HAL_GPIO_WritePin(PORT_CS, CS, GPIO_PIN_RESET);
};

void CS_UnSelect(void){
	HAL_GPIO_WritePin(PORT_CS, CS, GPIO_PIN_SET);
};

void NRF24::CE_Enable(void){
	HAL_GPIO_WritePin(PORT_CE, CE, GPIO_PIN_SET);
};

void NRF24::CE_Disable(void){
	HAL_GPIO_WritePin(PORT_CE, CE, GPIO_PIN_RESET);
};

void NRF24::WriteReg(uint8_t Reg, uint8_t data){
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = data;
	// Pull the CS Pin LOW to select the device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
};

void NRF24::WriteRegMulti(uint8_t Reg, uint8_t *data, int size){
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	// Pull the CS Pin LOW to select the device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
};

uint8_t NRF24::ReadReg(uint8_t Reg){
	uint8_t data=0;
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);
	CS_UnSelect();
	return data;
};

void NRF24::ReadRegMulti(uint8_t Reg, uint8_t *data, int size){
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);
	CS_UnSelect();
}

void sendCmd(uint8_t cmd){
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
	CS_UnSelect();
};

void NRF24::reset(uint8_t Reg){
	if(Reg == STATUS){
		WriteReg(STATUS, 0x00);
	} else if(Reg == FIFO_STATUS){
		WriteReg(FIFO_STATUS, 0x11);
	} else{
		WriteReg(CONFIG, 0x08);
		WriteReg(EN_AA, 0x3F);
		WriteReg(EN_RXADDR, 0x03);
		WriteReg(SETUP_AW, 0x03);
		WriteReg(SETUP_RETR, 0x03);
		WriteReg(RF_CH, 0x02);
		WriteReg(RF_SETUP, 0x0E);
		WriteReg(STATUS, 0x00);
		WriteReg(OBSERVE_TX, 0x00);
		WriteReg(CD, 0x00);
		uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
		uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
		WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
		WriteReg(RX_ADDR_P2, 0xC3);
		WriteReg(RX_ADDR_P3, 0xC4);
		WriteReg(RX_ADDR_P4, 0xC5);
		WriteReg(RX_ADDR_P5, 0xC6);
		uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		WriteRegMulti(TX_ADDR, tx_addr_def, 5);
		WriteReg(RX_PW_P0, 0);
		WriteReg(RX_PW_P1, 0);
		WriteReg(RX_PW_P2, 0);
		WriteReg(RX_PW_P3, 0);
		WriteReg(RX_PW_P4, 0);
		WriteReg(RX_PW_P5, 0);
		WriteReg(FIFO_STATUS, 0x11);
		WriteReg(DYNPD, 0);
		WriteReg(FEATURE, 0);
	}
};

void NRF24::beginTransmission(uint8_t *Address, uint8_t channel){
	CE_Disable();
	WriteReg(RF_CH, channel);
	WriteRegMulti(TX_ADDR, Address, 5);
	uint8_t config= ReadReg(CONFIG);
	//	config = config | (1<<1);   // write 1 in the PWR_UP bit
	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	WriteReg (CONFIG, config);
	CE_Enable();
};

bool NRF24::transfer(uint8_t *data){
	uint8_t cmdtosend = 0;
	CS_Select();
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);
	CS_UnSelect();
	HAL_Delay(1);
	uint8_t fifostatus = ReadReg(FIFO_STATUS);
	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		cmdtosend = FLUSH_TX;
		sendCmd(cmdtosend);
		reset(FIFO_STATUS);
		return true;
	}
	return false;
};

void NRF24::beginReception(uint8_t *Address, uint8_t channel){
	CE_Disable();
	reset(STATUS);
	WriteReg(RF_CH, channel);
	uint8_t en_rxaddr = ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	WriteReg (EN_RXADDR, en_rxaddr);
	WriteRegMulti(RX_ADDR_P1, Address, 5);
	WriteReg(RX_ADDR_P2, 0xEE);
	WriteReg (RX_PW_P2, 32);   // 32 bit payload size for pipe 2
	uint8_t config = ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	WriteReg (CONFIG, config);
	CE_Enable();
};

bool NRF24::available(int pipenum){
	uint8_t status = ReadReg(STATUS);
	if ((status&(1<<6))&&(status&(pipenum<<1))){
		WriteReg(STATUS, (1<<6));
		return 1;
	}
	return 0;
};

void receive(uint8_t *datta){
	uint8_t cmdtosend = 0;
	CS_Select();
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, datta, 32, 1000);
	CS_UnSelect();
	HAL_Delay(1);
	cmdtosend = FLUSH_RX;
	sendCmd(cmdtosend);
};

void NRF24::readAll(uint8_t *data){
	for (int i=0; i<10; i++){
		*(data+i) = ReadReg(i);
	}

	ReadRegMulti(RX_ADDR_P0, (data+10), 5);
	ReadRegMulti(RX_ADDR_P1, (data+15), 5);
	*(data+20) = ReadReg(RX_ADDR_P2);
	*(data+21) = ReadReg(RX_ADDR_P3);
	*(data+22) = ReadReg(RX_ADDR_P4);
	*(data+23) = ReadReg(RX_ADDR_P5);
	ReadRegMulti(RX_ADDR_P0, (data+24), 5);
	for (int i=29; i<38; i++){
		*(data+i) = ReadReg(i-12);
	}
};



