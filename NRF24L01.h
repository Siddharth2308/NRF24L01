/*
 * NRF24L01.h
 *
 *  Created on: Jul 31, 2021
 *      Author: Siddharth
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_


#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

class NRF24{
	public:
//		NRF24(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CE_PORT, GPIO_TypeDef CE_Pin, GPIO_TypeDef *CSN_PORT, GPIO_TypeDef CSN_Pin);
		NRF24(SPI_HandleTypeDef*hspi, GPIO_TypeDef* CE_PORT, uint16_t CE_Pin, GPIO_TypeDef*CSN_PORT, uint16_t CSN_Pin);
		void beginTransmission(uint8_t *Address, uint8_t channel); // txmode
		bool transfer(uint8_t *data);
		void beginReception(uint8_t *Address, uint8_t channel);
		bool available(int pipenum);
		void receive(uint8_t *data);
		void readAll(uint8_t *data);
//		void CS_Select(void);
//		void CS_UnSelect(void);
		void CE_Enable(void);
		void CE_Disable(void);
		void WriteReg(uint8_t Reg, uint8_t data);
		void WriteRegMulti(uint8_t Reg, uint8_t *data, int size);
		uint8_t ReadReg(uint8_t Reg);
		void ReadRegMulti(uint8_t Reg, uint8_t *data, int size);
//		void sendCmd(uint8_t cmd);
		void reset(uint8_t Reg);
	private:
//		GPIO_TypeDef *PORT_CE, *PORT_CS;
//		uint16_t CE, CS;

};


#endif /* INC_NRF24L01_H_ */
