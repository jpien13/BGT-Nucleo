/*
 * bgt60ltr11_spi.c
 *
 *  Created on: Feb 26, 2025
 *      Author: jasonpien
 */

#include "bgt60ltr11_spi.h"
#include "gpio.h"

#define BGT60_CS_PORT GPIOA
#define BGT60_CS_PIN  GPIO_PIN_4

/*
 *
 * 1. Transfer starts with the falling edge of SPICS
 * 2. 7-bit address is sent MSB first
 * 3. 8th bit (RW) is "0" for read access
 * 4. The next 16 bits on SPIDI are ignored (don't care)
 * 5. Read data (16 bits) is shifted out on SPIDO
 * 6. Transfer ends with rising edge of SPICS
 *
 * Page 19 of BGT60LTR11AIP User guide
 * https://www.infineon.com/dgdl/Infineon-UG124434_User_guide_to_BGT60LTR11AIP-UserManual-v01_80-EN.pdf?fileId=8ac78c8c8823155701885724e6d72f8f
 *
 */
uint8_t bgt60ltr11_spi_read(uint8_t reg_addr, uint16_t *data) {
    uint8_t tx_data[3];
    uint8_t rx_data[3] = {0, 0, 0};

    /* We send the register address from where we want to read
     * and then we read 2 bytes with dummy data
     */
    tx_data[0] = (uint8_t)((reg_addr << 1) & 0xFE); //  shifts the 7-bit address to make room for the RW bit. Address (7 bits) + RW bit (0)
    tx_data[1] = 0; 								// Dummy byte (ignored by radar)
    tx_data[2] = 0; 								// Dummy byte (ignored by radar)

    // CS low to start SPI transfer occurs here
    HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, sizeof(tx_data), 100) != HAL_OK) {
        HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_SET);
        return HAL_ERROR;
    }
    // CS high to end
    HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_SET);

    // After transmission, the 16-bit register value is reconstructed from the received bytes:
    *data = ((uint16_t)(rx_data[1] << 8) | (uint16_t)(rx_data[2]));
    return HAL_OK;
}

/*
 *
 * 1. Transfer starts with the falling edge of SPICS
 * 2. 7-bit address is sent MSB first
 * 3. 8th bit (RW) is "1" for write access
 * 4. 16-bit payload (data) is sent MSB first
 * 5. During this time, GSR0 and previous register data are shifted out on SPIDO
 * 6. Transfer ends with rising edge of SPICS
 *
 * Page 19 of BGT60LTR11AIP User guide
 * https://www.infineon.com/dgdl/Infineon-UG124434_User_guide_to_BGT60LTR11AIP-UserManual-v01_80-EN.pdf?fileId=8ac78c8c8823155701885724e6d72f8f
 *
 */
unit8_t bgt60ltr11_spi_write(uint8_t reg_addr, uint16_t data){
	unt8_t tx_data[3];
	uint16_t wrdata = data;

	tx_data[0] = (uint8_t)((reg_addr << 1) | 0x01); // Shifts the 7-bit address to make room for the RW bit. Address (7 bits) + RW bit (1)
	tx_data[1] = (uint8_t)((wrdata >> 8) & 0xFF);   // Upper 8 bits of data (MSB first)
	tx_data[2] = (uint8_t)(wrdata & 0xFF);			// Lower 8 bits of data

    // CS low to start SPI transfer occurs here
	HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data)/sizeof(uint8_t), 100) != HAL_OK) {
		HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_SET);
		return HAL_ERROR;
	}
	// CS high to end
	HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_SET);
	return HAL_OK;
}
