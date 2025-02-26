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

uint8_t bgt60ltr11_spi_read(uint8_t reg_addr, uint16_t *data) {
    uint8_t tx_data[3];
    uint8_t rx_data[3] = {0, 0, 0};

    /* We send the register address from where we want to read
     * and then we read 2 bytes with dummy data
     */
    tx_data[0] = (uint8_t)((reg_addr << 1) & 0xFE);
    tx_data[1] = 0;
    tx_data[2] = 0;

    HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, sizeof(tx_data), 100) != HAL_OK) {
        HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_SET);
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(BGT60_CS_PORT, BGT60_CS_PIN, GPIO_PIN_SET);

    *data = ((uint16_t)(rx_data[1] << 8) | (uint16_t)(rx_data[2]));
    return HAL_OK;
}

unit8_t bgt60ltr11_spi_write(){

}
