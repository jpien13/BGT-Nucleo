/*
 * bgt60ltr11_spi.h
 *
 *  Created on: Feb 26, 2025
 *      Author: jasonpien
 */

#ifndef INC_BGT60LTR11_SPI_H_
#define INC_BGT60LTR11_SPI_H_

#include "stdint.h"
#include "spi.h"

// Return HAL_OK or HAL_ERROR
uint8_t bgt60ltr11_spi_read(uint8_t reg_addr, uint16_t *data);
uint8_t bgt60ltr11_spi_write(uint8_t reg_addr, uint16_t data);
uint8_t bgt60ltr11_adc_status(void);
uint8_t bgt60ltr11_soft_reset(uint8_t wait);
uint8_t bgt60ltr11_pulsed_mode_init(void);
uint8_t bgt60ltr11_get_RAW_data(uint16_t *ifi, uint16_t *ifq);
uint8_t bgt60ltr11_HW_reset(void);


#endif /* INC_BGT60LTR11_SPI_H_ */
