/*
 * bgt60ltr11_spi.c
 *
 *  Created on: Feb 26, 2025
 *      Author: jasonpien
 */

#include "bgt60ltr11_spi.h"
#include "main.h"

#define RADAR1_CS_PORT GPIOA
#define RADAR1_CS_PIN  GPIO_PIN_4
#define RADAR2_CS_PORT GPIOA
#define RADAR2_CS_PIN  GPIO_PIN_10

extern void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
#define LD3_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_3
extern SPI_HandleTypeDef hspi1;

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
 *
 *Arbitration Protocol - Follow this sequence for each access:
    * Drive SPICSN high for ≥100 ns
    * Wait until SPIDO is low
    * Drive SPICSN low to reserve the bus
    * Wait ≥100 ns for SPICSN synchronization
    * Check SPIDO again - if high, restart from step 1
    * Perform the data transmission
    * Drive SPICSN high to release the bus
 *
 */
uint8_t bgt60ltr11_spi_read(RadarId_t radar_id, uint8_t reg_addr, uint16_t *data) {
    uint8_t tx_data[3];
    uint8_t rx_data[3] = {0, 0, 0};

    /* We send the register address from where we want to read
     * and then we read 2 bytes with dummy data
     */
    tx_data[0] = (uint8_t)((reg_addr << 1) & 0xFE); //  shifts the 7-bit address to make room for the RW bit. Address (7 bits) + RW bit (0)
    tx_data[1] = 0; 								// Dummy byte (ignored by radar)
    tx_data[2] = 0; 								// Dummy byte (ignored by radar)

    uint8_t retry_count = 0;
    const uint8_t MAX_RETRIES = 5;

    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    if (radar_id == RADAR_1) {
		cs_port = RADAR1_CS_PORT;
		cs_pin = RADAR1_CS_PIN;
	} else {
		cs_port = RADAR2_CS_PORT;
		cs_pin = RADAR2_CS_PIN;
	}

    // Arbitration protocol (Page 18 of https://www.infineon.com/dgdl/Infineon-UG124434_User_guide_to_BGT60LTR11AIP-UserManual-v01_80-EN.pdf?fileId=8ac78c8c8823155701885724e6d72f8f)
    // Note: If you block the SPI interface for too long (by keeping CS low), you disrupt the sequencer's timing

    while (retry_count < MAX_RETRIES) {
    	// 1. Initially SPICSN is driven high. Minimum for ≥100 ns
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
		// Small delay to ensure we meet the ≥100 ns requirement
		for(volatile uint8_t i = 0; i < 10; i++);

		// 2. Wait until SPIDO is low (high means the internal sequencer is using the SPI)
		//     - That should not take long. However, if something is completely broken a timeout may help
		//       in detecting such issues. Skip this step if “miso_drv”=0!
		// In a multi-radar setup, this checks if the internal sequencer is using SPI
		// Could implement a timeout here for robustness
		uint32_t timeout = HAL_GetTick() + 5; // 5ms timeout
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) { // Assuming MISO is on PA6
			if(HAL_GetTick() > timeout) {
				retry_count++;
				continue; // Go back to step 1
			}
		}

		// 3. Drive SPICSN low (try to reserve the bus)
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

		// 4. Wait for ≥100 ns (the time needed for synchronization of SPICSN)
		for(volatile uint8_t i = 0; i < 10; i++);

		// 5. Check SPIDO again. If it is high now (sequencer has just started SPI also), go to step 1
		//		- Otherwise, continue to step 6
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
			retry_count++;
			continue; // Go back to start of the loop (step 1)
		}

		// 6. Perform the data transmission
		if(HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, sizeof(tx_data), 100) != HAL_OK) {
			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
			return HAL_ERROR;
		}

		// 7. Drive SPICSN high to release the bus
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

		// Success! Process the data and return
		// After transmission, the 16-bit register value is reconstructed from the received bytes:
		*data = ((uint16_t)(rx_data[1] << 8) | (uint16_t)(rx_data[2]));
		return HAL_OK;
    }
    // If we reach here, exceeded the retry limit currently set as 5
    return HAL_ERROR;
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
uint8_t bgt60ltr11_spi_write(RadarId_t radar_id, uint8_t reg_addr, uint16_t data){
	uint8_t tx_data[3];
	uint8_t retry_count = 0;
	const uint8_t MAX_RETRIES = 5;
	uint16_t wrdata = data;

	GPIO_TypeDef* cs_port;
	uint16_t cs_pin;

	if (radar_id == RADAR_1) {
		cs_port = RADAR1_CS_PORT;
		cs_pin = RADAR1_CS_PIN;
	} else {
		cs_port = RADAR2_CS_PORT;
		cs_pin = RADAR2_CS_PIN;
	}

	tx_data[0] = (uint8_t)((reg_addr << 1) | 0x01); // Shifts the 7-bit address to make room for the RW bit. Address (7 bits) + RW bit (1)
	tx_data[1] = (uint8_t)((wrdata >> 8) & 0xFF);   // Upper 8 bits of data (MSB first)
	tx_data[2] = (uint8_t)(wrdata & 0xFF);			// Lower 8 bits of data


	// Arbitration protocol
	while (retry_count < MAX_RETRIES) {
		// 1. Initially SPICSN is high
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

		// Delay for ≥100 ns
		for(volatile uint8_t i = 0; i < 10; i++);

		// 2. Wait until SPIDO is low (when miso_drv=1)
		// Assuming MISO is on PA6 - adjust to your actual pin
		uint32_t timeout = HAL_GetTick() + 5; // 5ms timeout
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
			if(HAL_GetTick() > timeout) {
				retry_count++;
				continue; // Go back to step 1
			}
		}

		// 3. Drive SPICSN low to reserve the bus
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

		// 4. Wait for ≥100 ns for SPICSN synchronization
		for(volatile uint8_t i = 0; i < 10; i++);

		// 5. Check SPIDO again - if high, go back to step 1
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
			// Release the bus and try again
			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
			retry_count++;
			continue; // Go back to start of the loop (step 1)
		}

		// 6. Perform the data transmission
		if(HAL_SPI_Transmit(&hspi1, tx_data, sizeof(tx_data)/sizeof(uint8_t), 100) != HAL_OK) {
			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
			return HAL_ERROR;
		}

		// 7. Drive SPICSN high to release the bus
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

		// Success!
		return HAL_OK;
	}

	// If we reach here, we've exceeded our retry limit
	return HAL_ERROR;

}

uint8_t bgt60ltr11_HW_reset(RadarId_t radar_id){
	GPIO_TypeDef* cs_port;
	uint16_t cs_pin;

	if (radar_id == RADAR_1) {
		cs_port = RADAR1_CS_PORT;
		cs_pin = RADAR1_CS_PIN;
	} else {
		cs_port = RADAR2_CS_PORT;
		cs_pin = RADAR2_CS_PIN;
	}
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	return HAL_OK;
}

/*
 * Register assignment of Reg36
 * Page 45 https://www.infineon.com/dgdl/Infineon-UG124434_User_guide_to_BGT60LTR11AIP-UserManual-v01_80-EN.pdf?fileId=8ac78c8c8823155701885724e6d72f8f
 */
uint8_t bgt60ltr11_adc_status(RadarId_t radar_id){
	uint16_t adc_status;
	if(bgt60ltr11_spi_read(radar_id, 0x24, &adc_status) != HAL_OK){
		return HAL_ERROR;
	}
	if(adc_status ==0){
		return HAL_ERROR;
	}
	return HAL_OK;
}

/*
 *  Resampling can be triggered by setting the reset pin or activating the soft reset by writing the soft_reset
 *  bit (Reg15[15]).
 *  There are 56 Registers according to register overview on page 21
 *  Page 6 https://www.infineon.com/dgdl/Infineon-UG124434_User_guide_to_BGT60LTR11AIP-UserManual-v01_80-EN.pdf?fileId=8ac78c8c8823155701885724e6d72f8f
 */
uint8_t bgt60ltr11_soft_reset(RadarId_t radar_id, uint8_t wait){
	bgt60ltr11_spi_write(radar_id, 0x0F, (1 << 15));
	uint16_t reg56 = 0;
	uint16_t reg0 = 0;

	if (wait){
		// wait till init_done in REG56 is set
		for (volatile uint16_t i = 0; i < 2048; i++){
			bgt60ltr11_spi_read(radar_id, 0x38, &reg56);
			bgt60ltr11_spi_read(radar_id, 0x00, &reg0);
			// check if REG0 has default values and REG56 bit init_done is set
			if (reg0 == 0 && reg56 & (1 << 13)){
				return HAL_OK;
			}
			HAL_Delay(1);
		}
		return HAL_ERROR;
	}
	return HAL_OK;
}

/*
 * Convert all ADC channels
 * A write access to Reg35 starts ADC conversion with the selected settings, even if the same data is written into
 * the register.
 * Address: 0x23
 * reset value: 0x0000
 */
uint8_t bgt60ltr11_ADC_Convert(RadarId_t radar_id){
	if (bgt60ltr11_spi_write(radar_id, 0x23, 0010) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	return HAL_OK;
}

/*
 * Test if SPI works by reading reg 0x02 and verify value is 0x2A00
 */
uint8_t bgt60ltr11_test(RadarId_t radar_id){
	uint16_t data = 0;
	if (bgt60ltr11_spi_read(radar_id, 0x02, &data) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if (data != 0x2A00){
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		return HAL_ERROR;
	}
	return HAL_OK;

}

/*
 * Read ADC channel data directly into the provided pointers
 */
uint8_t bgt60ltr11_get_RAW_data(RadarId_t radar_id, uint16_t *ifi, uint16_t *ifq){
	if (bgt60ltr11_spi_read(radar_id, 0x28, ifi) != HAL_OK) return HAL_ERROR;
	if (bgt60ltr11_spi_read(radar_id, 0x29, ifq) != HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

uint8_t bgt60ltr11_pulsed_mode_init(RadarId_t radar_id) {
	// Perform soft reset
	if (bgt60ltr11_soft_reset(radar_id, 0) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

    // Set miso_drv bit (Reg15[6]) to 1, critical for making SPIDO available for arbitration
    if (bgt60ltr11_spi_write(radar_id, 0x0F, 0x0040) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1);


	// Write to each register and check the result

	if (bgt60ltr11_spi_write(radar_id, 0x00, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x01, 0x0000) != HAL_OK) return HAL_ERROR;
	    HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x02, 0x2A00) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	// TODO need to check the value for the REG3

	if (bgt60ltr11_spi_write(radar_id, 0x04, 0x0F3A) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x05, 0x0FB0) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x06, 0x6800) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x07, 0x0557) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x08, 0x000E) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x09, 0x00E8) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x0A, 0x004F) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x0C, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x0D, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x0E, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x0F, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x22, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x23, 0x0000) != HAL_OK) return HAL_ERROR;
	    HAL_Delay(1);

	if (bgt60ltr11_spi_write(radar_id, 0x24, 0x0000) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);
	/*
	// ADC clock EN, bandgap EN, ADC EN
	if (bgt60ltr11_spi_write(0x22, 0x0007) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);
	*/
	if (bgt60ltr11_spi_write(radar_id, 0x0F, 0x4040) != HAL_OK) return HAL_ERROR;
	HAL_Delay(1);



	return HAL_OK;
}

uint8_t bgt60ltr11_pulsed_mode_init_extended_range(RadarId_t radar_id) {
    // First call the original initialization
    if (bgt60ltr11_pulsed_mode_init(radar_id) != HAL_OK) return HAL_ERROR;

    // Then apply range enhancements
    if (bgt60ltr11_set_max_range(radar_id) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

uint8_t bgt60ltr11_set_max_range(RadarId_t radar_id) {
    // 1. Set a low detector threshold for higher sensitivity (66 is the minimum recommended value)
    if (bgt60ltr11_spi_write(radar_id, 0x02, 66) != HAL_OK) return HAL_ERROR;

    // 2. Set maximum MPA gain (Reg7[2:0] = 7) to 4.5 dBm
    uint16_t reg7_value;
    if (bgt60ltr11_spi_read(radar_id, 0x07, &reg7_value) != HAL_OK) return HAL_ERROR;
    reg7_value = (reg7_value & ~0x0007) | 0x0007; // Set bits [2:0] to 111 (max gain)
    if (bgt60ltr11_spi_write(radar_id, 0x07, reg7_value) != HAL_OK) return HAL_ERROR;

    // 3. Set maximum baseband PGA gain to 50 dB (Reg9[3:0] = 8)
    uint16_t reg9_value;
    if (bgt60ltr11_spi_read(radar_id, 0x09, &reg9_value) != HAL_OK) return HAL_ERROR;
    reg9_value = (reg9_value & ~0x000F) | 0x0008; // Set bits [3:0] to 1000 (50 dB)
    if (bgt60ltr11_spi_write(radar_id, 0x09, reg9_value) != HAL_OK) return HAL_ERROR;

    uint16_t reg2_value;
    if (bgt60ltr11_spi_read(radar_id, 0x02, &reg2_value) != HAL_OK) return HAL_ERROR;
    reg2_value |= (1 << 15); // Set HPRT bit
    if (bgt60ltr11_spi_write(radar_id, 0x02, reg2_value) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}
