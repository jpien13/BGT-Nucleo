/*
 * spi.h
 *
 *  Created on: Feb 26, 2025
 *      Author: jasonpien
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif


#endif /* INC_SPI_H_ */
