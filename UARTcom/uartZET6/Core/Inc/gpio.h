/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED0_port   GPIOB
#define LED0_pin    GPIO_PIN_5
#define LED0(x)     do{x ?\
  HAL_GPIO_WritePin(LED0_port, LED0_pin, GPIO_PIN_SET):\
  HAL_GPIO_WritePin(LED0_port, LED0_pin, GPIO_PIN_RESET);\
                    }while(0)
#define LED0_TOGGLE do{HAL_GPIO_TogglePin(LED0_port,LED0_pin);}while(0)
#define LED1_port   GPIOE
#define LED1_pin    GPIO_PIN_5
#define LED1(x)     do{x ?\
  HAL_GPIO_WritePin(LED1_port, LED1_pin, GPIO_PIN_SET):\
  HAL_GPIO_WritePin(LED1_port, LED1_pin, GPIO_PIN_RESET);\
                    }while(0)
#define LED1_TOGGLE do{HAL_GPIO_TogglePin(LED1_port,LED1_pin);}while(0)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
