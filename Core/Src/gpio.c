/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /* USER CODE BEGIN 2 */
  #if __BOARD_VERSION__ == 0x0100
	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, RELAY_4_Pin | RELAY_5_Pin | EXT_PA2_PIN | EXT_PA3_PIN | EXT_PA4_PIN | EXT_PA5_PIN | EXT_PA6_PIN | EXT_PA7_PIN | EXT_PA8_PIN, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin | LED_RED_Pin | RELAY_1_Pin | RELAY_2_Pin | RELAY_3_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
							   PAPin */
	  GPIO_InitStruct.Pin = RELAY_4_Pin |RELAY_5_Pin | EXT_PA2_PIN | EXT_PA3_PIN | EXT_PA4_PIN | EXT_PA5_PIN | EXT_PA6_PIN | EXT_PA7_PIN | EXT_PA8_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
	  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|RELAY_1_Pin|RELAY_2_Pin|RELAY_3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  #elif __BOARD_VERSION__ >= 0x0200
	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, RELAY_3_Pin|RELAY_4_Pin|RELAY_5_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : PAPin PAPin PAPin PAPin
							   PAPin */
	  GPIO_InitStruct.Pin = RELAY_3_Pin|RELAY_4_Pin|RELAY_5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
	  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|RELAY_1_Pin|RELAY_2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  #endif

  /* USER CODE END 2 */

}

