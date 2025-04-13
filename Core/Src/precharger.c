/**
  ******************************************************************************
  * @file    passive_balance.c
  * @brief   This file provides code for the ctrl an rading of the passive-balancer
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 René Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "precharger.h"
#include "string.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"



_PC_CTRL_STRUCT pc_ctrl;


void PC_OutputEnable(GPIO_PinState PinState) {
	//HAL_GPIO_WritePin(SPI1_OE_GPIO_Port, SPI1_OE_Pin, !PinState);
}

void PreCharger_init(void)
{
	HAL_StatusTypeDef result=HAL_OK;
	//uint8_t spi_buffer[3]={0x00,0x00,0x00};

	PC_OutputEnable(0);


	pc_ctrl.time_index = 0;
	pc_ctrl.ch_ebable_mask = 0U;

	//set pins to Low
}


void PreCharger_set(uint8_t channel, uint8_t value)
{

	assert(channel<PC_MAX_CHANNEL);

	pc_ctrl.ch_val[channel] = value;

	if(value) {
		pc_ctrl.ch_ebable_mask |= 1<<channel;
	}
}



uint8_t	process_PreCharger(void)
{

	//uint8_t ch_mask=0;

	if (pc_ctrl.ch_ebable_mask) {

		uint8_t ch;
		//uint8_t spi_buffer[3]={0x00,0x00,0x00};
		HAL_StatusTypeDef result=HAL_OK;

		pc_ctrl.time_index++;

		for (ch=0; ch<PC_MAX_CHANNEL; ch++) {

			if (pc_ctrl.ch_ebable_mask & 1<<ch) {

				if (pc_ctrl.ch_val[ch]==0) {
					pc_ctrl.ch_ebable_mask &= ~(1<<ch);
				}else {

					if (!(pc_ctrl.time_index >= pc_ctrl.ch_val[ch])) {
						//set
						//ch_mask |= (1<<(ch%8));
					}
				}
			}
		}

	}

	return 0;
}
