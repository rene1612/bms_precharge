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
	//HAL_StatusTypeDef result=HAL_OK;
	//uint8_t spi_buffer[3]={0x00,0x00,0x00};

	PC_OutputEnable(0);


	pc_ctrl.time_index = 0;
	pc_ctrl.ch_ebable_mask = 0x00;
	pc_ctrl.ch_replay_mask = 0x00;

	pc_ctrl.ch[0].GPIOx = RELAY_1_GPIO_Port;
	pc_ctrl.ch[0].GPIO_Pin = RELAY_1_Pin;
	pc_ctrl.ch[1].GPIOx = RELAY_2_GPIO_Port;
	pc_ctrl.ch[1].GPIO_Pin = RELAY_2_Pin;
	pc_ctrl.ch[2].GPIOx = RELAY_3_GPIO_Port;
	pc_ctrl.ch[2].GPIO_Pin = RELAY_3_Pin;

	//set pins to Low
}


uint8_t PreCharger_set(uint8_t channel, uint16_t value, uint8_t replay, _PRECHARGER_MODE mode)
{

	assert(channel<PC_MAX_CHANNEL);

	pc_ctrl.ch[channel].value = value;
	pc_ctrl.ch[channel].mode = mode;

	if(mode) {
		pc_ctrl.ch_ebable_mask |= 1<<channel;

		if(replay) {
			pc_ctrl.ch_replay_mask |= 1<<channel;
		}

		switch (mode) {

			case PC_PWM:


			case PC_ON:
				HAL_GPIO_WritePin(pc_ctrl.ch[channel].GPIOx, pc_ctrl.ch[channel].GPIO_Pin, GPIO_PIN_SET);
				break;

			case PC_OFF:
				HAL_GPIO_WritePin(pc_ctrl.ch[channel].GPIOx, pc_ctrl.ch[channel].GPIO_Pin, GPIO_PIN_RESET);
				break;

			default:
				break;

		}

		pc_ctrl.ch[channel].time_index = pc_ctrl.time_index;

		return 1;
	}

	return 0;
}



uint8_t	process_PreCharger(void)
{

	//uint8_t ch_mask=0;
	HAL_StatusTypeDef result=HAL_OK;

	if (pc_ctrl.ch_ebable_mask) {

		uint8_t ch;
		//uint8_t spi_buffer[3]={0x00,0x00,0x00};

		pc_ctrl.time_index++;

		for (ch=0; ch<PC_MAX_CHANNEL; ch++) {

			if (pc_ctrl.ch_ebable_mask & 1<<ch) {

				if (pc_ctrl.ch[ch].value==0) {
					pc_ctrl.ch_ebable_mask &= ~(1<<ch);
				}else {

					if (!(pc_ctrl.time_index >= pc_ctrl.ch[ch].value)) {
						//set
						//ch_mask |= (1<<(ch%8));
					}
				}
			}
		}

	}

	return result;
}
