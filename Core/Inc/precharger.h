/**
  ******************************************************************************
  * @file    precharger.h
  * @brief   This file contains all the function prototypes for
  *          the precharver.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Rene Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRECHARGER_H__
#define __PRECHARGER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


#define PC_MAX_CHANNEL 3

typedef enum
{
	NO_PC,
	PC_RUN_MODE,
	PC_MODE_END
}_PRECHARGER_STATE;

/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	 uint8_t	time_index;				//10ms index of running pb
	 uint32_t	ch_ebable_mask;
	 uint8_t	ch_val[PC_MAX_CHANNEL];
	 //uint8_t	last_spi_buf[3];
 }_PC_CTRL_STRUCT;


void PreCharger_init(void);
void PreCharger_set(uint8_t channel, uint8_t value);
uint8_t	process_PreCharger(void);
void PC_OutputEnable(GPIO_PinState PinState);


#ifdef __cplusplus
}
#endif

#endif /* __PRECHARGER_H__ */

