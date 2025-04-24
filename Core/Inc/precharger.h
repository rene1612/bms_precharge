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


typedef enum
{
	PC_ON=1,
	PC_OFF,
	PC_ON_OFF,
	PC_PWM,
	PC_DONE,
}_PRECHARGER_MODE;

/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	 uint16_t			time_index;				//10ms index of running pb
	 uint16_t			value;
	 uint16_t			on_time;
	 uint16_t			off_time;
	 _PRECHARGER_MODE	mode;
	 GPIO_TypeDef 		*GPIOx;
	 uint16_t 			GPIO_Pin;

 }_PC_CHANNEL;


/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	 uint16_t		time_index;				//10ms index of running pb
	 uint8_t		ch_ebable_mask;
	 uint8_t		ch_replay_mask;
	 _PC_CHANNEL	ch[PC_MAX_CHANNEL];
 }_PC_CTRL_STRUCT;


void 	PreCharger_init(void);
uint8_t PreCharger_set(uint8_t channel, uint16_t value, uint8_t replay, _PRECHARGER_MODE mode);
uint8_t	process_PreCharger(void);
void 	PC_OutputEnable(GPIO_PinState PinState);


#ifdef __cplusplus
}
#endif

#endif /* __PRECHARGER_H__ */

