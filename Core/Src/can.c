/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include <string.h>

#include "precharger.h"
#include "main.h"

//extern const _DEV_CONFIG_REGS* pDevConfig;

//ADS131M08_Can_Msg	ads_can_msg={};
CAN_TxHeaderTypeDef	TxHeader, ReplayHeader, AlertHeader, BroadcastHeader;
uint8_t				CanTxData[8];
uint32_t            TxMailbox;
uint8_t				can_task_scheduler;
CAN_RxHeaderTypeDef RxHeader;
uint8_t				can_replay_msg;
uint8_t             CanRxData[8];
//CAN_FilterTypeDef 	canfilterconfig;
//uint8_t				channels_2_send, current_channel_2_send;
//uint8_t				current_blk_data_2_send;
//_BMSM_ADC_DATA1		bmsm_adc_data[CHANNEL_COUNT];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = main_regs.dev_config.app_can_bitrate;
  //hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN CAN_Init 2 */
  can_task_scheduler = PROCESS_NO_TASK;

  main_regs.can_rx_cmd_id = (main_regs.dev_config.dev_id<<4) + CANRX_SA;
  main_regs.can_tx_data_id = (main_regs.dev_config.dev_id<<4) + CANTX_SA;
  main_regs.can_tx_heartbeat_id = (main_regs.dev_config.dev_id<<4) + CANTX_HA;
  main_regs.can_filterMask = RXFILTERMASK;
  main_regs.can_filterID = (main_regs.dev_config.dev_id<<4); // Only accept bootloader CAN message ID
  main_regs.can_rx_brdc_cmd_id = (main_regs.dev_config.can_broadcast_id<<4) + 0xF;

  TxHeader.DLC = 5;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = main_regs.can_tx_data_id;
  TxHeader.RTR = CAN_RTR_DATA;

  ReplayHeader.DLC = 2;
  ReplayHeader.IDE = CAN_ID_STD;
  ReplayHeader.StdId = main_regs.can_tx_data_id;
  ReplayHeader.RTR = CAN_RTR_DATA;

  AlertHeader.DLC = 2;
  AlertHeader.IDE = CAN_ID_STD;
  AlertHeader.StdId = main_regs.can_tx_data_id;
  AlertHeader.RTR = CAN_RTR_DATA;

  BroadcastHeader.DLC = 1;
  BroadcastHeader.IDE = CAN_ID_STD;
  BroadcastHeader.StdId = (main_regs.dev_config.can_broadcast_id<<4)+0xF;
  BroadcastHeader.RTR = CAN_RTR_DATA;

	/* config_can_filter ---------------------------------------------------------*/
	/* Setup Can-Filter                                                           */
   CAN_FilterTypeDef sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  //sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterBank = 13;

  sFilterConfig.FilterIdHigh = main_regs.can_filterID << 5;
  sFilterConfig.FilterIdLow = 0;

  sFilterConfig.FilterMaskIdHigh = main_regs.can_filterMask << 5;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */




uint8_t	process_CAN(void)
{
	//uint8_t ch;
	uint8_t len;
	uint16_t sys_reg, max_sys_reg_size;
	uint8_t* p_sys_reg_offset;





	if (can_task_scheduler & PROCESS_CAN_SEND_NEW_ALIVE_DATA)
	{
		CanTxData[0] = ALIVE_CMD;
		ReplayHeader.DLC = 1;

		can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_ALIVE_DATA;
		can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;

		//can_task_scheduler &= ~PROCESS_CAN_SEND_NEW_ADC_DATA;
		//return can_task_scheduler;
	}


	if (can_task_scheduler & PROCESS_CAN_ON_BRDC_MSG)
	{
		if (CanRxData[0] <= ALIVE_CMD) {
			can_task_scheduler |= PROCESS_CAN_ON_MSG;
		}
		can_task_scheduler &= ~PROCESS_CAN_ON_BRDC_MSG;
	}


	if (can_task_scheduler & PROCESS_CAN_ON_MSG)
	{
		switch (CanRxData[0])
		{
		case BMSP_SET_CMD:

			if (main_regs.ctrl & (1<<REG_CTRL_ENABLE_PC)) {

				if (PreCharger_set(CanRxData[1], (uint16_t)CanRxData[2], CanRxData[4], CanRxData[5])) {
					CanTxData[1] = ACK;
				}
				else {
					CanTxData[1] = NACK;
				}
			}
			else {
				CanTxData[1] = NACK;

			}

			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			ReplayHeader.DLC = 2;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;


		case BMSP_SET_EXT_OUT:

			//EXT_PA2
			if (CanRxData[1] & EXT_PA2)	{
				if (CanRxData[2] & EXT_PA2) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA2_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA2_PIN, GPIO_PIN_RESET);
				}
			}

			//EXT_PA3
			if (CanRxData[1] & EXT_PA3)	{
				if (CanRxData[2] & EXT_PA3) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA3_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA3_PIN, GPIO_PIN_RESET);
				}
			}

			//EXT_PA4
			if (CanRxData[1] & EXT_PA4)	{
				if (CanRxData[2] & EXT_PA4) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA4_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA4_PIN, GPIO_PIN_RESET);
				}
			}

			//EXT_PA5
			if (CanRxData[1] & EXT_PA5)	{
				if (CanRxData[2] & EXT_PA5) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA5_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA5_PIN, GPIO_PIN_RESET);
				}
			}

			//EXT_PA6
			if (CanRxData[1] & EXT_PA6)	{
				if (CanRxData[2] & EXT_PA6) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA6_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA6_PIN, GPIO_PIN_RESET);
				}
			}

			//EXT_PA7
			if (CanRxData[1] & EXT_PA7)	{
				if (CanRxData[2] & EXT_PA7) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA7_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA7_PIN, GPIO_PIN_RESET);
				}
			}

			//EXT_PA8
			if (CanRxData[1] & EXT_PA8)	{
				if (CanRxData[2] & EXT_PA8) {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA8_PIN, GPIO_PIN_SET);
				}
				else {
					HAL_GPIO_WritePin(EXT_PAX_GPIO_Port, EXT_PA8_PIN, GPIO_PIN_RESET);
				}
			}

			CanTxData[1] = ACK;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			ReplayHeader.DLC = 2;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		case ALIVE_CMD:
			alive_timer = main_regs.alive_timeout;
			break;



		case SYS_RESET_CMD:
			//printf("SYS_RESET\n");
			HAL_NVIC_SystemReset();
			break;

		case SYS_APP_RESET_CMD:
			//printf("APP_RESET\n");
			JumpToApp();
			break;

		case SYS_BOOT_CMD:
			//printf("SYS_BOOT\n");
			//leave a message in a bottle for the bootloader,
			//so the bootloader will not start app again and stay in bootloader-mode
			*(uint32_t *)_MAGIC_RAM_ADDRESS_ = _MAGIC_RAM_DWORD_;
			JumpToBtld();
			break;

		case SYS_READ_REG_CMD:
			//printf("ADC_READ_REG_CMD\n");
			sys_reg = (CanRxData[2]<<7)+CanRxData[3];

			len=CanRxData[4];

			if(!len || len >7)
				len=1;

			switch(CanRxData[1]) {
				case GET_SW_INFO_REGS:
					p_sys_reg_offset=(uint8_t *)&main_regs.sw_info;
					max_sys_reg_size=sizeof(_SW_INFO_REGS);
					break;

				case GET_DEV_CFG_REGS:
					p_sys_reg_offset=(uint8_t *)&main_regs.dev_config;
					max_sys_reg_size=sizeof(_DEV_CONFIG_REGS);
					break;

				case GET_BRD_INFO_REGS:
					p_sys_reg_offset=(uint8_t *)&main_regs.board_info;
					max_sys_reg_size=sizeof(_BOARD_INFO_STRUCT);
					break;

				case GET_MAIN_REGS:
				default:
					p_sys_reg_offset=(uint8_t *)&main_regs;
					max_sys_reg_size=sizeof(_MAIN_REGS)-sizeof(_SW_INFO_REGS)-sizeof(_DEV_CONFIG_REGS)-sizeof(_BOARD_INFO_STRUCT);
					break;
			}

			if ((sys_reg + 1) < max_sys_reg_size) {
				if ((sys_reg + len) >= max_sys_reg_size) {
					len = max_sys_reg_size - sys_reg;
				}
				CanTxData[0] = REPLAY_DATA_CMD;
				//CanTxData[1] = sys_reg;
				memcpy((uint8_t *)&CanTxData[1],(p_sys_reg_offset+sys_reg),len);
				//CanTxData[2] = *(((uint8_t *)&main_regs)+sys_reg);
				ReplayHeader.DLC = len+1;
			}
			else {
				CanTxData[0] = REPLAY_AKC_NACK_CMD;
				CanTxData[1] = NACK;
				ReplayHeader.DLC = 2;
			}

			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;

		case SYS_WRITE_REG_CMD:
			//printf("WRITE_REG_CMD\n");
			sys_reg = CanRxData[1];

			if (sys_reg < sizeof(main_regs))
			{
				*(((uint8_t *)&main_regs)+sys_reg) = CanRxData[2];
				CanTxData[1] = ACK;
			}
			else
			{
				CanTxData[1] = NACK;
			}
			ReplayHeader.DLC = 2;
			CanTxData[0] = REPLAY_AKC_NACK_CMD;
			can_task_scheduler |= PROCESS_CAN_SEND_REPLAY;
			break;


		default:
			break;
		}
		can_task_scheduler &= ~PROCESS_CAN_ON_MSG;
	}


	if (can_task_scheduler & PROCESS_CAN_SEND_REPLAY)
	{
		if (!HAL_CAN_IsTxMessagePending(&hcan, TxMailbox))
		{
			if (HAL_CAN_AddTxMessage(&hcan, &ReplayHeader, CanTxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

			set_signal_led(CAN_LED, LED_100MS_FLASH);
			can_task_scheduler &= ~PROCESS_CAN_SEND_REPLAY;
		}
	}

	return can_task_scheduler;
}



/* HAL_CAN_RxCallback -----------------------------------------------------*/
/* Interrupt callback to manage Can Receive                                */
/*-------------------------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRxData) != HAL_OK){
		Error_Handler();
	}

	if ((RxHeader.StdId == main_regs.can_rx_cmd_id)){
		can_task_scheduler |= PROCESS_CAN_ON_MSG;
    	main_task_scheduler |= PROCESS_CAN;
	}else if (RxHeader.StdId == main_regs.can_rx_brdc_cmd_id) {
		can_task_scheduler |= PROCESS_CAN_ON_BRDC_MSG;
    	main_task_scheduler |= PROCESS_CAN;
	}
}


/* HAL_CAN_ErrorCallback -----------------------------------------------------*/
/* Interrupt callback to manage Can Errors                                    */
/*----------------------------------------------------------------------------*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *phcan){
	//uint32_t transmitmailbox;
	int retry=0;

	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX0);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST1) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX1);
		retry=1;
	}
	if((phcan->ErrorCode & HAL_CAN_ERROR_TX_ALST2) || (phcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2)){
		HAL_CAN_AbortTxRequest(phcan,CAN_TX_MAILBOX2);
		retry=1;
	}

	HAL_CAN_ResetError(phcan);

	if(retry==1){
		HAL_CAN_DeactivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(phcan,CAN_IT_TX_MAILBOX_EMPTY);
		//HAL_CAN_AddTxMessage(phcan,&(CanTxList.SendMsgBuff.header),CanTxList.SendMsgBuff.Data,&transmitmailbox);
	}
}

/* HAL_CAN_TxCpltCallback ----------------------------------------------------*/
/* Interrupt callback to manage Can Tx Ready                                  */
/*----------------------------------------------------------------------------*/
void CAN_TX_Cplt(CAN_HandleTypeDef* phcan){

}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* phcan){
	CAN_TX_Cplt(phcan);
}

/* send_broadcast_msg ---------------------------------------------------------*/
/*                                                                            */
/*---------------------------------------------------------------------------*/
void can_send_brdc_msg(uint8_t* p_msg, uint8_t len){

	assert(len<8);
	//CanTxData[0] = SYS_ALLERT_MSG;
	memcpy(CanTxData, p_msg, len);
	AlertHeader.DLC=len;

	HAL_CAN_AddTxMessage(&hcan, &BroadcastHeader, CanTxData, &TxMailbox);

	set_signal_led(CAN_LED, LED_100MS_FLASH);
}


/* send_allert_msg ---------------------------------------------------------*/
/*                                                                           */
/*----------------------------------------------------------------------------*/
void can_send_alert_msg(uint8_t* p_allert_msg, uint8_t len){

	assert(len<7);
	CanTxData[0] = SYS_ALERT_MSG;
	memcpy(CanTxData+1, p_allert_msg, len);
	AlertHeader.DLC=len+1;

	HAL_CAN_AddTxMessage(&hcan, &AlertHeader, CanTxData, &TxMailbox);

	set_signal_led(CAN_LED, LED_100MS_FLASH);
}

/* send_allert_msg ---------------------------------------------------------*/
/*                                                                           */
/*----------------------------------------------------------------------------*/
void can_send_warn_msg(uint8_t* p_warn_msg, uint8_t len){

	assert(len<7);
	CanTxData[0] = SYS_WARN_MSG;
	memcpy(CanTxData+1, p_warn_msg, len);
	AlertHeader.DLC=len+1;

	HAL_CAN_AddTxMessage(&hcan, &AlertHeader, CanTxData, &TxMailbox);

	set_signal_led(CAN_LED, LED_100MS_FLASH);
}


/* USER CODE END 1 */
