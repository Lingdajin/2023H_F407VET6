/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define USART_REC_LEN               200         /* 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街斤拷锟斤拷 200 */
#define USART_EN_RX                 1           /* 使锟杰ｏ拷1锟斤拷/锟斤拷止锟斤拷0锟斤拷锟斤拷锟斤拷1锟斤拷锟斤拷 */
#define RXBUFFERSIZE   				1        	/* 锟斤拷锟斤拷锟叫� */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 锟斤拷锟秸伙拷锟斤拷,锟斤拷锟経SART_REC_LEN锟斤拷锟街斤拷.末锟街斤拷为锟斤拷锟叫凤拷 */
extern uint16_t g_usart_rx_sta;                 /* 锟斤拷锟斤拷状态锟斤拷锟� */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL锟斤拷USART锟斤拷锟斤拷Buffer */
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

