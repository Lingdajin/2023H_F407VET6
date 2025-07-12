/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* 锟斤拷锟秸伙拷锟斤拷, 锟斤拷锟経SART_REC_LEN锟斤拷锟街斤拷. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  锟斤拷锟斤拷状态
 *  bit15锟斤拷      锟斤拷锟斤拷锟斤拷杀锟街�
 *  bit14锟斤拷      锟斤拷锟秸碉拷0x0d
 *  bit13~0锟斤拷    锟斤拷锟秸碉拷锟斤拷锟斤拷效锟街斤拷锟斤拷目
*/
uint16_t g_usart_rx_sta = 0;

uint8_t g_rx_buffer[RXBUFFERSIZE];  /* HAL锟斤拷使锟矫的达拷锟节斤拷锟秸伙拷锟斤拷 */
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief       锟斤拷锟斤拷锟斤拷锟捷斤拷锟秸回碉拷锟斤拷锟斤拷
                锟斤拷锟捷达拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
 * @param       huart:锟斤拷锟节撅拷锟�
 * @retval      锟斤拷
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)                      /* 锟斤拷锟斤拷谴锟斤拷锟�1 */
    {
        if ((g_usart_rx_sta & 0x8000) == 0)             /* 锟斤拷锟斤拷未锟斤拷锟� */
        {
            if (g_usart_rx_sta & 0x4000)                /* 锟斤拷锟秸碉拷锟斤拷0x0d锟斤拷锟斤拷锟截筹拷锟斤拷锟斤拷 */
            {
                if (g_rx_buffer[0] != 0x0a)             /* 锟斤拷锟秸碉拷锟侥诧拷锟斤拷0x0a锟斤拷锟斤拷锟斤拷锟角伙拷锟叫硷拷锟斤拷 */
                {
                    g_usart_rx_sta = 0;                 /* 锟斤拷锟秸达拷锟斤拷,锟斤拷锟铰匡拷始 */
                }
                else                                    /* 锟斤拷锟秸碉拷锟斤拷锟斤拷0x0a锟斤拷锟斤拷锟斤拷锟叫硷拷锟斤拷 */
                {
                    g_usart_rx_sta |= 0x8000;           /* 锟斤拷锟斤拷锟斤拷锟斤拷锟� */
                }
            }
            else                                        /* 锟斤拷没锟秸碉拷0X0d锟斤拷锟斤拷锟截筹拷锟斤拷锟斤拷 */
            {
                if (g_rx_buffer[0] == 0x0d)
                    g_usart_rx_sta |= 0x4000;
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0];
                    g_usart_rx_sta++;

                    if (g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;             /* 锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷,锟斤拷锟铰匡拷始锟斤拷锟斤拷 */
                    }
                }
            }
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
    }
}
/* USER CODE END 1 */
