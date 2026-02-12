/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PUMP1_MANUAL_Pin GPIO_PIN_2
#define PUMP1_MANUAL_GPIO_Port GPIOE
#define PUMP1_CHECK_Pin GPIO_PIN_3
#define PUMP1_CHECK_GPIO_Port GPIOE
#define PUMP1_PS_Pin GPIO_PIN_4
#define PUMP1_PS_GPIO_Port GPIOE
#define RS485_DE_Pin GPIO_PIN_13
#define RS485_DE_GPIO_Port GPIOC
#define RS485_RE_Pin GPIO_PIN_14
#define RS485_RE_GPIO_Port GPIOC
#define ADDR_DIP0_Pin GPIO_PIN_0
#define ADDR_DIP0_GPIO_Port GPIOC
#define ADDR_DIP1_Pin GPIO_PIN_1
#define ADDR_DIP1_GPIO_Port GPIOC
#define ADDR_DIP2_Pin GPIO_PIN_2
#define ADDR_DIP2_GPIO_Port GPIOC
#define ADDR_DIP3_Pin GPIO_PIN_3
#define ADDR_DIP3_GPIO_Port GPIOC
#define RUN_LED_Pin GPIO_PIN_0
#define RUN_LED_GPIO_Port GPIOA
#define ERR_LED_Pin GPIO_PIN_1
#define ERR_LED_GPIO_Port GPIOA
#define UART2_TXD_Pin GPIO_PIN_2
#define UART2_TXD_GPIO_Port GPIOA
#define UART2_RXD_Pin GPIO_PIN_3
#define UART2_RXD_GPIO_Port GPIOA
#define PUMP4_PS_Pin GPIO_PIN_4
#define PUMP4_PS_GPIO_Port GPIOA
#define PUMP4_CHECK_Pin GPIO_PIN_5
#define PUMP4_CHECK_GPIO_Port GPIOA
#define PUMP4_MANUAL_Pin GPIO_PIN_6
#define PUMP4_MANUAL_GPIO_Port GPIOA
#define PUMP4_STOP_Pin GPIO_PIN_7
#define PUMP4_STOP_GPIO_Port GPIOA
#define PUMP4_AUTO_Pin GPIO_PIN_4
#define PUMP4_AUTO_GPIO_Port GPIOC
#define PUMP5_PS_Pin GPIO_PIN_5
#define PUMP5_PS_GPIO_Port GPIOC
#define PUMP5_CHECK_Pin GPIO_PIN_0
#define PUMP5_CHECK_GPIO_Port GPIOB
#define PUMP5_MANUAL_Pin GPIO_PIN_7
#define PUMP5_MANUAL_GPIO_Port GPIOE
#define PUMP5_STOP_Pin GPIO_PIN_8
#define PUMP5_STOP_GPIO_Port GPIOE
#define PUMP5_AUTO_Pin GPIO_PIN_9
#define PUMP5_AUTO_GPIO_Port GPIOE
#define PUMP6_PS_Pin GPIO_PIN_10
#define PUMP6_PS_GPIO_Port GPIOE
#define PUMP6_CHECK_Pin GPIO_PIN_11
#define PUMP6_CHECK_GPIO_Port GPIOE
#define PUMP6_MANUAL_Pin GPIO_PIN_12
#define PUMP6_MANUAL_GPIO_Port GPIOE
#define PUMP6_STOP_Pin GPIO_PIN_13
#define PUMP6_STOP_GPIO_Port GPIOE
#define PUMP6_AUTO_Pin GPIO_PIN_14
#define PUMP6_AUTO_GPIO_Port GPIOE
#define HANJUN_LED_Pin GPIO_PIN_15
#define HANJUN_LED_GPIO_Port GPIOE
#define BALJUN_LED_Pin GPIO_PIN_10
#define BALJUN_LED_GPIO_Port GPIOB
#define SW_LOCK_ON_Pin GPIO_PIN_11
#define SW_LOCK_ON_GPIO_Port GPIOB
#define SW_LOCK_OFF_Pin GPIO_PIN_12
#define SW_LOCK_OFF_GPIO_Port GPIOB
#define TERMINATION_LED_Pin GPIO_PIN_13
#define TERMINATION_LED_GPIO_Port GPIOB
#define ROW0_Pin GPIO_PIN_10
#define ROW0_GPIO_Port GPIOD
#define ROW1_Pin GPIO_PIN_11
#define ROW1_GPIO_Port GPIOD
#define ROW2_Pin GPIO_PIN_12
#define ROW2_GPIO_Port GPIOD
#define ROW3_Pin GPIO_PIN_13
#define ROW3_GPIO_Port GPIOD
#define PUMP3_CHECK_Pin GPIO_PIN_6
#define PUMP3_CHECK_GPIO_Port GPIOC
#define PUMP3_AUTO_Pin GPIO_PIN_7
#define PUMP3_AUTO_GPIO_Port GPIOC
#define PUMP3_STOP_Pin GPIO_PIN_8
#define PUMP3_STOP_GPIO_Port GPIOC
#define PUMP3_MANUAL_Pin GPIO_PIN_9
#define PUMP3_MANUAL_GPIO_Port GPIOC
#define PUMP3_PS_Pin GPIO_PIN_8
#define PUMP3_PS_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOA
#define PUMP1_DIP1_Pin GPIO_PIN_10
#define PUMP1_DIP1_GPIO_Port GPIOC
#define PUMP2_DIP2_Pin GPIO_PIN_11
#define PUMP2_DIP2_GPIO_Port GPIOC
#define PUMP3_DIP3_Pin GPIO_PIN_12
#define PUMP3_DIP3_GPIO_Port GPIOC
#define PUMP4_DIP4_Pin GPIO_PIN_0
#define PUMP4_DIP4_GPIO_Port GPIOD
#define PUMP5_DIP5_Pin GPIO_PIN_1
#define PUMP5_DIP5_GPIO_Port GPIOD
#define PUMP6_DIP6_Pin GPIO_PIN_2
#define PUMP6_DIP6_GPIO_Port GPIOD
#define COLUMN0_Pin GPIO_PIN_3
#define COLUMN0_GPIO_Port GPIOD
#define COLUMN1_Pin GPIO_PIN_4
#define COLUMN1_GPIO_Port GPIOD
#define COLUMN2_Pin GPIO_PIN_5
#define COLUMN2_GPIO_Port GPIOD
#define COLUMN3_Pin GPIO_PIN_6
#define COLUMN3_GPIO_Port GPIOD
#define COLUMN4_Pin GPIO_PIN_7
#define COLUMN4_GPIO_Port GPIOD
#define PUMP2_AUTO_Pin GPIO_PIN_4
#define PUMP2_AUTO_GPIO_Port GPIOB
#define PUMP2_STOP_Pin GPIO_PIN_5
#define PUMP2_STOP_GPIO_Port GPIOB
#define PUMP2_MANUAL_Pin GPIO_PIN_6
#define PUMP2_MANUAL_GPIO_Port GPIOB
#define PUMP2_CHECK_Pin GPIO_PIN_7
#define PUMP2_CHECK_GPIO_Port GPIOB
#define PUMP2_PS_Pin GPIO_PIN_8
#define PUMP2_PS_GPIO_Port GPIOB
#define PUMP1_AUTO_Pin GPIO_PIN_0
#define PUMP1_AUTO_GPIO_Port GPIOE
#define PUMP1_STOP_Pin GPIO_PIN_1
#define PUMP1_STOP_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

void PUMP1_AUTO_LED_ON(void);
void PUMP1_AUTO_LED_OFF(void);
void PUMP1_STOP_LED_ON(void);
void PUMP1_STOP_LED_OFF(void);
void PUMP1_MANUAL_LED_ON(void);
void PUMP1_MANUAL_LED_OFF(void);
void PUMP1_CHECK_LED_ON(void);
void PUMP1_CHECK_LED_OFF(void);
void PUMP1_PS_LED_ON(void);
void PUMP1_PS_LED_OFF(void);

void PUMP2_AUTO_LED_ON(void);
void PUMP2_AUTO_LED_OFF(void);
void PUMP2_STOP_LED_ON(void);
void PUMP2_STOP_LED_OFF(void);
void PUMP2_MANUAL_LED_ON(void);
void PUMP2_MANUAL_LED_OFF(void);
void PUMP2_CHECK_LED_ON(void);
void PUMP2_CHECK_LED_OFF(void);
void PUMP2_PS_LED_ON(void);
void PUMP2_PS_LED_OFF(void);

void PUMP3_AUTO_LED_ON(void);
void PUMP3_AUTO_LED_OFF(void);
void PUMP3_STOP_LED_ON(void);
void PUMP3_STOP_LED_OFF(void);
void PUMP3_MANUAL_LED_ON(void);
void PUMP3_MANUAL_LED_OFF(void);
void PUMP3_CHECK_LED_ON(void);
void PUMP3_CHECK_LED_OFF(void);
void PUMP3_PS_LED_ON(void);
void PUMP3_PS_LED_OFF(void);

void PUMP4_AUTO_LED_ON(void);
void PUMP4_AUTO_LED_OFF(void);
void PUMP4_STOP_LED_ON(void);
void PUMP4_STOP_LED_OFF(void);
void PUMP4_MANUAL_LED_ON(void);
void PUMP4_MANUAL_LED_OFF(void);
void PUMP4_CHECK_LED_ON(void);
void PUMP4_CHECK_LED_OFF(void);
void PUMP4_PS_LED_ON(void);
void PUMP4_PS_LED_OFF(void);

void PUMP5_AUTO_LED_ON(void);
void PUMP5_AUTO_LED_OFF(void);
void PUMP5_STOP_LED_ON(void);
void PUMP5_STOP_LED_OFF(void);
void PUMP5_MANUAL_LED_ON(void);
void PUMP5_MANUAL_LED_OFF(void);
void PUMP5_CHECK_LED_ON(void);
void PUMP5_CHECK_LED_OFF(void);
void PUMP5_PS_LED_ON(void);
void PUMP5_PS_LED_OFF(void);

void PUMP6_AUTO_LED_ON(void);
void PUMP6_AUTO_LED_OFF(void);
void PUMP6_STOP_LED_ON(void);
void PUMP6_STOP_LED_OFF(void);
void PUMP6_MANUAL_LED_ON(void);
void PUMP6_MANUAL_LED_OFF(void);
void PUMP6_CHECK_LED_ON(void);
void PUMP6_CHECK_LED_OFF(void);
void PUMP6_PS_LED_ON(void);
void PUMP6_PS_LED_OFF(void);

void HANJUN_LED_ON(void);
void HANJUN_LED_OFF(void);
void BALJUN_LED_ON(void);
void BALJUN_LED_OFF(void);

void SW_LOCK_ON_LED_ON(void);
void SW_LOCK_ON_LED_OFF(void);
void SW_LOCK_OFF_LED_ON(void);
void SW_LOCK_OFF_LED_OFF(void);
void TERMINATION_LED_ON(void);
void TERMINATION_LED_OFF(void);

void ERR_LED_ON(void);
void ERR_LED_OFF(void);

void Read_Key(void);
void Read_Address(void);
void Read_Pump_Dip(void);

void Buzzer_ON(void);
void Buzzer_OFF(void);

void All_LED_ON(void);
void All_LED_OFF(void);

void SW_Com(void);
int SW_report_com(void);

void Save_Flash(uint32_t Address);
void Read_Flash(uint32_t ST_Address);

void SUB_Com_SW_V(void);

#define rx2_buf_len 40
extern uint8_t Uart_rx2_buf[rx2_buf_len] ;
extern uint8_t Uart_rx2_buf_tmp[3] ;
extern int rx2_State ;
extern int rx2_buf_count;
extern int rx2_buf_count_tmp;
extern int rx2_Receive_complete;

#define tx_buf_len 17
extern uint8_t Uart_tx_buf[tx_buf_len] ;

extern uint8_t Pre_Key_input[20];
extern uint8_t Key_input[20];
extern uint8_t Pre_Key_State;

extern uint8_t LED_State[35];
extern uint8_t LED_Pre_State[35];
extern uint8_t LED_State_tmp[35];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
