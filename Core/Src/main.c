/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Compile_Data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

uint8_t SW_Version = 0x0a ;//version 10 -> 1.0

uint8_t MCC_Address_tmp[4] ;
uint8_t MCC_Address[1] ;

uint8_t Pre_Key_input[20];
uint8_t Key_input[20];
/*
key input	0	->	PUMP1_SW_MAN
key input	1	->	PUMP1_SW_STOP
key input	2	->	PUMP1_SW_AUTO
key input	3	->	PUMP2_SW_MAN
key input	4	->	PUMP2_SW_STOP
key input	5	->	PUMP2_SW_AUTO
key input	6	->	PUMP3_SW_MAN
key input	7	->	PUMP3_SW_STOP
key input	8	->	PUMP3_SW_AUTO
key input	9	->	PUMP4_SW_MAN
key input	10	->	PUMP4_SW_STOP
key input	11	->	PUMP4_SW_AUTO
key input	12	->	PUMP5_SW_MAN
key input	13	->	PUMP5_SW_STOP
key input	14	->	PUMP5_SW_AUTO
key input	15	->	PUMP6_SW_MAN
key input	16	->	PUMP6_SW_STOP
key input	17	->	PUMP1_SW_AUTO
key input	18	->	SW_LOCK
key input	19	->	FORCED_TERMINATION
*/

uint8_t Pump_Dip[6];
uint8_t Address_Dip[4];
uint8_t Pre_Key_State = 0;
uint8_t LED_State[35];
uint8_t LED_Pre_State[35];
uint8_t LED_State_tmp[35];

uint8_t Uart_tmp[] = "Test\r\n";

uint8_t Uart_rx2_buf[rx2_buf_len] ;
uint8_t Uart_rx2_buf_tmp[3] ;
int rx2_State ;
int rx2_buf_count;
int rx2_buf_count_tmp;
int rx2_Receive_complete;

uint8_t Uart_tx_buf[tx_buf_len] ;

uint8_t UI_PUMP1_SW_MAN[6],UI_PUMP1_SW_STOP[6],UI_PUMP1_SW_AUTO[6];

uint8_t  Key_State = 0;

#define  buf_len_flash 32
#define FLASH_ROW_SIZE          32

/* !!! Be careful the user area should be in another bank than the code !!! */
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_246   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_255 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */

#define FLASH_ST1_ADDR   ADDR_FLASH_PAGE_255   /* Start @ of user Flash area */


#define DATA_64                 ((uint64_t)0x1234567812345678)
#define DATA_32                 ((uint32_t)0x12345678)

uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

uint8_t Type_Mode ;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;


static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);

#define tx_Q_buf_len 18
uint8_t Uart_tx_Q_buf[tx_Q_buf_len] ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////Start timer interrupt operation function///////////////////

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1){
	  HAL_IWDG_Refresh(&hiwdg);
  }

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


  /* huart2 RX Interrupt  Enable */
  /* Process Unlocked */
  __HAL_UNLOCK(&huart2);
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
  //HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);

  Buzzer_ON();
  //HAL_Delay(300);
  HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
  //Buzzer_OFF();

  Read_Address();

  MCC_Address_tmp[3] =  ~(HAL_GPIO_ReadPin(ADDR_DIP0_GPIO_Port, ADDR_DIP0_Pin)) & 0x01;
  MCC_Address_tmp[2] =  ~(HAL_GPIO_ReadPin(ADDR_DIP1_GPIO_Port, ADDR_DIP1_Pin)) & 0x01;
  MCC_Address_tmp[1] =  ~(HAL_GPIO_ReadPin(ADDR_DIP2_GPIO_Port, ADDR_DIP2_Pin)) & 0x01;
  MCC_Address_tmp[0] =  ~(HAL_GPIO_ReadPin(ADDR_DIP3_GPIO_Port, ADDR_DIP3_Pin)) & 0x01;

 // MCC_Address[0] = (MCC_Address_tmp[3] << 3)|(MCC_Address_tmp[2] << 2)|(MCC_Address_tmp[1] << 1)|(MCC_Address_tmp[0] << 0);
  MCC_Address[0] = (MCC_Address_tmp[2] << 2)|(MCC_Address_tmp[1] << 1)|(MCC_Address_tmp[0] << 0);


  Type_Mode = MCC_Address_tmp[3]; // 1 Control , 0 Repeater

  if(MCC_Address[0] == 0){
    PUMP1_AUTO_LED_ON();
    PUMP1_STOP_LED_ON();
    PUMP1_MANUAL_LED_ON();
    PUMP1_CHECK_LED_ON();
    PUMP1_PS_LED_ON();

    PUMP2_AUTO_LED_ON();
    PUMP2_STOP_LED_ON();
    PUMP2_MANUAL_LED_ON();
    PUMP2_CHECK_LED_ON();
    PUMP2_PS_LED_ON();

    PUMP3_AUTO_LED_ON();
    PUMP3_STOP_LED_ON();
    PUMP3_MANUAL_LED_ON();
    PUMP3_CHECK_LED_ON();
    PUMP3_PS_LED_ON();

    PUMP4_AUTO_LED_ON();
    PUMP4_STOP_LED_ON();
    PUMP4_MANUAL_LED_ON();
    PUMP4_CHECK_LED_ON();
    PUMP4_PS_LED_ON();

    PUMP5_AUTO_LED_ON();
    PUMP5_STOP_LED_ON();
    PUMP5_MANUAL_LED_ON();
    PUMP5_CHECK_LED_ON();
    PUMP5_PS_LED_ON();

    PUMP6_AUTO_LED_ON();
    PUMP6_STOP_LED_ON();
    PUMP6_MANUAL_LED_ON();
    PUMP6_CHECK_LED_ON();
    PUMP6_PS_LED_ON();

    HANJUN_LED_ON();
    BALJUN_LED_ON();

    SW_LOCK_ON_LED_ON();
    SW_LOCK_ON_LED_ON();
    TERMINATION_LED_ON();

    SW_LOCK_ON_LED_ON();

    HAL_Delay(10000);

    PUMP1_AUTO_LED_OFF();
    PUMP1_STOP_LED_OFF();
    PUMP1_MANUAL_LED_OFF();
    PUMP1_CHECK_LED_OFF();
    PUMP1_PS_LED_OFF();

    PUMP2_AUTO_LED_OFF();
    PUMP2_STOP_LED_OFF();
    PUMP2_MANUAL_LED_OFF();
    PUMP2_CHECK_LED_OFF();
    PUMP2_PS_LED_OFF();

    PUMP3_AUTO_LED_OFF();
    PUMP3_STOP_LED_OFF();
    PUMP3_MANUAL_LED_OFF();
    PUMP3_CHECK_LED_OFF();
    PUMP3_PS_LED_OFF();

    PUMP4_AUTO_LED_OFF();
    PUMP4_STOP_LED_OFF();
    PUMP4_MANUAL_LED_OFF();
    PUMP4_CHECK_LED_OFF();
    PUMP4_PS_LED_OFF();

    PUMP5_AUTO_LED_OFF();
    PUMP5_STOP_LED_OFF();
    PUMP5_MANUAL_LED_OFF();
    PUMP5_CHECK_LED_OFF();
    PUMP5_PS_LED_OFF();

    PUMP6_AUTO_LED_OFF();
    PUMP6_STOP_LED_OFF();
    PUMP6_MANUAL_LED_OFF();
    PUMP6_CHECK_LED_OFF();
    PUMP6_PS_LED_OFF();

    HANJUN_LED_OFF();
    BALJUN_LED_OFF();

    SW_LOCK_OFF_LED_OFF();
    SW_LOCK_OFF_LED_OFF();
    TERMINATION_LED_OFF();

    SW_LOCK_OFF_LED_OFF();
  }

  LED_State[0] = 0 ;//-> PUMP1_AUTO
  LED_State[1] = 1 ;//-> PUMP1_STOP
  LED_State[2] = 0 ;//-> PUMP1_MANUAL
  LED_State[3] = 0 ;//-> PUMP1_CHECK
  LED_State[4] = 0 ;//-> PUMP1_PS

  LED_State[5] = 0 ;//-> PUMP2_AUTO
  LED_State[6] = 1 ;//-> PUMP2_STOP
  LED_State[7] = 0 ;//-> PUMP2_MANUAL
  LED_State[8] = 0 ;//-> PUMP2_CHECK
  LED_State[9] = 0 ;//-> PUMP2_PS

  LED_State[10] = 0 ;//-> PUMP3_AUTO
  LED_State[11] = 1 ;//-> PUMP3_STOP
  LED_State[12] = 0 ;//-> PUMP3_MANUAL
  LED_State[13] = 0 ;//-> PUMP3_CHECK
  LED_State[14] = 0 ;//-> PUMP3_PS

  LED_State[15] = 0 ;//-> PUMP4_AUTO
  LED_State[16] = 1 ;//-> PUMP4_STOP
  LED_State[17] = 0 ;//-> PUMP4_MANUAL
  LED_State[18] = 0 ;//-> PUMP4_CHECK
  LED_State[19] = 0 ;//-> PUMP4_PS

  LED_State[20] = 0 ;//-> PUMP5_AUTO
  LED_State[21] = 1 ;//-> PUMP5_STOP
  LED_State[22] = 0 ;//-> PUMP5_MANUAL
  LED_State[23] = 0 ;//-> PUMP5_CHECK
  LED_State[24] = 0 ;//-> PUMP5_PS

  LED_State[25] = 0 ;//-> PUMP6_AUTO
  LED_State[26] = 1 ;//-> PUMP6_STOP
  LED_State[27] = 0 ;//-> PUMP6_MANUAL
  LED_State[28] = 0 ;//-> PUMP6_CHECK
  LED_State[29] = 0 ;//-> PUMP6_PS

  LED_State[30] = 0 ;//-> HANJUN5
  LED_State[31] = 0 ;//-> BALJUN
  LED_State[32] = 0 ;//-> SW_LOCK_ON
  LED_State[33] = 1 ;//-> SW_LOCK_OFF
  LED_State[34] = 0 ;//-> TERMINATION
/*
  Read_Flash(FLASH_ST1_ADDR);

  for( int j=0; j< 32; j++){
    LED_State[j] = LED_State_tmp[j];
  }
*/

  PUMP1_AUTO_LED_OFF();
  PUMP1_STOP_LED_OFF();
  PUMP1_MANUAL_LED_OFF();
  PUMP1_CHECK_LED_OFF();
  PUMP1_PS_LED_OFF();

  PUMP2_AUTO_LED_OFF();
  PUMP2_STOP_LED_OFF();
  PUMP2_MANUAL_LED_OFF();
  PUMP2_CHECK_LED_OFF();
  PUMP2_PS_LED_OFF();

  PUMP3_AUTO_LED_OFF();
  PUMP3_STOP_LED_OFF();
  PUMP3_MANUAL_LED_OFF();
  PUMP3_CHECK_LED_OFF();
  PUMP3_PS_LED_OFF();

  PUMP4_AUTO_LED_OFF();
  PUMP4_STOP_LED_OFF();
  PUMP4_MANUAL_LED_OFF();
  PUMP4_CHECK_LED_OFF();
  PUMP4_PS_LED_OFF();

  PUMP5_AUTO_LED_OFF();
  PUMP5_STOP_LED_OFF();
  PUMP5_MANUAL_LED_OFF();
  PUMP5_CHECK_LED_OFF();
  PUMP5_PS_LED_OFF();

  PUMP6_AUTO_LED_OFF();
  PUMP6_STOP_LED_OFF();
  PUMP6_MANUAL_LED_OFF();
  PUMP6_CHECK_LED_OFF();
  PUMP6_PS_LED_OFF();

  HANJUN_LED_OFF();
  BALJUN_LED_OFF();

  SW_LOCK_ON_LED_OFF();
  SW_LOCK_OFF_LED_OFF();
  TERMINATION_LED_OFF();

  SW_LOCK_OFF_LED_ON();

  All_LED_ON();

  Compile_Date();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	  /* Enable the UART Data Register not empty Interrupt */
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	    Read_Key();

	    if(rx2_Receive_complete == 1){
	      SW_Com();
	      for(int j = 0 ; j < 28; j++){
	        if(LED_Pre_State[j] == LED_State[j]){
	        }
	        else{
	          //Save_Flash(FLASH_ST1_ADDR);
	          break;
	        }
	      }
	    }


	    Read_Pump_Dip();

	    if(rx2_Receive_complete == 1){
	      SW_Com();
	    }


	  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PUMP1_MANUAL_Pin|PUMP1_CHECK_Pin|PUMP1_PS_Pin|PUMP5_MANUAL_Pin
                          |PUMP5_STOP_Pin|PUMP5_AUTO_Pin|PUMP6_PS_Pin|PUMP6_CHECK_Pin
                          |PUMP6_MANUAL_Pin|PUMP6_STOP_Pin|PUMP6_AUTO_Pin|HANJUN_LED_Pin
                          |PUMP1_AUTO_Pin|PUMP1_STOP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS485_DE_Pin|RS485_RE_Pin|PUMP4_AUTO_Pin|PUMP5_PS_Pin
                          |PUMP3_CHECK_Pin|PUMP3_AUTO_Pin|PUMP3_STOP_Pin|PUMP3_MANUAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RUN_LED_Pin|ERR_LED_Pin|PUMP4_PS_Pin|PUMP4_CHECK_Pin
                          |PUMP4_MANUAL_Pin|PUMP4_STOP_Pin|PUMP3_PS_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUMP5_CHECK_Pin|BALJUN_LED_Pin|SW_LOCK_ON_Pin|SW_LOCK_OFF_Pin
                          |TERMINATION_LED_Pin|PUMP2_AUTO_Pin|PUMP2_STOP_Pin|PUMP2_MANUAL_Pin
                          |PUMP2_CHECK_Pin|PUMP2_PS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, COLUMN0_Pin|COLUMN1_Pin|COLUMN2_Pin|COLUMN3_Pin
                          |COLUMN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PUMP1_MANUAL_Pin PUMP1_CHECK_Pin PUMP1_PS_Pin PUMP5_MANUAL_Pin
                           PUMP5_STOP_Pin PUMP5_AUTO_Pin PUMP6_PS_Pin PUMP6_CHECK_Pin
                           PUMP6_MANUAL_Pin PUMP6_STOP_Pin PUMP6_AUTO_Pin HANJUN_LED_Pin
                           PUMP1_AUTO_Pin PUMP1_STOP_Pin */
  GPIO_InitStruct.Pin = PUMP1_MANUAL_Pin|PUMP1_CHECK_Pin|PUMP1_PS_Pin|PUMP5_MANUAL_Pin
                          |PUMP5_STOP_Pin|PUMP5_AUTO_Pin|PUMP6_PS_Pin|PUMP6_CHECK_Pin
                          |PUMP6_MANUAL_Pin|PUMP6_STOP_Pin|PUMP6_AUTO_Pin|HANJUN_LED_Pin
                          |PUMP1_AUTO_Pin|PUMP1_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DE_Pin RS485_RE_Pin PUMP4_AUTO_Pin PUMP5_PS_Pin
                           PUMP3_CHECK_Pin PUMP3_AUTO_Pin PUMP3_STOP_Pin PUMP3_MANUAL_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin|RS485_RE_Pin|PUMP4_AUTO_Pin|PUMP5_PS_Pin
                          |PUMP3_CHECK_Pin|PUMP3_AUTO_Pin|PUMP3_STOP_Pin|PUMP3_MANUAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR_DIP0_Pin ADDR_DIP1_Pin ADDR_DIP2_Pin ADDR_DIP3_Pin
                           PUMP1_DIP1_Pin PUMP2_DIP2_Pin PUMP3_DIP3_Pin */
  GPIO_InitStruct.Pin = ADDR_DIP0_Pin|ADDR_DIP1_Pin|ADDR_DIP2_Pin|ADDR_DIP3_Pin
                          |PUMP1_DIP1_Pin|PUMP2_DIP2_Pin|PUMP3_DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RUN_LED_Pin ERR_LED_Pin PUMP4_PS_Pin PUMP4_CHECK_Pin
                           PUMP4_MANUAL_Pin PUMP4_STOP_Pin PUMP3_PS_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = RUN_LED_Pin|ERR_LED_Pin|PUMP4_PS_Pin|PUMP4_CHECK_Pin
                          |PUMP4_MANUAL_Pin|PUMP4_STOP_Pin|PUMP3_PS_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP5_CHECK_Pin BALJUN_LED_Pin SW_LOCK_ON_Pin SW_LOCK_OFF_Pin
                           TERMINATION_LED_Pin PUMP2_AUTO_Pin PUMP2_STOP_Pin PUMP2_MANUAL_Pin
                           PUMP2_CHECK_Pin PUMP2_PS_Pin */
  GPIO_InitStruct.Pin = PUMP5_CHECK_Pin|BALJUN_LED_Pin|SW_LOCK_ON_Pin|SW_LOCK_OFF_Pin
                          |TERMINATION_LED_Pin|PUMP2_AUTO_Pin|PUMP2_STOP_Pin|PUMP2_MANUAL_Pin
                          |PUMP2_CHECK_Pin|PUMP2_PS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW0_Pin ROW1_Pin ROW2_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP4_DIP4_Pin PUMP5_DIP5_Pin PUMP6_DIP6_Pin */
  GPIO_InitStruct.Pin = PUMP4_DIP4_Pin|PUMP5_DIP5_Pin|PUMP6_DIP6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN0_Pin COLUMN1_Pin COLUMN2_Pin COLUMN3_Pin
                           COLUMN4_Pin */
  GPIO_InitStruct.Pin = COLUMN0_Pin|COLUMN1_Pin|COLUMN2_Pin|COLUMN3_Pin
                          |COLUMN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void PUMP1_AUTO_LED_ON(void){
	HAL_GPIO_WritePin(PUMP1_AUTO_GPIO_Port, PUMP1_AUTO_Pin, GPIO_PIN_RESET);
}
void PUMP1_AUTO_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP1_AUTO_GPIO_Port, PUMP1_AUTO_Pin, GPIO_PIN_SET);
}

void PUMP1_STOP_LED_ON(void){
	HAL_GPIO_WritePin(PUMP1_STOP_GPIO_Port, PUMP1_STOP_Pin, GPIO_PIN_RESET);
}
void PUMP1_STOP_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP1_STOP_GPIO_Port, PUMP1_STOP_Pin, GPIO_PIN_SET);
}

void PUMP1_MANUAL_LED_ON(void){
	HAL_GPIO_WritePin(PUMP1_MANUAL_GPIO_Port, PUMP1_MANUAL_Pin, GPIO_PIN_RESET);
}
void PUMP1_MANUAL_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP1_MANUAL_GPIO_Port, PUMP1_MANUAL_Pin, GPIO_PIN_SET);
}

void PUMP1_CHECK_LED_ON(void){
	HAL_GPIO_WritePin(PUMP1_CHECK_GPIO_Port, PUMP1_CHECK_Pin, GPIO_PIN_RESET);
}
void PUMP1_CHECK_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP1_CHECK_GPIO_Port, PUMP1_CHECK_Pin, GPIO_PIN_SET);
}

void PUMP1_PS_LED_ON(void){
	HAL_GPIO_WritePin(PUMP1_PS_GPIO_Port, PUMP1_PS_Pin, GPIO_PIN_RESET);
}
void PUMP1_PS_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP1_PS_GPIO_Port, PUMP1_PS_Pin, GPIO_PIN_SET);
}

void PUMP2_AUTO_LED_ON(void){
	HAL_GPIO_WritePin(PUMP2_AUTO_GPIO_Port, PUMP2_AUTO_Pin, GPIO_PIN_RESET);
}
void PUMP2_AUTO_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP2_AUTO_GPIO_Port, PUMP2_AUTO_Pin, GPIO_PIN_SET);
}

void PUMP2_STOP_LED_ON(void){
	HAL_GPIO_WritePin(PUMP2_STOP_GPIO_Port, PUMP2_STOP_Pin, GPIO_PIN_RESET);
}
void PUMP2_STOP_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP2_STOP_GPIO_Port, PUMP2_STOP_Pin, GPIO_PIN_SET);
}

void PUMP2_MANUAL_LED_ON(void){
	HAL_GPIO_WritePin(PUMP2_MANUAL_GPIO_Port, PUMP2_MANUAL_Pin, GPIO_PIN_RESET);
}
void PUMP2_MANUAL_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP2_MANUAL_GPIO_Port, PUMP2_MANUAL_Pin, GPIO_PIN_SET);
}

void PUMP2_CHECK_LED_ON(void){
	HAL_GPIO_WritePin(PUMP2_CHECK_GPIO_Port, PUMP2_CHECK_Pin, GPIO_PIN_RESET);
}
void PUMP2_CHECK_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP2_CHECK_GPIO_Port, PUMP2_CHECK_Pin, GPIO_PIN_SET);
}

void PUMP2_PS_LED_ON(void){
	HAL_GPIO_WritePin(PUMP2_PS_GPIO_Port, PUMP2_PS_Pin, GPIO_PIN_RESET);
}
void PUMP2_PS_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP2_PS_GPIO_Port, PUMP2_PS_Pin, GPIO_PIN_SET);
}

void PUMP3_AUTO_LED_ON(void){
	HAL_GPIO_WritePin(PUMP3_AUTO_GPIO_Port, PUMP3_AUTO_Pin, GPIO_PIN_RESET);
}
void PUMP3_AUTO_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP3_AUTO_GPIO_Port, PUMP3_AUTO_Pin, GPIO_PIN_SET);
}

void PUMP3_STOP_LED_ON(void){
	HAL_GPIO_WritePin(PUMP3_STOP_GPIO_Port, PUMP3_STOP_Pin, GPIO_PIN_RESET);
}
void PUMP3_STOP_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP3_STOP_GPIO_Port, PUMP3_STOP_Pin, GPIO_PIN_SET);
}

void PUMP3_MANUAL_LED_ON(void){
	HAL_GPIO_WritePin(PUMP3_MANUAL_GPIO_Port, PUMP3_MANUAL_Pin, GPIO_PIN_RESET);
}
void PUMP3_MANUAL_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP3_MANUAL_GPIO_Port, PUMP3_MANUAL_Pin, GPIO_PIN_SET);
}

void PUMP3_CHECK_LED_ON(void){
	HAL_GPIO_WritePin(PUMP3_CHECK_GPIO_Port, PUMP3_CHECK_Pin, GPIO_PIN_RESET);
}
void PUMP3_CHECK_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP3_CHECK_GPIO_Port, PUMP3_CHECK_Pin, GPIO_PIN_SET);
}

void PUMP3_PS_LED_ON(void){
	HAL_GPIO_WritePin(PUMP3_PS_GPIO_Port, PUMP3_PS_Pin, GPIO_PIN_RESET);
}
void PUMP3_PS_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP3_PS_GPIO_Port, PUMP3_PS_Pin, GPIO_PIN_SET);
}

void PUMP4_AUTO_LED_ON(void){
	HAL_GPIO_WritePin(PUMP4_AUTO_GPIO_Port, PUMP4_AUTO_Pin, GPIO_PIN_RESET);
}
void PUMP4_AUTO_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP4_AUTO_GPIO_Port, PUMP4_AUTO_Pin, GPIO_PIN_SET);
}

void PUMP4_STOP_LED_ON(void){
	HAL_GPIO_WritePin(PUMP4_STOP_GPIO_Port, PUMP4_STOP_Pin, GPIO_PIN_RESET);
}
void PUMP4_STOP_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP4_STOP_GPIO_Port, PUMP4_STOP_Pin, GPIO_PIN_SET);
}

void PUMP4_MANUAL_LED_ON(void){
	HAL_GPIO_WritePin(PUMP4_MANUAL_GPIO_Port, PUMP4_MANUAL_Pin, GPIO_PIN_RESET);
}
void PUMP4_MANUAL_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP4_MANUAL_GPIO_Port, PUMP4_MANUAL_Pin, GPIO_PIN_SET);
}

void PUMP4_CHECK_LED_ON(void){
	HAL_GPIO_WritePin(PUMP4_CHECK_GPIO_Port, PUMP4_CHECK_Pin, GPIO_PIN_RESET);
}
void PUMP4_CHECK_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP4_CHECK_GPIO_Port, PUMP4_CHECK_Pin, GPIO_PIN_SET);
}

void PUMP4_PS_LED_ON(void){
	HAL_GPIO_WritePin(PUMP4_PS_GPIO_Port, PUMP4_PS_Pin, GPIO_PIN_RESET);
}
void PUMP4_PS_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP4_PS_GPIO_Port, PUMP4_PS_Pin, GPIO_PIN_SET);
}

void PUMP5_AUTO_LED_ON(void){
	HAL_GPIO_WritePin(PUMP5_AUTO_GPIO_Port, PUMP5_AUTO_Pin, GPIO_PIN_RESET);
}
void PUMP5_AUTO_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP5_AUTO_GPIO_Port, PUMP5_AUTO_Pin, GPIO_PIN_SET);
}

void PUMP5_STOP_LED_ON(void){
	HAL_GPIO_WritePin(PUMP5_STOP_GPIO_Port, PUMP5_STOP_Pin, GPIO_PIN_RESET);
}
void PUMP5_STOP_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP5_STOP_GPIO_Port, PUMP5_STOP_Pin, GPIO_PIN_SET);
}

void PUMP5_MANUAL_LED_ON(void){
	HAL_GPIO_WritePin(PUMP5_MANUAL_GPIO_Port, PUMP5_MANUAL_Pin, GPIO_PIN_RESET);
}
void PUMP5_MANUAL_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP5_MANUAL_GPIO_Port, PUMP5_MANUAL_Pin, GPIO_PIN_SET);
}

void PUMP5_CHECK_LED_ON(void){
	HAL_GPIO_WritePin(PUMP5_CHECK_GPIO_Port, PUMP5_CHECK_Pin, GPIO_PIN_RESET);
}
void PUMP5_CHECK_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP5_CHECK_GPIO_Port, PUMP5_CHECK_Pin, GPIO_PIN_SET);
}

void PUMP5_PS_LED_ON(void){
	HAL_GPIO_WritePin(PUMP5_PS_GPIO_Port, PUMP5_PS_Pin, GPIO_PIN_RESET);
}
void PUMP5_PS_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP5_PS_GPIO_Port, PUMP5_PS_Pin, GPIO_PIN_SET);
}

void PUMP6_AUTO_LED_ON(void){
	HAL_GPIO_WritePin(PUMP6_AUTO_GPIO_Port, PUMP6_AUTO_Pin, GPIO_PIN_RESET);
}
void PUMP6_AUTO_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP6_AUTO_GPIO_Port, PUMP6_AUTO_Pin, GPIO_PIN_SET);
}

void PUMP6_STOP_LED_ON(void){
	HAL_GPIO_WritePin(PUMP6_STOP_GPIO_Port, PUMP6_STOP_Pin, GPIO_PIN_RESET);
}
void PUMP6_STOP_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP6_STOP_GPIO_Port, PUMP6_STOP_Pin, GPIO_PIN_SET);
}

void PUMP6_MANUAL_LED_ON(void){
	HAL_GPIO_WritePin(PUMP6_MANUAL_GPIO_Port, PUMP6_MANUAL_Pin, GPIO_PIN_RESET);
}
void PUMP6_MANUAL_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP6_MANUAL_GPIO_Port, PUMP6_MANUAL_Pin, GPIO_PIN_SET);
}

void PUMP6_CHECK_LED_ON(void){
	HAL_GPIO_WritePin(PUMP6_CHECK_GPIO_Port, PUMP6_CHECK_Pin, GPIO_PIN_RESET);
}
void PUMP6_CHECK_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP6_CHECK_GPIO_Port, PUMP6_CHECK_Pin, GPIO_PIN_SET);
}

void PUMP6_PS_LED_ON(void){
	HAL_GPIO_WritePin(PUMP6_PS_GPIO_Port, PUMP6_PS_Pin, GPIO_PIN_RESET);
}
void PUMP6_PS_LED_OFF(void){
	HAL_GPIO_WritePin(PUMP6_PS_GPIO_Port, PUMP6_PS_Pin, GPIO_PIN_SET);
}

void HANJUN_LED_ON(void){
	HAL_GPIO_WritePin(HANJUN_LED_GPIO_Port, HANJUN_LED_Pin, GPIO_PIN_RESET);
}
void HANJUN_LED_OFF(void){
	HAL_GPIO_WritePin(HANJUN_LED_GPIO_Port, HANJUN_LED_Pin, GPIO_PIN_SET);
}

void BALJUN_LED_ON(void){
	HAL_GPIO_WritePin(BALJUN_LED_GPIO_Port, BALJUN_LED_Pin, GPIO_PIN_RESET);
}
void BALJUN_LED_OFF(void){
	HAL_GPIO_WritePin(BALJUN_LED_GPIO_Port, BALJUN_LED_Pin, GPIO_PIN_SET);
}

/*
void SW_LOCK_ON_LED_ON(void){
	HAL_GPIO_WritePin(SW_LOCK_ON_GPIO_Port, SW_LOCK_ON_Pin, GPIO_PIN_SET);
}
void SW_LOCK_ON_LED_OFF(void){
	HAL_GPIO_WritePin(SW_LOCK_ON_GPIO_Port, SW_LOCK_ON_Pin, GPIO_PIN_RESET);
}

void SW_LOCK_OFF_LED_ON(void){
	HAL_GPIO_WritePin(SW_LOCK_OFF_GPIO_Port, SW_LOCK_OFF_Pin, GPIO_PIN_SET);
}
void SW_LOCK_OFF_LED_OFF(void){
	HAL_GPIO_WritePin(SW_LOCK_OFF_GPIO_Port, SW_LOCK_OFF_Pin, GPIO_PIN_RESET);
}

*/
void SW_LOCK_ON_LED_ON(void){
	HAL_GPIO_WritePin(SW_LOCK_OFF_GPIO_Port, SW_LOCK_OFF_Pin, GPIO_PIN_SET);
}
void SW_LOCK_ON_LED_OFF(void){
	HAL_GPIO_WritePin(SW_LOCK_OFF_GPIO_Port, SW_LOCK_OFF_Pin, GPIO_PIN_RESET);
}

void SW_LOCK_OFF_LED_ON(void){
	HAL_GPIO_WritePin(SW_LOCK_ON_GPIO_Port, SW_LOCK_ON_Pin, GPIO_PIN_SET);
}
void SW_LOCK_OFF_LED_OFF(void){
	HAL_GPIO_WritePin(SW_LOCK_ON_GPIO_Port, SW_LOCK_ON_Pin, GPIO_PIN_RESET);
}

void TERMINATION_LED_ON(void){
	HAL_GPIO_WritePin(TERMINATION_LED_GPIO_Port, TERMINATION_LED_Pin, GPIO_PIN_RESET);
}
void TERMINATION_LED_OFF(void){
	HAL_GPIO_WritePin(TERMINATION_LED_GPIO_Port, TERMINATION_LED_Pin, GPIO_PIN_SET);
}

void ERR_LED_ON(void){
	HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_RESET);
}
void ERR_LED_OFF(void){
	HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_SET);
}

void Buzzer_ON(void){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void Buzzer_OFF(void){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}


void Read_Key(void){
  int i;

  Key_State = 0;

 // if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {     //SW_LOCK_OFF
    HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    /*
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[0] == 0){
        Key_input[0] = 1;
        Key_State = 1;
        LED_State[2] = 1 ;//-> PUMP1_MANUAL
      }
    }
    else{
      Key_input[0] = 0;
      LED_State[2] = 0 ;//-> PUMP1_MANUAL
    }
    */
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {
        if(Pre_Key_input[0] == 0){        //PUMP1_SW_MAN
          Key_input[0] = 1;
          Key_State = 1;
          //-> PUMP1_MANUAL
          Buzzer_ON();
          HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
          if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
            if(LED_State[2] == 1){          //PUMP1_MANUAL on 상태이면
              //LED_State[2] = 0;
            }
            else{

              LED_State[2] = 1;
              LED_State[1] = 0;//-> PUMP1_STOP off
              LED_State[0] = 0;//-> PUMP1_Auto off

            }
            UI_PUMP1_SW_MAN[0] = 0;
          }

          // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

        }
      }
    }
    else{
      Key_input[0] = 0;
    }

    if( HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[5] == 0){
        Key_input[5] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP2_AUTO

        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[5] == 1){
            //LED_State[5] = 0;
          }
          else{
            LED_State[5] = 1;
            LED_State[6] = 0 ;//-> PUMP2_STOP_off
            LED_State[7] = 0 ;//-> PUMP2_MANUAL_off
          }
        }

        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[5] = 0;
    }
    if(HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[10] == 0){       //PUMP4_SW_STOP
        Key_input[10] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP4_STOP
        if(LED_State[16] == 1){
          //LED_State[16] = 0;
        }
        else{
          LED_State[16] = 1;
          LED_State[15] = 0;//-> PUMP4_AUTO
          LED_State[17] = 0 ;//-> PUMP4_MANUAL
        }

        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[10] = 0;
    }
    /*
    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[15] == 0){
        Key_input[15] = 1;
        Key_State = 1;
        LED_State[27] = 1 ;//-> PUMP6_MANUAL
      }
    }
    else{
      Key_input[15] = 0;
      LED_State[27] = 0 ;//-> PUMP6_MANUAL
    }
    */
    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {
        if(Pre_Key_input[15] == 0){       //PUMP6_SW_MAN
          Key_input[15] = 1;
          Key_State = 1;
          Buzzer_ON();
          HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
          //-> PUMP6_MANUAL
          if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
            if(LED_State[27] == 1){
              //LED_State[27] = 0;
            }
            else{
              LED_State[27] = 1;
              LED_State[25] = 0;//-> PUMP6_STOP off
              LED_State[26] = 0;//-> PUMP6_Auto off

            }
          }
          // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }
        }
      }
    }
    else{
      Key_input[15] = 0;
    }


    if(rx2_Receive_complete == 1){
      SW_Com();
    }

    HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[1] == 0){        //PUMP1_SW_STOP
        Key_input[1] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP1_STOP
        if(LED_State[1] == 1){
          //LED_State[1] = 0;
        }
        else{
          LED_State[1] = 1;
          LED_State[0] = 0 ;//-> PUMP1_AUTO off
          LED_State[2] = 0 ;//-> PUMP1_MANUAL off
        }
        UI_PUMP1_SW_STOP[0] = 0;
      }

      // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

    }
    else{
      Key_input[1] = 0;
    }
    /*
    if(HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[6] == 0){
        Key_input[6] = 1;
        Key_State = 1;
        LED_State[12] = 1 ;//-> PUMP3_MANUAL
      }
    }
    else{
      Key_input[6] = 0;
      LED_State[12] = 0 ;//-> PUMP3_MANUAL
    }
*/
    if(HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {
        if(Pre_Key_input[6] == 0){        //-> PUMP3_SW_MAN
          Key_input[6] = 1;
          Key_State = 1;
          Buzzer_ON();
          HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
          //-> PUMP3_MANUAL
          if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
            if(LED_State[12] == 1){
              //LED_State[12] = 0;
            }
            else{
              LED_State[12] = 1;
              LED_State[11]= 0;//-> PUMP3_STOP off
              LED_State[10] = 0;//-> PUMP3_Auto off

            }
          }
          // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

        }
      }
    }
    else{
      Key_input[6] = 0;
    }

    if(HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[11] == 0){       //PUMP4_SW_AUTO
        Key_input[11] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP4_AUTO
        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[15] == 1){
            //LED_State[15] = 0;
          }
          else{
            LED_State[15] = 1;
            LED_State[16] = 0 ;//-> PUMP4_STOP_off
            LED_State[17] = 0 ;//-> PUMP4_MANUAL_off
          }
        }

        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }


      }
    }
    else{
      Key_input[11] = 0;
    }
    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[16] == 0){       //PUMP6_SW_STOP
        Key_input[16] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP4_STOP
        if(LED_State[26] == 1){
          //LED_State[26] = 0;
        }
        else{
          LED_State[26] = 1;
          LED_State[25] = 0 ;//-> PUMP6_AUTO off
          LED_State[27] = 0 ;//-> PUMP6_MANUAL off
        }
      }

      // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

    }
    else{
      Key_input[16] = 0;
    }


    if(rx2_Receive_complete == 1){
      SW_Com();
    }

    HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if((Pre_Key_input[2] == 0) | (UI_PUMP1_SW_AUTO[0] == 1)){ // PUMP1_SW_AUTO
        Key_input[2] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP1_AUTO
        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[0] == 1){
            //LED_State[0] = 0;
          }
          else{
            LED_State[0] = 1;
            LED_State[1] = 0;//-> PUMP1_STOP off
            LED_State[2] = 0;//-> PUMP1_MANUAL_off
          }
          UI_PUMP1_SW_AUTO[0] = 0;
        }

        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[2] = 0;
    }
    if(HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[7] == 0){        //PUMP3_SW_STOP
        Key_input[7] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP3_STOP
        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[11] == 1){
            //LED_State[11] = 0;
          }
          else{
            LED_State[11] = 1;
            LED_State[10] = 0 ;//-> PUMP3_AUTO off
            LED_State[12] = 0 ;//-> PUMP3_MANUAL off
          }
        }
        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[7] = 0;
    }
    /*
    if(HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[12] == 0){
        Key_input[12] = 1;
        Key_State = 1;
        LED_State[22] = 1 ;//-> PUMP5_MANUAL
      }
    }
    else{
      Key_input[12] = 0;
      LED_State[22] = 0 ;//-> PUMP5_MANUAL
    }
*/
    if(HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){
      if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {
       if(Pre_Key_input[12] == 0){        //PUMP5_SW_MAN
          Key_input[12] = 1;
          Key_State = 1;
          Buzzer_ON();
          HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
          //-> PUMP1_MANUAL
          if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
            if(LED_State[22] == 1){
              //LED_State[22] = 0;
            }
            else{
              LED_State[22] = 1;
              LED_State[21] = 0;//-> PUMP5_AUTO off
              LED_State[20] = 0;//-> PUMP5_AUTO off
            }
          }
          // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }
        }
      }
    }
    else{
      Key_input[12] = 0;
    }

    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[17] == 0){       //PUMP1_SW_AUTO
        Key_input[17] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP6_AUTO
        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[25] == 1){
            //LED_State[25] = 0;
          }
          else{
            LED_State[25] = 1;
            LED_State[26] = 0;    //PUMP6_STOP_off
            LED_State[27] = 0;    //PUMP6_MANUAL_off
          }
        }

        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[17] = 0;
    }


    if(rx2_Receive_complete == 1){
      SW_Com();
    }


    HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    /*
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[3] == 0){
        Key_input[3] = 1;
        Key_State = 1;
        LED_State[7] = 1 ;//-> PUMP2_MANUAL
      }
    }
    else{
      Key_input[3] = 0;
      LED_State[7] = 0 ;//-> PUMP2_MANUAL
    }
*/
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {
        if(Pre_Key_input[3] == 0){        //PUMP2_SW_MAN
          Key_input[3] = 1;
          Key_State = 1;
          Buzzer_ON();
          HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
          //-> PUMP2_MANUAL
          if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
            if(LED_State[7] == 1){
              //LED_State[7] = 0;
            }
            else{
              LED_State[7] = 1;
              LED_State[6] = 0;//-> PUMP2_STOP off
              LED_State[5] = 0;//-> PUMP2_Auto off

            }
          }

          // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }
        }
      }
    }
    else{
      Key_input[3] = 0;
    }

    if(HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[8] == 0){        //PUMP3_SW_AUTO
        Key_input[8] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP3_AUTO
        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[10] == 1){
            //LED_State[10] = 0;
          }
          else{
            LED_State[10] = 1;
            LED_State[11] = 0 ;//-> PUMP3_STOP off
            LED_State[12] = 0 ;//-> PUMP3_MANUAL off
          }
        }
        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }
      }
    }
    else{
      Key_input[8] = 0;
    }
    if(HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[13] == 0){
        Key_input[13] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP5_STOP
        if(LED_State[21] == 1){
          //LED_State[21] = 0;
        }
        else{
          LED_State[21] = 1;
          LED_State[20] = 0 ;//-> PUMP5_STOP off
          LED_State[22] = 0 ;//-> PUMP5_MANUAL off
        }
        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[13] = 0;
    }
    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[18] == 0){
        Key_input[18] = 1;
        Key_State = 1;
        LED_State[32] = 1 ;       //SW_LOCK_ON
        LED_State[33] = 0 ;       //SW_LOCK_OFF
      }
    }
    else{
      Key_input[18] = 0;
    }


    if(rx2_Receive_complete == 1){
      SW_Com();
    }

    HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(ROW0_GPIO_Port, ROW0_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[4] == 0){        //PUMP2_SW_STOP
        Key_input[4] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP2_STOP
        if(LED_State[6] == 1){
          //LED_State[6] = 0;
        }
        else{
          LED_State[6] = 1;
          LED_State[5] = 0 ;//-> PUMP2_AUTO off
          LED_State[7] = 0 ;//-> PUMP2_MANUAL off
        }
      }
      // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }
    }
    else{
      Key_input[4] = 0;
    }
    /*
    if(HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[9] == 0){
        Key_input[9] = 1;
        Key_State = 1;
        LED_State[17] = 1 ;//-> PUMP4_MANUAL
      }
    }
    else{
      Key_input[9] = 0;
      LED_State[17] = 0 ;//-> PUMP4_MANUAL
    }
*/
    if(HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_SET){
      if((LED_State[32]  == 0)&(LED_State[33]  == 1)) {
        if(Pre_Key_input[9] == 0){        //PUMP4_SW_MAN
          Key_input[9] = 1;
          Key_State = 1;
          Buzzer_ON();
          HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
          //-> PUMP4_MANUAL
          if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
            if(LED_State[17] == 1){
              //LED_State[17] = 0;
            }
            else{
              LED_State[17] = 1;
              LED_State[16] = 0;//-> PUMP4_STOP off
              LED_State[15] = 0;//-> PUMP4_Auto off

            }
          }
          // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }
        }
      }
    }
    else{
      Key_input[9] = 0;
    }

    if(HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[14] == 0){       //PUMP5_SW_AUTO
        Key_input[14] = 1;
        Key_State = 1;
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
        //-> PUMP5_AUTO
        if(LED_State[34] == 0){           // 강제 정지 상태 아닌때에만 동작
          if(LED_State[20] == 1){
            //LED_State[20] = 0;
          }
          else{
            LED_State[20] = 1;
            LED_State[21] = 0 ;//-> PUMP5_STOP off
            LED_State[22] = 0 ;//-> PUMP5_MANUAL off
          }
        }
        // 20222 03 30 자동 잠금 기능을 수동모든 버튼에만 용 요청
          if (Key_input[18] == 0){
            Key_input[18] = 1;
            LED_State[32] = 1 ;       //SW_LOCK_ON
            LED_State[33] = 0 ;       //SW_LOCK_OFF
          }

      }
    }
    else{
      Key_input[14] = 0;
    }
    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[19] == 0){
        Key_State = 1;
        /*
        Key_input[19] = 1;
        Key_State = 1;
        if( LED_State[34] == 0){
          LED_State[1] = 1 ;//-> PUMP1_STOP
          LED_State[6] = 1 ;//-> PUMP2_STOP
          LED_State[11] = 1 ;//-> PUMP3_STOP
          LED_State[16] = 1 ;//-> PUMP4_STOP
          LED_State[21] = 1 ;//-> PUMP3_STOP
          LED_State[26] = 1 ;//-> PUMP4_STOP
          LED_State[34] = 1;

          LED_State[0] = 0;//-> PUMP1_AUTO off
          LED_State[2] = 0 ;//-> PUMP1_MANUAL off

          LED_State[5] = 0;//-> PUMP2_AUTO off
          LED_State[7] = 0 ;//-> PUMP2_MANUAL off

          LED_State[10] = 0;//-> PUMP3_AUTO off
          LED_State[12] = 0 ;//-> PUMP3_MANUAL off

          LED_State[15] = 0 ;//-> PUMP4_AUTO off
          LED_State[17] = 0 ;//-> PUMP4_MANUAL off

          LED_State[20] = 0 ;//-> PUMP5_AUTO off
          LED_State[22] = 0 ;//-> PUMP5_MANUAL off

          LED_State[25] = 0 ;//-> PUMP6_AUTO off
          LED_State[27] = 0 ;//-> PUMP6_MANUAL off

        }
        else{
          LED_State[1] = 0 ;//-> PUMP1_STOP
          LED_State[6] = 0 ;//-> PUMP2_STOP
          LED_State[11] = 0 ;//-> PUMP3_STOP
          LED_State[16] = 0 ;//-> PUMP4_STOP
          LED_State[21] = 0 ;//-> PUMP3_STOP
          LED_State[26] = 0 ;//-> PUMP4_STOP
          LED_State[34] = 0;
        }
        */
      }
    }
    else{
      Key_input[19] = 0;
    }
 // }

  if((LED_State[32]  == 1)&(LED_State[33]  == 0)) {     //SW_LOCK_ON

    HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);


    if(HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_SET){
      if(Pre_Key_input[18] == 0){
        Key_input[18] = 1;
        Key_State = 1;
        LED_State[32] = 0 ;       //SW_LOCK_ON
        LED_State[33] = 1 ;       //SW_LOCK_OFF
        Buzzer_ON();
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
      }
    }
    else{
      Key_input[18] = 0;
    }
  }
  //}

  if(rx2_Receive_complete == 1){
      SW_Com();
    }

  HAL_GPIO_WritePin(COLUMN0_GPIO_Port, COLUMN0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COLUMN4_GPIO_Port, COLUMN4_Pin, GPIO_PIN_RESET);

  if( LED_State[34] == 1){
    LED_State[1] = 1 ;//-> PUMP1_STOP
    LED_State[6] = 1 ;//-> PUMP2_STOP
    LED_State[11] = 1 ;//-> PUMP3_STOP
    LED_State[16] = 1 ;//-> PUMP4_STOP
    LED_State[21] = 1 ;//-> PUMP3_STOP
    LED_State[26] = 1 ;//-> PUMP4_STOP
  }


  int set_count = 0;

  if(Key_State == 1){
    if(Pre_Key_State == 0){

    /*
      // 키 눌렸을때 무조건 키락 상태로 변경
      // 20222 02 07 자동 잠금 기능을 수동  버튼에만 적용 요청
      if (Key_input[18] == 0){
        Key_input[18] = 1;
        LED_State[32] = 1 ;       //SW_LOCK_ON
        LED_State[33] = 0 ;       //SW_LOCK_OFF
      }
      */
      //Buzzer_ON();
      //HAL_Delay(200);
      //Buzzer_OFF();
      //HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
      Pre_Key_State = 1;
    }


    while(1){

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

      if(rx2_Receive_complete == 1){
        if(SW_report_com() == 1){
          set_count++;
        }
        if(set_count == 2){
          break;
        }
      }
    }
  }
  else{
    Pre_Key_State = 0;

    /*
    // 키 인풋이 있을경우 통신값을 갱신하지 않는다.
    LED_State[0] = UI_PUMP1_SW_AUTO[0];
    LED_State[1] = UI_PUMP1_SW_STOP[0];
    LED_State[2] = UI_PUMP1_SW_MAN[0];

    LED_State[5] = UI_PUMP1_SW_AUTO[1];
    LED_State[6] = UI_PUMP1_SW_STOP[1];
    LED_State[7] = UI_PUMP1_SW_MAN[1];

    LED_State[10] = UI_PUMP1_SW_AUTO[2];
    LED_State[11] = UI_PUMP1_SW_STOP[2];
    LED_State[12] = UI_PUMP1_SW_MAN[2];

    LED_State[15] = UI_PUMP1_SW_AUTO[3];
    LED_State[16] = UI_PUMP1_SW_STOP[3];
    LED_State[17] = UI_PUMP1_SW_MAN[3];

    LED_State[20] = UI_PUMP1_SW_AUTO[4];
    LED_State[21] = UI_PUMP1_SW_STOP[4];
    LED_State[22] = UI_PUMP1_SW_MAN[4];

    LED_State[25] = UI_PUMP1_SW_AUTO[5];
    LED_State[26] = UI_PUMP1_SW_STOP[5];
    LED_State[27] = UI_PUMP1_SW_MAN[5];
*/
  }

  for(i=0;i<20;i++){
    Pre_Key_input[i] = Key_input[i];
  }


}

void All_LED_ON(void){

  if(LED_State[0]  == 1){
    PUMP1_AUTO_LED_ON();}
  else{
    PUMP1_AUTO_LED_OFF();}

   if(LED_State[1]  == 1){
    PUMP1_STOP_LED_ON();}
  else{
    PUMP1_STOP_LED_OFF();}

   if(LED_State[2]  == 1){
    PUMP1_MANUAL_LED_ON();}
  else{
    PUMP1_MANUAL_LED_OFF();}

   if(LED_State[3]  == 1){
    PUMP1_CHECK_LED_ON();}
  else{
    PUMP1_CHECK_LED_OFF();}

   if(LED_State[4]  == 1){
    PUMP1_PS_LED_ON();}
  else{
    PUMP1_PS_LED_OFF();}

  if(LED_State[5]  == 1){
    PUMP2_AUTO_LED_ON();}
  else{
    PUMP2_AUTO_LED_OFF();}

   if(LED_State[6]  == 1){
    PUMP2_STOP_LED_ON();}
  else{
    PUMP2_STOP_LED_OFF();}

   if(LED_State[7]  == 1){
    PUMP2_MANUAL_LED_ON();}
  else{
    PUMP2_MANUAL_LED_OFF();}

   if(LED_State[8]  == 1){
    PUMP2_CHECK_LED_ON();}
  else{
    PUMP2_CHECK_LED_OFF();}

   if(LED_State[9]  == 1){
    PUMP2_PS_LED_ON();}
  else{
    PUMP2_PS_LED_OFF();}

  if(LED_State[10]  == 1){
    PUMP3_AUTO_LED_ON();}
  else{
    PUMP3_AUTO_LED_OFF();}

   if(LED_State[11]  == 1){
    PUMP3_STOP_LED_ON();}
  else{
    PUMP3_STOP_LED_OFF();}

   if(LED_State[12]  == 1){
    PUMP3_MANUAL_LED_ON();}
  else{
    PUMP3_MANUAL_LED_OFF();}

   if(LED_State[13]  == 1){
    PUMP3_CHECK_LED_ON();}
  else{
    PUMP3_CHECK_LED_OFF();}

   if(LED_State[14]  == 1){
    PUMP3_PS_LED_ON();}
  else{
    PUMP3_PS_LED_OFF();}

  if(LED_State[15]  == 1){
    PUMP4_AUTO_LED_ON();}
  else{
    PUMP4_AUTO_LED_OFF();}

   if(LED_State[16]  == 1){
    PUMP4_STOP_LED_ON();}
  else{
    PUMP4_STOP_LED_OFF();}

   if(LED_State[17]  == 1){
    PUMP4_MANUAL_LED_ON();}
  else{
    PUMP4_MANUAL_LED_OFF();}

   if(LED_State[18]  == 1){
    PUMP4_CHECK_LED_ON();}
  else{
    PUMP4_CHECK_LED_OFF();}

   if(LED_State[19]  == 1){
    PUMP4_PS_LED_ON();}
  else{
    PUMP4_PS_LED_OFF();}

  if(LED_State[20]  == 1){
    PUMP5_AUTO_LED_ON();}
  else{
    PUMP5_AUTO_LED_OFF();}

   if(LED_State[21]  == 1){
    PUMP5_STOP_LED_ON();}
  else{
    PUMP5_STOP_LED_OFF();}

   if(LED_State[22]  == 1){
    PUMP5_MANUAL_LED_ON();}
  else{
    PUMP5_MANUAL_LED_OFF();}

   if(LED_State[23]  == 1){
    PUMP5_CHECK_LED_ON();}
  else{
    PUMP5_CHECK_LED_OFF();}

   if(LED_State[24]  == 1){
    PUMP5_PS_LED_ON();}
  else{
    PUMP5_PS_LED_OFF();}

  if(LED_State[25]  == 1){
    PUMP6_AUTO_LED_ON();}
  else{
    PUMP6_AUTO_LED_OFF();}

   if(LED_State[26]  == 1){
    PUMP6_STOP_LED_ON();}
  else{
    PUMP6_STOP_LED_OFF();}

   if(LED_State[27]  == 1){
    PUMP6_MANUAL_LED_ON();}
  else{
    PUMP6_MANUAL_LED_OFF();}

   if(LED_State[28]  == 1){
    PUMP6_CHECK_LED_ON();}
  else{
    PUMP6_CHECK_LED_OFF();}

   if(LED_State[29]  == 1){
    PUMP6_PS_LED_ON();}
  else{
    PUMP6_PS_LED_OFF();}

  if(LED_State[30]  == 1){
    HANJUN_LED_ON();}
  else{
    HANJUN_LED_OFF();}

   if(LED_State[31]  == 1){
    BALJUN_LED_ON();}
  else{
    BALJUN_LED_OFF();}

   if(LED_State[32]  == 1){
    SW_LOCK_ON_LED_ON();}
  else{
    SW_LOCK_ON_LED_OFF();}

   if(LED_State[33]  == 1){
    SW_LOCK_OFF_LED_ON();}
  else{
    SW_LOCK_OFF_LED_OFF();}

   if(LED_State[34]  == 1){
    TERMINATION_LED_ON();}
  else{
    TERMINATION_LED_OFF();}
}

void All_LED_OFF(void){

  //PUMP1_AUTO_LED_OFF();
  PUMP1_STOP_LED_OFF();
  PUMP1_MANUAL_LED_OFF();
  //PUMP1_CHECK_LED_OFF();
  //PUMP1_PS_LED_OFF();

  //PUMP2_AUTO_LED_OFF();
  PUMP2_STOP_LED_OFF();
  PUMP2_MANUAL_LED_OFF();
  //PUMP2_CHECK_LED_OFF();
  //PUMP2_PS_LED_OFF();

  //PUMP3_AUTO_LED_OFF();
  PUMP3_STOP_LED_OFF();
  PUMP3_MANUAL_LED_OFF();
  //PUMP3_CHECK_LED_OFF();
  //PUMP3_PS_LED_OFF();

  //PUMP4_AUTO_LED_OFF();
  PUMP4_STOP_LED_OFF();
  PUMP4_MANUAL_LED_OFF();
  //PUMP4_CHECK_LED_OFF();
  //PUMP4_PS_LED_OFF();

  //PUMP5_AUTO_LED_OFF();
  PUMP5_STOP_LED_OFF();
  PUMP5_MANUAL_LED_OFF();
  //PUMP5_CHECK_LED_OFF();
  //PUMP5_PS_LED_OFF();

  //PUMP6_AUTO_LED_OFF();
  PUMP6_STOP_LED_OFF();
  PUMP6_MANUAL_LED_OFF();
  //PUMP6_CHECK_LED_OFF();
  //PUMP6_PS_LED_OFF();

  //HANJUN_LED_OFF();
  //BALJUN_LED_OFF();

  //SW_LOCK_ON_LED_OFF();
  SW_LOCK_OFF_LED_OFF();
  TERMINATION_LED_OFF();

}

int Uart2_crc = 0;
int Uart_crc = 0;

void SW_Com(void){

  // CRC 체크

  int i;

  // 전송할 중계기 정보를 확인한다.

  Uart2_crc = 0;

  for(i = 0; i <rx2_buf_count_tmp - 5 ; i++){
    Uart2_crc = Uart2_crc ^ Uart_rx2_buf[i+2];
  }

  if(Uart2_crc == Uart_rx2_buf[rx2_buf_count_tmp - 3]){
    //0x51 , 'Q' ,중계기 정보 요청 (UI에서 설정한 값만)

    if(Uart_rx2_buf[4] == 0x4D){        // MCC board : '0x4D'
      if(Uart_rx2_buf[5] == MCC_Address[0]){      // Address MCC_Address[0]
        if(Uart_rx2_buf[3] == 0x51){    // Report request : 'Q' ,'0x51'

          Uart_tx_buf[0] = 0x53;    //S
          Uart_tx_buf[1] = 0x54;    //T
          Uart_tx_buf[2] = 0x4D;    //M
          Uart_tx_buf[3] = 0x52;    //R
          Uart_tx_buf[4] = MCC_Address[0];    //address
          Uart_tx_buf[5] = SW_Version;    //version 10 -> 1.0
          Uart_tx_buf[6] = 0x00;    //LED정보 1 ~8
          Uart_tx_buf[7] = 0x00;    //LED정보 9 ~16
          Uart_tx_buf[8] = 0x00;    //LED정보 17 ~24
          Uart_tx_buf[9] = 0x00;    //LED정보 25 ~32
          Uart_tx_buf[10] = 0x00;   //LED정보 33 ~40
          Uart_tx_buf[11] = 0x00;   //Dip sw정보 1 ~8
          Uart_tx_buf[12] = 0x00;   //dummy1
          Uart_tx_buf[13] = 0x00;   //dummy2
          Uart_tx_buf[14] = 0x00;   //CRC
          Uart_tx_buf[15] = 0x45;   //E
          Uart_tx_buf[16] = 0x44;   //D

          /*
          Uart_tx_buf[6] = (LED_State[0]0x01 << 7)|(LED_State[1] << 6)|(LED_State[2] << 5)|(LED_State[3] << 4)|
                            (LED_State[4] << 3)|(LED_State[5] << 2)|(LED_State[6] << 1)|(LED_State[7] << 0);
          Uart_tx_buf[7] = (LED_State[8] << 7)|(LED_State[9] << 6)|(LED_State[10] << 5)|(LED_State[11] << 4)|
                            (LED_State[12] << 3)|(LED_State[13] << 2)|(LED_State[14] << 1)|(LED_State[15] << 0);
          Uart_tx_buf[8] = (LED_State[16] << 7)|(LED_State[17] << 6)|(LED_State[18] << 5)|(LED_State[19] << 4)|
                            (LED_State[20] << 3)|(LED_State[21] << 2)|(LED_State[22] << 1)|(LED_State[23] << 0);
          Uart_tx_buf[9] = (LED_State[24] << 7)|(LED_State[25] << 6)|(LED_State[26] << 5)|(LED_State[27] << 4)|
                            (LED_State[28] << 3)|(LED_State[29] << 2)|(LED_State[30] << 1)|(LED_State[31] << 0);
          Uart_tx_buf[10] = (LED_State[32] << 7)|(LED_State[33] << 6)|(LED_State[34] << 5);
          Uart_tx_buf[11] = ((~(Pump_Dip[0]) & 0x01)<< 7)|((~(Pump_Dip[1]) & 0x01)<< 6)|((~(Pump_Dip[2]) & 0x01)<< 5)|
                            ((~(Pump_Dip[3]) & 0x01)<< 4)|((~(Pump_Dip[4]) & 0x01)<< 3)|((~(Pump_Dip[5]) & 0x01)<< 2);
*/

          Uart_tx_buf[6] = ((LED_State[0] & 0x01) << 7)|((LED_State[1] & 0x01) << 6)|((LED_State[2] & 0x01) << 5)|((LED_State[3] & 0x01) << 4)|
                            ((LED_State[4] & 0x01) << 3)|((LED_State[5] & 0x01) << 2)|((LED_State[6] & 0x01) << 1)|((LED_State[7] & 0x01) << 0);
          Uart_tx_buf[7] = ((LED_State[8] & 0x01) << 7)|((LED_State[9] & 0x01) << 6)|((LED_State[10] & 0x01) << 5)|((LED_State[11] & 0x01) << 4)|
                            ((LED_State[12] & 0x01) << 3)|((LED_State[13] & 0x01) << 2)|((LED_State[14] & 0x01) << 1)|((LED_State[15] & 0x01) << 0);
          Uart_tx_buf[8] = ((LED_State[16] & 0x01) << 7)|((LED_State[17] & 0x01) << 6)|((LED_State[18] & 0x01) << 5)|((LED_State[19] & 0x01) << 4)|
                            ((LED_State[20] & 0x01) << 3)|((LED_State[21] & 0x01) << 2)|((LED_State[22] & 0x01) << 1)|((LED_State[23] & 0x01) << 0);
          Uart_tx_buf[9] = ((LED_State[24] & 0x01) << 7)|((LED_State[25] & 0x01) << 6)|((LED_State[26] & 0x01) << 5)|((LED_State[27] & 0x01) << 4)|
                            ((LED_State[28] & 0x01) << 3)|((LED_State[29] & 0x01) << 2)|((LED_State[30] & 0x01) << 1)|((LED_State[31] & 0x01) << 0);
          Uart_tx_buf[10] = ((LED_State[32] & 0x01) << 7)|((LED_State[33] & 0x01) << 6)|((LED_State[34] & 0x01) << 5);
          Uart_tx_buf[11] = ((~(Pump_Dip[0]) & 0x01)<< 7)|((~(Pump_Dip[1]) & 0x01)<< 6)|((~(Pump_Dip[2]) & 0x01)<< 5)|
                            ((~(Pump_Dip[3]) & 0x01)<< 4)|((~(Pump_Dip[4]) & 0x01)<< 3)|((~(Pump_Dip[5]) & 0x01)<< 2);
          Uart_crc = 0;

          for(i = 0; i <tx_buf_len - 5 ; i++){
            Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
          }
          Uart_tx_buf[tx_buf_len-3] = Uart_crc;

          HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
          HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

          HAL_Delay(1);

          if(HAL_UART_Transmit(&huart2, Uart_tx_buf, sizeof(Uart_tx_buf), 1000)!= HAL_OK)
              {
                Error_Handler();
              }

          HAL_Delay(1);

          HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

        }
        else if((Uart_rx2_buf[3] == 0x53)&(Key_State == 0)){       //Setting : 'S' '0x53' , 키 설정이 있을때에는 통신값으로 설정하지 않는다.
          /*
          for(i=0;i<8;i++){
            LED_State[i] = ( Uart_rx2_buf[6] >> (7 - i)) & 0x01;
            LED_State[i+8] = ( Uart_rx2_buf[7] >> (7 - i)) & 0x01;
            LED_State[i+8+8] = ( Uart_rx2_buf[8] >> (7 - i)) & 0x01;
            LED_State[i+8+8+8] = ( Uart_rx2_buf[9] >> (7 - i)) & 0x01;
            //LED_State[i+8+8+8+8] = ( Uart_rx2_buf[10] >> (7 - i)) & 0x01;
          } */

          //if(Key_State == 0){   //키 입력이 존재할때 키 입력을 우선한다.
            LED_State[0] = (Uart_rx2_buf[6] >> 7) & 0x01 ;//-> PUMP1_AUTO
            LED_State[1] = (Uart_rx2_buf[6] >> 6) & 0x01 ;//-> PUMP1_STOP
            LED_State[2] = (Uart_rx2_buf[6] >> 5) & 0x01 ;//-> PUMP1_MANUAL
            LED_State[3] = (Uart_rx2_buf[6] >> 4) & 0x01 ;//-> PUMP1_CHECK
            LED_State[4] = (Uart_rx2_buf[6] >> 3) & 0x01 ;//-> PUMP1_PS

            LED_State[5] = (Uart_rx2_buf[6] >> 2) & 0x01 ;//-> PUMP2_AUTO
            LED_State[6] = (Uart_rx2_buf[6] >> 1) & 0x01 ;//-> PUMP2_STOP
            LED_State[7] = (Uart_rx2_buf[6] >> 0) & 0x01 ;//-> PUMP2_MANUAL
            LED_State[8] = (Uart_rx2_buf[7] >> 7) & 0x01 ;//-> PUMP2_CHECK
            LED_State[9] = (Uart_rx2_buf[7] >> 6) & 0x01 ;//-> PUMP2_PS

            LED_State[10] = (Uart_rx2_buf[7] >> 5) & 0x01 ;//-> PUMP3_AUTO
            LED_State[11] = (Uart_rx2_buf[7] >> 4) & 0x01 ;//-> PUMP3_STOP
            LED_State[12] = (Uart_rx2_buf[7] >> 3) & 0x01 ;//-> PUMP3_MANUAL
            LED_State[13] = (Uart_rx2_buf[7] >> 2) & 0x01 ;//-> PUMP3_CHECK
            LED_State[14] = (Uart_rx2_buf[7] >> 1) & 0x01 ;//-> PUMP3_PS

            LED_State[15] = (Uart_rx2_buf[7] >> 0) & 0x01 ;//-> PUMP4_AUTO
            LED_State[16] = (Uart_rx2_buf[8] >> 7) & 0x01 ;//-> PUMP4_STOP
            LED_State[17] = (Uart_rx2_buf[8] >> 6) & 0x01 ;//-> PUMP4_MANUAL
            LED_State[18] = (Uart_rx2_buf[8] >> 5) & 0x01 ;//-> PUMP4_CHECK
            LED_State[19] = (Uart_rx2_buf[8] >> 4) & 0x01 ;//-> PUMP4_PS

            LED_State[20] = (Uart_rx2_buf[8] >> 3) & 0x01 ;//-> PUMP5_AUTO
            LED_State[21] = (Uart_rx2_buf[8] >> 2) & 0x01 ;//-> PUMP5_STOP
            LED_State[22] = (Uart_rx2_buf[8] >> 1) & 0x01 ;//-> PUMP5_MANUAL
            LED_State[23] = (Uart_rx2_buf[8] >> 0) & 0x01 ;//-> PUMP5_CHECK
            LED_State[24] = (Uart_rx2_buf[9] >> 7) & 0x01 ;//-> PUMP5_PS

            LED_State[25] = (Uart_rx2_buf[9] >> 6) & 0x01 ;//-> PUMP6_AUTO
            LED_State[26] = (Uart_rx2_buf[9] >> 5) & 0x01 ;//-> PUMP6_STOP
            LED_State[27] = (Uart_rx2_buf[9] >> 4) & 0x01 ;//-> PUMP6_MANUAL
            LED_State[28] = (Uart_rx2_buf[9] >> 3) & 0x01 ;//-> PUMP6_CHECK
            LED_State[29] = (Uart_rx2_buf[9] >> 2) & 0x01 ;//-> PUMP6_PS

            LED_State[30] = (Uart_rx2_buf[9] >> 1) & 0x01 ;//-> HANJUN
            LED_State[31] = (Uart_rx2_buf[9] >> 0) & 0x01 ;//-> BALJUN
            LED_State[32] = (Uart_rx2_buf[10] >> 7) & 0x01 ;//-> SW_LOCK_ON
            LED_State[33] = (Uart_rx2_buf[10] >> 6) & 0x01 ;//-> SW_LOCK_OFF
            //LED_State[34] = (Uart_rx2_buf[10] >> 5) & 0x01 ;//-> TERMINATION

            /*
            UI_PUMP1_SW_AUTO[0] = (Uart_rx2_buf[6] >> 7) & 0x01 ;//-> PUMP1_AUTO
            UI_PUMP1_SW_STOP[0] = (Uart_rx2_buf[6] >> 6) & 0x01 ;//-> PUMP1_STOP
            UI_PUMP1_SW_MAN[0] = (Uart_rx2_buf[6] >> 5) & 0x01 ;//-> PUMP1_MANUAL

            UI_PUMP1_SW_AUTO[1] = (Uart_rx2_buf[6] >> 2) & 0x01 ;//-> PUMP2_AUTO
            UI_PUMP1_SW_STOP[1] = (Uart_rx2_buf[6] >> 1) & 0x01 ;//-> PUMP2_STOP
            UI_PUMP1_SW_MAN[1] = (Uart_rx2_buf[6] >> 0) & 0x01 ;//-> PUMP2_MANUAL

            UI_PUMP1_SW_AUTO[2] = (Uart_rx2_buf[7] >> 5) & 0x01 ;//-> PUMP3_AUTO
            UI_PUMP1_SW_STOP[2] = (Uart_rx2_buf[7] >> 4) & 0x01 ;//-> PUMP3_STOP
            UI_PUMP1_SW_MAN[2] = (Uart_rx2_buf[7] >> 3) & 0x01 ;//-> PUMP3_MANUAL

            UI_PUMP1_SW_AUTO[3] = (Uart_rx2_buf[7] >> 0) & 0x01 ;//-> PUMP4_AUTO
            UI_PUMP1_SW_STOP[3] = (Uart_rx2_buf[8] >> 7) & 0x01 ;//-> PUMP4_STOP
            UI_PUMP1_SW_MAN[3] = (Uart_rx2_buf[8] >> 6) & 0x01 ;//-> PUMP4_MANUAL

            UI_PUMP1_SW_AUTO[4] = (Uart_rx2_buf[8] >> 3) & 0x01 ;//-> PUMP5_AUTO
            UI_PUMP1_SW_STOP[4] = (Uart_rx2_buf[8] >> 2) & 0x01 ;//-> PUMP5_STOP
            UI_PUMP1_SW_MAN[4] = (Uart_rx2_buf[8] >> 1) & 0x01 ;//-> PUMP5_MANUAL

            UI_PUMP1_SW_AUTO[5] = (Uart_rx2_buf[9] >> 6) & 0x01 ;//-> PUMP6_AUTO
            UI_PUMP1_SW_STOP[5] = (Uart_rx2_buf[9] >> 5) & 0x01 ;//-> PUMP6_STOP
            UI_PUMP1_SW_MAN[5] = (Uart_rx2_buf[9] >> 4) & 0x01 ;//-> PUMP6_MANUAL
            */
          //}
        }
        else if(Uart_rx2_buf[3] == 0x56){

        	SUB_Com_SW_V();

        }
      }
    }
	else if(Uart_rx2_buf[4] == 0x42){        // Relay board : '0x52'
		  if(Type_Mode == 1){      // Address MCC_Address[0] //벽부형시 EBC 응답
			if(Uart_rx2_buf[3] == 0x51){    // Report request : 'Q' ,'0x51'

				Uart_tx_Q_buf[0] = 0x53;    //S
				Uart_tx_Q_buf[1] = 0x54;    //T
				Uart_tx_Q_buf[2] = 0x42;    //B
				Uart_tx_Q_buf[3] = 0x52;    //R
				Uart_tx_Q_buf[4] = 0x01;    //address
				Uart_tx_Q_buf[5] = 0x0A;    //version 10 -> 1.0
				Uart_tx_Q_buf[6] = 0x00;
				Uart_tx_Q_buf[7] = 0x00;
				Uart_tx_Q_buf[8] = 0x00;
				Uart_tx_Q_buf[9] = 0x00;
				Uart_tx_Q_buf[10] = 0x00;
				Uart_tx_Q_buf[11] = 0x00;
				Uart_tx_Q_buf[12] = 0x00;
				Uart_tx_Q_buf[13] = 0x00;   //dummy1
				Uart_tx_Q_buf[14] = 0x00;   //dummy2
				Uart_tx_Q_buf[15] = 0x00;   //CRC
				Uart_tx_Q_buf[16] = 0x45;   //E
				Uart_tx_Q_buf[17] = 0x44;   //D

				Uart_tx_Q_buf[14] = 1;

				Uart_crc = 0;

				for(i = 0; i <tx_buf_len - 5 ; i++){
				  Uart_crc = Uart_crc ^ Uart_tx_Q_buf[i+2];
				}
				Uart_tx_Q_buf[15] = Uart_crc;

				HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
				HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

				HAL_Delay(1);

				if(HAL_UART_Transmit(&huart2, Uart_tx_Q_buf, sizeof(Uart_tx_Q_buf), 1000)!= HAL_OK)
				  {
					Error_Handler();
				  }

				HAL_Delay(1);

				HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);


			}
		  }
	}

    rx2_Receive_complete = 0;

  }
}


int SW_report_com(void){

  // CRC 체크

  int i , report = 0;

  // 전송할 중계기 정보를 확인한다.

  Uart2_crc = 0;

  for(i = 0; i <rx2_buf_count_tmp - 5 ; i++){
    Uart2_crc = Uart2_crc ^ Uart_rx2_buf[i+2];
  }

  if(Uart2_crc == Uart_rx2_buf[rx2_buf_count_tmp - 3]){
    //0x51 , 'Q' ,중계기 정보 요청 (UI에서 설정한 값만)

    if(Uart_rx2_buf[4] == 0x4D){        // Relay board : '0x52'
      if(Uart_rx2_buf[5] == MCC_Address[0]){      // Address 01
        if(Uart_rx2_buf[3] == 0x51){    // Report request : 'Q' ,'0x51'

          Uart_tx_buf[0] = 0x53;    //S
          Uart_tx_buf[1] = 0x54;    //T
          Uart_tx_buf[2] = 0x4D;    //M
          Uart_tx_buf[3] = 0x52;    //R
          Uart_tx_buf[4] = MCC_Address[0];    //address
          Uart_tx_buf[5] = SW_Version;    //version 10 -> 1.0
          Uart_tx_buf[6] = 0x00;    //LED정보 1 ~8
          Uart_tx_buf[7] = 0x00;    //LED정보 9 ~16
          Uart_tx_buf[8] = 0x00;    //LED정보 17 ~24
          Uart_tx_buf[9] = 0x00;    //LED정보 25 ~32
          Uart_tx_buf[10] = 0x00;   //LED정보 33 ~40
          Uart_tx_buf[11] = 0x00;   //Dip sw정보 1 ~8
          Uart_tx_buf[12] = 0x00;   //dummy1
          Uart_tx_buf[13] = 0x00;   //dummy2
          Uart_tx_buf[14] = 0x00;   //CRC
          Uart_tx_buf[15] = 0x45;   //E
          Uart_tx_buf[16] = 0x44;   //D



          Uart_tx_buf[6] = (LED_State[0] << 7)|(LED_State[1] << 6)|(LED_State[2] << 5)|(LED_State[3] << 4)|
                            (LED_State[4] << 3)|(LED_State[5] << 2)|(LED_State[6] << 1)|(LED_State[7] << 0);
          Uart_tx_buf[7] = (LED_State[8] << 7)|(LED_State[9] << 6)|(LED_State[10] << 5)|(LED_State[11] << 4)|
                            (LED_State[12] << 3)|(LED_State[13] << 2)|(LED_State[14] << 1)|(LED_State[15] << 0);
          Uart_tx_buf[8] = (LED_State[16] << 7)|(LED_State[17] << 6)|(LED_State[18] << 5)|(LED_State[19] << 4)|
                            (LED_State[20] << 3)|(LED_State[21] << 2)|(LED_State[22] << 1)|(LED_State[23] << 0);
          Uart_tx_buf[9] = (LED_State[24] << 7)|(LED_State[25] << 6)|(LED_State[26] << 5)|(LED_State[27] << 4)|
                            (LED_State[28] << 3)|(LED_State[29] << 2)|(LED_State[30] << 1)|(LED_State[31] << 0);
          Uart_tx_buf[10] = (LED_State[32] << 7)|(LED_State[33] << 6)|(LED_State[34] << 5);
          Uart_tx_buf[11] = ((~(Pump_Dip[0]) & 0x01)<< 7)|((~(Pump_Dip[1]) & 0x01)<< 6)|((~(Pump_Dip[2]) & 0x01)<< 5)|
                            ((~(Pump_Dip[3]) & 0x01)<< 4)|((~(Pump_Dip[4]) & 0x01)<< 3)|((~(Pump_Dip[5]) & 0x01)<< 2);


          Uart_crc = 0;

          for(i = 0; i <tx_buf_len - 5 ; i++){
            Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
          }
          Uart_tx_buf[tx_buf_len-3] = Uart_crc;

          HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
          HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

          HAL_Delay(1);

          if(HAL_UART_Transmit(&huart2, Uart_tx_buf, sizeof(Uart_tx_buf), 1000)!= HAL_OK)
              {
                Error_Handler();
              }

          HAL_Delay(1);

          HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

          report = 1;

        }
      }
    }
  }

  rx2_Receive_complete = 0;

  return report;

}

void Read_Address(void){

  if(HAL_GPIO_ReadPin(ADDR_DIP0_GPIO_Port, ADDR_DIP0_Pin) == GPIO_PIN_SET){
    Address_Dip[0] = 1;
  }
  else{
    Address_Dip[0] = 0;
  }

  if(HAL_GPIO_ReadPin(ADDR_DIP1_GPIO_Port, ADDR_DIP1_Pin) == GPIO_PIN_SET){
    Address_Dip[1] = 1;
  }
  else{
    Address_Dip[1] = 0;
  }

  if(HAL_GPIO_ReadPin(ADDR_DIP2_GPIO_Port, ADDR_DIP2_Pin) == GPIO_PIN_SET){
    Address_Dip[2] = 1;
  }
  else{
    Address_Dip[2] = 0;
  }

  if(HAL_GPIO_ReadPin(ADDR_DIP3_GPIO_Port, ADDR_DIP3_Pin) == GPIO_PIN_SET){
    Address_Dip[3] = 1;
  }
  else{
    Address_Dip[3] = 0;
  }
}

void Read_Pump_Dip(void){

  if(HAL_GPIO_ReadPin(PUMP1_DIP1_GPIO_Port , PUMP1_DIP1_Pin ) == GPIO_PIN_SET){
    Pump_Dip[0] = 1;
  }
  else{
    Pump_Dip[0] = 0;
  }

  if(HAL_GPIO_ReadPin(PUMP2_DIP2_GPIO_Port , PUMP2_DIP2_Pin ) == GPIO_PIN_SET){
    Pump_Dip[1] = 1;
  }
  else{
    Pump_Dip[1] = 0;
  }

  if(HAL_GPIO_ReadPin(PUMP3_DIP3_GPIO_Port , PUMP3_DIP3_Pin ) == GPIO_PIN_SET){
    Pump_Dip[2] = 1;
  }
  else{
    Pump_Dip[2] = 0;
  }

  if(HAL_GPIO_ReadPin(PUMP4_DIP4_GPIO_Port , PUMP4_DIP4_Pin ) == GPIO_PIN_SET){
    Pump_Dip[3] = 1;
  }
  else{
    Pump_Dip[3] = 0;
  }

  if(HAL_GPIO_ReadPin(PUMP5_DIP5_GPIO_Port , PUMP5_DIP5_Pin ) == GPIO_PIN_SET){
    Pump_Dip[4] = 1;
  }
  else{
    Pump_Dip[4] = 0;
  }

  if(HAL_GPIO_ReadPin(PUMP6_DIP6_GPIO_Port , PUMP6_DIP6_Pin ) == GPIO_PIN_SET){
    Pump_Dip[5] = 1;
  }
  else{
    Pump_Dip[5] = 0;
  }
}


/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}

void Save_Flash(uint32_t ST_Address){

  uint32_t Address , i, process_fail = 0;
  uint64_t Data64, Data64_L , Data64_H;


  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* Get the 1st page to erase */
  FirstPage = GetPage(ST_Address);
  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(ST_Address+FLASH_PAGE_SIZE) - FirstPage + 1;
  /* Get the bank */
  BankNumber = GetBank(FLASH_OPTR_IWDG_STDBY);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;


  process_fail = 0;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    /*
      Error occurred while mass erase.
      User can add here some code to deal with this error.
      To know the code error, user can call function 'HAL_FLASH_GetError()'
    */
    /* Infinite loop */
    process_fail = 1;
  }

  if(process_fail == 1){
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {

    }
  }
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = ST_Address;
  i = 0;
  process_fail = 0;

  while (Address < (ST_Address + (buf_len_flash) - 1))
  {
    Data64_L = (LED_State[i])|(LED_State[i+1]<<(8*1))|(LED_State[i+2]<<(8*2))|(LED_State[i+3]<<(8*3));
    Data64_H = (LED_State[i+4])|(LED_State[i+5]<<(8*1))|(LED_State[i+6]<<(8*2))|(LED_State[i+7]<<(8*3));
    Data64 = (Data64_H << 32) | (Data64_L);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data64) == HAL_OK)
    {
      Address = Address + 8;
      i = i+ 8;
    }
   else
    {
      process_fail = 1;
    }

    if(process_fail == 1){
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data64) == HAL_OK)
      {
        Address = Address + 8;
        i = i+ 8;
      }
    }
  }
  HAL_FLASH_Lock();
}

void Read_Flash(uint32_t ST_Address){

  int i;
  uint32_t Address = 0;

  Address =  ST_Address;
  for(i=0 ; i<buf_len_flash;i++){
   LED_State_tmp[i] = *(__IO uint8_t *)Address;
    Address = Address + sizeof(uint8_t);
  }


}

#define Sub_MCC_length      15

void SUB_Com_SW_V(void){

	uint8_t Uart_crc;

    Uart_tx_buf[0] = 0x53;    //S
    Uart_tx_buf[1] = 0x54;    //T
    Uart_tx_buf[2] = 0x53;    //S
    Uart_tx_buf[3] = 0x76;    //v
    Uart_tx_buf[4] = 0x4D;    //TB
    Uart_tx_buf[5] = MCC_Address[0];    //address
    Uart_tx_buf[6] = F_Version_Year;
    Uart_tx_buf[7] = F_Version_Month;
    Uart_tx_buf[8] = F_Version_Day;
    Uart_tx_buf[9] = F_Version_Hour;
    Uart_tx_buf[10] = F_Version_Min;
    Uart_tx_buf[11] = F_Version_Sec;
    Uart_tx_buf[12] = 0x00;   //CRC
    Uart_tx_buf[13] = 0x45;   //E
    Uart_tx_buf[14] = 0x44;   //D


    for(int i = 0; i <Sub_MCC_length - 5 ; i++){
      Uart_crc = Uart_crc ^ Uart_tx_buf[i+2];
    }
    Uart_tx_buf[Sub_MCC_length-3] = Uart_crc;


    HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_SET);

    HAL_Delay(1);
    if(HAL_UART_Transmit(&huart2, Uart_tx_buf, Sub_MCC_length, 1000)!= HAL_OK)
        {
          Error_Handler();
        }

    HAL_Delay(1);

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port , RS485_DE_Pin , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port , RS485_RE_Pin , GPIO_PIN_RESET);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
