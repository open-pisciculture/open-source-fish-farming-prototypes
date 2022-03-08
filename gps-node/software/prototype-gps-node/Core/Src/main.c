/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * Structure for storing the gps information received
 */
typedef struct {
	float latitude ;
	float longitude;
	float utctime;
	char ns;
	char ew;
	char valid;
}GPS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*  Lora message structure, 10 bytes long
 *  Bytes 3-0:		latitude
 *  Bytes 7-4:		longitude
 *  Byte 8:			North or South
 *  Byte 9:			East or West
 */
#define LORA_MESSAGE_NUM_BYTES      10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*
 * Choose the LoRa Spreading Factor with this macro.
 */
#define LORA_SF 					DR_SF7
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/*
 * Stores the current index in the GPS buffer
 */
uint32_t RxIndex = 0;

/*
 * The current data received from the GPS
 */
uint8_t RxData = 0;

/*
 * Stores all the data received by the GPS
 */
uint8_t GPSBuffer[100] = {0};

/*
 * Variable for storing the GPS information received.
 */
GPS gps = {0};

/*
 * Stores the current tick
 */
static volatile uint32_t tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/*
 * GPS private function prototypes
 */
void SaveGPSData();

/*
 * LoRaWAN private function prototypes
 */
uint8_t getSysTickActiveCounterFlag();

/*
 * Other private function prototypes
 */
void MYPRINT(char *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * LoRaWAN configuration
 * Should be edited according to the information from The Things Network
 */

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t APPEUI[8] = {0x5F, 0x9B, 0x1C, 0x3F, 0x8B, 0x4A, 0x4A, 0x00};
void os_getArtEui(u1_t *buf) {
	memcpy(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t DEVEUI[8] = { 0x5F, 0x37, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t *buf) {
	memcpy(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t APPKEY[16] = { 0xA4, 0x38, 0xC0, 0x57, 0x17, 0xEF, 0xEF, 0xBB, 0xE2, 0x5D, 0xB4, 0x9B, 0x7B, 0xB1, 0x34, 0x63 };
void os_getDevKey(u1_t *buf) {
	memcpy(buf, APPKEY, 16);
}

/*
 * Array that stores the LoRaWAN packet payload
 * In this case corresponds to the GPS information.
 */
static uint8_t loradata[LORA_MESSAGE_NUM_BYTES] = { 0 };

/*
 * TODO: Llenar esto del port y el confirmed bien
 */
static uint8_t lora_port = 2;
static uint8_t lora_confirmed = 0;

/*
 * Structure for storing the LoRaWAN transmissions
 */
static osjob_t sendjob;

/*
 * TODO: Este del retry interval no se si se usa aqui
 */
static uint16_t retry_interval = 1;

/*
 * Indicates if a transmission was completed
 */
static volatile uint8_t txcomplete = 0;

/*
 * Pin mapping, unused in this implementation
 */
const lmic_pinmap lmic_pins = { .nss = 6, .rxtx = LMIC_UNUSED_PIN, .rst = 5,
		.dio = { 2, 3, LMIC_UNUSED_PIN }, };

/*
 * Schedules the next transmission, if possible.
 */
void do_send(osjob_t *j) {
	lmic_tx_error_t error = 0;

	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		MYPRINT("OP_TXRXPEND, not sending\r\n");
	} else {
		MYPRINT("Packet queued\r\n");
		// Prepare upstream data transmission at the next possible time.
		error = LMIC_setTxData2(lora_port, loradata, LORA_MESSAGE_NUM_BYTES,
				lora_confirmed);
	}

	switch (error) {
	case LMIC_ERROR_TX_BUSY:
		MYPRINT("LMIC_ERROR_TX_BUSY\r\n");
		break;
	case LMIC_ERROR_TX_TOO_LARGE:
		MYPRINT("LMIC_ERROR_TX_TOO_LARGE\r\n");
		break;
	case LMIC_ERROR_TX_NOT_FEASIBLE:
		MYPRINT("LMIC_ERROR_TX_NOT_FEASIBLE\r\n");
		break;
	case LMIC_ERROR_TX_FAILED:
		MYPRINT("LMIC_ERROR_TX_FAILED\r\n");
		break;
	default:
		break;
	}
}

/*
 * TODO: LoRaWAN Events
 */
void onEvent(ev_t ev) {
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		MYPRINT("EV_SCAN_TIMEOUT\r\n");
		break;
	case EV_BEACON_FOUND:
		MYPRINT("EV_BEACON_FOUND\r\n");
		break;
	case EV_BEACON_MISSED:
		MYPRINT("EV_BEACON_MISSED\r\n");
		break;
	case EV_BEACON_TRACKED:
		MYPRINT("EV_BEACON_TRACKED\r\n");
		break;
	case EV_JOINING:
		// normally starts at DR_SF7 and increases if not heard
		// /* set SF10 to get reliable join, potentially decreased over time by ADR */
		// LMIC_setDrTxpow(DR_SF10,14);
		MYPRINT("EV_JOINING\r\n");
		break;
	case EV_JOINED:
		MYPRINT("EV_JOINED\r\n");
		/* enable ADR */
//		LMIC_setAdrMode(1);
		LMIC_setAdrMode(0);
		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
		LMIC_setLinkCheckMode(0);
		break;
	case EV_JOIN_FAILED:
		MYPRINT("EV_JOIN_FAILED\r\n");
		break;
	case EV_REJOIN_FAILED:
		MYPRINT("EV_REJOIN_FAILED\r\n");
		break;
	case EV_TXCOMPLETE:
		MYPRINT("EV_TXCOMPLETE\r\n");
		if (LMIC.txrxFlags & TXRX_ACK) {
			MYPRINT("Received ack\r\n");
		}

		if (LMIC.dataLen) {
			MYPRINT("Received data\r\n");

//              fence_parse_rx(&(LMIC.frame[LMIC.dataBeg]), LMIC.dataLen);
		}
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		txcomplete = 1;
		break;
	case EV_JOIN_TXCOMPLETE:
		// join accept not received
		MYPRINT("EV_JOIN_TXCOMPLETE\r\n");
		break;
	case EV_LOST_TSYNC:
		MYPRINT("EV_LOST_TSYNC\r\n");
		break;
	case EV_RESET:
		MYPRINT("EV_RESET\r\n");
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		MYPRINT("EV_RXCOMPLETE\r\n");
		break;
	case EV_LINK_DEAD:
		MYPRINT("EV_LINK_DEAD\r\n");
		break;
	case EV_LINK_ALIVE:
		MYPRINT("EV_LINK_ALIVE\r\n");
		break;
	case EV_TXSTART:
		MYPRINT("EV_TXSTART\r\n");
		break;
	default:
		MYPRINT("Unknown event\r\n");
		break;
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);

  MYPRINT("LMiC init\r\n");
  LMIC_setDrTxpow(LORA_SF, 14);
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);
  txcomplete = 0;

  //TODO: lo de los subchannels llenarlo bien

  //Aqui comienza la Prueba
  LMIC_selectSubBand(1);
  //Disable FSB1, channels 0-7
  for(int i = 0; i < 7; i++)
  {
	  //		  if (i != 10)
	  LMIC_disableChannel(i);
  }

  //Disable FSB3-8, channels 16-72+
  for (int i = 16; i < 73; i++)
  {
	  //		  if (i != 10)
	  LMIC_disableChannel(i);
  }

  //Adaptive Data Rate is disabled for this prototype
  LMIC_setAdrMode(0);
  LMIC.dn2Dr = LORA_SF;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(LORA_SF, 14);

  GPSInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  os_runloop_once();
	  /* when no jobs are pending and not joining, toggle led */
	  if (!os_queryTimeCriticalJobs(sec2osticks(retry_interval)) && !(LMIC.opmode & OP_JOINING) && !(LMIC.opmode & OP_TXRXPEND))
	  {
		  if (txcomplete) {
			HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
			HAL_Delay(1000);
		  }
		  else
		  {
			// TODO: timeout with txcomplete check
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFM95_RST_GPIO_Port, RFM95_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFM95_RST_Pin */
  GPIO_InitStruct.Pin = RFM95_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFM95_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO2_Pin DIO1_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin|DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*
 * Callback that stores the data recieved from the GPS module into the GPSBuffer
 *
 * The character '$' indicates the beginning. The RxIndex is reset to 0 in this case and the GPSBuffer is overwritten. Information continues to be received from the GPS module.
 * The character '\n' indicates the end. Coordinates are retrieved from the buffer and GPS module data reception is disabled until the user button is pressed.
 * For other characters the data is saved in the buffer and information continues to be received from the GPS module.
 */
void UART_GPS_Handler()
{
	if (RxData == '$')
	{
		RxIndex = 0;
		GPSBuffer[RxIndex] = RxData;
		HAL_UART_Receive_IT(&huart3, &RxData, 1);
	}
	else if (RxData == '\n')
	{
		RxIndex++;
		GPSBuffer[RxIndex] = RxData;
		SaveGPSData();
	}
	else
	{
		RxIndex++;
		GPSBuffer[RxIndex] = RxData;
		HAL_UART_Receive_IT(&huart3, &RxData, 1);
	}
}

/*
 * Initializes the GPS data reception, disables the user button and turns the user led on.
 */
void GPSInit()
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 1);
	txcomplete = 0;
	char message[] = "Taking reading...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) message, sizeof(message), 100);
	HAL_UART_Receive_IT(&huart3, &RxData, 1);
}

/*
 * Begins GPS data reception.
 */
void GPSRetry()
{
	HAL_UART_Receive_IT(&huart3, &RxData, 1);
}

/*
 * Checks the GPS buffer for the $GPGLL message.
 * If $GPGLL was received, the latitude, longitude, north/south and east/west coordinates are saved to the LoRaWAN payload array and tranmission is scheduled.
 * If any other type of message was received or the $GPGLL message was incorrectly read, the GPS buffer is discarded and GPS data reception continues until
 * the $GPGLL message is correctly received.
 */
void SaveGPSData()
{
	if (!strncmp((char *) GPSBuffer, "$GPGLL", 6))
	{
		if (sscanf((char *) GPSBuffer, "$GPGLL,%f,%c,%f,%c,%f,%c", &gps.latitude, &gps.ns, &gps.longitude, &gps.ew, &gps.utctime, &gps.valid) != 6)
		{
			GPSRetry();
		}
		else
		{
			char message[] = "Location obtained.\r\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *) message, sizeof(message), 100);

//			GPS *ptr = (GPS *) loradata;
//			*ptr = gps;

			float *ptr = (float *) loradata;
			*ptr = gps.latitude;
			*(ptr+1) = gps.longitude;

			loradata[8] = gps.ns;
			loradata[9] = gps.ew;

			do_send(&sendjob);
		}
	}
	else
	{
		GPSRetry();
	}
}

/*
 * Function for printing messages using the USART2 peripheral.
 */
void MYPRINT(char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg) + 1,
	HAL_MAX_DELAY); //TODO: Revisar lo del timeout
}

/*
 * Configures the system tick interrupt
 * Should increase every 1ms
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
	/*Configure the SysTick to have interrupt in 1ms time basis*/
	HAL_SYSTICK_Config(SystemCoreClock / 1000);

	/*Configure the SysTick IRQ priority */
	HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0);

	/* Return function status */
	return HAL_OK;
}

/*
 * Obtains the current microseconds
 */
uint32_t getCurrentMicro(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
//  LL_SYSTICK_IsActiveCounterFlag();
  getSysTickActiveCounterFlag();

  //TODO: Revisar porque hice doble este codigo
  uint32_t m = HAL_GetTick();
  uint32_t u = SysTick->LOAD - SysTick->VAL;

  if(getSysTickActiveCounterFlag())
  {
    m = HAL_GetTick();
    u = SysTick->LOAD - SysTick->VAL;
  }
  return ( m * 1000 + (u * 1000) / SysTick->LOAD);
}

/*
 * TODO: Acordarme esto que hacia
 */
uint8_t getSysTickActiveCounterFlag()
{
	return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

/*
 * Increases the tick
 */
void HAL_IncTick(void)
{
	tick += (uint32_t) uwTickFreq;
}

/*
 * Returns the current tick
 */
uint32_t HAL_GetTick(void)
{
	return tick;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
