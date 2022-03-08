/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *-
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*  Lora message structure, 8 bytes long
 *  Byte 1-0:		Microcontroller Internal Temperature
 *  Byte 3-2:		Water Temperature
 *  Byte 5-4:		Water pH
 *  Byte 7-6:		Water Dissolved Oxygen
 */
#define LORA_MESSAGE_NUM_BYTES      8

/*
 * Location in memory of the temperature sensor factor calibration data
 * Each device is individually factory-calibrated by ST
 * Can be used to obtain more accurate temperature values
 * These are the memory addresses for the STM32L476xx microcontroller. These might not be the same for
 * other microcontrollers and should be consulted in the datasheet before reading the temperature values
 *
 * TEMPCAL1ADDR stores the temperature sensor ADC raw data acquired at 30°C (+- 5°C) at a reference voltage of 3.0V (+- 10mV)
 * TEMPCAL2ADDR stores the temperature sensor ADC raw data acquired at 110°C (+- 5°C) at a reference voltage of 3.0V (+- 10mV)
 */
#define TEMPCAL1ADDR				(0x1FFF75A8)
#define TEMPCAL2ADDR				(0x1FFF75CA)

/*
 * Sensor I2C addresses
 * Can be modified with any number between 1 and 127 with the command I2C,#NEWADDR, where #NEWADDR is the new address
 * All three sensors must have different addresses
 */
#define TEMP_SENSOR_I2C_ADDRESS 	102
#define PH_SENSOR_I2C_ADDRESS 		103
#define DO_SENSOR_I2C_ADDRESS 		104

/*
 * Macro that enables or disables LoRaWAN Adaptive Data Rate (ADR)
 * ADR optimizes data rates, airtime and energy consumption
 * Should probably be enabled, except for specific cases
 */
#define ENABLE_ADR					1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/*
 * Variable that stores the current tick
 */
static volatile uint32_t tick = 0;

/*
 * Structure that contains the Flash configuration for erasing
 */
FLASH_EraseInitTypeDef FlashEraseInit = {0};
uint32_t flashError = 0;

/*
 * Variable that indicates if the microcontroller woke up from Standby Mode
 * This is done by checking the Standby flag (SBF) in the Power status register 1 (PWR_SR1)
 *
 */
uint8_t from_standby = 0;

/*
 * Variables that store the adc raw data found at TEMPCAL1ADDR and TEMPCAL2ADDR
 * These will be corrected depending on the reference voltage used for the ADC
 */
float tempcal1;
float tempcal2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/*
 * Function prototypes
 */
void RTC_WakeupConfig();
//void MYPRINT(char *msg);
uint8_t getSysTickActiveCounterFlag();
void MCUTempInit();
uint16_t readMCUTemp();
uint8_t remove_decimal(char* pStr, char c);
void readSensors(AtlasSensorData* pSample);
void WriteFlash(uint8_t data);
void InitFlashErase();
void SaveSession();
void RestoreSession();
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
static const u1_t APPEUI[8] = { 0x97, 0x18, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui(u1_t *buf) {
	memcpy(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t DEVEUI[8] = { 0x5F, 0x9B, 0x1C, 0x3F, 0x8B, 0x4A, 0x4A, 0x00 };
void os_getDevEui(u1_t *buf) {
	memcpy(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t APPKEY[16] = { 0x07, 0x28, 0x32, 0x62, 0xF7, 0x0A, 0x6B, 0x8F,
		0x57, 0x24, 0x47, 0x77, 0xE3, 0xBB, 0xF0, 0xD6 };
void os_getDevKey(u1_t *buf) {
	memcpy(buf, APPKEY, 16);
}

/*
 * Array that stores the LoRaWAN packet payload
 * In this case, this corresponds to the 4 variables read from the sensors
 */
static uint8_t loradata[LORA_MESSAGE_NUM_BYTES] = { 0 };

/*
 * TODO
 */
static uint8_t lora_port = 2;
static uint8_t lora_confirmed = 0;

/*
 * Structure for storing the LoRaWAN transmissions
 */
static osjob_t sendjob;

/*
 * This is the array where the tick, overflow and LoRaWAN session are stored before entering Standby mode
 * A section of memory was reserved for user data. Check the STM32L476RGTXFLASH.ld file for details.
 */
__attribute__((__section__(".user_data"))) uint8_t userConfig[1024];
const uint32_t LMIC_BYTE_SIZE = sizeof(LMIC);

/*
 * Go to sleep if there are no pending jobs for the next interval stored in this variable
 */
static uint16_t retry_interval = 60;

/*
 * Indicates when a transmission was completed
 */
static volatile uint8_t txcomplete = 0;

/*
 * The sleep period in seconds.
 * Used to set the wakeup timer interrupt.
 */
const uint32_t SLEEP_SECONDS = 60;

/*
 * Pointers to the temperature sensor calibration data
 */
uint16_t *pTempCal1 = (uint16_t *) TEMPCAL1ADDR;
uint16_t *pTempCal2 = (uint16_t *) TEMPCAL2ADDR;

// Pin mapping, unused in this implementation
//Lo meti en el main.h
const lmic_pinmap lmic_pins = { .nss = 6, .rxtx = LMIC_UNUSED_PIN, .rst = 5,
		.dio = { 2, 3, LMIC_UNUSED_PIN }, };

/*
 * Schedules the next transmission, if possible.
 */
void do_send(osjob_t *j) {
	lmic_tx_error_t error = 0;

	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
//		("OP_TXRXPEND, not sending\r\n");
	} else {
//		MYPRINT("Packet queued\r\n");
		// Prepare upstream data transmission at the next possible time.
		error = LMIC_setTxData2(lora_port, loradata, LORA_MESSAGE_NUM_BYTES,
				lora_confirmed);
	}

	switch (error) {
	case LMIC_ERROR_TX_BUSY:
//		MYPRINT("LMIC_ERROR_TX_BUSY\r\n");
		break;
	case LMIC_ERROR_TX_TOO_LARGE:
//		MYPRINT("LMIC_ERROR_TX_TOO_LARGE\r\n");
		break;
	case LMIC_ERROR_TX_NOT_FEASIBLE:
//		MYPRINT("LMIC_ERROR_TX_NOT_FEASIBLE\r\n");
		break;
	case LMIC_ERROR_TX_FAILED:
//		MYPRINT("LMIC_ERROR_TX_FAILED\r\n");
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
//		MYPRINT("EV_SCAN_TIMEOUT\r\n");
		break;
	case EV_BEACON_FOUND:
//		MYPRINT("EV_BEACON_FOUND\r\n");
		break;
	case EV_BEACON_MISSED:
//		MYPRINT("EV_BEACON_MISSED\r\n");
		break;
	case EV_BEACON_TRACKED:
//		MYPRINT("EV_BEACON_TRACKED\r\n");
		break;
	case EV_JOINING:
		// normally starts at DR_SF7 and increases if not heard
		// /* set SF10 to get reliable join, potentially decreased over time by ADR */
		// LMIC_setDrTxpow(DR_SF10,14);
//		MYPRINT("EV_JOINING\r\n");
		break;
	case EV_JOINED:
//		MYPRINT("EV_JOINED\r\n");
		/* enable ADR */
		LMIC_setAdrMode(ENABLE_ADR);
		// Disable link check validation (automatically enabled
		// during join, but not supported by TTN at this time).
		LMIC_setLinkCheckMode(0);
		break;
	case EV_JOIN_FAILED:
//		MYPRINT("EV_JOIN_FAILED\r\n");
		break;
	case EV_REJOIN_FAILED:
//		MYPRINT("EV_REJOIN_FAILED\r\n");
		break;
	case EV_TXCOMPLETE:
//		MYPRINT("EV_TXCOMPLETE\r\n");
		if (LMIC.txrxFlags & TXRX_ACK) {
//			MYPRINT("Received ack\r\n");
		}

		if (LMIC.dataLen) {
//			MYPRINT("Received data\r\n");

//              fence_parse_rx(&(LMIC.frame[LMIC.dataBeg]), LMIC.dataLen);
		}
		txcomplete = 1;
		break;
	case EV_JOIN_TXCOMPLETE:
		// join accept not received
//		MYPRINT("EV_JOIN_TXCOMPLETE\r\n");
		break;
	case EV_LOST_TSYNC:
//		MYPRINT("EV_LOST_TSYNC\r\n");
		break;
	case EV_RESET:
//		MYPRINT("EV_RESET\r\n");
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
//		MYPRINT("EV_RXCOMPLETE\r\n");
		break;
	case EV_LINK_DEAD:
//		MYPRINT("EV_LINK_DEAD\r\n");
		break;
	case EV_LINK_ALIVE:
//		MYPRINT("EV_LINK_ALIVE\r\n");
		break;
	case EV_TXSTART:
//		MYPRINT("EV_TXSTART\r\n");
		break;
	default:
//		MYPRINT("Unknown event\r\n");
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
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  RTC_WakeupConfig();
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_TIM_Base_Start(&htim3);
  InitFlashErase();
  MCUTempInit();

  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
	  from_standby = 1;

	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	  //MYPRINT("Woke up from standby mode...\r\n");
  }
  else
  {
	  from_standby = 0;
	  //MYPRINT("System running after reset...\r\n");
  }

  //	MYPRINT("Start LMIC\r\n");

  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
  AtlasSensorData sample = {0};
  readSensors(&sample);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  txcomplete = 0;
//  txcomplete = 1;

  if(from_standby)
  {
	  //Woke up from Standby mode

	  //Restore session and send one measurement
	  RestoreSession();

	  //MYPRINT("Session restored...\r\n");

//	  MYPRINT("Report queued...\r\n");
  }
  else
  {
	  // settings

	  //The internal RC clock is used here and the clock error is set to 5%
	  //If other clocks are used, such as an external crystal oscillator, this error may be lower
	  LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

	  //Disable unavailable channels
	  //TODO: Explicar este mejor que es importante

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

	  LMIC_setAdrMode(ENABLE_ADR);
	  LMIC.dn2Dr = DR_SF10;

	  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	  LMIC_setDrTxpow(DR_SF10, 14);
  }

  //TODO: Organizar mejor esto del loradata, quizas meterlo en un metodo?
  loradata[0] = (uint8_t) (sample.mcu_temperature >> 8);
  loradata[1] = (uint8_t) sample.mcu_temperature;
  loradata[2] = (uint8_t) (sample.temperature >> 8);
  loradata[3] = (uint8_t) sample.temperature;
  loradata[4] = (uint8_t) (sample.pH >> 8);
  loradata[5] = (uint8_t) sample.pH;
  loradata[6] = (uint8_t) (sample.dissolvedOxygen >> 8);
  loradata[7] = (uint8_t) sample.dissolvedOxygen;

  //Schedules the next transmission with the obtained samples
  do_send(&sendjob);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	os_runloop_once();

	//Yo quiero quitar esto despues.
	/* when no jobs pending and not joining, go to standby */
	if (!os_queryTimeCriticalJobs(sec2osticks(retry_interval)) && !(LMIC.opmode & OP_JOINING) && !(LMIC.opmode & OP_TXRXPEND))
		{
			if (txcomplete)
			{
				//TODO: Poner el codigo que vaya a standby mode.
				//Seria chevere poner una macro para facilmente cambiar entre standby mode y delay
	//				MYPRINT("Save session\r\n");

	//				MYPRINT("Standby\r\n")*-/8-;

	//			MYPRINT("Entering standby mode...\r\n");
//				HAL_GPIO_WritePin(RFM95_CONTROL_GPIO_Port, RFM95_CONTROL_Pin, GPIO_PIN_SET);
//				SaveSession();
//				HAL_PWR_EnterSTANDBYMode();
				HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
				HAL_Delay(1000);
			}
			else
			{
				// TODO: timeout with txcomplete check
			}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00303D5B;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim3.Init.Prescaler = 16;
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
  HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_NSS_Pin|RFM95_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SENSORS_PWR_Pin */
  GPIO_InitStruct.Pin = SENSORS_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSORS_PWR_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFM95_RST_Pin */
  GPIO_InitStruct.Pin = RFM95_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFM95_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//void MYPRINT(char *msg) {
//	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg) + 1, HAL_MAX_DELAY); //TODO: Revisar lo del timeout
//}

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

  //TODO: Revisar porque se hace doble este codigo
  getSysTickActiveCounterFlag();

  //Porque se hace esto 2 veces?
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
 * Removes the character indicated in the parameter c from the array indicated by the pointer pStr.
 * Used to remove the dot character from the received sensor values
 * Returns the location of the dot character. However, this is unused.
 */
uint8_t remove_decimal(char* pStr, char c)
{
	char *pr = pStr, *pw = pStr;
	uint8_t loc = 0; uint8_t i = 0;

	while(*pr)
	{
		*pw = *pr++;

		if (*pw != c)
		{
			pw++;
		}
		else
		{
			loc = i;
		}
		i++;
	}

	*pw = '\0';

	return loc;
}

/*
 * Reads the raw ADC temperature sensor calibration values and adjusts for the difference in voltage reference values.
 * Here, the reference voltage used is 3.3V. This should be edited if the reference voltage changes.
 * TODO: Quizas poner una macro? No podria probar que funciona...
 */
void MCUTempInit()
{
	tempcal1 = (float) (*pTempCal1)*(3/3.3f);
	tempcal2 = (float) (*pTempCal2)*(3/3.3f);
}

/*
 * Reads the internal microcontroller temperature.
 * Returns the value as an unsigned integer. The correct temperature value can be recovered by dividing by 1000.
 * This is done to reduce the size of the variable by 2 bytes.
 */
uint16_t readMCUTemp()
{
	HAL_ADC_Start(&hadc1);
	while(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK);

	uint16_t adctemp = (uint16_t) HAL_ADC_GetValue(&hadc1);
	float temp = ((float)(adctemp - tempcal1)/(tempcal2 - tempcal1))*(80) + 30;

	return (uint16_t) (temp*1000);
}

/*
 * Reads the values from each sensor.
 * The sensor circuits must be in I2C mode with a correct I2C address assigned.
 * After reading, the decimal character is removed and the values are stored as unsigned integers to reduce the variable size for transmission.
 * The correct values can be recovered by dividing by 1000. This is done, in this case, by the payload formatter in The Things Network.
 */
void readSensors(AtlasSensorData* pSample)
{
	uint16_t mcuTemp = readMCUTemp();
	pSample->mcu_temperature = mcuTemp;

	char cmd[] = "R";
//	char cmd[] = "Cal,21";
	uint8_t temp_buffer[8] = {0};
	uint8_t pH_buffer[7] = {0};
	uint8_t do_buffer[7] = {0};

	//  uint8_t ready = 254;

	HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, GPIO_PIN_SET);

	HAL_Delay(1500);

	uint8_t temp_fail = HAL_I2C_Master_Transmit(&hi2c3, TEMP_SENSOR_I2C_ADDRESS << 1, (uint8_t*) cmd, sizeof(cmd), 1000) != HAL_OK;
	uint8_t do_fail = HAL_I2C_Master_Transmit(&hi2c3, DO_SENSOR_I2C_ADDRESS << 1, (uint8_t*) cmd, sizeof(cmd), 1000) != HAL_OK;
	uint8_t pH_fail = HAL_I2C_Master_Transmit(&hi2c3, PH_SENSOR_I2C_ADDRESS << 1, (uint8_t*) cmd, sizeof(cmd), 1000) != HAL_OK;

	HAL_Delay(1500);

	if(!temp_fail)
	{
		HAL_I2C_Master_Receive(&hi2c3, TEMP_SENSOR_I2C_ADDRESS << 1, temp_buffer, sizeof(temp_buffer), 1000);
	}

	if(!do_fail)
	{
		HAL_I2C_Master_Receive(&hi2c3, DO_SENSOR_I2C_ADDRESS << 1, do_buffer, sizeof(do_buffer), 1000);
	}

	if(!pH_fail)
	{
		HAL_I2C_Master_Receive(&hi2c3, PH_SENSOR_I2C_ADDRESS << 1, pH_buffer, sizeof(pH_buffer), 1000);
	}

	uint8_t decloc_temp = remove_decimal( (char*) (temp_buffer+1), '.');
	uint8_t decloc_do = remove_decimal( (char*) (do_buffer+1), '.');
	uint8_t decloc_pH = remove_decimal( (char*) (pH_buffer+1), '.');

	//Avoids the unused warnings
	UNUSED(decloc_temp);
	UNUSED(decloc_do);
	UNUSED(decloc_pH);

	pSample->temperature = atoi( (const char*) (temp_buffer+1));
	pSample->dissolvedOxygen = atoi( (const char*) (do_buffer+1));
	pSample->pH = atoi( (const char*) (pH_buffer+1));

	HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, GPIO_PIN_RESET);
}

/*
 * Configures the wakeup timer interrupt according to the sleep period indicated in SLEEP_SECONDS.
 * An additional second is substracted to correct for initialization time.
 */
void RTC_WakeupConfig()
{
	//Substracting an additional second because of the initialization time.
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, SLEEP_SECONDS - 1 - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
		{
			Error_Handler();
		}
	//	MYPRINT("RTC configured.\r\n");
}

/*
 * Function called when the wakeup interrupt occurs.
 * Reconfigures the wakeup timer, reads the sensors and schedules the next LoRaWAN transmission.
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
//	MYPRINT("Wakeup Triggered.\r\n");
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);
	txcomplete = 0;
	RTC_WakeupConfig();
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

	AtlasSensorData sample = {0};
	readSensors(&sample);

	loradata[0] = (uint8_t) (sample.mcu_temperature >> 8);
	loradata[1] = (uint8_t) sample.mcu_temperature;
	loradata[2] = (uint8_t) (sample.temperature >> 8);
	loradata[3] = (uint8_t) sample.temperature;
	loradata[4] = (uint8_t) (sample.pH >> 8);
	loradata[5] = (uint8_t) sample.pH;
	loradata[6] = (uint8_t) (sample.dissolvedOxygen >> 8);
	loradata[7] = (uint8_t) sample.dissolvedOxygen;

	do_send(&sendjob);
}

/*
 * Sets the flash information for erasing
 */
void InitFlashErase()
{
	FlashEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashEraseInit.Banks = FLASH_BANK_2;
	FlashEraseInit.Page = 448;
	FlashEraseInit.NbPages = 64;
}

/*
 * Stores the current tick (adjusted with the sleep period), the overflow and the LMIC structure in the Flash memory.
 * This is important, because the microcontroller must load this information after waking up from Standby mode to transmit data.
 *
 * This is not the best solution. Depending on the sleep period, the Flash memory may quickly degrade and
 * fail. A better solution involves using EEPROM memory. However, the current STM32L476RG microcontroller
 * doesn't include EEPROM. Thus, a different microcontroller would have to be used that includes EEPROM memory
 * or an external EEPROM chip should be included. 1KB should be enough to store this information.
 */
void SaveSession()
{
	//1. Unlock the flash and erase pages 448-551
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	HAL_FLASHEx_Erase(&FlashEraseInit, &flashError);

	//2. Store the current tick + the amount of ticks that would occur during the sleep period.
	//Since SLEEP_SECONDS is in seconds and each tick occurs every 1ms, this amount is SLEEP_SECONDS*1000
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t) (userConfig), tick + SLEEP_SECONDS*1000) != HAL_OK)
	{
		Error_Handler();
	}

	//3. Saves the overflow
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t) (userConfig+8), hal_get_overflow()) != HAL_OK)
	{
		Error_Handler();
	}

	//4. Saves the LMIC structure containing the LMIC information
	uint64_t* pLMIC = (uint64_t*) &LMIC;

	uint32_t i = 16;

	while(i < LMIC_BYTE_SIZE + 16)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t) (userConfig+i), *pLMIC) != HAL_OK)
		{
			Error_Handler();
		}
		pLMIC++;
		i+=8;
	}
}

/*
 * Loads the stored session from the Flash memory
 */
void RestoreSession()
{
	tick = *((uint32_t*)(userConfig));

	hal_set_overflow(*(uint8_t*) (userConfig+8));

	LMIC = *((struct lmic_t*) (userConfig+16));
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
	while (1) {
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
