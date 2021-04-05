/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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
CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s5;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* Default Settings */
char settingsMode = 'R';
uint8_t settingsVolume = 70;					// Potentiometer for volume
uint8_t settingsDataLength = 20;				// Databytes in audio packet
uint8_t settingsKeyPacketLength = 4;			// Aantal bytes voor sleutel (= #sleutelbits %8)
uint8_t settingsResolution = 8;					// 8 or 12 (ADC resolution)
uint8_t settingsEncryption = 1;					// 0 = off, 1 = on
uint32_t settingsFrequency = 245000;			// Frequency in kHz

/* General variables */
uint16_t adcVal;
uint16_t adcValDownSampled;

/* Debug variables */
uint8_t returnValue;
uint32_t counter;
uint32_t Packets_Received;
uint32_t Packets_Send;

//debug
uint8_t KeyCounter;
uint8_t KeyCounterLength = 5;
uint8_t TxKeys[5];
uint8_t RxKeys[5];
uint8_t RxKeysAfterHamming[5];
uint8_t TxRSSIValues[8];
uint8_t TxRSSIMeanValues[8];
uint8_t RxRSSIValues[8];
uint8_t RxRSSIMeanValues[8];
uint32_t TxCounterTable[8];
uint32_t RxCounterTable[8];

/* External interrupts */
uint8_t INT_PACKET_RECEIVED = 0;
uint8_t INT_PACKET_SEND = 0;

/* Tx variables */
cbuf_handle_t Tx_buffer_handle_t;
uint8_t TX_BUFFER_BASE;

/* Rx variables */
cbuf_handle_t Rx_buffer_handle_t;
uint8_t RX_BUFFER_BASE;
uint8_t Pkt_length;
uint8_t Pkt_type;
uint8_t Data_length;
uint8_t Rx_Encryption_byte;
uint8_t Rx_Dummy_byte;
uint8_t Rx_CRC_response;

/* Encryption variables */
uint32_t Key_32bit_Array[4] = {0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA};
uint32_t Key_32bit = 0xAAAAAAAA;

uint8_t Key_chosen_wait_timer;

uint32_t KeyCounter32bit;
uint8_t Key_32bit_Array_Index;

uint32_t Old_Key_32bit;
uint8_t Old_Key_32bit_index;


/* Button states */
static volatile int POWER_state = 1;
static volatile int TALK_state = 1;
static volatile int UP_state = 1;
static volatile int DOWN_state = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S5_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM9_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

/* Basic functions */
void Startup(void);
void Setup();

/* Audio functions */
uint8_t ReadPacket(void);
void generateKeyGraycode(void);
void makeGraycode(void);
void SendDummyByte(uint8_t);
void Send_e_line(void);


/* EEPROM functions */
void Clear_Flash_Sector4(void);
void Write_Flash(uint32_t, uint32_t);


/* -------------------------------------------------------------------------------------------------------------------- */
/* VARIABLES NEW ALGORITHM*/
uint8_t RSSI;
uint8_t RSSI_counter;

uint8_t RSSI_Measured[128];
// constantly checked if newly read RSSI value is not new min or new max
uint8_t RSSI_Range[2]; // max and min values of 128 measured RSSI values (~ form key)

uint8_t intervals = 8;
uint8_t quantisation_intervals[7]; // 7 levels to determine 8 intervals between RSSImax en RSSImin //quantisation_intervals[intervals-1]

uint8_t d0_line[128];
uint8_t d1_line[128];

uint8_t e_line[16]; //indexed by a


uint8_t keyGraycode_long[128];
uint32_t keyGraycode[4];
/* -------------------------------------------------------------------------------------------------------------------- */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Callback external interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case ADF7242_IRQ1_Pin:
			INT_PACKET_SEND = 1;
			ADF_clear_Tx_flag();
			Packets_Send++;

			break;

		case ADF7242_IRQ2_Pin:
			INT_PACKET_RECEIVED = 1;
			ADF_clear_Rx_flag();
			Packets_Received++;

			break;

		case BUTTON_TALK_Pin:
			if (TALK_state){
				TALK_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;

		case BUTTON_UP_Pin:
			if (UP_state){
				UP_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;

		case BUTTON_DOWN_Pin:
			if (DOWN_state){
				DOWN_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;

		case BUTTON_PWR_Pin:
			if (POWER_state){
				POWER_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}

			break;
	}
}

/* Callback timers */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/* Play audio samples on DAC */
	if (htim->Instance == TIM1){
		//Play_Audio();
	}

	if (htim->Instance == TIM5){
			SendDummyByte(0xAA);
		}

	/* Timer debouncing for external interrupts */
	if (htim->Instance == TIM11){
		if (HAL_GPIO_ReadPin(BUTTON_PWR_GPIO_Port, BUTTON_PWR_Pin)){
			POWER_state = 1;

			ADF_sleep();

			OLED_shutdown();												// Shutdown OLED display
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_RESET);			// Shutdown LM386
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_RESET);		// Shutdown the analog microphone

			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);								// Stop the DAC interface
			HAL_TIM_Base_Stop_IT(&htim1);									// Stop timer 1 (frequency = 8 kHz)

			HAL_Delay(250);

			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
			HAL_PWR_EnterSTANDBYMode();

			HAL_TIM_Base_Stop_IT(&htim11);
		}

		if (HAL_GPIO_ReadPin(BUTTON_TALK_GPIO_Port, BUTTON_TALK_Pin)){
			TALK_state = 1;

			if (settingsMode == 'R'){
				settingsMode = 'T';
			}
			else if (settingsMode == 'T'){
				settingsMode = 'R';
			}

			// Stop the TIM Base generation
			HAL_TIM_Base_Stop_IT(&htim11);

			// Initialize correct timers, components, interrupts...
			Setup();
		}


		/* Up-button is pressed in RX -> make threshold value larger */
		if (HAL_GPIO_ReadPin(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin)){
			UP_state = 1;
			if (settingsMode == 'R'){
				OLED_update();

				/*
				if (settingsVolume < 70)
				{
					settingsVolume = settingsVolume + 10;
					Potmeter_Init(settingsVolume);
					OLED_print_volume(settingsVolume);
					OLED_update();
				}
				else
				{
					OLED_print_volume(settingsVolume);
					OLED_update();
				}
				*/
			}

			// Stop the TIM Base generation
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		/* Down-button is pressed in RX -> make threshold value smaller */
		if (HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin)){
			DOWN_state = 1;
			if (settingsMode == 'R'){
				OLED_update();

				/*
				if (settingsVolume > 0)				{
					settingsVolume = settingsVolume - 10;
					Potmeter_Init(settingsVolume);
					OLED_print_volume(settingsVolume);
					OLED_update();
				}
				else				{
					OLED_print_volume(settingsVolume);
					OLED_update();
				}
				*/
			}
			HAL_TIM_Base_Stop_IT(&htim11);
		}
	}

	/* Update OLED display */
	if (htim->Instance == TIM9){
		/* Update every 3 seconds */


		OLED_print_date_and_time();

		if (settingsMode == 'T'){
			OLED_print_variable("Encryption: ", settingsEncryption, 0, 16);
			OLED_print_stoptalk();
		}
		else if (settingsMode == 'R'){
			OLED_print_talk();
		}

		OLED_update();
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
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2S5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_RTC_Init();
  MX_CRC_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  Startup();

  Clear_Flash_Sector4();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while (1){

  		// Introduce artificial delay between receive from RX and send new dummy packet to RX
  		/*//zie TIM5
	  	if (settingsMode == 'T'){
	  		////////////// Bepaalde delay voor entropy
		  	if (delay passed){ // werken met timers?
		  		SendDummyByte(0xAA);
		  	}
	  	}
	*/

	  	/*
	  	if (settingsMode == 'T'){
	  			  // Send audio packet
	  			  if (circular_buf_size(Tx_buffer_handle_t) > settingsDataLength){
	  				  if (settingsResolution == 8){
	  					  SendPacket8bit();
	  				  }
	  			  }
	  		  }
		*/


		// Packet received TX-side
		if(INT_PACKET_RECEIVED){
		  	INT_PACKET_RECEIVED = 0;

		  	// Extract Pkt_length, Pkt_type, Data_length, Data_array[Data_length] and RSSI from received package
			RSSI = ReadPacket();
			RSSI_counter++;

		  	/*
			// Debug: write RSSI value
			uint32_t address0 = 0x08012000 + (16*(RSSI_counter-1));
			Write_Flash(address0, (uint32_t) RSSI);
			*/

			// Update RSSI_Measured
		  	RSSI_Measured[RSSI_counter] = RSSI;

			// Update min and max RSSI on the fly */
			if(RSSI < RSSI_Range[0]){
				RSSI_Range[0] = RSSI;
			}
			else if(RSSI > RSSI_Range[1]){
				RSSI_Range[1] = RSSI;
			}


			if(RSSI_counter == 127){
				RSSI_counter = 0;
				makeGraycode(); // make e- and d-lines

				//reset RSSI_range //--> ["infinity", 0]
				RSSI_Range[0] = 0XFF; //min
				RSSI_Range[1] = 0X00; //max

				if(settingsMode == 'T'){
					Send_e_line();
					generateKeyGraycode();
					//save/write key to flash
					//NEEDS UPDATE
					uint32_t address1 = 0x08011000 + (4*(KeyCounter32bit-1));
				  	Write_Flash(address1, Key_32bit);
				}
			}


			/* Check packet type
					// 0xAA = dummy-packet -> change to audio packet later on in development
					// 0xFF = e-line
			*/
			/*
		  	if(Pkt_type == 0xFF){
				if(settingsMode == 'R'){
					SendDummyByte(0xAA); // Send dummy packet with type 0xFF after reception of e-line

					// Replace e-line from RX with e-line received from TX (~ stored in Rx_data if packet type = 0x...)

					//for(int i=0; i<sizeof(e_line)/sizeof(e_line[0]); i++){
				    //    e_line[i] = Data_array[i];
				    //}

					generateKeyGraycode();
					//save/write key to flash
					//NEEDS UPDATE
					uint32_t address1 = 0x08011000 + (4*(KeyCounter32bit-1));
				  	Write_Flash(address1, Key_32bit);
				}
			}
			*/
	  	}

		// Packet received RX-side
	  	if (INT_PACKET_SEND){
		  	INT_PACKET_SEND = 0;
		  	if (settingsMode == 'R'){
			  	SendDummyByte(0xAA);
			  	ADF_set_Rx_mode();

			  	// Extract Pkt_length, Pkt_type, Data_length, Data_array[Data_length] and RSSI from received package
				RSSI = ReadPacket();


			  	/*
				// Debug: write RSSI value
				uint32_t address0 = 0x08012000 + (16*(RSSI_counter-1));
				Write_Flash(address0, (uint32_t) RSSI);
				*/

				// Update RSSI_Measured
			  	RSSI_Measured[RSSI_counter] = RSSI;

				RSSI_counter++;

				// Update min and max RSSI on the fly */
				if(RSSI < RSSI_Range[0]){
					RSSI_Range[0] = RSSI;
				}
				else if(RSSI > RSSI_Range[1]){
					RSSI_Range[1] = RSSI;
				}



				if(RSSI_counter == 128){
					RSSI_counter = 0;
					makeGraycode(); // make e- and d-lines

					//reset RSSI_range //--> ["infinity", 0]
					RSSI_Range[0] = 0XFF; //min
					RSSI_Range[1] = 0X00; //max

					/*
					if(settingsMode == 'T'){
						Send_e_line();
						generateKeyGraycode();
						//save/write key to flash
						//NEEDS UPDATE
						uint32_t address1 = 0x08011000 + (4*(KeyCounter32bit-1));
					  	Write_Flash(address1, Key_32bit);
					}
					*/
				}


				/* Check packet type
						// 0xAA = dummy-packet -> change to audio packet later on in development
						// 0xFF = e-line
				*/
			  	if(Pkt_type == 0xFF){
					if(settingsMode == 'R'){
						SendDummyByte(0xAA); // Send dummy packet with type 0xFF after reception of e-line
						ADF_set_Rx_mode();
						// Replace e-line from RX with e-line received from TX (happens in ReadPacket with reception of 0xFF type)

						generateKeyGraycode();
						//save/write key to flash
						//NEEDS UPDATE
						uint32_t address1 = 0x08011000 + (4*(KeyCounter32bit-1));
					  	Write_Flash(address1, Key_32bit);
					}
				}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.I2SClockSelection = RCC_I2SAPBCLKSOURCE_PLLR;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S5_Init(void)
{

  /* USER CODE BEGIN I2S5_Init 0 */

  /* USER CODE END I2S5_Init 0 */

  /* USER CODE BEGIN I2S5_Init 1 */

  /* USER CODE END I2S5_Init 1 */
  hi2s5.Instance = SPI5;
  hi2s5.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s5.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s5.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s5.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s5.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s5.Init.CPOL = I2S_CPOL_LOW;
  hi2s5.Init.ClockSource = I2S_CLOCK_PLLR;
  hi2s5.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S5_Init 2 */

  /* USER CODE END I2S5_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 59;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 24;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 12000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 60000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 48000;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 48000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 500;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin|POT_CS_Pin|A_MIC_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_TALK_Pin */
  GPIO_InitStruct.Pin = BUTTON_TALK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_TALK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_UP_Pin BUTTON_OK_Pin BUTTON_DOWN_Pin ADF7242_IRQ1_Pin
                           ADF7242_IRQ2_Pin */
  GPIO_InitStruct.Pin = BUTTON_UP_Pin|BUTTON_OK_Pin|BUTTON_DOWN_Pin|ADF7242_IRQ1_Pin
                          |ADF7242_IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_PWR_Pin */
  GPIO_InitStruct.Pin = BUTTON_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_CS_Pin SWITCH_E_Pin POT_CS_Pin A_MIC_POWER_Pin */
  GPIO_InitStruct.Pin = ADF7242_CS_Pin|SWITCH_E_Pin|POT_CS_Pin|A_MIC_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void makeGraycode(void){
	// Setup quantisation intervals based on min and max RSSI
	uint8_t intervalWidth = (RSSI_Range[1]-RSSI_Range[0])/((sizeof(quantisation_intervals)/sizeof(quantisation_intervals[0])));
	for(int i = 0; i < sizeof(quantisation_intervals)/sizeof(quantisation_intervals[0]); i++){
		quantisation_intervals[i] = RSSI_Range[0] + i*intervalWidth;
	}


	// Set up Gray code
	uint8_t a = 15;

	for(int i=0; i < sizeof(RSSI_Measured)/sizeof(RSSI_Measured[0]); i++){
		if(RSSI_Measured[i] >= quantisation_intervals[4]){
			d1_line[i]=0x01;
		}
		else{
			d1_line[i]=0x00;
		}

		if(RSSI_Measured[i] >= quantisation_intervals[2] && RSSI_Measured[i] < quantisation_intervals[6]){
			d0_line[i]=0x01;
		}
		else{
			d0_line[i]=0x00;
		}


		if(settingsMode == 'T'){
			// Fix that we use 8-bit datatypes ~ uint8_t e_line[16] but we are using 128 samples
			if(i > 0 && i%8 == 0){
				a -= 1;
			}

			if(RSSI_Measured[i] < quantisation_intervals[1] || ((RSSI_Measured[i] >= quantisation_intervals[3]) && (RSSI_Measured[i] < quantisation_intervals[5])) || RSSI_Measured[i] >= quantisation_intervals[7]){
				e_line[a] &= ~(1UL << (i%8)); //clearing bit
			}
			else{
				e_line[a] |= (1UL << (i%8)); //setting bit
			}
		}
	}
}

void generateKeyGraycode(void){
	uint8_t a = 7;
	uint8_t b = 3;
	for(int i=0; i<128; i++){
		if(i > 0 && i%8 == 0){
			a -=1;
		}
		if(i > 0 && i%32 == 0){
			b -=1;
		}

		/* sleutel is wel omgekeerd maar maakt niet zoveel uit */
		if((e_line[a] >> (i%8)) & 1){
			if(d1_line[i] == 0x01){
				keyGraycode[b] |= (1UL << (i%32)); //setting bit
			}
			else{
				keyGraycode[b] &= ~(1UL << (i%32)); //clearing bit
			}
		}
		else{
			if(d0_line[i] == 0x01){
				keyGraycode[b] |= (1UL << (i%32)); //setting bit
			}
			else{
				keyGraycode[b] &= ~(1UL << (i%32)); //clearing bit
			}
		}
	}
}



void Setup(){
	switch(settingsMode){
		case 'T':
			OLED_print_text("Tx", 0, 0);
			OLED_print_stoptalk();
			OLED_update();

			/* Reverse HAL audio settings for Rx mode */
			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);										// Stop the DAC interface
			HAL_TIM_Base_Stop_IT(&htim1);											// Stop timer 1 (frequency = 8 kHz)

			/* ADF settings */
			ADF_set_turnaround_Tx_Rx();

			/* Power settings */
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_RESET);					// Shutdown LM386 to prevent power consumption
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_SET);				// Enable the analog microphone

			/* Buffer Settings for Tx mode */
			uint16_t Tx_buffer_size = 400;
			uint16_t *Tx_buffer = malloc(Tx_buffer_size * sizeof(uint16_t));		// Tx_buffer with size of 400 bytes
			Tx_buffer_handle_t = circular_buf_init(Tx_buffer, Tx_buffer_size);		// Tx buffer handle type

			HAL_TIM_Base_Start_IT(&htim5);

			break;

		case 'R':
			OLED_print_text("Rx", 0, 0);
			OLED_print_talk();
			OLED_update();


			HAL_TIM_Base_Stop_IT(&htim5);

			/* ADF settings */
			ADF_set_turnaround_Rx_Tx();

			/* Power settings */
			HAL_GPIO_WritePin(GPIOB, A_MIC_POWER_Pin, GPIO_PIN_RESET);				// Shutdown the analog microphone to prevent power consumption
			HAL_GPIO_WritePin(GPIOB, SWITCH_E_Pin, GPIO_PIN_SET);					// Enable the LM386

			/* Buffer Settings for Rx mode */
			uint16_t Rx_buffer_size = 400;
			uint16_t *Rx_buffer = malloc(Rx_buffer_size * sizeof(uint16_t));		// Rx_buffer with size of 400 bytes = 5 packets
			Rx_buffer_handle_t = circular_buf_init(Rx_buffer, Rx_buffer_size);		// Rx buffer handle type

			/* HAL audio settings for Rx mode */
			HAL_DAC_Start(&hdac, DAC_CHANNEL_1);									// Start the DAC interface
			HAL_TIM_Base_Start_IT(&htim1);											// Start timer 1 (frequency = 8 kHz)

			/* Rx mode */
			ADF_set_Rx_mode();														// Rx mode

			break;
	}
}


void SendDummyByte(uint8_t pkt_type){
	uint8_t PacketTotalLength = 6; // Packet type byte, Audio packet length byte, RSSI byte, SQI byte

	/*
	PACKET-STRUCTURE:
	[TOTALPACKETLENGTH --- PACKETTYPE --- DATALENGTH --- D___A___T___A --- RSSI --- SQI]
	*/

	uint8_t header[] = {0x10, PacketTotalLength, pkt_type, 0x01}; /*SPI_PKT_WR-command, TOTALPACKETLENGTH, packettype, packet length(= 0x01 byte)*/

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, header, 4);

	//write dummy data byte
	uint8_t sample[1] = {0x00};
	HAL_SPI_Transmit_IT(&hspi2, sample, 1);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	ADF_set_Tx_mode();
}


void Startup(void){
	/* Clear PWR wake up Flag */
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	/* Welcome screen */
	OLED_init();
	OLED_print_text("Jelle's", 39, 16);
	OLED_print_text("Walkie Talkie", 18, 26);
	OLED_update();
	HAL_Delay(1000);

	/* OLED settings */
	OLED_clear_screen();

	/* RTC time */
	OLED_print_date_and_time();

	OLED_update();

	ADF_Init(settingsFrequency);

	/* Setup */
	Setup();
}


void Send_e_line(void){
	uint8_t PacketTotalLength = 16 + 4; // Packet type byte, Audio packet length byte, RSSI byte, SQI byte

	/*
	PACKET-STRUCTURE:
	[TOTALPACKETLENGTH --- PACKETTYPE --- DATALENGTH --- e___L___i___n___e --- RSSI --- SQI]
	*/

	uint8_t header[] = {0x10, PacketTotalLength, 0xAA, 0x10}; /*SPI_PKT_WR-command, TOTALPACKETLENGTH, packettype, packet length(= 0x10 = 16 byte)*/

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, header, 4);

	// Write e-line bytes
	uint8_t sample[1];
	for (int i=0; i<sizeof(e_line)/sizeof(e_line[0]); i++)		{
		sample[0]=e_line[i];
		HAL_SPI_Transmit_IT(&hspi2, sample, 1);
	}

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	ADF_set_Tx_mode();
}



uint8_t ReadPacket(void){
	/* SPI_PKT_RD-command, SPI_NOP (use for dummy writes)*/
	uint8_t bytes[] = {0x30, 0xff};

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	// Send commands to ADF7242 that we want to read received packet
	HAL_SPI_Transmit_IT(&hspi2, bytes, 2);

	HAL_SPI_Receive_IT(&hspi2, &Pkt_length, 1);
	HAL_SPI_Receive_IT(&hspi2, &Pkt_type, 1);
	HAL_SPI_Receive_IT(&hspi2, &Data_length, 1);

	if(Pkt_type == 0xFF){ //received e-line from TX
		if(Data_length == 16){
			HAL_SPI_Receive_IT(&hspi2, e_line, Data_length);
		}
	}
	else{
		uint8_t Data_array[Data_length];
		HAL_SPI_Receive_IT(&hspi2, Data_array, Data_length);
	}


	HAL_SPI_Receive_IT(&hspi2, &RSSI, 1);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	while (ADF_SPI_READY() == 0);

	/*
	// Debug
	uint32_t address0 = 0x08012000 + (4*(RSSI_counter-1));
	Write_Flash(address0, (uint32_t) RSSI);
	*/

	return RSSI;
}


/********** EEPROM functions **********/
void Clear_Flash_Sector4(void){
     HAL_FLASH_Unlock();
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
     FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);
     HAL_FLASH_Lock();
}

void Write_Flash(uint32_t address, uint32_t data){
     HAL_FLASH_Unlock();
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
//     FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);
     HAL_FLASH_Program(TYPEPROGRAM_WORD, address, data);
     HAL_FLASH_Lock();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
