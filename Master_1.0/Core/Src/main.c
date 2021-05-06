/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <ssh1106.h>
#include "usbd_cdc_if.h"
#include "CBUF.h"
#include "string.h"
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
ADC_HandleTypeDef hadc1;

CRYP_HandleTypeDef hcryp;
__ALIGN_BEGIN static const uint32_t pKeyCRYP[4] __ALIGN_END = {
                            0x2B7E1516,0x28AED2A6,0xABF71588,0x09CF4F3C};

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* Default Settings */
char settings_mode = 'R';

/* External interrupts */
uint8_t INT_PACKET_RECEIVED = 0;
uint8_t INT_PACKET_SENT = 0;

/* Extra variables */
uint8_t ADF_status; //handig voor debug
uint8_t HGM = 0;
uint16_t packets_received = 0; //handig voor debug
uint16_t packets_received_prev = 0;
uint16_t packets_sent = 0;
uint16_t packets_sent_prev = 0;
uint32_t RSSI_prev = 0;

/* Tranciever variables */
uint32_t settings_frequency = 245000;			// Frequency in kHz
uint8_t TX_BUFFER_BASE;
uint8_t RX_BUFFER_BASE;

/* Rx variables */
uint8_t Pkt_length;
uint8_t Pkt_type;   //handig voor debug


/* Button states */
static volatile int POWER_state = 1;
static volatile int TALK_state = 1;
static volatile int UP_state = 1;
static volatile int DOWN_state = 1;
static volatile int LEFT_state = 1;
static volatile int RIGHT_state = 1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM9_Init(void);
static void MX_CRYP_Init(void);
/* USER CODE BEGIN PFP */

/* Interrupt functions */
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

/* Basic functions */
void startup(void);
void setup(void);

void LED_RGB_status(uint16_t , uint16_t , uint16_t );

/* Gray functions */
uint8_t ReadPacket(void);
void generateKeyGraycode(void);
void makeGraycode(void);
void SendDummyByte(uint8_t);
void Send_e_line(void);

/* Helper functions */
void delay_us (uint16_t us);


/* -------------------------------------------------------------------------------------------------------------------- */
/* VARIABLES NEW ALGORITHM*/
uint8_t dummy_byte;
uint8_t send_packet;
uint8_t send_e;
uint8_t RSSI;
uint8_t RSSI_counter;

uint8_t RSSI_Measured[128];
// Constantly checked if newly read RSSI value is not new min or new max
uint8_t RSSI_Range[2]; // max and min values of 128 measured RSSI values (~ form key)

uint8_t intervals = 8;
uint8_t quantisation_intervals[7]; // 7 levels to determine 8 intervals between RSSImax en RSSImin

uint8_t d0_line[128];
uint8_t d1_line[128];

uint8_t e_line[16]; //indexed by a

uint8_t keyGraycode_long[128];
uint32_t keyGraycode[4];
uint32_t key_counter = 0;
/* -------------------------------------------------------------------------------------------------------------------- */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_UART5_Init();
  MX_RTC_Init();
  MX_TIM5_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_CRYP_Init();
  /* USER CODE BEGIN 2 */

  /* Start PWM timers for RGB led */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* delay_us timer start */
  HAL_TIM_Base_Start(&htim7);

  LED_RGB_status(0, 0, 35);

  startup();

  ssh1106_Init();  // Initialise OLED
  ssh1106_SetCursor(10,10);
  ssh1106_WriteString ("Algoritme 2", Font_7x10, White);
  ssh1106_SetCursor(10, 30);
  ssh1106_WriteString ("Debugging", Font_7x10, White);
  ssh1106_UpdateScreen();
  ssh1106_SetDisplayOn(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while(1){

  		// Introduce artificial delay between receive from RX and send new dummy packet to RX
  		// !!!! TX is now transmitting every 5ms mogelijks niet deftig naar RX aan het gaan? !!!!

	  	if (settings_mode == 'T'){
		  	if(send_e & send_packet){
		  		// er mag een nieuw pakket gestuurd worden op TX + dit pakket moet e-lijn bevatten
		  		Send_e_line();
		  		send_e = 0;
		  		send_packet = 0;
		  	}
		  	else if (send_packet){
		  		// er mag een nieuw pakket gestuurd worden op TX
		  		SendDummyByte(0xDF);
		  		send_packet = 0;
		  	}
	  	}

		// Packet received
		if(INT_PACKET_RECEIVED){
		  	INT_PACKET_RECEIVED = 0;
		  	if(settings_mode == 'T'){

		  		ADF_set_Rx_mode();
		  		RSSI = ReadPacket(); // Extract Pkt_length, Pkt_type, Data and RSSI from received package

		  		/* Check packet type (soort beveiliging voor "random" pakketten)
		  		 	 0xDF = dummy-packet -> change to audio packet later on in development
		  		 	 0xEF = e-line
		  		*/

		  		if(Pkt_type == 0xDF){ //normaal krijgt hij enkel DF (dummy als TX)

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


					if(RSSI_counter == 128){ //als RSSI_counter = 128 ==> RSSI_Measured[127] werd ingevuld en daarna werd de counter verhoogd
						RSSI_counter = 0;
						makeGraycode(); // make e- and d-lines

						//reset RSSI_range //--> ["infinity", 0]
						//opmerking: wss zijn min en max ook omgekeerd omdat het tweecomplement is (255 ==> -127 dBm denk ik), maar vooral entropy is belangrijk dus maakt niet uit

						RSSI_Range[0] = 0XFF; //min
						RSSI_Range[1] = 0X00; //max

						if(settings_mode == 'T'){
							send_e = 1;
							generateKeyGraycode();
							key_counter++;
							// Transmit 32-bit key over USB (zal je 4 keer moeten doen om volledige 128-bit sleutel te printen [heb dit zelf nog niet gedaan])
						  	//opmerking: mss kunnen we ook de key_counter mee sturen, maar je moet goed opletten dat je buffer groot genoeg is, maar ook niet té groot omdat anders USB buffer vol komt
							uint8_t TxBuf[62];
							sprintf(TxBuf, "K:%lu;", keyGraycode[0]);
							CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
							HAL_Delay(200);
							for(int i=1; i<3; i++){
						  		sprintf(TxBuf, "%lu;", keyGraycode[i]);
						  		CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
						  		HAL_Delay(200);
						  	}
							sprintf(TxBuf, "%lu\r\n", keyGraycode[0]);
							CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
							HAL_Delay(200);

						}
					}
		  		}
		  	}

		  	else if (settings_mode == 'R'){

				ADF_set_Rx_mode(); // gebeurt normaal automatisch nadat een pakket gestuurd is door de ADF_set_turnaround_Tx_Rx !!!! Nakijken
				RSSI = ReadPacket(); // Extract Pkt_length, Pkt_type, data and RSSI from received package
				HAL_Delay(1);

		  		if(Pkt_type == 0xDF || Pkt_type == 0xEF){
		  			SendDummyByte(0xDF); // Enkel antwoorden na filter
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

					if(RSSI_counter == 128){ //als RSSI_counter = 128 ==> RSSI_Measured[127] werd ingevuld en daarna werd de counter verhoogd
						RSSI_counter = 0;
						makeGraycode(); // make e- and d-lines

						//reset RSSI_range //--> ["infinity", 0]
						RSSI_Range[0] = 0XFF; //min
						RSSI_Range[1] = 0X00; //max

						/* Check packet type */
							// 0xDF = dummy-packet -> change to audio packet later on in development
							// 0xEF = e-line

					}

					if(Pkt_type == 0xEF){
							//if(RSSI_counter == 0){ //zie ppt dia 22
							//e-line replacement happens in readPacket (e-line from TX is directly read in this array)

							generateKeyGraycode();
							key_counter++;

							// Transmit 32-bit key over USB (zal je 4 keer moeten doen om volledige 128-bit sleutel te printen [heb dit zelf nog niet gedaan])
						  	//opmerking: mss kunnen we ook de key_counter mee sturen, maar je moet goed opletten dat je buffer groot genoeg is, maar ook niet té groot omdat anders USB buffer vol komt

							uint8_t TxBuf[34];
							sprintf(TxBuf, "K:%lu;", keyGraycode[0]);
							CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
							HAL_Delay(200);
							for(int i=1; i<3; i++){
						  		sprintf(TxBuf, "%lu;", keyGraycode[i]);
						  		CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
						  		HAL_Delay(200);
						  	}
							sprintf(TxBuf, "%lu\r\n", keyGraycode[0]);
							CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
							HAL_Delay(200);
						//}
					}
		  		}

		  	}
		}

	  	if (INT_PACKET_SENT){
		  	INT_PACKET_SENT = 0;
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T5_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRYP Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRYP_Init(void)
{

  /* USER CODE BEGIN CRYP_Init 0 */

  /* USER CODE END CRYP_Init 0 */

  /* USER CODE BEGIN CRYP_Init 1 */

  /* USER CODE END CRYP_Init 1 */
  hcryp.Instance = CRYP;
  hcryp.Init.DataType = CRYP_DATATYPE_8B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.pKey = (uint32_t *)pKeyCRYP;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_BYTE;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRYP_Init 2 */

  /* USER CODE END CRYP_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 31;
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
  hspi1.Init.CRCPolynomial = 10;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 8400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = (2625)*2-1;
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
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 2625-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  htim9.Init.Prescaler = 4000-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 42000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim11.Init.Prescaler = 48000-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 175-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CLASS_D_SHDN_Pin|MIC_SHDN_Pin|DPOT_CS_Pin|ADF7242_GP3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA_LNA_HGM_Pin|ADF7242_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ADF7242_GP1_Pin|ADF7242_GP0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLASS_D_SHDN_Pin MIC_SHDN_Pin DPOT_CS_Pin ADF7242_GP3_Pin */
  GPIO_InitStruct.Pin = CLASS_D_SHDN_Pin|MIC_SHDN_Pin|DPOT_CS_Pin|ADF7242_GP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA_LNA_HGM_Pin ADF7242_CS_Pin */
  GPIO_InitStruct.Pin = PA_LNA_HGM_Pin|ADF7242_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_IRQ1_Pin BTN_TALK_Pin BTN_PWR_Pin */
  GPIO_InitStruct.Pin = ADF7242_IRQ1_Pin|BTN_TALK_Pin|BTN_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_GP1_Pin ADF7242_GP0_Pin */
  GPIO_InitStruct.Pin = ADF7242_GP1_Pin|ADF7242_GP0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF7242_IRQ2_Pin BTN_UP_Pin BTN_RIGHT_Pin BTN_LEFT_Pin
                           BTN_DOWN_Pin */
  GPIO_InitStruct.Pin = ADF7242_IRQ2_Pin|BTN_UP_Pin|BTN_RIGHT_Pin|BTN_LEFT_Pin
                          |BTN_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LBO_Pin */
  GPIO_InitStruct.Pin = LBO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LBO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* Callback external interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	switch(GPIO_Pin){
		case ADF7242_IRQ1_Pin:
			INT_PACKET_SENT = 1;
			packets_sent++;
			ADF_clear_Tx_flag();
			break;

		case ADF7242_IRQ2_Pin:
			INT_PACKET_RECEIVED = 1;
			packets_received++;
			delay_us(10);
			ADF_clear_Rx_flag();
			break;

		case BTN_TALK_Pin:
			if(TALK_state){
				TALK_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}
			break;

		case BTN_UP_Pin:
			if(UP_state){
				UP_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}
			break;


		case BTN_LEFT_Pin:
			if(LEFT_state){
				LEFT_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}
			break;

		case BTN_DOWN_Pin:
			if(DOWN_state){
				DOWN_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}
			break;

		case BTN_PWR_Pin:
			if(POWER_state){
				POWER_state = 0;
				HAL_TIM_Base_Start_IT(&htim11);
			}
			break;

		case LBO_Pin:
			LED_RGB_status(15,0,0);
			break;

	}
}


/* Callback timers */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
		// verander TIM2 zodat frequentie = +-166 Hz (elke 6 ms pakket)
		// moet aangepast kunnen worden naar +-333 Hz (elke 3 ms), +-93 Hz (elke 12 ms), +-41 ms (elke 24 ms)
		// als we snel pakketten sturen, werkt het algoritme wss minder goed en genereert deze sleutels met ZEER slechte entropie
		// hoe trager we gaan sturen, zou het hopelijk een hogere entropy moeten hebben
		send_packet = 1;
	}

	// Timer for external interrupts (buttons)
	if (htim->Instance == TIM11){
		// Power button pressed
		if (HAL_GPIO_ReadPin(BTN_PWR_GPIO_Port, BTN_PWR_Pin)){

			//ADD FUNCTIONALITY

			POWER_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);

		}

		// Talk button pressed
		if (HAL_GPIO_ReadPin(BTN_TALK_GPIO_Port, BTN_TALK_Pin)){
			// Change mode
			if (settings_mode == 'R'){
				settings_mode = 'T';
			}
			else if (settings_mode == 'T'){
				settings_mode = 'R';
			}

			TALK_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);

			// Initialize correct timers, components, interrupts for selected mode
			setup();

		}

		// Up button pressed
		if (HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin)){
			//Set High Gain Mode
			if(settings_mode == 'R'){
				if (HGM){
					LED_RGB_status(15, 0, 20);
					HGM =0;
					HAL_GPIO_WritePin(PA_LNA_HGM_GPIO_Port, PA_LNA_HGM_Pin, GPIO_PIN_SET);
				}
				else{
					LED_RGB_status(0, 0, 10);
					HGM =1;
					HAL_GPIO_WritePin(PA_LNA_HGM_GPIO_Port, PA_LNA_HGM_Pin, GPIO_PIN_RESET);
				}
			}

			UP_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		// Down button pressed
		if (HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin)){

			//ADD FUNCTIONALITY

			DOWN_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		// Left button pressed
		if (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin)){

			//ADD FUNCTIONALITY

			LEFT_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		// Right button pressed
		if (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin)){

			//ADD FUNCTIONALITY

			RIGHT_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}


	}
}

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


		if(settings_mode == 'T'){
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

		/* sleutel is wel "omgekeerd" (~LSB-first?) maar maakt niet zoveel uit, kunnen we in excel desnoods ez omdraaien. Het belangrijkste is de sleutelentropy*/
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


void startup(void){
	// Clear PWR wake up Flag
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	// Setup registers for transceiver
	ADF_Init(settings_frequency);

	// Shutdown Class D audio amplifier
	HAL_GPIO_WritePin(CLASS_D_SHDN_GPIO_Port, CLASS_D_SHDN_Pin, GPIO_PIN_RESET); //GPIO_PIN_RESET
	// Shutdown microphone preamplifier
	HAL_GPIO_WritePin(MIC_SHDN_GPIO_Port, MIC_SHDN_Pin, GPIO_PIN_RESET);

	HAL_Delay(500);

	ADF_set_turnaround_Tx_Rx();

	// Setup for MCU
	setup();
}

void setup(){
	switch(settings_mode){
		case 'T':
			LED_RGB_status(0, 10, 0);

			// Stop the DAC interface and timer2 (8 kHz)
			//HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

			/* Start transmit timer */
			HAL_TIM_Base_Start_IT(&htim2);


			// Start timer5 (16 kHz) and ADC interrupt triggered by TIM5
			//HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_1);
			//HAL_ADC_Start_IT(&hadc1);

			//HAL_TIM_Base_Start_IT(&htim9);

			break;

		case 'R':
			LED_RGB_status(0, 0, 10);

			// Stop timer5 (16 kHz) and ADC interrupt triggered by TIM5
			HAL_TIM_OC_Stop(&htim5, TIM_CHANNEL_1);
			//HAL_ADC_Stop_IT(&hadc1);

			/* Start transmit timer */
			HAL_TIM_Base_Stop_IT(&htim2);

			// Start the DAC interface and timer2 (8 kHz)
			//HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			//HAL_TIM_Base_Start_IT(&htim2);

			ADF_set_Rx_mode();
			break;
	}

}


void LED_RGB_status(uint16_t red, uint16_t green, uint16_t blue){
	// Brightness limiting due to low resistance values
	if(red>20){
		red = 20;
	}
	if(green>30){
		green = 30;
	}
	if(blue>35){
		blue = 35;
	}
	htim1.Instance->CCR1 = red;
	htim3.Instance->CCR4 = green;
	htim3.Instance->CCR3 = blue;
}

void delay_us (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim7,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim7) < us);  // wait for the counter to reach the us input in the parameter
}

void SendDummyByte(uint8_t type){
	uint8_t packet_length = 5; // Packet length (1byte), Packet type (1byte), dummy byte [0xAB] (1byte), RSSI byte (1byte)

	uint8_t header[] = {0x10, packet_length, type, 0x01, 0x02};

	// Write packet to RAM
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, header, sizeof(header)/sizeof(header[0]));
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	// Set RC to TX
	ADF_set_Tx_mode();
	while(ADF_SPI_READY()==0);
}

void Send_e_line(void){
	uint8_t packet_length = 3 + 16; // Packet length, Packet type, E_L_I_N_E (16 bytes), RSSI byte

	uint8_t header[] = {0x10, packet_length, 0xEF}; /*SPI_PKT_WR-command, TOTALPACKETLENGTH, packettype, packet length(= 0x10 = 16 byte)*/

	// Write packet to RAM
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, header, sizeof(header)/sizeof(header[0]));
	// Write e-line bytes
	uint8_t sample[1];
	for (int i = 0; i<sizeof(e_line)/sizeof(e_line[0]); i++){
		sample[0]=e_line[i];
		HAL_SPI_Transmit_IT(&hspi1, sample, 1);
	}
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	// Set RC to TX
	ADF_set_Tx_mode();
}



uint8_t ReadPacket(void){
	/* SPI_PKT_RD-command, SPI_NOP (use for dummy writes)*/
	uint8_t bytes[] = {0x30, 0xff};

	while (ADF_SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);

	// Send commands to ADF7242 that we want to read received packet
	HAL_SPI_Transmit_IT(&hspi1, bytes, 2);

	HAL_SPI_Receive_IT(&hspi1, &Pkt_length, 1);
	HAL_SPI_Receive_IT(&hspi1, &Pkt_type, 1);


	if(Pkt_type == 0xEF){
		// e-line from TX
		HAL_SPI_Receive_IT(&hspi1, e_line, 16);
		HAL_SPI_Receive_IT(&hspi1, &RSSI, 1);
	}
	else if(Pkt_type == 0xDF){
		// dummy packet from RX
		HAL_SPI_Receive_IT(&hspi1, &dummy_byte, 1);
		HAL_SPI_Receive_IT(&hspi1, &dummy_byte, 1);
		HAL_SPI_Receive_IT(&hspi1, &RSSI, 1);
	}

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
	while (ADF_SPI_READY() == 0);


	//opmerking: misschien een veiligheid inbouwen mocht je terug vaak random pakketten krijgen?
	return RSSI; //RSSI vaak tussen 150 en 250
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
