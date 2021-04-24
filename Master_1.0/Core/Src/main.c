/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ssh1106.h"
#include "OLED.h"
#include <math.h>
#include "CBUF.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t ID;
	uint32_t RSSI_counter;			// received packets
	uint16_t RSSI_Mean;				// exponential moving average
	double RSSI_Mean_Double;

	uint8_t keybits_8bit;			// current number of bits of 8-bit key
	uint8_t key_8bit;				// current 8-bit key

	uint8_t keybytes_32bit;			// current number of new 8-bit keys in 32-bit key
	uint32_t key_32bit;				// current 32-bit key
	uint16_t key_counter_32bit;		// number of generated 32-bit keys

	uint8_t keywords_128bit;		// current number of new 32-bit keys in 128-bit key (also serves as index)
	uint32_t key_128bit[4];			// current 128-bit key
	uint16_t key_counter;			// number of generated 128-bit keys

	uint8_t key_chosen_wait_timer;	// delay for new keybit
} device;

// Pointer to device struct
device *ptrdev;

// Struct init device1
device dev1 = {
		ID: 0x01,
		RSSI_counter: 0x00,
		RSSI_Mean: 0x0000,
		RSSI_Mean_Double:  0,
		keybits_8bit: 0,
		key_8bit: 0xA1,
		keybytes_32bit: 0,
		key_32bit: 0x80808080,
		key_counter_32bit: 0,
		keywords_128bit: 0,
		key_128bit: {0x2B7E1516,0x28AED2A6,0xABF71588,0x09CF4F3C},
		key_counter: 0x00,
		key_chosen_wait_timer: 0
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint32_t time_start, time_end, time_diff;
#define TIME_MEASURE_START time_start = SysTick->VAL
#define TIME_MEASURE_STOP time_end = SysTick->VAL; \
		time_diff = time_start - time_end

/*
// Start point for SysTick
HAL_SYSTICK_Config(0xFFFFFFF);
TIME_MEASURE_START;
...
TIME_MEASURE_STOP;
*/

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

/* ---Default settings--- */
char settings_mode = 'R';								// mode can be R (RX) or T (TX)

#define settings_audiosamples_length 48					// samplecount per audio packet -> packetrate = 8000^-1 * settings_audiosamples_length
uint8_t settings_volume = 30;
uint8_t HGM = 0;										// 0 = LGM LNA, 1 = HGM LNA
uint8_t settings_encryption = 1;						// 0 = off, 1 = on
uint8_t settings_threshold = 3;							// threshold for choosing keybits
double ALPHA = 0.1;										// weighting factor exponential moving average
const uint8_t settings_keybit_delay = 8;				// packets between new possible keybit

uint8_t setting_debugscreen = 0;

uint8_t encryption_byte = 0;

uint8_t dest_ID = 0xFF;
const uint8_t source_ID = 0x01;
const uint8_t broadcast_ID = 0xFF;


/* ---Packet settings--- */
const uint8_t packet_type_audio = 0xFE;
const uint8_t packet_type_audio_encrypted = 0xFF;

const uint8_t packet_type_keybit_chosen = 0xAA;			// keybit chosen by TX
const uint8_t packet_type_keybit_chosen_Hamming = 0xAB; // 8th keybit chosen by TX + Hamming
const uint8_t packet_type_keybit_chosen_CRC = 0xAC; 	// 8th keybit chosen by TX + Hamming (4th byte) + CRC

const uint8_t packet_type_keybit_CRC_ok = 0xA0;			// CRC ok reply by RX
const uint8_t packet_type_keybit_CRC_bad = 0xB0;		// CRC bad reply by RX

const uint8_t packet_type_reply = 0x0F;

/* ---Packet reception (read)--- */
uint8_t Rx_packet_length;
uint8_t Rx_packet_type;
uint8_t Rx_from_ID;
uint8_t Rx_to_ID;
uint8_t Rx_data_length;
uint8_t Hamming;
uint8_t Rx_RSSI;

/* ---External interrupts--- */
uint8_t INT_PACKET_RECEIVED = 0;
uint8_t INT_PACKET_SENT = 0;

uint16_t packets_received = 0;
uint16_t packets_received_prev = 0;
uint16_t packets_sent = 0;
uint16_t packets_sent_prev = 0;
uint32_t RSSI_prev = 0;

/* ---Button states for debouncing--- */
static volatile int POWER_state = 1;
static volatile int TALK_state = 1;
static volatile int UP_state = 1;
static volatile int DOWN_state = 1;
static volatile int LEFT_state = 1;
static volatile int RIGHT_state = 1;

/* ---Audio--- */
uint8_t adc_value = 0;
uint16_t adc_value_downsampled = 0;
uint32_t adc_counter = 0;
cbuf_handle_t audio_buffer_handle_t;
uint16_t cbuf_size = 0;
uint8_t data[settings_audiosamples_length];
uint8_t samples[settings_audiosamples_length];

/* ---Transceiver variables--- */
const uint32_t settings_frequency = 245000;				// ADF frequency in [Hz/10kHz] -> 2,45 GHz
// Buffer starting address
uint8_t RX_BUFFER_BASE;
uint8_t TX_BUFFER_BASE;

uint8_t ADF_status;										// SPI status word return ADF7242


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

void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);

void startup(void);
void setup(void);

void digipotInit(uint8_t);
void LED_RGB_status(uint16_t , uint16_t , uint16_t );

void playAudio(void);

void test_transmitDummyPacket(void);
void test_receiveDummyPacket(void);
void transmitAudioPacket(void);
void readPacket(void);

void writeKeyPacket(void);
void writeKeybitPacket(device*, uint8_t);

uint8_t Hamming_create(uint8_t);
uint8_t Hamming_correct(uint8_t, uint8_t);

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

  HAL_TIM_Base_Start(&htim7);


  // Start PWM timers for RGB led
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  LED_RGB_status(0, 0, 35);

  startup();
  digipotInit(settings_volume);

  // OLED interrupt TIM9 (1s)
  HAL_TIM_Base_Start_IT(&htim9);

  // USB VCP variables
  int8_t buffer[25];
  int16_t RSSI_buf_16[16];

  ptrdev = &dev1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	  if (settings_mode == 'T') {
		  // Send audio packet whenever there are enough samples in circular buffer
		  cbuf_size = circular_buf_size(audio_buffer_handle_t);
		  if (cbuf_size > settings_audiosamples_length){
			  transmitAudioPacket();
		  }
	  }

	  if (INT_PACKET_RECEIVED){
		  ADF_clear_Rx_flag(); //test
		  INT_PACKET_RECEIVED = 0;

		  if (settings_mode == 'R'){
			  readPacket();
			  if((Rx_to_ID == source_ID) || (Rx_to_ID == broadcast_ID)){
				  //writeKeyPacket();
				  encryption_byte = packet_type_reply;
				  if (!(ptrdev->RSSI_counter)){
					  ptrdev->RSSI_Mean_Double = Rx_RSSI;
					  ptrdev->RSSI_Mean = round(ptrdev->RSSI_Mean_Double);
				  }
				  else{
					  ptrdev->RSSI_Mean_Double = (ALPHA*Rx_RSSI) + ((1-ALPHA)*ptrdev->RSSI_Mean_Double);
					  ptrdev->RSSI_Mean = round(ptrdev->RSSI_Mean_Double);
				  }
				  (ptrdev->RSSI_counter)++;


				  if(Rx_packet_type == packet_type_keybit_chosen){
					  // Point to correct device with Rx_from_ID

					  // Generate new keybit
					  if(Rx_RSSI <= ptrdev->RSSI_Mean){
						  if(ptrdev->keybits_8bit < 8){
							  (ptrdev->key_8bit) = ((ptrdev->key_8bit)<<1) | 0;
							  (ptrdev->keybits_8bit)++;
						  }
					  }
					  else if(Rx_RSSI > ptrdev->RSSI_Mean){
						  if(ptrdev->keybits_8bit < 8){
							  (ptrdev->key_8bit) = ((ptrdev->key_8bit)<<1) | 1;
							  (ptrdev->keybits_8bit)++;
						  }
					  }

					  encryption_byte = packet_type_reply;
				  }

				  else if(Rx_packet_type == packet_type_keybit_chosen_Hamming){
					  // Point to correct device with Rx_from_ID

					  // Generate last keybit of 8-bit key
					  if(Rx_RSSI <= ptrdev->RSSI_Mean){
						  (ptrdev->key_8bit) = ((ptrdev->key_8bit)<<1) | 0;
					  }
					  else if(Rx_RSSI > ptrdev->RSSI_Mean){
						  (ptrdev->key_8bit) = ((ptrdev->key_8bit)<<1) | 1;
					  }

					  // Do a Hamming correction with Rx_Hamming_code
					  (ptrdev->key_8bit) = Hamming_correct(Hamming, ptrdev->key_8bit);

					  // Shift 8-bit key in 32-bit key + update parameters
					  ptrdev->key_32bit = ((ptrdev->key_32bit)<<8) | (ptrdev->key_8bit);
					  ptrdev->key_8bit = 0;
					  ptrdev->keybits_8bit = 0;
					  (ptrdev->keybytes_32bit)++;

					  encryption_byte = packet_type_reply;

					  // Transmit over USB
					  uint8_t TxBuf[32];
					  sprintf(TxBuf, "%lu\r\n", (unsigned long) (ptrdev->key_32bit));
					  CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
				  }

				  else if(Rx_packet_type == packet_type_keybit_chosen_CRC){
					  // Point to correct device with Rx_from_ID

					  // Generate last keybit of 8-bit key
					  if(Rx_RSSI <= ptrdev->RSSI_Mean){
						  (ptrdev->key_8bit) = ((ptrdev->key_8bit)<<1) | 0;
					  }
					  else if(Rx_RSSI > ptrdev->RSSI_Mean){
						  (ptrdev->key_8bit) = ((ptrdev->key_8bit)<<1) | 1;
					  }

					  // Do a Hamming correction with Rx_Hamming_code
					  (ptrdev->key_8bit) = Hamming_correct(Hamming, ptrdev->key_8bit);

					  // Shift 8-bit key in 32-bit key + update parameters
					  ptrdev->key_32bit = ((ptrdev->key_32bit)<<8) | (ptrdev->key_8bit);
					  ptrdev->key_8bit = 0;
					  ptrdev->keybits_8bit = 0;
					  (ptrdev->keybytes_32bit) = 0;

					  // Generate CRC of Rx-key + do CRC check with Tx CRC

					  // If CRC does match, set packet_type = CRC_OK & add 32bit key to 128bit key; if CRC doesn't match, set packet_type = CRC_BAD
					  encryption_byte = packet_type_keybit_CRC_ok;
				  }

				  else if(Rx_packet_type == packet_type_audio_encrypted){
					  HAL_CRYP_Decrypt(&hcryp, data, settings_audiosamples_length, samples, 50);
					  for(uint8_t i = 0; i < settings_audiosamples_length; i++){
						  circular_buf_put_overwrite(audio_buffer_handle_t, samples[i]);
					  }
					  encryption_byte = packet_type_reply;
				  }
				  else if(Rx_packet_type == packet_type_audio){
					  for(uint8_t i = 0; i < settings_audiosamples_length; i++){
						  circular_buf_put_overwrite(audio_buffer_handle_t, data[i]);
					  }
					  encryption_byte = packet_type_reply; //hoeft eigenlijk geen packetten te sturen als reply
				  }

				  writeKeybitPacket(ptrdev, encryption_byte);

				  // Werkt
				  //sprintf(buffer, "%d %d\r\n", (int8_t) Rx_RSSI, (int8_t) ptrdev->RSSI_Mean);
				  //CDC_Transmit_FS((int8_t *)buffer, strlen(buffer));
			  }
		  }

		  else if (settings_mode == 'T'){
			  readPacket();
			  if (!(ptrdev->RSSI_counter)){
				  ptrdev->RSSI_Mean_Double = Rx_RSSI;
				  ptrdev->RSSI_Mean = round(ptrdev->RSSI_Mean_Double);
			  }
			  else{
				  ptrdev->RSSI_Mean_Double = (ALPHA*Rx_RSSI) + ((1-ALPHA)*ptrdev->RSSI_Mean_Double);
				  ptrdev->RSSI_Mean = round(ptrdev->RSSI_Mean_Double);
			  }
			  (ptrdev->RSSI_counter)++;


			  // ---Key generation algorithm TX---
			  // Wait for 100 RSSI values
			  if(ptrdev->RSSI_counter > 100){
				  // Delay for new keybit is passed
				  if (ptrdev->key_chosen_wait_timer == 0){
					  // RSS below threshold -> keybit = 0
					  if (Rx_RSSI < (ptrdev->RSSI_Mean - settings_threshold)){
						  if ((ptrdev->keybits_8bit) < 8){
							  ptrdev->key_8bit = ((ptrdev->key_8bit)<<1) | 0;
							  (ptrdev->keybits_8bit)++;
							  encryption_byte = packet_type_keybit_chosen;

							  ptrdev->key_chosen_wait_timer = settings_keybit_delay;
						  }
					  }
					  // RSS above threshold -> keybit = 1
					  else if (Rx_RSSI > (ptrdev->RSSI_Mean + settings_threshold)){
						  if ((ptrdev->keybits_8bit) < 8){
							  ptrdev->key_8bit = ((ptrdev->key_8bit)<<1) | 1;
							  (ptrdev->keybits_8bit)++;
							  encryption_byte = packet_type_keybit_chosen;

							  ptrdev->key_chosen_wait_timer = settings_keybit_delay;
						  }
					  }
				  }
				  else{
					  (ptrdev->key_chosen_wait_timer)--;
				  }

				  // Hamming
				  if(ptrdev->keybits_8bit == 8){
					  // Prepare Hamming-code
					  Hamming = Hamming_create(ptrdev->key_8bit);

					  // Shift 8-bit key in 32-bit key
					  ptrdev->key_32bit = ((ptrdev->key_32bit)<<8) | (ptrdev->key_8bit);
					  // Reset 8-bit key
					  ptrdev->key_8bit = 0;
					  ptrdev->keybits_8bit = 0;
					  // Update number of 8-bit keys in 32-bit key
					  (ptrdev->keybytes_32bit)++;

					  encryption_byte = packet_type_keybit_chosen_Hamming;

					  // Transmit over USB
					  uint8_t TxBuf[32];
					  sprintf(TxBuf, "%lu\r\n", (unsigned long) (ptrdev->key_32bit));
					  CDC_Transmit_FS((int8_t *)TxBuf, strlen(TxBuf));
				  }

				  // CRC
				  if(ptrdev->keybytes_32bit == 4){

					  // calculate CRC

					  // Update number of "new" 32-bit keys
					  (ptrdev->key_counter_32bit)++;
					  (ptrdev->keybytes_32bit) = 0;

					  encryption_byte = packet_type_keybit_chosen_CRC;

				  }
			  }
			  if(encryption_byte !=0){
				  HAL_Delay(1); //Delay om RX niet té flooden met packets ( was 1)
				  writeKeybitPacket(ptrdev, encryption_byte);
				  encryption_byte = 0;
			  }


			  //sprintf(buffer, "%d %d\r\n", (int8_t) Rx_RSSI, (int8_t) ptrdev->RSSI_Mean);
			  //CDC_Transmit_FS((int8_t *)buffer, strlen(buffer));
		  }
	  }

	  if (INT_PACKET_SENT){
		  INT_PACKET_SENT = 0;
		  if (settings_mode == 'R'){
			  //while(ADF_SPI_READY()==0);
			  //ADF_set_Rx_mode();
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10500-1;
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
  htim5.Init.Period = (2625)-1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
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

void delay_us (uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim7,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim7) < us);  // wait for the counter to reach the us input in the parameter
}


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
			delay_us(10);//6
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
	// Play audio samples on DAC
	if (htim->Instance == TIM2){
		playAudio();
	}

	// Timer for external interrupts (buttons)
	if (htim->Instance == TIM11){
		// Power button pressed
		if (HAL_GPIO_ReadPin(BTN_PWR_GPIO_Port, BTN_PWR_Pin)){
			POWER_state = 1;

			ADF_sleep();

			OLED_shutdown();
			HAL_GPIO_WritePin(CLASS_D_SHDN_GPIO_Port, CLASS_D_SHDN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MIC_SHDN_GPIO_Port, MIC_SHDN_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_OC_Stop(&htim5, TIM_CHANNEL_1);
			HAL_ADC_Stop_IT(&hadc1);

			HAL_Delay(250);

			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
			HAL_PWR_EnterSTANDBYMode();

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
			HAL_Delay(1);//added later, not sure if needed
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

			//ADD FUNCTIONALITY (function up capped)

			UP_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		// Down button pressed
		if (HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin)){

			//ADD FUNCTIONALITY (function down capped)
			if(setting_debugscreen==0){
				setting_debugscreen=1;
				OLED_clear_screen();
				OLED_print_status(settings_mode);
				OLED_update();
			}
			else{
				setting_debugscreen=0;
				OLED_clear_screen();
				OLED_print_status(settings_mode);
				OLED_update();
			}

			DOWN_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		// Left button pressed
		if (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin)){

			//ADD FUNCTIONALITY (menu left cyclic)

			LEFT_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}

		// Right button pressed
		if (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin)){

			//ADD FUNCTIONALITY (menu right cyclic)

			RIGHT_state = 1;
			HAL_TIM_Base_Stop_IT(&htim11);
		}


	}

	// Audio has vibrato :( [comment out TIM9 'T']
	if(setting_debugscreen){
		if(htim->Instance == TIM9){
			if(settings_mode == 'R'){
				OLED_clear_screen();
				OLED_print_status(settings_mode);
				OLED_print_variable("Packets T:  ", packets_sent, 5, 20);
				OLED_print_variable("Packets R:  ", packets_received, 5, 30);
				OLED_print_variable("Packet/s R: ", packets_received-packets_received_prev, 5, 40);
				OLED_update();
				packets_received_prev = packets_received;
			}
			else if(settings_mode == 'T'){
				OLED_clear_screen();
				OLED_print_status(settings_mode);
				OLED_print_variable("Packets T:  ", packets_sent, 5, 20);
				OLED_print_variable("Packets R:  ", packets_received, 5, 30);
				OLED_print_variable("Packet/s T: ", packets_sent-packets_sent_prev, 5, 40);
				OLED_update();
				packets_sent_prev = packets_sent;
			}
		}
	}

}


/* Callback ADC conversion */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	/*
	// ADC running at 8 kHz (TIM5: (2625*2-1))
	adc_value = HAL_ADC_GetValue(&hadc1);
	circular_buf_put_overwrite(audio_buffer_handle_t, adc_value);
	*/

	// ADC running at 16 kHz (TIM5: (2625-1)), with sample averaging to 8 kHz to improve SNR
	if(adc_counter%2){
		adc_value = HAL_ADC_GetValue(&hadc1);
		adc_counter++;
	}
	else{
		adc_value_downsampled = HAL_ADC_GetValue(&hadc1);
		adc_value_downsampled += adc_value;
		adc_value_downsampled /= 2;
		circular_buf_put_overwrite(audio_buffer_handle_t, adc_value_downsampled);
		adc_counter++;
	}

}


void startup(void){
	// Clear PWR wake up Flag
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	OLED_init();
	OLED_print_credits();
	OLED_print_status(settings_mode);
	OLED_update();
	// Setup registers for transceiver
	ADF_Init(settings_frequency);


	HAL_Delay(1000);
	//OLED_clear_screen();
	ssh1106_Fill(Black);
	ssh1106_UpdateScreen();

	ADF_set_turnaround_Tx_Rx();

	// Make audio buffer for 400 8-bit samples
	uint16_t buffer_size = 400;
	uint16_t *buffer = malloc(buffer_size * sizeof(uint16_t));
	audio_buffer_handle_t = circular_buf_init(buffer, buffer_size);

	// Setup for MCU
	setup();

}


/* ---Called upon startup or talk-button press--- */
void setup(){
	// Reset buffer
	circular_buf_reset(audio_buffer_handle_t);

	switch(settings_mode){
		case 'T':
			LED_RGB_status(0, 10, 0);

			ADF_clear_Rx_flag(); //test
			ADF_clear_Tx_flag(); //test

			// Stop the DAC interface and timer2 (8 kHz)
			HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim2);

			// Shutdown Class D audio amplifier
			HAL_GPIO_WritePin(CLASS_D_SHDN_GPIO_Port, CLASS_D_SHDN_Pin, GPIO_PIN_RESET); //GPIO_PIN_RESET
			// Enable microphone preamplifier
			HAL_GPIO_WritePin(MIC_SHDN_GPIO_Port, MIC_SHDN_Pin, GPIO_PIN_SET);

			// Start timer5 (16 kHz) and ADC interrupt triggered by TIM5
			HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_1);
			HAL_ADC_Start_IT(&hadc1);

			OLED_print_status(settings_mode);
			OLED_update();
			//HAL_TIM_Base_Start_IT(&htim9);

			break;

		case 'R':
			LED_RGB_status(0, 0, 10);

			ADF_clear_Rx_flag(); //test
			ADF_clear_Tx_flag(); //test

			// Stop timer5 (16 kHz) and ADC interrupt triggered by TIM5
			HAL_TIM_OC_Stop(&htim5, TIM_CHANNEL_1);
			HAL_ADC_Stop_IT(&hadc1);

			// Shutdown microphone preamplifier
			HAL_GPIO_WritePin(MIC_SHDN_GPIO_Port, MIC_SHDN_Pin, GPIO_PIN_RESET);
			// Enable Class D audio amplifier
			HAL_GPIO_WritePin(CLASS_D_SHDN_GPIO_Port, CLASS_D_SHDN_Pin, GPIO_PIN_SET);

			// Start the DAC interface and timer2 (8 kHz)
			HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim2);

			OLED_print_status(settings_mode);
			//OLED_print_variable("Volume: ", 20, 5, 30);
			//OLED_print_text("< Thresh.", 1, 54);
			//OLED_print_text("Encryp. >", 73, 54);
			OLED_update();
			//while(ADF_RC_READY()==0); //blijft soms hangen als RC niet ready komt
			ADF_set_Rx_mode(); //werkte 21/03
			break;
	}

}

void digipotInit(uint8_t volume){
	uint8_t value[1];
	value[0]= volume;
	HAL_GPIO_WritePin(DPOT_CS_GPIO_Port, DPOT_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, value, 1, 50);
	HAL_GPIO_WritePin(DPOT_CS_GPIO_Port, DPOT_CS_Pin, GPIO_PIN_SET);
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


void test_transmitDummyPacket(void){
	// Packet length (1byte), packet type (1byte), ID (1byte), RSSI byte (1byte)
	//uint8_t packet_total_length = 5+40;
	uint8_t packet_total_length = 5;
	dest_ID = 0xFF;

	//SPI_PKT_WR, packet total length, packet type, ID
	uint8_t header[] = {0x10, packet_total_length, 0xBB, dest_ID, source_ID};


	//uint8_t array[20] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20};//, 0x21222324, 0x25262728, 0x29303132};
	//uint8_t array[40] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20,0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20};

	// Write data to packet RAM
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, header, 5);
	//HAL_SPI_Transmit_IT(&hspi1, array, 20);
	//for(int i=0; i<40; i++){
	//	HAL_SPI_Transmit_IT(&hspi1, &array[i], 1);
	//}
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	ADF_set_Tx_mode();

}

void test_receiveDummyPacket(void){
	// SPI_PKT_RD and SPI_NOP command
	uint8_t SPI_commands[] = {0x30, 0xff};
	uint8_t array[20];
	while (ADF_SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, SPI_commands, 2);

	HAL_SPI_Receive_IT(&hspi1, &Rx_packet_length, 1);
	HAL_SPI_Receive_IT(&hspi1, &Rx_packet_type, 1);
	HAL_SPI_Receive_IT(&hspi1, &Rx_to_ID, 1);
	HAL_SPI_Receive_IT(&hspi1, &Rx_from_ID, 1);
	//HAL_SPI_Receive_IT(&hspi1, array, 20);
	HAL_SPI_Receive_IT(&hspi1, &Rx_RSSI, 1);
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	// Wait for SPI to finish
	while (ADF_SPI_READY() == 0);
}

void transmitAudioPacket(void){
	uint8_t packet_type;
	uint8_t cbuf_ret = 0;

	// Packet length (1byte), packet type (1byte), dest_ID, source_ID (1byte), RSSI byte (1byte)
	uint8_t packet_total_length = 5 + settings_audiosamples_length;

	if(settings_encryption){
		packet_type = packet_type_audio_encrypted;
	}
	else{
		packet_type = packet_type_audio;
	}

	// SPI_PKT_WR, packet total length, packet type, ID
	uint8_t header[] = {0x10, packet_total_length, packet_type, dest_ID = 0xFF, source_ID};

	// Read samples from audio buffer
	for(uint8_t i = 0; i < settings_audiosamples_length; i++){
		cbuf_ret = circular_buf_get(audio_buffer_handle_t, &samples[i]);
	}

	// Encryption
	if(settings_encryption){
		HAL_CRYP_Encrypt(&hcryp, samples, settings_audiosamples_length, data, 50); //blocking
	}

	// ---Write data to packet RAM---
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, header, 5);

	if(settings_encryption){
		for(uint8_t i = 0; i < settings_audiosamples_length; i++){
			HAL_SPI_Transmit_IT(&hspi1, &data[i], 1);
		}
	}
	else{
		for(uint8_t i = 0; i < settings_audiosamples_length; i++){
			HAL_SPI_Transmit_IT(&hspi1, &samples[i], 1);
		}
	}

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

	// Transmit packet
	ADF_set_Tx_mode();

}

void readPacket(void){
	// SPI_PKT_RD and SPI_NOP command
	uint8_t SPI_commands[] = {0x30, 0xff};

	while (ADF_SPI_READY() == 0);

	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, SPI_commands, 2);

	HAL_SPI_Receive_IT(&hspi1, &Rx_packet_length, 1);
	HAL_SPI_Receive_IT(&hspi1, &Rx_packet_type, 1);
	HAL_SPI_Receive_IT(&hspi1, &Rx_to_ID, 1);
	HAL_SPI_Receive_IT(&hspi1, &Rx_from_ID, 1);

	if(Rx_to_ID == broadcast_ID){
		if(Rx_packet_type == packet_type_audio_encrypted || Rx_packet_type == packet_type_audio){
			for(uint8_t i = 0; i < settings_audiosamples_length; i++){
				HAL_SPI_Receive_IT(&hspi1, &data[i], 1);
			}

			HAL_SPI_Receive_IT(&hspi1, &Rx_RSSI, 1);
			HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
		}
	}
	else if (Rx_to_ID == source_ID){
		// Reply from audio packet
		if(Rx_packet_type == packet_type_reply){
			asm("nop");;
			//for(uint8_t i = 0; i < settings_audiosamples_length; i++){
			//	HAL_SPI_Receive_IT(&hspi1, &data[i], 1);
			//}
		}
		// Keybit chosen by TX
		else if(Rx_packet_type == packet_type_keybit_chosen){
			asm("nop");;
		}
		// Keybit chosen by TX with Hamming-code
		else if(Rx_packet_type == packet_type_keybit_chosen_Hamming){
			HAL_SPI_Receive_IT(&hspi1, &Hamming, 1);
		}
		else if(Rx_packet_type == packet_type_keybit_chosen_CRC){
			asm("nop");;
		}
		else if (Rx_packet_type == packet_type_keybit_CRC_ok){
			asm("nop");;
			// Update 128-bit key with new 32-bit key ==> niet hierin doen, maar enkel eens CRC response ok
			// (ptrdev->keywords_128bit)++; //enkel als response CRC ok is
			// (ptrdev->key_32bit) = 0;
		}
		else if (Rx_packet_type == packet_type_keybit_CRC_bad){
			asm("nop");;
			// (ptrdev->key_32bit) = 0;
			// Remove 32-bit key and DONT shift it in 128-bit key
		}
		HAL_SPI_Receive_IT(&hspi1, &Rx_RSSI, 1);
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
	}
	else{
		asm("nop");
		// Ignore packet
	}
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);

}


void writeKeyPacket(void){
	// Packet length (1byte), packet type (1byte), dest_ID, source_ID (1byte), RSSI byte (1byte)
	uint8_t packet_total_length = 5+settings_audiosamples_length;

	dest_ID = Rx_from_ID;

	//SPI_PKT_WR, packet total length, packet type, ID
	uint8_t header[] = {0x10, packet_total_length, packet_type_reply, dest_ID, source_ID};

	uint8_t array[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

	// Write data to packet RAM
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1, header, 5);
	for(int i=0; i<16; i++){
		HAL_SPI_Transmit_IT(&hspi1, &array[i], 1);
	}
	HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
	ADF_set_Tx_mode();
}

void writeKeybitPacket(device *ptrdev, uint8_t keybit_type){
	// Packet length (1byte), packet type (1byte), dest_ID, source_ID (1byte), RSSI byte (1byte)
	uint8_t packet_total_length = 5;

	if(keybit_type == packet_type_keybit_chosen){
		uint8_t header[] = {0x10, packet_total_length, packet_type_keybit_chosen, dest_ID = source_ID , source_ID};
		// Write data to packet RAM
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, header, packet_total_length);
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
		ADF_set_Tx_mode();
	}
	else if(keybit_type == packet_type_keybit_chosen_Hamming){
		//packet length + Hamming-code (1byte)
		packet_total_length+=1;

		uint8_t header[] = {0x10, packet_total_length, packet_type_keybit_chosen_Hamming, dest_ID = ptrdev->ID , source_ID, Hamming};
		// Write data to packet RAM
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, header, packet_total_length);
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
		ADF_set_Tx_mode();
	}
	else if(keybit_type == packet_type_keybit_chosen_CRC){
		//packet length + Hamming-code (1byte) + CRC (4byte)
		//packet_total_length+5;
		packet_total_length+=1;

		uint8_t CRC_check[4];
		//Calculate CRC from ptrdev
		uint8_t header[] = {0x10, packet_total_length, packet_type_keybit_chosen_CRC, dest_ID = ptrdev->ID , source_ID, Hamming};
		//uint8_t header[] = {0x10, packet_total_length, packet_type_keybit_chosen_CRC, dest_ID = ptrdev->ID , source_ID, Hamming_code, CRC_check[0], CRC_check[1], CRC_check[2], CRC_check[3]};

		// Write data to packet RAM
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, header, packet_total_length);
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
		ADF_set_Tx_mode();
	}

	else if(keybit_type == packet_type_keybit_CRC_ok){
		uint8_t header[] = {0x10, packet_total_length, packet_type_keybit_CRC_ok, dest_ID = ptrdev->ID , source_ID};

		// Write data to packet RAM
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, header, packet_total_length);
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
		ADF_set_Tx_mode();
	}

	else if(keybit_type == packet_type_reply){
		uint8_t header[] = {0x10, packet_total_length, packet_type_reply, dest_ID = ptrdev->ID , source_ID};

		// Write data to packet RAM
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, header, packet_total_length);
		HAL_GPIO_WritePin(ADF7242_CS_GPIO_Port, ADF7242_CS_Pin, GPIO_PIN_SET);
		ADF_set_Tx_mode();
	}

}

void playAudio(){
	uint16_t sample;
	uint8_t cbuf_ret = 0;
	cbuf_size = circular_buf_size(audio_buffer_handle_t);
	// Check if circular buffer is filled with samples
	if(cbuf_size){
		cbuf_ret = circular_buf_get(audio_buffer_handle_t, &sample);
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_8B_R, sample);
	}
}

uint8_t Hamming_create(uint8_t key){
	uint8_t bit_0 = key & 0x01;
	uint8_t bit_1 = (key>>1) & 0x01;
	uint8_t bit_2 = (key>>2) & 0x01;
	uint8_t bit_3 = (key>>3) & 0x01;
	uint8_t bit_4 = (key>>4) & 0x01;
	uint8_t bit_5 = (key>>5) & 0x01;
	uint8_t bit_6 = (key>>6) & 0x01;
	uint8_t bit_7 = (key>>7) & 0x01;

	uint8_t parity_0 = bit_0 ^ bit_1 ^ bit_3 ^ bit_4 ^ bit_6;
	uint8_t parity_1 = bit_0 ^ bit_2 ^ bit_3 ^ bit_5 ^ bit_6;
	uint8_t parity_2 = bit_1 ^ bit_2 ^ bit_3 ^ bit_7;
	uint8_t parity_3 = bit_4 ^ bit_5 ^ bit_6 ^ bit_7;

	uint8_t Tx_code = (parity_3 * 8) + (parity_2 * 4) + (parity_1 * 2) + parity_0;
	Tx_code = 0x0F & Tx_code;
	return Tx_code;
}

uint8_t Hamming_correct(uint8_t Tx_code, uint8_t key){
	uint8_t bit_0 = key & 0x01;
	uint8_t bit_1 = (key>>1) & 0x01;
	uint8_t bit_2 = (key>>2) & 0x01;
	uint8_t bit_3 = (key>>3) & 0x01;
	uint8_t bit_4 = (key>>4) & 0x01;
	uint8_t bit_5 = (key>>5) & 0x01;
	uint8_t bit_6 = (key>>6) & 0x01;
	uint8_t bit_7 = (key>>7) & 0x01;

	uint8_t parity_0 = bit_0 ^ bit_1 ^ bit_3 ^ bit_4 ^ bit_6;
	uint8_t parity_1 = bit_0 ^ bit_2 ^ bit_3 ^ bit_5 ^ bit_6;
	uint8_t parity_2 = bit_1 ^ bit_2 ^ bit_3 ^ bit_7;
	uint8_t parity_3 = bit_4 ^ bit_5 ^ bit_6 ^ bit_7;

	uint8_t Rx_code = (parity_3 * 8) + (parity_2 * 4) + (parity_1 * 2) + parity_0;

	if (Rx_code != Tx_code){
		uint8_t control_0 = 0;
		uint8_t control_1 = 0;
		uint8_t control_2 = 0;
		uint8_t control_3 = 0;

		if (parity_0 != (Tx_code & 0x01))
			control_0 = 1;
		if (parity_1 != ((Tx_code>>1) & 0x01))
			control_1 = 1;
		if (parity_2 != ((Tx_code>>2) & 0x01))
			control_2 = 1;
		if (parity_3 != ((Tx_code>>3) & 0x01))
			control_3 = 1;

		uint8_t control = (control_3 * 8) + (control_2 * 4) + (control_1 * 2) + control_0;

		// Key correction
		switch(control)	{
			case 3:
				key ^= 1UL << 0;
				break;
			case 5:
				key ^= 1UL << 1;
				break;
			case 6:
				key ^= 1UL << 2;
				break;
			case 7:
				key ^= 1UL << 3;
				break;
			case 9:
				key ^= 1UL << 4;
				break;
			case 10:
				key ^= 1UL << 5;
				break;
			case 11:
				key ^= 1UL << 6;
				break;
			case 12:
				key ^= 1UL << 7;
				break;
		}
	}
	return key;
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
