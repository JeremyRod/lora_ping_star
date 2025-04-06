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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "subghz.h"
#include <string.h>
#include <stdio.h>
//#include "circular_buff.h"
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//typedef struct {
//    uint8_t event_queue[EVENT_QUEUE_SIZE];
//    circular_buffer_t circ_buf;
//} event_queue_t;
//
//event_queue_t event_queue;

typedef enum
{
  STATE_NULL,
  STATE_MASTER,
  STATE_SLAVE
} state_t;

typedef enum
{
  SSTATE_NULL,
  SSTATE_RX,
  SSTATE_TX
} substate_t;

typedef struct
{
  state_t state;
  substate_t subState;
  uint32_t rxTimeout;
  uint32_t rxMargin;
  uint32_t randomDelay;
  char rxBuffer[RX_BUFFER_SIZE];
  uint8_t rxSize;
} pingPongFSM_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EVENT_QUEUE_SIZE 32
#define EVENT_TIMER_DURATION 100 //ms
#define EVENT_TIMER_FREQ 10 // Hz
#define ONE_SECOND EVENT_TIMER_FREQ * 1
#define ONE_HUNDRED_MS EVENT_TIMER_FREQ / 10
#define TWO_MIN EVENT_TIMER_FREQ * 120
#define TEN_SECONDS EVENT_TIMER_FREQ * 10
#define PING_BYTE 1
#define ALARM_BYTE 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RF_FREQUENCY                                915000000 /* Hz */
#define TX_OUTPUT_POWER                             22        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */ // enum val, is 125khz
#define LORA_SPREADING_FACTOR                       12 //was 7
#define LORA_CODINGRATE                             1 // enum val,pu is 4/5
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */


//#define PERIPH 1 // comment this out to flash as master


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef PERIPH
const char dev_id[4] = "P001"; // flash each device with a new id
#else
const char dev_id[4] = "M001"; // flash each device with a new id
#endif

volatile PacketStatus_t recPackStat;
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;
static uint32_t secCounter = 0;
static uint8_t recCount = 0;

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void eventTxDone(pingPongFSM_t *const fsm);
void eventRxDone(pingPongFSM_t *const fsm);
void eventTxTimeout(pingPongFSM_t *const fsm);
void eventRxTimeout(pingPongFSM_t *const fsm);
void eventRxError(pingPongFSM_t *const fsm);
void enterMasterRx(pingPongFSM_t *const fsm);
void enterSlaveRx(pingPongFSM_t *const fsm);
void enterMasterTx(pingPongFSM_t *const fsm);
void enterSlaveTx(pingPongFSM_t *const fsm);
void transitionRxDone(pingPongFSM_t *const fsm);

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
  pingPongFSM_t fsm;
  char uartBuff[100];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO Clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// DEBUG_RF_{HSE32RDY, NRESET} pins
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// DEBUG_RF_{SMPSRDY, LDORDY} pins
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// RF_BUSY pin
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// RF_{IRQ0, IRQ1, IRQ2} pins
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

//  BSP_LED_Init(LED_GREEN);
//  BSP_LED_Init(LED_RED);

  strcpy(uartBuff, "\n\rLora Receiver\r\n---------------\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
  sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d\r\n", (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR);
  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

  radioInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t rnd = 0;
  SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  rnd = SUBGRF_GetRandom();

  fsm.state = STATE_NULL;
  fsm.subState = SSTATE_NULL;
  fsm.rxTimeout = 3333; // 3000 ms
  fsm.rxMargin = 200;   // 200 ms
  fsm.randomDelay = rnd >> 22; // [0, 1023] ms
  sprintf(uartBuff, "rand=%lu\r\n", fsm.randomDelay);
  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

  HAL_Delay(fsm.randomDelay);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  SUBGRF_SetRx(fsm.rxTimeout << 6);
#ifdef PERIPH
  fsm.state = STATE_SLAVE;
  fsm.subState = SSTATE_RX;
  enterSlaveRx(&fsm);
#else
  fsm.state = STATE_MASTER;
  fsm.subState = SSTATE_TX;
  enterMasterTx(&fsm);
#endif

  // Start timer
  HAL_TIM_Base_Start_IT(&htim17);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	eventReceptor = NULL;
	while (eventReceptor == NULL);
	eventReceptor(&fsm);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 6;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void radioInit(void)
{
  // Initialize the hardware (SPI bus, TCXO control, RF switch)
  SUBGRF_Init(RadioOnDioIrq);

  // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
  // "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
  SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
  SUBGRF_SetRegulatorMode();

  // Use the whole 256-byte buffer for both TX and RX
  SUBGRF_SetBufferBaseAddress(0x00, 0x00);

  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  SUBGRF_SetStopRxTimerOnPreambleDetect(false);

  SUBGRF_SetPacketType(PACKET_TYPE_LORA);

  SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
  SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

  ModulationParams_t modulationParams;
  modulationParams.PacketType = PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
  modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
  modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
  modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
  SUBGRF_SetModulationParams(&modulationParams);

  packetParams.PacketType = PACKET_TYPE_LORA;
  packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
  SUBGRF_SetPacketParams(&packetParams);

  //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
  // RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}

/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
  switch (radioIrq)
  {
    case IRQ_TX_DONE:
      eventReceptor = eventTxDone;
      break;
    case IRQ_RX_DONE:
      eventReceptor = eventRxDone;
      break;
    case IRQ_RX_TX_TIMEOUT:
      if (SUBGRF_GetOperatingMode() == MODE_TX)
      {
        eventReceptor = eventTxTimeout;
      }
      else if (SUBGRF_GetOperatingMode() == MODE_RX)
      {
        eventReceptor = eventRxTimeout;
      }
      break;
    case IRQ_CRC_ERROR:
      eventReceptor = eventRxError;
      break;
    default:
      break;
  }
}

/**
  * @brief  Process the TX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventTxDone(pingPongFSM_t *const fsm)
{
  //HAL_UART_Transmit(&huart1, (uint8_t *)"Event TX Done\r\n", 15, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterMasterRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterSlaveRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxDone(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Event RX Done\r\n", 15, HAL_MAX_DELAY);
  switch(fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          transitionRxDone(fsm);
          recCount++;
          if (recCount == 2)
          {
//            BSP_LED_Off(LED_GREEN);
//            BSP_LED_Toggle(LED_RED);
        	  recCount = 0;
            enterMasterTx(fsm);
            fsm->subState = SSTATE_TX;
          } else {
        	  enterMasterRx(fsm);
          }
//          else if (strncmp(fsm->rxBuffer, "PING", 4) == 0)
//          {
//            enterSlaveRx(fsm);
//            fsm->state = STATE_SLAVE;
//          }
//          else
//          {
//            enterMasterRx(fsm);
//          }
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_RX:
        	transitionRxDone(fsm);
//          if (strncmp(fsm->rxBuffer, "PING", 4) == 0)
//          {
            //BSP_LED_Off(LED_RED);
            //BSP_LED_Toggle(LED_GREEN);
            enterSlaveTx(fsm);
            fsm->subState = SSTATE_TX;
//          }
//          else
//          {
//            enterMasterRx(fsm);
//            fsm->state = STATE_MASTER;
//          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void eventTxTimeout(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Event TX Timeout\r\n", 18, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterMasterRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_TX:
          enterSlaveRx(fsm);
          fsm->subState = SSTATE_RX;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void eventRxTimeout(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Event RX Timeout\r\n", 18, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          HAL_Delay(fsm->randomDelay);
          recCount = 0;
          enterMasterTx(fsm);
          fsm->subState = SSTATE_TX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          enterSlaveRx(fsm);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void eventRxError(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Event Rx Error\r\n", 16, HAL_MAX_DELAY);
  switch (fsm->state)
  {
    case STATE_MASTER:
      switch (fsm->subState)
      {
        case SSTATE_RX:
//          HAL_Delay(fsm->randomDelay);
//          recCount = 0;
//          enterMasterTx(fsm);
//          fsm->subState = SSTATE_TX;
          break;
        default:
          break;
      }
      break;
    case STATE_SLAVE:
      switch (fsm->subState)
      {
        case SSTATE_RX:
          enterSlaveRx(fsm);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

/**
  * @brief  Entry actions for the RX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterMasterRx(pingPongFSM_t *const fsm)
{
  //HAL_UART_Transmit(&huart1, (uint8_t *)"Master Rx start\r\n", 17, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}

void enterSlaveRx(pingPongFSM_t *const fsm)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"Slave Rx start\r\n", 16, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}

void enterMasterTx(pingPongFSM_t *const fsm)
{
  HAL_Delay(fsm->rxMargin);

  //HAL_UART_Transmit(&huart1, (uint8_t *)"...PING\r\n", 9, HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart1, (uint8_t *)"Master Tx start\r\n", 17, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
  packetParams.Params.LoRa.PayloadLength = 0x4;
  uint8_t payload[4];
  memcpy(payload, dev_id, 4);
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SendPayload(payload, 4, 0);
}

void enterSlaveTx(pingPongFSM_t *const fsm)
{
  HAL_Delay(fsm->rxMargin);

  //HAL_UART_Transmit(&huart1, (uint8_t *)"...PONG\r\n", 9, HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart1, (uint8_t *)"Slave Tx start\r\n", 16, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
  packetParams.Params.LoRa.PayloadLength = 0x5;
	uint8_t payload[5] = {0};
	memcpy(payload, dev_id, 4);
	payload[4] = recPackStat.Params.LoRa.RssiPkt;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SendPayload(payload, 5, 0);
}

/**
  * @brief  Transition actions executed on every RX Done event (helper function)
  * @param  fsm pointer to FSM context
  * @retval None
  */
void transitionRxDone(pingPongFSM_t *const fsm)
{
  PacketStatus_t packetStatus;

  // Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
  SUBGRF_WriteRegister(0x0920, 0x00);
  SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

  SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
  SUBGRF_GetPacketStatus(&packetStatus);

#ifdef PERIPH
  recPackStat = packetStatus;
#else
  char id[4];
  char uartBuff[50];
  memcpy(id, (uint8_t*)fsm->rxBuffer, 4);
  sprintf(uartBuff, "ID: %s, RSSI: %d, Secs: %lu\r\n", id, (int8_t)fsm->rxBuffer[4], secCounter); // here should print id of device and lora RSSI.
  HAL_UART_Transmit(&huart1, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
#endif
}

//The event timer is run on the HW low power timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef  *htim)
{
	if (htim == &htim17)
	{
		//Run timer increment function
		secCounter++;
	}
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
