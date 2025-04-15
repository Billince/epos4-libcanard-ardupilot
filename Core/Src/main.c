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
#include "usb_host.h"
#include "../libcanard/canard.h"
#include <stdlib.h>
#include <string.h>
#include "epos4.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// DroneCAN/UAVCAN definitions
#define CANARD_CAN_EXT_ID_MASK         0x1FFFFFFFU
#define NODE_ID                        96
#define UAVCAN_NODE_STATUS_MESSAGE_SIZE 7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID 341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE 0x0f0868d0c1a7c6f1ULL

// GetNodeInfo service definitions
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID 1
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE 0x0b2a812620a11d40ULL
#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE 80

// Parameter protocol definitions
#define UAVCAN_PARAM_STRING_ID 11
#define UAVCAN_PARAM_STRING_SIGNATURE 0xe02f25d6e0c98ae0ULL
#define UAVCAN_PARAM_GETSET_ID 10
#define UAVCAN_PARAM_GETSET_SIGNATURE 0xa7b622f939d1a4d5ULL

// Read/Write register definitions (used by ArduPilot for node naming)
#define UAVCAN_PROTOCOL_PARAM_VALUE_SIZE 60
#define UAVCAN_PROTOCOL_REGISTER_ACCESS_ID 85
#define UAVCAN_PROTOCOL_REGISTER_ACCESS_SIGNATURE 0xb7d72a90bc4bc330ULL

// UAVCAN Raw Command message (for motor control)
#define UAVCAN_RAW_COMMAND_ID 30
#define UAVCAN_RAW_COMMAND_SIGNATURE 0x59276cb31ad99c85ULL

// EPOS4 node ID for CANopen control
#define EPOS4_NODE_ID                  1   // Default node ID for EPOS4
#define EPOS4_RESOLUTION               4096 // Encoder resolution (counts per rev)
#define EPOS4_MAX_VELOCITY             5000 // Default velocity for movements
#define EPOS4_MAX_ACCELERATION         10000 // Default acceleration for movements

// SDO command definitions for EPOS4
#define EPOS4_SDO_WRITE                0x600 // SDO write request base ID
#define EPOS4_SDO_READ_REPLY           0x580 // SDO read reply base ID

// EPOS4 Object Dictionary indexes
#define EPOS4_CONTROLWORD_INDEX        0x6040
#define EPOS4_STATUSWORD_INDEX         0x6041
#define EPOS4_MODE_OF_OPERATION_INDEX  0x6060
#define EPOS4_TARGET_VELOCITY_INDEX    0x60FF
#define EPOS4_ACTUAL_VELOCITY_INDEX    0x606C
#define EPOS4_TARGET_POSITION_INDEX    0x607A
#define EPOS4_ACTUAL_POSITION_INDEX    0x6064

// Position limits for tilt control
#define TILT_MIN_POSITION              -10000  // Minimum tilt position
#define TILT_MAX_POSITION              10000   // Maximum tilt position
#define TILT_CENTER_POSITION           0       // Center position

#define UAVCAN_NODE_HEALTH_OK          0
#define UAVCAN_NODE_HEALTH_WARNING     1
#define UAVCAN_NODE_HEALTH_ERROR       2
#define UAVCAN_NODE_HEALTH_CRITICAL    3

#define UAVCAN_NODE_MODE_OPERATIONAL   0
#define UAVCAN_NODE_MODE_INITIALIZATION 1
#define UAVCAN_NODE_MODE_MAINTENANCE   2
#define UAVCAN_NODE_MODE_SOFTWARE_UPDATE 3
#define UAVCAN_NODE_MODE_OFFLINE       7

// Ensure CanardTransferTypeMessage is defined in case the libcanard version doesn't match
#ifndef CanardTransferTypeMessage
#define CanardTransferTypeMessage 2
#endif

// Node name and version info
#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 0
#define APP_NODE_NAME "STM32-EPOS4"  // Shorter name that ArduPilot can display better

// Remove conflicting position macros
// #define MIN_POSITION -10000  // Minimum tilt position
// #define MAX_POSITION 10000   // Maximum tilt position
// #define CENTER_POSITION 0    // Center position
#define COMMAND_DEADBAND 100 // Deadband for stability

// Remove duplicate EPOS4 configuration - already defined above
// #define EPOS4_NODE_ID          1
// #define EPOS4_MAX_ACCELERATION 5000
// #define EPOS4_MAX_VELOCITY     1000
// #define EPOS4_RESOLUTION       1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Memory pool for libcanard
static uint8_t canard_memory_pool[1024];
// Canard instance 
CanardInstance canard;

// Debug LED states
uint8_t can_tx_led = 0;
uint8_t can_rx_led = 0;

// Transfer ID for UAVCAN messages
static uint8_t transfer_id = 0;

// Message port ID for Real64 array (Yakut test message)
#define MSG_PORT_ID 1620U
static uint8_t my_message_transfer_id = 0;
uint32_t test_uptimeSec = 0;
// Buffer for heartbeat messages
#define HEARTBEAT_BUFFER_SIZE 16
uint8_t hbeat_ser_buf[HEARTBEAT_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void* memAllocate(CanardInstance* const ins, const size_t amount);
static void memFree(CanardInstance* const ins, void* const pointer);
void process_canard_TX_queue(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void send_heartbeat(void);
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                uint64_t* out_data_type_signature,
                                uint16_t data_type_id,
                                CanardTransferType transfer_type,
                                uint8_t source_node_id);
void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
void send_node_name(void);
void Error_Handler(void);

// EPOS4 functions via CANopen communication
void CANopen_NMT_Command(CAN_HandleTypeDef* hcan, uint8_t command, uint8_t node_id);
void EPOS4_SetOperationMode(CAN_HandleTypeDef* hcan, uint8_t node_id, uint8_t mode);
void EPOS4_SetProfileVelocity(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t velocity);
void EPOS4_SetProfileAcceleration(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t acceleration);
void EPOS4_SetProfileDeceleration(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t deceleration);

// Functions for epos4 via libcanard
void epos4_send_sdo_write(uint16_t index, uint8_t subindex, uint32_t data, uint8_t data_length);
void epos4_send_sdo_read(uint16_t index, uint8_t subindex);
void epos4_set_operation_mode(uint8_t mode);
void epos4_set_target_velocity(int32_t target_velocity);
void epos4_set_target_position(int32_t target_position);
void epos4_enable_motor(void);
void epos4_disable_motor(void);
void epos4_send_pdo(uint16_t cob_id, uint8_t* data, uint8_t length);
void handle_ardupilot_command(int16_t command_value);

// EPOS4 CANopen interface functions
bool EPOS4_SetTargetPosition_Legacy(CAN_HandleTypeDef* hcan, uint8_t node_id, int32_t position);
void EPOS4_SetControlWord(CAN_HandleTypeDef* hcan, uint8_t node_id, uint16_t control_word);
bool EPOS4_Init_Legacy(CAN_HandleTypeDef* hcan, uint8_t node_id);
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
  MX_CAN1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  
  /* USER CODE BEGIN 2 */
  // Initialize LEDs
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET); // Green LED off
  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET); // Red LED off
  HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET); // Orange LED off
  HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET); // Blue LED off
  
  // Initialize libcanard
  canardInit(&canard, 
             canard_memory_pool, 
             sizeof(canard_memory_pool), 
             onTransferReceived, 
             shouldAcceptTransfer,
             NULL);
  
  // Set node ID
  canardSetLocalNodeID(&canard, NODE_ID);
  
  // Configure CAN filter to accept all messages
  CAN_FilterTypeDef canFilter;
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter.FilterActivation = CAN_FILTER_ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
    Error_Handler();
  }
  
  // Start CAN in normal mode
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  
  // Enable CAN interrupts for receive
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  
  // Main timing variables
  uint32_t last_heartbeat_ms = 0;
  uint32_t last_name_send_ms = 0;
  uint32_t last_epos4_statuscheck_ms = 0;
  uint32_t last_epos4_control_ms = 0;
  
  // Wait for EPOS4 controller to boot up
  HAL_Delay(1000);
  
  // Flash all LEDs to show we're starting EPOS4 initialization
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
  HAL_Delay(300);
  
  // Initialize EPOS4 controller for position control mode
  
  // Step 1: Reset communications
  CANopen_NMT_Command(&hcan1, 0x82, EPOS4_NODE_ID); // 0x82 = Reset communications
  HAL_Delay(500); // Wait for reset to complete
  
  // Step 2: Start remote node
  CANopen_NMT_Command(&hcan1, 0x01, EPOS4_NODE_ID); // 0x01 = Start remote node
  HAL_Delay(200);
  
  // Step 3: Set operation mode to position mode
  EPOS4_SetOperationMode(&hcan1, EPOS4_NODE_ID, MODE_PROFILE_POSITION);
  HAL_Delay(100);
  
  // Step 4: Set control word to shutdown state (prepare for enable)
  EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x0006);
  HAL_Delay(100);
  
  // Step 5: Set control word to switch on state
  EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x0007);
  HAL_Delay(100);
  
  // Step 6: Configure motion profile parameters for smooth tilt movement
  // Profile velocity (speed during positioning)
  EPOS4_SetProfileVelocity(&hcan1, EPOS4_NODE_ID, 5000);
  HAL_Delay(50);
  
  // Profile acceleration (ramp-up rate)
  EPOS4_SetProfileAcceleration(&hcan1, EPOS4_NODE_ID, 10000);
  HAL_Delay(50);
  
  // Profile deceleration (ramp-down rate)
  EPOS4_SetProfileDeceleration(&hcan1, EPOS4_NODE_ID, 10000);
  HAL_Delay(50);
  
  // Step 7: Enable operation
  EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x000F);
  HAL_Delay(100);
  
  // Turn on green LED to show initialization complete
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
  
  // Turn on orange LED (LD3) to show main loop is running
  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
  
  // Use TILT_ macros instead of local variables
  // const int32_t min_position = -144000;  // Minimum position (fully retracted)
  // const int32_t max_position = 144000;   // Maximum position (fully extended)
  // const int32_t center_position = 0;     // Center/neutral position
  int32_t target_position = TILT_CENTER_POSITION; // Start at center position
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // Send node name immediately at startup for faster discovery
  send_node_name();
  
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    
    // Send heartbeat message every 1 second
    uint32_t current_ms = HAL_GetTick();
    if (current_ms - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = current_ms;
        
        // Send ArduPilot compatible heartbeat message
        send_heartbeat();
    }
    
    // Send node name as param string every 3 seconds
    if (current_ms - last_name_send_ms >= 3000) {
        last_name_send_ms = current_ms;
        
        // Send node name in UAVCAN param string format
        send_node_name();
    }
    
    // Check EPOS4 status every 500ms
    if (current_ms - last_epos4_statuscheck_ms >= 500) {
        last_epos4_statuscheck_ms = current_ms;
        
        // Read the actual position
        int32_t current_position = EPOS4_GetActualPosition(&hcan1, EPOS4_NODE_ID);
        
        // Blink blue LED (LD6) to indicate EPOS4 status check
        HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
    }
    
    // Process any pending CAN transmissions
    process_canard_TX_queue();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  
  // Configure for 1Mbps with 168MHz clock (APB1 = 42MHz)
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;  // Changed from 12TQ to 11TQ
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;   // Changed from 1TQ to 2TQ
  
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;      // Enable auto recovery from bus-off state
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

  // Initialize EPOS4 motor controller after CAN initialization
  if (EPOS4_Init_Legacy(&hcan1, EPOS4_NODE_ID)) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Green LED on success
  } else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red LED on failure
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_SCL_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(Audio_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
    (void)ins;
    return malloc(amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
    (void)ins;
    free(pointer);
}

void process_canard_TX_queue(void)
{
    const CanardCANFrame* txf = canardPeekTxQueue(&canard);
    while (txf != NULL)
    {
        CAN_TxHeaderTypeDef tx_header;
        uint32_t tx_mailbox;

        tx_header.StdId = 0;
        tx_header.ExtId = txf->id & CANARD_CAN_EXT_ID_MASK;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = txf->data_len;

        // Turn on green LED (LD4) while trying to send
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*)txf->data, &tx_mailbox);
        
        if (status != HAL_OK)
        {
            // If send failed, turn on red LED (LD5) to indicate error
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
            
            // Check specific error conditions
            uint32_t error_code = HAL_CAN_GetError(&hcan1);
            if (error_code & HAL_CAN_ERROR_TX_ALST0) {
                // Transmit arbitration lost - keep orange LED (LD3) on
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            }
            if (error_code & HAL_CAN_ERROR_TX_TERR0) {
                // Transmit error - blink orange LED (LD3)
                HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            }
            if (error_code & HAL_CAN_ERROR_BOF) {
                // Bus off error - turn off all LEDs except red LED (LD5)
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
            }
            break;
        }
        else
        {
            // If send succeeded, turn off error LEDs
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        }

        // Turn off green LED (LD4) after send attempt
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

        canardPopTxQueue(&canard);
        txf = canardPeekTxQueue(&canard);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    // Toggle orange LED (LD3) to show RX activity
    can_rx_led = !can_rx_led;
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, can_rx_led ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Process EPOS4 SDO responses if applicable (StdId 0x580-0x5FF)
    if (rx_header.StdId >= 0x580 && rx_header.StdId < 0x600) {
        EPOS4_ProcessRxMessage(hcan, &rx_header, rx_data);
        return;
    }

    // Prepare CanardCANFrame from received data
    CanardCANFrame frame;
    frame.id = rx_header.IDE == CAN_ID_EXT ? rx_header.ExtId : rx_header.StdId;
    frame.data_len = rx_header.DLC;
    memcpy(frame.data, rx_data, rx_header.DLC);

    // Process the received frame with canard
    canardHandleRxFrame(&canard, &frame, HAL_GetTick() * 1000);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Similar to FIFO0 callback if needed
}

void send_heartbeat(void)
{
    uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
    
    // Pack node status message
    uint32_t uptime_sec = HAL_GetTick() / 1000;
    buffer[0] = (uint8_t)(uptime_sec & 0xFF);
    buffer[1] = (uint8_t)((uptime_sec >> 8) & 0xFF);
    buffer[2] = (uint8_t)((uptime_sec >> 16) & 0xFF);
    buffer[3] = (uint8_t)((uptime_sec >> 24) & 0xFF);
    
    // Cycle through different health states to make node more visible
    static uint8_t health_counter = 0;
    uint8_t health = UAVCAN_NODE_HEALTH_OK;
    
    // Every 15 heartbeats (15 seconds), report a different health
    if (health_counter == 15) {
        health = UAVCAN_NODE_HEALTH_WARNING;
    } else if (health_counter == 30) {
        health = UAVCAN_NODE_HEALTH_OK;
        health_counter = 0;  // Reset counter
    }
    health_counter++;
    
    buffer[4] = health;  // Health
    buffer[5] = UAVCAN_NODE_MODE_OPERATIONAL;  // Mode
    
    // Set vendor-specific status code to 0xA501 (distinctive value)
    // Bytes 6-7 form a uint16_t vendor-specific status code (little-endian)
    uint16_t vendor_status = 0xA501;  // A unique value for this specific implementation
    buffer[6] = (uint8_t)(vendor_status & 0xFF);  // LSB
    buffer[7] = (uint8_t)((vendor_status >> 8) & 0xFF);  // MSB
    
    // Send node status message
    canardBroadcast(&canard,
                  UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                  UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                  &transfer_id,
                  CANARD_TRANSFER_PRIORITY_MEDIUM,
                  buffer,
                  UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    // Toggle green LED to indicate heartbeat transmission
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
                  
    // Increment transfer ID (wraps at 31)
    transfer_id = (transfer_id + 1) & 31;
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
                                uint64_t* out_data_type_signature,
                                uint16_t data_type_id,
                                CanardTransferType transfer_type,
                                uint8_t source_node_id)
{
    // We're only interested in the node status message from other nodes
    if ((transfer_type == CanardTransferTypeMessage) && 
        (data_type_id == UAVCAN_NODE_STATUS_DATA_TYPE_ID))
    {
        *out_data_type_signature = UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE;
        return true;
    }
    
    // Accept GetNodeInfo service requests
    if ((transfer_type == CanardTransferTypeRequest) && 
        (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        // Highly visible notification for GetNodeInfo request at the acceptance stage
        // Alternate orange and green LEDs rapidly to show the request was received
        for (int i = 0; i < 3; i++) {
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);   // Orange on
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);   // Green on
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Orange off
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Green off
            HAL_Delay(50);
        }
        
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
        return true;
    }
    
    // Accept parameter requests - this is critical for node name recognition
    if ((transfer_type == CanardTransferTypeRequest) && 
        (data_type_id == UAVCAN_PARAM_STRING_ID))
    {
        *out_data_type_signature = UAVCAN_PARAM_STRING_SIGNATURE;
        return true;
    }
    
    // Accept parameter get/set requests (another way ArduPilot gets params)
    if ((transfer_type == CanardTransferTypeRequest) && 
        (data_type_id == UAVCAN_PARAM_GETSET_ID))
    {
        *out_data_type_signature = UAVCAN_PARAM_GETSET_SIGNATURE;
        return true;
    }
    
    // Accept register access requests (ArduPilot uses this for naming)
    if ((transfer_type == CanardTransferTypeRequest) && 
        (data_type_id == UAVCAN_PROTOCOL_REGISTER_ACCESS_ID))
    {
        *out_data_type_signature = UAVCAN_PROTOCOL_REGISTER_ACCESS_SIGNATURE;
        return true;
    }
    
    // Also accept messages on port 1620 (Yakut test messages)
    if ((transfer_type == CanardTransferTypeMessage) && 
        (data_type_id == MSG_PORT_ID))
    {
        // Use a placeholder signature for the Real64 array
        *out_data_type_signature = 0x1234567890ABCDEF;
        return true;
    }
    
    // Reject all other messages
    return false;
}

// Handle GetNodeInfo service request
void handle_get_node_info(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Very visible LED pattern - blink green and orange together to indicate GetNodeInfo request
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);     // Green on
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);     // Orange on
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);   // Green off
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);   // Orange off
        HAL_Delay(50);
    }
    
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE] = {0};
    uint8_t* ptr = buffer;
    
    // Status info - copy the same info we send in heartbeat
    uint32_t uptime_sec = HAL_GetTick() / 1000;
    ptr[0] = (uint8_t)(uptime_sec & 0xFF);
    ptr[1] = (uint8_t)((uptime_sec >> 8) & 0xFF);
    ptr[2] = (uint8_t)((uptime_sec >> 16) & 0xFF);
    ptr[3] = (uint8_t)((uptime_sec >> 24) & 0xFF);
    ptr[4] = UAVCAN_NODE_HEALTH_OK;
    ptr[5] = UAVCAN_NODE_MODE_OPERATIONAL;
    ptr[6] = 0;  // Sub mode
    
    ptr += 7;
    
    // Software version
    ptr[0] = APP_VERSION_MAJOR;
    ptr[1] = APP_VERSION_MINOR;
    ptr[2] = 0;  // Optional field flags (empty)
    memset(ptr + 3, 0, 1);  // VCS commit (empty)
    ptr += 4;
    
    // Hardware version
    ptr[0] = 1;  // Major
    ptr[1] = 0;  // Minor
    
    // Create a unique ID based on STM32 device ID and CAN setup
    // This helps ArduPilot to distinguish this node consistently
    uint32_t device_id_words[3] = {0};
    device_id_words[0] = HAL_GetUIDw0();
    device_id_words[1] = HAL_GetUIDw1();
    device_id_words[2] = HAL_GetUIDw2();
    
    // Copy unique ID - use device ID + custom data
    for (int i = 0; i < 4; i++) {
        ptr[2 + i] = (device_id_words[0] >> (i*8)) & 0xFF;
        ptr[2 + i + 4] = (device_id_words[1] >> (i*8)) & 0xFF;
        ptr[2 + i + 8] = (device_id_words[2] >> (i*8)) & 0xFF;
    }
    // Add identifier bytes for this specific application
    ptr[2 + 12] = 'S';
    ptr[2 + 13] = 'T';
    ptr[2 + 14] = 'M';
    ptr[2 + 15] = NODE_ID; // Add node ID to make it unique per node
    
    ptr += 18;
    
    // Node name string - must match exact node name format
    const char* name = APP_NODE_NAME;
    size_t name_len = strlen(name);
    ptr[0] = (uint8_t)name_len;
    memcpy(ptr + 1, name, name_len);
    ptr += 1 + name_len;
    
    size_t total_size = ptr - buffer;
    
    // Send response
    int result = canardRequestOrRespond(ins,
                         transfer->source_node_id,
                         UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                         UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                         &transfer->transfer_id,  // Use the request's transfer_id
                         transfer->priority,
                         CanardResponse,
                         buffer,
                         total_size);
                         
    // Blink both orange and green LEDs together to indicate response was sent
    if (result >= 0) {
        // Success - blink orange and green LEDs together 3 times
        for (int i = 0; i < 3; i++) {
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);   // Orange on
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);   // Green on
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Orange off
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Green off
            HAL_Delay(50);
        }
    } else {
        // Error - blink red LED
        for (int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
            HAL_Delay(50);
        }
    }
}

void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_type == CanardTransferTypeMessage) && 
        (transfer->data_type_id == UAVCAN_NODE_STATUS_DATA_TYPE_ID))
    {
        // Toggle LED6 (blue) when we receive heartbeat from another node
        HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
        
        // Extract node status info directly
        if (transfer->payload_len >= UAVCAN_NODE_STATUS_MESSAGE_SIZE)
        {
            // In libcanard v0, we need to use the payload_head field
            const uint8_t* payload = (const uint8_t*)transfer->payload_head;
            
            // Extract uptime (little-endian)
            uint32_t uptime = payload[0] | 
                    (payload[1] << 8) | 
                    (payload[2] << 16) | 
                    (payload[3] << 24);
                    
            uint8_t health = payload[4];
            uint8_t mode = payload[5];
            uint8_t submode = payload[6];
            
            // Do something with the node status information
            // For now, we just toggle the LED
            (void)uptime;
            (void)health;
            (void)mode;
            (void)submode;
        }
    }
    else if ((transfer->transfer_type == CanardTransferTypeRequest) && 
             (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        // Handle GetNodeInfo request
        handle_get_node_info(ins, transfer);
    }
    else if ((transfer->transfer_type == CanardTransferTypeRequest) && 
             (transfer->data_type_id == UAVCAN_PARAM_STRING_ID))
    {
        // Handle parameter request (like when ArduPilot asks for our name)
        // Blink both orange and green LEDs to indicate parameter request
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);   // Orange on
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);   // Green on
        HAL_Delay(100);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Orange off
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Green off
        
        // Extract parameter name
        bool found = false;
        uint8_t buffer[128] = {0};
        
        // For parameter get by name
        uint16_t index = 0;
        size_t name_len = 0;
        char name[32] = {0};
        
        // First byte should be 0 for string access or index for numerical access
        canardDecodeScalar(transfer, 0, 8, false, &index);
        
        if (index == 0) {
            // Get parameter by name
            canardDecodeScalar(transfer, 8, 8, false, &name_len);
            
            if (name_len > 0 && name_len < sizeof(name)) {
                for (uint8_t i = 0; i < name_len; i++) {
                    uint8_t c = 0;
                    canardDecodeScalar(transfer, 16 + i * 8, 8, false, &c);
                    name[i] = (char)c;
                }
                name[name_len] = '\0';
                
                // Check if we support this parameter
                if (strcmp(name, "name") == 0) {
                    found = true;
                    
                    // Prepare response with "name" parameter
                    uint16_t offset = 0;
                    // Parameter index (we use string-based access)
                    buffer[offset++] = 0;
                    
                    // Parameter name length and string
                    buffer[offset++] = (uint8_t)name_len;
                    memcpy(&buffer[offset], name, name_len);
                    offset += name_len;
                    
                    // Value is a string (type 0)
                    buffer[offset++] = 0;
                    
                    // Value length and data
                    const char* value = APP_NODE_NAME;
                    size_t value_len = strlen(value);
                    buffer[offset++] = (uint8_t)value_len;
                    memcpy(&buffer[offset], value, value_len);
                    offset += value_len;
                    
                    // Default value (same as current)
                    buffer[offset++] = 0;  // String type
                    buffer[offset++] = (uint8_t)value_len;
                    memcpy(&buffer[offset], value, value_len);
                    offset += value_len;
                    
                    // Min value (empty string)
                    buffer[offset++] = 0;  // String type
                    buffer[offset++] = 0;  // Empty
                    
                    // Max value (empty string)
                    buffer[offset++] = 0;  // String type
                    buffer[offset++] = 0;  // Empty
                    
                    // Send parameter response
                    int result = canardRequestOrRespond(ins,
                                             transfer->source_node_id,
                                             UAVCAN_PARAM_STRING_SIGNATURE,
                                             UAVCAN_PARAM_STRING_ID,
                                             &transfer->transfer_id,
                                             transfer->priority,
                                             CanardResponse,
                                             buffer,
                                             offset);
                    
                    // Visual feedback about parameter response
                    if (result >= 0) {
                        // Success - blink orange and green LEDs together
                        for (int i = 0; i < 2; i++) {
                            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
                            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
                            HAL_Delay(50);
                            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
                            HAL_Delay(50);
                        }
                    } else {
                        // Error - blink red LED
                        for (int i = 0; i < 5; i++) {
                            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
                            HAL_Delay(50);
                            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
                            HAL_Delay(50);
                        }
                    }
                }
            }
        }
        
        // If parameter not found, send an empty response
        if (!found) {
            // Empty parameter response
            buffer[0] = 0;  // Empty name
            
            canardRequestOrRespond(ins, 
                                  transfer->source_node_id,
                                  UAVCAN_PARAM_STRING_SIGNATURE,
                                  UAVCAN_PARAM_STRING_ID,
                                  &transfer->transfer_id,
                                  transfer->priority,
                                  CanardResponse,
                                  buffer,
                                  1);
        }
    }
    else if ((transfer->transfer_type == CanardTransferTypeRequest) && 
             (transfer->data_type_id == UAVCAN_PARAM_GETSET_ID))
    {
        // Handle parameter get/set request
        // Flash green LED to indicate parameter get/set
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        
        // For now, just send empty response which means parameter not found
        uint8_t buffer[2] = {0};
        
        canardRequestOrRespond(ins, 
                              transfer->source_node_id,
                              UAVCAN_PARAM_GETSET_SIGNATURE,
                              UAVCAN_PARAM_GETSET_ID,
                              &transfer->transfer_id,
                              transfer->priority,
                              CanardResponse,
                              buffer,
                              2);
    }
    else if ((transfer->transfer_type == CanardTransferTypeRequest) && 
             (transfer->data_type_id == UAVCAN_PROTOCOL_REGISTER_ACCESS_ID))
    {
        // Handle register access request
        // Flash orange LED to indicate register access
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        
        // Parse the register name
        bool found = false;
        uint8_t buffer[128] = {0};
        
        uint8_t name_len = 0;
        canardDecodeScalar(transfer, 0, 8, false, &name_len);
        
        if (name_len > 0 && name_len < 92) {
            char name[92] = {0};
            
            // Extract the register name
            for (uint8_t i = 0; i < name_len; i++) {
                uint8_t c = 0;
                canardDecodeScalar(transfer, 8 + i * 8, 8, false, &c);
                name[i] = (char)c;
            }
            name[name_len] = '\0';
            
            // Check if this is a request for "uavcan.node.name"
            if (strcmp(name, "uavcan.node.name") == 0) {
                // Flash green LED rapidly to indicate node name register request
                for (int i = 0; i < 3; i++) {
                    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
                    HAL_Delay(20);
                    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
                    HAL_Delay(20);
                }
                
                // Prepare response with node name
                // Value type (0 = string)
                buffer[0] = 0;
                
                // For string value
                const char* value = APP_NODE_NAME;
                size_t value_len = strlen(value);
                
                buffer[1] = (uint8_t)value_len;
                memcpy(&buffer[2], value, value_len);
                
                found = true;
                
                // Send the register value
                int result = canardRequestOrRespond(ins,
                                       transfer->source_node_id,
                                       UAVCAN_PROTOCOL_REGISTER_ACCESS_SIGNATURE,
                                       UAVCAN_PROTOCOL_REGISTER_ACCESS_ID,
                                       &transfer->transfer_id,  // Use the request's transfer ID
                                       transfer->priority,
                                       CanardResponse,
                                       buffer,
                                       2 + value_len);
                
                if (result >= 0) {
                    // Success - blink green LED
                    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
                    HAL_Delay(100);
                    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
                }
            }
        }
        
        // If register not found, send empty response
        if (!found) {
            // Empty response - value type 0 (string) with empty string
            buffer[0] = 0;
            buffer[1] = 0;
            
            canardRequestOrRespond(ins, 
                                  transfer->source_node_id,
                                  UAVCAN_PROTOCOL_REGISTER_ACCESS_SIGNATURE,
                                  UAVCAN_PROTOCOL_REGISTER_ACCESS_ID,
                                  &transfer->transfer_id,  // Use the request's transfer ID
                                  transfer->priority,
                                  CanardResponse,
                                  buffer,
                                  2);
        }
    }
    else if ((transfer->transfer_type == CanardTransferTypeMessage) && 
             (transfer->data_type_id == MSG_PORT_ID))
    {
        // Handle Real64 array message on port 1620 (Yakut test message)
        // Toggle LD4 (green) when we receive a Real64 array
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        
        // Process Real64 array data
        if (transfer->payload_len > 0)
        {
            // First byte is array length - access via payload_head
            uint8_t array_length = transfer->payload_head[0];
            
            // Check payload size matches expected size for the array
            if (transfer->payload_len == 1 + (array_length * sizeof(double)))
            {
                // Extract the array values
                // Note: Real64 values need to be carefully extracted due to alignment and endianness
                for (uint8_t i = 0; i < array_length && i < 10; i++)  // limit to 10 elements for safety
                {
                    // Extract each double (Real64) value - we need to copy to ensure proper alignment
                    double value;
                    
                    // In libcanard v0, we need to use canardDecodeScalar for safely extracting values
                    canardDecodeScalar(transfer, 
                                       8 + (i * sizeof(double) * 8), // bit offset (8 bits for length + position)
                                       64,                           // bit length (64 bits for double)
                                       false,                        // not signed
                                       &value);                      // destination
                    
                    // Do something with each value
                    // For demonstration, we'll flash the LED differently based on value
                    if (value > 0)
                    {
                        // Positive value - blink green LED for a short time
                        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
                        HAL_Delay(50);
                        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
                    }
                    else
                    {
                        // Negative or zero value - blink orange LED for a short time
                        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
                        HAL_Delay(50);
                        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                    }
                }
            }
        }
    }
    else if ((transfer->transfer_type == CanardTransferTypeMessage) && 
             (transfer->data_type_id == UAVCAN_RAW_COMMAND_ID))
    {
        // Flash green and orange LEDs to show raw command received
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        
        // Parse the raw command message
        if (transfer->payload_len > 0)
        {
            // Raw command format: 
            // Byte 0: Command type
            // Bytes 1-2: Command value (int16_t, little-endian)
            
            const uint8_t* payload = (const uint8_t*)transfer->payload_head;
            
            // For simplicity, we'll just extract the first command value (int16_t)
            // In a real implementation, you might want to check command type and handle accordingly
            int16_t command_value = 0;
            
            if (transfer->payload_len >= 3) {
                // Extract command value (little-endian)
                command_value = (int16_t)(payload[1] | (payload[2] << 8));
                
                // Process the command, translating to EPOS4 format
                handle_ardupilot_command(command_value);
            }
        }
    }
}

// New function to send node name as param string message
void send_node_name(void)
{
    // Format parameter message according to UAVCAN specification for ArduPilot
    // Parameter name: "name" (name of the parameter)
    // Parameter value: APP_NODE_NAME (value of the parameter)
    
    const char *param_name = "name";
    const char *param_value = APP_NODE_NAME;
    size_t name_len = strlen(param_name);
    size_t value_len = strlen(param_value);
    
    // Buffer for parameter message
    uint8_t buffer[128] = {0};
    uint16_t offset = 0;
    
    // Empty key (parameter index) - 0 means not using index
    buffer[offset++] = 0;
    
    // Parameter name as a string
    buffer[offset++] = (uint8_t)name_len;
    memcpy(&buffer[offset], param_name, name_len);
    offset += name_len;
    
    // Parameter value - tag for string (0 = string, 1 = integer, etc.)
    buffer[offset++] = 0;  // Tag for string value
    
    // Parameter value - string
    buffer[offset++] = (uint8_t)value_len;
    memcpy(&buffer[offset], param_value, value_len);
    offset += value_len;
    
    // Parameter default value (same as current value for simplicity)
    buffer[offset++] = 0;  // Tag for string value
    buffer[offset++] = (uint8_t)value_len;
    memcpy(&buffer[offset], param_value, value_len);
    offset += value_len;
    
    // Parameter max value (empty string)
    buffer[offset++] = 0;  // Tag for string
    buffer[offset++] = 0;  // Empty string
    
    // Parameter min value (empty string)
    buffer[offset++] = 0;  // Tag for string
    buffer[offset++] = 0;  // Empty string
    
    // Send the param string message
    static uint8_t param_transfer_id = 0;
    int result = canardBroadcast(&canard,
                  UAVCAN_PARAM_STRING_SIGNATURE,
                  UAVCAN_PARAM_STRING_ID,
                  &param_transfer_id,
                  CANARD_TRANSFER_PRIORITY_MEDIUM,
                  buffer,
                  offset);
    
    if (result >= 0) {
        // Blink green LED to indicate name message sent
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    } else {
        // Error sending - blink red LED
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    }
    
    // Increment transfer ID
    param_transfer_id = (param_transfer_id + 1) & 31;
}

// EPOS4 communication functions
void epos4_send_sdo_write(uint16_t index, uint8_t subindex, uint32_t data, uint8_t data_length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0};
    
    // Standard SDO write command format
    // Byte 0: Command specifier (0x22 for write <= 4 bytes with size in CS)
    // Byte 1-2: Index (little-endian)
    // Byte 3: Subindex
    // Byte 4-7: Data (little-endian)
    
    // Command byte depends on data length
    switch (data_length) {
        case 1:
            tx_data[0] = 0x2F;  // Write 1 byte
            break;
        case 2:
            tx_data[0] = 0x2B;  // Write 2 bytes
            break;
        case 3:
            tx_data[0] = 0x27;  // Write 3 bytes
            break;
        case 4:
            tx_data[0] = 0x23;  // Write 4 bytes
            break;
        default:
            // Unsupported length
            return;
    }
    
    // Index (little-endian)
    tx_data[1] = (uint8_t)(index & 0xFF);         // LSB
    tx_data[2] = (uint8_t)((index >> 8) & 0xFF);  // MSB
    
    // Subindex
    tx_data[3] = subindex;
    
    // Data (little-endian)
    for (uint8_t i = 0; i < data_length; i++) {
        tx_data[4 + i] = (uint8_t)((data >> (i * 8)) & 0xFF);
    }
    
    // Configure CAN message
    tx_header.StdId = EPOS4_SDO_WRITE + EPOS4_NODE_ID;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;  // Standard ID, not extended
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;  // Always 8 bytes for SDO
    
    // Send the message
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    
    // Blink orange LED (LD3) to indicate EPOS4 communication
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void epos4_send_sdo_read(uint16_t index, uint8_t subindex)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0};
    
    // Standard SDO read command format
    // Byte 0: Command specifier (0x40 for read)
    // Byte 1-2: Index (little-endian)
    // Byte 3: Subindex
    // Byte 4-7: Reserved (should be 0)
    
    tx_data[0] = 0x40;  // Read command
    
    // Index (little-endian)
    tx_data[1] = (uint8_t)(index & 0xFF);         // LSB
    tx_data[2] = (uint8_t)((index >> 8) & 0xFF);  // MSB
    
    // Subindex
    tx_data[3] = subindex;
    
    // Configure CAN message
    tx_header.StdId = EPOS4_SDO_WRITE + EPOS4_NODE_ID;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;  // Standard ID, not extended
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;  // Always 8 bytes for SDO
    
    // Send the message
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    
    // Blink orange LED (LD3) to indicate EPOS4 communication
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void epos4_set_operation_mode(uint8_t mode)
{
    // Set mode of operation (8-bit value)
    epos4_send_sdo_write(EPOS4_MODE_OF_OPERATION_INDEX, 0x00, mode, 1);
    
    // Wait a bit for the command to process
    HAL_Delay(10);
}

void epos4_set_target_velocity(int32_t target_velocity)
{
    // Set target velocity (32-bit value)
    epos4_send_sdo_write(EPOS4_TARGET_VELOCITY_INDEX, 0x00, (uint32_t)target_velocity, 4);
}

void epos4_set_target_position(int32_t target_position)
{
    // Set target position (32-bit value)
    epos4_send_sdo_write(EPOS4_TARGET_POSITION_INDEX, 0x00, (uint32_t)target_position, 4);
}

void epos4_enable_motor(void)
{
    // Write controlword to enable operation (0x000F)
    // 0x06: Shutdown (prepare for enable)
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x06, 2);
    HAL_Delay(10);
    
    // 0x07: Switch on
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x07, 2);
    HAL_Delay(10);
    
    // 0x0F: Enable operation
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x0F, 2);
    HAL_Delay(10);
    
    // Blink green LED (LD4) to indicate motor enabled
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
}

void epos4_disable_motor(void)
{
    // Write controlword to disable operation (0x0006)
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x06, 2);
    
    // Blink red LED (LD5) to indicate motor disabled
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
}

void epos4_send_pdo(uint16_t cob_id, uint8_t* data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    // Configure CAN message
    tx_header.StdId = cob_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;  // Standard ID, not extended
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    
    // Send the message
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
    
    // Blink orange LED (LD3) to indicate EPOS4 communication
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void handle_ardupilot_command(int16_t command_value)
{
    static int32_t last_position = TILT_CENTER_POSITION;
    int32_t target_position;
    
    // Map ArduPilot command value (-10000 to 10000) to position range
    // Apply deadband near center for stability
    if (command_value < -COMMAND_DEADBAND) {
        target_position = (int32_t)((float)(command_value + COMMAND_DEADBAND) / (10000.0f - COMMAND_DEADBAND) * (TILT_MIN_POSITION - TILT_CENTER_POSITION) + TILT_CENTER_POSITION);
    } else if (command_value > COMMAND_DEADBAND) {
        target_position = (int32_t)((float)(command_value - COMMAND_DEADBAND) / (10000.0f - COMMAND_DEADBAND) * (TILT_MAX_POSITION - TILT_CENTER_POSITION) + TILT_CENTER_POSITION);
    } else {
        target_position = TILT_CENTER_POSITION;
    }
    
    // Only update if the position has changed significantly
    if (abs(target_position - last_position) > 10) {
        // Set the target position on EPOS4
        EPOS4_SetTargetPosition_Legacy(&hcan1, EPOS4_NODE_ID, target_position);
        
        // Send control word to trigger the move
        EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x000F | 0x0010);
        
        last_position = target_position;
        
        // Indicate command processed
        HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
    }
}

// EPOS4 CANopen interface functions
bool EPOS4_SetTargetPosition_Legacy(CAN_HandleTypeDef* hcan, uint8_t node_id, int32_t position)
{
    uint8_t dlc = 4;
    uint8_t data[8] = {0};
    
    // Format SDO message for EPOS4 position control
    uint32_t can_id = 0x600 + node_id;  // 0x600 is the SDO transmit ID base
    
    // Format SDO write command
    data[0] = 0x23;  // SDO write command (2 bytes)
    data[1] = 0x7A;  // Index LSB (0x607A = target position)
    data[2] = 0x60;  // Index MSB
    data[3] = 0x00;  // Subindex
    
    // Format the position data (4 bytes, little-endian)
    data[4] = (uint8_t)(position & 0xFF);
    data[5] = (uint8_t)((position >> 8) & 0xFF);
    data[6] = (uint8_t)((position >> 16) & 0xFF);
    data[7] = (uint8_t)((position >> 24) & 0xFF);
    
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = can_id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    uint32_t tx_mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &tx_mailbox);
    
    return (status == HAL_OK);
}

void EPOS4_SetControlWord(CAN_HandleTypeDef* hcan, uint8_t node_id, uint16_t control_word)
{
    uint8_t dlc = 2;
    uint8_t data[8] = {0};
    uint32_t can_id = 0x600 + node_id;  // SDO request ID
    
    // Command to write to object dictionary at 0x6040 (control word)
    data[0] = 0x2B;  // Write command (2=write, B=2 bytes)
    data[1] = 0x40;  // Object index low byte
    data[2] = 0x60;  // Object index high byte
    data[3] = 0x00;  // Subindex
    
    // Control word value in little-endian format
    data[4] = (uint8_t)(control_word & 0xFF);
    data[5] = (uint8_t)((control_word >> 8) & 0xFF);
    
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = can_id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &mailbox);
    
    // Small delay to ensure message transmission
    HAL_Delay(5);
}

// Function to initialize the EPOS4 controller
bool EPOS4_Init_Legacy(CAN_HandleTypeDef* hcan, uint8_t node_id)
{
    // Reset communication
    EPOS4_SetControlWord(hcan, node_id, 0x0080);  // Fault reset
    HAL_Delay(10);
    
    // Set mode of operation (Profile Position Mode)
    uint8_t data[8] = {0};
    uint32_t can_id = 0x600 + node_id;
    
    // Set up write message to change operation mode
    data[0] = 0x2F;  // SDO write command (1 byte)
    data[1] = 0x60;  // Index LSB (0x6060 = Mode of operation)
    data[2] = 0x60;  // Index MSB
    data[3] = 0x00;  // Subindex
    data[4] = 0x01;  // Profile Position Mode
    
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = can_id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    uint32_t tx_mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &tx_mailbox);
    
    if (status != HAL_OK) {
        return false;
    }
    
    HAL_Delay(10);
    
    // Send shutdown command
    EPOS4_SetControlWord(hcan, node_id, CMD_SHUTDOWN);
    HAL_Delay(10);
    
    // Send switch on command
    EPOS4_SetControlWord(hcan, node_id, CMD_SWITCH_ON);
    HAL_Delay(10);
    
    // Enable operation
    EPOS4_SetControlWord(hcan, node_id, CMD_ENABLE_OPERATION);
    HAL_Delay(10);
    
    return true;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
  // In case of error, turn on red LED (LD5) and keep it on
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
  
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

/**
 * @brief Send CANopen NMT command
 * @param hcan CAN handle
 * @param command NMT command (e.g., 0x01 = start, 0x02 = stop, 0x80 = pre-operational, 0x81 = reset node, 0x82 = reset communication)
 * @param node_id Target node ID or 0 for all nodes
 */
void CANopen_NMT_Command(CAN_HandleTypeDef* hcan, uint8_t command, uint8_t node_id)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[2];
    
    // NMT message is always ID 0x000
    TxHeader.StdId = 0x000;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // First byte is the command
    TxData[0] = command;
    // Second byte is the node ID (0 means all nodes)
    TxData[1] = node_id;
    
    // Send the NMT command
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    
    // Short delay to ensure message is processed
    HAL_Delay(10);
}

/**
 * @brief Set EPOS4 mode of operation
 * @param hcan CAN handle
 * @param node_id EPOS4 node ID
 * @param mode Operation mode (e.g., MODE_PROFILE_POSITION)
 */
void EPOS4_SetOperationMode(CAN_HandleTypeDef* hcan, uint8_t node_id, uint8_t mode)
{
    // Send SDO command to set operation mode
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = {0};
    
    TxHeader.StdId = 0x600 + node_id; // SDO transmit ID
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Command to write 1 byte to object 0x6060 (modes of operation) subindex 0
    TxData[0] = 0x2F;  // Write 1 byte (expedited)
    TxData[1] = 0x60;  // Index LSB
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    TxData[4] = mode;  // Mode value
    // Bytes 5-7 are padding (0)
    
    // Send message
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    
    // Wait for message to be processed
    HAL_Delay(10);
}

/**
 * @brief Set EPOS4 profile velocity
 * @param hcan CAN handle
 * @param node_id EPOS4 node ID
 * @param velocity Velocity value (encoder counts/sec)
 */
void EPOS4_SetProfileVelocity(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t velocity)
{
    // Send SDO command to set profile velocity
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = {0};
    
    TxHeader.StdId = 0x600 + node_id; // SDO transmit ID
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Command to write 4 bytes to object 0x6081 (profile velocity) subindex 0
    TxData[0] = 0x23;  // Write 4 bytes (expedited)
    TxData[1] = 0x81;  // Index LSB
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    
    // Velocity value in little-endian format
    TxData[4] = (uint8_t)(velocity & 0xFF);
    TxData[5] = (uint8_t)((velocity >> 8) & 0xFF);
    TxData[6] = (uint8_t)((velocity >> 16) & 0xFF);
    TxData[7] = (uint8_t)((velocity >> 24) & 0xFF);
    
    // Send message
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    
    // Wait for message to be processed
    HAL_Delay(10);
}

/**
 * @brief Set EPOS4 profile acceleration
 * @param hcan CAN handle
 * @param node_id EPOS4 node ID
 * @param acceleration Acceleration value (encoder counts/sec²)
 */
void EPOS4_SetProfileAcceleration(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t acceleration)
{
    // Send SDO command to set profile acceleration
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = {0};
    
    TxHeader.StdId = 0x600 + node_id; // SDO transmit ID
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Command to write 4 bytes to object 0x6083 (profile acceleration) subindex 0
    TxData[0] = 0x23;  // Write 4 bytes (expedited)
    TxData[1] = 0x83;  // Index LSB
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    
    // Acceleration value in little-endian format
    TxData[4] = (uint8_t)(acceleration & 0xFF);
    TxData[5] = (uint8_t)((acceleration >> 8) & 0xFF);
    TxData[6] = (uint8_t)((acceleration >> 16) & 0xFF);
    TxData[7] = (uint8_t)((acceleration >> 24) & 0xFF);
    
    // Send message
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    
    // Wait for message to be processed
    HAL_Delay(10);
}

/**
 * @brief Set EPOS4 profile deceleration
 * @param hcan CAN handle
 * @param node_id EPOS4 node ID
 * @param deceleration Deceleration value (encoder counts/sec²)
 */
void EPOS4_SetProfileDeceleration(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t deceleration)
{
    // Send SDO command to set profile deceleration
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = {0};
    
    TxHeader.StdId = 0x600 + node_id; // SDO transmit ID
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Command to write 4 bytes to object 0x6084 (profile deceleration) subindex 0
    TxData[0] = 0x23;  // Write 4 bytes (expedited)
    TxData[1] = 0x84;  // Index LSB
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    
    // Deceleration value in little-endian format
    TxData[4] = (uint8_t)(deceleration & 0xFF);
    TxData[5] = (uint8_t)((deceleration >> 8) & 0xFF);
    TxData[6] = (uint8_t)((deceleration >> 16) & 0xFF);
    TxData[7] = (uint8_t)((deceleration >> 24) & 0xFF);
    
    // Send message
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    
    // Wait for message to be processed
    HAL_Delay(10);
}
