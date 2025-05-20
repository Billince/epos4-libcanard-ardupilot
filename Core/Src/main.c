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
#include <stdio.h>  // Add stdio.h for printf
#include <stdarg.h> // Add stdarg.h for variable arguments
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_cortex.h"
#include <math.h>
// Add IWDG suppor
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Silence unused variable warnings
#define UNUSED(x) (void)(x)

// UAVCAN message type definitions
#define UAVCAN_PROTOCOL_NODE_STATUS_MESSAGE_ID    341
#define UAVCAN_PROTOCOL_GETNODEINFO_ID            1
#define UAVCAN_PROTOCOL_PARAM_GETSET_ID           11
#define UAVCAN_PROTOCOL_ACCESS_COMMAND_ID         11

// UAVCAN Equipment Actuator Status message definitions
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_DATA_TYPE_ID            1011
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_DATA_TYPE_SIGNATURE     0x5E9BBA44CBEADED3ULL
#define UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE                48  // Bytes

// UAVCAN Equipment Actuator ArrayCommand message definitions
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID 1010
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE 0xd8a7486238ec3af3ULL

// ArduPilot specific signature for debugging (message type 20007)
#define ARDUPILOT_GNSS_STATUS_ID 20007
#define ARDUPILOT_GNSS_STATUS_SIGNATURE 0xFEDCBA0987654321ULL  // Placeholder signature

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// DroneCAN/UAVCAN definitions
#define CANARD_CAN_EXT_ID_MASK         0x1FFFFFFFU
#define NODE_ID                        9
#define UAVCAN_NODE_STATUS_MESSAGE_SIZE 7  // Standard size (not 90)
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID 341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE 0x0f0868d0c1a7c6f1ULL

#define UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE 0xEE468A8121C46A9EULL
#define UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE 0xEE468A8121C46A9EULL

// GetNodeInfo service definitions
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID 1
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE 0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE ((3015 + 7) / 8)

// Add the correct dynamic node ID allocation signature
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE 0x0b2a812620a11d40ULL

// Add the correct actuator array command signature
#define UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE 0xd8a7486238ec3af3ULL

#define UNIQUE_ID_LENGTH_BYTES                                      16

// Parameter protocol definitions
#define UAVCAN_PARAM_STRING_ID 341 // Changed from 11 to avoid conflict with node ID
#define UAVCAN_PARAM_STRING_SIGNATURE 0xe02f25d6e0c98ae0ULL
#define UAVCAN_PARAM_GETSET_ID 11
#define UAVCAN_PARAM_GETSET_SIGNATURE 0xa7b622f939d1a4d5

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
#define EPOS4_MAX_VELOCITY             20000 // Default velocity for movements
#define EPOS4_MAX_ACCELERATION         50000 // Default acceleration for movements

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
#define EPOS4_ACTUAL_POSITION_SUBINDEX 0x00


#define UAVCAN_NODE_HEALTH_OK          0
#define UAVCAN_NODE_HEALTH_WARNING     1
#define UAVCAN_NODE_HEALTH_ERROR       2
#define UAVCAN_NODE_HEALTH_CRITICAL    3

#define UAVCAN_NODE_MODE_OPERATIONAL   0
#define UAVCAN_NODE_MODE_INITIALIZATION 1
#define UAVCAN_NODE_MODE_MAINTENANCE   2
#define UAVCAN_NODE_MODE_SOFTWARE_UPDATE 3
#define UAVCAN_NODE_MODE_OFFLINE       7

// Flash storage defines for position memory
#define FLASH_SECTOR_POSITION      FLASH_SECTOR_11  // Use sector 11 for position storage  
#define POSITION_FLASH_ADDRESS     0x080E0000       // Address in flash for position storage
#define POSITION_MAGIC_NUMBER      0xEABF3412       // Magic number to validate position data

// Ensure CanardTransferTypeMessage is defined in case the libcanard version doesn't match
#ifndef CanardTransferTypeMessage
#define CanardTransferTypeMessage 2
#endif

// Node name and version info
#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 0
#define APP_NODE_NAME "epos4.motorctl"  // Better name to match parameter format
#define APP_NODE_NAME_MAX_LENGTH 80 // Maximum length of node name string
#define GIT_HASH  0xBADC0FFE

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

// Parameter system definitions
#define MAX_PARAMETERS              100
#define PARAM_NAME_MAX_LENGTH       16
#define EPOS4_POSITION_LIMIT        100000

int printf(const char* format, ...) {
    // Remove all the code inside and replace with:
    return 0;  // Do nothing, just return 0
}
// Define param_t type
typedef struct parameter param_t;

// Parameter type enumeration
enum param_type {
    PARAM_TYPE_EMPTY = 0,
    PARAM_TYPE_INTEGER = 1,
    PARAM_TYPE_BOOLEAN = 2,
    PARAM_TYPE_REAL = 3,
    PARAM_TYPE_STRING = 4
};

// Parameter structure definition
struct parameter {
    char name[PARAM_NAME_MAX_LENGTH];
    enum param_type type;
    union {
        int32_t i;
        float f;
        bool b;
        char str[PARAM_NAME_MAX_LENGTH];
    } value;
    int32_t val;
    int64_t defval;  // Add default value
    int64_t max;     // Add max value
    int64_t min;     // Add min value
};

// Parameter storage
static struct parameter parameters[MAX_PARAMETERS] = {
    {"epos4.node_name", PARAM_TYPE_STRING, .value.str = "motorctl", .val = 0, .min = 0, .max = 0, .defval = 0},
    {"epos4.can_id", PARAM_TYPE_INTEGER, .value.i = NODE_ID, .val = NODE_ID, .min = 1, .max = 127, .defval = NODE_ID},
    {"epos4.drv_id", PARAM_TYPE_INTEGER, .value.i = EPOS4_NODE_ID, .val = EPOS4_NODE_ID, .min = 1, .max = 127, .defval = EPOS4_NODE_ID},
    {"epos4.min_pos", PARAM_TYPE_REAL, .value.f = -1.0f, .val = -1, .min = -1, .max = 0, .defval = -1},
    {"epos4.max_pos", PARAM_TYPE_REAL, .value.f = 1.0f, .val = 1, .min = 0, .max = 1, .defval = 1},
    {"epos4.center", PARAM_TYPE_REAL, .value.f = 0.0f, .val = 0, .min = -1, .max = 1, .defval = 0},
    {"epos4.velocity", PARAM_TYPE_INTEGER, .value.i = 50000, .val = 50000, .min = 5000, .max = 100000, .defval = 50000},  // Default velocity
    {"epos4.accel_rate", PARAM_TYPE_INTEGER, .value.i = 20000, .val = 20000, .min = 5000, .max = 100000, .defval = 20000}, // Default acceleration
    {"epos4.decel_rate", PARAM_TYPE_INTEGER, .value.i = 20000, .val = 20000, .min = 5000, .max = 100000, .defval = 20000}, // Default deceleration
    {"epos4.mode_vel", PARAM_TYPE_INTEGER, .value.i = 30000, .val = 30000, .min = 5000, .max = 100000, .defval = 30000},   // Mode change velocity
    {"epos4.mode_accel", PARAM_TYPE_INTEGER, .value.i = 40000, .val = 40000, .min = 5000, .max = 100000, .defval = 40000}, // Mode change acceleration
    {"epos4.fbwa_vel", PARAM_TYPE_INTEGER, .value.i = 25000, .val = 25000, .min = 5000, .max = 100000, .defval = 25000},   // FBWA mode velocity
    {"epos4.qhover_vel", PARAM_TYPE_INTEGER, .value.i = 35000, .val = 35000, .min = 5000, .max = 100000, .defval = 35000}, // QHover mode velocity
    {"epos4.vel_max", PARAM_TYPE_INTEGER, .value.i = EPOS4_MAX_VELOCITY, .val = EPOS4_MAX_VELOCITY, .min = 0, .max = 100000, .defval = EPOS4_MAX_VELOCITY},
    {"epos4.accel", PARAM_TYPE_INTEGER, .value.i = EPOS4_MAX_ACCELERATION, .val = EPOS4_MAX_ACCELERATION, .min = 0, .max = 100000, .defval = EPOS4_MAX_ACCELERATION},
    {"epos4.op_mode", PARAM_TYPE_INTEGER, .value.i = 1, .val = 1, .min = 0, .max = 10, .defval = 1},  // 1=Position, 3=Velocity
    {"epos4.res", PARAM_TYPE_INTEGER, .value.i = EPOS4_RESOLUTION, .val = EPOS4_RESOLUTION, .min = 1, .max = 10000, .defval = EPOS4_RESOLUTION},
    {"epos4.pos", PARAM_TYPE_REAL, .value.f = 0.0f, .val = 0, .min = -1, .max = 1, .defval = 0}
};

// Parameter handling function declarations
static inline param_t * getParamByIndex(uint16_t index);
static inline param_t * getParamByName(const char * name);

// Make position variables global
static int32_t maxPosition = 370000;           // Reduced maximum position
static int32_t midPosition = -270000;                // Middle position
static int32_t minPosition = -180000;          // Reduced minimum position
static int32_t targetPosition = 0;             // Start at middle position
static uint8_t positionIndex = 1;              // Start at middle position

// Global variable for tracking flight mode
uint8_t current_flight_mode = 0;    // 0=unknown, 1=FBWA, 2=QHover, etc.

// Parameters for smooth motor motion - reduced to stay within EPOS4 limits
static uint32_t profile_velocity = 7000;      // Reduced velocity to avoid exceeding range
static uint32_t profile_acceleration = 1000;   // Reduced acceleration to avoid exceeding range
static uint32_t profile_deceleration = 2000;   // Reduced deceleration to avoid exceeding range

// External variables
extern int32_t current_position;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Global variables for command handling
volatile bool command_pending = false;
volatile int32_t pending_position = 0;
volatile uint32_t last_1hz_tasks_ms = 0;
volatile uint32_t last_tx_processing_ms = 0;
volatile uint64_t current_time_us = 0;
volatile uint64_t current_time_ms = 0;
// Variables for initial position synchronization
volatile bool initial_sync_completed = false;
volatile int16_t latest_ardupilot_command = 0;
volatile bool ardupilot_command_received = false;

CAN_HandleTypeDef hcan1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

// Add current_position variable
int32_t current_position = 0;  // Current position of the EPOS4 motor

// Add last position tracking for mode changes
int32_t last_mode_position = 0; // Last position before mode change
volatile bool mode_change_pending = false;
uint32_t last_mode_change_ms = 0;

// Variable to track if all parameters have been broadcast
volatile bool all_params_broadcast = false;

/* USER CODE BEGIN PV */
// Memory pool for libcanard
static uint8_t canard_memory_pool[16384];  // Increased to match CAN_D1_UC_POOL
// Canard instance
CanardInstance canard;

// Debug LED states
uint8_t can_tx_led = 0;
uint8_t can_rx_led = 0;

// Flag for CAN error tracking
volatile bool last_sdo_error = false;  // Set when an SDO error occurs

// Message port ID for Real64 array (Yakut test message)
#define MSG_PORT_ID 1620U

uint32_t test_uptimeSec = 0;
// Buffer for heartbeat messages
#define HEARTBEAT_BUFFER_SIZE 16
uint8_t hbeat_ser_buf[HEARTBEAT_BUFFER_SIZE];

// UART related variables
uint8_t uart_rx_ready = 0;
uint8_t uart_rx_buffer[256];

// Add these to the top of the file, after includes but before your declarations
#define UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE 0xa7b622f939d1a4d5ULL

// In the Private variables section, add these variables
uint64_t next_1hz_service_at = 0;
uint64_t ts = 0;
uint32_t current_ms = 0;
uint32_t last_epos4_statuscheck_ms = 0;

// Add these function prototypes in the function prototypes section around line 360
void save_position_to_flash(int32_t position);
bool restore_position_from_flash(void);
void periodic_position_save(void);

// Add this variable with the other global variables around line 260-270
volatile bool position_restored = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

// Add function declarations
void send_get_node_info_request(uint8_t target_node_id);
void send_actuator_status(void);

/* USER CODE BEGIN PFP */
static void* memAllocate(CanardInstance* const ins, const size_t amount);
static void memFree(CanardInstance* const ins, void* const pointer);
void process_canard_TX_queue(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void send_NodeStatus(void);
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                uint64_t* out_data_type_signature,
                                uint16_t data_type_id,
                                CanardTransferType transfer_type,
                                uint8_t source_node_id);
void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
void handle_get_node_info(CanardInstance* ins, CanardRxTransfer* transfer);
void handleActuatorArrayCommand(CanardInstance* ins, CanardRxTransfer* transfer);
void initializePositionParameters(void);
uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE]);
void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]);
void Error_Handler(void);

// Helper function for CAN message transmission
bool wait_for_tx_complete(uint32_t timeout_ms);
HAL_StatusTypeDef send_with_check(CAN_TxHeaderTypeDef* header, uint8_t* data);

// EPOS4 functions via CANopen communication
void CANopen_NMT_Command(CAN_HandleTypeDef* hcan, uint8_t command, uint8_t node_id);
void EPOS4_SetOperationMode(CAN_HandleTypeDef* hcan, uint8_t node_id, uint8_t mode);
void EPOS4_SetProfileVelocity(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t velocity);
void EPOS4_SetProfileAcceleration(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t acceleration);
void EPOS4_SetProfileDeceleration(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t deceleration);

// Functions for epos4 via libcanard
void epos4_send_sdo_write(uint16_t index, uint8_t subindex, uint32_t data, uint32_t data_length);
void epos4_send_sdo_read(uint16_t index, uint8_t subindex);
void epos4_read_position(uint16_t index, uint8_t subindex);
void epos4_set_operation_mode(uint8_t mode);
void epos4_set_target_velocity(int32_t target_velocity);
void epos4_set_target_position(int32_t target_position);
void epos4_enable_motor(void);
void epos4_send_pdo(uint16_t cob_id, uint8_t* data, uint8_t length);
void handle_ardupilot_command(int16_t command_value);

// EPOS4 CANopen interface functions
void EPOS4_SetControlWord(CAN_HandleTypeDef* hcan, uint8_t node_id, uint16_t control_word);

// Add these function prototypes to the proper section (in USER CODE BEGIN PFP)
void processTxRxOnce(CanardInstance *ins, uint32_t timeout_ms);
uint64_t micros64(void);
void process1HzTasks(CanardInstance* ins);
void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer);
void save_settings(void);
void rawcmdHandleCanard(CanardRxTransfer* transfer);
float translateEPOS4Position(int32_t position);
float getEPOS4NormalizedPosition(void);
int32_t getNormalizedToEPOS4Position(float normalized_position);
// Add this declaration with the other function prototypes
void broadcast_all_parameters(void);
// Add our new function prototype with the other function prototypes in the appropriate section
int force_motor_movement(int32_t position);
void broadcast_parameter(param_t* p);
void check_epos4_status(void);
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

 /* Check reset flags to determine reset cause */
   if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0) {
     printf("System reset caused by Independent Watchdog (IWDG)\r\n");
     RCC->CSR |= RCC_CSR_RMVF; // Clear all reset flags
   }

   /* Configure IWDG with direct register access */
   // Start by writing the key to unlock IWDG registers
   IWDG->KR = 0x5555; // Enable write access to IWDG_PR and IWDG_RLR

   // Set prescaler to maximum value (256) using direct register access
   IWDG->PR = 0x07; // Max prescaler (divide by 256)

   // Set reload value to maximum (0xFFF = 4095) using direct register access
   IWDG->RLR = 0xFFF; // Max reload value

   // Reload the watchdog counter to apply settings
   IWDG->KR = 0xAAAA;

   // Start the watchdog
   IWDG->KR = 0xCCCC;

   printf("IWDG configured with maximum timeout using direct register access\r\n");

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

  printf("-------------------------------------\r\n");
  printf("EPOS4 UAVCAN Node - Starting up\r\n");
  printf("STM32F4Discovery with CAN interface\r\n");
  printf("Node ID: %d\r\n", NODE_ID);
  printf("-------------------------------------\r\n");
  
  // Configure CAN filter to accept ALL messages with no filtering
  CAN_FilterTypeDef canFilter;
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;  // Accept any ID
  canFilter.FilterMaskIdHigh = 0x0000;  // Don't mask any bits (accept everything)
  canFilter.FilterMaskIdLow = 0x0000;   // Don't mask any bits (accept everything)
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter.FilterActivation = CAN_FILTER_ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
    Error_Handler();
  }
  
  // Enable CAN bus
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  
  // Enable CAN interrupts for receive
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  
  // Start libcanard
  canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);
  canardSetLocalNodeID(&canard, NODE_ID);



  // Initialize EPOS4 controller for position control mode

  // Step 1: Reset communications
  CANopen_NMT_Command(&hcan1, 0x82, EPOS4_NODE_ID); // 0x82 = Reset communications
  HAL_Delay(100); // Wait longer for reset to complete
  // Step 2: Start remote node
  CANopen_NMT_Command(&hcan1, 0x01, EPOS4_NODE_ID); // 0x01 = Start remote node
  HAL_Delay(100);

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
  printf("Setting profile velocity: %lu\n", (unsigned long)profile_velocity);
  EPOS4_SetProfileVelocity(&hcan1, EPOS4_NODE_ID, profile_velocity);
  HAL_Delay(100);

  printf("Setting profile acceleration: %lu\n", (unsigned long)profile_acceleration);
  EPOS4_SetProfileAcceleration(&hcan1, EPOS4_NODE_ID, profile_acceleration);
  HAL_Delay(100);

  printf("Setting profile deceleration: %lu\n", (unsigned long)profile_deceleration);
  EPOS4_SetProfileDeceleration(&hcan1, EPOS4_NODE_ID, profile_deceleration);
  HAL_Delay(100);

  // Step 7: Enable operation
  EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x000F);
  HAL_Delay(100);

  // Step 8: Configure default motion profile parameters - IMPORTANT!
  printf("Setting default motion parameters\r\n");
  printf("Position limits set to: min=%ld, mid=%ld, max=%ld\n",
         (long)minPosition, (long)midPosition, (long)maxPosition);
  printf("Motion profile: velocity=%lu, accel=%lu, decel=%lu\n",
         (unsigned long)profile_velocity,
         (unsigned long)profile_acceleration,
         (unsigned long)profile_deceleration);

  // Try to restore position from flash
  position_restored = restore_position_from_flash();
  if (!position_restored) {
      printf("No saved position found - starting at current position\n");
  }

  /* USER CODE END 2 */

  // *** Add after the main loop initialization ***
  uint64_t last_getnodeinfo_self_request_time_ms = 0;


  // Initialize position parameters with defaults
  initializePositionParameters();

  // Broadcast parameters to improve detection speed
  broadcast_all_parameters();

  // Set up for initial position synchronization with ArduPilot
  printf("Waiting for first ArduPilot command to synchronize position...\n");
  initial_sync_completed = false;
  ardupilot_command_received = false;
  
  /* USER CODE END 3 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // Refresh the watchdog timer to prevent system reset
    IWDG->KR = 0xAAAA;

    // Get the current time
    uint64_t current_time_ms = HAL_GetTick();
    uint64_t current_time_us = micros64();
    
    // Check if we need to perform initial position sync (only once at startup)
    if (!initial_sync_completed && ardupilot_command_received) {
        // First ArduPilot command received - use it to synchronize position
        printf("Initial sync: First ArduPilot command received: %d\n", latest_ardupilot_command);

        // Calculate position based on the command value using the same mapping logic
        int32_t sync_position;
        
        // Special handling for vectored yaw mode (values near 15360)
        if (abs(latest_ardupilot_command - 15360) < 1000) {
            // Calculate deviation from center (15360)
            int16_t deviation = latest_ardupilot_command - 15360;
            // Map to position around minPosition
            float normalized = (float)deviation / 1000.0f;  // +/-1.0 for +/-1000 deviation
            sync_position = minPosition - (int32_t)(normalized * 100000);

        }
        // Standard mapping for regular commands
        else if (latest_ardupilot_command <= 15000) {
            sync_position = minPosition;
        }
        else if (latest_ardupilot_command >= 48000) {
            sync_position = maxPosition;
        }
        else {
            // For values in the normal range
            float normalized = (float)(latest_ardupilot_command - 15360) / (float)(48128 - 15360);
            if (normalized < 0.0f) normalized = 0.0f;
            if (normalized > 1.0f) normalized = 1.0f;
            sync_position = minPosition + (int32_t)(normalized * (maxPosition - minPosition));

        }
        
        // Apply safety limits
        if (sync_position > maxPosition) sync_position = maxPosition;
        if (sync_position < minPosition) sync_position = minPosition;
        
        // Set EPOS4 position counter (object 0x2081)
        printf("Initial sync: Setting EPOS4 position counter to %ld\n", (long)sync_position);
        epos4_send_sdo_write(0x2081, 0x00, sync_position, 4);
        
        // Wait for the command to process
        HAL_Delay(50);

        // Update internal tracking
        current_position = sync_position;
        targetPosition = sync_position;

        // Mark sync as completed
        initial_sync_completed = true;
        printf("Initial position synchronization completed\n");

        // Set following error window to 2000 increments
        printf("Setting following error window (0x6065) to 2000 increments\n");
        epos4_send_sdo_write(0x6065, 0x00, 100000000, 4);

        // Also read back the value to verify it was set correctly
        printf("Reading back following error window value...\n");
        epos4_send_sdo_read(0x6065, 0x00);
        HAL_Delay(50); // Wait for response

        // Process any received messages to get the response
        processTxRxOnce(&canard, 10);
    }

    // Simple loop counter for debugging how many cycles we've completed
    static uint32_t loop_counter = 0;
    if (loop_counter++ % 10000 == 0) {
        // Reduced frequency to once per 10000 cycles
    }
    
    // Process CAN messages and send heartbeat every second - with timeout
    process1HzTasks(&canard);

    // Check for any received CAN frames (non-blocking, 10ms timeout)
    processTxRxOnce(&canard, 10);

    // Refresh watchdog again
    IWDG->KR = 0xAAAA;

    // Cleanup any stale transfers
    canardCleanupStaleTransfers(&canard, current_time_us);

    // Check if there is a pending command to process
    if (command_pending) {
        // Only log once at the start of command processing
        static bool command_logged = false;
        if (!command_logged) {
        printf("Processing command: position=%ld\r\n", (long)pending_position);
            command_logged = true;
        }

        // Call the state machine function - it will handle timing internally
        // Return values: 0=failure, 1=success, 2=in progress
        int result = force_motor_movement(pending_position);

        if (result == 1) {
            // Command completed successfully
            printf("Motor movement successful - target position set to %ld\r\n", (long)pending_position);
        
        // Update current position
        current_position = pending_position;

        // Toggle LED to show command processed
        HAL_GPIO_TogglePin(GPIOD, LD4_Pin);

            // Clear the command pending flag and reset the log flag
            command_pending = false;
            command_logged = false;
        }
        else if (result == 0) {
            // Command failed
            printf("Command failed - motor movement error\r\n");

            // Turn on error LED
            HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);

            // Clear the command pending flag and reset the log flag
            command_pending = false;
            command_logged = false;
        }
        // If result == 2, command is still in progress, keep the command_pending flag set
    }
    
    // Reduce main loop delay to minimize command latency (was 1ms)
    HAL_Delay(0);

    // Periodically check EPOS4 status and reset any faults - reduced frequency for lower latency
    static uint32_t last_epos4_check_ms = 0;
    if (current_time_ms - last_epos4_check_ms >= 500) { // Every 500ms (reduced from 1000ms)
        // Send an NMT heartbeat to keep the EPOS4 in operational state
        CANopen_NMT_Command(&hcan1, 0x01, EPOS4_NODE_ID); // 0x01 = Start remote node
        HAL_Delay(2); // Reduced from 5ms
        
        // Send fault reset command when not actively moving
        if (!command_pending) {
            // Reset any faults with control word 0x0080
            EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x0080);
            HAL_Delay(2); // Reduced from 5ms

            // Set to operational state (0x000F)
            EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x000F);
            HAL_Delay(2); // Reduced from 5ms
        }

        last_epos4_check_ms = current_time_ms;
    }
    
    // Check if we need to save position to flash
    periodic_position_save();
  }
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
    // Process TX queue with timeout to prevent blocking
    const uint32_t timeout_ms = 10;  // 10ms timeout
    uint32_t start_time = HAL_GetTick();

    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        // Check if we've exceeded our timeout
        if (HAL_GetTick() - start_time >= timeout_ms) {
            break;
        }

        CAN_TxHeaderTypeDef tx_header;
        uint32_t tx_mailbox;

        tx_header.StdId = 0;
        tx_header.ExtId = txf->id & CANARD_CAN_EXT_ID_MASK;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = txf->data_len;

        // Try to send the frame with a short timeout
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*)txf->data, &tx_mailbox) == HAL_OK) {
            // Success - remove from queue
            canardPopTxQueue(&canard);

            // Toggle TX LED (optional)
            can_tx_led = !can_tx_led;
        } else {
            // Failed to send - break and try again next time
            break;
        }
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    // Toggle orange LED (LD3) to show RX activity
    can_rx_led = !can_rx_led;

    // Record reception timestamp
    uint64_t timestamp_usec = micros64();

    // Prepare the frame structure for Canard
    CanardCANFrame frame;

    // Set ID and flags
    if (rx_header.IDE == CAN_ID_EXT) {
        // Extended ID frame with the CANARD_CAN_FRAME_EFF flag set
        frame.id = rx_header.ExtId | CANARD_CAN_FRAME_EFF;
    } else {
        // Standard ID frame
        frame.id = rx_header.StdId;
    }

    // Set RTR flag if this is a remote frame
    if (rx_header.RTR == CAN_RTR_REMOTE) {
        frame.id |= CANARD_CAN_FRAME_RTR;
    }

    // Copy data and set timestamp
    frame.data_len = rx_header.DLC;
    memcpy(frame.data, rx_data, frame.data_len);

    // Process the received frame with Canard
    int result = canardHandleRxFrame(&canard, &frame, timestamp_usec);

    // Check if this is a SDO read response for position
    if (rx_header.StdId == (EPOS4_SDO_READ_REPLY + EPOS4_NODE_ID)) {
        // Feed watchdog during message processing
        IWDG->KR = 0xAAAA;

        // IMPROVED DEBUGGING: Print all SDO responses with their index and data
        printf("SDO_RESPONSE: Command=0x%02X, Index=0x%02X%02X, Subindex=0x%02X, Data=0x%02X%02X%02X%02X\n",
            rx_data[0], rx_data[1], rx_data[2], rx_data[3],
            rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

        // Check for actual position response
        if (rx_data[0] == 0x43 &&  // Read response
            rx_data[1] == (EPOS4_ACTUAL_POSITION_INDEX & 0xFF) &&
            rx_data[2] == ((EPOS4_ACTUAL_POSITION_INDEX >> 8) & 0xFF) &&
            rx_data[3] == EPOS4_ACTUAL_POSITION_SUBINDEX) {

            // Extract position value (4 bytes, little-endian)
            int32_t position_value = (int32_t)rx_data[4] |
                                  ((int32_t)rx_data[5] << 8) |
                                  ((int32_t)rx_data[6] << 16) |
                                  ((int32_t)rx_data[7] << 24);

            // Update global position variable
            current_position = position_value;

            // Print position more prominently for debugging
            printf("CURRENT POSITION: %ld\n", (long)current_position);

            // Update position parameter with normalized value
            param_t* pos_param = getParamByName("epos4.pos");
            if (pos_param != NULL) {
                pos_param->value.f = translateEPOS4Position(current_position);
            }
        }
        // Check for statusword response (improved debugging)
        else if ((rx_data[0] & 0x7F) == 0x4B &&  // Read response (2 bytes)
                rx_data[1] == (EPOS4_STATUSWORD_INDEX & 0xFF) &&
                rx_data[2] == ((EPOS4_STATUSWORD_INDEX >> 8) & 0xFF) &&
                rx_data[3] == 0x00) {  // Subindex 0

            // Extract the statusword (2 bytes, little-endian)
            uint16_t statusword = (uint16_t)rx_data[4] | ((uint16_t)rx_data[5] << 8);

            // Detailed status interpretation
            
            
            // Detailed response based on status
            if (statusword & 0x0008) {
                // Try to reset fault
                epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x0080, 2); // Fault reset
                // Turn on red LED to indicate error
                // Then try to re-enable
                epos4_enable_motor();
            } else if (!(statusword & 0x0004)) {
                // Attempt to enable operation
                epos4_enable_motor();
            } else {
                // Operation enabled - turn off error LED
            }
        }
        // Also check for generic SDO responses to handle errors
        else if ((rx_data[0] & 0x80) == 0x80) {
            // This is an SDO abort message
            uint32_t abort_code = (uint32_t)rx_data[4] |
                              ((uint32_t)rx_data[5] << 8) |
                              ((uint32_t)rx_data[6] << 16) |
                              ((uint32_t)rx_data[7] << 24);

            
            // Set the global error flag to trigger error handling in state machine
            last_sdo_error = true;
            
            // Provide more detail on common error codes
            switch (abort_code) {
                case 0x05030000:
                    printf("  Toggle bit not alternated\n");
                    break;
                case 0x05040001:
                    printf("  Client/server command specifier not valid or unknown\n");
                    break;
                case 0x06010000:
                    printf("  Unsupported access to an object\n");
                    break;
                case 0x06010001:
                    printf("  Attempt to read a write only object\n");
                    break;
                case 0x06010002:
                    printf("  Attempt to write a read only object\n");
                    break;
                case 0x06020000:
                    printf("  Object does not exist in the object dictionary\n");
                    break;
                case 0x06040042:
                    printf("  PDO length exceeded\n");
                    break;
                case 0x06060000:
                    printf("  Access failed due to a hardware error\n");
                    break;
                case 0x06070010:
                    printf("  Data type does not match, length does not match\n");
                    break;
                case 0x06090011:
                    printf("  Subindex does not exist\n");
                    break;
                case 0x06090030:
                    printf("  Value range of parameter exceeded\n");
                    break;
                case 0x08000000:
                    printf("  General error\n");
                    break;
                default:
                    printf("  Unknown error code\n");
                    break;
            }

            // Blink red LED to indicate error
            HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Similar to FIFO0 callback if needed
}

// Change this from static to non-static to match declaration
void processTxRxOnce(CanardInstance *ins, uint32_t timeout_ms)
{
    // Feed the watchdog before starting
    IWDG->KR = 0xAAAA;

    // Set timeout for processing
    uint32_t start_time = HAL_GetTick();


    // Process outgoing transfers (from library to CAN driver)
    int tx_attempts = 0;  // Limit TX attempts to avoid getting stuck
    for (const CanardCANFrame* txf = NULL; tx_attempts < 5 && (txf = canardPeekTxQueue(ins)) != NULL; tx_attempts++)
    {
        // Check if we've exceeded the timeout
        if (timeout_ms > 0 && HAL_GetTick() - start_time > timeout_ms) {
            break;
        }

        // Feed watchdog during processing
        IWDG->KR = 0xAAAA;

        // Process an outbound CAN frame using the STM32 HAL
        CAN_TxHeaderTypeDef tx_header;
        uint32_t tx_mailbox;

        tx_header.StdId = 0;
        tx_header.ExtId = txf->id & CANARD_CAN_EXT_ID_MASK;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = txf->data_len;

        // Print detailed information about the CAN message being sent
        uint16_t data_type_id = (txf->id >> 8) & 0xFFFF;

        // Short timeout attempt to send message
        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t*)txf->data, &tx_mailbox);

        if (status == HAL_OK) {
            // Remove from queue only if sent successfully
            canardPopTxQueue(ins);
        } else {
            // Break out if TX mailboxes are full
            break;
        }
    }

    // Feed watchdog again
    IWDG->KR = 0xAAAA;

    // Check for received CAN frames
    {
        // Check if there are pending messages
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
        {
            // Process the pending message
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        }
    }

    // Feed watchdog one more time
    IWDG->KR = 0xAAAA;
}

// Change this from static to non-static to match declaration
void send_NodeStatus(void)
{
    // Prepare the message
    uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    // Set the status fields
    uint32_t uptime_sec = HAL_GetTick() / 1000;
    memcpy(&buffer[0], &uptime_sec, 4);

    // Health: 0 = NOMINAL, no health alerts
    buffer[4] = UAVCAN_NODE_HEALTH_OK;

    // Mode: 0 = OPERATIONAL, we're running fine
    buffer[5] = UAVCAN_NODE_MODE_OPERATIONAL;

    // Sub-mode is unused (0)
    buffer[6] = 0;

    // Vendor-specific status code - use to indicate EPOS4 status
    buffer[7] = 0;  // Default to 0 for normal operation

    // Send as broadcast with minimal output
    static uint8_t transfer_id = 0;
    int result = canardBroadcast(&canard,
                                UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                                UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                                &transfer_id,
                                CANARD_TRANSFER_PRIORITY_MEDIUM,
                                buffer,
                                UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    // Only log heartbeat occasionally
    static uint32_t heartbeat_count = 0;
    

    // Add delay after sending heartbeat
    HAL_Delay(1);  // Small delay to allow transmission
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
                                  uint64_t* out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id)
{
    // Debug print for all received transfers


    // Accept messages from our own node ID
    if (source_node_id == NODE_ID) {
        return true;
    }

    // Skip messages with ID 0x4e27 (ArduPilot GNSS status) as they cause -17 errors
    if (data_type_id == ARDUPILOT_GNSS_STATUS_ID) {
        // Return false to completely skip processing this message
        return false;
    }

    // Accept GetNodeInfo request/response
    if (transfer_type == CanardTransferTypeRequest &&
        data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID) {
        printf("  GetNodeInfo request detected from node %d (data_type_id=0x%04x)\r\n",
               source_node_id, data_type_id);

        // Handle broadcast address specially (commonly used by Mission Planner)
        if (source_node_id == 127) {
            printf("  Detected GetNodeInfo from broadcast address (ID 127) - likely Mission Planner discovery\r\n");
        }

        // Use the correct signature value defined in the header
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;

        // Print the actual value to verify it's not zero
        printf("  Using GetNodeInfo signature: 0x%llx\r\n",
               (unsigned long long)*out_data_type_signature);

        printf("  ACCEPTING GetNodeInfo transfer\r\n");
        return true;
    }

    // Accept parameter get/set requests
    if (transfer_type == CanardTransferTypeRequest &&
        data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) {
        printf("  Parameter get/set request from node %d (data_type_id=0x%04x)\r\n",
               source_node_id, data_type_id);
        *out_data_type_signature = UAVCAN_PARAM_GETSET_SIGNATURE;
        printf("  ACCEPTING parameter get/set transfer\r\n");
        return true;
    }

    // Accept NodeStatus messages from any node for monitoring
    if (transfer_type == CanardTransferTypeMessage &&
        data_type_id == UAVCAN_PROTOCOL_NODE_STATUS_MESSAGE_ID) {
        *out_data_type_signature = UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE;
        return true;
    }
    
    // Accept actuator array command messages with correct signature
    if (transfer_type == CanardTransferTypeMessage && data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
        return true;
    }
    
    return false;
}
void readUniqueID(uint8_t* out_uid)
{
    for (uint8_t i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++)
    {
        out_uid[i] = i;
    }
}
// Handle GetNodeInfo service request
void handle_get_node_info(CanardInstance* ins, CanardRxTransfer* transfer)
{
    printf("Processing GetNodeInfo request from node %d\r\n", transfer->source_node_id);

     uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
        memset(buffer,0,UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
        uint16_t len = makeNodeInfoMessage(buffer);
        int result = canardRequestOrRespond(ins,
                                            transfer->source_node_id,
                                            UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                            UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                            &transfer->transfer_id,
                                            transfer->priority,
                                            CanardResponse,
                                            &buffer[0],
                                            (uint16_t)len);
}
uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);

    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;
    buffer[9] = 1;                          // Optional field flags, VCS commit is set
    uint32_t u32 = GIT_HASH;
    canardEncodeScalar(buffer, 80, 32, &u32);

    readUniqueID(&buffer[24]);
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);
    return 41 + name_len ;
}
void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    uint32_t uptime_sec = (HAL_GetTick() / 1000);
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Feed the watchdog during message processing to prevent resets
    IWDG->KR = 0xAAAA;
    


    // Handle GetNodeInfo request
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID))
    {


        // Special handling for broadcast requests (typically from Mission Planner)
        if (transfer->source_node_id == 127) {
            printf("INFO: Received broadcast GetNodeInfo request - normal for Mission Planner\r\n");
            printf("INFO: Will respond with node info (node_id=%d)\r\n", NODE_ID);
        }

        handle_get_node_info(ins, transfer);
    }
    // Handle GetNodeInfo response
    else if ((transfer->transfer_type == CanardTransferTypeResponse) &&
             (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID))
    {
        
        // Parse the response if we have enough data
        if (transfer->payload_len >= 7)  // Minimum size for node status
        {
            uint32_t uptime_sec = 0;
            uint8_t health = 0;
            uint8_t mode = 0;
            uint8_t sub_mode = 0;

            canardDecodeScalar(transfer, 0, 32, false, &uptime_sec);
            canardDecodeScalar(transfer, 32, 8, false, &health);
            canardDecodeScalar(transfer, 40, 8, false, &mode);
            canardDecodeScalar(transfer, 48, 8, false, &sub_mode);

            printf("  Node Status:\r\n");
            printf("    Uptime: %lu seconds\r\n", uptime_sec);
            printf("    Health: %u\r\n", health);
            printf("    Mode: %u\r\n", mode);
            printf("    Sub-mode: %u\r\n", sub_mode);

            // Try to parse version info if available
            if (transfer->payload_len >= 17)  // +10 bytes for version info
            {
                uint8_t protocol_version = 0;
                uint8_t hw_version_major = 0;
                uint8_t hw_version_minor = 0;
                uint8_t sw_version_major = 0;
                uint8_t sw_version_minor = 0;
                uint32_t sw_vcs_commit = 0;

                canardDecodeScalar(transfer, 56, 8, false, &protocol_version);
                canardDecodeScalar(transfer, 64, 8, false, &hw_version_major);
                canardDecodeScalar(transfer, 72, 8, false, &hw_version_minor);
                canardDecodeScalar(transfer, 80, 8, false, &sw_version_major);
                canardDecodeScalar(transfer, 88, 8, false, &sw_version_minor);
                canardDecodeScalar(transfer, 96, 32, false, &sw_vcs_commit);


            }
        }
        else
        {

        }
    }

    // Handle NodeStatus messages
    else if ((transfer->transfer_type == CanardTransferTypeMessage) &&
             (transfer->data_type_id == UAVCAN_PROTOCOL_NODE_STATUS_MESSAGE_ID))
    {
        // Parse the received NodeStatus message
        // According to UAVCAN protocol, a NodeStatus message has the following structure:
        // - Uptime (32 bits)
        // - Health (8 bits)
        // - Mode (8 bits)
        // - Sub-mode (8 bits)
        // - Vendor-specific status code (16 bits)
        if (transfer->payload_len >= 7)
        {
            uint32_t uptime = 0;
            uint8_t health = 0;
            uint8_t mode = 0;
            uint8_t sub_mode = 0;
            uint16_t vendor_status = 0;

            // Extract data using canardDecodeScalar
            canardDecodeScalar(transfer, 0, 32, false, &uptime);
            canardDecodeScalar(transfer, 32, 8, false, &health);
            canardDecodeScalar(transfer, 40, 8, false, &mode);
            canardDecodeScalar(transfer, 48, 8, false, &sub_mode);

            if (transfer->payload_len >= 9)
            {
                canardDecodeScalar(transfer, 56, 16, false, &vendor_status);
            }

            // Interpret the health field
            const char* health_str = "UNKNOWN";
            switch (health)
            {
                case 0: health_str = "NOMINAL"; break;
                case 1: health_str = "WARNING"; break;
                case 2: health_str = "ERROR"; break;
                case 3: health_str = "CRITICAL"; break;
            }

            // Interpret the mode field
            const char* mode_str = "UNKNOWN";
            switch (mode)
            {
                case 0: mode_str = "OPERATIONAL"; break;
                case 1: mode_str = "INITIALIZATION"; break;
                case 2: mode_str = "MAINTENANCE"; break;
                case 3: mode_str = "SOFTWARE_UPDATE"; break;
                case 7: mode_str = "OFFLINE"; break;
            }


        }
        else
        {
            printf("WARNING: Received a truncated NodeStatus message from node %d\r\n",
                   transfer->source_node_id);
        }
    }

    // Handle custom message ID 1010 (UAVCAN_LOCAL_NODE_STATUS)
    else if ((transfer->transfer_type == CanardTransferTypeMessage) &&
             (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID))
    {

        // Process the actuator command message
        handleActuatorArrayCommand(ins, transfer);
    }

    // Handle parameter get/set requests
    else if ((transfer->transfer_type == CanardTransferTypeRequest) &&
             (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID))
    {
        handle_param_getset(ins, transfer);
    }

    // Handle ArduPilot GNSS status message - Fix for message type 0x4e27 (20007)
    else if ((transfer->transfer_type == CanardTransferTypeMessage) &&
             (transfer->data_type_id == ARDUPILOT_GNSS_STATUS_ID))
    {
        // Skip detailed processing of this message type to avoid issues
        // Just acknowledge receipt and don't try to decode it
        printf("Received ArduPilot GNSS status from node %d (skipping detailed processing)\r\n",
               transfer->source_node_id);
    }


}

/**
 * Tasks to run at 1Hz (and other low frequencies)
 */
void process1HzTasks(CanardInstance* ins)
{
    static uint64_t last_1hz_task_ms = 0;
    static uint64_t last_actuator_status_ms = 0;
    static uint64_t last_position_read_ms = 0;
    static uint64_t last_param_broadcast_ms = 0;
    static uint8_t param_broadcast_counter = 0;

    // Get current time in milliseconds
    uint64_t current_ms = HAL_GetTick();


    // 1Hz tasks
    if (current_ms - last_1hz_task_ms >= 1000) {
        // Send node status message
        send_NodeStatus();
        


            // Create parameter response buffer
            uint8_t buffer[UAVCAN_PROTOCOL_PARAM_VALUE_SIZE] = {0};
            int offset = 0;
            param_t* p = &parameters[param_broadcast_counter];

            // Encode based on type
            switch (p->type) {
                case PARAM_TYPE_INTEGER:
                    buffer[offset++] = PARAM_TYPE_INTEGER;  // Integer type tag
                    int64_t int_val = (int64_t)p->value.i;
                    memcpy(&buffer[offset], &int_val, sizeof(int64_t));
                    offset += sizeof(int64_t);
                    int64_t default_val = (int64_t)p->defval;
                    memcpy(&buffer[offset], &default_val, sizeof(int64_t));
                    offset += sizeof(int64_t);
                    int64_t min_val = (int64_t)p->min;
                    memcpy(&buffer[offset], &min_val, sizeof(int64_t));
                    offset += sizeof(int64_t);
                    int64_t max_val = (int64_t)p->max;
                    memcpy(&buffer[offset], &max_val, sizeof(int64_t));
                    offset += sizeof(int64_t);
                    break;

                case PARAM_TYPE_REAL:
                    buffer[offset++] = PARAM_TYPE_REAL;  // Real type tag
                    float real_val = p->value.f;
                    memcpy(&buffer[offset], &real_val, sizeof(float));
                    offset += sizeof(float);
                    float def_val_f = (float)p->defval;
                    memcpy(&buffer[offset], &def_val_f, sizeof(float));
                    offset += sizeof(float);
                    float min_val_f = (float)p->min;
                    memcpy(&buffer[offset], &min_val_f, sizeof(float));
                    offset += sizeof(float);
                    float max_val_f = (float)p->max;
                    memcpy(&buffer[offset], &max_val_f, sizeof(float));
                    offset += sizeof(float);
                    break;

                case PARAM_TYPE_STRING: {
                    buffer[offset++] = PARAM_TYPE_STRING;

                    // First string is empty (default value)
                    buffer[offset++] = 0;

                    // Second string is the actual value
                    uint8_t str_len = strlen(p->value.str);
                    buffer[offset++] = str_len;
                    memcpy(&buffer[offset], p->value.str, str_len);
                    offset += str_len;

                    // Third string is empty (min value - not used for strings)
                    buffer[offset++] = 0;

                    // Fourth string is empty (max value - not used for strings)
                    buffer[offset++] = 0;
                    break;
                }

                default:
                    buffer[offset++] = PARAM_TYPE_EMPTY;  // Empty type tag
                    break;
            }

            // Add parameter name at the end
            uint8_t name_len = strlen(p->name);
            buffer[offset++] = name_len;
            memcpy(&buffer[offset], p->name, name_len);
            offset += name_len;

            // Add parameter index
            buffer[offset++] = param_broadcast_counter & 0xFF;
            buffer[offset++] = (param_broadcast_counter >> 8) & 0xFF;

            // Send parameter with high priority
            static uint8_t transfer_id = 0;
            canardRequestOrRespond(&canard,
                                  CANARD_BROADCAST_NODE_ID,
                                  UAVCAN_PARAM_GETSET_SIGNATURE,
                                  UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                                  &transfer_id,
                                  CANARD_TRANSFER_PRIORITY_HIGH,
                                  CanardResponse,
                                  buffer,
                                  offset);

            // Process TX queue to ensure it's sent
            processTxRxOnce(&canard, 0);

            // Move to next parameter
            param_broadcast_counter++;

            // Reset counter if we've gone through all parameters
            if (param_broadcast_counter >= MAX_PARAMETERS) {
                param_broadcast_counter = 0;
            }
        }

        last_1hz_task_ms = current_ms;


    // 10Hz tasks for actuator status - Only send when requested, not automatically
    /*
    if (current_ms - last_actuator_status_ms >= 100) {
        // Send actuator status message
        send_actuator_status();
        last_actuator_status_ms = current_ms;
    }
    */

    // Periodically broadcast parameters every 5 seconds to help with discovery
    if (current_ms - last_param_broadcast_ms >= 5000) {
        // Full parameter broadcast every 5 seconds
        broadcast_all_parameters();
        last_param_broadcast_ms = current_ms;
    }
}



void epos4_send_sdo_read(uint16_t index, uint8_t subindex)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0};

    // Prepare SDO read message
    tx_data[0] = 0x40;  // Command to read
    tx_data[1] = (uint8_t)(index & 0xFF);         // Index LSB
    tx_data[2] = (uint8_t)((index >> 8) & 0xFF);  // Index MSB
    tx_data[3] = subindex;
    // Bytes 4-7 are reserved and should be 0

    tx_header.StdId = EPOS4_SDO_WRITE + EPOS4_NODE_ID;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;  // Standard ID, not extended
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;  // Always 8 bytes for SDO

    // Add delay before sending
        HAL_Delay(1);

    // Send the message
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    // Add delay after sending
    HAL_Delay(1);

    // Blink orange LED (LD3) to indicate EPOS4 communication
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void epos4_set_operation_mode(uint8_t mode)
{
    // Set mode of operation (8-bit value)
    epos4_send_sdo_write(EPOS4_MODE_OF_OPERATION_INDEX, 0x00, mode, 1);

    // Wait a bit for the command to process
}

void epos4_set_target_velocity(int32_t target_velocity)
{
    // Set target velocity (32-bit value)
    epos4_send_sdo_write(EPOS4_TARGET_VELOCITY_INDEX, 0x00, (uint32_t)target_velocity, 4);
}

void epos4_set_target_position(int32_t target_position)
{
    // Feed watchdog before sending position command
    IWDG->KR = 0xAAAA;

    printf("Sending position: %ld to EPOS4\r\n", (long)target_position);

    // STEP 1: First send SDO write to set target position (0x607A)
    CAN_TxHeaderTypeDef TxHeader1;
    uint8_t TxData1[8] = {0};
    uint32_t TxMailbox1;

    // Set up CAN message header - OPTIMIZED FOR SPEED
    TxHeader1.StdId = 0x600 + EPOS4_NODE_ID;  // SDO_RX_ID + nodeID
    TxHeader1.ExtId = 0;
    TxHeader1.RTR = CAN_RTR_DATA;
    TxHeader1.IDE = CAN_ID_STD;
    TxHeader1.DLC = 8;  // Always 8 bytes for SDO
    TxHeader1.TransmitGlobalTime = DISABLE;

    // Command byte: 0x23 = write 4 bytes (expedited transfer)
    TxData1[0] = 0x23;

    // Object dictionary index (0x607A = target position)
    TxData1[1] = 0x7A;  // Index low byte
    TxData1[2] = 0x60;  // Index high byte

    // Subindex (0x00)
    TxData1[3] = 0x00;

    // Data (4 bytes, little-endian)
    TxData1[4] = (uint8_t)(target_position & 0xFF);
    TxData1[5] = (uint8_t)((target_position >> 8) & 0xFF);
    TxData1[6] = (uint8_t)((target_position >> 16) & 0xFF);
    TxData1[7] = (uint8_t)((target_position >> 24) & 0xFF);
    // Reset watchdog before sending
    IWDG->KR = 0xAAAA;

    // Send position command
    HAL_StatusTypeDef status1 = HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1);

    // Reduced timeout to minimize latency - max 10ms timeout (down from 50ms)
    uint32_t start_time = HAL_GetTick();
    uint32_t timeout_ms = 10;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3 && HAL_GetTick() - start_time < timeout_ms) {
        // Feed watchdog while waiting
        IWDG->KR = 0xAAAA;
    }
    
    // Process any pending CAN messages
    processTxRxOnce(&canard, 1);

    // Reset watchdog before next command
    IWDG->KR = 0xAAAA;

    // STEP 2: Send control word with bit 4 set to trigger the motion (0x001F = 0x000F with bit 4 set)
    // Reduced delay between commands for faster response

    CAN_TxHeaderTypeDef TxHeader2;
    uint8_t TxData2[8] = {0};
    uint32_t TxMailbox2;

    // Set up CAN message header for control word - HIGH PRIORITY
    TxHeader2.StdId = 0x600 + EPOS4_NODE_ID;  // SDO_RX_ID + nodeID
    TxHeader2.ExtId = 0;
    TxHeader2.RTR = CAN_RTR_DATA;
    TxHeader2.IDE = CAN_ID_STD;
    TxHeader2.DLC = 8;  // Always 8 bytes for SDO
    TxHeader2.TransmitGlobalTime = DISABLE;

    // Command byte: 0x2B = write 2 bytes (expedited transfer)
    TxData2[0] = 0x2B;

    // Object dictionary index (0x6040 = control word)
    TxData2[1] = 0x40;  // Index low byte
    TxData2[2] = 0x60;  // Index high byte

    // Subindex (0x00)
    TxData2[3] = 0x00;

    // Data - Control word 0x001F (0x000F with bit 4 set to start motion)
    TxData2[4] = 0x1F;  // 0x1F = 00011111b (bits 0-4 set)
    TxData2[5] = 0x00;
    TxData2[6] = 0x00;
    TxData2[7] = 0x00;

    // Before sending, abort any pending transmissions to ensure mailbox is free
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

    // Send the CAN message to trigger motion with high priority
    HAL_StatusTypeDef status2 = HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, TxData2, &TxMailbox2);

    // Toggle Green LED (LD4) to show position command was sent
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

    // Feed watchdog
    IWDG->KR = 0xAAAA;
}

void epos4_enable_motor(void)
{
    printf("ENABLE_MOTOR: Starting sequence\n");

    // Step 1: Reset any faults
    printf("ENABLE_MOTOR: Sending fault reset command\n");
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x0080, 2);
    HAL_Delay(10);

    // Step 2: Shutdown command (prepare for switch on)
    printf("ENABLE_MOTOR: Sending shutdown command\n");
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x0006, 2);
    HAL_Delay(50);
    
    // Process any responses
    processTxRxOnce(&canard, 10);
    
    // Step 3: Switch on (but don't enable operation yet)
    printf("ENABLE_MOTOR: Sending switch on command\n");
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x0007, 2);
    HAL_Delay(50);
    
    // Process any responses
    processTxRxOnce(&canard, 10);
    
    // Step 4: Enable operation
    printf("ENABLE_MOTOR: Sending enable operation command\n");
    epos4_send_sdo_write(EPOS4_CONTROLWORD_INDEX, 0x00, 0x000F, 2);
    HAL_Delay(50);
    
    // Process any responses
    processTxRxOnce(&canard, 10);

    // Set following error window to 2000 increments
    printf("CHANGE_ERROR_WINDOW: Setting following error window to 2000 increments\n");
    epos4_send_sdo_write(0x6065, 0x00, 1000000, 4);  // 4 bytes for Uint32

    // Process any responses
    processTxRxOnce(&canard, 10);
    
    // Step 5: Check status
    printf("ENABLE_MOTOR: Checking motor status\n");
    check_epos4_status();
    
    // Visual indication
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    
    printf("ENABLE_MOTOR: Sequence completed\n");
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

    // Add delay before sending
    HAL_Delay(1);

    // Send the message
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);

    // Add delay after sending
    HAL_Delay(1);

    // Blink orange LED (LD3) to indicate EPOS4 communication
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void handle_ardupilot_command(int16_t command_value)
{
    // Log the command for debugging
    printf("Command received: %d (0x%04X)\n", command_value, (unsigned int)command_value);

    // Feed watchdog
    IWDG->KR = 0xAAAA;

    // Determine target position based on command value
    int32_t target_position;

    // Special handling for known command values
    if (command_value == 15360) { // Special value for max position
        printf("Special command 15360 detected - using maxPosition\n");
        target_position = maxPosition;
    }
    else if (command_value >= 45000) { // Special value for min position
        printf("Special command 45000+ detected - using minPosition\n");
        target_position = minPosition;
    }
    else {
        // For all other values, use linear interpolation
        // Map command range -32768 to 32767 to motor position range
        if (command_value >= 0 && command_value <= 32767) {
            // Positive commands: map 0→32767 to midPosition→minPosition
            float normalized = (float)command_value / 32767.0f;
            target_position = midPosition + normalized * (minPosition - midPosition);
        } 
        else {
            // Negative commands: map -32768→0 to maxPosition→midPosition
            float normalized = (float)(command_value + 32768) / 32768.0f;
            target_position = maxPosition + normalized * (midPosition - maxPosition);
        }
    }
    
    // Apply safety limits
    if (target_position > maxPosition) target_position = maxPosition;
    if (target_position < minPosition) target_position = minPosition;
    
    // Send command directly to motor
    printf("Sending position command: %ld\n", (long)target_position);
    force_motor_movement(target_position);
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

// Add printf function
//int printf(const char* format, ...) {
  //  va_list args;
    //va_start(args, format);
    //char buffer[128];
    //vsnprintf(buffer, sizeof(buffer), format, args);
    //va_end(args);
    //for (char* p = buffer; *p != '\0'; p++) {
      //  ITM_SendChar(*p);
    //}
//}

// Parameter definitions
#define MAX_PARAMETERS 100
#define PARAM_NAME_MAX_LENGTH 16



// Parameter count helper macro
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

// Save settings function placeholder
void save_settings(void) {
    // In a real implementation, this would save parameters to flash memory
    // For now, just log that settings would be saved
    printf("Settings would be saved to flash memory\r\n");
    
    // Example implementation would use HAL_FLASH functions:
    // 1. Unlock flash
    // 2. Erase sector
    // 3. Program parameters
    // 4. Lock flash
}

// Parameter get/set handler for UAVCAN
void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Feed the watchdog during parameter processing to prevent resets
    IWDG->KR = 0xAAAA;

    uint16_t index = 0xFFFF;
    uint8_t tag = 0;
    int offset = 0;
    int64_t val = 0;
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_VALUE_SIZE] = {0};
    param_t* p = NULL;
    bool param_value_changed = false;  // Track if parameter value changed

    // Clear buffer
    memset(buffer, 0, sizeof(buffer));
    
    // Decode the request
    canardDecodeScalar(transfer, offset, 13, false, &index);
    offset += 13;
    canardDecodeScalar(transfer, offset, 3, false, &tag);
    offset += 3;

    if (tag == 1) {
        canardDecodeScalar(transfer, offset, 64, false, &val);
        offset += 64;
    }

    // Check if request includes a parameter name (with safety checks)
    uint16_t n = 0;
    if (transfer->payload_len > (offset / 8)) {
        n = (transfer->payload_len - (offset / 8));
    }
    
    uint8_t name[PARAM_NAME_MAX_LENGTH + 1] = {0};
    if (n > 0) {
        uint8_t name_len = 0;
        if (canardDecodeScalar(transfer, offset, 8, false, &name_len)) {
            offset += 8;
            
            // Ensure name_len is within bounds to prevent buffer overflow
            if (name_len > 0 && name_len < PARAM_NAME_MAX_LENGTH) {
                for (int i = 0; i < name_len && i < PARAM_NAME_MAX_LENGTH && ((offset + 8) <= (transfer->payload_len * 8)); i++) {
                    canardDecodeScalar(transfer, offset, 8, false, &name[i]);
                    offset += 8;
                }
                name[name_len] = '\0'; // Ensure null termination
            }
        }
    }

    // Find the parameter
    if (strlen((char const*)name) > 0) {
        p = getParamByName((const char*)name);
        printf("Parameter request by name: %s\r\n", name);
    } else {
        p = getParamByIndex(index);
        printf("Parameter request by index: %d\r\n", index);
    }
    
    // If this is a set request and we found the parameter, update its value
    if ((p) && (tag == 1)) {
        // Store old value to detect changes
        int64_t old_int_val = 0;
        float old_float_val = 0.0f;
        bool old_bool_val = false;

        // Save old value based on parameter type
        switch (p->type) {
            case PARAM_TYPE_INTEGER:
                old_int_val = p->value.i;
                break;
            case PARAM_TYPE_REAL:
                old_float_val = p->value.f;
                break;
            case PARAM_TYPE_BOOLEAN:
                old_bool_val = p->value.b;
                break;
            default:
                break;
        }

        // Update based on parameter type
        switch (p->type) {
            case PARAM_TYPE_INTEGER:
                p->value.i = (int32_t)val;
                p->val = (int32_t)val;
                param_value_changed = (old_int_val != p->value.i);
                break;
            case PARAM_TYPE_REAL:
                p->value.f = *((float*)&val); // Cast int64 bits to float
                p->val = (int32_t)val;
                param_value_changed = (fabsf(old_float_val - p->value.f) > 0.0001f);
                break;
            case PARAM_TYPE_BOOLEAN:
                p->value.b = (val > 0);
                p->val = (val > 0) ? 1 : 0;
                param_value_changed = (old_bool_val != p->value.b);
                break;
            case PARAM_TYPE_STRING: {
                // Keep this simple for now
                strncpy(p->value.str, "param", PARAM_NAME_MAX_LENGTH-1);
                break;
            }
            default:
                break;
        }
        printf("Setting parameter value to: %ld\r\n", (long)val);
        
        // If parameter value changed, broadcast the updated parameter immediately
        if (param_value_changed) {
            printf("Parameter %s changed - broadcasting update\r\n", p->name);

            // Broadcast only the changed parameter
            broadcast_parameter(p);

            // Optionally save settings to non-volatile memory
            // save_settings();
        }
    }
    
    // For simplicity, just create a minimal response that won't cause issues
    offset = 0;

    if (p != NULL) {
        // Just encode the parameter value as an integer (simplest approach)
        buffer[offset++] = PARAM_TYPE_INTEGER;  // Integer type tag
        int64_t resp_val = p->val;
        memcpy(&buffer[offset], &resp_val, sizeof(int64_t));
        offset += sizeof(int64_t);

        // Default value
        memcpy(&buffer[offset], &resp_val, sizeof(int64_t));
        offset += sizeof(int64_t);

        // Min value
        int64_t min_val = minPosition;
        memcpy(&buffer[offset], &min_val, sizeof(int64_t));
        offset += sizeof(int64_t);

        // Max value
        int64_t max_val = maxPosition;
        memcpy(&buffer[offset], &max_val, sizeof(int64_t));
        offset += sizeof(int64_t);
        
        // Parameter name
        uint8_t name_len = strlen(p->name);
        buffer[offset++] = name_len;
        memcpy(&buffer[offset], p->name, name_len);
        offset += name_len;
    } else {
        // Parameter not found - send empty response
        buffer[offset++] = PARAM_TYPE_EMPTY;
        buffer[offset++] = 0; // Empty name
    }

    // Ensure we don't exceed buffer size
    if (offset > UAVCAN_PROTOCOL_PARAM_VALUE_SIZE) {
        offset = UAVCAN_PROTOCOL_PARAM_VALUE_SIZE;
    }

    // Send response - simple direct approach
    static uint8_t param_transfer_id = 0;
    canardRequestOrRespond(ins,
                          transfer->source_node_id,
                          UAVCAN_PARAM_GETSET_SIGNATURE,
                          UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                          &param_transfer_id,
                          CANARD_TRANSFER_PRIORITY_HIGH,
                          CanardResponse,
                          buffer,
                          offset);

    // Feed watchdog again
    IWDG->KR = 0xAAAA;
        
    printf("Parameter response sent\r\n");
}

void send_get_node_info_request(uint8_t target_node_id)
{
    static uint8_t dummy_payload[1] = {0};  // Empty payload
    static uint8_t transfer_id = 0;

    int result = canardRequestOrRespond(&canard,
                                        target_node_id,                        // Destination Node ID
                                        UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE, // Signature of GetNodeInfo
                                        UAVCAN_PROTOCOL_GETNODEINFO_ID,         // Data Type ID = 1
                                        &transfer_id,
                                        CANARD_TRANSFER_PRIORITY_LOW,           // or MEDIUM
                                        CanardRequest,                          // Transfer type REQUEST
                                        dummy_payload,
                                        0);  // Payload size 0 (empty)

    printf("Sent GetNodeInfo request to node %d, result: %d\r\n", target_node_id, result);
}

// Add this function to get microsecond timestamps
uint64_t micros64(void) {
    static uint32_t last_tick = 0;
    static uint64_t overflow_count = 0;

    uint32_t tick = HAL_GetTick();

    // Handle overflow
    if (tick < last_tick) {
        overflow_count += 1;
    }
    last_tick = tick;

    // Return microseconds (tick is in milliseconds, so multiply by 1000)
    return (overflow_count << 32) | (tick * 1000ULL);
}

/**
 * Translate EPOS4 raw position to normalized position (-1.0 to 1.0)
 * @param position Raw EPOS4 position value
 * @return Normalized position (-1.0 to 1.0)
 */
float translateEPOS4Position(int32_t position)
{
    // Clamp position to the defined range
    if (position > maxPosition) position = maxPosition;
    if (position < minPosition) position = minPosition;

    // Convert to normalized value (-1.0 to 1.0)
    // Our physical range is:
    // maxPosition (100000) = command -10000 = normalized -1.0
    // midPosition (0) = command 0 = normalized 0.0
    // minPosition (-100000) = command 10000 = normalized 1.0
    float normalized = (float)(midPosition - position) / (float)(midPosition - minPosition);

    // Clamp to valid range
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    return normalized;
}

/**
 * Get the current EPOS4 position as a normalized value between -1.0 and 1.0
 * using the globally defined min/mid/max positions
 * @return Normalized position (-1.0 to 1.0)
 */
float getEPOS4NormalizedPosition(void)
{
    // Get the current position from the global variable
    int32_t pos = current_position;

    // Convert to normalized value
    float normalized = 0.0f;

    if (pos >= midPosition) {
        // Position is in upper half (0 to -140000)
        normalized = (float)(pos - midPosition) / (float)(maxPosition - midPosition);
    } else {
        // Position is in lower half (-140000 to -280000)
        normalized = (float)(pos - midPosition) / (float)(midPosition - minPosition);
    }

    return normalized;
}

/**
 * Convert normalized position (-1.0 to 1.0) to EPOS4 encoder position
 * @param normalized_position Normalized position between -1.0 and 1.0
 * @return Raw EPOS4 encoder position
 */
int32_t getNormalizedToEPOS4Position(float normalized_position)
{
    // Clamp normalized position
    if (normalized_position < -1.0f) normalized_position = -1.0f;
    if (normalized_position > 1.0f) normalized_position = 1.0f;

    // Convert to encoder position
    // Our physical range is:
    // normalized -1.0 = maxPosition (100000)
    // normalized 0.0 = midPosition (0)
    // normalized 1.0 = minPosition (-100000)
    int32_t position = midPosition - (int32_t)(normalized_position * (midPosition - minPosition));

    // Clamp to valid range
    if (position > maxPosition) position = maxPosition;
    if (position < minPosition) position = minPosition;

    return position;
}

/**
 * Send actuator status message via UAVCAN
 * This provides feedback about the EPOS4 position
 */
void send_actuator_status(void)
{
    // Feed the watchdog to prevent reset during status send
    IWDG->KR = 0xAAAA;

    static uint8_t transfer_id = 0;

    // Prepare buffer for the message
    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
    memset(buffer, 0, UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE);

    // Get current timestamp and position
    uint64_t timestamp_usec = micros64();
    float normalized_position = getEPOS4NormalizedPosition();

    // Encode the timestamp (64 bits)
    canardEncodeScalar(buffer, 0, 64, &timestamp_usec);

    // Encode actuator ID (8 bits)
    uint8_t actuator_id = NODE_ID;  // Use node ID as the actuator ID for better identification
    canardEncodeScalar(buffer, 64, 8, &actuator_id);

    // Encode normalized position (-1.0 to 1.0, 32 bits float)
    canardEncodeScalar(buffer, 72, 32, &normalized_position);

    // Encode force (not used, set to 0)
    float force = 0.0f;
    canardEncodeScalar(buffer, 104, 32, &force);

    // Use current velocity (a fraction of max velocity based on position)
    float speed = 0.0f;
    canardEncodeScalar(buffer, 136, 32, &speed);

    // Encode power rating (not used, set to 0)
    float power_rating = 0.0f;
    canardEncodeScalar(buffer, 168, 32, &power_rating);

    // Minimal logging - just log that we're sending the status
    // Log what we're sending
    static uint32_t status_count = 0;

    // Directly construct a CAN message for a more direct approach - less likely to hang
    uint32_t message_id = (transfer_id++ & 0x1F) |
                         (UAVCAN_EQUIPMENT_ACTUATOR_STATUS_DATA_TYPE_ID << 8) |
                         ((CANARD_TRANSFER_PRIORITY_MEDIUM & 0x1F) << 24);

    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = 0;
    tx_header.ExtId = message_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;  // First 8 bytes

    // Send directly with a few retries if needed
    HAL_StatusTypeDef status;
    for (int i = 0; i < 3; i++) {  // 3 attempts maximum
        status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, buffer, &tx_mailbox);
        if (status == HAL_OK) {
            break;
        }
        HAL_Delay(1);  // Short delay between attempts
    }
    
    // IWDG refresh again after sending
    IWDG->KR = 0xAAAA;
}

void epos4_read_position(uint16_t index, uint8_t subindex)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    // Setup CAN message header
    tx_header.StdId = 0x600 + EPOS4_NODE_ID;  // SDO client -> server
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // Setup SDO read command
    tx_data[0] = 0x40;  // Read command
    tx_data[1] = index & 0xFF;
    tx_data[2] = (index >> 8) & 0xFF;
    tx_data[3] = subindex;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    // Send CAN message
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

// Add these defines at the top of the file
#define UAVCAN_PROTOCOL_PARAM_GETSET_ID 11

// Parameter handling function declarations
static inline param_t * getParamByIndex(uint16_t index);
static inline param_t * getParamByName(const char * name);

// Parameter encoding for UAVCAN
uint16_t encodeParamCanard(param_t * p, uint8_t * buffer)
{
    uint8_t n     = 0;
    int offset    = 0;
    uint8_t tag   = 1;
    if(p==NULL)
    {
        tag = 0;
        canardEncodeScalar(buffer, offset, 5, &n);
        offset += 5;
        canardEncodeScalar(buffer, offset,3, &tag);
        offset += 3;

        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset,2, &tag);
        offset += 2;

        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset, 2, &tag);
        offset += 2;
        buffer[offset / 8] = 0;
        return ( offset / 8 + 1 );
    }
    canardEncodeScalar(buffer, offset, 5,&n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->val);
    offset += 64;

    canardEncodeScalar(buffer, offset, 5, &n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->defval);
    offset += 64;

    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset, 2, &tag);
    offset += 2;
    canardEncodeScalar(buffer, offset, 64, &p->max);
    offset += 64;

    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset,2,&tag);
    offset += 2;
    canardEncodeScalar(buffer, offset,64,&p->min);
    offset += 64;

    memcpy(&buffer[offset / 8], p->name, strlen((char const*)p->name));
    return  (offset/8 + strlen((char const*)p->name));
}

void rawcmdHandleCanard(CanardRxTransfer* transfer)
{
    uint16_t rc_pwm[6] = {0};

    int offset = 0;
    for (int i = 0; i<6; i++)
    {
        if (canardDecodeScalar(transfer, offset, 14, true, &rc_pwm[i])<14) { break; }
        offset += 14;
    }

    // Log the RC values
    printf("RC values: %d %d %d %d %d %d\r\n",
           rc_pwm[0], rc_pwm[1], rc_pwm[2], rc_pwm[3], rc_pwm[4], rc_pwm[5]);

    // Map RC values to motor control if needed
    if (rc_pwm[0] > 1500) {
        // Example: use first channel for position control
        int32_t position = (rc_pwm[0] - 1500) * 10; // Simple linear mapping
        epos4_set_target_position(position);
    }
}

// ... existing code ...

// ... existing code ...

// Parameter handling function declarations
static inline param_t * getParamByIndex(uint16_t index);
static inline param_t * getParamByName(const char * name);

// ... existing code ...

// Parameter handling function implementations
static inline param_t * getParamByIndex(uint16_t index)
{
    if(index >= ARRAY_SIZE(parameters))
    {
        return NULL;
    }

    return &parameters[index];
}

static inline param_t * getParamByName(const char * name)
{
    for(uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
    {
        if(strcmp(name, parameters[i].name) == 0)
        {
            return &parameters[i];
        }
    }
    return NULL;
}

/**
 * Broadcast all parameters to speed up parameter detection
 */
void broadcast_all_parameters(void)
{

    // First, broadcast a heartbeat to ensure our node is visible
    send_NodeStatus();

    // Small delay before sending parameters
    HAL_Delay(20);

    // Count of valid parameters
    int param_count = 0;

    // Loop through all parameters and broadcast each one
    for (uint16_t i = 0; i < MAX_PARAMETERS; i++) {
        // Skip empty parameters
        if (parameters[i].name[0] == '\0') {
            continue;
        }

        param_count++;

        // Broadcast this parameter with retries for reliability
        for (int retry = 0; retry < 2; retry++) {
            // Use our broadcast_parameter function
            broadcast_parameter(&parameters[i]);

            // Process queue to ensure it's sent
            for (int j = 0; j < 3; j++) {
                processTxRxOnce(&canard, 0);
            }

            // Small delay between retries and parameters
            HAL_Delay(10);
        }

        // Feed watchdog to prevent reset during broadcast
        IWDG->KR = 0xAAAA;

        // Short delay between parameters
        HAL_Delay(20);
    }

    // Mark that we've broadcast all parameters
    all_params_broadcast = true;

    // Final heartbeat at end of parameter broadcast
    send_NodeStatus();

    // Process any pending messages
    for (int i = 0; i < 5; i++) {
        processTxRxOnce(&canard, 0);
        HAL_Delay(2);
    }

}

/**
 * Handle UAVCAN Equipment Actuator ArrayCommand
 * This receives commands from ArduPilot for servo/motor control
 */
void handleActuatorArrayCommand(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Add command latency tracking
    static uint32_t last_command_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (last_command_time > 0) {
        // Only print latency after first command to have a valid measurement
        printf("Command latency: %lu ms\n", current_time - last_command_time);
    }
    last_command_time = current_time;
    // Feed the watchdog during command processing to prevent resets
    IWDG->KR = 0xAAAA;

    // Check for a valid message size - ArduPilot sends custom 4-byte format
    if (transfer->payload_len < 4) {
        printf("ERROR: Message too short for command (4 bytes needed)\n");
        return;
    }

    // Process short messages from ArduPilot (their custom format)
    if (transfer->payload_len == 4) {
        // Extract command value from the message
        uint16_t command_value = 0;
        
        // ArduPilot format: bytes 2-3 contain the command value (little-endian)
        uint8_t lsb = 0, msb = 0;
        canardDecodeScalar(transfer, 16, 8, false, &lsb);  // 3rd byte (16-bit offset)
        canardDecodeScalar(transfer, 24, 8, false, &msb);  // 4th byte (24-bit offset)
        command_value = (uint16_t)lsb | ((uint16_t)msb << 8);
        
        // Store the command value for initial synchronization
        latest_ardupilot_command = command_value;
        ardupilot_command_received = true;
        
        // Print the command value for debugging with more detail for vectored yaw
        if (command_value < 15360 && command_value > 14000) {
            // This is likely a vectored yaw command - print more detail
            printf("VECTORED YAW COMMAND: %u (0x%04X), deviation from center: %d\n", 
                   command_value, command_value, 15360 - command_value);
        } else {
            printf("ACTUATOR COMMAND: %u (0x%04X)\n", command_value, command_value);
        }
        
        // Track command changes to optimize movement
        static uint16_t last_command = 0;
        static uint32_t last_command_time = 0;
        uint32_t current_time = HAL_GetTick();
        
        // Calculate command delta - used to determine if this is a small adjustment
        int16_t command_delta = abs((int16_t)command_value - (int16_t)last_command);
        
        // Check if this is a significantly new command or if enough time has passed
        // Process all commands immediately for vectored yaw
        bool should_process = false;
        
        // Always process vectored yaw commands immediately (near 15360 range)
        if (abs(command_value - 15360) < 1000) {
            should_process = true;
            printf("Processing vectored yaw command immediately\n");
        }
        // For small changes (under 50 units), process immediately with no debounce
        else if (command_delta <= 50) {
            should_process = true;
            printf("Processing small command adjustment (delta=%d)\n", command_delta);
        } 
        // For larger changes, use a very short debounce time of 50ms
        else if (command_delta > 50 && (current_time - last_command_time) > 50) {
            should_process = true;
            printf("Processing regular command (delta=%d)\n", command_delta);
        }
        else if (command_value != last_command) {
            printf("Command change detected but debounce active (delta=%d)\n", command_delta);
        }
        
        if (should_process) {
            // Map the command to a position in our safe range
            int32_t position;
            
            // Special handling for vectored yaw mode (values near 15360)
            if (abs(command_value - 15360) < 1000) {
                // Enhanced mapping for vectored yaw commands in VTOL mode
                // 15360 = center yaw position (should be near minPosition)
                // 14360 = max right yaw
                // 16360 = max left yaw
                
                // Calculate deviation from center (15360)
                int16_t deviation = command_value - 15360;
                

                // Map the deviation to a position range around minPosition
                // For VTOL mode, 15360 corresponds to minPosition (negative numbers)
                int32_t base_position = minPosition;
                
                // Apply the deviation - negative deviation (< 15360) increases position 
                // positive deviation (> 15360) decreases position
                float normalized = (float)deviation / 1000.0f;  // +/-1.0 for +/-1000 deviation
                
                // Calculate position with more smoothing for small changes
                position = base_position - (int32_t)(normalized * 100000);
                
                // Ensure position stays within limits
                if (position < minPosition) position = minPosition;
                if (position > maxPosition) position = maxPosition;
                
                printf("VTOL Yaw command: value=%d, deviation=%d, normalized=%.4f, position=%ld\n", 
                       command_value, deviation, normalized, (long)position);
            }
            // Standard mapping for regular commands
            else if (command_value <= 15000) {
                // Handle minimum bound
                position = minPosition;
                printf("Command at lower bound - using min position: %ld\n", (long)position);
            }
            else if (command_value >= 48000) {
                // Handle maximum bound
                position = maxPosition;
                printf("Command at upper bound - using max position: %ld\n", (long)position);
            }
            else {
                // For values in the normal range
                float normalized = (float)(command_value - 15360) / (float)(48128 - 15360);
                
                // Ensure we're in the valid range [0,1]
                if (normalized < 0.0f) normalized = 0.0f;
                if (normalized > 1.0f) normalized = 1.0f;
                
                // Apply mapping from normalized to physical position
                position = minPosition + (int32_t)(normalized * (maxPosition - minPosition));
                
                printf("Command mapped to position: %ld (normalized: %.2f)\n",
                       (long)position, normalized);
            }
            
            // Optimize motion parameters based on command delta and mode
            if (abs(command_value - 15360) < 1000) {
                // Vectored yaw mode - maximum responsiveness
                profile_velocity = 200000;     // Extreme velocity for yaw movements
                profile_acceleration = 200000; // Extreme acceleration for instant response
                profile_deceleration = 200000; // Matching deceleration
                printf("Using MAXIMUM RESPONSE profile for vectored yaw control\n");
                
                // If it's a very small change, make sure we don't skip it
                if (command_delta < 10) {
                    printf("Small vectored yaw adjustment - forcing movement\n");
                }
            }
            else if (command_delta <= 20) {
                // Very small adjustment - use highest velocity
                profile_velocity = 20000;     // Maximum velocity for smallest moves
                profile_acceleration = 20000; // Higher acceleration for instant response
                profile_deceleration = 20000; // Matching deceleration
                printf("Using max speed profile for tiny adjustment\n");
            }
            else if (command_delta <= 100) {
                // Small to medium adjustment - high performance profile
                profile_velocity = 100000;
                profile_acceleration = 100000;
                profile_deceleration = 100000;
                printf("Using high-speed profile for small adjustment\n");
            }
            else {
                // Large movement - balanced profile
                profile_velocity = 50000;
                profile_acceleration = 10000;
                profile_deceleration = 10000;
                printf("Using balanced profile for larger movement\n");
            }
            
            // Remember this command
            last_command = command_value;
            last_command_time = current_time;
            
            // Set the pending position and flag for processing in main loop
            pending_position = position;
            command_pending = true;
            
            // Also echo the current and target positions for debugging
            printf("POSITION INFO: Current=%ld, Target=%ld, Delta=%ld\n", 
                   (long)current_position, (long)position, 
                   (long)(position - current_position));
        }
    }
    else {
        // For standard format messages (future support)
        printf("WARNING: Unsupported command format (length=%u)\n", 
               (unsigned int)transfer->payload_len);
    }
}


/**
 * Initialize position parameters with sensible defaults for servo control
 */
void initializePositionParameters(void)
{
    // Update parameters if they exist in parameter array
    param_t* min_param = getParamByName("epos4.min_pos");
    if (min_param != NULL) {
        min_param->value.f = -1.0f;  // Minimum normalized position
    }

    param_t* max_param = getParamByName("epos4.max_pos");
    if (max_param != NULL) {
        max_param->value.f = 1.0f;   // Maximum normalized position
    }

    param_t* center_param = getParamByName("epos4.center");
    if (center_param != NULL) {
        center_param->value.f = 0.0f; // Center position
    }
    
    // Update parameters if they exist
    param_t* velocity_param = getParamByName("epos4.velocity");
    if (velocity_param != NULL) {
        velocity_param->value.i = 1000;  // Very conservative velocity
    }
    
    param_t* accel_param = getParamByName("epos4.accel_rate");
    if (accel_param != NULL) {
        accel_param->value.i = 500;  // Very conservative acceleration
    }
    
    param_t* decel_param = getParamByName("epos4.decel_rate");
    if (decel_param != NULL) {
        decel_param->value.i = 500;  // Very conservative deceleration
    }
    
    // Initialize mode-specific parameters with conservative values
    param_t* mode_vel_param = getParamByName("epos4.mode_vel");
    if (mode_vel_param != NULL) {
        mode_vel_param->value.i = 1000; // Conservative for mode changes
    }
    
    // Initialize ArduPilot flight mode parameters
    param_t* fbwa_vel_param = getParamByName("epos4.fbwa_vel");
    if (fbwa_vel_param != NULL) {
        fbwa_vel_param->value.i = 1000; // Conservative for FBWA mode
    }
    
    param_t* qhover_vel_param = getParamByName("epos4.qhover_vel");
    if (qhover_vel_param != NULL) {
        qhover_vel_param->value.i = 1000; // Conservative for QHover mode
    }
    
    // Print initialization values
    printf("Position limits set to: min=%ld, mid=%ld, max=%ld\n",
           (long)minPosition, (long)midPosition, (long)maxPosition);
    printf("Motion profile: velocity=%lu, accel=%lu, decel=%lu\n",
           (unsigned long)profile_velocity, 
           (unsigned long)profile_acceleration,
           (unsigned long)profile_deceleration);
}

// EPOS4 SDO write function
void epos4_send_sdo_write(uint16_t index, uint8_t subindex, uint32_t data, uint32_t data_length)
{
    // Feed watchdog
    IWDG->KR = 0xAAAA;

    printf("Sending SDO write: idx=%04x:%02x, data=%08lx, len=%lu\r\n",
           index, subindex, (unsigned long)data, (unsigned long)data_length);

    // Configure header for SDO message
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;

    tx_header.StdId = 0x600 + EPOS4_NODE_ID;  // SDO client -> server
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // Set command specifier based on data length
    // 0x2F = write 1 byte, 0x2B = write 2 bytes, 0x27 = write 3 bytes, 0x23 = write 4 bytes
    uint8_t command;
    switch (data_length) {
        case 1: command = 0x2F; break;
        case 2: command = 0x2B; break;
        case 3: command = 0x27; break;
        default: command = 0x23; break; // Default to 4-byte write
    }

    // Fill in the SDO message
    tx_data[0] = command;
    tx_data[1] = index & 0xFF;         // Low byte of index
    tx_data[2] = (index >> 8) & 0xFF;  // High byte of index
    tx_data[3] = subindex;             // Subindex

    // Add data in little-endian format
    for (uint8_t i = 0; i < data_length && i < 4; i++) {
        tx_data[4 + i] = (data >> (i * 8)) & 0xFF;
    }

    // Try to send the message with retry logic (up to 3 attempts)
    HAL_StatusTypeDef status = HAL_ERROR;
    for (int retry = 0; retry < 3; retry++) {
        // Wait for a transmit mailbox to be free
        if (!wait_for_tx_complete(50)) {
            printf("TX buffer full, retry %d\r\n", retry + 1);
            HAL_Delay(10 * (retry + 1)); // Progressive delay for retries
            continue;
        }

        // Attempt to send the message
        status = HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
        if (status == HAL_OK) {
            printf("SDO write command sent successfully (attempt %d)\r\n", retry + 1);
            break; // Exit retry loop if successful
        } else {
            printf("Failed to send SDO command, status=%d, retry %d\r\n", status, retry + 1);
            HAL_Delay(10 * (retry + 1)); // Progressive delay for retries
        }
    }

    // Process any pending transmit/receive operations
    processTxRxOnce(&canard, 10);
}

/**
 * Broadcast a single parameter to inform other nodes of its value
 * @param p Pointer to the parameter to broadcast
 */
void broadcast_parameter(param_t* p)
{
    if (p == NULL) {
        printf("ERROR: Cannot broadcast NULL parameter\r\n");
        return;
    }

    // Create parameter response buffer
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_VALUE_SIZE] = {0};
    int offset = 0;

    // Encode based on type
    switch (p->type) {
        case PARAM_TYPE_INTEGER:
            buffer[offset++] = PARAM_TYPE_INTEGER;  // Integer type tag
            int64_t int_val = (int64_t)p->value.i;
            memcpy(&buffer[offset], &int_val, sizeof(int64_t));
            offset += sizeof(int64_t);
            int64_t default_val = (int64_t)p->defval;
            memcpy(&buffer[offset], &default_val, sizeof(int64_t));
            offset += sizeof(int64_t);
            int64_t min_val = (int64_t)p->min;
            memcpy(&buffer[offset], &min_val, sizeof(int64_t));
            offset += sizeof(int64_t);
            int64_t max_val = (int64_t)p->max;
            memcpy(&buffer[offset], &max_val, sizeof(int64_t));
            offset += sizeof(int64_t);
            break;

        case PARAM_TYPE_REAL:
            buffer[offset++] = PARAM_TYPE_REAL;  // Real type tag
            float real_val = p->value.f;
            memcpy(&buffer[offset], &real_val, sizeof(float));
            offset += sizeof(float);
            float def_val_f = (float)p->defval;
            memcpy(&buffer[offset], &def_val_f, sizeof(float));
            offset += sizeof(float);
            float min_val_f = (float)p->min;
            memcpy(&buffer[offset], &min_val_f, sizeof(float));
            offset += sizeof(float);
            float max_val_f = (float)p->max;
            memcpy(&buffer[offset], &max_val_f, sizeof(float));
            offset += sizeof(float);
            break;

        case PARAM_TYPE_BOOLEAN:
            buffer[offset++] = PARAM_TYPE_BOOLEAN;  // Boolean type tag
            int64_t bool_val = p->value.b ? 1 : 0;
            memcpy(&buffer[offset], &bool_val, sizeof(int64_t));
            offset += sizeof(int64_t);
            int64_t bool_default = p->defval ? 1 : 0;
            memcpy(&buffer[offset], &bool_default, sizeof(int64_t));
            offset += sizeof(int64_t);
            int64_t bool_min = 0;
            memcpy(&buffer[offset], &bool_min, sizeof(int64_t));
            offset += sizeof(int64_t);
            int64_t bool_max = 1;
            memcpy(&buffer[offset], &bool_max, sizeof(int64_t));
            offset += sizeof(int64_t);
            break;

        case PARAM_TYPE_STRING: {
            buffer[offset++] = PARAM_TYPE_STRING;

            // First string - default value
            uint8_t def_str_len = strlen(p->value.str);
            buffer[offset++] = def_str_len;
            if (def_str_len > 0) {
                memcpy(&buffer[offset], p->value.str, def_str_len);
                offset += def_str_len;
            }

            // Second string - actual value
            uint8_t str_len = strlen(p->value.str);
            buffer[offset++] = str_len;
            memcpy(&buffer[offset], p->value.str, str_len);
            offset += str_len;

            // Third string is empty (min value - not used for strings)
            buffer[offset++] = 0;

            // Fourth string is empty (max value - not used for strings)
            buffer[offset++] = 0;
            break;
        }

        default:
            buffer[offset++] = PARAM_TYPE_EMPTY;  // Empty type tag
            break;
    }

    // Add parameter name at the end
    uint8_t name_len = strlen(p->name);
    buffer[offset++] = name_len;
    memcpy(&buffer[offset], p->name, name_len);
    offset += name_len;

    // Find parameter index
    uint16_t param_index = 0;
    for (uint16_t i = 0; i < MAX_PARAMETERS; i++) {
        if (&parameters[i] == p) {
            param_index = i;
            break;
        }
    }

    // Add parameter index
    buffer[offset++] = param_index & 0xFF;
    buffer[offset++] = (param_index >> 8) & 0xFF;

    // Send parameter with high priority
    static uint8_t transfer_id = 0;
    int result = canardRequestOrRespond(&canard,
                              CANARD_BROADCAST_NODE_ID,
                              UAVCAN_PARAM_GETSET_SIGNATURE,
                              UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                              &transfer_id,
                              CANARD_TRANSFER_PRIORITY_HIGH,
                              CanardResponse,
                              buffer,
                              offset);

    

    // Process TX queue to ensure it's sent
    processTxRxOnce(&canard, 0);
}

/**
 * Calculate optimal motion parameters based on the magnitude of movement
 * This allows for smooth small movements and fast large movements
 * 
 * @param target_pos Target position in encoder counts
 * @param current_pos Current position in encoder counts
 */
void optimize_motion_parameters(int32_t target_pos, int32_t current_pos)
{
    // Calculate how far we need to move
    int32_t distance = abs(target_pos - current_pos);
    
    // Get base parameters - use global defaults initially
    uint32_t velocity = profile_velocity;
    uint32_t acceleration = profile_acceleration;
    uint32_t deceleration = profile_deceleration;
    
    // For very small movements, use slower velocity for precision
    if (distance < 10000) {
        velocity = 10000;  // Slower for small adjustments
        acceleration = 20000;
        deceleration = 20000;
        printf("Small movement detected (%ld): Using slow precise settings\n", (long)distance);
    }
    // For medium movements
    else if (distance < 50000) {
        velocity = 20000;  // Medium speed
        acceleration = 30000;
        deceleration = 30000;
        printf("Medium movement detected (%ld): Using medium speed settings\n", (long)distance);
    }
    // For large movements, use maximum velocity
    else {
        velocity = 50000;  // Fast for large movements
        acceleration = 50000;
        deceleration = 50000;
        printf("Large movement detected (%ld): Using maximum speed settings\n", (long)distance);
    }
    
    // Apply motion parameters to EPOS4 controller
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    
    // Set profile velocity
    TxHeader.StdId = 0x600 + EPOS4_NODE_ID;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Command to write profile velocity (0x6081)
    TxData[0] = 0x23;  // Write 4 bytes
    TxData[1] = 0x81;  // Index LSB (0x6081)
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    
    // Velocity value (little endian)
    TxData[4] = velocity & 0xFF;
    TxData[5] = (velocity >> 8) & 0xFF;
    TxData[6] = (velocity >> 16) & 0xFF;
    TxData[7] = (velocity >> 24) & 0xFF;
    
    // Send velocity command
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(5);
    IWDG->KR = 0xAAAA;
    
    // Command to write profile acceleration (0x6083)
    TxData[0] = 0x23;  // Write 4 bytes
    TxData[1] = 0x83;  // Index LSB (0x6083)
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    
    // Acceleration value (little endian)
    TxData[4] = acceleration & 0xFF;
    TxData[5] = (acceleration >> 8) & 0xFF;
    TxData[6] = (acceleration >> 16) & 0xFF;
    TxData[7] = (acceleration >> 24) & 0xFF;
    
    // Send acceleration command
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(5);
    IWDG->KR = 0xAAAA;
    
    // Command to write profile deceleration (0x6084)
    TxData[0] = 0x23;  // Write 4 bytes
    TxData[1] = 0x84;  // Index LSB (0x6084)
    TxData[2] = 0x60;  // Index MSB
    TxData[3] = 0x00;  // Subindex
    
    // Deceleration value (little endian)
    TxData[4] = deceleration & 0xFF;
    TxData[5] = (deceleration >> 8) & 0xFF;
    TxData[6] = (deceleration >> 16) & 0xFF;
    TxData[7] = (deceleration >> 24) & 0xFF;
    
    // Send deceleration command
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(5);
    IWDG->KR = 0xAAAA;
    
    // Store the values in global variables for later use
    profile_velocity = velocity;
    profile_acceleration = acceleration;
    profile_deceleration = deceleration;
}

/**
 * Force motor movement using a simplified, reliable state machine approach
 * This implementation focuses on the essential commands needed for position control
 * @param position Target position in encoder counts
 * @return 1 if successful, 0 if failed, 2 if in progress
 */
int force_motor_movement(int32_t position)
{

    // Define state machine states
    typedef enum {
        STATE_INIT,            // Starting state
        STATE_CHECK_POSITION,  // Check current position
        STATE_RESET_FAULT,     // Reset any fault conditions
        STATE_WAIT_RESET,      // Wait for reset to complete
        STATE_SET_MODE,        // Set operation mode
        STATE_WAIT_MODE,       // Wait for mode setting to complete
        STATE_ENABLE_OP,       // Enable operation command
        STATE_WAIT_ENABLE,     // Wait for enable to complete
        STATE_SET_PARAMS,      // Set motion parameters
        STATE_WAIT_PARAMS,     // Wait for parameters to apply
        STATE_SET_POSITION,    // Set target position
        STATE_APPLY_POSITION,  // Begin position movement
        STATE_COMPLETE         // Movement completed
    } movement_state_t;
    
    // Use a static state machine to remember where we are
    static movement_state_t current_state = STATE_INIT;
    static uint32_t last_state_change = 0;
    static uint32_t position_to_reach = 0;
    
    // Reset if new position is requested
    if (position_to_reach != position) {
        printf("MOTOR_DEBUG: Starting movement to position: %ld\n", (long)position);
        current_state = STATE_INIT;
        position_to_reach = position;
    }
    
    // Get current time for state timeouts
    uint32_t current_time = HAL_GetTick();
    
    // Ultra-fast state transitions for minimal command latency (was 5ms)
    const uint32_t STATE_TRANSITION_DELAY = 2;
    
    // Main state machine
    switch (current_state) {
        case STATE_INIT:
            // Just move to first state - Check position first
            current_state = STATE_CHECK_POSITION;
            last_state_change = current_time;
            break;

        case STATE_CHECK_POSITION:
            // Move directly to the next state - no position checking
            current_state = STATE_RESET_FAULT;
            last_state_change = current_time;
            break;

        case STATE_RESET_FAULT:
            // Send NMT reset for fault recovery
            printf("Sending fault reset command\n");
            epos4_send_sdo_write(0x6040, 0x00, 0x00000080, 2); // Fault reset bit

            // Move to next state
            current_state = STATE_WAIT_RESET;
            last_state_change = current_time;
            break;

        case STATE_WAIT_RESET:
            // Check if enough time has passed before next command
            if (current_time - last_state_change >= STATE_TRANSITION_DELAY) {
                current_state = STATE_SET_MODE;
                last_state_change = current_time;
            }
            break;

        case STATE_SET_MODE:
            // Set operation mode to Profile Position (PP)
            printf("Setting operation mode to Profile Position\n");
            epos4_send_sdo_write(0x6060, 0x00, 0x00000001, 1); // Profile Position mode

            // Move to next state
            current_state = STATE_WAIT_MODE;
            last_state_change = current_time;
            break;

        case STATE_WAIT_MODE:
            // Check if enough time has passed before next command
            if (current_time - last_state_change >= STATE_TRANSITION_DELAY) {
                current_state = STATE_ENABLE_OP;
                last_state_change = current_time;
            }
            break;

        case STATE_ENABLE_OP:
            // Send control word to enable operation (0x000F)
            printf("Sending enable operation command\n");
            epos4_send_sdo_write(0x6040, 0x00, 0x0000000F, 2); // Enable operation state

            // Move to next state
            current_state = STATE_WAIT_ENABLE;
            last_state_change = current_time;
            break;

        case STATE_WAIT_ENABLE:
            // Check if enough time has passed before next command
            if (current_time - last_state_change >= STATE_TRANSITION_DELAY) {
                current_state = STATE_SET_PARAMS;
                last_state_change = current_time;
            }
            break;

        case STATE_SET_PARAMS:
            // Apply current motion profile parameters
            // These should be already set by the command handler based on movement size
            printf("Applying motion parameters: v=%lu, a=%lu, d=%lu\n",
                   (unsigned long)profile_velocity,
                   (unsigned long)profile_acceleration,
                   (unsigned long)profile_deceleration);

            // Set profile velocity
            epos4_send_sdo_write(0x6081, 0x00, profile_velocity, 4);

            // Move to next state with minimal delay
            current_state = STATE_WAIT_PARAMS;
            last_state_change = current_time;
            break;

        case STATE_WAIT_PARAMS:
            // Check if enough time has passed before next command
            if (current_time - last_state_change >= STATE_TRANSITION_DELAY) {
                current_state = STATE_SET_POSITION;
                last_state_change = current_time;
            }
            break;

        case STATE_SET_POSITION:
            // Set target position
            printf("Setting target position: %ld\n", (long)position);
            epos4_send_sdo_write(0x607A, 0x00, position, 4);

            // Move to next state
            current_state = STATE_APPLY_POSITION;
            last_state_change = current_time;
            break;

        case STATE_APPLY_POSITION:
            // Send control word with bit 4 set (new position trigger)
            // This is CRITICAL - bit 4 must be set to trigger the move!
            printf("Triggering position movement\n");
            epos4_send_sdo_write(0x6040, 0x00, 0x0000001F, 2); // 0x000F with bit 4 set

            // Motor is now moving - process is complete
            current_state = STATE_COMPLETE;
            last_state_change = current_time;
            break;

        case STATE_COMPLETE:
            // Reset state machine for next movement
            current_state = STATE_INIT;
            return 1; // Success
    }
    
    // Still in progress
    return 2;
}

/**
 * Helper function to ensure CAN buffer isn't overwhelmed
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if a TX mailbox is available, false if timed out
 */
bool wait_for_tx_complete(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    // Wait until at least one TX mailbox is free or timeout occurs
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 && 
            HAL_GetTick() - start_time < timeout_ms) {
        // Keep processing received messages while waiting
        processTxRxOnce(&canard, 0);
        // Keep watchdog happy
        IWDG->KR = 0xAAAA;
    }
    return HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0;
}

/**
 * Helper function to send CAN message with buffer checking
 * @param header Pointer to CAN header
 * @param data Pointer to data buffer
 * @return HAL status code
 */
HAL_StatusTypeDef send_with_check(CAN_TxHeaderTypeDef* header, uint8_t* data) {
    uint32_t mailbox;
    if (!wait_for_tx_complete(20)) {
        printf("TX buffer full, message skipped\n");
        return HAL_ERROR;
    }
    return HAL_CAN_AddTxMessage(&hcan1, header, data, &mailbox);
}

void initParameters()
{
    // Feed watchdog during initialization
    IWDG->KR = 0xAAAA;
    
    // Set default position values that match our tilt motor requirements
    // maxPosition = 100000;  (Already set globally)
    // midPosition = -100000; (Already set globally)
    // minPosition = -300000; (Already set globally)
    
    // Set moderate motion profile parameters - reliable but not too slow
    profile_velocity = 10000;     // Moderate speed (encoder counts/second)
    profile_acceleration = 5000;  // Gentle acceleration
    profile_deceleration = 5000;  // Gentle deceleration
    
    // Update parameters if they exist
    param_t* velocity_param = getParamByName("epos4.velocity");
    if (velocity_param != NULL) {
        velocity_param->value.i = profile_velocity;
    }
    
    param_t* accel_param = getParamByName("epos4.accel_rate");
    if (accel_param != NULL) {
        accel_param->value.i = profile_acceleration;
    }
    
    param_t* decel_param = getParamByName("epos4.decel_rate");
    if (decel_param != NULL) {
        decel_param->value.i = profile_deceleration;
    }
    
    // Print initialization values
    printf("Position limits set to: min=%ld, mid=%ld, max=%ld\n",
           (long)minPosition, (long)midPosition, (long)maxPosition);
    printf("Motion profile: velocity=%lu, accel=%lu, decel=%lu\n",
           (unsigned long)profile_velocity, 
           (unsigned long)profile_acceleration,
           (unsigned long)profile_deceleration);
    
    
}

// Add this function to check EPOS4 status
void check_epos4_status(void) {
    // Create message to read statusword
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;
    
    printf("STATUS_DEBUG: Requesting EPOS4 statusword\n");
    
    // Configure header for SDO message
    tx_header.StdId = 0x600 + EPOS4_NODE_ID;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    // Request statusword (0x6041)
    tx_data[0] = 0x40;  // Read command
    tx_data[1] = 0x41;  // Index low byte
    tx_data[2] = 0x60;  // Index high byte
    tx_data[3] = 0x00;  // Subindex
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    
    // Send the request
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    
    // Allow time for response
    
    // Process any CAN messages
    processTxRxOnce(&canard, 10);
    
    // Also request current position
    printf("STATUS_DEBUG: Requesting current position\n");
    tx_data[0] = 0x40;  // Read command
    tx_data[1] = 0x64;  // Index low byte of 0x6064 (position actual value)
    tx_data[2] = 0x60;  // Index high byte
    tx_data[3] = 0x00;  // Subindex
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    
    // Send the request
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    
    // Allow time for response
    
    // Process any CAN messages
    processTxRxOnce(&canard, 10);
    
    printf("STATUS_DEBUG: Status check complete\n");
}

/**
 * Save current motor position to flash memory to maintain position through power cycles
 * @param position Current motor position to save
 */
void save_position_to_flash(int32_t position) {
    // First make sure we have valid position
    if (position < minPosition - 10000 || position > maxPosition + 10000) {
        printf("Invalid position for saving: %ld\n", (long)position);
        return;
    }

    FLASH_EraseInitTypeDef EraseInit;
    uint32_t SectorError = 0;
    HAL_StatusTypeDef status;

    // Unlock flash
    HAL_FLASH_Unlock();
    
    // Erase the sector
    EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInit.Sector = FLASH_SECTOR_POSITION;
    EraseInit.NbSectors = 1;
    EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    printf("Saving position %ld to flash...\n", (long)position);
    
    status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
    if (status != HAL_OK) {
        printf("Flash erase failed: %d\n", (int)status);
        HAL_FLASH_Lock();
        return;
    }
    
    // Write magic number first (for validation)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, POSITION_FLASH_ADDRESS, POSITION_MAGIC_NUMBER);
    if (status != HAL_OK) {
        printf("Magic number write failed: %d\n", (int)status);
        HAL_FLASH_Lock();
        return;
    }
    
    // Then write the position
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, POSITION_FLASH_ADDRESS + 4, (uint32_t)position);
    if (status != HAL_OK) {
        printf("Position write failed: %d\n", (int)status);
        HAL_FLASH_Lock();
        return;
    }
    
    // Lock flash
    HAL_FLASH_Lock();
    printf("Position saved to flash successfully\n");
}

/**
 * Restore motor position from flash memory on startup
 * @return true if position was restored successfully
 */
bool restore_position_from_flash(void) {
    // Check if there is valid position data in flash
    uint32_t magic = *(__IO uint32_t*)POSITION_FLASH_ADDRESS;
    
    // Verify magic number
    if (magic != POSITION_MAGIC_NUMBER) {
        printf("No valid position data in flash (magic=%08lx)\n", (unsigned long)magic);
        return false;
    }
    
    // Read the position value
    int32_t saved_position = *(__IO uint32_t*)(POSITION_FLASH_ADDRESS + 4);
    
    // Validate that the position is within reasonable bounds
    if (saved_position < minPosition - 10000 || saved_position > maxPosition + 10000) {
        printf("Saved position out of range: %ld\n", (long)saved_position);
        return false;
    }
    
    printf("Restoring saved position: %ld\n", (long)saved_position);
    
    // Set the EPOS4 position counter to match the saved position
    // Using SDO write to object 0x2081 subindex 0 (position counter)
    epos4_send_sdo_write(0x2081, 0x00, saved_position, 4);
    HAL_Delay(100); // Wait for command to complete
    
    // Important: Also update our internal position tracking
    targetPosition = saved_position;
    current_position = saved_position;
    
    return true;
}

/**
 * Periodically check if position has changed and save to flash if needed
 */
void periodic_position_save(void) {
    static uint32_t last_save_time = 0;
    static int32_t last_saved_position = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Save position every 30 seconds if it changed significantly
    if (current_time - last_save_time > 30000) {
        // Only save if position changed by more than 1000 steps
        if (abs(current_position - last_saved_position) > 1000) {
            save_position_to_flash(current_position);
            last_saved_position = current_position;
        }
        last_save_time = current_time;
    }
}





