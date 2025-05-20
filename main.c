/* USER CODE BEGIN PV */
// Make position variables global
static int32_t maxPosition = 0;          // Maximum position value (0)
static int32_t midPosition = -144000;    // Middle position (-144000)
static int32_t minPosition = -288000;    // Minimum position (-288000)
static int32_t targetPosition = -144000; // Start at middle position
static uint8_t positionIndex = 1;        // Start at middle position (1=mid)

/* CAN2 Configuration for second EPOS4 */
#define EPOS4_NODE_ID_2           0x01   // Node ID for second EPOS4 on CAN2
#define DRONECAN_NODE_ID_2        0x0A   // Node ID 10 for second motor (Servo 10)

// Position limits for second motor (Tilt2)
static int32_t maxPosition2 = 0;          // Maximum position for second motor
static int32_t midPosition2 = -144000;    // Middle position for second motor
static int32_t minPosition2 = -288000;    // Minimum position for second motor
static int32_t targetPosition2 = -144000; // Start at middle position
static int32_t current_position2 = -144000; // Current position of second motor
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
void InitializeLEDs(void);
void TestAllLEDs(void);
void Blink_Blue_LED(void);
void ProcessCANMessage(CAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[]);
void ProcessCAN2Message(CAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[]); // For CAN2 messages

/* EPOS4 Control Function Prototypes */
HAL_StatusTypeDef CANopen_SDO_Write_Quick(CAN_HandleTypeDef *hcan, uint8_t nodeID, uint16_t index, uint8_t subIndex, uint32_t data, uint8_t len);
HAL_StatusTypeDef EPOS4_SetOperationMode(CAN_HandleTypeDef *hcan, uint8_t nodeID, int8_t mode);
HAL_StatusTypeDef EPOS4_SetControlWord(CAN_HandleTypeDef *hcan, uint8_t nodeID, uint16_t controlWord);
HAL_StatusTypeDef EPOS4_SetTargetPosition(CAN_HandleTypeDef *hcan, uint8_t nodeID, int32_t position);


/* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    // Feed watchdog
    HAL_IWDG_Refresh(&hiwdg);
    
    // Toggle activity LED to show system is running
    Blink_Blue_LED();
    
    // Send periodic status updates
    static uint32_t lastStatusTime = 0;
    uint32_t currentTime = HAL_GetTick();
    
    if (currentTime - lastStatusTime > 100) {  // 10Hz status updates
      // Read position from each EPOS4
      EPOS4_ReadActualPosition(&hcan1, EPOS4_NODE_ID);
      EPOS4_ReadActualPosition(&hcan2, EPOS4_NODE_ID_2);
      
      lastStatusTime = currentTime;
    }
    
    // Small delay to prevent CPU hogging
    HAL_Delay(10);
  }
  /* USER CODE END 3 */

/* USER CODE BEGIN 4 */

// Function to process CAN2 messages (for second tilt motor)
void ProcessCAN2Message(CAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[])
{
  // Check if it's a DroneCAN message for motor 10
  if (RxHeader->StdId == 0x110) {  // ArduPilot sends servo 10 commands on this ID
    // Extract command value from message (similar format as servo 9)
    if (RxHeader->DLC >= 4) {
      // ArduPilot format: extract command value (little-endian)
      uint16_t command_value = (RxData[3] << 8) | RxData[2];
      
      // Show activity and log command
      HAL_GPIO_TogglePin(GPIOD, LD5_Pin); // Toggle Orange LED for activity
      
      // Map command to position for second tilt motor
      int32_t target_position;
      
      // Use same mapping logic as main motor
      if (command_value >= 48000) {
        target_position = maxPosition2;
      } 
      else if (command_value <= 15000) {
        target_position = minPosition2;
      }
      else {
        // Linear interpolation for values in between
        float normalized = (float)(command_value - 15360) / (float)(48128 - 15360);
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;
        
        target_position = minPosition2 + (int32_t)(normalized * (maxPosition2 - minPosition2));
      }
      
      // Send position command to EPOS4 on CAN2
      EPOS4_SetTargetPosition(&hcan2, EPOS4_NODE_ID_2, target_position);
      EPOS4_SetControlWord(&hcan2, EPOS4_NODE_ID_2, 0x001F); // Start motion
    }
  }
  
  // Check for CANopen response from second EPOS4
  else if (RxHeader->StdId == (SDO_TX_ID + EPOS4_NODE_ID_2)) {
    // Extract index, subindex and data
    uint16_t index = (RxData[2] << 8) | RxData[1];
    uint8_t subIndex = RxData[3];
    uint32_t data = (RxData[7] << 24) | (RxData[6] << 16) | (RxData[5] << 8) | RxData[4];
    
    // Handle different response types
    if (index == OD_STATUSWORD && subIndex == 0) {
      // Process status word from second motor
      uint16_t statusWord = (RxData[5] << 8) | RxData[4];
      
      // Request actual position 
      CANopen_SDO_Read(&hcan2, EPOS4_NODE_ID_2, OD_ACTUAL_POSITION, 0);
    }
    else if (index == OD_ACTUAL_POSITION && subIndex == 0) {
      // Store position of second motor
      current_position2 = (int32_t)data;
    }
  }
}

// Update CAN RX callback to handle both CAN controllers
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  
  // Check which CAN controller triggered the interrupt
  if (hcan->Instance == CAN1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
      // Process CAN1 message (Servo 9 - First tilt motor)
      ProcessCANMessage(&RxHeader, RxData);
      HAL_GPIO_TogglePin(GPIOD, LD4_Pin); // Toggle Green LED for CAN1 activity
    }
  }
  else if (hcan->Instance == CAN2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
      // Process CAN2 message (Servo 10 - Second tilt motor)
      ProcessCAN2Message(&RxHeader, RxData);
    }
  }
}

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
  // Initialize basic hardware
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
  MX_IWDG_Init();
  MX_CAN2_Init();
  
  /* USER CODE BEGIN 2 */
  // Initialize LEDs
  InitializeLEDs();
  TestAllLEDs();
  
  // Configure CAN1 Filter to accept all messages
  CAN_FilterTypeDef canFilter1;
  canFilter1.FilterBank = 0;
  canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter1.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter1.FilterIdHigh = 0x0000;
  canFilter1.FilterIdLow = 0x0000;  
  canFilter1.FilterMaskIdHigh = 0x0000;
  canFilter1.FilterMaskIdLow = 0x0000;
  canFilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter1.FilterActivation = CAN_FILTER_ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan1, &canFilter1) != HAL_OK) {
    Error_Handler();
  }
  
  // Configure CAN2 Filter to accept all messages
  CAN_FilterTypeDef canFilter2;
  canFilter2.FilterBank = 14;  // CAN2 filters start at 14
  canFilter2.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter2.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter2.FilterIdHigh = 0x0000;
  canFilter2.FilterIdLow = 0x0000;
  canFilter2.FilterMaskIdHigh = 0x0000;
  canFilter2.FilterMaskIdLow = 0x0000;
  canFilter2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter2.FilterActivation = CAN_FILTER_ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan2, &canFilter2) != HAL_OK) {
    Error_Handler();
  }
  
  // Start CAN1
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  
  // Start CAN2
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  
  // Enable interrupts for CAN1
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  
  // Enable interrupts for CAN2
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  
  // Initialize EPOS4 on CAN1
  CANopen_NMT_Command(&hcan1, 0x82, EPOS4_NODE_ID); // Reset communications
  HAL_Delay(100);
  CANopen_NMT_Command(&hcan1, 0x01, EPOS4_NODE_ID); // Start node
  HAL_Delay(50);
  EPOS4_SetOperationMode(&hcan1, EPOS4_NODE_ID, MODE_PROFILE_POSITION);
  HAL_Delay(50);
  EPOS4_SetProfileVelocity(&hcan1, EPOS4_NODE_ID, 5000); // Set profile parameters
  HAL_Delay(50);
  EPOS4_SetProfileAcceleration(&hcan1, EPOS4_NODE_ID, 10000);
  HAL_Delay(50);
  EPOS4_SetControlWord(&hcan1, EPOS4_NODE_ID, 0x000F); // Enable operation
  
  // Initialize EPOS4 on CAN2
  CANopen_NMT_Command(&hcan2, 0x82, EPOS4_NODE_ID_2); // Reset communications
  HAL_Delay(100);
  CANopen_NMT_Command(&hcan2, 0x01, EPOS4_NODE_ID_2); // Start node
  HAL_Delay(50);
  EPOS4_SetOperationMode(&hcan2, EPOS4_NODE_ID_2, MODE_PROFILE_POSITION);
  HAL_Delay(50);
  EPOS4_SetProfileVelocity(&hcan2, EPOS4_NODE_ID_2, 5000); // Set profile parameters
  HAL_Delay(50);
  EPOS4_SetProfileAcceleration(&hcan2, EPOS4_NODE_ID_2, 10000);
  HAL_Delay(50);
  EPOS4_SetControlWord(&hcan2, EPOS4_NODE_ID_2, 0x000F); // Enable operation
  
  // Set following error window for both motors
  CANopen_SDO_Write(&hcan1, EPOS4_NODE_ID, 0x6065, 0x00, 2000, 4);  // Following error window
  CANopen_SDO_Write(&hcan2, EPOS4_NODE_ID_2, 0x6065, 0x00, 2000, 4); // Following error window
} 