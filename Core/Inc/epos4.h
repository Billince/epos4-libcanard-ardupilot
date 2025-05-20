#ifndef __EPOS4_H
#define __EPOS4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* EPOS4 Object Dictionary */
#define OBJ_CONTROLWORD                  0x6040 // Control word
#define OBJ_STATUSWORD                   0x6041 // Status word
#define OBJ_MODE_OF_OPERATION            0x6060 // Mode of operation
#define OBJ_MODE_OF_OPERATION_DISPLAY    0x6061 // Mode of operation display
#define OBJ_POSITION_ACTUAL_VALUE        0x6064 // Actual position
#define OBJ_TARGET_POSITION              0x607A // Target position
#define OBJ_PROFILE_VELOCITY             0x6081 // Profile velocity
#define OBJ_PROFILE_ACCELERATION         0x6083 // Profile acceleration
#define OBJ_PROFILE_DECELERATION         0x6084 // Profile deceleration

/* EPOS4 Control Commands */
#define CMD_SHUTDOWN                     0x0006
#define CMD_SWITCH_ON                    0x0007
#define CMD_ENABLE_OPERATION            0x000F
#define CMD_DISABLE_OPERATION           0x0007
#define CMD_QUICK_STOP                  0x0002
#define CMD_FAULT_RESET                 0x0080

/* EPOS4 Modes of Operation */
#define MODE_PROFILE_POSITION            0x01
#define MODE_PROFILE_VELOCITY            0x03
#define MODE_PROFILE_TORQUE              0x04
#define MODE_HOMING                      0x06
#define MODE_INTERPOLATED_POSITION       0x07
#define MODE_CYCLIC_SYNCHRONOUS_POSITION 0x08
#define MODE_CYCLIC_SYNCHRONOUS_VELOCITY 0x09
#define MODE_CYCLIC_SYNCHRONOUS_TORQUE   0x0A

/* EPOS4 Command Types */
#define EPOS4_READ_OBJECT               0x40
#define EPOS4_WRITE_OBJECT              0x22

/* Function Declarations - supporting both implementations */
/* epos4.c implementation */
bool EPOS4_Init(CAN_HandleTypeDef *hcan, uint8_t nodeId, uint32_t encoder_resolution, 
               uint32_t acceleration, uint32_t velocity);
bool EPOS4_SetTargetPosition(CAN_HandleTypeDef *hcan, uint8_t nodeId, int32_t position);
int32_t EPOS4_GetActualPosition(CAN_HandleTypeDef *hcan, uint8_t nodeId);
bool EPOS4_SendSDO(CAN_HandleTypeDef *hcan, uint8_t nodeId, uint8_t command, uint16_t index, 
                  uint8_t subIndex, uint32_t data);
bool EPOS4_ReceiveSDO(uint32_t *data, uint32_t timeout);
void EPOS4_ProcessRxMessage(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);

/* main.c implementation */
bool EPOS4_SetTargetPosition_Legacy(CAN_HandleTypeDef* hcan, uint8_t node_id, int32_t position);
void EPOS4_SetControlWord(CAN_HandleTypeDef* hcan, uint8_t node_id, uint16_t control_word);
bool EPOS4_Init_Legacy(CAN_HandleTypeDef* hcan, uint8_t node_id);

#ifdef __cplusplus
}
#endif

<<<<<<< HEAD
#endif /* __EPOS4_H */ 
=======
#endif /* __EPOS4_H */ 
>>>>>>> 78e9d6b53c4725235187a1d5d9e888d566978986
