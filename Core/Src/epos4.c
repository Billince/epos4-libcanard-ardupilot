/**
 * @file epos4.c
 * @brief EPOS4 Motor Controller Driver Implementation
 */

#include "epos4.h"
#include <stdbool.h>
#include <string.h>

/* Static variables for SDO response handling */
static bool sdo_response_received = false;
static uint32_t sdo_response_data = 0;

/**
 * @brief Initialize EPOS4 motor controller
 * @param hcan CAN handle
 * @param nodeId EPOS4 node ID
 * @param encoder_resolution Encoder resolution (counts per turn)
 * @param acceleration Profile acceleration
 * @param velocity Profile velocity
 * @return true if initialization successful, false otherwise
 */
bool EPOS4_Init(CAN_HandleTypeDef *hcan, uint8_t nodeId, uint32_t encoder_resolution, 
               uint32_t acceleration, uint32_t velocity)
{
    /* Reset fault if any */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_CONTROLWORD, 0, CMD_FAULT_RESET))
    {
        return false;
    }
    
    HAL_Delay(10);
    
    /* Set mode of operation (Profile Position Mode) */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_MODE_OF_OPERATION, 0, MODE_PROFILE_POSITION))
    {
        return false;
    }
    
    /* Set profile velocity */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_PROFILE_VELOCITY, 0, velocity))
    {
        return false;
    }
    
    /* Set profile acceleration */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_PROFILE_ACCELERATION, 0, acceleration))
    {
        return false;
    }
    
    /* Set profile deceleration (same as acceleration) */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_PROFILE_DECELERATION, 0, acceleration))
    {
        return false;
    }
    
    /* Shutdown command */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN))
    {
        return false;
    }
    
    HAL_Delay(10);
    
    /* Switch on command */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_CONTROLWORD, 0, CMD_SWITCH_ON))
    {
        return false;
    }
    
    HAL_Delay(10);
    
    /* Enable operation command */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_CONTROLWORD, 0, CMD_ENABLE_OPERATION))
    {
        return false;
    }
    
    return true;
}

/**
 * @brief Set target position for EPOS4
 * @param hcan CAN handle
 * @param nodeId EPOS4 node ID
 * @param position Target position in encoder counts
 * @return true if command sent successfully, false otherwise
 */
bool EPOS4_SetTargetPosition(CAN_HandleTypeDef *hcan, uint8_t nodeId, int32_t position)
{
    /* Set target position */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_TARGET_POSITION, 0, (uint32_t)position))
    {
        return false;
    }
    
    /* Execute movement (bit 4 triggers the motion) */
    uint16_t controlWord = CMD_ENABLE_OPERATION | 0x0010;
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_WRITE_OBJECT, OBJ_CONTROLWORD, 0, controlWord))
    {
        return false;
    }
    
    return true;
}

/**
 * @brief Get actual position from EPOS4
 * @param hcan CAN handle
 * @param nodeId EPOS4 node ID
 * @return Current position in encoder counts, or -1 if error
 */
int32_t EPOS4_GetActualPosition(CAN_HandleTypeDef *hcan, uint8_t nodeId)
{
    /* Reset response flags */
    sdo_response_received = false;
    
    /* Request position */
    if (!EPOS4_SendSDO(hcan, nodeId, EPOS4_READ_OBJECT, OBJ_POSITION_ACTUAL_VALUE, 0, 0))
    {
        return -1;
    }
    
    /* Wait for response with timeout */
    if (!EPOS4_ReceiveSDO(&sdo_response_data, 100))
    {
        return -1;
    }
    
    return (int32_t)sdo_response_data;
}

/**
 * @brief Send SDO message to EPOS4
 * @param hcan CAN handle
 * @param nodeId EPOS4 node ID
 * @param command SDO command (read/write)
 * @param index Object dictionary index
 * @param subIndex Object dictionary subindex
 * @param data Data to send (for write commands)
 * @return true if message sent successfully, false otherwise
 */
bool EPOS4_SendSDO(CAN_HandleTypeDef *hcan, uint8_t nodeId, uint8_t command, 
                  uint16_t index, uint8_t subIndex, uint32_t data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;
    
    /* SDO message ID: 0x600 + nodeId */
    txHeader.StdId = 0x600 + nodeId;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    
    /* Format SDO message */
    txData[0] = command;
    txData[1] = (uint8_t)(index & 0xFF);        /* LSB of index */
    txData[2] = (uint8_t)((index >> 8) & 0xFF); /* MSB of index */
    txData[3] = subIndex;
    
    /* For write commands, include data */
    if (command == EPOS4_WRITE_OBJECT)
    {
        txData[4] = (uint8_t)(data & 0xFF);
        txData[5] = (uint8_t)((data >> 8) & 0xFF);
        txData[6] = (uint8_t)((data >> 16) & 0xFF);
        txData[7] = (uint8_t)((data >> 24) & 0xFF);
    }
    
    /* Send message */
    if (HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox) != HAL_OK)
    {
        return false;
    }
    
    /* Wait for message to be sent */
    uint32_t timeout = HAL_GetTick() + 100; /* 100ms timeout */
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
    {
        if (HAL_GetTick() > timeout)
        {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Wait for and process SDO response
 * @param data Pointer to store received data
 * @param timeout Timeout in milliseconds
 * @return true if response received, false if timeout
 */
bool EPOS4_ReceiveSDO(uint32_t *data, uint32_t timeout)
{
    uint32_t startTime = HAL_GetTick();
    
    /* Wait for response with timeout */
    while (!sdo_response_received)
    {
        if ((HAL_GetTick() - startTime) > timeout)
        {
            return false;
        }
        
        /* Small delay to prevent CPU hogging */
        HAL_Delay(1);
    }
    
    /* Copy response data */
    *data = sdo_response_data;
    sdo_response_received = false;
    
    return true;
}

/**
 * @brief Process received CAN message for EPOS4
 * @param hcan CAN handle
 * @param rxHeader CAN Rx header
 * @param rxData Received data buffer
 */
void EPOS4_ProcessRxMessage(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    /* Check if this is an SDO response (0x580 + nodeId) */
    if ((rxHeader->StdId >= 0x580) && (rxHeader->StdId < 0x600))
    {
        /* Process SDO response */
        uint8_t command = rxData[0];
        
        /* Check if this is a read response */
        if ((command & 0x40) != 0)
        {
            /* Extract data from response */
            sdo_response_data = 0;
            sdo_response_data |= (uint32_t)rxData[4];
            sdo_response_data |= (uint32_t)rxData[5] << 8;
            sdo_response_data |= (uint32_t)rxData[6] << 16;
            sdo_response_data |= (uint32_t)rxData[7] << 24;
            
            /* Mark response as received */
            sdo_response_received = true;
        }
    }
<<<<<<< HEAD
}
=======
} 
>>>>>>> 78e9d6b53c4725235187a1d5d9e888d566978986
