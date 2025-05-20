#ifndef UAVCAN_PROTOCOL_H
#define UAVCAN_PROTOCOL_H

#include <stdint.h>

// UAVCAN Protocol Constants
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE 0x0
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID 0x0

// UAVCAN Message Types
#define UAVCAN_PROTOCOL_NODE_STATUS_ID 0x0
#define UAVCAN_PROTOCOL_GET_NODE_INFO_ID 0x1
#define UAVCAN_PROTOCOL_RESTART_NODE_ID 0x2
#define UAVCAN_PROTOCOL_BEGIN_FIRMWARE_UPDATE_ID 0x3
#define UAVCAN_PROTOCOL_FILE_BEGIN_ID 0x4
#define UAVCAN_PROTOCOL_FILE_READ_ID 0x5
#define UAVCAN_PROTOCOL_FILE_WRITE_ID 0x6
#define UAVCAN_PROTOCOL_FILE_END_ID 0x7
#define UAVCAN_PROTOCOL_GET_TRANSPORT_STATS_ID 0x8
#define UAVCAN_PROTOCOL_DEBUG_ID 0x9


// UAVCAN Data Types
typedef struct {
    uint32_t uptime_sec;
    uint8_t health;
    uint8_t mode;
    uint8_t sub_mode;
    uint16_t vendor_specific_status_code;
} uavcan_protocol_NodeStatus;

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t optional_field_flags;
    uint8_t hardware_version;
    uint8_t software_version;
    uint64_t software_vcs_revision_id;
    uint8_t unique_id[16];
    char name[50];
    char software_version_string[50];
    char hardware_version_string[50];
} uavcan_protocol_GetNodeInfo;

#endif // UAVCAN_PROTOCOL_H 