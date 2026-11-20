#ifndef RUNWAY_FAULT_IPC_H
#define RUNWAY_FAULT_IPC_H

#include "runway_ipc.h"

#define RUNWAY_FAULT_CHANNEL "runway_fault_manager"
#define RUNWAY_FAULT_BLE_OBSERVATION 0x5210

/* Adapter-to-fault-manager only. The supervisor receives runway_ble_message. */
struct runway_fault_ble_observation {
    uint16_t type;
    uint16_t version;
    uint16_t tag;
    uint8_t address_type; /* HCI public=0, random=1 */
    uint8_t reserved;
    uint64_t timestamp_ms;
    uint64_t sequence; /* Increment independently for each tag. Start at 1. */
    int32_t rssi_dbm;
    uint8_t address[6]; /* In HCI report byte order. */
};

#define RUNWAY_FAULT_TEST_COMMAND 0x5211

enum runway_fault_test_action {
    TEST_DELAY_AIR, TEST_REPLAY_BUTTON, TEST_CONFLICT_AC1
};

struct runway_fault_test_command {
    uint16_t type;
    uint16_t version;
    uint32_t action;
    uint32_t delay_ms;
};

#define RUNWAY_FAULT_HEARTBEAT 0x5212
#define RUNWAY_HEARTBEAT_INTERVAL_MS UINT64_C(500)
#define RUNWAY_HEARTBEAT_LIMIT_MS UINT64_C(2000)

enum runway_heartbeat_source { HEARTBEAT_IR, HEARTBEAT_RANGE, HEARTBEAT_BUTTON, HEARTBEAT_BLE, HEARTBEAT_COUNT };

struct runway_heartbeat_message {
    uint16_t type;
    uint16_t version;
    uint32_t source;
    uint64_t timestamp_ms;
};

#define RUNWAY_SUPERVISOR_LINK 0x5213
#define RUNWAY_SUPERVISOR_LINK_VERSION 1u

enum runway_link_action { RUNWAY_LINK_PROBE, RUNWAY_LINK_RESUME_RANGE };

struct runway_link_message {
    uint16_t type;
    uint16_t link_version;
    uint32_t action;
    uint64_t timestamp_ms;
};

#endif
