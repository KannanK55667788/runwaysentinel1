#ifndef RUNWAY_IPC_H
#define RUNWAY_IPC_H

#include <stdint.h>
#include <time.h>

#define RUNWAY_CHANNEL "runway_supervisor"
#define RUNWAY_BUTTON_MESSAGE 0x5201
#define RUNWAY_IPC_VERSION 3
#define RUNWAY_MAX_AGE_MS UINT64_C(1000)

enum runway_event {
    RUNWAY_BUTTONS_READY,
    RUNWAY_VEHICLE_ENTER,
    RUNWAY_VEHICLE_EXIT
};

struct runway_button_message {
    uint16_t type;
    uint16_t version;
    uint32_t event;
    uint64_t sequence;
    uint64_t timestamp_ms;
    uint64_t vehicle_count;
};

#define RUNWAY_SENSOR_MESSAGE 0x5202

enum sensor_kind {
    SENSORS_READY,
    AIRCRAFT_ENTER,
    AIRCRAFT_EXIT,
    AIRCRAFT_RANGE,
    VEHICLE_RANGE,
    IR_BEAMS
};

#define RUNWAY_RANGE_LIMIT_MM 2000u
enum range_state { RANGE_NO_ECHO, RANGE_OUTSIDE, RANGE_INSIDE, RANGE_BEYOND_LIMIT };

struct runway_sensor_message {
    uint16_t type;
    uint16_t version;
    uint32_t kind;
    uint64_t sequence;
    uint64_t timestamp_ms;
    uint32_t range_state;
    uint32_t distance_mm; /* Zero for a missing or beyond-limit echo. */
    int32_t radial_mm_per_s;
    uint32_t motion_valid;
    uint32_t ir_mask; /* bit 0: IR1 blocked, bit 1: IR2 blocked */
};

#define RUNWAY_BLE_MESSAGE 0x5203
#define RUNWAY_BLE_MAX_AGE_MS UINT64_C(3000)

enum runway_tag { TAG_AC1, TAG_AC2, TAG_GV1, TAG_COUNT };
enum runway_ble_kind { TAG_SEEN, SELECT_GATE, SET_APPROACH, RECONCILE_TRACKS, REGISTER_LANDING };

struct runway_ble_message {
    uint16_t type;
    uint16_t version;
    uint16_t kind;
    uint16_t tag;
    uint64_t timestamp_ms;
    int32_t rssi_dbm; /* 127 means RSSI unavailable; valid range -127..20 */
};

static inline int runway_range_has_distance(uint32_t state)
{
    return state == RANGE_INSIDE || state == RANGE_OUTSIDE;
}

static inline const char *runway_tag_name(unsigned tag)
{
    static const char *const names[] = {"AC1", "AC2", "GV1"};
    return tag < TAG_COUNT ? names[tag] : "--";
}

/* The A/C/B display zone is independent of the conflict range threshold. */
static inline const char *runway_range_zone(uint32_t mm)
{
    if (mm < 500)
        return "A";
    if (mm > 1000)
        return "B";
    if (mm >= 510 && mm <= 990)
        return "C";
    return "UNASSIGNED";
}

static inline const char *runway_range_direction(int32_t radial_mm_per_s)
{
    return radial_mm_per_s < 0 ? "TOWARD" : radial_mm_per_s > 0 ? "AWAY" : "STILL";
}

static inline int runway_monotonic_ms(uint64_t *now_ms)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) == -1)
        return -1;

    *now_ms = (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
    return 0;
}

#endif
