#define _POSIX_C_SOURCE 200809L
#define _QNX_SOURCE 1

#include "runway_fault_ipc.h"
#include "runway_schedule.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef RUNWAY_HOST_TEST
#include <signal.h>
#include <sys/dispatch.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#endif

/* RunwaySentinel tabletop demonstrator: NOT an operational aviation system. */
#define BIT(tag) (1u << (tag))
#define AIR_TAGS (BIT(TAG_AC1) | BIT(TAG_AC2))
#define DASHBOARD_SIZE 4096
#define ALERT_DEADLINE_NS UINT64_C(100000000) /* Demo target: 100 ms. */
#define RSSI_WINDOW 5
#define RSSI_MARGIN_DB 12
#define RSSI_MAX_SAMPLE_AGE_MS UINT64_C(3000)
#define DASHBOARD_PERIOD_MS UINT64_C(1500)
#define BACKGROUND_SAMPLES 5u
#define BACKGROUND_TOLERANCE_MM 20u
#define OBJECT_DELTA_MM 35u
#define MIN_ECHO_MM 20u
#define MIN_BACKGROUND_MM (MIN_ECHO_MM + OBJECT_DELTA_MM + 1u)
#define BEYOND_LIMIT_BACKGROUND_MM (RUNWAY_RANGE_LIMIT_MM + OBJECT_DELTA_MM + 1u)
#define IR_SEQUENCE_MS UINT64_C(3000) /* Matches the sensor worker crossing window. */
#define MOTION_GAP_MS UINT64_C(750)
#define MIN_MOTION_INTERVAL_MS UINT64_C(50)
#define STILL_MM_PER_S 2.0
#define DEPARTURE_STEP_MM 10
#define DEPARTURE_STEPS 2u
#define AIRCRAFT_CAPACITY 2u

/* Raspberry Pi 4 panel outputs. Use BCM GPIO numbers. */
#define GPIO_LED_CLEAR 13
#define GPIO_LED_OCCUPIED 19
#define GPIO_LED_VERIFY 16
#define GPIO_LED_CONFLICT 26
#define GPIO_BUZZER 12
#define BUZZER_ACTIVE_LOW 0

enum panel_output {
    PANEL_VERIFY, PANEL_CLEAR, PANEL_OCCUPIED, PANEL_CONFLICT, PANEL_BUZZER, PANEL_OUTPUT_COUNT
};

static const int panel_pins[PANEL_OUTPUT_COUNT] = {
    [PANEL_VERIFY] = GPIO_LED_VERIFY,
    [PANEL_CLEAR] = GPIO_LED_CLEAR,
    [PANEL_OCCUPIED] = GPIO_LED_OCCUPIED,
    [PANEL_CONFLICT] = GPIO_LED_CONFLICT,
    [PANEL_BUZZER] = GPIO_BUZZER
};

static const char *const runway_labels[] = {
    [PANEL_VERIFY] = "HOLD / VERIFY",
    [PANEL_CLEAR] = "CLEAR",
    [PANEL_OCCUPIED] = "OCCUPIED",
    [PANEL_CONFLICT] = "CONFLICT"
};

struct runway_panel {
    int fd[PANEL_OUTPUT_COUNT];
    unsigned opened;
    enum panel_output lit;
};

struct vehicle_state {
    int connection;
    uint64_t sequence, timestamp_ms, count;
    uint64_t leaving_ms;
    bool exited, on_runway, connection_lost, left_runway;
};

struct range_motion {
    uint64_t timestamp_ms;
    uint32_t distance_mm;
    double radial_mm_per_s;
    unsigned outward_steps;
    bool valid;
};

struct sensor_state {
    int connection;
    bool range_only;
    uint64_t aircraft_count, gate_sequence, gate_time_ms;
    uint64_t ir_sequence, ir_time_ms;
    uint64_t ir_started_ms;
    uint64_t entries, exits;
    unsigned ir_mask, ir_seen_mask;
    struct runway_sensor_message range[2];
    uint32_t background_mm[2], calibration_sum[2];
    unsigned calibration_count[2];
    bool calibrated[2];
    struct range_motion motion[2];
};

struct aircraft_track {
    unsigned bit;
    uint64_t landing_echo_ms; /* Zero for a gate-confirmed track. */
    uint64_t takeoff_ms;
};

struct tracking {
    uint64_t last_seen[TAG_COUNT];
    uint64_t approach_started_ms[TAG_GV1];
    unsigned seen, parked, selected, airborne, pending_entry;
    uint64_t selected_until, last_exit_ms, pending_entry_ms;
    bool unresolved, wait_air_clear;
    int rssi[TAG_COUNT][RSSI_WINDOW];
    uint64_t rssi_time[TAG_COUNT][RSSI_WINDOW];
    unsigned rssi_head[TAG_COUNT], rssi_count[TAG_COUNT];
    struct aircraft_track fifo[AIRCRAFT_CAPACITY];
    unsigned fifo_count;
    char last_event[80];
};

struct runway_status {
    enum panel_output indicator;
    bool air_range, gv_range, gv_present, gv_identified, possible_conflict;
    bool clear, collision;
    unsigned aircraft_count, air_tags, active_aircraft, advised_aircraft, taxiway_tag, landing;
    unsigned landed;
    unsigned landing_rssi_samples;
};

struct dashboard_display {
    uint64_t next_ms;
    enum panel_output indicator;
    unsigned aircraft_count, active_aircraft, advised_aircraft, taxiway_tag, landing;
    bool gv_present, gv_identified, possible_conflict;
};

struct alert_metrics {
    bool active, measured;
    uint64_t validated_ns, latency_ns, misses;
};

struct dashboard_snapshot {
    struct vehicle_state vehicle;
    struct sensor_state sensor;
    struct tracking tracks;
    struct runway_status status;
    struct alert_metrics alerts;
    uint64_t now;
};

struct console_output {
    pthread_mutex_t lock;
    pthread_cond_t changed;
    bool pending, stop;
    struct dashboard_snapshot snapshot;
};

#ifndef RUNWAY_HOST_TEST
union receive_buffer {
    uint16_t type;
    struct _pulse pulse;
    struct runway_button_message button;
    struct runway_sensor_message sensor;
    struct runway_ble_message ble;
    struct runway_link_message link;
};

static volatile sig_atomic_t stopped;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stopped = 1;
}
#endif

static int gpio_write(int fd, const char *command)
{
    size_t size = strlen(command);
    ssize_t n;
    do {
        n = write(fd, command, size);
    } while (n < 0 && errno == EINTR);
    if (n == (ssize_t)size)
        return 0;
    if (n >= 0)
        errno = EIO;
    return -1;
}

static int panel_set_output(const struct runway_panel *panel, enum panel_output output, bool active)
{
    bool high = active != (output == PANEL_BUZZER && BUZZER_ACTIVE_LOW);
    return gpio_write(panel->fd[output], high ? "on" : "off");
}

static void panel_close(struct runway_panel *panel)
{
    for (unsigned i = panel->opened; i > 0; --i) {
        panel_set_output(panel, i - 1, false);
        close(panel->fd[i - 1]);
    }
    panel->opened = 0;
    panel->lit = PANEL_OUTPUT_COUNT;
}

static void panel_open(struct runway_panel *panel)
{
    *panel = (struct runway_panel){.lit = PANEL_OUTPUT_COUNT};
    for (unsigned i = 0; i < PANEL_OUTPUT_COUNT; ++i) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/gpio/%d", panel_pins[i]);
        int fd = open(path, O_RDWR);
        if (fd >= 0) {
            panel->fd[panel->opened++] = fd;
            if (gpio_write(fd, "out") == 0 && panel_set_output(panel, i, false) == 0)
                continue;
        }
        fprintf(stderr, "Runway indicator panel unavailable at %s: %s\n", path, strerror(errno));
        panel_close(panel);
        return;
    }
}

static void panel_update(struct runway_panel *panel, enum panel_output next)
{
    if (panel->opened != PANEL_OUTPUT_COUNT || panel->lit == next)
        return;
    if ((panel->lit == PANEL_CONFLICT && panel_set_output(panel, PANEL_BUZZER, false) < 0) ||
        (panel->lit != PANEL_OUTPUT_COUNT && panel_set_output(panel, panel->lit, false) < 0) ||
        panel_set_output(panel, next, true) < 0 ||
        (next == PANEL_CONFLICT && panel_set_output(panel, PANEL_BUZZER, true) < 0)) {
        perror("Runway indicator panel update");
        panel_close(panel);
        return;
    }
    panel->lit = next;
}

static bool is_recent(uint64_t timestamp, uint64_t age, uint64_t now)
{
    return timestamp != 0 && now >= timestamp && now - timestamp <= age;
}

static bool fresh_ble(const struct tracking *t, unsigned tag, uint64_t now)
{
    return (t->seen & BIT(tag)) != 0 &&
           is_recent(t->last_seen[tag], RUNWAY_BLE_MAX_AGE_MS, now);
}

static unsigned visible_aircraft(const struct tracking *t, uint64_t now)
{
    return (fresh_ble(t, TAG_AC1, now) ? BIT(TAG_AC1) : 0u) |
           (fresh_ble(t, TAG_AC2, now) ? BIT(TAG_AC2) : 0u);
}

static bool echo_follows_ble(const struct tracking *t, unsigned tag, uint64_t echo_ms)
{
    uint64_t started_ms = t->approach_started_ms[tag];
    return started_ms != 0 && echo_ms > started_ms;
}

static unsigned runway_aircraft(const struct tracking *t)
{
    unsigned tags = 0;
    for (unsigned i = 0; i < t->fifo_count; ++i)
        tags |= t->fifo[i].bit;
    return tags;
}

static bool fresh_range(const struct runway_sensor_message *r, uint64_t now)
{
    return r->sequence != 0 && is_recent(r->timestamp_ms, RUNWAY_MAX_AGE_MS, now);
}

static int validate_button(const struct vehicle_state *s, int connection,
                           const struct runway_button_message *m, uint64_t now)
{
    if (m->version != RUNWAY_IPC_VERSION || m->event > RUNWAY_VEHICLE_EXIT)
        return EINVAL;
    if (m->timestamp_ms > now || now - m->timestamp_ms > RUNWAY_MAX_AGE_MS)
        return ETIMEDOUT;
    if (m->event == RUNWAY_BUTTONS_READY) {
        if (s->connection != -1 &&
            (s->connection != connection || m->timestamp_ms <= s->timestamp_ms))
            return EBUSY;
        return m->sequence == 0 && m->vehicle_count == 0 ? 0 : EINVAL;
    }
    if (s->connection != connection) return ENOTCONN;
    if (s->sequence == UINT64_MAX || m->sequence != s->sequence + 1 ||
        m->timestamp_ms < s->timestamp_ms)
        return EINVAL;
    /* One GV1 tag: ENTER -> 1, EXIT -> 0, no duplicate button transitions. */
    if (m->event == RUNWAY_VEHICLE_ENTER)
        return s->count == 0 && m->vehicle_count == 1 ? 0 : EINVAL;
    return s->count == 1 && m->vehicle_count == 0 ? 0 : EINVAL;
}

static void apply_button(struct vehicle_state *s, int connection,
                         const struct runway_button_message *m)
{
    if (m->event == RUNWAY_BUTTONS_READY && (s->on_runway || s->count))
        s->connection_lost = true;
    s->connection = connection;
    s->sequence = m->sequence;
    s->timestamp_ms = m->timestamp_ms;
    s->count = m->vehicle_count;
    if (m->event != RUNWAY_BUTTONS_READY) {
        s->exited = m->event == RUNWAY_VEHICLE_EXIT;
        s->leaving_ms = 0;
        if (!s->exited)
            s->left_runway = false;
        if (s->exited) {
            s->connection_lost = false;
            s->on_runway = false;
        }
    }
}

static int handle_monitor_link(struct sensor_state *sensor, struct tracking *tracks,
                               int connection, const struct runway_link_message *msg,
                               uint64_t now)
{
    if (msg->link_version != RUNWAY_SUPERVISOR_LINK_VERSION ||
        msg->timestamp_ms > now || now - msg->timestamp_ms > RUNWAY_MAX_AGE_MS)
        return EINVAL;
    if (msg->action == RUNWAY_LINK_PROBE)
        return 0;
    if (msg->action != RUNWAY_LINK_RESUME_RANGE)
        return EINVAL;
    if (sensor->connection >= 0 && sensor->connection != connection)
        return EBUSY;
    if (sensor->connection < 0) {
        sensor->connection = connection;
        sensor->range_only = true;
        tracks->unresolved = true;
        tracks->selected = 0;
        tracks->pending_entry = 0;
    }
    return 0;
}

static bool foreground_target(const struct sensor_state *s, unsigned idx, uint64_t now)
{
    const struct runway_sensor_message *r = &s->range[idx];
    return s->calibrated[idx] && fresh_range(r, now) &&
           runway_range_has_distance(r->range_state) &&
           r->distance_mm <= RUNWAY_RANGE_LIMIT_MM &&
           r->distance_mm + OBJECT_DELTA_MM < s->background_mm[idx];
}

static bool foreground_echo(const struct sensor_state *s, unsigned idx, uint64_t now)
{
    const struct runway_sensor_message *r = &s->range[idx];
    bool in_zone = idx == 0 || r->range_state == RANGE_INSIDE;
    return in_zone && foreground_target(s, idx, now);
}

static bool ir_in_progress(const struct sensor_state *s, uint64_t now)
{
    return s->ir_mask ||
           (s->ir_seen_mask && is_recent(s->ir_started_ms, IR_SEQUENCE_MS, now));
}

static void update_motion(struct sensor_state *s, unsigned idx, uint64_t now)
{
    const struct runway_sensor_message *r = &s->range[idx];
    struct range_motion *motion = &s->motion[idx];
    motion->valid = false;
    if (!foreground_target(s, idx, now)) {
        motion->outward_steps = 0;
        /* An occasional missed ping can be bridged. A background echo ends
           the foreground track so its distance cannot enter the estimate. */
        if (r->range_state != RANGE_NO_ECHO)
            motion->timestamp_ms = 0;
        return;
    }
    uint64_t elapsed_ms = r->timestamp_ms - motion->timestamp_ms;
    if (elapsed_ms < MIN_MOTION_INTERVAL_MS)
        return;
    if (motion->timestamp_ms && elapsed_ms <= MOTION_GAP_MS) {
        int delta_mm = (int)r->distance_mm - (int)motion->distance_mm;
        double velocity = delta_mm * 1000.0 / elapsed_ms;
        motion->radial_mm_per_s = velocity > -STILL_MM_PER_S &&
            velocity < STILL_MM_PER_S ? 0.0 : velocity;
        motion->valid = true;
        if (delta_mm >= DEPARTURE_STEP_MM) {
            if (motion->outward_steps < DEPARTURE_STEPS)
                ++motion->outward_steps;
        } else {
            motion->outward_steps = 0;
        }
    } else {
        motion->outward_steps = 0;
    }
    motion->timestamp_ms = r->timestamp_ms;
    motion->distance_mm = r->distance_mm;
}

static int accept_sensor(struct sensor_state *s, int connection,
                         const struct runway_sensor_message *m, uint64_t now)
{
    if (m->version != RUNWAY_IPC_VERSION || m->kind > IR_BEAMS)
        return EINVAL;
    if (m->timestamp_ms > now || now - m->timestamp_ms > RUNWAY_MAX_AGE_MS)
        return ETIMEDOUT;
    if (m->kind == SENSORS_READY) {
        if (m->sequence != 0) return EINVAL;
        if (s->connection >= 0) {
            if (s->connection != connection || m->timestamp_ms <= s->gate_time_ms ||
                m->timestamp_ms <= s->ir_time_ms ||
                m->timestamp_ms <= s->range[0].timestamp_ms ||
                m->timestamp_ms <= s->range[1].timestamp_ms)
                return EBUSY;
            /* A producer restarted behind the same fault-monitor connection.
               Keep the stationary background calibration and drop old readings. */
            struct sensor_state next = {.connection = connection, .gate_time_ms = m->timestamp_ms};
            for (unsigned i = 0; i < 2; ++i) {
                next.calibrated[i] = s->calibrated[i];
                if (s->calibrated[i])
                    next.background_mm[i] = s->background_mm[i];
            }
            *s = next;
        } else {
            *s = (struct sensor_state){.connection = connection, .gate_time_ms = m->timestamp_ms};
        }
        return 0;
    }
    if (s->connection != connection) return ENOTCONN;
    if (s->range_only && m->kind != AIRCRAFT_RANGE && m->kind != VEHICLE_RANGE)
        return ENOTCONN;
    if (m->kind == IR_BEAMS) {
        if (m->ir_mask > 3 || s->ir_sequence == UINT64_MAX ||
            m->sequence != s->ir_sequence + 1 || m->timestamp_ms < s->ir_time_ms)
            return EINVAL;
        s->ir_sequence = m->sequence;
        s->ir_time_ms = m->timestamp_ms;
        if (m->ir_mask) {
            if (!s->ir_seen_mask ||
                !is_recent(s->ir_started_ms, IR_SEQUENCE_MS, m->timestamp_ms) ||
                (!s->ir_mask && s->ir_seen_mask == m->ir_mask)) {
                s->ir_seen_mask = 0;
                s->ir_started_ms = m->timestamp_ms;
            }
            s->ir_seen_mask |= m->ir_mask;
        } else if (s->ir_seen_mask == 3u) {
            s->ir_seen_mask = 0;
        }
        s->ir_mask = m->ir_mask;
        return 0;
    }
    if (m->kind == AIRCRAFT_ENTER || m->kind == AIRCRAFT_EXIT) {
        if (s->gate_sequence == UINT64_MAX || m->sequence != s->gate_sequence + 1 ||
            m->timestamp_ms < s->gate_time_ms)
            return EINVAL;
        if (m->kind == AIRCRAFT_ENTER) {
            ++s->entries;
        } else {
            ++s->exits;
        }
        s->gate_sequence = m->sequence;
        s->gate_time_ms = m->timestamp_ms;
        s->ir_seen_mask = 0;
        s->motion[0] = (struct range_motion){0};
        return 0;
    }
    struct runway_sensor_message *last = &s->range[m->kind - AIRCRAFT_RANGE];
    if (m->sequence <= last->sequence ||
        m->timestamp_ms < last->timestamp_ms || m->range_state > RANGE_BEYOND_LIMIT ||
        m->motion_valid > 1 || m->ir_mask != 0 ||
        (!runway_range_has_distance(m->range_state) ?
          (m->distance_mm != 0 || m->motion_valid != 0) :
          (m->distance_mm < MIN_ECHO_MM || m->distance_mm > RUNWAY_RANGE_LIMIT_MM)))
        return EINVAL;
    *last = *m;
    /* Calibrate only during an empty, quiet startup. Never learn a moving
       object as the background after a runway entry. */
    unsigned idx = m->kind - AIRCRAFT_RANGE;
    if (!s->calibrated[idx] && s->aircraft_count == 0 &&
        s->entries == 0 && s->ir_mask == 0 &&
        m->range_state != RANGE_NO_ECHO) {
        /* A long echo marks the background beyond the measuring window.
           Keep a calibration bound internally, separate from raw distance. */
        uint32_t sample = m->range_state == RANGE_BEYOND_LIMIT ?
                          BEYOND_LIMIT_BACKGROUND_MM : m->distance_mm;
        if (sample < MIN_BACKGROUND_MM) {
            /* A background this close leaves no measurable foreground range. */
            s->calibration_count[idx] = 0;
            s->calibration_sum[idx] = 0;
            s->background_mm[idx] = 0;
        } else {
            if (s->calibration_count[idx] &&
                (sample > s->background_mm[idx] + BACKGROUND_TOLERANCE_MM ||
                 sample + BACKGROUND_TOLERANCE_MM < s->background_mm[idx])) {
                s->calibration_count[idx] = 0;
                s->calibration_sum[idx] = 0;
            }
            s->calibration_sum[idx] += sample;
            ++s->calibration_count[idx];
            s->background_mm[idx] = s->calibration_sum[idx] / s->calibration_count[idx];
            if (s->calibration_count[idx] >= BACKGROUND_SAMPLES)
                s->calibrated[idx] = true;
        }
    }
    update_motion(s, idx, now);
    return 0;
}

static int accept_ble(struct tracking *t, const struct runway_ble_message *m,
                      uint64_t now)
{
    if (m->version != RUNWAY_IPC_VERSION || m->tag >= TAG_COUNT ||
        m->kind > REGISTER_LANDING)
        return EINVAL;
    const uint64_t max_age = m->kind == TAG_SEEN ? RUNWAY_BLE_MAX_AGE_MS : RUNWAY_MAX_AGE_MS;
    if (m->timestamp_ms > now || now - m->timestamp_ms > max_age)
        return ETIMEDOUT;
    unsigned bit = BIT(m->tag);
    if (m->kind == TAG_SEEN) {
        if (m->rssi_dbm != 127 && (m->rssi_dbm < -127 || m->rssi_dbm > 20))
            return EINVAL;
        if ((t->seen & bit) && m->timestamp_ms <= t->last_seen[m->tag])
            return EINVAL;
        if ((bit & AIR_TAGS) && !fresh_ble(t, m->tag, m->timestamp_ms))
            t->approach_started_ms[m->tag] = m->timestamp_ms;
        t->seen |= bit;
        t->last_seen[m->tag] = m->timestamp_ms;
        if (m->rssi_dbm != 127) {
            unsigned index = t->rssi_head[m->tag];
            t->rssi[m->tag][index] = m->rssi_dbm;
            t->rssi_time[m->tag][index] = m->timestamp_ms;
            t->rssi_head[m->tag] = (index + 1) % RSSI_WINDOW;
            if (t->rssi_count[m->tag] < RSSI_WINDOW) ++t->rssi_count[m->tag];
        }
        return 0;
    }
    if (m->kind == RECONCILE_TRACKS || m->kind == REGISTER_LANDING)
        return EINVAL; /* handled by supervisor with occupancy validation */
    if (m->tag == TAG_GV1) return EINVAL;
    if (m->kind == SET_APPROACH) {
        if (runway_aircraft(t) & bit) return EBUSY;
        if (t->pending_entry == bit) t->pending_entry = 0;
        t->parked &= ~bit;
        t->airborne &= ~bit;
        t->approach_started_ms[m->tag] = fresh_ble(t, m->tag, now) ? now : 0;
        return 0;
    }
    t->selected = bit;
    t->selected_until = now + UINT64_C(15000);
    return 0;
}

static unsigned rssi_sum(const struct tracking *t, unsigned tag, uint64_t now,
                         unsigned limit, int *sum)
{
    *sum = 0;
    unsigned count = 0;
    while (count < limit && count < t->rssi_count[tag]) {
        unsigned index = (t->rssi_head[tag] + RSSI_WINDOW - 1 - count) % RSSI_WINDOW;
        if (!is_recent(t->rssi_time[tag][index], RSSI_MAX_SAMPLE_AGE_MS, now))
            break;
        *sum += t->rssi[tag][index];
        ++count;
    }
    return count;
}

static unsigned stronger_aircraft(const struct tracking *t, unsigned eligible,
                                  int margin_db, uint64_t now)
{
    if (eligible != AIR_TAGS)
        return eligible;
    int ac1, ac2;
    if (rssi_sum(t, TAG_AC1, now, RSSI_WINDOW, &ac1) != RSSI_WINDOW ||
        rssi_sum(t, TAG_AC2, now, RSSI_WINDOW, &ac2) != RSSI_WINDOW)
        return 0;
    int difference = ac1 - ac2;
    int margin = margin_db * RSSI_WINDOW;
    if (difference > 0 && difference >= margin) return BIT(TAG_AC1);
    if (difference < 0 && -difference >= margin) return BIT(TAG_AC2);
    return 0;
}

static unsigned landing_rssi_priority(const struct tracking *t, uint64_t now,
                                      unsigned *samples)
{
    int ac1, ac2;
    unsigned ac1_count = rssi_sum(t, TAG_AC1, now, RSSI_WINDOW, &ac1);
    unsigned ac2_count = rssi_sum(t, TAG_AC2, now, RSSI_WINDOW, &ac2);
    *samples = ac1_count < ac2_count ? ac1_count : ac2_count;
    if (!*samples) return 0;

    /* Compare the same number of recent samples for both aircraft. */
    if (ac1_count > *samples) rssi_sum(t, TAG_AC1, now, *samples, &ac1);
    if (ac2_count > *samples) rssi_sum(t, TAG_AC2, now, *samples, &ac2);
    if (ac1 == ac2) return 0;
    return ac1 > ac2 ? BIT(TAG_AC1) : BIT(TAG_AC2);
}

static unsigned gate_entry_candidate(const struct tracking *t, unsigned selected,
                                     uint64_t now)
{
    unsigned eligible = visible_aircraft(t, now);
    for (unsigned i = 0; i < t->fifo_count; ++i) {
        if (!t->fifo[i].landing_echo_ms)
            eligible &= ~t->fifo[i].bit; /* Already entered through the gate. */
    }
    if (selected)
        return eligible & selected;
    eligible &= ~t->airborne;
    unsigned parked = eligible & t->parked;
    return stronger_aircraft(t, parked ? parked : eligible, RSSI_MARGIN_DB, now);
}

static void assign_crossing(struct tracking *t, struct sensor_state *s,
                            unsigned kind, uint64_t crossing_ms, uint64_t now)
{
    unsigned selected = now <= t->selected_until ? t->selected : 0;
    t->selected = 0;
    t->selected_until = 0;
    for (unsigned i = 0; i < t->fifo_count; ++i)
        t->fifo[i].takeoff_ms = 0;
    if (kind == AIRCRAFT_ENTER) {
        unsigned candidate = gate_entry_candidate(t, selected, now);
        if (!candidate) {
            snprintf(t->last_event, sizeof(t->last_event),
                     "IR entry ignored: no unique BLE match");
            return;
        }
        unsigned tag = candidate == BIT(TAG_AC1) ? TAG_AC1 : TAG_AC2;
        for (unsigned i = 0; i < t->fifo_count; ++i) {
            struct aircraft_track *track = &t->fifo[i];
            if (track->bit == candidate && track->landing_echo_ms) {
                /* The gate confirmed the same aircraft US1 picked up earlier. */
                track->landing_echo_ms = 0;
                ++s->aircraft_count;
                snprintf(t->last_event, sizeof(t->last_event), "%s: taxiway to runway",
                         runway_tag_name(tag));
                return;
            }
        }
        if (t->fifo_count == AIRCRAFT_CAPACITY) {
            t->unresolved = true;
            snprintf(t->last_event, sizeof(t->last_event), "Extra IR entry: verify runway");
            return;
        }
        if (t->pending_entry && t->pending_entry != candidate) {
            snprintf(t->last_event, sizeof(t->last_event), "Gate entry still waiting for US1");
            return;
        }
        t->pending_entry = candidate;
        t->pending_entry_ms = crossing_ms;
        snprintf(t->last_event, sizeof(t->last_event), "%s: gate entry, waiting for US1",
                 runway_tag_name(tag));
        return;
    }

    t->last_exit_ms = crossing_ms;
    t->pending_entry = 0;
    t->wait_air_clear = foreground_echo(s, 0, now);
    if (!t->fifo_count) {
        snprintf(t->last_event, sizeof(t->last_event), "IR exit: no tracked aircraft");
        return;
    }
    if (selected && selected != t->fifo[0].bit) {
        t->unresolved = true;
        snprintf(t->last_event, sizeof(t->last_event), "Unmatched IR exit: verify runway");
        return;
    }
    unsigned departed = t->fifo[0].bit;
    if (!t->fifo[0].landing_echo_ms && s->aircraft_count)
        --s->aircraft_count;
    --t->fifo_count;
    if (t->fifo_count)
        t->fifo[0] = t->fifo[1];
    t->parked |= departed;
    unsigned tag = departed == BIT(TAG_AC1) ? TAG_AC1 : TAG_AC2;
    snprintf(t->last_event, sizeof(t->last_event), "%s: runway to taxiway",
             runway_tag_name(tag));
}

static void observe_air_echo(struct tracking *t, struct sensor_state *s,
                            uint64_t now)
{
    unsigned live = visible_aircraft(t, now);
    if (t->pending_entry && !(live & t->pending_entry)) {
        t->pending_entry = 0;
        t->wait_air_clear |= foreground_echo(s, 0, now);
        snprintf(t->last_event, sizeof(t->last_event), "Gate entry cancelled: BLE expired");
    }
    if (s->connection < 0 || ir_in_progress(s, now) || !fresh_range(&s->range[0], now) ||
        s->range[0].timestamp_ms <= t->last_exit_ms)
        return;
    if (!foreground_echo(s, 0, now)) {
        t->wait_air_clear = false;
        return;
    }
    if (t->wait_air_clear || t->unresolved || t->fifo_count == AIRCRAFT_CAPACITY)
        return;

    unsigned candidate = t->pending_entry;
    bool via_gate = candidate != 0;
    if (via_gate) {
        /* A hand at the gate must not be mistaken for the aircraft on US1. */
        if (s->range[0].timestamp_ms <= t->pending_entry_ms)
            return;
    } else {
        if (t->fifo_count) return;
        unsigned eligible = live & ~(t->parked | t->airborne);
        for (unsigned tag = TAG_AC1; tag <= TAG_AC2; ++tag) {
            if (!echo_follows_ble(t, tag, s->range[0].timestamp_ms))
                eligible &= ~BIT(tag);
        }
        unsigned samples;
        candidate = eligible == AIR_TAGS ? landing_rssi_priority(t, now, &samples) : eligible;
    }
    if (!candidate) return;
    t->fifo[t->fifo_count++] = (struct aircraft_track){.bit = candidate,
        .landing_echo_ms = via_gate ? 0 : s->range[0].timestamp_ms};
    if (via_gate) ++s->aircraft_count;
    t->pending_entry = 0;
    t->parked &= ~candidate;
    t->airborne &= ~candidate;
    unsigned tag = candidate == BIT(TAG_AC1) ? TAG_AC1 : TAG_AC2;
    snprintf(t->last_event, sizeof(t->last_event), "%s detected on runway (BLE + US1)",
             runway_tag_name(tag));
}

static void observe_vehicle(struct vehicle_state *v, const struct sensor_state *s,
                            uint64_t now)
{
    if (v->connection >= 0 && v->count && foreground_echo(s, 1, now)) {
        v->on_runway = true;
        v->left_runway = false;
    }
}

static bool departure_cleared(const struct sensor_state *s, unsigned idx,
                              uint64_t departing_ms, uint64_t now)
{
    const struct runway_sensor_message *r = &s->range[idx];
    if (!is_recent(departing_ms, MOTION_GAP_MS, r->timestamp_ms) ||
        foreground_echo(s, idx, now))
        return false;
    return r->range_state == RANGE_BEYOND_LIMIT ||
           (runway_range_has_distance(r->range_state) &&
            r->distance_mm >= s->motion[idx].distance_mm);
}

/* Called only on a new range report. Each ultrasonic sensor observes one target. */
static bool observe_departure(struct tracking *t, struct vehicle_state *v,
                               struct sensor_state *s, unsigned idx,
                               uint64_t now)
{
    const struct runway_sensor_message *r = &s->range[idx];
    const struct range_motion *motion = &s->motion[idx];
    bool receding = motion->valid && motion->outward_steps >= DEPARTURE_STEPS;
    bool returning = motion->valid && motion->radial_mm_per_s < 0;
    if (idx == 0) {
        if (t->fifo_count != 1 || t->unresolved || ir_in_progress(s, now))
            return false;
        struct aircraft_track *track = &t->fifo[0];
        unsigned bit = track->bit;
        const char *who = runway_tag_name(bit == BIT(TAG_AC1) ? TAG_AC1 : TAG_AC2);
        if (departure_cleared(s, idx, track->takeoff_ms, now)) {
            if (!track->landing_echo_ms && s->aircraft_count)
                --s->aircraft_count;
            t->fifo_count = 0;
            t->airborne |= bit;
            t->last_exit_ms = r->timestamp_ms;
            snprintf(t->last_event, sizeof(t->last_event), "%s taken off (model)", who);
            return true;
        }
        bool was_receding = track->takeoff_ms != 0;
        if (receding)
            track->takeoff_ms = r->timestamp_ms;
        else if (returning || !is_recent(track->takeoff_ms, MOTION_GAP_MS, r->timestamp_ms))
            track->takeoff_ms = 0;
        if (!was_receding && track->takeoff_ms)
            snprintf(t->last_event, sizeof(t->last_event), "%s takeoff detected by US1", who);
        return was_receding != (track->takeoff_ms != 0);
    }

    if (!v->on_runway)
        return false;
    bool was_leaving = v->leaving_ms != 0;
    if (receding)
        v->leaving_ms = r->timestamp_ms;
    if (departure_cleared(s, idx, v->leaving_ms, now)) {
        v->on_runway = false;
        v->left_runway = true;
        v->leaving_ms = 0;
        snprintf(t->last_event, sizeof(t->last_event), "%s left runway (US2)",
                 fresh_ble(t, TAG_GV1, now) ? "GV1" : "Unknown vehicle");
        return true;
    }
    if (returning || !is_recent(v->leaving_ms, MOTION_GAP_MS, r->timestamp_ms))
        v->leaving_ms = 0;
    if (!was_leaving && v->leaving_ms)
        snprintf(t->last_event, sizeof(t->last_event), "%s moving away (US2)",
                 fresh_ble(t, TAG_GV1, now) ? "GV1" : "Unknown vehicle");
    return was_leaving != (v->leaving_ms != 0);
}

static bool clear_inputs(const struct sensor_state *s, const struct vehicle_state *v,
                         uint64_t now)
{
    return s->connection >= 0 && !s->range_only && v->connection >= 0 && !ir_in_progress(s, now) &&
           s->calibrated[0] && s->calibrated[1] &&
           fresh_range(&s->range[0], now) && fresh_range(&s->range[1], now) &&
           !foreground_echo(s, 0, now) && !foreground_echo(s, 1, now);
}

static struct runway_status assess(const struct vehicle_state *v,
                                   const struct sensor_state *s,
                                   const struct tracking *t, uint64_t now)
{
    unsigned visible = visible_aircraft(t, now);
    struct runway_status st = {
        .air_range = foreground_echo(s, 0, now),
        .gv_range = foreground_echo(s, 1, now),
        .gv_present = v->on_runway,
        .gv_identified = fresh_ble(t, TAG_GV1, now),
        .air_tags = runway_aircraft(t),
        .active_aircraft = visible & ~(t->parked | t->airborne)
    };
    st.advised_aircraft = st.active_aircraft;
    if (st.gv_present)
        st.advised_aircraft |= (visible & t->parked) | st.air_tags;
    st.aircraft_count = t->fifo_count;
    if (st.air_range && s->ir_mask && s->ir_seen_mask == 3u &&
        is_recent(s->ir_started_ms, IR_SEQUENCE_MS, now))
        st.taxiway_tag = gate_entry_candidate(t,
            now <= t->selected_until ? t->selected : 0, now);
    for (unsigned i = 0; i < t->fifo_count; ++i) {
        if (t->fifo[i].landing_echo_ms && !t->fifo[i].takeoff_ms)
            st.landed |= t->fifo[i].bit;
    }
    st.landed &= ~st.taxiway_tag;
    st.collision = st.aircraft_count >= 2 || (st.aircraft_count && st.gv_present);
    st.possible_conflict = !st.collision &&
        ((st.aircraft_count && st.gv_range) ||
         (st.air_range && st.gv_present) ||
         (st.air_range && st.gv_range));
    st.clear = clear_inputs(s, v, now) && !t->fifo_count && !t->pending_entry && !st.gv_present &&
               !t->unresolved && !t->wait_air_clear && !v->connection_lost;
    st.indicator = st.collision ? PANEL_CONFLICT : st.clear ? PANEL_CLEAR :
        t->unresolved || t->wait_air_clear || v->connection_lost ? PANEL_VERIFY :
        st.aircraft_count || st.gv_present ? PANEL_OCCUPIED : PANEL_VERIFY;
    unsigned priority = st.active_aircraft & ~st.air_tags;
    if (priority == AIR_TAGS)
        priority = landing_rssi_priority(t, now, &st.landing_rssi_samples);
    if (st.clear)
        st.landing = priority;
    return st;
}

/* Optional operator identification of a direct landing in the tabletop demo. */
static int register_landing(struct tracking *t, const struct sensor_state *s,
                            unsigned tag, uint64_t now)
{
    if (tag >= TAG_GV1) return EINVAL;
    unsigned bit = BIT(tag);
    if (t->unresolved || t->wait_air_clear) return EBUSY;
    if (t->fifo_count || t->pending_entry) return EBUSY;
    if (!foreground_echo(s, 0, now) || !fresh_ble(t, tag, now) ||
        !echo_follows_ble(t, tag, s->range[0].timestamp_ms))
        return EAGAIN;
    t->fifo[0] = (struct aircraft_track){.bit = bit,
        .landing_echo_ms = s->range[0].timestamp_ms};
    t->fifo_count = 1;
    t->parked &= ~bit;
    t->airborne &= ~bit;
    snprintf(t->last_event, sizeof(t->last_event), "%s landing identified by operator",
             runway_tag_name(tag));
    return 0;
}

static int reconcile_tracks(struct tracking *t, const struct sensor_state *s,
                            struct vehicle_state *v, uint64_t now)
{
    if (!clear_inputs(s, v, now) || s->aircraft_count || v->count) return EBUSY;
    t->unresolved = false;
    t->wait_air_clear = false;
    t->selected = 0;
    t->pending_entry = 0;
    t->fifo_count = 0;
    v->connection_lost = false;
    v->on_runway = false;
    snprintf(t->last_event, sizeof(t->last_event), "Empty model runway reconciled");
    return 0;
}

static void append(char *output, const char *fmt, ...)
{
    size_t used = strlen(output);
    if (used >= DASHBOARD_SIZE - 1) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(output + used, DASHBOARD_SIZE - used, fmt, args);
    va_end(args);
}

static const char *warning_text(struct runway_status st)
{
    if (!st.collision)
        return st.possible_conflict ? "POSSIBLE CONFLICT / VERIFY" : "NONE";
    if (st.aircraft_count >= 2 && st.gv_present)
        return "MULTIPLE CONFLICTS";
    if (st.aircraft_count >= 2)
        return st.air_tags == AIR_TAGS ? "AC1 + AC2" : "TWO AIRCRAFT";
    if (st.air_tags == BIT(TAG_AC1))
        return st.gv_identified ? "AC1 + GV1" : "AC1 + UNKNOWN VEHICLE";
    if (st.air_tags == BIT(TAG_AC2))
        return st.gv_identified ? "AC2 + GV1" : "AC2 + UNKNOWN VEHICLE";
    return "POSSIBLE CONFLICT / VERIFY";
}

static void format_dashboard(char *out, const struct vehicle_state *v,
                             const struct sensor_state *s,
                             const struct tracking *t, struct runway_status st,
                             uint64_t now)
{
    static const char *line = "+--------------------------------------------------------------+\n";
    static const char *track_line =
        "+--------+----------+---------------+------------+-------------+\n";
    static const char *sensor_line =
        "+----------+----------+--------------+-------------------------+\n";
    const char *runway = runway_labels[st.indicator];
    unsigned shown_ble = st.active_aircraft;
    shown_ble |= visible_aircraft(t, now) & t->parked;
    unsigned ble_count = !!(shown_ble & BIT(TAG_AC1)) + !!(shown_ble & BIT(TAG_AC2)) +
                         (!v->exited && st.gv_identified);
    char summary[56];
    snprintf(summary, sizeof(summary), "Aircraft: %u/2   Vehicle: %u/1   BLE: %u/3",
             st.aircraft_count, (unsigned)st.gv_present, ble_count);
    append(out, "%s| %-60s |\n%s", line, "RUNWAYSENTINEL | QNX RTOS | MODEL RUNWAY", line);
    append(out, "| RUNWAY : %-51s |\n| %-60s |\n%s", runway, summary, line);
    append(out, "| %-60s |\n%s", "OBJECT TRACKING", track_line);
    append(out, "| Object | BLE      | Location      | RSSI Avg   | Status      |\n%s", track_line);
    for (unsigned tag = 0; tag < TAG_COUNT; ++tag) {
        unsigned bit = BIT(tag);
        bool vehicle = tag == TAG_GV1;
        bool taxiway = (st.taxiway_tag & bit) != 0;
        bool parked = vehicle ? v->exited : (t->parked & bit) != 0 && !taxiway;
        bool waiting_us1 = !vehicle && t->pending_entry == bit;
        bool present = vehicle ? st.gv_present : (st.air_tags & bit) != 0;
        bool takeoff = !vehicle && present && t->fifo_count == 1 && t->fifo[0].takeoff_ms;
        bool seen = fresh_ble(t, tag, now);
        bool unknown_vehicle = vehicle && (v->count || present) && !seen;
        const char *ble = vehicle && parked ? "Hidden" : !seen ? "Not seen" :
                          present ? "Active" : "Detected";
        const char *loc = vehicle ? "Unknown" : "--";
        const char *state = "NOT TRACKED";
        if (taxiway) {
            loc = "Taxiway";
            state = "CROSSING";
        } else if (!vehicle && !waiting_us1 && (t->airborne & bit)) {
            loc = "Airborne";
            state = "TAKEN OFF";
        } else if (takeoff) {
            loc = "Departing";
            state = "TAKEN OFF";
        } else if (vehicle && v->left_runway) {
            loc = "Outside";
            state = "LEFT RUNWAY";
        } else if (present) {
            loc = "Runway";
            state = (st.landed & bit) ? "LANDED" :
                    (vehicle ? st.gv_range : st.air_range && seen) ? "OCCUPIED" : "TRACKED";
            if (vehicle && v->leaving_ms)
                state = "LEAVING";
        } else if (vehicle && (st.gv_range || v->connection_lost ||
                              !s->calibrated[1] || !fresh_range(&s->range[1], now))) {
            state = "VERIFY";
        } else if (vehicle && v->count) {
            loc = "Side Road";
            state = "WAITING";
        } else if (vehicle || parked) {
            loc = vehicle ? "Stop Area" : "Taxiway";
            state = vehicle ? "PARKED" : "WAITING";
        } else if (st.active_aircraft & bit) {
            state = "WAIT US1";
        }
        if (waiting_us1) state = "WAIT US1";
        if (unknown_vehicle && !v->leaving_ms && !v->left_runway)
            state = "UNKNOWN";
        int sum;
        char rssi[16] = "--";
        unsigned limit = !vehicle && st.landing_rssi_samples ?
                         st.landing_rssi_samples : RSSI_WINDOW;
        unsigned count = vehicle && parked ? 0 : rssi_sum(t, tag, now, limit, &sum);
        if (count)
            snprintf(rssi, sizeof(rssi), "%.1f dBm", (double)sum / count);
        append(out, "| %-6s | %-8s | %-13s | %-10s | %-11s |\n",
               unknown_vehicle ? "UNK GV" : runway_tag_name(tag), ble, loc, rssi, state);
    }
    append(out, "%s", track_line);
    if (st.landed) {
        for (unsigned tag = TAG_AC1; tag <= TAG_AC2; ++tag) {
            if (st.landed & BIT(tag))
                append(out, "| LANDING STATUS : %s is landed%30s |\n",
                       runway_tag_name(tag), "");
        }
        append(out, "%s", line);
    }
    append(out, "| %-60s |\n%s", "SENSOR MONITORING (0-200 cm)", sensor_line);
    append(out, "| Sensor   | Distance | Speed        | Direction               |\n%s", sensor_line);
    const char *sensor_row = "| %-8s | %-8s | %-12s | %-23s |\n";
    for (unsigned i = 0; i < 2; ++i) {
        const struct runway_sensor_message *r = &s->range[i];
        const struct range_motion *motion = &s->motion[i];
        const char *label = i == 0 ? "Aircraft" : "Vehicle";
        char distance[16] = "--", speed[16] = "--", cal_progress[32];
        const char *direction = "UNKNOWN";
        bool fresh = fresh_range(r, now);
        bool valid_echo = fresh && runway_range_has_distance(r->range_state);
        if (fresh && (r->range_state == RANGE_BEYOND_LIMIT ||
                      (valid_echo && r->distance_mm > RUNWAY_RANGE_LIMIT_MM))) {
            append(out, sensor_row, label, "--", "--", "--");
            continue;
        }
        if (valid_echo)
            snprintf(distance, sizeof(distance), "%.1f cm", r->distance_mm / 10.0);
        if (!s->calibrated[i]) {
            if (s->entries || s->aircraft_count) {
                direction = "CAL LOCK: IR ENTRY";
            } else if (s->ir_mask) {
                direction = "CAL: CLEAR IR BEAMS";
            } else if (!fresh) {
                direction = r->sequence ? "CAL: DATA STALE" : "CAL: WAIT FOR DATA";
            } else if (!valid_echo) {
                direction = "CAL: NO ECHO";
            } else if (r->distance_mm < MIN_BACKGROUND_MM) {
                direction = "CAL: ECHO TOO NEAR";
            } else {
                snprintf(cal_progress, sizeof(cal_progress), "CAL: %u/%u ECHOES",
                         s->calibration_count[i], BACKGROUND_SAMPLES);
                direction = cal_progress;
            }
        } else if (valid_echo) {
            if (!foreground_target(s, i, now)) {
                direction = "BACKGROUND";
            } else if (motion->valid) {
                double velocity = motion->radial_mm_per_s;
                direction = velocity < 0 ? "NEARER" : velocity > 0 ? "FARTHER" : "STILL";
                double magnitude = velocity < 0 ? -velocity : velocity;
                snprintf(speed, sizeof(speed), "%.1f cm/s", magnitude / 10.0);
            } else {
                direction = "MEASURING";
            }
        }
        append(out, sensor_row, label, distance, speed, direction);
    }
    append(out, "%s", sensor_line);
    if (st.advised_aircraft) {
        append(out, "| %-60s |\n%s",
               st.gv_present ? "RUNWAY ENTRY ADVISORY" : "LANDING ADVISORY", line);
        for (unsigned tag = TAG_AC1; tag <= TAG_AC2; ++tag) {
            unsigned bit = BIT(tag);
            if (!(st.advised_aircraft & bit)) continue;
            const char *advice = st.gv_present ? "DO NOT ENTER RUNWAY NOW" :
                st.collision || (st.taxiway_tag & bit) ? "DO NOT LAND" :
                (st.air_tags & bit) ? "RUNWAY OCCUPIED" :
                !st.clear ? "DO NOT LAND" :
                (st.landing & bit) ? "MAY LAND (DEMO)" :
                st.landing ? "DO NOT LAND" :
                st.landing_rssi_samples ? "HOLD - RSSI TIE" : "HOLD - WAIT FOR RSSI";
            append(out, "| %-3s : %-54s |\n", runway_tag_name(tag), advice);
        }
        append(out, "%s", line);
    }
    append(out, "| COLLISION WARNING : %-40s |\n%s", warning_text(st), line);
    append(out, "| LAST EVENT : %-47.47s |\n%s",
           t->last_event[0] ? t->last_event : "Waiting for sensor events", line);
}

static uint64_t alert_clock_ns(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
        return 0;
    return (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
}

static void update_alert(struct alert_metrics *metrics, struct runway_panel *panel,
                         struct runway_status status)
{
    if (status.collision && !metrics->active)
        metrics->validated_ns = alert_clock_ns();
    if (!status.collision && metrics->active && metrics->validated_ns) {
        /* Panel activation was never confirmed during this conflict. */
        ++metrics->misses;
        metrics->validated_ns = 0;
    }
    metrics->active = status.collision;
    panel_update(panel, status.indicator);
    if (status.collision && metrics->validated_ns && panel->lit == PANEL_CONFLICT) {
        uint64_t now = alert_clock_ns();
        if (now >= metrics->validated_ns) {
            metrics->latency_ns = now - metrics->validated_ns;
            metrics->measured = true;
            if (metrics->latency_ns > ALERT_DEADLINE_NS)
                ++metrics->misses;
            metrics->validated_ns = 0;
        }
    } else if (status.collision && metrics->validated_ns) {
        uint64_t now = alert_clock_ns();
        if (now > metrics->validated_ns && now - metrics->validated_ns > ALERT_DEADLINE_NS) {
            ++metrics->misses;
            metrics->validated_ns = 0;
        }
    }
}

#ifndef RUNWAY_HOST_TEST
static void *console_worker(void *arg)
{
    struct console_output *console = arg;
    runway_schedule("supervisor console", SCHED_RR, RUNWAY_PRIO_ROUTINE);
    for (;;) {
        pthread_mutex_lock(&console->lock);
        while (!console->pending && !console->stop)
            pthread_cond_wait(&console->changed, &console->lock);
        if (console->stop && !console->pending) {
            pthread_mutex_unlock(&console->lock);
            return NULL;
        }
        struct dashboard_snapshot snap = console->snapshot;
        console->pending = false;
        pthread_mutex_unlock(&console->lock);
        char output[DASHBOARD_SIZE] = "";
        format_dashboard(output, &snap.vehicle, &snap.sensor, &snap.tracks, snap.status, snap.now);
        if (snap.alerts.measured)
            append(output, "Alert latency: %.3f ms | Deadline misses: %" PRIu64 "\n",
                   snap.alerts.latency_ns / 1000000.0, snap.alerts.misses);
        else
            append(output, "Alert latency: -- | Deadline misses: %" PRIu64 "\n", snap.alerts.misses);
        if (isatty(STDOUT_FILENO))
            fputs("\033[H\033[J", stdout);
        fputs(output, stdout);
        fflush(stdout);
    }
}
#endif

static void refresh_dashboard(const struct vehicle_state *v,
                              const struct sensor_state *s,
                              const struct tracking *t,
                              struct dashboard_display *d,
                              struct runway_panel *panel, struct alert_metrics *alerts,
                              struct console_output *console, bool force)
{
    uint64_t now;
    if (runway_monotonic_ms(&now) != 0) return;
    struct runway_status st = assess(v, s, t, now);
    update_alert(alerts, panel, st);
    bool changed = st.indicator != d->indicator ||
                   st.active_aircraft != d->active_aircraft ||
                   st.advised_aircraft != d->advised_aircraft ||
                   st.taxiway_tag != d->taxiway_tag || st.gv_identified != d->gv_identified ||
                   st.aircraft_count != d->aircraft_count || st.landing != d->landing ||
                   st.gv_present != d->gv_present ||
                   st.possible_conflict != d->possible_conflict;
    if (!force && !changed && now < d->next_ms) return;
    *d = (struct dashboard_display){.next_ms = now + DASHBOARD_PERIOD_MS,
        .indicator = st.indicator,
        .aircraft_count = st.aircraft_count, .active_aircraft = st.active_aircraft,
        .advised_aircraft = st.advised_aircraft,
        .taxiway_tag = st.taxiway_tag, .gv_identified = st.gv_identified,
        .landing = st.landing,
        .gv_present = st.gv_present,
        .possible_conflict = st.possible_conflict};
    pthread_mutex_lock(&console->lock);
    console->snapshot = (struct dashboard_snapshot){
        .vehicle = *v, .sensor = *s, .tracks = *t, .status = st, .alerts = *alerts, .now = now
    };
    console->pending = true;
    pthread_cond_signal(&console->changed);
    pthread_mutex_unlock(&console->lock);
}

#ifndef RUNWAY_HOST_TEST
int main(int argc, char **argv)
{
    bool parked_start = argc == 2 && strcmp(argv[1], "--parked-start") == 0;
    if (argc != 1 && !parked_start) {
        fprintf(stderr, "Usage: %s [--parked-start]\n", argv[0]);
        return EXIT_FAILURE;
    }
    struct sigaction action = {.sa_handler = request_stop};
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) < 0 || sigaction(SIGTERM, &action, NULL) < 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    name_attach_t *server = name_attach(NULL, RUNWAY_CHANNEL, 0);
    if (server == NULL) { perror("name_attach runway_supervisor"); return EXIT_FAILURE; }

    struct vehicle_state vehicle = {.connection = -1};
    struct sensor_state sensor = {.connection = -1};
    /* Accept the legacy launch argument while starting with unknown aircraft locations. */
    struct tracking tracks = {0};
    struct dashboard_display display = {0};
    struct alert_metrics alerts = {0};
    struct console_output console = {
        .lock = PTHREAD_MUTEX_INITIALIZER, .changed = PTHREAD_COND_INITIALIZER
    };
    pthread_t output_thread;
    int thread_error = pthread_create(&output_thread, NULL, console_worker, &console);
    if (thread_error != 0) {
        fprintf(stderr, "Start supervisor console: %s\n", strerror(thread_error));
        name_detach(server, 0);
        return EXIT_FAILURE;
    }
    runway_schedule("supervisor", SCHED_FIFO, RUNWAY_PRIO_CRITICAL);
    struct runway_panel panel;
    panel_open(&panel);
    const uint64_t poll_ns = UINT64_C(250000000);
    refresh_dashboard(&vehicle, &sensor, &tracks, &display, &panel, &alerts, &console, true);

    while (!stopped) {
        union receive_buffer input = {0};
        struct _msg_info info;
        if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL,
                         &poll_ns, NULL) == -1) {
            perror("TimerTimeout"); break;
        }
        int rcvid = MsgReceive(server->chid, &input, sizeof(input), &info);
        if (rcvid == -1) {
            if (errno == ETIMEDOUT) {
                refresh_dashboard(&vehicle, &sensor, &tracks, &display, &panel, &alerts, &console, false);
                continue;
            }
            if (errno == EINTR) continue;
            perror("MsgReceive"); break;
        }
        if (rcvid == 0) {
            if (input.pulse.code == _PULSE_CODE_DISCONNECT) {
                ConnectDetach(input.pulse.scoid);
                if (vehicle.connection == input.pulse.scoid) {
                    vehicle.connection = -1;
                    vehicle.connection_lost = vehicle.on_runway || vehicle.count != 0;
                }
                if (sensor.connection == input.pulse.scoid) {
                    sensor = (struct sensor_state){.connection = -1};
                    tracks.selected = 0;
                    tracks.pending_entry = 0;
                    tracks.unresolved = true;
                }
                refresh_dashboard(&vehicle, &sensor, &tracks, &display, &panel, &alerts, &console, true);
            }
            continue;
        }
        if (info.msglen < (ssize_t)sizeof(input.type)) {
            MsgError(rcvid, EINVAL); continue;
        }
        if (input.type == _IO_CONNECT) {
            MsgReply(rcvid, 0, NULL, 0); continue;
        }
        uint64_t now;
        if (runway_monotonic_ms(&now) == -1) {
            MsgError(rcvid, errno); continue;
        }
        int result = ENOSYS;
        bool force = false;
        if (input.type == RUNWAY_SUPERVISOR_LINK) {
            if (info.srcmsglen != (ssize_t)sizeof(input.link) ||
                info.msglen < (ssize_t)sizeof(input.link)) {
                MsgError(rcvid, EINVAL);
                continue;
            }
            result = handle_monitor_link(&sensor, &tracks, info.scoid, &input.link, now);
            if (result == 0)
                MsgReply(rcvid, 0, NULL, 0);
            else
                MsgError(rcvid, result);
            continue;
        }
        if (input.type == RUNWAY_BUTTON_MESSAGE) {
            if (info.srcmsglen != (ssize_t)sizeof(input.button) ||
                info.msglen < (ssize_t)sizeof(input.button)) {
                MsgError(rcvid, EINVAL); continue;
            }
            result = validate_button(&vehicle, info.scoid, &input.button, now);
            if (result == 0) {
                apply_button(&vehicle, info.scoid, &input.button);
                if (input.button.event == RUNWAY_VEHICLE_ENTER)
                    sensor.motion[1] = (struct range_motion){0};
                if (input.button.event != RUNWAY_BUTTONS_READY) {
                    const char *who = fresh_ble(&tracks, TAG_GV1, now) ? "GV1" : "Unknown vehicle";
                    const char *action = input.button.event == RUNWAY_VEHICLE_ENTER ? "entry" : "exit";
                    snprintf(tracks.last_event, sizeof(tracks.last_event), "%s %s button pressed",
                             who, action);
                }
                force = input.button.event != RUNWAY_BUTTONS_READY;
            }
        } else if (input.type == RUNWAY_SENSOR_MESSAGE) {
            if (info.srcmsglen != (ssize_t)sizeof(input.sensor) ||
                info.msglen < (ssize_t)sizeof(input.sensor)) {
                MsgError(rcvid, EINVAL); continue;
            }
            const bool sensor_restarted = input.sensor.kind == SENSORS_READY && sensor.connection >= 0;
            result = accept_sensor(&sensor, info.scoid, &input.sensor, now);
            if (result == 0 && sensor_restarted) {
                tracks.selected = 0;
                tracks.pending_entry = 0;
                tracks.unresolved = true;
                force = true;
            }
            if (result == 0 && (input.sensor.kind == AIRCRAFT_ENTER ||
                                input.sensor.kind == AIRCRAFT_EXIT)) {
                assign_crossing(&tracks, &sensor, input.sensor.kind,
                                input.sensor.timestamp_ms, now);
                force = true;
            }
            if (result == 0 && input.sensor.kind == IR_BEAMS) force = true;
        } else if (input.type == RUNWAY_BLE_MESSAGE) {
            if (info.srcmsglen != (ssize_t)sizeof(input.ble) ||
                info.msglen < (ssize_t)sizeof(input.ble)) {
                MsgError(rcvid, EINVAL); continue;
            }
            if (input.ble.version != RUNWAY_IPC_VERSION ||
                input.ble.timestamp_ms > now || now - input.ble.timestamp_ms >
                    (input.ble.kind == TAG_SEEN ? RUNWAY_BLE_MAX_AGE_MS : RUNWAY_MAX_AGE_MS)) {
                result = EINVAL;
            } else if (input.ble.kind == RECONCILE_TRACKS) {
                result = reconcile_tracks(&tracks, &sensor, &vehicle, now);
            } else if (input.ble.kind == REGISTER_LANDING) {
                /* Explicit model operator confirmation of a direct landing.
                   Never infer this event merely from BLE proximity. */
                result = register_landing(&tracks, &sensor, input.ble.tag, now);
            } else {
                result = accept_ble(&tracks, &input.ble, now);
            }
            force = result == 0 && input.ble.kind != TAG_SEEN;
        }
        if (result == 0) {
            observe_air_echo(&tracks, &sensor, now);
            bool gv_was_on_runway = vehicle.on_runway;
            observe_vehicle(&vehicle, &sensor, now);
            if (!gv_was_on_runway && vehicle.on_runway)
                snprintf(tracks.last_event, sizeof(tracks.last_event), "%s",
                         fresh_ble(&tracks, TAG_GV1, now) ?
                         "GV1 entered runway (button + US2)" : "Unknown vehicle entered runway");
            if (input.type == RUNWAY_SENSOR_MESSAGE &&
                (input.sensor.kind == AIRCRAFT_RANGE || input.sensor.kind == VEHICLE_RANGE))
                force |= observe_departure(&tracks, &vehicle, &sensor,
                                           input.sensor.kind == VEHICLE_RANGE, now);
            refresh_dashboard(&vehicle, &sensor, &tracks, &display, &panel, &alerts, &console, force);
            MsgReply(rcvid, 0, NULL, 0);
        } else MsgError(rcvid, result);
    }
    panel_close(&panel);
    name_detach(server, 0);
    pthread_mutex_lock(&console.lock);
    console.stop = true;
    pthread_cond_signal(&console.changed);
    pthread_mutex_unlock(&console.lock);
    pthread_join(output_thread, NULL);
    pthread_cond_destroy(&console.changed);
    pthread_mutex_destroy(&console.lock);
    return stopped ? EXIT_SUCCESS : EXIT_FAILURE;
}
#endif
