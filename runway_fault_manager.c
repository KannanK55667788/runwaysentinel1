#define _POSIX_C_SOURCE 200809L
#define _QNX_SOURCE 1

#include "runway_fault_ipc.h"
#include "runway_schedule.h"

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#include <unistd.h>

enum stream {
    SENSOR_READY_STREAM, CROSSING_STREAM, AIR_RANGE_STREAM, GV_RANGE_STREAM,
    IR_STREAM, BUTTON_READY_STREAM, BUTTON_STREAM,
    BLE_FIRST_STREAM, BLE_LAST_STREAM = BLE_FIRST_STREAM + TAG_COUNT - 1,
    STREAM_COUNT, NO_STREAM = STREAM_COUNT
};

enum decision {
    ACCEPT, DROP_INVALID, DROP_FUTURE, DROP_STALE, DROP_DUPLICATE,
    DROP_OUT_OF_ORDER, DROP_SESSION, IDENTITY_CONFLICT,
    DROP_QUEUE_FULL, DROP_STALE_QUEUED, FORWARD_FAILED, DECISION_COUNT
};

enum { QUEUE_CAPACITY = 64, EVENT_RESERVE = 16, SENSOR_MIN_MM = 20 };

static const char *const stream_names[] = {
    "sensor-ready", "crossing", "air-range", "vehicle-range", "ir",
    "button-ready", "button", "ble-ac1", "ble-ac2", "ble-gv1"
};

static const char *const decision_names[] = {
    "ACCEPT", "INVALID", "FUTURE", "STALE", "DUPLICATE", "OUT_OF_ORDER",
    "SESSION", "IDENTITY_CONFLICT", "QUEUE_FULL", "STALE_QUEUED", "FORWARD_FAILED"
};

union wire_message {
    uint16_t type;
    struct _pulse pulse;
    struct runway_button_message button;
    struct runway_sensor_message sensor;
    struct runway_ble_message ble;
    struct runway_fault_ble_observation ble_observation;
    struct runway_fault_test_command test;
    struct runway_heartbeat_message heartbeat;
};

struct report {
    union wire_message wire;
    size_t size;
    enum stream stream;
    uint64_t stamp_ms;
    uint64_t max_age_ms;
    uint64_t sequence;
    bool periodic;
};

struct watermark {
    bool seen;
    uint64_t timestamp_ms;
    uint64_t sequence;
    pid_t pid;
    union wire_message wire;
};

struct tag_address {
    bool configured;
    uint8_t type;
    uint8_t bytes[6];
};

struct state {
    pthread_mutex_t lock;
    pthread_cond_t queued;
    bool stop;
    bool details, test_enabled, test_replay_next_button;
    uint32_t test_delay_ms;
    uint64_t test_due_ms;
    union wire_message test_delayed;
    pid_t test_pid;
    struct report queue[QUEUE_CAPACITY];
    size_t count;
    struct watermark last[STREAM_COUNT];
    struct tag_address tags[TAG_COUNT];
    uint64_t decisions[DECISION_COUNT];
    uint64_t rejected[STREAM_COUNT + 1];
    uint64_t heartbeats[HEARTBEAT_COUNT];
    uint64_t forwarding_since_ms;
    int last_forward_error;
    enum stream last_forward_stream;
    const char *last_forward_stage;
    uint64_t forwarded;
    uint64_t coalesced;
    bool event_loss;
    enum decision last_drop;
    enum stream last_drop_stream;
    uint64_t last_drop_sequence;
    uint64_t last_drop_capture_ms;
    uint64_t last_drop_age_ms;
};

static volatile sig_atomic_t stopping;

static void stop_signal(int signo)
{
    (void)signo;
    stopping = 1;
}

static bool valid_range(const struct runway_sensor_message *msg)
{
    if (msg->ir_mask != 0 || msg->motion_valid > 1 ||
        (!msg->motion_valid && msg->radial_mm_per_s != 0))
        return false;
    if (runway_range_has_distance(msg->range_state))
        return msg->distance_mm >= SENSOR_MIN_MM &&
               msg->distance_mm <= RUNWAY_RANGE_LIMIT_MM;
    return (msg->range_state == RANGE_NO_ECHO ||
            msg->range_state == RANGE_BEYOND_LIMIT) &&
           msg->distance_mm == 0 && msg->motion_valid == 0;
}

static bool decode_sensor(const struct runway_sensor_message *msg,
                          struct report *report)
{
    if (msg->version != RUNWAY_IPC_VERSION || msg->kind > IR_BEAMS)
        return false;
    static const enum stream streams[] = {
        SENSOR_READY_STREAM, CROSSING_STREAM, CROSSING_STREAM,
        AIR_RANGE_STREAM, GV_RANGE_STREAM, IR_STREAM
    };
    report->stream = streams[msg->kind];
    report->stamp_ms = msg->timestamp_ms;
    report->sequence = msg->sequence;
    report->periodic = msg->kind == AIRCRAFT_RANGE || msg->kind == VEHICLE_RANGE;
    if ((msg->sequence == 0) != (msg->kind == SENSORS_READY))
        return false;
    if (report->periodic)
        return valid_range(msg);
    return msg->range_state == RANGE_NO_ECHO && msg->distance_mm == 0 &&
           msg->motion_valid == 0 && msg->radial_mm_per_s == 0 &&
           (msg->kind == IR_BEAMS ? msg->ir_mask <= 3 : msg->ir_mask == 0);
}

static bool decode_button(const struct runway_button_message *msg,
                          struct report *report)
{
    report->stream = msg->event == RUNWAY_BUTTONS_READY ? BUTTON_READY_STREAM : BUTTON_STREAM;
    report->stamp_ms = msg->timestamp_ms;
    report->sequence = msg->sequence;
    return msg->version == RUNWAY_IPC_VERSION && msg->event <= RUNWAY_VEHICLE_EXIT &&
           ((msg->sequence == 0) == (msg->event == RUNWAY_BUTTONS_READY)) &&
           msg->vehicle_count == (uint64_t)(msg->event == RUNWAY_VEHICLE_ENTER);
}

static bool valid_rssi(int32_t rssi)
{
    return rssi == 127 || (rssi >= -127 && rssi <= 20);
}

static bool decode_ble(const struct runway_ble_message *msg, struct report *report)
{
    if (msg->version != RUNWAY_IPC_VERSION || msg->kind != TAG_SEEN ||
        msg->tag >= TAG_COUNT || !valid_rssi(msg->rssi_dbm))
        return false;
    report->stamp_ms = msg->timestamp_ms;
    report->max_age_ms = RUNWAY_BLE_MAX_AGE_MS;
    report->periodic = true;
    report->stream = BLE_FIRST_STREAM + msg->tag;
    return true;
}

static enum decision decode(const struct state *state, const union wire_message *wire,
                            size_t size, struct report *report)
{
    *report = (struct report){
        .wire = *wire, .size = size, .stream = NO_STREAM,
        .max_age_ms = RUNWAY_MAX_AGE_MS
    };
    switch (wire->type) {
    case RUNWAY_SENSOR_MESSAGE:
        return size == sizeof(wire->sensor) && decode_sensor(&wire->sensor, report)
            ? ACCEPT : DROP_INVALID;
    case RUNWAY_BUTTON_MESSAGE:
        return size == sizeof(wire->button) && decode_button(&wire->button, report)
            ? ACCEPT : DROP_INVALID;
    case RUNWAY_BLE_MESSAGE:
        if (size != sizeof(wire->ble) || !decode_ble(&wire->ble, report))
            return DROP_INVALID;
        /* A configured tag requires address-bearing input; legacy control commands still work. */
        if (report->periodic && state->tags[wire->ble.tag].configured)
            return IDENTITY_CONFLICT;
        return ACCEPT;
    case RUNWAY_FAULT_BLE_OBSERVATION: {
        const struct runway_fault_ble_observation *obs = &wire->ble_observation;
        if (size != sizeof(*obs) || obs->address_type > 1 ||
            obs->reserved != 0 || obs->sequence == 0)
            return DROP_INVALID;
        struct runway_ble_message ble = {
            .type = RUNWAY_BLE_MESSAGE, .version = obs->version, .kind = TAG_SEEN,
            .tag = obs->tag, .timestamp_ms = obs->timestamp_ms, .rssi_dbm = obs->rssi_dbm
        };
        if (!decode_ble(&ble, report))
            return DROP_INVALID;
        report->sequence = obs->sequence;
        const struct tag_address *tag = &state->tags[obs->tag];
        if (tag->configured &&
            (tag->type != obs->address_type || memcmp(tag->bytes, obs->address, 6) != 0))
            return IDENTITY_CONFLICT;
        report->wire.ble = ble;
        report->size = sizeof(ble);
        return ACCEPT;
    }
    default:
        return DROP_INVALID;
    }
}

static enum decision check_age(const struct report *report, uint64_t now)
{
    if (report->stamp_ms > now)
        return DROP_FUTURE;
    return now - report->stamp_ms > report->max_age_ms ? DROP_STALE : ACCEPT;
}

static bool is_ready(enum stream stream)
{
    return stream == SENSOR_READY_STREAM || stream == BUTTON_READY_STREAM;
}

static enum decision check_order(const struct state *state, const struct report *report,
                                 pid_t pid)
{
    enum stream stream = report->stream;
    const struct watermark *last = &state->last[stream];
    if (stream >= CROSSING_STREAM && stream <= IR_STREAM) {
        const struct watermark *ready = &state->last[SENSOR_READY_STREAM];
        if (!ready->seen || ready->pid != pid || report->stamp_ms < ready->timestamp_ms)
            return DROP_SESSION;
    } else if (stream == BUTTON_STREAM) {
        const struct watermark *ready = &state->last[BUTTON_READY_STREAM];
        if (!ready->seen || ready->pid != pid || report->stamp_ms < ready->timestamp_ms)
            return DROP_SESSION;
    }

    if (!last->seen)
        return ACCEPT;
    if (is_ready(stream)) {
        if (last->pid == pid)
            return DROP_DUPLICATE;
        return report->stamp_ms > last->timestamp_ms ? ACCEPT : DROP_OUT_OF_ORDER;
    }
    if (report->stamp_ms < last->timestamp_ms)
        return DROP_OUT_OF_ORDER;
    if (report->sequence == 0 || last->pid != pid || last->sequence == 0)
        return report->stamp_ms == last->timestamp_ms ? DROP_DUPLICATE : ACCEPT;
    if (report->sequence == last->sequence)
        return DROP_DUPLICATE;
    return report->sequence < last->sequence ? DROP_OUT_OF_ORDER : ACCEPT;
}

static void record_decision(struct state *state, enum decision value,
                            const struct report *report, uint64_t now)
{
    ++state->decisions[value];
    if (value == ACCEPT)
        return;
    if (value != FORWARD_FAILED) {
        ++state->rejected[report->stream];
        state->last_drop = value;
        state->last_drop_stream = report->stream;
        state->last_drop_sequence = report->sequence;
        state->last_drop_capture_ms = report->stamp_ms;
        state->last_drop_age_ms = now >= report->stamp_ms ? now - report->stamp_ms : 0;
    }
    if (report->stream != NO_STREAM && !report->periodic &&
        (value == DROP_QUEUE_FULL || value == DROP_STALE || value == DROP_STALE_QUEUED ||
         value == FORWARD_FAILED))
        state->event_loss = true;
}

/* Only coalesce the trailing run of periodic reports. An event is an order barrier. */
static bool enqueue(struct state *state, const struct report *report)
{
    if (report->periodic) {
        bool replaced = false;
        for (size_t end = state->count; end > 0 && state->queue[end - 1].periodic; --end) {
            if (state->queue[end - 1].stream == report->stream) {
                memmove(&state->queue[end - 1], &state->queue[end],
                        (state->count - end) * sizeof(state->queue[0]));
                --state->count;
                ++state->coalesced;
                replaced = true;
                break;
            }
        }
        if (!replaced && state->count >= QUEUE_CAPACITY - EVENT_RESERVE)
            return false;
    } else if (state->count == QUEUE_CAPACITY) {
        return false;
    }
    state->queue[state->count++] = *report;
    pthread_cond_signal(&state->queued);
    return true;
}

static enum decision accept_report(struct state *state, const union wire_message *wire,
                          size_t size, pid_t pid, uint64_t now)
{
    struct report report;
    enum decision result = decode(state, wire, size, &report);
    if (result == ACCEPT)
        result = check_age(&report, now);
    pthread_mutex_lock(&state->lock);
    if (result == ACCEPT)
        result = check_order(state, &report, pid);
    if (result == ACCEPT && !enqueue(state, &report))
        result = DROP_QUEUE_FULL;
    if (result == ACCEPT) {
        if (report.stream == SENSOR_READY_STREAM)
            memset(&state->last[CROSSING_STREAM], 0,
                   (IR_STREAM - CROSSING_STREAM + 1) * sizeof(state->last[0]));
        else if (report.stream == BUTTON_READY_STREAM)
            state->last[BUTTON_STREAM] = (struct watermark){0};
        state->last[report.stream] = (struct watermark){
            .seen = true, .timestamp_ms = report.stamp_ms,
            .sequence = report.sequence, .pid = pid, .wire = report.wire
        };
    }
    record_decision(state, result, &report, now);
    pthread_mutex_unlock(&state->lock);
    return result;
}

static void print_input(const struct watermark last[], enum stream stream,
                        const char *label, uint64_t now, uint64_t rejected)
{
    const struct watermark *sample = &last[stream];
    const bool range = stream == AIR_RANGE_STREAM || stream == GV_RANGE_STREAM;
    const bool ble = stream >= BLE_FIRST_STREAM && stream <= BLE_LAST_STREAM;
    const uint64_t age = sample->seen && now >= sample->timestamp_ms
        ? now - sample->timestamp_ms : 0;
    char observation[32] = "--";
    char age_text[24] = "--";
    const char *status = "Waiting for report";

    if (range) {
        const struct watermark *ready = &last[SENSOR_READY_STREAM];
        const bool overdue = sample->seen ? age > RUNWAY_MAX_AGE_MS
            : ready->seen && now > ready->timestamp_ms &&
              now - ready->timestamp_ms > RUNWAY_MAX_AGE_MS;
        if (sample->seen) {
            const struct runway_sensor_message *msg = &sample->wire.sensor;
            if (runway_range_has_distance(msg->range_state))
                snprintf(observation, sizeof(observation), "%u.%u cm",
                         (unsigned)(msg->distance_mm / 10), (unsigned)(msg->distance_mm % 10));
            else if (msg->range_state == RANGE_NO_ECHO)
                strcpy(observation, "No echo");
            status = overdue ? "Report late" : "Fresh";
        } else if (overdue) {
            status = "Report late";
        }
    } else if (ble) {
        const bool recent = sample->seen && age <= RUNWAY_BLE_MAX_AGE_MS;
        status = recent ? "Recently seen" : "Not seen";
        if (sample->seen) {
            if (sample->wire.ble.rssi_dbm == 127)
                strcpy(observation, "Tag detected");
            else
                snprintf(observation, sizeof(observation), "%" PRId32 " dBm",
                         sample->wire.ble.rssi_dbm);
        }
    } else {
        status = sample->seen ? "Last event" : "No event yet";
        if (sample->seen && stream == CROSSING_STREAM)
            strcpy(observation, sample->wire.sensor.kind == AIRCRAFT_ENTER
                   ? "Crossing toward runway" : "Crossing toward taxiway");
        else if (sample->seen)
            strcpy(observation, sample->wire.button.event == RUNWAY_VEHICLE_ENTER
                   ? "Vehicle entered" : "Vehicle exited");
    }

    if (sample->seen) {
        if (age >= 1000)
            snprintf(age_text, sizeof(age_text), "%" PRIu64 ".%u s",
                     age / 1000, (unsigned)((age % 1000) / 100));
        else
            snprintf(age_text, sizeof(age_text), "%" PRIu64 " ms", age);
    }
    char sequence[24] = "--";
    if (sample->seen && sample->sequence)
        snprintf(sequence, sizeof(sequence), "%" PRIu64, sample->sequence);
    printf("| %-19s | %-23s | %4s | %7s | %-15s | %8" PRIu64 " |\n",
           label, observation, sequence, age_text, status, rejected);
}

static void print_status(struct state *state, uint64_t now)
{
    uint64_t rejected[STREAM_COUNT + 1], heartbeats[HEARTBEAT_COUNT], forwarding_since;
    struct watermark last[STREAM_COUNT];
    enum decision reason;
    enum stream stream;
    uint64_t seq, capture_ms, age, forwarding_failures;
    bool event_loss;
    int forward_error;
    enum stream forward_stream;
    const char *forward_stage;
    pthread_mutex_lock(&state->lock);
    memcpy(last, state->last, sizeof(last));
    memcpy(rejected, state->rejected, sizeof(rejected));
    memcpy(heartbeats, state->heartbeats, sizeof(heartbeats));
    forwarding_since = state->forwarding_since_ms;
    reason = state->last_drop;
    stream = state->last_drop_stream;
    seq = state->last_drop_sequence;
    capture_ms = state->last_drop_capture_ms;
    age = state->last_drop_age_ms;
    forwarding_failures = state->decisions[FORWARD_FAILED];
    forward_error = state->last_forward_error;
    forward_stream = state->last_forward_stream;
    forward_stage = state->last_forward_stage;
    event_loss = state->event_loss;
    pthread_mutex_unlock(&state->lock);

    const char *term = getenv("TERM");
    const bool refresh = isatty(STDOUT_FILENO) && term && strcmp(term, "dumb") != 0;
    if (refresh)
        fputs("\033[H", stdout);
    puts("RUNWAYSENTINEL | FAULT MANAGER");
    puts("| Input               | Latest observation      |  Seq |     Age | Status          | Rejected |");
    puts("|---------------------|-------------------------|------|---------|-----------------|----------|");
    print_input(last, AIR_RANGE_STREAM, "Aircraft ultrasonic", now, rejected[AIR_RANGE_STREAM]);
    print_input(last, GV_RANGE_STREAM, "Vehicle ultrasonic", now, rejected[GV_RANGE_STREAM]);
    print_input(last, CROSSING_STREAM, "IR taxiway", now,
                rejected[CROSSING_STREAM] + rejected[IR_STREAM]);
    print_input(last, BUTTON_STREAM, "Vehicle button", now, rejected[BUTTON_STREAM]);
    for (unsigned tag = 0; tag < TAG_COUNT; ++tag) {
        char label[12];
        snprintf(label, sizeof(label), "BLE %s", runway_tag_name(tag));
        print_input(last, BLE_FIRST_STREAM + tag, label, now, rejected[BLE_FIRST_STREAM + tag]);
    }
    uint64_t unassigned = rejected[NO_STREAM] + rejected[SENSOR_READY_STREAM] + rejected[BUTTON_READY_STREAM];
    if (unassigned)
        printf("| %-19s | %-23s | %4s | %7s | %-15s | %8" PRIu64 " |\n",
               "Other IPC", "--", "--", "--", "--", unassigned);

    if (event_loss || forwarding_failures)
        printf("Forwarding failures: %" PRIu64 "%s\n", forwarding_failures,
               event_loss ? " | Event delivery degraded" : "");
    if (forward_error)
        printf("Last forwarding error: %s / %s: %s (%d)\n",
               forward_stream == NO_STREAM ? "unknown" : stream_names[forward_stream],
               forward_stage, strerror(forward_error), forward_error);
    bool forwarding_stalled = forwarding_since && now > forwarding_since &&
        now - forwarding_since > RUNWAY_HEARTBEAT_LIMIT_MS;
    static const char *const worker_names[] = {"IR", "Ultrasonic", "Buttons", "BLE"};
    for (unsigned i = 0; i < HEARTBEAT_COUNT; ++i) {
        if (heartbeats[i] && now > heartbeats[i] && now - heartbeats[i] > RUNWAY_HEARTBEAT_LIMIT_MS)
            printf("Watchdog: %s unresponsive\n", worker_names[i]);
    }
    if (forwarding_stalled)
        puts("Watchdog: supervisor IPC stalled");
    if (state->details) {
        for (unsigned i = AIR_RANGE_STREAM; i <= GV_RANGE_STREAM; ++i) {
            if (last[i].seen && last[i].wire.sensor.range_state == RANGE_BEYOND_LIMIT)
                printf("%s: last reading beyond %u cm\n", stream_names[i],
                       RUNWAY_RANGE_LIMIT_MM / 10);
        }
        puts("| Stream        | Last accepted capture (monotonic ms) |");
        puts("|---------------|--------------------------------------|");
        for (unsigned i = 0; i < STREAM_COUNT; ++i) {
            if (last[i].seen)
                printf("| %-13s | %36" PRIu64 " |\n", stream_names[i], last[i].timestamp_ms);
        }
        puts("| Rejected stream | Sequence | Capture (ms) | Age (ms) | Reason |");
        if (reason == ACCEPT || reason == FORWARD_FAILED)
            puts("| -- | -- | -- | -- | No rejection recorded |");
        else
            printf("| %s | %" PRIu64 " | %" PRIu64 " | %" PRIu64 " | %s |\n",
                   stream == NO_STREAM ? "unknown" : stream_names[stream],
                   seq, capture_ms, age, decision_names[reason]);
    }
    if (refresh)
        fputs("\033[J", stdout);
    else
        putchar('\n');
    fflush(stdout);
}

static void *print_reports(void *arg)
{
    struct state *state = arg;
    const struct timespec interval = {.tv_sec = 1};
    runway_schedule("fault monitor console", SCHED_RR, RUNWAY_PRIO_ROUTINE);
    for (;;) {
        pthread_mutex_lock(&state->lock);
        bool done = state->stop;
        pthread_mutex_unlock(&state->lock);
        uint64_t now;
        if (done || runway_monotonic_ms(&now) != 0)
            return NULL;
        print_status(state, now);
        nanosleep(&interval, NULL);
    }
}

static int send_supervisor(int coid, const void *data, size_t size, uint64_t budget_ms)
{
    const uint64_t timeout_ns = budget_ms * UINT64_C(1000000);
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,
                     NULL, &timeout_ns, NULL) == -1)
        return errno;
    int status = MsgSend(coid, data, size, NULL, 0);
    return status < 0 ? errno : status;
}

static int send_link(int coid, enum runway_link_action action)
{
    struct runway_link_message message = {
        .type = RUNWAY_SUPERVISOR_LINK,
        .link_version = RUNWAY_SUPERVISOR_LINK_VERSION, .action = action
    };
    if (runway_monotonic_ms(&message.timestamp_ms) == -1)
        return errno;
    return send_supervisor(coid, &message, sizeof(message), 200);
}

static int send_report_to_supervisor(int coid, const struct report *report, uint64_t *now,
                                    enum decision *decision)
{
    if (runway_monotonic_ms(now) == -1)
        return errno;
    if (check_age(report, *now) != ACCEPT || *now - report->stamp_ms == report->max_age_ms) {
        *decision = DROP_STALE_QUEUED;
        return 0;
    }
    uint64_t budget = report->max_age_ms - (*now - report->stamp_ms);
    return send_supervisor(coid, &report->wire, report->size, budget > 200 ? 200 : budget);
}

static void *forward_reports(void *arg)
{
    struct state *state = arg;
    int coid = -1;
    runway_schedule("fault monitor forwarding", SCHED_FIFO, RUNWAY_PRIO_CRITICAL);
    for (;;) {
        uint64_t now;
        if (runway_monotonic_ms(&now) == -1)
            break;
        pthread_mutex_lock(&state->lock);
        if (state->stop) {
            pthread_mutex_unlock(&state->lock);
            break;
        }
        if (state->count == 0) {
            pthread_cond_wait(&state->queued, &state->lock);
            pthread_mutex_unlock(&state->lock);
            continue;
        }
        struct report report = state->queue[0];
        --state->count;
        state->forwarding_since_ms = now;
        memmove(&state->queue[0], &state->queue[1], state->count * sizeof(state->queue[0]));
        pthread_mutex_unlock(&state->lock);

        enum decision decision = ACCEPT;
        int error = 0;
        const char *stage = "report rejected or IPC failed";
        if (check_age(&report, now) != ACCEPT) {
            decision = DROP_STALE_QUEUED;
        } else if (coid == -1) {
            coid = name_open(RUNWAY_CHANNEL, 0);
            if (coid == -1) {
                stage = "open supervisor";
                error = errno;
            } else {
                error = send_link(coid, RUNWAY_LINK_PROBE);
                if (error) {
                    stage = error == ENOSYS || error == EINVAL || error == ENOTSUP
                        ? "supervisor version mismatch (deploy matching build)"
                        : "supervisor handshake";
                    name_close(coid);
                    coid = -1;
                }
            }
        }
        if (!error && decision == ACCEPT) {
            error = send_report_to_supervisor(coid, &report, &now, &decision);
            if (error == ENOTCONN &&
                (report.stream == AIR_RANGE_STREAM || report.stream == GV_RANGE_STREAM)) {
                /* Range is a replaceable observation. ENOTCONN means it was
                   rejected before it could change the supervisor's state. */
                stage = "restore range link";
                error = send_link(coid, RUNWAY_LINK_RESUME_RANGE);
                if (!error) {
                    stage = "range after link restore";
                    error = send_report_to_supervisor(coid, &report, &now, &decision);
                }
            }
        }
        if (error)
            decision = FORWARD_FAILED;

        pthread_mutex_lock(&state->lock);
        state->forwarding_since_ms = 0;
        if (decision == ACCEPT) {
            ++state->forwarded;
        } else {
            if (error) {
                state->last_forward_error = error;
                state->last_forward_stream = report.stream;
                state->last_forward_stage = stage;
            }
            record_decision(state, decision, &report, now);
        }
        pthread_mutex_unlock(&state->lock);
        /* Never replay a button or crossing with uncertain delivery. */
        if (coid != -1 && (error == EBADF || error == ESRCH || error == ENXIO)) {
            name_close(coid);
            coid = -1;
        }
    }
    if (coid != -1)
        name_close(coid);
    return NULL;
}

/* The test client feeds one observation back through the normal validator. */
static int apply_test_command(struct state *state, const struct runway_fault_test_command *m,
                              uint64_t now)
{
    if (!state->test_enabled)
        return EPERM;
    if (m->version != RUNWAY_IPC_VERSION)
        return EINVAL;
    if (m->action == TEST_DELAY_AIR) {
        if (m->delay_ms <= RUNWAY_MAX_AGE_MS || m->delay_ms > 10000)
            return EINVAL;
        if (state->test_delay_ms || state->test_due_ms)
            return EBUSY;
        state->test_delay_ms = m->delay_ms;
        return 0;
    }
    if (m->action == TEST_REPLAY_BUTTON) {
        const struct watermark *last = &state->last[BUTTON_STREAM];
        if (!last->seen || now < last->timestamp_ms || now - last->timestamp_ms > RUNWAY_MAX_AGE_MS) {
            if (state->test_replay_next_button)
                return EBUSY;
            state->test_replay_next_button = true;
            return 0;
        }
        accept_report(state, &last->wire, sizeof(last->wire.button), last->pid, now);
        return 0;
    }
    if (m->action == TEST_CONFLICT_AC1) {
        const struct tag_address *tag = &state->tags[TAG_AC1];
        if (!tag->configured)
            return ENOENT;
        union wire_message synthetic = {.ble_observation = {
            .type = RUNWAY_FAULT_BLE_OBSERVATION, .version = RUNWAY_IPC_VERSION,
            .tag = TAG_AC1, .address_type = tag->type,
            .timestamp_ms = now, .sequence = 1, .rssi_dbm = -50
        }};
        memcpy(synthetic.ble_observation.address, tag->bytes, sizeof(tag->bytes));
        synthetic.ble_observation.address[0] ^= 1;
        accept_report(state, &synthetic, sizeof(synthetic.ble_observation), 0, now);
        return 0;
    }
    return EINVAL;
}

static void release_delayed_report(struct state *state)
{
    uint64_t now;
    if (state->test_due_ms && runway_monotonic_ms(&now) == 0 && now >= state->test_due_ms) {
        accept_report(state, &state->test_delayed, sizeof(state->test_delayed.sensor),
                      state->test_pid, now);
        state->test_due_ms = 0;
    }
}

static int receive_reports(name_attach_t *channel, struct state *state)
{
    while (!stopping) {
        release_delayed_report(state);
        union wire_message wire = {0};
        struct _msg_info info;
        const uint64_t timeout_ns = UINT64_C(250000000);
        if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE,
                         NULL, &timeout_ns, NULL) == -1)
            return -1;
        int rcvid = MsgReceive(channel->chid, &wire, sizeof(wire), &info);
        if (rcvid == -1) {
            if (errno == EINTR || errno == ETIMEDOUT)
                continue;
            return -1;
        }
        if (rcvid == 0) {
            if (wire.pulse.code == _PULSE_CODE_DISCONNECT)
                ConnectDetach(wire.pulse.scoid);
            continue;
        }
        if (info.msglen >= (ssize_t)sizeof(wire.type) && wire.type == _IO_CONNECT) {
            MsgReply(rcvid, EOK, NULL, 0);
            continue;
        }
        if (info.srcmsglen != info.msglen || info.msglen < (ssize_t)sizeof(wire.type)) {
            memset(&wire, 0, sizeof(wire));
            info.msglen = 0;
        }
        bool operator_command = wire.type == RUNWAY_BLE_MESSAGE &&
            info.msglen == (ssize_t)sizeof(wire.ble) && wire.ble.kind != TAG_SEEN;
        uint64_t now;
        if (runway_monotonic_ms(&now) == -1) {
            int error = errno;
            MsgError(rcvid, error);
            errno = error;
            return -1;
        }
        if (wire.type == RUNWAY_FAULT_HEARTBEAT && info.msglen == (ssize_t)sizeof(wire.heartbeat)) {
            const struct runway_heartbeat_message *beat = &wire.heartbeat;
            if (beat->version != RUNWAY_IPC_VERSION || beat->source >= HEARTBEAT_COUNT ||
                beat->timestamp_ms > now || now - beat->timestamp_ms > RUNWAY_HEARTBEAT_LIMIT_MS) {
                MsgError(rcvid, EINVAL);
            } else {
                pthread_mutex_lock(&state->lock);
                if (beat->timestamp_ms > state->heartbeats[beat->source])
                    state->heartbeats[beat->source] = beat->timestamp_ms;
                pthread_mutex_unlock(&state->lock);
                MsgReply(rcvid, EOK, NULL, 0);
            }
            continue;
        }
        if (wire.type == RUNWAY_FAULT_TEST_COMMAND && info.msglen == (ssize_t)sizeof(wire.test)) {
            int result = apply_test_command(state, &wire.test, now);
            if (result)
                MsgError(rcvid, result);
            else
                MsgReply(rcvid, EOK, NULL, 0);
            continue;
        }
        if (state->test_delay_ms && wire.type == RUNWAY_SENSOR_MESSAGE &&
            info.msglen == (ssize_t)sizeof(wire.sensor) && wire.sensor.kind == AIRCRAFT_RANGE) {
            const struct watermark *ready = &state->last[SENSOR_READY_STREAM];
            struct report candidate;
            if (ready->seen && ready->pid == info.pid &&
                decode(state, &wire, sizeof(wire.sensor), &candidate) == ACCEPT &&
                check_age(&candidate, now) == ACCEPT) {
                state->test_delayed = wire;
                state->test_pid = info.pid;
                state->test_due_ms = now + state->test_delay_ms;
                state->test_delay_ms = 0;
                MsgReply(rcvid, EOK, NULL, 0);
                continue;
            }
        }
        enum decision result = accept_report(state, &wire, (size_t)info.msglen, info.pid, now);
        if (result == ACCEPT && state->test_replay_next_button &&
            wire.type == RUNWAY_BUTTON_MESSAGE && wire.button.event != RUNWAY_BUTTONS_READY) {
            state->test_replay_next_button = false;
            accept_report(state, &wire, sizeof(wire.button), info.pid, now);
        }
        /* Legacy operator commands should retain the supervisor's synchronous result.
           Send those via RUNWAY_CHANNEL; this channel is for adapter telemetry. */
        if (operator_command)
            MsgError(rcvid, ENOTSUP);
        else
            MsgReply(rcvid, EOK, NULL, 0);
    }
    return 0;
}

/* Specify addresses using the usual displayed order, e.g. AC1=0,AA:BB:CC:DD:EE:FF. */
static bool configure_tag(struct state *state, const char *arg)
{
    if (strlen(arg) != 23 || arg[3] != '=' || arg[5] != ',' ||
        (arg[4] != '0' && arg[4] != '1'))
        return false;
    unsigned tag = 0;
    while (tag < TAG_COUNT && strncmp(arg, runway_tag_name(tag), 3) != 0)
        ++tag;
    if (tag == TAG_COUNT || state->tags[tag].configured)
        return false;
    struct tag_address address = {.configured = true, .type = (uint8_t)(arg[4] - '0')};
    for (unsigned i = 0; i < 6; ++i) {
        unsigned byte;
        const char *part = arg + 6 + i * 3;
        if (strspn(part, "0123456789abcdefABCDEF") != 2 ||
            (i < 5 && part[2] != ':') || sscanf(part, "%2x", &byte) != 1)
            return false;
        address.bytes[5 - i] = (uint8_t)byte;
    }
    for (unsigned i = 0; i < TAG_COUNT; ++i) {
        if (state->tags[i].configured && state->tags[i].type == address.type &&
            memcmp(state->tags[i].bytes, address.bytes, sizeof(address.bytes)) == 0)
            return false;
    }
    state->tags[tag] = address;
    return true;
}

int main(int argc, char *argv[])
{
    struct state state = {.lock = PTHREAD_MUTEX_INITIALIZER};
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--details") == 0) {
            state.details = true;
            continue;
        }
        if (strcmp(argv[i], "--test") == 0) {
            state.test_enabled = true;
            continue;
        }
        if (!configure_tag(&state, argv[i])) {
            fprintf(stderr, "Usage: %s [--details] [--test] [AC1=0,AA:BB:CC:DD:EE:FF] [AC2=...] [GV1=...]\n",
                    argv[0]);
            return EXIT_FAILURE;
        }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    pthread_condattr_t attr;
    int error = pthread_condattr_init(&attr);
    if (error == 0) {
        error = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        if (error == 0)
            error = pthread_cond_init(&state.queued, &attr);
        pthread_condattr_destroy(&attr);
    }
    if (error != 0) {
        fprintf(stderr, "Initialize queue condition: %s\n", strerror(error));
        return EXIT_FAILURE;
    }
    struct sigaction action = {.sa_handler = stop_signal};
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) == -1 || sigaction(SIGTERM, &action, NULL) == -1) {
        perror("sigaction");
        return EXIT_FAILURE;
    }
    sigset_t signals, original;
    sigemptyset(&signals);
    sigaddset(&signals, SIGINT);
    sigaddset(&signals, SIGTERM);
    error = pthread_sigmask(SIG_BLOCK, &signals, &original);
    if (error != 0) {
        fprintf(stderr, "Block shutdown signals: %s\n", strerror(error));
        return EXIT_FAILURE;
    }
    name_attach_t *channel = name_attach(NULL, RUNWAY_FAULT_CHANNEL, 0);
    if (channel == NULL) {
        perror("name_attach runway_fault_manager");
        return EXIT_FAILURE;
    }
    pthread_t forwarder;
    error = pthread_create(&forwarder, NULL, forward_reports, &state);
    if (error != 0) {
        fprintf(stderr, "Start forwarding thread: %s\n", strerror(error));
        name_detach(channel, 0);
        return EXIT_FAILURE;
    }
    pthread_t console;
    error = pthread_create(&console, NULL, print_reports, &state);
    if (error != 0)
        fprintf(stderr, "Start fault monitor console: %s\n", strerror(error));
    pthread_sigmask(SIG_SETMASK, &original, NULL);
    runway_schedule("fault monitor intake", SCHED_FIFO, RUNWAY_PRIO_INTAKE);
    int status = -1;
    if (error == 0) {
        status = receive_reports(channel, &state);
        if (status == -1)
            perror("Receive adapter report");
    }
    name_detach(channel, 0);
    pthread_mutex_lock(&state.lock);
    state.stop = true;
    pthread_cond_signal(&state.queued);
    pthread_mutex_unlock(&state.lock);
    pthread_join(forwarder, NULL);
    if (error == 0)
        pthread_join(console, NULL);
    pthread_cond_destroy(&state.queued);
    pthread_mutex_destroy(&state.lock);
    return status == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
