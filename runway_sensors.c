#define _QNX_SOURCE 1

#include "runway_heartbeat.h"
#include "runway_schedule.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <unistd.h>

enum {
    IR1, IR2, AIR_TRIG, AIR_ECHO, GV_TRIG, GV_ECHO, PIN_COUNT,
    IR_DEBOUNCE_MS = 20, CROSSING_MS = 3000,
    MIN_MM = 20, MAX_MM = 4000, SPEED_SAMPLES = 5, DISPLAY_INTERVAL_MS = 1500
};

#define FRAME_NS UINT64_C(300000000)
/* Last pulse width that rounds to a distance of 2000 mm. */
#define MAX_ECHO_PULSE_NS ((UINT64_C(2000000) * RUNWAY_RANGE_LIMIT_MM + 999999u) / 343u)
#define MAX_SAMPLE_GAP_NS UINT64_C(750000000)
#define MAX_EDGE_WINDOW_NS UINT64_C(250000)
#define BETWEEN_TRIGGERS_NS UINT64_C(65000000)
#define STILL_THRESHOLD_CM_S 0.2

struct settings {
    int fd[PIN_COUNT];
    int button_fd[2];
    int connection;
    bool local;
    int ir_active[2];
    uint32_t zone_mm[2];
    atomic_bool running;
};

enum gate_phase { WAIT_CLEAR, IDLE, FIRST, SECOND };
struct gate {
    enum gate_phase phase;
    unsigned first;
    unsigned previous;
    uint64_t start_ms;
};

struct debounce {
    bool candidate, stable;
    uint64_t since_ms;
};

struct sample {
    uint64_t timestamp_ns;
    uint32_t distance_mm;
};

struct motion {
    struct sample samples[SPEED_SAMPLES];
    size_t count;
};

static volatile sig_atomic_t stopped;

static void stop(int signal_number)
{
    (void)signal_number;
    stopped = 1;
}

static void sleep_ns(long ns)
{
    const struct timespec delay = {.tv_nsec = ns};
    nanosleep(&delay, NULL);
}

static int monotonic_ns(uint64_t *ns)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
        return -1;
    *ns = (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
    return 0;
}

static int gpio_write(int fd, const char *text)
{
    size_t size = strlen(text);
    ssize_t n = write(fd, text, size);
    if (n == (ssize_t)size)
        return 0;
    if (n >= 0)
        errno = EIO;
    return -1;
}

static int gpio_read(int fd)
{
    char value[8];
    ssize_t n = read(fd, value, sizeof(value));
    if (n == 0 && lseek(fd, 0, SEEK_SET) >= 0)
        n = read(fd, value, sizeof(value));
    if (n > 0 && (value[0] == '0' || value[0] == '1'))
        return value[0] - '0';
    if (n >= 0)
        errno = EIO;
    return -1;
}

static bool debounce(struct debounce *input, bool detected, uint64_t now)
{
    if (detected != input->candidate) {
        input->candidate = detected;
        input->since_ms = now;
    }
    if (now - input->since_ms >= IR_DEBOUNCE_MS)
        input->stable = input->candidate;
    return input->stable;
}

static int crossing(struct gate *gate, unsigned mask, uint64_t now)
{
    unsigned previous = gate->previous;
    gate->previous = mask;
    switch (gate->phase) {
    case WAIT_CLEAR:
        if (mask == 0)
            gate->phase = IDLE;
        break;
    case IDLE:
        if (mask == 3) {
            gate->phase = WAIT_CLEAR;
        } else if (mask != 0) {
            gate->phase = FIRST;
            gate->first = mask;
            gate->start_ms = now;
        }
        break;
    case FIRST:
        if (now - gate->start_ms > CROSSING_MS ||
            (previous == 0 && mask == gate->first)) {
            gate->phase = mask == 0 ? IDLE : WAIT_CLEAR;
        } else if ((mask & (3u ^ gate->first)) != 0) {
            gate->phase = SECOND;
        }
        break;
    case SECOND:
        if (now - gate->start_ms > CROSSING_MS || mask == gate->first ||
            (mask == 3 && previous != 3) || (mask == 0 && previous == 3)) {
            gate->phase = mask == 0 ? IDLE : WAIT_CLEAR;
        } else if (mask == 0) {
            gate->phase = IDLE;
            return gate->first == 2 ? AIRCRAFT_ENTER : AIRCRAFT_EXIT;
        }
        break;
    }
    return 0;
}

static int send_report(struct settings *s, struct runway_sensor_message *report)
{
    if (s->local)
        return 0;

    report->type = RUNWAY_SENSOR_MESSAGE;
    report->version = RUNWAY_IPC_VERSION;
    const uint64_t timeout_ns = UINT64_C(1000000000);
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL,
                     &timeout_ns, NULL) == -1)
        return -1;
    return MsgSend(s->connection, report, sizeof(*report), NULL, 0);
}

static void *ir_worker(void *arg)
{
    struct settings *s = arg;
    runway_schedule("IR worker", SCHED_FIFO, RUNWAY_PRIO_IR);
    struct gate gate = {.phase = WAIT_CLEAR};
    struct debounce input[2] = {
        {.candidate = true, .stable = true},
        {.candidate = true, .stable = true}
    };
    uint64_t sequence = 0;
    uint64_t ir_sequence = 0;
    unsigned previous_mask = 0;
    uint64_t aircraft_count = 0;
    uint64_t next_heartbeat = 0;
    while (atomic_load(&s->running)) {
        int first = gpio_read(s->fd[IR1]);
        int second = gpio_read(s->fd[IR2]);
        uint64_t now;
        if (first < 0 || second < 0 || runway_monotonic_ms(&now) == -1)
            break;

        unsigned mask = debounce(&input[0], first == s->ir_active[0], now) ? 1u : 0u;
        if (debounce(&input[1], second == s->ir_active[1], now))
            mask |= 2u;
        /* Apply the crossing before publishing the final clear IR mask. */
        int event = crossing(&gate, mask, now);
        if (event != 0) {
            struct runway_sensor_message report = {
                .kind = (uint32_t)event, .sequence = ++sequence, .timestamp_ms = now
            };
            if (send_report(s, &report) == -1)
                break;
            if (event == AIRCRAFT_ENTER)
                ++aircraft_count;
            else if (aircraft_count > 0)
                --aircraft_count;

            /* Print crossing diagnostics only in standalone mode. */
            if (s->local) {
                flockfile(stdout);
                puts("\n+------------------+----------------+");
                puts("| IR Gate Event    | IR Net Count   |");
                puts("+------------------+----------------+");
                printf("| %-16s | %14" PRIu64 " |\n",
                       event == AIRCRAFT_ENTER ? "AIRCRAFT ENTER" : "AIRCRAFT EXIT",
                       aircraft_count);
                puts("+------------------+----------------+");
                funlockfile(stdout);
            }
        }
        if (mask != previous_mask) {
            struct runway_sensor_message beams = {
                .kind = IR_BEAMS, .sequence = ++ir_sequence,
                .timestamp_ms = now, .ir_mask = mask
            };
            if (send_report(s, &beams) == -1)
                break;
            previous_mask = mask;
        }
        runway_heartbeat(s->connection, HEARTBEAT_IR, now, &next_heartbeat);
        sleep_ns(4000000);
    }
    if (atomic_exchange(&s->running, false))
        perror("IR worker");
    return NULL;
}

struct echo_edge {
    uint64_t earliest_ns, latest_ns, sample_started_ns;
};

enum level_result { LEVEL_ERROR = -1, LEVEL_TIMEOUT, LEVEL_FOUND, LEVEL_HELD };
enum echo_result { ECHO_ERROR = -1, ECHO_MISSING, ECHO_MEASURED, ECHO_BEYOND_LIMIT };

/* GPIO polling bounds the transition between two samples. */
static enum level_result wait_level(int fd, int wanted, uint64_t deadline,
                                    uint64_t previous_sample, struct echo_edge *edge)
{
    for (;;) {
        uint64_t started, observed;
        if (monotonic_ns(&started) == -1)
            return LEVEL_ERROR;
        int level = gpio_read(fd);
        if (level < 0 || monotonic_ns(&observed) == -1)
            return LEVEL_ERROR;
        if (started >= deadline)
            return level == wanted ? LEVEL_TIMEOUT : LEVEL_HELD;
        if (level == wanted) {
            *edge = (struct echo_edge){previous_sample, observed, started};
            return LEVEL_FOUND;
        }
        previous_sample = started;
    }
}

static enum echo_result read_echo_distance(const struct echo_edge *rise,
                                           const struct echo_edge *fall, uint32_t *mm)
{
    uint64_t rise_window = rise->latest_ns - rise->earliest_ns;
    uint64_t fall_window = fall->latest_ns - fall->earliest_ns;
    if (rise_window > MAX_EDGE_WINDOW_NS || fall_window > MAX_EDGE_WINDOW_NS)
        return ECHO_MISSING;

    uint64_t rise_time = rise->earliest_ns + rise_window / 2;
    uint64_t fall_time = fall->earliest_ns + fall_window / 2;
    if (fall_time <= rise_time)
        return ECHO_MISSING;

    uint64_t pulse_ns = fall_time - rise_time;
    if (pulse_ns > MAX_ECHO_PULSE_NS)
        return ECHO_BEYOND_LIMIT;
    uint64_t distance = (pulse_ns * 343u + 1000000u) / 2000000u;
    if (distance < MIN_MM)
        return ECHO_MISSING;
    *mm = (uint32_t)distance;
    return ECHO_MEASURED;
}

static enum echo_result measure(int trig, int echo, uint32_t *mm, uint64_t *sample_ns)
{
    uint64_t idle_sample;
    if (monotonic_ns(&idle_sample) == -1)
        return ECHO_ERROR;
    int idle = gpio_read(echo);
    if (idle < 0)
        return ECHO_ERROR;
    if (idle != 0) {
        return ECHO_MISSING;
    }

    if (gpio_write(trig, "on") == -1)
        return ECHO_ERROR;
    uint64_t start;
    uint64_t now;
    int status = monotonic_ns(&start);
    if (status == 0) {
        do {
            status = monotonic_ns(&now);
        } while (status == 0 && now - start < 12000);
    }
    if (gpio_write(trig, "off") == -1 || status == -1)
        return ECHO_ERROR;

    struct echo_edge rise, fall;
    enum level_result edge = wait_level(echo, 1, now + UINT64_C(30000000), idle_sample, &rise);
    if (edge == LEVEL_ERROR)
        return ECHO_ERROR;
    if (edge != LEVEL_FOUND || rise.latest_ns - rise.earliest_ns > MAX_EDGE_WINDOW_NS)
        return ECHO_MISSING;
    /* Use the latest possible rising edge as the cutoff anchor. */
    edge = wait_level(echo, 0, rise.latest_ns + MAX_ECHO_PULSE_NS + 1,
                      rise.sample_started_ns, &fall);
    if (edge == LEVEL_ERROR)
        return ECHO_ERROR;
    if (edge == LEVEL_HELD)
        return ECHO_BEYOND_LIMIT;
    if (edge != LEVEL_FOUND)
        return ECHO_MISSING;
    *sample_ns = fall.latest_ns;
    return read_echo_distance(&rise, &fall, mm);
}

static void estimate_motion(struct motion *history, uint64_t timestamp_ns,
                            struct runway_sensor_message *report)
{
    if (!runway_range_has_distance(report->range_state)) {
        history->count = 0;
        return;
    }

    if (history->count > 0) {
        uint64_t previous = history->samples[history->count - 1].timestamp_ns;
        if (timestamp_ns <= previous || timestamp_ns - previous > MAX_SAMPLE_GAP_NS)
            history->count = 0;
    }
    if (history->count == SPEED_SAMPLES) {
        memmove(history->samples, history->samples + 1,
                (SPEED_SAMPLES - 1) * sizeof(history->samples[0]));
        --history->count;
    }
    history->samples[history->count++] =
        (struct sample){timestamp_ns, report->distance_mm};
    if (history->count < SPEED_SAMPLES)
        return;

    /* Match the median pairwise-slope estimate used in the bench program. */
    double slopes[SPEED_SAMPLES * (SPEED_SAMPLES - 1) / 2];
    size_t count = 0;
    for (size_t i = 0; i < SPEED_SAMPLES; ++i) {
        for (size_t j = i + 1; j < SPEED_SAMPLES; ++j) {
            int delta = (int)history->samples[j].distance_mm - (int)history->samples[i].distance_mm;
            uint64_t elapsed = history->samples[j].timestamp_ns - history->samples[i].timestamp_ns;
            double mm_per_s = delta * 1.0e9 / (double)elapsed;
            size_t position = count;
            while (position > 0 && slopes[position - 1] > mm_per_s) {
                slopes[position] = slopes[position - 1];
                --position;
            }
            slopes[position] = mm_per_s;
            ++count;
        }
    }

    double velocity = (slopes[(count - 1) / 2] + slopes[count / 2]) / 2.0;
    if (velocity > -10.0 * STILL_THRESHOLD_CM_S && velocity < 10.0 * STILL_THRESHOLD_CM_S)
        velocity = 0;
    if (velocity > INT32_MAX)
        velocity = INT32_MAX;
    if (velocity < INT32_MIN)
        velocity = INT32_MIN;

    report->radial_mm_per_s = (int32_t)velocity;
    report->motion_valid = 1;
}

static void print_ultrasonic(const char *label, const char *no_target,
                             const struct runway_sensor_message *report)
{
    const char *echo = report->range_state == RANGE_NO_ECHO ? no_target : "--";
    const char *zone = "--";
    const char *direction = "--";
    char distance[20] = "--", rate[20] = "--";
    if (runway_range_has_distance(report->range_state)) {
        echo = "Echo detected";
        zone = runway_range_zone(report->distance_mm);
        snprintf(distance, sizeof(distance), "%.1f cm", report->distance_mm / 10.0);
        if (report->motion_valid) {
            double speed = report->radial_mm_per_s / 10.0;
            direction = runway_range_direction(report->radial_mm_per_s);
            snprintf(rate, sizeof(rate), "%.1f cm/s", speed < 0 ? -speed : speed);
        }
    }
    printf("| %-15s | %-14s | %-10s | %-10s | %-8s | %-10s |\n",
           label, echo, distance, zone, direction, rate);
}

static void *ultrasonic_worker(void *arg)
{
    struct settings *s = arg;
    runway_schedule("ultrasonic worker", SCHED_RR, RUNWAY_PRIO_RANGE);
    const int trigger[] = {AIR_TRIG, GV_TRIG};
    const int echo[] = {AIR_ECHO, GV_ECHO};
    struct motion history[2] = {0};
    uint64_t sequence[2] = {0};
    uint64_t next_display_ms = 0;
    uint64_t next_heartbeat = 0;
    while (atomic_load(&s->running)) {
        uint64_t start, now;
        if (monotonic_ns(&start) == -1)
            goto failed;
        struct runway_sensor_message readings[2];
        for (unsigned i = 0; i < 2; ++i) {
            if (!atomic_load(&s->running))
                return NULL;
            struct runway_sensor_message report = {
                .kind = i == 0 ? AIRCRAFT_RANGE : VEHICLE_RANGE, .sequence = ++sequence[i]
            };
            uint64_t timestamp_ns;
            enum echo_result reading = measure(s->fd[trigger[i]], s->fd[echo[i]],
                                               &report.distance_mm, &timestamp_ns);
            if (reading == ECHO_ERROR ||
                (reading != ECHO_MEASURED && monotonic_ns(&timestamp_ns) == -1))
                goto failed;
            report.timestamp_ms = timestamp_ns / UINT64_C(1000000);
            if (reading == ECHO_MEASURED)
                report.range_state = report.distance_mm <= s->zone_mm[i] ? RANGE_INSIDE : RANGE_OUTSIDE;
            else if (reading == ECHO_BEYOND_LIMIT)
                report.range_state = RANGE_BEYOND_LIMIT;
            estimate_motion(&history[i], timestamp_ns, &report);
            if (send_report(s, &report) == -1)
                goto failed;

            readings[i] = report;
            if (i == 0)
                sleep_ns(BETWEEN_TRIGGERS_NS);
        }

        uint64_t start_ms = start / UINT64_C(1000000);
        if (s->local && start_ms >= next_display_ms) {
            flockfile(stdout);
            puts("| Sensor          | Echo status    | Range      | Zone       | Motion   | Speed      |");
            print_ultrasonic("Aircraft", "No aircraft", &readings[0]);
            print_ultrasonic("Ground vehicle", "No GV", &readings[1]);
            funlockfile(stdout);
            next_display_ms = start_ms + DISPLAY_INTERVAL_MS;
        }

        if (monotonic_ns(&now) == -1)
            goto failed;
        runway_heartbeat(s->connection, HEARTBEAT_RANGE, now / 1000000u, &next_heartbeat);
        uint64_t elapsed = now - start;
        uint64_t remaining = elapsed < FRAME_NS ? FRAME_NS - elapsed : 0;
        sleep_ns(remaining > BETWEEN_TRIGGERS_NS ? remaining : BETWEEN_TRIGGERS_NS);
    }
    return NULL;

failed:
    if (atomic_exchange(&s->running, false))
        perror("ultrasonic worker");
    return NULL;
}

static int parse_int(const char *text, long min, long max)
{
    char *end;
    errno = 0;
    long value = strtol(text, &end, 10);
    return errno == 0 && end != text && *end == '\0' && value >= min && value <= max
        ? (int)value : -1;
}

static int self_test(void)
{
    const struct echo_edge rise = {.earliest_ns = 1000000, .latest_ns = 1020000};
    struct echo_edge fall = {.earliest_ns = 6831000, .latest_ns = 6851000};
    uint32_t mm = 0;
    if (read_echo_distance(&rise, &fall, &mm) != ECHO_MEASURED || mm != 1000)
        return EXIT_FAILURE;
    fall = (struct echo_edge){.earliest_ns = 12662000, .latest_ns = 12682000};
    if (read_echo_distance(&rise, &fall, &mm) != ECHO_MEASURED || mm != RUNWAY_RANGE_LIMIT_MM)
        return EXIT_FAILURE;
    fall.earliest_ns = rise.earliest_ns + MAX_ECHO_PULSE_NS + 1;
    fall.latest_ns = rise.latest_ns + MAX_ECHO_PULSE_NS + 1;
    mm = 0;
    if (read_echo_distance(&rise, &fall, &mm) != ECHO_BEYOND_LIMIT || mm)
        return EXIT_FAILURE;
    fall.latest_ns = fall.earliest_ns + MAX_EDGE_WINDOW_NS + 1;
    if (read_echo_distance(&rise, &fall, &mm) != ECHO_MISSING)
        return EXIT_FAILURE;

    const unsigned masks[] = {0,1,3,2,0,0,2,3,1,0,1,0,0};
    const uint64_t times[] = {0,20,40,60,80,100,120,140,160,180,200,220,3400};
    struct gate gate = {.phase = WAIT_CLEAR};
    for (unsigned i = 0; i < sizeof(masks)/sizeof(*masks); ++i) {
        int expected = i == 4 ? AIRCRAFT_EXIT : i == 9 ? AIRCRAFT_ENTER : 0;
        if (crossing(&gate, masks[i], times[i]) != expected)
            return EXIT_FAILURE;
    }
    struct motion history = {0};
    for (unsigned i = 0; i < SPEED_SAMPLES; ++i) {
        struct runway_sensor_message sample = {
            .range_state = RANGE_INSIDE, .distance_mm = i == 2 ? 950 : 600 - 20 * i
        };
        estimate_motion(&history, FRAME_NS * (i + 1), &sample);
        if (i + 1 < SPEED_SAMPLES ? sample.motion_valid :
            (!sample.motion_valid || sample.radial_mm_per_s != (int32_t)(-20.0e9 / (double)FRAME_NS)))
            return EXIT_FAILURE;
    }

    struct runway_sensor_message missing = {.range_state = RANGE_NO_ECHO};
    estimate_motion(&history, 2 * FRAME_NS * SPEED_SAMPLES, &missing);
    if (history.count != 0 || missing.motion_valid)
        return EXIT_FAILURE;
    if (strcmp(runway_range_zone(499), "A") != 0 ||
        strcmp(runway_range_zone(510), "C") != 0 ||
        strcmp(runway_range_zone(990), "C") != 0 ||
        strcmp(runway_range_zone(1001), "B") != 0 ||
        strcmp(runway_range_zone(500), "UNASSIGNED") != 0 ||
        strcmp(runway_range_zone(1000), "UNASSIGNED") != 0)
        return EXIT_FAILURE;
    puts("SENSOR_SELF_TEST PASS");
    return EXIT_SUCCESS;
}

enum {
    RELEASED = 0,
    PRESSED = 1,
    DEBOUNCE_MS = 30,
    ENTER_BUTTON = 0,
    EXIT_BUTTON = 1,
    BUTTON_COUNT = 2
};

static const struct {
    const char *path;
    const char *event;
} buttons[BUTTON_COUNT] = {
    [ENTER_BUTTON] = {"/dev/gpio/5", "GV1_ENTER"},
    [EXIT_BUTTON] = {"/dev/gpio/6", "GV1_EXIT"}
};

struct button_state {
    int stable;
    int candidate;
    uint64_t changed_ms;
};

static bool pressed(struct button_state *state, int level, uint64_t now_ms)
{
    if (level != state->candidate) {
        state->candidate = level;
        state->changed_ms = now_ms;
    }
    if (state->candidate == state->stable || now_ms - state->changed_ms < DEBOUNCE_MS)
        return false;

    state->stable = state->candidate;
    return state->stable == PRESSED;
}

static uint64_t update_occupancy(uint64_t count, size_t button)
{
    (void)count;
    /* One BLE identity GV1 represents one ground vehicle. */
    if (button == ENTER_BUTTON)
        return 1;
    return 0;
}

static int publish_event(int coid, enum runway_event event, uint64_t sequence,
                         uint64_t timestamp_ms, uint64_t count)
{
    const struct runway_button_message msg = {
        .type = RUNWAY_BUTTON_MESSAGE,
        .version = RUNWAY_IPC_VERSION,
        .event = event,
        .sequence = sequence,
        .timestamp_ms = timestamp_ms,
        .vehicle_count = count
    };

    const uint64_t timeout_ns = UINT64_C(1000000000);
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &timeout_ns, NULL) == -1) {
        perror("TimerTimeout");
        return -1;
    }
    if (MsgSend(coid, &msg, sizeof(msg), NULL, 0) == -1) {
        perror("MsgSend runway_fault_manager");
        return -1;
    }
    return 0;
}

static int publish_ready(int coid)
{
    uint64_t now_ms;
    if (runway_monotonic_ms(&now_ms) == -1) {
        perror("clock_gettime");
        return -1;
    }
    return publish_event(coid, RUNWAY_BUTTONS_READY, 0, now_ms, 0);
}

static int read_buttons(struct settings *settings)
{
    const int *fd = settings->button_fd;
    const int coid = settings->connection;
    struct button_state state[BUTTON_COUNT] = {0};
    for (size_t i = 0; i < BUTTON_COUNT; ++i) {
        int level = gpio_read(fd[i]);
        if (level < 0) {
            perror(buttons[i].path);
            return EXIT_FAILURE;
        }
        /* A button held at startup must be released before its next press. */
        state[i].stable = level;
        state[i].candidate = level;
    }

    if (!settings->local && publish_ready(coid) == -1)
        return EXIT_FAILURE;

    puts("READY | count=0 | Button 1: GV1_ENTER | Button 2: GV1_EXIT | Ctrl+C: stop");
    uint64_t vehicle_count = 0;
    uint64_t next_heartbeat = 0;
    const struct timespec poll_interval = {.tv_nsec = 10000000};
    uint64_t sequence = 0;
    while (atomic_load(&settings->running)) {
        for (size_t i = 0; i < BUTTON_COUNT; ++i) {
            int level = gpio_read(fd[i]);
            if (level < 0) {
                if (!atomic_load(&settings->running) && errno == EINTR)
                    return EXIT_SUCCESS;
                perror(buttons[i].path);
                return EXIT_FAILURE;
            }

            uint64_t now_ms;
            if (runway_monotonic_ms(&now_ms) == -1) {
                perror("clock_gettime");
                return EXIT_FAILURE;
            }
            if (pressed(&state[i], level, now_ms)) {
                uint64_t next_count = update_occupancy(vehicle_count, i);
                if (next_count == vehicle_count)
                    continue;

                enum runway_event event = i == ENTER_BUTTON
                    ? RUNWAY_VEHICLE_ENTER : RUNWAY_VEHICLE_EXIT;
                if (!settings->local &&
                    publish_event(coid, event, sequence + 1, now_ms, next_count) == -1)
                    return !atomic_load(&settings->running) && errno == EINTR ? EXIT_SUCCESS : EXIT_FAILURE;

                vehicle_count = next_count;
                ++sequence;
                if (settings->local)
                    printf("%s seq=%" PRIu64 " t_ms=%" PRIu64 " count=%" PRIu64 "\n",
                           buttons[i].event, sequence, now_ms, vehicle_count);
            }
        }
        uint64_t now_ms;
        if (runway_monotonic_ms(&now_ms) == 0)
            runway_heartbeat(coid, HEARTBEAT_BUTTON, now_ms, &next_heartbeat);
        nanosleep(&poll_interval, NULL);
    }
    return EXIT_SUCCESS;
}

static int button_self_test(void)
{
    struct button_state one = {.stable = RELEASED, .candidate = RELEASED};
    struct button_state two = {.stable = PRESSED, .candidate = PRESSED};

    bool ok = !pressed(&one, PRESSED, 10)        /* first bounce */
           && !pressed(&one, RELEASED, 16)
           && !pressed(&one, PRESSED, 20)
           && !pressed(&one, PRESSED, 49)
           &&  pressed(&one, PRESSED, 50)        /* first stable press */
           && !pressed(&one, PRESSED, 100)       /* long hold */
           && !pressed(&two, PRESSED, 100)       /* held at startup */
           && !pressed(&two, RELEASED, 110)
           && !pressed(&two, RELEASED, 140)       /* stable release */
           && !pressed(&two, PRESSED, 150)
           &&  pressed(&two, PRESSED, 180)
           && !pressed(&one, RELEASED, 190)
           && !pressed(&one, RELEASED, 220)
           && !pressed(&one, PRESSED, 225)
           &&  pressed(&one, PRESSED, 255);      /* second press */

    const size_t presses[] = {
        EXIT_BUTTON, ENTER_BUTTON, ENTER_BUTTON, ENTER_BUTTON,
        EXIT_BUTTON, EXIT_BUTTON, EXIT_BUTTON, EXIT_BUTTON
    };
    const uint64_t expected_counts[] = {0, 1, 1, 1, 0, 0, 0, 0};
    uint64_t count = 0;
    for (size_t i = 0; i < sizeof(presses) / sizeof(presses[0]); ++i) {
        count = update_occupancy(count, presses[i]);
        ok = ok && count == expected_counts[i];
    }

    puts(ok ? "SELF_TEST PASS" : "SELF_TEST FAIL");
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

static void *button_worker(void *arg)
{
    struct settings *settings = arg;
    runway_schedule("push-button worker", SCHED_FIFO, RUNWAY_PRIO_BUTTON);
    if (read_buttons(settings) != EXIT_SUCCESS)
        atomic_store(&settings->running, false);
    return NULL;
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--self-test") == 0)
        return self_test() == EXIT_SUCCESS && button_self_test() == EXIT_SUCCESS
            ? EXIT_SUCCESS : EXIT_FAILURE;

    bool local = argc > 1 && strcmp(argv[1], "--local") == 0;
    int offset = local ? 1 : 0;
    if (argc != 11 + offset) {
        fprintf(stderr, "Usage: %s [--local] IR1 IR2 AIR_TRIG AIR_ECHO GV_TRIG GV_ECHO IR1_ACTIVE IR2_ACTIVE AIR_ZONE_MM GV_ZONE_MM\n", argv[0]);
        return EXIT_FAILURE;
    }

    struct settings s = {
        .connection = -1, .local = local, .running = true, .button_fd = {-1, -1}
    };
    int pins[PIN_COUNT];
    unsigned used = (1u << 5) | (1u << 6);
    for (unsigned i = 0; i < PIN_COUNT; ++i) {
        s.fd[i] = -1;
        pins[i] = parse_int(argv[i + 1 + offset], 0, 27);
        if (pins[i] < 0 || (used & (1u << pins[i])) != 0) {
            fputs("Use six different BCM GPIO numbers and reserve GPIO5/6 for buttons.\n", stderr);
            return EXIT_FAILURE;
        }
        used |= 1u << pins[i];
    }
    for (unsigned i = 0; i < 2; ++i) {
        int level = parse_int(argv[7 + i + offset], 0, 1);
        int limit = parse_int(argv[9 + i + offset], MIN_MM, MAX_MM);
        if (level < 0 || limit < 0) {
            fputs("IR active levels must be 0 or 1. Zone limits must be 20–4000 mm.\n", stderr);
            return EXIT_FAILURE;
        }
        s.ir_active[i] = level;
        s.zone_mm[i] = (uint32_t)limit;
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    runway_schedule("sensor adapter main", SCHED_RR, RUNWAY_PRIO_ROUTINE);
    struct sigaction action = {.sa_handler = stop};
    sigemptyset(&action.sa_mask);
    sigset_t signals, original_mask;
    sigemptyset(&signals);
    sigaddset(&signals, SIGINT);
    sigaddset(&signals, SIGTERM);
    if (sigaction(SIGINT, &action, NULL) == -1 || sigaction(SIGTERM, &action, NULL) == -1) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    int status = EXIT_FAILURE;
    for (unsigned i = 0; i < PIN_COUNT; ++i) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/gpio/%d", pins[i]);
        s.fd[i] = open(path, O_RDWR);
        bool output = i == AIR_TRIG || i == GV_TRIG;
        if (s.fd[i] < 0 || gpio_write(s.fd[i], output ? "out" : "in") == -1 ||
            (output && gpio_write(s.fd[i], "off") == -1)) {
            perror(path);
            goto cleanup;
        }
    }

    /* Establish an empty gate count only with both beams currently clear. */
    if (gpio_read(s.fd[IR1]) != 1 - s.ir_active[0] ||
        gpio_read(s.fd[IR2]) != 1 - s.ir_active[1]) {
        fputs("Clear both IR beams and verify their active levels before starting.\n", stderr);
        goto cleanup;
    }
    for (unsigned i = 0; i < BUTTON_COUNT; ++i) {
        s.button_fd[i] = open(buttons[i].path, O_RDWR);
        if (s.button_fd[i] < 0 || gpio_write(s.button_fd[i], "in") < 0) {
            perror(buttons[i].path);
            goto cleanup;
        }
    }
    if (!s.local) {
        s.connection = name_open(RUNWAY_FAULT_CHANNEL, 0);
        struct runway_sensor_message ready = {.kind = SENSORS_READY};
        if (s.connection == -1 || runway_monotonic_ms(&ready.timestamp_ms) == -1 ||
            send_report(&s, &ready) == -1) {
            perror("sensor connection (start runway_fault_manager first)");
            goto cleanup;
        }
    }
    printf("SENSORS READY | mode=%s | aircraft_count=0\n", s.local ? "LOCAL" : "IPC");

    int error = pthread_sigmask(SIG_BLOCK, &signals, &original_mask);
    if (error != 0) {
        fprintf(stderr, "pthread_sigmask: %s\n", strerror(error));
        goto cleanup;
    }
    void *(*const work[])(void *) = {ir_worker, ultrasonic_worker, button_worker};
    enum { WORKER_COUNT = sizeof(work) / sizeof(work[0]) };
    pthread_t workers[WORKER_COUNT];
    unsigned started = 0;
    for (; started < WORKER_COUNT; ++started) {
        error = pthread_create(&workers[started], NULL, work[started], &s);
        if (error != 0) {
            fprintf(stderr, "pthread_create: %s\n", strerror(error));
            atomic_store(&s.running, false);
            break;
        }
    }
    pthread_sigmask(SIG_SETMASK, &original_mask, NULL);
    while (!stopped && atomic_load(&s.running))
        sleep_ns(50000000);
    status = started == WORKER_COUNT && stopped ? EXIT_SUCCESS : EXIT_FAILURE;
    atomic_store(&s.running, false);
    for (unsigned i = 0; i < started; ++i)
        pthread_join(workers[i], NULL);

cleanup:
    for (unsigned i = 0; i < BUTTON_COUNT; ++i) {
        if (s.button_fd[i] >= 0)
            close(s.button_fd[i]);
    }
    if (s.connection >= 0)
        name_close(s.connection);
    for (unsigned i = 0; i < PIN_COUNT; ++i) {
        if (s.fd[i] >= 0)
            close(s.fd[i]);
    }
    return status;
}
