/*
 * RunwaySentinel: probe and scan on Raspberry Pi 4 onboard Bluetooth.
 * Bluetooth HCI H4 over a serial device, e.g. QNX /dev/ser4.
 * The host BSP must expose the internal UART and power the controller.
 * qcc -Vgcc_ntoaarch64le -std=c11 -Wall -Wextra -g runway_ble_uart.c -o runway_ble_uart
 */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif
#define _POSIX_C_SOURCE 200809L
#if defined(__QNXNTO__) && !defined(_QNX_SOURCE)
#define _QNX_SOURCE 1
#endif
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include "runway_heartbeat.h"
#include "runway_schedule.h"

enum {
    HCI_LE_META_EVENT = 0x3e,
    HCI_LE_ADV_REPORT = 0x02,
    HCI_EVENT_MAX = 257,
    LEGACY_ADV_DATA_MAX = 31
};
typedef struct {
    uint8_t address_type;
    uint8_t address[6];
    const char *tag_id;
    int rssi;
} advertisement_t;
static const char *tag_id_from_data(const uint8_t *data, size_t length)
{
    const char *id = NULL;
    size_t offset = 0;
    while (offset < length) {
        uint8_t field_length = data[offset++];
        if (field_length == 0)
            break;
        if ((size_t)field_length > length - offset)
            return NULL;
        uint8_t type = data[offset];
        size_t prefix;
        switch (type) {
        case 0x08: /* Shortened local name */
        case 0x09: /* Complete local name */
            prefix = 0;
            break;
        case 0x16: /* Service data with 16-bit UUID */
        case 0xff: /* Manufacturer data, following company ID */
            prefix = 2;
            break;
        case 0x20: /* Service data with 32-bit UUID */
            prefix = 4;
            break;
        case 0x21: /* Service data with 128-bit UUID */
            prefix = 16;
            break;
        default:
            offset += field_length;
            continue;
        }
        if ((size_t)field_length == 1 + prefix + 3) {
            const uint8_t *value = data + offset + 1 + prefix;
            const char *candidate = NULL;
            if (memcmp(value, "AC1", 3) == 0)
                candidate = "AC1";
            if (memcmp(value, "AC2", 3) == 0)
                candidate = "AC2";
            if (memcmp(value, "GV1", 3) == 0)
                candidate = "GV1";
            if (candidate != NULL) {
                if (id != NULL && strcmp(id, candidate) != 0)
                    return NULL;
                id = candidate;
            }
        }
        offset += field_length;
    }
    return id;
}
/* Returns false for malformed reports, true for other HCI events. */
static bool parse_hci_event(const uint8_t *event, size_t length,
                            advertisement_t *reports, size_t capacity,
                            size_t *report_count)
{
    *report_count = 0;
    if (length < 2 || (size_t)event[1] + 2 != length)
        return false;
    if (event[0] != HCI_LE_META_EVENT)
        return true;
    if (length < 3)
        return false;
    if (event[2] != HCI_LE_ADV_REPORT)
        return true;
    if (length < 4 || event[3] == 0 || event[3] > capacity)
        return false;
    size_t count = event[3];
    size_t offset = 4;
    for (size_t i = 0; i < count; ++i) {
        /* Event type, address type, six-byte address, length, data, RSSI. */
        if (length - offset < 10)
            return false;
        size_t data_length = event[offset + 8];
        if (data_length > LEGACY_ADV_DATA_MAX || data_length > length - offset - 10)
            return false;
        reports[i].address_type = event[offset + 1];
        memcpy(reports[i].address, event + offset + 2, 6);
        reports[i].tag_id = tag_id_from_data(event + offset + 9, data_length);
        uint8_t raw_rssi = event[offset + 9 + data_length];
        reports[i].rssi = raw_rssi < 128 ? raw_rssi : (int)raw_rssi - 256;
        offset += 10 + data_length;
    }
    if (offset != length)
        return false;
    *report_count = count;
    return true;
}
static int self_test(void)
{
    static const uint8_t two_reports[] = {
        0x3e, 34, 0x02, 2,
        0, 0, 6, 5, 4, 3, 2, 1, 5, 4, 0x09, 'A', 'C', '1', 0xc4,
        4, 1, 1, 2, 3, 4, 5, 6, 7, 6, 0xff, 0x12, 0x34, 'A', 'C', '2', 0x7f
    };
    static const uint8_t unrelated[] = {0x0e, 4, 1, 0x03, 0x0c, 0x00};
    static const uint8_t unrelated_name[] = {5, 0x09, 'A', 'C', '1', '0'};
    static const uint8_t bad_field[] = {6, 0x09, 'A', 'C', '1'};
    static const uint8_t conflicting_fields[] = {
        4, 0x09, 'A', 'C', '1', 4, 0x08, 'A', 'C', '2'
    };
    advertisement_t reports[2];
    size_t count;
    bool ok = parse_hci_event(two_reports, sizeof(two_reports), reports, 2, &count)
        && count == 2
        && reports[0].tag_id != NULL && strcmp(reports[0].tag_id, "AC1") == 0
        && reports[0].address[0] == 6 && reports[0].rssi == -60
        && reports[1].tag_id != NULL && strcmp(reports[1].tag_id, "AC2") == 0
        && reports[1].rssi == 127
        && parse_hci_event(unrelated, sizeof(unrelated), reports, 2, &count)
        && count == 0
        && !parse_hci_event(two_reports, sizeof(two_reports), reports, 1, &count)
        && !parse_hci_event(two_reports, sizeof(two_reports) - 1, reports, 2, &count)
        && tag_id_from_data(unrelated_name, sizeof(unrelated_name)) == NULL
        && tag_id_from_data(bad_field, sizeof(bad_field)) == NULL
        && tag_id_from_data(conflicting_fields, sizeof(conflicting_fields)) == NULL;
    uint8_t malformed[sizeof(two_reports)];
    memcpy(malformed, two_reports, sizeof(malformed));
    malformed[12] = 32;
    ok = ok && !parse_hci_event(malformed, sizeof(malformed), reports, 2, &count);
    static const uint8_t gv_name[] = {4, 0x09, 'G', 'V', '1'};
    const char *gv = tag_id_from_data(gv_name, sizeof(gv_name));
    ok = ok && gv != NULL && strcmp(gv, "GV1") == 0;
    puts(ok ? "SELF_TEST PASS" : "SELF_TEST FAIL");
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
enum {
    H4_COMMAND = 0x01,
    H4_EVENT = 0x04,
    HCI_COMMAND_COMPLETE = 0x0e,
    HCI_COMMAND_STATUS = 0x0f,
    COMMAND_TIMEOUT_MS = 2000,
    SCAN_WAIT_MS = 250
};
typedef struct {
    int fd;
    uint8_t frame[HCI_EVENT_MAX + 1];
    size_t used;
} uart_hci_t;
static volatile sig_atomic_t stop_requested;
static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}
static uint64_t monotonic_ms(void)
{
    struct timespec now = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}
static int wait_io(int fd, short events, uint64_t deadline)
{
    for (;;) {
        uint64_t now = monotonic_ms();
        if (now >= deadline)
            return ETIMEDOUT;
        struct pollfd p = {.fd = fd, .events = events};
        int rc = poll(&p, 1, (int)(deadline - now));
        if (rc == 0)
            return ETIMEDOUT;
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            return errno;
        }
        if (p.revents & events)
            return 0;
        if (p.revents & POLLNVAL)
            return EBADF;
        if (p.revents & (POLLERR | POLLHUP))
            return EIO;
    }
}
static int read_event(uart_hci_t *radio, size_t *length, uint64_t deadline)
{
    for (;;) {
        size_t need = 1;
        if (radio->used > 0) {
            if (radio->frame[0] != H4_EVENT) {
                fprintf(stderr, "Unexpected H4 packet type 0x%02x; check UART settings.\n",
                        (unsigned)radio->frame[0]);
                radio->used = 0;
                return EPROTO;
            }
            need = radio->used < 3 ? 3 : 3u + radio->frame[2];
        }
        if (radio->used == need) {
            *length = radio->used - 1;
            radio->used = 0;
            return 0;
        }
        int rc = wait_io(radio->fd, POLLIN, deadline);
        if (rc != 0)
            return rc;
        ssize_t got = read(radio->fd, radio->frame + radio->used, need - radio->used);
        if (got > 0) {
            radio->used += (size_t)got;
            continue;
        }
        if (got == 0)
            return EIO;
        if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
            return errno;
    }
}
static int hci_command(uart_hci_t *radio, const char *name, uint16_t opcode,
                       const uint8_t *parameters, uint8_t count)
{
    uint8_t command[4 + UINT8_MAX] = {H4_COMMAND, (uint8_t)opcode,
                                     (uint8_t)(opcode >> 8), count};
    if (count > 0)
        memcpy(command + 4, parameters, count);
    uint64_t deadline = monotonic_ms() + COMMAND_TIMEOUT_MS;
    size_t written = 0;
    size_t size = 4u + count;
    while (written < size) {
        int rc = wait_io(radio->fd, POLLOUT, deadline);
        if (rc != 0) {
            fprintf(stderr, "%s UART write: %s\n", name, strerror(rc));
            return rc;
        }
        ssize_t sent = write(radio->fd, command + written, size - written);
        if (sent > 0)
            written += (size_t)sent;
        else if (sent < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
            return errno;
    }
    for (;;) {
        size_t length = 0;
        int rc = read_event(radio, &length, deadline);
        if (rc != 0) {
            fprintf(stderr, "%s HCI reply: %s\n", name, strerror(rc));
            return rc;
        }
        const uint8_t *event = radio->frame + 1;
        if (length < 6)
            continue;
        if (event[0] == HCI_COMMAND_STATUS && event[2] != 0 &&
            event[4] == (uint8_t)opcode && event[5] == (uint8_t)(opcode >> 8)) {
            fprintf(stderr, "%s HCI status=0x%02x\n", name, (unsigned)event[2]);
            return EPROTO;
        }
        if (event[0] != HCI_COMMAND_COMPLETE || event[3] != (uint8_t)opcode ||
            event[4] != (uint8_t)(opcode >> 8))
            continue;
        if (event[5] != 0) {
            fprintf(stderr, "%s HCI status=0x%02x\n", name, (unsigned)event[5]);
            return EPROTO;
        }
        if (opcode == 0x1001 && length != 14) {
            fprintf(stderr, "Malformed Read Local Version reply.\n");
            return EPROTO;
        }
        printf("HCI_OK %s\n", name);
        if (opcode == 0x1001) {
            printf("HCI_VERSION hci=0x%02x revision=0x%04x manufacturer=0x%04x subversion=0x%04x\n",
                   (unsigned)event[6], event[7] | ((unsigned)event[8] << 8),
                   event[10] | ((unsigned)event[11] << 8),
                   event[12] | ((unsigned)event[13] << 8));
        }
        return 0;
    }
}
static int configure_scan(uart_hci_t *radio, bool *enable_attempted)
{
    const uint8_t event_mask[] = {0, 0, 0, 0, 0, 0, 0, 0x20};
    const uint8_t le_mask[] = {0x02, 0, 0, 0, 0, 0, 0, 0};
    /* Active legacy scan, 60 ms interval, 30 ms window, public own address. */
    const uint8_t scan_parameters[] = {1, 0x60, 0, 0x30, 0, 0, 0};
    const uint8_t scan_enable[] = {1, 0};
    int rc = hci_command(radio, "Set Event Mask", 0x0c01, event_mask, sizeof(event_mask));
    if (rc == 0)
        rc = hci_command(radio, "LE Set Event Mask", 0x2001, le_mask, sizeof(le_mask));
    if (rc == 0)
        rc = hci_command(radio, "LE Set Scan Parameters", 0x200b,
                         scan_parameters, sizeof(scan_parameters));
    if (rc == 0) {
        *enable_attempted = true;
        rc = hci_command(radio, "LE Scan Enable", 0x200c, scan_enable, sizeof(scan_enable));
    }
    return rc;
}
static int tag_number(const char *name)
{
    for (int tag = 0; tag < TAG_COUNT; ++tag) {
        if (strcmp(name, runway_tag_name((unsigned)tag)) == 0)
            return tag;
    }
    return -1;
}

static int send_tag(int coid, unsigned kind, unsigned tag, uint64_t now_ms, int rssi)
{
    struct runway_ble_message report = {
        .type = RUNWAY_BLE_MESSAGE, .version = RUNWAY_IPC_VERSION,
        .kind = kind, .tag = tag, .timestamp_ms = now_ms, .rssi_dbm = rssi
    };
    const uint64_t timeout_ns = UINT64_C(1000000000);
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &timeout_ns, NULL) == -1)
        return -1;
    return MsgSend(coid, &report, sizeof(report), NULL, 0);
}

static int send_observation(int coid, unsigned tag, uint64_t now_ms, uint64_t sequence,
                            const advertisement_t *advertisement)
{
    struct runway_fault_ble_observation report = {
        .type = RUNWAY_FAULT_BLE_OBSERVATION, .version = RUNWAY_IPC_VERSION,
        .tag = (uint16_t)tag, .address_type = advertisement->address_type,
        .timestamp_ms = now_ms, .sequence = sequence, .rssi_dbm = advertisement->rssi
    };
    memcpy(report.address, advertisement->address, sizeof(report.address));
    const uint64_t timeout_ns = UINT64_C(1000000000);
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,
                     NULL, &timeout_ns, NULL) == -1)
        return -1;
    return MsgSend(coid, &report, sizeof(report), NULL, 0);
}

static int gate_command(const char *name, unsigned kind)
{
    int tag = tag_number(name);
    if (tag < 0 || tag == TAG_GV1) {
        fputs("Use AC1 or AC2.\n", stderr);
        return EXIT_FAILURE;
    }
    runway_schedule("BLE operator command", SCHED_FIFO, RUNWAY_PRIO_CRITICAL);
    int coid = name_open(RUNWAY_CHANNEL, 0);
    if (coid == -1) {
        perror("Start runway_supervisor");
        return EXIT_FAILURE;
    }
    int result = send_tag(coid, kind, (unsigned)tag, monotonic_ms(), 127);
    if (result == -1)
        perror("Aircraft selection");
    name_close(coid);
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

static int scan(uart_hci_t *radio, unsigned long seconds, bool continuous, int coid)
{
    uint64_t total = 0, malformed = 0;
    uint64_t tags[TAG_COUNT] = {0}, next_publish[TAG_COUNT] = {0};
    uint64_t sequences[TAG_COUNT] = {0};
    uint64_t next_heartbeat = 0;
    advertisement_t last_published[TAG_COUNT] = {0};
    uint64_t end = continuous ? UINT64_MAX : monotonic_ms() + (uint64_t)seconds * 1000;
    int rc = 0;
    if (continuous)
        puts("SCAN_START mode=continuous");
    else
        printf("SCAN_START seconds=%lu\n", seconds);
    for (uint64_t now = monotonic_ms(); !stop_requested && now < end; now = monotonic_ms()) {
        uint64_t tick = now + SCAN_WAIT_MS;
        size_t length = 0;
        rc = read_event(radio, &length, tick < end ? tick : end);
        if (rc == 0 || rc == ETIMEDOUT)
            runway_heartbeat(coid, HEARTBEAT_BLE, monotonic_ms(), &next_heartbeat);
        if (rc == ETIMEDOUT) {
            rc = 0;
            continue;
        }
        if (rc != 0)
            break;
        uint64_t received_ms = monotonic_ms();
        advertisement_t reports[HCI_EVENT_MAX / 10];
        size_t count;
        if (!parse_hci_event(radio->frame + 1, length, reports,
                             sizeof(reports) / sizeof(reports[0]), &count)) {
            ++malformed;
            continue;
        }
        total += count;
        for (size_t i = 0; i < count; ++i) {
            const advertisement_t *report = &reports[i];
            if (report->tag_id == NULL)
                continue;
            int tag = tag_number(report->tag_id);
            if (tag < 0)
                continue;
            ++tags[tag];
            if (coid >= 0) {
                const advertisement_t *last = &last_published[tag];
                bool different_address = report->address_type != last->address_type ||
                    memcmp(report->address, last->address, sizeof(report->address)) != 0;
                /* Do not hide a second advertiser behind the per-tag throttle. */
                if (received_ms >= next_publish[tag] || different_address) {
                    if (send_observation(coid, (unsigned)tag, received_ms,
                                         ++sequences[tag], report) == -1)
                        return errno;
                    next_publish[tag] = received_ms + 100;
                    last_published[tag] = *report;
                }
                continue;
            }

            printf("BLE_ID id=%s rx_ms=%" PRIu64
                   " addr_type=%u addr=%02X:%02X:%02X:%02X:%02X:%02X",
                   report->tag_id, received_ms, (unsigned)report->address_type,
                   (unsigned)report->address[5], (unsigned)report->address[4],
                   (unsigned)report->address[3], (unsigned)report->address[2],
                   (unsigned)report->address[1], (unsigned)report->address[0]);
            if (report->rssi == 127)
                puts(" rssi_dbm=unavailable");
            else
                printf(" rssi_dbm=%d\n", report->rssi);
        }
    }
    printf("SCAN_END reports=%" PRIu64 " AC1=%" PRIu64 " AC2=%" PRIu64
           " GV1=%" PRIu64 " malformed=%" PRIu64 "\n",
           total, tags[TAG_AC1], tags[TAG_AC2], tags[TAG_GV1], malformed);
    return rc;
}
static int setup_serial(int fd, struct termios *saved)
{
    if (tcgetattr(fd, saved) != 0)
        return errno;
    struct termios config = *saved;
    config.c_iflag = 0;
    config.c_oflag = 0;
    config.c_lflag = 0;
    config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | HUPCL);
    config.c_cflag |= CS8 | CREAD | CLOCAL;
#if defined(__QNXNTO__)
    config.c_cflag |= IHFLOW | OHFLOW;
#elif defined(CRTSCTS)
    config.c_cflag |= CRTSCTS;
#endif
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 0;
    if (cfsetispeed(&config, B115200) != 0 || cfsetospeed(&config, B115200) != 0)
        return errno;
    if (tcsetattr(fd, TCSANOW, &config) != 0)
        return errno;
    if (tcflush(fd, TCIOFLUSH) != 0) {
        int rc = errno;
        tcsetattr(fd, TCSANOW, saved);
        return rc;
    }
    return 0;
}
int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--self-test") == 0)
        return self_test();
    if (argc == 3 && strcmp(argv[1], "--gate") == 0)
        return gate_command(argv[2], SELECT_GATE);
    if (argc == 3 && strcmp(argv[1], "--approach") == 0)
        return gate_command(argv[2], SET_APPROACH);
    if (argc == 3 && strcmp(argv[1], "--land") == 0)
        return gate_command(argv[2], REGISTER_LANDING);
    if (argc == 2 && strcmp(argv[1], "--reconcile") == 0) {
        runway_schedule("BLE reconcile command", SCHED_FIFO, RUNWAY_PRIO_CRITICAL);
        int coid = name_open(RUNWAY_CHANNEL, 0);
        if (coid == -1) { perror("Start runway_supervisor"); return EXIT_FAILURE; }
        int rc = send_tag(coid, RECONCILE_TRACKS, TAG_AC1, monotonic_ms(), 127);
        if (rc == -1) perror("Reconcile (both echoes and counts must be clear)");
        name_close(coid);
        return rc == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    bool use_defaults = argc == 1;
    const char *device = use_defaults ? "/dev/ser4" : argv[1];
    /* No arguments in Momentics = continuous BLE scan + QNX IPC. */
    bool publish = use_defaults || (argc == 4 && strcmp(argv[3], "--ipc") == 0);
    bool probe = argc == 3 && strcmp(argv[2], "--probe") == 0;
    bool continuous = use_defaults ||
        ((argc == 3 || publish) && strcmp(argv[2], "--continuous") == 0);
    unsigned long seconds = 0;
    if (argc >= 3 && !probe && argv[2][0] >= '0' && argv[2][0] <= '9') {
        char *end;
        errno = 0;
        seconds = strtoul(argv[2], &end, 10);
        if (errno != 0 || *end != '\0' || seconds > UINT32_MAX)
            seconds = 0;
    }
    if ((!use_defaults && argc != 3 && !publish) ||
        (!probe && !continuous && seconds == 0)) {
        fprintf(stderr, "Usage: %s  (continuous IPC scan on /dev/ser4)\n"
                        "       %s /dev/serN --probe\n"
                        "       %s /dev/serN (seconds | --continuous) [--ipc]\n"
                        "       %s (--gate | --approach) (AC1 | AC2)\n"
                        "       %s --reconcile\n",
                argv[0], argv[0], argv[0], argv[0], argv[0]);
        return EXIT_FAILURE;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    struct sigaction action = {0};
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) != 0 || sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }
    runway_schedule("BLE scanner", SCHED_RR, RUNWAY_PRIO_BLE);
    int coid = publish ? name_open(RUNWAY_FAULT_CHANNEL, 0) : -1;
    if (publish && coid == -1) {
        perror("Start runway_fault_manager before BLE scanner");
        return EXIT_FAILURE;
    }
    uart_hci_t radio = {0};
    radio.fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (radio.fd < 0) {
        perror(device);
        if (coid >= 0) name_close(coid);
        return EXIT_FAILURE;
    }
    struct termios saved;
    int rc = setup_serial(radio.fd, &saved);
    if (rc != 0) {
        fprintf(stderr, "UART setup: %s\n", strerror(rc));
        close(radio.fd);
        if (coid >= 0) name_close(coid);
        return EXIT_FAILURE;
    }
    printf("UART_OPEN path=%s baud=115200 flow=rtscts\n", device);
    bool enable_attempted = false;
    rc = hci_command(&radio, "Reset", 0x0c03, NULL, 0);
    if (rc == 0)
        rc = hci_command(&radio, "Read Local Version", 0x1001, NULL, 0);
    if (rc == 0 && !probe && !stop_requested)
        rc = configure_scan(&radio, &enable_attempted);
    if (rc == 0 && enable_attempted)
        rc = scan(&radio, seconds, continuous, coid);
    if (enable_attempted) {
        const uint8_t disable[] = {0, 0};
        int stopped = hci_command(&radio, "LE Scan Disable", 0x200c, disable, sizeof(disable));
        if (rc == 0)
            rc = stopped;
    }
    if (rc != 0)
        fprintf(stderr, "BLE UART test failed: %s\n", strerror(rc));
    if (tcsetattr(radio.fd, TCSANOW, &saved) != 0 && rc == 0) {
        rc = errno;
        perror("UART restore");
    }
    close(radio.fd);
    if (coid >= 0)
        name_close(coid);
    return rc == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
