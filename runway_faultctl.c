#define _QNX_SOURCE 1

#include "runway_fault_ipc.h"
#include "runway_schedule.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

static unsigned parse_number(const char *text, unsigned maximum)
{
    char *end;
    errno = 0;
    unsigned long value = strtoul(text, &end, 10);
    return errno || end == text || *end || value > maximum ? 0 : (unsigned)value;
}

static void *runway_load_worker(void *arg)
{
    const uint64_t end = *(const uint64_t *)arg;
    runway_schedule("test CPU load", SCHED_RR, RUNWAY_PRIO_LOAD);
    volatile uint64_t work = 1;
    uint64_t now;
    do {
        for (unsigned i = 0; i < 4096; ++i)
            work = work * UINT64_C(1664525) + UINT64_C(1013904223);
    } while (runway_monotonic_ms(&now) == 0 && now < end);
    return NULL;
}

static int cpu_load(unsigned seconds, unsigned workers)
{
    uint64_t now;
    if (runway_monotonic_ms(&now) != 0)
        return EXIT_FAILURE;
    uint64_t end = now + (uint64_t)seconds * 1000;
    pthread_t threads[4];
    unsigned started = 0;
    int error = 0;
    for (; started < workers; ++started) {
        error = pthread_create(&threads[started], NULL, runway_load_worker, &end);
        if (error)
            break;
    }
    for (unsigned i = 0; i < started; ++i)
        pthread_join(threads[i], NULL);
    if (error)
        fprintf(stderr, "Start load worker: %s\n", strerror(error));
    return error ? EXIT_FAILURE : EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
    struct runway_fault_test_command command = {
        .type = RUNWAY_FAULT_TEST_COMMAND, .version = RUNWAY_IPC_VERSION
    };
    if (argc == 3 && strcmp(argv[1], "delay") == 0 &&
        (command.delay_ms = parse_number(argv[2], 10000)) > RUNWAY_MAX_AGE_MS) {
        command.action = TEST_DELAY_AIR;
    } else if (argc == 2 && strcmp(argv[1], "replay-button") == 0) {
        command.action = TEST_REPLAY_BUTTON;
    } else if (argc == 2 && strcmp(argv[1], "identity-ac1") == 0) {
        command.action = TEST_CONFLICT_AC1;
    } else if ((argc == 3 || argc == 4) && strcmp(argv[1], "load-cpu") == 0) {
        unsigned seconds = parse_number(argv[2], 60);
        unsigned workers = argc == 4 ? parse_number(argv[3], 4) : 4;
        if (seconds && workers)
            return cpu_load(seconds, workers);
        goto usage;
    } else {
        goto usage;
    }

    int coid = name_open(RUNWAY_FAULT_CHANNEL, 0);
    if (coid == -1) {
        perror("Connect to runway_fault_manager");
        return EXIT_FAILURE;
    }
    const uint64_t timeout_ns = UINT64_C(1000000000);
    int rc = TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,
                          NULL, &timeout_ns, NULL);
    if (rc != -1)
        rc = MsgSend(coid, &command, sizeof(command), NULL, 0);
    if (rc == -1)
        perror("Fault test request");
    else
        puts("Fault test accepted");
    name_close(coid);
    return rc == -1 ? EXIT_FAILURE : EXIT_SUCCESS;

usage:
    fprintf(stderr, "Usage: %s delay MILLISECONDS | replay-button | identity-ac1 | load-cpu SECONDS [WORKERS]\n",
            argv[0]);
    return EXIT_FAILURE;
}
