#ifndef RUNWAY_SCHEDULE_H
#define RUNWAY_SCHEDULE_H

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

enum {
    RUNWAY_PRIO_LOAD = 5,
    RUNWAY_PRIO_ROUTINE = 10,
    RUNWAY_PRIO_BLE = 18,
    RUNWAY_PRIO_RANGE = 20,
    RUNWAY_PRIO_BUTTON = 23,
    RUNWAY_PRIO_INTAKE = 24,
    RUNWAY_PRIO_IR = 24,
    RUNWAY_PRIO_CRITICAL = 30
};

/* Continue with the OS-assigned policy if the target denies the request. */
static inline void runway_schedule(const char *thread, int policy, int priority)
{
    const struct sched_param parameters = {.sched_priority = priority};
    int result = pthread_setschedparam(pthread_self(), policy, &parameters);
    if (result != 0)
        fprintf(stderr, "%s: cannot set priority %d: %s\n", thread, priority, strerror(result));
}

#endif
