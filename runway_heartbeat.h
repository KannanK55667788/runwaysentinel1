#ifndef RUNWAY_HEARTBEAT_H
#define RUNWAY_HEARTBEAT_H

#include "runway_fault_ipc.h"
#include <sys/neutrino.h>

/* Called from the worker that made progress, including cycles with no event. */
static inline void runway_heartbeat(int coid, enum runway_heartbeat_source source,
                                     uint64_t now, uint64_t *next)
{
    if (coid < 0 || now < *next)
        return;
    const struct runway_heartbeat_message message = {
        .type = RUNWAY_FAULT_HEARTBEAT, .version = RUNWAY_IPC_VERSION,
        .source = source, .timestamp_ms = now
    };
    const uint64_t timeout_ns = UINT64_C(20000000);
    *next = now + RUNWAY_HEARTBEAT_INTERVAL_MS;
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,
                     NULL, &timeout_ns, NULL) != -1)
        (void)MsgSend(coid, &message, sizeof(message), NULL, 0);
}

#endif
