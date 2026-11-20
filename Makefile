QCC ?= qcc
TARGET ?= gcc_ntoaarch64le
CFLAGS ?= -O2 -g -Wall -Wextra
BINARIES = runway_supervisor runway_fault_manager runway_sensors runway_ble_uart runway_faultctl
HEADERS = runway_ipc.h runway_fault_ipc.h runway_schedule.h runway_heartbeat.h

.PHONY: all clean
all: $(BINARIES)

%: %.c $(HEADERS)
	$(QCC) -V$(TARGET) -std=c11 $(CFLAGS) -pthread -o $@ $<

clean:
	rm -f $(BINARIES)
