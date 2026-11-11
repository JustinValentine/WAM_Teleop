#pragma once

/* Keeps every CPU out of its deep idle states while the teleop runs: the
 * kernel's PM QoS request, /dev/cpu_dma_latency held at 0 us until closed.
 * The deepest state here takes about 1 ms to leave; with it on, the UDP
 * threads took 0.2 ms (median) and up to 3.6 ms to pass a packet between the
 * two programs, and the 500 Hz loop woke up to 1.3 ms late. Held at 0: 12 us
 * median, 0.3-1 ms worst. A packet later than the phase between the two
 * loops misses the other arm's next cycle (2 ms more delay). */

#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>

class CpuLatencyHold {
public:
	CpuLatencyHold() : fd(open("/dev/cpu_dma_latency", O_RDWR)) {
		const int32_t zero = 0;
		if (fd < 0 || write(fd, &zero, sizeof(zero)) != sizeof(zero)) {
			printf("Warning: cannot hold /dev/cpu_dma_latency at 0: deep CPU idle states stay on "
					"(slower UDP between the arms)\n");
		}
	}
	~CpuLatencyHold() {
		if (fd >= 0) {
			close(fd);
		}
	}

private:
	int fd;

	CpuLatencyHold(const CpuLatencyHold&);
	CpuLatencyHold& operator=(const CpuLatencyHold&);
};
