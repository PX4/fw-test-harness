#include <time.h>
#include <drivers/drv_hrt.h>

static hrt_abstime px4_timestart = 0;

/*
 * Convert a timespec to absolute time.
 */
hrt_abstime ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/*
 * Get absolute time.
 */
hrt_abstime hrt_absolute_time(void)
{
	struct timespec ts;

	if (!px4_timestart) {
		clock_gettime(CLOCK_MONOTONIC, &ts);
		px4_timestart = ts_to_abstime(&ts);
	}

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts) - px4_timestart;
}
