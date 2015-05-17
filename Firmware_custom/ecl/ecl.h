#include <stdint.h>
#include <chrono>

#pragma once

using namespace std::chrono;

typedef std::uint64_t hrt_abstime;
inline hrt_abstime abs_time_us() {
    return (hrt_abstime)duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

#define ecl_absolute_time() (abs_time_us())
#define ecl_elapsed_time(_arg) (ecl_absolute_time() - *_arg)

