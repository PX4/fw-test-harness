#include <cstdint>

// XXX just some hacks for now, provide interface for time later
#define ecl_absolute_time() 0
#define ecl_elapsed_time(_arg) *(_arg)
typedef std::uint64_t    hrt_abstime;

