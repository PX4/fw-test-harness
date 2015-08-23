#include <string.h>
#include "systemlib/param/param.h"
#include "param_load.h"

// in the fw test harness the params used by the blocks classes in mtecs need
// to be loaded through these functions, this file is linked instead of the
// Firmware param.c
// code is adapted from Firmware param.c
#define N_PARAMS_MAX 100
#define UNUSED(x) (void)(x)
static struct param_info_s params_[N_PARAMS_MAX];
static unsigned n_params_ = 0;
static const struct param_info_s *param_info_base = (struct param_info_s *) &(params_[0]);

void load_params(struct param_info_s params[], size_t n_params)
{
    n_params_ = sizeof(params_)/sizeof(params_[0]) ? n_params : sizeof(params_)/sizeof(params_[0]);
    for (size_t i = 0; i < n_params_; ++i)
    {
	memcpy(&(params_[i]), &(params[i]), sizeof(params_[i]));
    }
}

static unsigned
get_param_info_count(void)
{
    return n_params_;
}

static bool
handle_in_range(param_t param)
{
    unsigned count = get_param_info_count();
    return (count && param < count);
}

static const void *
param_get_value_ptr(param_t param)
{
    const void *result = NULL;

    if (handle_in_range(param)) {
	result = &param_info_base[param].val;
    }

    return result;

}

size_t
param_size(param_t param)
{
    UNUSED(param);
    return sizeof(float);
}

int
param_get(param_t param, void *val)
{
    int result = -1;

    const void *v = param_get_value_ptr(param);

    if (val != NULL) {
	memcpy(val, v, param_size(param));
	result = 0;
    }

    return result;
}

param_t
param_find(const char *name)
{
    param_t param;


    /* perform a linear search of the known parameters */
    for (param = 0; handle_in_range(param); param++) {
	if (!strcmp(param_info_base[param].name, name)) {
	    return param;
	}
    }

    /* not found */
    return PARAM_INVALID;
}
