#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "daq_mirror.h"

dm_t *dm_new(size_t priv_data_size)
{
    dm_t *ret = calloc(1, sizeof(dm_t) + priv_data_size);
	if (!ret) {
		return NULL;
	}

	if (priv_data_size) {
		ret->priv_data = ret + 1;
	}

    return ret;
}
