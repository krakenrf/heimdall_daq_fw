#ifndef DAQ_MIRROR_RTL_TCP_H
#define DAQ_MIRROR_RTL_TCP_H

#include <stdbool.h>
#include <stdint.h>

#include "daq_mirror.h"

typedef struct dm_rtl_tcp_priv {
    uint16_t        port;
    bool            terminate;
    pthread_t       listen_thread;
} dm_rtl_tcp_priv_t;

dm_t *dm_rtl_tcp_new(uint16_t port);

#endif /* DAQ_MIRROR_RTL_TCP_H */
