#ifndef DAQ_MIRROR_RTL_TCP_H
#define DAQ_MIRROR_RTL_TCP_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/queue.h>

#include "daq_mirror.h"

#define DM_RTL_TCP_RING_SIZE    20

typedef struct ring_buf_entry {
	char                *data;
	size_t              len;
} ring_buf_entry_t;

typedef struct ring_buf {
    ring_buf_entry_t	entries[DM_RTL_TCP_RING_SIZE];
    uint32_t            start_idx;
    uint32_t            end_idx;
} ring_buf_t;

typedef struct dm_rtl_tcp_priv {
    uint16_t            port;
    bool                terminate;
    pthread_t           listen_thread;
    bool                client_count;
    pthread_mutex_t     client_count_mutex;
    ring_buf_t          ch_data;
    pthread_mutex_t     ch_data_mutex;
    pthread_cond_t      ch_data_cond;
    pthread_mutex_t     ch_data_cond_mutex;
} dm_rtl_tcp_priv_t;

dm_t *dm_rtl_tcp_new(uint16_t port);

#endif /* DAQ_MIRROR_RTL_TCP_H */
