#ifndef DAQ_MIRROR_H
#define DAQ_MIRROR_H

#include <sys/queue.h>

#define DAQ_MIRROR_MEM_LIMIT 100000000UL /* 100 MB */

typedef struct data_entry {
	char                    *data;
	size_t                  len;
    uint8_t                 read_count;
    SLIST_ENTRY(data_entry) entries;
} data_entry_t;

SLIST_HEAD(channel_data, data_entry);
typedef struct channel_data channel_data_t;

typedef struct daq_mirror dm_t;

struct daq_mirror {
    uint32_t         client_count;
    pthread_mutex_t  client_count_mutex;
    channel_data_t   ch_data;
    pthread_mutex_t  ch_data_mutex;
    pthread_cond_t   ch_data_cond;
    pthread_mutex_t  ch_data_cond_mutex;
    size_t           used_memory;

    int32_t (*start)(dm_t *dm);
    uint32_t (*copy_ch_data)(dm_t *dm, uint8_t *ptr, size_t len);
    void    (*free)(dm_t *dm);

    void    *priv_data;
};

dm_t *dm_new(size_t priv_data_size);
void dm_free(dm_t *dm);
void dm_inc_client_count(dm_t *dm);
void dm_dec_client_count(dm_t *dm);
void dm_on_entry_read_done(dm_t *dm, data_entry_t *e);



#endif /* DAQ_MIRROR_H */
