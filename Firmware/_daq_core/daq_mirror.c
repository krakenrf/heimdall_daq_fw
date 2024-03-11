#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "daq_mirror.h"
#include "log.h"


static void _free_ch_data(dm_t *dm)
{
    data_entry_t *e;

    pthread_mutex_lock(&dm->ch_data_mutex);
    while (!SLIST_EMPTY(&dm->ch_data)) {
        e = SLIST_FIRST(&dm->ch_data);
        free(e->data);
        SLIST_REMOVE_HEAD(&dm->ch_data, entries);
        free(e);
    }
    SLIST_INIT(&dm->ch_data);
    pthread_mutex_unlock(&dm->ch_data_mutex);
}

void dm_inc_client_count(dm_t *dm)
{
    if (dm == NULL) {
        log_error("NULL mirror pointer");
    }

    pthread_mutex_lock(&dm->client_count_mutex);
    dm->client_count += 1;
    pthread_mutex_unlock(&dm->client_count_mutex);
}

void dm_dec_client_count(dm_t *dm)
{
    uint32_t client_count;

    if (dm == NULL) {
        log_error("NULL mirror pointer");
    }

    pthread_mutex_lock(&dm->client_count_mutex);
    dm->client_count -= 1;
    client_count = dm->client_count;
    pthread_mutex_unlock(&dm->client_count_mutex);

    if (client_count == 0) {
        _free_ch_data(dm);
    }
}

uint32_t dm_get_client_count(dm_t *dm)
{
    uint32_t ret = 0;

    if (dm == NULL) {
        log_error("NULL mirror pointer");
        return 0;
    }

    pthread_mutex_lock(&dm->client_count_mutex);
    ret = dm->client_count;
    pthread_mutex_unlock(&dm->client_count_mutex);

    return ret;
}

uint32_t dm_used_memory(dm_t *dm)
{
    uint32_t ret = 0;

    if (dm == NULL) {
        log_error("NULL mirror pointer");
        return 0;
    }

    pthread_mutex_lock(&dm->client_count_mutex);
    ret = dm->used_memory;
    pthread_mutex_unlock(&dm->client_count_mutex);

    return ret;
}

void dm_on_entry_read_done(dm_t *dm, data_entry_t *e)
{
    if (dm == NULL || e == NULL) {
        log_warn("NULL argument in dm_on_entry_read_done()");
        return;
    }

    pthread_mutex_lock(&dm->ch_data_mutex);
    e->read_count += 1;
    if (e->read_count >= dm->client_count) {
        /* all clients have read this entry, so remove it */
        SLIST_REMOVE(&dm->ch_data, e, data_entry, entries);
    }
    pthread_mutex_unlock(&dm->ch_data_mutex);
}

// TODO: don't allow if mirror has not been started
static uint32_t _copy_ch_data(dm_t *dm, uint8_t *data, size_t len)
{
    data_entry_t *entry = NULL;

    /* Ignore the data if there is no client to read it  */
    if (dm_get_client_count(dm) == 0) {
        return 0;
    }

    if (dm_used_memory(dm) + len > DAQ_MIRROR_MEM_LIMIT) {
        log_warn("DAQ mirror memory limit %u reached, ignoring data len %u",
                 DAQ_MIRROR_MEM_LIMIT, len);
        return 0;
    }

    entry = calloc(1, sizeof(data_entry_t));
    if (entry == NULL) {
        log_warn("Failed to allocate memory for a channel data entry");
        return 0;
    }
    entry->data = malloc(len);
    if (entry->data == NULL) {
        log_warn("Failed to allocate memory for channel data copy");
        free(entry);
        return 0;
    }

    memcpy(entry->data, data, len);
    entry->len = len;

    pthread_mutex_lock(&dm->ch_data_mutex);
    SLIST_INSERT_HEAD(&dm->ch_data, entry, entries);
    dm->used_memory += sizeof(data_entry_t) + len;
    pthread_mutex_unlock(&dm->ch_data_mutex);

    pthread_mutex_lock(&dm->ch_data_cond_mutex);
    pthread_cond_broadcast(&dm->ch_data_cond);
    pthread_mutex_unlock(&dm->ch_data_cond_mutex);

    return 0;
}

void dm_free(dm_t *dm)
{
    if (!dm) {
        return;
    }

    _free_ch_data(dm);

    pthread_cond_destroy(&dm->ch_data_cond);
    pthread_mutex_destroy(&dm->ch_data_cond_mutex);
    pthread_mutex_destroy(&dm->ch_data_mutex);
    pthread_mutex_destroy(&dm->client_count_mutex);
}

dm_t *dm_new(size_t priv_data_size)
{
    dm_t *ret = calloc(1, sizeof(dm_t) + priv_data_size);
	if (!ret) {
		return NULL;
	}

    pthread_mutex_init(&ret->client_count_mutex, NULL);

    SLIST_INIT(&ret->ch_data);
    pthread_mutex_init(&ret->ch_data_mutex, NULL);

    pthread_mutex_init(&ret->ch_data_cond_mutex, NULL);

    pthread_cond_init(&ret->ch_data_cond, NULL);

    ret->copy_ch_data = _copy_ch_data;
    ret->free = dm_free;

    if (priv_data_size) {
		ret->priv_data = ret + 1;
	}

    return ret;
}
