#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "daq_mirror_rtl_tcp.h"
#include "log.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

static SOCKET s;

static void *tcp_worker(void *arg)
{
    dm_rtl_tcp_priv_t *priv = (dm_rtl_tcp_priv_t *)arg;
    ring_buf_entry_t *entry;
    int bytesleft, bytessent, index;
    struct timeval tv = { 1, 0 };
    struct timespec ts;
    struct timeval tp;
    fd_set writefds;
    int r = 0;
    uint32_t i;

    pthread_mutex_lock(&priv->client_count_mutex);
    priv->client_count += 1;
    pthread_mutex_unlock(&priv->client_count_mutex);

    while(1) {
        if(priv->terminate)
            pthread_exit(0);

        pthread_mutex_lock(&priv->ch_data_cond_mutex);
        gettimeofday(&tp, NULL);
        ts.tv_sec  = tp.tv_sec+1;
        ts.tv_nsec = tp.tv_usec * 1000;
        r = pthread_cond_timedwait(&priv->ch_data_cond, &priv->ch_data_cond_mutex, &ts);
        if(r == ETIMEDOUT) {
            pthread_mutex_unlock(&priv->ch_data_cond_mutex);
            log_warn("TCP mirror: worker cond timeout");
            //sighandler(0); TODO: check if we need one
            pthread_exit(NULL);
        }

        pthread_mutex_unlock(&priv->ch_data_cond_mutex);

        pthread_mutex_lock(&priv->ch_data_mutex);
        i = priv->ch_data.start_idx;
        entry = &priv->ch_data.entries[i];
        while(i != priv->ch_data.end_idx) {
            bytesleft = entry->len;
            index = 0;
            bytessent = 0;
            while(bytesleft > 0) {
                FD_ZERO(&writefds);
                FD_SET(s, &writefds);
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                r = select(s+1, NULL, &writefds, NULL, &tv);
                if(r) {
                    bytessent = send(s,  &entry->data[index], bytesleft, 0);
                    bytesleft -= bytessent;
                    index += bytessent;
                }
                if(bytessent == SOCKET_ERROR || priv->terminate) {
                    pthread_mutex_unlock(&priv->ch_data_mutex);
                    log_info("TCP mirror: worker socket bye");
                    //sighandler(0); TODO: check if we need one
                    pthread_exit(NULL);
                }
            }

            i++;
                /*prev = curelem;
            curelem = curelem->next;
            free(prev->data);
            free(prev);*/
        }
        pthread_mutex_unlock(&priv->ch_data_mutex);
    }

    pthread_mutex_lock(&priv->client_count_mutex);
    priv->client_count -= 1;
    pthread_mutex_unlock(&priv->client_count_mutex);

    // TODO: if client count is 0, clean up ring buffer
}

static void _listen_socket(dm_rtl_tcp_priv_t *priv)
{
    SOCKET listensocket;
    struct sockaddr_in local, remote;
    socklen_t rlen;
    int r;
    char* addr = "0.0.0.0";
    struct timeval tv = { 1, 0 };
    struct linger ling = { 1, 0 };
    fd_set readfds;
    pthread_t tcp_worker_thread;
    dongle_info_t dongle_info;
    void *status;

    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(priv->port);
    local.sin_addr.s_addr = inet_addr(addr);

    listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    r = 1;
    setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
    setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
    bind(listensocket, (struct sockaddr *)&local, sizeof(local));

#ifdef _WIN32
    ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
    r = fcntl(listensocket, F_GETFL, 0);
    r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

    while(1) {
        /*printf("listening...\n");
        printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
               "(gr-osmosdr) source\n"
               "to receive samples in GRC and control "
               "rtl_tcp parameters (frequency, gain, ...).\n",
               addr, priv->port);*/
        listen(listensocket, 1);

        while(1) {
            FD_ZERO(&readfds);
            FD_SET(listensocket, &readfds);
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            r = select(listensocket+1, &readfds, NULL, NULL, &tv);
            if(priv->terminate) {
                goto out;
            } else if(r) {
                rlen = sizeof(remote);
                s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
                break;
            }
        }

        setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

        //printf("client accepted!\n");

        memset(&dongle_info, 0, sizeof(dongle_info));
        memcpy(&dongle_info.magic, "RTL0", 4);

        /*r = rtlsdr_get_tuner_type(dev);
        if (r >= 0)
            dongle_info.tuner_type = htonl(r);

        r = rtlsdr_get_tuner_gains(dev, gains);
        if (r >= 0)
            dongle_info.tuner_gain_count = htonl(r);*/

        r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
        if (sizeof(dongle_info) != r) {
            log_error("TCP mirror: failed to send dongle information");
        }

        pthread_create(&tcp_worker_thread, NULL, tcp_worker, (void *)priv);

        pthread_join(tcp_worker_thread, &status);

        closesocket(s);
    }
    out:
        return;
}

static void * _listen_thread_fn(void *arg)
{
    dm_rtl_tcp_priv_t *priv = (dm_rtl_tcp_priv_t *)arg;

    _listen_socket(priv);

    return NULL;
}

static int32_t _start(dm_t *dm)
{
    dm_rtl_tcp_priv_t *priv = dm->priv_data;
    pthread_t *thread = &priv->listen_thread;
    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(thread, &attr, &_listen_thread_fn, (void *)priv);

    // TODO: cancel spawned listening thread
    // TODO: set termination flag

    pthread_attr_destroy(&attr);

    return 0;
}

/* Must be called under a lock */
static int32_t _ring_buf_write(ring_buf_t *rb, uint8_t *data, size_t len)
{
    ring_buf_entry_t *entry = &rb->entries[rb->end_idx];

    if (entry->data) {
        log_debug("TCP mirror: freeing entry %d", rb->end_idx);
        free(entry->data);
        entry->data = NULL;
    }

    entry->data = malloc(len);
    if (entry->data == NULL) {
        log_error("TCP mirror: failed to allocate memory for chain data");
        return ENOMEM;
    }

    memcpy(entry->data, data, len);
    entry->len = len;
    rb->end_idx = (rb->end_idx + 1) % DM_RTL_TCP_RING_SIZE;

    if (rb->end_idx == rb->start_idx) {
        if (rb->entries[rb->end_idx].data != NULL) {
            free(rb->entries[rb->end_idx].data);
            rb->entries[rb->end_idx].data = NULL;
            rb->entries[rb->end_idx].len = 0;
        }
        rb->start_idx = (rb->start_idx + 1) % DM_RTL_TCP_RING_SIZE;
    }

    return len;
}

// TODO: it makes more sense to move it to daq_mirror.c, and then
//       only implement reading part in the inherited mirrors.
// TODO: don't allow if mirror has not been started
static int32_t _copy_ch_data(dm_t *dm, uint8_t *data, size_t len)
{
    dm_rtl_tcp_priv_t *priv = dm->priv_data;
    bool active_clients;

    pthread_mutex_lock(&priv->client_count_mutex);
    active_clients = priv->client_count > 0;
    pthread_mutex_unlock(&priv->client_count_mutex);

    /* Ignore the data if there is no client to read it  */
    if (! active_clients) {
        return 0;
    }

    pthread_mutex_lock(&priv->ch_data_mutex);
    _ring_buf_write(&priv->ch_data, data, len);
    pthread_mutex_unlock(&priv->ch_data_mutex);

    pthread_mutex_lock(&priv->ch_data_cond_mutex);
    pthread_cond_broadcast(&priv->ch_data_cond);
    pthread_mutex_unlock(&priv->ch_data_cond_mutex);

    return 0;
}

// call upon the last client disconnect
// ring_buf_free()

// TODO: call from rtl_daq.c
static void _free(dm_t *dm)
{
    dm_rtl_tcp_priv_t *priv;

    if (!dm) {
        return;
    }

    priv = dm->priv_data;

    //TODO: _ring_buf_free();

    pthread_cond_destroy(&priv->ch_data_cond);
    pthread_mutex_destroy(&priv->ch_data_cond_mutex);
    pthread_mutex_destroy(&priv->ch_data_mutex);
    pthread_mutex_destroy(&priv->client_count_mutex);

    dm->free(dm);
}

dm_t *dm_rtl_tcp_new(uint16_t port)
{
    dm_t *ret = dm_new(sizeof(dm_rtl_tcp_priv_t));
    dm_rtl_tcp_priv_t *priv;

    if (!ret) {
        return NULL;
    }

    ret->start = _start;
    ret->copy_ch_data = _copy_ch_data;
    ret->free = _free;

    priv = (dm_rtl_tcp_priv_t *)ret->priv_data;
    priv->port = port;

    pthread_mutex_init(&priv->client_count_mutex, NULL);
    pthread_mutex_init(&priv->ch_data_mutex, NULL);
    pthread_mutex_init(&priv->ch_data_cond_mutex, NULL);
    pthread_cond_init(&priv->ch_data_cond, NULL);

    return ret;
}
