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

typedef struct {
    dm_rtl_tcp_priv_t *priv;
    SOCKET s;
} tcp_worker_arg_t;

static bool terminate = false;

static void *tcp_worker(void *a)
{
    tcp_worker_arg_t *arg = (tcp_worker_arg_t *) a;
    dm_rtl_tcp_priv_t *priv = arg->priv;
    dm_t *dm = priv->dm;
    data_entry_t *entry;
    int bytesleft, bytessent, index;
    struct timeval tv = { 1, 0 };
    struct timespec ts;
    struct timeval tp;
    fd_set writefds;
    int r = 0;
    bool locked;

    dm_inc_client_count(dm);

    while(1) {
        if(terminate) {
            pthread_exit(0);
        }

        pthread_mutex_lock(&dm->ch_data_cond_mutex);
        gettimeofday(&tp, NULL);
        ts.tv_sec  = tp.tv_sec+1;
        ts.tv_nsec = tp.tv_usec * 1000;
        r = pthread_cond_timedwait(&dm->ch_data_cond, &dm->ch_data_cond_mutex, &ts);
        if(r == ETIMEDOUT) {
            pthread_mutex_unlock(&dm->ch_data_cond_mutex);
            log_warn("Worker cond timeout");
            //sighandler(0); TODO: check if we need one
            pthread_exit(NULL);
        }
        pthread_mutex_unlock(&dm->ch_data_cond_mutex);

        while (!SLIST_EMPTY(&dm->ch_data)) {
            entry = SLIST_FIRST(&dm->ch_data);
            if (!SLIST_NEXT(entry, entries)) {
                pthread_mutex_lock(&dm->ch_data_mutex);
                locked = true;
            }
            bytesleft = entry->len;
            index = 0;
            bytessent = 0;
            while(bytesleft > 0) {
                FD_ZERO(&writefds);
                FD_SET(arg->s, &writefds);
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                r = select(arg->s+1, NULL, &writefds, NULL, &tv);
                if(r) {
                    bytessent = send(arg->s,  &entry->data[index], bytesleft, 0);
                    bytesleft -= bytessent;
                    index += bytessent;
                }
                if(bytessent == SOCKET_ERROR || terminate) {
                    pthread_mutex_unlock(&dm->ch_data_mutex);
                    log_info("Worker socket bye");
                    //sighandler(0); TODO: check if we need one
                    pthread_exit(NULL);
                }
            }

            if (locked) {
                pthread_mutex_unlock(&dm->ch_data_mutex);
                locked = false;
            }

            dm_on_entry_read_done(dm, entry);
        }
    }

    dm_dec_client_count(dm);

    closesocket(arg->s);
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
    tcp_worker_arg_t *arg;
    SOCKET s;

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
            if(terminate) {
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
            log_error("Failed to send dongle information");
        }

        arg = malloc(sizeof(tcp_worker_arg_t));
        if (!arg) {
            log_error("Failed to allocate memory for TCP worked arg");
            goto out;
        }

        arg->priv = priv;
        arg->s = s;

        pthread_create(&tcp_worker_thread, NULL, tcp_worker, (void *)arg);

        pthread_join(tcp_worker_thread, &status);
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

// call upon the last client disconnect

static void _free(dm_t *dm)
{
    if (!dm) {
        return;
    }

    terminate = true;

    dm_free(dm);
}

dm_t *dm_rtl_tcp_new(uint16_t port)
{
    dm_t *ret = dm_new(sizeof(dm_rtl_tcp_priv_t));
    dm_rtl_tcp_priv_t *priv;

    if (!ret) {
        log_error("Failed to create a TCP mirror");
        return NULL;
    }

    ret->start = _start;
    ret->free = _free;

    priv = (dm_rtl_tcp_priv_t *)ret->priv_data;
    priv->dm = ret;
    priv->port = port;

    return ret;
}
