#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
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

static void _listen_socket(dm_rtl_tcp_priv_t *priv)
{
    SOCKET listensocket, s;
    struct sockaddr_in local, remote;
    socklen_t rlen;
    int r;
    char* addr = "127.0.0.1";
    struct timeval tv = {1,0};
    struct linger ling = {1,0};
    fd_set readfds;

    memset(&local,0,sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(priv->port);
    local.sin_addr.s_addr = inet_addr(addr);

    listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    r = 1;
    setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
    setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
    bind(listensocket,(struct sockaddr *)&local,sizeof(local));

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
        listen(listensocket,1);

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

        /*memset(&dongle_info, 0, sizeof(dongle_info));
        memcpy(&dongle_info.magic, "RTL0", 4);

        r = rtlsdr_get_tuner_type(dev);
        if (r >= 0)
            dongle_info.tuner_type = htonl(r);

        r = rtlsdr_get_tuner_gains(dev, gains);
        if (r >= 0)
            dongle_info.tuner_gain_count = htonl(r);
        if (verbosity)
        {
            fprintf(stderr, "Supported gain values (%d): ", r);
            for (i = 0; i < r; i++)
                fprintf(stderr, "%.1f ", gains[i] / 10.0);
            fprintf(stderr, "\n");
        }

        r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
        if (sizeof(dongle_info) != r)
            printf("failed to send dongle information\n");
        */
        /*
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
        r = pthread_create(&command_thread, &attr, command_worker, NULL);
        pthread_attr_destroy(&attr);

        r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, buf_len);

        pthread_join(tcp_worker_thread, &status);
        pthread_join(command_thread, &status);
        */
        closesocket(s);
    }
    out:
        log_info("the end");
    //printf("the end\n");
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

    pthread_attr_destroy(&attr);

    //pthread_join(listen_thread, NULL);

    return 0;
}

static int32_t _cache_ch_data(dm_t *dm)
{
    return 0;
}

static void _free(dm_t *dm)
{
    if (!dm) {
        return;
    }

    dm->free(dm);
}

dm_t *dm_rtl_tcp_new(uint16_t port)
{
    dm_t *ret = dm_new(sizeof(dm_rtl_tcp_priv_t));
    dm_rtl_tcp_priv_t *priv;

    log_error("dm_rtl_tcp_new()");

    if (!ret) {
        return NULL;
    }

    ret->start = _start;
    ret->cache_ch_data = _cache_ch_data;
    ret->free = _free;

    priv = (dm_rtl_tcp_priv_t *)ret->priv_data;
    priv->port = port;

    return ret;
}
