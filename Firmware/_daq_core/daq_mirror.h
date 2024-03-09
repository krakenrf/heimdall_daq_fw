#ifndef DAQ_MIRROR_H
#define DAQ_MIRROR_H

typedef struct daq_mirror dm_t;

struct daq_mirror {
    int32_t (*start)(dm_t *dm);
    int32_t (*copy_ch_data)(dm_t *dm, uint8_t *ptr, size_t len);
    void    (*free)(dm_t *dm);

    void    *priv_data;
};

dm_t *dm_new(size_t priv_data_size);

#endif /* DAQ_MIRROR_H */
