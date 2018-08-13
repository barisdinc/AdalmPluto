/*
 * ad9361.h
 *
 *  Created on: Oct 20, 2016
 *      Author: marcin
 */
 
#ifndef AD9361_H_
#define AD9361_H_
 
 
/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
 
/* RX is input, TX is output */
enum iodev {
    RX, TX
};
//float time[1500000];
/* common RX and TX streaming params */
struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
};
 
/* static scratch mem for strings */
static char tmpstr[64];
 
/* IIO structs required for streaming */
    static struct iio_context *ctx = NULL;
    static struct iio_channel *rx0_i = NULL;
    static struct iio_channel *rx0_q = NULL;
    static struct iio_channel *tx0_i = NULL;
    static struct iio_channel *tx0_q = NULL;
    static struct iio_buffer *rxbuf = NULL;
    static struct iio_buffer *txbuf = NULL;
 
    static bool stop;
 
/* cleanup and exit */
static void shutdown() {
    printf("* Destroying buffers\n");
    if (rxbuf) {
        iio_buffer_destroy(rxbuf);
    }
    if (txbuf) {
        iio_buffer_destroy(txbuf);
    }
 
    printf("* Disabling streaming channels\n");
    if (rx0_i) {
        iio_channel_disable(rx0_i);
    }
    if (rx0_q) {
        iio_channel_disable(rx0_q);
    }
    if (tx0_i) {
        iio_channel_disable(tx0_i);
    }
    if (tx0_q) {
        iio_channel_disable(tx0_q);
    }
 
    printf("* Destroying context\n");
    if (ctx) {
        iio_context_destroy(ctx);
    }
 
    exit(0);
}
 
static void handle_sig(int sig) {
    printf("Waiting for process to finish...\n");
    stop = true;
}
 
/* check return value of attr_write function */
static void errchk(int v, const char* what) {
    if (v < 0) {
        fprintf(stderr,
                "Error %d writing to channel \"%s\"\nvalue may not be supported.\n",
                v, what);
        shutdown();
    }
}
 
/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val) {
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}
 
/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what,
        const char* str) {
    errchk(iio_channel_attr_write(chn, what, str), what);
}
 
/* helper function generating channel names */
static char* get_ch_name(const char* type, int id) {
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}
 
/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx) {
    struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
    assert(dev && "No ad9361-phy found");
    return dev;
}
 
/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d,
        struct iio_device **dev) {
    switch (d) {
    case TX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        return *dev != NULL;
    case RX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
        return *dev != NULL;
    default:
        assert(0);
        return false;
    }
}
 
/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d,
        struct iio_device *dev, int chid, struct iio_channel **chn) {
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid),
                d == TX);
    return *chn != NULL;
}
 
/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid,
        struct iio_channel **chn) {
    switch (d) {
    case RX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx),
                get_ch_name("voltage", chid), false);
        return *chn != NULL;
    case TX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx),
                get_ch_name("voltage", chid), true);
        return *chn != NULL;
    default:
        assert(0);
        return false;
    }
}
 
/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d,
        struct iio_channel **chn) {
    switch (d) {
    // LO chan is always output, i.e. true
    case RX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx),
                get_ch_name("altvoltage", 0), true);
        return *chn != NULL;
    case TX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx),
                get_ch_name("altvoltage", 1), true);
        return *chn != NULL;
    default:
        assert(0);
        return false;
    }
}
 
/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg,
        enum iodev type, int chid) {
    struct iio_channel *chn = NULL;
 
    // Configure phy and lo channels
    printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) {
        return false;
    }
    wr_ch_str(chn, "rf_port_select", cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);
 
    // Configure LO channel
    printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn)) {
        return false;
    }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}
 
#endif /* AD9361_H_ */

