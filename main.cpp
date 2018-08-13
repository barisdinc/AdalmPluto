//============================================================================
// Name        : test_test.cpp
// Author      : Marcin
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <cstdio>
#include <vector>
#include <iostream>
#include <liquid/liquid.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <liquid/liquid.h>
#include <math.h>
#include "ad9361.h"
 
#define SAMP 2.5
 
using namespace std;
 
int main() {
 
// Streaming devices
    struct iio_device *tx;
    struct iio_device *rx;
 
    // RX and TX sample counters
    size_t nrx = 0;
    size_t ntx = 0;
 
    // Stream configurations
    struct stream_cfg rxcfg;
    struct stream_cfg txcfg;
 
    // Listen to ctrl+c and assert
    signal(SIGINT, handle_sig);
 
    // RX stream config
    rxcfg.bw_hz = MHZ(2);   // 2 MHz rf bandwidth
    rxcfg.fs_hz = MHZ(SAMP);   // 2.5 MS/s rx sample rate
    rxcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
    rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)
 
    // TX stream config
    txcfg.bw_hz = MHZ(1.0); // 1.5 MHz rf bandwidth
    txcfg.fs_hz = MHZ(SAMP);   // 2.5 MS/s tx sample rate
    txcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
    txcfg.rfport = "A"; // port A (select for rf freq.)
 
    printf("* Acquiring IIO context\n");
    assert((ctx = iio_create_default_context()) && "No context");
    assert(iio_context_get_devices_count(ctx) > 0 && "No devices");
 
    printf("* Acquiring AD9361 streaming devices\n");
    assert(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
    assert(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");
 
    printf("* Configuring AD9361 for streaming\n");
    assert(
            cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0)
                    && "RX port 0 not found");
    assert(
            cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0)
                    && "TX port 0 not found");
 
    printf("* Initializing AD9361 IIO streaming channels\n");
    assert(
            get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i)
                    && "RX chan i not found");
    assert(
            get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q)
                    && "RX chan q not found");
    assert(
            get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i)
                    && "TX chan i not found");
    assert(
            get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q)
                    && "TX chan q not found");
 
    printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);
    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);
 
    printf("* Creating non-cyclic IIO buffers with 1 MiS\n");
    rxbuf = iio_device_create_buffer(rx, 8190 * 2, false);
    if (!rxbuf) {
        perror("Could not create RX buffer");
        shutdown();
    }
    txbuf = iio_device_create_buffer(tx, 1024 * 1024, false); //ilosc w probkach zespolonych
    if (!txbuf) {
        perror("Could not create TX buffer");
        shutdown();
    }
    int ile = iio_device_get_sample_size(tx);
 
    float f_sig = 100000;
    int j = 0;
    float k = 0.0f;
    std::vector<short> samples_i;
    std::vector<short> samples_q;
    for (j = 0; j < 1024 * 1024; j++) {
        samples_i.push_back((32767 * cos(2 * M_PI * f_sig * k)));
        samples_q.push_back((32767 * sin(2 * M_PI * f_sig * k)));
        k = k + (1.0 / txcfg.fs_hz);
    }
    //print some samples to check
    for (size_t i = 0; i < 100; i++)
        printf("%d \n", samples_i[i]);
    ;
    printf("In vector is : %d       samples \n", samples_i.size());
 
    printf("* Starting IO streaming (press CTRL+C to cancel)\n");
    ssize_t nbytes_rx, nbytes_tx;
 
    while (!stop) {
 
        iio_channel_write(tx0_i, txbuf, &samples_i[0], 1024 * 1024 * 2);
        iio_channel_write(tx0_q, txbuf, &samples_q[0], 1024 * 1024 * 2);
        nbytes_tx = iio_buffer_push(txbuf);
 
        ntx += nbytes_tx / iio_device_get_sample_size(tx);
        printf("\tRX %8.2f MSmp, TX %8.2f MSmp\n", nrx / 1e6, ntx / 1e6);
    }
 
    shutdown();
 
    return 0;
 
}

