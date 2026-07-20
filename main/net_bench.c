/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Throughput benchmarks for the ESP-Hosted link, registered as console
 * commands by wifi_console.c.
 *
 *   sdiospeed  - how fast the P4 can push bytes across SDIO into the C6.
 *                Frames are handed to esp_wifi_internal_tx(), which on a
 *                remote-Wi-Fi target is a straight write into the hosted
 *                transport, so the measurement is the SDIO write path plus
 *                the C6's receive handling.
 *
 *                Run it DISCONNECTED for the bus number in isolation: the C6
 *                drops the frames once they arrive, so nothing touches the
 *                air. Run it CONNECTED and the same frames are also
 *                transmitted, so the result folds in radio backpressure.
 *                The command says which regime it measured.
 *
 *   wifispeed  - end-to-end TCP throughput to a sink on a host PC
 *                (P4 -> SDIO -> C6 -> air -> AP -> PC). This is the number
 *                that matters for actually moving thermal frames off the
 *                board; compare it against sdiospeed to see which of the two
 *                hops is the limit.
 */

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "wifi_console.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"   /* esp_wifi_internal_tx() - the hosted TX path */
#include "esp_timer.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "net_bench";

/* ESP-Hosted's host->C6 Wi-Fi flow control flag, owned by
 * eh_host_mcu_transport_channels.c. The C6 raises it via an SDIO
 * START_THROTTLE interrupt when its Wi-Fi TX buffers pass the high-water mark
 * and lowers it again on STOP_THROTTLE. While it is set, eh_host_transport_tx()
 * *drops* every STA frame instead of queueing it - so a throttle that latches
 * on looks exactly like a dead link: TCP retransmits into a hole forever.
 *
 * Not a public symbol; declared here so wifispeed can say whether a stall is
 * flow control or something else. Guarded on the same config that defines it. */
#if CONFIG_ESP_HOSTED_HOST_TO_ESP_WIFI_DATA_THROTTLE
extern volatile uint32_t wifi_tx_throttling;
#define BENCH_THROTTLED() (wifi_tx_throttling != 0)
#else
#define BENCH_THROTTLED() false
#endif

#define ETH_HDR_LEN        14
#define DEFAULT_FRAME_LEN  1500
#define MIN_FRAME_LEN      64
#define MAX_FRAME_LEN      1514
#define DEFAULT_SECONDS    5

/* Print a throughput line from a byte count and an elapsed time in microseconds. */
static void report(const char *what, uint64_t bytes, int64_t us)
{
    if (us <= 0) {
        printf("%s: no elapsed time measured\n", what);
        return;
    }
    double secs = (double)us / 1e6;
    double mbits = ((double)bytes * 8.0) / (secs * 1e6);
    printf("%s: %llu bytes in %.2f s = %.2f Mbit/s (%.2f MB/s)\n",
           what, (unsigned long long)bytes, secs, mbits, mbits / 8.0);
}

/* ------------------------------------------------------------- sdiospeed */

static struct {
    struct arg_int *secs;
    struct arg_int *len;
    struct arg_end *end;
} s_sdio_args;

static int cmd_sdiospeed(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_sdio_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_sdio_args.end, argv[0]);
        return 1;
    }
    int secs = s_sdio_args.secs->count ? s_sdio_args.secs->ival[0] : DEFAULT_SECONDS;
    int len  = s_sdio_args.len->count  ? s_sdio_args.len->ival[0]  : DEFAULT_FRAME_LEN;

    if (secs < 1 || secs > 60) {
        printf("duration must be 1..60 s\n");
        return 1;
    }
    if (len < MIN_FRAME_LEN || len > MAX_FRAME_LEN) {
        printf("frame length must be %d..%d bytes\n", MIN_FRAME_LEN, MAX_FRAME_LEN);
        return 1;
    }

    if (!wifi_console_link_up()) {
        printf("C6 link is down - nothing to benchmark. Run 'c6boot' first.\n");
        return 1;
    }

    uint8_t src_mac[6] = { 0 };
    if (esp_wifi_get_mac(WIFI_IF_STA, src_mac) != ESP_OK) {
        printf("cannot read STA MAC - is the C6 link up?\n");
        return 1;
    }

    wifi_ap_record_t ap;
    bool connected = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

    uint8_t *frame = malloc(len);
    if (!frame) {
        printf("out of memory for a %d byte frame\n", len);
        return 1;
    }
    /* Ethernet II header: locally-administered unicast destination that no
     * station will claim, our own MAC as source, and an unassigned ethertype
     * so nothing on the LAN tries to interpret the payload. */
    const uint8_t dst_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 };
    memcpy(frame + 0, dst_mac, 6);
    memcpy(frame + 6, src_mac, 6);
    frame[12] = 0x88; frame[13] = 0xB5;          /* IEEE 802 local experimental */
    memset(frame + ETH_HDR_LEN, 0xA5, len - ETH_HDR_LEN);

    printf("sdiospeed: %d s, %d byte frames, %s\n", secs, len,
           connected ? "CONNECTED (includes over-the-air TX)"
                     : "disconnected (SDIO write path only)");

    uint64_t sent_bytes = 0, sent_frames = 0, busy = 0;
    int64_t  t0 = esp_timer_get_time();
    int64_t  deadline = t0 + (int64_t)secs * 1000000;
    int64_t  now;

    while ((now = esp_timer_get_time()) < deadline) {
        /* Burst between yields: the hosted TX path is queue-backed, so
         * yielding on every frame would measure the scheduler, not the bus.
         * 64 x 1500 B per 1 ms tick is a ~768 Mbit/s ceiling, comfortably
         * above the ~160 Mbit/s the bus itself can do, so this loop never
         * becomes the limiting factor. */
        for (int i = 0; i < 64; i++) {
            int rc = esp_wifi_internal_tx(WIFI_IF_STA, frame, (uint16_t)len);
            if (rc == 0) {
                sent_bytes += len;
                sent_frames++;
            } else {
                busy++;
                break;      /* queue full - back off and let the link drain */
            }
        }
        /* Always yield: this task must not starve the hosted TX task, and on
         * a 1000 Hz tick one tick is a ~0.1% duty cost at these rates. */
        vTaskDelay(1);
    }
    int64_t elapsed = esp_timer_get_time() - t0;
    free(frame);

    report("sdiospeed", sent_bytes, elapsed);
    printf("  %llu frames accepted, %llu queue-full backoffs\n",
           (unsigned long long)sent_frames, (unsigned long long)busy);
    printf("  SDIO configured for %d-bit @ %d kHz = %.0f Mbit/s theoretical ceiling\n",
           CONFIG_ESP_HOSTED_HOST_SDIO_BUS_WIDTH, CONFIG_ESP_HOSTED_HOST_SDIO_CLK_KHZ,
           (double)CONFIG_ESP_HOSTED_HOST_SDIO_BUS_WIDTH *
           (double)CONFIG_ESP_HOSTED_HOST_SDIO_CLK_KHZ / 1000.0);
    if (connected) {
        printf("  note: run `leave` first to measure the bus without the radio\n");
    }
    return 0;
}

/* ------------------------------------------------------------- wifispeed */

static struct {
    struct arg_str *host;
    struct arg_int *port;
    struct arg_int *secs;
    struct arg_end *end;
} s_wifi_args;

static int cmd_wifispeed(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_wifi_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_wifi_args.end, argv[0]);
        return 1;
    }
    const char *host = s_wifi_args.host->sval[0];
    int port = s_wifi_args.port->count ? s_wifi_args.port->ival[0] : 5001;
    int secs = s_wifi_args.secs->count ? s_wifi_args.secs->ival[0] : DEFAULT_SECONDS;

    if (secs < 1 || secs > 60) {
        printf("duration must be 1..60 s\n");
        return 1;
    }

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        printf("not associated - run `join <ssid> [password]` first\n");
        return 1;
    }

    struct sockaddr_in dest = { 0 };
    dest.sin_family = AF_INET;
    dest.sin_port   = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &dest.sin_addr) != 1) {
        printf("'%s' is not a valid IPv4 address\n", host);
        return 1;
    }

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        printf("socket: errno %d\n", errno);
        return 1;
    }

    printf("connecting to %s:%d ...\n", host, port);
    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        printf("connect: errno %d (is the sink script running?)\n", errno);
        close(sock);
        return 1;
    }

    const size_t chunk = 4096;
    uint8_t *buf = malloc(chunk);
    if (!buf) {
        printf("out of memory for a %u byte buffer\n", (unsigned)chunk);
        close(sock);
        return 1;
    }
    memset(buf, 0x5A, chunk);

    /* Bound the blocking send so a stalled peer cannot wedge the console. Kept
     * short (1 s) because a timeout is not fatal here - the loop rides it out
     * and keeps going - so this only sets how far past the deadline the run can
     * overshoot, and how coarsely a stall is measured. */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    /* No RSSI here: it is not readable on this stack (see wifi_console.c) and
     * a scan mid-benchmark would perturb the very thing being measured. */
    printf("wifispeed: sending for %d s to %s:%d (ch %d)\n",
           secs, host, port, ap.primary);

    uint64_t total = 0, stalls = 0;
    uint64_t throttle_samples = 0, throttle_hits = 0;
    int64_t  stalled_us = 0;
    int64_t  t0 = esp_timer_get_time();
    int64_t  deadline = t0 + (int64_t)secs * 1000000;

    while (esp_timer_get_time() < deadline) {
        /* Sampled before each send: if the hosted throttle is latched on, this
         * is where it shows up, and the send below is writing into a hole. */
        throttle_samples++;
        if (BENCH_THROTTLED()) {
            throttle_hits++;
        }
        int64_t s0 = esp_timer_get_time();
        int n = send(sock, buf, chunk, 0);
        if (n < 0) {
            /* EAGAIN/EWOULDBLOCK is the SO_SNDTIMEO firing: the send window is
             * full and no ACK arrived within the timeout. That is a stall to
             * measure, not an error to abort on - bailing out on the first one
             * is what made a wedged link report a bogus "13.33 s" for a 10 s
             * run. Keep pushing until the deadline and count the dead time. */
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                stalls++;
                stalled_us += esp_timer_get_time() - s0;
                continue;
            }
            printf("send: errno %d after %llu bytes - aborting\n",
                   errno, (unsigned long long)total);
            break;
        }
        total += (uint64_t)n;
    }
    int64_t elapsed = esp_timer_get_time() - t0;

    free(buf);
    close(sock);

    report("wifispeed (TX)", total, elapsed);
    if (stalls) {
        printf("  %llu stall(s), %.2f s of the run spent with the send window "
               "full and no ACK\n", (unsigned long long)stalls,
               (double)stalled_us / 1e6);
    }
#if CONFIG_ESP_HOSTED_HOST_TO_ESP_WIFI_DATA_THROTTLE
    printf("  hosted TX throttle: set on %llu of %llu samples (%s)\n",
           (unsigned long long)throttle_hits, (unsigned long long)throttle_samples,
           throttle_hits == 0            ? "never engaged - flow control is NOT the stall"
           : throttle_hits == throttle_samples
                                         ? "LATCHED ON - C6 never sent STOP_THROTTLE, "
                                           "every STA frame is being dropped"
                                         : "engaging and clearing - normal backpressure");
#else
    (void)throttle_hits; (void)throttle_samples;
#endif
    printf("  note: the byte count above is what lwIP accepted, not what arrived.\n"
           "        Compare it against the total tcp_sink.py prints on the PC.\n");
    return 0;
}

/* ------------------------------------------------------------------ init */

esp_err_t net_bench_register(void)
{
    s_sdio_args.secs = arg_int0("t", "time", "<s>", "duration, default 5");
    s_sdio_args.len  = arg_int0("l", "len", "<bytes>", "frame size 64..1514, default 1500");
    s_sdio_args.end  = arg_end(2);
    const esp_console_cmd_t sdio_cmd = {
        .command = "sdiospeed",
        .help = "Measure host->C6 SDIO write throughput (run disconnected to isolate the bus)",
        .hint = NULL, .func = &cmd_sdiospeed, .argtable = &s_sdio_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&sdio_cmd), TAG, "reg sdiospeed");

    s_wifi_args.host = arg_str1(NULL, NULL, "<ip>", "IPv4 address of the sink PC");
    s_wifi_args.port = arg_int0("p", "port", "<port>", "TCP port, default 5001");
    s_wifi_args.secs = arg_int0("t", "time", "<s>", "duration, default 5");
    s_wifi_args.end  = arg_end(2);
    const esp_console_cmd_t wifi_cmd = {
        .command = "wifispeed",
        .help = "Measure end-to-end TCP throughput to a host PC sink",
        .hint = NULL, .func = &cmd_wifispeed, .argtable = &s_wifi_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&wifi_cmd), TAG, "reg wifispeed");

    return ESP_OK;
}
