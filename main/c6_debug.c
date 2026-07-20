/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raw access to the ESP32-C6 coprocessor's UART0, for diagnosing a dead
 * ESP-Hosted SDIO link.
 *
 * When `sdio_card_fn_init failed` / CMD52 timeouts appear at boot, the P4 is
 * driving the bus correctly but nothing is answering. The usual cause is that
 * the C6 is not running ESP-Hosted coprocessor firmware at all, so there is no
 * SDIO slave function to respond. The only way to tell that apart from a
 * wiring or pull-up fault is to listen to what the C6 says on its own console.
 *
 * On the JC-ESP32P4-M3 the C6's UART0 is wired to the P4:
 *     C6_U0RXD <- P4 GPIO13   (P4 transmits here)
 *     C6_U0TXD -> P4 GPIO12   (P4 receives here)
 * and the C6's EN/reset is P4 GPIO54 - the same pin ESP-Hosted uses to reset
 * the coprocessor, active low.
 *
 * Commands:
 *   c6mon  [-t s] [-b baud]   passively capture whatever the C6 emits
 *   c6boot [-t s] [-b baud]   pulse EN, then capture the boot banner
 *
 * Reading the result of `c6boot`:
 *   - ROM banner ("ESP-ROM:esp32c6...") then "waiting for download" or a
 *     flash-read error  -> C6 is blank or has no valid app: flash the
 *     ESP-Hosted coprocessor firmware.
 *   - ROM banner then an app banner mentioning esp_hosted / SDIO slave
 *     -> firmware is right; suspect pin/pull-up/version mismatch instead.
 *   - ROM banner then some other app (AT, Zigbee, factory test)
 *     -> wrong firmware, reflash with the hosted coprocessor build.
 *   - nothing at all -> C6 held in reset, unpowered, or the UART is not on
 *     GPIO12/13 after all; try other bauds before concluding.
 */

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "wifi_console.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "c6_dbg";

#define C6_UART_PORT   UART_NUM_2          /* UART1 is the thermal camera */
#define C6_TX_IO       13                  /* -> C6_U0RXD */
#define C6_RX_IO       12                  /* <- C6_U0TXD */
#define C6_EN_IO       CONFIG_ESP_HOSTED_HOST_RESET_GPIO   /* 54, active low */
/* C6 GPIO9 (BOOT strap). Idles high through a 5.1k pull-up on the module; a
 * bodge wire brings it to P4 GPIO11 so we can hold it low across a reset and
 * drop the C6 into ROM download mode. Drive it LOW only - never drive it high
 * against the pull-up; release by going back to input (high-Z). */
#define C6_BOOT_IO     11
#define C6_RX_BUF      4096
#define DEFAULT_BAUD   115200
#define DEFAULT_SECS   3

static bool s_uart_ready;

/*
 * Reset the C6, optionally holding BOOT low so the ROM comes up in download
 * mode instead of trying (and failing) to boot the empty flash.
 *
 * BOOT is open-drain by intent: LOW is driven, "high" is high-Z and left to
 * the module's 5.1k pull-up. Driving it high would fight that resistor.
 */
static esp_err_t c6_reset(bool into_download)
{
    gpio_config_t en = {
        .pin_bit_mask = 1ULL << C6_EN_IO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&en), TAG, "gpio_config EN");

    if (into_download) {
        gpio_config_t boot = {
            .pin_bit_mask = 1ULL << C6_BOOT_IO,
            .mode         = GPIO_MODE_OUTPUT_OD,   /* pull low, release high-Z */
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&boot), TAG, "gpio_config BOOT");
        gpio_set_level(C6_BOOT_IO, 0);
        vTaskDelay(pdMS_TO_TICKS(10));             /* settle before EN falls */
    }

    gpio_set_level(C6_EN_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(C6_EN_IO, 1);

    if (into_download) {
        /* Hold BOOT past the strap latch, then let the pull-up take it back
         * so the pin is free for the C6's own use once it is running. */
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(C6_BOOT_IO, 1);             /* open-drain -> high-Z */
    }
    return ESP_OK;
}

/* Hand UART2 back so esp-serial-flasher can configure it on its own terms
 * (it sets its own buffer sizes and timeouts). Safe to call when idle. */
void c6_debug_release_uart(void)
{
    if (s_uart_ready) {
        uart_driver_delete(C6_UART_PORT);
        s_uart_ready = false;
    }
}

static esp_err_t c6_uart_setup(int baud)
{
    if (s_uart_ready) {
        /* Already installed - just retune the baud rate. */
        return uart_set_baudrate(C6_UART_PORT, baud);
    }

    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(C6_UART_PORT, C6_RX_BUF, 0, 0, NULL, 0),
                        TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(C6_UART_PORT, &cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(C6_UART_PORT, C6_TX_IO, C6_RX_IO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");
    s_uart_ready = true;
    return ESP_OK;
}

/*
 * Drain the C6's UART for `secs`, printing it as it arrives. Lines are echoed
 * verbatim; non-printable bytes become '.' so a baud mismatch shows up as
 * obvious garbage rather than scrambling the terminal.
 */
static size_t c6_capture(int secs)
{
    uint8_t buf[256];
    size_t  total = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)secs * 1000000;

    while (esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(C6_UART_PORT, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;
        }
        total += (size_t)n;
        for (int i = 0; i < n; i++) {
            uint8_t c = buf[i];
            if (c == '\n' || c == '\r' || (c >= 0x20 && c < 0x7F)) {
                putchar((char)c);
            } else {
                putchar('.');
            }
        }
        fflush(stdout);
    }
    return total;
}

static void c6_report(size_t got, int baud)
{
    printf("\n--- %u bytes captured @ %d baud ---\n", (unsigned)got, baud);
    if (got == 0) {
        printf("Nothing received. Either the C6 is held in reset / unpowered,\n"
               "its UART0 is not on GPIO%d/GPIO%d, or it boots silently.\n"
               "Try another rate: c6boot -b 74880   (ROM default on some chips)\n",
               C6_TX_IO, C6_RX_IO);
    }
}

/* ----------------------------------------------------------------- c6mon */

static struct {
    struct arg_int *secs;
    struct arg_int *baud;
    struct arg_end *end;
} s_mon_args;

static int cmd_c6mon(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_mon_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_mon_args.end, argv[0]);
        return 1;
    }
    int secs = s_mon_args.secs->count ? s_mon_args.secs->ival[0] : DEFAULT_SECS;
    int baud = s_mon_args.baud->count ? s_mon_args.baud->ival[0] : DEFAULT_BAUD;
    if (secs < 1 || secs > 60) {
        printf("duration must be 1..60 s\n");
        return 1;
    }

    esp_err_t err = c6_uart_setup(baud);
    if (err != ESP_OK) {
        printf("UART setup failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("listening to C6 UART0 (GPIO%d) for %d s @ %d baud...\n",
           C6_RX_IO, secs, baud);
    uart_flush_input(C6_UART_PORT);
    size_t got = c6_capture(secs);
    c6_report(got, baud);
    return 0;
}

/* ---------------------------------------------------------------- c6boot */

static struct {
    struct arg_lit *download;
    struct arg_int *secs;
    struct arg_int *baud;
    struct arg_end *end;
} s_boot_args;

static int cmd_c6boot(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_boot_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_boot_args.end, argv[0]);
        return 1;
    }
    bool dl   = s_boot_args.download->count > 0;
    int  secs = s_boot_args.secs->count ? s_boot_args.secs->ival[0] : DEFAULT_SECS;
    int  baud = s_boot_args.baud->count ? s_boot_args.baud->ival[0] : DEFAULT_BAUD;
    if (secs < 1 || secs > 60) {
        printf("duration must be 1..60 s\n");
        return 1;
    }

    esp_err_t err = c6_uart_setup(baud);
    if (err != ESP_OK) {
        printf("UART setup failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    printf("resetting C6 via GPIO%d (active low)%s, capturing %d s @ %d baud\n\n",
           C6_EN_IO,
           dl ? ", BOOT held low on GPIO11 -> download mode" : "",
           secs, baud);

    uart_flush_input(C6_UART_PORT);
    err = c6_reset(dl);
    if (err != ESP_OK) {
        printf("reset failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    size_t got = c6_capture(secs);
    c6_report(got, baud);

    if (got > 0 && dl) {
        printf("\nLook for 'waiting for download' and a boot mode of\n"
               "DOWNLOAD_BOOT(UART0/UART1/SDIO_REI_REO_V2). If you see that,\n"
               "the BOOT bodge to GPIO11 works and the C6 can be flashed.\n");
    } else if (got > 0) {
        printf("\nA ROM banner with an invalid-header error means the flash is\n"
               "empty, as expected. Re-run as 'c6boot -d' to confirm the C6 will\n"
               "enter download mode before we try to flash it.\n");
    }
    return 0;
}

/* ------------------------------------------------------------------ init */

esp_err_t c6_debug_register(void)
{
    s_mon_args.secs = arg_int0("t", "time", "<s>", "capture duration, default 3");
    s_mon_args.baud = arg_int0("b", "baud", "<rate>", "baud rate, default 115200");
    s_mon_args.end  = arg_end(2);
    const esp_console_cmd_t mon_cmd = {
        .command = "c6mon",
        .help = "Listen to the C6 coprocessor's UART0 without resetting it",
        .hint = NULL, .func = &cmd_c6mon, .argtable = &s_mon_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&mon_cmd), TAG, "reg c6mon");

    s_boot_args.download = arg_lit0("d", "download", "hold BOOT low -> ROM download mode");
    s_boot_args.secs = arg_int0("t", "time", "<s>", "capture duration, default 3");
    s_boot_args.baud = arg_int0("b", "baud", "<rate>", "baud rate, default 115200");
    s_boot_args.end  = arg_end(2);
    const esp_console_cmd_t boot_cmd = {
        .command = "c6boot",
        .help = "Reset the C6 (-d for download mode) and capture its boot log",
        .hint = NULL, .func = &cmd_c6boot, .argtable = &s_boot_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&boot_cmd), TAG, "reg c6boot");

    return ESP_OK;
}
